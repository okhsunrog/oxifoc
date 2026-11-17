# BLE pairing for the skateboard

> **STATUS: design, not implemented (2026-11-17).** Today the bridge accepts
> any central and forwards its Ergot traffic.

The C5 bridge is the BLE peripheral and the trust anchor. The remote, phones
and laptops are centrals. The design targets accidental pairing and
unauthorized control by nearby devices. Physical access to the board is
outside the threat model: whoever can open the board can reach USB.

## Principle: Ergot carries the keys

Pairing is managed with ordinary Ergot commands to the bridge. Any trusted
Ergot channel can send them, whatever the transport. There is no
transport-specific pairing mode.

- To pair a new device with a closed board, at least one paired device must
  be available.
- With no paired device left, open the board and use USB. There is no other
  recovery path (no power-cycle window, no recovery code).

## Invariants

1. **Only trusted channels reach Ergot.**
   - USB: trusted (physical access).
   - BLE: trusted only after the link is encrypted with an LTK from a stored
     bond (authenticated, LE Secure Connections). Before that the bridge
     neither accepts nor forwards Ergot frames from the link, and the link's
     router interface stays inactive. Only SMP runs on it.
   - Future internal CAN between C5s: trusted. Wi-Fi: untrusted until it has
     its own authentication.
2. **Any trusted channel can manage pairing:** open a window, supply or
   confirm a code, list, rename and remove bonds. Roles limit drive authority,
   not pairing rights (see Roles).
3. **New devices pair only inside a window.**
   - A trusted channel opens it with a command. The window lasts at most 60 s
     and ends after one successful bond, a failure, a cancellation or the
     timeout.
   - Opening requires a stopped controller with drive inhibited.
   - Outside a window the controller's filter accept list admits only bonded
     identities, so unknown centrals cannot take a connection slot.
4. **Every pairing is MITM-protected (no Just Works).** The code always
   travels over a trusted channel (see Methods). A candidate that offers only
   `NoInputNoOutput` is rejected.
5. **Bond changes are visible.** The bridge publishes every addition, removal
   and failed attempt to all trusted channels.
6. **At most 8 bonds.** A ninth pairing is rejected with "bond table full";
   remove a bond first.
7. **Bonds persist on the bridge**, with metadata: name, role and the bond
   that added it. The remote also stores its bridge bond.

## Methods

The bridge declares `KeyboardDisplay`. Its keyboard and display are a trusted
device at the other end of Ergot: the remote's screen or OxiFOC Host. LE Secure
Connections selects the method from the candidate's IO capabilities
([Core Specification, Security Manager, Table 2.8](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core_v6.3/out/en/host/security-manager-specification.html)):

| Candidate | Method | Code flow |
|---|---|---|
| `DisplayOnly` (the remote) | Passkey Entry, candidate displays | The remote shows six digits; the user types them in OxiFOC Host on a trusted device; Host sends them to the bridge, which passes them to Trouble (`pass_key_input`). |
| `KeyboardOnly` | Passkey Entry, bridge displays | The bridge publishes the passkey; a trusted device shows it; the user types it on the candidate. |
| `KeyboardDisplay` (phones, laptops) | Numeric Comparison | Both the candidate and a trusted device show the same number; the user confirms on the candidate and on the trusted device. |
| `NoInputNoOutput` | Just Works | Rejected (invariant 4). |

A wrong code, a rejection, a cancellation or the timeout ends the attempt and
clears its code.

## Roles

- **Admin** (laptop, phone): configuration and service commands.
- **Remote**: may become the selected drive controller. Only the selected
  remote's drive intent is accepted. Reconnection requires neutral followed by
  fresh intent ([remote design](remote-design.md), [safety](../safety.md)).

All bonded devices may manage pairing (invariant 2). The remote can show and
confirm codes on its screen. Service commands that can drive the motor or start
detection still require an explicit service mode with the board stopped.

## Flows

**First admin (bench, board open).** OxiFOC Host over USB opens a window. The
laptop pairs over BLE using Numeric Comparison, with the number shown by Host
on the same laptop. It is now trusted over BLE.

**Add a remote.** A trusted device opens a window with role Remote. The remote
enters pairing mode (button), connects and shows its passkey. The user enters
it in Host. Both save the bond. The bridge confirms that the remote reconnects
with it. The admin selects it as the drive controller and may remove the old
remote. If the attempt fails, the previously selected remote reconnects and
resumes. If the old remote holds the last free slot, the bridge frees it for
the candidate and keeps the old bond and selection.

**Add a phone or laptop.** A trusted device opens a window with role Admin.
The candidate pairs from its system Bluetooth dialog or the app; the user
compares the number with the one on the remote's screen or in Host.

**Remove.** Any trusted device lists bonds and removes one. Removing the
current drive controller stops drive acceptance until another remote is
selected.

## Ergot interface (sketch)

Served by the bridge, not the motor controller. Names and paths follow
`oxifoc-core/src/icd.rs`; versioning follows
[protocol versioning](protocol-versioning.md).

`PairingEndpoint`, `cmd/pairing`: `PairingRequest` → `PairingResponse`.

| Request | Response | Notes |
|---|---|---|
| `OpenWindow { role, seconds }` | `WindowOpened { attempt, expires_ms }` | Rejected unless stopped and drive inhibited, or if the table is full |
| `SubmitPasskey { attempt, passkey }` | `Ok` | Candidate-displayed code (the remote) |
| `ConfirmComparison { attempt, accept }` | `Ok` | Numeric Comparison |
| `Cancel { attempt }` | `Ok` | |
| `ListBonds` | `Bonds(Vec<BondEntry, 8>)` | id, name, role, identity address, selected flag |
| `RemoveBond { id }` | `Ok` | |
| `RenameBond { id, name }` | `Ok` | |
| `SelectRemote { id }` | `Ok` | Drive authority |

Errors: `NotStopped`, `TableFull`, `NoSuchAttempt`, `NoSuchBond`, `WrongRole`.

`PairingEventTopic`, `events/pairing`: `PairingEvent`

- `WindowOpened { attempt, role, expires_ms }`, `WindowClosed { attempt, reason }`
- `CandidateConnected { attempt, address }`
- `ShowPasskey { attempt, passkey }`: bridge displays (candidate `KeyboardOnly`)
- `EnterPasskey { attempt }`: candidate displays, a trusted device must submit it
- `CompareNumber { attempt, number }`
- `Bonded { id, name, role }`, `BondRemoved { id }`, `Failed { attempt, reason }`

Bond record in bridge flash: id, identity address, IRK, LTK, security level,
role, name (≤ 24 bytes), added-by id.

## Implementation notes

- Enable Trouble's `security` feature on the bridge and the remote.
  `HostResources<…, BONDS = 8>`; esp-radio `whitelist_size` and
  `ll_resolv_list_size` 8. Check that the C5 controller accepts a resolving
  list of 8; if not, lower the bond limit to what it holds.
- Gate per link slot in `ble_runtime.rs`: process RX writes and drain the
  slot's queue only after the link is encrypted with a bonded key. Consider
  GATT permissions that require encryption on RX/TX, so phones start pairing
  when they first touch the characteristics.
- Bridge storage: `sequential-storage` map over the C5 flash, as on the remote.
- Remote: bridge bond in its settings map. Connect with a resolvable private
  address, fresh per boot, instead of today's fresh static random address.
- The bridge's fixed address stays for now; with bonding the remote can find
  its bridge by identity instead.

## Validation

- **RPA after a quick remote restart.** A fresh static address avoids the
  stale-link rejection today
  ([reliability note](ble-link-reliability.md)). Check that the C5 does not
  resolve the new RPA to the old identity and reject it the same way. If it
  does, drop the old link when the same identity reconnects.
- Encryption start cost on reconnect (expect two connection events).
- Laptop first-admin over USB; remote enrollment with the code typed in Host;
  wrong code, cancellation and timeout; bond persistence across reboots of
  both sides; recovery of the previous remote after a failed replacement.
- Android enrollment with Numeric Comparison; revocation; phone telemetry
  together with remote control; unknown centrals rejected outside a window;
  ninth bond rejected.
- Remote link loss: the STM32 drive deadman stays independent of BLE state.

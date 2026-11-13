# BLE pairing for the skateboard

The C5 is the BLE peripheral for the remote and the host app. Both are
centrals. A trusted laptop or phone can manage enrollment and configure the
board. The selected remote controls traction and braking. Service commands
that can drive the motor or start detection require an explicit service mode
with the board stopped. This design targets accidental pairing and unauthorized
control by nearby devices. The commissioning procedure assumes physical access
to the equipment at home.

## First setup

- Enroll the first administrator through a local setup path while the C5 is
  accessible on the bench. The administrator can enroll a phone using LE Secure
  Connections Passkey Entry and the phone's system dialog.
- Keep enrollment closed during normal operation. Enrollment requires a stopped
  controller with drive inhibited and expires after a short window.
- Persist BLE bonds and their roles on C5. Admit Ergot traffic only after the
  device has authenticated using the enrolled bond. BLE addresses help find a
  peer. The stored bond establishes its identity.
- Keep an optional physical recovery path in mind before sealing the enclosure.
  A QR code held privately in a password manager or backed-up encrypted storage
  could support recovery later. It represents a unique board secret, with
  proof of possession if used over BLE. Ordinary enrollment uses the trusted
  administrator, so recovery QR support can wait.

## Add or replace a remote

1. Connect the new remote to a trusted laptop by USB. Leave the C5 installed.
   OxiFOC Host connects to C5 over its existing BLE bond and to the remote
   over native USB Serial/JTAG with Ergot/COBS.
2. Host reads the remote's device type, identity and enrollment state over USB.
   The remote terminates this USB connection locally. It exposes hardware
   identity and start, status and cancel operations for enrollment.
3. Host requests a short enrollment window from C5 while stopped. C5 supplies
   a fresh six-digit passkey. Host delivers the passkey and target board identity
   to the remote over USB. Store the passkey in RAM for this attempt and discard
   it when the attempt ends.
4. The remote connects to C5 as central and supplies the passkey to Trouble
   during LE Secure Connections pairing. Each device persists its own bond.
5. Host confirms a reconnect using that bond before selecting the new remote as
   the sole drive controller. A failed or cancelled attempt leaves the old
   remote usable. The administrator can revoke the old bond afterwards.

Start with a host CLI command such as `pair remote` and reuse the workflow
in the GUI. This also supports replacing the remote's microcontroller once its
firmware implements the USB enrollment commands and the BLE profile. Board
firmware can keep the same configuration and stay inside the enclosure.

## Runtime and validation

The current bridge supports one BLE connection and the remote currently uses a
fixed bridge address. Enrollment needs the administrator and candidate remote
connected concurrently. In normal use, reserve capacity for the selected remote
while the phone is connected. Evaluate the C5 controller's accept list with
private phone addresses. Check authorization before forwarding traffic and
keep the STM32 command watchdog independent of BLE connection state.

Start with a laptop administrator and one selected remote. Verify persistent
bonds across reboot, rejection of an unknown peer, cancellation and replacement
with the previous remote still available until confirmation. Then verify
Android enrollment, revocation, simultaneous phone telemetry and remote control,
and loss of the remote link. Reconnection requires neutral followed by fresh
driver intent. See the [remote design](remote-design.md) and
[drive watchdog](../safety.md).

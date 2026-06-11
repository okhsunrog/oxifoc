# Handheld Remote — Design Notes

Design decisions for the ESK8 throttle remote and its link to the motor
controller, captured from the 2026-06-12 design session. Status markers:
**[decided]** · **[open]** · **[bench]** (validate on hardware) · **[later]**
(deferred, not v1).

The remote is an ESP32-C6 handheld: a magnetic throttle, a PMIC, optional
display/haptics, talking to the board over a wireless link that routes through
the `oxifoc-bridge` (ESP32-C6) into the motor controller. The whole network is
[ergot](https://github.com/jamesmunns/ergot), so transport choices are
decoupled from the application/routing layer — any link can be swapped without
app rework.

---

## 1. Hardware strategy — validate before customizing

**[decided]** Do **not** build the full custom remote PCB first. A
System-on-Module (SoM) core decouples the *invariant* electronics (MCU + PMIC +
charging) from the *iterable* human-interface (throttle feel, display, buttons,
grip ergonomics). The core can be designed now in parallel with validation; the
enclosure/throttle iterate with a 3D printer.

Staging:
- **Stage 0 (now):** laptop/phone are already ergot Edge nodes — command
  throttle from them with the board on a stand. Validates the full network path
  (BLE → bridge → CAN → MC) + motor + failsafe **without a rider**. No remote
  hardware needed; catches ~80% of issues.
- **Stage 1 (first ride):** minimal handheld = ESP32-C6 + magnetic throttle +
  LiPo + 3D-printed grip with a spring-return trigger (the mechanical deadman).
  Reuse the `oxifoc-remote` firmware.
- **Stage 2 (after rides):** custom PCB, designed to the *learned* ergonomics
  and throttle feel — not guessed. The electronics are all proven by then, so
  the spin is "integrate proven blocks," low risk; the prototype existed to fix
  *feel and layout*, not to prove the parts work.

The deferral rationale is **ergonomics/feel unknowns**, not electronics risk —
every block (AXP2101, magnetic encoder, C6) is already de-risked.

---

## 2. Core module (SoM)

**[decided]** A compact core module, sized to drop into "roughly any" remote
enclosure. On-module = invariant; off-module = ergonomics-dependent.

**On the module:** ESP32-C6-MINI-1U + AXP2101 + LiPo connector (JST) + charging
+ power rails + USB-C. An **expansion interface** (board-to-board / FFC /
header) breaks out: I2C (throttle encoder + PMIC + display share the bus), a few
GPIO (buttons, vibration-motor gate via the PMIC's DLDO1), power-key, 3V3/GND.

**Off the module** (iterate with the printer): the hall/magnet throttle and its
position, display, buttons, grip shape.

Module also makes a reusable open-source artifact (like the crate stack —
`drv8301-dd`, `axp2101-dd`, `ergot`): an "ESP32-C6 + AXP2101 core" is useful to
anyone building a battery-powered C6 handheld.

### PCB

- **[decided] 4-layer.** *Not* for RF (the C6-MINI is a shielded module with its
  own antenna — RF is handled). The drivers are: (a) the AXP2101 is a switching
  PMIC whose DCDC loops want a solid ground plane, and (b) the throttle is
  safety-critical analog whose reference/return must stay clean of switcher
  noise. At proto quantities the 4-layer cost delta is ~a few dollars; it turns
  "carefully route returns under a switcher" into "drop a ground plane." Stack:
  top sig / GND / PWR / bottom sig.
- **[decided] Reflow assembly.** The AXP2101 QFN (thermal pad) mandates reflow
  regardless of the module — so the MINI-1U's bottom pads cost nothing extra
  (same stencil + paste + hotplate pass). Order PCB + stencil from JLCPCB
  (~$7); hotplate is fine for this size. **Escape hatch:** JLCPCB PCBA assembles
  it (both AXP2101 and the C6 module are standard parts) — removes the soldering
  variable for the first spin.
- **[decided] USB-C, dual purpose:** the C6's native USB (Serial/JTAG) for
  flash/debug, and the same VBUS feeds the AXP2101 for charging.
- **[later] Qwiic/STEMMA I2C connector** for rapid prototype attach of
  display/sensor without soldering.
- Reuse the **M5Core2 v1.1 schematic** (`Sch_Core2_v1.1`) as the proven AXP2101
  rail/charging reference — don't design the PMIC circuit from scratch.

### Module selection: ESP32-C6-MINI-1U-N4

**[decided]** vs the alternatives:

| | MINI-1U-N4 (chosen) | MINI-1-N4 | WROOM-1U-N4 |
|---|---|---|---|
| Size | ~13.2×13.2×2.4 | ~13.2×16.6×2.4 | ~18.0×19.2×3.2 (~2× area) |
| Antenna | external (U.FL) | PCB | external (U.FL) |
| Pads | bottom (reflow) | bottom (reflow) | castellated |
| GPIO | enough for a remote | enough | more |

- **External antenna (1U) is a real plus for a handheld:** the hand wraps the
  grip and detunes a PCB antenna — the #1 RF failure mode of a handheld. U.FL
  lets the radiator sit away from the hand (top of the remote). Cost: fragile
  U.FL connector + coax + enclosure space. For a link where loss = throttle loss,
  worth it. (If the enclosure naturally keeps a PCB antenna clear of the hand,
  MINI-1 is simpler — but a hand-wrapped grip rarely does.)
- **MINI-1U also has no antenna keepout** on the carrier → free placement,
  smaller board.
- **WROOM-1U is ~2× the area and thicker;** its only edge (castellated →
  hand-solderable) is moot since the AXP2101 forces reflow anyway, and a remote
  needs few GPIO. Reserve WROOM for the controller/bridge board if it needs more
  pins or the 105 °C grade.
- **N4 vs H4:** same 4 MB flash; N4 = −40…+85 °C (separate flash die), H4 = the
  ESP32-C6FH4 (in-package flash), −40…+105 °C. A remote is hand-temperature →
  **N4**; H4's extra range only matters near heat (e.g. a C6 on the controller
  board by the power stage). Check JLC/Mouser stock either way.

---

## 3. PMIC — AXP2101

**[decided]** De-risked: it is the user's own published `axp2101-dd` driver,
proven on M5Core2 v1.1. For a remote the AXP2101 earns its place (not bling):

- **State-of-charge fuel gauge** — know the remote's battery % *before* a ride
  (a safety thing, not just UX).
- **Vibration motor** via DLDO1 — haptic feedback (see §8 — link-degradation
  buzz is the high-value use).
- **Separate power rails** (LDO/DCDC) for display, encoder, etc.
- **Fast/efficient charging**, battery/vbus/vsys/temperature ADC — and crucially
  the AXP2101 reads battery on **its own ADC**, keeping the C6 ADC out of the
  battery path (see §4).

Driver API notes (`axp2101-dd`, reviewed from `m5core2.../pmic.rs`):
- **[good]** Two-tier: convenience methods (`set_ldo_voltage_mv`, etc.) + a
  `.ll` register escape hatch (device-driver). Typed enums, unit-suffixed names,
  `bisync` (one source → sync+async).
- **[open]** Ensure the *headline* ops have top-level methods — SoC is read via
  `.ll.battery_percentage()` in the demo while `get_battery_level()` exists;
  audit that the common 90% (SoC, charging, vbus-good, voltages) are all
  top-level so consumers don't reach into `.ll`.
- **[open]** `write` vs `modify`: `write` clobbers other fields of a register.
  Add doc notes on multi-field registers.

---

## 4. Throttle sensor — I2C magnetic encoder, not the C6 ADC

**[decided]** The ESP32-C6 SAR ADC is mediocre (nonlinear, noisy,
calibration-dependent). Don't put the safety-critical throttle on it. Use an
**I2C magnetic angle sensor**: contactless, clean digital angle, joins the
existing I2C bus (PMIC + display), **and the C6 ADC is then out of every path
that matters** (battery is on the AXP2101 ADC; throttle on the encoder).

- **AS5600 (12-bit)** for throttle-only — simpler, cheaper (~$1), ubiquitous,
  has a hardware programmable angle range (maps a partial trigger sweep to full
  scale), magnet-status diagnostics (MD/ML/MH). 12-bit is already overkill for a
  throttle.
- **MT6701 (14-bit)** if one part should serve **both** the throttle **and** a
  future motor encoder (AS5600 is too slow/low-res for FOC commutation; MT6701
  does both, ABZ/SSI/I2C). Fits the "one part + one `mt6701-dd` crate" pattern.
- **[decided]** SoC/throttle split makes the C6 ADC quality a non-issue.

**Safety bonus:** the encoder's field-strength diagnostics ("magnet missing /
too weak") give a **detectable throttle-sensor fault** → failsafe. An analog
hall + ADC reads a failed sensor as some arbitrary voltage that could look like
throttle. The I2C encoder makes sensor failure *legible*.

**Mechanics:** diametric magnet (6×2.5 mm) on the trigger pivot over the sensor,
air gap ~0.5–3 mm, **spring return to neutral** = the mechanical deadman.
Contactless → no wear over the vehicle's life.

---

## 5. Throttle timing & command flow

Two distinct rates:

- **[decided] Poll the encoder at 100–200 Hz** + a light LP/median filter
  (cleans magnetic noise). Poll and send are decoupled.
- **[decided] Send to the controller at ~50 Hz** (every ~20 ms),
  **periodically** (not on-change), latest filtered value, **fire-and-forget**.

The periodic send is the **"affirm"** (the project's `AFFIRM_POLICY`): re-confirm
the current setpoint every cycle even if unchanged. Why:
1. It is the **deadman heartbeat** — the controller's 150 ms ISR staleness
   deadman is fed by affirms; they stop → failsafe.
2. **Loss self-heals** — a lost command is re-delivered by the next affirm
   ~20 ms later.
3. **Steady-state liveness** — holding the throttle steady still sends, so a held
   throttle isn't mistaken for a dead link.
4. **Fire-and-forget on purpose** — retries would *mask* a dying link; with
   periodic affirm a degrading link simply means affirms stop arriving and the
   deadman notices.

Other:
- **[decided] BLE connection interval ~15 ms.** Don't send faster than the
  interval — the radio can't deliver faster; latency is dominated by the BLE
  interval (the wired bridge↔MC hop is sub-ms).
- **[decided] Controller slew-limits the setpoint** (existing velocity/current
  ramps) so even at 50 Hz the torque is smooth, and a throttle jump isn't a
  lurch.

---

## 6. Wireless link — BLE primary, ESP-NOW as a measured plan B

The remote↔board control link. Both ends are ESP32-C6.

**[decided] BLE primary.** **[bench/later] ESP-NOW as plan B** — but framed as a
*tradeoff*, not an upgrade:

| Failure mode | Winner | Why |
|---|---|---|
| Narrowband interference / 2.4 GHz congestion | **BLE** | AFH hops off bad channels; ESP-NOW is single fixed channel |
| Connection drop + slow reconnect | **ESP-NOW** | connectionless — nothing to drop, next packet just goes |

Because the failure modes are **independent**, dual-link (both, controller takes
freshest, deadman handles staleness) is *complementary* redundancy — but on a
**single C6 radio** the coexistence cost (time-sharing ESP-NOW + BLE) can degrade
both, so dual-link is **not** a free win.

**[decided] Fix BLE first** rather than reach for ESP-NOW. You don't need a
perfect link — you need: rarely disconnect, reconnect fast, and the gap is safe
(the deadman already gives the last). So:
- **Avoid disconnects:** decouple BLE supervision timeout (1–2 s, ride out
  interference via AFH — safe because the 150 ms deadman handles freshness
  separately) from the app-layer deadman. Short conn interval (15 ms), slave
  latency 0, TX power up.
- **Fast reconnect:** bond once → skip re-pairing; **cache GATT handles** → skip
  service discovery (the biggest reconnect-latency killer); fast/directed
  advertising on disconnect; continuous scan on the central. → tens-to-low-
  hundreds of ms.
- The deadman makes any reconnect gap a *graceful slowdown*, not a hazard.

**[bench] ESP-NOW criterion:** move the throttle to ESP-NOW (off the BLE stack)
only if the bench shows BLE can't hold the **throttle + phone** connections with
the throttle's timing under the phone's telemetry load. That's a measurable
test, not a guess. (ESP-NOW would still need self-implemented encryption +
sequence/replay protection + liveness; the deadman story is unchanged.)

---

## 7. BLE roles

**[decided] Board = peripheral to both** the remote and the phone; both connect
as **centrals**.
- The phone *must* be central (phone BLE APIs are central-role). So the board is
  peripheral for the phone anyway.
- The remote being central too is fine: both ends are powered during a ride, so
  the remote can scan continuously for fast reconnect (board fast-advertises).
- Single role on the board = simpler stack/scheduling, and the safety-critical
  throttle link doesn't compete with role-switching. (Dual central+peripheral is
  possible but unnecessary.)

**[decided] Per-connection tuning:** throttle connection = short interval +
priority; phone connection = relaxed interval + larger MTU/2M PHY for the
decimated telemetry stream. The phone's stream must not starve the throttle's
connection events.

---

## 8. BLE stack — TrouBLE

**[decided]** `trouble` (embassy-rs), not NimBLE — pure Rust, async,
embassy-native, no C blob; coherent with the all-Rust stack. Tradeoff: it is
**pre-1.0** ("future goal of qualification", basic GATT) vs NimBLE's maturity.

Confirmed from the source: `HostResources<…, CONNS, CHANNELS, ADV_SETS, BONDS>`
— multi-connection via the `CONNS` const, both `central` and `peripheral`
features can be enabled together, bonding built in (`BONDS`), L2CAP CoC with
credit management. So the §7 design is expressible (`CONNS=2`, peripheral,
bonding).

**[bench] Validate the pre-1.0 paths on hardware:**
- Multi-connection (peripheral to 2 centrals) — examples are mostly
  single-connection; structurally supported but under-exercised.
- **Per-connection parameter control** (tight throttle vs relaxed phone) — verify
  the API exposes it.
- The combo: trouble (host) + esp-hal/esp-radio BLE **controller** over HCI +
  multi-connection + the throttle's timing under phone-stream load (this is the
  measurable ESP-NOW-fallback criterion from §6).

---

## 9. ergot over BLE — GATT, custom UUIDs

**[decided] GATT for external (host + app) BLE**, not L2CAP CoC. Reason:
**Windows has no app-level BLE L2CAP CoC**, and desktop BLE config is a **hard
requirement** — a built skateboard's USB is enclosed (reaching it = disassembly),
so a laptop's only practical link is BLE. VESC Tool does exactly this (its BLE is
a GATT/Nordic-UART-style service). Not matching = a real UX downgrade.

GATT-only is also a *good* fit, not just a Windows concession:
- **Universal** — every desktop (Win/Lin/mac) + phone.
- **Write-without-response for the throttle** is unreliable at the ATT layer,
  which matches the latest-value-wins / fire-and-forget model better than
  L2CAP's reliable in-order delivery. *(Caveat: the BLE **link layer** still
  retransmits + orders every PDU regardless — so staleness must be handled with
  a sequence number, see §10, not assumed from "without response".)*
- **Decimated telemetry fits GATT** — full 20 kHz × 46 B = 7.4 Mbps is USB-only
  on *any* transport; over BLE the stream is decimated to a few hundred Hz–few
  kHz, which GATT-notify (DLE + 2M PHY + 247 MTU) carries fine.

**[later] L2CAP CoC** as an optional perf upgrade for the **phone** telemetry
(Android/iOS/Linux/macOS support it) if GATT throughput ever limits live charts.
ergot's transport abstraction lets it be added without app rework; Windows stays
GATT. The internal board↔remote link could also be L2CAP (both ESP), but
GATT-everywhere is simpler and the throttle semantics favor write-without-
response.

**[decided] Custom 128-bit UUIDs**, not the Nordic UART Service UUIDs:
- Semantic identity — host/app filters by the oxifoc service UUID; random NUS
  terminal apps don't think they own the controller (a mild safety point for a
  motor controller), and we don't accidentally match other NUS devices.
- 128-bit UUIDs are free (no SIG registration). nRF Connect still works for
  debug by addressing the UUIDs manually.
- **Mirror the NUS structure** under our UUIDs: one `RX` (write-without-response,
  host→device) + one `TX` (notify, device→host). One "oxifoc ergot transport"
  service, all peers (remote/phone/desktop) speak ergot over it. (If L2CAP is
  added later, hand the PSM out via a small extra characteristic in the same
  service.)

---

## 10. Command latency & staleness — the safety/UX core

**Late commands are expected under interference**, not a rare bug. Sources: BLE
link-layer retransmission (delays even write-without-response — the "unreliable"
is only ATT-layer), multi-hop queueing, 2.4 GHz congestion, brief disconnects.

**Already safe by design** (existing mechanisms):
- **Slew-limit** at the controller → no step, smooth.
- **50 Hz affirm** → a single loss is covered by the next affirm.
- **150 ms ISR deadman** → sustained loss → controlled failsafe (graceful
  slowdown per policy).

The two worst cases converge to safe: *(released throttle, "zero" delayed)* —
either the zero arrives < 150 ms or the deadman trips, motor backs off either
way (residual: ≤150 ms held last setpoint, tunable). *(stale "full" delayed,
arrives late)* — at most a slew-limited bump before the next "zero".

**The Flipsky experience** (occasional 0.5–1 s delayed throttle) = "stale command
applied late" + "no feedback the link is degraded". The discomfort is the
**surprise** (primed for instant response, none, then a late surge), not just the
delay. Three additions, layered:

1. **[decided/cheap] Sequence number, latest-wins.** Each command carries a
   monotonic seq; the controller applies only the highest seq, **drops older**.
   Kills "stale backlog applied late" for loss/reordering/reconnect-flush —
   newer affirms supersede the stuck command. No clock sync needed. Probably
   already beats Flipsky (which seems to lack any staleness drop).
2. **[decided/high-value] Haptic on link degradation.** Buzz the remote
   (AXP2101 vibration) when RTT/staleness exceeds a threshold. The discomfort was
   the *surprise* — feeling "link is laggy" means a late response is *expected*,
   not alarming. Cheap, directly fixes the UX, and uses the vibration motor for a
   real purpose. **Don't drop stale commands silently — tell the rider.**
3. **[later] TTL with loose clock-sync.** For *sustained end-to-end latency*
   (every command 1 s late but in order — receive-time deadman + seq both look
   healthy, only absolute age catches it): each command carries a send timestamp;
   the controller drops anything older than a TTL (~150 ms). Only **loose** (ms)
   sync is needed for a 150 ms TTL — a simple ping-pong offset estimate, re-synced
   for drift, **not** PTP. Carry both seq (ordering, no sync) and timestamp (TTL,
   loose sync) in the command — complementary.

Layering summary: safety (graceful slowdown) is handled by the failsafe;
comfort/UX (no stale surge + no surprise) is seq-drop → haptic → TTL.

---

## 11. Wired bridge ↔ motor-controller link

Where the bridge (ESP32-C6) meets the STM32 motor controller. Decided by
physical integration + EMI, **not** bandwidth/latency (both negligible vs the
BLE interval).

| | When | Cost |
|---|---|---|
| **[decided] UART** | first hardware; and production if the C6 is on-board the controller PCB | none (ergot COBS-stream transport exists) |
| **[later] CAN FD peer** | bridge is a separate module on a cable in the vehicle | MCP2518FD on the C6 (its TWAI is classic-CAN only, can't join an FD backbone) + a driver crate |
| Classic-CAN spur | want differential robustness without an extra IC, bridge behind one MC | not on the FD backbone (separate bus) |
| **SPI — rejected** | — | master/slave fights ergot's symmetric model (needs a DATA_READY GPIO + handshake), slave role is fiddly, more single-ended lines (incl. clock) = worse on a noisy cable, and its bandwidth is wasted (BLE is the real bottleneck) |

**[decided]** Start UART. ergot decouples transport, so the production choice
(UART if on-board; CAN FD if a separate cabled module) can be deferred until the
physical layout is fixed — no firmware rework.

---

## 12. Bench validation checklist (remote-specific)

- **[bench]** Failsafe on lost BLE / lost CAN-hop — the first thing, before any
  ride: jam packets and confirm seq-drop + deadman give a *graceful slowdown*,
  not a stale-throttle surge.
- **[bench]** trouble + esp-controller multi-connection: throttle + phone
  simultaneously, throttle timing under phone telemetry load (the ESP-NOW-
  fallback criterion).
- **[bench]** BLE reconnect latency with bonding + cached handles + fast
  advertising (target tens-to-low-hundreds of ms).
- **[bench]** Encoder magnet-fault detection → failsafe.
- **[bench]** Haptic latency-warning threshold tuning (RTT/staleness).
- **[bench]** Throttle feel: slew-limit + 50 Hz affirm + 15 ms BLE interval — is
  the response immediate enough? Tune the staleness window (≤150 ms hold).

---

## 13. Decision log (summary)

| Topic | Decision | Key reason |
|---|---|---|
| HW approach | SoM core now, custom PCB after rides | defer *ergonomics*, not electronics |
| Module | ESP32-C6-MINI-1U-N4 | external antenna (hand-detuning), smallest, reflow OK |
| PMIC | AXP2101 | own proven driver; SoC + haptic earn it on a remote |
| Throttle | I2C magnetic encoder (AS5600 / MT6701) | bypass the mediocre C6 ADC; fault-detectable |
| PCB | 4-layer, reflow | PMIC switchers + safety analog (not RF); QFN forces reflow |
| Send rate | 50 Hz periodic affirm, fire-and-forget | deadman heartbeat + loss self-heal |
| Wireless | BLE primary; ESP-NOW plan B (measured) | fix BLE reconnect first; ESP-NOW is a tradeoff not upgrade |
| BLE roles | board peripheral to both; remote+phone central | phone must be central; single role = simpler |
| BLE stack | TrouBLE | pure-Rust embassy-native; pre-1.0 (validate multi-conn) |
| ergot/BLE | GATT, custom 128-bit UUIDs (NUS structure) | Windows has no BLE L2CAP; desktop BLE is required |
| Staleness | seq latest-wins + haptic warn + (later) TTL | kill late-stale surge; *tell the rider* it's laggy |
| Bridge↔MC | UART now; CAN FD if separate cabled module | integration/EMI decides; SPI rejected |

---

## Reference map

- `oxifoc-remote/` — the remote firmware (ESP32-C6, riscv32imac)
- `oxifoc-bridge/` — the BLE↔wired bridge (ESP32-C6)
- `axp2101-dd` (own, published) — PMIC driver
- `~/code/rust/m5core2v1-1_esp-hal_demo/src/pmic.rs` — AXP2101 usage reference + M5Core2 v1.1 rail map
- `trouble/` (`~/code/rust/trouble`) — BLE host: `HostResources<…CONNS…BONDS>`, central+peripheral features, L2CAP CoC
- `docs/safety.md` — `AFFIRM_POLICY`, the layered failsafe + 150 ms ISR deadman
- `docs/startup-and-sampling.md` — failsafe / flying-restart context

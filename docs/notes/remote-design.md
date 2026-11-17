# Handheld Remote — Design Notes

The current `oxifoc-remote` prototype runs on a nice!nano v2 clone
(nRF52840) with the Voyager SSD1306 wiring. It is a BLE central for
`oxifoc-bridge`, which runs on an ESP32-C5. The planned custom PCB keeps the
nRF52840 and adds an AXP2101 PMIC and a vibration motor (§2–§3).

> **STATUS: design (forward-looking), partly implemented.** The BLE link
> (remote central ↔ C5 peripheral, ergot over a NUS-shaped GATT service),
> the C5 ↔ STM32 SPI link, and the controller's session/seq ownership,
> affirm and ISR deadman exist. The remote does not send throttle yet; only
> the `motor-test` bench image drives the motor. Per-section status uses the
> markers below.

Design decisions for the ESK8 throttle remote and its link to the motor
controller, captured from the 2026-06-12 design session. Status markers:
**[decided]** · **[open]** · **[bench]** (validate on hardware) · **[later]**
(deferred, not v1).

The remote is a handheld with a throttle trigger, a battery, optional
display/haptics, talking to the board over a wireless link that routes through
the `oxifoc-bridge` (ESP32-C5) into the motor controller. The prototype is an
nRF52840 with a potentiometer trigger read by the SAADC. The whole network is
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
  (BLE → C5 bridge → SPI → MC) + motor + failsafe **without a rider**. No
  remote hardware needed; catches ~80% of issues.
- **Stage 1 (first ride):** minimal handheld = nice!nano v2 clone
  (nRF52840) + potentiometer trigger on the SAADC + LiPo + 3D-printed grip with a spring-return trigger
  (the mechanical deadman). Reuse the `oxifoc-remote` firmware.
- **Stage 2 (after rides):** custom PCB, designed to the *learned* ergonomics
  and throttle feel — not guessed. The electronics are all proven by then, so
  the spin is "integrate proven blocks," low risk; the prototype existed to fix
  *feel and layout*, not to prove the parts work.

The deferral rationale is **ergonomics/feel unknowns**, not electronics risk —
every block (AXP2101, nRF52840, SAADC trigger) is already de-risked or
running on the prototype.

---

## 2. Core module (SoM)

**[decided] MCU: nRF52840** (revised 2026-11-17, replaces the ESP32-C6
plan). Reasons: low power for a battery handheld, and a usable SAADC for
the v1 potentiometer trigger (§4). The first iteration runs on a
nice!nano v2 clone (§1, Stage 1). The custom PCB uses the same chip.

**[decided]** A compact core module, sized to drop into "roughly any" remote
enclosure. On-module = invariant; off-module = ergonomics-dependent.

**On the module:** nRF52840 + AXP2101 + vibration motor driver (DLDO1) +
LiPo connector (JST) + charging + power rails + USB-C. An **expansion
interface** (board-to-board / FFC / header) breaks out: the trigger analog
input, I2C (PMIC + display share the bus), a few GPIO (buttons),
power-key, 3V3/GND.

**Off the module** (iterate with the printer): the trigger mechanism and its
position, display, buttons, grip shape.

Module also makes a reusable open-source artifact (like the crate stack —
`drv8301-dd`, `axp2101-dd`, `ergot`): an "nRF52840 + AXP2101 core" is
useful to anyone building a battery-powered BLE handheld.

### PCB

- **[decided] 4-layer.** The drivers are: (a) the AXP2101 is a switching
  PMIC whose DCDC loops want a solid ground plane, and (b) the throttle is
  safety-critical analog (a potentiometer on the SAADC in v1) whose
  reference/return must stay clean of switcher noise. At proto quantities
  the 4-layer cost delta is ~a few dollars; it turns "carefully route
  returns under a switcher" into "drop a ground plane." Stack: top sig /
  GND / PWR / bottom sig.
- **[decided] Reflow assembly.** The AXP2101 QFN (thermal pad) mandates
  reflow regardless of the radio part. Order PCB + stencil from JLCPCB
  (~$7); hotplate is fine for this size. **Escape hatch:** JLCPCB PCBA.
- **[decided] USB-C, dual purpose:** the nRF52840's native USB for
  flashing (bootloader) and the same VBUS feeds the AXP2101 for charging.
  Keep SWD pads for debug.
- **[later] Qwiic/STEMMA I2C connector** for rapid prototype attach of
  display/sensor without soldering.
- Reuse the **M5Core2 v1.1 schematic** (`Sch_Core2_v1.1`) as the proven AXP2101
  rail/charging reference — don't design the PMIC circuit from scratch.

### Radio part — **[open]**

Bare nRF52840 vs a certified module is open. Criteria carried over from the
ESP32-C6 module study:

- **Antenna away from the hand.** A hand wrapped around the grip detunes a
  PCB antenna — the #1 RF failure mode of a handheld. Prefer a module or
  layout with an external (U.FL) antenna placed at the top of the remote,
  unless the enclosure keeps a PCB/chip antenna clear of the hand. Loss of
  this link = throttle loss.
- **Size and pads.** Bottom-pad modules cost nothing extra, since the
  AXP2101 forces reflow anyway.
- **Certification.** A pre-certified module avoids RF layout and
  certification work for a small run.

---

## 3. PMIC — AXP2101

**[decided]** De-risked: it is the user's own published `axp2101-dd` driver,
proven on M5Core2 v1.1. For a remote the AXP2101 earns its place (not bling):

- **State-of-charge fuel gauge** — know the remote's battery % *before* a ride
  (a safety thing, not just UX).
- **Vibration motor** via DLDO1 — haptic feedback (see §10 — link-degradation
  buzz is the high-value use).
- **Separate power rails** (LDO/DCDC) for display, encoder, etc.
- **Fast/efficient charging**, battery/vbus/vsys/temperature ADC — and crucially
  the AXP2101 reads battery on **its own ADC**, leaving the SAADC to the
  trigger (see §4). The nice!nano prototype reads VDDH/5 on the SAADC
  instead.

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

**[decided] v1 (2026-11-17):** the first remote uses a potentiometer trigger
on the nRF52840 SAADC. The nRF52840 SAADC is calibrated and supports
oversampling, so the C6 ADC argument below does not carry over directly.
**[open]** Make pot faults detectable: keep the usable travel away from both
rails so an open wiper, open end or short reads out of range and becomes a
throttle fault (send neutral, never a value). A magnetic encoder stays the
option for a later revision.

The original ESP32-C6 reasoning follows.

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

## 6. Wireless link — BLE only

The remote↔board control link. The remote is an nRF52840 and the bridge is an
ESP32-C5.

**[decided] BLE only.** **[later] ESP-NOW is deferred and not planned**
(2026-11-17). It needs an Espressif chip at both ends, and the remote is an
nRF52840. The comparison below is kept in case the decision is revisited. It
is a *tradeoff*, not an upgrade:

| Failure mode | Winner | Why |
|---|---|---|
| Narrowband interference / 2.4 GHz congestion | **BLE** | AFH hops off bad channels; ESP-NOW is single fixed channel |
| Connection drop + slow reconnect | **ESP-NOW** | connectionless — nothing to drop, next packet just goes |

Because the failure modes are **independent**, dual-link (both, controller takes
freshest, deadman handles staleness) is *complementary* redundancy — but on a
**single C5 radio** the coexistence cost (time-sharing ESP-NOW + BLE) can degrade
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

**[bench] Revisit criterion:** reconsider a second radio link only if the
bench shows BLE can't hold the **throttle + phone** connections with
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
  multi-connection + the throttle's timing under phone-stream load (the
  revisit criterion from §6).

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

### Drive architecture — three axes (decided 2026-09-10)

Supersedes the 2026-08-13 "role resolution" section. The design was
re-cut along three independent axes; the earlier role/root debate was a
consequence of conflating them.

**Axis 1 — what the remote sends: vehicle INTENT, never a motor command.**
`DriveIntentTopic` payload: `session: u64, seq: u32, source: SourceClass,
drive: f32 ∈ [−1, 1]` (positive = drive, negative = brake, 0 = released =
safe mode, which is also the re-arm), `mode: u8` (eco/sport/reverse —
interpreted by the controller). Published at 50 Hz as the affirm
(unchanged deadman semantics). No TTL/timestamp yet (§10 item 3 stays
[later]). One signed channel instead of throttle+brake rules out "both at
once" by construction.

**Axis 2 — transport: a TOPIC, not an endpoint.** The remote needs no
destination address, no discovery, no re-resolution after reconnect:
routers flood the topic across the tree (one transmission on a shared
bus), every subscribed controller receives it. Feedback is symmetric —
each controller publishes `DriveStatusTopic` (~50 Hz: `session_obeyed,
failsafe_state, erpm, iq, fet_temp, fault_generation, limp`), the remote
subscribes and tells senders apart by `src`. The only address the remote
ever uses is its link-local peer (the bridge) for ping/RTT haptics. The
host's per-controller endpoints + SocketQuery resolution remain the tool
for config/diagnostics — see the host selector follow-up in TODO.md.

**Axis 3 — where intent→per-motor mapping lives: REPLICATED on every
controller.** Vehicle logic (torque ratio, brake bias, bus-current
share, modes; later sibling-derate) is a portable core module with a
pure `map(intent, my_vehicle_config, sibling_states) → MotorCommand`;
each controller runs it locally. Rejected for now: a drive master
(VESC-style fan-out — puts a node in the torque path, needs remote-side
resolution) and a gateway on the radio MCU (authority on the least
stable node; two code paths on the bench). Door kept open: a
*coordinator* that only publishes correction coefficients (also a
topic, its death decays to 1.0) can run on the C5 later without
touching the remote or the controllers.

**Ownership across sources.** `SourceClass` order `Remote > BenchHost >
Phone`: an active setpoint is accepted if its class ≥ the current
owner's; equal class keeps today's session/seq rule; safe modes bypass
as today. Two remotes of equal class = first-come, with indication.

**Vehicle config (new append-only group #12, ICD v5 with the topics):**
`position` (FL/FR/RL/RR/single), `torque_ratio`, `brake_bias`,
`bus_share`. The sibling set is NOT config — sibling-deadman applies to
whoever was alive at arm time. Root/upstream selection is C5 config, not
controller config (see Topology).

**Topology (revised 2026-11-17): every controller is an SPI edge behind
its own ESP32-C5. The C5s are the routers.** Single motor: one C5 is the
root Router; the STM32 is its only SPI DirectEdge, with no USB or other
ergot transport. Two or more motors: each controller keeps its own C5, and
the C5s join one CAN FD bus through their on-chip TWAI-FD controllers. One
C5 is root by config. The others run as bridge-mode routers with
`upstream = Can`. Root-ness must be config, because an ergot bridge-mode
router cannot degrade to root when its upstream never appears (its
downstream nets are leased from upstream). The controller firmware is the
same in both layouts. Only the C5 config changes. Design-ins: (1)
`upstream: None | Can` in the C5 config; (2) the bridge's upstream
lifecycle (discovery ping, lease acquire/refresh/release) as a reusable
module in the bridge crate; (3) nothing master-centric in the bridge.
With intent as a topic the application layer is topology-blind.
**[open]** Which C5 carries the remote's BLE link, and whether each C5
advertises separately. A dead root C5 is a sibling-loss event for the
other controllers (see below).

**Multi-motor failsafe — symmetric, always brake.** See
[safety.md → Multi-motor vehicles](../safety.md#multi-motor-vehicles--symmetric-failsafe).
Summary: an isolated controller cannot distinguish "sibling pushing"
from "downhill" by any local physics, so it may only brake; symmetry is
produced by the hearing side — every controller also holds a
sibling-deadman and enters failsafe on sibling silence or a sibling's
failsafe status; a coast sync window precedes braking so no asymmetric
torque exists while the others catch up. Limp-home is never automatic
mid-ride. Sibling-aware *terminal state* (coast when siblings drive)
was considered and REJECTED — the signal is unavailable exactly when
needed.

## 11. Wired bridge ↔ motor-controller link

Where the bridge (ESP32-C5) meets the STM32 motor controller.

**[decided, implemented on MK5] SPI, STM32 as ergot DirectEdge and SPI
master, C5 as root Router and SPI slave**, with a `ready` GPIO from the C5
(`oxifoc-bridge/src/transport/spi.rs`; wiring in
[vesc6-mk5.md](../hw/vesc6-mk5.md#c5-bridge-wiring-and-continuity-check)).
This replaces the earlier UART choice. The reasons:

- **Bandwidth for fast telemetry.** Native-rate Packed28 streams go from the
  STM32 to the C5 and out over USB Serial/JTAG, later possibly Wi-Fi. BLE is
  no longer the only consumer, so the link must carry the full stream, which
  UART cannot.
- **Less work on the motor MCU.** The STM32 has one ergot transport and runs
  as an edge. It forwards nothing between nodes and has no USB stack in the
  release image. Routing, USB, BLE and later Wi-Fi run on the C5.
- **The old objections are handled.** The STM32 drives the clock and the
  DMA schedule. The C5's `ready` line replaces the DATA_READY handshake. The
  link is a short on-board or harness-internal connection, not a cable
  through the vehicle.

Cost: the C5 is now in the command path for every host and remote. A C5
hang or reset looks like link loss to the controller, and the link gate and
the ISR deadman cover it. Host USB connects to the C5, not the STM32. The
STM32 keeps separate USB and RTT diagnostic build profiles for bench work.

Inter-controller links use CAN FD between C5s, not between STM32s. The F405
has only bxCAN (classic), while the C5 has two TWAI controllers that
support FD frames. esp-hal already has the TWAI-FD driver for the C5
(upstreamed by the project author). The ergot CAN transport on top of it
is the remaining work. See Topology in §10.

---

## 12. Bench validation checklist (remote-specific)

- **[bench]** Failsafe on lost BLE / lost CAN-hop — the first thing, before any
  ride: jam packets and confirm seq-drop + deadman give a *graceful slowdown*,
  not a stale-throttle surge.
- **[bench]** trouble + esp-controller multi-connection: throttle + phone
  simultaneously, throttle timing under phone telemetry load (the §6
  revisit criterion).
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
| Remote prototype | nice!nano v2 clone (nRF52840) | current `oxifoc-remote` |
| Remote custom PCB | nRF52840 + AXP2101 + vibration motor | low power, usable SAADC; fuel gauge + haptics; radio part open |
| PMIC | AXP2101 | own proven driver; SoC + haptic earn it on a remote |
| Throttle | v1: potentiometer on nRF52840 SAADC; later: magnetic encoder | simple first trigger; pot faults must read out of range |
| PCB | 4-layer, reflow | PMIC switchers + safety analog (not RF); QFN forces reflow |
| Send rate | 50 Hz periodic affirm, fire-and-forget | deadman heartbeat + loss self-heal |
| Wireless | BLE only; ESP-NOW deferred, not planned | nRF52840 remote; fix BLE reconnect first |
| BLE roles | board peripheral to both; remote+phone central | phone must be central; single role = simpler |
| BLE stack | TrouBLE | pure-Rust embassy-native; pre-1.0 (validate multi-conn) |
| ergot/BLE | GATT, custom 128-bit UUIDs (NUS structure) | Windows has no BLE L2CAP; desktop BLE is required |
| Staleness | seq latest-wins + haptic warn + (later) TTL | kill late-stale surge; *tell the rider* it's laggy |
| Bridge↔MC | SPI, STM32 edge + master, C5 root router + slave | fast telemetry bandwidth; no routing or USB on the motor MCU |
| Remote payload | vehicle intent, one signed channel | remote never knows motors; no throttle+brake conflict |
| Intent transport | topic (flood), 50 Hz affirm | no address/discovery on the remote; topology-blind |
| Intent→motor logic | replicated per controller, portable core module | no node in the torque path; coordinator later via correction topic |
| Multi-motor link loss | symmetric failsafe via sibling-deadman + coast sync window; always brake | isolated node can't tell sibling-push from downhill |
| Root | one C5 root by config; other C5s `upstream = Can` | bridge-mode can't self-promote; controllers stay SPI edges |
| Multi-motor bus | CAN FD between per-controller C5s (on-chip TWAI-FD) | F405 has no CAN FD; controller firmware unchanged |

---

## Reference map

- `oxifoc-remote/` — the remote firmware (nRF52840, `thumbv7em-none-eabihf`)
- `oxifoc-bridge/` — the root router on ESP32-C5: SPI to the controller, USB Serial/JTAG, optional BLE
- `axp2101-dd` (own, published) — PMIC driver
- `~/code/rust/m5core2v1-1_esp-hal_demo/src/pmic.rs` — AXP2101 usage reference + M5Core2 v1.1 rail map
- `trouble/` (`~/code/rust/trouble`) — BLE host: `HostResources<…CONNS…BONDS>`, central+peripheral features, L2CAP CoC
- `docs/safety.md` — `AFFIRM_POLICY`, the layered failsafe + 150 ms ISR deadman
- `docs/startup-and-sampling.md` — failsafe / flying-restart context

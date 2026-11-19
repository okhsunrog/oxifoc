# Handheld remote — design notes

Design of the eSK8 throttle remote and its link to the motor controller.
Markers: **[decided]**, **[open]**, **[bench]** (validate on hardware),
**[later]** (not v1).

**Status (2026-11-19).** The prototype `oxifoc-remote` runs on a nice!nano v2
clone (nRF52840) with an SSD1306 display. It is a bonded BLE central of
`oxifoc-bridge` (ESP32-C5) and speaks ergot over the OxiFOC GATT service.
Implemented: bonding with the passkey on the remote's screen, cached GATT
handles, the C5 ↔ STM32 SPI link, and on the controller the session/seq
ownership, affirms and the ISR deadman. The remote does not send throttle
yet; only the `motor-test` bench image drives the motor. The drive intent
model (§10) is designed, not implemented.

The network is [ergot](https://github.com/jamesmunns/ergot), so a transport
can change without touching the application.

## 1. Hardware strategy

**[decided]** Prototype first, custom PCB after rides. The unknowns are
ergonomics and throttle feel, not electronics: every block (nRF52840,
AXP2101, SAADC trigger) already runs on the prototype or on another board.

- **Stage 0:** drive from a laptop or phone with the board on a stand. This
  exercises BLE → C5 → SPI → controller and the failsafe without a rider.
- **Stage 1 (first ride):** nice!nano v2 clone + potentiometer trigger on
  the SAADC + LiPo + 3D-printed grip with a spring-return trigger, which is
  the mechanical deadman.
- **Stage 2:** custom PCB built around the ergonomics learned in Stage 1.

## 2. Core module and PCB

**[decided] nRF52840** (2026-11-17, replaces the ESP32-C6 plan): low power
for a battery handheld and a usable SAADC for the trigger.

**[decided]** A small core module holds what does not depend on
ergonomics: nRF52840, AXP2101, vibration motor driver (DLDO1), LiPo
connector, charging, rails and USB-C. An expansion connector breaks out the
trigger input, I2C (PMIC and display), a few GPIOs, the power key and
3V3/GND. The trigger mechanism, display, buttons and grip stay off the
module and iterate with the printer.

- **[decided] 4 layers.** The AXP2101 switchers need a solid ground plane,
  and the analog throttle reference must stay clean of switching noise. At
  prototype quantities the extra cost is a few dollars.
- **[decided] Reflow.** The AXP2101 QFN has a thermal pad. Stencil plus
  hotplate is enough; JLCPCB PCBA is the fallback.
- **[decided] USB-C** for both flashing (native USB bootloader) and
  charging (VBUS to the AXP2101). Keep SWD pads.
- Use the M5Core2 v1.1 schematic as the AXP2101 rail and charging reference.
- **[later]** Qwiic/STEMMA connector for quick display or sensor experiments.

**[open] Radio: bare nRF52840 or a certified module.**
- A hand around the grip detunes a PCB antenna, and losing this link means
  losing the throttle. Put the antenna (or a U.FL antenna) at the top of
  the remote, clear of the hand.
- Bottom-pad modules cost nothing extra since the PMIC already forces reflow.
- A pre-certified module avoids RF layout and certification work.

## 3. PMIC — AXP2101

**[decided]** Driven by the published `axp2101-dd` crate, already proven on
an M5Core2 v1.1. It provides:

- a fuel gauge, so the rider knows the remote's charge before a ride;
- the vibration motor on DLDO1 for link-degradation warnings (§9);
- separate rails for display and sensors;
- charging and its own battery ADC, which leaves the SAADC to the trigger.
  The nice!nano prototype measures VDDH/5 on the SAADC instead.

## 4. Throttle sensor

**[decided] v1:** a potentiometer trigger on the nRF52840 SAADC, which is
calibrated and supports oversampling.

**[open]** Make pot faults detectable. Keep the usable travel away from
both rails, so an open wiper, an open end or a short reads out of range and
becomes a throttle fault: send neutral, never a value.

**[later]** An I2C magnetic angle sensor: contactless, no wear, and
magnet-missing/weak diagnostics that turn a sensor failure into a fault.
AS5600 (12-bit) is enough for a throttle. MT6701 (14-bit) could also serve
as a motor encoder. Mechanics: a diametric magnet on the trigger pivot,
0.5–3 mm air gap, spring return to neutral.

## 5. Throttle timing

- **[decided]** Sample the trigger at 100–200 Hz with a light low-pass or
  median filter. Sampling and sending are decoupled.
- **[decided]** Send the latest filtered value at about 50 Hz, periodically
  rather than on change, fire-and-forget. Each send is an affirm:
  1. It feeds the controller's 150 ms ISR deadman; if affirms stop, the
     failsafe engages.
  2. A lost command is replaced by the next affirm about 20 ms later.
  3. A steady throttle still sends, so it is not mistaken for a dead link.
  4. No retries: they would hide a dying link from the deadman.
- **[decided]** The controller slew-limits the setpoint, so a 50 Hz command
  stream gives smooth torque and a throttle jump is not a lurch.
- Do not send faster than the BLE connection interval (15–20 ms); the radio
  cannot deliver faster, and the C5 → STM32 hop is sub-millisecond.

## 6. Wireless link — BLE only

**[decided] BLE only.** ESP-NOW is not planned: it needs Espressif chips at
both ends, and the remote is an nRF52840.

The goal is a link that rarely drops, reconnects fast, and is safe during a
gap (the deadman covers the gap):

- **Avoid drops:** the BLE supervision timeout (1 s) is separate from the
  150 ms application deadman, so AFH can ride out interference. Connection
  interval 15–20 ms, peripheral latency 0. *(Implemented.)*
- **Fast reconnect:** bonding skips re-pairing and cached GATT handles skip
  service discovery. *(Implemented.)* Fast advertising on disconnect and
  continuous scanning on the remote.

Measurements and the reliability plan:
[ble-link-reliability.md](ble-link-reliability.md).

## 7. BLE roles and stack

**[decided]** The bridge is a peripheral; the remote, phones and laptops are
centrals. Phones can only be centrals anyway, both ends are powered during
a ride, and a single role on the bridge keeps the stack simple. The bridge
accepts two centrals at once. Only bonded devices reach ergot; see
[ble-pairing.md](ble-pairing.md).

**[decided] TrouBLE** (embassy-rs): pure Rust, async, no C blob. It is
pre-1.0, so its less-exercised paths need hardware validation.

**[bench]** Throttle timing with a phone streaming telemetry on the second
connection. The phone's connection must not starve the remote's connection
events.

## 8. ergot over BLE — GATT

**[decided] GATT, not L2CAP CoC.** Windows has no application-level L2CAP
CoC, and a laptop must work over BLE because the USB port of a built board
is inside the enclosure. GATT works on every desktop and phone. Full-rate
telemetry is USB-only; BLE carries a decimated stream, which GATT notify
handles. The BLE link layer still retransmits and orders PDUs even for
write-without-response, so staleness is handled with sequence numbers (§9).

**[later]** L2CAP CoC for phone telemetry if GATT throughput limits live
charts. The ergot transport abstraction allows adding it.

**[decided, implemented] Custom 128-bit UUIDs** with the NUS structure.
Service `0b960001-d748-4b2e-b03a-6ada6467f15d`, RX `…0002…`
(write-without-response, host → device), TX `…0003…` (notify), defined in
`oxifoc_core::ble`. The bridge advertises the service UUID and hosts filter
on it, so generic NUS apps and other NUS devices do not match.

## 9. Late and stale commands

Late commands are expected under interference: link-layer retransmission,
queueing, 2.4 GHz congestion, short disconnects. The existing safety
mechanisms already bound the outcome:

- released throttle with a delayed zero: the zero arrives within 150 ms or
  the deadman trips;
- a stale "full" arriving late: at most a slew-limited bump before the next
  value.

The remaining problem is comfort. On a Flipsky remote, throttle sometimes
arrives 0.5–1 s late with no warning; the surprise is worse than the delay.
Three layers:

1. **[decided] Sequence number, latest wins.** The controller drops commands
   older than the highest sequence seen, so a stale backlog is never
   applied late. No clock sync needed.
2. **[decided] Haptic warning** when RTT or staleness exceeds a threshold.
   A rider who feels that the link is laggy expects a late response.
3. **[later] TTL with loose clock sync,** for sustained end-to-end latency
   that seq and the receive-time deadman cannot see. Each command carries a
   send timestamp; the controller drops anything older than about 150 ms.
   Millisecond-level sync from a ping-pong offset estimate is enough.

## 10. Drive architecture (decided 2026-09-10, not implemented)

**What the remote sends: vehicle intent, never a motor command.**
`DriveIntentTopic { session: u64, seq: u32, source: SourceClass, drive: f32
∈ [−1, 1], mode: u8 }`. Positive drives, negative brakes, 0 is released
(the safe mode and the re-arm). `mode` selects eco/sport/reverse on the
controller. One signed channel rules out throttle and brake at once.
Published at 50 Hz as the affirm.

**Transport: a topic, not an endpoint.** The remote needs no destination
address, discovery or re-resolution. Routers flood the topic, and every
subscribed controller receives it. Each controller publishes
`DriveStatusTopic` (~50 Hz: obeyed session, failsafe state, eRPM, Iq, FET
temperature, fault generation, limp). The remote addresses only its
link-local peer, for RTT. Hosts keep per-controller endpoints for
configuration and diagnostics.

**Intent → motor mapping: replicated on every controller.** A portable core
function `map(intent, vehicle_config, sibling_states) → MotorCommand` runs
locally on each controller. Rejected: a drive master (VESC-style fan-out
puts a node in the torque path) and a gateway on the radio MCU (authority
on the least stable node). A C5 coordinator that only publishes correction
coefficients remains possible later.

**Ownership:** `SourceClass` order `Remote > BenchHost > Phone`. An active
setpoint is accepted if its class is at least the owner's; equal classes
keep the session/seq rule; safe modes always pass.

**Vehicle config** (new config group, ICD v5): `position`, `torque_ratio`,
`brake_bias`, `bus_share`. The sibling set is not configuration: the
sibling deadman covers whoever was alive at arm time.

**Topology (revised 2026-11-17).** Each controller is an SPI edge behind its
own ESP32-C5; the C5s are the routers. With several motors the C5s share a
CAN FD bus. One C5 is root by config, the others run as bridge-mode routers
with `upstream = Can`, because a bridge-mode router cannot become root when
its upstream never appears (its downstream nets are leased from upstream).
The controller firmware is the same in every layout.
**[open]** Which C5 carries the remote's BLE link, and whether each C5
advertises.

Multi-motor link loss is symmetric and always brakes; see
[safety.md → Multi-motor vehicles](../safety.md#multi-motor-vehicles--symmetric-failsafe).

## 11. Bridge ↔ motor controller link

**[decided, implemented on MK5] SPI.** The STM32 is the SPI master and an
ergot DirectEdge; the C5 is the SPI slave and the root router, with a
`ready` GPIO (`oxifoc-bridge/src/transport/spi.rs`; wiring in
[vesc6-mk5.md](../hw/vesc6-mk5.md#c5-bridge-wiring-and-continuity-check)).
This replaces the earlier UART plan:

- **Bandwidth.** Native-rate telemetry goes STM32 → C5 → USB Serial/JTAG,
  later Wi-Fi. UART cannot carry it.
- **Less work on the motor MCU.** The STM32 has one ergot transport,
  forwards nothing and has no USB stack in the release image. Routing, USB,
  BLE and later Wi-Fi run on the C5.

The remote path needed an external radio chip in any design. What changed
is that host USB now also goes through the C5. The STM32 keeps USB and RTT
diagnostic build profiles for bench work.

Controllers talk over CAN FD between their C5s. The F405 has only classic
bxCAN; the C5 has two TWAI controllers with FD support, and the TWAI-FD
driver is in esp-hal. The ergot CAN transport on top of it is the remaining
work.

## 12. Bench checklist

- **[bench]** Throttle timing with a phone streaming telemetry (§7).
- **[bench]** Reconnect latency with bonding, cached handles and fast
  advertising.
- **[bench]** Pot out-of-range faults → neutral (§4).
- **[bench]** Haptic warning threshold (§9).
- **[bench]** Throttle feel: slew limit, 50 Hz affirms, 15–20 ms interval.

## References

- `oxifoc-remote/` — remote firmware (nRF52840)
- `oxifoc-bridge/` — root router on the ESP32-C5: SPI, USB Serial/JTAG, BLE
- `axp2101-dd` — PMIC driver
- [safety.md](../safety.md) — layered failsafe and the 150 ms ISR deadman
- [startup-and-sampling.md](startup-and-sampling.md) — failsafe and flying
  restart
- [ble-pairing.md](ble-pairing.md), [ble-link-reliability.md](ble-link-reliability.md)

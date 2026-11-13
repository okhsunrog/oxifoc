# OxiFOC architecture

[Commissioning](commissioning.md) · [Safety](safety.md) ·
[Bench evidence](README.md#bench-and-research)

## Control cycle

On STM32, TIM1 drives complementary center-aligned PWM and triggers injected
ADC conversions. The ADC interrupt runs the shared control cycle with a new
sample. Board code owns pin assignments, ADC scaling, PWM and gate-driver
setup. The control loop, estimators and protection logic live in
[`oxifoc-core`](../oxifoc-core/src/lib.rs).

1. The platform reads phase currents, bus voltage and board-specific sensors.
   The [cycle handler][cycle] processes motor commands and protections before
   advancing the [FOC driver][driver].
2. The driver selects an angle and rotor-speed estimate from the
   [phase manager][phase]. FOC transforms phase currents into the rotating
   frame, regulates `Id` and `Iq`, and commits PWM through the board adapter.
   The estimator receives voltage aligned to the current sample. See
   [observer voltage input][voltage] for the measured/reconstructed paths.
3. The fast telemetry producer enqueues samples at the requested decimation.
   The async streaming task batches and sends them. The control interrupt
   does not wait for the receiver.

| Mode | Control behavior |
|---|---|
| Current | Regulate requested `Id` and `Iq` in amperes, subject to limits. |
| Throttle | Convert a signed fraction of the available drive or brake current into a request. |
| Velocity | The outer PI runs on a 1 ms grid and requests torque current. The current loop still runs at the native FOC rate. |
| Position | Relative multi-turn electrical position error feeds a speed request to the same velocity loop. Resolution follows the angle source. The motor API supports the mode. |
| Open loop / Direct voltage | Explicit phase reference for calibration, detection and diagnostics. |
| Stop / Coast / Brake | Neutral and braking behavior. See [safety](safety.md) for how these interact with ownership and the failsafe. |

See the [mode enum][modes] and [mode execution][execution] for the API and
controller paths.

## Rotor feedback

The commissioned `Hall` policy starts on calibrated Hall, then blends angle
and speed into the back-EMF observer as speed rises. It uses the configured
band and waits for a ready observer. `Observer` is the sensorless policy. It
can use an open-loop startup sequence from rest. `Auto` chooses the hybrid
when Hall calibration and a valid observer model are present, direct Hall
when only Hall is available, and the observer when only the model is
available. With neither, it selects Manual. `HallWithoutCrossover` uses
Hall across the speed range and still allows observer recovery on Hall loss.

The [feedback config][feedback] controls the commissioned policy. The
host's `source` command changes the runtime source while stopped. HFI uses
a separate estimator slot and is active only in applicable diagnostic or
crossover regimes. See the [source enum][sources] and [MK5 observer tuning
note](notes/hall-observer-tuning.md) for readiness, fallback, crossover and
bench data.

## Commands and configuration

The [ICD][icd] defines the typed endpoints and topics. A host connects
through ergot, then checks application protocol and the controller UUID
before opening command and telemetry access. The [host lifecycle][host]
runs this admission on each connection generation.

- Motor commands carry a source session and a sequence. The ISR acknowledges
  the result after handling the command. Affirmations refer to the applied
  setpoint and keep the deadman alive while that setpoint remains active.
- The [config server][config] validates and applies typed, revisioned groups.
  Only the groups marked live-safe can be changed while running. Persistence
  saves the requested revision while stopped. A failed flash write can leave
  a successful live change. PWM frequency is staged for the next boot.
- Commissioning operations have device-assigned session and operation IDs.
  Measurements run in a temporary mode. Saving the reviewed setup installs
  the motor model, feedback, Hall calibration if present, and current limits
  as one drive record. See [commissioning](commissioning.md).

## Transport and telemetry

On the MK5 SPI image, the STM32 is an ergot DirectEdge and the SPI master.
The ESP32-C5 is the root Router and SPI slave. The host reaches it over USB
Serial/JTAG. The STM32 has separate USB and RTT diagnostic build profiles.
See [MK5 bridge wiring and transport](hw/vesc6-mk5.md#c5-bridge-wiring-and-continuity-check).

The release MK5 recipe uses a [48 KiB CCM telemetry queue][streaming]. Startup
zeroes its NOLOAD section. SPI DMA buffers are in ordinary SRAM. The Packed28
capture path includes the commutation angle and the Hall angle when available.
A finite queue can overflow. Use the reported sample gaps, producer and host
drops and the recording end marker to qualify a capture.

## Virtual device

[`oxifoc-virtual`][virtual] runs the shared FOC driver and cycle handler
against a PMSM simulation. It is suitable for testing the command path,
estimators and control behavior with a simulated clock. It does not validate
interrupt deadlines, switching waveforms or board-specific sensing. See
[plant model limitations](notes/virtual-motor-fidelity.md).

[cycle]: ../oxifoc-core/src/state.rs
[driver]: ../oxifoc-core/src/motor/foc_driver/mod.rs
[phase]: ../oxifoc-core/src/foc/phase/manager/mod.rs
[voltage]: ../oxifoc-core/src/motor/foc_driver/driver.rs
[modes]: ../oxifoc-core/src/types.rs
[execution]: ../oxifoc-core/src/motor/foc_driver/modes.rs
[feedback]: ../oxifoc-core/src/storage.rs
[sources]: ../oxifoc-core/src/foc/phase/source.rs
[icd]: ../oxifoc-core/src/icd.rs
[host]: ../oxifoc-host-lib/src/connection.rs
[config]: ../oxifoc-core/src/runtime/servers.rs
[streaming]: ../oxifoc-core/src/runtime/streaming.rs
[virtual]: ../oxifoc-virtual/src/sim.rs

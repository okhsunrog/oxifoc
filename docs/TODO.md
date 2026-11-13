# Open work

Use the [bench and research index](README.md#bench-and-research) for results
and design notes. The [previous backlog snapshot](archive/backlog-through-2026-11-13.md)
preserves older review findings and experimental detail.

## Motor control and detection

- [ ] Compare the Hall crossover bands under repeatable load and test a
  physical single-wire Hall failure. Use the
  [observer tuning plan](notes/hall-observer-tuning.md) for signals and
  acceptance criteria.
- [ ] Tune velocity PI and acceleration with drivetrain inertia and load.
  Compare actual speed against an independent reference where the estimator
  is under test.
- [ ] Reproduce the historical sensorless torque oscillation with the
  current firmware before retuning the PLL, voltage model or λ adaptation.
  See the [older sensorless bench evidence](archive/backlog-through-2026-11-13.md#sensorless-startup-see-notesstartup-and-samplingmd).
- [ ] Qualify loaded sensorless cold start, flying restart in either
  direction, and loss of angle trust at low speed. Keep HFI standstill
  control and crossover as separate hardware tests.
- [ ] Compare the observer's voltage and phase lag at high modulation.
  Check the actuation delay and phase-voltage filters before changing PLL
  gains. See [phase sensing](notes/phase-sense.md).
- [ ] Compare AC inductance, fundamental inductance and the detector's
  estimate on representative motors. Decide what the observer and
  decoupling each need using the [inductance study](notes/bench-inductance-model-2026-11-09.md).
- [ ] Evaluate pole-pair auto-detection, encoder offset calibration and
  whether position control needs a dedicated host entry point.
- [ ] Evaluate field weakening, MTPA and overmodulation when a bench use
  case justifies them. Reference candidates are in the
  [comparison notes](notes/reference-fw-comparison.md).

## Safety

- [ ] Qualify controlled stopping, regenerative limits, dissipative
  braking and parking-brake entry with the relevant source and load.
- [ ] Verify F405 hardware watchdog reset leaves the bridge safe. Arm and
  validate the corresponding watchdog when the G474 motor path is enabled.
- [ ] Define the riding policy after confirmed Hall loss at low speed.
  See the [fault design](notes/fault-overhaul.md) and the
  [observed fallback behavior](notes/hall-observer-tuning.md).
- [ ] Validate remote fault indication, link loss, explicit re-arm and
  parking-brake control before relying on the remote in use.
- [ ] Define multi-controller ownership, loss of a sibling, coordinated
  braking and re-arm when dual-motor hardware is available. See
  [multi-motor safety](safety.md#multi-motor-vehicles--symmetric-failsafe).

## Firmware and hardware

- [ ] Bring up the G474 + IHM08M1 motor stage with the measured analog
  scaling, hardware break, current-offset calibration and watchdog.
  Follow the [board notes](hw/nucleo-g474re-ihm08m1.md).
- [ ] Revisit platform ISR duplication when enabling that motor stage.
  Compare CORDIC and ADC timing on target before changing shared hot paths.
- [ ] Recheck fast capture losses under load using the [MK5 bench baseline](notes/foc-loop-research.md#native-rate-recording-on-mk5).
  Measure the producer queue and command delivery separately. Keep
  pre-trigger RAM capture as a separate feature if needed for faults.
- [ ] Audit F405/BLE frame size compatibility before enabling fast captures
  through the radio link.
- [ ] Reproduce old board timing issues before applying any fixes described
  in the [backlog snapshot](archive/backlog-through-2026-11-13.md).

## Host and simulation

- [ ] Expand typed GUI configuration coverage for active commissioning
  and tuning groups. Keep wire fields and validation shared with the CLI.
- [ ] Add explicit UUID selection on multi-controller connections and
  fault-injection tests for reconnect/admission and command delivery.
- [ ] Revisit bridge/remote pairing and radio-link qualification. See
  [remote design](notes/remote-design.md).
- [ ] Route virtual commissioning through the live virtual motor when
  capture alignment needs testing end to end. Track plant-model gaps in
  [virtual motor fidelity](notes/virtual-motor-fidelity.md).

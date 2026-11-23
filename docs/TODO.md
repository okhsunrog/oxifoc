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
- [ ] Smooth velocity control below about 50 electrical rad/s on Hall
  feedback. With a regeneration allowance the unloaded bench motor holds
  the mean speed at +40 but still stops and surges about five times a
  second; the pauses fall in four of the six Hall sectors. Softer or
  stiffer PI gains and open-loop commutation do not remove them, and the
  HFI source gives no torque in velocity control. Try position-dependent
  torque feedforward in `sim_trace` first, then HFI.
- [ ] Reproduce the historical sensorless torque oscillation with the
  current firmware before retuning the PLL, voltage model or λ adaptation.
  See the [older sensorless bench evidence](archive/backlog-through-2026-11-13.md#sensorless-startup-see-notesstartup-and-samplingmd).
- [ ] Qualify loaded sensorless cold start, flying restart in either
  direction, and loss of angle trust at low speed. Keep HFI standstill
  control and crossover as separate hardware tests.
- [ ] Compare the observer's voltage and phase lag at high modulation.
  Check the actuation delay and phase-voltage filters before changing PLL
  gains. See [phase sensing](notes/phase-sense.md).
- [ ] Widen the actuation advance bound before riding above 5000
  electrical rad/s at 15 kHz. MK5 turns the output vector by 1.5 PWM
  periods of rotation and `MAX_ACTUATION_ADVANCE` stops it at 0.5 rad:
  5000 rad/s at 15 kHz, 6670 at 20 kHz. The bench motor (3.98 mWb, 7 pole
  pairs) reaches about 6100 rad/s unloaded on 42 V and 7300 on 50 V, where
  15 kHz needs 0.61 and 0.73 rad; the shortfall comes off the current
  loop's phase margin. These are calculated, not measured: 24 V stops at
  about 3500 rad/s. At 0.8 rad the small-angle rotation is still within
  about 1 % in amplitude. Check in the virtual motor that a velocity spike
  from the estimator still cannot rail the phases, the reason for the bound.
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
- [ ] Bench-test a fast telemetry session through the bridge's BLE link.
  The host asks for 478-byte blocks (15 samples) there, so each block fits
  one 505-byte notification; nothing on that path fragments a frame.
- [ ] Keep the bridge alive under a BLE backlog
  ([measurements](notes/ble-link-reliability.md#bridge-reset-and-hang-under-a-ble-backlog)).
  esp-radio panics when the BLE controller's mbuf pool runs out: bound the
  host's ACL queue below the pool (`acl_buf_count`) and report it upstream.
  A capture over BLE alone hangs the bridge without a reset: find where,
  and run a watchdog so a hang ends in a reset instead of a power cycle.
- [ ] Find why a setpoint held over BLE beside Wi-Fi still trips the
  deadman ([measurements](notes/ble-link-reliability.md#a-host-as-central-with-wi-fi-on-the-same-bridge)).
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

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

- [ ] Report why a drive command does nothing. After a `DriverFault` at
  power-on the boot current-sensor calibration stays cancelled: every
  command is answered `Applied`, the state stays `Stopped`, no fault is
  raised and only the controller's log says `NotCalibrated`. Repeat the
  calibration once the fault is cleared, and tell the host the reason.
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
- [ ] Fetch a recording from the XIAO bridge over BLE.
- [ ] Time the relay with the motor driven now that ergot copies COBS
  frames in runs: with the motor stopped a 391-byte block is sent in
  0.26 ms instead of 0.47, and what is left of the send is not encoding
  ([measurements](hw/xiao-esp32c5.md#wired-to-the-mk5-2026-11-24)).
- [ ] Reach a bridge behind the CAN bus without the root. It has no nets
  until the root has leased them, so a host on its USB port reaches
  nothing while the root is away or was never there. Accepted for the
  first stage; think again about how the roles are designed
  ([what is there](../oxifoc-bridge/README.md#can-fd-between-bridges)).
- [ ] Finish the CAN link of the bridges:
  - notice a restarted or absent root at once, not at the next renewal;
  - keep a restarted bridge's nets instead of leasing new ones while the
    old run out (the root has 16);
  - report the bus in `req/bridge_diag`, and leave bus-off when idle, not
    only at the next transmission;
  - find the two transmissions the root counts as failed in some updates
    of the second bridge over the bus, two of six so far;
  - read a segment's net when it is used: a bridge behind the bus reads 0
    at start, which the relay's flow control then compares with;
  - list the bridges and controllers a host can reach, and name a
    controller as `--bridge` names a bridge;
  - set the role in the bridge's own settings instead of at build time.
- [ ] Settle what the review of ergot's CAN transport (#227) left open: a
  bus on which nothing is acknowledged does not show in the interface's
  state, the sink and the worker are each told the frame size and nothing
  compares the two, and a sender keeps 512 bytes of message counters, one
  for every destination a bus can have.
- [ ] Move the queues of the host links into the XIAO's PSRAM and give the
  radio heap their room: the Wi-Fi queue, the USB queue and the TCP
  socket's buffer are 80 KiB of internal RAM, and esp-radio allocates only
  there. A 20 kHz capture beside a BLE central left 7 352 bytes of the
  94 208 free at the lowest. PSRAM shares the 32 KiB cache with code from
  flash, and is out of reach while flash is written: measure both before
  keeping it.
- [ ] Fetch a recording faster over Wi-Fi: 8 MB take 62 s, one chunk a
  round trip. Several reads in flight, or a stream with its own
  acknowledgement, would use the link.
- [ ] Carry a fast telemetry session over the bridge's BLE link. Captures
  at 1 to 5 kHz stalled before the relay: the link's 1 KiB queue refuses
  most of the blocks one SPI slot delivers. The relay sends them smaller
  but does not wait for room on a BLE link. Size the queue for a slot, or
  hold the slot as the USB and Wi-Fi paths do.
- [ ] Compare the bridge's task priorities again with its flash at 80 MHz.
  The relay and the BLE host were raised to the slot task's priority at
  40 MHz, where the bridge had no time to spare
  ([measurements](notes/ble-link-reliability.md#the-relay-and-the-flash-speed)).
- [ ] Keep a held SPI slot from delaying the other links. While the slot
  task waits for room on the link that takes a capture, up to 50 ms a
  slot, the replies in that slot wait too, and the controller's own queue
  fills behind it. This is the suspected cause of late status replies to a
  BLE central during a 10 kHz capture over USB with the flash at 40 MHz:
  16 gaps over 150 ms in 78 s, up to 615 ms. The 20 kHz captures at 80 MHz
  had none.
- [ ] Carry CMSIS-DAP over Ergot, with a local probe port in the host
  library, so the wireless probe needs no second connection to the bridge.
  Keep its packets exactly-once and its pacing while the SPI link is busy.
- [ ] Follow [esp-hal #6895](https://github.com/esp-rs/esp-hal/issues/6895):
  esp-radio panics when the buffer for a BLE packet cannot be allocated,
  which an exhausted radio heap causes
  ([measurements](notes/ble-link-reliability.md#bridge-reset-and-radio-stall-under-ble-load)).
- [ ] Measure the remote, not a host, holding a setpoint beside Wi-Fi, and
  find what stopped a host's holds with `CommTimeout`: four stops in about
  750 s of driving before the radio heap grew, none in five holds since
  ([measurements](notes/ble-link-reliability.md#bridge-reset-and-radio-stall-under-ble-load)).
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

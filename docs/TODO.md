# Project TODO / backlog

The single backlog of open work. Rule: **done items get deleted** (history
lives in git log and [archive/](archive/)), decisions with rationale go to
[decisions.md](decisions.md), analysis/ideas to [notes/](notes/), the
failsafe design to [safety.md](safety.md). Documentation map: [README.md](README.md).

## Safety

- [ ] **Fault overhaul — remainder** (phases 1–5 landed, see
  [notes/fault-overhaul.md](notes/fault-overhaul.md)): phase 6 =
  sensorless promotion on confirmed hall death (after the bench validates
  HFI); remote-side FaultTopic UX (vibration by severity, display —
  blocked on remote maturity); bench-tune the derating ramp numbers
  (battery cutoff / regen-OV / speed ceiling / motor NTC — defaults
  enable only FET 85→100 °C).
- [ ] **Bench-tune regen-brake**: `brake_current_a`, `standstill_rad_s`,
  low-speed coast floor; confirm no OV trip on the bus under regen;
  `BRAKE_ENTRY_MAX_E_RAD_S` (parking-brake entry gate) + windings
  short-circuit current at that speed within FET ratings.
- [ ] **Parking brake follow-ups**: GUI button + remote mapping.
- [ ] **Dissipative braking near OV** (downhill on a full battery, see
  safety.md): when the OV derate cuts the regen brake — dissipate the
  energy in the windings (active short / d-current) with thermal control.
  Needs the bench.
- [ ] **Position hold** (after position control): capture the target on
  engage, cascade position P → `VelocityLoop` → current. `Brake` stays
  the default.
- [ ] G474: arm the IWDG once the motor modules come alive (FOC ISR).
- [ ] Bench: IWDG reset → PWM safe on hardware (provoke a hang).
- [ ] Boot: read reset-reason + detect a spinning motor +
  flying-restart synchronization.
- [ ] Configurable post-watchdog policy (coast / regen / hold).
- [ ] `Idempotent` marker trait + `call`/`call_once` helpers in host-lib.

## Velocity / position

Tuning constraint (learned in sim): hall updates the velocity estimate
only on edges (6 per electrical rev) — aggressive kp/ki produce a
±100 rad/s limit cycle. Defaults are deliberately soft (kp 0.01, ki 0.2,
accel 500 erad/s²).

- [ ] Hall velocity lag bounds the loop bandwidth — consider a less laggy
  source (observer velocity when available) before chasing hot gains.
- [ ] Bench: tune kp/ki/accel for the Flipsky + board mass; behavior
  through the hall→observer crossover.
- [ ] PositionControl: position P → `omega_target` into the same loop
  (cascade); needs an unwrapped position source first.

## Algorithms (ladder, cheap → expensive)

- [ ] Position loop (see above).
- [ ] **Field weakening V2** (MESC: exponential d-current from the voltage
  vector hitting the circle — no motor parameters needed).
- [ ] MTPA.
- [ ] A real overmodulation strategy (today `modulation_limit` up to 1.2
  just clamps duty above the linear SVPWM zone).
- [ ] Pole-pair auto-detection; encoder offset calibration.
- [ ] **Saliency monitor for HFI** (the sim can now trigger the failure:
  `lq_sat_k` + the `closed_loop_hfi_saliency_collapse_loses_tracking_silently`
  test). Saliency collapse/inversion under load is INVISIBLE to
  confidence (eps≈0 reads as a perfect lock) — the angle silently walks
  away under a healthy carrier. IMPORTANT (from the Opus 4.8 dialogue,
  2026-06-12): passively monitoring the existing d channel is degenerate
  IN PRINCIPLE — with the PLL locked (e≈0) the response
  `cos²e/Ld_eff + sin²e/Lq_eff = 1/Ld_eff` does not depend on Lq at all;
  the 2θ component (whose amplitude IS the saliency) is observable only
  with ANGULAR excitation. Implementation ladder:
  1) **hall consistency** (our board always has halls) — "HFI angle vs
  hall sector diverged by more than a sector" catches the collapse within
  one sector transit, nearly free; 2) **interleaved probe**: every N ms a
  few carrier periods on the estimated q axis (or ±45°),
  `saliency_est ∝ amp_d − amp_q`, below the floor → untrusted →
  crossover; bonus — sign-INVERSION detection (positive PLL feedback,
  the worst case); 3) HFI45 (MESC) — structural, but replaces the whole
  tracking scheme. Pure sensorless needs (2); until a monitor exists,
  keep HFI motors with margin against Lq saturation.
- [ ] Detection HFI amplitude: floor from the ADC resolution —
  gimbal-class (ripple ~30 mA against a 15 mA LSB) gives Lq −14%, λ +8%
  on the non-ideal plant; the adaptation currently targets a fraction of
  the hold current without looking at the LSB.
- [ ] **Fix the voltage-pulse inductance method** (now g431's *only* L
  detector — `exp/g431-flash-slim` gated HFI inductance behind `hfi-detect`,
  off on the drone board). Two defects, found via `cargo run -p oxifoc-core
  --example detection_report --features virtual-motor,std` (voltage-pulse
  column):
  1) **Systematic +15-19% L overestimate** on *every* non-salient motor
  (even the 5010 drone: +15.9%), near-identical from 15 µH→3 mH and
  R 0.02→8 Ω. The accumulator (`detection::voltage_pulse::VoltagePulseMeasurement`)
  passes <1% on *analytic* `di`, so the math is fine — the `di` measured
  through `sweep::measure_inductance_pulse` comes out ~16% short. Look at
  the pulse-application vs `i_before`/`i_after` sampling window (and the
  actuation-delay self-alignment) in `measure_inductance_pulse`
  (sweep.rs:~738) against the VirtualMotor harness response.
  2) **Fails on the non-ideal plant** (dead-time + 12-bit ADC noise +
  1-cycle delay) for nearly every motor — SNR/window robustness.
  When fixed: untie the cfg gate — `run_full_detection_high_r_low_vbus` +
  `run_full_detection_nonideal_plant_with_delay` are `#[cfg(feature =
  "hfi-detect")]` and `E2E_L_TOL` is 0.30 for the voltage-pulse path
  (detection/mod.rs); tighten both. HFI by contrast: ~0-1% ideal, 1-6%
  non-ideal (all pass).
- [ ] HallPll: a PLL variant of the hall estimator built on the
  `BackEmfObserver` structure (the boundary anchor is already done) —
  prototype on VirtualMotor. [notes/hall-improvements.md §4]
- [ ] Hall: exact anchor on a skipped edge (low-prio). `crossed_boundary`
  does not check adjacency: the midpoint of non-adjacent centroids = the
  centroid of the skipped sector, −30° off the real boundary (the old
  code lied by +30° — not a regression; the centroid fallback is NOT
  better: the same error mirrored plus worse velocity). The exact fix:
  midpoint(centroid of the sector preceding IN SEQUENCE for the current
  direction, centroid of the new one) — exact both for the base and for
  the traversed velocity on single skips. Mitigations: hardware capture
  (OVERCAPTURES==0), error_count, drift correction self-heals in ~5 ms.
  Naturally subsumed by HallPll.

## Sensorless startup (see notes/startup-and-sampling.md)

- [ ] **Align → ramp → handoff** state machine for a cold start without
  HFI (replacing the fixed-52-rad/s nudge in `try_observer_fallback`).
- [ ] **Flying restart** (kick-push case): a measure-only observer pass /
  HFI probe before torque out of Stopped/Coast; seed and go straight to
  closed loop.
- [ ] Current-scheduled ramp ceiling (VESC `openloop_rpm_max = map(I)`).
- [ ] Host tests on VirtualMotor: cold start from an arbitrary angle
  without a reverse jerk; freewheel catch.

## Sensorless tracking / BEMF (bench-blocked)

- [ ] Bring up the B-G431B-ESC1 phase dividers (BEMF sense) as ADC channels.
- [ ] MESC-style TRACKING: gates off → measured v_αβ into the observer →
  flying start from a converged observer. Hall-based already works; this
  is the sensorless case. Also unlocks the spin-down flux method on
  hardware (`supports_coast_telemetry`).

## Firmware / core

- [ ] The virtual device only simulates CurrentControl/Stopped;
  OpenLoop/DirectVoltage/SixStep/Brake are accepted and ignored; no
  fault injection (the host fault path is not covered e2e); config does
  not reach the VirtualMotor physics. Also: detection runs in a PRIVATE
  sim (`with_sim` on a blocking thread, instant timers) invisible to
  fast telemetry — `detect --record` captures the idle main sim on
  virtual (meaningful on hardware only). The principled fix is to route
  the virtual detection backend through the LIVE sim via the protocol
  once OpenLoop/DirectVoltage are simulated — then detection traces work
  in sim exactly like on the bench.
- [ ] Remaining ISR deduplication: ADC snapshot assembly is still a
  per-platform copy (voltage/temp fault checks moved into core
  `run_protection` 2026-06-13). Move the rest of the ISR glue into core
  BEFORE reviving g474 (otherwise it will reproduce already-fixed F405
  bugs).
- [ ] g474 motor modules are commented out until the IHM08M1 is
  connected; `control/foc.rs` is synced by hand with no compile check.
- [ ] **g474 + IHM08M1: checklist before powering a motor**
  (see [hw/nucleo-g474re-ihm08m1.md](hw/nucleo-g474re-ihm08m1.md)):
  - config.rs BOARD holds IHM07M1 constants: for IHM08M1 — 0.010 Ω
    shunts, TSV994, gain ≈5.18, offset ≈1.71 V, ≈51.8 mV/A, FS ≈ ±31 A;
    JP2 changes the feedback — verify the actual gain on the bench.
  - CURRENT REF (PB4 PWM) is optional: the BKIN comparators are
    autonomous (fixed Vref ≈30 A); PB4 is the threshold of a separate
    U23 → CPOUT → TIM1_ETR.
  - BKIN PA6 (AF6) active-LOW + BKF; optional PA11 = BKIN2; BKIN flag in
    the FOC ISR + MOE re-arm (port from g431).
  - Injected ADC per the mapping (ADC1 PA0/PA1/PC2, ADC2 PC1/PC0,
    TRGO2); delete the commented-out internal-OPAMP plan in
    peripherals.rs.
  - Re-enable control/motor/calibration; GPIO_BEMF (PC9) off; IWDG;
    PB15/PB14 strictly Hi-Z.
  - Shield jumpers (factory default is 1-shunt/6-step!): J5/J6 → 3-Sh,
    JP1+JP2 closed, remove C3/C5/C7, JP3 closed, J9 open, Nucleo JP5 → E5V.

## VirtualMotor fidelity (analysis: notes/virtual-motor-fidelity.md)

Every effect sits behind an optional parameter with an ideal default
(decisions.md); every upgrade ships with a test that fails without the
matching compensation. Done 2026-06-12: sub-stepping (`substeps`),
dead-time (`dead_time_v`), quantization+noise (`adc_lsb_a`/`adc_noise_a`),
Lq saturation (`lq_sat_k`), duty-driven harness with ARR 4250, one-cycle
delay (`actuation_delay_steps` — immediately exposed the phase-advance
measurement-frame bug and the detection pipeline-skew, see decisions.md
and the item above). Remaining:

- [ ] **Vbus sag** (`vbus0 − i_bus·R_esr`) — UV-dip / regen-OV scenarios,
  the basis for a dynamic Vmax.
- [ ] **Coulomb friction + ω² load** — stiction for standstill
  HFI/open-loop starts; physical realism for the eskate/drone catalog rows.
- [ ] Later: coupled dynamometer (two motors on one shaft), hall glitches
  (0/7, bounce), cogging — only together with anticogging.
- [ ] Non-sinusoidal back-EMF (5th/7th harmonics of λ) — observer angle
  bias on a real machine.

## Size / performance

Current numbers and rules — [flash-size.md](flash-size.md); benchmarks —
[perf-bench-2026-06-11.md](perf-bench-2026-06-11.md).

- [ ] f405/g474 build with `opt-level = 3`, g431 with `"z"` — deliberate,
  but unmeasured: what `"z"` would cost f405/g474 in ISR time.
- [ ] Live ISR load counter (DWT CYCCNT min/max/avg → SlowTelemetry once
  a second): confirm the shipped-"z" build in situ; also settles the F405
  double-trigger suspicion via the measured ISR rate.

## Documentation

- [ ] architecture.md (~1400 lines) — doc-rot candidate: inlined
  signature snippets are synced by hand (precedent: icd.rs). Move the
  invariants into doc-comments/doctests (the compiler guards them), keep
  topology, diagrams and rationale. [Opus 4.8 review]

## Host

The CLI (2026-06-12) covers the whole ICD: every ControlMode, all 10
config groups read/write (`config get/set/dump/reset`), fault
query/clear, info/status, the full detection chain with `--apply`,
`--json` everywhere, `record` → parquet (CIC metadata, exit 1 on seq
gaps).

`maneuver` done 2026-06-12 ([maneuvers/](../maneuvers/)): timeline +
capture, seq-anchored event log in the metadata, terminal command on
every exit path, limits gate. Verified on virtual: events within ±0.1 ms
of plan, 0 gaps, command→response latency is now measurable (~10 ms on
the sim = batch tick).

- [ ] **Device-RAM burst capture** (the VESC `sample` pattern): raw
  20 kHz into a ring buffer (~100–200 ms on g431) downloaded afterwards,
  pre-trigger around faults (a "black box"). The 5 kHz CIC stream is for
  long logs, burst for full-bandwidth snapshots.
- [ ] `protocol_version` in `HardwareInfo` + `env!("CARGO_PKG_VERSION")`
  instead of the hardcoded "oxifoc-0.1.0" — mandatory before any
  release/distribution (postcard schema has no self-description: a
  mismatch = silent garbage).
- [ ] Reconnect state machine has no test coverage; slint-wgpu-plot: the
  ring index arithmetic (`renderer.rs:262`) under a large zoom-out +
  scroll-back may compute the Y auto-range over a different window than
  the shader draws.
- [ ] bridge/remote: pairing via a hardcoded MAC; stub tests. Remote
  design — [notes/remote-design.md](notes/remote-design.md).

## Bench (waiting for hardware)

- [ ] **Hall timer-capture validation** (2026-06-10 migration): turn by
  hand — sequence 1→3→2→6→4→5, velocity without the 2× skew,
  `OVERCAPTURES == 0`, `read_hall_state_raw` reads the pins in AF mode.
- [ ] **Hall boundary-anchor fix (9f936bb)**: d-current at constant speed
  must be centered (before the fix a ~30° lead gave a cosine torque loss
  + d offset).
- [ ] Re-run detection — the stored Flipsky parameters are off by 1.5×
  after the SVPWM normalization fix; λ measured by the GUI step before
  2026-06-12 is garbage (q-axis method) — re-measure. Also verify the L
  step under real dead-time: before 2026-06-12 vd_hold was computed as
  R·I, and on the Flipsky (0.3 V hold against the g431's 0.38 V
  distortion) it would have failed with MotorNotResponding — the fix
  (settled hold + comp in apply_dq) is sim-proven, not hardware-proven.
- [ ] Detection PI (0.01/10) is 10× hotter than VESC (0.001/1.0): verify
  convergence on hardware.
- [ ] **F405 ADC double-trigger**: TIM1_CH4 compare fires twice per
  center-aligned period; g431 `COMPARE_OC4`→TRGO2 is not fundamentally
  immune. The robust fix is one deterministic trigger per period (update
  event or TIM→DMA→ADC). Check the JEOC rate under load.
- [ ] **Detection lag probe on hardware**: the pipeline-skew fix
  (2026-06-12, decisions.md) measures the command→apply depth in place —
  on the bench, log the probed lag (sim predicts 2 cycles; async command
  delivery may add more), verify the |Z| cross-check stays quiet, and
  cross-check the detected L against the Flipsky LCR numbers (mind the
  small-signal-vs-incremental saturation difference).
- [ ] OCP with the BKF break filter under real load (g431).
- [ ] Dead-time compensation at low speed.
- [ ] Hall dropout at speed and the sensorless crossover.
- [ ] HFI on the real B-G431B-ESC1: the carrier amplitude is now solved
  from the measured L (Flipsky: ~25 µH equivalent, saliency Lq/Ld ≈ 1.5
  per LCR and VESC detection — HFI is physically meaningful); verify the
  2 A ripple target on hardware, tune the polarity-probe
  amplitude/duration in place. Carrier pre-heat and the cold-demod trust
  gate (jam) — verify the downward crossover on the bench.
- [ ] Source switching end-to-end via `oxifoc-host-cli source ...`.
- [ ] Current quality at ~90% modulation + HFI demod SNR on a single V0
  sample (V0_V7 — only if the bench shows a problem)
  [notes/startup-and-sampling.md, scope decision].

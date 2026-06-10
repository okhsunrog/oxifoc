# Project TODO / backlog

Working list of known gaps and planned work. A full external review
with verified bugs, gap analysis and a borrow-list from reference
projects (VESC/MESC/moteus/ODrive/MCSDK) lives in
[review-2026-06-10.md](review-2026-06-10.md); items below reference it. Safety-specific items live in
[safety.md](safety.md#open-questions--todo); hardware bring-up notes in the
board docs.

## Deferred until needed

- [ ] **`protocol_version` in `HardwareInfo`** + `env!("CARGO_PKG_VERSION")`
  instead of the hardcoded `"oxifoc-0.1.0"` strings. Not relevant while GUI
  and firmware are always built from the same checkout (`cargo run`), but
  required before any release/distribution: the wire schema (postcard) has
  no self-description, so a version mismatch shows up as silent garbage,
  not an error. The schema has already changed several times
  (`SlowTelemetry.phase_source`, `ConfigResponse::Busy`,
  `PhaseSourceEndpoint`).

## Firmware / core

- [ ] Virtual device only simulates `CurrentControl`/`Stopped`; OpenLoop,
  DirectVoltage and SixStep are accepted and ignored (limits/gains
  commands too).
- [ ] Remaining ISR dedup: ADC snapshot assembly + voltage/temp fault
  checks are still per-platform copies (small).
- [ ] g474 motor modules are commented out until the IHM08M1 shield is
  connected; `control/foc.rs` is kept in sync by hand but not
  compile-checked.

## VirtualMotor model fidelity (from moteus sim comparison, 2026-06-10)

Keep every new effect behind an optional param defaulting to ideal (the
`sat_k` pattern) so existing tests stay pinned. High-priority items
de-risk the sensorless bench; run `detection_report` before/after each.

- [ ] **Sub-stepping** (~10 internal Euler steps per `step()`). Not for
  stability — to break the shared-discretization circularity: the sim's
  per-step `di = (v − R·i)·dt/L` is exactly the model the estimators
  assume, so detection errors near 0.0% are partly self-confirmation.
  Sub-stepping approximates the continuous plant and makes the
  estimators' own discretization error visible. Free accuracy audit
  before the bench.
- [ ] **Dead-time voltage distortion**: `v_err = −sign(i_phase)·v_dt`,
  `v_dt ≈ t_dead·f_pwm·vbus`. Three algorithms claim dead-time
  robustness and none of it is currently exercised in sim: 2-point R
  measurement (exists to cancel this bias), HFI inductance via
  `apply_dq` (skips dead-time comp), flux observer (integrates
  commanded, not actual volts). Add to a couple of catalog rows and
  watch the error columns.
- [ ] **Current quantization + noise**: 12-bit ADC over the shunt range
  plus deterministic Gaussian noise (seeded xorshift, no_std, no `rand`
  dep). Makes HFI demod SNR honest (the carrier-amplitude floor in
  `observer.rs` was tuned on noiseless currents) and shows whether
  `check_current_faults` needs a persistence filter.
- [ ] **One-cycle PWM delay in the closed-loop test harness** (not the
  model): apply the previous step's v_αβ, like moteus's `prior_pwm`.
  Observer/crossover tests then see the phase lag real hardware always
  has.
- [ ] **Q-axis saturation** `Lq_eff = Lq/(1 + sat_kq·|iq|)` — the
  classic HFI failure mode: saliency ratio collapses under load. Lets a
  test pin "HFI confidence drops with iq, manager falls back to
  observer", which the PhaseManager logic supports but the sim cannot
  currently trigger.
- [ ] **Vbus sag**: `vbus = vbus0 − i_bus·R_esr`. Realistic UV-dip /
  regen-OV scenarios for the fault checkers (currently tested with
  hand-fed constants); groundwork for MESC-style dynamic Vmax if
  borrowed.
- [ ] **Coulomb friction + ω² load**: `T_load += T_c·sign(ω) + k_d·ω²`.
  Coulomb matters for standstill HFI / open-loop starts (stiction is
  where startup stumbles on real hardware); ω² makes the drone/eskate
  catalog rows physically meaningful.
- [ ] Later, with the velocity loop: **coupled dynamometer mode** (two
  VirtualMotors on a shared shaft, mirroring moteus's
  `simulation_dynamometer_test.cc`) so speed-control tests run against
  an active load and can later be mirrored on a physical rig.
- [ ] Later: **hall glitches** (rare 0/7 states, edge bounce) to cover
  invalid-state recovery in closed loop; **cogging** only if/when
  anticogging calibration lands.

Explicitly not planned: temperature model, iron losses, f64/unwrapped
position (only needed once a position loop exists; keep the no_std
model f32).

## Size / performance

- [ ] **g431 flash headroom is ~3.3 KB** (123 564 / 126 976 bytes) after
  enabling `build-std = ["core"]` (libcore rebuilt with opt-level="z";
  the shipped one is opt-level=3). When it runs out again: trim
  `.rodata` (~14 KB, mostly postcard schema tables), consider
  `panic_immediate_abort`.
- [ ] VSQRT (`vsqrt.f32`) instead of `libm::sqrtf` on Cortex-M4F hot paths.
- [ ] f405/g474 build with `opt-level = 3`, g431 with `"z"` (flash
  pressure). Intentional, but unmeasured: check what `"z"` would cost
  f405/g474 in ISR time, or whether it matters at all at 20 kHz.

## From external review (verified pending / host side)

- [ ] **F405 ADC trigger suspicion (bench)**: ADC triggers from TIM1_CH4
  compare, which fires twice per period in center-aligned mode (G431
  correctly uses TRGO2/COMPARE_OC4, one edge). May work by accident
  (second trigger lands in a still-running injected sequence). Verify
  ISR rate on hardware or move F405 to TRGO.
- [ ] Host command queue is strictly serial: a `Stop` queued behind a
  running `Detect` waits for it (up to the ~70 s retry budget). The
  device-side link failsafe covers the safety angle, but the UX is
  wrong — route Stop around the queue or cancel in-flight detect.

2026-06-10: fixed the other review host items — CLI `--baud` config
override, framed-transport (UDP/USB/BLE) handshake check + reconnect
loop, GUI `unwrap_or(0.0)` field parsing, dead `oxifoc-virtual
--pole-pairs` — and added the GUI phase-source switcher with
`SlowTelemetry.phase_source` read-back. See git log.

## Hardware bench (waiting for the rig)

- [ ] **Validate timer-capture hall acquisition** (2026-06-10 migration:
  TIM4/TIM3 XOR + input capture replaced 200 kHz TIM6 polling; hall ticks
  are now 1 MHz hardware timestamps). Hand-spin the motor and check:
  hall states cycle 1→3→2→6→4→5, velocity magnitude is sane (a wrong
  TIM clock assumption would skew it 2×), `OVERCAPTURES` stays 0, and
  calibration (`read_hall_state_raw`) still reads pins in AF mode.
- [ ] Re-run motor detection — stored Flipsky params are 1.5× off after
  the SVPWM normalization fix.
- [ ] OCP with the BKF break filter under real load (g431).
- [ ] Dead-time compensation at low speed.
- [ ] Hall-dropout-at-speed and sensorless crossover behavior.
- [ ] HFI on the real B-G431B-ESC1: carrier defaults (1 kHz, 12.5% vbus)
  and polarity-probe pulse amplitude/length (`HFI_POLARITY_*` constants)
  may need tuning per motor.
- [ ] Source switching end-to-end via `oxifoc-host-cli source ...`.

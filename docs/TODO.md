# Project TODO / backlog

Single backlog for all known gaps and planned work. The **analysis** behind
these items — verified bugs, gap analysis, and a borrow-list from reference
projects (VESC/MESC/moteus/ODrive/MCSDK) — lives in [review.md](review.md);
the failsafe **design** in [safety.md](safety.md); hardware bring-up facts in
the board docs. Those docs no longer keep their own TODO lists — everything
actionable is collected here.

## Safety

Failsafe-layer *design* and rationale live in [safety.md](safety.md); the
remaining work:

- [ ] **Layer 2: ISR command-staleness deadman** + configurable failsafe mode.
  The only link-loss gate today is async-executor-dependent and 3–5 s
  ([review.md](review.md) §1, §3). Stamp `last_cmd_tick` in the ISR
  `CMD_CHANNEL` drain; force a self-contained failsafe mode past a small
  threshold.
- [ ] Shorten control-link liveness timeout (interim, until Layer 2).
- [ ] Once Layer 2 exists, fold/remove the Layer 1 `link_active` gate.
- [ ] Integrating current/voltage fault detector (replace the single-sample
  trip — nuisance-trips on regen/EMI); signed open-loop override
  (direction = sign of last velocity). [review.md §3]
- [ ] G474: arm the IWDG when the motor modules (FOC ISR) wake up.
- [ ] Bench: verify IWDG reset → PWM safe on real hardware (induce a hang).
- [ ] Boot: reset-reason read + spinning-motor detection + flying-restart sync.
- [ ] Configurable post-watchdog policy (controlled coast / regen / hold).
- [ ] `Idempotent` marker trait + `call`/`call_once` helpers in host-lib.

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
- [ ] **g474 + IHM08M1: remaining work before enabling the motor stack**
  (see [nucleo-g474re-ihm08m1.md](nucleo-g474re-ihm08m1.md)). Done
  2026-06-11: hall on TIM2/PA15+PB3+PB10 (32-bit captures via generic
  `CaptureTimebase<u32>`), time driver tim2 → tim5, resources.rs pins +
  CN comments, `mod sensors` compiled to prevent rot. Still open when
  the shield arrives (hardware audit 2026-06-11):
  - **config.rs BOARD has IHM07M1 constants**: shunt 0.33 Ω + internal
    OPAMP ×16 are wrong. IHM08M1 (from Fig. 5): 0.010 Ω 1W shunts,
    TSV994 difference amps per phase — Vshunt→680Ω→(+) with 6.8k bias
    to 3V3 (via JP1) , Kelvin GND→1k→(−), feedback 4.7k ⇒ gain ≈5.18,
    offset ≈1.71 V (≈2122 counts), ≈51.8 mV/A, FS ≈ ±31 A. JP2 alters
    the feedback network — verify effective FOC gain at bench via
    zero-offset (calibrate()) + a known current. VBUS divider
    169k/9.31k = 19.15 (config's 19.12 accidentally close).
  - **CURRENT REF (PB4 PWM) is optional, not mandatory** (corrected
    after reading Fig. 5 in full): the BKIN comparators (U24-26,
    LMV331) compare raw shunt voltage against a FIXED divider Vref
    (R179 33k / R180 3.3k → ≈0.3 V ≈ 30 A) — hardware OCP is armed
    autonomously, the bridge starts without firmware help. PB4's
    RC-filtered PWM (R21 33k + C16) is the threshold of the SEPARATE
    U23 comparator on the amplified phase-B signal → CPOUT → PA12 =
    TIM1_ETR, for optional cycle-by-cycle limiting.
  - **BKIN PA6 (AF6), active-LOW** ("goes to ground", UM1996 §4.1.2)
    + BKF filter; optional PA11 = BKIN2. Plus BKIN-flag check in the
    FOC ISR + MOE re-arm (port from g431 when control/foc.rs wakes).
  - ADC injected per the mapping doc: ADC1 = PA0/IN1 + PA1/IN2 +
    PC2/IN8, ADC2 = PC1/IN7 + PC0/IN6, TRGO2 trigger; **delete** the
    commented-out internal-OPAMP plan in peripherals.rs (the shield
    conditions signals externally).
  - Re-enable control/motor/calibration (foc.rs already edited to take
    now_ticks from sensors::hall); GPIO_BEMF (PC9) off for FOC; arm
    IWDG; **keep PB15/PB14 Hi-Z**.
  - Shield jumpers (factory default is 1-shunt/6-step!): J5/J6 → 3-Sh,
    JP1+JP2 closed, remove C3/C5/C7, JP3 closed, J9 open, Nucleo
    JP5 → E5V.

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

Hot-path math reworked 2026-06-11 after on-target benchmarks
([perf-bench-2026-06-11.md](perf-bench-2026-06-11.md)): HFI estimator
generic over SinCos (CORDIC on G4, FastSinCos on F405), `vsqrt.f32` +
polynomial atan2 in `fast_math`. HFI went from >150% of the 20 kHz ISR
budget to 13.9% — it was unusable on hardware before this.

- [x] **g431 flash recovery done 2026-06-11**: 126 656 → 118 668
  (headroom 320 B → 8.3 KB), zero accuracy loss. Everything
  flash-size related — rules for new code, measurement workflow
  (`just size`), measured reserves (detection gate −14.7 KB, etc.),
  dead ends — now lives in [flash-size.md](flash-size.md).
- [ ] F405/G474 still use plain `unwrap` and no dep defmt features —
  port the g431 treatment from [flash-size.md](flash-size.md) when
  convenient (no flash pressure there: 30% / 60% used).
- [ ] f405/g474 build with `opt-level = 3`, g431 with `"z"` (flash
  pressure). Intentional, but unmeasured: check what `"z"` would cost
  f405/g474 in ISR time, or whether it matters at all at 20 kHz.
- [ ] Live ISR utilization counter (DWT CYCCNT min/max/avg →
  SlowTelemetry or defmt once a second): the bench numbers are from the
  test profile; the shipped "z" build should be confirmed in situ. Also
  settles the F405 double-trigger suspicion via the measured ISR rate.

## From external review (verified pending)

### Firmware (2026-06-11 re-review; detail in [review.md](review.md) §1–§2)

- [ ] **F405: SPI to DRV8301 inside a critical section masks the FOC ISR**
  (PRIMASK gates all interrupts, incl. the control loop, during the blocking
  SPI read on a gate-driver fault). Move the SPI device out of the CS-mutex.
  [§1 HIGH]
- [ ] **F405: FOC ISR left at default priority** — comms ISRs jitter/preempt
  the control loop. Set NVIC priority 0 like G431. [§1 HIGH]
- [ ] **F405: `OverTemp` not critical + motor temp measured but never
  fault-checked.** Add `OverTemp` to `is_critical`; add a motor-temp
  threshold to `BoardConfig` + a second `check_temperature_fault`. [§1]
- [ ] **Detection: spin-down flux is dead on hardware** —
  `read_coast_telemetry` not overridden on `EmbassyDetectionHardware`, so the
  R-independent path always falls back. Implement it (phase-voltage ADC +
  observer ωe) or stop advertising it. [§1]
- [ ] **Detection: inductance gives `Ld ≤ Lq` by construction** (bin-2
  magnitude only) — use the complex bin-2 to recover saliency sign / catch a
  90°-off lock. [§1]
- [ ] **g431: storage region has no `const_assert`** against firmware overlap
  (f405/g474 have one) — self-brick risk on the 128 KB single-bank part.
  Port the assert + `FIRMWARE_END_OFFSET`. [§1]
- [ ] Stale safety comment: g431 `init_overcurrent_protection` says
  "Temporarily disabled" but OCP (BKIN + BKF filter) is enabled — delete it.
  [§3]
- [ ] **F405 ADC trigger (bench)**: ADC triggers from TIM1_CH4 compare, which
  fires twice per center-aligned period. Works only by timing accident (the
  2nd trigger lands inside the still-running injected sequence). Note: G431's
  `COMPARE_OC4`→TRGO2 is **not** immune either (OC4REF asserts on both
  passes); the robust fix is one deterministic trigger/period (update event
  or TIM→DMA→ADC). Verify the JEOC rate under load. [§2]
- [ ] **Detection: inductance HFI pipeline skew (bench)** — `record()` pairs
  current with the previous-iteration injection, but the command→apply→sample
  latency on real hardware may exceed one iteration (tests apply synchronously
  so it's invisible). Verify against a reference inductance. [§2]

### Host

- [ ] **`HostRuntime` has no `Drop`** — leaks a tokio runtime + thread (still
  holding the port) on every GUI reconnect. Add `Drop`/shutdown of the old
  slot. [review.md §3]
- [ ] Host command queue is strictly serial: a `Stop` queued behind a
  running `Detect` waits for it (up to the ~70 s retry budget). The
  device-side link failsafe covers the safety angle, but the UX is
  wrong — route Stop around the queue or cancel in-flight detect.
- [ ] RTT transport: `expect()` in the detached thread silently kills the
  link with no reconnect; control-block scan hardcoded to 32 KB
  (`0x20000000..0x20008000`) breaks on F405/G474 (RAM > 32 KB). [review.md §3]
- [ ] CLI `start`/`stop`/`source` are fire-and-forget (always exit 0);
  `--duty` is actually current (`× 0.1 A`). Route through the ack; rename the
  flag. [review.md §3]
- [ ] GUI: telemetry rate above link bandwidth drops silently (compare
  `actual_fast_hz`); RPM uses preset pole-pairs, not the device's
  (`HardwareInfo` has none); `motor_running` not reset on disconnect.
  [review.md §3]

2026-06-10: fixed the other review host items — CLI `--baud` config
override, framed-transport (UDP/USB/BLE) handshake check + reconnect
loop, GUI `unwrap_or(0.0)` field parsing, dead `oxifoc-virtual
--pole-pairs` — and added the GUI phase-source switcher with
`SlowTelemetry.phase_source` read-back. See git log.

## Sensorless tracking / flying start (bench-blocked)

- [ ] Bring up the B-G431B-ESC1 phase-voltage dividers (BEMF sense) as
  ADC channels.
- [ ] MESC-style TRACKING mode: gates off → feed measured v_αβ to the
  back-EMF observer → flying start = seed commutation from a converged
  observer instead of blind open-loop. Hall-based flying restart already
  works (hall estimator + decoupling feedforward run regardless of motor
  state); this item covers the sensorless case. See safety.md
  *Boot-time recovery*.

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

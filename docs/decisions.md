# Decisions (append-only)

A log of decisions with rationale — so they don't get relitigated and the
"why" doesn't have to be excavated from git archaeology. Format: date,
decision, why, pointer to details. New entries are appended at the end of
their section.

## Safety

- **2026-06-10 — `panic_immediate_abort` is FORBIDDEN.** It would drop
  ~8 KB of panic strings, but it bypasses `#[panic_handler]` — i.e. the
  gate-kill in safety.rs. Motor safety beats flash. [flash-size.md → Dead ends]
- **2026-06-11 — `CordicSinCos` panicking on an uninitialized CORDIC is an
  intentional fail-fast.** No libm fallbacks: silent degradation in the
  ISR is worse than a loud crash at bring-up.
- **2026-06-11 — default failsafe policy `ControlledStop` + terminal
  `ParkBrake`.** Longboard: link loss = smooth regen braking to a stop,
  then windings short (holds on a slope with zero energy). The re-arm
  latch clears only on an explicit Stopped/Coast/Brake. [safety.md]
- **2026-06-11 — hall staleness: DO NOT add a fixed timeout backstop
  (won't-fix).** A standstill rotor legitimately produces no edges; a
  backstop would declare the halls dead at every stop → an open-loop
  override at a traffic light. A dead-while-parked sensor is
  fundamentally indistinguishable from a parked rotor by edges alone; a
  broken cable is caught by the invalid-state check (pull-up → 0b111),
  motion is covered by the speed-adaptive `is_stale_at_speed` path.
- **2026-06-12 — bus current limits: `< 0` = disabled (default), `0` =
  full ban, otherwise amps.** `bus_regen_max_a = 0` is the standard
  lab-PSU-safe mode; ControlledStop then degrades itself to coast via the
  no-progress watchdog, and the windings-short Brake never touches the bus.

## Firmware / platform

- **2026-06-11 — g431 defaults to the baked-config profile.** Runtime
  flash persistence cost −25.5 KB (config server + sequential-storage +
  postcard codecs). Workflow: detect → live tuning → `config dump --rust`
  → rebuild. [flash-size.md]
- **2026-06-12 — the g431 storage profile is REMOVED ENTIRELY.** The
  detection fixes overflowed the 124K layout; instead of dieting the
  backup profile, the board gave up persistence for good: one memory.x at
  128K, config baked only, the config server RAM-backed
  (persist-capable = false). f405/g474 keep storing as before.
  [flash-size.md, commit 7a6c923]
- **2026-06-11 — no libm trig in firmware-reachable code** (5 KB of flash
  + ~6200 cycles due to the `-fp64` softfloat in rem_pio2f). Only the
  SinCos backends (CORDIC/FastSinCos) and fast_math. [flash-size.md → Rules]
- **2026-06-11 — rc_w0 status registers: complement-mask writes only
  (`clear_rc_w0!`), `modify` is forbidden** (LDR+BIC+STR loses a flag set
  between the read and the write). For rc_w1 the pattern is the
  OPPOSITE — the macro is forbidden there. [register-access.md]
- **2026-06-11 — the two-firmware detect/run split is REJECTED** (for
  now): the detect image ⊇ the run image without core gates on run-only
  subsystems — it would stop fitting sooner. The reserve ladder lives in
  flash-size.md.
- **2026-06-12 — detection: the q-axis flux method is demoted to
  diagnostics.** In open loop it is biased by cos(load angle) up to −98%;
  production is back-EMF-vector (magnitude), spin-down only when coast
  telemetry exists. The method ladders (`measure_*_auto`) are shared by
  full and step-by-step detection.
- **2026-06-11 — F405: the blocking 128 KB sector erase is accepted as a
  known residual.** The TOCTOU is closed by the Busy gate (the motor is
  guaranteed stopped for the duration of the write), the IWDG margin is
  the backstop. Not fixing until it hurts. *Correction 2026-08-02: the
  "~0.5 s" figure was the 16 KB-sector number — a 128 KB erase is 1 s
  typical / 2 s max, so the 1 s IWDG had NO margin and reset the chip at
  spec-typical erase times; the timeout is now 3 s.*
- **2026-06-12 — hall calibration convention: the table stores
  CENTROIDS** (what `HallCalibrator`'s sin/cos averaging measures);
  interpolation anchors on the BOUNDARY = the midpoint of adjacent
  centroids (VESC-style). Sector widths are not stored — midpoints of
  measured centroids absorb mounting asymmetry. `VirtualMotor` centers
  sector k on `k·60° + hall_offset` — the same convention.
  [notes/hall-improvements.md, commit 9f936bb]

- **2026-06-12 — HFI: "margin for the bounded, gate for the unbounded".**
  The HFI demod without a carrier measures silence → update() and
  injection only toggle as a pair (`hfi_active()`): never in non-HFI
  sources (~10% of the ISR budget back in the default hall config), off
  above the crossover latch in HFI sources, with a **pre-heat** of one
  hysteresis band above the latch-release threshold (the demod needs a
  few carrier periods to lock; `restart_demod()` on resume — otherwise
  stale filters masquerade as confidence). The margin is deliberately NOT
  sized for mechanically-unbounded deceleration (a jam punches through
  any finite band instantly) — that tail is covered by the trust gate: a
  cold demod = confidence 0 = `angle_trustworthy()` false = iq cut for
  the few milliseconds of re-lock near zero speed. The same philosophy as
  the deadman ("don't guarantee delivery — remove the positive
  confirmation"). Both cases are pinned by closed-loop tests
  (controller-bounded ramp: lock-before-weight; jam: the gate holds until
  re-lock). From the Opus 4.8 dialogue.
- **2026-06-12 — do NOT add a feature flag for runtime HFI** (Opus 4.8
  proposal rejected): the flag only bought flash, which after the g431
  storage removal is sufficient (17+ KB); the ISR cost was already
  removed by the gate above, and the price is cfg noise in the manager +
  one more CI configuration. If flash gets tight again — it's a line in
  the flash-size.md reserve ladder.
- **2026-06-12 — the HFI carrier amplitude is derived from the measured
  L** (`V = I_target·ω_c·L`, ceiling = the old vbus ratio): a raw
  12.5%·vbus on a low-inductance outrunner (25 µH Flipsky) would have
  produced tens of amps of ripple. The same logic as the adaptive
  detection amplitude.

- **2026-06-12 — current limits: the layered "motor rating / operational"
  scheme.** The rating (continuous thermal current `√(P_loss/R/1.5)` —
  VESC's i_max formula; we already computed it during detection and threw
  it away) is a property of the MOTOR, persisted in the MotorParams group
  together with the dissipation class (W). Operational limits
  (CurrentLimits) are a property of the session/installation; on apply
  they clamp: `effective = min(operational, rating, board)`, the OC trip
  ≤ 1.5×rating (VESC `l_abs_current_max`), an unset operational defaults
  to the rating itself. Unlike VESC, the user cannot grant the motor more
  than it tolerates. Bus limits are NOT in the motor config — they are a
  property of the supply (PSU/battery). The HFI ripple target scales from
  the RATING (0.15×, capped at 2 A), not the operational limit: a bench
  iq cap must not strangle the carrier SNR. The MotorParams blob grew —
  an old record on f405/g474 falls back to defaults; rewrite the group
  once.
- **2026-06-13 — HFI is per-board, behind two feature flags; six-step is
  gone.** B-G431B-ESC1 is a drone-motor board: non-salient SPM, so HFI
  (which exploits Ld≠Lq) does nothing there, and it's flash-tight (128K).
  Split HFI into `hfi` (runtime sensorless observer) and `hfi-detect`
  (rotating-injection + FFT inductance measurement, pulls `microfft`) —
  two flags, off on g431, on for g474/f405. Six-step removed entirely
  (unused). **−8.9 KB on g431** (headroom 8.3 → 17.2 KB) for the room to
  add features. **Wire is preserved across boards**: `PhaseSource` keeps
  all `Hfi*` variants (command + status) — g431 rejects an HFI source at
  runtime (`HfiNotConfigured`), it is not a compile-time enum difference;
  removing `ControlMode::SixStep` shifts `Brake`'s discriminant uniformly
  on every board (host rebuilt in lockstep). With `hfi-detect` off,
  inductance falls to the **voltage-pulse** method, now fixed (see the
  next entry) — g431 measures L to a few percent without HFI.
  [flash-size.md history]
- **2026-06-13 — voltage-pulse inductance: discharge-anchored, absolute-
  current accumulator.** It was the g431-only L detector and was biased
  +15-19% on every motor and failed outright on the non-ideal plant. Root
  cause (one bug, two faces): the winding never discharged between pulses
  (the open-loop `vd_hold` restore lasted a few PWM periods, but L/R is
  far longer), so the current *ratcheted up* — (a) the accumulator's
  `pulse − R·di/2` term assumed each pulse started from the steady I_hold,
  over-crediting the inductor as the baseline climbed → the +16%; (b) on a
  delayed pipeline the still-elevated decaying tail tripped the edge
  detector with a spurious negative di → `InsufficientSamples`. Fix:
  (1) the accumulator computes `L = (V_applied − R·i_avg)·dt/di` from the
  *absolute* current, immune to baseline drift; (2) each pulse first
  discharges back to I_hold (polls until settled, L/R-agnostic); (3) the
  application edge is the *argmax* single-period rise, not a fixed
  threshold — immune to the actuation delay and to ADC noise (the real
  step dwarfs it). Result on `detection_report`: non-salient motors within
  ±2% on the realistic plant (5010 drone +0.8%; the ideal column's −2…−6%
  is the sim's sub-step-1 forward-Euler `R·dt/2L` bias, not the
  algorithm). Saliency (IPM Lq) and the high-R gimbal on a low bus remain
  HFI's domain — not g431 targets. `E2E_L_TOL` unified to 0.10 and the two
  HFI-only E2E regressions un-gated to run on the pulse path too.
- **2026-06-13 — voltage-pulse step is current-limited (VESC-style).** A
  fixed `vbus·0.577` step drives `di = V·dt/L`, which is unbounded as L
  shrinks: a real 24 µH drone winding spikes ~14 A in one period off a 12 V
  bus and ~28 A off 24 V — straight into the ±31 A sensor rail / OCP, and
  *worse* on a bigger bus. Mirror VESC's `mcpwm_foc_measure_inductance_current`
  (ramp the amplitude under a current target), but applied to our di/dt pulse,
  not its HFI injection — VESC has no standalone di/dt L method, and it merely
  scales the HFI result by 0.9 to hide the same overestimate we removed at the
  source. `measure_inductance_auto` splits the safe-current budget (lock 0.4·,
  target di 0.5·) and `calibrate_pulse_voltage` ramps the step from small until
  one period's di hits the target, trimming exactly (di ∝ pulse_v). Peak
  (`I_hold + di`) is then bounded for ANY L and **independent of vbus** — the
  24 µH motor measures Ld +0.7…+2% with a ~9 A peak at 12/20/24 V alike (was
  17/26/31 A). The smaller step is more dead-time-sensitive (eskate-class non-
  ideal L drifts to ~+8% from +1%), still inside `E2E_L_TOL`; high-R/high-L
  motors that can't reach the target just use the full bus headroom (best
  available SNR). NB: use `clamp_f32`, never `f32::clamp` — the latter's assert
  pulls in `core::fmt::float` (~13 KB on g431; the device clippy
  `disallowed-methods` catches it, host clippy does not).
- **2026-06-13 — sensorless startup is a dedicated state machine
  (`phase/startup.rs`), current-only on every board.** A pure back-EMF
  observer can't commutate from standstill, so a pure-`Observer` source needs
  a bootstrap. Phase A = cold-start align→ramp→handoff (VESC I/f, with a
  current-scheduled ramp ceiling); Phase B = deadshort flying restart
  (MESC-style: zero-vector probe, `e = −L·dI/dt` → angle/speed, seed the
  observer and go straight to closed loop). Both run on commanded-v +
  measured-i — **no phase-voltage sense, so g431-capable** (A+B = +1272 B).
  Triggered by `FocDriver::set_mode` (idle→drive) via
  `PhaseProvider::begin_cold_start`; the deadshort holds the bridge at the
  zero vector via `PhaseProvider::wants_short` → `step_direct_voltage(0,0)`.
  Two deliberate boundaries: (1) **kept separate from the hall-dropout
  recovery `OpenLoopOverride`**, which stays UNtrusted (no torque into a
  fabricated frame under a moving rider) — a cold start IS trusted because
  it's a deliberate I/f bring-up; (2) **phase sensing (future VESC board)
  plugs in at the observer's voltage input** (`observer_voltage_source`
  Commanded/Measured), NOT the startup SM — measured back-EMF would just be a
  cleaner flying-restart than the deadshort, the SM is unchanged. Bench-pending;
  v1 limits (deadshort sign assumes commanded direction; handoff angle step)
  tracked in TODO.md. [notes/startup-and-sampling.md]

- **2026-07-07 — the minimal load-bearing set for sensorless spin (ZD2808
  bench, closes the estimator campaign).** What the motor actually NEEDS
  to spin well, each with its bench evidence — anything estimator-side
  not in this table was deleted the same day (next entry) or demoted to
  documented-backstop. The regression harness for this table is
  `scripts/benchsuite.py` (spin-punch-15-2k / openloop-960 / endurance-20s
  with agreed thresholds); run it before and after touching anything here.

  | Mechanism | Why it is load-bearing | Bench evidence |
  |---|---|---|
  | Trust-gate **lag compensation + Schmitt latch** (`observer.rs` readiness: err excused by min(α̂/ki, 0.4), acquire 0.2 / hold 0.6) | The type-2 PLL carries a STRUCTURAL err = α/ki under acceleration; counting it as lock loss made the trust gate a 4–7 Hz bang-bang torque governor — THE mid-band "band oscillation" | gate-fix captures: drops all at err=0.200, gate closed 54% of hold → after: ±60° wobble eliminated, torque delivery 96% |
  | **Speed ceiling** 800 el rad/s + per-cycle `speed_cut` in the iq clamp | The ONLY speed bound in torque mode on an unloaded rotor; the current loop (ωc=1000) loses regulation ~2000 el rad/s → OC | free rotor ran to 19k erpm and OC'd without it; 78 Hz decimated scale bang-banged ±2.3k erpm until the cut went per-cycle |
  | **Flash prefetch** (PRFTEN, `hardware.rs` — embassy G4 leaves it off) | 4-WS flash + branchy hot path without prefetch ⇒ CPI ≈ 2.5; the "ISR speed wall" never existed | one bit: ISR 91–95% → 82–83% FLAT at any speed, worst pass 20k → 7.5k cy, overruns → 0 |
  | **Phase tracker** ωn 100 / ζ 1.2 + `a_est·τ` catch-up lead (`manager.rs`) | First-order pull toward a ramping velocity has a standing ω̇·τ deficit → permanent clamp ride; ωn 30 filtered a wobble that no longer exists while lagging max at the governor's 3.5 Hz | trkfix: drive frame 0.56 → 0.10 rad behind the estimate; governor hunt collapsed as a side effect (cruise ±6% → ±2.3%) |
  | **Two-sided confirm probe + fast-seed** (`startup.rs`: probe ∈ [0.5×, 2×] claim; ≥300 el rad/s failing HIGH seeds immediately; probe current cap 8 A < the 10.8 A trip) | A phantom-locked observer passes every internal gate; only a measurement can refuse the handoff. Retrying a probe against an accelerating rotor walks into the OC (short current ≈ λω/\|Z\|) | confirm2/fastseed captures + every benchsuite start: probe measures 525–578 el rad/s vs observer claim ~195 → seeds → clean closed loop, 0 OC |
  | **External validity** (e_q corroboration gating readiness/λ-learning) | The observer can track the machine's own residual-distortion flux on a STANDING rotor and pass every internal criterion (phantom-handoff deadlock, reproducer #4) | 908886f bench session: staircase deadlock reproduced → fixed; λ-learning no longer follows a runaway |
  | **Deadshort probes** (standstill detect + flying-restart seed) | Distinguishes "ramp cold start" from "rotor already spinning" before driving anything | every benchsuite cold start logs the standstill verdict; catch-spinning path seeds within ~1 el-rev |
  | **Velocity-loop hold during startup** (PI reset while `is_starting`, drive at STARTUP_MIN_DRIVE_A) | The PI integrates against the whole ramp (rotor below target by construction) and hands off with ~6.6 A banked → instant OC | velmode-1 OC reproduced → fix → clean handoff (cruise gains still untuned — open) |
  | **Accel prior** 500 + 10500/A (BACKSTOP, not proven load-bearing) | The slow coherent phantom (PI winds vq up as the back-EMF of its own acceleration) is bounded by nothing else in closed loop: e_q validity is ratio-based and sticky-granted, the confirm probe is startup-only. per_amp = 1.3× the measured free-rotor 9300 el rad/s²/1.15 A | observed on hardware (pre-fix era); A/B 2026-07-07 (ab-prior-off-1,2) PASSED at no-load — kept because the guarded class is real and un-provokable on this bench |
  | **PSU-safe baked profile** (bus_in 3.5 A, bus_regen 0, RampToZero) | 12 V/4 A lab PSU: regen into a CC supply or a >4 A draw browns out / trips it; g431 has no flash persistence so it MUST be baked | standing bench rule since 2026-06-12; violated once (pre-cap), never since |

- **2026-07-09 — boot vbus comes from a measurement, not a config constant.**
  `BoardConfig.initial_vbus_volts` deleted. The old
  `(measured).max(initial_vbus_volts)` seed conflated "not measured yet"
  with "measured low" and could override a real reading upward; on the
  g431 it was outright dead code — the read ran before the ADC IRQ was
  enabled, so the constant always won and the boot-time HFI carrier
  amplitude ceiling was computed from a guess. Now: the ISR sets a
  `VBUS_MEASURED` flag after its first conversion and init awaits it
  (`first_vbus_v()`, one PWM period in practice; 100 ms timeout → dead
  ADC pipeline → log + continue at 0 V, `FocController` floors at
  `MIN_VBUS`, board stays alive for host debugging). On the g431 the
  observer/controller construction moved after IRQ enable to make a
  measurement possible at all. g474 keeps a placeholder literal until
  its dormant FOC ISR is armed. Rationale: the only lasting consumer of
  the boot value is the HFI amplitude ceiling — everything else is
  overwritten by the live per-cycle measurement, and a wrong constant
  there is a silent misconfiguration.

- **2026-07-07 — estimator-campaign dead code DELETED after audit + bench
  A/B** (the "cleanup session"; five parallel audits over freq-led, eddy
  ladder, slip gate, accel prior, rotating carrier, HFI paths):
  - **Observer eddy L(f) ladder** (+ its `MotorParamsConfig` ΔL/τ fields
    and g431 wiring): the pulsating-carrier sweep proved the apparent
    ΔL 146 µH / τ 1.39 ms was the ROTOR swinging on the hold-current
    spring — the true d-axis L is flat ~17 µH to 100 kHz. The storage
    fields had NO runtime writer anywhere (the ladder could never turn
    on), so this was dead infra with a wire-format cost. Postcard layout
    change: old f405/g474 MotorParams blobs fall back to defaults once.
    The virtual-motor PLANT ladder stays (sim disturbance model).
  - **Slip gate** (driver iq-err detector + observer PLL hold): premise
    refuted twice (flat winding kills the L(f)-kick theory; the ratchet
    was root-caused as the trust-gate bang-bang, fixed at source). Its
    bench record was only ever "declawed, not cured", and it could
    freeze the PLL up to 30 ms during exactly the load transients where
    tracking matters. A/B with the gate forced off: full suite PASS.
  - **Rotating-carrier impedance probe**: its one job — exposing the
    motional contamination as the PULS-vs-ROT difference — is done and
    recorded (this file, baked_config.rs); pulsating-d is the instrument.
  - **freq-led**: was already deleted code-wise (PhaseTracker superseded
    it); only a stale test string remained.
  - **HFI**: nothing deleted — correctly feature-gated off on g431, the
    wire-enum/CLI stubs are a deliberate compatibility decision (see the
    2026-06-13 entry).
  All three deletions validated on hardware: canonical firmware
  reflashed, full benchsuite PASS (captures/bench/final-noslip-1).
  Trap for next time: the host CLI must be REBUILT with the firmware
  after a config-struct layout change — a stale host-side
  `MotorParamsConfig` turns every config read into 2 s ack-retry stalls.

## Host / tooling

- **2026-06-11 — esp-config (kconfig TUI) REJECTED** for configuration:
  our config is measured data (R/L/λ/calibrations), not hand tunables.
- **2026-06-10 — push to origin only on an explicit command** (workflow).
- **2026-06-12 — fast telemetry: CIC O2 instead of sample-dropping**
  (Opus 4.8 analysis adopted). Decimation by dropping folds noise in
  (√M floor rise) and real 5th/7th harmonics into fake spectral lines.
  The filter is a triangular 2M−1 window (sinc², nulls exactly on
  k·f_out, −26 dB sidelobes) WITHOUT classic CIC integrators (unbounded
  f32 drifts): per window it keeps A=Σx and U=Σ(k+1)·x, both reset at
  the dump. ~14 cycles per ISR for 7 channels. Angle (wraps at 2π!),
  erpm and hall are instantaneous at the dump. Group delay is M−1 input
  samples — written into the parquet metadata. M=1 is the identity.
- **2026-06-12 — CLI: config access via JSON merge, no per-field code.**
  All stored structs are Serialize+Deserialize+Default → `config get/set`
  works generically for all 10 groups (read → patch → typed write);
  unknown fields are rejected against the real object's keys (serde
  silently ignores them otherwise). `--json` is global: one JSON document
  on stdout, logs on stderr, exit code = success.
- **2026-06-12 — `record` to parquet with an integrity contract**:
  provenance in the file metadata (firmware identity, config snapshot, M,
  CIC group delay), `seq` kept raw — deltas ≠ M count as losses, exit 1
  on gaps (a scripted capture must not silently analyze hole-ridden
  data); a 150 ms warm-up swallows the enable transient.
- **2026-06-12 — oxifoc-virtual: a single consumer of the fast queue.**
  The bbqueue is single-consumer; a stream task per EACH tcp connection
  left zombies of dead CLI sessions stealing frames into dead interfaces
  (NoRoute, 155/5000 samples). The server is single-client: a new accept
  cancels the previous connection. Additionally std builds get a deep
  queue (64K vs the firmware's 2K — tens of ms of tokio jitter must not
  fake losses that embassy doesn't have) and a half-batch drain cadence.

- **2026-08-05 — config-validation split: rich validation belongs to
  host-lib; the device only refuses what it would not apply.** There is
  exactly one host funnel (host-lib — CLI, GUI and benchsuite all ride
  it) and writing to the device around ergot is impractical, so
  field-level validation with actionable messages is implemented once,
  host-side (open item in TODO). The device keeps three narrow gates,
  none of which "validates the host": (1) the boot gate over its own
  flash — old blobs and layout drift never pass through host-lib at
  all; (2) the config endpoint mirrors the pre-existing ISR command
  gate (`DriverCommand::is_sane`), so a write the driver would silently
  drop is refused loudly instead of acknowledged-and-ignored (the
  masking failure found in the 2026-07 review); (3) groups nothing
  consumes are rejected — the device is the source of truth for its own
  capabilities, same as the persist-capable flag. The stale-host layout
  class is closed by the planned schema hash, not by per-field checks.

## Virtual motor / tests

- **2026-06-10 — new plant effects only behind optional parameters with
  ideal defaults** (the `sat_k` pattern) — existing tests stay pinned.
  Every sim upgrade must ship with a test that fails without the
  corresponding compensation. [notes/virtual-motor-fidelity.md]
- **2026-06-12 — the hall-convention lesson: paired sim/estimator
  conventions can mutually cancel and hide systematic biases.** Angle
  estimator tests must check against an INDEPENDENT continuous rotor
  model, not against their own stored angles.
- **2026-06-12 — the plant gained the non-idealities our own firmware
  compensates for**: `substeps` (Euler sub-steps — break the detection's
  lockstep self-confirmation), `dead_time_v` (per phase by instantaneous
  current signs; not applied in `step_shorted` — no switching),
  `adc_lsb_a`/`adc_noise_a` (quantization + deterministic xorshift noise
  on the MEASURED currents only), `lq_sat_k` (saliency
  collapse/inversion). All behind ideal defaults, pinned by
  fails-without-comp tests. The detection harness drives the plant with
  the VOLTAGE RECONSTRUCTED FROM DUTIES (amplitude-invariant Clarke of
  the terminal voltages, ARR 4250 like the hardware) — otherwise the duty
  domain (comp, quantization) is invisible to the plant; it mirrors the
  firmware's dead-time comp configuration automatically.
- **2026-06-12 — pre-bench sim finding: an open-loop `R·I` hold for the
  DirectVoltage detection steps collapses under dead-time distortion**
  (g431: 0.38 V > the entire hold voltage of a low-resistance outrunner;
  the honest open-circuit gate fired MotorNotResponding). The fix is
  threefold: `apply_dq` applies dead-time comp from the measured current
  signs (zero currents → the comp cancels geometrically),
  `settled_hold_voltage` = `R·I + (avg_vd − R·avg_id)` — the measured
  make-up voltage, also robust to an unconverged PI on high-R motors (the
  single-sample capture caught 1.77 V out of 4.16 V on the gimbal), and
  the pulse path moved to the same helper.
- **2026-06-12 — R probe: retry at a higher current only after
  OutOfRange** (a near-short reading). A very-low-R motor on a quiet
  system sits below the duty resolution (ΔV < 1 count → R≈0); ADC noise
  normally dithers this away — the ideal (noiseless) plant exposed the
  degeneracy. A retry at 0.5·current_max is safe: anything ≥ ~0.1 Ω
  already resolves at the gentle probe and never reaches the retry.
- **2026-06-12 — phase advance: into the ACTUATION frame only** (bug
  found by the plant with `actuation_delay_steps`). The old code advanced
  the single commutation angle → the measurement Park frame shifted too →
  the PI parked the current vector `δ = ωe·dt·cycles` off the q axis
  (`id_true = −iq·sin δ`; at full Flipsky speed ~29% of iq as parasitic
  d — torque loss + heat, invisible in the commanded frame). Now: Park at
  the raw angle, `set_actuation_advance()` rotates the OUTPUT vector with
  a small-angle approximation (no second CORDIC; error δ⁴/24 ≈ 3·10⁻⁴ at
  δ=0.3). The steady-state benefit of the actuation-side part is absorbed
  by the PI — what is pinned is exactly the frame split
  (`actuation_advance_must_not_displace_current_vector`).
- **2026-06-12 — the `update_phase_with_prev_voltage` convention is
  sim-proven**: on a plant with a one-period pipeline, prev-pairing of
  the observer's voltage matches the no-delay baseline bit for bit;
  same-cycle pairing degrades 2.2×. Test:
  `observer_prev_voltage_pairing_matches_actuation_delay`. DETECTION
  lacks the same discipline at the needed depth — pipeline-skew confirmed
  critical (TODO.md; bench L untrusted until the fix).
- **2026-06-12 — pipeline-skew fix: measure the latency, don't assume it.**
  The HFI demod paired currents with the injection at a hard-coded depth
  of one cycle; a one-cycle actuation pipeline makes the true depth two
  (90° of carrier phase at 5 kHz/20 kHz) and corrupted L by +1000%+ while
  looking plausible. Now: (1) `probe_hfi_pipeline_lag` cross-correlates
  the current response against carrier references at lags 1–4 before
  every HFI run — the discrete response phase sits at the period CENTER
  (`−cos(φ + ω_c·dt/2)`), so the score projects onto the half-step-rotated
  reference (raw −cos splits 45°/45° between adjacent bins and cannot
  discriminate); (2) `hfi_collect` pairs through an explicit command
  history ring (`record()`'s pairing contract is now documented);
  (3) the probe's |Z| magnitude (`L = √((A/|i|)² − R²)/ω_c`, invariant
  under pairing rotation) cross-checks the phase-sensitive demod — gross
  mismatch → LowConfidence → pulse fallback; the probe runs even when the
  lag is overridden, an override must not disable the safety net (a test
  caught exactly that); (4) the pulse fallback self-aligns by scanning
  for the application edge instead of assuming a one-period window;
  (5) the harness mirrors the firmware's actuation advance SCALED TO THE
  PLANT'S pipeline depth — mirroring the firmware's fixed 1.0 on the
  ideal (zero-pipeline) plant rotated the frame itself and biased the
  flux vector method by |v|·ω·dt cross-terms (gimbal λ −5.8%).
  Regression net: the report's non-ideal table now runs WITH
  `actuation_delay_steps: 1` (10/10 green), plus
  `run_full_detection_nonideal_plant_with_delay` and
  `hfi_mispairing_caught_by_magnitude_cross_check`. Bench L is
  trustworthy once the hardware lag lands within the probe's 1–4 range —
  the probed value gets logged for the bench protocol.
- **2026-06-12 — `HallWithFallback` merged into `HallToObserver`.** The two
  variants had byte-identical behavior (blend + fallback are orthogonal
  duties of one hybrid mode, and the failure chain — observer if ready,
  else open-loop override — is shared by ALL hall sources anyway); the
  only difference was `HallWithFallback.timeout_us`, a dead TODO field
  (the real staleness control is the velocity-adaptive check plus the
  `HallTuning.timeout_us` config). One deliberate postcard variant
  renumber: PhaseSource is never persisted and host + firmware build from
  one tree, so the break is build-time only. CLI keeps `source
  hall-fallback` as the name for the merged mode.
- **2026-06-12 — fault severity classes (phase 1 of the overhaul).**
  `FaultSeverity` (Warning / GracefulStop / Kill) on the wire in
  `FaultInfo`; one central policy (`FaultCategory::severity`, pinned by a
  test) instead of per-board `is_critical`. The `run_foc_cycle` gate is
  class-based: Kill = high-Z + Error latch (as before); GracefulStop =
  the failsafe machinery (ramp/controlled stop), no Error latch — restart
  blocked by the start gate while the fault is active plus the failsafe
  re-arm latch after; Warning never touches the motor (prerequisite for
  hall-health warnings). OverTemp/UnderVoltage downgraded Kill →
  GracefulStop (VESC/MESC reference: stop is the response to "derating
  failed", not to "hot"; the no-restart-while-hot property survives via
  the start gate). The deadman and the Layer-1 link gate now RAISE
  CommTimeout (GracefulStop) instead of silently arming the failsafe —
  same reaction, but visible to the host/remote; a drained SetMode
  (accepted or not) auto-clears it. Design: notes/fault-overhaul.md.
- **2026-06-12 — current-limit ladder hardened (phase 2 of the fault
  overhaul).** The 1.3 headroom factor is now a named invariant
  (`OVERCURRENT_HEADROOM`) enforced everywhere: the board value is the
  ABS trip line (same line the per-phase ISR check kills at) and the iq
  ceiling sits hw/1.3 below it — before, the ceiling WAS the line, so a
  board-limit config met the per-phase Kill exactly at full throttle;
  the dq trip ceiling is the board line itself (was 1.3×hw, ABOVE the
  per-phase Kill — dead code). Incoherent config pairs
  (`max_phase < 1.3·max_iq`, non-finite fields) are rejected loudly at
  the config boundary (`is_coherent` → `ConfigResponse::Invalid`, new
  appended variant) so the user learns the rule; `from_config_clamped`
  additionally clamps whatever arrives by baked/boot paths — protection
  wins over torque: iq is lowered, the trip is never raised. E2E-verified
  against the virtual device.
- **2026-06-13 — hall health → faults (phase 3 of the overhaul).** Hall
  degradation now reaches the registry as a STICKY `HallError(kind)`
  warning: the bridge in `run_foc_cycle` is set-only (a sensor that lied
  once stays on record until host clear; live fallback behavior recovers
  immediately — flapping partial failures must not buzz the remote at
  rev rate). The payload names the degradation: a per-bit wire detector
  in `HallSensor` counts transitions per hall input (each live bit
  toggles 2×/electrical rev) and names dead wires, gated on
  invalid-state events in the same window — rocking across one sector
  boundary toggles one bit legitimately and must not read as "two wires
  dead", while real rotation with a stuck bit produces an invalid state
  every revolution (full rationale in `note_wire_activity`). Error-rate
  window (bounce/EMI) reports only, by decision — degrading commutation
  on a lying-but-half-right hall at low speed is worse than riding it
  until the sensorless promotion (phase 6) gives a real alternative.
  `angle_trustworthy()` is now false while the open-loop recovery
  override fabricates the angle (variant A, agreed 2026-06-13): the
  existing iq gate coasts instead of pushing random-direction torque;
  recovery comes from physical motion (kick-push → observer locks). The
  override also deactivates when the hall itself recovers — previously
  nothing did, so one glitch left it active forever.
- **2026-06-13 — FaultTopic: push faults to consumers (phase 4 of the
  overhaul).** Faults now broadcast on `telemetry/faults` as the FULL
  `FaultResponse` snapshot on every registry change — raise, payload
  refinement (the registry's `set()` now signals when an existing
  entry's value changes; identical re-sets from per-cycle detectors stay
  silent), and clear — plus once at stream start. Snapshot-not-delta
  because ergot topics are fire-and-forget: a lost packet costs
  staleness, never a wrong state; the loss backstop is the consumer's
  SlowTelemetry poll (`fault_count` mismatch → re-query FaultEndpoint).
  Consumers key UI/vibration off `FaultInfo::severity`, never off
  hardcoded categories. Host side: `HostRuntime::fault_rx` +
  `oxifoc-host-cli faults --watch`. E2E-verified against the virtual
  device.
- **2026-06-13 — graduated derating + protection consolidation (phase 5
  of the fault overhaul).** Continuous power rolloff BEFORE any fault
  (`motor/derating.rs`): two min-composed scales — drive (thermal with
  VESC `l_temp_accel_dec` asymmetry, battery cutoff, speed soft ceiling)
  and brake (thermal, regen-OV) — applied per-direction on the iq budget
  in `step_current_control`; braking is NEVER speed-limited and survives
  a sag, a hot board loses acceleration before brakes. New `derating`
  config group (key 11), boundary-validated, live-applied, defaults =
  FET 85→100 °C only (per-vehicle ramps need bench numbers). The
  `Derating` warning (auto set/clear at 0.8/0.95 — a live state, unlike
  the sticky hall record) plus scale percentages in SlowTelemetry answer
  "why does the board feel weak". Voltage faults switched from
  single-sample trips to V·s excursion integrals (VESC's
  wrong_voltage_integrator; ~3 ms at 1 V over) — regen spikes and sense
  blips stop costing torque. Consolidation along the way: voltage/temp
  fault checks moved from per-platform ISR copies into
  `FocDriver::run_protection`, and core raises faults via
  `PlatformFault::from_category` (no per-category value parameters in
  `run_foc_cycle` any more) — closes part of the ISR-dedup TODO.
- **2026-06-13 — code-quality pass: strict lints, import hygiene, fault
  dedup.** Curated `[workspace.lints]` (rust `unused_qualifications`;
  clippy `use_self`, `uninlined_format_args`, `cast_lossless`,
  `manual_let_else`, `semicolon_if_nothing_returned`,
  `redundant_closure_for_method_calls`), verbatim copies in the
  workspace-excluded device crates. Full `clippy::pedantic` REJECTED:
  ~1500 warnings, mostly noise (`must_use_candidate`, `doc_markdown`),
  and `suboptimal_flops` would rewrite FOC math to `mul_add` — different
  rounding on hardware-validated control paths, not before a bench
  baseline. ~330 inline `crate::...` qualified paths became `use`
  imports (doc examples, rustdoc links, `$crate::` macros and genuine
  disambiguations stay). G431/G474/Virtual fault enums were
  byte-identical clones — now one `core::foc::fault::StandardFault`
  (F405 keeps its enum for the DRV8301 payload); the dead
  `is_recoverable`/`auto_clear_recoverable` pair (no callers since the
  overhaul) is gone. host-cli main.rs (1355 lines) split into
  config_cli/detect/watch modules. Considered and DEFERRED: an ISR-glue
  trait for the per-board FOC interrupt skeleton (hardware-validated ISR
  code, not before the bench session; the remaining dedup is tracked in
  TODO). Considered and REJECTED: a macro to collapse the per-config-
  group plumbing (ConfigKey/GroupId/Write/Response/Payload across 5
  files) — the postcard append-only wire discipline depends on the
  variant numbering staying explicit and reviewable; a macro would hide
  exactly the thing reviews must see.
- **2026-06-13 — lint wave 2: failure-class guardrails + firmware panic
  policy** (inspired by emschwartz.me "your clippy config should be
  stricter"; measured against this codebase first). Added zero-hit
  guardrails to every lint table (`dbg_macro`, `string_slice`,
  `rc_mutex`, `debug_assert_with_mut_call`, `expl_impl_clone_on_copy`,
  `infallible_try_from`, `invalid_upcast_comparisons`, `large_futures`,
  `unused_result_ok`, `iter_not_returning_iterator`, `mem_forget`,
  `undocumented_unsafe_blocks`, `multiple_unsafe_ops_per_block`,
  `lossy_float_literal`) — they cost nothing today and lock the
  discipline in. The panic family (`unwrap_used`, `panic`, `todo`,
  `unimplemented`, `get_unwrap`, `unwrap_in_result`,
  `panic_in_result_fn`) applies to FIRMWARE only: a crate-level
  `#![warn]` block in oxifoc-core/src/lib.rs (composes on top of the
  inherited workspace table — core stays a normal member) and the
  device-crate tables; host crates are exempt (CLI/GUI panics are an
  acceptable failure mode, thousands of legit unwraps). clippy.toml
  `allow-unwrap-in-tests` & co exempt test code; build.rs files carry a
  file-level allow (panicking IS a build script's failure mode).
  Measured before enabling: firmware core had 1 unwrap / 0 panics — the
  policy was already true, now it's enforced. Real catches: 2 f32
  literals in trig.rs flagged by `lossy_float_literal` (rewritten as
  `(1i64 << 31) as f32`), a gated `take().unwrap()` in sweep.rs
  (rewritten as a let-chain). Deliberate sites documented in place:
  `mem::forget` of CapturePins (Drop would revert AF config) and the
  NVIC bring-up unsafe blocks got `#[expect(reason)]` + `// SAFETY:`.
  Reviewed all 24 `let _ =` sites (`let_underscore_must_use`): every one
  is deliberate heapless-truncation or best-effort ergot reply — lint
  stays off. Also NOT enabled, with numbers: `indexing_slicing` (67
  const-bounded phase-array sites = churn), `float_cmp` (80/81 hits in
  tests), `cast_sign_loss` (14 sites needing invariant docs — revisit).
- **2026-06-13 — embassy slice is now clippy-gated.** The embassy-only
  modules of oxifoc-core (hall_embassy & co) are compiled by no workspace
  member, so workspace clippy never saw them — the no-default-features CI
  line ran `cargo check`, which skips lints. Found via a cast_sign_loss
  audit: a `redundant_closure_for_method_calls` from our own wave-1 set
  had survived there. The justfile line is now `cargo clippy -- -D
  warnings`; stragglers fixed (Cell::get method ref, TickSourceFn alias,
  HallAngleProxy derives Default).
- **2026-06-13 — async-debloat experiment (tweedegolf "Debloat your async
  Rust" applied with measurements).** Wins on g431: DetectionBackend
  impls were pure pass-throughs (`async fn { inner().await }`) — now
  `fn -> impl Future` returning the inner future (−380 B flash g431,
  −616 B f405); the boot current-sense calibration buffered 1024 ADC
  samples (heapless::Vec, 6 KB) across its sampling awaits just to take
  a MEAN — now streaming sums (.bss 23 860 → 17 716, embassy_main arena
  7 736 → 1 608 B; on a 32 KB-RAM part that's +6 KB of stack headroom).
  Found via `large_futures` with `future-size-threshold = 2048` in
  clippy.toml — NOTE: device crates have their OWN clippy.tomls (they
  shadow the root one; that's also where the f32::clamp ban lives), the
  threshold is set in all of them. `unused_async` enabled everywhere
  (2 host transports de-asynced). NEGATIVE RESULT, kept for the record:
  collapsing the config-server's 9 per-arm `send().await`s into one
  awaited loop over a command Vec made BOTH flash and arena BIGGER
  (+176 B / +232 B) — the coroutine layout already overlap-allocates
  per-arm states (storage_conflicts), while the hoisted Vec lives across
  the await on top of the send future. "Share await points" only pays
  when arms duplicate the same awaited code path; reverted. The
  "host-side" exception: virtual's ~6 KB udp server future is Box::pin'd
  (threshold is firmware-tuned; the host has a heap). FOC ISR is sync —
  none of this touches control timing.

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
- **2026-06-11 — F405: the blocking 128 KB sector erase (~0.5 s) is
  accepted as a known residual.** The TOCTOU is closed by the Busy gate
  (the motor is guaranteed stopped for the duration of the write), the
  IWDG margin is the backstop. Not fixing until it hurts.
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

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
- [ ] **Detection runs with the baked-params decoupling feedforward active —
  circular, and measurably biases flux (2026-07-06 bench).** Same motor,
  same 2800 eRPM spin, same math: pre-bake firmware measured λ = 1.167 mWb;
  post-bake (motor_params baked → `from_runtime_config` arms the dq
  decoupling FF built from λ/L — the very quantities detection measures)
  the device AND an independent reconstruction from the capture both give
  1.33 mWb (+16 % vs the 1/ω-extrapolated true 1.145). vd grew to −0.40 V
  at steady spin. Suspect the FF contribution is not consistently
  contained in the reported vd/vq that the back-EMF-vector math consumes.
  Fix direction: detection should disable the decoupling FF (and any other
  param-derived feedforward) for the duration of a measurement — measuring
  through a model built from the previous answer is circular even when the
  reporting is consistent. Verify by re-running flux at 2800 with FF
  forced off and expecting ~1.17.
- [ ] **Control-grade AC inductance from detection — the "unknown motor"
  gap** (why the 2026-07-05 baked ZD2808 config carries an LCR-meter
  value). The g431's only L method (voltage-pulse) measures ~DC L; on
  eddy-current-heavy motors (laminations + conductive magnets — ZD2808:
  DC 86–129 µH vs the AC ~24 µH plateau from ~1 kHz) that is 4–5× the L
  the control loops need (`kp = L·bw`, the observer's `L·di/dt`), so a
  full detect + `--apply` would hand out ~4× hot PI gains. No
  self-sufficient path exists today. Options, in rough order:
  1. Finish the `impedance-sweep` experiment (one-lock R(f)/L(f), exists
     behind the feature flag; noisy as of 2026-06). Its known next steps
     are written down in
     [notes/inductance-freq-detection.md](notes/inductance-freq-detection.md)
     — fixed d-axis injection (drop the 625 Hz rotation beat), phase
     calibration — and its blocking prerequisite (full-rate capture for
     an offline spectrum) fell 2026-07-05: detection now records
     loss-free. Caveat: the raw diag frame decimates by plain
     sample-dropping (no CIC), so the sweep spectrum wants an M=1
     capture (lossy under drive is acceptable for a spectrum look).
  2. A slim fixed-axis HFI-|Z| probe at 1–3 kHz as the production L step
     (reads the plateau value; no FFT needed — |Z| is phase-robust).
  3. At minimum: tag pulse-L as DC-L in the result and refuse to derive
     PI gains from it above some DC/AC divergence heuristic.
- [ ] Detection HFI amplitude: floor from the ADC resolution —
  gimbal-class (ripple ~30 mA against a 15 mA LSB) gives Lq −14%, λ +8%
  on the non-ideal plant; the adaptation currently targets a fraction of
  the hold current without looking at the LSB.
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

Phase A (cold-start align→ramp→handoff, current-scheduled ceiling) + Phase B
(deadshort flying restart) implemented 2026-06-13 in `phase/startup.rs`.

- [x] **Bench-validated 2026-07-06** (ZD2808, 12 V/4 A): cold start →
  align → ramp → early handoff → acceleration to the 12 V no-load
  ceiling (~50.4k erpm = 7200 mech rpm, vq saturated ~6.8 V, consistent
  with Kv 688) → 2.5 s stable 0.3 A hold → clean unload
  (`maneuvers/spin-gentle.json`, captures/spin-gentle2/3.parquet).
  Three bugs found and fixed on the way — every earlier "divergent"
  spin attempt is explained by them:
  1. Host affirms were sent to a nonexistent socket name (`"affirm"`
     instead of `"motor"`) — silently dropped, deadman fired +152 ms
     into EVERY drive (08225f1).
  2. Deadshort false-caught a standstill rotor: the bridge-enable
     current transient (~0.4 A, ~200 µs) read as back-EMF → ω≈46 just
     over the 45 floor → observer seeded with garbage. Fixed with an
     8-period settle before the baseline + floor 45→60 (8938bed).
  3. The handoff gate waited for the RAMP to reach 60 rad/s while the
     unloaded rotor slipped ahead of the I/f drag and ran away (real
     380–800 rad/s, confirmed by phase-current frequency) — now a
     READY observer at handoff speed takes over immediately (05505d3).
- [ ] **1.5 A mid-speed limit cycle (estimation chain)** — the remaining
  hard problem after the 2026-07-06 current-loop session (commit
  ca35522 carries the full experiment matrix):
  - FIXED that session: the overcurrent DIVERGENCE at ~800 rad/s —
    the dq-decoupling undercompensated 4.5× by the AC L; fundamental
    (pulse) Ld/Lq in the decoupling removed the trip. The decoupling FF
    is reference-current-based now (measured-current form is a delayed
    ω·L feedback path).
  - REMAINS: a bounded ±7–9 A dq limit cycle at 1.5 A, speed wandering
    3–10k erpm, iq mean collapsing episodically — across EVERY
    combination of kp {0.024, 0.1075} × dec {24, 86/129} ×
    obs {24, 108-salient}. At 0.3 A all configs are stable to the
    no-load ceiling → perturbation scales with L·i. Sim: the current
    loop with a perfect angle is unconditionally stable (any advance),
    so the cycle lives in the OBSERVER↔loop interaction; the single-L
    sim plant does not reproduce it (frequency-dependent L missing).
  - Investigation plan: sim with dead_time_v + adc noise at bench
    levels; λ-tracker dynamics under flux-vector wobble; observer
    readiness needs EXTERNAL validity (N consistent flux revolutions,
    not confidence+PLL-error — the align swing produces false ready at
    585–786 rad/s); eddy-branch plant model (parallel R-L) if the
    above doesn't reproduce; VESC-style observer gain scheduling.
  - **Two-inductance model**: MotorParamsConfig needs an explicit
    second inductance (HF/AC vs fundamental Ld/Lq) — the estimation
    chain (observer L·i, deadshort e=−L·dI/dt) is hardware-validated
    on the AC value, the decoupling needs the fundamental pair;
    today's split lives as a set_decoupling override in g431 foc.rs.
  - **Align swing (partially fixed 4ca41ab, redesign pending)**: the
    fixed-angle align resonates the rotor on its undamped magnetic
    spring (~8 Hz mech). Current soft-start (0.15 s ramp) + the 35%
    runaway gate eliminated the false handoffs (observer locking onto
    the swing), but ~2/5 cold starts at 1.5 A still trip the dq
    overcurrent during align/early ramp — no current-shaping fixes a
    resonance with no damping. Next: VESC-style align = 0 (ramp from
    the unknown angle — a rotating field doesn't pump the resonance),
    or active damping during align. Same investigation as the observer
    readiness work: the swing is also what poisons its flux integrator.
- [x] ~~10 kHz capture during drive still trips the deadman~~ ROOT CAUSE
  FOUND 2026-07-06 (afternoon session): **ISR saturation during align**,
  not a comms/freeze mystery. VECTACTIVE sampling over SWD (ICSR @
  0xE000ED04 next to DWT_PCSR) showed the ADC1_2 FOC ISR at 96-100% of
  ALL samples for ~200 ms at drive engage: the align path (drive step +
  startup tick + observer + 1 kHz telemetry push + profiling marks)
  costs ~7.9-8.3k of the 8500-cycle budget — *just under* the overrun
  counter's threshold, which is why `over=` stayed low while thread mode
  got 0-5% CPU. Consequences: RxWorker→motor-server→CMD_CHANNEL latency
  inflates 20-50× → affirms don't drain within the 150 ms deadman →
  stochastic trips (2026-07-06 afternoon: ~2/3 of spins — the freshly
  added isr-profiling marks (~200 cycles/cycle) were the tipping straw
  vs the clean morning runs on 8a42bb6). At 10 kHz the same mechanism
  starved the telemetry reader → the historic 1502-sample gap. Fix =
  the ISR tier-2 shave below (headroom is a *correctness* requirement
  now, not just a capture-rate wish). Host-side hardening landed the
  same day: command sends are committed inline (ordered) but response
  waits are detached observers, so a slow round-trip can no longer
  delay the first affirm past the deadman (see send_motor_now in
  host-lib). Diagnostic toolbox kept: device `stale_max_us` /
  `pump/s gap+timer_late` / `exec stall` marker / `isr over=` /
  `hall_edges`, host per-phase stall watch + down-write gap +
  `OXIFOC_PC_SAMPLE` SWD PC/VECTACTIVE sampler.
- [ ] deadshort **sign-from-progression**: the ±90° angle offset assumes
  the rotor spins in the *commanded* direction (kick-push). A rotor
  freewheeling AGAINST the command needs a ±180° PLL pull it can't do —
  track the back-EMF vector's rotation across the probe cycles for the
  sign. [`startup.rs` `deadshort_estimate`]
- [ ] deadshort from an **already-shorted** entry (Brake/Stopped): steady
  current → ~0 dI/dt → speed underestimated (clean case is Coast). Discharge
  to i≈0 (brief high-Z) before the probe if it matters on the bench.
- [ ] **handoff smoothing** ramp→observer: the commutation angle steps by
  the load angle at the switch; seed/blend it, and consider preloading the
  current-PI integrators (MESC seeds Vd/Vq) to kill the re-engage transient.
- [ ] Affirms rejected during a latched failsafe warn at 20/s
  ("Mode rejected: failsafe latched") — rate-limit the warn; and the
  stopping-fault refusal of Stopped→active is silent — add a one-shot
  warn naming the blocking fault.

## Sensorless tracking / BEMF (bench-blocked)

- [ ] Bring up the B-G431B-ESC1 phase dividers (BEMF sense) as ADC channels.
- [ ] MESC-style TRACKING: gates off → measured v_αβ into the observer →
  flying start from a converged observer. Hall-based already works; this
  is the sensorless case. Also unlocks the spin-down flux method on
  hardware (`supports_coast_telemetry`).

## Firmware / core

- [ ] **embassy-time thread-timer freeze under ISR load (g431, 2026-07-05)** —
  moving the RTT TX loop to a SAI1 InterruptExecutor (P6) froze ALL
  thread-executor embassy timers for a deterministic ~44.93 s while
  detection+streaming ran (1 Hz stats task, 4 ms detect-ramp timers, a
  dedicated 2 ms keeper task — all dead; revived only by incoming RX
  traffic, i.e. by some fresh `schedule_wake` re-arming the TIM2 alarm).
  With the TX loop back on the thread executor its frequent short backoff
  timers mask the problem. Root cause in embassy-stm32's gp16 time driver
  (or our use of it) not found — the ~44.93 s constant reproduces ±5 ms
  across different firmware builds, so it is NOT a random race. Plan:
  enable `rtos-trace` on embassy-executor (+ SystemView or a postcard
  trace sink over RTT) and trace executor/timer events around the freeze;
  also check TIM2 IRQ priority vs ADC1_2(P0)/SAI1(P6) and the
  `next_period`/CCIE(1) re-enable path. The pluggable schedulers
  (`scheduler-priority`/`scheduler-deadline`) are NOT the fix — the
  default scheduler is already fair; the wakes themselves are missing.
- [ ] **Voltage-pulse L step is incompatible with a concurrent fast
  stream** — `pulse_once` takes the max *per-wait_telemetry-frame*
  current rise and assumes one frame = one FOC cycle (`dt = 1/f_pwm`).
  Under a concurrent stream the executor serves the detect task every
  ~10–26 cycles, so a "frame" spans several cycles of the L/R
  exponential: measured on the ZD2808 as Ld/Lq = 169/306 µH with a
  10 kHz recording vs the true 86/129 µH without (≈2× high). R (2-point
  steady-state averages) and flux (steady-spin averages) are immune.
  Fix options: sample the pulse window ISR-side (robust, more plumbing);
  or use FocOutput.seq deltas + an exponential fit instead of the
  1-cycle linearization; or have the detect server pause fast telemetry
  around the pulse train. Until then: run `detect inductance` WITHOUT
  `--record`.
- [x] **RTT attach intermittently fails on the next host session** —
  RESOLVED 2026-07-05 (commit ca635b4). Root cause: the blocking RTT I/O
  thread (owns the probe-rs Session, busy-polls USB) was never joined;
  process exit killed it mid-USB-transfer, leaving the ST-Link with a
  torn command — the next open timed out on GET_CURRENT_MODE, recovered
  into Jtag mode by probe-rs's USB reset, then timed out again on
  JTAG_EXIT (~50% of back-to-back runs, roughly alternating). Fix:
  teardown-ordered shutdown — interface teardown releases the transport
  (drops the reader), the thread notices per-iteration via
  `is_closed()`, the shutdown path joins it (bounded) so Session::drop
  detaches the probe cleanly; HostRuntime joins the backend thread in
  shutdown()/Drop. Verified 15/15 back-to-back attaches + 3 stream
  cycles. The attach poll loop also logs the real attach_region error
  now (anyhow used to hide it).
- [ ] **Branch-review leftovers (2026-07-06, low/latent — fix by
  opportunity).** From the full June-commit review; the real bugs were
  fixed same-day (a50ff1f/118ce54/bce662a/922a15a):
  - `MOTOR_POLE_PAIRS` is latched at init on all three boards — a runtime
    `ConfigWrite(MotorParams)` desyncs the frame's rpm (old pp) from host
    erpm (new pp) until reboot; boot with no params → rpm hard-0 all
    power-up. Re-apply on config write, or read the config each cycle.
  - Fixed-point packers wrap instead of clamping past range ends
    (`pack_vbus` >131 V, `pack_volt` >±65.5 V, `pack_rpm` >±65534) —
    unreachable on current 12–57 V hardware; one `.clamp()` per packer.
  - Host→device RTT down-channel write loop spins forever on `Ok(0)` if
    the device stops draining (hung firmware) — bound it or check
    `is_closed()` inside.
  - u16 seq: losses ≥ one wrap (>3.3 s at 20 kHz) alias modulo 65536 in
    both t_s and the gap counter; a host-clock plausibility check would
    catch it.
  - g474/f405: no `compile_error!` on a zero-transport build (silently
    unreachable brick); f405 `init_clock`'s boot defmt line is emitted
    before the sink exists (0da8693 reorder).
  - bridge/remote report an all-zero `BoardCalib` — if that handshake
    ever feeds `build_enrich_ctx`, `adc_max_counts = 0` divides by zero
    (NaN currents). Should be `Option<BoardCalib>`.
  - GUI enrich ctx is built once per connect click and never rebuilt on
    auto-reconnect/device reboot (stale offsets); angle/erpm chart pushes
    (0,0) without a ctx although both decode calibration-free.
  - virtual sim's raw-ADC encode truncates instead of rounding (+15 mA
    avg per phase — the measured 0.06 A Kirchhoff residue); `+0.5` fixes.
  - `impedance-sweep` (debug feature): the "robust" `l_from_z` uses DC R
    at every frequency (self-inconsistent — AC-R rise is the point) and
    returns only the last sweep point as (ld, lq).
  - HFI probe-current budget: the 0.2 V floor can exceed the 2 A cap on
    very-low-R+low-L machines (`sweep.rs` `calibrate_pulse_voltage`
    comment overstates the guarantee).
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
  bugs). Same refactor should absorb the `init_foc` boot-sequence tail,
  duplicated byte-for-byte across g431/g474/f405: `set_failsafe` /
  `set_velocity_config` / `set_derating` from stored config, the ADC
  settle delay, `calibrate().await`, the DcOffsets publish block, and
  the `FOC_DRIVER` install. No technical blocker — all three crates
  declare the identical `Mutex<RefCell<oxifoc_core::storage::RuntimeConfig>>`
  static, so a core helper taking `&'static` refs works as-is; the
  g474 DcOffsets miss caught in the 2026-07 branch review is exactly
  the bug class this eliminates.
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
- [x] Live ISR load counter — DONE on g431 2026-07-05 (DWT CYCCNT
  avg/max/load 1 Hz over defmt, `isr/s:` line; g474/f405 still pending).
  First real numbers, and they are damning: **Stopped + 20 kHz stream =
  5128 cycles avg = 60% of the 8500 budget; OpenLoop detection = ~6600 =
  78%**. The perf-bench composites (~900–2900) never included the ISR
  glue — the baseline glue alone is ~5100 cycles (2× ADC-handle CS locks,
  hall snapshot, run_foc_cycle dispatch+protection, telemetry encode+push,
  state CS update, waker). This is what caps the telemetry pipeline at
  ~10–14.6 k samples/s during detection (thread mode gets ≤25% CPU) —
  detect --record therefore records at 10 kHz (`--record-hz 10000`),
  loss-free. The deferred "ISR-glue refactor" now has a concrete target:
  profile the glue by parts (CYCCNT around sections) and get the Stopped
  baseline well under ~3000 cycles before expecting 20 kHz capture during
  drive.
- [x] **Per-section ISR profiling + tier 1 — DONE 2026-07-06 (6e31fa5)**.
  Permanent 1 Hz `isrp/s` (ISR sections) + `isrc/s` (run_foc_cycle
  internals, core feature `isr-profiling`) defmt lines. Measured and
  fixed: per-cycle `pwm.disable()` at Stopped (429→26), estimator
  update on constant zeros at Stopped (decimated ×4 dt-scaled,
  1429→387), NTC `libm::logf` every cycle (every 128th now, adc1
  561→223). **Stopped + 20 kHz: 6184→4530 cycles (73%→53%), capture
  loss-free again** (was 472 gaps/34k lost).
- [ ] **ISR tier 2** — now a CORRECTNESS item, not just a capture-rate
  wish: the drive/align ISR at ~90-100% of the 8500-cycle budget starves
  thread mode and trips the deadman at engage (see the root-caused
  10 kHz item in the sensorless section). The step section split landed
  2026-07-06 (`isrd/s: gate/ctrl(trig)/post/est`); scaled from mixed
  engage windows (drive ≈30% of window): **est≈1900** (phase manager:
  observer+PLL+startup — the mountain), **gate≈1150** (?! clamps/
  derating/`phase.get`/`read_currents` — suspiciously fat for scalar
  work), **ctrl≈1070** (of which CORDIC trig≈170), **post≈590**; sum
  ≈4700 ≈ the earlier step=4900. Per-cycle drive total ≈6800 + 7000 on
  each 1 kHz push cycle. Older section numbers still valid: `cmd=742`
  (cache link/fault flags in atomics), `prot=605` (decimate derating
  mirror + temp CS), `pub=1205` avg under stream. Order of attack:
  est (why is the estimator 1900?), gate (should be ~200), cmd/prot
  CS-consolidation. Goal: drive-mode total ≤ ~6500 AND align ≤ drive
  (align must not be the costliest phase). NOTE: the isr-profiling
  marks themselves cost ~200 cycles/cycle — measure with them, budget
  without them.
- [ ] **g431 RAM: stack → CCM SRAM split** (idea, robustness — not a
  throughput lever: raw-Pod made 20 kHz loss-free at Stopped, and the
  under-drive cap is ISR CPU, not buffers — see the ISR-load item above).
  G431's 32 K = SRAM1 16 K +
  SRAM2 6 K + CCM 10 K; CCM is dual-mapped (native `0x10000000`, alias
  `0x20005800` glued after SRAM2). `memory.x` today declares one 32 K region
  via the alias; flip-link gives the stack whatever statics leave over —
  measured: boot OK at ≥7.8 K stack, LOCKUP at ≤5.8 K, and every added
  static silently eats the budget. Split instead: `RAM 22K @0x20000000`
  (statics) + `CCMRAM 10K @0x10000000` with
  `_stack_start = ORIGIN(CCMRAM) + LENGTH(CCMRAM)`. Gains: fixed 10 K stack
  budget decoupled from statics; hard overflow protection for free (nothing
  is mapped below `0x10000000` → BusFault, flip-link no longer needed);
  zero-wait-state stack on the dedicated core port (no contention with the
  host's constant SWD/RTT reads of SRAM). Costs: no new memory (statics are
  ~25 K > 22 K today → first shave ~3 K: defmt ring 1024→512, the 4.6 K
  `protocol_servers` task POOL is the fattest target); CCM is CPU-only
  (fine for stack, no DMA there ever). Step 1 when picked up: paint the
  stack region at boot and read the high-water mark over SWD after a heavy
  stream+detect session — the real usage is somewhere in 5.8–7.8 K, split
  numbers should be measured, not guessed.

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

- [ ] **GUI config coverage** (after the host-lib `ops` layer, 2026-06-13,
  which backs both front-ends so config/phase/detect can't diverge; the GUI
  also gained a faults panel, Coast/Brake, Config Reset and an Id field):
  the GUI config form still edits only 6 of the 11 groups (motor-params,
  current/voltage-limits, pi-gains, velocity, failsafe). Add typed forms for
  **hall-tuning** and **derating** (the remaining hand-tunables). dc-offsets,
  hall-calibration and pwm-config stay CLI-only by design — calibration
  outputs / build-time, not hand-edited. The CLI keeps the full 11-group
  `config get/set/dump`.
- [ ] **Device-RAM burst capture** (the VESC `sample` pattern): raw
  20 kHz into a ring buffer (~100–200 ms on g431) downloaded afterwards,
  pre-trigger around faults (a "black box"). The 5 kHz CIC stream is for
  long logs, burst for full-bandwidth snapshots. **Motivated by the
  2026-06-13 bench finding:** live telemetry over the g431 ST-Link tops out
  at ~1.8 kHz effective on BOTH transports — UART (921600 ≈ 2 kHz) and RTT
  (this ST-Link's RTT sustains ~82 KB/s; at 5 kHz only ~37 % of frames
  arrived and `NoBlockSkip` overflow corrupted COBS frames → ergot decode
  errors). RTT ergot itself works (control-block now pinned to `_SEGGER_RTT`
  in host-lib + flashprobe-mcp), but it's not a bandwidth win on this probe.
  Levers before burst-capture: enlarge the RTT ergot buffer (RAM is free) and
  try `BlockIfFull` to stop frame corruption — but neither beats the probe's
  sustained throughput; burst-capture is the real fix for full-rate snapshots.
- [ ] **`detect --record` rate**: it's hardwired to the FOC rate (M=1,
  20 kHz) which floods every available link and starved the detect response
  over UART (host hung, device kept driving the motor — see runaway note).
  Make the record rate configurable (≤2 kHz is link-feasible; the HFI carrier
  case still wants burst-capture), and/or have the device-side detection
  abort the drive on host link-loss instead of running to completion blind.
- [ ] **Protocol versioning & compatibility** — full design in
  [notes/protocol-versioning.md](notes/protocol-versioning.md). Premise of the
  old note was wrong: ergot keys = hash(path + recursive schema), so a type
  change → new wire address → `NoRoute` (fail-closed), **not** silent garbage —
  except **topics**, which fail as *silent absence* (the real reason to gate at
  connect). Plan: ergot-side `ergot_proto_version` in well-known `DeviceInfo` +
  a `DeviceInfo` handshake endpoint (L1); socket-table introspection
  `served_digest` + `SocketQuery` enumerate-all (L2); later opt-in
  `#[schema(evolve)]` append-tolerant keys. oxifoc-side: split the custom
  device-info — identity → ergot `DeviceInfo`, motor descriptor → lean
  `AppInfoEndpoint` (foc/current/**BoardCalib**/semver via
  `env!("CARGO_PKG_VERSION")`), drop the HardwareInfo handshake role. Mandatory
  before any release/distribution.
- [ ] **Fast-telemetry enrichment** (raw 18-byte frame → engineering units in
  CLI/GUI via one shared `oxifoc-core` path) —
  [notes/telemetry-enrichment.md](notes/telemetry-enrichment.md): `Scale`
  fixed-point codec (paired enc/dec, one LSB per field) + `enrich()` reusing
  `ShuntCurrentSense::convert_raw` + `clarke`/`park`; `BoardCalib` as a
  sub-struct of `BoardConfig` carried in `AppInfoEndpoint`; offsets/pole_pairs
  via existing config reads; round-trip/golden tests in core.
- [ ] Reconnect state machine has no test coverage; slint-wgpu-plot: the
  ring index arithmetic (`renderer.rs:262`) under a large zoom-out +
  scroll-back may compute the Y auto-range over a different window than
  the shader draws.
- [ ] bridge/remote: pairing via a hardcoded MAC; stub tests. Remote
  design — [notes/remote-design.md](notes/remote-design.md).

## Bench (waiting for hardware)

### Bench session 2026-07-05 (g431 + ZD2808) — detection re-measured, with recording

First detection run with full telemetry recording (the original goal of the
RTT pipeline work). Setup: B-G431B-ESC1 + ZD2808 (wye, 7 pp), 12 V / 4 A PSU,
firmware at 295a800 (transport-rtt + detection), `--record-hz 10000` (M=2,
loss-free; M=1 loses ~half the samples during detection — see the ISR-load
item under Size / performance).

| step | 2026-07-05 | 2026-06-13 | notes |
|------|------------|------------|-------|
| R    | **0.1271 Ω** (recorded, 71 k rows 0 gaps) / 0.1273 no-record | 0.127 | ΔV/ΔI recomputed from the capture = 0.127 ✓; LCR 0.104/ph + dead-time |
| Ld/Lq | **85.7 / 129.4 µH** (no-record — MUST run unrecorded, see Firmware/core item) | 86 / 122 | pulse ≈ DC L; AC L stays ~24 µH (eddy currents, known) |
| λ    | **1.2786 mWb** (recorded, 111 k rows 0 gaps, spin confirmed 700 eRPM) | 1.30 | expected ≈1.13 → bias REPRODUCED, then RESOLVED (below) |
| Kv   | **616 RPM/V** | 611 (after √3 fix) | nameplate 700 |

Everything reproduces June within ~2 % — June's numbers were solid (they were
taken without recording, so the ISR-trigger-loss bug never touched them).
Captures: `captures/detect-r-10k.parquet`, `detect-flux-10k.parquet`
(`detect-l-10k.parquet` exists but its device result is the distorted one).

**λ +13 % bias RESOLVED (same session, offline analysis of the capture +
validation run).** The back-EMF-vector λ at one speed carries an additive
`V_err/ω` bias: regressing the capture's per-window λ against 1/ω over the
ramp (iq held at 2 A) gives a clean fit (rms ~1 %) with **λ_true =
1.145 mWb** (intercept) and **V_err ≈ 9 mV** — the residual bridge/dead-time
error after compensation (0.38 V raw → 9 mV ≈ 97.6 % cancelled). At the
default `--erpm 700` the BEMF is only ~0.09 V (vs R·i ≈ 0.25 V), so 9 mV =
+12 %; the measurement regime is just too slow for a 12 V low-λ drone motor.
Validation: `detect flux --erpm 2800` → device reported **1.1673 mWb / Kv
675**, within 0.8 % of the model's prediction (1.176). True Kv ≈ **688** —
1.7 % off the nameplate 700. Motor and firmware math are fine.
- [ ] flux step: raise the default spin speed (scale `openloop_erpm` to the
  motor: target BEMF ≥ R·I, e.g. ω ≥ 3·R·I/λ_est) and/or measure at 2–3
  speeds and extrapolate 1/ω → 0 device-side; document the bias otherwise.

### Bench session 2026-06-13 (g431 + ZD2808 700 KV sensorless) — findings

First real-hardware run of the g431 firmware on a sensorless drone motor
(ZD2808, 700 KV, 7 pp; 12 V / 4 A lab PSU). Detection ran end-to-end:

- [x] **HW comparator OCP — RESOLVED 2026-06-13: unusable on this board, break
  disabled.** Proven by on-device DAC sweep + stm32-data + host PWM test (full
  account in docs/hw/b-g431b-esc1.md). COMP1/2/4 tap the *raw shunt pad* (idle
  128 mV, slope only R_shunt×4/7 ≈ 1.71 mV/A — the ×16 PGA is downstream, so the
  comparator never sees the amplified signal), not the op-amp output. No current
  threshold clears the PWM switching-noise band; even ST's near-rail DAC=4083
  (≈3.29 V), with the break enabled, trips to Error on the *first* PWM-output
  enable every time (capacitive gate-drive transient spikes the hi-Z pad to the
  rail) → the motor can't start. ST parks the DAC at the rail (≈1850 A-equiv =
  effectively off) and relies on software OCP; so do we. `set_break_enable(false)`
  in `motor.rs`; COMP+DAC still configured near-rail for one-line re-arm *if*
  ST-style enable-sequencing (boot-cap-charge + non-fatal enable-window break) is
  added. Real protection = software measured-OC (40 A, ×9.14 ADC) + PSU CC.
- [x] **Detection biases — ALL three explained/fixed** (kept as a pointer;
  the original item's dead-time theory for L was WRONG). (1) L 3.6–5×
  "high": the pulse method measures ~DC L and L is genuinely
  frequency-dependent (eddy currents, NOT dead-time — disproven on HW
  2026-06-13c, see
  [notes/inductance-freq-detection.md](notes/inductance-freq-detection.md));
  the remaining gap is control-grade AC-L from detection — open item in
  «Algorithms». (2) λ +15 %: resolved 2026-07-05 — additive `V_err/ω`
  bias at the slow default spin, λ_true = 1.145 mWb (see the 2026-07-05
  bench section above). (3) Kv ×√3: fixed 2026-06-13 (`calculate_kv`
  carries the phase→line factor; verified 616–688 vs nameplate 700).



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

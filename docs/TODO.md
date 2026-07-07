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
- [ ] **Velocity loop is untuned for sensorless** (bench 2026-07-07,
  velmode-1/2 on the ZD2808): with the default soft gains a 300 el
  rad/s cruise overshoots 2.6×, oscillates through zero and drops
  trust (restart mid-run). FIXED en route: the PI used to keep
  integrating while the cold-start sequencer owned commutation — the
  rotor sits below the target for the whole ramp, the integrator
  banked 6.6 A by handoff and tripped OC into the confirm probe; the
  loop now holds reset during `is_starting` and the startup drives at
  STARTUP_MIN_DRIVE_A. Cruise-gain tuning (against observer velocity
  dynamics, not hall lag) is the remaining work — it is also the
  right long-term answer to smooth speed-holding on an unloaded
  rotor, where torque mode + speed ceiling necessarily hunts.
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
    no-load ceiling → perturbation scales with L·i.
  - ~~(1) external validity + (2) restart on trust loss~~ LANDED IN SIM
    2026-07-06 (908886f); **both bench blockers cleared 2026-07-06 late
    (d7137e9 ISR tier-2 shave + 9bc8bc3 handoff 60→180 rad/s el, ramp
    0.4→1.2 s keeping ~150 rad/s² el). Bench = HEAD firmware again.**
    **STARTUP CHAIN COMPLETE on hardware 2026-07-06 late-late
    (030c356 + ffa55b8): first probe-CONFIRMED handoff ever** —
    spin-gentle-180: "handoff confirmed by probe (probe_vel=337.6
    observer_vel=187.1)", closed loop engaged on the observer's own
    converged state, λ sane. The investigation en route REFRAMED the
    earlier findings — read this before trusting old attributions:
    - **The "hold-ratchet" was a REAL I/f runaway rotor, not observer
      divergence.** The unloaded rotor slips ahead of the 180 rad/s
      hold field and free-accelerates to 500–1000 rad/s el; the
      observer tracked it correctly the whole time (ready=true, e_q =
      genuine runaway back-EMF). Proven three ways: (a) clean OpenLoop
      discriminators (prof-openloop-ramp180*.json) — the observer
      tracks a real 180 rad/s spin flawlessly at 0.3/0.5 A, ±6 rad/s,
      no ratchet, e_q = λω exactly; (b) probe-raw current windows:
      5.5–7.6 A SETTLED short-circuit current — passive circuit, only
      real back-EMF can drive it — pinning the rotor at 530–800; (c)
      λ-tracker traces. The 2 Hz `obs:` trace (vel/conf/e_q/travel/λ)
      is now permanent — the fast frame only shows the ACTIVE source.
    - **The confirm/deadshort probe MODEL was the actual bug**:
      e = −L·dI/dt is invalid on a low-τ motor (τ = L/R ≈ 0.19 ms ≈ 4
      PWM periods on the ZD2808) — the short current fully settles
      inside the settle window and the probe under-read |ω| by ~ω·τ
      (≈10× at speed), structurally vetoing every correct handoff.
      REWRITTEN to the settled-current model: average i over the probe
      window, e = −(R+jωL)·i, |ω| = |Z|·|i|/λ, angle from the phasor +
      impedance lag + half-window advance + the π flip (i OPPOSES e —
      caught only by the independent-plant sim test; local unit tests
      shared the synth convention, the hall-improvements lesson again).
      Deadshort settle 8→16 periods (settled assumption must hold).
    - Observer fixes that ARE real (030c356): λ tracking now adapts
      only while validity is granted (startup churn dragged λ to the
      λ₀/2 clamp → inflated confidence); e_q normalized by measured
      |x|, not λ (λ error counted twice → validity deadlock with the
      gate). λ converges to ~1.42 mWb at 180 rad/s el in clean spin
      (vs 1.145 stored) — the low-current dead-time residual it
      absorbs; that gap is the remaining distortion-floor lever.
    - ConfirmResult::SeedAndHandoff backstop: 3 consecutive probes
      measuring a real spinning rotor (≥ catch floor) that still fails
      the observer comparison → seed the observer FROM the probe (a
      standing rotor reads ~0 and can never seed).
    - **REMAINING KILLER, cleanly isolated**: post-handoff dq OC while
      closed loop accelerates the unloaded rotor through the ω_e
      400–800 L(f) band — reachable in one 10 s spin-gentle-180 run.
      NEW STRUCTURE (capreport --marks zoom of spin-gentle-180-3,
      confirmed visually): NOT a continuous oscillation — the drive is
      CALM at ~0.5 A between discrete, growing bipolar id/iq spikes
      (−4 → ±10 A over 0.4 s) at ~80 ms cadence, each with an erpm dip;
      the cadence ≈ the MECHANICAL revolution period at that speed
      (12–19 Hz) → suspect a once-per-rev feature (rotor eccentricity /
      magnet weak spot / PLL pole-slip re-lock events), which reframes
      the investigation: find the per-rev trigger and why each ring
      grows, rather than tune continuous loop gains first. λ-tracker
      dynamics, PLL accel lag, eddy-branch plant, dead-time comp
      quality remain the levers; 20 kHz capture of one spike (loss is
      acceptable, the spike is 5–10 ms) is the first data to get.
      Repeatability (spin-gentle-180 ×3 + band-punch + spin-punch-15):
      startup 5/5 confirmed handoffs (incl. one give-up recycle and one
      mid-run trust-loss restart → SECOND confirmed handoff — the whole
      resilience chain works on hardware); closed loop 5/5 dq OC within
      0.25–2.6 s. The historical "fast 1.5 A punch through the band is
      clean" no longer applies: that punch crossed the band INSIDE the
      startup open loop (ramp ceiling era); at handoff-180 the closed
      loop engages at ~200–300 rad/s and crosses the band itself —
      every closed-loop transit rings. No sustained spin on HEAD until
      the spike mechanism is understood.
      MECHANISM CHARACTERIZED (2026-07-06 night, offline spike analysis +
      passive discriminator + FF experiment; captures/punch15-2k-1 =
      loss-free 2 kHz sawtooth artifact, spin-punch-15-1, olramp960-1):
      - NOT once-per-rev (spike intervals don't lock to the mech period;
        ratios 0.15–2.1).
      - The post-handoff failure is an ESTIMATE SAWTOOTH: ω̂ self-
        accelerates away from the rotor at 2–3× the physical torque
        bound (kt·iq/J), collapses at ~850–870 rad/s el, re-locks near
        the true rotor speed, repeats at ~2 Hz; the iq spikes are beat
        pulses (~30 Hz ⇒ slip ~190 rad/s) during the mis-lock, and the
        OC fires when a beat pulse crosses 10 A. The re-lock floors
        trace the TRUE rotor: it crawls (1400→1700 erpm over 1.7 s at
        1.5 A commanded — torque is wasted in beats).
      - The observer is INNOCENT passively: openloop stepped drive to
        960 rad/s el (olramp960) tracks flawlessly (conf ≥0.95, e_q =
        λω exactly, λ stable) — through the whole "L(f) band".
      - The λ·ω̂ decoupling feedforward is NOT the pump (flux FF = 0
        experiment: identical sawtooth).
      - punch-15's SECOND engagement proved the loop CAN work: real
        confirmed 260 rad/s seed → clean physical acceleration to
        3668 rad/s el (e_q 4.8 V real — a phantom at that vq would
        draw 37 A, currents stayed ≤8.5) before an OC at 25k erpm.
      - Corrected J from that climb: ≈3.2e-5 (sim uses 5e-5 — may be
        why sim doesn't reproduce the sawtooth).
      SIM EXCLUSION RESULTS (2026-07-06 night, explore_sawtooth_bench_
      config matrix; VirtualMotor now HAS the eddy L(f) ladder — ψ =
      L_hf·i + ΔL·i_f, τ_e·di_f/dt = i−i_f, per-axis ΔL, gated test
      eddy_branch_drops_inductance_with_frequency): the sawtooth does
      NOT reproduce in sim under (a) the exact bench parameter set
      (obs_l=ctrl_l=24 µH, j=3.2e-5) on the ideal plant, (b) the same +
      eddy plant at τ_e 0.15/0.3/0.6 ms (clean handoff, tracking within
      2%, sustained spin every time), (c) + plant skew (2–3× dead-time,
      λ+15–30%, R+10–20%) — that breaks the RAMP CAPTURE instead
      (phantom class returns, probe correctly refuses, no handoff).
      MECHANISM FOUND (2026-07-06 night, `obs-debug-telem` fast-frame
      mapping — erpm ← PLL ω̂, angle ← phase_pll, vd ← instantaneous
      PLL error; captures/sawtooth-obsdbg-1.parquet, 2 kHz):
      **slip-kick ratchet**. Anatomy of one sawtooth: (a) PLL error
      stays ±0.35 rad the whole time — the PLL faithfully tracks a flux
      VECTOR that genuinely spins 60→890 rad/s (the integrator is the
      problem, not the PLL); (b) iq sits FLAT at the command between
      discrete pole-slip events every ~30 ms (deep −8 A dips as the
      drive frame passes ±π vs the rotor); (c) each slip KICKS the flux
      integrator forward ~+0.3 rad (PLL error = sawtooth of positive
      jumps), sustained mean ≈ +0.25 rad × pll_ki 20000 ≈ the observed
      ω̂ climb rate — slip → kick → ω̂ up → drive faster → next slip
      sooner: positive feedback; (d) at ~870 rad/s the estimate
      collapses (error goes −0.4), re-locks near the true rotor,
      repeats at ~2 Hz; (e) vq mini-sawtooth = the PI integrator
      winding against the phantom back-EMF, reset by every slip.
      The kick source closes with L(f): a slip transient is a
      100–300 Hz event where the true L is ~40–80 µH, but the observer
      subtracts −L·Δi with the AC 24 µH → under-subtraction = net
      forward flux kick per transient. Steady tracking has Δi ≈ 0 —
      which is why the passive olramp960 run is clean and why the sim
      (which never enters a slip) can't self-start the cycle.
      FIX-CANDIDATE RESULTS (2026-07-06 night, second pass):
      - Observer eddy-ladder (ψ_stator = L_hf·i + ΔL·i_f) IMPLEMENTED
        (`BackEmfObserver::with_eddy_ladder`, manager setter, harness
        support) and bench-tested: DOES NOT remove the ratchet (A/B:
        same sawtooth, ceiling slightly lower, kicks slightly bigger).
        The feature stays (physically truer subtraction, all gates
        green) but the g431 override was reverted — it has not earned
        the deviation from the validated bench config.
      - DECISIVE: offline observer replay (scripts/replay_observer.py,
        captures/sawtooth-plain-1.parquet — reconstruct v_αβ/i_αβ from
        a plain-firmware capture and re-run the flux integrator+PLL
        with arbitrary params): R ±26%, ladder on/off produce
        IDENTICAL trajectories ⇒ the ratchet is encoded in the
        recorded v,i themselves — closed-loop self-consistent, NO
        observer-parameter change can fix it. The lever is in the
        loop.
      - PLL gain experiment (kp/ki 1000/20000 → 500/5000, temp
        override, reverted): **first zero-OC closed-loop run ever** —
        full 8.7 s maneuver, no trip, mid-run trust-loss restart +
        second confirmed handoff, system keeps recovering. The
        estimate still oscillates wildly (70–490 rad/s el swings,
        conf 0.7–1.0) — declawed, not cured. Captures:
        sawtooth-ladder-1, sawtooth-plain-1, sawtooth-pll4-1.
      FIX DESIGN, refined (2026-07-07 early, after pre-slip analysis +
      kick-seeded sim matrix):
      - Pre-slip onset (captures sawtooth-{obsdbg,plain}-1): after a
        clean handoff the loop develops a coherent growing angle
        oscillation (~35–80 Hz; PLL error biased +0.3/−0.05, visible in
        vd) over ~50–100 ms while iq stays regulated flat — a LINEAR
        mid-band instability whose nonlinear end state is the ratchet.
      - Sim CANNOT host the ratchet even seeded: a direct 1 rad
        observer phase kick on the eddy plant recovers cleanly (final
        2825, no trip) — the bench's linear instability is missing
        physics deeper than first-order eddy + ideal-comp dead-time
        (candidates: higher-order L(f), saturation dynamics, sense
        transfer function). Sim-first fix validation is OFF the table;
        bench-first with replay + obs-debug-telem instrumentation.
      - Global PLL ki reduction is an analgesic, not a fix: ki/4 on
        the HEALTHY sim loop degrades it (final 672, untrusted 0.19)
        while on the bench it merely declaws the ratchet (messy spin).
      - THE candidate: **slip-gated PLL error** — full gains always,
        but while a slip transient is detected (|iq_ref − iq_meas|
        above a threshold, or iq sign dip while driving) the PLL
        velocity integration is held (dead-reckon the angle). Subtle
        point from the replay: gating the PLL is sufficient even
        though the passive flux vector x ratchets in the recorded
        data — in the CLOSED loop the commutation (and hence v) stops
        accelerating, which is what feeds x. Design questions: freeze
        scope (ki only vs kp+ki), threshold/hysteresis, interplay with
        e_q validity accrual while gated, and NOT gating during the
        deadshort/confirm shorts (observer must keep integrating real
        short currents there — or should it? the probe windows are
        also giant current transients feeding the integrator; revisit
        together).
      - Also on the table for the onset itself: the linear mid-band
        mode (grows BEFORE any slip) — slip-gating stops the ratchet
        but may leave a bounded oscillation; damping the onset likely
        needs the true L(f)/loop-phase story (impedance-sweep TODO is
        the measurement).
      **ACCEL PRIOR LANDED (2026-07-07): first zero-OC maneuvers with a
      genuinely spinning rotor.** The slow-phantom escape (currents
      perfectly regulated, PLL err small-but-positive, ω̂ growing 3× the
      torque bound) is caught by the one physical fact it can't fake:
      |ω̂| growth is clamped to an envelope slewing at
      `floor + per_amp·|iq|` el/s² (`BackEmfObserver::set_accel_prior`;
      envelope, not per-step Δv — a per-step clamp asymmetrically clips
      PLL ringing and biases legitimate tracking; deceleration and
      magnitude-shrinking corrections stay free so load braking is
      never fought; seeds carry their own envelope). g431 interim
      numbers (J not in config yet): floor 500, per_amp 3400 =
      1.3·1.5·pp²·λ/J with J≈3.2e-5. Driver feeds |iq_meas| (τ=10 ms).
      Bench: spin-gentle-180 AND spin-punch-15 both ran their FULL
      maneuvers with ZERO OC and a real spinning rotor — bounded
      oscillating spin 150–600 rad/s el, e_q corroborating throughout,
      self-recovering through dips (captures/prior-1,2). RAM note:
      OUT_QUEUE 3328→3264 (observer state grew; .bss was 20 B over).
      REMAINING QUALITY WORK (no longer trip-class): (a) the bounded
      mid-band oscillation itself (~±40% around 450) — the linear onset
      story (L(f)/loop phase; impedance-sweep) — CONFIRMED as the
      drive-chases-estimate mechanism by freq-led trial 2 below (the
      filter kills it outright); (b–d) CLOSED 2026-07-07 PM
      (churn-hygiene commits 50e193c + d96cc9f): DEADSHORT_MIN_CATCH_VEL
      60→140 (rideability bound; the confirm probe's strong-probe floor
      stays 60 as CONFIRM_STRONG_PROBE_VEL so the hold-ratchet escape
      keeps working; a probe window ended by the 15 A cap bypasses the
      floor — capped = proven-fast rotor, and ramping into one is the
      worse failure); λ learning now needs EARNED corroboration travel
      (2 revs since last seed/reset, `lambda_learn_travel` — seeds grant
      torque validity but not tracker credit; bench: λ stays
      1.15–1.30 mWb through restarts, no more λ₀/2 walks); and the real
      0.3 A-cruise killer found and fixed — a mid-cruise restart
      INHERITS the weak command, so the ramp ran at |i|≈0.1–0.3 A and
      capture was luck (hold watched a standing rotor at conf 0.95,
      give-up recycle churn). STARTUP_MIN_DRIVE_A=1.5 floors the drive
      while the sequencer owns commutation (sign kept, soft-start
      still applies, bounded by max_current_a; sim gate
      weak_command_cold_start_captures_via_floor). Bench before/after
      on spin-punch-15: 3 starts / 2 confirms / 1 give-up-recycle →
      3/3/0 with ramp-exit |i| 1.9–2.4 A; spin-gentle-180 also 3/3/0.
      Startup ISR logging is now behind a 10-frames/s token bucket
      (SensorlessStartup::log_tick/log_allow, dropped frames counted
      and summarized once per window) — the churn defmt storms
      (26k-cycle single ISRs, 126–138% load) are structurally capped.
      **FREQ-LED COMMUTATION: VALIDATED ON THE BENCH, parked pending an
      ISR tier-3 shave (2026-07-07 PM, trial 2).**
      `PhaseManager::set_freq_led(rate, k_theta)`: the drive follows a
      slew-limited commutation frequency (open-loop stiffness) with a
      slow phase pull to the observer; seeds continuity at handoff; sim
      gate cold_start_with_freq_led_commutation_spins_up. Trial 2
      (rate 5000 / k_theta 30, on top of the churn-hygiene fixes) was
      the FIRST GENUINELY SMOOTH sensorless run: ONE cold start, ONE
      confirmed handoff, monotone climb 0→24.9k erpm el over 5.6 s
      (~480 rad/s² el at 1.5 A, still climbing at 0.3 A), zero
      restarts, λ stable, e_q tracking λ·ω̂ the whole way
      (captures/freqled2-punch-1) — the mid-band limit cycle and the
      user-visible jerk are GONE with the filter in. What ended it at
      t=6.1 s is the ISR BUDGET, not control: the freq-led branch costs
      ~+600 cy in EST_OUT (300→930 — grossly more than its ~50 cy of
      arithmetic; needs the PC-profiling pass), on top of a drive
      baseline that CREPT 6190→~7600 avg since the tier-2 shave (the
      estimator hardening: slip gate, accel prior, validity/e_q,
      settled normalization). Loop at 96–103%, over=1200/s,
      device-executor stalls 20–144 ms, then a fault (iq spike ~7.6 A
      in the 1 kHz capture, likely dq OC under control jitter) latched
      Error — and its defmt frame was LOST in the storm (the t=8 ack
      returned `state: Error, fault_count: 1`; RTT under saturation
      drops frames — don't trust grep-for-fault alone, check the ack
      states). Also noticed: both floor-* runs show an identical
      deterministic 6240-sample (312 ms) telemetry gap — same family as
      the ~410 ms host quantum; telemetry-only, control unaffected.
      **ISR TIER-3 SHAVE DONE + FREQ-LED ENABLED BY DEFAULT
      (2026-07-07 PM, fbd45b6).** Root cause of the budget creep: the
      tier-2 profile fix never landed — the oxifoc-core override in
      g431/Cargo.toml SAID "speed-optimize" but carried opt-level="z"
      (no-op), so the whole hot path ran machine-outlined (PC sampling:
      OUTLINED_FUNCTION_*, f32::max/clamp_f32/wrap_angle as CALLS,
      memcpy struct returns, LDREX/STREX). Fixes (each bench-measured):
      core override z→2 (+876 B), build-std libcore z→2 (+248 B — even
      f32::max was an outlined call from libcore-z), and the `isr-speed`
      core feature = nightly #[optimize(speed)] on the hot GENERIC entry
      points (+720 B) — generics monomorphize into the size-optimized
      device crate where package overrides can't reach; whole-crate "s"
      overflows the 128 K part by 23 KB. Trace decimation counters:
      static AtomicU32 → plain fields. Repo now pins ONE nightly via
      root rust-toolchain.toml (host workspace was on stable and
      couldn't build the feature gate). NUMBERS: openloop 6563→5756
      (below tier-2's 6190 with the whole estimator arc on top);
      closed-loop drive 7500→6350 (73–75%, over 300–500/s→2–4/s);
      drive+freq-led 8140→6840 (79–80%, ZERO executor stalls, pump
      stale_max ~51 ms vs 100–175). Flash 124 272 / 131 072 (headroom
      6.8 K). Freq-led (5000/30) is ON in the g431 default build.
      **HIGH-SPEED OC ROOT-CAUSED AS STRUCTURAL (2026-07-07 evening,
      4026eed).** Instrumented via the new obs-debug vd ← Δ(drive,
      phase_pll) mapping + a freq-led-off control run: with freq-led the
      frame-vs-estimate gap locks at −90°±1° within 100 ms of handoff
      regardless of seed (plain commutation centers at 0). A
      frequency-led frame is an I/f drive — the rotor flux locks onto
      the commanded current vector, so the flux estimate ALWAYS sits
      ~90°−load off the frame BY CONSTRUCTION; no frame-side pull can
      close it (the plant re-establishes it — verified: P pull, LPF
      pull (= negative damping, OC moved DOWN to ~1000), integral pull
      (wobble-entrains the slew limiter), speed-fade blends (slams the
      standing angle) all failed exactly as this model predicts).
      Torque-per-amp is therefore set by the load angle, the effective
      torque ceiling crosses the bench drag curve at ~28 k erpm el
      (observer-frame commutation reached the ~50 k vbus ceiling), and
      the hunting mode rides a flat torque slope. WHAT LANDED: hunting
      damper (LPF'd 0.5×slew-crushed velocity error into the angle
      advance — passes the 8–20 Hz hunt band, rejects the 35–100 Hz
      wobble band) + freq-led seeds angle from the OBSERVER / frequency
      from the last output. THE REDESIGN (next session): commutate
      torque in the OBSERVER frame (torque axis = estimate + 90°, like
      the plain path) and use freq-led ONLY as a slew limiter on that
      frame's rotation RATE — frequency smoothing without surrendering
      the torque axis to the I/f geometry. obs-debug caveat now
      documented in code: host enrichment derives id/iq from the
      recorded angle column, so obs-debug captures carry id/iq in the
      PLL frame.
      **PHASE-TRACKER REDESIGN LANDED (2026-07-08, f803fd5): OC class
      eliminated, smoothness open.** freq-led replaced by a 2nd-order
      phase tracker on the observer angle (set_phase_tracker(ωn, ζ)) +
      hard load-angle clamp (0.6 rad — the flat-top ride is now
      geometrically impossible) + acceleration feedforward (τ 30 ms
      trend of the estimate velocity — kills the type-2 lag that
      otherwise self-settles at ~77° under punch accelerations for any
      wobble-filtering ωn). Bench: ZERO Overcurrent in every config
      (wn 60 / 30 / +clamp / +ff) vs the deterministic 2533–2950 OC of
      freq-led; torque axis restored (climb rate ~3× freq-led's at the
      same current). NOT recovered: smoothness — the mid-band limit
      cycle + trust-loss restarts are back (3–5 starts/run, 40–600
      rad/s swings), same as plain commutation. KEY NEGATIVE RESULT:
      the tracker's angle-wobble transmission is analytically tiny
      (×0.03–0.12 at 60 Hz) yet the cycle returns ⇒ the mid-band
      instability does NOT propagate mainly through the commutation
      angle — freq-led suppressed it by fully DECOUPLING drive from
      estimate (I/f), not by filtering. NEXT SESSION: offline tracker
      replay against the recorded estimate traces (obs-debug captures
      exist) + identify the real loop carrier (velocity output into
      decoupling FF / actuation advance? torque ripple? estimate
      self-excitation?) before any more bench darts. Canonical bench
      firmware = tracker wn=30 ζ=1.2: oscillating but self-recovering,
      zero faults, PSU-safe.
      **MID-BAND CARRIER ELIMINATION CAMPAIGN (2026-07-08, 86fcf7e):
      the oscillation is the ESTIMATE ITSELF.** 2 kHz instrumented
      spectra (trk-dbg-2k-1..gov-2k-1): Δ(frame,pll) wobbles broadband
      30–90 Hz ±45–75°, iq beats 5–25 Hz, visible jerk = 1–6 Hz ω̂
      envelope ±50–90 rad/s. Interventions bench-tested and ELIMINATED
      as carriers: commutation smoothing (tracker filters, cycle
      persists), frame hunt-damping (kept, no change), validity
      relaxation oscillator (knobs softened 0.25/25 ms/0.4 s — kept,
      no change), accel-prior rectifier (REAL defect — the envelope
      chased wobble troughs and clipped recoveries; redesigned to a
      50 ms trend + proportional governor, contract: sustained phantom
      ≤ 2× envelope rate; but prior-OFF rode the same oscillation ⇒
      contributor, not carrier). CONCLUSION: the carrier is estimate
      quality in the L(f)/dead-time band — the long-standing "linear
      onset story". NEXT SESSION = attack the SOURCE: impedance-sweep
      characterization of the 400–1000 rad/s band, dead-time comp
      quality at low modulation (λ tracker converging to 1.42 vs
      1.145 mWb is the same lever), properly parameterized observer
      eddy ladder (infra exists, was tried with guessed params only).
      Note the prior-OFF run's "clean escape" climbed tight-Δ to 2.5k+
      and OC'd — possibly the genuine slow-phantom class; do not read
      single clean runs as cures. Bench canon: tracker 30/1.2 + all
      protections, 4/4 confirms, zero OC/CommTimeout, self-recovering.
      **L(f) MEASURED + LADDER PARAMETERIZED (2026-07-08, 24d3dc8).**
      The impedance sweep (regridded 100–1680 Hz, 0.8 V floor — at
      0.2 V the carrier sat below the dead-time distortion and returned
      |Z| < R_DC) measured the real ZD2808 d-axis L(f): 105 µH@100 Hz
      → 30@350 → 16 plateau; single-pole fit L_hf 15 µH / ΔL 155 µH /
      τ 1.39 ms — the guessed ladder's corner (0.3 ms) was 5× too high
      and over-compensated the slip band 3×. g431 observer now runs
      set_observer_eddy_ladder(146e-6, 1.4e-3). Post-change bench:
      bounded band oscillation, 2 starts / 20 s (was constant churn),
      zero OC, e_q corroborated. **SHARPEST OPEN QUESTION: torque
      non-delivery during the band oscillation** — endurance-hold at
      1.5 A commanded rides 230–470 rad/s el for 20 s with measured iq
      averaging only 0.2–0.4 A (beats σ 0.4–0.8): the climb-through
      energy is being lost somewhere between the command and the
      rotor. Candidates: PI vs beat dynamics, slip-gate freeze duty,
      frame-vs-rotor geometry during swings (mind: obs-debug id/iq are
      PLL-frame — recheck on a PLAIN build), voltage saturation windows.
      Answer this FIRST next session — it is measurable with existing
      telemetry and likely IS the band-transit blocker.
      **THE L(f) THEORY IS DEAD — the winding is flat; the "eddy curve"
      was ROTOR MOTION (2026-07-08 evening, pulsating-carrier sweep).**
      The rotating HFI carrier torque-ripples the rotor even under a
      perfect d lock; below ~500 Hz the rotor's motional EMF masquerades
      as inductance. A torque-free PULSATING d-axis carrier (same lock,
      same demod) measures the true winding: **L(d) ≈ 17 µH FLAT from
      100 Hz to 1.68 kHz**, converging with the rotating variant only at
      900+ Hz (inertia locks the rotor) — and the bench LCR (clamped
      rotor) shows the same flat plateau to 100 kHz, while a FREE-rotor
      LCR at 100 Hz collapses to 7–90 µH depending on the rotor angle
      (torque-coupled angles → motional cancellation). Consequences:
      (1) baked eddy ladder → OFF (yesterday's ΔL 146 µH/τ 1.39 ms was
      rotor swing, not eddy); (2) the slip-kick ratchet's L(f)
      explanation ("100–300 Hz events where L = 40–80 µH") is
      unfounded — the ratchet is empirically real (slip gate fixed it)
      but its mechanism must be re-derived, with ROTOR MECHANICS
      (swing/hunting dynamics the observer's electrical model does not
      contain) as the prime suspect — note the free-rotor mechanical
      resonance is ~10–20 Hz, right in the measured 1–6 Hz…25 Hz beat
      bands; (3) the voltage-pulse "fundamental" Ld/Lq (86/129 µH) is
      now provenance-suspect for the same reason (the rotor moves
      during pulse trains) — yet decoupling with those values
      empirically removed the 800 rad/s OC; whether it worked for the
      stated reason needs re-examination; the small-signal winding
      saliency IS real (hand-displaced-rotor sweep read q-mixtures
      48–87 µH vs pure-d 17). (4) Clamp lesson: a mechanical clamp
      fights the electrical lock (displaced angle = q-mix + renewed
      torque coupling) — on-device controls must be electrical
      (pulsating carrier), mechanical clamps only for meter
      measurements without a lock.
      **TORQUE NON-DELIVERY ANSWERED + THE MID-BAND "BAND OSCILLATION"
      ROOT-CAUSED: it was the TRUST GATE all along (2026-07-07,
      captures/gate-dbg-1 … gate-fix-9, plain-fix-1..3).** Forensics
      chain, each step bench-verified:
      (1) *Conviction.* obs-debug now maps vq ← readiness err signed by
      `angle_trustworthy` (4th debug slot). gate-dbg-1 @1.5 A hold:
      P(gate closed | current collapsed) = **0.96**, gate closed **54%**
      of the hold, every trust drop at err ≈ 0.200 exactly — and inside
      chop windows the commanded |v| collapses to 2.6% of available
      (loop obeying a zeroed reference — the signature of the gate, not
      of saturation). The iq trust gate (foc_driver, the
      `angle_trustworthy` backstop) was the chopper.
      (2) *Mechanism.* A type-2 PLL under constant acceleration carries
      the STRUCTURAL steady-state error α/ki. At ki = 20 000 the strict
      0.2-rad readiness bound caps the drivable acceleration at
      0.2·ki = 4 000 el rad/s² ≈ 0.8 A worth of torque on this free
      rotor — the measured free spin-up is **9 300 el rad/s² at
      ~1.15 A delivered** (~8 100 /A). Any honest 1.5 A acceleration
      MUST trip the gate: torque cut → decel → err falls → torque on →
      a 4–7 Hz relaxation limit cycle (gate-dbg-1: erpm sawing
      2 000↔4 800, exactly the historical "band oscillation" /
      slip-kick "kicks"). The 2–4.8 k erpm "cruise" of every earlier
      1.5 A run was never an equilibrium — it was the gate bang-bang
      governing speed by accident. The mid-band CARRIER hunt is CLOSED:
      there was no carrier; commutation smoothing / frame damping /
      validity / prior-rectifier were bystanders (matches every
      elimination A/B). The rotor-mechanics suspicion for slip-kick is
      also closed by this (bang-bang torque at few Hz IS the kick).
      (3) *Fixes (landed, tests
      ready_lock_quality_is_a_schmitt_trigger,
      readiness_excuses_pll_acceleration_lag_but_not_divergence).*
      Readiness lag compensation: err_eff = max(0, phase_err_filt −
      min(|α̂|/ki, 0.4)), α̂ = 20 ms LPF of the post-envelope PLL
      velocity slope, cached once per update (the gate consults it ~6×
      per cycle; per-call divisions cost real ISR budget — see below).
      Schmitt latch: acquire < 0.2 strict (handoff quality unchanged),
      hold < 0.6 (diverged PLL ~π/2 still trips through the capped
      excuse: 0.4 + 0.6 = 1.0 < π/2). Accel prior corrected 3 400 →
      10 500 /A (the old J came from a 2026-07-06 climb that was
      gate-throttled — J read ~3× high, and the envelope governor then
      FOUGHT honest spin-ups; the estimate was dragged below the rotor
      mid-acceleration and the current loop lost the frame at ~6 k erpm
      → OC, gate-fix-3).
      (4) *What the un-gated system exposed (each stepped on in turn).*
      NOTHING bounds speed in torque mode on an unloaded rotor: the
      free spin ran past 19 k erpm in 150 ms of handoff — with e_q
      corroborating and λ adapting: it was REAL rotation, the observer
      tracked it fine — until the current loop (ωc = 1 000 rad/s)
      lost regulation ~2 000 el rad/s → OC. The chattering gate had
      been the accidental speed governor; the intended mechanism
      (derating speed ceiling, default OFF) is now baked: 400 el rad/s
      + rolloff from 0.5×, sized for the ISR wall (below), and the
      speed-cut term is evaluated PER CYCLE in the iq clamp
      (DeratingConfig::speed_cut) — the 78 Hz decimated scale
      bang-banged ±2.3 k erpm around the ceiling at 9 300 el rad/s²
      (plain-fix-2 sawtooth 2.6 Hz). bus_in_max baked 10 → 3.5 A
      (PSU CC is 4 A; the gate had been hiding the real bus draw too).
      (5) *Validation (plain-fix-2/3, canonical plain build).* Full
      maneuver, zero OC, zero deadman, zero restarts, single confirmed
      handoff. **Below the ceiling the torque question is closed: iq
      median 1.44 of 1.5 commanded, |i| < 0.5 A only 1% of the climb**
      (was: mean 0.2–0.4 A of 1.5, 22% chopped). 0.3 A cruise: erpm
      3 990 ± 570, 2% micro-chops. The "band oscillation" as a
      phenomenon is gone from the telemetry.
      OPEN (new, ranked):
      - [x] **~~ISR speed wall~~ SOLVED (2026-07-07 profiler session):
        it was flash prefetch, not speed.** The openloop 30→960 el
        rad/s staircase showed the est-chain cost FLAT with speed
        (saturates by ~120 el rad/s: drive-state cost, tracker +
        full estimator active). The deadman cascades were stall
        pileups: FLASH_ACR read back 0x00040604 — 4 WS, I/D caches
        on, **PRFTEN OFF** (embassy G4 RCC never enables it) — CPI
        ≈ 2.5 on the branchy hot path (tracker: ~200 instructions,
        542 cy, measured via the new OUT_TRK/OBS_TAIL buckets). One
        bit in init_clock: drive avg 7 075 → 6 400 cy openloop,
        closed loop 91–95% → 82–83%, worst single pass 20 k → 7.5 k,
        overruns → 0, first loss-free 2 kHz closed-loop captures.
        Speed ceiling re-raised 400 → 800 el rad/s and validated
        (ceil800-1: full punch maneuver at 7–10 k erpm, zero
        gaps/faults, 82% flat; 0.3 A cruise ±6%). REMAINING shave
        candidates (not blockers): observer update ≈ 1 400 cy (divs
        in e_q normalization etc.), tracker per-cycle `dt/τ`
        divisions (alphas are constants for fixed dt — precompute),
        EST_OUT dispatch glue ≈ 450 cy, occasional 20–26 k passes
        during command processing (~5–15/s, thread-visible only).
        Also noted: at 600+ el rad/s OPENLOOP (0.5 A passive), the
        observer dropped readiness mid-staircase in one run — likely
        a real pole-slip of the lightly-driven rotor, but watch it.
      - [ ] **Ceiling hunt**: even with the per-cycle cut, 1.5 A hunts
        3.2↔7.2 k erpm at ~3 Hz around the 3.8 k ceiling (governor
        gain/lag vs free-rotor momentum + drag asymmetry). Consider a
        proper speed loop or a wider taper; harmless (bounded, no
        faults) but ugly and it visits the ISR wall.
      - [ ] **Ramp slip-ratchet + one-sided confirm**: the rotor runs
        AHEAD of the 90 rad/s I/f ramp (probe reads 520–580 el rad/s at
        the handoff gates — consistent across every run), the observer
        under-reads (~195, envelope-capped pre-handoff), and
        feed_confirm's vel_ok is one-sided (probe ≥ 0.5×claim: a probe
        3× ABOVE the claim "confirms"). The handoff works because the
        observer catches up to reality post-handoff, but the ramp is
        not actually dragging this rotor at 1.5 A — it is kicking it
        forward pole by pole. Revisit ramp current/rate and make
        confirm two-sided (|probe − claim| band) once the ceiling work
        settles.
      - [ ] **Tracker rides its 0.6 hard clamp through hard
        acceleration** (gap pinned 0.62–0.67 the whole climb =
        commutating ~35° behind the estimate; torque factor cos ≈ 0.83
        and a standing offset the current loop must absorb). Feed the
        observer accel into the tracker ff, or raise ωn during
        confirmed acceleration.
      - [ ] OUT_QUEUE = 3072 is a hard FLOOR (three ~1 019 B COBS
        grants; a 3 056 probe gapped telemetry from mid-ramp). RAM for
        new state must come from elsewhere (this round: ergot-down RTT
        512 → 496).
      **The original frontier note (for context) — deterministic dq OC at ~2950 rad/s el** (~28 k
      erpm, |v| 3.6 of 6.9 V available, iq beat envelope growing with
      speed, clean fault frame in log, PSU-safe held): both canonical
      maneuvers now do one start → one confirmed handoff → smooth
      monotone climb → OC at the same speed. Pre-freq-led runs never
      reached this band (they oscillated at 2–6 k erpm); the morning
      align-era build DID reach the ~50 k vbus ceiling — so something
      in the modern arc (freq-led k_theta lag at speed? actuation
      advance? slip-gate never engaging at 0.3 A? dead-time at high
      modulation?) degrades commutation above ~25 k. Also unexplained:
      freq-led's EST_OUT cost is +450 cy over the plain observer arm
      and GROWS with speed (582→731 across the climb) — far above its
      ~50 cy of arithmetic even un-outlined; profile at speed.
      **DEADMAN FALSE-POSITIVE CLASS root-caused and fixed
      (2026-07-07)**: the host's single ordered command/affirm task
      goes silent for a DETERMINISTIC ~410 ms while an acked command's
      response round trip stalls (three runs measured stale_max
      410.15–410.19 ms — a host retry quantum, find and fix it
      host-side eventually), and the post-handoff second runs the ISR
      hottest (bursts starve the pump further). Fixes:
      STARTUP_STALENESS 400→700 ms, the handoff edge now RE-OPENS the
      engage grace window (not just refreshes the stamp), bench baked
      staleness 400→800 ms (riding default stays 150 ms). Validation:
      2/2 canonical runs with zero OC and zero CommTimeout, confirmed
      handoffs incl. mid-run restart re-confirms.
      **SLIP GATE LANDED (2026-07-07): strict improvement, not yet a
      cure.** Implementation: driver flags cycles with |iq_ref −
      iq_meas| > max(1.5 A, 0.5·|iq_ref|) (hysteresis ×0.5); while
      flagged, the observer's PLL holds — dead-reckoned angle, no
      velocity/λ/error-filter/validity learning; flux integrator keeps
      running; 30 ms duty limit against a latched gate (V-SAT class).
      Unit gates: slip_gate_holds_pll_and_dead_reckons,
      slip_gate_duty_limit_force_opens; full suite green (the relative
      threshold part exists because an absolute 1.5 A froze legitimate
      6 A-start transients and broke the classic cold-start sim test).
      Bench (captures/slipgate-1,2): the estimate sawtooth is GONE —
      constant 1.5 A ran 2.4 s of bounded oscillation (erpm 3109±1232,
      BEMF-OK) instead of ratcheting to OC in 0.25 s; at 0.5 A the loop
      did its first genuinely PHYSICAL acceleration through the band
      (137→546 rad/s el over 2.3 s at the 0.5 A torque rate, e_q
      corroborating). REMAINING: (a) an estimate ESCAPE still ends the
      runs at ~550 rad/s el (546→1978 in <0.4 s — 3× the physical
      bound; anatomy of the escape from the 2 kHz captures is the next
      analysis: is the gate not engaging — beat dips at light command
      below the 1.5 A floor? — or is it the linear onset growing
      un-slipped); (b) the torque DOWN-step (1.5→0.3 A) still
      destabilizes within 0.2 s (reproducer #2's class, now isolated
      on top of the gate). Bench firmware note: plain HEAD reflashed.
      Bench capture rates: 2 kHz loss-free, 10 kHz is not (366 ms
      gaps).
    Distortion-floor context for the record: at ramp 60–63 the observer
    read 32–62 with confidence DECAYING 1.0→0.59 and validity never
    corroborating — λω ≈ 72 mV is at/below the post-comp dead-time
    residual (retroactively taints the align-era "handoffs at
    60.0–60.2" as likely distortion-locks). At 180 rad/s λω ≈ 0.21 V
    clears the residual.
    What landed, and what the sim taught en route:
    - Passive checks are STRUCTURALLY insufficient alone: a steady
      phantom (observer tracking the machine's own residual-distortion
      flux on a standing rotor) pins its PLL exactly where |e|/λ = ω̂,
      so `confidence ≈ 1` **implies** back-EMF-proxy ratio ≈ 1 — no
      terminal-voltage criterion can reject it. The passive layer still
      catches the TRANSIENT class (catch-swing 231-vs-ramp-23 false
      ready) and the post-collapse deadlock (vq ≈ 0.01 V vs λω̂ →
      ratio ≈ 0.02): `BackEmfObserver::is_ready` now also requires the
      e_q proxy (v − R·i projected on the flux-quadrature axis) to
      corroborate λ·ω̂ for 2 consecutive electrical revolutions;
      granted validity is sticky (0.2 s sustained violation to revoke —
      instant revocation turned the iq trust gate into a torque chopper
      and destabilized an otherwise-clean spin).
    - The decisive external check is PHYSICAL: a deadshort **confirm
      probe** at every ramp/hold handoff (settle 32 + probe 8 PWM
      periods, e = −L·dI/dt must agree with the observer in |ω| (≥0.5×)
      and angle (≤1 rad)). One probe instant can honestly read ~0 on a
      genuinely captured rotor (capture HUNT swings ±40 rad/s), so an
      unconfirmed probe returns to Hold and retries every 0.1 s; a hold
      that cannot confirm within 1 s recycles the whole start
      (deadshort→ramp) with an observer reset — this breaks phantoms
      AND re-catches runaway rotors by measurement. Driver resets the
      current-loop PI on every probe→commutation exit (stale-integrator
      spike otherwise).
    - Restart on trust loss (fix 2): drive mode + nonzero command +
      angle untrustworthy for 0.5 s continuous → re-enter cold start
      (`SENSORLESS_RESTART_AFTER_S`, foc_driver). The deadshort probe
      makes it safe both ways. Sim gates:
      `phantom_handoff_blocked_by_confirm_probe`,
      `cold_start_recovers_through_dead_time_distortion`,
      `trust_loss_mid_drive_restarts_cold_start`, confirm-probe unit
      tests in startup.rs.
    - Sim findings for the REMAINING limit-cycle work: (a) reproducer
      4's phantom is reproducible on the single-L plant with bench-level
      dead-time residual (`run_zd2808_cold_start` harness,
      `explore_phantom_handoff` ignored test = the exploration matrix);
      the residual is the phantom's energy source — better low-current
      dead-time comp would shrink the whole class. (b) A limit cycle
      with reproducer-3's exact signature (speed swinging hundreds of
      rad/s, iq slamming ±limits, observer≠rotor) appears in an IDEAL
      plant whenever ω̇_el exceeds what the observer PLL can track
      (steady lag = ω̇/pll_ki) — the old
      `sensorless_cold_start_ramps_then_hands_off` plant (j = 1e-4,
      6 A → lag 2.2 rad) was doing exactly this and passing by
      coincidence of its end-sample; PLL gain scheduling / acceleration
      feedforward is a concrete lever for reproducer 3. (c) Sim-harness
      trap now documented in the harness: drive the plant from DUTIES,
      not `FocOutput.v_alpha/v_beta` — the dead-time comp lives only in
      the duty path, feeding the pre-modulation voltage silently
      disables it.
    - Note the no-align tradeoff stands: handoff observer_vel went 60
      (align era) → 113–231 (catch transient excites the observer
      harder); resonance-OC data still justifies no-align.
  - FOUR deterministic reproducers (2026-07-06 evening, cold motor,
    captures on disk):
    1. SLOW crawl through the ω_e ≈ 400–800 L(f) band → dq OC ~1.1 s
       after handoff, 6/6 — maneuvers/prof-hold-1a.json (fast 1.5 A
       punch through the same band is clean → band-transit dynamics,
       not steady current; captures/final3*).
    2. Torque DOWN-step at the voltage ceiling (1.5→0.5 A, t=4.0 of
       endurance-20s.json) → OC in ~0.4 s, 6/6 across two series
       (early steps, before the ceiling, are harmless — why spin-gentle
       always worked; captures/endurance4-*).
    3. Constant iq*=1.5 A sustained: the limit cycle over 17 s —
       speed 518–8736 erpm (median 0.5 s swing 7126 erpm!), iq ±9 A
       around a COLLAPSED 0.83 A mean, vq ≈ 0.57 V (never near the
       ceiling). Protection-clean but visibly jerky rotation
       (endurance-hold.json 4/4; captures/endurance-hold-*). The
       "bounded" short-run view undersold it.
    4. current-staircase.json (5 s at 0.3/0.5/0.8/1.2/1.5 A): PHANTOM
       handoff (observer 231 at ramp 23) → estimator collapse → the
       untrusted-angle iq gate zeros torque → trust-loss DEADLOCK:
       15 s of iq_mean 0.00, observer frozen at 644 erpm, vq≈0.01 V
       (rotor truly stopped), audible whine on the 1.2/1.5 A legs as
       the gate flickers (captures/staircase-1.parquet). Sim: the current
    loop with a perfect angle is unconditionally stable (any advance),
    so the cycle lives in the OBSERVER↔loop interaction; the single-L
    sim plant does not reproduce it (frequency-dependent L missing).
  - Investigation plan (remaining after the external-validity work
    above): λ-tracker dynamics under flux-vector wobble; PLL
    acceleration-lag lever (gain scheduling / accel feedforward — see
    sim finding (b) above); eddy-branch plant model (parallel R-L) for
    the L(f) band; low-current dead-time comp quality (the phantom's
    energy source, finding (a)).
  - ~~Two-inductance model~~ CLOSED 2026-07-08 (1c4789a): the config
    now carries the full inductance MODEL — inductance_d/q_h keep the
    AC/plateau semantics, new ld/lq_fundamental_h feed the decoupling
    (from_runtime_config picks them when present), new
    eddy_delta_l_h/eddy_tau_s parameterize the observer ladder; both
    g431 board-file hardcodes deleted, ZD2808 numbers baked with
    provenance. REMAINING (future): detection should fill the new
    fields (sweep is still an experiment feature; natural with G474
    persistence); deadshort/confirm probes still use the flat AC L —
    at catch speeds (20–50 Hz) the real L(ω) is ~150 µH, giving a
    ~10° φ_z bias in the catch-seed angle: evaluate L(ω) from the
    ladder inside short_current_estimate (needs bench validation).
  - ~~Align swing~~ RESOLVED 2026-07-06 (ccfb233): the align phase is
    GONE — VESC-style ramp-from-unknown-angle with the current
    soft-start folded into early ramp. Bench: align-OC eliminated,
    handoff 9/9. Sim gate: cold_start_captures_rotor_from_any_initial_
    angle. Observer readiness (external validity) remains open — the
    catch transient at ramp entry can still briefly swing the rotor,
    which is what the 35% runaway gate covers.
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
- [ ] **TIM4 timebase glitches backward under ISR saturation** (found
  2026-07-06 via a false deadman at engage+70 ms = one 65.5 ms TIM4
  period, staleness meter pegged at u32::MAX): `hall::now_ticks()`
  assembles TIM4 CNT + a software overflow count; when the saturated
  FOC ISR delays the TIM4 overflow interrupt past a wrap, the assembled
  time steps BACK ~65 ms. The deadman now uses saturating_sub (ccfb233)
  so it merely loses that window, but hall-edge timestamps and velocity
  math on hall builds see the same glitch — audit `CaptureTimebase`
  (read CNT twice / pending-UIF handling) or move the timebase off the
  starving interrupt.
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
- [x] **ISR tier 2 — DONE 2026-07-06 late (d7137e9)** — the 908886f
  blocker is cleared. Measured on-target (prof-openloop.json = new
  clean-steady-seconds gate, 0.3 A OpenLoop @ 1 kHz stream, marks ON):
  **7670 → 6190 cy avg (90% → 72%)**; Stopped + 1 kHz 3960 → ~2500;
  Stopped + 20 kHz stream 4530 → 3852 (45%, prof-20k-idle loss-free);
  **startup path 223% (over=31606/s, thread starved, RTT death) →
  87% (over=30–70/s)** — the command pump stays alive through repeated
  hold-give-up recycles, no CommTimeout, maneuvers complete. What it
  was (all measured, PC-sample + marks): FaultRegistry CS+RefCell+Vec
  scans 3–5×/cycle → atomic summary bitmask; ADC handle locks → direct
  JDR reads (372→132); 4 CS entries in run_foc_cycle → 1; 9 vdiv/cycle
  in adc_to_current → cached amps_per_count; embassy set_duty ARR
  re-reads → direct CCR writes; fmodf in step_open_loop → wrap_angle;
  hall snapshot skipped on SENSORLESS build; deadman u64 math ×2 → ×1.
  Remaining fat if a tier 3 is ever needed: obs mark ~1250 (the single
  biggest block; PC profile says the observer's own math is ~5%, the
  rest is attributed to inlined helpers + flash-WS stalls — decimating
  the e_q validity block ×4 is the next candidate), pub=345,
  state-mirror copies ~2%, ramfunc idea below.
  Original tranche-1 notes (2026-07-06 evening; numbers superseded):
  - DONE: remquof/fmodf purged (`wrap_angle` %, `angle_difference`
    remainderf, `fast_sin` %TAU ×2 per sin_cos → branch+subtract, cold
    fallback); hall/encoder sampling gated by `requires_*` (est
    remainder 390→289); `[profile.release.package.oxifoc-core]
    opt-level=2` (+1.9K flash — killed the align saturation: engage
    `over=` 409-744 → ~10). Steady drive: **7304/8500 = 86%**.
  - VERDICT machine-outliner: disabling it (llvm-args) is
    neutral-to-WORSE for speed here (step 4665→4794, obs 806→954) —
    flash wait states favor compact code locality. Keep `z`+outliner.
    Note: generic FOC code monomorphizes in the DEVICE crate at its
    opt-level — core@O2 only speeds non-generic core fns.
  - Current split: step=4665 (gate 956 of which read_currents 281,
    ctrl 1042 of which CORDIC trig 172, post 625, est 1485 = obs 806 +
    startup 127 + out 263 + rest), cmd 756, prot 595, ctail ~170,
    adc/snap/pub ≈1100. PC-profile is FLAT (top fn 3.4%) — remaining
    fat: struct-copy memcpys (~2%, FocOutput by-value chain), CS+RefCell
    (~3%, ~9 enters/cycle → cmd/prot/pub consolidation), f32 min/max
    clamp chains (~3%), obs=806 (why? objdump/derive), gate−curr=675
    (clamps should be ~200).
  - IDEA (from the flash-WS finding): hot ISR code as ramfunc — CCM
    leftover is only ~350 B today; would need the RAM ledger revisited.
  - Engage deadman at the standard 150 ms bound is STILL marginal
    (fires ~sometimes; fine at 400 ms bench bound) — align at ~87%+
    leaves thread mode just enough most runs. The est/gate/cmd/prot
    shave list above is the fix. Goal unchanged: drive total ≤ ~6500
    AND align ≤ drive.
- [x] **g431 RAM: stack → CCM SRAM split** — DONE 2026-07-06 (commit
  after the deadman hunt). Measured first, as this item demanded: stack
  paint + 1 Hz `stack: free_min=` HWM showed **408 B of idle headroom**
  (statics 25.3K left 7.48K of stack, idle peak 7.07K) — every deep
  drive-engage path overflowed it, and ALL of the day's "RTT read pointer
  changed" reboots were this. Final map: RAM 22K statics, CCMSTACK 9K
  (idle free 3184 B, drive peak ~7.6K), CCMDATA 1K relocated statics,
  flip-link retired (unmapped hole below CCM = hard BusFault on
  overflow). Shaves: defmt ring 512, down-ring 512, RECV_BUF 512
  (new MAX_RX_PACKET_SIZE), OUT_QUEUE 3328. Up-ring is FLOOR 4096:
  3072 regressed 20 kHz Stopped capture to 2.7% loss (regression gate:
  maneuvers/prof-20k-idle.json). Original analysis kept below for
  reference:
  (idea, robustness — not a
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

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
(deadshort flying restart) implemented 2026-06-13 in `phase/startup.rs` —
current-only / g431-capable, host-test covered (cold-start spin-up +
freewheel-catch on VirtualMotor). See decisions.md. Remaining is bench
validation + v1 refinements:

- [ ] **Bench-validate the startup** (the real gate): a cold-start spin-up
  from standstill and a freewheel catch on hardware — sim can't show
  cogging / saturation / sensor noise.
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
- [ ] **g431 RAM: stack → CCM SRAM split** (idea, robustness — not needed for
  throughput since raw-Pod made 20 kHz loss-free). G431's 32 K = SRAM1 16 K +
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
- [ ] **Detection biases confirmed (all high) — the √3 + dead-time concerns
  below, now measured.** ZD2808 results vs LCR/nameplate: R 0.127 Ω
  (LCR ≈0.105 Ω/phase, +20 % residual dead-time — fine); **Ld 86 µH / Lq
  122 µH (LCR ≈24 µH/phase → 3.6–5× HIGH)**; λ 1.30 mWb (≈1.13 expected,
  +15 %); **Kv 1051 RPM/V (nameplate 700 → 1.50× ≈ √3)**. Two systematic
  errors: (1) the g431 voltage-pulse L step inflates L badly on a low-L motor
  because the small pulse voltage is dominated by the ~0.38 V 800 ns dead-time
  distortion; (2) a √3 (≈1.5×) normalization in the λ/Kv path (`Kv =
  60/(2π·λ·Pp)` omits the √3). A sensorless spin on the as-measured L would
  give 3.6× hot PI gains + a biased observer `−L·Δi` term → fix L (or feed the
  LCR value) before trusting closed-loop. Decompose the √3 vs the SVPWM
  amplitude-invariance convention.



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

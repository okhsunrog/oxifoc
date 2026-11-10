# FOC execution-time research

## Resume here

Updated: 2026-11-11. Baseline commit: `3ee9089`.
Goal: shorten the real F405 FOC/ADC path at 20 kHz, including worst observed
cycles and PWM-update timing, while preserving protections and native telemetry.
Current stage: rotating baseline and measurement-method audit complete. Quiet
baseline is the reference for optimization. No algorithm optimization accepted.
Next: attribute estimator front-end and velocity wrapper costs, and measure PWM
commit phase before claiming that a long body missed the actuation deadline.
Normal production firmware restored; Stopped, fault_count=0.

Artifacts: `/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-foc-research-2026-11-11/`.
Use this one living note for progress; add conclusions/evidence and next actions,
not transcripts of questions. Do not repeat completed A/Bs without a reason.

## Established facts and constraints

- Production: STM32 DirectEdge, SPI master → C5 router → USB Serial JTAG;
  telemetry is Packed28 through Ergot/SPI, native nominal 20 kHz.
- Motor secured/unloaded; user authorizes motor excitation and flashing.
  Last confirmed PSU: 24 V / 5.5 A. Preserve persisted model, Hall geometry,
  calibration and current limits. No NVM writes planned.
- Normal image currently restored, Stopped, fault_count=0, Vbus about 24 V.
- Previous clock/memory experiment: see
  [clock/memory report](f405-clock-memory-bench-2026-11-10.md), commit `3ee9089`.
  Its current loop had phase outputs inhibited, so it is not a rotating baseline.
  Flash/SRAM data at 168 MHz: 36.76 µs representative body; SRAM code/CCM data:
  40.68 µs, with overruns. At 240 MHz, combined placement: 29.72 µs versus
  flash 31.65 µs. Defaults unchanged. Do not repeat this whole matrix now.
- Earlier implemented optimizations and failed candidates are indexed in
  [previous audit](performance-audit-2026-11-09.md), especially “Further ADC
  optimization”. Read targeted sections, not that entire historical log.
- Existing detailed profiling covers ADC, snapshot, commands, protections,
  control/PI/trig, estimators/observer/startup/phase tracking, PWM and publication.
  Buckets are nested; do not add parent and child times.
- Existing one-second means are not per-cycle percentiles. Body timing excludes
  the final bookkeeping/exception return. Entry gaps alone do not prove a lost
  ADC conversion. Detailed profiling changes the workload; final acceptance
  requires a minimally instrumented build.

## Work queue

1. Add bounded RAM histograms for active ISR-body and entry-gap distributions;
   read/export only while Stopped. Keep detailed and minimal builds separate.
2. Repeat the same Hall current/velocity maneuvers with 20-kHz capture and
   unchanged configuration. Retain failures and telemetry/ACK loss separately.
3. Attribute the largest costs, inspect linked assembly, choose at most two
   local optimizations. Compare correctness and identical hardware workloads.
4. Verify PWM update timing, then refine clocks near promising candidates.
5. Finish with minimally instrumented current/velocity runs, long enough to
   expose command/thermal transitions. Restore normal firmware and commit each
   validated logical step.

## Results

### 2026-11-11: baseline, rotating motor, SPI/Packed28

Three repeats each, unchanged persisted configuration, zero faults, all captures
lossless. Exact quantile bins/entry maxima/sequence spans and raw data are in
`baseline-minimal/manifest.json` under the artifact root. This image includes
bounded distribution bookkeeping, so final acceptance must also remove it.

| Workload | Mean body µs across repeats | Observed body max µs | Body overruns across repeats |
|---|---:|---:|---:|
| Id hold 0.6 A | 37.323–37.325 | 46.53 | 0 |
| Current pulse to 0.9 A | 37.883–38.678 | 47.17 | 0 |
| Hall velocity +40 el rad/s | 41.294–41.312 | 53.90 | 16 |
| Hall velocity +120 el rad/s | 42.692–42.719 | 54.95 | 27 |

The current pulses have differing rotor trajectories; those mean differences
are not a repeatable CPU optimization signal. Entry intervals have a ~50-µs
median, with isolated 78–86-µs maxima: investigate interrupt masking as well
as body time. All recording success claims are separate from deadline/control
quality. No speedup claimed.

### Timing-contract audit prompted by the user

Readback saved in `peripherals-verified.json`: TIM1 center-aligned mode 3,
PSC=0, ARR=4195, RCR=1, CCR4=4112; calculated PWM=20004.272 Hz. ADC1/2/3
independent, 12-bit scan, TIM1_CC4 rising trigger, 3 injected ranks each:
[10,3,0], [11,14,1], [12,13,2]. All use 15-cycle acquisition; only ADC3 owns
JEOC IRQ. ADC clock=20.979480 MHz, acquisition=0.715 µs, ideal three-rank
conversion=3.861 µs (excluding trigger synchronization). Hardware deadtime
reads back as 357.49 ns. Histogram ISR-entry median agrees with one 50-µs cycle.
The new harness asserts this full contract before excitation.

Previous overclock readbacks kept PWM around 20004–20007 Hz, but ADC clocks
changed: 168→20.98, 204→25.45, 216→26.94, 240→29.92, 260→32.40 MHz.
With unchanged 15-cycle sampling, acquisition shrinks to 0.463 µs at the last
step. Those were motor-inhibited CPU experiments, not analog-accuracy
qualification. They also changed SPI/APB clocks, so do not attribute the whole
gain solely to CPU. Future clock comparisons must export the complete ADC,
trigger, timer and deadtime contract; qualify acquisition/settling and sampling
phase before energized overclock measurements. These frequency estimates use
the earlier measured HSE, not a fresh independent scope measurement.


### Detailed profiling and failed attempts

Detailed hold: 39.506–39.509 µs versus minimal 37.323–37.325 µs, about 2.18 µs
instrumentation/layout overhead. Fresh diagnostic means: current pulse
39.975–39.998 µs, velocity40 43.322–43.333 µs; velocity40 had 58–62 body
overruns per run and producer losses. The captures remain diagnostic failures,
with complete events/error metadata embedded in their Parquet files.

- `baseline-sections`: three hold repeats pass; current-0 loses 26 samples,
  CLI correctly exits nonzero. Harness aborted/restored before histogram export.
- `sections-remaining`: current-0 loses one sample; a harness error looked for
  the CLI error in outer stderr instead of the capture log. Aborted/restored.
- `sections-complete`: three current repeats pass; three velocity40 repeats
  export histograms despite explicitly marked recording loss. Velocity120-0
  misses a command/terminal ACK and loses 882 producer samples: harness aborts,
  emergency Stop succeeds, normal image restored. Do not repeat +120 with full
  instrumentation. Minimal +120 already passed all three captures.

Stable detailed +40 window, nested cycle counts: ADC ~350, snapshot ~428,
whole shared FOC ~5675, publication ~762. Inside shared FOC: command ~464,
protection ~246, step ~4544, tail ~202. Inside step: gate ~427, controller ~883,
PWM/post ~315, estimator ~1983. Inside estimator: observer ~620, startup ~31,
output ~113. About 1200 estimator cycles are outside those child buckets;
measure sensor sampling and voltage preparation before choosing an optimization.
These are diagnostic attributions, not minimally instrumented runtime costs.


### Quiet-build control: reporting itself delays ADC entry

Three repeats each, warning-only logs like production, same RAM histogram,
clock, ADC setup, limits and 20-kHz Packed28 capture:

| Workload | Mean body µs | Maximum entry gap µs across repeats |
|---|---:|---:|
| Id hold | 37.241–37.243 | 52.81 |
| Velocity40 | 41.272–41.295 | 56.01 |
| Velocity120 | 42.668–42.698 | 56.76 |

All nine quiet captures are lossless, zero faults. Removing periodic reports
reduces entry maxima from 83–86 µs to 53–57 µs while preserving body means.
This is strong evidence that reporting critical sections caused the longest
entry delays. It is a compile-time A/B, so code layout is not independently
held constant. Do not optimize production around jitter introduced by info logs.
Rare body overruns remain (quiet velocity120 maximum 55.00 µs); neither an
entry histogram nor body maximum proves a missed TIM1 preload-transfer deadline.

`results.json` verifies 30 captures / 3,067,910 samples / 672 ACKs, including
explicitly failed detailed velocity40 captures. The 21 minimal/quiet runs all
have complete, lossless sessions; diagnostic losses remain separately marked.
`distributions.png` was visually inspected; it plots body and entry-gap CCDFs,
with 64-cycle bins (~0.381 µs). This is useful here; FFT is not relevant.
PCSR-only sampling also ran during native-rate velocity40 on the normal image
(`pc-velocity40/`); mixed active/Stopped windows and rounded per-site percentages
make that exploratory evidence, not precise CPU accounting.

Config before/after matches for every group except automatic boot DC offsets.
No motor/Hall model, limits or board-calibration writes occurred.

## Exact next actions

- Add counters for the phase-manager sensor front-end and voltage preparation;
  print the already collected `VELOCITY_REQUEST` bucket. Use a short +40
  diagnostic run; avoid the rejected full-profiling +120 workload.
- Inspect linked code at the largest bucket before changing algorithms. Preserve
  fallback/health checks and native protection cadence. Test any local candidate
  against `baseline-quiet`, three repeats at +40/+120 with 20-kHz Packed28.
- Use the existing finite PWM-commit trace with a current/velocity trigger to
  verify the time of CCR writes and the actual preload-transfer deadline.
- Recheck candidate without histogram/section instrumentation before accepting.

Recipes centralize flags: `just bench-mk5-foc OUTPUT RECOVERY minimal 3
hold,velocity40,velocity120 quiet`; detailed mode is `sections` with `timing`.
Recovery image used here: the known normal ELF in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-overclock-2026-11-10/current-light-repeat/recovery.elf`.
`just flash-mk5-spi` rebuilds/restores the current normal image. Do not overwrite
artifact directories or promote a detailed/gapped run into baseline acceptance.

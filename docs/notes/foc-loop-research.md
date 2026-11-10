# FOC execution-time research

## Resume here

Updated: 2026-11-11. Baseline commit: `3ee9089`.
Goal: shorten the real F405 FOC/ADC path at 20 kHz, including worst observed
cycles and PWM-update timing, while preserving protections and native telemetry.
Current stage: rejected all four local algorithm candidates; original Hall/driver
algorithms retained. Full-rate PWM-deadline instrumentation qualified hold and
+40 electrical rad/s, but overloaded +120 (7 producer drops, ACK timeout).
Three minimally instrumented +120 controls remain lossless. Do not claim
high-speed PWM deadline qualification from the failed diagnostic recording.
Next: reduce measurement overhead before qualifying the +120 actuation deadline;
then choose an optimization from measured costs. No algorithm speedup accepted.
Normal image is restored by each finite suite's cleanup.

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


### Selective profiling without reporting I/O

`fine-sections` and `sections-ram` retain the failed full-instrumentation +40
attempts: command ACKs and the stream overloaded even after removing log I/O.
Emergency Stop and restore succeeded. These are measurement-load failures,
not accepted performance runs. Do not keep rerunning the full-mark image live.

Added a selected phase group instead: 2 front-end markers per cycle and the
existing due-only velocity-request marker. A one-shot RAM window is published
only after >1.1 active seconds; this excludes the mixed entry/Stopped window.
The host rearms/export while Stopped. No per-window text I/O. `phase-selective`
passed hold/+40/+120, all native captures lossless, zero faults.

| Case | Sensor/front-end cycles | Voltage preparation cycles | Velocity-request cycles averaged per native cycle |
|---|---:|---:|---:|
| Hold | 801 | 48 | 0 |
| +40 | 983 | 48 | 37 |
| +120 | 1159 | 48 | 37 |

The outer velocity request is about 740 cycles per scheduled evaluation if
using the nominal 20:1 ratio, not 37 cycles per evaluation. More granularity
is now affordable without enabling every nested marker. Full core tests:
696 unit tests passed (2 ignored), plus 11 doctests after the counter additions.



### Independent IRQ-cadence cross-check

Applied-command ACK timestamps bracket the first active and final Stop/Coast
transitions in host monotonic time. The histogram's active ISR count divided
by these duration bounds does not use HSE, DWT scaling or firmware `t_s`.
Every successful baseline agrees with the TIM1-derived ~20.004 kHz. For the
4.8-s holds, typical host-only bounds are ~19.97–20.05 kHz (transport latency
limits precision); they exclude a doubled or CPU-scaled cadence. Future
profile runs check this automatically, alongside register readback. Previous
overclock figures remain RCC/HSE-derived; no new scope measurement is claimed.


### Rejected local candidates

`phase-baseline-repeat` uses the exact same ELF SHA as `phase-selective`
(`c0ec8558...`). Two further repeats: hold 37.395–37.397 µs, +40
41.272–41.274 µs, +120 42.543–42.552 µs. All captures lossless. Slight
trajectory/state variation exists even with an identical ELF; keep that scale
in mind when judging sub-microsecond results.

| Candidate | Hold mean µs | +40 mean µs | +120 mean µs | Decision |
|---|---:|---:|---:|---|
| Edge age returned with sample, 3 repeats | 37.089–37.091 | 41.171–41.250 | 42.620–42.656 | Reject: high-speed regression |
| Same with private helper forced inline, screening | 37.205 | 41.211 | 42.627 | Reject: high-speed regression |
| Runtime cruise-context proof, 3 repeats | 37.536–37.537 | 41.086–41.127 | 42.446–42.453 | Not accepted: direct-current regression |

All these captures were native-rate and lossless, no faults. Edge-age sources,
patches and tests are preserved in `hall-age-candidate/`, `hall-age-inline/`
and `hall-age-tests.log`; original Hall source is restored. Do not repeat the
same return-value fusion without a new explanation of the ABI/code-generation
cost. The runtime context is preserved in `modes-enum-context.rs` and
`cruise-context/`; standard core tests passed.



Static cruise-context screening (`cruise-static`): hold 38.496 µs, +40
40.805 µs, +120 42.039 µs. Faster velocity at the cost of a ~1.10-µs
regression in direct current; reject the whole family for this iteration.
All screening captures lossless, no faults. Source retained in
`modes-static-context.rs`; standard unit/doctests passed before hardware.
Original `modes.rs` is restored. Do not keep these refactors solely because
the types look cleaner: their measured whole-workload tradeoff did not meet
acceptance. No pure-software speedup is claimed from these candidates.

The next useful qualification is actuation timing: PWM is written before the
estimator/publication tail. Measure end-of-CCR writes against the next actual
TIM1 peak, plus the UDIS interval. This distinguishes a long ISR body from a
missed PWM update and provides a concrete deadline/headroom target.


### Actual PWM commit deadline (2026-11-11)

Default-off `bench-foc-deadline` uses a free-running TIM5 (PSC=0), TIM1
counter/direction at ADC entry, and a timestamp after CCR writes and observed
UDIS restoration. RCC checks the TIM1:TIM5 2:1 ratio. Timestamping before the
entry-phase reads makes the computed next-peak margin a conservative estimate.
Counters freeze on Stop; no per-cycle logs or SWD reads during excitation.
This measures driven commits, including the final Stop write, separately from
the whole ISR body. It does not change the control algorithm or normal image.

`pwm-deadline-v2`, three repeats per completed mode, native Packed28/20 kHz:

| Workload | Minimum margin across repeats, µs | Largest whole body, µs | Late commits |
|---|---:|---:|---:|
| Hold Id=0.6 A | 13.918–15.217 | 47.326 | 0 |
| Velocity target +40 electrical rad/s | 9.843–10.558 | 54.720 | 0 |

576,340 driven writes qualified, 688,086 captured records, zero ambiguous
phase samples, masked commits or out-of-ADC writes. All six captures complete,
lossless and all commands acknowledged; independent host-time cadence bounds
agree with the ~20.004-kHz TIM1/ADC rate. Worst observed margin is not a WCET
proof or a qualification of every speed/load. Whole-body >50 µs can occur
*after* the PWM commit; those overruns alone do not demonstrate late actuation.
The diagnostic itself adds cost (hold body ~37.859 µs versus ~37.24–37.40 baseline).

The first +120 diagnostic run failed at a command ACK deadline (200 ms) and
lost 7 producer records; host drops=0. Finite test stopped, recovery image
restored, fault_count=0. Retain the incomplete Parquet and error metadata;
no frozen deadline result was exported, so high-speed timing remains unknown.
`deadline-control`: three +120 minimal-instrumentation controls, all lossless,
body means 42.861–42.885 µs, maxima 54.613–55.048 µs. This is consistent with measurement
load contributing to the failed diagnostic workload; host scheduling delays
remain another possible contributor. It does not prove exact causality or
qualify velocity tracking. Native current/angle plots were inspected.

Evidence: `pwm-deadline-v2/manifest.json`, `deadline-control/manifest.json`,
`pwm-deadline.png`, `pwm-deadline-failed120.png` under the artifact root.
The new source module is retained alongside the exact ELF because the initial
source.patch was captured before the new file was added to the index.


`pwm-deadline-current-repeat`: direct Iq steps 0.3/0.6/0.9 A, one complete
lossless capture, 20,023 driven writes, minimum margin 15.539 µs, no late,
ambiguous, masked or out-of-ADC commits. Body mean 39.116 µs, maximum
47.701 µs. Actual TIM5 readback: running, PSC=0, ARR=0xffffffff;
83.917920 MHz from RCC, versus TIM1 167.835843 MHz (integer rounding).
The preceding `pwm-deadline-current` attempt aborted before its second step:
host affirm gaps 93/151 ms and a late-command refusal. That partial stream
itself was complete and lossless; preserve it as a failed maneuver, not a
successful current test. The repeat changed no parameters or deadlines.
Final recovery: Stopped, fault_count=0, bus 24.053 V, FET 29.5 °C,
motor 28.7 °C. Six histogram/decoder tests and four peripheral-contract tests
pass. Production transport, model/calibration, limits and PWM target unchanged.

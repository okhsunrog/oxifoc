# STM32F405 ADC ISR optimization

Updated 2026-11-17. This is the single record of accepted optimizations,
measured alternatives and rejected experiments. Motor tuning, Hall behavior
and transport qualification remain in the [bench journal](foc-loop-research.md).
Timings belong to specific MK5 images; gains from different series are not additive.
Follow the [measurement workflow](../performance.md) for new experiments.

## Accepted implementation

Process commands every ISR. A shared native-cycle frame schedules the outer
PI at approximately 1 kHz and prepares its scalar current-authority bounds one
slot earlier. Fresh drive activity can postpone the PI by at most one ISR.
Deadman expiry is polled once per frame and derating refreshes every eight
frames. These approved paths are enabled by default through the regular
`service-slots`, `pi-bounds-ahead`, `iq-authority`, `lean-angle` and
`observer-turn` features. The last two retain bit-identical angle semantics.
The implementation inlines the current-control helper and outlines velocity
startup/unavailable branches with `#[cold] #[inline(never)]`. Firmware uses the
ordinary target, frame pointers and the configured PWM frequency.

| Work | Cadence and behavior |
|---|---|
| Current PI, phase estimation, observer, fast protection, PWM | Every ISR, normally 15 or 20 kHz |
| Command FIFO, Stop, accepted drive/affirmation timestamp | Every ISR |
| Velocity/position PI | Approximately 1 kHz, actual elapsed `dt`, slot frame with elapsed-time backstop |
| PI postponement on drive activity | At most one ISR: 66.7 / 50 us at 15 / 20 kHz |
| Deadman expiry polling | Once per frame, approximately 1 kHz; timeout 150 ms |
| Temperature conversion | Every 128 ADC cycles |
| Derating refresh | Every eight frames, approximately 8 ms |
| PI authority bounds | Prepared one native cycle ahead, single-use, fresh fallback after invalidation |

A postponed PI returns to its slot frame with bounded delay.
Actuator feedback continues accumulating every ISR, and command floods cannot
starve the PI indefinitely. Postponing the first update holds a zero Iq request.
Angle/speed trust gates and live current constraints still run every ISR.
Velocity and position modes use the outer PI; current control uses the inner
current PI every ISR.

The tradeoff is one ISR of additional PI jitter and about 1 ms before
detecting deadman expiry while the ISR runs. Link gating and execution of active
faults remain native-rate. Decimated polling does not replace the independent
watchdog if the ISR itself stops.

The implementation lives in `motor/foc_driver/{velocity_schedule,protection,modes}.rs`
and `state.rs` under `oxifoc-core/src/`. The original bounded PI deferral came
from measured commit `4b4986f`. The slot frame and pipelined bounds were approved
on November 17. Increased `.text` is accepted for measured speed improvements.

## Measurement contract

- Mini V6 MK5, STM32F405RG, unloaded H5055, approximately 24 V. CPU about
  167.836 MHz, Flash WS5, ART prefetch/I-cache/D-cache enabled.
- HallToObserver with 150–300 electrical rad/s blending; targets +120 and +450.
  Native-rate Packed28 recording through SPI → C5 → USB Serial JTAG.
- `nightly-2026-11-13`, opt3, fat LTO, CGU1, incremental=false,
  `+vfp2,-fp64`, quiet `DEFMT_LOG=warn`, unless stated otherwise.
- DWT measures the ISR body before final statistics bookkeeping. Entry gaps
  and CCR-write margin before PWM preload are separate measurements. Observed
  maxima are not a complete exception WCET bound; zero late CCR writes do not
  prove that the entire ISR fits within one period.
- Every result identifies its own ELF/SHA256, features, revision and scenario.
  Repeat the same ELF. Build paths, inlining, LTO and Flash placement can change
  timing even when normalized assembly looks similar.
- Accept captures only with every command acknowledged, terminal Stop, complete
  raw sequences/END totals and unchanged `fault_generation`. Preserve rejected
  attempts separately. Host affirmation gaps, capture loss and DWT epoch changes
  invalidate normal comparisons.
- Wait 1.3 s in Stopped after the last SWD setup access: probe disconnect may
  disable DWT until the supervisor repairs it.

## Earlier optimization history

| Date / stage | Retained changes and evidence |
|---|---|
| Nov 9–10: first batches | Removed redundant atomic/snapshot work, prepared observer/Hall coefficients and shortened encoder/drain work. A separate Id-hold series improved from 6668 to about 6264 CPU cycles; it is not the later velocity baseline. |
| Nov 10: math and diagnostics | Shared FastSinCos reduction, applied-voltage ring with prepared weights, exact NTC lookup tables (+16 KiB Flash, no table RAM). At +40 rad/s and 20 kHz, mean 41.77–42.05 → 39.36–39.42 us; rare peaks remained 51.7–52.2 us. NTC agreement: 8194 MCU comparisons. |
| Nov 11: instrumentation | Added finite RAM body/entry-gap histograms and a separate PWM witness. Periodic reporting itself delayed ADC entry; acceptance uses quiet images. |
| Nov 13: nightly migration | Recognized the new `vfp2sp` feature name and restored hardware `vsqrt.f32` instead of libm. Uncorrected +120 mean: 48.31–48.33 us; corrected: 46.67–46.71. Kept `+vfp2,-fp64`. |
| Nov 14: Embassy/clocks | `3b867e0` aligned PLL and center-aligned ARR calculation with the HAL while retaining physical TIM1/ADC timing and deadtime. |
| Nov 14: incremental/PRIMASK | Matched nonincremental builds reduced the post-dependency regression from about 0.8 to 0.15–0.18 us. Address-preserving ISB→NOP measured 0.15–0.20 us attributable to barriers. Upstream ISB/compiler ordering were retained. |
| Nov 14: ADC ownership | `4ae27e1` transferred ADC handles to ISR-local storage; hot ADC PRIMASK/ISB sites fell from 9 to 6. +120 mean: 46.76–46.83 → 46.23–46.28 us. |
| Nov 14: FOC ownership | `20f9eab`, then `8fa1e5b`: ISR owns the driver; rare retiming is out of line; availability/VBUS share one word. Nonincremental +120/+450: 46.256/45.592 → 46.266/45.524 us. Release incremental was disabled. Detection/cancellation retain 15 → 20 → 15 kHz restoration. |

### Nov 14 batch above `8fa1e5b`

15 kHz, +120 / +450; mean ISR body time in us.

| Stage | Mean | Disposition |
|---|---:|---|
| Baseline | 46.362 / 45.661 | Repeat: 46.344 / 45.670 |
| Hall snapshot reuse + initial arithmetic | 45.191 / 44.595 | Hall contribution not independently isolated |
| + empty-FIFO fast path, direct clamped CCR writes under UDIS | 43.625 / 42.984 | Retained |
| + temperature/angle/bus-bound reciprocals | 43.751 / 43.103 | No independently measured time saving |
| + prepared source dispatch | 42.840 / 42.213 | Retained |
| Later series: cached readiness, integer angle fast paths, no Hall reuse | 38.97 / 38.62 | Final pre-scheduling state; different build series |

Observer readiness is refreshed whenever its inputs mutate; debug builds
compare cached and live results. Angle helpers test ordinary f32 ranges using
integer bit comparisons and keep general fallbacks out of line. Frozen-old-code
comparisons cover boundaries and two million random inputs. Hall snapshot reuse
was removed: after the angle fix, removal simplified the code and improved the
ISR by 0.6–0.7 us. That reuse scheme is absent from the accepted implementation.

Five failsafe/Kill defmt messages were commented out. Each cost about 15 us,
and several could occur together. A deliberate 0.4-s host pause reduced the
observed maximum from 91.0 to 52.3 us. The fault registry retains the event.
Other rare ISR messages remain; not every exceptional path has a WCET bound.

### Numerical contracts

- Reciprocal bus bounds: up to two f32 ULP versus the old calculation; physical
  bound `4 * EPSILON * max(bus_limit, 1 A)`, below 48 uA at 100 A. The measured
  current trip is independent of this tolerance.
- Temperature error below 0.00025 °C for input quantization of 0.1 °C. At an
  exact threshold, rounding may trip one input quantum earlier; tested positive
  thresholds never trip later than the old calculation.
- Current-circle fast path applies strictly inside the squared budget. Boundaries
  and underflow retain sqrt; differential cases include subnormals, zero,
  infinity and NaN.
- Codec reciprocals may change encoding by one wire quantum. Tests cover all
  16-bit boundaries and neighbors. The angular bound was checked over ±4 turns,
  not arbitrary huge f32 angles. Unsuitable reciprocals retain division.

## Scheduling experiments, Nov 15

Long steady-state cycles mostly came from coincident velocity PI and command
work: about 5.6 + 3.7 us beyond an ordinary cycle. Startup can also combine
observer/gate events without PI+command; scheduling alone cannot bound every peak.

**P**: bounded PI postponement. **C**: command FIFO near 1 kHz. **W**: decimated
deadman expiry. **H**: only Hall-warning publication, with the controlling Hall
estimator unchanged. **T**: less frequent thermal checks.

| Variant, 20 kHz | Mean +450, us | Largest peak at +120/+450, us | Decision |
|---|---:|---:|---|
| Quiet baseline | 39.321 | 52.301 | Rare bodies exceed the period |
| C+W, 1-ms wall clock | 38.525 | 48.965 | Stop waits up to about 1 ms; not selected |
| FIFO every ISR except PI, W near 1 kHz | 38.987 | 48.089 | Fixed PI grid; commands may wait one ISR |
| P+W, existing `/16` | 38.489 | 47.832 | Selected: commands every ISR |
| C+W, generic reset counter 20 | 38.675 | 48.470 | Generic scheduler overhead |
| C+W, simple counter 0…19 (`e910406`) | 38.357 | 47.195 | Faster counter; command-latency tradeoff remains |
| C+W, constant `%20`, ordinary wrap | 38.485 | 47.672 | Phase changes at u32 overflow |
| C+W, wrap-corrected `%20` | 38.912 | 48.041 | Slower linked image |
| C+W, protection counter `/16` | 38.519 | 47.642 | Also changes PI frequency |
| C+W, `/32` | 38.374 | 47.552 | PI falls to 469 Hz at 15 kHz; worse ripple |

The outer PI already ran near 1 kHz before these experiments. C+W can add
about 1.5 ms from a velocity command to its new Iq request, or about 1 ms for
current/Stop, because commands and PI occupy different slots. Selected P+W
adds no command service slot. Observed P+W PI intervals at 20 kHz were mostly
20 ISR, occasionally 19/21; mean dt was 999.996 us. Host ACK timing includes
transport and does not measure a 50-us MCU application delay.

At 15 kHz, T/H did not provide enough convincing benefit to accept their added
latency. H alone measured 39.289 versus 39.412 us at +450, with peaks
50.585 versus 51.747 us; C+W+H measured 38.492 us. The controlling Hall path
was never decimated. The initial H prototype broke warning retention; its
publication logic was corrected before hardware measurement. Only W was
selected, reusing the existing protection tick.

Why fewer operations did not always mean a faster ISR:

- Empty FIFO polling was already a cheap atomic fast path. Generic scheduler
  clock/state/branches consumed some savings; the simple counter removed them.
- `%16` and `&15` produced identical loadable ARM sections. Runtime divisors
  cost more than constants. `wrapping_add` + `%20` changes cadence at u32 wrap,
  about every 59.6 hours at 20 kHz; reset and power-of-two counters avoid this.
- `/16` and `/32` do not preserve a millisecond: PI becomes 1250/625 Hz at
  20 kHz, or 938/469 Hz at 15 kHz. Repeated `/32` runs at 15 kHz increased
  speed-estimate SD to 13.64–13.87 from baseline 6.59–7.46 electrical rad/s.
- LTO changes inlining, registers and addresses. The aligned counter retained
  7900 identical normalized instructions but slowed by about 0.16 us. Exact ART
  stall counts were not measured.

## Further optimization of selected P+W

20 kHz, 14 s each at +120 / +450; native DWT, without sampler or PWM hooks.

| Variant / original commit | Mean, us | Maximum, us |
|---|---:|---:|
| P+W control `c7b7c0a` | 38.918 / 38.485 | 47.368 / 47.487 |
| Compact OuterClock `ab2c82a` | 38.974 / 38.570 | 48.107 / 47.844 |
| Angle helpers without cold `edf31b0` | 39.074 / 38.668 | 47.392 / 47.219 |
| Angle helpers inline `a6f3a13` | 39.184 / 38.700 | 46.992 / 46.921 |
| Current helper inline `fd91b0b` | 38.732 / 38.302 | 47.046 / 49.155 |
| Velocity startup/gates cold `81e8a21` | 38.752 / 38.363 | 47.511 / 46.980 |
| Current + angle inline `6675aa4` | 38.810 / 38.447 | 46.986 / 46.814 |
| **Current inline + velocity cold `4b4986f`** | **38.588 / 38.160** | **47.356 / 46.826** |

Repeating the exact same ELF at +450:

| Image | Mean first / repeat, us | Maximum first / repeat, us |
|---|---:|---:|
| P+W | 38.485 / 38.490 | 47.487 / 47.975 |
| Selected | 38.160 / 38.159 | 46.826 / 46.605 |

Additional mean saving: about 0.33 us / 0.86% of the ISR body. At +120 the
mean improves similarly, while the observed maximum barely changes.
`.text` grows 398684 → 405980 bytes, **+7296**; `.rodata`, `.data` and `.bss`
are unchanged. Velocity-cold-only saves 0.12–0.17 us for +104 bytes, but the
larger speed improvement was selected.

Inlining removes the separate current-helper call boundary. Outlining startup
reduces the velocity wrapper from 5120 to 2348 bytes versus current-inline-only.
PC profiles support lower save/restore overhead, without precisely attributing
the complete 0.33-us gain to individual instructions.

### Hardware evidence

- +120 → +450 → +120 → −120 → +120 → Stop: all commands acknowledged,
  lossless capture, unchanged fault generation. P+W/selected mean:
  38.811/38.474 us; maximum: 49.173/48.303; phase-current peak: 3.14/3.38 A.
  Late mean speed differs by less than 0.2 electrical rad/s. One unloaded
  maneuver does not prove equal loaded transients.
- Separate PWM witness: 280140/280103 driven writes; minimum margin
  15.205/15.348 us at +120/+450; late/ambiguous/still-masked = 0/0/0.
- Intentional 0.4-s host pause: generation 0→2, final Stopped, zero faults,
  maximum body 46.599 us. Expected raw discontinuity makes this fault injection,
  not an ordinary accepted lossless run.
- Deep P+W series: 28 accepted captures, 8,325,366 samples, 64 commands.
  The second research series before integration totals 116 captures,
  32,149,101 samples and 256 commands. Rejected attempts remain separate;
  the deadman threshold was not relaxed.

### Stock 15-kHz integration: archived images

The initial integration images were exercised without the carrier override:
normal-image +120/+450 holds and the reversal maneuver, plus separate DWT and
PWM witnesses. Seven captures retained 1,267,584 raw samples without loss;
all 18 commands were acknowledged and fault generation stayed unchanged.

| Separate measurement image | +120 | +450 |
|---|---:|---:|
| DWT mean ISR body, us | 38.677 | 38.245 |
| DWT observed maximum, us | 46.939 | 47.797 |
| Minimum PWM margin, us | 31.793 | 31.531 |
| Driven PWM writes | 210051 | 210055 |

No measured body exceeded the period; PWM late/ambiguous/masked = 0/0/0.
The ordinary image was restored at 15 kHz, Stopped, zero faults and all phase
PWM enables off. Normal-image SHA256:
`a49890da1bd04767da6773c58c7a901ddaa476a0202e059cb5ec4762a7cd5e9c`.
These timings identify the archived images, before later iterator-only source
cleanup; they are not measurements of every subsequent rebuild.

## Other experiments and rejected alternatives

| Experiment | Result and conclusion |
|---|---|
| Compact OuterClock / Option niche | ARM `Option<u64>` is 16 B; `Option<NonZeroU64>` is 8 B. Full u64 timestamps plus valid flags shrink the clock 48→32 B and pass 500,000 differential steps, but slow the ISR. A direct NonZero runtime variant was not measured; `timestamp+1` loses `u64::MAX`. |
| `-Cforce-frame-pointers=no` | Did not override target `always`; runtime sections were identical. This is not evidence from actually removing frame pointers. |
| Custom `may-omit` target | Frame setups 326→26, `.text` −8424 B, but slower. Repeated +450: built-in/prebuilt core 38.490; custom/rebuilt core/always 38.555; custom/rebuilt core/may-omit 38.697 us. The additional 0.142 us persists with matched core rebuilds. Register/layout effects and application/core FP changes were not separately isolated. |
| `cold_path()` on command presence | Fixed-counter +450 mean 38.357→38.489 us; rejected. Mark specific rare paths, not an entire native-rate service. |
| Function alignment 16 / MIR opt3 | Counter 38.357→38.518/38.525 us. Global function alignment gave no convincing gain; earlier block alignment added 8% `.text` and about 0.3 us. `-Zmin-function-alignment=16` failed the reset-vector linker assertion. |
| `target-cpu=cortex-m4` | Counter +450 mean 38.357→39.139 us. Earlier ARM-flag comparisons also did not justify changing normal flags. |
| `+execute-only` / no jump tables | Nov 14 baseline regressed by 0.42–0.49 / 0.58–0.63 us; rejected. |
| Temperature cache / command handler `inline(never)` | Early cache A/B slowed the mean by about 0.62 us; outlining the command handler did not help. |
| Observer algebra / Hall decay cache | Nov 10 H3/H5 baseline 6251 cycles became 6283/6282; rejected. Additional state can cost more than recomputation. |
| Shadow-observer multirate | Earlier screening reduced loss but still failed continuous capture; reverted. Accepted observer work remains native-rate. |
| Whole STATE/driver in CCM | Stock-clock, phase-inhibited workload: 36.76→36.81 us; no useful gain. Existing CPU-only telemetry-queue CCM placement is a separate setting. |
| Hot code in SRAM (`ram-foc`) | At 168 MHz with phases inhibited: 36.76→40.62–40.68 us and two body overruns. Not selected; CCM cannot execute code. |

`DriverCommand` is **56 bytes** on the measured ARM target, not 176. Larger
copies came from rare model/config application. Command enum/FIFO layout and
copying were not reduced in this series.

VESC reference: local `/home/okhsunrog/motor_control/bldc`, `1ad6eab3` (7.00).
Speed/position PID defaults to about 1 kHz in a thread with actual dt;
thermal/soft-limit and supervisor threads sleep 1 ms. Current control,
Hall/observer and fast electrical trips remain ADC-rate. This supports dividing
work by cadence, not decimating every protection check.

## Tools and reproduction

| Question | Tool |
|---|---|
| Isolated counters, modulo/masks and time-reading cost | `tests/stm32f405`, embedded-test + DWT, gate off; `just bench-embedded-f405 OUTPUT RECOVERY` |
| Arithmetic kernels and controlled ART/cache behavior | The same embedded-test runner, filters `z_bench_arithmetic` and `z_bench_layout`; exact images and repeat manifests |
| Short-window fetch/LSU stall costs | Filter `z_bench_art_counters`; SRAM observer, raw 8-bit DWT residues, overflow calibration and ambiguity counts |
| Full ISR-body and entry-gap distributions | `bench-foc-distribution`, `scripts/bench_foc_profile.py`; `just bench-mk5-foc OUTPUT RECOVERY minimal 3 velocity120 quiet` |
| Margin before PWM preload | Separate `bench-foc-deadline`; use `deadline` instead of `minimal` above |
| Actual TIM1/ADC frequency and phase | `scripts/f405_timing.py`, register readback while Stopped |
| Motor behavior and transitions | Host CLI maneuver, Packed28, ACK, fault generation, terminal Stop |
| Executed code | ARM disassembly and PC sampler against the exact ELF/SHA; attach/detach only while Stopped |

Run from the direnv/Nix shell, using `uv run` for Python. `OUTPUT` is a new
directory under `/home/okhsunrog/tmp_zfs/codex_tmp/`; `RECOVERY` is a verified
working image. Normal builds retain their ordinary target/cache paths.
The isolated benchmark masks interrupts and lacks ADC, motion and real command/PI
collisions. Its whole-window cost includes loop/black_box/DWT overhead; it is
not a guaranteed instruction-latency measurement.

The unchanged PCSR sampler records about 5,600 PC/CCER samples/s. Scheduling:
58 ELF × 2 speeds, 116 profiles, 5,166,528 driven samples. Further P+W work:
11 ELF × 2 speeds, 22 profiles, 935,296 samples. Each is symbolized using its
own DWARF inline frames. These are whole-MCU statistical occupancy profiles,
not exact call costs or WCET. Inline/save-restore groups overlap and are not additive.

Disabled ISR logs cannot establish fault-free operation. `scripts/benchsuite.py`
also compares status `fault_generation` before/after: a timeout cleared by
terminal Stop still rejects the run. Missing/unreadable counters fail the check.

## Artifacts and remaining work

Local artifact root:

```text
/home/okhsunrog/tmp_zfs/codex_tmp/
```

| Directory under the root | Evidence |
|---|---|
| `oxifoc-adc-opt-2026-11-10/` | Initial math/NTC batches, exact ELF, raw captures, report |
| `oxifoc-foc-research-2026-11-11/` | Early body/gap/PWM work, rejected Hall/cruise variants |
| `oxifoc-nightly-ab.C6IY3o/`, `oxifoc-arm-flags.uLpWZf/` | Nightly/FPU/flag comparisons; early printed PWM margins are doubled, use raw TIM1 ticks |
| `oxifoc-embassy-ab.ox3qLp/`, `oxifoc-isr-cause.9mxWCf/` | Embassy/clocks, address-preserving PRIMASK A/B, incremental |
| `oxifoc-adc-ownership.h0xmdu/`, `oxifoc-foc-ownership.E7QTle/`, `oxifoc-foc-regression.Ydk196/` | ADC/FOC ownership, detection/retime/cancel, recovery |
| `oxifoc-isr-opt-2026-11-14/`, `oxifoc-isr-review.dW5aab/` | Arithmetic/FIFO/CCR, readiness/angles, logs, original PC sampler |
| `oxifoc-cadence-2026-11-15/` | First cadence matrix, embedded-test, plans, raw and rejected attempts |
| `oxifoc-cadence-2026-11-15/services/` | `service-capture-rollup.json`, `pw-native-summary.csv`, PC manifests in `pc-all/` and `pc-pw/`, analysis/disassembly/build provenance |
| `oxifoc-cadence-2026-11-15/adoption-final/` | Stock 15-kHz integration images, captures and restoration evidence |
| `oxifoc-arithmetic-2026-11-15/` | Arithmetic matrix, MCU numerical traces, identical-object placement sweeps, ART fixtures, native timing and PC profiles |
| `oxifoc-art-counters-2026-11-15/` | DWT CPI/LSU calibration, four identical-object placements, cache-state matrix and native torque windows |
| `oxifoc-adc-callgraph-2026-11-15/` | Linked ADC callees, call sites, tail branches, resolved wakers, callback candidates and placement coverage |
| `oxifoc-callee-layout-2026-11-15/` | Extended 14-candidate placement sweeps, fixed-address compiler-object controls and native captures |
| `oxifoc-compact-arithmetic-2026-11-15/` | Compact-layout arithmetic follow-up, 492-function address controls, opcode controls, fresh embedded-test results and native validation |
| `oxifoc-final-compact-atan-2026-11-16/` | Main-derived compact-layout/atan2-FMA integration, release and diagnostic ELFs, 15/20-kHz transition and PWM-margin qualification |
| `oxifoc-rare-peaks-2026-11-16/` | Startup/braking attribution, RTT-only logger trial, same-ELF output control, scalar MCU benchmarks and atomic-logger assembly audit |

Reference 20-kHz research ELF: `services/pw-current-cold20/firmware.elf`, SHA256
`1f66ad2379f65da87cdb054025ecd36eeea7fe86ba2ddab33f27db3220053827`.
It contains a temporary carrier override and is not the normal image.

Evidence covers unloaded MK5/H5055 with HallToObserver. Remaining qualification:
loaded control, sensorless/HFI startup, other modes and rare fault/config paths,
and complete worst-case stack use. Instrumented PGO, new SRAM-transfer boundaries
and other MIR/LLVM thresholds remain open. Arithmetic and explicit placement
results follow below.

The rare-path investigation below attributes the Observer-startup peak to its
reseed warning and the braking peaks to an expensive authority probe coinciding
with other native-cycle work. Further candidates include a smaller q-authority
dispatcher and an atomic or single-writer logging backend.

## Arithmetic experiments, Nov 15

Source baseline `6a52d74`; instrumented 20-kHz baseline `49f337d`.
Pinned nightly/FPU flags, opt3/fat LTO/CGU1 and incremental=false match across
the matrix. Each of 24 images has native +120/+450 holds and a separate +450
PCSR profile. Numerical regressions remain in the evidence alongside the motor
measurements. Arithmetic policy selection awaits review.

Full ISR mean at +450 electrical rad/s, in us. Each row changes only the named
kernel; the common ordinary-arithmetic baseline is 38.195.

| Changed kernel | `mul_add_relaxed` | Required FMA | `algebraic_*` |
|---|---:|---:|---:|
| Sine polynomial | 38.426 | 38.426 | 38.190 |
| Park / inverse Park | 38.190 | 38.191 | 38.193 |
| PI integration/output | 38.186 | 38.188 | 38.175 |
| Observer / filters / PLL | 38.079 | 38.079 | 37.952 |
| atan2 polynomial | 37.954 | 37.953 | 38.141 |
| All five groups | 38.380 | 38.380 | 38.195 |

Other images measured 38.096 us for core sqrt, 39.971 for guarded Q15 Park,
38.944 for the ordinary baseline with `target-cpu=cortex-m4`, 39.066 for
all-FMA/M4 and 38.951 for all-algebraic/M4. Q15 includes f32 conversion,
packing, saturation and a general-input fallback, using a 128-unit scale for
both current and voltage coordinates. These timings include its integration cost.

Exact baseline repeats span 38.186–38.195 us. Individual gains do not add
when combined.

| Combination | Mean +450, us |
|---|---:|
| Observer algebraic + atan2 FMA | 38.292 |
| Observer algebraic + core sqrt | 38.320 |
| Observer algebraic + atan2 FMA + core sqrt | 38.198 |

### Instructions and placement

Relaxed/FMA pairs have byte-identical allocated sections for sine, Park, PI,
observer, atan2 and the combined image. Their x86 host tests can differ because
relaxed rounding permits a different instruction choice on that target.
The M4 CPU flag removes ordinary VMLA selection, expands `.text` by 7088 bytes
in the baseline and slows the full ISR. Core sqrt exposes VSQRT to LLVM;
the former assembly wrapper already used the same hardware instruction.

VMLA and VFMA both have a nominal three-cycle cost on M4. Instruction
dependencies and Flash placement still affect execution. The linked sine-FMA
image moves `run_foc_cycle` by four bytes. An address-preserving control replaces
111 sine VMLA/VMLS instructions with VFMA/VFMS and measures 38.187–38.194 us,
overlapping the baseline. This control retains every other executable byte.
[Arm Cortex-M4 TRM, section 7.2.3](https://web.eece.maine.edu/~hummels/classes/ece486/docs/arm_cortexm4_processor_trm_100166_0001_04_en.pdf#page=67)

The ordinary `grant_torque` body is the same 736 bytes in the baseline and
the individual atan2/observer images. Its address changes from offset 12 to
offset 4 within a 16-byte line. PCSR occupancy falls by about 0.2 us per
50-us period. The combined observer+atan2 image puts it back at offset 12.
A separate identical-object relink experiment isolates this placement effect.

The gate-disabled MCU benchmark covers 55 cases with three exact-image repeats,
48 rounds and 64 calls per round. Selected gross cycles per call follow.

| Kernel | Ordinary | Alternative | Result |
|---|---:|---:|---|
| Sine polynomial | 38.014 | 38.128 FMA | Essentially equal |
| Complete bounded sin/cos | 96.697 | 96.778 FMA | Essentially equal |
| Park | 38.212 | 38.212 FMA / 67.254 Q15 | Conversion makes this Q15 path slower |
| atan2 | 72.187 | 73.119 FMA / 74.169 algebraic | No isolated speed gain |
| PI with coefficients opaque at each call | 61.848 | 57.800 algebraic | About four cycles saved |
| Sqrt | 32.883 assembly | 32.818 core | Same hardware operation |
| Packed signed dot | 27.018 Rust | 26.127 DSP | Two multiply instructions become one SMLAD |

Loop/input/black-box overhead is included. The PI comparison prevents coefficient
hoisting out of the batch. Its disassembly still shows useful reassociation and
different register/stack scheduling, while the full PI image saves only about
0.02 us. Rust and intrinsic qadd/ssat/smulwb already select the same instructions;
their roughly one-cycle timing differences expose placement/scheduling sensitivity.

### Numerical contracts

- On MCU sweeps of 65,537 inputs, the sine polynomial's maximum absolute
  error is 7.01e-7 for ordinary arithmetic and 6.81e-7 with FMA. The maximum
  change is one f32 ULP, 1.19e-7. Complete sin/cos stays within 9.07e-7 over
  the sampled −2π…4π range.
- With coordinates up to 100 A, Park arithmetic alone has maximum error
  12.59 uA for ordinary arithmetic and 10.96 uA with FMA. Including sin/cos,
  the measured maximum is 0.130 mA. Q15 reaches 10.51 mA; a 1-A zero-angle
  round trip becomes 0.996063 A. Its Hall-glitch simulation reaches 13.03 A
  true |Id| against the existing 12-A bound.
- A 100,000-step prescribed current-PI trajectory has approximately
  1.1–1.3 uV maximum error against the f64 reference. The velocity-filter
  trajectory reaches 0.000281 electrical rad/s error.
- The observer cached-versus-recomputed regression compares the torque
  filter with an older unfused oracle. Required FMA changes it by at most
  0.119 uA in the trajectory. Updating that diagnostic oracle to FMA restores
  bitwise agreement. This comparison exercises coefficient caching with the
  current observer algorithm on both sides.
- Separate optimized MCU observer trajectories execute 56,000 identical input
  updates, including variable dt, reversal, reset and configuration changes.
  FMA/algebraic maximum phase changes are 1.19e-6 / 3.34e-6 electrical rad,
  and velocity changes are 0.000244 / 0.000305 electrical rad/s. Numerical
  maxima cover every fourth state plus readiness transitions. Readiness agrees
  at every update. Host-debug algebraic agreement does not predict MCU rounding.
- Assembly/core sqrt agree bitwise on non-NaN results from a 500,011-input-pattern
  MCU sweep, with matching NaN classifications. atan2's existing approximation
  error reaches 0.01015 rad, while the maximum FMA change is 2.38e-7 rad over
  65,537 angles at four input scales.
- PI-FMA's bus-clamp simulation exceeds the existing requested-minus-measured
  2-A bound for nine 50-us steps. At the 3.366-A maximum, request/grant is
  1.116 A and measured Iq is −2.250 A. Maximum requested-minus-granted current
  is only 0.478 mA; peak integral is 1.517 A in both baseline and PI-FMA.
  Speed settling/overshoot stay within that scenario's bounds. The current
  transient and the original anti-windup intent require separate decisions.

`algebraic_*` permits reassociation and broader changes to zero/nonfinite
behavior. The finite-input measurements above bound the sampled workloads;
the API also permits other results. Required FMA fixes one rounding, while
relaxed multiply-add permits one or two.
[Rust floating-point contracts](https://doc.rust-lang.org/nightly/core/primitive.f32.html#algebraic-operators)

Artifacts are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-arithmetic-2026-11-15/`.
`timing-summary.csv`, `pc-summary.json`, per-image `build.json` and raw captures
identify source revisions, ELF hashes, flags and rejected attempts. MCU kernels
live in `tests/stm32f405/tests/support/arithmetic.rs`; prescribed observer traces
use `observer_trajectory.rs`. Gross kernel cycles include loop/input overhead.
Per-invocation opaque PI coefficients distinguish an ISR step from loop hoisting.

## Function placement experiments, Nov 15

Each address sweep compiles once and relinks the captured final LLD inputs.
The instructions, register choices, constants and function sizes stay fixed.
Emitted relocations account for every changed executable/data byte. Other
allocated section addresses stay fixed within each single-function sweep.
Call sites and external tail branches receive their required relocations.

The table shows mean full-ISR body time at +450, in us, for one base address.
Each family uses ordinary arithmetic with the selected function extracted into
its own section. Extraction can move the remaining `.text` contents relative to
the original baseline. Comparisons between offsets within one family isolate
placement using identical objects. Offsets are the entry address modulo 16.

| Moved function | Bytes | Offset 0 | Offset 4 | Offset 8 | Offset 12 |
|---|---:|---:|---:|---:|---:|
| `grant_torque`, base `0x0807E000` | 736 | 38.035 | 37.989 | 38.074 | 38.187 |
| `run_foc_cycle`, base `0x08070000` | 30072 | 37.975 | 37.822 | 38.188 | 37.956 |
| `update_phase_with_applied_voltage`, base `0x08074000` | 13200 | 38.193 | 38.315 | 38.094 | 38.085 |
| ADC ISR, base `0x0807B000` | 3680 | 38.393 | 38.264 | 38.288 | 38.188 |

The second bases are respectively `0x0807D000`, `0x08078000` and `0x0807A000`.
Matching offsets agree within 0.009 us across these bases. The grant differences
agree within 0.001 us. Best/worst exact images also have repeat holds, +120
holds and separate PCSR profiles. Entry alignment changes the position of every
internal block and literal pool. The best entries here are at offsets 4, 4, 12
and 12 respectively. ADC uses one base and directly selects its existing
`.text.ADC` input from the baseline objects. The original baseline ADC entry
already has offset 12.
Further grant moves by 16/64/256/1024 bytes preserve the ranking. Offset 4
measures 37.980–37.989 us and offset 12 measures 38.186–38.191 us across
those four bases.

F405 Flash reads 128-bit lines. ART retains 64 instruction lines and eight data
lines for literal loads, with LRU replacement. Branch misses and literal fetches
can stall execution. The observed 16-byte sensitivity is consistent with that
fetch structure. The DWT study below measures the associated stall costs.
Cache-index mapping and individual miss counts remain unmeasured.
[ST RM0090, section 3.5.2](https://www.st.com/resource/en/reference_manual/DM00031020.pdf)

### Controlled ART/cache fixture

Twelve embedded-test ELFs relink one object set at bases `0x08050000`,
`0x08050010` and `0x08050400`, each with offsets 0/4/8/12. Three repeats cover
all eight combinations of prefetch/I-cache/D-cache, three current requests
and warm/cold/pressure conditions. GPIO readback confirms the gate driver and
phase outputs remain disabled. Five Flash wait states are retained.

The real torque-limiter kernel is instantiated with inert IO and a manual phase
source. Normal current steps first prepare active bus limiting. This instance
has 732 bytes versus the native driver's 736 bytes. Warm windows contain 64
calls. Cold windows reset caches before one timed call. Pressure windows execute
96 branch lines and 64 volatile Flash-data reads before one timed call. Cache
maintenance and pressure generation are outside the measured window.

For the +0.4-A request, gross CPU-cycle ranges across the twelve placements are

| Condition | Prefetch + I-cache + D-cache | All three disabled |
|---|---:|---:|
| Warm | 181.13–183.13 | 341.17–369.17 |
| Cold | 241–253 | 347–375 |
| After pressure | 253–261 | 364–378 |

With I-cache/D-cache enabled and prefetch disabled, warm timing is 181.14 cycles
at every placement. Cold timing becomes 342–363 cycles. A warm isolated loop
therefore hides much of the placement sensitivity seen in the full ISR.
Bitwise outputs agree across every placement, cache mode and repeat for all
three requests. The 864 case summaries and 36 run manifests are in
`art-summary.csv` and `art-b*/manifest.json` under the arithmetic artifact root.

### Short-window DWT event profiling

The tested F405 exposes the optional DWT profiling counters (`DWT_CTRL`
initially `0x40000001`). CPICNT records extra instruction/fetch cycles, LSUCNT
records extra load/store cycles, and EXCCNT records exception overhead. Each
event counter is eight bits. The observer executes from ordinary SRAM and
encloses every event-counter read inside a CYCCNT pair. Disassembly verifies
the SRAM addresses, measured call and read order. The empty fixture costs
75 cycles, including CPI 7 and LSU 35. Reported values retain this overhead.
[Arm Cortex-M4 TRM, chapter 9](https://documentation-service.arm.com/static/5fce431be167456a35b36ade)

CPI and LSU count mutually exclusive extra cycles. For observed residues
`cpi` and `lsu` inside `cycles`, `cpi + lsu + 256 > cycles` proves that neither
counter wrapped within its read interval. Other windows carry an explicit
ambiguity flag. SRAM calibration uses 0/1/8/32/64/128 repeated UDIV or LDM
instructions. Longer cases deliberately wrap and are flagged. For example,
32 divides produce 468 cycles and CPI residue 105, representing 361 CPI
cycles. Shorter cases verify the expected progression before wrap.
[Armv7-M architecture, section C1.8](https://www.pjrc.com/teensy/DDI0403Ee_arm_v7m_ref_manual.pdf)

Four ELFs place the same 732-byte fixture at `0x08050000 + {0,4,8,12}`.
Three repeats cover eight cache modes, four callbacks and warm/cold/pressure
conditions, totaling 36,864 single-call windows plus 4,608 calibration windows.
Every fixture window has zero exception count and a proven unwrapped CPI/LSU
pair. Outputs agree bitwise across placements, modes and repeats.

Mean gross cycles for the +0.4-A fixture, shown as CYCCNT / CPI / LSU

| ART condition | Entry offset 0 | Entry offset 8 |
|---|---:|---:|
| All enabled, warm | 217 / 38 / 36 | 215 / 36 / 36 |
| All enabled, cold | 277 / 88 / 46 | 266 / 84 / 39 |
| All enabled, after pressure | 277 / 88 / 46 | 271 / 84 / 44 |
| All disabled, warm | 375.96 / 185.96 / 47 | 348 / 163 / 42 |

The cold 11-cycle difference decomposes into four CPI and seven LSU cycles.
The disabled-cache difference decomposes into about 23 CPI and five LSU cycles.
CPI includes multicycle instruction costs, so translating these totals into
an ART miss count requires additional information about individual fetches.
The full matrix and exact ELF hashes are in `fixture-counters.csv` and
`offset-*/manifest.json` under the DWT artifact root. `source-hashes-fixture.json`
identifies the modified source inputs used for those images.

The native `bench-torque-counters` feature records 4096 torque-grant windows
after 30,000 driven warm-up calls. Raw CYCCNT and packed CPI/LSU/EXC/FOLD
residues are read over SWD after terminal Stop. Four native ELFs relink one
object set at `0x0807E000 + {0,4,8,12}`, retaining all other allocated addresses.
Differences are verified relocations and the decoded destination of LLD's
SRAM-to-Flash veneer. The native kernel has 736 bytes and matches the earlier
grant-placement kernel except for its relocated braking-helper call.

Native 15-kHz holds at +120/+450 produce the following +450 means

| Entry offset | Window cycles | CPI | LSU | FOLD | Full ISR mean, us |
|---|---:|---:|---:|---:|---:|
| 0 | 264.66 | 90.01 | 34.55 | 1 | 38.793 |
| 4 | 256.58 | 84.59 | 30.91 | 0 | 38.735 |
| 8 | 270.33 | 89.83 | 39.33 | 0 | 38.825 |
| 12 | 288.87 | 103.37 | 45.33 | 1 | 38.934 |

The +120 means are within 0.11 window cycles of the corresponding +450 values.
Offset 4 beats offset 12 by 32.29 cycles (about 0.192 us), consistent with the
full-ISR difference of 0.199 us. Their CPI/LSU difference totals 33.21 cycles,
while offset 12 folds one more instruction. The earlier clean 20-kHz grant
sweep measures a 0.198-us difference between these offsets at the same base.
All 32,768 main-matrix windows
prove unwrapped CPI/LSU counts and record zero exception overhead. A separate
offset-0 +450 repeat agrees within 0.021 cycles. The largest observed full ISR
body in this matrix is 48.482 us, with body-over-budget count zero. These are
instrumented 15-kHz images. The earlier clean 20-kHz placement results retain
their own measurement provenance. Raw windows, capture acceptance and hashes
are under `native-runs/`; `native-counters.csv` summarizes them.

Two further +450 holds set `ACTLR.DISFOLD`, retaining the same ELFs, followed
by reset/default-control repeats. FOLD becomes zero for both offsets 4/12.
The offset-4 window stays near 256.52–256.59 cycles. Offset 12 changes from
288.87–288.96 to 288.33 cycles, with CPI falling to 102.03 and LSU to 45.16.
Whole-ISR means increase by 0.09–0.10 us at offset 4 and 0.08–0.09 us at offset
12. Global folding control changes the surrounding pipeline/cache timing as
well as the folded instruction count. The default-control repeats preserve
the placement ranking. All 53,248 native windows across 13 holds have proven
unwrapped CPI/LSU counts and zero exception overhead. The largest observed
body is 48.750 us, with no body-budget exceedances.

The rebuilt current native source matches every allocated byte/address of
the measured offset-0 image, including the SRAM recorder. Source hashes and
comparison evidence are in `current-source-equivalence.json`. A recovery image
at 15 kHz is restored after every hardware batch. Final live readback in
`final-hardware.json` confirms Stopped, zero active faults, phase PWM enables
off, all ART options enabled and default ACTLR controls.

### Grouping and linker ordering

Seven ordering policies relink the baseline object set. The control matches
every allocated byte/address of the original baseline. A separate object set
groups the three selected functions before `.rodata`. Its 45056-byte reservation
holds 44008 bytes of function code and 1048 bytes of padding. All other allocated
section addresses stay fixed across its three policies. Function bodies match
the baseline except for their relocation fields.

| Policy | Mean +120, us | Mean +450, us |
|---|---:|---:|
| Baseline order control | 38.617 | 38.193 |
| ADC → cycle → phase → current → grant → velocity | 38.684 | 38.300 |
| ADC → grant → current → velocity → phase → cycle | 38.751 | 38.370 |
| Descending PC occupancy | 38.826 | 38.440 |
| Shuffle seed 1 | 38.571 | 38.138 |
| Shuffle seed 2 | 38.446 | 38.068 |
| Shuffle seed 3 | 38.859 | 38.418 |
| Compact cycle/phase/grant, all entry offsets 0 | 38.479 | 38.131 |
| Same group, offsets 4/12/4 | 38.158 | 37.819 |
| Reversed group, same per-function offsets | 38.151 | 37.820 |

The controlled group improves by about 0.31 us when its entry offsets change.
Reversing its order preserves essentially the same timing. The best measured group
saves about 0.37 us at +450 versus the baseline order control. Whole-program
sorting can also slow execution, including sorting directly by PC occupancy.
Placement choices need measurement against the final ELF and operating modes.
Placement policy adoption awaits review.

The best group's exact repeat measures 37.813 us at +450. Its largest observed
body across holds, repeat and reversal is 47.368 us. All three groups complete
+120 → +450 → −120 → −450 → Stop with the capture contract satisfied.
A separate PWM witness records 280114/280149 writes at +120/+450, minimum
margins 14.836/15.730 us and late/ambiguous/still-masked counts 0/0/0. That
witness has its own code generation and ELF: its cycle function is 32292 bytes,
compared with 30072 in the clean DWT image.

The arithmetic/placement study totals 175 accepted native timing windows and
48,861,077 driven ISR observations, with body over-budget count 0. Separate
PCSR captures cover 49 images and 3,859,136 driven samples. All seven ordering
and three grouping policies have profiles. Single-function sweeps profile
their measured extremes, with all four original grant offsets also profiled.
ADC offsets 0/12 show approximately 9.26/9.01 us of statistical ADC occupancy
per 50-us period, consistent with their clean DWT difference of about 0.20 us.
The general PCSR limitations in the tools section apply.

Final ordering/group/ADC captures use six Tokio workers with RTKit SCHED_RR
priority 5, a 200-ms continuous-RT-CPU limit, CPU weight 1000, 512-MiB MemoryLow
and MemorySwapMax=0. Setup/ELF parsing uses SCHED_OTHER with nice −10. Earlier
host-load attempts with late affirmations/raw loss are excluded by the capture
contract. MCU DWT measurement and the 150-ms deadman use the existing paths.

The qualified normal image is restored at 15 kHz, Stopped, zero active faults
and phase PWM enables off. Its SHA256 is
`46b75754b889bfe787740e9af9889e7439882a4942333940c8e8b98941e3729f`.
Final live readback is `final-hardware.json` in the arithmetic artifact root.

### Linked ADC callees and remaining placement candidates

The qualified `stock15/normal15/firmware.elf` identified above has 99 reachable
machine-function bodies including ADC, hence 98 callees. Of those, 60 belong to
OxiFOC. The 20-kHz baseline (`88043be3…d7287fe`) has the same named set plus
the recorder's `__aeabi_memclr8`. Both inventories use the existing exact ELFs.

The analysis follows BL/BLX calls, their conditional IT forms and cross-function
tail branches. All 31 TBB/TBH dispatch tables in each graph remain within their
owning function.
DWARF identifies 29 indirect call sites as Waker clone/wake operations. Their
targets match the single Embassy RawWaker vtable in the ELF. Panic formatting
has a separate conservative set of 14 callback/helper candidates. The vtable's
unused `waker::drop` member is listed separately. Symbol aliases share one body.

Complete lists are `stock15/linked-functions.csv` and `stock15/inventory.txt`
under the callgraph artifact root. Each entry gives address, bytes, entry modulo
16, callers, an ADC path, DWARF source, prior placement coverage and matching
20-kHz PC occupancy. `callback-candidates.csv` preserves dynamic-formatting
uncertainty. The `baseline20/` directory has the corresponding research image.

The earlier dedicated entry-offset sweeps cover ADC, `run_foc_cycle`,
`update_phase_with_applied_voltage` and `grant_torque`. The expanded series below
adds the following 14 bodies. Addresses and sizes refer to stock15, and prior
coverage describes the experiments preceding this series.

| Function | Address | Bytes | Relevant execution | Prior targeted placement |
|---|---:|---:|---|---|
| `FocController::step_with_injection` | `0x0804A0F0` | 1704 | Native current FOC | Six-function ordering |
| `FocDriver::step_scheduled_velocity` | `0x08034438` | 2348 | Every ISR in velocity/position mode, including inlined current-step work | Six-function ordering |
| `PhaseManager::blend_with_observer` | `0x08039788` | 712 | Sensor-to-observer source selection | Global ordering |
| `space_vector_pwm` | `0x08060998` | 600 | Native voltage-to-duty conversion | Global ordering |
| `PhaseManager::injection` | `0x08051938` | 444 | Current-step HFI selection, also called with HFI inactive | Global ordering |
| `FocDriver::update_velocity_request` | `0x08034D64` | 2404 | Due outer update, approximately 1 kHz | Global ordering |
| `angle_difference_general` | `0x08059580` | 528 | General angle-difference path | Global ordering |
| `wrap_angle_general` | `0x08059790` | 116 | General angle-wrap path | Global ordering |
| `FocDriver::grant_torque_constrained` | `0x0804BB38` | 260 | Two authority probes per due outer update | Global ordering |
| `PhaseManager::tracker_output_inner` | `0x08039B04` | 744 | Observer commutation-tracker modes | Global ordering |
| `FocDriver::grant_dissipative_brake` | `0x0804AE5C` | 3292 | Dissipative braking | Global ordering |
| `FocDriver::step_direct_voltage` | `0x080336B4` | 1768 | Direct-voltage mode | Global ordering |
| `compiler_builtins::mem::memcpy` | `0x08061D1E` | 660 | Shared copy helper | Global ordering |
| `embassy_executor::raw::waker::wake` | `0x08053E94` | 256 | ISR notifications and foreground wakeups | Global ordering |

The first four rows have substantial occupancy in the existing +450 profile.
`step_scheduled_velocity` and `step_with_injection` account for about 4.63 and
4.28 us of sampled occupancy per 50-us period, respectively. Blending and SVPWM
account for 0.97 and 0.65 us. These are exclusive linked-body PC samples from
80,000 driven whole-MCU observations. Shared helpers can include foreground
execution. The tracker and dissipative-braking bodies have zero samples in
that hold and require their own operating scenarios.

The inlined current-control helper, current PI, bounded sin/cos, Park transforms
and observer arithmetic already occupy their callers' sections. Their separate
placement would first require new outlining and another code-generation study.
A source method can have both inline instances and an out-of-line body; the
inventory covers the linked calls that survive in these two ELFs.
The stock `memcpy` entry has residue 14, so two-byte entry increments are also
relevant for helpers whose input section permits that alignment.

### Expanded individual sweeps

The selected scope is the 14 candidates in the table above. Their existing
compiler objects produce 64 placement ELFs at `0x08070000 + offset`. Twelve
functions admit offsets 0/4/8/12. The `memcpy` and Waker input sections admit
all eight even offsets from 0 through 14.

The linker input clones the selected function section into its own output and
retains a zero-filled hole of the original size at the old address. Every other
allocated section keeps its baseline address and size. An audit of all 64 ELFs
confirms that byte differences consist of the old hole and relocations involving
the selected function. Moved-body changes are checked against those relocations.
This uses the existing baseline object set and requires no source recompile.

The original recorder enables windows only in current/velocity mode. Four
derived direct-voltage images extend its predicate with a one-byte immediate
change at `0x080005CE`, from `cmp r1, #2` to `cmp r1, #5`. DWARF confirms the
relevant discriminants and the preceding subtraction. Stopped remains excluded.
The separate motion-timing predicate and all control instructions retain their
original bytes. Parent images, hashes and the exact byte audit are preserved.

Native +120/+450 holds cover the common path. Separate scenarios exercise the
observer tracker, dissipative-braking authority and direct-voltage control.
Four PCSR captures cover the three special scenarios and a measured combined
layout. They qualify execution of the 14 selected bodies separately
from clean DWT measurements. All 64 offsets retain native timing coverage.
Exact images, section audit and raw attempts are under the callee-layout
artifact root.

The results below give whole-ISR mean ranges at +450, in us. Each row compares
offsets in its own operating scenario. The direct-voltage row uses the paired
14-s confirmation; the other rows use the initial screening. Tracker measurements
use Observer startup and hold for 14 s. Dissipative braking uses the velocity
PI's negative authority probe with RAM-only `max_dissipation_w=0.05` and
`bus_regen_max_a=0`. Direct voltage applies `vq=0.03 V`, `vd=0`, angle zero for
3 s. The remaining rows use normal HallToObserver velocity holds for 14 s.

| Moved body | Best entry modulo 16 | Mean range, us | Spread, us |
|---|---:|---:|---:|
| `step_with_injection` | 4 | 38.189–38.298 | 0.109 |
| `step_scheduled_velocity` | 0 | 38.188–38.251 | 0.064 |
| `blend_with_observer` | 0 | 38.189–38.260 | 0.070 |
| `space_vector_pwm` | 12 | 38.152–38.187 | 0.035 |
| `injection` | 0 | 38.189–38.236 | 0.046 |
| `update_velocity_request` | 12 | 38.192–38.203 | 0.011 |
| `angle_difference_general` | 0 | 38.185–38.208 | 0.023 |
| `wrap_angle_general` | 12 | 38.178–38.188 | 0.011 |
| `grant_torque_constrained` | 8 | 38.184–38.190 | 0.005 |
| `tracker_output_inner` | 8 | 34.979–35.043 | 0.064 |
| `grant_dissipative_brake` | 12 | 38.441–38.449 | 0.008 |
| `step_direct_voltage`, confirmed 0/8 pair | 8 | 30.054–30.228 | 0.174 |
| `memcpy` | 2 | 38.186–38.194 | 0.008 |
| `wake` | 12 | 38.188–38.191 | 0.003 |

The common-path matrix contains 104 windows and 29,132,704 driven observations,
with body over-budget count zero and maximum 48.363 us. The special scenarios
show three single-cycle Observer startup exceedances and 933 exceedances in
the dissipative-authority scenario. Matching unmoved baseline captures show
one Observer exceedance, maximum 54.667 us, and 216 dissipative-authority
exceedances, maximum 52.629 us. These captures retain zero fault-generation
changes. Placement gives little improvement to the dissipative solver's cost.

Direct-voltage paired repeats give 29.8293/29.6549 us at offsets 0/8 over 3 s,
and 30.2281/30.0542 us over 14 s. Both pairs support a placement difference of
about 0.174 us. The shared execution-time level varies by about 0.40 us between
captures; its cause remains unisolated. The initial 0.572-us screening spread
therefore overstates the reproducible placement effect.

Three simultaneous 14-body groups reserve 16160 Flash bytes while preserving
all original addresses and holes. The chosen offsets minimize the average of
the available +120/+450 means, with one scenario for each special-mode body.
The reverse group uses the same offsets in reverse function order.

| Whole-image candidate | +450 mean, us | +450 max body, us |
|---|---:|---:|
| Unmoved baseline, two controls | 38.189–38.195 | 47.558 |
| 14-body group, original residues | 38.184 | 46.992 |
| 14-body group, selected residues | 38.136 | 46.677 |
| Selected residues, reverse order | 38.137 | 47.493 |
| Selected group, independent repeat | 38.135 | 47.040 |
| Earlier compact cycle/phase/grant, fresh repeat | 37.822 | 47.725 |

The selected group also records 38.572 us at +120 and completes
+120 → +450 → −120 → −450 → Stop, with largest body 47.654 us. These common
controls and repeats have zero body over-budget samples. The two fresh baseline
means differ by 0.00554 us, putting the smallest individual spreads near the
observed run-to-run variation.

The earlier compact three-body candidate retains the strongest measured
velocity result. Among the added common-path candidates, SVPWM offset 12
improves both +120 and +450 by about 0.033–0.035 us against offset 0 at the same
base. The current-step and velocity-wrapper baseline residues already match
their best measured +450 residues. The four PCSR captures confirm all 14
selected bodies with 317312 driven whole-MCU samples. Shared-helper counts
include foreground execution. PC sampling and clean timing use separate runs.

The expanded primary matrix totals 116 timing windows and 31,613,763 driven
observations. Raw attempts, four direct-voltage repeats, eleven control windows,
ELF SHA256 values and the per-body PC coverage remain in the artifact root.
Final live readback in `final-hardware.json` confirms the qualified image at
15 kHz, Stopped, zero faults, phase PWM enables off and restored current limits.

### Compact-layout arithmetic follow-up, November 16

The new ordinary-arithmetic build has byte-identical allocated sections and
addresses to the earlier `compact-best` image. Four source policies and the
observer/atan pair use that compact cycle/phase/grant layout at residues 4/12/4.
Required FMA means `core::f32::math::mul_add` in the selected expressions.

A second linker layout assigns common slots to 454 main-object function
sections and 913 constant sections, keeping the three hot functions compact.
The linked-symbol audit confirms identical addresses for all 492 functions
across the baseline, four single policies and combined policy. Slot capacities
cover the largest body in the single-policy builds. Each ELF has a disassembly,
source hashes and SHA256 manifest. The fixed-slot baseline reproduces the
ordinary compact timing within the observed run-to-run variation.

Short screening uses six-second +450 holds. Values below are whole-ISR means
in us, compared within each column.

| Arithmetic policy | Original compact layout | Fixed function addresses |
|---|---:|---:|
| Ordinary baseline | 37.776 | 37.776 |
| PI integration/output FMA | 37.780 | 37.772 |
| Observer algebraic | 37.906 | 37.903 |
| atan2 polynomial FMA | **37.641** | **37.644** |
| Core sqrt intrinsic | 37.948 | 37.914 |
| Observer algebraic + atan2 FMA | 37.856 | 37.849 |

Independent 14-second holds confirm the atan2 candidate.

| Layout and policy | +120 mean, us | +450 mean, us | Reversal max body, us |
|---|---:|---:|---:|
| Original compact, ordinary | 38.154 | 37.816 | 47.892 |
| Original compact, atan2 FMA | **38.023** | **37.682** | 49.078 |
| Fixed slots, ordinary | 38.156 | 37.818 | 48.118 |
| Fixed slots, atan2 FMA | **38.020** | **37.683** | 47.475 |

All four validation images complete +120 → +450 → −120 → −450 → Stop.
The gain is about 0.13–0.14 us, approximately 22 CPU cycles per ISR. The
32 accepted native timing windows contain 5,602,630 driven observations and
zero body over-budget samples. One earlier reference attempt with a late host
affirmation warning and changed fault generation is retained as rejected and
was repeated. Valid captures retain unchanged fault generation.

Two opcode controls distinguish the instruction semantics from local code
generation. Replacing three PI or six atan VMLA/VMLS sites with same-size
VFMA/VFMS instructions in the ordinary compact image gives 37.776/37.786 us.
Conversely, replacing all eight atan VFMA/VFMS sites in the faster image with
VMLA/VMLS, preserving every other byte, gives 37.649 us in the original layout
and 37.641 us with fixed slots. The gain survives restoration of two-rounding
MAC instructions. This points to the new local code generation and instruction
placement as the source of the improvement.

The phase-update body grows 13200 → 13260 bytes. Its static stack-operation
count stays at 91, while 32-bit instructions crossing a 16-byte line decrease
333 → 303. These counts cover the whole body, including conditional paths.
Exact fetch-stall attribution remains open. Two separate PCSR captures contain
155904 driven whole-MCU samples and show phase-body occupancy of about
10.435/10.266 us per 50-us period for ordinary/atan-FMA fixed-slot images.
The core-sqrt phase body retains four VSQRT instructions and adds two static
FP stack stores and one FP stack load.

Fresh execution of the existing `tests/stm32f405` arithmetic benchmark image
checks kernel cycles and numerical error with phase PWM and gate enables off.
The isolated atan fixture measures 72.187/73.119 cycles for ordinary/FMA.
Across 65537 angular samples, FMA changes atan output by at most
2.3841858e-7 rad, approximately 0.0000137 degrees. Maximum error against the
reference is 0.010149727/0.010149709 rad. The sqrt comparison finds identical
non-NaN bits and NaN classifications across 500011 inputs. The PI trajectory
fixture measures about 1.319/1.080 microvolts of maximum error against its
wide reference for ordinary/FMA over 100000 steps.

PI FMA also changes the virtual-motor cruise bus-clamp result. With a 0.5-A
bus limit, peak requested-minus-delivered current reaches 3.366 A against the
existing 2-A acceptance bound. The speed and overshoot bounds still pass.
The PI images remain in the hardware evidence despite this numerical-contract
failure. Their timing difference is within the reference-repeat variation.

The recommended F405 candidate is the compact layout with atan2 FMA.
The candidate ELF SHA256 is
`c6ff6062644f424f0000bc6641dc198a1145ba113b818e8a96abfc72addae12a`.
The qualified 15-kHz image is restored after the series, Stopped, zero active
faults and phase PWM enables off. `final-hardware.json` records the live
readback and restored current limits.

### Final compact-layout/atan2-FMA qualification, November 16

The candidate is integrated on top of main `6a52d74`. MK5 selects core features
`compact-foc` and `atan2-fma`. Named function sections and `foc-hot.x` group
cycle → phase update → torque grant with entry residues 4/12/4 modulo 16.
The linker uses the actual body sizes without fixed-slot reservations.
RAM-FOC builds retain their existing section policy. The atan2 policy uses
required FMA through the nightly core API on the MCU and `f32::mul_add` on
the host. The final disassemblies contain eight VFMA sites.

Three exact ELFs cover normal release, DWT body timing and the PWM deadline
witness. Each runs at both frequencies through the existing saved PWM setting
and restart. There is no compiled 20-kHz override. TIM1/ADC readback confirms
the carrier, sampling position, injected ranks and deadtime at each rate.
TIM5 readback confirms a running unprescaled counter at 83,917,932 Hz,
half the TIM1 rate used to decode deadline margins.

Sixteen windows cover a 14-second +450 hold, current-control start, +120 →
+450 → −120 → −450, Coast → +120 restart, Stop, and a deliberate 0.4-second
host pause. The diagnostic active-window predicate now includes Coast, so
the same frozen window contains the subsequent restart. Normal release
omits this diagnostic predicate.

| Nominal PWM | +450 mean body, us | Largest body across scenarios, us | Minimum PWM-write margin, us |
|---|---:|---:|---:|
| 15 kHz | 37.709 | 47.719 | 31.984 |
| 20 kHz | 37.612 | 47.386 | 15.170 |

The body recorder covers 959,471 observations with zero over-budget bodies.
The witness covers 941,944 phase-enabled PWM writes with zero late writes,
ambiguous entry phases or writes still masked by UDIS. Ordinary captures
contain 2,224,935 rows with complete raw sequences and END totals, all command
ACKs, terminal Stop and unchanged fault generation. Each intentional host
pause raises fault generation, reaches Stopped and retains successful command
ACKs and terminal Stop in Parquet metadata. Expected transport loss from these
six injected pauses is recorded separately.

Body and PWM measurements use separate diagnostic ELFs. The ordinary release
is checked functionally. All three retain the measured hot-entry residues,
while their absolute addresses differ and PWM instrumentation enlarges the
cycle body from 30,072 to 32,292 bytes. PWM margins measure time remaining
after the commit hook until the next preload deadline. Body duration excludes
the final diagnostic bookkeeping and exception return.

The normal-release candidate SHA256 is
`c9e867ea88e63a6fbcf0813e12f8f44a98b71cbff24957073b885e70b4d8c56d`.
`summary.json` and per-image `build.json` files retain all three hashes,
source hashes, exact addresses, features and disassemblies. The initial
20-kHz configuration attempt was rejected because PWM settings require
persistence and restart. It is retained outside accepted measurements.

These results support the compact layout plus atan2 FMA for the measured
HallToObserver scenarios. The earlier 52–55-us Observer-startup and
dissipative-authority peaks remain the next investigation. After the series,
the qualified pre-experiment image is restored at saved and active 15 kHz,
Stopped, zero active faults and phase enables off. Current limits and live
readback are recorded in `final-hardware.json`.

### Rare startup/braking paths and RTT-only logging, November 16

Production-equivalent control ELF `6d6e8a93…` reproduces a 54.071-us Observer
startup body and 51.747–53.213-us braking peaks. A later exact-ELF repeat gives
37.621 us in the ordinary +450 hold and 37.867 us with dissipative authority,
with a braking maximum of 52.623 us. These controls use native 20-kHz recording.

Detailed per-event markers locate the startup cost in `SeedAndHandoff`.
One instrumented event spends 26.633 us in startup, including 21.384 us in
the reseed warning, 2.556 us in the confirmation probe and 1.782 us reseeding
the observer. These nested measurements identify contributions inside a
profiling build. Clean body maxima come from separate minimal-timing ELFs.

The negative torque-authority probe takes 5.708 us in the detailed braking
profile, including 3.754 us in the scalar winding solver. The positive probe
takes 0.429 us. The complete due-only velocity update takes 10.445 us.
Slow cycles also coincide with protection refresh or longer Hall sampling.
A retained counter records 7,897 solver calls and **zero current-circle
bisection searches** in an eight-second braking scenario. The fixed 16-step
search contributes code size here, while the executed endpoint calculations
and their callers account for the measured solver cost.

| Braking candidate, 20 kHz | Mean body, us | Largest body, us | Outcome |
|---|---:|---:|---|
| Research source control | 38.228–38.238 | 54.845 | Control for the first compiler variants |
| Forced dissipative-helper inlining | 38.350–38.366 | 52.611 | Slower mean |
| Outlined 16-step circle search | 38.008–38.017 | 52.587 | Caller shrinks, peaks persist |
| q-only authority prototype | 38.048–38.050 | 51.580 | Fewer overruns, added ordinary-path dispatch |
| RTT-only, cleaned diagnostic gating | 37.853–37.864 | 52.438 | Logging change preserves braking cost |
| RTT-only + q-only prototype | 37.881–37.897 | 51.485 | 35 overruns across two holds, versus 250 for RTT-only |

The q-only prototype computes the same available iq while omitting an id
result that the authority caller discards. In `tests/stm32f405`, its real-motor
fixture measures 545.392 cycles per call versus 758.799 for the frozen full
solver, including fixture/loop costs. Full and projected q outputs have zero
bit differences across 30,000 sampled parameter sets. Both ordinary and
outlined MCU builds repeat this check. The outlined circle fixture gets
slower, 1465.594 → 1537.973 cycles for the full grant.

Assembly shows 3292 → 1964 bytes in the outlined grant, with a separate
1442-byte, unrolled circle helper. The q-only helper is 2876 bytes and removes
two static divisions and three square roots relative to the full grant.
Its plain paths add dispatch/tail calls into `grant_torque_constrained`.
The cleaned whole-image ordinary mean worsens 37.606 → 37.718 us with this
prototype. It remains experimental. The follow-up below evaluates this
dispatcher, paired authority calculations and additional PI deferral.

Early new profiling marks changed ordinary code generation even when their
counter operations optimized away. Explicit `cfg(isr-profiling)` now excludes
the new marks and enlarged section snapshots from normal builds. The cleaned
cycle body returns to 30,072 bytes and the ordinary mean to 37.606 us. The
earlier compiler controls are retained as a separate series in `summary.json`.

#### Logger ownership and measured output cost

The pinned ergot logger holds a `critical-section` guard from `acquire` through
`release`, protecting its shared encoder, RTT channel and network accumulator.
On this single-core Cortex-M build the guard uses PRIMASK. The stock
`rtt-target 0.6.2` defmt logger also serializes the entire frame this way.
Its incremental encoder callbacks repeatedly construct/commit RTT writes,
including SeqCst cursor operations and their DMB instructions.

| Warning path | Cycles | Time, us |
|---|---:|---:|
| ergot RTT + network, detailed event | 3589 | 21.384 |
| Stock RTT-only, three retained events | 2768 | 16.492 |
| Same RTT-only ELF, channel disabled in RAM, two events | 932 | 5.553 |

The RAM control changes only the stopped logger's channel-present byte.
It preserves code, addresses, encoder and warning call. The measured difference
is 1836 cycles, 10.939 us, for the RTT output path in these events. The disabled
output control is diagnostic evidence. Reset restores the channel.

The F405 RTT-only trial enables `rtt-target/defmt`, removes ergot's sink features
and forwarding task, and retains the existing RTT channel setup. Commands and
raw telemetry continue through their existing transports. Defmt messages in
this trial are available through RTT. Probe reads occur after Stop and use
`--rtt-channel-mode no-block-skip`.

The cleaned RTT-only startup maximum is 49.942 us at a measured 49.989-us PWM
period. Body duration excludes final bookkeeping and exception return.
A separate witness sees zero late PWM writes across 840,357 writes. Minimum
margins are 14.860 us in Observer startup, 11.297 us with dissipative authority
and 14.705 us in the ordinary hold. The exact normal-release ELF also completes
Observer startup at 15/20 kHz and current/velocity/reverse/Coast/Stop transitions
at 20 kHz, with lossless captures and unchanged fault generation.

#### Removing the logging critical section

`defmt-rtt 1.3.1` provides `drop-on-contention` together with
`disable-blocking-mode`. It identifies the execution context through IPSR,
claims the encoder with atomic compare-exchange, stages a complete frame and
publishes the write cursor at release. A colliding context or full buffer drops
the whole frame. This contract fits one cooperative thread-mode context plus
interrupts. Its own RTT control block requires a separate integration choice
for the project's multi-channel RTT diagnostic profiles.

A Cortex-M4 compile and linked assembly audit find zero CPSID/PRIMASK operations
in `_defmt_acquire`, `_defmt_write` and `_defmt_release`. Acquire contains
LDREX/STREX and an IPSR read. This backend has **not been run on the board**.
[defmt-rtt backend contract](https://docs.rs/defmt-rtt/1.3.1/defmt_rtt/)

A custom alternative gives one foreground task exclusive ownership of encoder
and RTT writer, with fixed-size events delivered from ADC through an SPSC
queue. Additional interrupt producers need their own queues or an MPSC design.
Queue capacity and dropped-event accounting become explicit. A foreground
writer using the current stock global logger would still take its PRIMASK guard.

The combined candidate queues ADC events for an ordinary Embassy thread-mode
task, which emits defmt through the atomic, nonblocking backend. Other direct
interrupt log calls use its drop-on-contention behavior. ADC publishes owned
parameters and an event timestamp, while the task handles encoding and output
in bounded batches below protocol work. The queue and wake path need an
implementation without a CriticalSectionRawMutex. Queue overflow and backend
frame loss are separate outcomes. This combined design is still a proposal.

The artifact root contains 73 accepted timing/witness windows, exact image
hashes, raw attempts, source hashes and disassemblies. One full-rate detailed
profile lost producer samples and is retained as rejected. Detailed attribution
then used 5-kHz recording with the ISR still at 20 kHz. Final readback restores
the committed main image at 15 kHz, Stopped, zero active faults and phase enables
off. The RTT-only and torque prototypes remain on the experimental branch.

#### Deferred ADC logger, November 16

The F405 logger uses a locally patched `rtt-target` 0.6.2.
`write_frame` always selects nonblocking, whole-frame output and ignores
debugger changes to channel mode. Application routing lives in `logging.rs`.
ADC serializes into a 2-KiB `Texas<_, MaiNotSpsc>` queue with a 256-byte maximum
raw record. One thread-mode worker encodes and outputs each record. Ordinary
direct logging and fatal logging have separate encoders with 512-byte output
buffers. Atomic try-acquire drops contending direct calls. Panic selects fatal
mode after gate shutdown, so a panic inside ADC bypasses the queue. An occupied
RTT writer can still lose a fatal message. `LOG_STATS` counts queued records,
queue overflow, oversize records, contention, full RTT and uninitialized output.

| Measurement | Stock RTT-only | Deferred ADC |
|---|---:|---:|
| Actual startup warning, CPU cycles | 2,768 | 1,285 in both recorded events |
| Observer startup maximum body, us | 49.942 | 43.364 |
| Ordinary +450 mean body, us | 37.606 | 37.723 |

The warning comparison uses detailed event markers. Body measurements use
separate minimal-timing images. The ordinary mean increases by 0.117 us in this
hold. Function addresses move while the compact hot-section entry residues
remain 4/12/4 and the cycle function stays 30,072 bytes. The normal image adds
2,960 bytes of `.text` and 3,840 bytes of `.data` plus `.bss` against stock RTT-only.

Two embedded-test runs measure 1,128.6–1,128.8 mean cycles for a two-float ADC
warning, including queue publication with a waiting consumer. Real-IRQ fixtures
exercise queue overflow, oversized records, preemption, contention and fatal
bypass with PWM disabled. The subsequently added nested-fatal fixture still
needs its hardware rerun at this checkpoint.

Normal-image Observer starts at 15/20 kHz and the 20-kHz current, velocity,
reverse, Coast and Stop sequence complete with lossless captures and unchanged
fault generation. Separate 20-kHz PWM witness windows record 320,131 enabled
writes, zero late writes and a minimum margin of 15.479 us. Running captures
report zero logger losses. The board is restored to the committed image at
15 kHz, Stopped, zero active faults and phase enables off.

The F405 firmware uses this deferred RTT backend. Artifacts, exact image
hashes and assembly audits are in
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-deferred-logging-2026-11-16/`.

### Dissipative authority follow-up, November 16

Control source `ecc4920`, nonincremental MK5 release builds. Each native
window covers a 14-second, no-load +450-rad/s HallToObserver hold at 20 kHz.
The authority scenario sets RAM-only `max_dissipation_w=0.05` and
`bus_regen_max_a=0`. The ordinary hold retains zero dissipation. All captures
require lossless Packed28 telemetry, acknowledged commands, Stop and unchanged
fault generation. A fresh control build reproduces the original control ELF
SHA-256 `e1609594…` byte for byte. The q-only image is `b972720f…`.

| Candidate | Ordinary mean, us | Authority mean, us | Authority maximum, us | Bodies over period per 100k |
|---|---:|---:|---:|---:|
| Control | 38.288 | 38.538 | 53.237 | 83.4 |
| q-only, compiler-selected function boundary | 38.303 | 38.469 | 51.675 | 10.7 |
| Inlined dispatch, outlined q solver | 38.338 | 38.522 | 51.783 | 18.6 |
| Fully inlined q path | 38.300 | 38.489 | 51.461 | 19.1 |
| Paired linear bounds, full nonlinear fallback | 38.426 | 38.706 | 54.238 | 362.8 |
| Paired bounds with q-only fallback | 38.302 | 38.523 | 52.343 | 42.8 |
| Direct motoring bound and nonlinear q call | 38.307 | 38.506 | 50.972 | 10.7 |
| Direct pair plus exact zero-root shortcut | 38.301 | 38.492 | 52.116 | 11.1 |
| q-only plus outlined circle search | 38.393 | 38.570 | 51.175 | 16.1 |
| Defer due PI after /256 derating refresh | 38.387 | 38.646 | 53.296 | 100.0 |
| Same deferral plus q-only | 38.408 | 38.587 | 52.819 | 24.3 |
| Defer due PI during a deadman poll | 38.489 | 38.738 | 53.755 | 70.3 |
| Same deferral plus q-only | 38.498 | 38.655 | 52.170 | 11.8 |

Control, q-only and the fully inlined q path each have two independent
windows per mode. Means are weighted by native cycle count. Rates normalize
the different repeat counts.
Single-window differences of a few hundredths of a microsecond have limited
ranking value. The measured period is approximately 49.989 us.

The q-only variant reduces exceedances from 467/560191 to 60/560246 cycles,
an 87% rate reduction. Those 60 events span approximately 28 seconds of
active operation, about 2.1 events/s. This series retains aggregate counts.
The November 17 tagged reproduction below records per-event combinations.
The earlier detailed profiles exercise a different instrumented image.
Its ordinary mean increases by 0.015 us. PI retains
its 1-kHz grid, and the actual actuator grant computes both id and iq.
The extra PI-deferral candidates add up to one native cycle of timing jitter
while retaining the grid and actual elapsed integration interval. Their
whole-image timing gives no benefit over the unforced q-only boundary.

The existing F405 embedded benchmark now also measures a complete synthetic
driver step with every outer update due. Fixed rotor/current providers and
initialization amortized over 64 steps are included. With the real-motor
coefficients, the dissipative fixture improves 3539.88 to 3297.86 cycles per
step, approximately 1.44 us. Its ordinary fixture increases 2564.00 to
2637.86 cycles. Full and projected q match bitwise on 30000 sampled MCU
parameter sets, with maximum q difference 0 A.

Linked assembly retains the 2404-byte outer-update caller at `0x0802d8bc`
for the unforced q-only path. Its 2876-byte helper has five static VDIV and
23 VSQRT instructions, compared with seven and 26 in the 3292-byte full grant.
These counts include the unrolled rare search. Inlining the dispatcher grows
the caller to 2672 bytes and increases static FP stack operations from seven
to 13. Full q inlining grows it to 3140 bytes. Outlining the circle shrinks
the q helper to 1524 bytes, with a slower ordinary whole-image mean.
The unforced q-only image adds 2880 bytes of `.text`.

Separate PCSR captures contain 79680/80384 driven samples. The control has
420 samples in `grant_dissipative_brake`; the q-only image has 375 in
`grant_iq_authority` and zero in the full dissipative grant. The mapped outer
caller has 360/366 samples. These profiles establish exercised function paths;
the DWT recordings and MCU fixtures provide cycle measurements.

Separate PWM-deadline witness images record 280103/280101 enabled writes,
zero late writes, and minimum margins of 10.367/11.809 us for control/q-only.
ISR-body exceedances remain present in the minimal-timing captures.
The q-only image's largest observed interval between ISR entries is 54.851 us.

The q-only path remains opt-in through `bench-iq-authority`. New paired-bound,
zero-root, dispatcher and PI-deferral implementations are removed after
measurement. The benchmark and comparison coverage remain available for
subsequent measurements. The bench finishes on the current integrated
source's normal image `60b7e672…`, with active/saved PWM at 15 kHz,
zero dissipation, Stopped, zero faults and phase enables off. Its final
two-second velocity/Stop capture is lossless.
Exact ELFs, linked assembly, source hashes, raw attempts, PC samples and
acceptance receipts are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-brake-authority-2026-11-16/`.

### Work-combination attribution and shared tooling, November 17

The frozen q-only ELF `b972720f…` repeats the same 20-kHz +450-rad/s,
HallToObserver, 0.05-W/zero-regen scenario. Two 14-second windows contain
560281 body samples, mean 38.469563 us, maximum 51.764860 us and 36 period
exceedances, approximately 1.29/s. This is an additional same-image control.

`bench-foc-work` records work flags and a bounded list of over-budget cycles.
All 225 tagged exceedances across two windows have complete records and
the velocity-PI flag. Their disjoint families are listed below. Full masks
also retain observer-error wrapping, angle normalization, Hall slew clipping
and Hall motion-witness wrapping.

| Work family | Events | Largest tagged body, us |
|---|---:|---:|
| PI + Hall slew/control-velocity angle wrapping | 93 | 52.426 |
| PI + derating refresh | 89 | 51.360 |
| PI + deadman poll | 42 | 50.841 |
| PI + Hall angle wrapping + derating | 1 | 53.189 |

The first family includes eight events with a deadman poll. Across the
complete records, both Hall slew and Hall control-velocity differences take
their general wrap paths in 94 exceedances. A new Hall witness edge/reference
is recorded once. Command, temperature-conversion, sensorless-startup and
current-circle-search flags each occur zero times in the recorded exceedances.
The Hall wrap paths are in `HallSensor::sample_at_mut` and
`HallControlMotion::observe`, where the angular representation crosses 0/2π.

Instrumentation controls use one identical work ELF with runtime tags on/off.

| Measurement group | Mean body, us | Maximum body, us | Exceedances / samples |
|---|---:|---:|---:|
| Frozen minimal q-only | 38.469563 | 51.764860 | 36 / 560281 |
| Work ELF, tags off | 38.849884 | 52.116394 | 223 / 560225 |
| Same work ELF, tags on | 38.874880 | 53.188870 | 225 / 560286 |

Runtime event publication adds about 0.025 us to the mean in this pair.
The complete work image adds about 0.405 us against minimal timing, including
its changed code/data layout and diagnostic execution. Work-record maxima
belong to that instrumented image. The untagged historical events retain
their original aggregate evidence. Source-level combinations from this
reproduction guide the next targeted scheduling/codegen experiments.

A minimal rebuild with work diagnostics disabled has byte-identical `.text`,
`.foc_hot`, `.data` and vector table against the frozen q-only image.
The equal-sized `.rodata` changes contain source-line location records.
The work image adds 17444 bytes of diagnostic `.bss`.

The shared [measurement workflow](../performance.md) now describes
fixed-ELF runs, work masks, nested section attribution, PWM witnesses,
CPU fixtures, PC sampling, artifact provenance and restoration. The existing
FOC runner supports separate build, fixed-image capture and offline report
operations. The active-drive PC sampler is maintained as a host example.
Fresh hardware captures exercise the consolidated runner, runtime tag control,
PC sampling and embedded brake fixtures. The capture/decoder verifies class
counts and cycle sums against the body recorder, plus record completeness.
The section-outlier recorder and distribution now use a shared body-end
timestamp before diagnostic storage. A final hardware window verifies matching
sample counts and identical maximum-cycle values in both exports. In its
longest record, the nested outer update takes 9.211 us, its negative authority
probe 4.445 us, sensor sampling 7.245 us and protection 3.533 us. These nested
diagnostic values retain their own instrumentation scope.

Artifacts are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-isr-workflow-2026-11-17/`, including
`attribution.json`, fixed images, manifests and the allocated-section audit.

### Service slots, wrap fallbacks and pipelined authority bounds, November 17

Control source `ecc4920` plus the uncommitted instrumentation; during this series
every variant was a default-off feature and the control build kept byte-identical `.text`,
`.foc_hot`, `.data` and vector table across all source edits of this series.
MK5 at 20 kHz, HallToObserver, +450 electrical rad/s. `velocity450` is the
ordinary hold; `brake-authority` adds `max_dissipation_w=0.05` and
`bus_regen_max_a=0`. Every image includes `bench-iq-authority`. Each row has
two 14-second captures per scenario unless stated, about 560,000 cycles.
The measured PWM period is 49.989 us.

The full combination was subsequently approved for production. The accepted
features lost their `bench-` prefix and are enabled by default. The historical
feature names and frozen ELF metadata below identify the measured builds.

| Feature | Change |
|---|---|
| `bench-service-slots` | Bounded frame counter of 20 / 15 native cycles. PI in slot 0, deferred PI in slot 1, deadman at length/4 every frame, derating at length/2 every eighth frame. Commands stay native-rate. |
| `bench-hall-single-turn`, `bench-hall-bit-step` | Inline single-turn steps at the Hall slew and Hall control-velocity sites |
| `bench-lean-angle` | Integer single-turn steps inside the cold `angle_difference_general` / `wrap_angle_general` bodies; the complete fallbacks move to separate cold functions |
| `bench-observer-turn` | Inline one-sided turn at the observer PLL error |
| `bench-pi-bounds-ahead` | Outer-PI authority bounds computed in the last slot of a frame, consumed by the PI in slot 0 or 1 |

#### Repeatability and placement controls

The same control ELF repeats its ordinary mean within 0.002 us and its
brake-authority exceedance count within one event (65 and 64). Exceedance
counts are image-specific: the identical control code relinked on a shared
function-address map has the same mean and 38–48 exceedances.

A source change moves other functions in the natural layout. The first
inline Hall image shifts 352 of 586 functions by four bytes; the slot image
changes 118 entry residues. `--link-reproduce` retains the linker inputs, and
one shared map then places every function of every variant at one address
with its control residue. Literal and read-only data addresses stay variable.

| Variant | Natural mean, us | Fixed-map mean, us |
|---|---:|---:|
| Control | 38.305 | 38.303 |
| Slots | 38.725 | 38.515 |
| Inline Hall single-turn | 38.539 | 38.462 |
| Slots + inline Hall | 38.672 | 38.662 |
| First pipelined bounds | 38.618 | 38.470 |

About half of the slot penalty and a third of the inline-Hall penalty belong
to moved callees. The rest stays inside the changed functions.

#### First series: slots and inline Hall steps

`brake-authority`, natural layout / fixed map.

| Variant | Mean, us | Maximum, us | Exceedances |
|---|---:|---:|---:|
| Control | 38.471 / 38.467 | 51.866 / 51.926 | 65 / 38 |
| Slots | 38.886 / 38.688 | 51.264 / 51.217 | 55 / 21 |
| Inline Hall single-turn | 38.719 / 38.621 | 52.146 / 51.443 | 134 / 70 |
| Slots + inline Hall | 38.840 / 38.830 | 50.829 / 50.734 | 12 / 11 |
| Inline Hall, bit-pattern checks (natural) | 38.547 | 52.599 | 50 |
| Slots + bit-pattern Hall (natural) | 38.777 | 51.264 | 28 |

The slot work image records exactly zero PI+deadman and PI+derating cycles.
Its remaining exceedances are PI cycles alone or with wrap paths: the image's
ordinary cycle is 0.38 us slower than control's, which moves the PI cycle's
own tail across the period. Fifty-five consecutive PI cycles during the speed
ramp, at about 390–410 electrical rad/s and still accelerating, account for
nearly half of that image's records: PI cycles are about 2 us longer in that
speed range than in the +450 hold. The inline Hall steps remove the Hall-seam family in the
work image (47 to 10 records) and grow the phase function by 336 bytes; the
slower ordinary cycle then turns PI+observer-wrap cycles into exceedances.
Bit-pattern range checks recover most of that mean. Neither inline Hall
variant is carried forward.

#### Wrap fallbacks and the observer turn

`angle_difference_general` shrinks from 528 to 74 bytes and
`wrap_angle_general` from 116 to 68 bytes. The lean bodies use no Flash
literal and no FP compare; hot callers are unchanged. The observer subtracts a
[0, 2*pi) PLL angle from a signed atan2 result, so its difference needs a turn
on half of every revolution. The inline one-sided step adds 28 bytes to the
phase function. Frozen-reference tests compare all three helpers bit for bit
over boundary values and one to two million random inputs.

#### Pipelined authority bounds

The outer update splits at its two authority probes. Stage A runs in the last
slot of the frame and stores `iq_min`/`iq_max`; stage B takes the feedback,
applies the fresh voltage-clip tightening and runs the PI in slot 0. The actual
current grant stays fresh in every native cycle, and prepared bounds are
single-use: they never outlive one update.

- Fresh drive activity in the prepare slot moves both stages by one cycle.
  Activity in slot 0 defers the update to slot 1, as before.
- Prepared bounds are dropped by any dequeued command, velocity invalidation
  or rearm, cleared requests (limit, derating, decoupling and velocity
  configuration), external derating-scale updates, an age above 250 us, a
  changed speed direction and a changed bus-modulation class. The update
  then runs both probes inline, which is the previous behavior.
- An elapsed-time backstop keeps the persistent one-period deadline of the
  microsecond grid. A regular frame reaches it only in slot 0. After missed
  native cycles or at a cold start it selects another slot; fresh drive
  activity still waits one cycle. With it, the six scheduled-velocity tests
  that encode the time grid pass again under both slot features.

The eight-frame derating refresh period was approved on November 17. The
cadence checks now cover both 15- and 20-kHz frames, retained scales between
refreshes and frame-counter wrap. Temperature thresholds and sentinel handling
retain their existing coverage.
The lean, observer-turn and Hall single-turn helpers pass all 683 tests.
An audit compares every prepared bound with same-cycle probes in five
closed-loop scenarios (53,955 updates): the largest difference is 39 mA in the
bus-clamp scenario and 5 mA in the dissipative descent, with 34 fallbacks from
direction changes and an injected scale update. Slot simulations cover both
rates, command floods and commands in either stage slot.

#### Follow-up series on one shared address map

All eight images share one function-address map (`fx5`).

| Variant | Ordinary mean, us | Brake mean, us | Brake maximum, us | Exceedances |
|---|---:|---:|---:|---:|
| Control | 38.310 | 38.478 | 51.175 | 48 |
| Lean fallbacks | 38.172 | 38.340 | 51.199 | 10 |
| Observer turn | 37.845 | 38.015 | 50.728 | 25 |
| Lean + observer turn | 37.821 | 37.983 | 51.145 | 5 |
| Slots | 38.553 | 38.725 | 51.640 | 19 |
| Slots + pipelined bounds | 38.531 | 38.704 | 49.221 | 0 |
| Slots + bounds + lean | 38.395 | 38.572 | 48.887 | 0 |
| Slots + bounds + lean + observer turn | 38.044 | 38.214 | 47.505 | 0 |

The ordinary hold has zero exceedances in every image. The observer turn
alone saves 0.46 us of ordinary mean, which establishes the cost of the
out-of-line observer wrap. The pipelined bounds remove every exceedance in
this scenario; the wrap changes add about 1.7 us of margin on top.
The largest ISR entry gap stays between 54.5 and 54.9 us in every image,
including those without body exceedances: it reflects entry latency.

#### Cycle-function entry residue

The same map with only the `run_foc_cycle` entry moved. Ordinary mean in us;
residue 4 is the current `foc-hot.x` choice. Residues 0, 8 and 12 use one
six-second capture per scenario.

| Entry modulo 16 | Control | Slots | Full |
|---|---:|---:|---:|
| 0 | 38.177 | 38.435 | 38.078 |
| 4 | 38.310 | 38.553 | 38.044 |
| 8 | 38.460 | 38.593 | 37.986 |
| 12 | 38.266 | 38.327 | 38.081 |
| Best | 38.177 | 38.327 | 37.986 |

The control code no longer prefers residue 4: residue 0 is 0.13 us faster.
Compared at each image's best residue, slots cost 0.15 us and the full
combination gains 0.19 us. Full has zero exceedances at every residue, with
`brake-authority` maxima between 47.3 and 49.2 us.

#### Phase-function entry residue

The same map with only the `update_phase_with_applied_voltage` entry moved and
the cycle function at residue 4. Ordinary mean in us; residue 12 is the current
choice; the others use one six-second capture per scenario.

| Entry modulo 16 | Control | Lean + observer turn | Full |
|---|---:|---:|---:|
| 0 | 38.032 | 37.790 | 38.012 |
| 4 | 38.064 | 37.686 | 37.909 |
| 8 | 38.136 | 37.885 | 38.109 |
| 12 | 38.310 | 37.821 | 38.044 |
| Best | 38.032 | 37.686 | 37.909 |

Residue 12 is now the slowest phase-function entry for the control code:
residue 0 is 0.28 us faster. The current hot-entry residues were tuned for
earlier code, so the control itself has about 0.3 us of placement gain
available. Compared at each image's best phase residue, the two wrap changes
keep 0.35 us and the full combination 0.12 us against control. Full has zero
exceedances at every phase residue. Cycle and phase residues were swept one
at a time; their joint optimum is unmeasured.

#### Work classes with the wrap class map

One 14-second `brake-authority` capture per work image, `bench-work-class-wraps`.
Class mean / maximum body in us. These images carry tag overhead; compare
classes inside one image and adders between images.

| Class | Control | Lean | Observer turn | Slots | Slots + bounds | Full |
|---|---:|---:|---:|---:|---:|---:|
| No tagged work | 38.34 / 41.99 | 38.28 / 40.64 | 38.02 / 40.95 | 38.55 / 41.50 | 38.36 / 41.84 | 38.09 / 40.99 |
| Observer-error wrap | 38.85 / 41.82 | 38.60 / 40.89 | 38.21 / 40.83 | 39.11 / 41.65 | 38.94 / 41.61 | 38.32 / 40.65 |
| Hall seam | 41.59 / 42.98 | 40.95 / 42.09 | 41.09 / 42.69 | 41.73 / 43.19 | 41.55 / 43.58 | 40.60 / 41.64 |
| Deadman | 39.41 / 41.61 | 39.36 / 41.30 | 39.08 / 41.84 | 39.53 / 42.45 | 39.37 / 42.48 | 39.08 / 41.60 |
| Derating | 40.78 / 42.39 | 40.70 / 42.39 | 40.42 / 42.48 | 41.04 / 43.55 | 40.84 / 43.97 | 40.54 / 42.13 |
| Command | 42.44 / 47.06 | 42.23 / 46.94 | 42.01 / 46.89 | 42.61 / 47.40 | 42.39 / 43.97 | 42.17 / 47.06 |
| PI | 47.87 / 50.70 | 47.81 / 49.72 | 47.50 / 50.12 | 47.74 / 50.07 | 42.41 / 45.53 | 42.13 / 44.01 |
| PI + Hall seam | 51.15 / 52.58 | 50.45 / 51.49 | 50.75 / 52.13 | 50.90 / 52.52 | 45.56 / 46.74 | 44.70 / 45.64 |
| Bounds stage | | | | | 45.30 / 48.57 | 45.01 / 46.81 |
| Bounds stage + Hall seam | | | | | 48.55 / 49.81 | 47.54 / 48.68 |
| Exceedances | 194 | 156 | 124 | 42 | 0 | 0 |

Full is slots + bounds + lean + observer turn. The observer-error wrap is
tagged in half of all cycles and costs 0.51 us there in the control image,
0.31 us with the lean fallback and 0.19–0.23 us with the inline turn. The Hall
seam costs 3.25 us in control and 2.5–2.7 us with the lean fallback. Every
control-image exceedance with a PI is one of: PI + Hall seam, PI + deadman,
PI + derating or the PI's own wrap tail.

Structural counts in these captures: PI cycles sharing a deadman or derating
cycle fall from 932 of 14,005 to zero in every slot image; PI cycles never
share a command cycle in any image; the bounds stage never shares a cycle with
a command, deadman or derating. All 42 slot-image exceedances are PI + Hall
seam. The bounds stage costs 6.9 us and the update stage 4.0 us, against 9.5 us
for the unsplit update: the split adds about 1.4 us of total work per frame
and makes the bounds stage with a coinciding Hall seam the heaviest class.
Inline fallbacks occur one to three times per capture, at the cold start.

The observer-turn image is also 0.32 us faster in cycles without any wrap.
Its normalized phase-function instructions match control except for the
inserted step, so that part is a placement effect: about 190 instructions
after the insertion move by 12 bytes. Only about 0.16 us of its 0.46-us mean
gain is the removed excursion itself.

#### Isolated single-turn costs

`z_bench_hall_single_turn` in `tests/stm32f405`, gate driver off, three
exact-image repeats agreeing within 0.003 cycles. Gross cycles per call in a
warm loop, including loop and `black_box` overhead.

| `angle_difference` input | Shipping fallback | Lean fallback | Inline single-turn |
|---|---:|---:|---:|
| Ordinary, no turn | 26.906 | 26.906 | 26.906 |
| 0/2*pi seam, one turn | 65.688 | 56.219 | 46.359 |

A warm out-of-line turn costs about 39 cycles, 29 with the lean body and 19
inline. In the native ISR the same turn costs about 85 cycles at the observer
and more at the Hall seam: cold Flash lines and literal loads dominate there,
and the warm loop understates what the lean body saves.

#### PWM deadline witnesses

Separate deadline images, 20 kHz `brake-authority`, 14 seconds, about 280,100
enabled writes each. Late, ambiguous and still-masked counts are zero in all.

| Image | Minimum margin, us |
|---|---:|
| Control | 11.285 |
| Lean fallbacks | 11.595 |
| Inline Hall single-turn | 10.999 |
| Slots (first series) | 12.965 |
| Slots + inline Hall | 11.618 |
| Slots + bounds | 14.002 |
| Slots + bounds + lean | 14.931 |
| Full | 14.896 |

#### Natural layout, 15 kHz and transitions

Natural layout is the ordinary `foc-hot.x` link without the shared map.

| Image, 20 kHz natural layout | Ordinary mean, us | Brake mean, us | Brake maximum, us | Exceedances |
|---|---:|---:|---:|---:|
| Control, two series | 38.305 / 38.304 | 38.471 / 38.473 | 51.866 / 53.362 | 65 / 64 |
| Lean fallbacks | 38.163 | 38.328 | 50.138 | 1 |
| Full | 38.331 | 38.513 | 48.315 | 0 |

Without any placement tuning the full combination is mean-neutral and keeps
zero exceedances. One earlier lean capture is rejected for host-side raw
telemetry loss while firmware builds loaded the host; its repeat is listed.

At 15 kHz (measured 15,001.4 Hz, one 14-second capture per scenario) no body
approaches the 66.7-us period. Control / full: ordinary mean 38.405 / 38.182 us,
`brake-authority` maximum 51.985 / 48.232 us, minimum PWM margin
28.480 / 31.650 us with zero late writes.

`transitions` and `brake-transitions`, one capture each, about 206,000 cycles
at 20 kHz:

| Image | Transitions max, us | Brake-transitions mean / max, us | Exceedances | Minimum PWM margin, us |
|---|---:|---:|---:|---:|
| Control | 48.583 | 38.106 / 51.938 | 80 | 10.320 |
| Full | 47.743 | 37.756 / 47.678 | 0 | 14.872 |

The full image also completes both sequences at 15 kHz with maxima of 47.028
and 47.552 us. In its 20-kHz work image the sequence has 9,505 PI updates,
9,504 prepared bounds and five inline fallbacks, with no PI or bounds stage
sharing a command, deadman or derating cycle; the largest tagged body is
49.590 us, for a bounds stage with a Hall seam. The control work image has 164
exceedances up to 54.720 us there.

Across this artifact root, 158 of 159 capture runs are accepted with complete
raw telemetry, acknowledged commands, terminal Stop and unchanged fault
generation; every directory has its recovery receipt. They total 37.0 million
timed cycles and 3.05 million witnessed PWM writes with zero late writes.

#### Scope and adoption

The full combination was approved and promoted to regular, default-enabled
features. Measurements cover the unloaded MK5 in HallToObserver velocity
control. Loaded operation,
position mode on hardware, sensorless sources and other dissipation limits
are unqualified. PI cycles are about 2 us longer near 400 electrical rad/s than
at the +450 hold, so the authority probe's cost depends on the operating point
and its worst case is not established. The approved slot frame polls deadman
once per frame and refreshes derating every eight frames. Cadence coverage now
checks both PWM rates and counter wrap.

Artifacts, exact images, link inputs, address maps, job lists, logs and the
rejected capture are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-slot-wrap-2026-11-17/`; its
`analysis/README.md` explains the directory names. That series ended on the
recovery image at 15 kHz, Stopped, zero faults and phase enables off.

### Production defaults and joint entry-residue sweep, November 17

The approved slot frame, pipelined bounds, scalar q-current authority and angle
helpers now use regular feature names and are enabled by default. Derating
coverage checks the approved eight-frame period at both PWM rates, including
counter wrap. Its temperature-threshold and sentinel coverage is retained.

One captured production-policy object supplies all placement trials. The natural
compact layout is swept over all 16 cycle/phase combinations at residues
0/4/8/12, with grant residue 4. Six further trials vary the grant residue for
the two mean-time leaders. Each screen uses one four-second `brake-authority`
capture, shuffled order and interleaved reference captures. All 22 screens
have zero body-period exceedances. Hot input sections require four-byte
alignment. Their bytes outside recorded relocations match across every ELF.

The two finalists then run two 14-second captures per scenario, bracketed by
reference captures. About 560,000 samples contribute to each scenario row.
Both reference and selected source rebuilds reproduce their measured relinks
in all file-backed allocated sections.

| Cycle / phase / grant | Ordinary mean, us | Brake mean, us | Brake maximum, us | Exceedances |
|---|---:|---:|---:|---:|
| Previous 4/12/4 | 38.231 | 38.414 | 48.673 | 0 |
| Alternative 4/4/8 | 37.995 | 38.174 | 48.512 | 0 |
| Selected 8/4/0 | 37.986 | 38.163 | 47.529 | 0 |

The selected layout favors mean cost and PWM write margin. Its ordinary-hold
maximum increases from 46.510 to 47.165 us. In a separate 10.6-second braking
transition sequence, maxima are 47.844 / 48.541 / 48.738 us for previous /
alternative / selected placement, all with zero exceedances. These tails keep
their scenario scope. The corresponding separate deadline witnesses show
14.002 / 14.693 us minimum transition margin for alternative / selected.
At the steady braking hold, margin improves from 13.525 to 15.170 us with the
selected placement, versus 14.705 us for the alternative. All witnesses have
zero late, ambiguous or still-masked writes.

At 15 kHz, one six-second capture per scenario gives ordinary means of
38.330 / 38.087 us and braking maxima of 47.600 / 47.386 us for previous /
selected placement. The selected PWM witness retains 31.793 us minimum margin.
Both ordinary and braking transition maneuvers complete at 20 kHz with zero
exceedances. Measurements use the unloaded HallToObserver bench.

`oxifoc-f405/foc-hot.x` selects residues 8/4/0. Exact images, shuffled job order,
source-equivalence checks, relocation audit and recovery receipts are under
`/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-hot-joint-2026-11-17/`.
The final normal image completes a two-second +120-rad/s capture with complete
telemetry and acknowledged Stop. It remains on the board at the saved 15-kHz
PWM setting, Stopped, zero active faults and phase enables off. Its exact build
and acceptance receipt are in `production-final/` under that artifact root.

### Further reading on instruction placement

Thomas Pornin's ARM Assembly Tricks memory notes describe measured contention
between instruction fetching and explicit memory accesses on Cortex-M4.
They discuss word alignment of 32-bit instructions and `.n`/`.w` encodings.
[ARM Assembly Tricks, Memory Access Notes](https://github.com/pornin/arm-asm-notes/blob/main/memory.md#arm-cortex-m4)

Schwabe and Stoffelen discuss Flash, alignment, pipelined loads, cache/prefetch
and data placement in section 2.3, pages 4–5. Their STM32F407 experiments are
directly relevant to the F405 memory architecture.
[All the AES You Need on Cortex-M3 and M4](https://cryptojedi.org/papers/aesarm-20161019.pdf#page=4)

Pornin's 2025 Falcon report also discusses instruction-fetch contention and
the interconnection matrix/cache behavior of STM32F407VGT6.
[Falcon on ARM Cortex-M4, an Update](https://eprint.iacr.org/2025/123.pdf)

LLVM review D51780 explains its Cortex-M3/M4 loop-alignment policy using the
pipeline-reload penalty for a branch to an unaligned 32-bit instruction. It
also distinguishes padding before a function from padding executed inside
an enclosing loop.
[LLVM Cortex-M loop alignment review](https://reviews.llvm.org/D51780)

Four-byte entry steps preserve word alignment while changing position within
16-byte Flash lines. Two-byte entry steps additionally change word alignment
of 32-bit instructions.
Changing individual instruction widths is a separate code-generation experiment.

### Available placement controls

| Mechanism | Control and evidence |
|---|---|
| Named function sections and a linker script | Stable `#[unsafe(link_section = ".hot_grant")]` selects the input section. `ALIGN`, `KEEP` and explicit order set entry offsets and grouping. The address sweeps use this mechanism. |
| Existing function input sections | A selector parsed before `link.x` can claim `*(.text.ADC)` directly. Four ADC images use the baseline compiler objects with no source recompile. `INSERT` controls output position, while parsing order controls input selection. |
| LLD symbol ordering | `--symbol-ordering-file=FILE` orders existing function sections using their actual linked symbol names. Explicit hot-chain and leaf-first images are built from identical objects. |
| LLD deterministic shuffling | `--shuffle-sections=.text.*=SEED` creates repeatable layout controls from identical objects. Useful for measuring layout sensitivity across many functions. |
| Per-function alignment | On `nightly-2026-11-13`, `#![feature(fn_align)]` with `#[rustc_align(N)]` works. Object probes confirm 16/32/64-byte alignment. `#[align(N)]` fails on this compiler. A function with a literal pool still requires four-byte input alignment. |
| Global alignment | `-Zmin-function-alignment=16` uses bytes. LLVM `-align-all-functions=4` uses log2. Object probes confirm both. Earlier full-firmware runs gave no convincing gain and the former encountered the Reset linker assertion. |
| Inlining and rare-path hints | `inline`, `inline(never)`, `cold` and `cold_path()` influence function boundaries and block order. The object probe puts an outlined cold function in `.text.unlikely.*`. Final placement depends on LTO and the linker. |
| Source order / profile-guided order | `-Zcodegen-source-order=yes` changes object section order in the probe. Final LTO/ELF order still needs inspection. PC occupancy can guide a linker ordering file. Instrumented PGO and call-graph ordering require separate evaluation. |
| Internal block alignment / SRAM execution | LLVM exposes block-alignment and block-placement options. Existing global block-alignment and broad SRAM-transfer experiments are recorded above. Smaller selected boundaries remain open. |

To select entry offset 4, an explicit linker placement can use
`. = ALIGN(16) + 4; KEEP(*(.hot_grant))`. A minimum alignment of four bytes
allows offsets 0, 4, 8 and 12. Padding before an entry is outside the function's
executed path. Output section patterns must be arranged so the ordinary
`.text.*` wildcard does not consume the selected input section first.
[LLD linker scripts](https://lld.llvm.org/ELF/linker_script.html)

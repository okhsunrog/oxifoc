# STM32F405 ADC ISR optimization

Updated 2026-11-15. This is the single record of accepted optimizations,
measured alternatives and rejected experiments. Motor tuning, Hall behavior
and transport qualification remain in the [bench journal](foc-loop-research.md).
Timings belong to specific MK5 images; gains from different series are not additive.

## Accepted implementation

Process commands every ISR. When fresh drive activity coincides with a due
velocity PI update, postpone the PI by at most one ISR. Check deadman expiry
using the existing protection counter `/16`. Inline the current-control helper
and outline velocity startup/unavailable branches with `#[cold] #[inline(never)]`.
Keep the ordinary target, frame pointers and configured PWM frequency; the
experimental 20-kHz boot override is absent.

| Work | Cadence and behavior |
|---|---|
| Current PI, phase estimation, observer, fast protection, PWM | Every ISR, normally 15 or 20 kHz |
| Command FIFO, Stop, accepted drive/affirmation timestamp | Every ISR |
| Velocity/position PI | Approximately 1 kHz, actual elapsed `dt`, original millisecond grid |
| PI postponement on drive activity | At most one ISR: 66.7 / 50 us at 15 / 20 kHz |
| Deadman expiry polling | `/16`: approximately 938 / 1250 Hz; default timeout remains 150 ms |
| Temperature conversion / derating refresh | Existing `/128` and `/256`; unchanged |

A postponed PI returns to its original grid: delay does not accumulate.
Actuator feedback continues accumulating every ISR, and command floods cannot
starve the PI indefinitely. Postponing the first update holds a zero Iq request.
Angle/speed trust gates and live current constraints still run every ISR.
Velocity and position modes use the outer PI; current control uses the inner
current PI every ISR.

The tradeoff is one ISR of additional PI jitter and up to 1.07 / 0.8 ms before
detecting deadman expiry while the ISR runs. Link gating and execution of active
faults remain native-rate. Decimated polling does not replace the independent
watchdog if the ISR itself stops.

Source: `motor/foc_driver/{velocity_schedule,protection,modes}.rs` and `state.rs`
under `oxifoc-core/src/`. The selected implementation comes from measured
commit `4b4986f`, excluding its temporary PWM override. Increased `.text` is
accepted in exchange for the measured speed improvement.

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

### Stock 15-kHz integration

The integrated source was additionally exercised without the carrier override:
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
| Overclock, SRAM+CCM at 240 MHz | Inhibited current workload: Flash 31.65 versus SRAM+CCM 29.72 us. Short integrity tests completed through requested 260 MHz. Rotating/loaded operation is unqualified; production retains the stock clock. |

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
| `oxifoc-overclock-2026-11-10/` | Clock/SRAM/CCM matrix, registers, inhibited outputs, `analysis.json` |
| `oxifoc-foc-research-2026-11-11/` | Early body/gap/PWM work, rejected Hall/cruise variants |
| `oxifoc-nightly-ab.C6IY3o/`, `oxifoc-arm-flags.uLpWZf/` | Nightly/FPU/flag comparisons; early printed PWM margins are doubled, use raw TIM1 ticks |
| `oxifoc-embassy-ab.ox3qLp/`, `oxifoc-isr-cause.9mxWCf/` | Embassy/clocks, address-preserving PRIMASK A/B, incremental |
| `oxifoc-adc-ownership.h0xmdu/`, `oxifoc-foc-ownership.E7QTle/`, `oxifoc-foc-regression.Ydk196/` | ADC/FOC ownership, detection/retime/cancel, recovery |
| `oxifoc-isr-opt-2026-11-14/`, `oxifoc-isr-review.dW5aab/` | Arithmetic/FIFO/CCR, readiness/angles, logs, original PC sampler |
| `oxifoc-cadence-2026-11-15/` | First cadence matrix, embedded-test, plans, raw and rejected attempts |
| `oxifoc-cadence-2026-11-15/services/` | `service-capture-rollup.json`, `pw-native-summary.csv`, PC manifests in `pc-all/` and `pc-pw/`, analysis/disassembly/build provenance |
| `oxifoc-cadence-2026-11-15/adoption-final/` | Stock 15-kHz integration images, captures and restoration evidence |

Reference 20-kHz research ELF: `services/pw-current-cold20/firmware.elf`, SHA256
`1f66ad2379f65da87cdb054025ecd36eeea7fe86ba2ddab33f27db3220053827`.
It contains a temporary carrier override and is not the normal image.

Evidence covers unloaded MK5/H5055 with HallToObserver. Remaining qualification:
loaded control, sensorless/HFI startup, other modes and rare fault/config paths,
and complete worst-case stack use. Sample PGO, new SRAM-transfer boundaries,
explicit hot-section grouping and other MIR/LLVM thresholds remain untested.
Each needs a separate measured A/B rather than applying a batch of compiler flags.

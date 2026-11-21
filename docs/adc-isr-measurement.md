# ADC ISR measurement workflow

Updated 2026-11-17.

Use this procedure for F405/MK5 performance experiments. Keep measured decisions
and optimization history in [ADC ISR optimization](notes/adc-isr-optimization.md).
Extend the tools and this procedure together when a new measurement is needed.

## Available tools

| Question | Tool | Output |
|---|---|---|
| Native ISR body cost and entry jitter | `bench-foc-distribution`, `scripts/bench_foc_profile.py` | Exact count/sum/max, body and entry-gap histograms, period exceedances |
| Which work coincides in long cycles | `bench-foc-work`, the same runner with `--detail work` | Work-class counts and every retained over-budget sequence/mask |
| Time inside nested stages | `isr-profiling`, `bench-cycle-outliers`, RAM section exports | Section cycles, first and longest diagnostic events |
| PWM preload deadline | `bench-foc-deadline`, the same runner with `--detail deadline` | Enabled writes, minimum margin, late/ambiguous/masked counters |
| Actual clocks and sampling phase | `scripts/f405_timing.py` | RCC, Flash/ART, TIM1, ADC configuration and derived timing |
| Isolated Cortex-M4 code cost | `tests/stm32f405`, `scripts/bench_embedded_f405.py` | Bounded DWT batches and `BENCHV1` cycle totals |
| Commands and physical motion | Host CLI maneuvers, Packed28, `scripts/capreport.py` | ACKs, terminal Stop, complete telemetry, motion/current statistics |
| Linked instructions and placement | `rust-nm`, `rust-objdump`, `rust-size` | Function addresses/sizes, calls, spills, divides, roots, code size |
| Sampled execution occupancy | PCSR sampler or `probe-rs profile` | Statistical PC/function samples |
| Task/interrupt interference | `bench-executor-trace`, `scripts/bench_executor_trace.py`, `bench_link_profile.py` | Exclusive task/IRQ windows and queue/poll timing |

The body timer ends at a shared point before every recorder, statistics
bookkeeping and exception return. In an outlier image it includes the final
section-counter snapshot. The builder records this boundary in its manifest.
Entry gaps include the effects of those operations and other interrupt activity.
The PWM witness measures the hardware write deadline independently. Nested
section timers overlap and require exclusive accounting before summation.

Observed maxima belong to the exercised image and scenario. Record the duration
and sample count alongside them. Specify the scope of a timing bound explicitly.

## Working sequence

1. State the hypothesis and numerical/control behavior it changes. Use the
   current checkout and an experiment branch. Adoption, commits, merges and
   publication follow the user's authorization for that work.
2. Select a known working recovery ELF and record its SHA-256. Verify Stopped,
   supply, temperature, phase source and saved PWM configuration. Use one bench
   controller process at a time. The runner checks the connected 24-V MK5 setup.
3. Build a control and one candidate with the normal build/cache paths. Retain
   the exact image, effective features, compiler version, verbose build command,
   source hashes, symbols, disassembly and section sizes. Release experiments
   use opt3, fat LTO, CGU1, incremental=false and the pinned F405 toolchain.
4. Inspect changed hot functions, addresses and call boundaries. Separate
   arithmetic/inlining changes from address-only relinks where practical.
   `--link-reproduce` retains the linker inputs of a build. One shared
   function-address map then relinks every candidate, and the entry residue
   of a changed hot function can be swept without recompiling. A source change
   inside a large function shifts its later blocks relative to 16-byte Flash
   lines, so compare candidates at their own best entry residue as well.
5. Run fixed-ELF repeats with identical mode, frequency, limits, transport,
   telemetry rate and logging policy. Use interleaved control/candidate/control
   runs when comparing small differences. Build once for same-image repeats.
6. Attribute an unexplained tail with work tags, section timers or PC sampling.
   Measure diagnostic overhead with matched controls. Return to minimal timing
   and a separate PWM witness before making a performance recommendation.
7. Restore the selected recovery image and original saved PWM frequency. Verify
   Stopped, zero active faults and `TIM1_CCER & 0x555 == 0`. Keep the restoration
   receipt with the measurements.

The runner uses finite maneuvers and a terminal Stop. Its recovery guard also
runs on exceptions, SIGINT and SIGTERM. SIGKILL and host power loss bypass a
Python recovery guard. Existing firmware deadman, faults and watchdog remain
part of the bench setup.

## Standard commands

Run from the repository root in its development shell, for example through
`direnv exec .`. Keep experiment artifacts outside the repository.
Use a new output directory for each invocation.

```sh
bench_root=/absolute/path/to/artifacts/EXPERIMENT
recovery_elf=/absolute/path/to/verified/firmware.elf

# Build and inspect without touching the controller.
uv run --no-project python -B scripts/bench_foc_profile.py \
  "$bench_root/control-build" --build-only --detail minimal

uv run --no-project python -B scripts/bench_foc_profile.py \
  "$bench_root/work-build" --build-only --detail work

# Run an immutable image. Each invocation restores recovery before returning.
uv run --no-project python -B scripts/bench_foc_profile.py \
  "$bench_root/control-run" --elf "$bench_root/control-build/firmware.elf" \
  --recovery-elf "$recovery_elf" --detail minimal --pwm-hz 20000 \
  --cases velocity450,brake-authority --repeats 3
```

Use the same capture command with the candidate ELF and a fresh output directory.
Summarize a completed directory offline with the same entry point.

```sh
uv run --no-project python -B scripts/bench_foc_profile.py \
  "$bench_root/control-run" --report-only
```

The report includes weighted body means, maxima, events per second and full
work combinations. Failed attempts remain listed separately.

`velocity450` runs a finite +450 electrical-rad/s request. `brake-authority`
uses the same request with volatile `max_dissipation=0.05` and
`bus_regen_max=0` in HallToObserver. `transitions` runs a fixed 10.6-second
sequence: 0.2-A current start, +120, +450, −120 and −450 electrical rad/s,
Coast, a +120 restart and Stop. `brake-transitions` adds the brake-authority
limits to that sequence. Each repeat resets the image, reinstalls
the scenario and stores the effective configuration. Defaults are 14 seconds,
native-rate capture, quiet logging and a 1-MHz SWD probe clock.

The runner obtains actual per-thread RTKit policy for the host capture process
and retains the result. Acceptance depends on the recorded ACKs, the terminal
Stop and the device side of the stream: a device-side telemetry drop rejects
the run. A host-side drop (the host fell behind a stream the device delivered
completely) is kept in the run as `host_capture_loss` and does not reject a
timing run, because the timing comes from the MCU recorder. The last SWD setup is followed by 1.3 seconds in Stopped, allowing
the DWT supervisor to repair debugger disconnect effects. Normal timing runs
perform SWD setup before excitation and export after Stop.

For the PWM witness, build and capture with `--detail deadline`. For stage
attribution use `--detail sections-ram` or `--detail phase-ram`. Use
`--detail outliers` for nested section times in the first/longest over-budget
cycles. The decoder retains raw words and names nested sections explicitly.
A changed
`--capture-hz` or instrumentation level defines a separate comparison group.
The `just bench-mk5-foc` entry point calls this runner. Its historical short
`hold/current/velocity40/velocity120` maneuver selection remains available.

## Exact work combinations

Build with `--detail work`, then capture that same ELF with `--detail work`.
The lower eight work bits index 256 class summaries. Full masks accompany
over-budget records and distinguish the angle-wrap branches.

| Bit | Recorded work |
|---:|---|
| 0 | Velocity/position outer update executes |
| 1 | At least one command is dequeued |
| 2 | Deadman expiry poll is due |
| 3 | Decimated derating refresh executes |
| 4 | Hall motion witness anchors a new edge/reference |
| 5 | Hall angle slew is clipped |
| 6 | ADC temperature conversion is due |
| 7 | Sensorless startup state machine executes |
| 8 | Hall slew angle difference enters its general wrap path |
| 9 | Hall control-velocity angle difference enters its general wrap path |
| 10 | Hall motion-witness angle difference enters its general wrap path |
| 11 | Observer phase error enters its general wrap path |
| 12 | General angle normalization executes |
| 13 | Dissipative current-circle bisection executes |
| 14 | Outer-PI authority bounds are prepared one slot ahead |
| 15 | Outer PI recomputes its authority bounds inline |

Class summaries use bits 0–7 by default. `bench-work-class-wraps` indexes the
classes by bits 0–3, 14, 15, 8 and 11 instead: PI, command, deadman, derating,
authority-ahead, authority-fallback, Hall slew wrap and observer-error wrap.
The image exports its table as `FOC_WORK_CLASS_BITS`; the decoder converts every
class index to the real event mask and stores the table as `class_bits`.
Compare class means only between runs with the same table.

Markers are compiled out of normal builds. A work image uses atomic event tags
inside the timed body and class accounting after its end timestamp. Its
bookkeeping contributes to entry gaps. This introduces measurement overhead.
`--work-tags off` disables event publication at runtime while retaining the
same ELF instructions and addresses. Compare on/off runs and the minimal image.

Each work run retains class count, sum, maximum and exceedance count, plus up
to 1024 `(sequence, body_cycles, full_mask)` records. The decoder verifies
counts and cycle sums against the independent body histogram. It explicitly
reports dropped records and `complete_attribution`. Shorten/repeat a window
when the finite record store fills. Preserve the partial recording as evidence.

The tags identify executed work. Causality additionally requires section
timings, instruction inspection or an intervention that removes/separates the
suspected overlap. Match a tagged reproduction to the original scenario before
assigning its work combinations to an earlier untagged measurement.

## Numerical and isolated benchmarks

Use `just bench-embedded-f405 OUTPUT RECOVERY FILTER REPEATS`, with details in
[`tests/stm32f405/README.md`](../tests/stm32f405/README.md). Filters cover scheduler
and time-read costs, angle wrapping, dissipative authority and deferred logging.
The brake fixture includes a synthetic complete driver step with every outer
update due. This removes physical sensor/protocol activity from that comparison.

Keep gross cycles, including loop, call and `black_box` overhead. Save warm-up,
repeat order and fixture inputs. Nominal clock conversion is an estimate unless
an independent frequency measurement accompanies it. Quantify numerical changes
in physical units and retain bitwise comparisons where they express the desired
contract. A failing numerical assertion remains a measured candidate whose
accuracy/performance tradeoff requires a decision.

## PC sampling and debugging

The active-drive-aware sampler is `oxifoc-host-lib/examples/pc_sampler.rs`.
It records `(PC, driven)` pairs over SWD. Classify drive state with phase-channel
enable bits, `TIM1_CCER & 0x555`. MOE can remain set in Stopped.

```sh
cargo build --release --locked -p oxifoc-host-lib --example pc_sampler
uv run --no-project python -B scripts/bench_foc_profile.py \
  "$bench_root/pc-run" --elf "$bench_root/control-build/firmware.elf" \
  --recovery-elf "$recovery_elf" --detail minimal --pwm-hz 20000 \
  --cases brake-authority --repeats 1 \
  --pc-sampler target/release/examples/pc_sampler
```

The runner attaches before excitation, commands Stop before detach and retains
raw samples, sampler log and binary hash. The manifest marks this as a separate
`pcsr` diagnostic. `just bench-probe-pc` also exposes the standalone sampler.

Attach/detach while Stopped, retain the exact ELF and raw samples, and run PC
sampling separately from reference timing. `just bench-mk5-foc-pc` provides the
existing `probe-rs profile` route; its flat output also needs workload and
active-state qualification. PC occupancy helps locate executed code. Per-event
composition comes from the work recorder.

GDB breakpoints and halt/read/resume inspection are useful with outputs disabled
or with the motor stopped. A halted run changes control deadlines. Keep halted
debug sessions separate from timing and physical-loop acceptance.

## Acceptance and result record

For each accepted motor capture retain the following evidence.

- Exact ELF SHA-256, build provenance, target, effective features and toolchain.
- Board/motor, source, voltage, PWM/ADC registers, Flash/ART settings and clocks.
- Scenario, limits, telemetry rate/encoding, logging and host scheduling.
- Every command ACK, terminal Stop, complete stream END totals, zero sequence
  gaps and host/producer drops, and unchanged `fault_generation`.
- DWT epoch continuity, sample count, duration, body mean/percentiles/maximum,
  entry gaps and period exceedances, including events per second.
- Work combinations and record completeness, or the remaining attribution gap.
- Separate PWM witness counts/margins and numerical error where relevant.
- Failed attempts, the performance/behavior recommendation and recovery receipt.

Save raw words/logs even when acceptance fails. Record the reason and instrumentation
group before retrying. Optional diagnostic loss can still inform an investigation;
its limitations accompany any conclusions drawn from it.

## Maintaining the workflow

For a new measurement, extend the existing recorder/runner/decoder where it fits.
Document its purpose, perturbation, command and evidence format here. Add a focused
decoder fixture and exercise the hardware path. Keep experiment-specific results
in the optimization history and retain their artifact references. Revisit this
procedure when a failure reveals a missing check or an ambiguous metric.

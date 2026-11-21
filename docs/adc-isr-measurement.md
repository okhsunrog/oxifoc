# ADC ISR measurement workflow

Use this procedure for F405/MK5 performance experiments. Keep measured decisions
and optimization history in [ADC ISR optimization](notes/adc-isr-optimization.md).
Extend the tools and this procedure together when a new measurement is needed.

## Available tools

| Question | Tool | Output |
|---|---|---|
| ISR load, mean/max cost and overruns | Every image: the 1-Hz `isr/s` and `isr-motion/s` defmt lines | Count, mean, max and over-budget cycles, load percent |
| Time inside nested stages and entry jitter | `isr-profiling` (`just build-mk5-profile`) | `isrp`/`isrc`/`isrd`/`isre`/`isrh` section averages and `isri` entry intervals |
| Actual clocks and sampling phase | `scripts/f405_timing.py` | RCC, Flash/ART, TIM1, ADC configuration and derived timing |
| Isolated Cortex-M4 code cost | `tests/stm32f405`, `scripts/bench_embedded_f405.py` | Bounded DWT batches and `BENCHV1` cycle totals |
| Commands and physical motion | Host CLI maneuvers, Packed28, `scripts/capreport.py` | ACKs, terminal Stop, complete telemetry, motion/current statistics |
| Hall/observer handoff behavior | `hall-diagnostics` (`just build-mk5-motion`, `scripts/bench_hall_edges.py`) | Finite Hall edge, handoff, velocity, crossover and fault records |
| Linked instructions and placement | `rust-nm`, `rust-objdump`, `rust-size` | Function addresses/sizes, calls, spills, divides, roots, code size |
| Sampled execution occupancy | `probe-rs profile` (`just profile-mk5`, `just bench-mk5-foc-pc`) | Statistical PC/function samples |

The `isr/s` timer reads DWT once at ADC entry and once after the detection
retiming check; the statistics bookkeeping after the second read falls into the
next entry interval. `isr-profiling` section timers nest and overlap, so they
need exclusive accounting before summation, and they add their own cost to the
measured body.

Observed maxima belong to the exercised image and scenario. Record the duration
and sample count alongside them. Specify the scope of a timing bound explicitly.

## Working sequence

1. State the hypothesis and numerical/control behavior it changes. Use the
   current checkout and an experiment branch.
2. Select a known working recovery ELF and record its SHA-256. Verify Stopped,
   supply, temperature, phase source and saved PWM configuration. Use one bench
   controller process at a time.
3. Build a control and one candidate with the normal build/cache paths. Retain
   the exact image, effective features, compiler version, symbols, disassembly
   and section sizes. Release experiments use opt3, fat LTO, CGU1,
   incremental=false and the pinned F405 toolchain.
4. Inspect changed hot functions, addresses and call boundaries. Separate
   arithmetic/inlining changes from address-only relinks where practical.
5. Run fixed-ELF repeats with identical mode, frequency, limits, transport,
   telemetry rate and logging policy. Use interleaved control/candidate/control
   runs when comparing small differences.
6. Attribute an unexplained tail with `isr-profiling` section timers or PC
   sampling. Return to the normal image before making a performance
   recommendation.
7. Restore the selected recovery image and original saved PWM frequency. Verify
   Stopped, zero active faults and `TIM1_CCER & 0x555 == 0`.

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
contract.

## PC sampling and debugging

`just profile-mk5 OUTPUT` runs `probe-rs profile` in PCSR mode while a separate
maneuver drives the motor; `just bench-mk5-foc-pc OUTPUT` combines both through
the C5. Classify drive state with the phase-channel enable bits,
`TIM1_CCER & 0x555`; MOE can remain set in Stopped. Attach/detach while Stopped,
retain the exact ELF and raw samples, and run PC sampling separately from
reference timing.

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
- The `isr/s` lines over the run: sample count, mean, maximum and overruns.
- Failed attempts, the performance/behavior recommendation and recovery receipt.

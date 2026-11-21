# STM32F405 / MK5 benchmarks

See the [ADC ISR measurement workflow](../../docs/adc-isr-measurement.md) for
the shared procedure and [optimization history](../../docs/notes/adc-isr-optimization.md)
for measured alternatives and the accepted implementation.

This `embedded-test` target contains math checks, synthetic FOC, two sin/cos
implementations, `wrap_angle` comparisons and `z_bench_schedule` for counters,
modulo/masks and time reads. `z_bench_brake` compares frozen full grants,
current full grants and q-only authority, with numerical results in amperes.
The gate driver stays disabled; PC5 holds MK5
power. TIM1/PWM never starts. This test image replaces the normal controller.

## Reproducible runs

From the repository root in the Nix development shell:

```sh
just bench-embedded-f405 NEW_OUTPUT_DIRECTORY VERIFIED_RECOVERY_ELF
```

The default is three independent runs of `z_bench_schedule` using one exact ELF.
For angle comparisons:

```sh
just bench-embedded-f405 NEW_OUTPUT_DIRECTORY VERIFIED_RECOVERY_ELF z_bench_wrap_angle 3
```

Use `z_bench_brake` as the filter for the dissipative-braking fixture.
Use `deferred_logging` for the separate `logging` target. It software-pends
real ADC/USART2 handlers with PWM disabled, measures one ADC log per DWT
window, and checks deferred output, queue overflow, oversized records,
preemption, contention drops and fatal bypass. The worker runs on the real
Embassy executor with the same application logger source. The test harness
selects `embassy-010` and patches its executor dependency to the pinned revision.
`scripts/bench_embedded_f405.py --features DEPENDENCY/FEATURE` selects a
compiler variant while retaining its features in the measurement manifest.

Place artifacts in a directory outside the repository. Recovery
must be a known working image for the connected bench. The runner verifies
Stopped, supply and temperature, builds without running in the usual target
directory, and saves the exact ELF/SHA256, tool versions, build log, raw logs
and decoded JSON. After completion or failure, `finally` restores recovery
under reset and checks Stopped, zero faults and disabled phase PWM outputs.
Restoration failure fails the run. The process must remain alive through
recovery; `finally` cannot survive SIGKILL or host power loss.

The runner explicitly selects release, opt3, fat LTO, CGU1 and incremental=false.
The test crate's Cargo.toml still has incremental=true, so a bare `cargo test`
does not use the same settings. Test RCC uses nominal HSE 8 MHz and PLL N=168;
results are retained in CPU cycles. Nominal 168 MHz is not an independent
measurement of the board's oscillator or the production calibrated clock.

## Measurement implementation

- `tests/support/dwt.rs`: DWT CYCCNT, bounded IRQ-masked windows, compiler
  fences and min/max/total; reporting happens after the timed region.
- `tests/support/schedule.rs`: 64 operations per window, 32 rounds, warm-up
  and alternating order. `black_box` protects results and opaque inputs.
  `scripts/bench_embedded_f405.py` decodes the `BENCHV1` records.
- Cases cover constant `%15/20/16/32`, runtime divisors, reset-at-threshold
  counters, `&15/31`, Embassy ticks/microseconds, direct TIM3 reads and shared
  `CaptureTimebase`/`scale_capture_ticks` primitives.
- Seeds 0 and `0x12345600` distinguish early runtime from larger operands.
  Timebase cases include zero/nonzero Q32 correction. These are not exhaustive
  division-latency or overflow tests.
- Keep gross window cost: call, loop, `black_box` and DWT overhead included.
  u32/u64 baseline loops help compare cases; subtraction does not establish
  exact instruction cost or a guaranteed operation maximum.
- The TIM3 adapter uses the Hall `now_ticks` primitives and register-read
  order, but does not invoke the function from a production ELF. Its timebase
  is opaque to LTO; otherwise overflow accounting disappears because this
  image has no Hall ISR updating the counter. Inspect ARM assembly for new
  cases instead of relying only on `black_box` around the result.

## Full-controller measurements

| Question | Tool |
|---|---|
| Isolated Cortex-M4 operation cost | This `embedded-test` target |
| Running ADC ISR body distribution | `bench-foc-distribution`, `diagnostics/foc_distribution.rs` |
| ISR entry intervals | Separate histogram in the same recorder |
| PWM applied before preload | `bench-foc-deadline`, `diagnostics/pwm_deadline.rs`, TIM5/TIM1 |
| Actual TIM1/ADC cadence and phase | `scripts/f405_timing.py`, Stopped register readback |
| Motor, commands and deadman | Bounded CLI maneuver, Packed28, ACK, fault generation |

Use `just bench-mk5-foc` / `scripts/bench_foc_profile.py` for the full controller.
Research matrices reuse those hardware recorders and decoders with fixed images
and scenarios. Isolated tests lack ADC interrupt load, protocol activity, real
motion and PI/command collisions. Body timing excludes final statistics; entry
intervals and PWM margins answer different questions. Do not combine their
values or treat different instrumentation levels as identical builds.

# Release incremental compilation check

Controller firmware only: CF2, VESC6 MK5 and the existing G474 image. All values
in this report and the preceding sweep are from **`cargo build --release`**.
This comparison changes release incremental compilation independently of the
MIR hint budgets. Source `76855c104b9b3a389c65e2ee117774251e989121`,
nightly-2026-10-14, opt-level 3, fat LTO and one codegen unit remain fixed.

## Result

`release.incremental=false` produces smaller images on all three configurations
and smaller fixed stack-frame sums on the inspected F405 control path. It is a
reasonable release setting to adopt for those benefits. Its execution-time
impact has not been measured. Per the requested scope, no board was used.

The checked native Cargo preset is
[`release-no-incremental.toml`](release-no-incremental.toml). It can be used alone
or together with [`transport-only.toml`](transport-only.toml). The existing
HAL=115 override stays in effect; no global hint budget is added. Normal
firmware manifests and dev profiles were not edited for this check.

| Release configuration | CF2 | VESC6 MK5 | G474 |
|---|---:|---:|---:|
| Current, incremental enabled | 372456 | 374764 | 208672 |
| Only incremental disabled | 370732 (-1724) | 373012 (-1752) | 205964 (-2708) |
| Only ergot=50, postcard=0 | 369520 (-2936) | 371824 (-2940) | 208360 (-312) |
| Both changes | 368064 (-4392) | 370348 (-4416) | 205584 (-3088) |

Bytes count `.vector_table + .text + .rodata + .data`, excluding ELF/debug
metadata and `.bss`. Static data+bss stays at 36248 / 36232 / 24516 bytes
respectively in all four configurations. G474 currently has no active FOC
loop, so its row does not establish G474 FOC behavior or timing.

On CF2, the native `profile.release.incremental=false` build is byte-for-byte
identical as an ELF to the earlier `CARGO_INCREMENTAL=0` experiment. This is
not a change that requires a further unstable rustc tuning flag. Cargo's
[normal release default](https://doc.rust-lang.org/cargo/reference/profiles.html#release)
already disables incremental compilation; these firmware manifests explicitly
enable it today. Dev incremental compilation can remain enabled independently.
Incremental compilation is intended to improve rebuild times; the elapsed
build times in these diagnostic logs are not a controlled throughput benchmark.

## Stack frames and linked hot code

The table gives **fixed software frame reservations** decoded from each
function's prologue, including saved general/FP registers, alignment padding
and local/outgoing-argument space. Both F405 board configurations give the same
values; adding the transport-only settings does not change these reservations.
There are no additional SP-decrement instructions later in these inspected
functions. The disassembly and decoded components are recorded in
[`release-check.json`](release-check.json).

| Function/frame | Incremental enabled | Incremental disabled |
|---|---:|---:|
| ADC entry, containing sampling/publication | 248 | 8-byte trampoline |
| Separate `run_foc_cycle` | 416 | inlined into real ADC body |
| Real `__cortex_m_rt_ADC` | part of ADC entry | 416 |
| `step_current_control` | 192 | 184 |
| `step_direct_voltage` | 176 | 120 |
| `step_velocity_control` | 80 | 88 |
| `update_phase_with_applied_voltage` | 160 | 160 |
| Separate `MotorPwm::set_duties` | expanded in callers | 24 |

The nonincremental ADC trampoline restores its 8-byte frame before tail-
branching to the real handler. Those 8 bytes therefore do not remain stacked
beneath the real ADC frame.

For the confirmed direct-BL current-control chain:

```text
incremental:
  ADC(248) -> run_foc_cycle(416) -> step_current_control(192) -> phase(160)
  sum of these fixed frames: 1016 bytes

nonincremental:
  ADC_body(416) -> step_current_control(184) -> phase(160)
  sum of these fixed frames: 760 bytes
```

That is **256 fewer bytes on this particular frame chain**. It is not the
maximum ISR stack requirement: exception-entry context, preempted thread state,
other callees, other branches and nested interrupts are excluded. A stack
watermark on hardware is still needed to establish the actual high-water mark.
It also does not establish execution speed.

The previously identified PWM call tradeoff remains: `set_duties` becomes an
out-of-line helper with three nested `get_max_duty` calls. Those max-duty
queries have moved rather than disappeared. The current-control frame becomes
slightly smaller while the velocity frame grows by eight bytes. There is no
basis here to describe every local change as uniformly better.

## Validation

All six release firmware builds passed. Default-budget, nonincremental release
host tests passed: **419 core tests, 14 host-library tests and 11 doctests**.
Two core tests and 18 core doctests remain ignored by the existing suite.

The G474 on-target suite also compiled with the native release preset and
HAL=115, using `cargo test --release --locked --no-run`. Disassembly has **35
scaled q31 conversion sites**, including both `vcvt.s32.f32 ..., #31` and
`vcvt.f32.s32 ..., #31`, and no `.f64` instructions. This suite was compiled,
not executed on a device.

All six new firmware builds retained the configured single-precision FPU
flags. Both F405 variants contain hardware `vsqrt.f32`; the harness rejects
software sqrt symbols and unsupported `.f64` instructions. Static data+bss did
not grow. CF2's specialized SPI transfer remains 372 bytes in both candidates.

The release host suite uses opt-level 3 and `CARGO_INCREMENTAL=0`, with the
**default hint budgets**, independently of the earlier global-500 tests.
Features: core `runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect`.
Host correctness tests and target test compilation are not MCU timing results.

## Reproduction

From `oxifoc-f405` or `oxifoc-g474`, preserving their normal target config:

```bash
export TMPDIR=/home/okhsunrog/tmp_zfs/codex_tmp/release-incremental-repro
mkdir -p "$TMPDIR"
cargo build --release --locked \
  --target-dir "$TMPDIR/target" \
  --config ../docs/experiments/mir-threshold-sweep/release-no-incremental.toml
```

Add `--config ../docs/experiments/mir-threshold-sweep/transport-only.toml`
to test the combined variant. For VESC6 also add
`--no-default-features --features transport-usb,transport-uart,board-vesc6-mk5`.
The original measurements used the fixed neutralized worktree from the sweep
and explicitly restored HAL=115 through a package override. All full build
logs, symbols and disassemblies are under the corresponding `*-release-noinc`
and `*-release-transport-noinc` cases in
`/home/okhsunrog/tmp_zfs/codex_tmp/mir-threshold-sweep/cases/`.

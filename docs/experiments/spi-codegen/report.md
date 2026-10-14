# SPI code-size regression investigation

Measured on 2026-10-15 for `thumbv7em-none-eabihf`, release opt-level 3,
fat LTO, one codegen unit, panic abort. The isolated crate in `repro/`
calls the default `SpiDevice::transfer` with two-byte input/output arrays
on an `ExclusiveDevice` wrapping blocking Embassy STM32 SPI and `NoDelay`.
It reproduces the **374 -> 1276 byte** function-size change observed in
CF2 without any OxiFOC code or floating-point operations.

## Result

The triggering Rust change is
[PR #159220](https://github.com/rust-lang/rust/pull/159220), merged
2026-07-28: restrict `SimplifyComparisonIntegral` across storage markers.
This fixes [a real miscompilation](https://github.com/rust-lang/rust/issues/158231).
Its effect on this HAL crosses a MIR inlining threshold and changes the
cross-crate call graph. The resulting LLVM optimization sequence leaves
a generic operation dispatcher in a known one-operation transaction.

The correctness fix must be retained. Raising a diagnostic compiler
threshold demonstrates causality; it is not a proposed firmware configuration.

## Compiler boundary

Same locked dependencies and source throughout the bisect:

| Compiler | LLVM | `spi_transfer` bytes |
|---|---|---:|
| nightly-2026-03-22 | 22.1.0 | 374 |
| nightly-2026-04-15 | 22.1.2 | 374 |
| nightly-2026-06-07 | 22.1.6 | 374 |
| nightly-2026-07-22 | 22.1.8 | 374 |
| nightly-2026-07-26 | 22.1.8 | 374 |
| **nightly-2026-07-28**, `09ee43b2d` | **22.1.8** | **374** |
| CI rollup `e7b595554e664e6bd281c8cf881093d6c71bc0e1` | 22.1.8 | 1246 |
| **nightly-2026-07-29**, `26ae60a9e` | **22.1.8** | **1246** |
| nightly-2026-07-30 | 22.1.8 | 1246 |
| nightly-2026-10-14, `822182db4` | 23.1.3 | 1276 |

The first jump is **872 bytes with the same LLVM version**. Another
30 bytes appear between July 29 and October 14; that smaller delta was
not separately bisected. The intermediate CI compiler includes the morning
rollup containing #159220 and excludes later same-day compiler changes.
The rollup has other PRs; the controlled pass and threshold experiments
below isolate the relevant transformation beyond the date correlation.

## Causal chain

1. `check_error_flags` checks four status-register bits. Its MIR before
   `SimplifyComparisonIntegral` is byte-for-byte identical across July 28/29.
   The old pass replaces `bool = mask != 0; switchInt(bool)` with
   `switchInt(mask)` and moves `StorageDead(mask)` into successors.
   The fixed pass declines this transformation when the compared local's
   lifetime ends before the branch, preserving four `Ne` assignments.
2. Final optimized MIR of this helper has 14 assignments and five switches
   on July 28, versus 18 assignments and five switches on July 29.
   [`CostChecker`](https://github.com/rust-lang/rust/blob/26ae60a9eeb20b4935be49d7a931a650fa1d2923/compiler/rustc_mir_transform/src/cost_checker.rs)
   charges five units per assignment/simple switch: **95 -> 115**.
   Storage markers do not contribute to this cost.
3. The MIR inliner's
   [applicable threshold](https://github.com/rust-lang/rust/blob/26ae60a9eeb20b4935be49d7a931a650fa1d2923/compiler/rustc_mir_transform/src/inline.rs#L381)
   is **100**, from `inline-mir-hint-threshold`. The helper was previously
   inlined into `check_rx_ready` and `check_tx_ready`; now it remains a call.
   The saved final MIR in `mir/` directly shows that difference.
4. The readiness helpers consequently cease to qualify for inferred
   [cross-crate inlining](https://github.com/rust-lang/rust/blob/26ae60a9eeb20b4935be49d7a931a650fa1d2923/compiler/rustc_mir_transform/src/cross_crate_inline.rs):
   that inference requires zero non-intrinsic calls. Their bodies used to
   be emitted into the consuming crate before its LLVM optimization; now
   they enter through the later fat-LTO stage. The pre-LLVM root modules
   confirm the old locally emitted bodies versus the new already optimized
   dependency bodies imported at LTO.
5. LLVM leaves a loop over the single 20-byte `Operation`, with a variable
   index into its stack allocation. Scalar replacement cannot expose the
   operation's known tag while that indexing remains. All five enum cases
   survive, including an impossible `DelayNs` case and `NoDelay` panic.
   ARM output contains a `tbh` dispatch table and additional stack traffic.

The cost units are compiler heuristics, not ARM instruction counts.
The four comparisons themselves are not 872 bytes of generated code;
their indirect effect on inlining and specialization causes that growth.

## Controlled checks

All threshold/pass changes in this table apply **only to `embassy_stm32`**
unless another crate is explicitly named. Firmware source is unchanged.

| Compiler / diagnostic | Function bytes |
|---|---:|
| July 28, normal threshold 100 | 374 |
| July 28, disable `SimplifyComparisonIntegral` in HAL | 1246 |
| July 28, disable same pass only in root crate | 374 |
| July 28, hint threshold **94** | 1246 |
| July 29, hint threshold **114** | 1246 |
| July 29, hint threshold **115** | 374 |
| October 14, normal threshold 100 | 1276 |
| October 14, hint threshold **115** | 372 |

This reproduces the regression using the old compiler and recovers compact
code using the fixed compiler, with a boundary matching the calculated MIR
cost. Disabling the pass only in `embedded_hal_bus` also leaves 374 bytes.
Increasing the ordinary MIR threshold (50 -> 60/100) does not help:
the applicable threshold is the separate hint threshold. Increasing
LLVM's inline threshold (100 -> 500/1000/10000) also leaves 1276 bytes.
Three root metadata/hash perturbations give the same 1276-byte result.

## LLVM recovery experiment

The saved October LLVM module in `llvm/latest.ll` is already optimized by
rustc. Re-running LLVM 23.1.3's `default<O3>` on it produces a 372-byte
function. Reducing that extra pass pipeline gives:

```text
loop(loop-rotate,loop-deletion),sroa,sccp,simplifycfg
```

That sequence gives **354 bytes**. Rotation/deletion eliminate the residual
single-operation loop, SROA replaces its stack loads with known operands,
and constant propagation/CFG cleanup removes unrelated enum cases. In
isolation, loop deletion plus SROA/propagation leaves 1276 bytes; rotation
plus SROA/propagation without deletion leaves 1102 bytes. A plain additional
SROA/constant-propagation pass does not recover the compact function.

This establishes an LLVM phase-order sensitivity with valid, fixed MIR.
It does not establish the best general LLVM pipeline change: adding a pass
globally can affect compile time and unrelated code. The triggering Rust
MIR change and exact inlining cutoff are established separately above.

Root-only `-C lto=off` and `-C lto=thin` produce 366/372-byte entry functions
respectively. Those figures exclude remaining out-of-line helpers and
are **not whole-firmware savings** or recommendations to change LTO.

## Reproduction

Install the requested toolchains with the ARM target; LLVM tools are needed
for `opt`/`llc` recovery. GNU Arm binutils must be in PATH. Then:

```sh
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-28
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-29
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-28 --disable-pass-in embassy_stm32
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-28 --mir-hint-threshold 94
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-29 --mir-hint-threshold 114
uv run scripts/reproduce-spi-codegen.py nightly-2026-07-29 --mir-hint-threshold 115
uv run scripts/reproduce-spi-codegen.py nightly-2026-10-14 --features=+vfp2sp
uv run scripts/reproduce-spi-codegen.py nightly-2026-10-14 --features=+vfp2sp --mir-hint-threshold 115
```

Each invocation writes its build log, assembly, LLVM IR, linked diagnostic
ELF and `result.json` under
`/home/okhsunrog/tmp_zfs/codex_tmp/spi-codegen/reproduced/`.
Both Cargo's build-dir and target-dir, plus TMPDIR, are explicitly redirected.
The build uses the committed lockfile. `measurements.json` preserves
fresh results from this script, including reachable text in the diagnostic
ELF with unused archive members discarded. That text includes the example's
minimal panic handler; it is not a production firmware-size measurement.

For the additional LLVM passes, use the October toolchain's LLVM tools
(host path below is for this x86_64 Linux machine):

```sh
SPI_LLVM="$(rustc +nightly-2026-10-14 --print sysroot)/lib/rustlib/x86_64-unknown-linux-gnu/bin"
"$SPI_LLVM/opt" -S -passes='loop(loop-rotate,loop-deletion),sroa,sccp,simplifycfg' \
    /home/okhsunrog/code/rust/oxifoc/docs/experiments/spi-codegen/llvm/latest.ll -o recovered.ll
"$SPI_LLVM/llc" -filetype=obj recovered.ll -o recovered.o
arm-none-eabi-nm -S recovered.o
```

Run these commands from a task directory under the dedicated temporary root.
No hardware timing or runtime acceptance was performed. The investigation
does not alter firmware dependencies, math implementation or target flags,
and does not apply a workaround. A Rust compiler regression report can use
this crate, compiler boundary and MIR cutoff; a later LLVM report can reuse
the standalone IR and reduced recovery sequence. No upstream issue/PR has
been published by this experiment.

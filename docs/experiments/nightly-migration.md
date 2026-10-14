# Adopt nightly-2026-10-14

The experimental branch now pins `nightly-2026-10-14` in the repository root
and all four firmware crates: G474, F405, bridge and remote. The official
[nightly channel manifest](https://static.rust-lang.org/dist/channel-rust-nightly.toml)
still reported this date when checked at 2026-10-14 22:46 UTC.

STM32 firmware and on-target test configs use `+vfp2sp`. The HAL-only MIR
hint threshold of 115 remains enabled in the two STM32 firmware configs.
The now-stable `optimize_attribute` feature gate is removed, while the
`isr-speed` feature and its `#[optimize(speed)]` annotations are retained.
`core_float_math` still requires nightly. No dependency versions or lockfiles
were changed for this migration.

Validation used the actual project pins/configs, without a toolchain or
RUSTFLAGS environment override:

- Release builds: G474, F405 CF2, F405 VESC6 Mk5, ESP bridge, ESP remote.
- Host workspace: `cargo check --workspace --all-targets --locked`.
- Core tests with runtime, virtual-motor, storage, std, delivery, hfi and
  hfi-detect: 419 passed, 2 ignored; doctests: 11 passed, 18 ignored.
- Both STM32 on-target test projects compile. G474's q31 round-trip test
  contains both `vcvt.s32.f32 ..., #31` and `vcvt.f32.s32 ..., #31`.
- Formatting checks pass for the workspace, all four firmware crates and
  both on-target test projects.
- Both F405 release ELFs retain 25 `vsqrt.f32` sites. None of the STM32
  ELFs contains the libm software-sqrt symbol/RSQRT table or double-precision
  FP instructions.

Fresh STM32 release results, in bytes:

| Configuration | Flash | Static RAM | Hardware sqrt sites |
|---|---:|---:|---:|
| G474 | 208448 | 24516 | 0 |
| F405 CF2 | 372056 | 36248 | 25 |
| F405 VESC6 Mk5 | 374380 | 36232 | 25 |

Flash is `.vector_table + .text + .rodata + .data`. G474 continues to omit
the control module. Absolute sizes from different worktree/build contexts
are not controlled threshold comparisons; use the paired
[SPI threshold measurements](spi-codegen/report.md) for that comparison.

Logs, ELFs and the section/assembly validation are under
`/home/okhsunrog/tmp_zfs/codex_tmp/nightly-migration/` on the measurement
host. TMPDIR, Cargo build-dir and target-dir are redirected there. No board
was flashed and no physical-device or motor acceptance is implied. Further
threshold exploration is paused until requested again.

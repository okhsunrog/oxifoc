# core_float_math and nightly code generation experiment

Measured 2026-10-15 on branch `experiment/core-float-math`, starting at
`d774f09`. No flashing or hardware timing was performed. This branch retains
all original toolchain pins; the new compiler was selected with
`RUSTUP_TOOLCHAIN=nightly-2026-10-14` for the independent comparison.

## Controlled variants

- Original: unchanged source and lockfiles, nightly-2026-03-22,
  Rust 1.96.0-nightly (20f19f461), LLVM 22.1.0.
- Sqrt only: replace the target-dependent asm/libm `fast_math::sqrtf`
  implementation with `core::f32::math::sqrt`; enable `core_float_math`.
- Core math: sqrt only plus replace remaining core-crate `libm::sqrtf`,
  `floorf` and `roundf` calls (including simulation/tests) with core APIs;
  remove unused direct libm dependencies from the two STM32 manifests.
  The core crate still needs libm.
- Latest original: unchanged original source/lockfiles on
  nightly-2026-10-14, Rust 1.101.0-nightly (822182db4), LLVM 23.1.3.
  This was the latest available nightly reported by rustup.
- Latest cfg repair: original source on the latest compiler, only broaden
  the sqrt asm condition to `any(target_feature = "vfp2",
  target_feature = "vfp2sp")`. This isolates the target-cfg regression
  from the compiler's other changes.
- Latest core math: the complete core-math variant on the latest compiler.

The firmware profiles, target, feature selections and target rustflags were
unchanged: `thumbv7em-none-eabihf`, `+vfp2,-fp64`, release opt-level 3, fat
LTO, one codegen unit, debug symbols retained. Builds used `just size`;
CF2 and VESC6 retain separate target directories. No dependency versions
were updated. Removing the board dependencies removes only their direct
libm lockfile edges, not the libm package or version.

## Flash results

Bytes loaded into flash: `.vector_table + .text + .rodata + .data`, not
ELF file size or debug sections.

| Variant | G474 | F405 CF2 | F405 VESC6 Mk5 |
|---|---:|---:|---:|
| Original, old nightly | 206700 | 370672 | 373116 |
| Sqrt only, old nightly | 206700 | 370628 | 373080 |
| Core math, old nightly | 206712 | 370644 | 373096 |
| Original, latest nightly | 208528 | 374160 | 375160 |
| Original + cfg repair, latest nightly | 208528 | 373628 | 374620 |
| Core math, latest nightly | 208448 | 373376 | 374380 |

Independent compiler upgrade with unchanged source: **+1828 / +3488 /
+2044 bytes**. With the cfg issue repaired but the original asm retained:
**+1828 / +2956 / +1504 bytes** versus the old compiler.

Core math versus original on the same compiler: old nightly **+12 / -28 /
-20 bytes**; latest nightly **-80 / -784 / -780 bytes**. Latest core math
versus original old nightly still grows **+1748 / +2704 / +1264 bytes**.
The small old-compiler changes are not a large flash optimization.

Static RAM (`.data + .bss`):

| Variant | G474 | F405 CF2 | F405 VESC6 Mk5 |
|---|---:|---:|---:|
| Original / sqrt only, old nightly | 24548 | 36288 | 36272 |
| Core math, old nightly | 24548 | 36288 | 36272 |
| Latest original / cfg repair / core math | 24516 | 36248 | 36232 |

G474 core math on the old compiler shifts 8 bytes from `.bss` to `.data`
and adds 4 bytes of `.text`; total static RAM is unchanged. Its small
flash delta is a layout change, not evidence of faster arithmetic.

**G474 limitation:** this checkout comments out `mod control` in
`oxifoc-g474/src/main.rs`. Its motor control path is not linked, and there
are no `vsqrt.f32` instructions in the original ELF. The unchanged sqrt
result therefore says nothing about an active G474 FOC loop. F405 is the
relevant firmware for the hot-path comparison.

## The latest compiler exposes a sqrt fallback regression

With exactly the same target flags, old rustc exposes
`target_feature="vfp2"`; latest rustc exposes `target_feature="vfp2sp"`
instead. The original sqrt implementation tests only `vfp2`, so the new
compiler silently selects the libm fallback.

Both original F405 binaries have 26 static `vsqrt.f32` instructions on the
old compiler and zero on the latest compiler. Latest original introduces
`libm::math::sqrt::sqrtf` (220 bytes of code) and its 256-byte `RSQRT_TAB`.
This changes executed code, not just flash accounting. The cfg repair
restores 26 static hardware sqrt sites. Latest core math has 25, with the
software sqrt symbol eliminated. Static site counts are not cycle counts.

Consequently, the apparent shrinking of some FOC functions in the raw
compiler-upgrade comparison must not be mistaken for a speed improvement:
hardware instructions have become calls to software sqrt, and inlining
also changes function boundaries.

`core::f32::math::sqrt` lowers to `vsqrt.f32` on both tested compilers and
avoids this fragile feature check. In the isolated wrapper:

```asm
; original asm wrapper, 14 bytes including prologue/epilogue
vsqrt.f32 s2, s0
vmov.f32  s0, s2

; core intrinsic wrapper, 10 bytes including prologue/epilogue
vsqrt.f32 s0, s0
```

The asm's `out(sreg)` operand prevents reuse of the input register. Inlined
real callers may already coalesce the result or allocate registers
differently, so the four-byte wrapper saving cannot be multiplied by the
number of call sites. On old nightly, CF2's `update_phase_with_applied_voltage`
shrinks by 56 bytes while other functions grow slightly; net flash is -44
for the sqrt-only variant.

## Other concrete compiler changes

On CF2, compare old original to latest **cfg-repaired** original so hardware
sqrt is present in both:

| Symbol | Old bytes | Latest bytes | Delta |
|---|---:|---:|---:|
| ADC ISR | 3200 | 3176 | -24 |
| `run_foc_cycle` | 14100 | 14088 | -12 |
| `update_phase_with_applied_voltage` | 13056 | 13076 | +20 |

The key FOC functions stay roughly the same size after correcting sqrt.
Most aggregate growth is outside them. In the unchanged-source latest
comparison:

- CF2 main task shrinks from 28476 to 27600 bytes (-876).
- G474 USB task grows from 7716 to 9460 (+1744); its main task shrinks
  from 15610 to 14920 (-690).
- VESC6 USB task grows from 6528 to 7058 (+530).
- `RegistryPropertyFeatureDescriptor::size` appears as a standalone
  1368-byte function; the old standalone `PropertyData::size` is 368
  bytes and disappears. These are different function boundaries, so
  1368 alone is not a net increase attributable to that source method.
- F405 protocol-server task body (11668 bytes) becomes inlined into its
  `TaskStorage::poll`: the wrapper grows from 172 to 11974. Combined,
  those two symbols change from 11840 to 11974 (+134), not +11802.
- Numerous `drop_in_place` symbols become `drop_glue` symbols with the
  same size. Treating their disappearance/appearance as removed/added
  code would give a misleading attribution.
- CF2 `.rodata` grows by 440 bytes on the raw upgrade, but latest core
  math removes 416 of those bytes; the software sqrt table contributes.

These are observed linked-code/inlining/layout changes. They do not identify
an individual LLVM pass, prove an upstream compiler bug, or establish
runtime speed. Small symbol-size deltas can include alignment and
register allocation; total loaded sections are the reliable flash metric.

## Which libm calls can actually be removed?

The published latest [`core::f32::math` API](https://doc.rust-lang.org/nightly/core/f32/math/index.html)
provides sqrt and rounding, but not sin/cos, atan2, ln, tanh or IEEE
remainder. `rem_euclid` is not the same operation as `remainderf`.

- `sqrt`: good replacement; removes its inline asm and gives LLVM the
  mathematical operation directly. Negative inputs remain NaN.
- `floor`, `round`: on M4F these call `floorf`/`roundf`; there is no hardware
  rounding instruction substitution here. Runtime math is supplied by
  compiler-builtins on the tested target. Source dependency removal is
  not equivalent to eliminating all software math from the binary.
- Sin/cos in firmware FOC already use FastSinCos or CORDIC. Keep these
  backends; replacing them with a general libm-style trig API would
  reintroduce costly argument reduction. Simulation and reference tests
  still use libm trig.
- Precise `atan2f` computes circular-average Hall calibration angles in
  `HallCalibrator::finish`. The observer's hot path already has the
  approximate `fast_math::atan2f`. Using that approximation for
  calibration would bake its roughly 0.011-rad worst error into stored
  angles; a separately validated, more accurate small polynomial is an
  option, not part of this API experiment.
- `logf` implements the NTC Beta-model conversion in
  `NtcConfig::temp_c_from_adc`. A compact logarithm approximation or a
  table can be evaluated in temperature-error units over supported NTC
  configs, ADC values and protection thresholds. It has not been
  substituted by this experiment.
- `remainderf` is the cold large-angle fallback in `angle_diff` and is also
  used in the virtual motor. A replacement must preserve its rounding
  semantics and boundary behavior; `%`/`rem_euclid` are not drop-in fixes.

Complete removal of libm from oxifoc-core therefore needs a separate
numerical-design change, and retaining it in dev-dependencies as an
independent reference would still be useful.

The two q31 asm conversions also remain justified: on the latest compiler,
plain Rust cast-and-scale emits a literal load, multiplication and ordinary
`vcvt` instead of the existing scaled `vcvt ... #31`. Each isolated Rust
wrapper is 28 bytes versus 14 for asm, including the literal/prologue.
`core_float_math` does not expose these fixed-point conversions. No q31
implementation or behavior was changed.

## Validation and reproducing

- Old nightly full core suite: 389 passed, 2 ignored; doctests 11 passed,
  14 ignored, with `std,virtual-motor,storage,hfi,hfi-detect`.
- Latest core-math suite: the same results, also enabling `isr-speed`.
  Latest reports that `optimize_attribute` has stabilized; the old pin
  still requires the feature gate, so it was retained.
- The committed sqrt test checks special values and one million
  deterministic f32 bit patterns against libm. Finite results and signed
  zero compare bit-for-bit; NaNs compare by category, not payload. A
  separate host probe also compared floor/round over the same sample.
- All six variants built all three STM32 configurations. No board was
  flashed, no DWT benchmark was run, and no physical motor acceptance is
  implied. ESP bridge/remote compiler migration was not tested.

Generate the isolated assembly using either compiler:

```sh
rustc +nightly-2026-10-14 --edition 2024 --crate-type lib \
  --target thumbv7em-none-eabihf -C opt-level=3 \
  -C target-feature=+vfp2,-fp64 --emit=asm \
  docs/experiments/core-float-math/codegen-probe.rs -o /tmp/math.s
```

Old and latest generated assembly are committed beside this report.
`measurements.json` retains section totals, selected instruction counts
and symbol-size differences against the original old-nightly binaries.
Use the helper with any two captured ELFs:

```sh
uv run scripts/compare-firmware-codegen.py old.elf new.elf > comparison.json
```

Captured ELFs, full disassembly, logs and the cfg-repair worktree are under
`/tmp/oxifoc-math-experiment/` on the measurement host. The six ELF sets are
`baseline`, `sqrt`, `core-math`, `latest-baseline-artifacts`, `latest-cfg`,
and `latest-math`; those temporary artifacts are not checked into Git.
The cfg-only comparison is reproduced by broadening the original sqrt
`vfp2` cfg as described above, without the core-math patch.

Recommendation: use the core sqrt intrinsic. The newest compiler does not
provide an aggregate flash-size win on these configurations, and upgrading
without repairing the old cfg slows the sqrt path. Keep compiler migration
and approximation work as independently measured changes. Floor/round
migration is primarily API cleanup, not a demonstrated speed improvement.

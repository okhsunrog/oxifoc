# core_float_math and nightly code generation experiment

Measured 2026-10-15 on branch `experiment/core-float-math`, starting at
`d774f09`. No flashing or hardware timing was performed. The original
comparisons retained the old pins and selected the new compiler with
`RUSTUP_TOOLCHAIN=nightly-2026-10-14`. After the experiments, the branch
adopts the tested October nightly; see [migration validation](../nightly-migration.md).

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

## Follow-up: full firmware validation with +vfp2sp

The follow-up uses hardware sqrt in **both** compiler variants. It does not
build or propose a software-sqrt configuration. Both q31 asm implementations
are retained verbatim. The latest test worktree pins nightly-2026-10-14 in
root and both STM32 toolchain files, replacing the board rustflags with only
`-C target-feature=+vfp2sp`. The reproducible configuration changes are in
`vfp2sp-toolchain.patch`; these settings are now adopted on this branch.

| Same core-math source | G474 flash | CF2 flash | VESC6 flash |
|---|---:|---:|---:|
| Old nightly, +vfp2,-fp64 | 206712 | 370644 | 373096 |
| Latest nightly, +vfp2sp | 208460 | 373408 | 374396 |
| Delta | +1748 | +2764 | +1300 |

All three release builds succeeded. Both latest F405 ELFs contain 25
`vsqrt.f32` sites, no libm software-sqrt symbol or RSQRT_TAB, and no `.f64`
arithmetic instructions. G474 still omits the control module as described
above. The G474 on-target test binary was additionally built on latest
nightly with `RUSTFLAGS='-C target-feature=+vfp2sp'`; its `q31_round_trip`
test links both actual q31 helpers. Disassembly contains both
`vcvt.s32.f32 ..., #31` and `vcvt.f32.s32 ..., #31`. It was compiled and
inspected, not executed on a board.

Artifacts for these builds are in `vfp2sp-artifacts` under the temporary
experiment root. Compared with the earlier latest `+vfp2,-fp64` builds,
flash differs by +12/+32/+16 bytes; the full ELFs are not byte-identical.
This does not change the hardware-math conclusions. The isolated FPU probe
has identical assembly with the two flag selections. Firmware-level layout
and hashes can vary with compiler flags and the separate worktree; do not
interpret those small deltas as extra FP instructions.

### Confirmed USB contributor: changed core UTF-16 counting

This is a change in core's implementation as well as compiler codegen:

- Old `EncodeUtf16` has no specialized `count`; its default counting loop
  repeatedly calls `next`.
- Latest `EncodeUtf16::count` directly classifies UTF-8 bytes to count
  UTF-16 units. At opt-level 3 the ARM code unrolls byte classification
  four ways, increasing code size when inlined into descriptors.
- The committed `utf16-probe.rs` reproduces it without any dependencies:
  `utf16_count` grows from 134 to 426 bytes; `utf16_count_next` remains
  134 bytes on both compilers. This is static size, not a throughput test.

A controlled full-CF2 diagnostic replaced **only** the three
`.encode_utf16().count()` calls in Embassy USB's `msos.rs` with
`.encode_utf16().fold(0usize, |n, _| n + 1)`. This exercises the generic
iteration path rather than the new specialized count. The tiny
`usb-utf16-diagnostic.patch` records this experiment; it is not a production
dependency change.

For a controlled comparison, both the original and folded builds used the
same local copy of embassy-usb 0.6.0 at the pinned Embassy revision,
identical latest compiler, `+vfp2sp`, firmware source, dependency versions,
features and optimization settings. Its five relative Embassy dependencies
were redirected to the same pinned Git revision. Both the crates.io and
Git embassy-usb sources were patched to that local package, so ergot and
the firmware still share one USB crate. The firmware lockfile changed only
the source identity of embassy-usb, not package versions. Local source
identity itself affects layout, hence the original local-source build is
measured separately rather than treating it as identical to the Git build.

| Controlled CF2 diagnostic | Flash | Static RAM |
|---|---:|---:|
| Local USB, original count | 373264 | 36248 |
| Same local USB, generic fold | 372296 | 36248 |
| Causal change | **-968** | **0** |

`.text` shrinks by 968; `.rodata`, `.data` and `.bss` are unchanged. The
1368-byte `RegistryPropertyFeatureDescriptor::size` disappears and the
368-byte `PropertyData::size` reappears; main-task code grows by 30 bytes,
with a small alignment remainder. Section/symbol results are preserved in
`usb-utf16-delta.json`. The independent probe test compares count and next
for every valid Unicode scalar plus mixed strings; it passes on latest
nightly. No runtime speed claim follows from the size measurement.

### Confirmed SPI contributor: the generic transaction survives

With the same hardware-math source, CF2's concrete
`ExclusiveDevice<...>::transfer` is 374 bytes on old nightly and 1276 on
latest `+vfp2sp`: **+902 bytes in that function**. This is not just a rename.
Both versions inline the transfer machinery, but the generated structure
is different:

- Old output specializes the single `Operation::Transfer` created by the
  default `SpiDevice::transfer`. The function has no operation-dispatch
  jump table and no call to `no_delay_panic`.
- Latest output materializes the 20-byte Operation on the stack, retains
  an operation loop and a `tbh` jump table covering the general operation
  variants, and includes a call to `embedded_hal_bus::spi::no_delay_panic`.
  That helper also appears separately (28 bytes).

The latest prologue additionally reserves 28 stack bytes and saves fp,
versus neither in the old version. This is the observed lost
specialization of a known one-element transaction. The subsequent
[compiler bisect and MIR analysis](../spi-codegen/report.md) identifies
the first bad nightly and the MIR inlining threshold crossed by Rust's
storage-marker correctness fix.
No dependency-source change is involved in this SPI comparison, and this
experiment does not introduce a SPI workaround.

USB's controlled -968 and SPI's +902 show concrete contributors on the
scale of most of CF2's +2764-byte growth. They are not a complete additive
accounting of the difference: changing USB inlining moves other code too,
and other functions both shrink and grow. Remaining changes include the
USB bus poll state machine, detect_server, packet serialization and
inlining boundaries. Thus the source of significant growth is established,
but attributing every remaining byte to a particular compiler pass is not.

The follow-up leaves the existing source/toolchain pins in this branch
intact and adds results plus reproducible diagnostic patches. The tested
new settings are ready to apply together when choosing compiler migration.

The diagnostic patches use zero context; replay with
`git apply --unidiff-zero <patch>` in the corresponding project checkout.

## Follow-up: size profiles and timing before an Embassy fold change

Measured on 2026-10-15. The updated Embassy main is `0c1abfbce` and pins
Rust 1.97; installed `1.97` resolves to 1.97.1 (8bab26f4f). Its three MS OS
UTF-16 counting sites still use `.count()`. The relevant size computations
are unchanged from the pinned Embassy revision in the firmware diagnostic;
main moves the file to `embassy-usb/src/device/msos.rs`.

This comparison changes only those three count calls in the local
embassy-usb copy. The complete CF2 size experiment still uses the existing
pinned `98d847be` dependency graph, not an integration of current Embassy
main into oxifoc. Each count/fold pair uses the same path, dependency
versions, profile, nightly-2026-10-14, `+vfp2sp`, fat LTO and codegen unit
count. All firmware variants retain the core hardware sqrt and q31 asm.
The global release opt-level is overridden to 3, s or z; the existing
`#[optimize(speed)]` hot-path overrides remain in place in both variants.

### Full CF2 flash, measured independently within each profile

| Opt-level | Count | Fold | Fold minus count |
|---|---:|---:|---:|
| 3 | 373264 | 372296 | **-968** |
| s | 297820 | 298128 | **+308** |
| z | 245928 | 245940 | **+12** |

Only `.text` changes within each pair. Static RAM stays 36248 bytes at 3,
36284 at s, and 36256 at z. Section-level data are preserved in
`utf16-firmware-profiles.json`. Thus fold is not a universal flash win;
opt-level s shows a material regression.

### Isolated ARM implementation, including its helpers

A small no_std object exports just one implementation at a time. The table
sums every `.text*` section in that object, including any emitted next()
or machine-outlining helpers. It does not include unwind/debug metadata.

| Compiler | Opt-level | Count text bytes | Fold text bytes |
|---|---|---:|---:|
| Rust 1.97.1 | 3 | 134 | 134 |
| Rust 1.97.1 | s | 124 | 124 |
| Rust 1.97.1 | z | 130 | 130 |
| Latest nightly | 3 | 426 | 134 |
| Latest nightly | s | 56 | 124 |
| Latest nightly | z | 56 | 130 |

Measuring only the fold wrapper on z would suggest 28 bytes in the shared
probe, but that excludes next() and outlining helpers. For a standalone
implementation the total is 130 bytes. Full-firmware marginal cost differs
because the UTF-16 writing code may already need the helpers.

Rust 1.97 predates the specialized count implementation, so count and fold
produce the same ARM code on the current Embassy pin. The flash advantage
at 3 applies to compilers with the new count implementation.

### Host timing: measured, not an ARM-cycle estimate

`probe-rs list` found no debug probe, so no Cortex-M4 cycle counts were
measured. Timing runs use Intel Core Ultra 5 125H, pinned to logical CPU 0,
Linux powersave governor, default x86-64 codegen (no target-cpu=native).
Each implementation has a non-inlined call; inputs/results use black_box.
Inputs are allocated before timing and passed at runtime. There are 15
samples per implementation/input/profile, calibrated to about 20 ms for
the slower path. Execution order alternates count/fold between samples.
No compilation ran concurrently with the timing phase. Frequency was not
fixed; the raw paired samples and quartiles are retained.

Median host ns per call on nightly-2026-10-14:

| Input | UTF-8 bytes / UTF-16 units | 3 count / fold | s count / fold | z count / fold |
|---|---|---:|---:|---:|
| Empty | 0 / 0 | 1.02 / 1.04 | 0.74 / 0.76 | 0.99 / 1.01 |
| DeviceInterfaceGUIDs | 20 / 20 | 9.86 / 12.27 | 12.48 / 18.87 | 11.92 / 20.02 |
| GUID | 38 / 38 | 26.26 / 31.61 | 23.56 / 27.39 | 25.54 / 29.27 |
| ASCII, 128 bytes | 128 / 128 | 46.11 / 61.36 | 50.18 / 66.74 | 48.22 / 66.19 |
| ASCII, 4096 bytes | 4096 / 4096 | 1013.19 / 1530.49 | 1184.38 / 1814.78 | 970.63 / 1713.30 |
| Cyrillic | 88 / 52 | 56.59 / 53.66 | 71.62 / 46.59 | 64.74 / 64.63 |
| CJK | 288 / 96 | 124.14 / 106.66 | 137.67 / 110.49 | 132.56 / 102.49 |
| Emoji | 256 / 128 | 108.49 / 92.77 | 124.02 / 110.19 | 118.12 / 107.24 |
| Mixed | 192 / 144 | 106.82 / 114.34 | 161.16 / 130.74 | 145.33 / 156.86 |

ASCII favors the specialized count in these host measurements; CJK and
emoji favor fold. This is not proof of the corresponding Cortex-M4
throughput or a measurement of complete descriptor-building latency.
Short-input values include function/black_box overhead. On Rust 1.97 the
host compiler even merges count/fold functions at 3 and z; differences
between their measurements are not an algorithmic improvement. The s host
functions remain distinct and have the same body size, with some noisy
short-input results; raw samples document that limitation.

The latest paired fold/count ratios for the property name are about
1.25 at 3, 1.52 at s and 1.67 at z. Ratios are computed per sample before
summarizing; a ratio of separate median ns values may differ slightly.

The exhaustive Unicode-scalar correctness test also passes on Rust 1.97.1;
latest nightly passed it in the earlier diagnostic. Both benchmarks assert
count/fold equality for all nine inputs before measuring.

### Consequence for an upstream change

Replacing count with fold offers a measured flash tradeoff for opt-level 3
on the new compiler, but regresses flash at s and generally costs ASCII
throughput on the host. It has no ARM code-size effect on Embassy's current
Rust 1.97 pin. A blanket replacement cannot be presented as universally
smaller or faster. The operation happens while building MS OS descriptors,
which are then stored as a byte slice; any throughput tradeoff is specific
to descriptor construction, not each USB packet.

Before an upstream fold PR, these profile results should be included and
the maintainers' preference for size versus descriptor-construction time
established. A compact counting implementation or a Rust/core codegen
improvement is another candidate; neither is evaluated by this fold test.
Actual ARM timing remains unmeasured until a board is available.

Reproduce the isolated sizes and host timing with:

```sh
uv run scripts/benchmark-utf16.py --out /tmp/utf16-results --cpu 0
```

Required installed tools: the two toolchains with thumbv7em-none-eabihf,
GNU Arm binutils and taskset. `--sizes-only` skips host timing.
The Rust host harness is `utf16-bench.rs`; complete metadata, samples,
medians, quartiles and isolated ARM sections are in
`utf16-profile-results.json`. The ELF pairs for all profiles remain under
`/tmp/oxifoc-math-experiment/usb-diagnostic/` as
`original[-s|-z].elf` and `fold[-s|-z].elf`. To reproduce the firmware pairs,
use the earlier local USB patch setup, override
`--config 'profile.release.opt-level="s"'` or `"z"`, build count and fold
sequentially in the same source path and preserve each ELF before changing
the dependency source. No firmware was flashed.

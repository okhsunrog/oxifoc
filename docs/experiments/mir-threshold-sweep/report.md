# Global and per-package MIR inlining budgets

Measured on 2026-10-15/16 against source `76855c104b9b3a389c65e2ee117774251e989121`,
using the already upgraded **nightly-2026-10-14** (`822182db4`, LLVM 23.1.3).
This experiment varies `-Zinline-mir-hint-threshold`, not LLVM's inlining budget.
The compiler upgrade, `+vfp2sp`, direct core sqrt imports, dependencies and
application source stay fixed. The starting point already has HAL threshold 115.

## Results

Two additional Cargo configs are provided:

- [`balanced.toml`](balanced.toml): HAL 115 retained, ergot 50, postcard 0,
  bbqueue 200, sequential-storage 50. No global threshold or incremental change.
- [`size.toml`](size.toml): the same package overrides, global threshold 500,
  release incremental disabled. This changes FOC/ADC inlining boundaries.

These are explicit experimental presets; the normal firmware build settings
remain at the measured baseline. A size reduction is established; a runtime
speed improvement is **not** established. Measure ISR execution on the boards
before choosing the broader change.

| Firmware | Current bytes | Balanced bytes (delta) | Size bytes (delta) |
|---|---:|---:|---:|
| CF2 / F405 | 372456 | 368308 (-4148) | 366100 (-6356) |
| VESC6 MK5 / F405 | 374764 | 370624 (-4140) | 368276 (-6488) |
| G474 | 208672 | 208288 (-384) | 205476 (-3196) |
| ESP bridge | 909234 | 909234 (0) | not tested |
| ESP remote | 863372 | 863372 (0) | not tested |

Static `data + bss` is unchanged for both presets on the three STM32 images:
36248 / 36232 / 24516 bytes respectively. The ESP package-only preset also
leaves this measure unchanged (94460 / 86600). A global threshold of 51 grows
the ESP images by 114 / 112 bytes, respectively; the package-only preset gives
no saving there. No ESP preset is adopted.

On STM32 the table counts flash payload: `.vector_table + .text + .rodata + .data`.
ESP values are ELF `text + data`, including instruction RAM; they are comparable
within the same board but are not packaged ESP image lengths. Debug sections,
ELF file length, and `.bss` do not count toward this measure. Static `data + bss`
is recorded separately in [`measurements.json`](measurements.json); it does not
include the runtime stack/heap high-water mark.

## Global sweep first

CF2, the manifest's incremental setting retained, **no package overrides** in
this table. In particular, a global value below 115 loses the known SPI recovery.
The baseline of 372456 bytes instead has the existing HAL-only 115 override.

| Global hint budget | Image bytes | Delta against baseline |
|---:|---:|---:|
| 0 | 373592 | +1136 |
| 25 | 371600 | -856 |
| 40 | 372208 | -248 |
| 50 | 371204 | -1252 |
| 51 | 371116 | -1340 |
| 60 | 373496 | +1040 |
| 75 | 373512 | +1056 |
| 100 | 373780 | +1324 |
| 115 | 372420 | -36 |
| 150 | 372264 | -192 |
| 200 | 371832 | -624 |
| 300 | 371368 | -1088 |
| 500 | 371340 | -1116 |
| 1000 | 371340 | -1116 |

There is no monotonic relationship: a larger budget can grow code or enable
specialization that shrinks it. Global 51 with HAL restored to 115 gives
369864 bytes (-2592), but the package-specific combinations below do better.

MIR costs are not instruction counts. The pinned compiler's
[`CostChecker`](https://github.com/rust-lang/rust/blob/822182db4c4a1827256881d8c7aa685f0bcf2b11/compiler/rustc_mir_transform/src/cost_checker.rs)
uses costs in multiples of five. The inliner can increase a small body's
threshold by a quarter, so not every numeric increment creates a new decision
boundary. Values 50 and 51 can have the same effective cost cutoffs yet give
slightly different final images (88 bytes globally, 4 bytes for postcard).
The exact cause of those small codegen-context differences was not isolated.
They are not evidence that every byte reflects a newly accepted inline call.
The balanced preset therefore uses round values and retains HAL 115 rather
than changing it to 300 for another 44 bytes in the combined CF2 image.

## Per-package sweep

Change one package at a time on CF2; HAL stays at 115 except in the HAL row.
Unlisted packages retain their default budget 100. Deltas are against 372456.

| Package | Lower-budget result | Higher-budget result |
|---|---:|---:|
| ergot | 50: -1672 | 500: +300 |
| postcard | 0: -1056 | 500: -412 |
| bbqueue | 51: 0 | 200: -768; 500: -772 |
| sequential-storage | 51: -216 | 500: +4 |
| embassy-stm32 | 150: 0 | 300/500: -40 |
| embassy-usb | 51: -60 | 500: 0 |
| embassy-executor | 51: -12 | 500: -32 |
| embassy-futures | 51: +4 | 500: -4 |
| serde_core | 51: +8 | 500: -4 |
| maitake-sync | 51: 0 | 500: 0 |
| embassy-sync | 51: 0 | 500: 0 |
| embedded-hal-bus | 51: 0 | 500: 0 |
| oxifoc-core | 51: 0 | 500: +4 |
| oxifoc-f405 | 51: 0 | 500: -16 |

Fine sweeps give ergot 0: +2852, 25: -924, 50/51: -1672, 75: -324,
200: +332; postcard 0: -1056, 25: -940, 50: -896, 51: -900, 75: +4,
200: -128; bbqueue 200/300: -768. Most other packages do not justify
additional configuration for a few bytes. The zero threshold does not remove
`#[inline(always)]` semantics or disable later LLVM inlining.

The gains do not simply add:

| Combined overrides, HAL 115 retained | Image bytes | Delta |
|---|---:|---:|
| ergot 51, postcard 51 | 369884 | -2572 |
| Above + bbqueue 500 | 369300 | -3156 |
| Above + sequential-storage 51 | 369084 | -3372 |
| Above, but HAL 300 | 369044 | -3412 |
| Balanced round values, excluding sequential-storage | 368516 | -3940 |
| Balanced round values | 368308 | -4148 |
| Balanced, HAL 300 | 368264 | -4192 |

Representative CF2 symbol changes locate much of the library-only gain in
transport/serialization code. With ergot 50 alone, the COBS `encode_frame_hdr`
instance shrinks from 606 to 318 bytes and `EdgePort::send_raw` from 1018 to 836.
With postcard 0 alone, the 344-byte COBS `serialize_u32` instance disappears
as a standalone symbol. These symbol deltas are not additive savings: inlining,
outlining and merging can move code into other functions. The whole-image
measure above accounts for those effects.

## Why local thresholds initially do almost nothing

The pinned compiler's
[`Inline::policy`](https://github.com/rust-lang/rust/blob/822182db4c4a1827256881d8c7aa685f0bcf2b11/compiler/rustc_mir_transform/src/inline.rs#L45)
disables normal MIR inlining at the default MIR optimization level when either:

- incremental compilation is active; or
- the crate's optimization level is neither 2 nor 3.

Both STM32 firmware manifests enable release incremental compilation. Cargo
applies incremental compilation to local workspace/path packages, while the
Git/registry dependencies in these builds are nonincremental. Thus the local
core/root thresholds mostly change compilation context, whereas the dependency
thresholds actually affect the normal MIR inliner. LLVM inlining and forced
MIR inlining are separate; this does not mean that incremental builds have no
inlining at all.

The ESP firmware uses `opt-level="s"`. The captured unit graph confirms target
units use `s` and host units use 0; normal MIR inlining is consequently disabled
there too. Budget changes alone do not enable it. Keep that distinction when
interpreting the ESP measurements rather than assuming the STM32 gains transfer.

Turning incremental off enables the local pass and provides another independent
size opportunity:

| CF2 variant | Image bytes | Delta |
|---|---:|---:|
| Current settings, incremental off | 370732 | -1724 |
| Balanced, only core incremental off | 367168 | -5288 |
| Balanced, only root incremental off | 367892 | -4564 |
| Balanced, all incremental off, default global 100 | 367284 | -5172 |
| Balanced, all incremental off, global 50 | 368244 | -4212 |
| Balanced, all incremental off, global 200 | 366284 | -6172 |
| Balanced, all incremental off, global 500 | 366100 | -6356 |

Extending the global sweep with the same package overrides and incremental off:

| CF2 global budget | Image bytes | Delta |
|---:|---:|---:|
| 300 | 366816 | -5640 |
| 1000 | 366116 | -6340 |
| 2000 | 365980 | -6476 |

2000 is the smallest CF2 image tested, another 120 bytes below 500. The shared
size preset stays at 500, which was tested across all three STM32 images.
Further package scoping in the nonincremental/global-500 variant gives 366936
bytes with core threshold 100, and 367000 with both core and root at 100.
Neither beats the shared size preset. These results illustrate a bounded
search, not a proof of the global minimum over all compiler settings.

## Generated code and speed limits

The six CF2 symbols below have these sizes in the current and balanced builds.
Numbers after the slash count **static** BL/BLX instructions in that symbol,
not dynamic calls per control cycle.

| Symbol | Current bytes/calls | Balanced bytes/calls |
|---|---:|---:|
| ADC | 3176 / 14 | 3152 / 13 |
| step_direct_voltage | 1172 / 16 | 1172 / 16 |
| step_current_control | 1752 / 13 | 1752 / 13 |
| step_velocity_control | 1308 / 3 | 1308 / 3 |
| update_phase_with_applied_voltage | 13008 / 115 | 13008 / 115 |
| ExclusiveDevice SPI transfer | 372 / 0 | 372 / 0 |

In the balanced build the indirect `blx r1` at current-image address
`0x08000b06` disappears. Current-image DWARF maps it to `Waker::wake_by_ref`
inside Embassy `AtomicWaker::wake`, from `state::update_telemetry` through
`publish_cycle_telemetry` in the ADC handler. The static difference is real;
why the queue setting enables this downstream change was not separately
isolated. The four FOC routines retain their sizes and static call counts, but this
alone is not an instruction-for-instruction or cycle-accurate equivalence test.
Isolated ergot 51 and postcard 51 preserve all six baseline sizes/call counts.

The size preset changes the structure substantially. ADC becomes a 12-byte
trampoline and its real `__cortex_m_rt_ADC` body is 16592 bytes with 153 static
calls. Comparing that trampoline with the original 3176-byte entry function
would grossly misstate the ISR reduction. Several previously separate routines
are inlined, while other calls remain outlined:

| CF2 symbol | Size preset bytes/calls |
|---|---:|
| ADC trampoline | 12 / 0 |
| Real ADC body | 16592 / 153 |
| step_direct_voltage | 1836 / 8 |
| step_current_control | 1640 / 9 |
| step_velocity_control | 1356 / 3 |
| update_phase_with_applied_voltage | 13812 / 113 |
| ExclusiveDevice SPI transfer | 372 / 0 |

The current build with incremental disabled but otherwise unchanged already
has a 17080-byte real ADC body with 145 calls. The changed function boundaries
come from enabling local MIR inlining, not just from increasing the budget.
Neither the larger isolated FOC symbol nor the ISR's static call count proves
that a control cycle becomes slower or faster. Measure the actual ISR on a
board; code-size measurements cannot substitute for that.

## Validation and reproduction

All measured STM32 ELFs are checked for unsupported `.f64` instructions and
software `libm::sqrtf`/`RSQRT_TAB` symbols. F405 images must contain hardware
`vsqrt.f32`. The q31 asm source is unchanged. SPI transfer remains 372
bytes in the balanced and size presets on CF2. VESC does not emit this
standalone SPI-transfer symbol. G474's current image has no FOC loop or sqrt
sites; its size results do not validate G474 FOC timing or numerical code.
No firmware was flashed or timed for this sweep.

The existing G474 on-target test suite was separately compiled with the size
preset, HAL 115 and `+vfp2sp`, using `cargo test --release --locked --no-run`.
Its actual q31 helpers emit 35 scaled-conversion sites, including both
`vcvt.s32.f32 ..., #31` and `vcvt.f32.s32 ..., #31`; no `.f64` instructions
appear. This is a compile/disassembly check, not a physical test run.

Optimized host tests with the size preset's effective flags and `opt-level=3`,
`CARGO_INCREMENTAL=0` passed: 419 core tests, 14 host-library tests and 11 core
doctests. Features: core `runtime,virtual-motor,storage,std,delivery,hfi,hfi-detect`.
The normal host release profile uses `s`, so the explicit opt-level 3 matters
for exercising the same normal MIR-inlining policy as the STM32 candidates.

The experiment runner uses Cargo's unstable
[`profile-rustflags`](https://doc.rust-lang.org/nightly/cargo/reference/unstable.html#profile-rustflags-option)
with per-package overrides. The captured unit graph verifies that a package's
flag vector replaces the global profile vector: global 75 plus HAL 115 gives
HAL 115, ergot 75 and root 75. Target `+vfp2sp` flags remain present.
It does not replace target flags using a process-wide `RUSTFLAGS` value.

Both native TOML presets were built on all three STM32 board configurations.
All six resulting ELF hashes match the corresponding CLI-driven cases exactly.
[`validation.json`](validation.json) records those pairs, the profile-scope
probe, ESP optimization-level counts, host tests, and q31 codegen check.

Full build logs, ELF files, demangled disassembly and symbols are retained under
`/home/okhsunrog/tmp_zfs/codex_tmp/mir-threshold-sweep/cases/<board>-<label>/`.
The committed JSON contains parameters, section sizes, ELF hashes, sqrt counts,
and critical symbol sizes/static calls for every completed case. Build times
are warm-cache diagnostics, **not** controlled compiler-throughput benchmarks.

To reproduce a controlled case without inheriting the existing HAL override:

```bash
repo=/home/okhsunrog/code/rust/oxifoc
sweep=/home/okhsunrog/tmp_zfs/codex_tmp/mir-threshold-sweep-repro
mkdir -p "$sweep"
export TMPDIR="$sweep"
git -C "$repo" worktree add --detach "$sweep/source" 76855c104b9b3a389c65e2ee117774251e989121
git -C "$sweep/source" apply --unidiff-zero "$repo/docs/experiments/mir-threshold-sweep/neutral-config.patch"
uv run "$repo/scripts/sweep-mir-threshold.py" \
  --source "$sweep/source" --output "$sweep" --board cf2 --label balanced \
  --package embassy-stm32=115 --package ergot=50 --package postcard=0 \
  --package bbqueue=200 --package sequential-storage=50
uv run "$repo/scripts/sweep-mir-threshold.py" \
  --source "$sweep/source" --output "$sweep" --board cf2 --label size \
  --package embassy-stm32=115 \
  --profile-config "$repo/docs/experiments/mir-threshold-sweep/size.toml"
```

From the regular checkout, use either preset as the extra `--config` shown in
its comments; the existing HAL override is retained. Board selections supported
by the runner are `cf2`, `vesc6-mk5`, `g474`, `bridge`, and `remote`. The VESC case
adds `--no-default-features --features transport-usb,transport-uart,board-vesc6-mk5`.
Different source paths can affect debug paths and compilation context; compare
paired builds in the same worktree. The runner clears inherited rustc/profile
flags, records source/config identity, and refuses to reuse a label for different
inputs. A numeric global sweep uses only `--global-threshold N`; package sweeps
keep `--package embassy-stm32=115` plus the package being varied.

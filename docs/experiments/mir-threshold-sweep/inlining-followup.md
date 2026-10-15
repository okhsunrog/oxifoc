# Controller-only inlining follow-up

This follow-up covers STM32 motor-controller firmware only. ESP build settings
are outside its scope. The source and compiler remain the sweep's fixed
`76855c104b9b3a389c65e2ee117774251e989121` and nightly-2026-10-14.
Normal build settings have not been changed by this investigation.

## Recommendation

The subsequent [release check](release-check.md) isolates incremental
compilation across all three controller configurations, including fixed stack
frames and release tests. It confirms size/frame benefits while leaving MCU
timing unmeasured.

Keep the existing **embassy-stm32=115** override. It has an isolated reproducer
and an explained specialization failure, rather than just a favorable total.

For an additional **size** improvement, prefer
[`transport-only.toml`](transport-only.toml): **ergot=50, postcard=0**, retaining
HAL 115, the default global hint threshold and the existing incremental setting.
This is a narrower candidate than the previous balanced preset. Its cost is
an extra out-of-line varint call in serialization; it is not a FOC speed tweak.

| Controller build | Current payload | Transport-only payload | Delta |
|---|---:|---:|---:|
| CF2 / F405 | 372456 | 369520 | -2936 |
| VESC6 MK5 / F405 | 374764 | 371824 | -2940 |
| G474 | 208672 | 208360 | -312 |

Static data+bss is unchanged. G474 still does not contain an active FOC loop,
so that row measures the existing firmware, not G474 FOC behavior. A native
Cargo-config build on CF2 matches the CLI-driven candidate's entire ELF hash.

Do not adopt global 500/2000 as a default speed optimization from these results.
They reduce image size, but also change PWM call boundaries, phase/startup
code, and ISR layout. They have no measured execution-time advantage here.

Disabling release incremental compilation is a reasonable separate candidate:
it re-enables the normal MIR inliner locally and removes 1724 bytes on CF2 even
without the transport tweaks. It also changes hot code, including outlining
`MotorPwm::set_duties`. Keep this experiment separate from the transport change
and compare the existing DWT cycle counters before adopting it for execution
speed. Retain dev-profile incremental compilation for normal development.

The extra bbqueue=200 and sequential-storage=50 overrides are unnecessary for
this narrower recommendation. The full balanced candidate saves another 1212
bytes on CF2, but bbqueue directly changes telemetry publication inside ADC.
Storage tuning adds configuration for a small gain. These are still valid size
experiments if additional flash savings become useful; they are not established
motor-control performance improvements.

## What was measured

Four complete CF2 builds dump normal `Inline` and `ForceInline` MIR before/after
for ergot, postcard, bbqueue, sequential-storage, core, F405, maitake-sync and
embassy-sync:

1. Current: HAL 115, otherwise defaults, local incremental enabled.
2. Balanced: the prior 115/50/0/200/50 library settings, incremental enabled.
3. Balanced with incremental disabled, global hint budget 100.
4. Balanced with incremental disabled, global hint budget 500.

All four **flash binary hashes match** the original sweep's corresponding
builds byte for byte. Whole ELF hashes differ, so they are not used to assert
that the diagnostic builds are identical. The flash comparison establishes
that inspecting MIR did not change the actual firmware being explained.

[`inlining-analysis.json`](inlining-analysis.json) contains build identities,
flash hashes, selected scope/call differences and the linked call comparisons.
Full MIR and disassembly are retained in
`/home/okhsunrog/tmp_zfs/codex_tmp/mir-inlining-analysis/` and the original sweep's
`cases/` directory. Unchanged cached crates reuse their last preceding MIR dump.
MIR inventories include generic/unlinked functions and nested scopes; they are
not a count of instructions executed by the motor loop.

## What changes at the MIR stage

There are separate budgets for ordinary callees (default 50), hinted/cross-crate
callees (100), and forwarding/deep callers (30). The sweep changes the second
one. The pinned
[selection code](https://github.com/rust-lang/rust/blob/822182db4c4a1827256881d8c7aa685f0bcf2b11/compiler/rustc_mir_transform/src/inline.rs#L368)
chooses between them and gives small bodies a quarter-budget bonus. Setting the
hint budget to zero is therefore not a blanket prohibition on inlining.

| Caller/body | Current decisions | Changed decisions |
|---|---|---|
| ergot `encode_frame_hdr` | Expands postcard `try_push_varint_u32`, its error mapping, and `Option::map(NameHash::to_u32)` | At ergot 50/postcard 0, those nested bodies stay out of the header encoder at this stage |
| ergot `EdgePort::send_raw` | Expands `HeaderSeq::clone`, nested address/key/appendix clones, `Header::from` and error mapping | At ergot 50, these bodies remain separate at MIR |
| postcard `serialize_u32` | Expands `Serializer::try_push_varint_u32` and `Result::map_err` | At postcard 0, it retains a call to `try_push_varint_u32` |
| bbqueue CAS `commit_inner` | Expands the public atomic methods, but retains internal atomic-load/store wrapper calls | At bbqueue 200, also expands `atomic_load`/`atomic_store` and their ordering dispatch/panic construction |
| sequential-storage `crc32_with_initial` | Expands range iteration/step/check wrappers | At storage 50, several of those scopes stay out of the CRC body |
| core `run_foc_cycle`, hint 100 -> 500, nonincremental | Some state/borrow helpers remain calls | Expands `RefCell::borrow_mut`/`try_borrow_mut`, state setters, closures and `FocDriver::derating` |
| core `update_phase_with_applied_voltage`, 100 -> 500 | Some voltage-reconstruction helpers remain calls | Expands `PhaseVoltageSense::phase_voltages`, three `counts_to_volts` instances and `applied_voltage` |
| F405 ADC, 100 -> 500, nonincremental | NTC helper bodies remain outside this MIR body | Expands `NtcConfig::temp_c_from_adc` and `libm::logf` |

The bbqueue change expands 4 store and 4 load wrapper sites in `commit_inner`.
The expanded MIR includes ordering switches and panic formatting even though
its callers use constant Acquire/Release orderings. Later simplification can
eliminate the inapplicable cases. This is early visibility of those operations,
not a change to memory ordering or queue semantics.

The NTC conversion is gated to **every 128th ISR cycle**, in
[F405 ADC](../../../oxifoc-f405/src/control/foc.rs). Its logarithm implements the
thermistor Beta model in
[`NtcConfig::temp_c_from_adc`](../../../oxifoc-core/src/foc/config.rs).
More MIR inlining here is not automatically an improvement to the main FOC
arithmetic. In fact the final ADC already has no call to logf in the baseline:
LLVM can inline it even when the MIR stage does not.

## MIR decisions are not the final call graph

The pinned [normal-pass policy](https://github.com/rust-lang/rust/blob/822182db4c4a1827256881d8c7aa685f0bcf2b11/compiler/rustc_mir_transform/src/inline.rs#L45)
skips normal MIR inlining for the local incremental builds in this project.
That does **not** disable LLVM inlining or fat LTO. These real counterexamples
show why sizes and MIR scopes alone were insufficient:

- Even at hint budget 500, MIR still calls `run_foc_cycle` from the ADC closure.
  The linked nonincremental image has it inlined into the real ADC handler by
  LLVM/LTO. Current and transport-only builds instead emit a separate
  **14020-byte** `run_foc_cycle`.
- MIR still calls `FocController::apply_dq` from `step_direct_voltage` at both
  nonincremental budgets 100 and 500. Those standalone calls disappear in the
  linked `step_direct_voltage`; LLVM has expanded the controller work there.
- With postcard 0, linked COBS `encode_frame_hdr` gains an actual BL to the
  shared `try_push_varint_u32`. Its size falls from **606 to 308 bytes**.
  That is the concrete size/call-overhead tradeoff, rather than free removal
  of serialization work. `EdgePort::send_raw` also becomes smaller and uses
  the shared header encoder instead of duplicating that work.
- The final images already contain no standalone internal atomic load/store
  wrapper symbols at the baseline. Raising bbqueue's MIR budget changes when
  the code becomes visible, not whether the final MCU executes generic
  out-of-line atomic wrappers.

## Hot-path assembly

For the **transport-only** CF2 candidate, compare six bodies after removing
linked branch addresses: ADC, `run_foc_cycle`, direct-voltage/current/velocity
steps and `update_phase_with_applied_voltage`.

All six have the same operation sequence, register operands and call targets.
The only remaining differences are MOVW immediates paired with MOVT that load
moved `.rodata` addresses: respectively **6, 17, 0, 1, 0, 7** sites. Direct-voltage
and velocity bodies match after branch-address normalization without even
those data-address differences. This is stronger evidence than equal sizes
and call counts; it still does not measure timing effects of flash placement
or changes in called transport code.

The broader balanced candidate changes ADC's register allocation, queue code
and instruction ordering, in addition to eliminating an indirect waker call.
Its 24-byte ISR reduction should not be read as only a 24-byte local edit.

Disabling incremental changes hot boundaries more substantially:

```text
current:
    step_current_control
      -> [PWM stores expanded in the caller]
          -> get_max_duty() for each of three phase writes

nonincremental:
    step_current_control
      -> MotorPwm::set_duties()       # separate 100-byte helper
          -> get_max_duty() for each of three phase writes
```

The three `get_max_duty` calls have moved, not been eliminated. The helper adds
its own call/prologue/epilogue on that path. Conversely, controller arithmetic
and other wrappers are expanded in other places. Static call totals do not
settle whether the net runtime tradeoff is favorable.

At global 500 versus nonincremental 100, the phase-update symbol grows from
**13212 to 13812 bytes**. Its two `probe_sample` calls and two `blend_angles`
calls disappear as standalone BL sites, along with an observer seed call and
one remainder helper call; extra `fmodf` calls and a defmt release appear elsewhere
in the expanded body. This is a redistribution of startup/observer work.
It is not a demonstrated improvement to the common current-control cycle.

The controller already measures DWT buckets for ADC work, snapshots, FOC and
publication, plus internal core buckets. Compare both ordinary cycles and
cycles with temperature conversion, startup/HFI, mode changes and telemetry
activity when evaluating the broader candidates. No hardware execution was
performed for this follow-up. Hardware sqrt remains present in every measured
F405 image; no unsupported `.f64` instructions or software sqrt were introduced.

## Reproducing the inspection

Use a fixed neutralized source worktree as in the original
[report](report.md#validation-and-reproduction). Start with an empty dedicated
build directory so Cargo cannot reuse an artifact without executing the dump
wrapper. Run from that worktree's `oxifoc-f405` directory:

```bash
repo=/home/okhsunrog/code/rust/oxifoc
study=/home/okhsunrog/tmp_zfs/codex_tmp/mir-inlining-repro
mkdir -p "$study"
export TMPDIR="$study"
export CARGO_BUILD_BUILD_DIR="$study/build"
export RUSTC_WRAPPER="$repo/docs/experiments/mir-threshold-sweep/dump-inline-wrapper.py"
export MIR_ANALYSIS_DUMP="$study/current/mir"
cargo build --release --locked -j4 -Zprofile-rustflags \
  --target-dir "$study/target" \
  --config 'profile.release.package.embassy-stm32.rustflags=["-Zinline-mir-hint-threshold=115"]'
export MIR_ANALYSIS_DUMP="$study/balanced/mir"
cargo build --release --locked -j4 -Zprofile-rustflags \
  --target-dir "$study/target" \
  --config 'profile.release.package.embassy-stm32.rustflags=["-Zinline-mir-hint-threshold=115"]' \
  --config "$repo/docs/experiments/mir-threshold-sweep/balanced.toml"
```

Save each ELF before starting another configuration. Repeat the balanced build
with `CARGO_INCREMENTAL=0`, then with the size preset, using separate dump
folders. Inspect `*.Inline.before.mir` / `*.Inline.after.mir` and compare scopes
and remaining calls. The wrapper also captures `ForceInline` dumps; these
must not be mistaken for the normal pass being enabled. The transport-only
preset does not change local inlining policy.

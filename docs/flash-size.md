# Firmware flash & RAM: budget, rules, history

Single place for everything firmware-size related — flash AND static
RAM: current numbers, how to measure, what to watch when adding code,
what has been done, and what reserves exist when a board runs out
again. Perf-side numbers (cycle counts) live in
[perf-bench-2026-06-11.md](perf-bench-2026-06-11.md).

## Current state (2026-06-13, after the g431 HFI/six-step slim)

| board | profile | flash used | flash region | headroom | static RAM (.data+.bss) | RAM region |
|---|---|---|---|---|---|---|
| g431 (B-G431B-ESC1) | **baked (the only profile)** | 113 824 (86%) | **128K** (no storage region) | **17.2 KB** | **18 712** | **32K** |
| g474 (Nucleo + IHM08M1) | storage | 171 784 | 256K (bank 1; bank 2 = config) | 88 KB | 21 748 | 128K |
| f405 | storage | 270 816 | 768K (sectors 0–9) | 503 KB | 32 236 | 128K (+64K CCM unused) |

g431 drops three things the drone board doesn't use, behind feature flags so
the roomy boards keep them: **six-step** (removed for all boards), **`hfi`**
(runtime HFI sensorless observer) and **`hfi-detect`** (rotating-injection +
FFT inductance measurement, pulls `microfft`). g474/f405 enable both HFI flags.
See the history table and [decisions.md](decisions.md → Firmware / platform).

`flash used` = `.vector_table + .text + .rodata + .data` (everything
that occupies flash; `.data` is load-image). Run `just size` for live
numbers. Everything RAM not claimed by `.data+.bss` is stack —
flip-link inverts the layout so the stack is bounded and overflow
faults instead of corrupting statics; on g431 every static byte saved
is a stack byte gained.

## How to measure

- **`just size`** — builds all STM32 firmwares and prints used/total
  per board. Run it after any feature lands on g431.
- **`cargo bloat --release --crates`** (in the firmware crate) — per-crate
  split. Caveats: embassy_executor's share is mostly *your* task bodies
  inlined into `poll()`; `[Unknown]` is mostly ISRs (ADC1_2 = the FOC
  ISR). Numbers are guesswork at the margins but fine for trends.
- **`cargo bloat --release -n 40`** — top symbols. Big async tasks
  dominate; that is normal.
- **`arm-none-eabi-nm --size-sort --print-size <elf> | grep -i <crate>`**
  — exact per-symbol sizes when bloat's attribution is unclear.
- **Who pulls a symbol in:** disassemble and grep for callers — this is
  how the libm dependency was traced to a single task:

  ```sh
  arm-none-eabi-objdump -d <elf> > /tmp/fw.asm
  # find functions containing a bl to the symbol
  grep -B400 'bl.*<symbol>' /tmp/fw.asm | grep -E '^[0-9a-f]{8} <' | tail -1
  ```

- **`.rodata` contents:**
  `arm-none-eabi-objcopy -O binary --only-section=.rodata <elf> /tmp/ro.bin && strings -n 8 /tmp/ro.bin`
  — on g431 it is ~13.4 KB, of which ~8 KB are panic/expect strings
  from dependencies (see *Dead ends*).
- The test crates (`tests/stm32*`) use `opt-level = "s"`; shipped
  firmware uses `"z"` — relative comparisons transfer, absolute sizes
  don't.

### Measuring RAM (the g431-critical axis since 2026-06-13)

- **`arm-none-eabi-size -A <elf>`** — `.data + .bss` = static RAM; the
  rest of the region is stack.
- **Embassy task arenas are the biggest single `.bss` consumers.** Each
  `#[embassy_executor::task]` owns a static `POOL` sized by its future —
  everything held across any `await` in the task (and in everything it
  awaits) lives there permanently:

  ```sh
  arm-none-eabi-nm --size-sort --radix=d <elf> | grep POOL
  ```

- **`large_futures` is the tripwire** (`future-size-threshold` in each
  board's `clippy.toml`, 2048 B on the STM32 boards, 4096 B in the ESP
  crates' *hidden* `.clippy.toml`s — note the device configs SHADOW the
  repo-root one). When it fires, clippy prints the future's exact size
  and the await site — this is how the 6 KB calibration buffer was
  found. Legitimately-large task futures get
  `#[expect(clippy::large_futures, reason = ...)]`.

## Standing build configuration (already maximal)

All three boards: `opt-level = "z"` (g431) / per-board, `lto = "fat"`,
`codegen-units = 1`, `build-std = ["core"]` (rebuilds libcore at "z"
instead of the shipped -O3), flip-link (RAM safety, not flash).
`debug = 2` costs zero flash — debug info is not loaded. There is no
remaining "compiler flag" win; everything below is about code.

## Rules when adding firmware code

1. **Never call `libm::sinf/cosf` (or anything trig) from
   firmware-reachable code.** The hardware-required `-fp64` rustflag
   makes every f64 op a softfloat call, and libm's `sinf/cosf` do
   argument reduction in f64 (`rem_pio2f`): the first reachable call
   site pays ~5 KB of flash (rem_pio2f + k_sinf/k_cosf + the entire
   f64 softfloat runtime) and ~6200 cycles/pair at runtime. Use the
   `SinCos` trait backends (`FastSinCos`, `CordicSinCos`) or
   `fast_math`. As of 2026-06-11 the only libm left in g431 is
   `atanf` + `remainderf` (602 B, pure f32, kept for exactness) —
   check it stays that way:

   ```sh
   arm-none-eabi-nm --size-sort --print-size <elf> | grep -i libm
   ```

2. **`libm::sqrtf` → `crate::foc::fast_math::sqrtf`** everywhere in
   oxifoc-core: it is `vsqrt.f32` on target (bit-identical IEEE, 4.4×
   faster) and libm on host. Inside `macro_rules!` use `$crate::`.

3. **`defmt::unwrap!` / `defmt::assert!` instead of plain
   `unwrap()/expect()`** in firmware crates. Plain unwrap pins
   `core::fmt` Debug formatting and puts the message in `.rodata`;
   defmt interns the string in `.defmt` (zero flash) and prints the
   payload if the type is `defmt::Format`. Note embassy-executor 0.10:
   the task fn returns `Result<SpawnToken>`, so the pattern is
   `spawner.spawn(defmt::unwrap!(my_task(...)))`.

4. **New dependency? Check for a `defmt` feature** (`cargo info <crate>`)
   and enable it: switches the crate's internal asserts to interned
   defmt strings and derives `Format` on its error types. Already on (all three boards):
   embassy-stm32/executor/sync/time/futures/embedded-hal, embassy-usb
   (f405/g474), postcard (`use-defmt`), heapless, embedded-io-async,
   sequential-storage, oxifoc-core. Deliberately off: `rtt-target/defmt` (defmt→RTT already
   goes through the ergot sink), `postcard-schema/defmt-v0_3` (would
   pull a second defmt 0.3 next to 1.0).

5. **Async task bodies are the biggest single symbols** — everything
   awaited gets inlined into the task's `poll()`. A new server/task on
   g431 is typically 1–10 KB of flash AND its full future size in
   `.bss` (see *Measuring RAM*). Budget for both; run `just size`
   before and after.

6. **Async-bloat rules** (from the 2026-06-13 debloat pass; background:
   tweedegolf "Debloat your async Rust"):
   - **No buffers across awaits.** Anything alive across an `await` is
     stored in the future, i.e. permanently in the task arena. Stream
     or chunk instead (the boot calibration held a 6 KB sample buffer
     across its sampling delays — to compute a mean).
   - **Pure trait pass-throughs return the inner future**:
     `fn f(..) -> impl Future<Output = T> { inner(..) }` instead of
     `async fn f(..) { inner(..).await }` — the async body wraps the
     inner future in one more generated state machine for nothing
     (DetectionBackend impls were this; −1 KB across g431+f405).
   - **Don't "optimize" awaits out of match arms by buffering.** The
     coroutine layout already overlap-allocates per-arm states
     (`storage_conflicts`); hoisting commands into a Vec that lives
     across one shared await made flash AND arena *bigger* (+176/+232 B,
     measured, reverted). Collapsing await points only pays when arms
     duplicate the same awaited code path.
   - `async fn` with no `await` is still a state machine — return
     `core::future::ready(..)` or drop the async (lint: `unused_async`).

7. **Generics multiply:** each concrete instantiation of a generic
   server/driver is a separate copy. Reuse existing instantiations
   (e.g. the one `NetStack` type) rather than introducing new type
   parameters in firmware.

## What was done (history)

| date | change | g431 effect |
|---|---|---|
| 2026-06-11 | SinCos trait threading + fast_math (perf fix; CORDIC on G4, FastSinCos on F405) | +664 B (HFI: 150% → 13.9% of ISR budget) |
| 2026-06-11 | observer upgrades (centering, λ tracking, active flux, …) | → 126 656, headroom **320 B** — crisis |
| 2026-06-11 | **libm diet** (`7a82d70`): hall_calibration → FastSinCos (the *only* firmware sinf/cosf — found by disassembly: every libm call hung off the detect_server task), detect-path sqrtf → vsqrt. Zero accuracy loss (<1e-6 sin/cos, bit-identical sqrt; atan2f/remainderf untouched) | **−6 872 B** |
| 2026-06-11 | `detection` feature gate, default-on (`a13f229`); `just check` compiles the gate-off config so it can't rot | 0 (reserve, see below) |
| 2026-06-11 | `defmt::unwrap!` conversion (14 sites) + defmt features on deps (`4089675`) | **−1 116 B** |
| | **total** | **126 656 → 118 668** |
| 2026-06-11 | same defmt treatment ported to f405/g474 (38 sites; embassy-usb defmt too) | f405 −1 044 B, g474 −1 024 B |
| 2026-06-12 | detection review fixes: guaranteed-delivery `send_command` (async, ~27 await sites), adaptive HFI amplitude, magnitude flux method replacing q-axis in the per-step server, bus-voltage clamps | **+1 760 B** (108 620 → 110 380 baked) |
| 2026-06-12 | **g431 storage profile removed** (see below) — that +1 760 B overflowed the 124K storage layout by ~700 B, forcing the standing decision | storage profile gone; baked unaffected |
| 2026-06-12 | per-step detect server gets the same method ladders as `run_full_detection` (`measure_inductance_auto`: HFI → voltage-pulse fallback; `measure_flux_linkage_auto`: spin-down gate → driven) — the pulse machinery is now reachable from firmware | **+2 676 B** (110 380 → 113 056) |
| 2026-06-12 | HFI run-gating (update+injection paired, off in non-Hfi sources — ~10% of the ISR budget back in the hall ride config), carrier pre-heat margin + `restart_demod`, amplitude solved from measured L | +400 B (→ 113 456) |
| 2026-06-12 | motor RATING layer: `MotorParamsConfig` +rating/+power-class, limits clamp `min(operational, rating, board)`, trip ≤ 1.5×rating, HFI ripple target from rating | +728 B (→ 114 184, headroom 16.9 KB) |
| 2026-06-13 | fault overhaul phases 1–5: severity classes + class gate, deadman→CommTimeout, limit-ladder fixes, hall wire detector + sticky warning bridge, FaultTopic publisher, derating module + `derating` config group + voltage-fault integrals + `run_protection` consolidation | +5 244 B (→ 122 232, headroom 8.8 KB) |
| 2026-06-13 | code-quality pass (lints, imports, StandardFault dedup) | +60 B (→ 122 292) |
| 2026-06-13 | async debloat: DetectionBackend pass-throughs, streaming-mean calibration (the big win is RAM: .bss 23 860 → 17 716, main task arena 7 736 → 1 608 — the 6 KB sample buffer is gone) | −412 B (→ 121 880, headroom 9.2 KB) |
| 2026-06-13 | phase-voltage sensing core (capability + converter + observer wiring; `AdcSnapshot.vphase`) — additive, no g431 sensing yet | +856 B (→ 122 736) |
| 2026-06-13 | **six-step removed** entirely (`ControlMode::SixStep`, `motor::six_step`, host-cli) — unused | −812 B (→ 121 924, all boards: g474 −148, f405 −508) |
| 2026-06-13 | **`hfi-detect` feature gate** (HFI inductance: rotating injection + FFT, `microfft`); g431 off → voltage-pulse only. The visible `microfft` symbols are ~588 B; the rest is the sweep monomorphized over SinCos/Hardware generics | **−6 076 B** (→ 115 848, headroom 15.2 KB) |
| 2026-06-13 | **`hfi` feature gate** (runtime HFI sensorless observer + PhaseManager slot); g431 off (no saliency on a drone motor). `PhaseSource` Hfi* variants kept for wire compat, rejected at runtime | **−2 024 B** (→ 113 824, headroom 17.2 KB; RAM −440 B) |

Panic handler kept `defmt::error!("PANIC: {}", Display2Format(info))`:
full panic text over RTT costs only 240 B once dependency fmt is gone
(measured), and the gate-kill ordering in safety.rs is untouched.

## g431 profiles: baked config (2026-06-11)

The 2026-06-11 safety/velocity work (deadman + failsafe, parking brake,
velocity loop, two config groups) cost **+6.2 KB** and dropped the g431
storage-profile headroom to ~2 KB. Per-commit attribution (rebuilt at each
commit): deadman+failsafe +3 344 B, brake+hardening +484 B, velocity loop
+680 B, ControlledStop v2 +676 B, velocity config persistence +828 B.

Measurements of candidate diets (temporary-patch builds, 2026-06-11):

| build | size | note |
|---|---|---|
| full (detection + storage) | 124 880 | the crisis state |
| no detection | 109 732 | −15.1 KB |
| detection, **no storage** | 99 340 | −25.5 KB (!) |
| no detection, no storage | 83 888 | −41.0 KB |

Key finding: **flash storage + config server cost −25.5 KB**, far more than
the −15.9 KB previously estimated — the config server drags in the postcard
codecs for every group, the TOCTOU machinery and a fat ergot server state
machine. So the decisive lever was removing runtime persistence, not
detection.

**Decision (2026-06-11): g431 defaults to the *baked-config* profile.**
**Decision (2026-06-12): the g431 `storage` profile is REMOVED entirely** —
the 2026-06-12 detection fixes (+1 760 B) overflowed the 124K storage layout
by ~700 B, and rather than dieting a reserve profile nobody flashes, the
board gave up runtime persistence for good. The 4K config region belongs to
code now.

What g431 has (the only profile):

- Configuration is compiled in (`src/baked_config.rs`), memory.x grants the
  full 128K to code, the config server runs RAM-backed (reads/writes/
  live-apply work — live tuning on the bench — but nothing persists across
  reboots; the server reports persist-capable = false). Detection stays in.
  Workflow: flash → detect → tune live →
  `oxifoc-host-cli config dump --rust > src/baked_config.rs` → rebuild →
  reflash.
- f405/g474 keep flash-backed storage as their default — they have flash to
  spare and persistence is convenient there.

Removed with the profile: `oxifoc-g431/src/storage.rs` (flash worker +
sequential-storage), `memory-storage.x` + the `build.rs` feature switch and
its `FIRMWARE_END_OFFSET` overlap assert (nothing to overlap any more),
the `sequential-storage`/`embedded-storage-async`/`embassy-embedded-hal`
deps, and the storage-profile steps in `just check`/`just size`. To ever
bring it back: revert the removal commit — but it must re-fit in 124K.

### Why NOT a separate detection firmware (the two-image idea, evaluated)

A detect/run firmware split only makes sense if the detect image also
*removes* run-only functionality — otherwise detect ⊇ run, and the moment
run stops fitting, detect stopped fitting earlier. Right now a "detect
image" would just be the run image + detection (+15.5 KB), pointless while
a single image holds everything with ~24 KB of headroom. The ladder when
pressure returns:

1. **Now**: one g431 image, baked profile — detection + safety + velocity,
   ~24 KB headroom.
2. **When tight**: build with `--no-default-features --features
   transport-uart` (detection off, −15.5 KB) — re-detection then needs a
   temporary reflash with detection on.
3. **Last resort**: the symmetric two-image split — requires core feature
   gates for the *run-only* subsystems detection doesn't need (sensorless
   estimators, velocity loop, failsafe machinery ≈ 8–13 KB): detect image =
   run − those + detection. Only then are two images genuinely
   complementary. Gating safety behind features is unpleasant; do this only
   under real pressure.

Additional measured idea for later: the per-config-group cost is dominated
by postcard codecs (+828 B for a 3-field group) — check whether the
`postcard_schema::Schema` derives are needed on-device at all or only by
the host; gating them could shave ~1 KB per group.

## Measured reserves (when g431 gets tight again)

Measured 2026-06-11 by temporarily removing the root reference and
letting fat LTO drop the subtree — re-verify when invoked, the numbers
age:

- **Runtime HFI (HfiObserver + polarity probe + injection plumbing)
  behind a separate feature: not implemented, estimate if ever needed.**
  The protocol enum PhaseSource cannot be gated (postcard indices); the
  manager already degrades via `hfi: Option` + `HfiNotConfigured`. The
  decision NOT to do it now — decisions.md 2026-06-12.
- **`detection` off: −14.7 KB.** The gate already exists
  (`oxifoc-core/detection`, default-on). Build the board with
  `default-features = false` + the rest of its feature list.
  `detection::types` and `pi_tuning` remain available unconditionally.
  Trade-off: motor must be configured with known params from the host.
- **`transport-rtt` instead of `transport-uart`: −2.6 KB.** Already a
  feature flag. Trade-off: device only talks through a debug probe —
  not for field use; default stays UART.
- **No persistent storage: −25.5 KB measured 2026-06-11** (storage_worker
  task + sequential-storage + flash driver + config server + postcard
  codecs; the old −15.9 KB estimate missed the config-server share).
  **Spent: this is now the only g431 configuration** (storage profile
  removed 2026-06-12, see above) — no longer a reserve. The RAM-backed
  config server itself is still compiled in; gating the per-group postcard
  codecs (~1 KB/group, see above) remains an unspent idea.

## Dead ends (evaluated, rejected)

- **`panic_immediate_abort`** would drop ~8 KB of dependency
  panic/expect strings and most of `core::fmt` — **forbidden**: it
  bypasses `#[panic_handler]`, i.e. the gate-kill in safety.rs. Motor
  safety wins.
- **`.rodata` panic strings (~8 KB)** come from dependency panic sites
  (embassy_sync, maitake, ergot, sequential_storage, microfft, cobs:
  slice indexing, `unwrap_failed` with Debug payloads). Not removable
  from our side; our own sites are already defmt.
- **Location-only panic handler** (drop `Display2Format`): saves 240 B,
  loses the panic message text. Not worth it.
- **Removing CORDIC** in favor of FastSinCos everywhere: user decision
  to keep it; it is also the fastest backend measured (103 cyc/pair)
  and its q31 path is hardware-validated.

# Firmware flash: budget, rules, history

Single place for everything flash-size related: current numbers, how to
measure, what to watch when adding code, what has been done, and what
reserves exist when a board runs out again. Perf-side numbers (cycle
counts) live in [perf-bench-2026-06-11.md](perf-bench-2026-06-11.md).

## Current state (2026-06-11)

| board | flash used | region | headroom | pressure |
|---|---|---|---|---|
| g431 (B-G431B-ESC1) | 118 668 | 124K (4K reserved for config) | **8.3 KB** | the constrained one |
| g474 (Nucleo + IHM08M1) | 155 584 | 256K (bank 1; bank 2 = config) | 104 KB | none |
| f405 | 232 072 | 768K (sectors 0–9) | 554 KB | none |

`flash used` = `.vector_table + .text + .rodata + .data` (everything
that occupies flash; `.data` is load-image). Run `just size` for live
numbers.

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
   g431 is typically 1–10 KB. Budget for it; run `just size` before
   and after.

6. **Generics multiply:** each concrete instantiation of a generic
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

Panic handler kept `defmt::error!("PANIC: {}", Display2Format(info))`:
full panic text over RTT costs only 240 B once dependency fmt is gone
(measured), and the gate-kill ordering in safety.rs is untouched.

## Measured reserves (when g431 gets tight again)

Measured 2026-06-11 by temporarily removing the root reference and
letting fat LTO drop the subtree — re-verify when invoked, the numbers
age:

- **`detection` off: −14.7 KB.** The gate already exists
  (`oxifoc-core/detection`, default-on). Build the board with
  `default-features = false` + the rest of its feature list.
  `detection::types` and `pi_tuning` remain available unconditionally.
  Trade-off: motor must be configured with known params from the host.
- **`transport-rtt` instead of `transport-uart`: −2.6 KB.** Already a
  feature flag. Trade-off: device only talks through a debug probe —
  not for field use; default stays UART.
- **No persistent storage: −15.9 KB** (storage_worker task +
  sequential-storage + flash driver + config postcard codecs; overlaps
  with the detection number — they share the config codecs, don't sum
  them). No feature gate exists; only worth building for a
  hypothetical hardcoded-config minimal target, pairs with
  detection-off.

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

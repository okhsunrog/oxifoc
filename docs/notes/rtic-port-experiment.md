# RTIC port experiment (oxifoc-f405)

Branch-only experiment (`claude/rtic-port-experiment-elsrsx`): the f405
platform crate runs on **RTIC 2.3** instead of embassy-executor. Everything
else is unchanged — the embassy-stm32 HAL, embassy-time (TIM2 time driver),
embassy-sync channels/signals, ergot, and all of oxifoc-core. The g431/g474
crates and the host workspace are untouched.

## Why

The hot path was never the motivation: the 20 kHz FOC loop is already a bare
max-priority ISR on every platform, and RTIC cannot make it faster. The
experiment targets the *thread-mode* side, where the cooperative executor has
a documented history:

1. **TX starvation** (g431, 2026-07-05): the RTT TX pump starved on the
   single cooperative executor; moving it to a medium-priority
   `InterruptExecutor` fixed throughput (10.5k → 14.6k samples/s)…
2. **…but froze all thread timers** for a deterministic ~44.93 s during
   detection+streaming — root cause in the gp16 time-driver interaction
   never found (see `oxifoc-g431/src/protocol.rs`, `run_tx_rtt` NOTE).
3. **Drive-engage deadman trips** (2026-07-06): ISR budget overruns starving
   thread mode, diagnosed with the exec-probe/timer-lateness instrumentation.

RTIC's model — multiple async executors at hardware-managed priorities — is
the structural answer to (1) without hand-rolled `yield_now()` pacing. The
port exists to measure whether that holds, and what it costs.

## Questions this branch answers

1. Do the I/O pumps hold throughput under detection+streaming load without
   starving, and **without** a thread-timer freeze? (The freeze mechanism —
   embassy-executor's integrated timer queue — is structurally absent here;
   see "generic timer queue" below.)
2. What does timer lateness look like under RTIC vs. the cooperative
   executor? The `probe/s: late_max=` line (1 kHz probe in `main.rs`) is the
   direct counterpart of the g431's `exec_probe_task`, and the `isr/s` lines
   are byte-compatible with the bench-suite parser.
3. How much friction does RTIC + embassy-stm32 + ergot actually generate?
   (Answered below — four concrete findings.)

## What changed

- `main.rs` is a `#[rtic::app]`. Sync hardware bring-up happens in
  `#[init]`; the config-dependent half of boot (config load → DRV8301 →
  `foc::init` → watchdog arm) moved into an async `startup` task, preserving
  the old ordering.
- The ADC (FOC loop) and TIM3 (hall capture) `#[interrupt]` handlers became
  RTIC hardware tasks calling `adc_isr(seq)` / `tim3_isr()`. Their NVIC
  priorities are now declared, not hand-poked: logical 16 → NVIC 0x00 and
  logical 15 → NVIC 0x10, both identical to the pre-port values. The old
  `static mut SEQ` handler-local is an RTIC task-local.
- The ~15 embassy tasks became RTIC software tasks in **two priority
  tiers**: I/O pumps (USB device + RX/TX workers, UART RX/TX) at priority 2,
  everything else (servers, detection, storage, telemetry streams, state
  monitor, LEDs, stats, startup) at priority 1. Dispatchers: the unused
  UART4/UART5 vectors. This is the tier layout the g431 starvation finding
  asks for: a long server step can no longer starve the byte pumps.
- Task bodies stayed where they were (`protocol/servers.rs`, `storage.rs`,
  `drv8301.rs`, `control/foc.rs`) as plain `async fn`s; the app module holds
  thin wrappers. Comms peripheral ISRs (OTG_FS/USART3/RNG/EXTI/TIM2) remain
  embassy-managed at their pre-port NVIC priorities, above both dispatchers.
- No RTIC shared resources yet: all state stays in the existing
  critical-section statics and atomics, so the diff is an executor swap, not
  a locking redesign. SRP-based ceilings for `FOC_DRIVER`/`STATE` (cheaper
  than global critical sections) would be a follow-up experiment.

## Friction findings (question 3, already answerable)

1. **embassy-time needs its generic timer queue.** embassy-stm32 0.6's time
   driver defaults to the *integrated* timer queue, which stores queue items
   in embassy-executor's task headers (undefined symbol
   `__embassy_time_queue_item_from_waker` at link time without it). Fix: a
   direct dependency on `embassy-time-queue-utils` with `generic-queue-64` —
   (instant, waker) pairs by value, executor-agnostic. Consequences worth
   knowing: pending timers are now bounded (64) and enqueue/dequeue is
   O(n) in pending timers rather than the intrusive-list version — but the
   *g431 freeze mechanism lived in the integrated path*, so this swap is
   itself part of the experiment.
2. **rtic-macros resolves every task signature even under a disabled
   `#[cfg]`.** Tasks whose argument types only exist under `transport-rtt`
   fail name resolution when the feature is off. The diagnostic RTT
   transport is therefore compile_error'd out on this branch (USB + UART,
   the production interfaces, work).
3. **`embassy_usb::UsbDevice` is `!Send`** (holds `&mut dyn Handler`), and
   RTIC spawn arguments must be `Send` because spawn queues are statics.
   `SendMove<T>` in main.rs is the documented single-core, move-once escape
   hatch.
4. **Macro-expanded `#[macro_export]` macros (assign-resources'
   `split_resources!`) cannot be named from inside the app module** — Rust
   refuses absolute paths to them and textual scope doesn't reach in. The
   resource structs are hand-expanded in `#[init]`; keep them in sync with
   `hardware/resources.rs`.

## Step 2: how far can embassy-time be removed?

Tested by migrating everything the f405 crate controls onto
**rtic-monotonics** (TIM5 @ 1 MHz, `src/time.rs`): LED loops, the 1 kHz
timer probe, `first_vbus_v`, the isr-stats cadence, the UART TX timeout, and
the core `Timer`-trait consumers (fast-telemetry stream) via a `MonoTimer`
impl. The TIM5 IRQ handler is macro-generated; rtic-monotonics reads RTIC's
`RTIC_ASYNC_MAX_LOGICAL_PRIO` symbol and places it just above the async
dispatchers — no manual NVIC work.

**Full removal is blocked by ergot**, whose `embassy-usb-v0_6` and
`nostd-seed-router` features (both required here) carry unconditional
`dep:embassy-time`, used for: RX-worker liveness timestamps
(`transports/eio.rs`, `eusb_0_6.rs`, `packet.rs`), router interface-state
`Instant`s (`profiles/router.rs`), and seed-router lease timing
(`net_stack/services.rs`). Removing embassy-time from the build means
adding a time abstraction to ergot (we own the fork — that list sizes the
change). Two smaller stragglers live in oxifoc-core, deliberately untouched
on this branch: `detection/embassy_hw.rs` hardcodes `EmbassyTimer` (the
sweep fns are already generic — the wrappers just pin the type) and the
current-offset calibrate delay in `foc/sensors.rs`.

Consequences of the resulting **dual timebase**:

- Cost: ~3.4 KB flash and ~2.2 KB .bss on top of the RTIC port (rtic-time
  queue + TIM5 backend), while the embassy-time driver + generic queue stay
  for ergot. Strictly worse on size — this step buys diagnosis, not bytes.
- Diagnostic value: app timing (probe, UART timeouts, telemetry pacing) now
  runs on rtic-time's intrusive queue, ergot's timing still on the gp16
  driver + generic queue. If the g431-class freeze ever reproduces, which
  half stalls identifies the faulty layer immediately. Note the 1 kHz probe
  no longer watches embassy-time at all — ergot liveness/timeout behavior
  is the embassy-time health signal now.
- `Ticker` has no rtic-monotonics equivalent; the isr-stats cadence uses
  the `delay_until(next += 1s)` pattern instead.

## Flash / RAM vs. the embassy baseline

Release builds, default features (`transport-usb,transport-uart,board-cf2`),
RTIC branch vs. `main` @ c610fbf, section sizes from readelf:

| section | embassy | RTIC | Δ |
|---|---|---|---|
| .text | 315,248 | 317,928 | **+2,680** |
| .rodata | 17,732 | 16,676 | −1,056 |
| .data | 2,224 | 3,252 | +1,028 |
| **flash total** | **335,204** | **337,856** | **+2,652 (+0.8%)** |
| .bss | 32,856 | 18,888 | −13,968 |

- **Flash: +2.6 KB** for the executor swap (dispatcher trampolines,
  per-priority executors, the generic timer queue's O(n) insert). Irrelevant
  on the F405 (768K firmware region), but decision-relevant for any future
  g431 port: that board could not afford 650 B for ISR profiling in the
  default profile (docs/flash-size.md), so ~2.6 KB likely doesn't fit.
- **.data +1,028 B** is the generic timer queue: the embassy-stm32 `DRIVER`
  static grows 56 → 1,056 B (64 × 16 B slots).
- **The .bss drop is an accounting artifact, not freed RAM.** The 18
  embassy task `POOL` statics (14,032 B of task futures) disappear, but
  RTIC 2.3 allocates its async-task executors in `main`'s stack frame
  (`AsyncTaskExecutorPtr::set_in_main` — the 4-byte `*_EXEC` statics just
  point there), and `main` never returns. The same ~14 KB now lives in the
  stack region where `size(1)` and flip-link's static accounting cannot see
  it. Net static RAM is roughly a wash (slightly worse, +1 KB of .data);
  treat stack-headroom numbers on this branch with suspicion until measured
  with a paint/high-water check on hardware.

## Status / how to run

- Builds clean (dev + release, both boards): `cargo build` /
  `cargo build --no-default-features --features
  transport-usb,transport-uart,board-vesc6-mk5`; clippy clean; release text
  ~335 KB (fits the 768K firmware region with margin).
- **Not yet hardware-tested.** Next step is the A/B on a CF2/MK5 bench:
  baseline `main` vs. this branch, comparing `probe/s: late_max`, `isr/s
  over=`, and host-measured telemetry samples/s under detection+streaming
  load — the exact loads that produced findings (1)–(3) above.

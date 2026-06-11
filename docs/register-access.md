# MMIO status-register access: why `modify` is banned on SR

Verified 2026-06-11 against the stm32-metapac source and the generated
Cortex-M assembly (release build of oxifoc-g431). This documents the
finding so the next person doesn't have to re-derive it — or worse,
"simplify" a complement write back into a `modify`.

## The rule

**Never use `reg.sr().modify(...)` to clear a flag in an rc_w0 status
register** (STM32 `TIMx_SR` and friends). Use
[`oxifoc_core::clear_rc_w0!`](../oxifoc-core/src/lib.rs):

```rust
oxifoc_core::clear_rc_w0!(pac::TIM1.sr(), |w| w.set_bif(0, false));
oxifoc_core::clear_rc_w0!(pac::TIM4.sr(), |w| {
    w.set_uif(false);
    w.set_ccof(0, false);
});
```

## Why: the `modify` race

rc_w0 semantics (RM0440, register description conventions): software
clears a flag by writing **0**; writing **1 has no effect**. Hardware sets
the flags asynchronously to the CPU.

metapac's `modify` (stm32-metapac `src/common.rs`) is a plain volatile
read → closure → volatile write of the whole snapshot:

```
ldr  r1, [r0]       ; SR snapshot
bic  r1, r1, #128   ; zero our flag in the snapshot
str  r1, [r0]       ; write the snapshot back
```

Any flag that hardware sets **between the `ldr` and the `str`** is 0 in
the snapshot, gets written back as 0, and is silently erased — the
consumer never sees the event. The window is only a few cycles plus the
APB bridge latency, but on a 20 kHz × forever ISR schedule "rare" still
happens.

This is not theoretical: embassy's own time driver hit it and documents
it (`embassy-stm32/src/time_driver/gp16.rs`):

> Clear all interrupt flags. Bits in SR are "write 0 to clear", so write
> the bitwise NOT. Other approaches such as writing all zeros, or RMWing
> won't work, they can miss interrupts.

ST HAL does the same (`__HAL_TIM_CLEAR_FLAG` ⇒ `SR = ~FLAG`). Writing 1s
into reserved bits is fine — both vendors have shipped that for years.
Note embassy is internally inconsistent (`timer/low_level.rs` still uses
`modify` in two helpers), so "embassy does it" is not an argument either
way — the time-driver comment is the one written *after* the bug.

## What the macro compiles to

The macro builds an **all-ones template** and lets the body zero only the
flags to clear, in a single `write` — no SR read at all:

```
; clear_rc_w0!(TIM1.sr(), |w| w.set_bif(0, false))
mvn  r1, #128       ; constant 0xFFFF_FF7F
str  r1, [r0]
```

Every other bit is written 1 ⇒ no effect, regardless of what happened in
any window. The conditional form (hall `clear_flags`) folds to
`mov r2, #-1` + conditional `bic`s + one `str` — still zero reads.

It is a macro, not a function, because metapac fieldsets (`SrAdv`,
`SrGp16`, `SrGp32`, …) share no raw-access trait — the type is inferred
at the call site.

## Where the race actually mattered here

- **Hall capture timers (TIM4 g431 / TIM3 f405 / TIM2 g474)** — real
  consumers: a lost `UIF` breaks the 16→64-bit timebase extension (a
  65 ms hole in velocity math), a lost `CC1IF` drops a hall edge. The
  complement write is load-bearing there.
- **TIM1 `BIF` (g431 enable() + ADC ISR)** — today nothing else consumes
  TIM1 SR flags, so the old `modify` was harmless *in practice*; the fix
  is correctness + future-proofing (the moment someone polls another
  TIM1 flag, the race is live).

## The two lookalike footguns

1. `reg.sr().write(|w| w.set_x(false))` — starts from **zeros**, i.e.
   clears *every* flag in the register, not just `x`. Looks almost
   identical to the safe pattern. This is why the safe pattern has a
   name: any bare `sr().write`/`sr().modify` in review is a red flag.
2. **rc_w1 registers** (e.g. the G4 ADC `ISR` — "write 1 to clear"):
   `clear_rc_w0!` is exactly wrong there — an all-ones write clears
   everything, and `modify` is *also* racy in the opposite direction
   (writing back a read 1 clears a flag someone else was about to
   consume). For rc_w1, write zeros + 1s in the bits to clear. We
   currently have no rc_w1 handling of our own (embassy's `InjectedAdc`
   owns the ADC flags).

## Auditing

`sr().modify` greps miss line-wrapped calls (that's how the ADC-ISR
instance survived the first sweep). Use a multiline-tolerant search:

```sh
grep -rn -E "\.(sr|isr)\(\)\s*$|\.(sr|isr)\(\)\." oxifoc-*/src tests \
  | grep -v "\.read()"
```

and eyeball every hit that isn't `clear_rc_w0!`.

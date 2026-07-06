/* STM32G431CB: 128KB Flash, 32KB RAM (SRAM1 16K + SRAM2 6K + CCM 10K).
 * No flash-backed config storage on this board (removed 2026-06-12,
 * docs/flash-size.md): the full 128KB belongs to the program; configuration
 * is baked at build time (src/baked_config.rs).
 *
 * Stack lives in CCM SRAM (2026-07-06): with the old single 32K region and
 * flip-link, the stack got whatever the ~25K of statics left over — measured
 * 7.48K with an idle high-water mark of ~7.07K (408 B headroom), and every
 * deep drive-engage path overflowed it into a HardFault + IWDG reboot.
 * CCM is dual-mapped: native 0x1000_0000 and aliased at 0x2000_5800 (glued
 * after SRAM2), so RAM here is capped at 22K — statics must NOT spill into
 * the alias of the stack. The stack gets the full native CCM block:
 *   - fixed 10K budget, decoupled from static growth,
 *   - overflow protection for free: nothing is mapped below 0x1000_0000, so
 *     running off the bottom is an immediate BusFault (gate-kill handler)
 *     instead of silent static corruption — flip-link is no longer needed,
 *   - zero-wait-state on the dedicated core port: the host's constant
 *     SWD/RTT polling of SRAM no longer contends with stack traffic.
 * CCM is CPU-only (no DMA) — fine, nothing DMAs on this board.
 */
MEMORY
{
    /* Program flash: full 128KB */
    FLASH  : ORIGIN = 0x08000000, LENGTH = 128K

    /* SRAM1 + SRAM2 via the contiguous alias mapping: statics only */
    RAM    : ORIGIN = 0x20000000, LENGTH = 22K

    /* CCM SRAM, native mapping: 9K stack + 1K relocated statics */
    CCMSTACK : ORIGIN = 0x10000000, LENGTH = 9K
    CCMDATA  : ORIGIN = 0x10002400, LENGTH = 1K
}

/* Stack grows DOWN from 9K into CCM toward the unmapped hole below
 * 0x1000_0000 (hard overflow protection; measured drive peak ~7.6K).
 * `_stack_end` marks the lowest legal stack address for cortex-m-rt's
 * sanity checks (its default points at the top of the statics in RAM —
 * a different region now). */
_stack_start = ORIGIN(CCMSTACK) + LENGTH(CCMSTACK);
_stack_end = ORIGIN(CCMSTACK);

/* CPU-only statics relocated out of the 22K SRAM squeeze (see
 * src/transport.rs / protocol.rs `#[link_section = ".ccmdata"]`).
 * NOLOAD: cortex-m-rt does NOT zero this — main() zeroes the whole
 * region in its first block, before any of these statics are touched.
 * Only zero-representation statics (StaticCell, plain buffers) go here. */
SECTIONS
{
    .ccmdata (NOLOAD) : ALIGN(4)
    {
        *(.ccmdata .ccmdata.*);
    } > CCMDATA
} INSERT AFTER .uninit;

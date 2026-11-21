/* Hot-path entry residues within 16-byte Flash lines on the F405.
 *
 * The control ISR's cost moves with the address residue (mod 16) of the
 * functions it runs: ART prefetch and Flash line boundaries. Left in the
 * natural .text layout, an unrelated edit anywhere in the image shifted
 * them and moved the ISR by -15...+21 cycles (MK5, 15 kHz). Each function
 * below is named in source by `link_section` and pinned at its measured
 * best residue; the rest of .text then moves the ISR by at most ~5 cycles.
 * Residues were swept one function at a time on the MK5 at 15 kHz,
 * HallToObserver velocity control at +120 and +450 electrical rad/s.
 * Re-measure them when these functions change substantially. */
SECTIONS
{
  .foc_hot : ALIGN(16)
  {
    . = ALIGN(16) + 8;
    __foc_cycle_start = .;
    KEEP(*(.text.foc_cycle));
    __foc_cycle_end = .;
    . = ALIGN(16) + 8;
    __foc_phase_start = .;
    KEEP(*(.text.foc_phase));
    __foc_phase_end = .;
    . = ALIGN(16);
    __foc_grant_start = .;
    KEEP(*(.text.foc_grant));
    __foc_grant_end = .;
    . = ALIGN(16) + 8;
    __foc_adc_start = .;
    KEEP(*(.text.foc_adc));
    __foc_adc_end = .;
    . = ALIGN(16) + 4;
    __foc_velocity_step_start = .;
    KEEP(*(.text.foc_velocity_step));
    __foc_velocity_step_end = .;
    . = ALIGN(16);
    __foc_current_step_start = .;
    KEEP(*(.text.foc_current_step));
    __foc_current_step_end = .;
    . = ALIGN(16) + 12;
    __foc_svpwm_start = .;
    KEEP(*(.text.foc_svpwm));
    __foc_svpwm_end = .;
    . = ALIGN(16);
    __foc_blend_start = .;
    KEEP(*(.text.foc_blend));
    __foc_blend_end = .;
    . = ALIGN(16) + 4;
    __foc_injection_start = .;
    KEEP(*(.text.foc_injection));
    __foc_injection_end = .;
  } > FLASH
}
INSERT BEFORE .rodata;

ASSERT(__foc_cycle_end > __foc_cycle_start, "Missing FOC cycle section");
ASSERT(__foc_phase_end > __foc_phase_start, "Missing FOC phase section");
ASSERT(__foc_grant_end > __foc_grant_start, "Missing FOC grant section");
ASSERT(__foc_adc_end > __foc_adc_start, "Missing ADC handler section");
ASSERT(__foc_velocity_step_end > __foc_velocity_step_start, "Missing velocity step section");
ASSERT(__foc_current_step_end > __foc_current_step_start, "Missing current step section");
ASSERT(__foc_svpwm_end > __foc_svpwm_start, "Missing SVPWM section");
ASSERT(__foc_blend_end > __foc_blend_start, "Missing Hall/observer blend section");
ASSERT(__foc_injection_end > __foc_injection_start, "Missing phase injection section");

/* The top 16 KiB of RAM is SRAM2. The control ISR runs about 155 cycles
 * slower with its driver there and 10 to 20 slower with only its small
 * statics there (measured; bus-matrix/DMA contention is the presumed
 * cause, not isolated). flip-link packs .data/.bss against the end of RAM,
 * so two cold task pools placed last fill that block. The SPI pump pool owns
 * the DMA slots: keeping it out of the ISR's SRAM1 is worth another 8 cycles
 * while telemetry streams. Embassy generates these pools, so they are
 * matched by name; the size ASSERT fails the link if they stop matching.
 * cortex-m-rt zeroes sections inserted after .bss (they precede __ebss). */
SECTIONS
{
  .bss_cold (NOLOAD) : ALIGN(4)
  {
    __bss_cold_start = .;
    *(.bss.*7servers16protocol_servers4POOL);
    *(.bss.*3spi4pump4POOL);
    . = ALIGN(4);
    __bss_cold_end = .;
  } > RAM
}
INSERT AFTER .bss;

/* flip-link links twice and moves RAM in between, so only the size can be
 * asserted here: the last 16 KiB of the packed RAM image are this section. */
ASSERT(__bss_cold_end - __bss_cold_start >= 0x4000, "The cold filler must be at least as large as SRAM2 (16 KiB)");
/* cortex-m-rt places .uninit after .bss, i.e. above the filler at the very
 * top of RAM. Anything there would take SRAM2 and push the filler down. */
ASSERT(__euninit == __suninit, ".uninit would sit above the SRAM2 filler; place it before .bss_cold");

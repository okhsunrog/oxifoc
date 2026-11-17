/* Measured F405 hot-path entry residues within 16-byte Flash lines. */
SECTIONS
{
  .foc_hot : ALIGN(16)
  {
    . = ALIGN(16) + 8;
    __foc_cycle_start = .;
    KEEP(*(.text.foc_cycle));
    __foc_cycle_end = .;
    . = ALIGN(16) + 4;
    __foc_phase_start = .;
    KEEP(*(.text.foc_phase));
    __foc_phase_end = .;
    . = ALIGN(16);
    __foc_grant_start = .;
    KEEP(*(.text.foc_grant));
    __foc_grant_end = .;
  } > FLASH
}
INSERT BEFORE .rodata;

ASSERT(__foc_cycle_end > __foc_cycle_start, "Missing FOC cycle section");
ASSERT(__foc_phase_end > __foc_phase_start, "Missing FOC phase section");
ASSERT(__foc_grant_end > __foc_grant_start, "Missing FOC grant section");

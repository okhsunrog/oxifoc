/* CPU-only data initialized by __pre_init. */
SECTIONS
{
    .ccm_data : ALIGN(4)
    {
        __sccmdata = .;
        *(.ccm.data .ccm.data.*);
        . = ALIGN(4);
        __eccmdata = .;
    } > CCM AT> FLASH
    __siccmdata = LOADADDR(.ccm_data);

    .ccm_bss (NOLOAD) : ALIGN(4)
    {
        __sccmbss = .;
        *(.ccm.bss .ccm.bss.*);
        . = ALIGN(4);
        __eccmbss = .;
    } > CCM

    /* Crash record (src/crash.rs): neither loaded nor zeroed, so it
     * survives a reset. Not a power cycle: CCM then holds noise, which the
     * record's magic and checksum reject. */
    .ccm_noinit (NOLOAD) : ALIGN(4)
    {
        __scrash = .;
        . += 256;
        __ecrash = .;
    } > CCM
}
/* cortex-m-rt assigns __edata AFTER .data, outside its output section.
 * Inserting there would move __edata into CCM and break ordinary RAM init. */
INSERT BEFORE .data;
ASSERT(__sccmdata % 4 == 0 && __eccmdata % 4 == 0 && __siccmdata % 4 == 0,
       "CCM data must be word aligned");
ASSERT(__sccmbss % 4 == 0 && __eccmbss % 4 == 0,
       "CCM BSS must be word aligned");
ASSERT(__scrash >= __eccmbss && __ecrash - __scrash == 256,
       "the crash record needs 256 bytes after CCM BSS, outside __pre_init");
ASSERT(__sdata >= ORIGIN(RAM) && __edata <= ORIGIN(RAM) + LENGTH(RAM)
       && __edata >= __sdata, "ordinary .data initialization must stay in SRAM");

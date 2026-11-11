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
}
/* cortex-m-rt assigns __edata AFTER .data, outside its output section.
 * Inserting there would move __edata into CCM and break ordinary RAM init. */
INSERT BEFORE .data;
ASSERT(__sccmdata % 4 == 0 && __eccmdata % 4 == 0 && __siccmdata % 4 == 0,
       "CCM data must be word aligned");
ASSERT(__sccmbss % 4 == 0 && __eccmbss % 4 == 0,
       "CCM BSS must be word aligned");
ASSERT(__sdata >= ORIGIN(RAM) && __edata <= ORIGIN(RAM) + LENGTH(RAM)
       && __edata >= __sdata, "ordinary .data initialization must stay in SRAM");

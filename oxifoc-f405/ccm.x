/* CPU-only initialized data; separate load image copied by __pre_init. */
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
}
/* cortex-m-rt assigns __edata AFTER .data, outside its output section.
 * Inserting there would move __edata into CCM and break ordinary RAM init. */
INSERT BEFORE .data;
ASSERT(__sccmdata % 4 == 0 && __eccmdata % 4 == 0 && __siccmdata % 4 == 0,
       "CCM data must be word aligned");
ASSERT(__sdata >= ORIGIN(RAM) && __edata <= ORIGIN(RAM) + LENGTH(RAM)
       && __edata >= __sdata, "ordinary .data initialization must stay in SRAM");

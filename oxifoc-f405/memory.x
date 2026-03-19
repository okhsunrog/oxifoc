/* STM32F405RG: 1MB Flash, 192KB RAM
 *
 * Flash: single bank
 *   0x08000000 - 0x080FFFFF (1024KB)
 *
 * RAM: 128KB SRAM + 64KB CCM
 *   SRAM:  0x20000000 - 0x2001FFFF (128KB)
 *   CCM:   0x10000000 - 0x1000FFFF (64KB) - not DMA-accessible
 */
MEMORY
{
    /* Program flash: 1024KB */
    FLASH : ORIGIN = 0x08000000, LENGTH = 1024K

    /* RAM: 128KB (main SRAM, DMA-accessible) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

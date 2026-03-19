/* STM32F405RG: 1MB Flash, 192KB RAM
 *
 * Flash layout (non-uniform sectors):
 *   Sectors 0-3:  16KB each  (0x00000 - 0x0FFFF)
 *   Sector 4:     64KB       (0x10000 - 0x1FFFF)
 *   Sectors 5-11: 128KB each (0x20000 - 0xFFFFF)
 *
 * Firmware: 0x00000 - 0xBFFFF (768KB, sectors 0-9)
 * Storage:  0xC0000 - 0xFFFFF (256KB, sectors 10-11, for sequential-storage)
 *
 * RAM: 128KB SRAM + 64KB CCM
 *   SRAM:  0x20000000 - 0x2001FFFF (128KB)
 *   CCM:   0x10000000 - 0x1000FFFF (64KB) - not DMA-accessible
 */
MEMORY
{
    /* Program flash: 768KB (sectors 0-9) */
    FLASH : ORIGIN = 0x08000000, LENGTH = 768K

    /* RAM: 128KB (main SRAM, DMA-accessible) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

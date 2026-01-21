/* STM32G474RET6: 512KB Flash (dual-bank), 128KB RAM
 * 
 * Dual-bank flash layout:
 *   Bank 1: 0x08000000 - 0x0803FFFF (256KB) - Firmware
 *   Bank 2: 0x08040000 - 0x0807FFFF (256KB) - Storage (async flash)
 *
 * Using bank 2 for storage allows non-blocking flash operations
 * while code runs from bank 1.
 */
MEMORY
{
    /* Program flash: Bank 1 only (256KB) */
    FLASH : ORIGIN = 0x08000000, LENGTH = 256K

    /* Config storage in Bank 2: handled by sequential_storage
     * Using last 4KB of bank 2: 0x0807F000 - 0x0807FFFF
     * This is offset 0x3F000 within bank 2 (or 0x7F000 from flash base) */

    /* RAM: 128KB (96KB SRAM1 + 32KB SRAM2) */
    RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

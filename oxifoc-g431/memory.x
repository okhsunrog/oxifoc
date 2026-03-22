/* STM32G431CB: 128KB Flash, 32KB RAM
 * Reserve last 4KB of flash for sequential_storage persistent storage
 * (sequential-storage requires at least 2 erase pages; erase size = 2KB)
 */
MEMORY
{
    /* Program flash: 124KB */
    FLASH : ORIGIN = 0x08000000, LENGTH = 124K

    /* Config storage: 4KB (2 pages × 2KB) - handled by sequential_storage */
    /* STORAGE : ORIGIN = 0x0801F000, LENGTH = 4K */

    /* RAM: 32KB */
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

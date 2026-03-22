/* STM32G431CB: 128KB Flash, 32KB RAM
 * Reserve last 2KB of flash for sequential_storage persistent storage
 */
MEMORY
{
    /* Program flash: 126KB */
    FLASH : ORIGIN = 0x08000000, LENGTH = 126K

    /* Config storage: 2KB (1 page × 2KB) - handled by sequential_storage */
    /* STORAGE : ORIGIN = 0x0801F800, LENGTH = 2K */

    /* RAM: 32KB */
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

/* STM32G431CB: 128KB Flash, 32KB RAM
 * Baked-config profile (`storage` feature off): no flash region is reserved
 * for sequential_storage — the full 128KB belongs to the program.
 */
MEMORY
{
    /* Program flash: full 128KB */
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K

    /* RAM: 32KB */
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

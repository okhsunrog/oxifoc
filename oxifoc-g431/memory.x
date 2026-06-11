/* STM32G431CB: 128KB Flash, 32KB RAM
 * No flash-backed config storage on this board (removed 2026-06-12,
 * docs/flash-size.md): the full 128KB belongs to the program; configuration
 * is baked at build time (src/baked_config.rs).
 */
MEMORY
{
    /* Program flash: full 128KB */
    FLASH : ORIGIN = 0x08000000, LENGTH = 128K

    /* RAM: 32KB */
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

MEMORY
{
  /* Adafruit nRF52 bootloader without S140, as in Voyager. The bootloader
     starts at 0xF4000. The two pages below it, 0xF2000..0xF4000, hold the
     settings map (src/storage.rs), so the firmware ends at 0xF2000. */
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xF1000
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

MEMORY
{
  /* Adafruit nRF52 bootloader without S140, as in Voyager. */
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xF3000
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

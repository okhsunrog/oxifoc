# Seeed XIAO ESP32-C5 as the bridge

The bridge firmware runs on two boards. The Waveshare ESP32-C5-Zero was the
first; the XIAO ESP32-C5 adds 8 MB of PSRAM, where the bridge
[records fast telemetry](../../oxifoc-bridge/README.md#recording-on-the-bridge).
Select it at build time:

```sh
OXIFOC_BRIDGE_BOARD=xiao-c5 just flash-bridge
```

## The board

- ESP32-C5HR8, revision v1.0, 48 MHz crystal: 8 MB of quad PSRAM in the
  package (chip id `9207135a5d0d`), no flash in it.
- 8 MB flash beside the chip (JEDEC manufacturer `85`, device `2017`).
- An antenna connector only (U.FL); the board comes with the FPC antenna
  A01, 2.8 dBi. Connect it before the radios are used.
- GPIO6 is the battery voltage input and is not brought out, GPIO26 enables
  that measurement, GPIO27 is the user LED, GPIO28 the BOOT button.
- The 5V pin is the USB bus voltage, in and out. Feeding it from the
  controller while USB is plugged in joins the two supplies.

## Wiring to the MK5

The firmware keeps the C5-Zero's GPIO numbers where the XIAO brings them
out. The SPI clock moves, because GPIO6 is not available, and the wireless
probe's pins move to the edge.

| MK5 | STM32 | XIAO pin | GPIO | C5-Zero GPIO |
|---|---|---|---|---|
| COMM pin 6, TX/SCL (MOSI) | PA7 | D3 | 7 | 7 |
| COMM pin 4, ADC1 (SCK) | PA5 | D8 | 8 | 6 |
| COMM pin 7, ADC2 (MISO) | PA6 | D9 | 9 | 9 |
| COMM pin 5, RX/SDA (CS) | PB11 | D10 | 10 | 10 |
| SERVO signal (READY) | PB6 | D1 | 0 | 0 |
| SWCLK pad | – | D4 | 23 | 4 |
| SWDIO pad | – | D5 | 24 | 5 |
| GND | | GND | | |

D6 and D7 (GPIO11, GPIO12) are kept for CAN FD. D0 and D2 are free.
See the [MK5 notes](vesc6-mk5.md#c5-bridge-wiring-and-continuity-check) for
the connector and for the series resistors on the probe's lines.

## Measured before wiring, 2026-11-24

With the board on USB alone and esp-hal `main` (`ba36471`).

Flash and PSRAM, by a program that read the flash controller's registers,
wrote and verified the PSRAM and timed copies into and out of it:

| | Mode | Result |
|---|---|---|
| Flash | quad I/O, 80 MHz (`SPI_MEM_CTRL` 0x012c200c and `SPI_MEM_CLOCK` 0x80000000 on both SPI0 and SPI1) | the bootloader enabled quad mode itself; the factory bootloader ran dual I/O at 80 MHz |
| PSRAM | quad, 80 MHz (read command `eb`, write `38`) | 248 passes of write and verify over all 8 MB, no bad word; 13.4 MiB/s written, 24 read |
| PSRAM | quad, 40 MHz | 7.3 MiB/s written, 13.5 read |

Radio, with the bridge firmware, the board on a desk beside the access
point, and 1400-byte datagrams from the board through the access point to
a wired host:

| | 5 GHz, channel 64 | 2.4 GHz, channel 11 |
|---|---|---|
| The board at the access point | −21 to −25 dBm | −21 to −24 dBm |
| The access point at the board | −24 to −28 dBm | −24 to −25 dBm |
| Rate up, down | 72, 72 Mbit/s | 72, 52 Mbit/s |
| Datagrams lost | 0 of 6423 | 0 of 6965 |
| Most the board sent, BLE advertising | 7.2 to 7.4 Mbit/s | 7.7 to 7.8 Mbit/s |

Without BLE in the image the board sent 12.0 Mbit/s at 5 GHz: advertising
costs Wi-Fi about 40 % on esp-hal `main`, as
[on the C5-Zero](../notes/ble-link-reliability.md#wi-fi-beside-ble-on-esp-hal-main).
A relayed 20 kHz stream needs 1.7 Mbit/s. A laptop received the board's
advertising at −27 to −35 dBm. At these levels the figures show that the
radio and the antenna work, not how far they reach.

The recorder was tried on the PSRAM with made-up blocks, no controller
being wired: see the [bridge README](../../oxifoc-bridge/README.md#recording-on-the-bridge).

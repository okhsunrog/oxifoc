# The bridge board: Seeed XIAO ESP32-C5

The bridge firmware runs on a Seeed XIAO ESP32-C5. Its 8 MB of PSRAM are
where the bridge
[records fast telemetry](../../oxifoc-bridge/README.md#recording-on-the-bridge).
The firmware first ran on a Waveshare ESP32-C5-Zero, and the earlier bench
notes were measured on that board.

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

| MK5 | STM32 | XIAO pin | GPIO |
|---|---|---|---|
| COMM, 5 V | | VBUS | |
| COMM, GND | | GND | |
| COMM pin 4, ADC1 (SCK) | PA5 | D10 | 10 |
| COMM pin 5, RX/SDA (CS) | PB11 | D9 | 9 |
| COMM pin 6, TX/SCL (MOSI) | PA7 | D8 | 8 |
| COMM pin 7, ADC2 (MISO) | PA6 | D7 | 12 |
| SERVO signal (READY) | PB6 | D1 | 0 |
| SWCLK pad | – | MTCK pad on the back | 4 |
| SWDIO pad | – | MTDO pad on the back | 5 |

The board is wired as the table says. The chip routes these signals to any
pin, and `src/bin/main.rs` of the bridge names the assignment in one place.
D0, D4, D5 and D6 are free; two of them are meant for a CAN FD
transceiver. Leave the strapping pins
unconnected to the controller: D2 (GPIO25) and D3 (GPIO7) on the header,
GPIO2 and GPIO3 among the pads on the back. The chip reads them at reset,
and a level from outside can change how it starts.

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

## Wired to the MK5, 2026-11-24

With the wiring above, the firmware on esp-hal `main`, and the bench motor
driven at +120, +450 and +120 rad/s for 19 s by the capturing host:

| 20 kHz capture | Result | Replies to commands |
|---|---|---|
| USB | complete, 372 185 records | after 7 to 11 ms |
| Wi-Fi | complete, 375 647 records | after 21 to 68 ms |

The SPI link carried 61 531 slots without an error. The wireless probe
read the controller's CPUID and reset it over the new pads.

Beside a BLE central that drove the motor for 78 s, a laptop paired with
this bridge, 60 s captures at 20 kHz were complete over Wi-Fi (1 194 810
records) and over USB (1 193 992). The central's status replies were more
than 150 ms apart 7 and 11 times, 166 ms at the longest. The heap had
7 352 bytes of 94 208 free at its lowest in these runs.

The relay took 0.87 ms an envelope over USB, against 0.47 ms on the first
board with the September releases. This board takes 0.47 ms as well with
that firmware, so the firmware differs, not the board. Timed apart, the
rewrite takes 0.23 to 0.24 ms in both, and sending the rewritten block
0.20 ms there and 0.54 ms here. The clock and cache registers are the same
in both; the flash controller's differ in the PSRAM's settings and in the
timing its driver sets.

The code that sends is the same ergot. Its COBS sink encodes a frame a
byte at a time through postcard's `Cobs` flavor, and what a byte costs is
the compiler's choice. With the releases the loop is inline in the block's
frame encoder, 382 bytes of code. It is still there with esp-hal `main`
alone and in the builds for a board without PSRAM. From the commit at
which the bridge answers `cmd/raw_record` with chunks of a recording, a
second type whose bytes go through that flavor, the encoder is 184 bytes
and every byte is a call to `cobs::enc::EncoderState::push`, with a frame
pointer and a stack check each time. `opt-level = 3` sends in 0.24 ms and
rewrites in 0.27, with 250 KiB more code. Encoding in runs inside ergot
would take the choice from the compiler
([wishlist](../notes/ergot-wishlist.md), item 15).

Recording is described in the
[bridge README](../../oxifoc-bridge/README.md#recording-on-the-bridge).

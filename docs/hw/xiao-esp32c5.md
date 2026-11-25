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

A CAN FD transceiver (TJA1051T/3, 5 V supply, 3.3 V on VIO) is on the
header: RXD on D5 (GPIO24), TXD on D4 (GPIO23), for the
[bus between bridges](../../oxifoc-bridge/README.md#can-fd-between-bridges).
TXD must not be on D6: GPIO11 is UART0 TX out of reset,
and what the bootloaders print there reaches the bus
([measured](#can-fd-between-two-boards-2026-11-24)). D0 and D6 are free.

Leave the strapping pins
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
rewrites in 0.27, with 250 KiB more code.

ergot #248 took the choice from the compiler: its COBS sink copies each
run of non-zero bytes at once. Timed over Wi-Fi with the motor stopped,
where a block is 391 bytes, the send went from 0.47 ms to 0.26 ms and a
whole block from 0.79 ms to 0.60 ms; the rewrite is 0.33 ms in both. The
motor was not driven for this, so the 0.87 ms above has no counterpart
yet.

Recording is described in the
[bridge README](../../oxifoc-bridge/README.md#recording-on-the-bridge).

## CAN FD between two boards, 2026-11-24

A second XIAO with a transceiver wired the same way, the two transceivers
on one bus, and on both boards a test built on the `canfd` driver of
esp-hal `main`. TXD was on D6 (GPIO11) for the runs in the table. Each board sent numbered 64-byte frames with the bit rate
switched and checked the numbers and the payloads of the other's; a
quarter of the payloads were all zeros and a quarter all ones, for the
most stuff bits. The controller's clock is 80 MHz.

| Arbitration | Data | Frames a second, each way | Frames each way | Missing or damaged |
|---|---|---|---|---|
| 500 kbit/s | 2 Mbit/s | 500 | 2 250 in 5 s | none |
| 1 Mbit/s | 5 Mbit/s | 500 | 10 750 in 22 s | none |
| 1 Mbit/s | 8 Mbit/s | 500 | 10 250 in 21 s | none |
| 1 Mbit/s | 5 Mbit/s | 2 500 | 51 200 in 21 s | none |

Once both boards ran, no transmission failed and both error counters
stayed at zero in every run. The data phases at 5 and 8 Mbit/s are 16 and
10 clock periods a bit, sampled at 75 and 70 %, with the secondary sample
point 12 and 7 periods after the measured transceiver delay. The
transceiver is specified up to 5 Mbit/s.

With TXD on GPIO11 a reset of either board disturbed the bus. The ROM
and the second-stage bootloader print their log on that pin, and the
transceiver sends it as dominant pulses. A board that transmitted alone, error-passive with its
transmit counter at 128 and its receive counter at 0, went bus-off once
when the other board was reset, and its receive counter read 511 from
then on. In the runs above the board that started first went bus-off up
to 5 times while the other started.

With TXD moved to D4 (GPIO23) on both boards the same reset left the
transmitting board's counters at 128 and 0, with no bus-off. At 1 and
5 Mbit/s the boards then exchanged 6 750 frames each way in 14 s at 500 a
second and 41 200 in 17 s at 2 500 a second: none missing or damaged, no
transmission failed from the start, and no bus-off.

## ergot over the bus, 2026-11-24

The two boards as bridges: the wired one the root, the other with the bus
as its upstream and no radio, no controller on its SPI port.

- The second bridge claimed node 19 of the bus's net and was leased nets
  for its SPI and USB segments. It kept the address through resets, and
  held the claim and the leases for 11.7 hours.
- A host on the root's USB port read the second bridge's diagnostics and
  recording state, and a host on the second bridge's USB port read the
  root's diagnostics and the controller's status, each through both
  bridges and the bus.
- The second bridge's firmware was updated over the bus from a host on the
  root: 453 760 bytes in 12 s, the restart included. The root sent 8 371
  CAN frames and received 1 253 for it, with no failed transmission, no
  overrun and no bus-off.
- After a restart of the root the second bridge claimed and leased again by
  itself. After its own restart it has new nets, and the old ones stay
  leased until they run out.
- The root's main stack is 20 328 bytes with the bus, 23 080 without: the
  link's queues and assembly buffers are in RTC RAM beside the TCP port's.
- A 20 kHz capture on the root over USB stayed complete.

On 2026-11-25 both bridges moved to the transport as reworked after its
review (message numbers per destination, what was assembling dropped when
frames were lost, no stale frames kept in the queues). The second bridge
claimed the same address, and an update of it over the bus took 457 488
bytes: the root sent 8 443 CAN frames and received 1 261, with no failed
transmission, no overrun and no bus-off. A host on the second bridge read
the controller's status through both.

## A controller left running by other firmware, 2026-11-25

The root was given firmware without the CAN link over the probe, while
the second board ran the CAN FD tests of esp-hal at other bit rates. The
root's controller stayed on the bus at 1 Mbit/s:

- The probe's reset does not reset the peripherals. GPIO23 was still the
  controller's output (`GPIO_FUNC23_OUT_SEL_CFG` read 66, TWAI0 TX).
- esp-hal's start-up turned off the register clock of the unused TWAI0
  (`PCR_TWAI0_CONF` read 4) and left its function clock on
  (`PCR_TWAI0_FUNC_CLK_CONF` read 0x00500000).

The second board, alone in transmitting, then lost the same classic
frames every time: at 500 kbit/s eight bytes of `00`, `07` or `0f` but
not of `f0`, `55` or `ff`, at 250 kbit/s `f0` and `ff`, and none at
1 Mbit/s or 125 kbit/s. The errors were a bit error in the CRC field or
a form error at the acknowledgement, after three retransmissions.
Nothing acknowledged its frames and it received none, so what disturbed
them was presumably the error and overload flags of the root's
controller. With that controller held in reset through the probe
(`PCR_TWAI0_CONF` written 2) every frame passed.

Firmware with the link starts the controller anew. A board on the bus
that gets firmware without it keeps the old controller until its
peripherals are reset; a power cycle was not tried.

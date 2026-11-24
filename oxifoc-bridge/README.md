# OxiFOC C5 bridge

`just build-bridge` and `just flash-bridge` build with `ble,swd-probe,ota`.
Use the optional feature-list argument for a different build. The bridge
routes ergot between the controller's SPI link, USB Serial/JTAG, and two BLE
NUS links when `ble` is enabled.

`OXIFOC_BRIDGE_BOARD` selects the board: `c5-zero` for the Waveshare
ESP32-C5-Zero, the default, or `xiao-c5` for the
[Seeed XIAO ESP32-C5](../docs/hw/xiao-esp32c5.md), which has the PSRAM for
recordings. Each board builds into its own directory under `target/`. An
image is for the board it was built for: the SPI clock, the probe's pins
and the antenna switch differ, and nothing yet refuses the wrong one.

The `wifi` feature adds WPA2-Personal Wi-Fi station mode and Ergot TCP.
The `swd-probe` feature enables Wi-Fi and adds an armed-only CMSIS-DAP TCP
listener. Set `OXIFOC_PROBE_WIFI_SSID` and
`OXIFOC_PROBE_WIFI_PASSWORD` in the environment or the git-ignored root
`.env` before building. The build without credentials leaves Wi-Fi off
and keeps the other links available. Keep credentialed firmware binaries
private because they contain the password.

The GPIO adapter uses the bit-bang driver now included in
[dap-rs](https://github.com/probe-rs/dap-rs/tree/a867ca7b41547b24aaad52b913528575685a5df5)
0.2.0 (MIT or Apache-2.0), the successor to espdap's standalone
`bitbang-dap`. The Espressif crates come from esp-hal `main` at a pinned
revision (`Cargo.toml`), which has the C5's CAN FD driver and PSRAM and BLE
fixes that no release carries.

See [Wireless SWD](../docs/hw/vesc6-mk5.md#wireless-swd) for wiring and use.

`just flash-bridge` writes the bootloader for quad I/O flash at 80 MHz. The
bridge runs its code from flash through a 32 KiB cache, and with the
defaults (dual I/O, 40 MHz) reloading that cache took most of its time:
see [the measurements](../docs/notes/ble-link-reliability.md#the-relay-and-the-flash-speed).
A firmware update over Ergot replaces the application only, so a bridge
installed before this change needs one wired `just flash-bridge`.

## Fast telemetry relay

A host asks the bridge, not the controller, for a fast telemetry session
(`cmd/raw_relay`). The bridge opens the session at the controller from its
own address, takes the Packed28 blocks, and sends them to the host as
Delta28: 10.4 bytes a record instead of 28.5 on the bench motor at
+120 rad/s, 5.7 at standstill. The host expands them to the identical
Packed28 bytes.

The session belongs to the host that asked. Another host is answered
`Busy` while the controller runs it. When the owner's link goes down the
bridge stops the session, so the next host can ask: after a host was
killed or frozen in the middle of a capture, another captured on its first
try.

`diag bridge` reports the relay as `relay: envelopes=… bytes_in=…
bytes_out=… missed=… dropped=… busy_us=…`: what it sent on, the bytes
before and after, blocks that never reached it, envelopes the host's link
refused, and the time it spent. On the bench the rewrite and the send took
about 0.47 ms an envelope, 15 % of the CPU at 20 kHz.

## Recording on the bridge

On a board with PSRAM the bridge can keep a session itself, for a host to
fetch later: start a recording from a phone before a ride, fetch it at
home over Wi-Fi or USB.

```sh
oxifoc-host-cli recording start --ring     # or without --ring: until full
oxifoc-host-cli recording status
oxifoc-host-cli recording stop
oxifoc-host-cli recording fetch --out ride.parquet
oxifoc-host-cli recording clear
```

A recording is the session's blocks as the relay would send them. Without
`--ring` it stops when the memory is full and holds the beginning. With
`--ring` it drops its oldest blocks and holds the time before it stopped:
by `stop`, or one second after a fault that stops the motor. The XIAO's
8 MB hold about 40 s of a 20 kHz session with the bench motor running.

Any host may send these requests over any link, and the recording stays
until it is cleared or the bridge restarts. While one runs, a host that
asks for a live session is answered `Busy`. `fetch --out` writes the same
parquet file as `record` and needs the controller connected, for its
calibration; `fetch --raw` saves the bytes as stored and needs only the
bridge.

Tried on the XIAO with made-up blocks, no controller being wired yet: the
memory filled to 8 388 259 of 8 388 608 bytes in 15 917 blocks, a fetch
took 22 s over USB and 62 s over Wi-Fi with identical bytes, and a ring
given 30 000 blocks kept the newest 15 917 in order. A session from the
controller, and the stop on a fault, have not been tried.

## Ergot over Wi-Fi

With Wi-Fi configured, the C5 listens on TCP port 2025 and routes the
connected host's Ergot traffic to the controller. Frames are COBS-encoded
on the stream, as on USB Serial/JTAG, and both ends use Ergot's stream
transport. The bridge serves one connection at a time; a new one is
accepted once the previous host closes or stops answering for five
seconds. Any host that reaches the port can connect, and the traffic is
plaintext motor control. Use a trusted LAN.

```sh
target/debug/oxifoc-host-cli --transport tcp --tcp-host 192.0.2.10 status
```

TCP repeats lost segments, so a native-rate capture needs no recovery of
its own. The bridge queues up to 36 KiB for the socket, about 170 ms of a
relayed 20 kHz stream, and keeps the TCP window at 16 KiB: its TCP stack
resends the whole window after a lost segment. It retransmits after 50 ms
without an acknowledgement (`SMOLTCP_TCP_MIN_RTO_MS` in
`.cargo/config.toml`; see `../vendor/smoltcp/PATCHES.md`). While the queue
of the link that takes a session is full, USB or Wi-Fi, the bridge holds
back the SPI link for up to 50 ms per slot, so the backlog collects in the
controller's queue first; a longer stall drops frames at the bridge and
fails the capture. `diag bridge` reports those drops, the fullest TCP
window and the access point's signal at the bridge as
`wifi: dropped=… max_backlog=… signal=… dBm`. A 20 kHz capture is complete
over Wi-Fi beside a BLE central that drives the motor, and over USB.

## Bridge logs

The CLI prints C5 text logs to stderr under the `remote_log` target.
If the controller is halted, use `--wait-secs 0` to skip waiting for its
handshake.

```sh
RUST_LOG=remote_log=info target/debug/oxifoc-host-cli --transport tcp --tcp-host 192.0.2.10 \
  --wait-secs 0 monitor --seconds 60 --fast-hz 0
```

The same command works over BLE. Ergot delivers live messages once the
link is up. Internal Ergot and driver diagnostics go to wired RTT. RTT
also carries early boot logs and panic output.

A panic or CPU exception resets the bridge. `diag bridge` shows where it
happened and its message after the reset, over any link; the record stays
until `diag bridge --clear-crash` or a power cycle.

## Bridge firmware updates

Any host that reaches the bridge over USB, a bonded BLE link or the Wi-Fi
port can update it and arm wireless SWD; nothing beyond the link itself
authorizes these requests. The same links already carry motor commands.

Install the dual-slot partition table once with `just flash-bridge` over
wired USB. Each wired install selects `ota_0` and preserves the BLE
bond partition. Image packaging uses `espflash`.
Build an app binary with the credentials and options you want the
bridge to boot with.

```sh
just build-cli
just build-bridge-ota bridge-app.bin
target/debug/oxifoc-host-cli --transport tcp --tcp-host 192.0.2.10 ota update bridge-app.bin
target/debug/oxifoc-host-cli --transport ble --ble-name "Oxifoc Bridge" ota update bridge-app.bin
```

Use one of the two update commands while the motor is stopped and SWD is
disarmed. The bridge checks each chunk against flash and verifies the
image digest before selecting the inactive slot. The CLI waits for the
bridge to reappear on the new slot. Repeating an interrupted upload can
resume within 60 seconds of its last chunk. Use `ota status` to read the
booted slot and `ota abort` to discard an active upload.

The stock ESP-IDF bootloader checks the image at boot. For a firmware
startup failure, use the wired `just flash-bridge` recovery recipe.

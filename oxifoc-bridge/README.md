# OxiFOC C5 bridge

`just build-bridge` and `just flash-bridge` build with `ble,swd-probe,ota`.
Use the optional feature-list argument for a different build. The bridge
routes ergot between the controller's SPI link, USB Serial/JTAG, and two BLE
NUS links when `ble` is enabled.

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
`bitbang-dap`. The station uses the bridge's `esp-radio` 1.0.0-beta.1 and
`embassy-net` 0.9.1.

See [Wireless SWD](../docs/hw/vesc6-mk5.md#wireless-swd) for wiring and use.

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
its own. The bridge keeps unacknowledged telemetry in a 56 KiB socket
buffer, about 90 ms at 20 kHz, and its TCP stack retransmits after 50 ms
without an acknowledgement (`SMOLTCP_TCP_MIN_RTO_MS` in
`.cargo/config.toml`; see `../vendor/smoltcp/PATCHES.md`). A stall that
outlasts the buffer drops frames at the bridge and fails the capture.
`diag bridge` reports those drops and the largest backlog as `wifi:
dropped=… max_backlog=…`. USB and BLE remain available.

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

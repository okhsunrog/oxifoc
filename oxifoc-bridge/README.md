# OxiFOC C5 bridge

`just build-bridge` and `just flash-bridge` build with `ble,swd-probe,ota`.
Use the optional feature-list argument for a different build. The bridge
routes ergot between the controller's SPI link, USB Serial/JTAG, and two BLE
NUS links when `ble` is enabled.

The `wifi` feature adds WPA2-Personal Wi-Fi station mode and Ergot UDP.
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

Set `OXIFOC_BRIDGE_ERGOT_PEER` to the host's IPv4 address as seen by the
bridge. With Wi-Fi configured, the C5 accepts that host on UDP port 2025
and routes its Ergot traffic to the controller. The host can reconnect
from a new source port once the previous link expires. Use a trusted LAN
for this port, which carries motor-control commands. The IPv4 filter
limits the configured peer. Ergot UDP itself carries plaintext.

```sh
target/debug/oxifoc-host-cli --transport udp --udp-host 192.0.2.10 status
```

UDP telemetry uses blocks sized for a standard 1500-byte IPv4 path MTU.
USB and BLE remain available.

## Bridge firmware updates

Generate a 32-byte OTA key with `openssl rand -hex 32` and keep it in the
git-ignored `.env` as `OXIFOC_BRIDGE_OTA_KEY`. Source that file for the host
CLI so it uses the key embedded in the bridge build. Keep credentialed app
binaries private. The CLI authenticates each upload operation with HMAC.

Install the dual-slot partition table once with `just flash-bridge` over
wired USB. Each wired install selects `ota_0` and preserves the BLE
bond partition. Image packaging uses `espflash`.
Build an app binary with the credentials and options you want the
bridge to boot with.

```sh
just build-cli
just build-bridge-ota bridge-app.bin
set -a
. ./.env
set +a
target/debug/oxifoc-host-cli --transport udp --udp-host 192.0.2.10 ota update bridge-app.bin
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

# OxiFOC C5 bridge

`just build-bridge` and `just flash-bridge` build with `ble,swd-probe`.
Use the optional feature-list argument for a different build. The bridge
routes ergot between the controller's SPI link, USB Serial/JTAG, and two BLE
NUS links when `ble` is enabled.

The `swd-probe` feature adds WPA2-Personal Wi-Fi station mode and an armed-only CMSIS-DAP
TCP listener. Set `OXIFOC_PROBE_WIFI_SSID` and
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

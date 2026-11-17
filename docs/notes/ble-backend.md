# Shared BLE backend

All host frontends use `blec` through `oxifoc-host-lib`. The backend is pinned
from the `oxifoc-integration` branch of
https://github.com/okhsunrog/tauri-plugin-blec. There is no `bluest` dependency.
The fork follows upstream 0.20.0, adapts `dioxus-blec` to the same Dioxus 0.8
commit as the GUIs, and fixes scan cancellation and initial notification races.
Foreign system connections are left alone; late completion of our own cancelled
attempt is cleaned up. The pinned fork revision is
`6c5f1b5b5e55ff79312e9564efb9a560cdc788ba`.

`HostConfig::ble_address` is a string (MAC on Linux/Android, peripheral UUID on
macOS). `BleDeviceInfo` contains presentation data and that address, never a
platform handle. Discovery filters by the OxiFOC service UUID
(`oxifoc_core::ble::SERVICE_UUID`), which the bridge advertises; the name is
in the scan response. The service mirrors the Nordic UART layout under its own
UUIDs, so generic NUS tools do not list the board. CLI selection
still rejects ambiguous results; `--ble-address` bypasses discovery.

The global BLE adapter and its event pump are initialized on a persistent
runtime, independent of temporary CLI/Slint scan runtimes. Each host connection
owns RX/TX workers, subscribes before marking the interface active, and joins
its workers, explicitly unsubscribes, and disconnects on shutdown/reconnect.
Unsubscribing matters on Android: another GATT client can keep the physical
radio link alive after our client closes. Pending connection attempts are also
cancelled. Android permissions remain
an OS decision and errors are exposed to the picker.

One ergot frame maps to one GATT write/notification. The bridge's 505-byte frame
limit requires an ATT MTU of at least 508. Smaller MTUs are rejected with an
explicit error; splitting packets would change the existing wire protocol.
Notifications enter a bounded 128-frame queue. Overflow, oversize packets,
write errors, disconnects and RX inactivity fail the session and enter the normal reconnect
policy. They are never routed through a latest-value UI signal.
After the first incoming packet, RX uses the same `LIVENESS_TIMEOUT_MS` as
TCP/UDP (currently one second). This detects a vanished GATT service even if
the platform still reports a connected radio link. Initial handshake silence
is bounded by the common connection loop.

## Frontends

- Slint's existing picker uses shared discovery and reports scan errors.
- CLI: `--transport ble --ble-address AA:BB:CC:DD:EE:FF` (or `--ble-name`).
- Dioxus Native and Desktop: set `OXIFOC_BLE_ADDRESS=AA:BB:CC:DD:EE:FF`.
  Without it they keep their TCP virtual-controller defaults.
- `experiments/ble-mobile` is a small Android Dioxus diagnostic frontend using
  this exact host library, with a scanner, connection state and real telemetry.

Android requires API 26+. Dioxus packages the fork's Kotlin library as a normal
Gradle module via `dioxus-blec`/Manganis. Enable its `android-webview` feature
and call `prepare_android()` from a mounted component/event before initializing
the shared backend: Wry supplies the Activity needed for runtime permissions.
`use_ble` does this automatically for applications using the wrapper's hooks.
Slint's cargo-apk path enables the
backend's `embedded-dex` feature and declares permissions in its manifest;
loading embedded DEX can be restricted by hardened Android policies. It is a
packaging alternative, not another BLE implementation. Neither frontend
requires Tauri.

## Reproduction

From the repository shell:

```sh
nix develop -c cargo run -p oxifoc-host-lib --no-default-features --example ble_scan
nix develop -c cargo test -p oxifoc-host-lib --lib
nix develop -c cargo test -p oxifoc-host-lib --no-default-features --features ble-mock-tests transport::ble
```

`ble-mock-tests` is exclusively for automated transport regression tests. It
must not be enabled in a normal GUI or hardware build. Runtime-drop, MTU,
subscription failure, repeated reconnect, bad packets and shutdown tests use
it. The live Android lab uses the real platform backend.

For a motor-free radio test, run `oxifoc-virtual --port 25267` and
`experiments/ble-virtual-bridge` on Linux. The latter advertises **OxiFOC Virtual**
and forwards complete GATT frames to the loopback COBS TCP connection. It opens
a fresh TCP connection for each subscription and closes it when notifications
stop. It does not change the adapter's power state or pairing configuration.
Only one central is supported. Stop the fixture after the test.

The mobile APK must be built with `dx` from the pinned Dioxus commit, not the
installed 0.7 CLI. See `experiments/ble-mobile/README.md` for build and device
validation results. This diagnostic does not establish Slint APK acceptance,
maximum telemetry rate, background operation or physical motor safety.

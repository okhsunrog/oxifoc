# Android NUS peripheral fixture

Diagnostic app, separate from both product frontends. It advertises a Nordic
UART service and bridges each raw BLE packet to one COBS TCP frame over
loopback. Telemetry and protocol responses come from `oxifoc-virtual` on the
laptop, via `adb reverse`; this app does not synthesize controller data.

Use it to test Linux BLE centrals with the laptop's single Bluetooth adapter.
The Slint and Dioxus product apps remain BLE centrals and never advertise.
This fixture has a deliberately plain diagnostic screen, not the product UI.

From the repository root, with Android SDK, NDK, Java and cargo-rapk installed:

```sh
mkdir -p experiments/ble-android-peripheral/target/aarch64-linux-android/debug/build
nix develop -c cargo rapk build --manifest-path experiments/ble-android-peripheral/Cargo.toml --lib
adb install --no-incremental -r experiments/ble-android-peripheral/target/debug/apk/oxifoc-ble-fixture.apk
adb reverse tcp:25267 tcp:25267
nix develop -c cargo run -p oxifoc-virtual -- --transport tcp --port 25267
```

Open **OxiFOC BLE test peripheral**, tap Start and grant Nearby devices.
Only one central is supported. Connect with Linux Slint or Dioxus. The TCP
session is recreated on every notification subscription. Use a dedicated
virtual controller; do not route this fixture to motor hardware.

At the end, tap Stop, stop the virtual controller, and remove the fixture:

```sh
adb shell am force-stop com.oxifoc.blefixture
adb reverse --remove tcp:25267
adb uninstall com.oxifoc.blefixture
```

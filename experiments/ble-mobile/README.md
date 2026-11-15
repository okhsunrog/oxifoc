# OxiFOC BLE Lab

Minimal Android Dioxus frontend for the shared `oxifoc-host-lib` BLE transport.
It scans for NUS devices and displays connection state, sample count, bus
voltage, motor state and faults. Data comes from the connected controller; no
mock feature is enabled. It does not send motor-control commands.

Requires Android API 26+, an Android SDK/NDK and `dx` built from Dioxus commit
`97adf7915e1adfef4e2da436c9bd0b6add0dfa39`. The `dioxus-blec` dependency packages
the Kotlin backend and permissions as a Gradle module. No embedded DEX loading
or Tauri host is needed here.

Enable `dioxus-blec/android-webview` and call `prepare_android()` after the
WebView is mounted, before initializing the shared host backend. This supplies
the current Activity for runtime permission requests. An Application context
from `ndk-context` alone cannot display that prompt.

## Build and run

Install a matching CLI separately from any existing 0.7 CLI. From the repository
root, choose a persistent tools directory for `DX_ROOT`:

```sh
export DX_ROOT="$HOME/tmp_zfs/codex_tmp/oxifoc-blec/dx"
nix develop -c cargo install dioxus-cli \
  --git https://github.com/DioxusLabs/dioxus \
  --rev 97adf7915e1adfef4e2da436c9bd0b6add0dfa39 --locked --root "$DX_ROOT"
nix develop -c bash -c 'cd experiments/ble-mobile; "$DX_ROOT/bin/dx" build --android --target aarch64-linux-android'
adb install -r experiments/ble-mobile/target/dx/oxifoc-ble-mobile/debug/android/app/app/build/outputs/apk/debug/app-debug.apk
adb shell am start -W -n com.oxifoc.blelab/dev.dioxus.main.MainActivity
```

Allow Nearby Devices when scanning. To test without a motor, start the
Linux [BLE virtual bridge](../ble-virtual-bridge/README.md), scan and select
**OxiFOC Virtual**. The path is Android GATT → Linux NUS peripheral → loopback
TCP → the normal `oxifoc-virtual` backend. The BLE client and protocol are real;
only the controller physics is virtual.

## Device validation

On 2026-11-15, the debug APK was built, installed and exercised on a Pixel 8 Pro
running Android API 37. Runtime permission request and NUS discovery worked.
The negotiated ATT MTU was 517; the UI received the virtual controller's
identity, `Stopped` state, 24.0 V bus voltage and fast telemetry configured at
100 Hz. No motor-control command was sent.

The initial stream delivered 3,145 fast samples in about 31 seconds, consistent
with the requested 100 Hz; individual UI polls receive batches. This is a
functional radio test, not a maximum-throughput or rendering benchmark.
Explicit Disconnect followed by reconnect resumed fast samples after adding
CCCD unsubscribe to shared session cleanup. Android reported that another
GATT client retained the physical link; the bridge nevertheless observed the
old notification session end and created a fresh TCP session on reconnect.
A 15-second Home/background → resume cycle kept the connection and resumed
the UI sample counter. This does not establish lossless recording in the
background: this diagnostic drains the shared UI queue only while its UI task
is scheduled.

Stopping the Linux NUS service while leaving Bluetooth powered initially
exposed a half-open connection: Android still reported connected but no new
packets arrived. With shared BLE RX liveness enabled, the final APK logged
`BLE liveness timeout` at 09:03:28 and reported disconnected. After the fixture
was restarted, it automatically rediscovered NUS and resumed fast samples at
09:03:38, without a UI connect action. This test covers service disappearance
and return, not a physical out-of-range radio test.

This is a diagnostic frontend, not the complete Drive Studio Android port.
Slint APK packaging, physical-controller acceptance, long-running background
operation/Doze and high-rate plotting remain separate validation work.

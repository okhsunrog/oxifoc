#!/usr/bin/env bash
# Run in the repository Nix development shell with Android SDK/NDK installed.
set -euo pipefail
cd "$(dirname "$0")/.."
# cargo-rapk expects this conventional directory even when Cargo uses build.build-dir.
mkdir -p target/aarch64-linux-android/{debug,release}/build
cargo rapk build -p oxifoc-host-slint --lib --no-default-features --features android --target aarch64-linux-android "$@"
profile=debug
if [[ " $* " == *" --release "* ]]; then profile=release; fi
manifest="$(<"target/$profile/apk/AndroidManifest.xml")"
if [[ "$manifest" != *'android:usesPermissionFlags="neverForLocation"'* ]]; then
    echo "cargo-rapk dropped BLE scan flags; install the pinned fork from oxifoc-host-slint/README.md." >&2
    exit 1
fi

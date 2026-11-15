#!/usr/bin/env bash
# Run inside the repository's Nix development shell.
set -euo pipefail
cd "$(dirname "$0")"
"${DX:-dx}" build --android --target aarch64-linux-android
app=target/dx/oxifoc-ble-mobile/debug/android/app
# The pinned dx unconditionally writes its default launcher resources.
# Remove alternate formats before overlaying to avoid duplicate resource names.
find "$app/app/src/main/res" -name 'ic_launcher*' -type f -delete
cp -R android-res/. "$app/app/src/main/res/"
(cd "$app" && ./gradlew assembleDebug)

# Generate application icons

Source: `oxifoc-ui/assets/oxifoc-icon.svg`. Requires `rsvg-convert` on PATH.

```sh
uv sync --project scripts/icon-tools
uv run --project scripts/icon-tools scripts/icon-tools/generate.py
```

Generates the PNG and embedded 64px RGBA window icon, Android density PNGs,
adaptive launcher foreground/background, and Android 13 monochrome vector.
Generated assets are committed; ordinary application builds need no image tools.

Build Android with `experiments/ble-mobile/build-android.sh` inside the repository
Nix shell (`DX` may select a matching CLI). The script builds a debug APK, overlays
these launcher resources after the pinned dx writes its defaults, then repackages
with Gradle. A raw `dx build` currently restores the default Dioxus Android icons.

Validated on 2026-11-15: APK resources and installed icon on Pixel 8 Pro; Native
and WebView Linux builds; Native and WebView icons visible in KWin window decorations.

#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
data=${XDG_DATA_HOME:-$HOME/.local/share}
mkdir -p "$data/applications" "$data/icons/hicolor/scalable/apps" "$data/oxifoc"
for variant in native webview slint; do
  if [[ $variant == native ]]; then
    crate=oxifoc-host-dioxus
    bin=oxifoc-host-dioxus
    suffix=Native
  elif [[ $variant == slint ]]; then
    crate=.
    bin=oxifoc-host-slint
    suffix=Slint
  else
    crate=experiments/dioxus-desktop
    bin=drive-studio
    suffix=WebView
  fi
  launcher="$data/oxifoc/launch-$variant"
  # Enter the matching runtime shell on NixOS; use the built binary elsewhere.
  cat > "$launcher" <<SCRIPT
#!/usr/bin/env bash
set -euo pipefail
cd "$root/$crate"
if command -v nix >/dev/null; then
  exec nix develop -c "$root/$crate/target/debug/$bin" "\$@"
else
  exec "$root/$crate/target/debug/$bin" "\$@"
fi
SCRIPT
  chmod +x "$launcher"
  id=com.oxifoc.studio.$variant
  cp "$root/oxifoc-ui/assets/oxifoc-icon.svg" "$data/icons/hicolor/scalable/apps/$id.svg"
  cat > "$data/applications/$id.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Name=OxiFOC Drive Studio ($suffix)
Comment=Motor control and diagnostics
Exec="$launcher"
Icon=$id
StartupWMClass=$id
Terminal=false
Categories=Development;Engineering;
DESKTOP
done
command -v update-desktop-database >/dev/null && update-desktop-database "$data/applications" || true

# Local Linux launchers

After building the Native, WebView and Slint debug binaries, run from the repository:

```sh
packaging/linux/install-launchers.sh
```

This installs separate Native, WebView and Slint desktop entries, SVG icons and launch
wrappers under `$XDG_DATA_HOME` (default `~/.local/share`). Their application IDs
match the Wayland window IDs. On NixOS the wrapper enters the matching dev shell.
Re-run after moving the checkout. These are development launchers; packaged
release executables should use their installed binary paths instead.

All three binaries also embed a 64px RGBA window icon. Linux Wayland compositors
normally resolve the desktop icon from the app ID; the embedded icon supports
platforms that accept window icons directly.

# oxifoc-plot

Shared WGPU telemetry renderer. No UI framework dependencies.

- `PlotBuffer`: synchronized ring buffer, incremental upload snapshots.
- `PlotRenderer`: cached GPU textures, instanced antialiased lines, compute peak
  envelopes, channel visibility, auto-range and PNG export.
- `required_features` / `required_limits`: device requirements used by both GUIs.

Slint owns `oxifoc-host-slint/ui/plot.slint` and its rendering notifier.
Dioxus owns `oxifoc-host-dioxus/src/plot.rs`, a Blitz custom-widget adapter.
Neither adapter owns a copy of the ring buffer, renderer or shaders.

```sh
cargo test -p oxifoc-plot
cargo test -p oxifoc-plot --release -- --include-ignored
```

The ignored test requires an actual GPU and checks the rendered pixels.

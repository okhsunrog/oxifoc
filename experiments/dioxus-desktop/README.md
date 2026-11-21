# Dioxus Desktop plotting experiment

Linux WebView experiment, separate from `oxifoc-host-dioxus` (Blitz/Native).
Dioxus is pinned to the same `97adf791` revision; Wry is locked to 0.57.2.
The Nix shell pins WebKitGTK 2.54.2 and GTK3. No Blitz or WGPU dependency is
included in this experiment.

## Run

From this directory:

```sh
nix develop -c cargo build --release --locked
nix develop -c ./target/release/oxifoc-desktop-plot-probe
```

Run `oxifoc-virtual --transport tcp --port 25266` separately, using the root
workspace's normal development environment. The application uses the existing
`oxifoc-host-lib::HostRuntime`, requests calibration and subscribes to 1 kHz
fast telemetry. It plots calibrated Ia/Ib/Ic, sends acknowledged 1 A current
control / stop commands, and shows RPM and bus voltage. Command outcomes are
logged. It only targets loopback, port 25266 (`OXIFOC_TCP_PORT` overrides it).
No mock data is used in the normal application mode.

`PLOT_BENCH=1` selects an explicit synthetic benchmark: 802×400 physical
pixels at scale 1, three channels, 65,536 samples of ring storage, 12 kHz
source generated in Rust using the earlier Native harness's formula,
fixed Y range −4…4. `PLOT_VISIBLE=1200` selects line segments and `60000`
selects peak envelopes. `PLOT_PAUSED=1` warms the benchmark for 4 seconds and then pauses it.
The synthetic path does not create a host runtime or connect to a controller.

## Rendering and reuse boundary

The current `oxifoc-plot` WGPU renderer requires storage buffers, compute
shaders and `IMMEDIATES`. It cannot run unchanged in WebGL2. This directory
contains a deliberately isolated GLSL ES adaptation of its instanced,
distance-antialiased line segments, using an R32F ring texture instead of
storage buffers. Only changed texture subregions cross the CPU/GPU boundary.
The wide-window path computes min/max on JavaScript's CPU because WebGL2 has
no compute shaders. It is an experimental adapter, not another production
plotting crate, and it does not change or duplicate the shared native core.

Telemetry crosses the Rust/WebView boundary in batches via `document::eval`,
not one IPC call per sample. The long-lived eval remains pending while rAF
callbacks use its channel. Pause stops requesting animation frames; wheel
and preset buttons redraw on demand. In normal mode, the host worker keeps
draining telemetry into a bounded queue while the chart is paused.

The WebGL adaptation uses a fixed Y range and three channels. It does not
implement the Native app's four plots, auto-range, channel toggles, parameters
or session journal. Window lengths are shown in samples; the virtual-controller stream is 1 kHz
and the benchmark stream is 12 kHz.

## Verification and performance

See [RESULTS.md](RESULTS.md) for observed capabilities and measured results.
`tools/measure.py` measures CPU and summed RSS/PSS for the entire application
process tree, including WebKit children. `tools/bench.sh` runs alternating
Desktop/Native cases in the current Wayland session. It uses the existing
Native harness from a `dioxus_native_test` checkout named by `DIOXUS_NATIVE_TEST`.

The integrated launcher `bash tools/run-virtual.sh` starts a fresh virtual
controller and the Desktop app, and stops the controller when the app closes.
It expects the root workspace's `target/debug/oxifoc-virtual` to be built.

In the isolated Wayland session, `GdkScreen` initially reported resolution
−1. WebKitGTK then exposed `devicePixelRatio=-0.0104167` and invalid CSS
geometry. The launcher code now sets a local 96 DPI fallback only when the
screen resolution is nonpositive; it preserves valid desktop DPI settings.
The benchmark verifies `devicePixelRatio=1` and canvas CSS width 802.

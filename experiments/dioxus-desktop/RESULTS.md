# Desktop WebGL experiment — 15 November 2026

On this Linux installation, Dioxus Desktop can render the telemetry plot
through WebGL2. WebGPU is absent in the tested WebKitGTK build, including
inside a secure `dioxus://index.html` context. This is a runtime observation
for this engine/build, not a claim about every Dioxus platform.

## Capabilities

- Dioxus `97adf7915e1adfef4e2da436c9bd0b6add0dfa39`, Wry 0.57.2,
  WebKitGTK 2.54.2, GTK3; no Blitz.
- WebGL 1 and WebGL 2 context creation succeeds. Clearing to
  (0.25, 0.5, 0.75, 1) and reading back returns [64, 128, 191, 255].
- The plot's compiled GLSL program returns GL error 0. A known green line
  reads back as [19, 175, 150, 255], confirming shader rendering.
- `isSecureContext=true`, `navigator.gpu` absent. No WebGPU adapter/device
  can be requested. `EXT_disjoint_timer_query_webgl2` is also unavailable.
- The exposed renderer string is masked as “Apple GPU”. Actual WebKit
  process DRM descriptors point to `/dev/dri/renderD128`; fdinfo identifies
  the `i915` driver and hardware render-engine accounting. This machine is
  Intel Core Ultra 5 125H / Intel Arc Meteor Lake.
- After fixing the invalid headless GDK DPI sentinel, the measured canvas
  is 802 CSS pixels wide with devicePixelRatio 1.

Raw capability observations: [capabilities.json](docs/capabilities.json).

## Comparable plot workload

Release builds, a single 802×400 plot, scale 1, three channels, 65,536-frame
ring, a Rust-generated 12 kHz source with the same sine/ripple formula,
fixed Y −4…4. History is filled before measurement. The Desktop benchmark
hides all panels and does not run HostRuntime or oxifoc-virtual. Native uses
the existing plot-only harness in `dioxus_native_test`, commit `b89f425`.
The native OxiFOC studio and other user applications were left alone.

Desktop uses batched JSON IPC, incremental R32F texture uploads, and
WebGL2 instanced antialiased segments. Its wide-window reduction runs on
the JS CPU; Native uses the existing WGSL compute reduction. This measures
these usable implementations, not an identical shader backend or an
intrinsic limit of either UI framework.

| Workload | Desktop CPU, one core | Native CPU, one core | Desktop updates/s | Native updates/s |
|---|---:|---:|---:|---:|
| Lines, 1,200 samples | 22.49–23.03% | 5.31–6.04% | 59.24–60.00 | 40.97–60.06 |
| Peaks, 60,000 samples | 26.16–28.36% | 6.56–7.48% | 60.00 | 59.42–59.97 |

Desktop PSS is 211.70–213.12 MiB, summed RSS 337.65–339.50 MiB.
Native PSS is 45.94–46.06 MiB, RSS 65.58–65.67 MiB. Desktop totals include
its WebKit WebProcess and NetworkProcess. PSS shares shared pages
proportionally; summed RSS counts shared pages in multiple processes.
Neither is a complete inventory of GPU allocations.

At comparable ~60 update/s runs, the Desktop implementation uses roughly
3.7× CPU for lines and 3.5–4.3× for peak envelopes, with about 4.6× PSS.
The first Native line run had a substantial timing outlier. The machine
was also running unrelated Java/compiler workloads; no processes or CPU
power settings were changed. It would be misleading to use that outlier
to claim a sustained frame-rate advantage for Desktop.

Paused after filling the Desktop history: 0.08% CPU, 202.72 MiB PSS.
Native paused: 0.00% CPU, 45.95 MiB PSS. Neither continuously redraws
an unchanged plot.

## Method and limits

Ten retained runs: two per moving scenario plus one paused run per frontend.
Initial moving scenarios alternated Desktop/Native. After matching the
original WGSL reducer's inclusive, overlapping bin boundaries (to remove
visible gaps), the two Desktop peak measurements were repeated separately. Moving runs have 6 s warmup and at least 12 s measurement. Desktop pause
was measured separately after filling and stopping the history (the initial
cold/empty paused attempt was replaced).
CPU is read from `/proc/PID/stat` over the complete child-process tree;
100% means one full CPU core. Memory is sampled every 0.5 s using
`smaps_rollup`. Governor: `powersave`. Isolated KWin Wayland session,
1200×800 virtual display, approximately 60 Hz.

“Updates/s” counts fresh WebGL draws / Native texture updates, **not**
Wayland presentation timestamps. Browser rAF callbacks and native render
callbacks have different scheduling semantics. Input-to-display latency,
120/144 Hz operation, peak unlimited FPS, power and GPU shader time were
not measured. WebKit's JS clock is visibly quantized; its draw-submission
CPU timing should not be compared directly with Rust's `Instant` timings.

Raw results, including the outlier: [measurements.jsonl](docs/measurements.jsonl).
The shell harness and process-tree sampler are in `tools/`.

## Consequence for OxiFOC

Desktop is viable if full browser CSS/DOM behavior is worth the WebView
cost. For the present OxiFOC project, Native remains the simpler way to
share the existing efficient WGPU renderer with Slint. The current WGSL
uses `IMMEDIATES` as well as compute/storage buffers; even a future WebGPU
port would need a portable parameter-binding path. On this Linux WebView,
WebGL requires a distinct rendering adapter rather than directly reusing
`oxifoc-plot::PlotRenderer`.

## Virtual-controller check

A fresh `oxifoc-virtual` instance on port 25266 supplied calibrated 1 kHz
telemetry through the shared HostRuntime. Applying Iq = 1 A returned
`Applied / Running`; telemetry reached 1876 RPM, 24 V, fault count 0.
Stop returned `Applied / Stopped`. Closing the window also terminated the
launcher-owned simulator. Wheel zoom was checked on a paused plot: the
window changed from 1,200 to 4,386 samples without advancing the data.

[Virtual-controller screenshot](docs/virtual-controller.png).
This is a functional integration check; the full panel's timing is separate
from the plot-only synthetic benchmark above. After repeated forced client
terminations against an old simulator, an earlier run hit `CommTimeout`;
the fresh-server check ran without that fault. Hardware and extended
reconnection reliability were not part of this plotting experiment.

A separate full-panel run with the virtual motor running measured **25.51
fresh updates/s**, 15.80% of one CPU core, 205.12 MiB PSS (332.11 MiB
summed RSS), at the same 1,200-sample window. The simulator is excluded
from those process-tree totals. See [full-panel.jsonl](docs/full-panel.jsonl).
Other interactive observations were around 25–44 updates/s. Thus the 60 Hz
plot-only result must not be advertised as the current full application's
refresh rate. The source is 1 kHz, not 1,000 rendered frames/s. The cause of
the full-panel slowdown has not been isolated between integration and
WebView/compositor scheduling; no TCP or common host-backend changes were
made as part of this experiment.

The final live Niri window uses devicePixelRatio 2 on this desktop. Its
802×400 canvas backing store is stretched at that scale; HiDPI sharpening
is still prototype work and the scale-1 benchmark numbers do not represent
that enlarged live window. Background/occluded live-window observations
were also much slower and are not included in the benchmark comparison.

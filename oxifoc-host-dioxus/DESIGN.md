# OxiFOC native workspace

A drive commissioning desk: plots occupy the main workspace; commands remain
visible beside them. Connection and motor state are separate, explicit statuses.
The interface consumes HostRuntime and host-lib operations exactly like Slint.

Palette: porcelain #f3f6fa, white #ffffff, ink #18334b, muted slate #718398,
blue #285ee8 and teal #087f8c. Signal colors have consistent channel meanings.
Typography: system sans for navigation and text, tabular numerals for readings.
An asymmetric layout gives currents more space than slow temperature telemetry.
Rounded plot surfaces sit on a quiet background; avoid a dashboard full of
unrelated statistic cards. Motion belongs to navigation and focus changes.

Layout: top connection strip; left navigation rail; central plotting workspace;
right persistent drive controls. Parameters and event history replace the center.
At narrow widths the controls flow below the workspace. Disconnected readings
are explicitly marked stale. Pause freezes display history, not acquisition.

First slice: TCP HostConfig (default localhost), four real telemetry plots,
current command and acknowledged stop, configuration readout, connection journal.
Further slices: revision-aware editing, commissioning/detection, transport chooser,
record/replay and full keyboard/accessibility validation.

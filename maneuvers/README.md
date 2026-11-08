# Maneuvers — scripted experiment protocols

A maneuver (in the flight-test / system-identification sense) is a flat,
timed sequence of control commands executed against the device while fast
telemetry records to a parquet file. The same file runs against
`oxifoc-virtual` and against the real board, producing A/B captures of an
identical input profile for sim/hardware diffing.

```sh
oxifoc-host-cli maneuver validate maneuvers/iq-step.json     # offline
oxifoc-host-cli --transport tcp --json maneuver run maneuvers/iq-step.json \
    --out captures/iq-step-$(date +%s).parquet
```

## File format

```json
{
  "name": "iq-step-2A",
  "description": "...",
  "capture": { "fast_hz": 5000, "tail_s": 1.0 },
  "terminal": "stop",
  "timeline": [
    { "t": 0.5, "cmd": { "start": { "iq": 2.0 } } },
    { "t": 2.5, "cmd": { "start": { "iq": 0.0 } } }
  ]
}
```

Commands: `start{iq,id}`, `velocity{rad_s}`, `openloop{current,velocity,angle}`,
`voltage{vd,vq,angle}`, `sixstep{duty}`, `stop{}`, `coast{}`, `brake{}`.
The `terminal` command (`stop`/`coast`/`brake`) is sent on EVERY exit path
of the timeline, including errors. Before anything is sent the timeline is
checked against the device's stored `max_iq_a` (`--force` skips the check;
the device clamps regardless).

The runner consumes telemetry while awaiting each motor response. Timeline
commands are sent once: a missing acknowledgement aborts the experiment
instead of repeating an excitation or sending overdue steps in a burst. The
response deadline is 200 ms after the planned command, or the next timeline
slot plus 5 ms, whichever is earlier. Sending a step more than 5 ms late also
aborts. These are host scheduling bounds, not hard real-time guarantees.
The terminal command gets a separate 2 s acknowledgement window; if it fails,
an emergency stop is attempted after telemetry shutdown. The capture is saved
before operational errors are returned, with all failed stages in
`oxifoc.maneuver_errors`, including terminal/shutdown failures.

`openloop` can also carry `"pi_gains": [0.01, 10.0]` to apply a runtime PI
override, using the existing device control-mode field. Omitting it retains
the current gains. The override is recorded in maneuver metadata and does not
write persistent configuration; it remains active until another override or
controller reconfiguration changes it. This lets stationary-current pilots
use the same conservative gains as detection without modifying flash config.

## Analysis

The parquet metadata embeds the maneuver itself (`oxifoc.maneuver`) and an
event log (`oxifoc.events`): per command — t_planned/t_sent/t_acked plus
**seq anchors** (`seq_before`, `seq_after_ack`). Epochs are cut by raw
device `seq`, not by wall-clock guesswork:

```python
import json, pyarrow.parquet as pq, polars as pl
f = pq.ParquetFile("cap.parquet")
ev = json.loads(dict(f.metadata.metadata)[b"oxifoc.events"])
df = pl.read_parquet("cap.parquet")
# window after event 0: df.filter(pl.col("seq") >= ev[0]["seq_after_ack"])
```

Caveats:

- `seq` is a wrapping 16-bit counter. Repeated values in a long recording
  cannot identify an epoch without surrounding ordered samples. An outage
  longer than a whole wrap (3.28 s at 20 kHz) makes the reconstructed time and
  loss count lower bounds; do not join such segments or fit R from them;
- the distance between an event anchor and the response edge in the data is
  the real command delivery+apply latency. On `oxifoc-virtual` it is ~10 ms
  and depends on `--batch` (commands are processed at the simulation batch
  boundary) — when diffing sim vs hardware, align epochs on the measured
  response edge rather than the anchor if the latencies differ;
- an unloaded virtual motor with default friction hits voltage saturation
  within ~100 ms — long current holds on it show the iq collapse (that is
  physics, not a bug);
- `oxifoc-virtual` is single-client: one CLI session at a time.

## Catalog

- `iq-step.json` — current-loop step response (PI gains, pipeline latency);
- `const-speed.json` — mean id at constant velocity (validates the hall
  boundary-anchor and the phase-advance frame split; an id offset growing
  with speed = commutation angle bias);
- `coast-decay.json` — spin up, then coast: the erpm decay shape gives
  friction (viscous = exponential, coulomb = linear tail); on hardware with
  BEMF sensing the coast window is the lambda ground truth.
- `stationary-voltage-pilot-0p6a.json` — low-current startup qualification,
  fixed electrical angle zero, runtime PI 0.01/10 and native 20 kHz capture.
- `stationary-voltage-signed-1p5a.json` and `stationary-voltage-signed-3a.json`
  — +I/−I/+I plateaus for offset, voltage-slope and drift comparisons. Each
  ramp uses 0.06 A steps every 20 ms, each hold is 1.8 s, with 0.5 s active-zero
  intervals. Both end with Stop and write no persistent configuration. The
  rotor can reorient even though commanded electrical velocity is zero.
  The MK5 bench setup uses 24 V, PSU limit 5.5 A, stored current limit 3 A
  and independent phase-current protection 6 A. Inspect the 1.5 A run before
  proceeding to 3 A; do not bypass a protection fault or incomplete capture.

Choose analysis by the measurement question. For stationary resistance, first
inspect settling, zero-current offsets, signed V/I plateaus and repeat drift.
FFT/PSD/spectrograms are useful for an observed oscillation, periodic artifact
or a noise question; they are not required for every recording. Preserve raw
20 kHz data so that a later hypothesis can be checked without repeating the run.

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

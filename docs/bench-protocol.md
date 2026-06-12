# Bench protocol — first hardware session (B-G431B-ESC1 + Flipsky 5065)

What to test on real hardware, in what order, with what commands, and how
to analyze the captures. Companion to the open items in
[TODO.md → Bench](TODO.md), the failsafe design in [safety.md](safety.md)
and the experiment files in [../maneuvers/](../maneuvers/README.md).

Everything here assumes the **lab-PSU-safe profile is baked**
(`oxifoc-g431/src/baked_config.rs`: failsafe `RampToZero`,
`bus_regen_max_a = 0`, `bus_in_max_a = 10 A`) — the g431 has no flash
persistence, live tweaks die with the session. Reference motor values
(LCR meter + VESC Tool, 2026-06): R = 37.6 mΩ, L ≈ 25.6 µH equivalent
(3-pair LCR: 44.4/51.6/35.8 µH), Lq−Ld ≈ 11 µH, λ = 3.27 mWb, 7 pole
pairs, Kv 270.

## Before the session

- [ ] PSU current limit at ~3 A for the first power-on, raised gradually.
- [ ] Motor mounted, free-spinning, nothing attached to the shaft.
- [ ] `mkdir captures/` — every parquet from the session goes there, named
  `<phase>-<what>-<n>.parquet`. The files carry their own provenance
  (firmware id, config snapshot, decimation, CIC group delay) in the
  parquet metadata.
- [ ] Pre-generate the sim side of every A/B pair against `oxifoc-virtual`
  (same maneuver files, captures named `sim-...`): the bench diff is then
  one Python session.
- [ ] Kill switch = PSU output button. Know where it is.

## Phase 0 — boot, link, telemetry floor

Goal: the device talks, telemetry is intact, and we know what the USB
link actually sustains (open question: 20 kHz raw fits TCP, USB CDC is
untested).

```sh
oxifoc-host-cli --json info          # fw identity, foc_freq
oxifoc-host-cli --json status        # Stopped, 0 faults, plausible vbus
oxifoc-host-cli --json faults
# Link rate ladder — motor OFF, just streaming:
for hz in 1000 5000 10000 20000; do
  oxifoc-host-cli --json record --out captures/p0-idle-$hz.parquet \
      --seconds 10 --fast-hz $hz --allow-gaps
done
```

Pass: `gaps == 0` up to some rate; note the highest clean rate — that is
the session's raw-capture budget (detection `--record` wants 20 kHz; if
USB can't, captures get gaps — analysis must window around them).

Analyze the idle captures for the **noise floor**: std of ia/ib/ic with
gates off is the ADC+EMI floor; compare against the 15 mA LSB the sim
assumes. This number calibrates every later SNR judgement.

Watch item (TODO): ISR load / F405-style ADC double-trigger — if
SlowTelemetry seq or the fast stream shows a 2× rate anomaly, check
JEOC rate before trusting anything else.

## Phase 1 — hall validation, BY HAND (no power stage)

Goal: the 2026-06-10 timer-capture migration and the boundary-anchor fix
feed commutation; they have never run on hardware.

Procedure: motor gates off (`stop`), spin the shaft by hand both ways
while recording:

```sh
oxifoc-host-cli --json record --out captures/p1-hall-hand.parquet \
    --seconds 20 --fast-hz 5000
```

Checks (Python below):
- `hall_state` walks 1→3→2→6→4→5 forward, reversed backward; **no 0 or 7**
  (those = invalid state, would mean wiring/pull-up trouble).
- `erpm` sign matches rotation direction, magnitude plausible for a hand
  spin (a 2× skew = the old prescaler bug class), **no spikes at sector
  boundaries**.
- Pull one hall wire mid-spin → `faults` must show HallError with the
  dead wire NAMED in the details ("dead wire: H2" — the per-bit detector,
  needs ~6 electrical revolutions of spinning to conclude; the bare
  invalid-state warning appears immediately). The record is sticky:
  it survives re-plugging the wire, clears on `faults --clear`.
- After a reboot with the rotor parked: `status` → first commutation
  must have a valid angle immediately (boot seed from the static hall
  state).

```python
import polars as pl
df = pl.read_parquet("captures/p1-hall-hand.parquet")
seq = df["hall_state"].to_list()
edges = [(a, b) for a, b in zip(seq, seq[1:]) if a != b]
fwd = {(1,3),(3,2),(2,6),(6,4),(4,5),(5,1)}
bad = [e for e in edges if e not in fwd and (e[1], e[0]) not in fwd]
print("invalid transitions:", bad)        # must be []
print("states seen:", sorted(set(seq)))   # must be ⊆ {1..6}
```

## Phase 2 — first powered spin, current quality

Goal: current calibration is sane, commutation geometry is right.

```sh
oxifoc-host-cli --json start --iq 1.0     # listen; should spin smoothly
oxifoc-host-cli --json record --out captures/p2-spin-1A.parquet \
    --seconds 5 --fast-hz 10000
oxifoc-host-cli stop
```

Analysis — the three classic signatures, all by **order analysis**
(resample by electrical angle, not time; `angle_rad` is in the capture):
- **1st order of eRPM in dq** → current-sensor offset error;
- **2nd order** → per-phase gain mismatch;
- **6th order** → dead-time/inverter distortion (we model it in sim — the
  measured amplitude calibrates `dead_time_v`);
- **mean id ≈ 0 at constant speed** — THE acceptance for the hall
  boundary-anchor fix (9f936bb) and the phase-advance frame split: before
  those fixes a ~30° commutation lead showed as a large negative id and
  cos-loss of torque. A mean id offset that **grows with speed** =
  residual commutation angle bias.

```python
import numpy as np, polars as pl
df = pl.read_parquet("captures/p2-spin-1A.parquet")
steady = df.slice(len(df)//2)             # second half = settled
print("mean id:", steady["id"].mean(), "mean iq:", steady["iq"].mean())
# order spectrum: resample ia against angle, FFT over revolutions
```

Then `maneuvers/const-speed.json` (velocity loop on soft default gains)
for the same check at a controlled speed, diffable against the sim run.

## Phase 3 — detection, with capture, against references

Goal: our detection becomes the trusted instrument. Every step runs with
`--record` (raw FOC-rate capture; decimated rates would CIC-null the HFI
carrier) and `--json` output saved.

```sh
oxifoc-host-cli --json detect resistance --apply \
    --record captures/p3-detect-r.parquet | tee captures/p3-r.json
oxifoc-host-cli --json detect inductance --apply \
    --record captures/p3-detect-l.parquet | tee captures/p3-l.json
oxifoc-host-cli --json detect flux --apply \
    --record captures/p3-detect-flux.parquet | tee captures/p3-flux.json
oxifoc-host-cli --json detect hall
```

Acceptance, per step:

| Step | Reference | Pass band | If it fails |
|---|---|---|---|
| R | LCR / multimeter DC: 37.6 mΩ | ±10% | probe-retry triggered? dead-time make-up visible in the capture's vd ramp |
| Ld/Lq | LCR 3-pair (saturation caveat below) | l_avg within ±20% of 25.6 µH; Lq>Ld | see lag triage below |
| λ | VESC Tool 3.27 mWb + Kv-270 physics anchor | ±10% | driven method: check the spin reached v_target; old GUI λ values are known garbage |
| hall | Phase-1 by-hand data | all 6 sectors, centroids ~60° apart | recalibrate; check mechanical mounting |

**The lag triage (the heart of this phase).** The detection log and the
`detect --record` metadata carry the probed pipeline lag:
- sim predicts **lag = 2** for the firmware pipeline; the probed value on
  hardware is a real measurement of the ergot→ISR command path — write it
  down, it also feeds the deadman-budget understanding;
- if the |Z| cross-check trips (LowConfidence → pulse fallback), the
  demod and magnitude disagree — capture analysis: correlate the recorded
  carrier against currents at lags 1–4 offline (same math as the probe)
  and check whether the lag drifts *within* a measurement (async delivery
  jitter — would need the probe to run per-window);
- LCR saturation caveat: the LCR measures small-signal L at zero bias,
  detection measures incremental L under the hold current — a 10–20%
  difference with detection BELOW LCR is expected physics, not an error.

**Repeatability**: run `detect resistance` and `detect inductance` 5×
each; the spread (std/mean) is the instrument's noise floor. > 5% spread
on R or > 10% on L → investigate before trusting single runs.

After all steps: `config dump --rust` → that is the new baked config,
**from our own detection**.

## Phase 4 — maneuvers (A/B against the sim)

Run the prepared maneuvers; each produces a parquet with a seq-anchored
event log. Diff against the pre-generated `sim-*` captures.

| Maneuver | What it validates | Key metric |
|---|---|---|
| `iq-step.json` | current-loop PI (detection-derived gains), command latency | rise 10–90% vs sim; command→edge latency (response edge, NOT the ack — ack arrives ~25 ms after the effect) |
| `const-speed.json` | commutation centering at speed | mean id over the steady window ≈ 0, flat vs speed |
| `coast-decay.json` | friction model, λ sanity | erpm decay shape: exponential = viscous, linear tail = coulomb; feeds the sim's friction params |

Epoch cutting recipe is in [../maneuvers/README.md](../maneuvers/README.md)
(`oxifoc.events` metadata, cut by `seq`, align on the measured edge when
latencies differ).

## Phase 5 — failsafe drills (low speed only)

Bench profile is `RampToZero` — these are PSU-safe:

- Link pull at ~1 A spin: unplug USB → motor must unload (no regen) within
  the deadman window; record beforehand
  (`record --seconds 30 --allow-gaps` in a second terminal won't survive
  the unplug — capture from the moment of reconnect instead, and check
  `status` shows the re-arm latch until an explicit `stop`).
- `brake` at near-standstill: engages; at speed: must be REJECTED
  (entry gate) — check the JSON error.
- OCP/BKF: do NOT provoke deliberately this session; if it trips during
  other tests, the capture around the trip is gold — save it.

## Phase 6 — sensored runs at speed

- `source hall` (default) vs `source hall-fallback`: ride through the
  blend band, record, compare angle continuity (no erpm/iq glitches at
  crossover).
- HFI on real iron (`source hfi` at standstill, gentle iq): the carrier
  amplitude now derives from measured L — verify the ~2 A ripple target
  in a raw capture, listen for acoustics; polarity probe correctness =
  no 180° runaway on first torque. Then `hfi-observer` and a slow
  ramp through the downward crossover (pre-heat + cold-demod trust gate —
  sim-proven, hardware-unproven).

## Analysis quick-reference

All captures: check integrity FIRST (`seq` deltas uniform = decimation M;
anything larger = dropped frames — the summary already counts them, but
windowed analysis must avoid the holes).

```python
import json, numpy as np, polars as pl, pyarrow.parquet as pq
f = pq.ParquetFile("captures/x.parquet")
meta = {k.decode(): v.decode() for k, v in f.metadata.metadata.items()}
df = pl.read_parquet("captures/x.parquet")
d = np.diff(df["seq"].to_numpy())
assert len(np.unique(d)) == 1, f"gaps! deltas {np.unique(d)}"
```

- Spectra: only on raw (M=1) captures or below the decimated Nyquist;
  remember the CIC group delay (M−1 input samples, in metadata) for any
  phase-sensitive work.
- Sim-vs-hw diffs: same maneuver, cut epochs by events, compare
  step-response metrics and order spectra; disagreement = unmodeled
  physics (candidate for the VirtualMotor fidelity ladder) or a bug —
  both are findings, write them into TODO.md/decisions.md same-day.
- Session output: `config dump --rust` (new baked config), the probed
  pipeline lag, the link-rate budget, the noise floor, and every capture
  with its JSON log.

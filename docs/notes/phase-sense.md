# Phase-voltage sensing — design & integration tracker

Living tracker for adding phase-terminal voltage sensing (back-EMF / undriven
rotation detection / flying start). Feeds the backlog item "detect a spinning
motor + flying-restart synchronization" in [../TODO.md](../TODO.md).

**Reference firmwares** (architecture only — VESC/MESC are GPL, oxifoc is
MIT/Apache, so this is clean-room: idea, not code):
- VESC `update_valpha_vbeta` (mcpwm_foc.c) — observer voltage source,
  `foc_offsets_voltage[_undriven]`, `FAULT_CODE_PHASE_FILTER`.
- MESC `MOTOR_STATE_TRACKING` (MESCfoc.c) — bridge off → measured `Vph` →
  `MESCfluxobs_run` (the MXLEMMING flux observer, which is the lineage of our
  `BackEmfObserver`).

## Status

| Phase | Scope | State |
|------|-------|-------|
| 0 | Core types: capability, snapshot field, converter, policy | ✅ done (host) |
| 1 | Observer voltage-source wiring in `FocDriver` | ✅ done (host) |
| 2 | f405 ADC acquisition (PA0/1/2) | ☐ HW |
| 3 | Undriven offset calibration + storage | ☐ |
| 4 | Flying start / spin-catch handoff | ☐ HW |
| 5 | Telemetry + open-phase diagnostics | ☐ opt |
| 6 | Docs / decisions | ☐ |

HW = needs bench validation before it can be trusted (mark in TODO.md).

## The relationship: sensing vs filters

- **Phase sensing** = each phase terminal is routed through a resistor divider
  to an ADC channel (raw capability).
- **Phase filters** = an RC low-pass *on those same sense lines*, so the
  measurement is valid *while the bridge is actively PWMing* (it averages out
  the switching node). Filters imply sensing; the reverse is not true.

Without filters the measurement is only meaningful when the bridge is high-Z
(all FETs off → terminal voltage = back-EMF). This is why the capability is
modelled as `Option<PhaseSense { divider_ratio, has_filters }>` — `has_filters`
lives *inside* `PhaseSense`, so it cannot be set without sensing.

### Board capability matrix

| Board | `phase_sense` | Notes |
|-------|---------------|-------|
| Cheap FOCer 2 (f405) | `Some { (39k+2.2k)/2.2k, has_filters: false }` | PA0/1/2 = SENS1/2/3; phase divider = Vbus divider; **no RC filter** → undriven BEMF only |
| B-G431B-ESC1 (g431) | `None` | BEMF nets are clamped (zero-crossing only) — useless for a full αβ projection |
| X-NUCLEO-IHM08M1 (g474) | `None` | not wired |

## Observer voltage source — the one decision point

Both VESC and MESC converge on: the *same* back-EMF observer runs continuously;
only its **voltage input** switches between commanded and measured.

`foc::phase_voltage::observer_voltage_source(sensor, bridge_driven)`:

| sensor | bridge | source |
|--------|--------|--------|
| `None` | any | Commanded |
| `Some` | high-Z (Stopped/Coast) | **Measured** (back-EMF) |
| `Some{filters}` | driven | **Measured** (phase-filter refine) |
| `Some{no filters}` | driven | Commanded |

`Brake` is excluded from "high-Z" (low sides shorted, terminals ≈ 0 V): it keeps
its own `(0 V, measured-i)` feed.

The back-EMF observer integrates `(v − R·i)·dt − L·Δi`. Undriven, currents are
≈ 0, so it integrates pure `∫v·dt = flux` and the PLL extracts angle + velocity
— which is exactly coasting-rotation detection / flying-start readiness.

## Data path (chosen: option A — single snapshot)

`AdcSnapshot.vphase: Option<[u16; 3]>` carries raw counts ISR→core. `AdcSnapshot`
is **internal** (not `Serialize`); the wire telemetry is the separate POD
`FastTelemetry`. So a no-sensing board pays only ~8 bytes in the single cached
`MotorControlState.last_adc` (one `None`), and **zero** on the wire / in the
telemetry ring buffer. Exposing phase voltage in `record` is a separate,
deliberate change to `FastTelemetry` (Phase 5), not part of the data path.

Converter (`foc::phase_voltage::PhaseVoltageSense`) uses the **full 3-input
Clarke** `((2Va−Vb−Vc)/3, (Vb−Vc)/√3)`, not the 2-input current Clarke: measured
phase voltages carry a common-mode (floating-neutral) term that the 3-input form
cancels. A consequence — the αβ projection is correct from the matched component
alone, so it degrades gracefully before the offsets are calibrated (offsets only
remove channel-to-channel *differential* bias).

## Phase notes

### Phase 0 — core types ✅
`config::PhaseSense` + `BoardConfig.phase_sense`; `AdcSnapshot.vphase` +
`with_phase_voltages`; `foc::phase_voltage` (`PhaseVoltageSense`,
`ObserverVoltage`, `observer_voltage_source`). 8 unit tests.

### Phase 1 — observer wiring ✅
`ControlMode::is_high_z()` (Stopped/Coast). `FocDriver` holds
`phase_voltage: Option<PhaseVoltageSense>` + per-cycle `vphase_raw`, with
`set_phase_voltage_sense` / `set_phase_voltage_raw`. `update_phase_with_prev_voltage`
picks measured vs commanded via `observer_voltage_source`; measured is *this*
cycle's sample (concurrent with the currents, no one-cycle delay, unlike the
commanded path). No device crate touched → g431/g474/f405 byte-identical.
Test `coast_observer_tracks_measured_bemf`: synthetic coasting BEMF as raw
counts → observer locks to ω (<5%, ready); no-sensing control board ignores it.

### Phase 2 — f405 ADC acquisition ☐
`init_adc`: 3rd injected channel per ADC (PA0→ADC1, PA1→ADC2, PA2→ADC3, const
generic 2→3). Current stays channel 0 (sampled first at TIM1_CC4, timing
unchanged); phase voltage ~2.6 µs later (fine for undriven BEMF). ISR: store
`VPHASE_A/B/C` atomics + `snapshot.with_phase_voltages([..])`; wire
`PhaseVoltageSense::from_board(&BOARD)` into the driver and
`set_phase_voltage_raw(snapshot.vphase)` each cycle.

### Phase 3 — undriven offset calibration ☐
Use a small streaming sum → `PhaseVoltageSense::calibrate_offsets_from_sums`.
Run at boot beside the ISR-owned current-offset calibration, during an explicit
undriven/high-Z window. Store as a new
`RuntimeConfig.phase_voltage_offsets` group; apply via `set_offsets`. VESC keeps
two sets (driven zero-vector + undriven, both minus the 3-phase average); we need
only the undriven set for CF2.

### Phase 4 — flying start / spin-catch ☐
With Phase 1, the observer already tracks in `Coast`. On a start command
(Stopped/Coast → CurrentControl), if `observer.is_ready()`, seed the control
angle/velocity from it (`PhaseManager::seed`/`force_phase`) instead of starting
from 0/open-loop. `Coast` + reading `observer.velocity()` already gives
"is it spinning / how fast" without committing to drive.

### Phase 5 — telemetry + diagnostics ☐ (optional, bench)
`bemf_erpm`/`bemf_angle` (or `va/vb/vc`) as plain `f32` in `FastTelemetry` for
bench visibility (POD = all boards pay the bytes; do it only when bench needs
it). Open-phase detector (one terminal stuck undriven). Skip
`FAULT_CODE_PHASE_FILTER` — it only applies to filtered boards.

### Phase 6 — docs ☐
Fold the settled design into decisions.md; drop the HW-unvalidated items into
TODO.md as they land.

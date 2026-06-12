# Fault system overhaul: response classes, derating, hall health

Status: partially landed — phase 1 (severity classes, class-based gate,
deadman → CommTimeout) landed 2026-06-12; phases 2–6 open. Companion to
[../safety.md](../safety.md) (failsafe layers) and the Bench section of
[../TODO.md](../TODO.md).

## Motivation

Three converging findings:

1. `run_foc_cycle` stops the motor on **any** registry fault —
   `PlatformFault::is_critical()` exists but the gate ignores it. Every
   fault is de-facto critical, so warning-class conditions (hall
   degradation) cannot be reported without killing the ride.
2. Hard-stop is the wrong response for most faults on a vehicle. VESC
   treats **derating as the primary mechanism** and hard faults as the
   end of the ramp; we have no derating layer at all (OverTemp = instant
   high-Z).
3. Hall failure handling is fully silent: `PhaseFault`/`hall_health`
   never leave `PhaseManager`, `FaultCategory::HallError` is defined on
   every board and never raised. The rider learns about a dead hall at
   the next standstill start.

## Reference findings (2026-06-12)

**VESC** (`mc_interface.c`): `update_override_limits()` at ~1 kHz computes
live current limits from ~10 simultaneous linear ramps — FET/motor temp
(start→end), **accel/brake asymmetric thermal derate** (`l_temp_accel_dec`:
hot controller loses acceleration before braking), battery-voltage input
cutoff (UV is a ramp, not a fault), regen-overvoltage cutoff, soft eRPM
cut, duty derate, wattage/BMS — combined via `min_abs`. Hard faults are
the END of ramps (temp past `l_temp_fet_end` → lo=0 AND fault) plus true
hardware trips (ABS overcurrent, DRV, BRK). Voltage faults go through an
**integrator** (tolerate brief excursions). On fault: rich snapshot
(I/V/duty/rpm/temp/comm-step) to the fault logger, stop PWM, block for
`m_fault_stop_time_ms` (500 ms default), then **auto-recover** — no
manual clear.

**MESC** (`MESCerror.c`, `MESCfoc.c`): harder line — `handleError` always
generates break (high-Z) + ERROR latch + 32-bit error mask (separate
`HALL Sensor [0]`/`[7]` bits) + first-error snapshot. But temperature is
the same philosophy: `T_rollback` continuously scales the current request
from `Thot` to `Tmax`; error only past `Tmax`.

Shared lesson: **stop is the response to "derating has already failed",
not to "hot" or "high current".**

## Design

### 1. Three response classes

```
Warning      → report only (registry + topic); motor untouched
GracefulStop → existing failsafe machinery (RampToZero / ControlledStop)
Kill         → immediate high-Z + Error latch (current behavior)
```

`run_foc_cycle` gate becomes class-based instead of `any()`. Class
assignment:

| Fault | Class | Why |
|---|---|---|
| ABS overcurrent (per-phase + dq trip), BKIN, DriverFault | Kill | inverter integrity — no choice |
| OverVoltage (outside the regen ramp) | Kill | power-stage protection; high-Z breaks the charge path |
| OverTemp (ramp ceiling), UnderVoltage (ramp floor), CommTimeout, Stall | GracefulStop | inverter is healthy — stop gracefully |
| HallError, derating-active | Warning | keep riding, inform |

The failsafe machinery is the **executor** of GracefulStop (reuse
`enter_failsafe()` — ramp, controlled stop, no-progress watchdog, re-arm
latch all exist); the deadman becomes one of its **detectors**: expiry
raises `CommTimeout` into the registry (ISR-safe set), fixing today's
invisible-deadman problem — after reconnect the host/remote sees the
cause, not just the re-arm latch. The deadman action stays ISR-resident
(Layer-2 property preserved).

Re-arm policy per class: GracefulStop faults auto-rearm after a timeout
**iff the condition cleared** (VESC-style, via the existing re-arm latch —
throttle must be released); Kill faults latch until explicit clear.
Stranding the rider over a transient is itself a hazard.

### 2. Severity on the wire + push to the remote

- `FaultInfo` grows a `severity` field (wire change — fine, one tree).
  The remote reacts to severity (vibration pattern, display), never to
  hardcoded categories.
- **FaultTopic**: a task on `registry.wait_for_change()` publishes the
  full snapshot (`Vec<FaultInfo>` + total — idempotent, BLE-loss
  tolerant). `SlowTelemetry.fault_count` stays as the poll backstop
  (count mismatch → re-query via `FaultEndpoint`).
- Fault moment snapshot: put I/vbus/erpm/temp at trip time into
  `details` (VESC fdata pattern). The full answer is the device-RAM
  burst capture with fault pre-trigger (TODO.md Host) — same trigger
  point, richer data.

### 3. Derating layer (graduated limits)

Continuous, BEFORE any fault — new `DeratingConfig` group:

- FET temp ramp `temp_fet_start..end`, motor temp ramp (scales the
  effective `max_current_a`);
- **accel/brake asymmetry** (`accel_derate_factor`, VESC
  `l_temp_accel_dec`): hot board cuts acceleration first, preserves
  braking;
- battery cutoff `vbus_cut_start..end` (input current → 0 as the pack
  sags; the board `min_vbus_mv` fault remains the last resort);
- regen OV cutoff `vbus_regen_start..end` (regen current → 0 as vbus
  approaches the ceiling — composes with the existing
  `bus_regen_max_a` clamp and the OV fault).

Mechanics: scale factors computed ISR-side, decimated (~/256 cycles —
temp/vbus are slow; ISR residency = executor-hang immunity), applied as
dynamic ceilings in the existing `clamp_targets` / `clamp_iq_for_bus`
points, composed via min. Derating beyond ~20% raises a Warning so the
rider knows why the board feels weak.

Voltage/current fault detectors get integrators (existing TODO item,
folded here): single-sample trips on EMI/regen transients are nuisance
torque loss.

### 4. Current-limit ladder audit (foot-gun found 2026-06-12)

Today's ladder (g431): `max_iq_a` soft clamp (10 A) → dq-magnitude trip
(config 40 A, ceiling min(1.3×hw, 1.5×rating)) → per-phase instantaneous
fault (`board.max_phase_current_a` = 40 A) → hardware COMP/BKIN 80 A
(currently disabled — MOE re-arm glitch).

Problems:

- **No cross-field validation**: config `max_iq_a = 40` +
  `max_phase_current_a = 40` is accepted → full throttle (+ PI
  overshoot + ~2 A HFI ripple) = instant Kill.
- **Layer inconsistency**: iq ceiling = hw_max = 40 A while the
  per-phase fault fires at exactly 40 A — zero margin at the board
  level; the dq trip (52 A after ceiling) sits ABOVE the per-phase
  fault, so the effective Kill is the per-phase check.

Fix: (a) config-write validation rejects
`max_phase_current_a < 1.3 × max_iq_a`; (b) harmonize:
`board.max_phase_current_a` is the ABS trip (VESC `l_abs_current_max`
analog), iq ceiling = hw/1.3; (c) headroom must cover PI overshoot +
HFI carrier ripple + noise.

### 5. Hall health → faults

- Bridge `PhaseManager` faults/health into the registry as **sticky
  Warning** `HallError` (latched until host clear even if hall
  "recovers" — a sensor that died once mid-ride is suspect, and
  partial-failure flapping must not buzz the remote at 10 Hz).
- **Per-bit wire detector** (core, no platform changes): in
  `HallSensor::update()` count transitions per hall bit (XOR new/prev
  raw_state). Over ~2 electrical revolutions every live bit toggles
  ~2×/rev; a dead wire = its counter at zero while others tick. Detects
  1 and 2 broken wires, names the culprit ("hall wire H2 dead" in
  details). Needs rotation — standstill is undetectable and harmless.
- **Error-rate window**: `error_count` delta per M valid edges catches
  bounce/EMI (stochastic errors the per-bit counters miss). Threshold →
  same sticky warning + stop trusting hall (PhaseManager degraded latch).
- **`angle_trustworthy()` honesty**: must return `false` while the
  open-loop recovery override is active (today hall sources hardcode
  `true` → full commanded iq against a fictitious angle at low speed;
  the existing iq gate in `step_current_control` then makes dead-hall
  low speed safe automatically, and failsafe ControlledStop stops
  commutating the brake against a fake angle).
- (post-bench) **Auto-promotion to sensorless**: confirmed hall death →
  internal switch to `HfiToObserverVolts` — full limp-home including
  standstill starts; sensorless stack is code-complete but
  hardware-unvalidated.

## Implementation order

1. **[landed 2026-06-12]** Classes + gate + deadman fold-in:
   `FaultSeverity` on the wire (central policy in
   `FaultCategory::severity`, pinned by `severity_policy_pinned`),
   class-based gate in `run_foc_cycle`
   (`severity_gate` tests), deadman/link gate → `CommTimeout` with
   auto-clear on a drained `SetMode`. +260 B on g431.
2. **Limit-ladder fixes**: cross-field validation + board-level
   harmonization (small, independent, removes the foot-gun).
3. **Hall package**: PhaseFault bridge (sticky warning), per-bit wire
   detector, error-rate window, `angle_trustworthy()` fix. All
   sim-testable (mask hall bits in VirtualMotor output).
4. **FaultTopic** push + remote severity UX (vibration/display) — needs
   remote firmware maturity, protocol side can land earlier.
5. **Derating layer** + integrating detectors (biggest piece; needs
   bench thermals to tune, but the structure and sim tests come first).
6. (post-bench) auto-promotion to sensorless; dissipative braking near
   OV (already in TODO Safety).

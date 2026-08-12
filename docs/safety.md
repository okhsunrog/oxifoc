# Safety Architecture & Failsafe Design Notes

Working design notes for the failsafe behaviour of the motor controller. This
is a vehicle (electric longboard): an uncontrolled motor at speed is dangerous,
so the guiding principle is **fail-safe by default** — the safe state must be
reached by the *absence* of positive control, never depend on a message being
delivered.

Status legend: **[done]** implemented · **[planned]** not yet · **[idea]** to evaluate.

## Principles

- **Fail-safe / negative logic.** The motor runs only while it receives
  continuous positive affirmation (fresh commands / a live link). Loss of that
  affirmation → safe state. We never rely on a "stop" command arriving, because
  on an unreliable transport it can be lost (ergot is at-most-once; see
  *Command delivery* below).
- **Defense in depth, by failure scope.** Separate mechanisms cover separate
  failure classes (host gone / async executor hung / firmware hung). They are
  not redundant link-checks; each catches what the layer above cannot.
- **The safety supervisor runs at least as reliably as the actuator.** Motor
  control lives in the FOC ISR, so the watchdog that guards it must also run in
  the ISR (or in hardware), not in an async task that can be starved.
- **Graceful degradation.** At speed, a hard PWM cut = freewheel (no braking);
  an abrupt brake can throw the rider. Failsafe actions are *controlled*
  (ramp-to-zero / configurable regen braking), not instantaneous cuts.

## Layered failsafe

| Layer | Failure it catches | Mechanism | Reaction time | Status |
|------|--------------------|-----------|---------------|--------|
| 1. Link gate | Host disconnected / link silent | ergot liveness → `state_notify` → `state_monitor` clears `link_active` → FOC routes through the failsafe policy | liveness timeout (1 s) | **[done]** |
| 2. ISR command-staleness deadman | Host alive but not commanding; **async executor hung** | Stamp `last_cmd_tick` in ISR when draining `CMD_CHANNEL`; if `now - last_cmd_tick > thr` → configurable failsafe mode | ~150 ms (configurable) | **[done]** |
| 3. Panic/HardFault gate kill | **Firmware panicked / hard-faulted** | Custom handlers clear `TIM1 BDTR.MOE` (+ EN_GATE low on F405) *before* any reporting (`safety.rs` per board) | immediate | **[done]** |
| 4. Independent watchdog (IWDG) | **FOC ISR itself stopped** (lockup, clock fault, priority lock) | IWDG petted from the FOC ISR; if even the ISR stops → MCU reset → PWM goes high-Z/off | 3 s (F405) | **[done]** (G474 pending — FOC ISR dormant) |

Key insight: Layer 1 is **async-executor-dependent** (liveness runs in the RX
worker; `link_active` is cleared by the async `state_monitor`). If the executor
hangs, Layer 1 does **not** fire. Layer 2 (ISR-resident) survives executor
starvation and **subsumes Layer 1's coverage** — once Layer 2 exists, the
Layer 1 gate can be removed or folded in as one input. Layers 3 and 4 are the
backstops below both.

### Layer 1 — link gate (implemented)

Chain: ergot liveness timeout → interface → `Down`/`Inactive` → `state_notify`
wakes `state_monitor` (`oxifoc-g474/src/protocol/servers.rs`) → on `!any_active`
calls `MotorControlState::set_link_inactive()` → `process_commands`
(`oxifoc-core/src/state.rs`, runs in the FOC ISR) forces `ControlMode::Stopped`
while `!link_active`. After reconnect the host must send a fresh command to run
again.

Now that Layer 2 exists, this gate **routes through the same failsafe policy**
(`process_commands` → `FocDriver::enter_failsafe`) rather than a hard `Stopped`,
and the liveness timeout was shortened 5 s → 1 s
([`LIVENESS_TIMEOUT_MS`](../oxifoc-core/src/icd.rs)). Layer 2 is the fast net.
The gate exempts the safe standing states (`Stopped`/`Coast`/`Brake`) and does
not force the shared state to stopped while the brake still runs — that syncs
at the failsafe terminal in `run_foc_cycle`, so the config server's
motor-running gate can't admit a flash stall mid-brake.

### Layer 2 — ISR command-staleness deadman (implemented 2026-06-11)

`FocDriver` carries `last_cmd_tick`; `process_commands` stamps it on every
drained `SetMode` (the host re-affirms the active setpoint every 50 ms —
`AFFIRM_POLICY` in host-lib, fire-and-forget so a dying link isn't masked by
retries). `run_foc_cycle` checks `now_ticks - last_cmd_tick > staleness_timeout`
(default 150 ms) while running and, if stale, raises the **CommTimeout fault**
(GracefulStop class) — the severity gate then arms the failsafe (see
[notes/fault-overhaul.md](notes/fault-overhaul.md): the deadman and the
Layer-1 link gate are *detectors*, the fault gate is the *executor*, and the
event is visible to the host/remote instead of silent). A drained `SetMode` —
accepted or not — clears the fault; the re-arm latch still demands the
explicit safe-mode acknowledgement. The check is ISR-resident (`now_ticks` is
the 1 MHz hall capture timer), so it survives an async-executor hang where
Layer 1 can't fire.

The failsafe is a self-contained controller
([`motor/failsafe.rs`](../oxifoc-core/src/motor/failsafe.rs)) run every cycle
with no further input, host-configurable via `FailsafeConfigStored` (ConfigKey
9):

- **Coast** — cut PWM (high-Z / free-wheel); reproduces the legacy hard Stop.
- **RampToZero** — slew the q-current to zero, then coast.
- **ControlledStop** — ramp to zero, then **regen-brake to a standstill at a
  bounded deceleration**: the velocity reference ramps to zero at
  `decel_rad_s2` through the failsafe's *own* `VelocityLoop` instance (fixed
  conservative gains — the host-tunable cruise loop is never the safety
  net), so the stop feels the same on a slope as on the flat, up to the
  current cap. Unidirectional (only ever opposes the original rotation),
  capped at `brake_current_a`, OV-derated. On a clean stop the configurable
  **terminal** applies: high-Z, or `ParkBrake` (default — hand over to
  `ControlMode::Brake` so the stopped board doesn't roll away). The give-up
  exits never engage the brake at speed:
  - **no-progress watchdog** — |ω| hasn't improved for 2 s (broken estimate,
    or a descent the cap can't beat) → coast; a brake merely *holding* speed
    on a hill gets the full window first.
  - hard time cap (`brake_time_s`, default 10 s) → coast.
  - angle source lost trust mid-brake → coast (never commutate blind).
  - regen pushing the bus into the OverVoltage fault → high-Z via the fault
    gate.

The same machinery serves the user-commanded **ramp-into-brake**: a `Brake`
command above the standstill gate is substituted with ControlledStop ending
in `ParkBrake` instead of being rejected (`FocDriver::enter_brake_ramp`) —
but as a *user* action it does not set the re-arm latch.

Physical limit (document once, accept forever): on a long descent with a
**full battery** regen has nowhere to put the energy — the OV derate sheds
the brake exactly when it's needed most. Electrically the only answer there
is dissipative braking (short/heat in the windings); until that exists,
guaranteed stopping power on a steep descent at full charge does not — the
rider's foot is the backstop.

**Scope.** The deadman covers only the *drive* modes (`CurrentControl`, plus
velocity/position once they exist) — what a vehicle rides on. Exempt:

- `Stopped`/`Coast`/`Brake` — safe standing states (parking `Brake` = all
  low-side FETs on / windings shorted, added 2026-06-11): they need no
  affirmation, and a parked board must stay braked through link loss, so the
  Layer-1 gate skips them too. `Brake` entry is speed-gated in
  `process_commands` (`BRAKE_ENTRY_MAX_E_RAD_S`): shorting the windings at
  speed dumps an uncontrolled back-EMF-driven current (→ λ/L) through the
  FETs. Note it is a *viscous* brake (torque ∝ speed, dissipated in the
  motor, zero draw at standstill) — on a slope the board creeps; true
  position hold is a future position-loop feature (see TODO).
- `OpenLoop`/`DirectVoltage`/`SixStep` — bench/calibration modes: on-device
  detection dwells up to ~1 s between `SetMode`s (R-measurement settle),
  which a 150 ms deadman would cut mid-measurement. The Layer-1 link gate
  (1 s) still covers them against a dead host.

**Re-arm latch.** Every failsafe engagement latches `failsafe_latched` in
`FocDriver`; while latched, `process_commands` rejects running modes (the
rejected `SetMode` still stamps the deadman — liveness ≠ acceptance) until
the host acknowledges with an explicit `Stopped`/`Coast`/`Brake` — "throttle
back to neutral", as on RC/e-bike systems. Defense in depth: host-lib also
drops its active setpoint on disconnect (and sends affirms from the same
task as commands, so an affirm can't overtake a Stop), but the device does
not trust the host — a reconnecting host replaying a stale throttle cannot
relaunch the board.

`enter_failsafe` is bumpless (seeds the ramp from the commanded
`CurrentControl`/`OpenLoop` q-current) and restores six-step-floated phases
before driving the current loop.

Open: bench-tune the brake constants and confirm no OV trip under regen (see
[TODO.md](TODO.md#safety)).

### Layer 3 — panic/HardFault gate kill (implemented)

Each STM32 firmware has its own `safety.rs` replacing `panic_probe`: the
panic handler and the HardFault exception clear `TIM1 BDTR.MOE` first (raw
PAC write, no peripheral ownership needed; F405 additionally drops the
DRV8301 EN_GATE pin), then report over defmt and halt. Standalone, the halt
ends in an IWDG reset; under a debugger UDF / vector catch halts the core
for inspection.

### Layer 4 — IWDG (implemented; G474 pending)

- Armed right after `foc::init` (the ISR is the sole feeder, so never
  earlier), petted at the end of every ADC ISR cycle via a raw `IWDG.KR`
  write.
- Timeout must outlive the longest CPU stall with no ISR running — an
  internal-flash erase stalls the chip since code executes from the same
  flash: **3 s on F405** (the
  storage region uses the 128 KB sectors 10-11: erase is 1 s typical, up
  to 2 s at corners). Config writes are additionally blocked while
  the motor runs (Busy gate + `FLASH_OP_PENDING` TOCTOU guard), so a stall
  with the motor energized cannot happen by design; the IWDG margin is the
  backstop.
- `DBGMCU` freezes the IWDG while the core is halted, so breakpoints don't
  reset the chip.
- After an IWDG reset the PWM peripheral comes up disabled (outputs
  high-Z). See *Boot-time recovery* — a watchdog reset while moving is a
  specific, dangerous case.
- **G474**: not armed — the FOC ISR (its feeder) is dormant until the
  IHM08M1 shield is connected. The panic hooks are in place.

## Boot-time recovery (idea / planned)

A reset (especially an IWDG reset) can happen while the board is **moving at
speed**. On reset the PWM outputs go safe (high-Z), so the motor is
**freewheeling** — at speed that means no braking and no control. Booting back
into a passive `Stopped`/idle state may not be the safe outcome.

On boot:

1. **Read the reset reason** (STM32 RCC CSR flags: IWDG / WWDG / software / pin
   / power-on / BOR; read early, then clear with RMVF). Distinguish a clean
   power-on from a watchdog/fault reset.
2. **Detect whether the motor is spinning.** With outputs off there is no phase
   current, but there is **back-EMF voltage** and the position sensor
   (Hall/encoder) shows changing angle. Estimate speed/angle from sensor delta
   and/or back-EMF magnitude.
3. **If spinning after a watchdog/fault reset → controlled "flying restart"**,
   not a blind re-energize.

> ⚠️ **Hazard — flying restart.** Re-energizing a spinning motor without first
> synchronizing to its actual rotor angle/speed produces a large current/torque
> transient (commanded voltage out of phase with back-EMF). Recovery **must**:
> (a) measure real speed/angle first (sensor, or let the observer converge on
> back-EMF), (b) align controller state (angle, flux, Id/Iq setpoints) to the
> present operating point, (c) only then take over smoothly. This is the
> standard "catch-on-the-fly" / "flying start" procedure (cf. VESC, industrial
> drives).

> ⚠️ **Do not auto-resume throttle.** Automatically re-entering the last active
> throttle mode after a watchdog reset is dangerous — the fault may recur and
> the rider isn't expecting torque. Safer default: catch the spinning motor into
> a **controlled neutral** (synchronized, zero torque / configurable controlled
> regen), and require explicit rider/host re-engagement to resume drive.
> Make the post-watchdog action configurable.

Note on persistence: an IWDG reset is unplanned, so you cannot save intent at
reset time. Recovery should be driven by **observed physical state on boot**
(is it spinning?), not by trying to restore a saved command.

Status 2026-06-11: **hall-based flying restart already mostly works** — the
hall estimator runs every ISR cycle regardless of motor state (angle and
velocity are fresh when a start command arrives), and the dq-decoupling
feedforward applies `vq = ω·(Ld·id + λ)` from the very first cycle, so the
PI loops start near the operating point instead of from zero. What is
missing is the **sensorless** case (MESC's `MOTOR_STATE_TRACKING`): with
the gates off there is no current to observe, so it requires the phase
BEMF voltage dividers (VESC-class boards have them) brought up as ADC
channels + a tracking mode that feeds measured v_αβ to the observer while
undriven. Bench-blocked; see TODO.

## Command delivery & idempotency

ergot is **at-most-once** (now-or-never, no built-in ARQ; no QoS/priority —
telemetry and command responses share one bounded outgoing queue per
interface). Reliability is the application's job:

- **At-least-once** via host-side `timeout + bounded retry` → requires
  **idempotent** commands (the receiver must tolerate duplicates).
- Prefer **declarative / level-triggered** commands (absolute setpoints:
  `MotorRequest::SetMode`, absolute config) — idempotent by construction, retry-safe, and
  they fail safe naturally (absence of fresh setpoint → deadman → safe state).
- For the few genuine **actions**: either no-op-if-already-running (e.g.
  `DetectEndpoint` returns `Busy` during a run) or dedup by an app-level
  `req_id` (effectively-once). ergot's `seq_no` does **not** help dedup (a retry
  gets a fresh seq).
- Encode retry policy in the **type** (a `Idempotent` marker trait + `call` /
  `call_once` host helpers) so it can't be misapplied and there's no per-call
  boilerplate. `MotorEndpoint`/`ConfigEndpoint` → `Idempotent`; `DetectEndpoint`
  → not.

`MotorEndpoint` adds application ordering above ergot: every request carries a
fresh process `source_session` and wrapping `seq`. The device accepts active
setpoints only from the current source and rejects duplicates/stale sequences
before they can refresh the ISR deadman. This is intentionally a narrow drive
guard, not a broad lease: monitoring, reads, and live-safe configuration remain
available from another client. `Stopped`, `Coast`, `Brake`, and
`EmergencyStop` always supersede the active source.

The response is emitted after the ISR gate and carries an explicit outcome; it
never reports a pre-application status snapshot as acknowledgement.
`EmergencyStop` reuses the motor endpoint (no extra socket cost), immediately
floats the bridge, and latches re-arm until a subsequent safe neutral command.

Safety corollary: even a universal stop command can be lost. The deadman remains
the backstop (absence of fresh accepted affirmation → safe); emergency stop
adds an immediate best-effort path, not a replacement for that backstop.

## Non-idempotent endpoints (track these)

- **`DetectEndpoint`** — motor characterization is an action, **not idempotent**
  across runs. Handled: the detect server dedups by `req_id` (replayed request
  returns the cached response, payload verified), and requests are served one
  at a time by the single server loop.

## Open questions / TODO

Done: Layers 1–4 (link gate, ISR command-staleness deadman + configurable
failsafe, panic/HardFault gate kill, IWDG from the FOC ISR).

The remaining actionable items (bench-tune the regen-brake, G474 IWDG,
flying-restart, post-watchdog policy, idempotency helpers, integrating fault
detector) live in the single backlog — [TODO.md](TODO.md#safety). This file
keeps the failsafe *design*; what's still to build is tracked there.

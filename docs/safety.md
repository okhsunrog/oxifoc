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
| 1. Link gate | Host disconnected / link silent | ergot liveness → `state_notify` → `state_monitor` clears `link_active` → FOC forces `Stopped` | liveness timeout (3–5 s today) | **[done]** |
| 2. ISR command-staleness deadman | Host alive but not commanding; **async executor hung** | Stamp `last_cmd_tick` in ISR when draining `CMD_CHANNEL`; if `now - last_cmd_tick > thr` → configurable failsafe mode | ~ multiple of command period (e.g. 60–100 ms) | **[planned]** |
| 3. Independent watchdog (IWDG) | **FOC ISR itself stopped** (hardfault, clock fault, priority lock) | IWDG petted from the FOC ISR; if even the ISR stops → MCU reset → PWM goes high-Z/off | IWDG period | **[planned]** |

Key insight: Layer 1 is **async-executor-dependent** (liveness runs in the RX
worker; `link_active` is cleared by the async `state_monitor`). If the executor
hangs, Layer 1 does **not** fire. Layer 2 (ISR-resident) survives executor
starvation and **subsumes Layer 1's coverage** — once Layer 2 exists, the
Layer 1 gate can be removed or folded in as one input. Layer 3 is the hardware
backstop below both.

### Layer 1 — link gate (implemented)

Chain: ergot liveness timeout → interface → `Down`/`Inactive` → `state_notify`
wakes `state_monitor` (`oxifoc-g474/src/protocol/servers.rs`) → on `!any_active`
calls `MotorControlState::set_link_inactive()` → `process_commands`
(`oxifoc-core/src/state.rs`, runs in the FOC ISR) forces `ControlMode::Stopped`
while `!link_active`. After reconnect the host must send a fresh command to run
again.

Caveat (today): liveness timeout is 3–5 s, so up to several seconds of
uncommanded running after a disconnect. Shorten the control-link liveness, or
rely on Layer 2's tighter threshold once it lands.

### Layer 2 — ISR command-staleness deadman (planned)

- Stamp `last_cmd_tick = now` inside the ISR `CMD_CHANNEL` drain (no
  cross-context atomic needed — the check is ISR-local).
- Threshold = a small multiple of the host's command period, **not** the
  liveness timeout. This is the fast safety reaction.
- The failsafe must be a **self-contained control mode** the FOC runs every
  cycle without further input (ramp current to zero / configurable regen brake
  to standstill), not a one-shot `Stopped` command.
- Behaviour configurable (coast vs smooth brake vs hold).

### Layer 3 — IWDG (planned)

- Pet the IWDG from the FOC ISR (the most reliable context).
- On expiry the MCU resets; ensure the reset state leaves PWM safe (outputs
  off / high-Z). See *Boot-time recovery* — a watchdog reset while moving is a
  specific, dangerous case.

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

## Command delivery & idempotency

ergot is **at-most-once** (now-or-never, no built-in ARQ; no QoS/priority —
telemetry and command responses share one bounded outgoing queue per
interface). Reliability is the application's job:

- **At-least-once** via host-side `timeout + bounded retry` → requires
  **idempotent** commands (the receiver must tolerate duplicates).
- Prefer **declarative / level-triggered** commands (absolute setpoints:
  `ControlMode`, absolute config) — idempotent by construction, retry-safe, and
  they fail safe naturally (absence of fresh setpoint → deadman → safe state).
- For the few genuine **actions**: either no-op-if-already-running (e.g.
  `DetectEndpoint` returns `Busy` during a run) or dedup by an app-level
  `req_id` (effectively-once). ergot's `seq_no` does **not** help dedup (a retry
  gets a fresh seq).
- Encode retry policy in the **type** (a `Idempotent` marker trait + `call` /
  `call_once` host helpers) so it can't be misapplied and there's no per-call
  boilerplate. `MotorEndpoint`/`ConfigEndpoint` → `Idempotent`; `DetectEndpoint`
  → not.

Safety corollary: a "stop" command is **not** a safety guarantee (it can be
lost). Safety is the deadman (absence of affirmation → safe), not the delivery
of stop.

## Non-idempotent endpoints (track these)

- **`DetectEndpoint`** — motor characterization is an action, **not idempotent**
  across runs. TODO: dedup by `req_id`, and guarantee no-op / `Busy` while a
  detection is already in progress.

## Open questions / TODO

- [ ] Layer 2: ISR command-staleness deadman + configurable failsafe mode.
- [ ] Layer 3: IWDG petted from FOC ISR; verify reset → PWM safe.
- [ ] Boot: reset-reason read + spinning-motor detection + flying-restart sync.
- [ ] Configurable post-watchdog policy (controlled coast / regen / hold).
- [ ] Once Layer 2 exists, fold/remove the Layer 1 `link_active` gate.
- [ ] `Idempotent` marker trait + `call`/`call_once` helpers in host-lib.
- [ ] Shorten control-link liveness timeout (interim, until Layer 2).

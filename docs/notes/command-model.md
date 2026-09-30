# Command model — rider controls, vehicle logic, motor commands

> **STATUS: RFC (open), 2026-09-30.** Nothing implemented. Builds on
> [remote-design.md](remote-design.md) § "Drive architecture" (the remote
> sends vehicle intent as a topic; the intent→motor mapping runs on every
> controller). Markers: **[decided]** · **[proposed]** · **[open]**
> (under discussion) · **[finding]** (a defect in today's code).

Three layers, each with its own vocabulary:

```text
rider  ── trigger, buttons ──▶ remote ── DriveIntent (50 Hz topic) ──▶
vehicle logic (per controller) ── ControlMode ──▶ FocDriver
host/bench tools ────────────── ControlMode ──────────▲
```

## 1. Reference: VESC and MESC (bldc c835e9f1, vesc_tool 59768655)

### VESC motor layer (`mc_control_mode`, datatypes.h:181)

| Mode | What it does | Near zero speed | Reverses? |
|---|---|---|---|
| `CURRENT` | iq = I, signed | nothing special | **yes** — negative I at standstill is full reverse drive (mcpwm_foc.c:3641-3659) |
| `CURRENT_BRAKE` | iq = −sign(ω)·\|I\| (3435-3437) | zero vector (all phases shorted) when the speed or vq sign flips, or duty ≈ 0, while the short current is below the target (3330-3350) | **never**; at standstill a passive short — never holds either |
| `DUTY` | duty setpoint, downramp PI regen-brakes (3387-3422) | duty 0 = zero vector; never auto-released | yes (signed duty) |
| `SPEED` | PID on rpm, braking allowed by default (`s_pid_allow_braking`) | below `s_pid_min_erpm` (900) → duty 0 = short (3355-3360) | yes |
| `POS` | PID on encoder angle, bidirectional | — | yes |
| `HANDBRAKE` | fixed open-loop stator vector at phase 0 (3587-3589) | locks the rotor like a stepper | — |

- **Release** = PWM off (true coast, observer keeps running on measured
  back-EMF, 3669-3704) once \|iq\|, \|id\| < `cc_min_current` (0.05 A) and
  the off-delay (0.1 s for the `_rel` setters) expired (3935-3960).
- Speed limits (ERPM rolloff) scale **only motoring** current; braking
  bypasses them (mc_interface.c:2523) — same as our derating.
- **Timeout** (timeout.c:225-239, every mode): after `timeout_msec`
  (1000 ms) → `set_brake_current(timeout_brake_current)`, default 0 A =
  coast.

### VESC input apps (app_ppm.c, app_adc.c, app_nunchuk.c)

- Pipeline: deadband 0.15 → throttle curve (separate +/−) → ramp
  (0.3–0.4 s up, 0.1–0.2 s down).
- **Neutral** = `set_current(0)` → released → coast.
- **Brake**: the `*_NOREV_BRAKE` / `*_BRAKE_*` types call
  `set_brake_current`: "brakes until the motor stops, but not further."
  The setup wizard defaults PPM to `CURRENT_NOREV_BRAKE`. The plain
  `CURRENT` / nunchuk `BIDIRECTIONAL` types brake by negative torque and
  roll straight into reverse at 0 rpm.
- **Reverse**, four variants:
  1. through zero — negative torque (above);
  2. a button/toggle (ADC `REV_BUTTON`, nunchuk Z), gated only by
     \|current\| < 0.2·max (app_nunchuk.c:253-268); remotes that know
     their state report it (`rev_has_state`) instead of edges;
  3. double brake (PPM `CURRENT_BRAKE_REV_HYST`): brake, back to neutral
     below 4000 erpm, brake again → reverse (app_ppm.c:231-291);
  4. smart reverse: brake held past −0.92 while \|duty\| < 0.0075 →
     reverse in duty mode, ≤ 7 % duty ramped over 3 s (app_ppm.c:432-467;
     nunchuk default ON).
- **Cruise**: ADC — hold the button at neutral → latch rpm, any input or
  release → off (app_adc.c:514-542); nunchuk — hold C → latch, the stick
  trims the target at 3000 erpm/s, never across 0, bumpless (296-340).
- **Safe start**: input must rest at neutral (ADC 500 ms) before driving
  after boot/timeout/fault.
- No app uses `HANDBRAKE` or holds position.

### MESC

- Iq is signed everywhere: the ADC2 "brake" is a negative summand and
  **reverses at standstill**; the REVERSE GPIO negates everything,
  brake included (MESCApp.c:85-125).
- Zero request → TRACKING (PWM off, observer on phase voltages), with a
  0.1/0.2 A hysteresis.
- `HANDBRAKE` → SLAMBRAKE: the zero-vector short with a per-phase
  overcurrent tri-state; with an incremental encoder a position hold
  (short inside a deadband, P + `park_current` beyond, MESCfoc.c:622-680).
- Safe start at boot only; no UART/CAN request timeout.

### Lessons

1. **Brake and reverse are different verbs at the motor layer.** VESC's
   `CURRENT_BRAKE` cannot reverse by construction; MESC's signed brake
   can, and every app has to paper over it.
2. **Neutral is a release** (bridge off, estimator running), not "regulate
   0 A".
3. Every reverse gesture trades convenience for the risk of a surprise
   reverse. Through-zero and smart reverse can start reversing while
   the rider is braking on a hill. The safe ones are explicit and gated
   at standstill.
4. **Nobody holds position on halls.** VESC's handbrake is the undamped
   fixed vector (our sim: it rings at ~1.5 Hz and, when the slope
   reverses, slips and runs away); MESC's hold needs an encoder.
5. VESC's timeout covers **every** mode, coast included.

## 2. Findings in today's code

- **[finding] Coast is exempt from both link-loss paths at any speed**
  (`deadman_expired_with`, the link gate in `process_commands_inner`).
  Fine for a bench spin-down; for a rider whose released throttle is a
  coast, link loss downhill = the board free-wheels with nobody in
  control. Must be closed before the remote rides.
- **[finding] Safe modes end drive ownership** (`DriveOwnership::
  record_applied`): while the rider brakes (if braking is `Brake`) or
  coasts, any other source's drive command is accepted.
- **[finding] `CurrentControl` with negative iq at ω = 0 is drive** (the
  grant counts standstill as motoring) — a brake mapped to negative iq
  reverses, the MESC foot-gun.
- `Brake` is the parking brake only: fixed `max_brake_a`, and a `Brake`
  at speed is silently substituted with a controlled stop
  (`enter_brake_ramp`) — a leftover from the plain-short era; the hybrid
  brake handles speed by itself now (sim: entered at 1500 el rad/s,
  peak < 25 A).

## 3. Motor commands (`ControlMode`, ICD v5) — [proposed]

Every mode belongs to a **class**, and the gates derive from the class
(today the list `Stopped | Coast | Brake` is repeated in ~10 places):

| Class | Modes | Accelerates? | Entry gate |
|---|---|---|---|
| idle | `Disarm` (today `Stopped`: bridge off, allowed with faults), `Coast` (bridge off, armed) | no | — |
| brake | `Brake { current }`, `Hold` | **never** | `Hold`: standstill only |
| drive | `Torque { iq, id }`, `Velocity { rad_s }`, `Position { rad }` | yes | faults, latch, ownership |
| bench | `OpenLoop`, `DirectVoltage` | yes | detection/bench only |

- **`Brake { current }`** = VESC `CURRENT_BRAKE` on our hybrid park-brake
  machinery: the windings short while the short current stays within
  `current`, current-regulated at `current` against the motion above,
  zero at standstill, never reverses. `current` is clamped to
  `max_brake_a` and slewed (`slew_a_per_s`), with dissipation on a full
  pack. The rider's brake trigger, the parking brake (`current = max`)
  and the failsafe stop (the same brake, slewed, with a terminal) become
  one mechanism. `enter_brake_ramp` goes away.
- **`Hold`** = the anchored brake (sim, 2026-09-30: MESC-style — short
  inside a deadband around the captured position, `base + kp·excess −
  kd·ω̂` beyond; ±7 mm of board travel on halls vs a free creep with the
  plain short). Entry only at standstill (else `RejectedSafety`), needs a
  trustworthy angle, current budget with a fall-back to `Brake { max }`.
- **`Torque`** stays signed (bench, and the vehicle logic's drive):
  crossing zero is a legitimate motor-level operation. Direction safety
  lives in the vehicle logic (§5), not here.
- `Position` travels by the cascade (position P → velocity loop) and
  parks at the target through the same anchor as `Hold`.

### Liveness: a session deadman, not a mode list — [proposed]

The deadman exists to stop a **moving** board whose controller has lost
its rider/host. So it follows the **drive session**, not the mode:

- a session opens with the first drive/brake/coast command from a source
  and stays open while the board moves under it — through coast and
  brake alike (this also keeps its **ownership**: no other source takes
  over while the rider brakes);
- a stale session at speed → failsafe (ControlledStop → terminal), in
  any mode, coast included — VESC's rule;
- the session closes (and the deadman disarms) only at standstill in a
  standing mode: `Disarm`, `Coast` or `Brake` with the board at rest,
  `Hold`;
- bench modes keep the long deadman; device-local sequences (detection)
  run without a session.

**[decided]** The failsafe terminal is configurable, as today
(`FailsafeTerminal`), extended with `Hold`: `Coast` / `Brake { max }` /
`Hold` (falls back to `Brake` without a trustworthy angle or past its
current budget).

## 4. Rider controls — [proposed]

Hardware from remote-design.md: one spring-return signed trigger
(`drive ∈ [−1, 1]`: forward = drive, back = brake, released = 0), a few
buttons, a vibration motor, an optional display.

**[proposed] The intent carries typed actions, not buttons.** The remote
will have 2–3 buttons, long presses, combinations, a display and its own
menu; which physical input triggers which action is the **remote's**
business (customizable there). The controller only ever sees semantic
actions and validates them against the vehicle state.

**[proposed] Delivery: the latest action with its sequence number, in
every frame.** `DriveIntent` is a latest-value-wins stream over a lossy
link: a level ("hold wanted") survives loss but turns a refused request
into a latent one (a hold refused at speed would fire later on its own at
the next stop — the auto-hold the rider does not want). An action
repeated in every frame with a wrapping `action_seq` survives loss **and**
executes once, when the seq changes, evaluated against the state at that
moment. `DriveStatus` echoes the seq and the verdict; the remote shows it
(haptic / display) on a refusal.

```text
DriveIntent { session, seq, source, drive: f32, profile: u8,
              action_seq: u8, action: Action }
Action = None | Lock | CruiseToggle | CruiseAdjust(erpm) | HoldToggle
       | Reverse(on: bool) | …            // append-only, grows with the remote
DriveStatus { …, state: Standing|Riding|Cruising|Holding|Reversing|Stopping|Parked|Fault,
              direction, profile, action_seq, verdict: Accepted|Refused(reason) }
```

`Lock` is the context action — "keep what I have now": rolling above the
cruise minimum → cruise (latch the speed); standing → hold (latch the
position); engaged → off. A remote may map it to one button, or map
`CruiseToggle` / `HoldToggle` to two.

**[open] Reverse — wanted, the gesture is under discussion.** Context:
on VESC the rider uses smart reverse and finds it convenient everywhere
except where the board must stand still (a traffic light) — which VESC
had no `Hold` for. Candidate: smart reverse — at standstill, with the
brake trigger held past a threshold (VESC: −0.92) and **not in Hold**,
the board creeps backward at a speed proportional to the brake depth,
capped (`Velocity` rather than VESC's 7 % duty — the loop holds the cap
on a slope too), ramped in; releasing the trigger stops it (→ `Brake`).
Variants to weigh: a dwell (brake held at standstill for N ms before the
creep), an explicit `Reverse` action instead of / next to the gesture;
through-zero and double-brake reverse look worst (a braking rider can
end up reversing).

**[open]** Profiles (eco/normal/sport scale the drive current, the accel
ramp and the speed ceiling; braking is never scaled down) — `profile` in
every frame (a level is right here: it is a setting, not an event).

### What the trigger does, per vehicle state

| State | Trigger forward | Released | Trigger back |
|---|---|---|---|
| Standing | pull away → Riding | `Coast` (session closes) | `Brake` (a short at rest); past the threshold → smart reverse |
| Riding | `Torque` = curve(drive)·profile | `Coast` (live session) | `Brake { \|drive\|·max_brake }` |
| Reversing | — (→ `Brake`, then Riding forward) | `Brake` → Standing | creep back: `Velocity { −cap·depth }` |
| Cruising | trim the target up at `drive·trim_rate` (VESC: 3000 erpm/s) | hold the latched speed (`Velocity`) | cancel cruise → `Brake` |
| Holding | above the hold current → release to `Torque` (hill start, bumpless) | `Hold` | `Hold` (no smart reverse) |
| Stopping (failsafe) | ignored until neutral | — | rider's brake kept if stronger (today's takeover) |
| Parked / latched | ignored until neutral | re-arm (the neutral acknowledges) | re-arm |

Cruise ends on the brake trigger, a `Lock`/`CruiseToggle` action, a
fault, or speed dropping below the cruise minimum (to `Coast`, not a
short — VESC's speed PID shorts below 900 erpm). Trimming down is the
brake's job (it cancels); a remote with a display may also send
`CruiseAdjust(±erpm)` steps.

## 5. Vehicle logic — [proposed]

A pure function per controller (replicated, remote-design.md Axis 3):

```rust
fn step(state: &mut VehicleState, intent: &DriveIntent, motor: &MotorStatus,
        cfg: &VehicleConfig) -> ControlMode
```

It owns direction (the smart-reverse state and the motor-direction
inversion), profiles, the throttle curve and ramps, the cruise/hold
latches, the standstill gates and the action-seq bookkeeping. Braking is
always `Brake`; backward motion only ever comes from the explicit
reverse state, never from a brake crossing zero. Being pure, it is
testable exhaustively without a plant: "`Lock` at speed engages cruise,
never hold", "a brake at speed never reverses", "no smart reverse while
holding", "a refused action is not latent", "a pull-away releases a hold
without a torque gap".

## 6. Testing

1. **Unit** — the vehicle logic's state machine (above).
2. **Sim, Rust (CI)** — motor-level physics: brake never reverses, hold on
   a slope and on a slope reversal, park creep, failsafe takeover of the
   rider's brake, coast-at-speed link loss.
3. **`sim_trace` + `simplot.py`** — investigation (`mode=pos|lock|anchor`,
   `j=`, `slope=`, `slopestep=` exist since 2026-09-30).
4. **Maneuvers** (virtual and hardware, A/B) — the whole chain through
   the ICD. New steps **[proposed]**:
   - `intent { drive, action }` — the host publishes `DriveIntent` like
     the remote: tests the vehicle logic on the device;
   - `link_drop { ms }` — stop sending (affirms included): the real
     deadman → failsafe → terminal chain, reproducibly (the TODO's
     deadman drill);
   - `env { slope, load }` — virtual only (rejected on hardware);
   - the new motor commands (`brake { current }`, `hold {}`,
     `position { rad }`).

   Pass criteria stay in `benchsuite.py` (drift, peak current, final
   state, command outcomes). Scenarios: hold on a slope and on a slope
   reversal; park creep; `Lock` at speed → cruise, trim, brake cancels;
   `Lock` standing → hold; pull-away from hold; smart reverse (and none
   while holding); link drop while riding, coasting, braking, holding,
   reversing — each failsafe terminal.

## 7. Order of work

1. Hall velocity spike at low-speed reversals (sim: ±50–140 el rad/s
   with the rotor at ~0) — everything regulating near zero depends on it.
2. Motor layer: classes, `Brake { current }`, `Hold`, `Position` =
   cascade + anchor, session deadman + ownership (closes both findings).
3. Vehicle logic + `DriveIntent`/`DriveStatus` (ICD v5, with the topics).
4. Maneuver steps and the scenario suite.

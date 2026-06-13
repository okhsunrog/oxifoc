# Sensorless Startup & Current Sampling — Ideas to Borrow from VESC & MESC

> **STATUS.** Part 1 (V0_V7) is a deliberate scope decision — no code changes
> without bench evidence. **Part 2 is IMPLEMENTED (2026-06-13)** as the
> `phase/startup.rs` state machine: align→ramp→handoff cold start with a
> current-scheduled ceiling (Phase A) + deadshort flying restart (Phase B),
> both current-only / g431-capable, host-test covered. See
> [decisions.md](../decisions.md → Firmware/platform). Remaining: bench
> validation + the v1 refinements in TODO.md «Sensorless startup». The Part 2
> analysis below is kept as the design rationale; the "Proposed work" maps to
> what shipped.

Working notes from a line-by-line comparison of the current-sampling path and
the sensorless-startup path against **VESC** (`bldc`) and **MESC**
(`MESC_Firmware`). Companion to [hall-improvements.md](hall-improvements.md).
Status legend: **[gap]** missing capability worth adding · **[bench]** verify
on hardware · **[scope]** deliberate narrowing, not a defect · **[idea]** worth
evaluating.

Reference points for "us":
- `oxifoc-g431/src/hardware.rs`, `oxifoc-g431/src/motor.rs` — ADC↔PWM timing
- `oxifoc-core/src/foc/current_sense.rs`, `current_reconstruction.rs`
- `oxifoc-core/src/foc/phase/manager.rs` — open-loop override / source selection
- `oxifoc-core/src/types.rs` — `ControlMode` (the building blocks)

---

## Part 1 — Current sampling

### What we do (g431)

Center-aligned complementary PWM (`motor.rs:63`,
`CenterAlignedBothInterrupts`). ADC trigger is the internal CH4 compare →
`TIM1_TRGO2` (`motor.rs:86`, `Mms2::COMPARE_OC4`), firing at **CNT=ARR = V0**
(all low-side FETs on — `motor.rs:80`). Both ADCs injected, fired together,
phase A at 6.5 ADC cycles (~0.15 µs), all phase currents within ~0.45 µs of the
trigger (`hardware.rs:169`). **One sample per period at V0.** Duty capped at 95%
(`pwm.rs:48`, also helps bootstrap charging). `current_reconstruction.rs` exists
(reconstruct the highest-duty phase via ia+ib+ic=0) but is **disabled on g431** —
the bias network gives bidirectional sensing, so no negative-clipping to fix.

### What the references do

- **VESC** — three sample modes (`datatypes.h:72`): `V0`, **`V0_V7`** (sample at
  *both* null vectors and average → 2× rate, better switching-ripple rejection,
  and the V7 sample is what HFI demodulates), `V0_V7_INTERPOL` (interpolate the
  phase between them — `mcpwm_foc.c:2936`). Supports 1/2/3 shunt + phase shunts,
  sorts phases by duty to pick which shunt to trust, reconstructs the worst one.
- **MESC** — `fastLoop` triggered at the PWM counter top (V0), dual-interrupt
  routine (`MESCfoc.c:427`).

### Assessment

Single-V0 sampling at the period center IS the standard "period-average current"
point; at the 95% cap the worst phase's low-side window (~2.5 µs at 20 kHz) still
dwarfs the ~0.6 µs sample aperture. So this is **[scope]**, not a bug: correct
for 3-shunt + low-side bias + duty cap. We can't run the 1–2-shunt /
high-overmodulation boards VESC can — a deliberate narrowing.

What we give up vs VESC, worth knowing:

1. **[bench] V0_V7 ripple averaging.** One sample/period means more
   switching-ripple aliasing into the current estimate at high modulation
   (80–95%), and the highest-duty phase is read directly (reconstruction off)
   with the shortest window. Measure current quality at ~90% modulation under
   load before trusting it.
2. **[bench] HFI on a single V0 sample.** VESC/MESC demodulate HFI at the V7
   vector; we demodulate in the FOC ISR off the one V0 sample. Sim says fine,
   but sim is an ideal plant — confirm SNR of the HFI demod on hardware.

No code changes recommended here unless (1)/(2) show problems on the bench. If
they do, the lever is adding a V7 sample (second injected trigger at CNT=0) and
averaging — non-trivial because it changes the ISR cadence.

---

## Part 2 — Sensorless startup (the real gap)

### Where we stand

Our zero-speed story is **HFI-centric** (`HfiToObserver`), and that's the right
bet — HFI gives true position at standstill, no blind open-loop ramp. For motors
*with* HFI configured, startup is solid.

The gap is the **pure back-EMF `Observer` path (no HFI)**. The only automatic
open-loop activation is in `try_observer_fallback` (`manager.rs:571`):

```rust
self.activate_open_loop_override(self.output.angle, dir * DEFAULT_OPENLOOP_MIN_VEL); // 52 rad/s
```

Problems:
- **Fixed single velocity**, not a ramp: `DEFAULT_OPENLOOP_MIN_VEL = 52 rad/s`
  (≈500 eRPM, `manager.rs:73`). `OpenLoopOverride.velocity` is set to a constant;
  `timer` just counts down `DEFAULT_OPENLOOP_TIME = 0.5 s`.
- **Repurposed Hall-failure recovery**, not a designed cold-start (docstring
  `manager.rs:53`: "When Hall fails and observer isn't ready…").
- **No alignment phase** — no d-axis energize at a known angle to pull the rotor
  to a known position before ramping.
- **No current scheduling** (boost/ramp).

Failure scenario to expect on the bench: select `PhaseSource::Observer` on a
no-HFI motor, command torque from standstill → rotor at unknown angle
(`output.angle = 0` at boot), the override jumps to 52 rad/s instantly from the
guessed angle with no alignment → jerk / cogging / possible reverse kick /
failed sync.

### What both references do (exactly to avoid this)

- **VESC** — full I/f state machine (`mcpwm_foc.c:4014`): `t_lock` (align:
  `openloop_rpm = 0`, just hold) → `t_ramp` (speed 0 → max) → `t_const`;
  `openloop_rpm_max` scales with current (`utils_map(openloop_current, …)`),
  has `boost_q`, handoff hysteresis (`m_min_rpm_hyst_timer`). Plus **flying
  restart**: estimate angle/speed of an already-spinning motor before applying
  torque.
- **MESC** — explicit `MOTOR_STATE_ALIGN` (`MESCfoc.c:594`) → open-loop
  `openloop_step` ramp (300 Hz tone, `:1384`), and an explicit "start from
  spinning" path that estimates angle and voltages of a freewheeling motor
  (`:1628`).

Neither capability exists here: no alignment, no true 0→speed ramp, no flying
restart.

### Why flying restart matters for THIS product

This is an electric longboard. "Kick-push, then apply throttle while the motor
is already turning" is a primary use case. Without flying restart, applying
torque to a freewheeling motor in a sensored-failed or pure-observer state means
commutating from a stale/guessed angle into a moving rotor — worst case a hard
brake or reverse jerk under the rider. VESC and MESC both treat this as a
first-class state.

### Proposed work

The building blocks already exist — `ControlMode::OpenLoop { angle, current,
velocity, pi_gains }` (`types.rs`) can do alignment AND a velocity ramp; what's
missing is the automatic in-firmware state machine that sequences them.

1. **[gap] Align → ramp → handoff state machine** for sensorless start:
   - **Align**: hold `OpenLoop { angle: θ0, current: I_align, velocity: 0 }` for
     `t_lock` so the rotor latches to θ0.
   - **Ramp**: ramp `velocity` 0 → `ω_handoff` over `t_ramp` (VESC maps the
     ramp ceiling to commanded current — copy that so a gentle command ramps
     gently).
   - **Handoff**: when `observer.is_ready()` AND `|ω| ≥ ω_handoff`, hand over
     (seed the observer from the open-loop angle/velocity, like the existing
     crossover reseed) and deactivate the override.
   - Replace the fixed-52-rad/s nudge in `try_observer_fallback` with this for
     the cold-start case; keep a fast nudge only for the genuine Hall-dropout
     recovery (where an angle history exists).

2. **[gap] Flying restart** (all sensorless modes):
   - Before applying torque from `Stopped`/`Coast`, run the observer (or a brief
     HFI probe) in a measure-only pass; if `|ω|` exceeds a floor and the
     estimate is converged, seed and go straight to closed loop, skipping
     align/ramp. If standstill, fall through to align→ramp (or HFI).
   - Mirrors VESC `m_phase_observer_override` bring-up and MESC `:1628`.

3. **[idea] Current-scheduled ramp ceiling** — VESC's
   `openloop_rpm_max = map(openloop_current, …)` (`mcpwm_foc.c:4019`): the
   open-loop top speed scales with commanded current so a small throttle gives a
   slow, safe ramp. Cheap to copy, improves feel.

### Test coverage to add (host, VirtualMotor)

- Cold start in `Observer` mode from standstill at a random rotor angle → assert
  the rotor reaches commanded direction without a reverse excursion beyond X°.
- Flying restart: spin the `VirtualMotor` to ω, set firmware to `Stopped`, then
  command torque → assert it catches the rotor (no hard brake, no reverse).
  *Caveat:* this only proves the logic against an ideal plant — see
  [virtual-motor-fidelity.md] for what the sim does NOT model (cogging,
  saturation curve, sensor noise) and therefore cannot catch.

---

## Priority order

1. **[bench]** Observe a no-HFI `Observer` cold start and a freewheel-catch on
   the bench — confirm the rough behavior before building the fix.
2. **[gap]** Flying restart — highest product value (kick-push use case),
   smallest state machine.
3. **[gap]** Align→ramp→handoff cold-start state machine.
4. **[idea]** Current-scheduled ramp ceiling.
5. **[bench]** Current quality at ~90% modulation + HFI demod SNR (Part 1) — only
   act if the bench shows a problem.

All Part 2 work is in the platform-agnostic `phase/manager.rs` +
`foc_driver.rs` and is host-testable against `VirtualMotor` (with the fidelity
caveats noted above). Part 1 is hardware-timing and only changes if the bench
demands it.

---

## Reference map

**ours**
- `oxifoc-g431/src/motor.rs:63,80,86` — center-aligned PWM, V0 trigger via OC4→TRGO2
- `oxifoc-g431/src/hardware.rs:169-202` — injected ADC setup, sample times
- `oxifoc-core/src/foc/current_reconstruction.rs` — highest-duty-phase reconstruction (disabled on g431)
- `oxifoc-core/src/foc/phase/manager.rs:53,73,485,571` — `OpenLoopOverride`, `DEFAULT_OPENLOOP_MIN_VEL`, `try_observer_fallback`
- `oxifoc-core/src/types.rs` — `ControlMode::OpenLoop { angle, current, velocity, pi_gains }`

**VESC** (`~/motor_control/bldc`)
- `datatypes.h:72` — `FOC_CONTROL_SAMPLE_MODE_{V0,V0_V7,V0_V7_INTERPOL}`
- `motor/mcpwm_foc.c:2936` — V0_V7 phase interpolation in the ISR
- `motor/mcpwm_foc.c:4014` — open-loop start ramp (t_lock / t_ramp / t_const, boost, hysteresis)
- `motor/mcpwm_foc.c:3483-3536` — `m_phase_observer_override` bring-up / boost_q

**MESC** (`~/motor_control/MESC_Firmware`)
- `MESC_Common/Src/MESCfoc.c:427` — `fastLoop` at PWM counter top (V0)
- `MESC_Common/Src/MESCfoc.c:594` — `MOTOR_STATE_ALIGN`
- `MESC_Common/Src/MESCfoc.c:1384` — `openloop_step` ramp tone
- `MESC_Common/Src/MESCfoc.c:1628` — start-from-spinning angle/voltage estimation

# Reference-firmware comparison (VESC / MESC / ODrive / moteus / MCSDK)

2026-07-07. Four parallel code audits of `~/motor_control/` reference trees
against oxifoc, done at the close of the estimator campaign. This file keeps
the load-bearing findings and the steal-list; the per-claim `file:line`
references below point into the reference trees as checked out that day.

## Where oxifoc sits in the landscape

- Our flux integrator `x += (v−R·i)dt − L·Δi` **is** VESC's default observer
  (`FOC_OBSERVER_MXLEMMING_LAMBDA_COMP`, bldc `foc_math.c:108-138`) and
  MESC's original (`MESCfluxobs.c:127-132`). We are on the field-proven
  formulation, not a fork of it.
- ODrive uses the Ortega/Praly nonlinear flux observer
  (`sensorless_estimator.cpp:61-85`); MCSDK uses a Luenberger current
  observer + PLL (`sto_pll_speed_pos_fdbk.c:264-320`). moteus has **no
  sensorless mode at all** (encoder/servo only).
- VESC/MESC commutate on **raw atan2** of the flux state (VESC adds a fixed
  lead `ω·dt·(0.5+foc_observer_offset)`, `mcpwm_foc.c:3439`; MESC has an
  optional half-step `0.5·PLL_int` advance, `MESCpwm.c:86-92`). Their PLLs
  produce *speed only*. We commutate through PLL → 2nd-order tracker — our
  bench history (freq-led saga) is why; keep in mind the field-proven
  firmwares get away without it.

## Unique to oxifoc (no analog in any of the four)

- **Confirm probe at handoff** (physical rotor re-measurement, [0.5×,2×]
  agreement, fast-seed escape). VESC trusts its seeded openloop ramp
  (`mcpwm_foc.c:4072-4108`, observer state force-seeded at ±45°); MESC's
  deadshort trusts its own angle immediately (`MESCfoc.c:1616-1698`);
  MCSDK checks observer-vs-*forced*-ramp speed band (15/16..17/16, ×2
  consecutive) — never against an independent measurement; ODrive checks
  nothing (handoff = "open-loop velocity reached", `axis.cpp:221-231`).
- **Multi-layer readiness/trust**: accel-lag-compensated phase err +
  Schmitt, e_q validity earned over 2 el revs (sticky, sustained revoke),
  accel-prior envelope, trust gate cutting iq, restart on trust loss.
  Closest competitor: MCSDK's speed-variance gate + bemf-consistency +
  ×3 debounce (`sto_pll_speed_pos_fdbk.c:381-536`). VESC has only
  geometric clamps + a PLL windup cap; MESC has no ready signal at all;
  ODrive has two hard error flags.
- **Two-inductance model** (fundamental Ld/Lq for decoupling vs AC/HF L for
  the estimation chain). All four use a single inductance everywhere.
  MESC has **no dq decoupling FF at all** (relies on the integrator,
  `MESCfoc.c:1082-1084`); the generated MCSDK project ships with the FF
  module NOT instantiated; ODrive/VESC decouple with the same single L
  (VESC from *measured filtered* currents — we use reference currents,
  which avoids the delayed-feedback path).
- **Deadshort settled-current model** `e=−(R+jωL)·i`: MESC's ancestor probe
  uses `−L·di/dt` (neglects R) and its −90° shift "breaks for reverse
  rotation" per the author's own comment (`MESCfoc.c:1649-1652`). Also:
  MESC's rolling start (MOTOR_STATE_TRACKING) requires phase-voltage
  sensors; our probe needs only the low-side shunts.
- **Failsafe depth**: command deadman → ControlledStop → ParkBrake vs
  VESC's single timeout brake current (`timeout.c:231-233`), MESC's
  break-all/SLAMBRAKE, ODrive/moteus disarm/derate.
- Back-calculation anti-windup: shared with MCSDK only (`pid_regulator.c:
  717-732`); VESC hard-truncates integrators (left back-calc commented
  out, `mcpwm_foc.c:4667-4669`), ODrive decays ×0.99, moteus clamps.
- MCSDK on our exact board confirms the OCP reality: COMP taps the shunt
  pad **before** the ×16 PGA, so ST also parks the DAC near the rail and
  has no meaningful HW current trip — same conclusion we reached.

## Gaps (they have it, we don't)

1. **Field weakening** — VESC: duty-triggered map + ramp + the
   `m_current_off_delay=1s` modulation hold that prevents body-diode
   braking on FW exit (`foc_math.c:702-742`) — that hold is the hard-won
   detail. MESC V2: inject −id only when duty saturated
   (`MESCfoc.c:1113-1127`). MCSDK: voltage-magnitude PI module
   (`flux_weakening_ctrl.c:121-200`).
2. **MTPA** — VESC closed form (2 lines, needs ld_lq_diff which we
   measure): `id = (λ − √(λ² + 8(ΔL·iq)²))/(4ΔL)` (`mcpwm_foc.c:3611`).
3. **Duty control mode** (VESC `mcpwm_foc.c:3380-3422`, MESC) — low
   priority for esk8 (riders use current control).
4. **Position control** (all four) — not our goal.

## Steal-list, prioritized for the esk8 (CF2 + hall Flipsky) plan

1. **moteus hall slow-mode + two-stage hall filter**
   (`motor_position.h:961-1211`): below a bandwidth ratio switch from PLL
   to direct inter-sample velocity; on direction change force v=0 and
   snap position; two decay filters pull the angle back when it leads.
   Best-in-class low-speed hall handling → our velocity loop on halls.
2. **VESC graduated battery/wattage limiting** (`mc_interface.c:2436-2489`):
   battery cut, regen-overvoltage cut, watt limits, soft roll-off
   (`l_in_current_map_start`) — riding-from-battery must-have; richer
   than our hard bus caps.
3. **moteus flux braking** (`bldc_servo_control.h:943-957`): dump excess
   regen as d-axis copper loss when vbus rises — resistor-free middle
   tool between "nothing" and our winding short; useful on the PSU bench
   too.
4. **ODrive spinout detection** (`controller.cpp:434-441`): mechanical
   power braking while electrical power positive → fault. Cheap,
   sensor-agnostic commutation-fault watchdog; complements our trust
   gate and covers hall mode.
5. **VESC accel-only thermal derating** (`mc_interface.c:2369-2394`):
   derate acceleration first, preserve braking headroom — vehicle safety.
6. **MESC freewheel integrator preload** (`MESCfoc.c:1607-1608`): while
   tracking/coasting, continuously seed the current-PI integrators with
   measured Vdq → bumpless PWM re-engage. Ready answer to our
   "handoff smoothing / seed Vd Vq" TODO.
7. **MCSDK adaptive sample point** (`r3_2_g4xx_pwm_curr_fdbk.c:1117-1184`):
   shift the ADC sample and flip the trigger edge when duty crowds the
   low-side window — needed when we ride at high modulation.
8. **MCSDK speed-variance gate** (`sto_pll_speed_pos_fdbk.c:419-437`):
   variance of a speed FIFO vs threshold — cheap orthogonal readiness
   signal (we gate on phase error, never on ω̂ variance).
9. **MESC lowest-duty-phase Clarke selection + HighPhase fallback**
   (`MESCfoc.c:823-870`, `MESCpwm.c:138-140`): pick the two
   best-sampling-window phases → current-sense SNR on 3-shunt boards.
10. **moteus PLL bandwidth recipe** (`motor_position.h:686-738`):
    `w_n = f_hz/2.48, kp = 2w_n, ki = w_n²` + auto-cap at source_rate/4 —
    documented Hz→gains mapping for velocity/PLL tuning.
11. MTPA closed form (VESC) — near-free, needs our measured l_delta.
12. VESC hall details we lack: interpolation-off floor below
    `foc_hall_interp_erpm` (snap to nearest hall angle — avoids being
    stuck 60° off on reversal) and invalid-hall-code fallback to
    open-loop (`foc_math.c:645-650, 686-688`).

## Non-actions (looked at, rejected or deferred)

- VESC bus-voltage/duty-scheduled observer gain: our integrator has no
  correction gain to schedule; n/a.
- MCSDK FeedForward module: same physics as our decoupling but with baked
  Workbench constants — adopting it would be a downgrade.
- MESC nonlinear two-sided flux centering: author himself flags it as
  gain-sensitive; our one-sided centering + clamp is the safer shape.
- MESC HFI_TYPE_45 and LR observer: interesting, not needed now (HFI is
  off on our non-salient targets; LR obs is self-described WIP).

# VirtualMotor Fidelity — What the Sim Proves, and What It Can't

> **STATUS: частично закрыт 2026-06-12.** Лэндинг первой волны апгрейдов:
> sub-stepping, dead-time distortion, квантование+шум АЦП, Lq-сатурация,
> duty-драйв detection-харнесса (ARR 4250). Уже окупились: sim предсказал
> коллапс DirectVoltage-hold под dead-time на низкоомных моторах ДО стенда
> (фикс: comp в apply_dq + settled hold, decisions.md) и закрепил слепоту
> confidence к saliency-коллапсу. Остаток (one-cycle delay, vbus sag,
> coulomb, гармоники) — в TODO.md «VirtualMotor fidelity». Тезис
> «спаренные конвенции прячут смещения» подтверждён дважды: hall-конвенцией
> сима и нулями идеальной таблицы детекции (lockstep-самоподтверждение).

Working notes on how far to trust the `VirtualMotor` plant model
(`oxifoc-core/src/virtual_motor.rs`) and the ~300 host tests built on it,
from a comparison against VESC's `virtual_motor.c` (the only other simulated
plant among the references — MESC ships none, it's hardware-first). Companion
to [hall-improvements.md](hall-improvements.md) and
[startup-and-sampling.md](startup-and-sampling.md).

Status legend: **[strength]** we already model this · **[ceiling]** neither sim
models it · **[blind-spot]** a compensation in our firmware is validated against
a plant that lacks the very effect it compensates · **[add]** proposed sim
upgrade.

---

## The architectural difference comes first

This is not "two implementations of one idea" — the two sims simulate different
things:

- **VESC `virtual_motor.c`** runs **on the target as hardware-in-the-loop**: it
  writes simulated currents/voltages straight into the `ADC_Value[]` array and
  calls the **real** `mcpwm_foc_adc_int_handler` from the TIM8 ISR
  (`virtual_motor.c:150-154`). So it exercises the **entire real firmware signal
  chain** — ADC scaling, current-offset handling, the real
  `foc_correct_hall`/observer, the real duty computation — on the real MCU at the
  real rate. Only the physical analog is simulated.
- **Our `virtual_motor.rs`** is a pure **host** model used in unit tests. It
  exercises the platform-agnostic core (controller, observer, hall_sensor) but
  **not** the platform ISR, ADC scaling, register access, or real timing.
- **MESC** ships no comparable plant. It has a BIST (built-in self test) of
  math/profiling, not a closed-loop sim. Hardware-first.

Consequence: VESC validates the **full firmware path** against a simple plant;
we validate a **richer plant** against the core only. Our platform layer (ISR,
ADC, timing) is not covered by this sim — `tests/stm32{g431,g474,f405}` cover
parts of it on-target, but those don't run the closed-loop plant.

---

## Our plant model is the RICHER one

The electrical/mechanical equations are identical — dq forward-Euler PMSM with
R, Ld, Lq (reluctance torque), λ, J, pole pairs, load torque
(`virtual_motor.rs:232-297` vs `virtual_motor.c:305-354`). What we add on top:

| Effect | oxifoc | VESC |
|---|---|---|
| Viscous friction (`friction_b`) | ✅ `virtual_motor.rs:262` | ❌ J + load only — an unloaded motor never decelerates |
| Hall simulation + mounting offset | ✅ `hall_state()`, `hall_offset` | ❌ Hall isn't modeled (it's GPIO) |
| **D-axis saturation** (`sat_k`, `ld_eff`) | ✅ `:75,249` — required to test the HFI polarity probe at all | ❌ purely linear |
| Coast / shorted modes | ✅ `step_coast`, `step_shorted` | ❌ |
| Explicit back-EMF output | ✅ `bemf_alpha/beta` | partial |

What VESC has that we don't: current clamp to ±(2048·FAC) — ADC range
saturation (`virtual_motor.c:328`). Minor.

**Net: our plant is objectively more faithful than VESC's** — especially d-axis
saturation (without it HFI is unsimulatable) and Hall+offset.

---

## The idealization ceiling is SHARED — and that's where the risk lives

Neither sim models the following (our docstring `virtual_motor.rs:131` admits
it: "no saturation, no iron losses, no temperature effects"):

1. **[ceiling] Cogging torque** — position-dependent reluctance at zero current.
   Affects low-speed smoothness, HFI, startup.
2. **[ceiling] Non-sinusoidal back-EMF** (5th/7th harmonics) — both assume an
   ideal sinusoidal λ. Real motors (esp. trapezoidal) don't. Hits observer
   angle accuracy.
3. **[ceiling] Dead-time distortion, FET Vds drop, body-diode conduction.**
4. **[ceiling] Current-sensor noise / quantization / offset drift** — our tests
   run on perfectly clean currents.
5. **[ceiling] Bus-voltage dynamics / regen** — cap charging from regen → OV.
6. **[ceiling] Cross-saturation** Lq(iq) — we model only d-axis sat, and only
   for the d-current dynamics (docstring `:68`: "the model stays minimal").
7. **[ceiling] One-PWM-cycle actuation latency** — VESC captures it by running
   the real ISR at the real rate; our host loop applies `v_alpha/v_beta` in the
   **same** step, with no delay.

---

## The sharp finding: compensations validated against a plant that lacks the effect

A direct consequence of the ceiling above, and the thing to keep in mind when
reading test results: several pieces of firmware compensation are "green" in the
sim only because the sim never produces the disturbance they exist to cancel.

- **[blind-spot] Dead-time compensation** (`controller.rs` `apply_dead_time_comp`)
  cancels dead-time distortion the plant **doesn't generate** → the comp runs
  but is never actually exercised in sim.
- **[blind-spot] Phase advance** (`DEFAULT_PHASE_ADVANCE_CYCLES = 1`,
  `foc_driver.rs`) compensates an ADC→actuation latency the plant **doesn't
  have** → not validated.
- **[blind-spot] HFI confidence / readiness thresholds** are tuned and tested on
  **noiseless** currents → on a real shunt the demod SNR may not clear those
  thresholds.
- **[blind-spot] Failsafe OV regen-derate** derates current inside the OV window,
  but there's **no bus dynamics** in the plant → the soft landing into OV is
  unvalidated by sim.
- **[blind-spot] Observer robustness to non-sinusoidal back-EMF** — plant is pure
  sinusoidal → untested.

This is the concrete form of "algorithms proven, system not": the 300 host tests
prove the **core math against an ideal plant**, not behavior under the
non-idealities that half the firmware's compensation code exists to handle.

---

## Proposed sim upgrades — add exactly the non-idealities that activate our own compensation

The plant is already the best of the three; don't rewrite it. Add, as optional
`MotorParams` fields (default off, so existing tests don't break; new tests opt
in one at a time and assert the matching compensation actually does something):

1. **[add] Dead-time distortion** — subtract `t_dt·f_pwm` per phase by current
   sign before integrating. Then `apply_dead_time_comp` is tested on *cancellation*
   instead of running idle. **Highest ROI** — it directly closes the biggest
   blind-spot and is a few lines.
2. **[add] Current-sensor noise + offset** (parameterized) — exercises the HFI /
   observer confidence thresholds and calibration robustness against the thing
   real shunts actually do.
3. **[add] Non-sinusoidal back-EMF** (optional 5th/7th harmonic on λ) — exercises
   observer angle bias on a realistic machine.
4. **[add] One-cycle actuation latency** (buffer `v` by one step) — validates the
   phase-advance compensation.
5. **[add] Bus-voltage dynamics** for regen (cap model) — validates the failsafe
   OV-derate soft landing. Larger scope; do last.

Each upgrade should come with a test that **fails without the compensation and
passes with it** — otherwise the compensation is still effectively untested.

---

## How to read existing sim-backed results (trust map)

| Claim a sim test makes | Trust level | Why |
|---|---|---|
| FOC math / transforms / sign conventions correct | **high** | ideal plant is sufficient — the math is the math |
| Observer/HFI converge & track on an ideal machine | **high** | that's exactly what's modeled |
| Detection (R/Ld/Lq/λ) numerically correct | **high** | validated vs known plant params, теперь и на non-ideal планте (отчёт detection_report, e2e `run_full_detection_nonideal_plant`) |
| HFI polarity probe works | **medium** | needs `sat_k`; modeled, but saturation curve is a simple `1/(1+k·id)` |
| Dead-time comp correct | **medium (2026-06-12)** | плант генерит дисторсию, тест `dead_time_compensation_cancels_plant_distortion` — cancellation, не idle; остаётся simple-sign модель |
| Phase advance correct | **low (untested)** | one-cycle latency всё ещё не моделируется |
| HFI robust under real sensing | **medium (2026-06-12)** | 12-bit квантование + шум закреплены (`hfi_locks_through_quantized_noisy_sensor`); non-sinusoidal bemf всё ещё нет |
| HFI under saliency collapse | **известный провал (закреплён)** | `lq_sat_k` + тест: confidence слеп к коллапсу, угол уезжает молча — TODO saliency-монитор |
| Failsafe OV soft-landing | **low (untested)** | no bus dynamics |
| Low-speed smoothness / startup feel | **low** | no cogging, no latency |

The "low" rows are precisely the bench-validation checklist — they can't be
closed in sim until the corresponding [add] upgrade lands.

---

## Reference map

**ours**
- `oxifoc-core/src/virtual_motor.rs` — `MotorParams` `:38`, `ld_eff()` `:75`,
  `step()` `:232`, friction `:262`, Hall sim `:167`, d-sat `:249`,
  "minimal model" docstring `:131`
- `oxifoc-core/src/foc/controller.rs` — `apply_dead_time_comp` (compensation under test)
- `oxifoc-core/src/motor/foc_driver.rs` — `DEFAULT_PHASE_ADVANCE_CYCLES`

**VESC** (`~/motor_control/bldc`)
- `motor/virtual_motor.c:150` — HIL: writes `ADC_Value[]`, calls the real ISR
- `motor/virtual_motor.c:305-354` — electrical + mechanical model (no friction, linear)
- `motor/virtual_motor.c:328` — ADC-range current clamp

**MESC** (`~/motor_control/MESC_Firmware`)
- No simulated plant (BIST is math/profiling only) — hardware-first.

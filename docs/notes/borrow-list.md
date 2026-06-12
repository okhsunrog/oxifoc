# Borrow-list: ideas from reference projects

What to borrow from VESC / MESC / moteus / ODrive / ST MCSDK (based on the
2026-06 review, see [archive](../archive/review-2026-06.md)). This is a
backlog of ideas, not commitments; implemented items get crossed off with
the commit noted.

Already implemented from this list: VESC-8 (signed open-loop), silent
HFI not needed (CORDIC is cheap), bus current limits (VESC override
matrix, first column, 7a30b5f), hall boundary anchor (VESC midpoint,
9f936bb), 2-point R + duty iteration for inductance (MESC/VESC, dee175d).


### VESC (`bldc`)
1. **Force-seeding the observer at handover:** on openloop→sensorless and HFI-start, force `x1/x2 = λ·cos/sin(θ)` (`mcpwm_foc.c:4014-4108`) — eliminates current spikes at capture. Maps directly onto your dual-slot/blend.
2. **Silent HFI (v4/v5):** injection synchronized with the PWM sequence, angle computed in the interrupt via PI + a double integrator on di/dt (`foc_math.c:744`, `mcpwm_foc.c:4801-4853`).
3. **Integrating OV/UV detector** (tolerates spikes, trips on sustained deviation, `mc_interface.c:1881-1907`) — the answer to your single-sample current fault.
4. **Graduated derating matrix:** linear limit roll-off by T_fet/T_motor/V_bat/ERPM/duty (`update_override_limits`, `mc_interface.c:2225+`).
5. Adaptive R-observer running in the background (temperature compensation of resistance, `mcpwm_foc.c:4133-4146`).
6. V0_V7_INTERPOL: a second inverse Park + SVM at mid-period with an extrapolated angle — smoothness at high eRPM.
7. Reconstructing a saturated shunt from the other two to extend the current range (`mcpwm_foc.c:3068-3083`).
8. Signed open-loop on sensor loss (direction = sign of the last velocity).

### MESC
1. **FW V2 — field weakening on pure saturation feedback:** exponential d-current ramp driven by the voltage vector hitting the circle (`MESCfoc.c:1107-1127`) — no dependence on motor parameters, self-limiting. Best candidate for your first FW.
2. **Deadshort re-capture:** briefly short the phases, compute the angle from V=L·di/dt of the rising current, preload the observer and PI integrators, go straight to RUN (`MESCfoc.c:1616-1698`) — re-capturing a spinning motor without phase-voltage sensing.
3. **Hall-flux startup:** online learning of flux vectors per hall segment, IIR-blended into the observer accumulators at low speed (`MESCfoc.c:450-459`) — smooth zero-speed torque on cheap hall sensors.
4. **Always-on blackbox:** ring-buffer log of Vbus/Iuvw/Vdq/angle in fastLoop, frozen around an error, streamed over CAN (`MESCfoc.c:713-732`) — invaluable for debugging faults on the bench.
5. **Dynamic fault thresholds:** Imax = 1.5× the requested current, Vmax tracks Vbus + 15% (`MESCfoc.c:1295-1312`) — catches regen spikes when running off a PSU.
6. Dead-time compensation by current sign (`MESCpwm.c:157-179`) + a dead-time auto-measurement mode and a built-in double-pulse test for hardware bring-up.
7. SVM-aware shunt-pair selection: Clarke from the two phases with the lowest duty (`MESCfoc.c:823-870`).

### moteus
1. **PendSV-split ISR:** prio-0 does only the critical sampling window, the rest of the loop runs in PendSV (prio 6); encoder interrupts can preempt in between (`bldc_servo.cc:446-469`). Will become relevant with the position loop.
2. **Limit-as-fault-code telemetry:** every limit that clipped a command is reported as a code without stopping — you can always see WHAT was cutting you back.
3. **Back-EMF + R·i feedforward in the current loop** (`bldc_servo_control.h:809-962`) — the very W1 that's missing.
4. **PLL gains derived from a single intuitive "Hz" parameter**, auto-capped at ¼ of the source's actual sampling rate (`motor_position.h:504-516`).
5. 32.32 fixed-point position (once a position loop exists) — resolution doesn't degrade with accumulated travel.
6. Hardware trigger chain TIM→DMA→LPTIM→ADC (works around G4 errata ES0430 §2.7.11) — zero software jitter.
7. GPIO readback of the phase state inside the sampling window as hardware proof that the duty didn't violate the window (`bldc_servo.cc:891-897`).
8. Flux braking: dumping regen into the windings via d-current on a Vbus threshold — no brake resistor needed.

### ODrive
1. **Spinout detection:** cross-checking electrical vs mechanical power (`controller.cpp:434-441`) — cheap protection against lost commutation / broken calibration.
2. **`std::optional` data ports reset every cycle** — structural protection against stale data between loop components (`main.cpp:371-398`).
3. **Swappable `PhaseControlLaw`:** R/L measurement is just another control law with the same arm/disarm/safety machinery (`motor.cpp:18-148`).
4. Phase-delay compensation: Park at `phase + vel·(t_meas−t_ctrl)`, inverse Park at `phase + vel·(t_pwm−t_ctrl)` (`foc.cpp:100,163`).
5. Anticogging calibration with a 3600-bin map (`controller.cpp:79-103`).
6. Documented per-task timing budget (`task_times_`, MEASURE_TIME).

### ST MCSDK (B-G431B-ESC1)
1. **Staged rev-up table + SWITCH_OVER crossfade:** parameterized phases (duration/speed/current) + a virtual speed sensor with a ramped handover of authority to the observer (`revup_ctrl.c`, `mc_tasks_foc.c:633-686`) — maps directly onto your blend-crossover.
2. **Variance-gating of observer reliability** (`STO_PLL_IsVarianceTight`) before trusting the sensorless speed — complements your readiness (flux+PLL+min speed).
3. **MC_DURATION fault:** cheap direct detection of FOC-cycle budget overrun (the loop didn't fit within the PWM period → fault) (`mc_tasks_foc.c:655-658`).
4. Park/inverse-Park delay compensation using velocity in dpp × a factor (fixed-point, `mc_tasks_foc.c:719,740`).

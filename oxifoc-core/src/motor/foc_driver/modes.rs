//! One control cycle for each `ControlMode` arm.

use super::{ControlMode, FocDriver, STARTUP_MIN_DRIVE_A, StepError, VOLTAGE_LIMITED_IQ_MARGIN_A};
use crate::foc::clamp_f32;
use crate::foc::controller::FocOutput;
use crate::foc::fast_math::sqrtf;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::pwm::PhaseState;
use crate::foc::sensors::CurrentSensor;
use crate::foc::transforms::clarke;
use crate::foc::transforms::park;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Parking brake, VESC-style (its `CURRENT_BRAKE` mode shorts the
    /// phases around zero and current-brakes above).
    ///
    /// While the short's current stays within `max_brake_a`, all low sides
    /// conduct: the windings short opposes motion either way with no angle
    /// or speed estimate, draws nothing at standstill, and holds a roll-back
    /// (as speed-proportional drag — not a position hold). When the rotor is
    /// pushed faster (a board rolling down a steep slope) the short would
    /// grow toward λ/L at whatever speed; instead the brake turns
    /// current-controlled at `max_brake_a` against the motion — never let
    /// go — and returns to the short once slow again. Without a trustworthy
    /// angle there is no controlled brake: a short current past the limit
    /// then releases to high-Z (the speed that produced it is unknown, and
    /// shorting on at it could mean ~100 A of uncontrolled current and a
    /// violent stop).
    pub(super) fn step_park_brake(
        &mut self,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let limit = self.current_limits.max_brake_a;
        let omega = self.phase.get().velocity;
        if let Some(iq) = self.park_iq
            && self.phase.angle_trustworthy()
        {
            let target = if omega > 0.0 { -limit } else { limit };
            let step = self.failsafe_cfg.slew_a_per_s * dt;
            let iq = iq + clamp_f32(target - iq, -step, step);
            let out = self.step_current_control(iq, 0.0, dt, now_ticks)?;
            // Back to the short once holding the limit takes a q voltage
            // against the motion (back-EMF below R·I): the short alone now
            // carries less than the limit — the switch point is where the
            // two currents meet, so no step. VESC switches on the same vq
            // sign change.
            self.park_iq = (out.vq * omega > 0.0).then_some(iq);
            return Ok(out);
        }
        self.park_iq = None;

        self.pwm
            .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
        self.controller.reset();

        // The short-circuit current is real and measurable (the low sides
        // conduct continuously, so low-side shunts see the phase currents
        // and the ADC keeps triggering): read it — it decides the brake
        // above, keeps the mode from being a protection blind spot, and
        // gives telemetry/estimators the truth instead of zeros.
        let (ia, ib, ic) = if self.current_sensor.is_calibrated() {
            self.current_sensor.read_currents()
        } else {
            (0.0, 0.0, 0.0)
        };
        let (i_alpha, i_beta) = clarke(ia, ib);
        // αβ magnitude equals the dq magnitude (Park preserves it). The
        // hard trip stays the backstop (a limit left unset).
        if self.current_limits.is_overcurrent(i_alpha, i_beta) {
            self.pwm.disable();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }
        let i_mag = sqrtf(i_alpha * i_alpha + i_beta * i_beta);
        if limit > 0.0 && i_mag > limit {
            if self.phase.angle_trustworthy() {
                // Continue from the short's current, capped, as a brake
                // against the motion (next cycle on).
                let start = i_mag.min(limit);
                self.park_iq = Some(if omega > 0.0 { -start } else { start });
            } else {
                warn!(
                    "park brake: short current {} A past the {} A limit without an angle, releasing",
                    i_mag, limit
                );
                self.pwm.disable();
                self.pwm_off = true;
                self.mode = ControlMode::Stopped;
            }
        }

        // Terminal voltage is zero while shorted; the estimators integrate
        // that with the real circulating currents.
        self.update_phase_with_applied_voltage(0.0, 0.0, i_alpha, i_beta, dt, now_ticks);
        Ok(FocOutput {
            ia,
            ib,
            ic,
            i_alpha,
            i_beta,
            angle_rad: self.phase.get().angle,
            ..Default::default()
        })
    }

    /// Execute one position-control cycle: the position error becomes a
    /// velocity target ([`PositionLoopConfig::speed_for`][speed_for] — P, bounded by
    /// the stopping distance at the velocity loop's ramp and by the travel
    /// ceiling) for the velocity loop, which also supplies the integral
    /// that holds a load. Without a trustworthy angle the velocity step
    /// degrades to zero torque, as in velocity mode.
    ///
    /// [speed_for]: crate::foc::position::PositionLoopConfig::speed_for
    pub(super) fn step_position_control(
        &mut self,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let err = self.position_target.minus(self.position.position());
        let decel = self.velocity_loop.config().accel_limit;
        let omega_target = self.position_cfg.speed_for(err, decel);
        self.step_velocity_control(omega_target, dt, now_ticks)
    }

    /// Execute one velocity-control cycle: the cruise loop turns the ω
    /// target into an iq command (slew-limited reference + clamped PI, see
    /// [`crate::foc::velocity`]), routed through the normal current loop so
    /// the current-limit clamp and overcurrent trip still apply.
    pub(super) fn step_velocity_control(
        &mut self,
        target_vel: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Velocity needs a usable estimate; a source that can't track (e.g.
        // a back-EMF observer below its speed floor) makes the loop
        // integrate garbage. Degrade to zero torque while staying in the
        // mode instead of hard-stopping: a sensorless cruise that dips
        // below the floor coasts, and resumes by itself once motion (push,
        // downhill) brings the observer back into lock — an Err here would
        // kick the driver to Stopped and require a fresh host command.
        // Hall/encoder sources are always trustworthy, so a sensored ride
        // never takes this branch.
        if !self.phase.angle_trustworthy() {
            // Keep the loop bumpless for re-entry: reference parked at the
            // (unreliable, but best-known) measured velocity, integrator
            // cleared — no stale torque step when lock returns.
            self.velocity_loop.reset(self.phase.get().velocity);
            return self.step_current_control(0.0, 0.0, dt, now_ticks);
        }
        // While the cold-start sequencer owns commutation, the PI must not
        // run: the rotor sits below the cruise target for the whole ramp
        // and the integrator banks amps of torque command against the I/f
        // frame (bench 2026-07-07, velmode-1: |i| = 6.6 A by handoff → OC
        // into the confirm probe). Hold the loop reset (bumpless engage at
        // handoff) and drive the startup at its proven capture current.
        if self.phase.is_starting() {
            self.velocity_loop.reset(self.phase.get().velocity);
            let dir = if target_vel < 0.0 { -1.0 } else { 1.0 };
            return self.step_current_control(dir * STARTUP_MIN_DRIVE_A, 0.0, dt, now_ticks);
        }
        let omega = self.phase.get().velocity;
        // The derating speed ceiling also caps the cruise reference: the
        // current-side rolloff alone would leave the PI winding up against
        // an unreachable target.
        let max_speed = self.derating_cfg.max_speed_erad_s;
        let target_vel = if max_speed > 0.0 {
            clamp_f32(target_vel, -max_speed, max_speed)
        } else {
            target_vel
        };
        // Anti-windup must see the actuator's REAL authority, not the raw
        // current limit: a PI clamped only at max_current_a silently banks
        // whatever the grant withholds as integral — released as a torque
        // step the moment the target (or the derate) moves. Near the
        // ceiling the per-cycle speed cut shrinks the motoring budget to the
        // friction level; a bus-limited or full-pack brake grants less than
        // its limit. So the bounds are the grant itself, asked for a full
        // request each way — asymmetric, since braking is never
        // speed-derated. Same for the q current the current loop actually
        // delivers while it is out of voltage. Without these the PI banked
        // the current limit against a bus-limited or top-speed motor (sim:
        // 10.8 A commanded against 1 A delivered), and the reference ran on
        // toward a target past top speed: told to slow down, the motor held
        // top speed for seconds while the reference ramped back.
        let full = if self.current_limits.max_current_a > 0.0 {
            self.current_limits.max_current_a
        } else {
            f32::INFINITY
        };
        let iq_min = self.grant_torque(0.0, -full, omega).1;
        let iq_max = self.grant_torque(0.0, full, omega).1;
        let clip = self.controller.q_voltage_clip();
        let (iq_min, iq_max) = if clip > 0.0 {
            (
                iq_min,
                iq_max.min(self.last_iq_meas.max(0.0) + VOLTAGE_LIMITED_IQ_MARGIN_A),
            )
        } else if clip < 0.0 {
            (
                iq_min.max(self.last_iq_meas.min(0.0) - VOLTAGE_LIMITED_IQ_MARGIN_A),
                iq_max,
            )
        } else {
            (iq_min, iq_max)
        };
        let iq_target = self
            .velocity_loop
            .step_blocked(target_vel, omega, iq_min, iq_max, clip, dt);
        self.step_current_control(iq_target, 0.0, dt, now_ticks)
    }

    /// Execute current control step
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub(super) fn step_current_control(
        &mut self,
        iq_target: f32,
        id_target: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let prof_t0 = crate::isr_prof::now();
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err(StepError::NotCalibrated);
        }

        // Layer 1: Clamp current targets (prevents absurd commands)
        let (id_target, iq_target) = self.current_limits.clamp_targets(id_target, iq_target);

        // Startup capture floor (see STARTUP_MIN_DRIVE_A): while the
        // sequencer owns commutation a weak drive command is raised to a
        // current that actually captures the rotor. Bounded by the
        // configured limit so a low max_current_a still wins.
        let iq_target = if self.phase.is_starting()
            && iq_target != 0.0
            && iq_target.abs() < STARTUP_MIN_DRIVE_A
        {
            let floor = if self.current_limits.max_current_a > 0.0 {
                STARTUP_MIN_DRIVE_A.min(self.current_limits.max_current_a)
            } else {
                STARTUP_MIN_DRIVE_A
            };
            if iq_target < 0.0 { -floor } else { floor }
        } else {
            iq_target
        };

        // Startup soft-start: the ramp phase ramps the torque command in
        // instead of step-engaging it onto the rotor's undamped magnetic
        // spring (see PhaseProvider::startup_current_scale). 1.0 outside
        // the startup sequencer.
        let scale = self.phase.startup_current_scale();
        let (id_target, iq_target) = (id_target * scale, iq_target * scale);

        // An untrustworthy angle may be π-flipped (pure HFI before its
        // polarity probe resolves) or frozen (back-EMF observer below its
        // speed floor) — iq there is torque in an unknown direction. Zero it;
        // id is polarity-symmetric, and commutation keeps following the
        // estimate so the HFI carrier/probe stays frame-aligned and the gate
        // self-clears once the source locks. Velocity/failsafe paths handle
        // this themselves; this is the backstop for direct current commands.
        let iq_target = if self.phase.angle_trustworthy() {
            iq_target
        } else {
            0.0
        };

        let (id_target, iq_target) =
            self.grant_torque(id_target, iq_target, self.phase.get().velocity);

        // Get phase from provider: the previous update's prediction of the
        // rotor at THIS current sample (PhaseProvider timing contract). It
        // is used as-is for the current Park — the pipeline delay is
        // compensated on the ACTUATION side only (output-vector rotation,
        // see set_actuation_delay).
        // Advancing this angle instead, as the code originally did, also
        // advanced the measurement frame: the PI then regulated the
        // current vector `ωe·dt·cycles` off the true q axis
        // (id_true = −iq·sin(δ) — ~29% of iq parasitic d-current for a
        // Flipsky-class motor at full speed). Found by the sim's
        // actuation-delay plant upgrade.
        let phase_out = self.phase.get();
        let angle_rad = phase_out.angle;
        self.controller
            .set_actuation_advance(self.actuation_advance(phase_out.velocity, dt));

        // HFI carrier for this cycle (zero for non-HFI sources). Must be
        // read between get() and update(): the estimator demodulates the
        // currents fed to update() against this exact carrier sample.
        let (vd_inject, vq_inject) = self.phase.injection();

        // Read currents and run FOC controller. The estimated electrical
        // velocity drives the dq-decoupling feedforward (no-op when no
        // motor params are configured).
        let prof_tc = crate::isr_prof::now();
        let currents = self.current_sensor.read_currents();
        crate::isr_prof::add(&crate::isr_prof::GATE_CURR, prof_tc, crate::isr_prof::now());
        let max_duty = self.pwm.max_duty();
        let prof_t1 = crate::isr_prof::now();
        crate::isr_prof::add(&crate::isr_prof::STEP_GATE, prof_t0, prof_t1);
        let out = self.controller.step_with_injection(
            currents,
            angle_rad,
            phase_out.velocity,
            id_target,
            iq_target,
            vd_inject,
            vq_inject,
            max_duty,
            dt,
        );
        let prof_t2 = crate::isr_prof::now();
        crate::isr_prof::add(&crate::isr_prof::STEP_CTRL, prof_t1, prof_t2);

        // Layer 2: Check measured current against hard overcurrent limit
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        // Track the q-axis modulation for the bus-limit clamp (next cycle).
        self.update_bus_mod_q(out.vq, dt);
        self.last_iq_meas = out.iq;

        self.pwm.set_duties(out.duties);
        let prof_t3 = crate::isr_prof::now();
        crate::isr_prof::add(&crate::isr_prof::STEP_POST, prof_t2, prof_t3);

        // Feed the physics acceleration prior with the measured torque
        // current (see BackEmfObserver::set_accel_prior).
        self.phase.note_torque_current(out.iq.abs(), dt);

        // Update phase provider for next step (feeds observer if present).
        // The observer gets the PREVIOUS command — the voltage that was
        // actually acting while these currents were measured.
        self.update_phase_with_applied_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );
        crate::isr_prof::add(&crate::isr_prof::STEP_EST, prof_t3, crate::isr_prof::now());

        Ok(out)
    }

    /// Execute open-loop control step (for calibration)
    ///
    /// Uses commanded angle instead of sensor feedback.
    /// Current feedback is still used to regulate the applied current.
    ///
    /// When `velocity_rad_s == 0`: locks rotor at `angle_rad`.
    /// When `velocity_rad_s != 0`: advances angle at the given velocity,
    /// enabling sensorless open-loop spinning.
    pub(super) fn step_open_loop(
        &mut self,
        angle_rad: f32,
        current: f32,
        velocity_rad_s: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err(StepError::NotCalibrated);
        }

        // Advance angle if velocity is set, otherwise use commanded angle.
        // wrap_angle, not `%`: f32 `%` is a libm fmodf call (~200 cycles,
        // caught by 2026-07-06 tier-2 PC profiling), while the increment
        // here is always a small step from an already-wrapped angle —
        // wrap_angle's branch+subtract hot path.
        let angle = if velocity_rad_s != 0.0 {
            self.open_loop_angle = wrap_angle(self.open_loop_angle + velocity_rad_s * dt);
            self.open_loop_angle
        } else {
            // A host-supplied angle is not wrapped (maneuvers, CLI); the
            // g474 CORDIC mapping breaks beyond ±3π.
            self.open_loop_angle = wrap_angle(angle_rad);
            self.open_loop_angle
        };

        // Clamp open-loop current to the target limit
        let current = if self.current_limits.max_current_a > 0.0 {
            clamp_f32(
                current,
                -self.current_limits.max_current_a,
                self.current_limits.max_current_a,
            )
        } else {
            current
        };

        // Read currents and run FOC controller with commanded angle.
        // When stationary (velocity=0): current on d-axis to lock rotor (resistance measurement).
        // When spinning (velocity≠0): current on q-axis to produce torque.
        let (id_target, iq_target) = if velocity_rad_s == 0.0 {
            (current, 0.0)
        } else {
            (0.0, current)
        };
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        // Same pipeline delay as closed loop; the commanded velocity is the
        // exact frame rate (also clears any stale advance from a previous
        // closed-loop cycle when velocity is 0).
        self.controller
            .set_actuation_advance(self.actuation_advance(velocity_rad_s, dt));
        let out = self
            .controller
            .step(currents, angle, id_target, iq_target, max_duty, dt);

        // Check measured overcurrent
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        self.pwm.set_duties(out.duties);

        // Update phase provider (for sensor tracking, even in open-loop)
        self.update_phase_with_applied_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );

        Ok(out)
    }

    /// Execute direct voltage step — no PI control.
    ///
    /// Applies the given dq voltages via `FocController::apply_dq`, reads
    /// currents for telemetry, and feeds the phase observer. Used for
    /// measurement modes (HFI inductance) and direct voltage control.
    pub(super) fn step_direct_voltage(
        &mut self,
        vd: f32,
        vq: f32,
        angle_rad: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let max_duty = self.pwm.max_duty();
        // Latest measured currents feed the dead-time compensation signs;
        // without a PI loop the distortion is otherwise uncompensated and
        // eats the commanded voltage (see `apply_dq` docs). Uncalibrated
        // sensor → zeros → compensation self-cancels.
        let currents = if self.current_sensor.is_calibrated() {
            Some(self.current_sensor.read_currents())
        } else {
            None
        };
        let (i_alpha_m, i_beta_m) = match currents {
            Some(c) => clarke(c.0, c.1),
            None => (0.0, 0.0),
        };
        let mut out = self
            .controller
            .apply_dq(vd, vq, angle_rad, i_alpha_m, i_beta_m, max_duty);

        self.pwm.set_duties(out.duties);

        // Read currents for telemetry and phase observer
        if let Some(currents) = currents {
            out.ia = currents.0;
            out.ib = currents.1;
            out.ic = currents.2;
            let (i_alpha, i_beta) = (i_alpha_m, i_beta_m);
            out.i_alpha = i_alpha;
            out.i_beta = i_beta;
            let (sin_a, cos_a) = S::sin_cos(angle_rad);
            let (id, iq) = park(i_alpha, i_beta, sin_a, cos_a);
            out.id = id;
            out.iq = iq;

            // No PI loop reins the current in here — the commanded voltage
            // is applied verbatim — so the measured check is the only
            // software protection this mode has.
            if self.current_limits.is_overcurrent(out.id, out.iq) {
                self.pwm.disable();
                self.controller.reset();
                self.mode = ControlMode::Stopped;
                return Err(StepError::Overcurrent);
            }
        }

        self.update_phase_with_applied_voltage(
            out.v_alpha,
            out.v_beta,
            out.i_alpha,
            out.i_beta,
            dt,
            now_ticks,
        );

        Ok(out)
    }

    /// Run the commanded mode for one cycle. Failsafe and the deadshort
    /// probe have already taken the cycle if they own the bridge.
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub(super) fn dispatch_mode(
        &mut self,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        match self.mode {
            ControlMode::Stopped => {
                let prof_t0 = crate::isr_prof::now();
                self.stopped_housekeeping = self.stopped_housekeeping.wrapping_add(1);
                // Safe-off: `disable()` is the platform's emergency-stop
                // (all channels off / high-Z on the current boards). Asserted
                // on the transition into Stopped, then re-asserted every 256
                // cycles as a backstop — the per-cycle disable measured ~430
                // cycles of ISR budget (G431) for a timer that is already off.
                if !self.pwm_off || self.stopped_housekeeping == 0 {
                    self.pwm.disable();
                    self.pwm_off = true;
                }
                let prof_t1 = crate::isr_prof::now();
                crate::isr_prof::add(&crate::isr_prof::STEP_PWMOFF, prof_t0, prof_t1);
                // The previous cycle's command WAS applied before this stop
                // took effect — let the estimators integrate it, then decay
                // to zero volts. Decimated ×4: with the bridge floated the
                // inputs are constant zeros (no phase-voltage sensing yet),
                // so integrating them at 5 kHz with dt×4 is the same math at
                // a quarter of the measured ~1430 cycles/ISR; sensored
                // sources keep their standstill angle (hall edges latch in
                // their own capture, read on the next update).
                if self.stopped_housekeeping & 0x3 == 0 {
                    self.update_phase_with_applied_voltage(0.0, 0.0, 0.0, 0.0, dt * 4.0, now_ticks);
                }
                crate::isr_prof::add(
                    &crate::isr_prof::STEP_PHASE,
                    prof_t1,
                    crate::isr_prof::now(),
                );
                Ok(FocOutput::default())
            }
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            ControlMode::VelocityControl { target_vel } => {
                self.step_velocity_control(target_vel, dt, now_ticks)
            }
            ControlMode::PositionControl { .. } => self.step_position_control(dt, now_ticks),
            ControlMode::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                ..
            } => self.step_open_loop(angle_rad, current, velocity_rad_s, dt, now_ticks),
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                // Host-supplied angle: wrap it (see step_open_loop).
                let angle = wrap_angle(angle_rad);
                self.step_direct_voltage(vd, vq, angle, dt, now_ticks)
            }
            ControlMode::Coast => {
                // High-Z: all FETs off, motor spins freely.
                self.pwm.set_phase_states([
                    PhaseState::Float,
                    PhaseState::Float,
                    PhaseState::Float,
                ]);
                self.pwm_off = true;
                self.controller.reset();
                self.update_phase_with_applied_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
            ControlMode::Brake => self.step_park_brake(dt, now_ticks),
        }
    }
}

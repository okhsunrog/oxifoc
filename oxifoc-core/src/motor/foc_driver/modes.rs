//! One control cycle for each `ControlMode` arm.

use super::{ControlMode, FocDriver, STARTUP_MIN_DRIVE, StepError, VOLTAGE_LIMITED_IQ_MARGIN};
use crate::foc::controller::FocOutput;
use crate::foc::fast_math::sqrtf;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::{PhasePwm, PhaseState};
use crate::foc::sensors::CurrentSensor;
use crate::foc::transforms::park;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;
use crate::isr_prof;
use crate::units::{
    AngleRepr, ConstZero, ElectricCurrent, ElectricPotential, ElectricalAngle,
    ElectricalAngularVelocity, Scalar, Sign, Time,
};

/// Host-only comparison of prepared bounds against the same-cycle probes.
#[cfg(test)]
pub(super) mod authority_audit {
    use crate::units::{ElectricCurrent, ElectricalAngularVelocity};
    use core::cell::Cell;

    #[derive(Clone, Copy, Debug, Default)]
    pub struct Audit {
        pub used: u32,
        pub fallback: u32,
        /// Largest |prepared - fresh| over both bounds, in amperes.
        pub max_delta_a: f32,
        /// Updates whose prepared and fresh bounds differ by more than 10 mA.
        pub above_10_ma: u32,
        /// The same count above 0.5 A.
        pub above_500_ma: u32,
    }

    std::thread_local! {
        pub static AUDIT: Cell<Audit> = const { Cell::new(Audit {
            used: 0, fallback: 0, max_delta_a: 0.0, above_10_ma: 0, above_500_ma: 0,
        }) };
    }

    pub fn used(
        prepared: (ElectricCurrent, ElectricCurrent),
        fresh: (ElectricCurrent, ElectricCurrent, ElectricalAngularVelocity),
    ) {
        let delta = |a: ElectricCurrent, b: ElectricCurrent| {
            if a == b { 0.0 } else { (a - b).abs().value }
        };
        let worst = delta(prepared.0, fresh.0).max(delta(prepared.1, fresh.1));
        AUDIT.with(|cell| {
            let mut audit = cell.get();
            audit.used += 1;
            audit.max_delta_a = audit.max_delta_a.max(worst);
            audit.above_10_ma += u32::from(worst > 0.01);
            audit.above_500_ma += u32::from(worst > 0.5);
            cell.set(audit);
        });
    }

    pub fn fallback() {
        AUDIT.with(|cell| {
            let mut audit = cell.get();
            audit.fallback += 1;
            cell.set(audit);
        });
    }
}

#[derive(Clone, Copy)]
enum CurrentFeedback {
    Ignore,
    Cruise,
}

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
    /// While the short's current stays within `max_brake`, all low sides
    /// conduct: the windings short opposes motion either way with no angle
    /// or speed estimate, draws nothing at standstill, and holds a roll-back
    /// (as speed-proportional drag — not a position hold). When the rotor is
    /// pushed faster (a board rolling down a steep slope) the short would
    /// grow toward λ/L at whatever speed; instead the brake turns
    /// current-controlled at `max_brake` against the motion — never let
    /// go — and returns to the short once slow again. Without a trustworthy
    /// angle there is no controlled brake: a short current past the limit
    /// then releases to high-Z (the speed that produced it is unknown, and
    /// shorting on at it could mean ~100 A of uncontrolled current and a
    /// violent stop).
    pub(super) fn step_park_brake(
        &mut self,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let limit = self.current_limits.max_brake;
        let omega = self.torque_speed();
        if let Some(iq) = self.park_iq
            && self.phase.angle_trustworthy()
        {
            let target = if omega.is_positive() { -limit } else { limit };
            let step = self.failsafe_cfg.slew * dt;
            let iq = iq + (target - iq).clamp_to(-step, step);
            let out = self.step_current_control(iq, ElectricCurrent::ZERO, dt, now_ticks)?;
            // Back to the short once holding the limit takes a q voltage
            // against the motion (back-EMF below R·I): the short alone now
            // carries less than the limit — the switch point is where the
            // two currents meet, so no step. VESC switches on the same vq
            // sign change.
            self.park_iq = (out.vq * omega).is_positive().then_some(iq);
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
            (
                ElectricCurrent::ZERO,
                ElectricCurrent::ZERO,
                ElectricCurrent::ZERO,
            )
        };
        let (i_alpha, i_beta) = self
            .controller
            .current_sense_topology()
            .clarke((ia, ib, ic));
        // αβ magnitude equals the dq magnitude (Park preserves it). The
        // hard trip stays the backstop (a limit left unset).
        if self.current_limits.is_overcurrent(i_alpha, i_beta) {
            self.pwm.disable();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }
        let i_mag = (i_alpha * i_alpha + i_beta * i_beta).sqrt_with(sqrtf);
        if limit.is_positive() && i_mag > limit {
            if self.phase.angle_trustworthy() {
                // Continue from the short's current, capped, as a brake
                // against the motion (next cycle on).
                let start = i_mag.min(limit);
                self.park_iq = Some(if omega.is_positive() { -start } else { start });
            } else {
                warn!(
                    "park brake: short current {:?} A past the {:?} A limit without an angle, releasing",
                    i_mag, limit
                );
                self.pwm.disable();
                self.pwm_off = true;
                self.mode = ControlMode::Stopped;
            }
        }

        // Terminal voltage is zero while shorted; the estimators integrate
        // that with the real circulating currents.
        self.update_phase_with_applied_voltage(
            ElectricPotential::ZERO,
            ElectricPotential::ZERO,
            i_alpha,
            i_beta,
            dt,
            now_ticks,
        );
        Ok(FocOutput {
            ia,
            ib,
            ic,
            i_alpha,
            i_beta,
            angle: self.phase.get().angle,
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
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        self.step_scheduled_velocity(None, dt, now_ticks)
    }

    /// Execute one native current step, evaluating cruise once per 1 ms grid
    /// period without replaying missed updates.
    pub(super) fn step_velocity_control(
        &mut self,
        target_vel: ElectricalAngularVelocity,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        self.step_scheduled_velocity(Some(target_vel), dt, now_ticks)
    }

    #[cold]
    #[inline(never)]
    fn velocity_unavailable(&mut self, dt: Time, now_ticks: u64) -> Result<FocOutput, StepError> {
        self.invalidate_velocity();
        self.step_current_control(ElectricCurrent::ZERO, ElectricCurrent::ZERO, dt, now_ticks)
    }

    #[cold]
    #[inline(never)]
    fn velocity_starting(
        &mut self,
        target: Option<ElectricalAngularVelocity>,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        self.invalidate_velocity();
        let direction = if self.velocity_target(target).is_negative() {
            -1.0
        } else {
            1.0
        };
        self.step_current_control(
            direction * STARTUP_MIN_DRIVE,
            ElectricCurrent::ZERO,
            dt,
            now_ticks,
        )
    }

    fn velocity_target(
        &self,
        target: Option<ElectricalAngularVelocity>,
    ) -> ElectricalAngularVelocity {
        target.unwrap_or_else(|| {
            let error = self.position_target.minus(self.position.position());
            self.position_cfg
                .speed_for_encoded(error, self.velocity_loop.ramp_limit_encoded())
        })
    }

    fn step_scheduled_velocity(
        &mut self,
        target: Option<ElectricalAngularVelocity>,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        // These gates remain native-rate. A held cruise request must never
        // outlive a usable physical speed or the commutation owner's trust.
        if !self.phase.angle_trustworthy() {
            return self.velocity_unavailable(dt, now_ticks);
        }
        if self.phase.is_starting() {
            return self.velocity_starting(target, dt, now_ticks);
        }
        let Some(velocity) = self.phase.rotor_velocity() else {
            return self.velocity_unavailable(dt, now_ticks);
        };
        let omega = velocity.velocity();
        let epoch = self.phase.motion_reference_epoch();
        if !self.velocity_feedback_available || !self.velocity_schedule.matches_epoch(epoch) {
            self.velocity_loop.reset(omega);
            self.velocity_schedule.rearm(epoch);
            self.velocity_feedback_available = true;
        }
        let tick = self.velocity_schedule.begin_pipelined_cycle(
            now_ticks,
            self.protection.service_slots.slot,
            self.protection.service_slots.length(),
        );
        match tick {
            super::velocity_schedule::VelocityTick::Hold => {}
            super::velocity_schedule::VelocityTick::Prepare => {
                self.prepare_velocity_authority(now_ticks);
            }
            super::velocity_schedule::VelocityTick::Update { dt } => {
                self.update_velocity_request(target, omega, dt, now_ticks);
            }
            super::velocity_schedule::VelocityTick::Reanchor => {
                self.velocity_loop.reset(omega);
                self.velocity_schedule.rearm(epoch);
            }
        }
        let requested = self.velocity_schedule.requested_iq();
        self.step_current_control_with_feedback(
            requested,
            ElectricCurrent::ZERO,
            dt,
            now_ticks,
            CurrentFeedback::Cruise,
        )
    }

    /// Both anti-windup probes at the current torque speed.
    #[inline(never)]
    fn velocity_authority(&self) -> (ElectricCurrent, ElectricCurrent, ElectricalAngularVelocity) {
        let full = if self.current_limits.max_current.is_positive() {
            self.current_limits.max_current
        } else {
            ElectricCurrent::INFINITY
        };
        let omega = self.torque_speed();
        let constraints = self.torque_constraints(omega);
        let iq_min = self.grant_iq_authority(-full, &constraints);
        let iq_max = self.grant_iq_authority(full, &constraints);
        (iq_min, iq_max, omega)
    }

    /// Stage A, one slot before the outer update. Commands, limit changes and
    /// velocity invalidation drop the stored bounds.
    #[inline(never)]
    fn prepare_velocity_authority(&mut self, now_ticks: u64) {
        let (iq_min, iq_max, omega) = self.velocity_authority();
        self.velocity_schedule.store_authority(
            iq_min,
            iq_max,
            omega,
            self.bus_modulation_class(),
            now_ticks,
        );
    }

    // Keep authority, position P, slew and PI register pressure off the held
    // current path. Only this due-only helper sees the elapsed outer-loop dt.
    #[inline(never)]
    fn update_velocity_request(
        &mut self,
        target: Option<ElectricalAngularVelocity>,
        omega: ElectricalAngularVelocity,
        dt: Time,
        now_ticks: u64,
    ) {
        let profile_start = isr_prof::now();
        let target_vel = self.velocity_target(target);
        let (tracking, mut blockage, mut voltage_clip) = self.velocity_schedule.take_feedback();
        self.velocity_loop.track_grant(tracking);
        // The derating speed ceiling also caps the cruise reference: the
        // current-side rolloff alone would leave the PI winding up against
        // an unreachable target.
        let max_speed = self.speed_ceiling(target_vel);
        let target_vel = if max_speed.is_positive() {
            target_vel.clamp_to(-max_speed, max_speed)
        } else {
            target_vel
        };
        // Anti-windup must see the actuator's REAL authority, not the raw
        // current limit: a PI clamped only at max_current silently banks
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
        // Prepared bounds are at most two native cycles old. Anything else
        // falls back to the same probes inline.
        let (iq_min, iq_max) = match self.velocity_schedule.take_authority(
            self.torque_speed(),
            self.bus_modulation_class(),
            now_ticks,
        ) {
            Some(bounds) => {
                #[cfg(test)]
                authority_audit::used(bounds, self.velocity_authority());
                bounds
            }
            None => {
                #[cfg(test)]
                authority_audit::fallback();
                let (iq_min, iq_max, _) = self.velocity_authority();
                (iq_min, iq_max)
            }
        };
        voltage_clip.note_clip(self.controller.q_voltage_clip());
        blockage.merge(voltage_clip);
        let iq_max = if voltage_clip.positive() {
            iq_max.min(self.last_iq_meas.max(ElectricCurrent::ZERO) + VOLTAGE_LIMITED_IQ_MARGIN)
        } else {
            iq_max
        };
        let iq_min = if voltage_clip.negative() {
            iq_min.max(self.last_iq_meas.min(ElectricCurrent::ZERO) - VOLTAGE_LIMITED_IQ_MARGIN)
        } else {
            iq_min
        };
        let iq_target = self
            .velocity_loop
            .step_with_blockage(target_vel, omega, iq_min, iq_max, blockage, dt);
        #[cfg(feature = "velocity-debug")]
        if self.record_velocity_updates {
            use crate::units::amps;
            use crate::velocity_telemetry as trace;
            let mut flags = trace::UPDATED | trace::FEEDBACK_VALID;
            for (present, flag) in [
                (iq_target <= iq_min + amps(1e-4), trace::LOWER_BOUND),
                (iq_target >= iq_max - amps(1e-4), trace::UPPER_BOUND),
                (blockage.positive(), trace::BLOCKED_POSITIVE),
                (blockage.negative(), trace::BLOCKED_NEGATIVE),
                (voltage_clip.positive(), trace::VOLTAGE_POSITIVE),
                (voltage_clip.negative(), trace::VOLTAGE_NEGATIVE),
            ] {
                if present {
                    flags |= flag;
                }
            }
            self.last_velocity_update = Some(super::VelocityUpdate {
                tick_us: now_ticks,
                target: target_vel,
                feedback: omega,
                iq_min,
                iq_max,
                flags,
            });
        }
        if !self.velocity_schedule.set_request(iq_target) {
            self.invalidate_velocity();
        }
        isr_prof::add(&isr_prof::VELOCITY_REQUEST, profile_start, isr_prof::now());
    }

    #[cfg(feature = "velocity-debug")]
    #[inline(never)]
    fn record_current_debug(
        &mut self,
        requested: ElectricCurrent,
        before_grant: (ElectricCurrent, ElectricCurrent),
        granted: ElectricCurrent,
        id: ElectricCurrent,
        omega: ElectricalAngularVelocity,
    ) {
        use crate::velocity_telemetry as trace;
        let mut frame = trace::VelocityDebugFrame {
            flags: trace::ACTIVE
                | trace::CURRENT_CONTROL
                | self.current_limit_flags(requested, before_grant, granted, omega),
            ..Default::default()
        };
        frame.feedback = frame.quantize_speed(omega);
        frame.requested_iq = frame.quantize_current(requested);
        frame.granted_iq = frame.quantize_current(granted);
        frame.granted_id = frame.quantize_current(id);
        self.last_current_debug = Some(frame);
    }

    /// Execute current control step
    #[inline(always)]
    pub(super) fn step_current_control(
        &mut self,
        iq_target: ElectricCurrent,
        id_target: ElectricCurrent,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        self.step_current_control_with_feedback(
            iq_target,
            id_target,
            dt,
            now_ticks,
            CurrentFeedback::Ignore,
        )
    }

    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn step_current_control_with_feedback(
        &mut self,
        iq_target: ElectricCurrent,
        id_target: ElectricCurrent,
        dt: Time,
        now_ticks: u64,
        feedback: CurrentFeedback,
    ) -> Result<FocOutput, StepError> {
        let prof_t0 = isr_prof::now();
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err(StepError::NotCalibrated);
        }

        #[cfg(feature = "velocity-debug")]
        let requested_iq = iq_target;
        // Layer 1: Clamp current targets (prevents absurd commands)
        let (id_target, iq_target) = self.current_limits.clamp_targets(id_target, iq_target);
        #[cfg(feature = "velocity-debug")]
        let clamped_iq = iq_target;

        // Startup capture floor (see STARTUP_MIN_DRIVE): while the
        // sequencer owns commutation a weak drive command is raised to a
        // current that actually captures the rotor. Bounded by the
        // configured limit so a low max_current still wins.
        let iq_target = if self.phase.is_starting()
            && iq_target != ElectricCurrent::ZERO
            && iq_target.abs() < STARTUP_MIN_DRIVE
        {
            let floor = if self.current_limits.max_current.is_positive() {
                STARTUP_MIN_DRIVE.min(self.current_limits.max_current)
            } else {
                STARTUP_MIN_DRIVE
            };
            if iq_target.is_negative() {
                -floor
            } else {
                floor
            }
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
            ElectricCurrent::ZERO
        };

        let omega = self.torque_speed();
        let (id_target, granted_iq) = self.grant_torque(id_target, iq_target, omega);
        #[cfg(feature = "velocity-debug")]
        if self.current_debug_due && matches!(self.mode, ControlMode::CurrentControl { .. }) {
            self.record_current_debug(
                requested_iq,
                (clamped_iq, iq_target),
                granted_iq,
                id_target,
                omega,
            );
        }
        let iq_target = granted_iq;

        self.last_granted_iq = iq_target;
        self.last_granted_id = id_target;

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
        let angle = phase_out.angle;
        self.controller
            .set_actuation_advance(self.actuation_advance(phase_out.velocity, dt));

        // HFI carrier for this cycle (zero for non-HFI sources). Must be
        // read between get() and update(): the estimator demodulates the
        // currents fed to update() against this exact carrier sample.
        let (vd_inject, vq_inject) = self.phase.injection();

        // Read currents and run FOC controller. The estimated electrical
        // velocity drives the dq-decoupling feedforward (no-op when no
        // motor params are configured).
        let prof_tc = isr_prof::now();
        let currents = self.current_sensor.read_currents();
        isr_prof::add(&isr_prof::GATE_CURR, prof_tc, isr_prof::now());
        let max_duty = self.pwm.max_duty();
        let prof_t1 = isr_prof::now();
        isr_prof::add(&isr_prof::STEP_GATE, prof_t0, prof_t1);
        let out = self.controller.step_with_injection(
            currents,
            angle,
            phase_out.velocity,
            id_target,
            iq_target,
            vd_inject,
            vq_inject,
            max_duty,
            dt,
        );
        let prof_t2 = isr_prof::now();
        isr_prof::add(&isr_prof::STEP_CTRL, prof_t1, prof_t2);

        // Layer 2: Check measured current against hard overcurrent limit
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err(StepError::Overcurrent);
        }

        // Track the q-axis modulation for the bus-limit clamp (next cycle).
        self.update_bus_mod_q(self.controller.bus_mod_q(), dt);
        self.update_duty_utilization(out.vd, out.vq, dt);
        self.last_iq_meas = out.iq;

        self.pwm.set_duties(out.duties);
        let prof_t3 = isr_prof::now();
        isr_prof::add(&isr_prof::STEP_POST, prof_t2, prof_t3);

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
        isr_prof::add(&isr_prof::STEP_EST, prof_t3, isr_prof::now());

        // Only the scheduled cruise caller owns this interval. Independent
        // current/failsafe steps must not charge an old cruise request. The
        // held request cannot change during this owned native call; reading
        // it here avoids retaining a whole telemetry return in the wrapper.
        // Every error exits above and the driver invalidates the interval.
        if matches!(feedback, CurrentFeedback::Cruise) {
            self.velocity_schedule.record_delivery(
                self.velocity_schedule.requested_iq(),
                self.last_granted_iq,
                self.controller.q_voltage_clip(),
            );
        }

        Ok(out)
    }

    /// Execute open-loop control step (for calibration)
    ///
    /// Uses commanded angle instead of sensor feedback.
    /// Current feedback is still used to regulate the applied current.
    ///
    /// When `velocity == 0`: locks rotor at `angle`.
    /// When `velocity != 0`: advances angle at the given velocity,
    /// enabling sensorless open-loop spinning.
    pub(super) fn step_open_loop(
        &mut self,
        angle: ElectricalAngle,
        current: ElectricCurrent,
        velocity: ElectricalAngularVelocity,
        dt: Time,
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
        let rotating = velocity != ElectricalAngularVelocity::ZERO;
        let angle = if rotating {
            self.open_loop_angle = wrap_angle(self.open_loop_angle + velocity * dt);
            self.open_loop_angle
        } else {
            // A host-supplied angle is not wrapped (maneuvers, CLI); the
            // g474 CORDIC mapping breaks beyond ±3π.
            self.open_loop_angle = wrap_angle(angle);
            self.open_loop_angle
        };

        // Clamp open-loop current to the target limit
        let current = if self.current_limits.max_current.is_positive() {
            current.clamp_to(
                -self.current_limits.max_current,
                self.current_limits.max_current,
            )
        } else {
            current
        };

        // Read currents and run FOC controller with commanded angle.
        let current = self.commissioning.as_mut().map_or(current, |context| {
            context.constrain_current::<S>(angle, current, rotating)
        });
        // When stationary (velocity=0): current on d-axis to lock rotor (resistance measurement).
        // When spinning (velocity≠0): current on q-axis to produce torque.
        let (id_target, iq_target) = if rotating {
            (ElectricCurrent::ZERO, current)
        } else {
            (current, ElectricCurrent::ZERO)
        };
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        // Same pipeline delay as closed loop; the commanded velocity is the
        // exact frame rate (also clears any stale advance from a previous
        // closed-loop cycle when velocity is 0).
        self.controller
            .set_actuation_advance(self.actuation_advance(velocity, dt));
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
        vd: ElectricPotential,
        vq: ElectricPotential,
        angle: ElectricalAngle,
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let max_duty = self.pwm.max_duty();
        // Outside bounded detection packets, latest measured currents feed
        // the dead-time compensation signs;
        // without a PI loop the distortion is otherwise uncompensated and
        // eats the commanded voltage (see `apply_dq` docs). Uncalibrated
        // sensor → zeros → compensation self-cancels.
        let currents = if self.current_sensor.is_calibrated() {
            Some(self.current_sensor.read_currents())
        } else {
            None
        };
        let (i_alpha_m, i_beta_m) = match currents {
            Some(c) => self.controller.current_sense_topology().clarke(c),
            None => (ElectricCurrent::ZERO, ElectricCurrent::ZERO),
        };
        #[cfg(feature = "detection")]
        let (i_comp_alpha, i_comp_beta) = self
            .detection_compensation_current
            .unwrap_or((i_alpha_m, i_beta_m));
        #[cfg(not(feature = "detection"))]
        let (i_comp_alpha, i_comp_beta) = (i_alpha_m, i_beta_m);
        let mut out = self
            .controller
            .apply_dq(vd, vq, angle, i_comp_alpha, i_comp_beta, max_duty);

        self.pwm.set_duties(out.duties);

        // Read currents for telemetry and phase observer
        if let Some((ia, ib, ic)) = currents {
            (out.ia, out.ib, out.ic) = (ia, ib, ic);
            let (i_alpha, i_beta) = (i_alpha_m, i_beta_m);
            out.i_alpha = i_alpha;
            out.i_beta = i_beta;
            let (sin_a, cos_a) = S::sin_cos(angle.radians());
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
        dt: Time,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        match self.mode {
            ControlMode::Stopped => {
                let prof_t0 = isr_prof::now();
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
                let prof_t1 = isr_prof::now();
                isr_prof::add(&isr_prof::STEP_PWMOFF, prof_t0, prof_t1);
                // The previous cycle's command WAS applied before this stop
                // took effect — let the estimators integrate it, then decay
                // to zero volts. Decimated ×4: with the bridge floated the
                // inputs are constant zeros (no phase-voltage sensing yet),
                // so integrating them at 5 kHz with dt×4 is the same math at
                // a quarter of the measured ~1430 cycles/ISR; sensored
                // sources keep their standstill angle (hall edges latch in
                // their own capture, read on the next update).
                if self.stopped_housekeeping & 0x3 == 0 {
                    self.update_phase_without_drive(dt * 4.0, now_ticks);
                }
                isr_prof::add(&isr_prof::STEP_PHASE, prof_t1, isr_prof::now());
                Ok(FocOutput::default())
            }
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            ControlMode::Throttle { fraction } => self.step_current_control(
                self.throttle_current(fraction),
                ElectricCurrent::ZERO,
                dt,
                now_ticks,
            ),
            ControlMode::VelocityControl { target_vel } => {
                self.step_velocity_control(target_vel, dt, now_ticks)
            }
            ControlMode::PositionControl { .. } => self.step_position_control(dt, now_ticks),
            ControlMode::OpenLoop {
                angle,
                current,
                velocity,
                ..
            } => self.step_open_loop(angle, current, velocity, dt, now_ticks),
            ControlMode::DirectVoltage { vd, vq, angle } => {
                // Host-supplied angle: wrap it (see step_open_loop).
                self.step_direct_voltage(vd, vq, wrap_angle(angle), dt, now_ticks)
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
                self.update_phase_without_drive(dt, now_ticks);
                Ok(FocOutput::default())
            }
            ControlMode::Brake => self.step_park_brake(dt, now_ticks),
        }
    }
}

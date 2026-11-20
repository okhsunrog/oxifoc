//! Hot path: `FocDriver::step` and the mode dispatch around it.

use super::driver::drive_direction;
use super::{ControlMode, FocDriver, SENSORLESS_RESTART_AFTER, StepError};
use crate::foc::constants::SQRT_3_DIV_2;
use crate::foc::controller::FocOutput;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
use crate::units::{ConstZero, ElectricCurrent, ElectricPotential, Time};

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Execute one FOC control step
    ///
    /// Call this from your ADC ISR synchronized with PWM.
    /// Uses the stored dt (set via `with_dt()` or defaulting to 20kHz).
    ///
    /// # Arguments
    /// * `now_ticks` - Monotonic microseconds, also used by the phase provider.
    ///   Hardware capture clocks must be normalized to this domain before
    ///   calling the driver (as required by its command watchdog as well).
    ///
    /// # Returns
    /// * `Ok(FocOutput)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready or overcurrent detected
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub fn step(&mut self, now_ticks: u64) -> Result<FocOutput, StepError> {
        #[cfg(feature = "service-slots")]
        self.protection.service_slots.advance();
        if self.current_offset.is_active() {
            return Ok(self.step_current_offset());
        }
        self.watchdog
            .note_cycle(now_ticks, self.mode, self.phase.is_starting());
        self.position.update(self.phase.get().angle);
        let result = self.step_inner(now_ticks);
        if result.is_err() {
            self.end_commissioning();
            self.restore_current_pi_gains();
            self.invalidate_velocity();
            self.last_granted_iq = ElectricCurrent::ZERO;
            self.last_granted_id = ElectricCurrent::ZERO;
        }
        let mut out = result?;
        if let Some(context) = self.commissioning.as_mut() {
            if matches!(
                self.mode,
                ControlMode::OpenLoop { .. } | ControlMode::DirectVoltage { .. }
            ) {
                if !context.source_allows(out.i_alpha, out.i_beta) {
                    self.safe_off();
                    return Err(StepError::SourceLimit);
                }
                let [a, b, c] = out.duties.map(i32::from);
                let period = f32::from(self.pwm.max_duty().max(1));
                // 1.5 * Clarke(duty): Ibus = m_alpha*Ialpha + m_beta*Ibeta.
                context.modulation.record_command(
                    0.5 * (2 * a - b - c) as f32 / period,
                    SQRT_3_DIV_2 * (b - c) as f32 / period,
                );
            } else {
                context.modulation.record_command(0.0, 0.0);
            }
        }
        self.stamp_step_telemetry(&mut out);
        Ok(out)
    }

    /// Stamp the active angle source's velocity into the telemetry — one
    /// chokepoint so every mode arm (incl. failsafe / deadshort / stop)
    /// carries it. `phase.get()` returns the cached output of this cycle's
    /// update, so this is a copy, not a recompute.
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn stamp_step_telemetry(&mut self, out: &mut FocOutput) {
        out.velocity = self.phase.get().velocity;
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn step_inner(&mut self, now_ticks: u64) -> Result<FocOutput, StepError> {
        let dt = self.dt;
        // Failsafe overrides the commanded mode while it runs (ramp-down /
        // regen-brake / coast), then cuts PWM and clears itself.
        if self.failsafe_ctrl.is_active() {
            return self.step_failsafe(dt, now_ticks);
        }
        if let Some(probed) = self.step_deadshort(dt, now_ticks) {
            return probed;
        }
        self.maybe_restart_sensorless(dt);
        self.dispatch_mode(dt, now_ticks)
    }

    /// Sensorless flying-restart probe (deadshort): hold the bridge at zero
    /// voltage so the phase manager can read the back-EMF-driven current
    /// slope and catch an already-spinning rotor. Overrides the commanded
    /// torque mode for the few cycles the probe runs; the zero vector +
    /// measured current go through the same apply_dq/observer path as a
    /// DirectVoltage(0,0), and the manager seeds the observer on a catch.
    ///
    /// `Some` while the probe owns the bridge. Retain current-PI bias across
    /// the brief short so regulation resumes with its pre-probe voltage bias.
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn step_deadshort(&mut self, dt: Time, now_ticks: u64) -> Option<Result<FocOutput, StepError>> {
        self.phase.startup_status().wants_short().then(|| {
            let zero = ElectricPotential::ZERO;
            self.step_direct_voltage(zero, zero, self.phase.get().angle, dt, now_ticks)
        })
    }

    /// Sensorless restart on trust loss (see [`SENSORLESS_RESTART_AFTER`]):
    /// a nonzero drive command with the angle continuously untrustworthy
    /// means the iq gate is coasting the motor and nothing will bring the
    /// estimator back by itself — re-enter the cold start. No-op for
    /// sensored sources and while the startup sequencer already runs
    /// (`begin_cold_start` guards on both).
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn maybe_restart_sensorless(&mut self, dt: Time) {
        if let Some(direction) = drive_direction(self.mode)
            && !self.phase.is_starting()
        {
            if self.phase.angle_trustworthy() {
                self.untrusted_time = Time::ZERO;
            } else {
                self.untrusted_time += dt;
                if self.untrusted_time >= SENSORLESS_RESTART_AFTER {
                    self.untrusted_time = Time::ZERO;
                    self.phase.begin_cold_start(direction);
                }
            }
        } else {
            self.untrusted_time = Time::ZERO;
        }
    }
}

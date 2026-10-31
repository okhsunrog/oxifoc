//! Hot path: `FocDriver::step` and the mode dispatch around it.

use super::{ControlMode, FocDriver, SENSORLESS_RESTART_AFTER_S, StepError};
use crate::foc::controller::FocOutput;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;

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
    /// * `now_ticks` - Monotonic ticks for phase sampling (sensor-defined timebase)
    ///
    /// # Returns
    /// * `Ok(FocOutput)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready or overcurrent detected
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub fn step(&mut self, now_ticks: u64) -> Result<FocOutput, StepError> {
        if self.current_offset.is_active() {
            return Ok(self.step_current_offset());
        }
        self.watchdog
            .note_cycle(now_ticks, self.mode, self.phase.is_starting());
        self.position.update(self.phase.get().angle);
        let mut out = self.step_inner(now_ticks)?;
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
        out.velocity_rad_s = self.phase.get().velocity;
        // Debug fast-frame mapping (bench estimator forensics): repurpose
        // four columns with observer internals — velocity/erpm ← PLL ω̂
        // (ALWAYS the observer, not the active source), angle ← phase_pll,
        // vd ← DRIVE-frame-vs-estimate gap `Δ(active_angle, phase_pll)`
        // (2026-07-07 freq-led high-speed OC forensics: the offline flux
        // replay aliases above ~2 k erpm at bench capture rates, so the
        // standing load-angle question — does the commutation frame really
        // ride 75–90° off the estimate/rotor — needs an on-device signal;
        // the earlier vd ← PLL error is recoverable from an eddy-plant sim
        // but this gap is not), vq ← trust-gate state
        // `phase_err_filt × (angle_trustworthy ? +1 : −1)` (2026-07-07
        // torque non-delivery forensics: |i| collapses in ms-scale windows
        // with the commanded voltage collapsing FIRST — the signature of
        // the iq gate, not of saturation — so the gate bit and the
        // lock-quality signal it rides on must sit next to the current
        // columns in one capture; |i_dq| is frame-invariant, so the
        // magnitude stays honest even in this PLL-frame mapping).
        // Currents/vbus stay honest. The wire format is untouched, so host
        // decode/enrichment work unchanged — just read the capture knowing
        // the mapping.
        #[cfg(feature = "obs-debug-telem")]
        if let Some((pll, _raw, vel, err_filt)) = self.phase.debug_observer() {
            let drive_angle = out.angle_rad;
            out.velocity_rad_s = vel;
            out.angle_rad = pll;
            out.vd = crate::foc::angle_difference(drive_angle, pll);
            out.vq = if self.phase.angle_trustworthy() {
                err_filt
            } else {
                -err_filt
            };
        }
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
    /// `Some` while the probe owns the bridge. The PI reset on the way out
    /// still runs when this returns `None`.
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn step_deadshort(&mut self, dt: f32, now_ticks: u64) -> Option<Result<FocOutput, StepError>> {
        let wants_short = self.phase.startup_status().wants_short();
        if self.was_short && !wants_short {
            // Probe short → commutation: the PI state predates the short
            // (amps of drive current that have since decayed into the
            // bridge). Re-integrating from zero is a soft re-engage;
            // slamming the stale voltage into ~zero current spikes |i_dq|
            // (sim: 6 A on the ZD2808 confirm-probe exit at 1.5 A command).
            self.controller.reset();
        }
        self.was_short = wants_short;
        wants_short
            .then(|| self.step_direct_voltage(0.0, 0.0, self.phase.get().angle, dt, now_ticks))
    }

    /// Sensorless restart on trust loss (see [`SENSORLESS_RESTART_AFTER_S`]):
    /// a nonzero drive command with the angle continuously untrustworthy
    /// means the iq gate is coasting the motor and nothing will bring the
    /// estimator back by itself — re-enter the cold start. No-op for
    /// sensored sources and while the startup sequencer already runs
    /// (`begin_cold_start` guards on both).
    #[inline(always)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn maybe_restart_sensorless(&mut self, dt: f32) {
        let drive_cmd = match self.mode {
            ControlMode::CurrentControl { iq_target, .. } => iq_target,
            ControlMode::VelocityControl { target_vel } => target_vel,
            _ => 0.0,
        };
        if drive_cmd != 0.0 && !self.phase.is_starting() {
            if self.phase.angle_trustworthy() {
                self.untrusted_time = 0.0;
            } else {
                self.untrusted_time += dt;
                if self.untrusted_time >= SENSORLESS_RESTART_AFTER_S {
                    self.untrusted_time = 0.0;
                    self.phase
                        .begin_cold_start(if drive_cmd < 0.0 { -1.0 } else { 1.0 });
                }
            }
        } else {
            self.untrusted_time = 0.0;
        }
    }
}

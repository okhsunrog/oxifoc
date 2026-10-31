//! Command-staleness deadman and `FailsafeController` integration.

use super::{ControlMode, FocDriver, StepError};
use crate::foc::clamp_f32;
use crate::foc::controller::FocOutput;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::pwm::PhaseState;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
use crate::motor::failsafe::FailsafeAction;
use crate::motor::failsafe::FailsafeConfig;
use crate::motor::failsafe::FailsafePolicy;
use crate::motor::failsafe::FailsafeTerminal;

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Cache the failsafe tuning (deadman timeout + reaction policy + brake
    /// params). Sane values only — garbage is ignored so a bad config can't
    /// disable the deadman or the brake.
    pub fn set_failsafe(&mut self, cfg: FailsafeConfig) {
        if cfg.is_sane() {
            self.failsafe_cfg = cfg;
        }
    }

    /// Active failsafe config.
    pub fn failsafe_config(&self) -> FailsafeConfig {
        self.failsafe_cfg
    }

    /// Set the over-voltage trip (V) used for the proactive regen-brake
    /// derate (0 = off → rely on the OV fault backstop). From the board
    /// config at boot.
    pub fn set_ov_threshold(&mut self, volts: f32) {
        self.ov_threshold_v = if volts.is_finite() && volts > 0.0 {
            volts
        } else {
            0.0
        };
    }

    /// Stamp a fresh setpoint arrival (called from the ISR when a `SetMode`
    /// is drained). The deadman's "positive affirmation".
    pub fn note_command_tick(&mut self, now_ticks: u64) {
        self.watchdog.note_command(now_ticks);
    }

    /// Current command staleness in µs (time since the last drained
    /// `SetMode`), or `None` in the deadman-exempt standing modes / before
    /// the first stamp. Diagnostic mirror of [`Self::deadman_expired`]'s
    /// input — lets the platform stats report how close each window came.
    pub fn command_staleness_us(&self, now_ticks: u64) -> Option<u64> {
        self.watchdog.staleness_us(self.mode, now_ticks)
    }

    /// Whether the command link has gone stale while the motor is running —
    /// the deadman trigger.
    ///
    /// The drive modes (CurrentControl and velocity/position loops) use the
    /// configured ~150 ms bound — those are what a
    /// vehicle rides on, and the host re-affirms them every 50 ms. Exempt
    /// entirely: Stopped/Coast/Brake — safe standing states: nothing to fail
    /// safe toward, and a parking brake must persist through link loss.
    ///
    /// OpenLoop/DirectVoltage — bench/calibration modes — get the LONG
    /// [`super::BENCH_STALENESS_TIMEOUT_US`] bound instead: on-device detection
    /// dwells up to ~5 s between `SetMode`s (settle waits, sample windows,
    /// the flux-spin collection), which the 150 ms deadman would cut
    /// mid-measurement — but a *hung* measurement must still be cut, because
    /// `DETECTION_ACTIVE` suspends the Layer-1 link gate for exactly these
    /// windows (until 2026-07-06 these modes were exempt entirely, leaving a
    /// detection with BOTH comm failsafes off). Host-driven bench modes are
    /// affirmed every 50 ms like drive modes, so the long bound never fires
    /// while the host lives.
    ///
    /// Also false while the failsafe already runs.
    pub fn deadman_expired(&self, now_ticks: u64) -> bool {
        self.deadman_expired_with(self.command_staleness_us(now_ticks), now_ticks)
    }

    /// [`Self::deadman_expired`] with the staleness precomputed by the
    /// caller — `run_foc_cycle` already needs [`Self::command_staleness_us`]
    /// for its margin meter every cycle, and computing the u64 tick math
    /// twice per ISR is measurable at 20 kHz.
    pub fn deadman_expired_with(&self, staleness_us: Option<u64>, now_ticks: u64) -> bool {
        if self.failsafe_ctrl.is_active() {
            return false;
        }
        self.watchdog.expired(
            self.mode,
            self.phase.is_starting(),
            now_ticks,
            staleness_us,
            self.failsafe_cfg.staleness_timeout_us,
        )
    }

    /// Arm the failsafe sequence (deadman or link-loss). Idempotent. Falls
    /// back to Coast when the current sensor isn't calibrated — the brake
    /// needs the current loop, which refuses to run uncalibrated.
    ///
    /// Also latches `failsafe_latched`: after any failsafe engagement the
    /// host must explicitly acknowledge with a safe mode (Stopped / Coast /
    /// Brake) before a running mode is accepted again — see the gate in
    /// `process_commands`. Defense in depth against a reconnecting host
    /// replaying a stale throttle.
    pub fn enter_failsafe(&mut self) {
        self.failsafe_latched = true;
        self.arm_failsafe_with(self.failsafe_cfg.policy, self.failsafe_cfg.terminal);
    }

    /// User-commanded ramp-into-parking-brake: a `Brake` command above the
    /// standstill gate is substituted with the ControlledStop sequence ending
    /// in `ControlMode::Brake`, instead of being rejected. Same machinery as
    /// the failsafe but **not a failsafe event** — it does not set the re-arm
    /// latch (the user asked for this; no "back to neutral" owed). Returns
    /// false when the current sensor is uncalibrated (can't current-brake;
    /// the caller keeps rejecting).
    pub fn enter_brake_ramp(&mut self) -> bool {
        if !self.current_sensor.is_calibrated() {
            return false;
        }
        if self.failsafe_ctrl.is_active() {
            // A stop is already in progress (failsafe brake) — `arm` would
            // be a no-op and silently drop the user's parking-brake intent.
            // Adopt the terminal; the running sequence finishes into Brake.
            self.failsafe_ctrl.set_terminal(FailsafeTerminal::ParkBrake);
            return true;
        }
        self.arm_failsafe_with(FailsafePolicy::ControlledStop, FailsafeTerminal::ParkBrake);
        true
    }

    /// Enable the bridge outputs if the current topology left them off
    /// (Stopped disables, Coast/calibration float every phase).
    pub(super) fn ensure_bridge_enabled(&mut self) {
        if self.pwm_off || matches!(self.mode, ControlMode::Stopped | ControlMode::Coast) {
            self.pwm.enable();
            self.pwm_off = false;
        }
    }

    /// Common arm body for the failsafe and the user brake ramp. Idempotent
    /// while a sequence is active. Falls back to Coast when the current
    /// sensor isn't calibrated — the brake needs the current loop, which
    /// refuses to run uncalibrated.
    fn arm_failsafe_with(&mut self, policy: FailsafePolicy, terminal: FailsafeTerminal) {
        if self.failsafe_ctrl.is_active() {
            return;
        }
        let policy = if self.current_sensor.is_calibrated() {
            policy
        } else {
            FailsafePolicy::Coast
        };
        // A brake ramp drives the bridge through duties: from Stopped or
        // Coast (a user Brake at speed while coasting) the channels are off.
        if !matches!(policy, FailsafePolicy::Coast) {
            self.ensure_bridge_enabled();
        }
        // Bumpless: seed the ramp from the q-current currently commanded
        // (the velocity loop's last PI output is what the current loop was
        // just driven with). Not OpenLoop's `current`: it is applied along
        // the open-loop angle, not the rotor's q axis — seeding it would
        // turn a load-angle-dependent current into full q torque in the
        // estimate's frame, sign included. Zero is the honest seed there.
        let current_iq = match self.mode {
            ControlMode::CurrentControl { iq_target, .. } => iq_target,
            ControlMode::VelocityControl { .. } | ControlMode::PositionControl { .. } => {
                self.velocity_loop.last_iq()
            }
            _ => 0.0,
        };
        // Bound the seed: mode payloads are finite-checked at the command
        // boundary but not clamped, and the failsafe slews from the seed at
        // a fixed rate — an absurd value must not stretch its hand-over
        // toward forever. The physically delivered current was never above
        // the limit anyway, so clamping keeps it bumpless. NaN (nothing
        // upstream should produce one, but the failsafe must not hinge on
        // that) seeds from zero.
        let seed_bound = if self.current_limits.max_current_a > 0.0 {
            self.current_limits.max_current_a
        } else {
            // Unlimited limits are a sim/test setup; still keep it finite.
            1_000.0
        };
        let current_iq = if current_iq.is_finite() {
            clamp_f32(current_iq, -seed_bound, seed_bound)
        } else {
            0.0
        };
        // One-shot (the is_active guard above): the WHY is logged at the
        // detector sites in state.rs; this is the WHAT actually armed
        // (policy may have been downgraded to Coast=0 above).
        warn!(
            "stop sequence armed: policy={} terminal={} seed_iq={}",
            policy as u8, terminal as u8, current_iq
        );
        self.failsafe_ctrl.arm(current_iq, policy, terminal);
    }

    /// Clear the failsafe sequence (a fresh command re-armed control, or a
    /// fault took over). Does NOT clear the re-arm latch — only an explicit
    /// safe-mode command does (see `set_mode`).
    pub fn failsafe_reset(&mut self) {
        self.failsafe_ctrl.reset();
    }

    /// Emergency de-energize: latch Stopped AND float the bridge *now*.
    ///
    /// `set_mode(Stopped)` only disables PWM on the *next* `step()` that
    /// runs in Stopped mode (the safe-off lives in step()'s Stopped arm).
    /// The Kill path returns without ever calling `step()`, and a
    /// software-only Kill (the integrated OverVoltage trip) has no hardware
    /// break to float the outputs for it — so on that path the timer would
    /// otherwise hold its last commanded duty and keep the phases
    /// energized. This cuts the gate drive immediately. `disable()` runs
    /// last so it wins over any phase re-enable inside `set_mode`.
    pub fn safe_off(&mut self) {
        self.cancel_current_offset_calibration();
        self.set_mode(ControlMode::Stopped);
        self.pwm.disable();
        self.pwm_off = true;
    }

    /// Operator-requested emergency stop: float immediately and require an
    /// explicit safe neutral command before any active mode can run again.
    /// Kept inside the existing motor endpoint to avoid another socket's
    /// flash/RAM cost.
    pub fn emergency_stop(&mut self) {
        self.failsafe_latched = true;
        self.failsafe_reset();
        self.safe_off();
    }

    /// Whether the failsafe is currently carrying the motor.
    pub fn failsafe_active(&self) -> bool {
        self.failsafe_ctrl.is_active()
    }

    /// Whether a failsafe engagement is awaiting an explicit safe-mode
    /// acknowledgement (Stopped / Coast / Brake) before running modes are
    /// accepted again. Checked by the `process_commands` gate.
    pub fn failsafe_latched(&self) -> bool {
        self.failsafe_latched
    }

    /// Release the post-failsafe re-arm latch.
    ///
    /// Must only be called from the host-command path when an explicit safe
    /// mode (Stopped/Coast/Brake) arrives — that command IS the "throttle
    /// back to neutral" acknowledgement. Internal stop paths (the fault-kill
    /// `safe_off`, step errors) go through [`set_mode`](Self::set_mode)
    /// directly and deliberately keep the latch: a failsafe brake that
    /// itself trips OC/OV mid-stop must still demand the acknowledgement,
    /// or a host's `faults clear` followed by its stale throttle replay
    /// relaunches the board — exactly what the latch exists to prevent.
    pub fn acknowledge_failsafe(&mut self) {
        self.failsafe_latched = false;
    }

    /// Run one cycle of the active failsafe sequence (see
    /// [`crate::motor::failsafe`]). `Drive` actions are routed back through
    /// the normal current-control path so the current-limit clamp and the
    /// measured-overcurrent trip still apply — the brake never bypasses
    /// protection. The terminal `Stop` cuts PWM and clears the controller.
    pub(super) fn step_failsafe(
        &mut self,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, StepError> {
        let omega_e = self.phase.get().velocity;
        let angle_trustworthy = self.phase.angle_trustworthy();
        let action = self.failsafe_ctrl.step(
            omega_e,
            dt,
            &self.failsafe_cfg,
            self.current_limits.max_brake_a,
            self.vbus,
            self.ov_threshold_v,
            angle_trustworthy,
        );
        match action {
            FailsafeAction::Drive {
                id_target,
                iq_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            FailsafeAction::Stop => {
                self.pwm.disable();
                self.controller.reset();
                self.mode = ControlMode::Stopped;
                self.failsafe_ctrl.reset();
                self.update_phase_with_applied_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
            // Stop with the parking-brake terminal: hand over to
            // ControlMode::Brake (see step_park_brake), starting as the
            // plain low-side short.
            FailsafeAction::EngageBrake => {
                self.pwm
                    .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
                self.controller.reset();
                self.park_iq = None;
                self.mode = ControlMode::Brake;
                self.failsafe_ctrl.reset();
                self.update_phase_with_applied_voltage(0.0, 0.0, 0.0, 0.0, dt, now_ticks);
                Ok(FocOutput::default())
            }
        }
    }
}

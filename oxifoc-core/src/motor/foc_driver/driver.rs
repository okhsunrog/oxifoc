//! Construction, mode entry, and the accessors platforms call every cycle.

use super::{
    ActuationModel, CommandWatchdog, ControlMode, CurrentLimits, FocDriver, ProtectionMonitor,
};
use crate::foc::controller::FocController;
use crate::foc::current_offset::CurrentOffsetCalibrator;
use crate::foc::phase::PhaseInput;
use crate::foc::phase::PhaseProvider;
use crate::foc::phase_voltage::ObserverVoltage;
use crate::foc::phase_voltage::PhaseVoltageSense;
use crate::foc::phase_voltage::observer_voltage_source;
use crate::foc::position::Position;
use crate::foc::position::PositionLoopConfig;
use crate::foc::position::PositionTracker;
use crate::foc::pwm::PhasePwm;
use crate::foc::pwm::SvpwmModulator;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
use crate::foc::velocity::VelocityLoop;
use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::failsafe::FailsafeConfig;
use crate::motor::failsafe::FailsafeController;

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Create a new FOC driver
    ///
    /// # Arguments
    /// * `controller` - FOC controller instance
    /// * `pwm` - PWM output
    /// * `current_sensor` - Current sensor (should be calibrated)
    /// * `phase` - Phase provider (manages angle sources)
    /// * `dt` - Control loop period in seconds (from `MotorPwmConfig::dt_s()`)
    pub fn new(
        mut controller: FocController<SvpwmModulator, S>,
        pwm: P,
        current_sensor: C,
        phase: Phase,
        dt: f32,
    ) -> Self {
        let vbus = controller.vbus();
        // The board's duty cap, so the controller never commands a vector
        // the bridge would clip (see FocController::set_duty_cap).
        if pwm.max_duty() > 0 {
            controller.set_duty_cap(f32::from(pwm.duty_limit()) / f32::from(pwm.max_duty()));
        }
        Self {
            controller,
            pwm,
            current_sensor,
            phase,
            mode: ControlMode::Stopped,
            vbus,
            dt,
            current_limits: CurrentLimits::default(),
            open_loop_angle: 0.0,
            actuation: ActuationModel::default(),
            watchdog: CommandWatchdog::default(),
            pwm_off: false,
            was_short: false,
            untrusted_time: 0.0,
            stopped_housekeeping: 0,
            failsafe_cfg: FailsafeConfig::default(),
            failsafe_ctrl: FailsafeController::new(),
            failsafe_latched: false,
            ov_threshold_v: 0.0,
            velocity_loop: VelocityLoop::new(VelocityLoopConfig::default()),
            position: PositionTracker::default(),
            position_cfg: PositionLoopConfig::default(),
            position_target: Position::default(),
            last_iq_meas: 0.0,
            park_iq: None,
            bus_mod_q_filt: 0.0,
            protection: ProtectionMonitor::default(),
            phase_voltage: None,
            vphase_raw: None,
            current_offset: CurrentOffsetCalibrator::new(),
            current_offset_outcome: None,
        }
    }

    /// Install the phase-voltage converter (boot config). Pass
    /// `PhaseVoltageSense::from_board(&BOARD)` — `None` on boards without phase
    /// sensing, which leaves the observer on the commanded-voltage path.
    pub fn set_phase_voltage_sense(&mut self, pv: Option<PhaseVoltageSense>) {
        self.phase_voltage = pv;
    }

    /// Mutable access to the phase-voltage converter (e.g. to apply calibrated
    /// undriven offsets). `None` on boards without phase sensing.
    pub fn phase_voltage_sense_mut(&mut self) -> Option<&mut PhaseVoltageSense> {
        self.phase_voltage.as_mut()
    }

    /// Push this cycle's raw phase-terminal ADC counts (from the ADC snapshot).
    /// Call from the ISR before `run_foc_cycle`/`step`; `None` clears them.
    pub fn set_phase_voltage_raw(&mut self, raw: Option<[u16; 3]>) {
        self.vphase_raw = raw;
    }

    /// Set the actuation delay `d` (PWM periods, clamped to
    /// `0..=MAX_ACTUATION_DELAY_CYCLES`; default
    /// [`DEFAULT_ACTUATION_DELAY_CYCLES`]): the time from a current sample
    /// until the duties computed from it start acting — a hardware
    /// property of the timer/ADC/ISR arrangement. Both pipeline
    /// compensations derive from it:
    /// - the output vector is rotated by `(d + ½)·ω·T`: it acts from
    ///   `t + d·T` for one period, centered `(d + ½)·T` after the sample
    ///   the Park angle describes (the Park angle itself stays at the
    ///   sample — see `actuation_advance_must_not_displace_current_vector`);
    /// - the estimators integrate the voltage that acted over the last
    ///   sampling interval (see [`Self::applied_voltage`]).
    ///
    /// These used to be two unrelated conventions — a fixed 1.0-cycle
    /// advance (= d ½) and a previous-command pairing (= d 0) — so no
    /// hardware delay satisfied both, and the back-EMF observer's angle
    /// ran up to a period early.
    pub fn set_actuation_delay(&mut self, cycles: f32) {
        self.actuation.set_delay(cycles);
    }

    /// Output-vector advance for the configured actuation delay.
    pub(super) fn actuation_advance(&self, velocity: f32, dt: f32) -> f32 {
        self.actuation.angle_advance(velocity, dt)
    }

    /// Voltage that acted over the interval ending at the current sample.
    fn applied_voltage(&self) -> (f32, f32) {
        self.actuation.applied_voltage()
    }

    /// Feed the phase provider with this cycle's measurements and the
    /// causally-matching voltage, then record this cycle's command.
    ///
    /// Two voltage sources, chosen by [`observer_voltage_source`]:
    /// - **Commanded** (default; every board without phase filters while
    ///   driving, and every board with no sensing): the voltage that acted
    ///   while these currents built up, reconstructed from the command
    ///   history and the actuation delay ([`Self::applied_voltage`]; the
    ///   command computed *this* cycle has not acted yet).
    /// - **Measured**: the phase terminals sampled *this* cycle (concurrent
    ///   with the currents, so no pipeline delay). Used when the bridge is
    ///   high-Z on a sensing board (coasting back-EMF) — or while driving on a
    ///   board with RC phase filters. Lets the observer track a free-spinning
    ///   rotor for flying start (mirrors MESC `MOTOR_STATE_TRACKING` and VESC's
    ///   released-motor observer).
    #[allow(clippy::too_many_arguments)] // per-cycle scalars, same as the step fns
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub(super) fn update_phase_with_applied_voltage(
        &mut self,
        v_alpha_new: f32,
        v_beta_new: f32,
        i_alpha: f32,
        i_beta: f32,
        dt: f32,
        now_ticks: u64,
    ) {
        let bridge_driven = !self.mode.is_high_z();
        let (v_alpha, v_beta) =
            match observer_voltage_source(self.phase_voltage.as_ref(), bridge_driven) {
                ObserverVoltage::Measured => match (self.phase_voltage.as_ref(), self.vphase_raw) {
                    (Some(pv), Some(raw)) => pv.alpha_beta(raw),
                    // Policy said Measured but no sample yet (e.g. ADC not wired
                    // this cycle) — fall back to the commanded path.
                    _ => self.applied_voltage(),
                },
                ObserverVoltage::Commanded => self.applied_voltage(),
            };
        self.phase.update(
            &PhaseInput {
                v_alpha,
                v_beta,
                i_alpha,
                i_beta,
                dt,
            },
            now_ticks,
        );
        self.actuation.record_command(v_alpha_new, v_beta_new);
    }

    /// Get the control loop period (dt) in seconds
    pub fn dt(&self) -> f32 {
        self.dt
    }

    /// Set current limits.
    pub fn set_current_limits(&mut self, limits: CurrentLimits) {
        self.current_limits = limits;
    }

    /// Get current limits.
    pub fn current_limits(&self) -> &CurrentLimits {
        &self.current_limits
    }

    /// Tune the cruise velocity loop (gains + accel limit). Sane values
    /// only — garbage is ignored. Takes effect immediately; the loop state
    /// (ramp/integrator) is preserved, so retuning mid-ride is bumpy only
    /// to the extent the gains differ.
    pub fn set_velocity_config(&mut self, cfg: VelocityLoopConfig) {
        self.velocity_loop.set_config(cfg);
    }

    /// Active velocity-loop tuning.
    pub fn velocity_config(&self) -> VelocityLoopConfig {
        self.velocity_loop.config()
    }

    /// Apply position-loop tuning (sane values only; see
    /// [`PositionLoopConfig::is_sane`]).
    pub fn set_position_config(&mut self, cfg: PositionLoopConfig) {
        if cfg.is_sane() {
            self.position_cfg = cfg;
        }
    }

    /// Active position-loop tuning.
    pub fn position_config(&self) -> PositionLoopConfig {
        self.position_cfg
    }

    /// The rotor's unwrapped electrical position (0 at boot) — the frame
    /// of `PositionControl` targets.
    pub fn position(&self) -> Position {
        self.position.position()
    }

    /// Set control mode
    pub fn set_mode(&mut self, mode: ControlMode) {
        // A motor-mode command supersedes maintenance. Cancellation floats
        // the bridge immediately; a non-Stopped mode is additionally rejected
        // by the command gate while maintenance is active.
        self.cancel_current_offset_calibration();
        // Any explicit mode set is an authoritative override: a fresh host
        // command (re-arm after link loss) or a fault-stop both cancel an
        // in-progress failsafe brake. (The failsafe's own terminal Stop sets
        // `self.mode` directly, not through here, so it doesn't self-cancel.)
        // The re-arm latch is NOT touched here: internal stops (fault kill,
        // step error) route through this method too, and only the host's
        // explicit safe-mode command may release it — see
        // [`acknowledge_failsafe`](Self::acknowledge_failsafe).
        // A stop sequence leaves `self.mode` alone while it runs, so the
        // cruise loop sat idle under it: its state predates the stop.
        let velocity_loop_live = matches!(
            self.mode,
            ControlMode::VelocityControl { .. } | ControlMode::PositionControl { .. }
        ) && !self.failsafe_ctrl.is_active();
        self.failsafe_ctrl.reset();

        // Re-enable the outputs for any mode that drives the bridge through
        // duties. Coast/Stopped/calibration float or disable the channels,
        // and `set_duties` never re-enables them — gating this on leaving
        // Stopped alone left Coast → CurrentControl writing duties into a
        // disabled bridge forever (no torque, PI winding up).
        if !matches!(
            mode,
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        ) {
            self.ensure_bridge_enabled();
        }

        // Entering velocity or position mode from another mode — or from
        // under a stop sequence — re-arms the loop bumpless: reference ramp
        // seeded at the measured velocity, integrator cleared. A retarget
        // *within* either (or a switch between the two, which share the
        // loop) keeps the loop state (the ramp carries the reference to the
        // new target). Resuming mid brake ramp with the pre-brake state
        // drove the rotor back toward the speed it was braked from.
        if matches!(
            mode,
            ControlMode::VelocityControl { .. } | ControlMode::PositionControl { .. }
        ) && !velocity_loop_live
        {
            let omega = self.phase.get().velocity;
            self.velocity_loop.reset(omega);
        }
        if let ControlMode::PositionControl { target_pos } = mode {
            self.position_target = Position::from_rad(target_pos);
        }

        // Apply PI gains override if provided (detection uses this)
        if let ControlMode::OpenLoop {
            pi_gains: Some((kp, ki)),
            ..
        } = mode
        {
            self.controller.id_pi.set_gains(kp, ki);
            self.controller.iq_pi.set_gains(kp, ki);
            self.controller.reset();
        }

        // Cold-start the sensorless commutation when leaving a non-driving mode
        // for a torque/velocity command: a pure back-EMF observer can't
        // commutate from standstill, so begin the deadshort→ramp→confirm
        // sequence.
        // No-op for sensored sources or an already-tracking observer (see
        // PhaseProvider::begin_cold_start). Direction = sign of the command.
        let was_idle = matches!(
            self.mode,
            ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
        );
        let start_dir = match mode {
            ControlMode::CurrentControl { iq_target, .. } => iq_target,
            ControlMode::VelocityControl { target_vel } => target_vel,
            _ => 0.0,
        };
        if was_idle && start_dir != 0.0 {
            self.phase
                .begin_cold_start(if start_dir < 0.0 { -1.0 } else { 1.0 });
        }

        // The parking brake always starts as the plain short.
        self.park_iq = None;
        self.mode = mode;
    }

    /// Get current control mode
    pub fn mode(&self) -> ControlMode {
        self.mode
    }

    /// Update bus voltage
    pub fn set_vbus(&mut self, vbus: f32) {
        self.controller.set_vbus(vbus);
        self.vbus = vbus;
    }

    /// Get bus voltage
    pub fn vbus(&self) -> f32 {
        self.vbus
    }

    /// Get mutable reference to current sensor (for calibration)
    pub fn current_sensor_mut(&mut self) -> &mut C {
        &mut self.current_sensor
    }

    /// Get reference to current sensor
    pub fn current_sensor(&self) -> &C {
        &self.current_sensor
    }

    /// Get mutable reference to phase provider
    pub fn phase_mut(&mut self) -> &mut Phase {
        &mut self.phase
    }

    /// Get reference to phase provider
    pub fn phase(&self) -> &Phase {
        &self.phase
    }

    /// Get mutable reference to FOC controller (for tuning)
    pub fn controller_mut(&mut self) -> &mut FocController<SvpwmModulator, S> {
        &mut self.controller
    }

    /// Get reference to FOC controller
    pub fn controller(&self) -> &FocController<SvpwmModulator, S> {
        &self.controller
    }

    /// Get reference to the PWM output (tests / diagnostics).
    pub fn pwm(&self) -> &P {
        &self.pwm
    }
}

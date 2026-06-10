//! Reusable FOC motor driver
//!
//! This module provides a high-level FOC driver that integrates:
//! - FOC controller (Clarke/Park transforms, PI control, SVPWM)
//! - Current sensing
//! - Phase provider (angle sensing/estimation)
//! - PWM output
//! - Control mode management
//! - Current limiting (target clamping + measured overcurrent protection)
//!
//! Platform code just needs to provide trait implementations and call `step()`.

use crate::foc::controller::{FocController, FocOutput};
use crate::foc::phase::{PhaseInput, PhaseProvider};
use crate::foc::pwm::{PhasePwm, PhaseState, SvpwmModulator};
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::{LibmSinCos, SinCos};
use crate::motor::six_step;

// Re-export ControlMode from types (single source of truth)
pub use crate::types::ControlMode;

/// Current limiting configuration for the FOC driver.
///
/// Two layers of protection:
/// 1. **Target clamp**: limits what the PI controller is asked to do (prevents
///    absurd commands). Uses circular clamp with d-axis priority.
/// 2. **Measured overcurrent**: checks actual dq current magnitude after the
///    FOC step. If it exceeds `overcurrent_threshold`, PWM is disabled and
///    an error is returned. This is the software equivalent of hardware
///    overcurrent protection for boards that lack it.
#[derive(Clone, Copy, Debug)]
pub struct CurrentLimits {
    /// Maximum current target magnitude (A). The PI controller will never
    /// be asked to produce more than this. Set from BoardConfig or
    /// CurrentLimitsConfig. 0 = no limit.
    pub max_current_a: f32,
    /// Hard overcurrent threshold on measured current (A). If actual
    /// sqrt(id² + iq²) exceeds this, PWM is immediately disabled.
    /// Typically set to 1.2-1.5× max_current_a. 0 = no limit.
    pub overcurrent_threshold_a: f32,
}

impl Default for CurrentLimits {
    fn default() -> Self {
        Self {
            max_current_a: 0.0,
            overcurrent_threshold_a: 0.0,
        }
    }
}

impl CurrentLimits {
    /// Create current limits from a maximum current value.
    /// Sets overcurrent threshold to 1.3× the max current.
    pub fn from_max_current(max_a: f32) -> Self {
        Self {
            max_current_a: max_a,
            overcurrent_threshold_a: max_a * 1.3,
        }
    }

    /// Build limits from a host-written config, clamped to the board's
    /// hardware ceiling.
    ///
    /// The stored config can lower the limits but never raise them above
    /// what the board hardware tolerates; zero/negative config values mean
    /// "not set" and fall back to the board defaults — a config must not be
    /// able to switch protection off.
    ///
    /// # Arguments
    /// * `cfg_max_iq_a` - configured target-current limit (A)
    /// * `cfg_max_phase_a` - configured instantaneous overcurrent threshold (A)
    /// * `hw_max_a` - board hardware phase-current ceiling (A)
    pub fn from_config_clamped(cfg_max_iq_a: f32, cfg_max_phase_a: f32, hw_max_a: f32) -> Self {
        let board = Self::from_max_current(hw_max_a);
        Self {
            max_current_a: if cfg_max_iq_a > 0.0 {
                cfg_max_iq_a.min(board.max_current_a)
            } else {
                board.max_current_a
            },
            overcurrent_threshold_a: if cfg_max_phase_a > 0.0 {
                cfg_max_phase_a.min(board.overcurrent_threshold_a)
            } else {
                board.overcurrent_threshold_a
            },
        }
    }

    /// Limits for boot: the stored config (clamped) when present, board
    /// defaults otherwise.
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&crate::storage::CurrentLimitsConfig>, hw_max_a: f32) -> Self {
        match cfg {
            Some(c) => Self::from_config_clamped(c.max_iq_a, c.max_phase_current_a, hw_max_a),
            None => Self::from_max_current(hw_max_a),
        }
    }

    /// Check if current limiting is enabled
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.max_current_a > 0.0
    }

    /// Clamp id/iq targets to the current limit circle.
    ///
    /// Uses d-axis priority: id is clamped first to the full budget,
    /// then iq gets the remaining circular margin. This is correct for
    /// IPM motors where id is used for field weakening.
    #[inline]
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn clamp_targets(&self, id_target: f32, iq_target: f32) -> (f32, f32) {
        if !(self.max_current_a > 0.0) {
            return (id_target, iq_target);
        }
        let limit = self.max_current_a;
        // D-axis priority: clamp id first
        let id = crate::foc::clamp_f32(id_target, -limit, limit);
        // Q-axis gets the remaining circular budget
        let iq_budget_sq = limit * limit - id * id;
        let iq_budget = if iq_budget_sq > 0.0 {
            // .max(0.0) lets the compiler prove -iq_budget <= iq_budget,
            // eliminating clamp's panic branch (sqrtf can't return NaN here,
            // but LLVM can't prove it).
            libm::sqrtf(iq_budget_sq).max(0.0)
        } else {
            0.0
        };
        let iq = crate::foc::clamp_f32(iq_target, -iq_budget, iq_budget);
        (id, iq)
    }

    /// Check if measured current exceeds the overcurrent threshold.
    /// Returns true if overcurrent is detected.
    #[inline]
    pub fn is_overcurrent(&self, id: f32, iq: f32) -> bool {
        if self.overcurrent_threshold_a <= 0.0 {
            return false;
        }
        let mag_sq = id * id + iq * iq;
        let threshold_sq = self.overcurrent_threshold_a * self.overcurrent_threshold_a;
        mag_sq > threshold_sq
    }
}

/// Reusable FOC driver
///
/// # Type Parameters
/// * `P` - PWM output implementing `PhasePwm`
/// * `C` - Current sensor implementing `CurrentSensor`
/// * `Phase` - Phase provider implementing `PhaseProvider`
///
/// # Example (platform code)
/// ```rust,ignore
/// use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
/// use oxifoc_core::foc::pwm::MotorPwmConfig;
///
/// static FOC_DRIVER: Mutex<NoopRawMutex, RefCell<Option<FocDriver<MotorPwm, CurrentSense, PhaseManager<HallSensor>>>>> =
///     Mutex::new(RefCell::new(None));
///
/// // During init - dt comes from PWM config:
/// let motor_pwm = MotorPwm::new(resources, config::PWM_CONFIG);
/// let driver = FocDriver::new(
///     controller,
///     motor_pwm,
///     current_sensor,
///     phase,
///     config::PWM_CONFIG.dt_s(),
/// );
///
/// #[interrupt]
/// fn ADC1_2() {
///     // Run FOC - dt is stored in driver
///     FOC_DRIVER.lock(|cell| {
///         if let Some(driver) = cell.borrow_mut().as_mut() {
///             if let Ok(telem) = driver.step(now_ticks) {
///                 MOTOR_TELEM.set(telem);
///             }
///         }
///     });
/// }
/// ```
pub struct FocDriver<P, C, Phase, S: SinCos = LibmSinCos>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
{
    /// FOC controller
    controller: FocController<SvpwmModulator, S>,
    /// PWM output
    pwm: P,
    /// Current sensor
    current_sensor: C,
    /// Phase provider (manages angle sources)
    phase: Phase,
    /// Current control mode
    mode: ControlMode,
    /// Bus voltage (V)
    vbus: f32,
    /// Control loop period in seconds (1/pwm_freq)
    dt: f32,
    /// Current limiting configuration
    current_limits: CurrentLimits,
    /// Accumulated angle for open-loop velocity mode
    open_loop_angle: f32,
}

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
        controller: FocController<SvpwmModulator, S>,
        pwm: P,
        current_sensor: C,
        phase: Phase,
        dt: f32,
    ) -> Self {
        let vbus = controller.vbus();
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
        }
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

    /// Set control mode
    ///
    /// When leaving SixStep mode, re-enables all PWM channels that may
    /// have been disabled (floated) during six-step commutation.
    pub fn set_mode(&mut self, mode: ControlMode) {
        let was_stopped = matches!(self.mode, ControlMode::Stopped);
        let will_be_active = !matches!(mode, ControlMode::Stopped);

        if matches!(self.mode, ControlMode::SixStep { .. })
            && !matches!(mode, ControlMode::SixStep { .. })
        {
            // Re-enable all phases so set_duties() in FOC modes works
            self.pwm
                .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
        }

        // Re-enable PWM outputs when leaving Stopped mode
        if was_stopped && will_be_active {
            self.pwm.enable();
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
    pub fn step(&mut self, now_ticks: u64) -> Result<FocOutput, &'static str> {
        let dt = self.dt;
        match self.mode {
            ControlMode::Stopped => {
                // Safe-off: all low-side ON (brake) or all OFF depending on
                // platform.  `disable()` is the platform's emergency-stop.
                self.pwm.disable();
                self.phase.update(
                    &PhaseInput {
                        dt,
                        ..Default::default()
                    },
                    now_ticks,
                );
                Ok(FocOutput::default())
            }
            ControlMode::CurrentControl {
                iq_target,
                id_target,
            } => self.step_current_control(iq_target, id_target, dt, now_ticks),
            ControlMode::VelocityControl { .. } => {
                // TODO: Implement velocity PI controller
                Err("Velocity control not implemented")
            }
            ControlMode::PositionControl { .. } => {
                // TODO: Implement position PI controller
                Err("Position control not implemented")
            }
            ControlMode::OpenLoop {
                angle_rad,
                current,
                velocity_rad_s,
                ..
            } => self.step_open_loop(angle_rad, current, velocity_rad_s, dt, now_ticks),
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                self.step_direct_voltage(vd, vq, angle_rad, dt, now_ticks)
            }
            ControlMode::Coast => {
                // High-Z: all FETs off, motor spins freely.
                use super::super::foc::pwm::PhaseState;
                self.pwm.set_phase_states([
                    PhaseState::Float,
                    PhaseState::Float,
                    PhaseState::Float,
                ]);
                self.controller.reset();
                self.phase.update(
                    &PhaseInput {
                        dt,
                        ..Default::default()
                    },
                    now_ticks,
                );
                Ok(FocOutput::default())
            }
            ControlMode::SixStep { duty } => self.step_six_step(duty, dt, now_ticks),
        }
    }

    /// Execute current control step
    fn step_current_control(
        &mut self,
        iq_target: f32,
        id_target: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Layer 1: Clamp current targets (prevents absurd commands)
        let (id_target, iq_target) = self.current_limits.clamp_targets(id_target, iq_target);

        // Get phase from provider (uses previous update's estimate)
        let phase_out = self.phase.get();
        let angle_rad = phase_out.angle;

        // HFI carrier for this cycle (zero for non-HFI sources). Must be
        // read between get() and update(): the estimator demodulates the
        // currents fed to update() against this exact carrier sample.
        let (vd_inject, vq_inject) = self.phase.injection();

        // Read currents and run FOC controller
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        let out = self.controller.step_with_injection(
            currents, angle_rad, id_target, iq_target, vd_inject, vq_inject, max_duty, dt,
        );

        // Layer 2: Check measured current against hard overcurrent limit
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err("Overcurrent: measured current exceeds limit");
        }

        // Set PWM duties and feed to current sensor for next-cycle reconstruction
        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Update phase provider for next step (feeds observer if present)
        self.phase.update(
            &PhaseInput {
                v_alpha: out.v_alpha,
                v_beta: out.v_beta,
                i_alpha: out.i_alpha,
                i_beta: out.i_beta,
                dt,
            },
            now_ticks,
        );

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
    fn step_open_loop(
        &mut self,
        angle_rad: f32,
        current: f32,
        velocity_rad_s: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Advance angle if velocity is set, otherwise use commanded angle
        let angle = if velocity_rad_s != 0.0 {
            self.open_loop_angle += velocity_rad_s * dt;
            self.open_loop_angle %= core::f32::consts::TAU;
            if self.open_loop_angle < 0.0 {
                self.open_loop_angle += core::f32::consts::TAU;
            }
            self.open_loop_angle
        } else {
            self.open_loop_angle = angle_rad;
            angle_rad
        };

        // Clamp open-loop current to the target limit
        let current = if self.current_limits.max_current_a > 0.0 {
            crate::foc::clamp_f32(
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
        let out = self
            .controller
            .step(currents, angle, id_target, iq_target, max_duty, dt);

        // Check measured overcurrent
        if self.current_limits.is_overcurrent(out.id, out.iq) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err("Overcurrent: measured current exceeds limit");
        }

        // Set PWM duties and feed to current sensor for next-cycle reconstruction
        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Update phase provider (for sensor tracking, even in open-loop)
        self.phase.update(
            &PhaseInput {
                v_alpha: out.v_alpha,
                v_beta: out.v_beta,
                i_alpha: out.i_alpha,
                i_beta: out.i_beta,
                dt,
            },
            now_ticks,
        );

        Ok(out)
    }

    /// Execute direct voltage step — no PI control.
    ///
    /// Applies the given dq voltages via `FocController::apply_dq`, reads
    /// currents for telemetry, and feeds the phase observer. Used for
    /// measurement modes (HFI inductance) and direct voltage control.
    fn step_direct_voltage(
        &mut self,
        vd: f32,
        vq: f32,
        angle_rad: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, &'static str> {
        let max_duty = self.pwm.max_duty();
        let mut out = self.controller.apply_dq(vd, vq, angle_rad, max_duty);

        self.pwm.set_duties(out.duties);
        self.current_sensor.update_duties(out.duties);

        // Read currents for telemetry and phase observer
        if self.current_sensor.is_calibrated() {
            let currents = self.current_sensor.read_currents();
            out.ia = currents.0;
            out.ib = currents.1;
            out.ic = currents.2;
            let (i_alpha, i_beta) = crate::foc::transforms::clarke(currents.0, currents.1);
            out.i_alpha = i_alpha;
            out.i_beta = i_beta;
            let (sin_a, cos_a) = S::sin_cos(angle_rad);
            let (id, iq) = crate::foc::transforms::park(i_alpha, i_beta, sin_a, cos_a);
            out.id = id;
            out.iq = iq;

            // No PI loop reins the current in here — the commanded voltage
            // is applied verbatim — so the measured check is the only
            // software protection this mode has.
            if self.current_limits.is_overcurrent(out.id, out.iq) {
                self.pwm.disable();
                self.controller.reset();
                self.mode = ControlMode::Stopped;
                return Err("Overcurrent: measured current exceeds limit");
            }
        }

        self.phase.update(
            &PhaseInput {
                v_alpha: out.v_alpha,
                v_beta: out.v_beta,
                i_alpha: out.i_alpha,
                i_beta: out.i_beta,
                dt,
            },
            now_ticks,
        );

        Ok(out)
    }

    /// Execute six-step (trapezoidal) commutation step
    ///
    /// Pure voltage-mode drive: no current loop, no Clarke/Park transforms.
    /// Derives commutation sector from the PhaseProvider angle and applies
    /// the appropriate phase states via `set_phase_states()`.
    ///
    /// Does NOT require current sensor calibration, making it suitable
    /// for initial board bringup.
    fn step_six_step(
        &mut self,
        duty: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocOutput, &'static str> {
        // Get current electrical angle from phase provider
        let phase_out = self.phase.get();
        let sector = six_step::angle_to_sector(phase_out.angle);

        // Duty sign determines direction
        let forward = duty >= 0.0;
        let duty_abs = crate::foc::clamp_f32(duty.abs(), 0.0, 1.0);
        let raw_duty = (duty_abs * self.pwm.max_duty() as f32) as u16;

        // Generate and apply phase states
        let states = six_step::commutate(sector, raw_duty, forward);
        self.pwm.set_phase_states(states);
        // Feed duties to current sensor for reconstruction (Float/Low → 0)
        let duties = states.map(|s| match s {
            PhaseState::Pwm(d) => d,
            PhaseState::Low | PhaseState::Float => 0,
        });
        self.current_sensor.update_duties(duties);

        // Update phase provider (keep sensor tracking active)
        self.phase.update(
            &PhaseInput {
                dt,
                ..Default::default()
            },
            now_ticks,
        );

        // Read currents opportunistically (if sensor is calibrated)
        let (ia, ib, ic) = if self.current_sensor.is_calibrated() {
            self.current_sensor.read_currents()
        } else {
            (0.0, 0.0, 0.0)
        };

        // Six-step has no current loop at all; check the measured magnitude
        // in αβ (same magnitude as dq — Park preserves it).
        let (i_alpha, i_beta) = crate::foc::transforms::clarke(ia, ib);
        if self.current_limits.is_overcurrent(i_alpha, i_beta) {
            self.pwm.disable();
            self.controller.reset();
            self.mode = ControlMode::Stopped;
            return Err("Overcurrent: measured current exceeds limit");
        }

        Ok(FocOutput {
            ia,
            ib,
            ic,
            angle_rad: phase_out.angle,
            duties,
            ..Default::default()
        })
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
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockPwm {
        duties: [u16; 3],
    }

    impl PhasePwm for MockPwm {
        fn max_duty(&self) -> u16 {
            1000
        }
        fn set_duties(&mut self, duties: [u16; 3]) {
            self.duties = duties;
        }
    }

    struct MockCurrentSensor {
        currents: (f32, f32, f32),
    }

    impl CurrentSensor for MockCurrentSensor {
        fn read_currents(&self) -> (f32, f32, f32) {
            self.currents
        }
        fn read_raw(&self) -> (u16, u16, u16) {
            (0, 0, 0)
        }
        fn is_calibrated(&self) -> bool {
            true
        }
        fn get_offsets(&self) -> (f32, f32, f32) {
            (0.0, 0.0, 0.0)
        }
    }

    /// SetPhaseSource through the command channel: a valid source switches
    /// the manager (and mirrors into shared state), an invalid one is
    /// rejected and leaves everything unchanged.
    #[test]
    fn process_commands_switches_phase_source() {
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
        use crate::foc::trig::LibmSinCos;
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
            fn is_critical(&self) -> bool {
                true
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();

        let mut mgr = PhaseManager::sensorless();
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            1.0 / 20_000.0,
        );
        // The driver must stay linked so process_commands doesn't force Stopped.
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        // Valid switch: HFI estimator is configured.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hfi));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.phase().source(), PhaseSource::Hfi);
        let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
        assert_eq!(mirrored, PhaseSource::Hfi);

        // Invalid switch: no hall sensor on a sensorless manager.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hall));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.phase().source(),
            PhaseSource::Hfi,
            "invalid source must be rejected, not applied"
        );
        let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
        assert_eq!(mirrored, PhaseSource::Hfi);
    }

    /// Non-finite numbers must die at the command boundary: a NaN target
    /// reaching the PI loop turns the SVPWM output into a garbage voltage
    /// vector (bounded by the saturating casts, but still a mechanical
    /// jolt until the overcurrent check reacts).
    #[test]
    fn process_commands_rejects_non_finite_payloads() {
        use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
        use crate::foc::phase::PhaseManager;
        use crate::foc::trig::LibmSinCos;
        use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
        use core::cell::RefCell;
        use critical_section::Mutex as CriticalSectionMutex;

        #[derive(Clone, Copy, PartialEq)]
        struct TestFault;
        impl PlatformFault for TestFault {
            fn category(&self) -> FaultCategory {
                FaultCategory::OverCurrent
            }
            fn details(&self) -> heapless::String<128> {
                heapless::String::new()
            }
            fn is_recoverable(&self) -> bool {
                false
            }
            fn is_critical(&self) -> bool {
                true
            }
        }

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<TestFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        // NaN current target must be dropped — driver stays Stopped.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: f32::NAN,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "NaN current target must be rejected"
        );

        // Infinite direct voltage likewise.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::DirectVoltage {
            vd: f32::INFINITY,
            vq: 0.0,
            angle_rad: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), ControlMode::Stopped);

        // NaN PI gains must not reach the controller.
        let gains_before = driver.controller().id_pi.gains();
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetPiGains {
            kp: f32::NAN,
            ki: 100.0,
        });
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.controller().id_pi.gains(),
            gains_before,
            "NaN gains must be rejected"
        );

        // A finite command still works.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.mode(),
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0
            }
        );
    }

    #[test]
    fn config_limits_clamp_to_hardware_ceiling() {
        // Stored config must never raise limits above the board's hardware
        // ceiling, and zero/negative config values (= "not set") fall back
        // to the board defaults instead of disabling protection.
        let hw_max = 10.0;

        // Normal config below the ceiling: passes through.
        let l = CurrentLimits::from_config_clamped(5.0, 8.0, hw_max);
        assert_eq!(l.max_current_a, 5.0);
        assert_eq!(l.overcurrent_threshold_a, 8.0);

        // Config above the ceiling: clamped to the board limits.
        let board = CurrentLimits::from_max_current(hw_max);
        let l = CurrentLimits::from_config_clamped(50.0, 100.0, hw_max);
        assert_eq!(l.max_current_a, board.max_current_a);
        assert_eq!(l.overcurrent_threshold_a, board.overcurrent_threshold_a);

        // Zeroed config: board defaults, NOT disabled protection.
        let l = CurrentLimits::from_config_clamped(0.0, 0.0, hw_max);
        assert_eq!(l.max_current_a, board.max_current_a);
        assert_eq!(l.overcurrent_threshold_a, board.overcurrent_threshold_a);
    }

    /// Every mode that energizes the motor and can read currents must trip
    /// the measured-overcurrent protection. DirectVoltage has no PI loop to
    /// rein the current in, and six-step has no current loop at all — a
    /// shorted phase in those modes must not cook the FETs just because
    /// the mode is "simple".
    #[test]
    fn direct_voltage_trips_overcurrent() {
        use crate::foc::trig::LibmSinCos;

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                // ~26 A magnitude — far above the 13 A threshold below.
                currents: (20.0, -10.0, -10.0),
            },
            crate::foc::phase::PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        driver.set_mode(ControlMode::DirectVoltage {
            vd: 1.0,
            vq: 0.0,
            angle_rad: 0.0,
        });

        let res = driver.step(0);
        assert!(
            res.is_err(),
            "overcurrent must abort the DirectVoltage step"
        );
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "overcurrent must latch Stopped"
        );
    }

    #[test]
    fn six_step_trips_overcurrent() {
        use crate::foc::trig::LibmSinCos;

        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (20.0, -10.0, -10.0),
            },
            crate::foc::phase::PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        driver.set_mode(ControlMode::SixStep { duty: 0.2 });

        let res = driver.step(0);
        assert!(res.is_err(), "overcurrent must abort the six-step step");
        assert_eq!(
            driver.mode(),
            ControlMode::Stopped,
            "overcurrent must latch Stopped"
        );
    }

    /// Closed-loop HFI through the full runtime path: FocDriver in
    /// CurrentControl with a PhaseManager(HfiObserver) source must apply
    /// the estimator's carrier injection itself — no detection-mode help.
    /// The rotor is parked on the far side (2.5 rad: the PLL's nearest
    /// saliency lock is the π-flipped one), so this exercises BOTH the
    /// carrier path and the saturation polarity probe under an active PI
    /// loop that partially fights the probe pulses.
    #[test]
    #[cfg(feature = "virtual-motor")]
    fn current_control_drives_hfi_estimator() {
        use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
        use crate::foc::trig::LibmSinCos;
        use crate::virtual_motor::{MotorParams, VirtualMotor};

        const DT: f32 = 1.0 / 20_000.0;
        const ROTOR_ANGLE: f32 = 2.5;
        // Same IPM as the estimator-level HFI tests: 3:1 saliency, heavy
        // rotor + friction so the injection itself doesn't move it, plus
        // d-axis saturation so the polarity probe has a signal.
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 1e-2,
            friction_b: 5e-2,
            hall_offset: 0.0,
            sat_k: 0.05,
        };
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(ROTOR_ANGLE);

        // 1000 rad/s current loop — well below the 1 kHz carrier, so the
        // PI regulates the fundamental without eating the carrier response.
        let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );
        let mut mgr = PhaseManager::sensorless();
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::Hfi).unwrap();

        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 0.0,
            id_target: 0.0,
        });

        let mut out = crate::virtual_motor::VirtualMotorOutput::default();
        for step in 1..20_000u64 {
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            let telem = driver.step(step * 50).expect("FOC step failed");
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        // Full-circle match: the polarity probe must have corrected the
        // π-flipped initial lock through the driver path.
        let true_angle = crate::foc::wrap_angle(out.angle_rad);
        let err = crate::foc::angle_difference(driver.phase().get().angle, true_angle).abs();
        assert!(
            err < 0.15,
            "HFI did not converge through FocDriver: est {} vs rotor {} (err {} full-circle)",
            driver.phase().get().angle,
            true_angle,
            err
        );
        let hfi = driver.phase().hfi_observer().unwrap();
        assert!(
            hfi.is_ready(),
            "HFI must be ready (locked + polarity resolved), confidence {}",
            hfi.confidence()
        );
        assert!(
            crate::foc::angle_difference(out.angle_rad, ROTOR_ANGLE).abs() < 0.15,
            "injection moved the rotor: {} rad",
            out.angle_rad
        );
    }
}

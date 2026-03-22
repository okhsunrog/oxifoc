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
        if matches!(self.mode, ControlMode::SixStep { .. })
            && !matches!(mode, ControlMode::SixStep { .. })
        {
            // Re-enable all phases so set_duties() in FOC modes works
            self.pwm
                .set_phase_states([PhaseState::Low, PhaseState::Low, PhaseState::Low]);
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

        // Read currents and run FOC controller
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        let out = self
            .controller
            .step(currents, angle_rad, id_target, iq_target, max_duty, dt);

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

        // Read currents and run FOC controller with commanded angle
        // id_target = 0 (no field weakening in open-loop)
        let currents = self.current_sensor.read_currents();
        let max_duty = self.pwm.max_duty();
        let out = self
            .controller
            .step(currents, angle, 0.0, current, max_duty, dt);

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

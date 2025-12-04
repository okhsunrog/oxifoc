//! Reusable FOC motor driver
//!
//! This module provides a high-level FOC driver that integrates:
//! - FOC controller (Clarke/Park transforms, PI control, SVPWM)
//! - Current sensing
//! - Phase provider (angle sensing/estimation)
//! - PWM output
//! - Control mode management
//!
//! Platform code just needs to provide trait implementations and call `step()`.

use crate::foc::controller::{FocController, FocTelemetry};
use crate::foc::phase::{PhaseInput, PhaseProvider};
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::transforms;

// Re-export ControlMode from types (single source of truth)
pub use crate::types::ControlMode;

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
pub struct FocDriver<P, C, Phase>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
{
    /// FOC controller
    controller: FocController,
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
}

impl<P, C, Phase> FocDriver<P, C, Phase>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
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
        controller: FocController,
        pwm: P,
        current_sensor: C,
        phase: Phase,
        dt: f32,
    ) -> Self {
        Self {
            controller,
            pwm,
            current_sensor,
            phase,
            mode: ControlMode::Stopped,
            vbus: 12.0, // Default, should be updated
            dt,
        }
    }

    /// Get the control loop period (dt) in seconds
    pub fn dt(&self) -> f32 {
        self.dt
    }

    /// Set control mode
    pub fn set_mode(&mut self, mode: ControlMode) {
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
    /// * `Ok(FocTelemetry)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready
    pub fn step(&mut self, now_ticks: u64) -> Result<FocTelemetry, &'static str> {
        let dt = self.dt;
        match self.mode {
            ControlMode::Stopped => {
                // Disable PWM outputs
                self.pwm.disable();
                // Still update phase provider for sensor tracking
                self.phase.update(
                    &PhaseInput {
                        dt,
                        ..Default::default()
                    },
                    now_ticks,
                );
                Ok(FocTelemetry::default())
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
            ControlMode::OpenLoop { angle_rad, current } => {
                self.step_open_loop(angle_rad, current, dt, now_ticks)
            }
            ControlMode::HfiInjection {
                hold_current,
                vd_inject,
                vq_inject,
            } => self.step_hfi_injection(hold_current, vd_inject, vq_inject, dt, now_ticks),
        }
    }

    /// Execute current control step
    fn step_current_control(
        &mut self,
        iq_target: f32,
        id_target: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocTelemetry, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Get phase from provider (uses previous update's estimate)
        let phase_out = self.phase.get();
        let angle_rad = phase_out.angle;

        // Read currents
        let currents = self.current_sensor.read_currents();
        let (i_alpha, i_beta) = transforms::clarke(currents.0, currents.1);

        // Run FOC controller
        let max_duty = self.pwm.max_duty();
        let telem = self
            .controller
            .step(currents, angle_rad, id_target, iq_target, max_duty, dt);

        // Set PWM duties
        self.pwm.set_duties(telem.duties);

        // Update phase provider for next step (feeds observer if present)
        self.phase.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha,
                i_beta,
                dt,
            },
            now_ticks,
        );

        Ok(telem)
    }

    /// Execute open-loop control step (for calibration)
    ///
    /// Uses commanded angle instead of sensor feedback to lock rotor position.
    /// Current feedback is still used to regulate the applied current.
    fn step_open_loop(
        &mut self,
        angle_rad: f32,
        current: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocTelemetry, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Read currents for feedback
        let currents = self.current_sensor.read_currents();
        let (i_alpha, i_beta) = transforms::clarke(currents.0, currents.1);

        // Use commanded angle instead of sensor
        // Apply current as q-axis (torque) to lock rotor at the commanded angle
        // id_target = 0 (no field weakening in open-loop)
        let max_duty = self.pwm.max_duty();
        let telem = self
            .controller
            .step(currents, angle_rad, 0.0, current, max_duty, dt);

        // Set PWM duties
        self.pwm.set_duties(telem.duties);

        // Update phase provider (for sensor tracking, even in open-loop)
        self.phase.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha,
                i_beta,
                dt,
            },
            now_ticks,
        );

        Ok(telem)
    }

    /// Execute HFI injection step (for inductance measurement)
    ///
    /// Locks rotor at angle 0 with d-axis current while injecting
    /// high-frequency voltage for inductance measurement.
    fn step_hfi_injection(
        &mut self,
        hold_current: f32,
        vd_inject: f32,
        vq_inject: f32,
        dt: f32,
        now_ticks: u64,
    ) -> Result<FocTelemetry, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Read currents for feedback
        let currents = self.current_sensor.read_currents();
        let (i_alpha, i_beta) = transforms::clarke(currents.0, currents.1);

        // Lock rotor at angle 0, apply d-axis holding current
        // The HFI injection voltages are added on top
        let max_duty = self.pwm.max_duty();
        let telem = self.controller.step_with_injection(
            currents,
            0.0,          // Lock at angle 0
            hold_current, // d-axis current to hold rotor
            0.0,          // No q-axis current (we're holding position)
            vd_inject,
            vq_inject,
            max_duty,
            dt,
        );

        // Set PWM duties
        self.pwm.set_duties(telem.duties);

        // Update phase provider
        self.phase.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha,
                i_beta,
                dt,
            },
            now_ticks,
        );

        Ok(telem)
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
    pub fn controller_mut(&mut self) -> &mut FocController {
        &mut self.controller
    }

    /// Get reference to FOC controller
    pub fn controller(&self) -> &FocController {
        &self.controller
    }
}

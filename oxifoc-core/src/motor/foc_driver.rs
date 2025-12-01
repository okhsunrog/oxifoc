//! Reusable FOC motor driver
//!
//! This module provides a high-level FOC driver that integrates:
//! - FOC controller (Clarke/Park transforms, PI control, SVPWM)
//! - Current sensing
//! - Angle sensing
//! - PWM output
//! - Control mode management
//!
//! Platform code just needs to provide trait implementations and call `step()`.

use crate::foc::controller::{FocController, FocTelemetry};
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::{AngleSensor, CurrentSensor};

/// Control mode for the motor
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlMode {
    /// Motor stopped, outputs disabled
    Stopped,
    /// Current control mode (torque control)
    CurrentControl {
        /// Target q-axis current (torque-producing)
        iq_target: f32,
        /// Target d-axis current (field-weakening)
        id_target: f32,
    },
    /// Velocity control mode (speed control)
    VelocityControl {
        /// Target velocity in rad/s
        target_vel: f32,
    },
    /// Position control mode
    PositionControl {
        /// Target position in radians
        target_pos: f32,
    },
    /// Open-loop mode for calibration - locks rotor to specified electrical angle
    ///
    /// In this mode, the FOC controller uses a commanded angle instead of the
    /// angle sensor. Current control still runs to regulate the applied current.
    /// This is used for Hall sensor calibration where we need to sweep the rotor
    /// through known electrical angles.
    OpenLoop {
        /// Target electrical angle (radians, 0 to 2π)
        angle_rad: f32,
        /// Current magnitude (Amps) - applied as q-current to lock rotor
        current: f32,
    },
}

/// Motor command (for channel-based API)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MotorCommand {
    /// Stop the motor
    Stop,
    /// Set current control targets
    SetCurrent {
        /// Target q-axis current (torque)
        iq: f32,
        /// Target d-axis current (field)
        id: f32,
    },
    /// Set velocity target
    SetVelocity {
        /// Target velocity in rad/s
        vel: f32,
    },
    /// Set position target
    SetPosition {
        /// Target position in radians
        pos: f32,
    },
    /// Set open-loop mode (for calibration)
    SetOpenLoop {
        /// Target electrical angle (radians)
        angle: f32,
        /// Current magnitude (Amps)
        current: f32,
    },
}

impl MotorCommand {
    /// Convert command to control mode
    pub fn to_mode(self) -> ControlMode {
        match self {
            MotorCommand::Stop => ControlMode::Stopped,
            MotorCommand::SetCurrent { iq, id } => ControlMode::CurrentControl {
                iq_target: iq,
                id_target: id,
            },
            MotorCommand::SetVelocity { vel } => ControlMode::VelocityControl { target_vel: vel },
            MotorCommand::SetPosition { pos } => ControlMode::PositionControl { target_pos: pos },
            MotorCommand::SetOpenLoop { angle, current } => ControlMode::OpenLoop {
                angle_rad: angle,
                current,
            },
        }
    }
}

/// Reusable FOC driver
///
/// # Type Parameters
/// * `P` - PWM output implementing `PhasePwm`
/// * `C` - Current sensor implementing `CurrentSensor`
/// * `A` - Angle sensor implementing `AngleSensor`
///
/// # Example (platform code)
/// ```rust,ignore
/// static FOC_DRIVER: Mutex<NoopRawMutex, RefCell<Option<FocDriver<MotorPwm, CurrentSense, HallSensor>>>> =
///     Mutex::new(RefCell::new(None));
///
/// #[interrupt]
/// fn ADC1_2() {
///     static mut STATE: ControlMode = ControlMode::Stopped;
///
///     // Process commands
///     while let Ok(cmd) = MOTOR_CMD.try_receive() {
///         *STATE = cmd.to_mode();
///     }
///
///     // Run FOC
///     FOC_DRIVER.lock(|cell| {
///         if let Some(driver) = cell.borrow_mut().as_mut() {
///             driver.set_mode(*STATE);
///             if let Ok(telem) = driver.step(DT) {
///                 MOTOR_TELEM.set(telem);
///             }
///         }
///     });
/// }
/// ```
pub struct FocDriver<P, C, A>
where
    P: PhasePwm,
    C: CurrentSensor,
    A: AngleSensor,
{
    /// FOC controller
    controller: FocController,
    /// PWM output
    pwm: P,
    /// Current sensor
    current_sensor: C,
    /// Angle sensor
    angle_sensor: A,
    /// Current control mode
    mode: ControlMode,
    /// Bus voltage (V)
    vbus: f32,
}

impl<P, C, A> FocDriver<P, C, A>
where
    P: PhasePwm,
    C: CurrentSensor,
    A: AngleSensor,
{
    /// Create a new FOC driver
    ///
    /// # Arguments
    /// * `controller` - FOC controller instance
    /// * `pwm` - PWM output
    /// * `current_sensor` - Current sensor (should be calibrated)
    /// * `angle_sensor` - Angle sensor
    pub fn new(controller: FocController, pwm: P, current_sensor: C, angle_sensor: A) -> Self {
        Self {
            controller,
            pwm,
            current_sensor,
            angle_sensor,
            mode: ControlMode::Stopped,
            vbus: 12.0, // Default, should be updated
        }
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
    ///
    /// # Arguments
    /// * `dt` - Time step in seconds (e.g., 1.0/20000.0 for 20kHz)
    /// * `now_ticks` - Monotonic ticks for angle sampling (sensor-defined timebase)
    ///
    /// # Returns
    /// * `Ok(FocTelemetry)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready
    pub fn step(&mut self, dt: f32, now_ticks: u64) -> Result<FocTelemetry, &'static str> {
        match self.mode {
            ControlMode::Stopped => {
                // Disable PWM outputs
                self.pwm.disable();
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
                self.step_open_loop(angle_rad, current, dt)
            }
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

        // Read sensors
        let currents = self.current_sensor.read_currents();
        let angle_rad = self
            .angle_sensor
            .sample(now_ticks)
            .map(|s| s.angle)
            .unwrap_or_else(|| self.angle_sensor.read_angle());

        // Run FOC controller
        let max_duty = self.pwm.max_duty();
        let telem = self
            .controller
            .step(currents, angle_rad, id_target, iq_target, max_duty, dt);

        // Set PWM duties
        self.pwm.set_duties(telem.duties);

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
    ) -> Result<FocTelemetry, &'static str> {
        // Check sensor calibration
        if !self.current_sensor.is_calibrated() {
            return Err("Current sensor not calibrated");
        }

        // Read currents for feedback
        let currents = self.current_sensor.read_currents();

        // Use commanded angle instead of sensor
        // Apply current as q-axis (torque) to lock rotor at the commanded angle
        // id_target = 0 (no field weakening in open-loop)
        let max_duty = self.pwm.max_duty();
        let telem = self
            .controller
            .step(currents, angle_rad, 0.0, current, max_duty, dt);

        // Set PWM duties
        self.pwm.set_duties(telem.duties);

        Ok(telem)
    }

    /// Get mutable reference to current sensor (for calibration)
    pub fn current_sensor_mut(&mut self) -> &mut C {
        &mut self.current_sensor
    }

    /// Get mutable reference to angle sensor
    pub fn angle_sensor_mut(&mut self) -> &mut A {
        &mut self.angle_sensor
    }

    /// Get mutable reference to FOC controller (for tuning)
    pub fn controller_mut(&mut self) -> &mut FocController {
        &mut self.controller
    }
}

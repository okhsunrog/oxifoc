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
    /// HFI injection mode for inductance measurement
    ///
    /// Locks rotor at angle 0 with a holding current while injecting
    /// high-frequency voltage for inductance measurement.
    HfiInjection {
        /// DC current to hold rotor in place (Amps)
        hold_current: f32,
        /// d-axis voltage to inject (V)
        vd_inject: f32,
        /// q-axis voltage to inject (V)
        vq_inject: f32,
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
    /// Set HFI injection mode (for inductance measurement)
    SetHfiInjection {
        /// DC current to hold rotor (Amps)
        hold_current: f32,
        /// d-axis injection voltage (V)
        vd_inject: f32,
        /// q-axis injection voltage (V)
        vq_inject: f32,
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
            MotorCommand::SetHfiInjection {
                hold_current,
                vd_inject,
                vq_inject,
            } => ControlMode::HfiInjection {
                hold_current,
                vd_inject,
                vq_inject,
            },
        }
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
///
/// static FOC_DRIVER: Mutex<NoopRawMutex, RefCell<Option<FocDriver<MotorPwm, CurrentSense, PhaseManager<HallSensor>>>>> =
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
///             if let Ok(telem) = driver.step(DT, now_ticks) {
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
    pub fn new(controller: FocController, pwm: P, current_sensor: C, phase: Phase) -> Self {
        Self {
            controller,
            pwm,
            current_sensor,
            phase,
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
    /// * `now_ticks` - Monotonic ticks for phase sampling (sensor-defined timebase)
    ///
    /// # Returns
    /// * `Ok(FocTelemetry)` - Control telemetry on success
    /// * `Err(&str)` - Error message if sensors not ready
    pub fn step(&mut self, dt: f32, now_ticks: u64) -> Result<FocTelemetry, &'static str> {
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

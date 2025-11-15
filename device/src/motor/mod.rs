//! Motor control module for B-G431B-ESC1 BLDC motor
//!
//! Motor: ZD2808-V1.9 700KV
//! - Configuration: 12N14P (12 stator slots, 14 poles = 7 pole pairs)
//! - KV rating: 700 KV
//! - Voltage: 3S-4S LiPo (11.1-14.8V)
//! - Type: Outrunner disc motor

pub mod pwm;
pub mod six_step;

use core::sync::atomic::{AtomicU8, Ordering};
use embassy_time::Duration;
use oxifoc_protocol::{MotorCommand, MotorState, MotorStatus};

use self::pwm::{MotorPwm, MotorPwmConfig};
use self::six_step::CommutationStep;

/// Motor physical parameters
pub struct MotorParams {
    /// Number of pole pairs (14 poles = 7 pole pairs)
    pub pole_pairs: u8,
    /// KV rating (RPM per volt)
    pub kv_rating: u16,
}

impl Default for MotorParams {
    fn default() -> Self {
        Self {
            pole_pairs: 7,      // ZD2808-V1.9: 14 poles = 7 pole pairs
            kv_rating: 700,     // 700 KV
        }
    }
}

/// Global motor state
static MOTOR_STATE: AtomicU8 = AtomicU8::new(MotorState::Stopped as u8);
static MOTOR_DUTY: AtomicU8 = AtomicU8::new(0);
static MOTOR_STEP: AtomicU8 = AtomicU8::new(0);

/// Set motor state
pub fn set_motor_state(state: MotorState) {
    MOTOR_STATE.store(state as u8, Ordering::Relaxed);
}

/// Get motor state
pub fn get_motor_state() -> MotorState {
    match MOTOR_STATE.load(Ordering::Relaxed) {
        0 => MotorState::Stopped,
        1 => MotorState::Running,
        _ => MotorState::Error,
    }
}

/// Set motor duty cycle
pub fn set_motor_duty(duty: u8) {
    MOTOR_DUTY.store(duty, Ordering::Relaxed);
}

/// Get motor duty cycle
pub fn get_motor_duty() -> u8 {
    MOTOR_DUTY.load(Ordering::Relaxed)
}

/// Set motor commutation step
pub fn set_motor_step(step: u8) {
    MOTOR_STEP.store(step, Ordering::Relaxed);
}

/// Get motor commutation step
pub fn get_motor_step() -> u8 {
    MOTOR_STEP.load(Ordering::Relaxed)
}

/// Get current motor status
pub fn get_motor_status() -> MotorStatus {
    MotorStatus {
        state: get_motor_state(),
        duty: get_motor_duty(),
        step: get_motor_step(),
    }
}

/// Motor control context
pub struct MotorController<'d> {
    pwm: MotorPwm<'d>,
    current_step: CommutationStep,
    target_duty: u8,
    commutation_period_ms: u32,
}

impl<'d> MotorController<'d> {
    /// Create a new motor controller
    pub fn new(pwm: MotorPwm<'d>) -> Self {
        set_motor_state(MotorState::Stopped);
        set_motor_duty(0);
        set_motor_step(0);

        Self {
            pwm,
            current_step: CommutationStep::Step0,
            target_duty: 0,
            commutation_period_ms: 500,  // Very slow for initial testing (500ms per step = ~2.8 RPM)
        }
    }

    /// Initialize motor PWM hardware
    pub fn init(
        tim1: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::TIM1>>,
        pa8: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA8>>,
        pc13: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PC13>>,
        pa9: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA9>>,
        pa12: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA12>>,
        pa10: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA10>>,
        pb15: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PB15>>,
    ) -> Self {
        let config = MotorPwmConfig::default();
        let pwm = MotorPwm::new(tim1, pa8, pc13, pa9, pa12, pa10, pb15, config);
        Self::new(pwm)
    }

    /// Handle motor command
    pub fn handle_command(&mut self, cmd: &MotorCommand) {
        match cmd {
            MotorCommand::Stop => {
                defmt::info!("Motor command: STOP");
                self.stop();
            }
            MotorCommand::Start { duty } => {
                defmt::info!("Motor command: START duty={}", duty);
                self.start(*duty);
            }
            MotorCommand::SetSpeed { duty } => {
                defmt::info!("Motor command: SET_SPEED duty={}", duty);
                self.set_speed(*duty);
            }
        }
    }

    /// Start the motor with specified duty cycle
    fn start(&mut self, duty: u8) {
        let duty = duty.min(100);
        self.target_duty = duty;
        set_motor_duty(duty);
        set_motor_state(MotorState::Running);

        // Reset to step 0
        self.current_step = CommutationStep::Step0;
        set_motor_step(0);

        defmt::info!("Motor started: duty={}%", duty);
    }

    /// Stop the motor
    fn stop(&mut self) {
        self.target_duty = 0;
        self.pwm.emergency_stop();
        set_motor_state(MotorState::Stopped);
        set_motor_duty(0);
        defmt::info!("Motor stopped");
    }

    /// Set motor speed (adjust duty while running)
    fn set_speed(&mut self, duty: u8) {
        let duty = duty.min(100);
        self.target_duty = duty;
        set_motor_duty(duty);
        defmt::info!("Motor speed set: duty={}%", duty);
    }

    /// Perform one commutation step
    pub fn commutate(&mut self) {
        if get_motor_state() != MotorState::Running {
            // Motor not running, ensure all phases are off
            self.pwm.emergency_stop();
            return;
        }

        // Get phase states for current step
        let (ph_a_en, ph_b_en, ph_c_en, _ph_a_high, _ph_b_high, _ph_c_high) =
            self.current_step.get_phase_states();

        // Apply commutation pattern with current duty cycle
        self.pwm.apply_commutation(
            self.target_duty,
            ph_a_en,
            ph_b_en,
            ph_c_en,
        );

        // Update global state
        set_motor_step(self.current_step.as_u8());

        // Advance to next step
        self.current_step = self.current_step.next();
    }

    /// Get commutation period based on desired speed
    pub fn get_commutation_period(&self) -> Duration {
        // For now, use fixed period
        // TODO: Calculate based on duty cycle for smoother speed control
        Duration::from_millis(self.commutation_period_ms as u64)
    }

    /// Set commutation period (for speed tuning)
    pub fn set_commutation_period_ms(&mut self, period_ms: u32) {
        self.commutation_period_ms = period_ms;
    }
}

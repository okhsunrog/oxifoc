//! Motor control for Simple FOCer 2 (STM32F405)
//!
//! TIM1 complementary PWM configuration for 3-phase BLDC/PMSM control
//! with dead-time insertion for shoot-through protection and
//! center-aligned PWM for optimal ADC sampling.
//! Plus motor state management for protocol telemetry.

use core::sync::atomic::{AtomicU8, Ordering};

use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals;
use embassy_stm32::time::khz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;

use crate::hardware::resources::MotorResources;
use oxifoc_core::foc::pwm::{self, MotorPwmConfig, PhasePwm};
use oxifoc_protocol::{MotorState, MotorStatus};

// ============================================================================
// Motor State Management (for protocol telemetry)
// ============================================================================

// Public API not yet wired to protocol handlers
#[allow(dead_code)]
static MOTOR_STATE: AtomicU8 = AtomicU8::new(MotorState::Stopped as u8);
#[allow(dead_code)]
static MOTOR_DUTY: AtomicU8 = AtomicU8::new(0);
#[allow(dead_code)]
static MOTOR_STEP: AtomicU8 = AtomicU8::new(0);

#[allow(dead_code)]
pub fn set_motor_state(state: MotorState) {
    MOTOR_STATE.store(state as u8, Ordering::Relaxed);
}

#[allow(dead_code)]
pub fn get_motor_state() -> MotorState {
    match MOTOR_STATE.load(Ordering::Relaxed) {
        0 => MotorState::Stopped,
        1 => MotorState::Running,
        _ => MotorState::Error,
    }
}

#[allow(dead_code)]
pub fn set_motor_duty(duty: u8) {
    MOTOR_DUTY.store(duty, Ordering::Relaxed);
}

#[allow(dead_code)]
pub fn get_motor_duty() -> u8 {
    MOTOR_DUTY.load(Ordering::Relaxed)
}

#[allow(dead_code)]
pub fn set_motor_step(step: u8) {
    MOTOR_STEP.store(step, Ordering::Relaxed);
}

#[allow(dead_code)]
pub fn get_motor_step() -> u8 {
    MOTOR_STEP.load(Ordering::Relaxed)
}

#[allow(dead_code)]
pub fn get_motor_status() -> MotorStatus {
    MotorStatus {
        state: get_motor_state(),
        duty: get_motor_duty(),
        step: get_motor_step(),
    }
}

// ============================================================================
// Motor PWM
// ============================================================================

/// STM32F405 timer clock frequency (168 MHz)
const TIMER_CLOCK_HZ: u32 = 168_000_000;

/// Motor PWM controller using TIM1
///
/// # Pin mapping (Simple FOCer 2 / VESC):
/// - Phase A: TIM1_CH1 (PA8) / TIM1_CH1N (PB13)
/// - Phase B: TIM1_CH2 (PA9) / TIM1_CH2N (PB14)
/// - Phase C: TIM1_CH3 (PA10) / TIM1_CH3N (PB15)
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, peripherals::TIM1>,
    max_duty: u16,
    duty_limit: u16,
}

impl<'d> MotorPwm<'d> {
    /// Initialize TIM1 complementary PWM for Simple FOCer 2 board
    ///
    /// Phase mapping:
    /// - Phase A: TIM1_CH1 (PA8) / TIM1_CH1N (PB13)
    /// - Phase B: TIM1_CH2 (PA9) / TIM1_CH2N (PB14)
    /// - Phase C: TIM1_CH3 (PA10) / TIM1_CH3N (PB15)
    pub fn new(resources: MotorResources, config: MotorPwmConfig) -> Self {
        // High-side pins (TIM1 CH1/2/3)
        let ch1 = PwmPin::new(resources.pa8, OutputType::PushPull);
        let ch2 = PwmPin::new(resources.pa9, OutputType::PushPull);
        let ch3 = PwmPin::new(resources.pa10, OutputType::PushPull);

        // Low-side pins (TIM1 CH1N/2N/3N)
        let ch1n = ComplementaryPwmPin::new(resources.pb13, OutputType::PushPull);
        let ch2n = ComplementaryPwmPin::new(resources.pb14, OutputType::PushPull);
        let ch3n = ComplementaryPwmPin::new(resources.pb15, OutputType::PushPull);

        let pwm_freq = khz(config.pwm_freq_hz / 1000);

        let mut pwm = ComplementaryPwm::new(
            resources.tim1,
            Some(ch1),
            Some(ch1n),
            Some(ch2),
            Some(ch2n),
            Some(ch3),
            Some(ch3n),
            None,
            None,
            pwm_freq,
            CountingMode::CenterAlignedBothInterrupts,
        );

        let max_duty = pwm.get_max_duty();

        // Calculate dead time using shared helper
        let dead_time = pwm::dead_time_ticks(config.dead_time_ns, TIMER_CLOCK_HZ);
        pwm.set_dead_time(dead_time);

        // Calculate duty limit using shared helper
        let duty_limit = pwm::duty_limit(max_duty, config.max_duty_percent);

        defmt::info!(
            "F405 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq_hz,
            max_duty,
            config.max_duty_percent
        );

        // Enable all three channels
        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    /// Emergency stop - disable all outputs
    pub fn emergency_stop(&mut self) {
        self.pwm.set_duty(Channel::Ch1, 0);
        self.pwm.set_duty(Channel::Ch2, 0);
        self.pwm.set_duty(Channel::Ch3, 0);
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }
}

impl<'d> PhasePwm for MotorPwm<'d> {
    fn max_duty(&self) -> u16 {
        self.max_duty
    }

    fn set_duties(&mut self, duties: [u16; 3]) {
        // Clamp to duty limit for safety
        let duty_a = duties[0].min(self.duty_limit);
        let duty_b = duties[1].min(self.duty_limit);
        let duty_c = duties[2].min(self.duty_limit);

        self.pwm.set_duty(Channel::Ch1, duty_a);
        self.pwm.set_duty(Channel::Ch2, duty_b);
        self.pwm.set_duty(Channel::Ch3, duty_c);
    }

    fn disable(&mut self) {
        self.emergency_stop();
    }
}

//! TIM1 complementary PWM configuration for Simple FOCer 2 (STM32F405)
//!
//! Hardware: Simple FOCer 2 / Cheap FOCer 2 / VESC-compatible designs
//! - TIM1 complementary outputs for 3-phase BLDC/PMSM control
//! - Dead-time insertion for shoot-through protection
//! - Center-aligned PWM for optimal ADC sampling

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::khz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::{Peri, peripherals};

/// PWM configuration for the motor controller
#[allow(dead_code)]
pub struct MotorPwmConfig {
    /// PWM switching frequency in Hz
    pub pwm_freq: u32,
    /// Dead time in nanoseconds
    pub dead_time_ns: u32,
    /// Maximum allowed duty cycle (0-100%)
    pub max_duty_percent: u8,
}

impl Default for MotorPwmConfig {
    fn default() -> Self {
        Self {
            pwm_freq: 20_000,     // 20 kHz
            dead_time_ns: 1000,   // 1 µs (conservative for DRV8301/2/3)
            max_duty_percent: 95, // 95% max duty
        }
    }
}

/// Motor PWM controller using TIM1
///
/// # Pin mapping (Simple FOCer 2 / VESC):
/// - Phase A: TIM1_CH1 (PA8) / TIM1_CH1N (PB13)
/// - Phase B: TIM1_CH2 (PA9) / TIM1_CH2N (PB14)  
/// - Phase C: TIM1_CH3 (PA10) / TIM1_CH3N (PB15)
#[allow(dead_code)]
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, peripherals::TIM1>,
    max_duty: u16,
    duty_limit: u16,
}

impl<'d> MotorPwm<'d> {
    /// Initialize TIM1 complementary PWM
    ///
    /// # Arguments
    /// * `tim1` - TIM1 peripheral
    /// * `pa8` - Phase A high-side (TIM1_CH1)
    /// * `pb13` - Phase A low-side (TIM1_CH1N)
    /// * `pa9` - Phase B high-side (TIM1_CH2)
    /// * `pb14` - Phase B low-side (TIM1_CH2N)
    /// * `pa10` - Phase C high-side (TIM1_CH3)
    /// * `pb15` - Phase C low-side (TIM1_CH3N)
    /// * `config` - PWM configuration
    #[allow(dead_code, clippy::too_many_arguments)]
    pub fn new(
        tim1: Peri<'d, peripherals::TIM1>,
        pa8: Peri<'d, peripherals::PA8>,
        pb13: Peri<'d, peripherals::PB13>,
        pa9: Peri<'d, peripherals::PA9>,
        pb14: Peri<'d, peripherals::PB14>,
        pa10: Peri<'d, peripherals::PA10>,
        pb15: Peri<'d, peripherals::PB15>,
        config: MotorPwmConfig,
    ) -> Self {
        // High-side pins
        let ch1 = PwmPin::new(pa8, OutputType::PushPull);
        let ch2 = PwmPin::new(pa9, OutputType::PushPull);
        let ch3 = PwmPin::new(pa10, OutputType::PushPull);

        // Low-side pins
        let ch1n = ComplementaryPwmPin::new(pb13, OutputType::PushPull);
        let ch2n = ComplementaryPwmPin::new(pb14, OutputType::PushPull);
        let ch3n = ComplementaryPwmPin::new(pb15, OutputType::PushPull);

        let pwm_freq = khz(config.pwm_freq / 1000);

        let mut pwm = ComplementaryPwm::new(
            tim1,
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

        // Calculate dead time in timer ticks
        // STM32F405 runs TIM1 at 168 MHz
        const TIMER_CLOCK_HZ: u32 = 168_000_000;
        let dead_time_ticks =
            ((config.dead_time_ns as u64 * TIMER_CLOCK_HZ as u64) / 1_000_000_000) as u16;
        pwm.set_dead_time(dead_time_ticks);

        // Calculate duty limit
        let duty_limit = (max_duty as u32 * config.max_duty_percent as u32 / 100) as u16;

        defmt::info!(
            "F405 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq,
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

// Implement PhasePwm trait for FOC integration
use oxifoc_core::foc::pwm::PhasePwm;

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

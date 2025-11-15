//! TIM1 complementary PWM configuration for 3-phase motor control

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::khz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::low_level::CountingMode;

/// PWM configuration for the motor
pub struct MotorPwmConfig {
    pub pwm_freq: u32,  // Hz
    pub dead_time_ns: u32,  // nanoseconds
    pub max_duty_percent: u8,  // 0-100
}

impl Default for MotorPwmConfig {
    fn default() -> Self {
        Self {
            pwm_freq: 20_000,  // 20 kHz
            dead_time_ns: 2000,  // 2 µs
            max_duty_percent: 15,  // 15% for very safe initial testing
        }
    }
}

/// Motor PWM controller
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>,
    max_duty: u16,
    duty_limit: u16,
}

impl<'d> MotorPwm<'d> {
    /// Initialize TIM1 complementary PWM for the B-G431B-ESC1 board
    pub fn new(
        tim1: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::TIM1>>,
        pa8: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA8>>,
        pc13: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PC13>>,
        pa9: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA9>>,
        pa12: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA12>>,
        pa10: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PA10>>,
        pb15: impl Into<embassy_stm32::Peri<'d, embassy_stm32::peripherals::PB15>>,
        config: MotorPwmConfig,
    ) -> Self {
        let tim1 = tim1.into();
        let pa8 = pa8.into();
        let pc13 = pc13.into();
        let pa9 = pa9.into();
        let pa12 = pa12.into();
        let pa10 = pa10.into();
        let pb15 = pb15.into();

        // High-side pins
        let ch1 = PwmPin::new(pa8, OutputType::PushPull);   // Phase A high
        let ch2 = PwmPin::new(pa9, OutputType::PushPull);   // Phase B high
        let ch3 = PwmPin::new(pa10, OutputType::PushPull);  // Phase C high

        // Low-side pins (complementary)
        let ch1n = ComplementaryPwmPin::new(pc13, OutputType::PushPull);  // Phase A low
        let ch2n = ComplementaryPwmPin::new(pa12, OutputType::PushPull);  // Phase B low
        let ch3n = ComplementaryPwmPin::new(pb15, OutputType::PushPull);  // Phase C low

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
        // At 170 MHz, each tick is ~5.88 ns
        // For 2 µs dead time, we need ~340 ticks
        // But dead time register has specific encoding - use a fraction of max_duty
        let dead_time_ticks = max_duty / 512;  // Conservative ~2µs at 20kHz
        pwm.set_dead_time(dead_time_ticks);

        // Calculate duty cycle limit based on max_duty_percent
        let duty_limit = (max_duty as u32 * config.max_duty_percent as u32 / 100) as u16;

        defmt::info!(
            "Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq,
            max_duty,
            config.max_duty_percent
        );

        // Enable all three channels initially (will be controlled by 6-step logic)
        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    /// Set duty cycle for a specific phase (0-100%)
    ///
    /// Duty is clamped to the configured max_duty_percent
    pub fn set_phase_duty(&mut self, channel: Channel, duty_percent: u8) {
        let duty_percent = duty_percent.min(100);
        let duty = (self.max_duty as u32 * duty_percent as u32 / 100) as u16;
        let duty = duty.min(self.duty_limit);
        self.pwm.set_duty(channel, duty);
    }

    /// Disable a specific phase (set to 0% duty, but keep channel enabled for fast switching)
    pub fn disable_phase(&mut self, channel: Channel) {
        self.pwm.set_duty(channel, 0);
    }

    /// Apply 6-step commutation pattern
    ///
    /// - enable flags: true = phase active with PWM, false = disabled (floating)
    /// - For active phases, duty_percent is applied
    /// - For inactive phases, duty is set to 0
    pub fn apply_commutation(
        &mut self,
        duty_percent: u8,
        ph_a_en: bool,
        ph_b_en: bool,
        ph_c_en: bool,
    ) {
        if ph_a_en {
            self.set_phase_duty(Channel::Ch1, duty_percent);
        } else {
            self.disable_phase(Channel::Ch1);
        }

        if ph_b_en {
            self.set_phase_duty(Channel::Ch2, duty_percent);
        } else {
            self.disable_phase(Channel::Ch2);
        }

        if ph_c_en {
            self.set_phase_duty(Channel::Ch3, duty_percent);
        } else {
            self.disable_phase(Channel::Ch3);
        }
    }

    /// Emergency stop - disable all phases immediately
    pub fn emergency_stop(&mut self) {
        self.disable_phase(Channel::Ch1);
        self.disable_phase(Channel::Ch2);
        self.disable_phase(Channel::Ch3);
    }

    /// Get maximum duty cycle value
    pub fn get_max_duty(&self) -> u16 {
        self.max_duty
    }
}

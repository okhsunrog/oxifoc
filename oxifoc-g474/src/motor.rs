//! Motor control for NUCLEO-G474RE + X-NUCLEO-IHM08M1
//!
//! TIM1 complementary PWM configuration for 3-phase BLDC motor control.

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin, Mms2, Ossr};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::Channel;

use crate::config::TIM1_CLOCK_HZ;
use crate::hardware::resources::MotorResources;
use oxifoc_core::foc::pwm::{self, MotorPwmConfig, PhasePwm};

/// Motor PWM controller using TIM1 with complementary outputs.
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>,
    max_duty: u16,
    duty_limit: u16,
}

impl<'d> MotorPwm<'d> {
    /// Initialize TIM1 complementary PWM for X-NUCLEO-IHM08M1.
    ///
    /// Phase mapping (IHM08M1 via Morpho):
    /// - Phase U: TIM1_CH1 (PA8) / TIM1_CH1N (PA7)
    /// - Phase V: TIM1_CH2 (PA9) / TIM1_CH2N (PB0)
    /// - Phase W: TIM1_CH3 (PA10) / TIM1_CH3N (PB1)
    pub fn new(resources: MotorResources, config: MotorPwmConfig) -> Self {
        let tim1 = resources.tim1;
        let pa8 = resources.pa8;
        let pa7 = resources.pa7;
        let pa9 = resources.pa9;
        let pb0 = resources.pb0;
        let pa10 = resources.pa10;
        let pb1 = resources.pb1;

        // High-side pins (TIM1 CH1/2/3)
        let ch1 = PwmPin::new(pa8, OutputType::PushPull); // Phase U high
        let ch2 = PwmPin::new(pa9, OutputType::PushPull); // Phase V high
        let ch3 = PwmPin::new(pa10, OutputType::PushPull); // Phase W high

        // Low-side pins (TIM1 CH1N/2N/3N)
        let ch1n = ComplementaryPwmPin::new(pa7, OutputType::PushPull); // Phase U low
        let ch2n = ComplementaryPwmPin::new(pb0, OutputType::PushPull); // Phase V low
        let ch3n = ComplementaryPwmPin::new(pb1, OutputType::PushPull); // Phase W low

        let pwm_freq = Hertz(config.pwm_freq_hz);

        // Center-aligned complementary PWM on all three phases.
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

        // Calculate dead time using shared helper
        let dead_time = pwm::dead_time_ticks(config.dead_time_ns, TIM1_CLOCK_HZ);
        pwm.set_dead_time(dead_time);

        // Enable OSSR for safer off-state behavior when channels are disabled.
        // IDLE_LEVEL means outputs go to their idle state when disabled.
        pwm.set_off_state_selection_run(Ossr::IDLE_LEVEL);

        // Channel 4: internal "sampling" channel to generate TIM1_TRGO2 for ADC.
        //
        // Sample at peak of triangle wave (V0 - all low-side ON).
        // In center-aligned mode, CNT=ARR is the V0 point where all low-side
        // FETs are ON for any duty < 100%.
        // Small offset ensures ADC completes before any switching edges.
        let peak_offset = max_duty / 50; // ~2% margin
        pwm.set_duty(Channel::Ch4, max_duty.saturating_sub(peak_offset));
        pwm.enable(Channel::Ch4);
        pwm.set_mms2(Mms2::COMPARE_OC4);

        // Calculate duty limit using shared helper
        let duty_limit = pwm::duty_limit(max_duty, config.max_duty_percent);

        defmt::info!(
            "G474 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq_hz,
            max_duty,
            config.max_duty_percent
        );

        // Enable all three phase channels (complementary outputs).
        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    /// Emergency stop - disable all phases immediately.
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
        // Set duty cycles for all three phases
        // Clamp to duty_limit for safety
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

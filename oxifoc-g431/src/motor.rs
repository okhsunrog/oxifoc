//! Motor control for B-G431B-ESC1
//!
//! TIM1 complementary PWM configuration for 3-phase BLDC motor control.

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin, Mms2, Ossr};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;

use crate::config::TIM1_CLOCK_HZ;
use crate::hardware::resources::MotorResources;
use oxifoc_core::foc::pwm::{self, MotorPwmConfig, PhasePwm, PhaseState};

/// Motor PWM controller using TIM1 with complementary outputs.
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>,
    max_duty: u32,
    duty_limit: u32,
}

impl<'d> MotorPwm<'d> {
    /// Initialize TIM1 complementary PWM for the B-G431B-ESC1 board.
    ///
    /// Phase mapping (B‑G431B‑ESC1):
    /// - Phase A: TIM1_CH1 (PA8) / TIM1_CH1N (PC13)
    /// - Phase B: TIM1_CH2 (PA9) / TIM1_CH2N (PA12)
    /// - Phase C: TIM1_CH3 (PA10) / TIM1_CH3N (PB15)
    pub fn new(resources: MotorResources, config: MotorPwmConfig) -> Self {
        let tim1 = resources.tim1;
        let pa8 = resources.pa8;
        let pc13 = resources.pc13;
        let pa9 = resources.pa9;
        let pa12 = resources.pa12;
        let pa10 = resources.pa10;
        let pb15 = resources.pb15;

        // High-side pins (TIM1 CH1/2/3)
        let ch1 = PwmPin::new(pa8, OutputType::PushPull); // Phase A high
        let ch2 = PwmPin::new(pa9, OutputType::PushPull); // Phase B high
        let ch3 = PwmPin::new(pa10, OutputType::PushPull); // Phase C high

        // Low-side pins (TIM1 CH1N/2N/3N)
        let ch1n = ComplementaryPwmPin::new(pc13, OutputType::PushPull); // Phase A low
        let ch2n = ComplementaryPwmPin::new(pa12, OutputType::PushPull); // Phase B low
        let ch3n = ComplementaryPwmPin::new(pb15, OutputType::PushPull); // Phase C low

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

        // Calculate duty limit using shared helper (convert to u16 for helper, then back to u32)
        let duty_limit = pwm::duty_limit(max_duty as u16, config.max_duty_percent) as u32;

        defmt::info!(
            "G431 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
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
        self.max_duty as u16
    }

    fn set_duties(&mut self, duties: [u16; 3]) {
        // Set duty cycles for all three phases
        // Clamp to duty_limit for safety
        let duty_a = (duties[0] as u32).min(self.duty_limit);
        let duty_b = (duties[1] as u32).min(self.duty_limit);
        let duty_c = (duties[2] as u32).min(self.duty_limit);

        self.pwm.set_duty(Channel::Ch1, duty_a);
        self.pwm.set_duty(Channel::Ch2, duty_b);
        self.pwm.set_duty(Channel::Ch3, duty_c);
    }

    fn disable(&mut self) {
        self.emergency_stop();
    }

    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        const CHANNELS: [Channel; 3] = [Channel::Ch1, Channel::Ch2, Channel::Ch3];
        for (state, &ch) in states.iter().zip(CHANNELS.iter()) {
            match state {
                PhaseState::Pwm(duty) => {
                    self.pwm.enable(ch);
                    self.pwm.set_duty(ch, (*duty as u32).min(self.duty_limit));
                }
                PhaseState::Low => {
                    self.pwm.enable(ch);
                    self.pwm.set_duty(ch, 0);
                }
                PhaseState::Float => {
                    self.pwm.set_duty(ch, 0);
                    self.pwm.disable(ch);
                }
            }
        }
    }
}

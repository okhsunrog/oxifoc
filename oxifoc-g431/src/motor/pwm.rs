//! TIM1 complementary PWM configuration for 3-phase BLDC motor control on B-G431B-ESC1.

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::khz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin, Mms2, Ossr};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;

use super::six_step::{CommutationStep, PhaseState};
use crate::hardware::resources::MotorResources;

/// PWM configuration for the motor.
pub struct MotorPwmConfig {
    /// PWM switching frequency in Hz.
    pub pwm_freq: u32,
    /// Requested dead time in nanoseconds (used as a hint).
    #[allow(dead_code)]
    pub dead_time_ns: u32,
    /// Maximum allowed duty cycle in percent (0-100).
    pub max_duty_percent: u8,
}

impl Default for MotorPwmConfig {
    fn default() -> Self {
        Self {
            pwm_freq: 20_000,     // 20 kHz
            dead_time_ns: 2000,   // ~2 µs
            max_duty_percent: 15, // 15% for very safe initial testing
        }
    }
}

/// Motor PWM controller using TIM1 with complementary outputs.
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, embassy_stm32::peripherals::TIM1>,
    max_duty: u16,
    duty_limit: u16,
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

        let pwm_freq = khz(config.pwm_freq / 1000);

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

        // Calculate dead-time based on timer clock frequency.
        // STM32G4 runs at 170MHz.
        const TIMER_CLOCK_HZ: u32 = 170_000_000;
        let dead_time_ticks =
            ((config.dead_time_ns as u64 * TIMER_CLOCK_HZ as u64) / 1_000_000_000) as u16;
        pwm.set_dead_time(dead_time_ticks);

        // Enable OSSR for safer off-state behavior when channels are disabled.
        // IDLE_LEVEL means outputs go to their idle state when disabled.
        pwm.set_off_state_selection_run(Ossr::IDLE_LEVEL);

        // Channel 4: internal "sampling" channel to generate TIM1_TRGO2 for ADC.
        //
        // In center-aligned mode, setting duty to max_duty / 2 places the compare
        // event near the middle of the PWM period.
        let mid = max_duty / 2;
        pwm.set_duty(Channel::Ch4, mid);
        pwm.enable(Channel::Ch4);
        pwm.set_mms2(Mms2::COMPARE_OC4);

        // Calculate duty cycle limit based on max_duty_percent.
        let duty_limit = (max_duty as u32 * config.max_duty_percent as u32 / 100) as u16;

        defmt::info!(
            "Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq,
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

    /// Set duty cycle for a specific phase (0-100%).
    ///
    /// Duty is clamped to the configured max_duty_percent.
    #[allow(dead_code)]
    pub fn set_phase_duty(&mut self, channel: Channel, duty_percent: u8) {
        let duty_percent = duty_percent.min(100);
        let duty = (self.max_duty as u32 * duty_percent as u32 / 100) as u16;
        let duty = duty.min(self.duty_limit);
        self.pwm.set_duty(channel, duty);
    }

    /// Disable a specific phase (all FETs off for that phase).
    #[allow(dead_code)]
    pub fn disable_phase(&mut self, channel: Channel) {
        self.pwm.set_duty(channel, 0);
        self.pwm.disable(channel);
    }

    /// Apply 6-step commutation pattern.
    ///
    /// Traditional 6-step behavior:
    /// - `PhaseState::High`: High-side FET PWMs at duty, low-side follows complementary with dead-time
    /// - `PhaseState::Low`: High-side OFF, low-side ON continuously (0% duty = low-side always on)
    /// - `PhaseState::Off`: Both FETs off (floating phase)
    pub fn apply_commutation(
        &mut self,
        duty_percent: u8,
        ph_a_state: PhaseState,
        ph_b_state: PhaseState,
        ph_c_state: PhaseState,
    ) {
        let duty_percent = duty_percent.min(100);
        let duty = (self.max_duty as u32 * duty_percent as u32 / 100) as u16;
        let duty = duty.min(self.duty_limit);

        self.apply_phase_state(Channel::Ch1, ph_a_state, duty);
        self.apply_phase_state(Channel::Ch2, ph_b_state, duty);
        self.apply_phase_state(Channel::Ch3, ph_c_state, duty);
    }

    /// Apply a commutation step directly.
    #[allow(dead_code)]
    pub fn apply_step(&mut self, step: CommutationStep, duty_percent: u8) {
        let (a, b, c) = step.get_phase_states();
        self.apply_commutation(duty_percent, a, b, c);
    }

    /// Apply the drive state for a single phase.
    fn apply_phase_state(&mut self, channel: Channel, state: PhaseState, duty: u16) {
        match state {
            PhaseState::Off => {
                // Floating: both FETs off
                self.pwm.set_duty(channel, 0);
                self.pwm.disable(channel);
            }
            PhaseState::High => {
                // High-side PWMs at duty, low-side complementary
                self.pwm.set_duty(channel, duty);
                self.pwm.enable(channel);
            }
            PhaseState::Low => {
                // High-side OFF, low-side ON (sink current)
                // With complementary PWM: 0% duty = high-side off, low-side on
                self.pwm.set_duty(channel, 0);
                self.pwm.enable(channel);
            }
        }
    }

    /// Emergency stop - disable all phases immediately.
    pub fn emergency_stop(&mut self) {
        self.apply_commutation(0, PhaseState::Off, PhaseState::Off, PhaseState::Off);
    }

    /// Get maximum duty cycle value.
    #[allow(dead_code)]
    pub fn get_max_duty(&self) -> u16 {
        self.max_duty
    }
}

// ========== Implement PhasePwm trait for FOC integration ==========

use oxifoc_core::foc::pwm::PhasePwm;

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

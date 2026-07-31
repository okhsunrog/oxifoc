//! Motor control for Simple FOCer 2 (STM32F405)
//!
//! TIM1 complementary PWM configuration for 3-phase BLDC/PMSM control
//! with dead-time insertion for shoot-through protection and
//! center-aligned PWM for optimal ADC sampling.

use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::PwmPin;

use crate::hardware::resources::MotorResources;
use oxifoc_core::foc::pwm::{self, MotorPwmConfig, PhasePwm, PhaseState};

/// Motor PWM controller using TIM1
///
/// # Pin mapping (Simple FOCer 2 / VESC):
/// - Phase A: TIM1_CH1 (PA8) / TIM1_CH1N (PB13)
/// - Phase B: TIM1_CH2 (PA9) / TIM1_CH2N (PB14)
/// - Phase C: TIM1_CH3 (PA10) / TIM1_CH3N (PB15)
pub struct MotorPwm<'d> {
    pwm: ComplementaryPwm<'d, peripherals::TIM1>,
    max_duty: u32,
    duty_limit: u32,
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

        let pwm_freq = Hertz(config.pwm_freq_hz);

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

        // Calculate dead time from actual timer clock frequency
        let tim1_clock_hz = embassy_stm32::rcc::frequency::<peripherals::TIM1>().0;
        let dead_time = pwm::dead_time_ticks(config.dead_time_ns, tim1_clock_hz);
        pwm.set_dead_time(dead_time);

        // Calculate duty limit using shared helper
        let duty_limit = u32::from(pwm::duty_limit(max_duty as u16, config.max_duty_percent));

        defmt::info!(
            "F405 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq_hz,
            max_duty,
            config.max_duty_percent
        );

        // Configure CH4 compare for ADC trigger at PWM peak.
        // CH4 NOT enabled here — enable_adc_trigger() does it after ADC handles are installed.
        // Note: embassy ComplementaryPwm on F405 only exposes 3 channels (CH1-3),
        // so CH4 must be configured via PAC directly.
        let trigger_point = max_duty - max_duty / 50; // ~2% margin from peak
        embassy_stm32::pac::TIM1
            .ccr(3)
            .write(|w| w.set_ccr(trigger_point as u16));
        defmt::info!("TIM1 CH4 ADC trigger at {}", trigger_point);

        // NOTE: Phase channels and CH4 are NOT enabled here.
        // Call enable_adc_trigger() after ADC handles are installed so the ISR
        // can process conversions when CH4 starts triggering.

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    /// Enable only the internal ADC trigger. Phase channels remain high-Z
    /// until the ISR-owned driver explicitly selects a topology.
    pub fn enable_adc_trigger(&mut self) {
        // CH4 has no complementary output, so enable via PAC directly
        // (ComplementaryPwm::enable would try set_ccne(3) which asserts n<3)
        embassy_stm32::pac::TIM1
            .ccer()
            .modify(|w| w.set_cce(3, true));
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
        self.max_duty as u16
    }

    fn set_duties(&mut self, duties: [u16; 3]) {
        // Clamp to duty limit for safety
        let duty_a = u32::from(duties[0]).min(self.duty_limit);
        let duty_b = u32::from(duties[1]).min(self.duty_limit);
        let duty_c = u32::from(duties[2]).min(self.duty_limit);

        self.pwm.set_duty(Channel::Ch1, duty_a);
        self.pwm.set_duty(Channel::Ch2, duty_b);
        self.pwm.set_duty(Channel::Ch3, duty_c);
    }

    fn disable(&mut self) {
        self.emergency_stop();
    }

    fn enable(&mut self) {
        // Must be overridden: the trait default is a no-op, but our disable()
        // turns the channels off, so FocDriver's Stopped→active transition
        // (set_mode → pwm.enable()) would otherwise write duties into disabled
        // channels and the motor could never start in FOC modes.
        // Set duties to 0 before enabling channels to prevent a glitch.
        self.pwm.set_duty(Channel::Ch1, 0);
        self.pwm.set_duty(Channel::Ch2, 0);
        self.pwm.set_duty(Channel::Ch3, 0);
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
        // CH4 (ADC trigger) is never disabled by emergency_stop(), so the
        // sampling/ISR chain keeps running across disable/enable cycles.
    }

    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        const CHANNELS: [Channel; 3] = [Channel::Ch1, Channel::Ch2, Channel::Ch3];
        for (state, &ch) in states.iter().zip(CHANNELS.iter()) {
            match state {
                PhaseState::Pwm(duty) => {
                    self.pwm.enable(ch);
                    self.pwm.set_duty(ch, u32::from(*duty).min(self.duty_limit));
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

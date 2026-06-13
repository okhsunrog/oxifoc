//! Motor control for B-G431B-ESC1
//!
//! TIM1 complementary PWM configuration for 3-phase BLDC motor control.

use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin, Mms2, Ossr};
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::low_level::{BreakInputPolarity, FilterValue};
use embassy_stm32::timer::simple_pwm::PwmPin;

use crate::hardware::MotorResources;
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

        // Calculate dead time from actual timer clock frequency
        let tim1_clock_hz = embassy_stm32::rcc::frequency::<embassy_stm32::peripherals::TIM1>().0;
        let dead_time = pwm::dead_time_ticks(config.dead_time_ns, tim1_clock_hz);
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
        // Ch4 NOT enabled here — enable_outputs() does it after ADC handles are installed.
        pwm.set_mms2(Mms2::COMPARE_OC4);

        // Route COMP1/2/4 outputs to TIM1 BKIN for hardware overcurrent protection.
        // COMP index: COMP1=0, COMP2=1, COMP4=3.
        // When any comparator output goes high (overcurrent), TIM1 MOE is cleared
        // and all PWM outputs are forced to safe state in hardware.
        pwm.set_break_comparator_enable(0, true); // COMP1 (phase A)
        pwm.set_break_comparator_enable(1, true); // COMP2 (phase B)
        pwm.set_break_comparator_enable(3, true); // COMP4 (phase C)
        // Disable external BKIN pin input (AF1.BKINE defaults to 1, pin may float)
        pwm.set_break_input_pin_enable(false);
        pwm.set_break_polarity(BreakInputPolarity::ACTIVE_HIGH);
        // Digital glitch filter on the break input: the comparators see
        // switching-edge noise on the shunt signals and trip falsely without
        // it. ST's MCSDK project for this same board uses BreakFilter=4
        // (fSAMPLING=fDTS/2, N=6) with COMP hysteresis/blanking both off —
        // a real overcurrent easily outlasts the 6-sample filter.
        pwm.set_break_filter(FilterValue::FDTS_DIV2_N6);
        // DISABLED — the COMP→BKIN break is unusable on this board (proven on
        // hardware 2026-06-13). A DAC sweep showed the comparators tap the raw
        // shunt pad (idle ~128 mV, only ~1.71 mV/A — the ×16 PGA gain is
        // downstream, invisible to them), so no real current threshold clears
        // the PWM switching noise. We tried ST's near-rail threshold
        // (config::HW_OCP_DAC_COUNTS ≈ 3.29 V) with the break ENABLED: the
        // device tripped to Error on the FIRST PWM-output enable, every time,
        // BEFORE any current flows — capacitive coupling from the gate-driver
        // turn-on transient spikes the high-impedance pad node to the rail. With
        // the break armed the motor cannot start at all. ST gets away with the
        // near-rail value because MCSDK sequences the enable through a controlled
        // boot-cap-charge phase and does not latch an enable-window break as a
        // fatal fault; we don't replicate that. Protection here is the software
        // measured-overcurrent trip (BOARD.max_phase_current_a, 40 A, read from
        // the ×9.14-amplified ADC signal) + the bench PSU current limit. The
        // COMP+DAC are still configured (near-rail) so re-arming is a one-liner
        // if enable-sequencing is added later. See docs/hw/b-g431b-esc1.md.
        pwm.set_break_enable(false);

        // Calculate duty limit using shared helper (convert to u16 for helper, then back to u32)
        let duty_limit = u32::from(pwm::duty_limit(max_duty as u16, config.max_duty_percent));

        defmt::info!(
            "G431 Motor PWM init: freq={}Hz, max_duty={}, limit={}%",
            config.pwm_freq_hz,
            max_duty,
            config.max_duty_percent
        );

        // NOTE: Phase channels are NOT enabled here.
        // Call enable_outputs() after FOC init is complete.
        // Enabling channels triggers ADC ISR at 20kHz which starves the main task.

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    /// Enable PWM outputs (phase channels + ADC trigger).
    /// Call after ADC handles are installed so the ISR can process conversions.
    pub fn enable_outputs(&mut self) {
        self.pwm.enable(Channel::Ch4); // ADC trigger via TIM1_TRGO2
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
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
        use embassy_stm32::pac;
        // Set duties to 0 before enabling channels to prevent glitch
        self.pwm.set_duty(Channel::Ch1, 0);
        self.pwm.set_duty(Channel::Ch2, 0);
        self.pwm.set_duty(Channel::Ch3, 0);
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
        // Clear any spurious break flag from channel enable transient
        // and re-enable MOE (Master Output Enable) in case BKIN tripped.
        oxifoc_core::clear_rc_w0!(pac::TIM1.sr(), |w| w.set_bif(0, false));
        pac::TIM1.bdtr().modify(|w| w.set_moe(true));
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

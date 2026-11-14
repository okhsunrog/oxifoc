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

/// Boot-only frozen UEV observations: CNT[14:0], DIR[15], UIF[16], CEN[17].
#[cfg(feature = "bench-pwm-commit")]
#[unsafe(no_mangle)]
pub static BENCH_PWM_UPDATE_WITNESSES: [core::sync::atomic::AtomicU32; 6] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 6];
/// Published as 1 only after all six phase witnesses pass.
#[cfg(feature = "bench-pwm-commit")]
#[unsafe(no_mangle)]
pub static BENCH_PWM_UPDATE_WITNESSES_READY: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

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
    /// One ADC/FOC cycle per full center-aligned TIM1 period. The counter runs
    /// 0..ARR..0 in 2*ARR ticks, unlike edge-aligned PWM's ARR+1 period.
    pub fn control_timing(&self) -> oxifoc_core::types::ControlTiming {
        let regs = embassy_stm32::pac::TIM1;
        let period_ticks =
            2 * u32::from(regs.arr().read().arr()) * (u32::from(regs.psc().read()) + 1);
        let clock_hz = embassy_stm32::rcc::frequency::<peripherals::TIM1>().0;
        let timing = oxifoc_core::types::ControlTiming {
            clock_hz,
            period_ticks,
        };
        defmt::assert!(timing.is_valid());
        timing
    }

    /// Nominal sample-to-actuation delay for the peak-only PWM preload latch.
    /// The next peak is approximately one full period after the near-peak ADC
    /// aperture; its small aperture/synchronization offset is not calibrated.
    pub const fn actuation_delay_cycles(&self) -> f32 {
        1.0
    }

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

        let tim = embassy_stm32::pac::TIM1;
        let tim1_clock_hz = embassy_stm32::rcc::frequency::<peripherals::TIM1>().0;
        // Use the same half-period rounding at boot and during detection.
        // Leave the HAL in charge of configuring the complementary outputs.
        let arr = Self::center_aligned_arr(tim1_clock_hz, config.pwm_freq_hz);
        tim.cr1().modify(|w| w.set_cen(false));
        tim.psc().write_value(0);
        tim.arr().write(|w| w.set_arr(arr));
        let max_duty = u32::from(arr);
        #[cfg(any(feature = "bench-executor-trace", feature = "bench-pwm-commit"))]
        assert!(
            max_duty < 0x8000,
            "PWM trace requires a 15-bit TIM1 counter"
        );

        // Calculate dead time from actual timer clock frequency
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

        #[cfg(feature = "bench-pwm-commit")]
        Self::verify_peak_updates(max_duty as u16);
        // The HAL constructor has already issued UG and started TIM1. Re-prime
        // after CCR4 is ready, independent of that arbitrary initial phase.
        Self::prepare_peak_updates();
        embassy_stm32::pac::TIM1.bdtr().modify(|w| w.set_moe(true));
        embassy_stm32::pac::TIM1.cr1().modify(|w| w.set_cen(true));

        // NOTE: Phase channels and CH4 are NOT enabled here.
        // Call enable_adc_trigger() after ADC handles are installed so the ISR
        // can process conversions when CH4 starts triggering.

        Self {
            pwm,
            max_duty,
            duty_limit,
        }
    }

    pub(crate) fn center_aligned_arr(clock_hz: u32, pwm_hz: u32) -> u16 {
        let arr = defmt::unwrap!(u16::try_from(clock_hz / (2 * pwm_hz)).ok());
        defmt::assert!(arr != 0);
        arr
    }

    /// One-time stopped setup: transfer CCR preloads only at the PWM peak.
    /// Phase topology changes through CCER remain immediate.
    fn prepare_peak_updates() {
        use embassy_stm32::pac::timer::vals::{Cms, Dir, Urs};
        let tim = embassy_stm32::pac::TIM1;
        tim.cr1().modify(|w| {
            w.set_cen(false);
            w.set_opm(false);
            w.set_udis(false);
            w.set_urs(Urs::CounterOnly);
        });
        tim.ccer().modify(|w| {
            for ch in 0..4 {
                w.set_cce(ch, false);
            }
            for ch in 0..3 {
                w.set_ccne(ch, false);
            }
        });
        tim.bdtr().modify(|w| w.set_moe(false));
        tim.dier().modify(|w| {
            w.set_uie(false);
            w.set_ude(false);
        });
        // DIR is read-only in center-aligned mode. RM0090 also disallows
        // changing CMS and DIR together, so establish edge mode first.
        tim.cr1().modify(|w| w.set_cms(Cms::EdgeAligned));
        tim.cr1().modify(|w| w.set_dir(Dir::Up));
        tim.cnt().write(|w| w.set_cnt(0));
        for ch in 0..3 {
            tim.ccr(ch).write(|w| w.set_ccr(0));
        }
        // UG loads PSC/ARR/CCR and internal REP=0 while stopped. RCR is itself
        // preloaded: writing 1 AFTER UG lets the first natural peak UEV load it.
        // Thereafter each valley only decrements REP; peaks transfer preloads.
        tim.rcr().write(|w| w.set_rep(0));
        tim.egr().write(|w| w.set_ug(true));
        tim.cr1().modify(|w| w.set_cms(Cms::CenterAligned3));
        tim.rcr().write(|w| w.set_rep(1));
        // No UG after RCR=1: that would change the chosen UEV phase. URS=1
        // excludes UG from UIF; clear stale compare/update flags before start.
        tim.sr().write(|_| {});
    }

    #[cfg(feature = "bench-pwm-commit")]
    fn verify_peak_updates(arr: u16) {
        use core::sync::atomic::Ordering;
        use embassy_stm32::pac::timer::vals::Dir;
        let tim = embassy_stm32::pac::TIM1;
        BENCH_PWM_UPDATE_WITNESSES_READY.store(0, Ordering::Relaxed);
        Self::prepare_peak_updates();
        tim.cr1().modify(|w| w.set_opm(true));
        for witness in 0..3 {
            tim.sr().write(|_| {});
            tim.cr1().modify(|w| w.set_cen(true));
            Self::verify_peak_witness(witness, arr);
        }

        // Let the first valley decrement REP with UDIS=0, then arm UDIS in
        // mid-up to suppress only a peak. Starting UDIS at the frozen peak
        // would cover two extrema and could hide a frozen repetition counter.
        tim.cr1().modify(|w| {
            w.set_opm(false);
            w.set_udis(false);
        });
        tim.sr().write(|_| {});
        tim.cr1().modify(|w| w.set_cen(true));
        let saw_valley = Self::wait_pwm_condition(|| tim.cr1().read().dir() == Dir::Up);
        let middle_up = saw_valley
            && Self::wait_pwm_condition(|| {
                let up = tim.cr1().read().dir() == Dir::Up;
                let count = tim.cnt().read().cnt();
                up && (arr / 3..=arr / 2).contains(&count)
            });
        let (count, down, update, running) = Self::record_update_witness(3);
        defmt::assert!(
            saw_valley
                && middle_up
                && !down
                && !update
                && running
                && (arr / 3..=arr / 2).contains(&count),
            "TIM1 pre-suppression checkpoint failed"
        );
        tim.cr1().modify(|w| w.set_udis(true));
        let saw_peak = Self::wait_pwm_condition(|| tim.cr1().read().dir() == Dir::Down);
        let middle_down = saw_peak
            && Self::wait_pwm_condition(|| {
                let down = tim.cr1().read().dir() == Dir::Down;
                let count = tim.cnt().read().cnt();
                down && (arr / 3..=arr / 2).contains(&count)
            });
        // Preserve a failed checkpoint in slot 3 before asserting. No logging
        // on the successful path between this phase check and releasing UDIS.
        let (count, down, update, running) = Self::record_update_witness(3);
        defmt::assert!(
            saw_peak
                && middle_down
                && down
                && !update
                && running
                && (arr / 3..=arr / 2).contains(&count),
            "TIM1 suppressed UEV checkpoint failed"
        );
        tim.cr1().modify(|w| w.set_opm(true));
        tim.cr1().modify(|w| w.set_udis(false));
        Self::verify_peak_witness(3, arr);
        for witness in 4..6 {
            tim.sr().write(|_| {});
            tim.cr1().modify(|w| w.set_cen(true));
            Self::verify_peak_witness(witness, arr);
        }
        BENCH_PWM_UPDATE_WITNESSES_READY.store(1, Ordering::Release);
        // new() runs the exact normal prepare/start sequence after this test.
    }

    #[cfg(feature = "bench-pwm-commit")]
    fn wait_pwm_condition(mut ready: impl FnMut() -> bool) -> bool {
        use embassy_time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_millis(2);
        while !ready() {
            if Instant::now() >= deadline {
                return false;
            }
        }
        true
    }

    #[cfg(feature = "bench-pwm-commit")]
    fn record_update_witness(witness: usize) -> (u16, bool, bool, bool) {
        use core::sync::atomic::Ordering;
        use embassy_stm32::pac::timer::vals::Dir;
        let tim = embassy_stm32::pac::TIM1;
        let down = tim.cr1().read().dir() == Dir::Down;
        let count = tim.cnt().read().cnt();
        let update = tim.sr().read().uif();
        let running = tim.cr1().read().cen();
        BENCH_PWM_UPDATE_WITNESSES[witness].store(
            u32::from(count)
                | (u32::from(down) << 15)
                | (u32::from(update) << 16)
                | (u32::from(running) << 17),
            Ordering::Relaxed,
        );
        (count, down, update, running)
    }

    #[cfg(feature = "bench-pwm-commit")]
    fn verify_peak_witness(witness: usize, arr: u16) {
        let tim = embassy_stm32::pac::TIM1;
        let stopped = Self::wait_pwm_condition(|| !tim.cr1().read().cen());
        // OPM freezes the phase at UEV. CH4, MOE and every phase output are off.
        let (count, down, update, running) = Self::record_update_witness(witness);
        defmt::info!(
            "TIM1 peak UEV {}: CNT={} ARR={} down={} UIF={} CEN={} RCR={}",
            witness + 1,
            count,
            arr,
            down,
            update,
            running,
            tim.rcr().read().rep()
        );
        defmt::assert!(
            stopped && !running && update && down && count >= arr.saturating_sub(2),
            "TIM1 peak UEV phase verification failed"
        );
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

    #[cfg(feature = "board-vesc6-mk5")]
    pub(crate) fn install_stopped_period(&mut self, arr: u16) {
        let tim = embassy_stm32::pac::TIM1;
        tim.cr1().modify(|w| w.set_cen(false));
        tim.arr().write(|w| w.set_arr(arr));
        tim.ccr(3).write(|w| w.set_ccr(arr - arr / 50));
        Self::prepare_peak_updates();
        self.max_duty = u32::from(arr);
        self.duty_limit = u32::from(pwm::duty_limit(
            arr,
            crate::config::PWM_CONFIG.max_duty_percent,
        ));
    }

    #[cfg(feature = "board-vesc6-mk5")]
    pub(crate) fn resume_control_timer() {
        let tim = embassy_stm32::pac::TIM1;
        tim.ccer().modify(|w| w.set_cce(3, true));
        tim.bdtr().modify(|w| w.set_moe(true));
        tim.cr1().modify(|w| w.set_cen(true));
    }

    /// Emergency stop - disable all outputs
    pub fn emergency_stop(&mut self) {
        self.write_pwm_preloads(|this| {
            this.pwm.set_duty(Channel::Ch1, 0);
            this.pwm.set_duty(Channel::Ch2, 0);
            this.pwm.set_duty(Channel::Ch3, 0);
        });
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }

    /// Keep CCR preloads coherent without stopping the counter. A batch that
    /// spans a peak can hold the old vector for another full PWM period.
    #[inline(always)]
    fn write_pwm_preloads(&mut self, write: impl FnOnce(&mut Self)) {
        #[cfg(any(feature = "bench-executor-trace", feature = "bench-pwm-commit"))]
        let before = Self::count_direction();
        let tim = embassy_stm32::pac::TIM1;
        let mut previous_udis = false;
        tim.cr1().modify(|w| {
            previous_udis = w.udis();
            w.set_udis(true);
        });
        write(self);
        // Preserve the caller's update-disable state without restoring live
        // counter/direction bits or generating an update event.
        tim.cr1().modify(|w| w.set_udis(previous_udis));
        #[cfg(feature = "bench-foc-deadline")]
        crate::diagnostics::pwm_deadline::committed();
        #[cfg(any(feature = "bench-executor-trace", feature = "bench-pwm-commit"))]
        {
            let after = Self::count_direction();
            // Append only after restoring UDIS; tracing must not extend its window.
            crate::diagnostics::trace::record_pwm_write(before, after);
        }
    }

    #[cfg(any(feature = "bench-executor-trace", feature = "bench-pwm-commit"))]
    #[inline(always)]
    fn count_direction() -> u16 {
        let tim = embassy_stm32::pac::TIM1;
        let count = tim.cnt().read().cnt();
        let down = tim.cr1().read().dir() == embassy_stm32::pac::timer::vals::Dir::Down;
        // ARR < 32768 is checked at init. The two register reads can straddle
        // a turn; the near-boundary count remains visible in the diagnostic.
        count | (u16::from(down) << 15)
    }

    /// All phase-enable paths pass through this compile-time output inhibit.
    /// ADC trigger CH4 is independent and must keep running for bench loads.
    fn set_phase_enabled(&mut self, channel: Channel, enabled: bool) {
        let permitted = !cfg!(feature = "bench-no-phase-outputs")
            && (!cfg!(feature = "bench-only-phase-a") || matches!(channel, Channel::Ch1));
        if enabled && permitted {
            self.pwm.enable(channel);
        } else {
            self.pwm.disable(channel);
        }
    }
}

impl<'d> PhasePwm for MotorPwm<'d> {
    fn max_duty(&self) -> u16 {
        self.max_duty as u16
    }

    fn duty_limit(&self) -> u16 {
        self.duty_limit as u16
    }

    fn set_duties(&mut self, duties: [u16; 3]) {
        // Clamp to duty limit for safety (the controller already keeps the
        // vector under it — see PhasePwm::duty_limit)
        let duty_a = u32::from(duties[0]).min(self.duty_limit);
        let duty_b = u32::from(duties[1]).min(self.duty_limit);
        let duty_c = u32::from(duties[2]).min(self.duty_limit);

        self.write_pwm_preloads(|this| {
            this.pwm.set_duty(Channel::Ch1, duty_a);
            this.pwm.set_duty(Channel::Ch2, duty_b);
            this.pwm.set_duty(Channel::Ch3, duty_c);
        });
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
        self.write_pwm_preloads(|this| {
            this.pwm.set_duty(Channel::Ch1, 0);
            this.pwm.set_duty(Channel::Ch2, 0);
            this.pwm.set_duty(Channel::Ch3, 0);
        });
        self.set_phase_enabled(Channel::Ch1, true);
        self.set_phase_enabled(Channel::Ch2, true);
        self.set_phase_enabled(Channel::Ch3, true);
        // CH4 (ADC trigger) is never disabled by emergency_stop(), so the
        // sampling/ISR chain keeps running across disable/enable cycles.
    }

    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        const CHANNELS: [Channel; 3] = [Channel::Ch1, Channel::Ch2, Channel::Ch3];
        self.write_pwm_preloads(|this| {
            for (state, &ch) in states.iter().zip(CHANNELS.iter()) {
                match state {
                    PhaseState::Pwm(duty) => {
                        this.set_phase_enabled(ch, true);
                        this.pwm.set_duty(ch, u32::from(*duty).min(this.duty_limit));
                    }
                    PhaseState::Low => {
                        this.set_phase_enabled(ch, true);
                        this.pwm.set_duty(ch, 0);
                    }
                    PhaseState::Float => {
                        this.pwm.set_duty(ch, 0);
                        this.pwm.disable(ch);
                    }
                }
            }
        });
    }
}

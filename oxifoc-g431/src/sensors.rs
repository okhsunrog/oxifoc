//! Sensor implementations for motor control

pub mod hall {
    //! Hall acquisition via the TIM4 hall-sensor interface.
    //!
    //! PB6/PB7/PB8 are TIM4_CH1/2/3 (AF2) — the board routes the hall
    //! connector there precisely for this. `CR2.TI1S` XORs the three inputs
    //! into TI1; IC1 captures the filtered XOR signal on both edges, so every
    //! hall transition latches its timestamp in hardware. The capture ISR
    //! (below the FOC ISR in priority) only picks the latched value up — its
    //! latency no longer affects hall timing, unlike the 200 kHz TIM6
    //! polling this replaces. The ICF input filter (~6 µs stable window)
    //! replaces the 7-read majority vote, and the free-running 1 MHz counter
    //! replaces 32768 Hz embassy timestamps.

    use core::cell::RefCell;
    use core::sync::atomic::{AtomicU32, Ordering};

    use embassy_stm32::gpio::Pull;
    use embassy_stm32::interrupt::typelevel::Interrupt;
    use embassy_stm32::timer::input_capture::CapturePin;
    use embassy_stm32::timer::low_level::{InputCaptureMode, InputTISelection, Timer};
    use embassy_stm32::timer::{Ch1, Ch2, Ch3, Channel};
    use embassy_stm32::{Peri, interrupt, pac, peripherals};
    use embassy_sync::blocking_mutex::CriticalSectionMutex;

    use oxifoc_core::foc::capture_timebase::CaptureTimebase;
    use oxifoc_core::foc::hall_embassy::{set_tick_source, update_hall_edge};

    // Re-export shared items from core
    pub use oxifoc_core::foc::hall_embassy::{
        HallAngleProxy, apply_stored_config, get_snapshot, init_estimator,
    };

    /// Hall timebase tick rate: TIM4 counts at 1 MHz.
    pub const HALL_TICKS_PER_SEC: u64 = 1_000_000;

    /// Keeps TIM4 alive (RCC enabled / not reused); the ISR talks to
    /// `pac::TIM4` directly and never takes this lock.
    static TIM_DRIVER: CriticalSectionMutex<RefCell<Option<Timer<'static, peripherals::TIM4>>>> =
        CriticalSectionMutex::new(RefCell::new(None));

    /// 16-bit CCR/CNT → 64-bit tick extension (overflow accounting).
    static TIMEBASE: CaptureTimebase<u16> = CaptureTimebase::new();

    /// Capture overruns: an edge arrived before the previous one was picked
    /// up, so one timestamp was lost (the estimator then sees a wider
    /// sector). Diagnostics only.
    pub static OVERCAPTURES: AtomicU32 = AtomicU32::new(0);

    /// "Now" in hall ticks (µs), assembled from TIM4 CNT + overflow count.
    ///
    /// Valid after [`init_hall`]. Safe from any context: the FOC ISR cannot
    /// observe a torn overflow update (the writer holds a critical section),
    /// thread context retries on concurrent updates.
    pub fn now_ticks() -> u64 {
        TIMEBASE.now(|| {
            // CNT before UIF: if the wrap lands between the two reads, UIF
            // is seen with an upper-half CNT and correctly not re-counted.
            let cnt = pac::TIM4.cnt().read().cnt();
            let uif = pac::TIM4.sr().read().uif();
            (cnt, uif)
        })
    }

    /// Clear selected TIM4 status flags (race-free rc_w0 complement write,
    /// see [`oxifoc_core::clear_rc_w0`]).
    fn clear_flags(uif: bool, cc1of: bool) {
        oxifoc_core::clear_rc_w0!(pac::TIM4.sr(), |w| {
            if uif {
                w.set_uif(false);
            }
            if cc1of {
                w.set_ccof(0, false);
            }
        });
    }

    /// Initialize hall acquisition: pins to TIM4 AF, XOR + capture setup.
    pub fn init_hall(
        pb6: Peri<'static, peripherals::PB6>,
        pb7: Peri<'static, peripherals::PB7>,
        pb8: Peri<'static, peripherals::PB8>,
        tim4: Peri<'static, peripherals::TIM4>,
    ) {
        // Pins to TIM4 channel inputs with pull-ups (CapturePin sets AF).
        // GPIO IDR still reflects pin levels in AF mode, so raw state reads
        // for calibration keep working.
        let ch1: CapturePin<'static, peripherals::TIM4, Ch1> = CapturePin::new(pb6, Pull::Up);
        let ch2: CapturePin<'static, peripherals::TIM4, Ch2> = CapturePin::new(pb7, Pull::Up);
        let ch3: CapturePin<'static, peripherals::TIM4, Ch3> = CapturePin::new(pb8, Pull::Up);
        core::mem::forget((ch1, ch2, ch3));

        init_estimator(HALL_TICKS_PER_SEC);
        set_tick_source(now_ticks);

        let timer = Timer::new(tim4);
        let clk = timer.get_clock_frequency().0 as u64;
        // Derive PSC from the actual RCC config — TIM4 is on APB1, whose
        // timer clock doubles when the APB prescaler is > 1. Hardcoding
        // would silently skew every hall velocity by that factor.
        let psc = clk / HALL_TICKS_PER_SEC - 1;
        defmt::assert!(
            clk.is_multiple_of(HALL_TICKS_PER_SEC) && psc <= u16::MAX as u64,
            "TIM4 clock {} not divisible to 1 MHz",
            clk
        );
        let regs = timer.regs_gp16();
        regs.psc().write_value(psc as u16);
        regs.arr().write(|w| w.set_arr(0xFFFF));
        // f_DTS = timer clock / 4; ICF = f_DTS/32, 8 samples: the XOR signal
        // must be stable ~6 µs before an edge is accepted. Hardware debounce
        // replacing the 7-read majority vote — and unlike software voting,
        // the filter delay is identical for every edge, so it cancels in
        // velocity math.
        regs.cr1()
            .modify(|w| w.set_ckd(pac::timer::vals::Ckd::DIV4));
        // XOR CH1^CH2^CH3 → TI1: any single hall transition flips parity.
        regs.cr2()
            .modify(|w| w.set_ti1s(pac::timer::vals::Ti1s::XOR));
        timer.set_input_ti_selection(Channel::Ch1, InputTISelection::Normal);
        timer.set_input_capture_filter(Channel::Ch1, pac::timer::vals::FilterValue::FDTS_DIV32_N8);
        timer.set_input_capture_mode(Channel::Ch1, InputCaptureMode::BothEdges);
        // Latch PSC into the shadow register (UG sets UIF as a side effect —
        // clear before enabling interrupts).
        regs.egr().write(|w| w.set_ug(true));
        clear_flags(true, true);
        timer.enable_channel(Channel::Ch1, true);
        timer.enable_input_interrupt(Channel::Ch1, true);
        timer.enable_update_interrupt(true);
        timer.start();

        TIM_DRIVER.lock(|cell| cell.replace(Some(timer)));

        unsafe {
            interrupt::typelevel::TIM4::unpend();
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM4);
            let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
            // NVIC::set_priority takes the RAW 8-bit IPR value; STM32
            // implements only the upper 4 bits. Below the FOC ADC ISR is
            // fine: edge timestamps are latched in hardware, so delaying
            // this handler only delays when the estimator learns of the
            // edge, not the timestamp itself.
            nvic.set_priority(interrupt::TIM4, 1 << 4);
        }

        defmt::info!(
            "Hall: TIM4 XOR capture @ 1 MHz (clk {} Hz, psc {}), ICF ~6us, H1=PB6 H2=PB7 H3=PB8",
            clk,
            psc
        );
    }

    /// Read Hall state from GPIOB IDR in a single register access.
    /// PB6=H1 (bit 0), PB7=H2 (bit 1), PB8=H3 (bit 2)
    #[inline(always)]
    fn read_hall_idr() -> u8 {
        ((pac::GPIOB.idr().read().0 >> 6) & 0b111) as u8
    }

    /// Read raw Hall sensor state from GPIO (public for calibration).
    #[cfg(feature = "detection")]
    #[inline]
    pub fn read_hall_state_raw() -> u8 {
        read_hall_idr()
    }

    /// TIM4 capture/update interrupt: extend the hardware-latched edge
    /// timestamp and feed the estimator.
    #[interrupt]
    fn TIM4() {
        let regs = pac::TIM4;
        let sr = regs.sr().read();

        if sr.ccof(0) {
            OVERCAPTURES.fetch_add(1, Ordering::Relaxed);
            clear_flags(false, true);
        }

        if sr.ccif(0) {
            // Reading CCR1 clears CC1IF.
            let captured = regs.ccr(0).read().0 as u16;
            let ticks = TIMEBASE.capture(captured, sr.uif(), || clear_flags(true, false));
            update_hall_edge(read_hall_idr(), ticks);
        } else if sr.uif() {
            TIMEBASE.overflow(|| clear_flags(true, false));
        }
    }
}

pub mod current {
    //! Current sensing — uses core's EmbassyCurrentSensor with G431 type aliases
    pub use oxifoc_core::foc::sensors::EmbassyCurrentSensor as G431CurrentSensor;
    pub use oxifoc_core::foc::sensors::EmbassyCurrentSensorExt as G431CurrentSensorExt;
}

pub use current::{G431CurrentSensor, G431CurrentSensorExt};
pub use hall::{HallAngleProxy, init_hall};
// Raw hall-pin read is only consumed by hall calibration (detection).
#[cfg(feature = "detection")]
pub use hall::read_hall_state_raw;

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

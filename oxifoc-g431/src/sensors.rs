//! Sensor implementations for motor control

pub mod hall {
    //! Hall sensor management
    //!
    //! Platform-specific GPIO init and TIM6 ISR live here.
    //! Shared state management (estimator, proxy, snapshot) comes from oxifoc-core.

    use embassy_stm32::gpio::{Input, Pull};
    use embassy_stm32::interrupt::typelevel::Interrupt;
    use embassy_stm32::timer::low_level::{RoundTo, Timer};
    use embassy_stm32::{Peri, interrupt, pac, peripherals};
    use embassy_sync::blocking_mutex::CriticalSectionMutex;

    use core::cell::RefCell;

    use oxifoc_core::foc::sensors::hall_polling::{
        MAJORITY_THRESHOLD, POLL_INTERVAL_US, READS_PER_POLL, majority_vote,
    };

    // Re-export shared items from core
    pub use oxifoc_core::foc::hall_embassy::{
        HallAngleProxy, apply_stored_config, get_snapshot, init_estimator,
    };

    /// TIM6 driver instance for ISR flag clearing.
    static TIM6_DRIVER: CriticalSectionMutex<RefCell<Option<Timer<'static, peripherals::TIM6>>>> =
        CriticalSectionMutex::new(RefCell::new(None));

    /// Initialize Hall sensor inputs and TIM6 for polling
    ///
    /// Configures TIM6 to fire every 5µs. Each ISR performs 7 rapid GPIO reads
    /// with majority voting to filter noise.
    ///
    /// Hall sensors are on PB6, PB7, PB8 — all on GPIOB, allowing single-register reads.
    pub fn init_hall(
        pb6: Peri<'static, peripherals::PB6>,
        pb7: Peri<'static, peripherals::PB7>,
        pb8: Peri<'static, peripherals::PB8>,
        tim6: Peri<'static, peripherals::TIM6>,
        timebase_ticks_per_sec: u64,
    ) {
        // Configure GPIO inputs with pull-up.
        let hall_h1 = Input::new(pb6, Pull::Up);
        let hall_h2 = Input::new(pb7, Pull::Up);
        let hall_h3 = Input::new(pb8, Pull::Up);
        core::mem::forget((hall_h1, hall_h2, hall_h3));
        defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

        // Initialize Hall estimator in core
        init_estimator(timebase_ticks_per_sec);

        // Configure TIM6 for Hall sensor polling
        let timer = Timer::new(tim6);
        timer.set_period_us(POLL_INTERVAL_US, RoundTo::Faster);
        timer.enable_update_interrupt(true);
        timer.set_autoreload_preload(true);
        timer.start();

        TIM6_DRIVER.lock(|cell| cell.replace(Some(timer)));

        unsafe {
            interrupt::typelevel::TIM6_DAC::unpend();
            cortex_m::peripheral::NVIC::unmask(interrupt::TIM6_DAC);
            let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
            nvic.set_priority(interrupt::TIM6_DAC, 1);
        }

        defmt::info!(
            "Hall sensor initialized with TIM6 polling ({}µs interval, {} reads/poll)",
            POLL_INTERVAL_US,
            READS_PER_POLL
        );
    }

    /// Read Hall state from GPIOB IDR in a single register access.
    /// PB6=H1 (bit 0), PB7=H2 (bit 1), PB8=H3 (bit 2)
    #[inline(always)]
    fn read_hall_idr() -> u8 {
        ((pac::GPIOB.idr().read().0 >> 6) & 0b111) as u8
    }

    /// Read raw Hall sensor state from GPIO (public for calibration).
    #[inline]
    pub fn read_hall_state_raw() -> u8 {
        read_hall_idr()
    }

    /// Read Hall sensor state with 7-read majority voting (VESC-style)
    #[inline]
    fn read_hall_state_voted() -> u8 {
        let mut h1_count = 0u8;
        let mut h2_count = 0u8;
        let mut h3_count = 0u8;

        for _ in 0..READS_PER_POLL {
            let state = read_hall_idr();
            if state & 0b001 != 0 {
                h1_count += 1;
            }
            if state & 0b010 != 0 {
                h2_count += 1;
            }
            if state & 0b100 != 0 {
                h3_count += 1;
            }
        }

        majority_vote(h1_count, h2_count, h3_count, MAJORITY_THRESHOLD)
    }

    /// TIM6 update interrupt: poll Hall sensors with majority voting
    #[interrupt]
    fn TIM6_DAC() {
        // Clear update interrupt flag
        TIM6_DRIVER.lock(|cell| {
            if let Some(timer) = cell.borrow().as_ref() {
                timer.clear_update_interrupt();
            }
        });

        let state = read_hall_state_voted();
        oxifoc_core::foc::hall_embassy::update_hall_state(state);
    }
}

pub mod current {
    //! Current sensing — uses core's EmbassyCurrentSensor with G431 type aliases
    pub use oxifoc_core::foc::sensors::EmbassyCurrentSensor as G431CurrentSensor;
    pub use oxifoc_core::foc::sensors::EmbassyCurrentSensorExt as G431CurrentSensorExt;
}

pub use current::{G431CurrentSensor, G431CurrentSensorExt};
pub use hall::{HallAngleProxy, init_hall, read_hall_state_raw};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

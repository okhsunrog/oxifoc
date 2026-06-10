//! Hall sensor management for Simple FOCer 2 (STM32F405)
//!
//! Uses TIM6-based polling at 5µs intervals with 7-read majority voting
//! for noise immunity. This approach filters sub-µs glitches while maintaining
//! good timing resolution.
//!
//! Hall sensors are on PC6, PC7, PC8 - all on GPIOC, allowing single-register reads.
//! Shared state management comes from oxifoc-core hall_embassy.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;

use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;

use oxifoc_core::foc::sensors::hall_polling::{
    MAJORITY_THRESHOLD, POLL_INTERVAL_US, READS_PER_POLL, majority_vote,
};

use crate::config::TIMEBASE_TICKS_PER_SEC;

// Re-export shared items from core
pub use oxifoc_core::foc::hall_embassy::{
    HallAngleProxy, apply_stored_config, get_snapshot, init_estimator,
};

/// TIM6 driver instance for ISR flag clearing.
static TIM6_DRIVER: CriticalSectionMutex<RefCell<Option<Timer<'static, peripherals::TIM6>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Hall Sensor Initialization ==========

/// Initialize Hall sensor inputs and TIM6 for polling
///
/// Configures TIM6 to fire every 5µs. Each ISR performs 7 rapid GPIO reads
/// with majority voting to filter noise.
///
/// GPIO pins are configured as inputs with pull-up, then read directly via
/// GPIOC IDR register for maximum speed (single read for all 3 sensors).
pub fn init_hall(
    pc6: Peri<'static, peripherals::PC6>,
    pc7: Peri<'static, peripherals::PC7>,
    pc8: Peri<'static, peripherals::PC8>,
    tim6: Peri<'static, peripherals::TIM6>,
) {
    // Configure GPIO inputs with pull-up.
    let hall_h1 = Input::new(pc6, Pull::Up);
    let hall_h2 = Input::new(pc7, Pull::Up);
    let hall_h3 = Input::new(pc8, Pull::Up);
    core::mem::forget((hall_h1, hall_h2, hall_h3));
    defmt::info!("Hall sensors configured: H1=PC6, H2=PC7, H3=PC8");

    // Initialize Hall estimator in core
    init_estimator(TIMEBASE_TICKS_PER_SEC);

    // Configure TIM6 for Hall sensor polling using embassy low-level Timer
    let timer = Timer::new(tim6);
    timer.set_period_us(POLL_INTERVAL_US, RoundTo::Faster);
    timer.enable_update_interrupt(true);
    timer.set_autoreload_preload(true);
    timer.start();

    // Store timer for ISR access
    TIM6_DRIVER.lock(|cell| cell.replace(Some(timer)));

    // SAFETY: HALL_ESTIMATOR is initialized above.
    unsafe {
        interrupt::typelevel::TIM6_DAC::unpend();
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM6_DAC);
        // NVIC::set_priority takes the RAW 8-bit IPR value; STM32 implements
        // only the upper 4 bits, so a raw 1 would silently become priority 0
        // — the same level as the FOC ADC ISR, which this poller must NOT
        // preempt or delay. Shift into the implemented bits for a real
        // level 1 (lower number = higher priority).
        let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
        nvic.set_priority(interrupt::TIM6_DAC, 1 << 4);
    }

    defmt::info!(
        "Hall sensor initialized with TIM6 polling ({}µs interval, {} reads/poll)",
        POLL_INTERVAL_US,
        READS_PER_POLL
    );
}

// ========== Fast Hall State Reading ==========

/// Read Hall state from GPIOC IDR in a single register access.
/// PC6=H1 (bit 0), PC7=H2 (bit 1), PC8=H3 (bit 2)
#[inline(always)]
fn read_hall_idr() -> u8 {
    // Single 32-bit read, extract bits 6-8, shift to bits 0-2
    ((pac::GPIOC.idr().read().0 >> 6) & 0b111) as u8
}

/// Read raw Hall sensor state (public API for calibration).
///
/// Returns 3-bit Hall state (0-7): H3<<2 | H2<<1 | H1
///
/// INIT ORDER: init_hall() must be called before any use of this function.
/// GPIO is configured there; TIM6 ISR starts after.
#[inline]
pub fn read_hall_state_raw() -> u8 {
    read_hall_idr()
}

/// Read Hall sensor state with 7-read majority voting (VESC-style)
///
/// Performs 7 rapid single-register reads and returns the state that appears most often.
/// This filters sub-microsecond noise glitches.
#[inline]
fn read_hall_state_voted() -> u8 {
    let mut h1_count = 0u8;
    let mut h2_count = 0u8;
    let mut h3_count = 0u8;

    // 7 rapid reads - each is a single GPIOC IDR access
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

    // Use shared majority voting helper from core
    majority_vote(h1_count, h2_count, h3_count, MAJORITY_THRESHOLD)
}

// ========== TIM6 Interrupt Handler ==========

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

//! Hall sensor management for Simple FOCer 2 (STM32F405)
//!
//! Uses TIM6-based polling at 5µs intervals with 7-read majority voting
//! for noise immunity. This approach filters sub-µs glitches while maintaining
//! good timing resolution.
//!
//! Hall sensors are on PC6, PC7, PC8 - all on GPIOC, allowing single-register reads.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;

use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;

use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use oxifoc_core::foc::sensors::{
    AngleSample, AngleSensor, HallSensorTrait, HallSnapshot,
    hall_polling::{MAJORITY_THRESHOLD, POLL_INTERVAL_US, READS_PER_POLL, majority_vote},
};

use crate::config::TIMEBASE_TICKS_PER_SEC;

// ========== Hall Estimator (shared state) ==========

/// Hall estimator - the single source of truth for Hall sensor state.
/// Accessed by TIM6 ISR (write) and telemetry tasks (read).
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

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
    // We keep them alive with forget() - the configuration persists in hardware.
    // Reading is done directly via GPIOC IDR register for speed.
    let hall_h1 = Input::new(pc6, Pull::Up);
    let hall_h2 = Input::new(pc7, Pull::Up);
    let hall_h3 = Input::new(pc8, Pull::Up);
    core::mem::forget((hall_h1, hall_h2, hall_h3));
    defmt::info!("Hall sensors configured: H1=PC6, H2=PC7, H3=PC8");

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

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
        // Set high priority (lower number = higher priority)
        // Priority 1 to ensure Hall sampling is responsive
        let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
        nvic.set_priority(interrupt::TIM6_DAC, 1);
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
    // Static state for edge detection (transformed to &mut by #[interrupt] macro)
    static mut LAST_STATE: u8 = 0;

    // Clear update interrupt flag
    TIM6_DRIVER.lock(|cell| {
        if let Some(timer) = cell.borrow().as_ref() {
            timer.clear_update_interrupt();
        }
    });

    // Read Hall state with majority voting
    let state = read_hall_state_voted();

    // Check for state change (0 and 7 are invalid Hall states)
    if state != *LAST_STATE && state != 0 && state != 7 {
        let ticks = embassy_time::Instant::now().as_ticks();

        // Update Hall estimator
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(state, ticks);
            }
        });

        *LAST_STATE = state;
    }
}

// ========== Public API for Telemetry ==========

/// Get current Hall sensor snapshot (for telemetry, polled at low rate)
///
/// Uses `HallSnapshot` from oxifoc-core.
pub fn get_snapshot(now_ticks: u64) -> Option<HallSnapshot> {
    HALL_ESTIMATOR.lock(|est| {
        est.borrow().as_ref().and_then(|h| {
            h.sample_at(now_ticks).map(|sample| HallSnapshot {
                angle_rad: sample.angle,
                velocity_rad_s: sample.omega,
                direction: sample.direction,
                state: h.logical_state(),
                error_count: h.error_count(),
            })
        })
    })
}

// ========== Apply Stored Config at Boot ==========

/// Apply stored Hall calibration and tuning parameters to the estimator.
///
/// Must be called after `init_hall()`. Skips silently if no config is stored.
pub fn apply_stored_config(config: &oxifoc_core::storage::RuntimeConfig) {
    HALL_ESTIMATOR.lock(|est| {
        if let Some(h) = est.borrow_mut().as_mut() {
            if let Some(ref cal) = config.hall_calibration
                && cal.is_calibrated()
            {
                h.set_calibration_raw(cal.angles);
                defmt::info!("Applied stored Hall calibration");
            }
            if let Some(ref tuning) = config.hall_tuning {
                h.set_interp_min_erpm(tuning.interp_min_erpm);
                h.set_drift_correction_gain(tuning.drift_correction_gain);
                h.set_rate_limit_factor(tuning.rate_limit_factor);
                h.set_timeout_us(tuning.timeout_us);
                defmt::info!("Applied stored Hall tuning params");
            }
        }
    });
}

// ========== Hall Angle Proxy for FOC ==========

/// Angle sensor proxy for the FOC driver; pulls snapshots from `HALL_ESTIMATOR`.
pub struct HallAngleProxy;

impl HallAngleProxy {
    pub const fn new() -> Self {
        Self
    }
}

impl AngleSensor for HallAngleProxy {
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().and_then(|h| h.sample_at(now_ticks)))
    }

    fn read_angle(&self) -> f32 {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now).map(|s| s.angle).unwrap_or(0.0)
    }

    fn read_direction(&self) -> Direction {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now)
            .map(|s| s.direction)
            .unwrap_or(Direction::Stopped)
    }

    fn error_count(&self) -> u32 {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().map(|h| h.error_count()).unwrap_or(0))
    }

    fn reset_errors(&mut self) {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                h.reset_errors();
            }
        });
    }
}

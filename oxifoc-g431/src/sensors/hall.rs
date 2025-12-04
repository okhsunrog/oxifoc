//! Hall sensor management for B-G431B-ESC1
//!
//! Uses TIM6-based polling at 5µs intervals with 7-read majority voting
//! for noise immunity. This approach filters sub-µs glitches while maintaining
//! good timing resolution.

use core::cell::RefCell;

use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use static_cell::StaticCell;

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

// ========== Hall GPIO Inputs ==========

/// Static storage for GPIO inputs
static HALL_INPUTS: StaticCell<(Input<'static>, Input<'static>, Input<'static>)> =
    StaticCell::new();

/// Unsafe pointer to hall inputs for fast ISR access
static mut HALL_INPUTS_PTR: Option<&'static (Input<'static>, Input<'static>, Input<'static>)> =
    None;

// ========== Hall Sensor Initialization ==========

/// Initialize Hall sensor inputs and TIM6 for polling
///
/// Configures TIM6 to fire every 5µs. Each ISR performs 7 rapid GPIO reads
/// with majority voting to filter noise.
pub fn init_hall(
    pb6: Peri<'static, peripherals::PB6>,
    pb7: Peri<'static, peripherals::PB7>,
    pb8: Peri<'static, peripherals::PB8>,
    _exti6: Peri<'static, peripherals::EXTI6>,
    _exti7: Peri<'static, peripherals::EXTI7>,
    _exti8: Peri<'static, peripherals::EXTI8>,
) {
    // Create GPIO inputs with pull-up
    let hall_h1 = Input::new(pb6, Pull::Up);
    let hall_h2 = Input::new(pb7, Pull::Up);
    let hall_h3 = Input::new(pb8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

    // Keep Hall inputs alive for ISR access
    let inputs = HALL_INPUTS.init((hall_h1, hall_h2, hall_h3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Configure TIM6 for 5µs polling
    // STM32G431 runs at 170MHz, TIM6 is on APB1 (170MHz with APB1 prescaler = 1)
    // For 5µs period: 170MHz * 5µs = 850 ticks
    // Use prescaler = 0, ARR = 849

    // Enable TIM6 clock
    pac::RCC.apb1enr1().modify(|w| w.set_tim6en(true));

    // Reset TIM6
    pac::RCC.apb1rstr1().modify(|w| w.set_tim6rst(true));
    pac::RCC.apb1rstr1().modify(|w| w.set_tim6rst(false));

    let tim6 = pac::TIM6;

    // Configure timer
    tim6.psc().write_value(0); // No prescaler
    tim6.arr().write(|w| w.set_arr(849)); // 850 ticks = 5µs at 170MHz
    tim6.dier().write(|w| w.set_uie(true)); // Enable update interrupt
    tim6.cr1().write(|w| {
        w.set_cen(true); // Enable counter
        w.set_arpe(true); // Auto-reload preload enable
    });

    // Enable TIM6 interrupt in NVIC with high priority
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

/// Read raw Hall sensor state from GPIO (public for calibration)
#[inline]
pub fn read_hall_state_raw() -> u8 {
    read_hall_state_fast()
}

/// Read Hall sensor state quickly from GPIO (single read)
#[inline]
fn read_hall_state_fast() -> u8 {
    if let Some((h1, h2, h3)) = unsafe { HALL_INPUTS_PTR } {
        let mut state = 0u8;
        if h1.is_high() {
            state |= 0b001;
        }
        if h2.is_high() {
            state |= 0b010;
        }
        if h3.is_high() {
            state |= 0b100;
        }
        state
    } else {
        0
    }
}

/// Read Hall sensor state with 7-read majority voting (VESC-style)
///
/// Performs 7 rapid GPIO reads and returns the state that appears most often.
/// This filters sub-microsecond noise glitches.
#[inline]
fn read_hall_state_voted() -> u8 {
    if let Some((h1, h2, h3)) = unsafe { HALL_INPUTS_PTR } {
        let mut h1_count = 0u8;
        let mut h2_count = 0u8;
        let mut h3_count = 0u8;

        // 7 rapid reads (takes ~200-300ns total)
        for _ in 0..READS_PER_POLL {
            if h1.is_high() {
                h1_count += 1;
            }
            if h2.is_high() {
                h2_count += 1;
            }
            if h3.is_high() {
                h3_count += 1;
            }
        }

        // Use shared majority voting helper from core
        majority_vote(h1_count, h2_count, h3_count, MAJORITY_THRESHOLD)
    } else {
        0
    }
}

// ========== TIM6 Interrupt Handler ==========

/// TIM6 update interrupt: poll Hall sensors with majority voting
#[interrupt]
fn TIM6_DAC() {
    // Clear update interrupt flag
    pac::TIM6.sr().write(|w| w.set_uif(false));

    // Read Hall state with majority voting
    let state = read_hall_state_voted();

    // Static state for edge detection (ISR has exclusive access)
    static mut LAST_STATE: u8 = 0;

    // Check for state change
    let last = unsafe { LAST_STATE };
    if state != last && state != 0 && state != 7 {
        // Valid state change detected
        let ticks = embassy_time::Instant::now().as_ticks();

        // Update Hall estimator
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(state, ticks);
            }
        });

        unsafe {
            LAST_STATE = state;
        }
    }
}

// ========== Public API for Control Loop ==========

/// Process Hall state (called from ADC ISR for compatibility)
///
/// With timer-based polling, this is now a no-op since Hall updates
/// happen directly in the TIM6 ISR. Kept for API compatibility.
#[inline]
pub fn process_edge(_last_seq: &mut u32) -> bool {
    // No longer needed - Hall updates happen in TIM6 ISR
    // Return false to indicate no new edge was processed here
    false
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

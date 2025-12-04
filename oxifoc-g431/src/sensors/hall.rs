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
) {
    // Create GPIO inputs with pull-up
    let hall_h1 = Input::new(pb6, Pull::Up);
    let hall_h2 = Input::new(pb7, Pull::Up);
    let hall_h3 = Input::new(pb8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

    // Keep Hall inputs alive for ISR access
    let inputs = HALL_INPUTS.init((hall_h1, hall_h2, hall_h3));
    // SAFETY: Single-threaded initialization before interrupts are enabled.
    // HALL_INPUTS_PTR is only written here once and read by TIM6 ISR afterward.
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Configure TIM6 for Hall sensor polling at POLL_INTERVAL_US
    // See config.rs for TIM6_CLOCK_HZ and TIM6_ARR calculation

    // Enable TIM6 clock
    pac::RCC.apb1enr1().modify(|w| w.set_tim6en(true));

    // Reset TIM6
    pac::RCC.apb1rstr1().modify(|w| w.set_tim6rst(true));
    pac::RCC.apb1rstr1().modify(|w| w.set_tim6rst(false));

    let tim6 = pac::TIM6;

    // Configure timer with computed ARR from config
    tim6.psc().write_value(0); // No prescaler
    tim6.arr().write(|w| w.set_arr(crate::config::TIM6_ARR));
    tim6.dier().write(|w| w.set_uie(true)); // Enable update interrupt
    tim6.cr1().write(|w| {
        w.set_cen(true); // Enable counter
        w.set_arpe(true); // Auto-reload preload enable
    });

    // SAFETY: All static data (HALL_INPUTS_PTR, HALL_ESTIMATOR) is initialized above.
    // Enabling the interrupt is safe because the ISR can now access valid data.
    // Peripherals::steal() is safe in single-core context during init.
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
    // SAFETY: HALL_INPUTS_PTR is initialized once in init_hall() before interrupts
    // are enabled, and never modified afterward. Reading a static pointer is safe.
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
    // SAFETY: HALL_INPUTS_PTR is initialized once in init_hall() before interrupts
    // are enabled, and never modified afterward. Reading a static pointer is safe.
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

    // Static state for edge detection
    // SAFETY: ISR has exclusive access - this handler runs atomically on single-core MCU
    // and cannot be preempted by itself. No other code accesses this static.
    static mut LAST_STATE: u8 = 0;
    let last = unsafe { LAST_STATE };

    // Check for state change
    if state != last && state != 0 && state != 7 {
        // Valid state change detected
        let ticks = embassy_time::Instant::now().as_ticks();

        // Update Hall estimator
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(state, ticks);
            }
        });

        // SAFETY: Same as above - exclusive ISR access
        unsafe { LAST_STATE = state };
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

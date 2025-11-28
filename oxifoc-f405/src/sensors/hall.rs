//! Hall sensor management with EXTI interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_stm32::{
    Peri, exti::ExtiInput, gpio::Pull, interrupt, interrupt::typelevel::Interrupt, peripherals,
};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use static_cell::StaticCell;

use crate::config::TIMEBASE_TICKS_PER_SEC;

// ========== Static State ==========

/// Hall sensor estimator
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Static storage for EXTI inputs
static HALL_INPUTS: StaticCell<(ExtiInput<'static>, ExtiInput<'static>, ExtiInput<'static>)> =
    StaticCell::new();

/// Unsafe pointer to hall inputs for fast ISR access
static mut HALL_INPUTS_PTR: Option<&'static (
    ExtiInput<'static>,
    ExtiInput<'static>,
    ExtiInput<'static>,
)> = None;

/// Atomically stored hall angle (as f32 bits)
static HALL_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);

/// Atomically stored hall direction (0=Stopped, 1=CW, 2=CCW)
static HALL_DIRECTION: AtomicU8 = AtomicU8::new(0);

/// Atomically stored hall state (3-bit value)
static HALL_STATE: AtomicU8 = AtomicU8::new(0);

// ========== Public API ==========

/// Snapshot of hall sensor state
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub struct HallSnapshot {
    pub angle_rad: f32,
    pub direction: Direction,
    pub state: u8,
}

/// Initialize hall sensor inputs and enable EXTI interrupts
pub fn init_hall(
    pc6: Peri<'static, peripherals::PC6>,
    pc7: Peri<'static, peripherals::PC7>,
    pc8: Peri<'static, peripherals::PC8>,
    exti6: Peri<'static, peripherals::EXTI6>,
    exti7: Peri<'static, peripherals::EXTI7>,
    exti8: Peri<'static, peripherals::EXTI8>,
) {
    let hall1 = ExtiInput::new(pc6, exti6, Pull::Up);
    let hall2 = ExtiInput::new(pc7, exti7, Pull::Up);
    let hall3 = ExtiInput::new(pc8, exti8, Pull::Up);

    let inputs = HALL_INPUTS.init((hall1, hall2, hall3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize hall sensor estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Enable EXTI9_5 interrupt for PC6/7/8
    unsafe {
        interrupt::typelevel::EXTI9_5::unpend();
        interrupt::typelevel::EXTI9_5::enable();
    }

    defmt::info!("Hall sensor initialized on PC6/PC7/PC8");
}

/// Get current hall sensor snapshot
#[allow(dead_code)]
pub fn get_hall_snapshot() -> HallSnapshot {
    let angle_bits = HALL_ANGLE_BITS.load(Ordering::Relaxed);
    let dir_u8 = HALL_DIRECTION.load(Ordering::Relaxed);
    let state = HALL_STATE.load(Ordering::Relaxed);

    let direction = match dir_u8 {
        1 => Direction::Clockwise,
        2 => Direction::CounterClockwise,
        _ => Direction::Stopped,
    };

    HallSnapshot {
        angle_rad: f32::from_bits(angle_bits),
        direction,
        state,
    }
}

// ========== Interrupt Handler ==========

/// Fast hall state reading for ISR
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

/// EXTI9_5 interrupt handler for hall sensor edges
#[interrupt]
fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;

    HALL_ESTIMATOR.lock(|est| {
        if let Some(h) = est.borrow_mut().as_mut()
            && let Ok(reading) = h.update_sample(state, ticks as u64)
        {
            HALL_ANGLE_BITS.store(reading.angle_rad.to_bits(), Ordering::Relaxed);

            let dir_u8 = match reading.direction {
                Direction::Stopped => 0,
                Direction::Clockwise => 1,
                Direction::CounterClockwise => 2,
            };
            HALL_DIRECTION.store(dir_u8, Ordering::Relaxed);
            HALL_STATE.store(reading.state, Ordering::Relaxed);
        }
    });

    interrupt::typelevel::EXTI9_5::unpend();
}

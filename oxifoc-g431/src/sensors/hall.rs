//! Hall sensor management for B-G431B-ESC1
//!
//! Provides Hall sensor edge detection via EXTI interrupts, angle estimation,
//! and integration with the FOC control loop.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{Peri, interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use static_cell::StaticCell;

use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use oxifoc_core::foc::sensors::{AngleSample, AngleSensor};

use crate::config::TIMEBASE_TICKS_PER_SEC;

// ========== Hall Sensor State (Global Atomics) ==========

/// Hall sensor data (updated by EXTI ISR and consumed by ADC ISR + servers).
/// Angle stored as f32 bit-pattern in AtomicU32.
static HALL_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);
/// Hall direction: 0=Stopped, 1=Clockwise, 2=CounterClockwise
static HALL_DIRECTION: AtomicU8 = AtomicU8::new(0);
/// Hall state (0-5)
static HALL_STATE: AtomicU8 = AtomicU8::new(0);
/// Hall error count
static HALL_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
/// Sequence counter for Hall sensor samples
static HALL_SEQ: AtomicU32 = AtomicU32::new(0);

// ========== Hall Edge Mailbox ==========

/// Mailbox for communicating Hall edges from EXTI ISR to ADC ISR
struct HallEdgeMailbox {
    seq: AtomicU32,
    state: AtomicU8,
    ticks: AtomicU32,
}

impl HallEdgeMailbox {
    const fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            state: AtomicU8::new(0),
            ticks: AtomicU32::new(0),
        }
    }

    fn write(&self, state: u8, ticks: u32) {
        self.state.store(state, Ordering::Relaxed);
        self.ticks.store(ticks, Ordering::Relaxed);
        self.seq.fetch_add(1, Ordering::Release);
    }

    pub fn load(&self) -> (u32, u8, u32) {
        let seq = self.seq.load(Ordering::Acquire);
        let state = self.state.load(Ordering::Relaxed);
        let ticks = self.ticks.load(Ordering::Relaxed);
        (seq, state, ticks)
    }
}

/// Mailbox for Hall edge updates from EXTI to ADC ISR.
static HALL_EDGE_MAILBOX: HallEdgeMailbox = HallEdgeMailbox::new();

// ========== Hall Estimator ==========

/// Hall estimator shared between EXTI/Hall task and ADC ISR.
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Hall EXTI Inputs ==========

/// Keep Hall ExtiInput instances alive for EXTI interrupt handling.
static HALL_INPUTS: StaticCell<(ExtiInput<'static>, ExtiInput<'static>, ExtiInput<'static>)> =
    StaticCell::new();
static mut HALL_INPUTS_PTR: Option<&'static (
    ExtiInput<'static>,
    ExtiInput<'static>,
    ExtiInput<'static>,
)> = None;

// ========== Hall Sensor Initialization ==========

/// Initialize Hall sensor inputs and estimator
pub fn init(
    pb6: Peri<'static, peripherals::PB6>,
    pb7: Peri<'static, peripherals::PB7>,
    pb8: Peri<'static, peripherals::PB8>,
    exti6: Peri<'static, peripherals::EXTI6>,
    exti7: Peri<'static, peripherals::EXTI7>,
    exti8: Peri<'static, peripherals::EXTI8>,
) {
    // Initialize Hall sensor inputs with pull-ups and EXTI (for async edge detection)
    let hall_h1 = ExtiInput::new(pb6, exti6, Pull::Up);
    let hall_h2 = ExtiInput::new(pb7, exti7, Pull::Up);
    let hall_h3 = ExtiInput::new(pb8, exti8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

    // Keep Hall EXTI inputs alive to maintain configuration
    let inputs = HALL_INPUTS.init((hall_h1, hall_h2, hall_h3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Enable EXTI9_5 interrupt for Hall lines 6/7/8
    unsafe {
        interrupt::typelevel::EXTI9_5::unpend();
        interrupt::typelevel::EXTI9_5::enable();
    }

    defmt::info!("Hall sensor initialized with EXTI edge detection");
}

// ========== Fast Hall State Reading ==========

/// Read Hall sensor state quickly from GPIO (for ISR use)
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

// ========== EXTI Interrupt Handler ==========

/// Handle Hall sensor edges (PB6/PB7/PB8) and timestamp them.
#[interrupt]
unsafe fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;
    HALL_EDGE_MAILBOX.write(state, ticks);

    // Clear EXTI pending bits for lines 6/7/8
    interrupt::typelevel::EXTI9_5::unpend();
}

// ========== Public API for Control Loop ==========

/// Process Hall edge from mailbox (called from ADC ISR)
///
/// Returns true if a new edge was processed
pub fn process_edge(last_seq: &mut u32) -> bool {
    let (edge_seq, edge_state, edge_ticks) = HALL_EDGE_MAILBOX.load();
    if edge_seq != *last_seq {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(edge_state, edge_ticks as u64);
            }
        });
        HALL_STATE.store(edge_state, Ordering::Relaxed);
        let err =
            HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().map(|h| h.error_count()).unwrap_or(0));
        HALL_ERROR_COUNT.store(err, Ordering::Relaxed);
        *last_seq = edge_seq;
        true
    } else {
        false
    }
}

/// Update global Hall state snapshot (called from ADC ISR)
pub fn update_snapshot(now_ticks: u64) {
    if let Some(sample) =
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().and_then(|h| h.sample_at(now_ticks)))
    {
        HALL_ANGLE_BITS.store(sample.angle.to_bits(), Ordering::Relaxed);
        let dir_u8 = match sample.direction {
            Direction::Stopped => 0,
            Direction::Clockwise => 1,
            Direction::CounterClockwise => 2,
        };
        HALL_DIRECTION.store(dir_u8, Ordering::Relaxed);
    }
}

/// Get current Hall sensor snapshot for protocol servers
pub fn get_snapshot() -> HallSnapshot {
    let seq = HALL_SEQ.fetch_add(1, Ordering::Relaxed);
    let angle_bits = HALL_ANGLE_BITS.load(Ordering::Relaxed);
    let angle_rad = f32::from_bits(angle_bits);
    let dir_u8 = HALL_DIRECTION.load(Ordering::Relaxed);
    let direction = match dir_u8 {
        1 => Direction::Clockwise,
        2 => Direction::CounterClockwise,
        _ => Direction::Stopped,
    };

    HallSnapshot {
        angle_rad,
        direction,
        state: HALL_STATE.load(Ordering::Relaxed),
        error_count: HALL_ERROR_COUNT.load(Ordering::Relaxed),
        seq,
    }
}

/// Snapshot of Hall sensor data for protocol use
pub struct HallSnapshot {
    pub angle_rad: f32,
    pub direction: Direction,
    pub state: u8,
    pub error_count: u32,
    pub seq: u32,
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

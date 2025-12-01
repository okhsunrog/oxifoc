//! Hall sensor management for Simple FOCer 2 (STM32F405)
//!
//! Uses raw PAC access for EXTI configuration to avoid conflicts with Embassy's
//! EXTI feature while maintaining low-latency interrupt-driven hall sensing.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{Peri, interrupt, pac, peripherals};
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

// ========== Hall GPIO Inputs ==========

/// Static storage for GPIO inputs
static HALL_INPUTS: StaticCell<(Input<'static>, Input<'static>, Input<'static>)> =
    StaticCell::new();

/// Unsafe pointer to hall inputs for fast ISR access
static mut HALL_INPUTS_PTR: Option<&'static (Input<'static>, Input<'static>, Input<'static>)> =
    None;

// ========== Hall Sensor Initialization ==========

/// Initialize Hall sensor inputs and enable EXTI interrupts
///
/// Note: We configure EXTI via PAC directly to avoid symbol conflicts with LTO.
/// The EXTI6/7/8 parameters are kept for API compatibility but unused.
pub fn init_hall(
    pc6: Peri<'static, peripherals::PC6>,
    pc7: Peri<'static, peripherals::PC7>,
    pc8: Peri<'static, peripherals::PC8>,
    _exti6: Peri<'static, peripherals::EXTI6>,
    _exti7: Peri<'static, peripherals::EXTI7>,
    _exti8: Peri<'static, peripherals::EXTI8>,
) {
    // Create GPIO inputs with pull-up
    let hall_h1 = Input::new(pc6, Pull::Up);
    let hall_h2 = Input::new(pc7, Pull::Up);
    let hall_h3 = Input::new(pc8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PC6, H2=PC7, H3=PC8");

    // Keep Hall inputs alive for ISR access
    let inputs = HALL_INPUTS.init((hall_h1, hall_h2, hall_h3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Configure EXTI for PC6, PC7, PC8 via PAC
    // SYSCFG_EXTICR2 controls EXTI6 and EXTI7 (pins 4-7)
    // SYSCFG_EXTICR3 controls EXTI8 (pins 8-11)
    // Port C = 0b0010
    pac::SYSCFG.exticr(1).modify(|w| {
        w.set_exti(2, 0b0010); // EXTI6 -> PC6
        w.set_exti(3, 0b0010); // EXTI7 -> PC7
    });
    pac::SYSCFG.exticr(2).modify(|w| {
        w.set_exti(0, 0b0010); // EXTI8 -> PC8
    });

    // Enable rising and falling edge triggers for lines 6, 7, 8
    pac::EXTI.rtsr(0).modify(|w| {
        w.set_line(6, true);
        w.set_line(7, true);
        w.set_line(8, true);
    });
    pac::EXTI.ftsr(0).modify(|w| {
        w.set_line(6, true);
        w.set_line(7, true);
        w.set_line(8, true);
    });

    // Clear any pending interrupts
    pac::EXTI.pr(0).write(|w| {
        w.set_line(6, true);
        w.set_line(7, true);
        w.set_line(8, true);
    });

    // Enable interrupt mask for lines 6, 7, 8
    pac::EXTI.imr(0).modify(|w| {
        w.set_line(6, true);
        w.set_line(7, true);
        w.set_line(8, true);
    });

    // Enable EXTI9_5 interrupt in NVIC
    unsafe {
        interrupt::typelevel::EXTI9_5::unpend();
        interrupt::typelevel::EXTI9_5::enable();
    }

    defmt::info!("Hall sensor initialized with EXTI edge detection (raw PAC)");
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

/// Read raw Hall sensor state (public API for calibration)
///
/// Returns 3-bit Hall state (0-7): H3<<2 | H2<<1 | H1
pub fn read_hall_state_raw() -> u8 {
    read_hall_state_fast()
}

// ========== EXTI Interrupt Handler ==========

/// Handle Hall sensor edges (PC6/PC7/PC8) and timestamp them.
#[interrupt]
fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;
    HALL_EDGE_MAILBOX.write(state, ticks);

    // Clear pending bits for lines 6, 7, 8
    pac::EXTI.pr(0).write(|w| {
        w.set_line(6, true);
        w.set_line(7, true);
        w.set_line(8, true);
    });
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

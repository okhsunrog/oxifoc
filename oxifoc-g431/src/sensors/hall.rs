//! Hall sensor management for B-G431B-ESC1
//!
//! Uses raw PAC access for EXTI configuration to avoid conflicts with Embassy's
//! EXTI feature while maintaining low-latency interrupt-driven hall sensing.

use core::cell::RefCell;

use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use static_cell::StaticCell;

use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use oxifoc_core::foc::sensors::{AngleSample, AngleSensor, HallSensorTrait};

use crate::config::TIMEBASE_TICKS_PER_SEC;

// ========== Hall Edge Mailbox (EXTI → ADC ISR) ==========

/// Hall edge data communicated from EXTI ISR to ADC ISR
#[derive(Clone, Copy)]
struct HallEdge {
    seq: u32,
    state: u8,
    ticks: u32,
}

impl HallEdge {
    const fn new() -> Self {
        Self {
            seq: 0,
            state: 0,
            ticks: 0,
        }
    }
}

/// Mailbox for Hall edge updates from EXTI to ADC ISR.
/// Uses CriticalSectionMutex to ensure atomic read/write of the entire struct.
static HALL_EDGE_MAILBOX: CriticalSectionMutex<RefCell<HallEdge>> =
    CriticalSectionMutex::new(RefCell::new(HallEdge::new()));

// ========== Hall Estimator (shared state) ==========

/// Hall estimator - the single source of truth for Hall sensor state.
/// Accessed by ADC ISR (write) and telemetry tasks (read).
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
pub fn init(
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

    // Configure EXTI for PB6, PB7, PB8 via PAC
    // SYSCFG_EXTICR2 controls EXTI6 and EXTI7 (pins 4-7)
    // SYSCFG_EXTICR3 controls EXTI8 (pins 8-11)
    // Port B = 0b0001
    pac::SYSCFG.exticr(1).modify(|w| {
        w.set_exti(2, 0b0001); // EXTI6 -> PB6
        w.set_exti(3, 0b0001); // EXTI7 -> PB7
    });
    pac::SYSCFG.exticr(2).modify(|w| {
        w.set_exti(0, 0b0001); // EXTI8 -> PB8
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

// ========== EXTI Interrupt Handler ==========

/// Handle Hall sensor edges (PB6/PB7/PB8) and timestamp them.
#[interrupt]
fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;

    HALL_EDGE_MAILBOX.lock(|cell| {
        let mut edge = cell.borrow_mut();
        edge.state = state;
        edge.ticks = ticks;
        edge.seq = edge.seq.wrapping_add(1);
    });

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
    let edge = HALL_EDGE_MAILBOX.lock(|cell| *cell.borrow());

    if edge.seq != *last_seq {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(edge.state, edge.ticks as u64);
            }
        });
        *last_seq = edge.seq;
        true
    } else {
        false
    }
}

// ========== Public API for Telemetry ==========

/// Snapshot of Hall sensor data for protocol use
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)] // velocity_rad_s not yet exposed via protocol
pub struct HallSnapshot {
    pub angle_rad: f32,
    pub velocity_rad_s: f32,
    pub direction: Direction,
    pub state: u8,
    pub error_count: u32,
}

/// Get current Hall sensor snapshot (for telemetry, polled at low rate)
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

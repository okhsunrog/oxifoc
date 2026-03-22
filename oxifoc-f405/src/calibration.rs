//! Calibration and motor parameter detection for F405 platform
//!
//! Implements DetectionHardware trait using the same pattern as oxifoc-g4,
//! with platform-specific ADC atomics and board config.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;
use core::future::poll_fn;
use core::sync::atomic::{AtomicU16, Ordering};
use core::task::Poll;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::BoardConfig;
use oxifoc_core::foc::controller::FocOutput;
use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::detection::sweep::{self, DetectionHardware, HallReader};
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::foc::trig::SinCos;
use oxifoc_core::motor::ControlMode;
use oxifoc_core::state::{self, MotorControlState};

use crate::sensors::read_hall_state_raw;

// Re-export types from core for convenience
pub use oxifoc_core::foc::detection::sweep::{DetectionParams, DetectionResult};
pub use oxifoc_core::foc::detection::{FluxLinkageParams, InductanceParams, ResistanceParams};

// ============================================================================
// Timer Implementation
// ============================================================================

/// Embassy timer implementation for async delays.
pub struct EmbassyTimer;

impl oxifoc_core::timer::Timer for EmbassyTimer {
    async fn after_millis(ms: u64) {
        Timer::after(Duration::from_millis(ms)).await;
    }

    async fn after_micros(us: u64) {
        Timer::after(Duration::from_micros(us)).await;
    }
}

// ============================================================================
// Hardware Implementation
// ============================================================================

/// F405 hardware abstraction for motor detection.
pub struct F405DetectionHardware {
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
}

impl F405DetectionHardware {
    /// Create a new F405 detection hardware instance.
    pub fn new(
        state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
        board: &'static BoardConfig,
    ) -> Self {
        Self {
            state_mutex,
            ia,
            ib,
            ic,
            board,
        }
    }
}

impl DetectionHardware for F405DetectionHardware {
    fn send_command(&self, mode: ControlMode) {
        let _ = state::CMD_CHANNEL.try_send(mode);
    }

    async fn wait_telemetry(&mut self) -> FocOutput {
        // Wait for ISR to complete a FOC cycle.
        // First poll: register waker and return Pending.
        // ISR calls TELEM_WAKER.wake() → executor re-polls → Ready.
        let mut registered = false;
        poll_fn(|cx| {
            if registered {
                Poll::Ready(())
            } else {
                state::TELEM_WAKER.register(cx.waker());
                registered = true;
                Poll::Pending
            }
        })
        .await;

        // Read latest FOC output from shared state
        critical_section::with(|cs| self.state_mutex.borrow(cs).borrow().last_foc)
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        let ia_raw = self.ia.load(Ordering::Relaxed);
        let ib_raw = self.ib.load(Ordering::Relaxed);
        let ic_raw = self.ic.load(Ordering::Relaxed);
        self.board.convert_raw_currents(ia_raw, ib_raw, ic_raw)
    }
}

// ============================================================================
// Hall Sensor Reader
// ============================================================================

/// Hall sensor reader implementation for F405.
pub struct F405HallReader;

impl HallReader for F405HallReader {
    fn read_hall_state(&self) -> u8 {
        read_hall_state_raw()
    }
}

// ============================================================================
// Public API
// ============================================================================

/// Measure motor phase resistance.
pub async fn measure_resistance(
    params: &ResistanceParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = F405DetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::measure_resistance::<_, EmbassyTimer>(&mut hw, params).await
}

/// Measure motor inductance using rotating HFI.
pub async fn measure_inductance<S: SinCos>(
    params: &InductanceParams,
    pwm_freq_hz: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<(f32, f32), DetectionError> {
    let mut hw = F405DetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::measure_inductance::<_, EmbassyTimer, S>(&mut hw, params, pwm_freq_hz).await
}

/// Measure motor flux linkage via open-loop spinning.
pub async fn measure_flux_linkage(
    params: &FluxLinkageParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = F405DetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::measure_flux_linkage::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection<S: SinCos>(
    params: DetectionParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<DetectionResult, DetectionError> {
    let mut hw = F405DetectionHardware::new(state_mutex, ia, ib, ic, board);
    sweep::run_full_detection::<_, EmbassyTimer, S>(&mut hw, params).await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<HallCalibrationResult, DetectionError> {
    let mut hw = F405DetectionHardware::new(state_mutex, ia, ib, ic, board);
    let reader = F405HallReader;
    sweep::calibrate_hall::<_, EmbassyTimer, _>(&mut hw, &reader, params).await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default(
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(
        HallCalibrationParams::default(),
        state_mutex,
        ia,
        ib,
        ic,
        board,
    )
    .await
}

//! Calibration and motor parameter detection for STM32G4 platforms
//!
//! Implements DetectionHardware trait and provides access to core detection functions.

#![allow(dead_code)]

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::config::BoardConfig;
use oxifoc_core::foc::controller::FocTelemetry;
use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::detection::sweep::{self, DetectionHardware, HallReader};
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::motor::ControlMode;
use oxifoc_core::state;

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

/// G4-family hardware abstraction for motor detection.
pub struct G4DetectionHardware {
    telem_rx: embassy_sync::watch::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        FocTelemetry,
        2,
    >,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
}

impl G4DetectionHardware {
    /// Create a new G4 detection hardware instance.
    pub fn new(
        ia: &'static AtomicU16,
        ib: &'static AtomicU16,
        ic: &'static AtomicU16,
        board: &'static BoardConfig,
    ) -> Self {
        Self {
            telem_rx: state::TELEMETRY.receiver().unwrap(),
            ia,
            ib,
            ic,
            board,
        }
    }
}

impl DetectionHardware for G4DetectionHardware {
    fn send_command(&self, mode: ControlMode) {
        // Send ControlMode directly to the state channel
        let _ = state::CMD_CHANNEL.try_send(mode);
    }

    async fn wait_telemetry(&mut self) -> FocTelemetry {
        self.telem_rx.changed().await
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

/// Hall sensor reader for G4 platforms.
/// Delegates to the shared hall module.
pub struct G4HallReader;

impl HallReader for G4HallReader {
    fn read_hall_state(&self) -> u8 {
        crate::hall::read_hall_state_raw()
    }
}

// ============================================================================
// Public API
// ============================================================================

/// Measure motor phase resistance.
pub async fn measure_resistance(
    params: &ResistanceParams,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = G4DetectionHardware::new(ia, ib, ic, board);
    sweep::measure_resistance::<_, EmbassyTimer>(&mut hw, params).await
}

/// Measure motor inductance using rotating HFI.
pub async fn measure_inductance(
    params: &InductanceParams,
    pwm_freq_hz: f32,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<(f32, f32), DetectionError> {
    let mut hw = G4DetectionHardware::new(ia, ib, ic, board);
    sweep::measure_inductance::<_, EmbassyTimer>(&mut hw, params, pwm_freq_hz).await
}

/// Measure motor flux linkage via open-loop spinning.
pub async fn measure_flux_linkage(
    params: &FluxLinkageParams,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    let mut hw = G4DetectionHardware::new(ia, ib, ic, board);
    sweep::measure_flux_linkage::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection(
    params: DetectionParams,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<DetectionResult, DetectionError> {
    let mut hw = G4DetectionHardware::new(ia, ib, ic, board);
    sweep::run_full_detection::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<HallCalibrationResult, DetectionError> {
    let mut hw = G4DetectionHardware::new(ia, ib, ic, board);
    let reader = G4HallReader;
    sweep::calibrate_hall::<_, EmbassyTimer, _>(&mut hw, &reader, params).await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default(
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(HallCalibrationParams::default(), ia, ib, ic, board).await
}

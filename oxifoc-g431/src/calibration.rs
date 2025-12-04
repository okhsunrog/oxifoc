//! Calibration and motor parameter detection for G431 platform
//!
//! Implements DetectionHardware trait and provides access to core detection functions.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocTelemetry;
use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::detection::sweep::{self, DetectionHardware, HallReader};
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::motor::ControlMode;
use oxifoc_core::state;

use crate::config::BOARD;
use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};
use crate::sensors::hall::read_hall_state_raw;

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

/// G431 hardware abstraction for motor detection.
pub struct G431DetectionHardware {
    telem_rx: embassy_sync::watch::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        FocTelemetry,
        2,
    >,
}

impl G431DetectionHardware {
    /// Create a new G431 detection hardware instance.
    pub fn new() -> Self {
        Self {
            telem_rx: state::TELEMETRY.receiver().unwrap(),
        }
    }
}

impl Default for G431DetectionHardware {
    fn default() -> Self {
        Self::new()
    }
}

impl DetectionHardware for G431DetectionHardware {
    fn send_command(&self, mode: ControlMode) {
        // Send ControlMode directly to the state channel
        let _ = state::CMD_CHANNEL.try_send(mode);
    }

    async fn wait_telemetry(&mut self) -> FocTelemetry {
        self.telem_rx.changed().await
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        convert_raw_currents(ia_raw, ib_raw, ic_raw)
    }
}

// ============================================================================
// Hall Sensor Reader
// ============================================================================

/// Hall sensor reader implementation for G431.
pub struct G431HallReader;

impl HallReader for G431HallReader {
    fn read_hall_state(&self) -> u8 {
        read_hall_state_raw()
    }
}

// ============================================================================
// Public API
// ============================================================================

/// Measure motor phase resistance.
pub async fn measure_resistance(params: &ResistanceParams) -> Result<f32, DetectionError> {
    let mut hw = G431DetectionHardware::new();
    sweep::measure_resistance::<_, EmbassyTimer>(&mut hw, params).await
}

/// Measure motor inductance using rotating HFI.
pub async fn measure_inductance(
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    let mut hw = G431DetectionHardware::new();
    sweep::measure_inductance::<_, EmbassyTimer>(&mut hw, params, pwm_freq_hz).await
}

/// Measure motor flux linkage via open-loop spinning.
pub async fn measure_flux_linkage(params: &FluxLinkageParams) -> Result<f32, DetectionError> {
    let mut hw = G431DetectionHardware::new();
    sweep::measure_flux_linkage::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection(
    params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    let mut hw = G431DetectionHardware::new();
    sweep::run_full_detection::<_, EmbassyTimer>(&mut hw, params).await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError> {
    let mut hw = G431DetectionHardware::new();
    let reader = G431HallReader;
    sweep::calibrate_hall::<_, EmbassyTimer, _>(&mut hw, &reader, params).await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default() -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(HallCalibrationParams::default()).await
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert raw ADC values to currents in Amps.
fn convert_raw_currents(raw_a: u16, raw_b: u16, raw_c: u16) -> (f32, f32, f32) {
    let offset = BOARD.adc_max_counts as f32 / 2.0;
    let scale = BOARD.adc_vref_mv as f32
        / 1000.0
        / BOARD.adc_max_counts as f32
        / BOARD.shunt_ohms
        / BOARD.amp_gain;

    let ia = (raw_a as f32 - offset) * scale;
    let ib = (raw_b as f32 - offset) * scale;
    let ic = (raw_c as f32 - offset) * scale;

    (ia, ib, ic)
}

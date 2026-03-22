//! Calibration and motor parameter detection for G431 platform
//!
//! Thin wrappers over oxifoc-g4 shared calibration, providing
//! platform-specific ADC atomics and board config.

#![allow(dead_code)]

use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};

use crate::config::BOARD;
use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};
use crate::STATE;

// Re-export types from oxifoc-g4 for convenience
pub use oxifoc_g4::calibration::{
    DetectionParams, DetectionResult, FluxLinkageParams, InductanceParams, ResistanceParams,
};

/// Measure motor phase resistance.
pub async fn measure_resistance(params: &ResistanceParams) -> Result<f32, DetectionError> {
    oxifoc_g4::calibration::measure_resistance(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Measure motor inductance using rotating HFI.
pub async fn measure_inductance(
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    oxifoc_g4::calibration::measure_inductance(
        params,
        pwm_freq_hz,
        &STATE,
        &IA_SAMPLE,
        &IB_SAMPLE,
        &IC_SAMPLE,
        &BOARD,
    )
    .await
}

/// Measure motor flux linkage via open-loop spinning.
pub async fn measure_flux_linkage(params: &FluxLinkageParams) -> Result<f32, DetectionError> {
    oxifoc_g4::calibration::measure_flux_linkage(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection(
    params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    oxifoc_g4::calibration::run_full_detection(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError> {
    oxifoc_g4::calibration::calibrate_hall(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default() -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(HallCalibrationParams::default()).await
}

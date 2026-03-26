//! Calibration and motor parameter detection for G431 platform
//!
//! Thin wrappers over oxifoc-core shared calibration, providing
//! platform-specific ADC atomics and board config.

#![allow(dead_code)]

use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::foc::trig::SinCos;

use crate::STATE;
use crate::config::BOARD;
use crate::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

// Re-export types from core for convenience
pub use oxifoc_core::foc::detection::embassy_hw::{
    DetectionParams, DetectionResult, FluxLinkageParams, InductanceParams, ResistanceParams,
};

/// Measure motor phase resistance.
pub async fn measure_resistance(params: &ResistanceParams) -> Result<f32, DetectionError> {
    oxifoc_core::foc::detection::embassy_hw::measure_resistance(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Measure motor inductance using rotating HFI.
pub async fn measure_inductance<S: SinCos>(
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    oxifoc_core::foc::detection::embassy_hw::measure_inductance::<S>(
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
    oxifoc_core::foc::detection::embassy_hw::measure_flux_linkage(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Run full motor parameter detection sequence.
pub async fn run_full_detection<S: SinCos>(
    params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    oxifoc_core::foc::detection::embassy_hw::run_full_detection::<S>(
        params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD,
    )
    .await
}

/// Run Hall sensor calibration.
pub async fn calibrate_hall(
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError> {
    oxifoc_core::foc::detection::embassy_hw::calibrate_hall(
        params,
        &STATE,
        &IA_SAMPLE,
        &IB_SAMPLE,
        &IC_SAMPLE,
        &BOARD,
        crate::sensors::read_hall_state_raw,
    )
    .await
}

/// Calibrate Hall sensors with default parameters.
pub async fn calibrate_hall_default() -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(HallCalibrationParams::default()).await
}

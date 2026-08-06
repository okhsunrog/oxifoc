//! Calibration and motor parameter detection for F405 platform
//!
//! Thin wrappers over oxifoc-core shared calibration, providing
//! platform-specific ADC atomics and board config.

#![allow(dead_code)]

use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::foc::trig::SinCos;

// Re-export types from core for convenience
pub use oxifoc_core::foc::detection::embassy_hw::{
    DetectionParams, DetectionResult, FluxLinkageParams, InductanceParams, ResistanceParams,
};

// ============================================================================
// Public API (taking explicit parameters)
// ============================================================================

use core::cell::RefCell;
use core::sync::atomic::AtomicU16;

use critical_section::Mutex as CriticalSectionMutex;

use oxifoc_core::foc::config::BoardConfig;
use oxifoc_core::state::MotorControlState;

/// Measure motor phase resistance.
pub async fn measure_resistance(
    params: &ResistanceParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    ia: &'static AtomicU16,
    ib: &'static AtomicU16,
    ic: &'static AtomicU16,
    board: &'static BoardConfig,
) -> Result<f32, DetectionError> {
    oxifoc_core::foc::detection::embassy_hw::measure_resistance(
        params,
        state_mutex,
        ia,
        ib,
        ic,
        board,
    )
    .await
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
    oxifoc_core::foc::detection::embassy_hw::measure_inductance::<S>(
        params,
        pwm_freq_hz,
        state_mutex,
        ia,
        ib,
        ic,
        board,
    )
    .await
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
    oxifoc_core::foc::detection::embassy_hw::measure_flux_linkage(
        params,
        state_mutex,
        ia,
        ib,
        ic,
        board,
    )
    .await
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
    oxifoc_core::foc::detection::embassy_hw::run_full_detection::<S>(
        params,
        state_mutex,
        ia,
        ib,
        ic,
        board,
    )
    .await
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
    oxifoc_core::foc::detection::embassy_hw::calibrate_hall(
        params,
        state_mutex,
        ia,
        ib,
        ic,
        board,
        crate::sensors::read_hall_state_raw,
    )
    .await
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

// ============================================================================
// Convenience wrappers using platform statics (for detect_server)
// ============================================================================

use crate::STATE;
use crate::config::BOARD;
use crate::control::foc::{IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

/// Measure resistance using platform statics.
pub async fn measure_resistance_ez(params: &ResistanceParams) -> Result<f32, DetectionError> {
    measure_resistance(params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD).await
}

/// Measure inductance using platform statics.
pub async fn measure_inductance_ez<S: SinCos>(
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    measure_inductance::<S>(
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

/// Measure flux linkage using platform statics.
pub async fn measure_flux_linkage_ez(params: &FluxLinkageParams) -> Result<f32, DetectionError> {
    measure_flux_linkage(params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD).await
}

/// Calibrate Hall sensors using platform statics with the host-supplied
/// parameters (current, timing) — silently substituting defaults made a
/// tuned `detect hall` request calibrate at the wrong current.
pub async fn calibrate_hall_ez(
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError> {
    calibrate_hall(params, &STATE, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE, &BOARD).await
}

//! Motor parameter detection algorithms.
//!
//! This module provides VESC-style motor parameter detection for:
//! - Phase resistance (R)
//! - Inductance (Ld, Lq) via HFI injection
//! - Flux linkage (λ)
//! - DC offset calibration
//! - Auto PI controller tuning
//!
//! # Usage
//!
//! The detection functions are platform-agnostic. Platform crates (like oxifoc-f405)
//! implement the actual measurement sweeps using async functions, while this module
//! provides the core algorithms and accumulators.
//!
//! ## Typical Detection Flow
//!
//! 1. **DC Offset Calibration** - Measure current sensor offsets
//! 2. **Resistance Measurement** - Apply DC current, measure V/I
//! 3. **Inductance Measurement** - HFI injection + FFT analysis
//! 4. **Flux Linkage Measurement** - Open-loop spin, measure Vq/ω
//! 5. **PI Tuning** - Calculate Kp/Ki from R and L
//!
//! ## Motor Size
//!
//! Test currents are determined by motor size to prevent overheating:
//!
//! | Size | max_power_loss | Typical motors |
//! |------|----------------|----------------|
//! | Mini | 20W | ~75g outrunners |
//! | Small | 50W | ~200g motors |
//! | Medium | 120W | ~750g motors |
//! | Large | 400W | ~2kg motors |
//!
//! ## Example
//!
//! ```ignore
//! use oxifoc_core::foc::detection::{
//!     types::{MotorSize, MotorParams, ResistanceParams},
//!     resistance::ResistanceMeasurement,
//!     pi_tuning::calculate_foc_gains,
//! };
//!
//! // Configure for medium motor
//! let params = ResistanceParams {
//!     motor_size: MotorSize::Medium,
//!     ..Default::default()
//! };
//!
//! // Create measurement accumulator
//! let mut measurement = ResistanceMeasurement::new(100);
//!
//! // Platform code collects samples during measurement sweep
//! // measurement.record(vd, id);
//!
//! // Get result
//! let resistance = measurement.finish()?;
//!
//! // Auto-tune PI gains
//! let mut motor_params = MotorParams::default();
//! motor_params.resistance_ohm = resistance;
//! motor_params.inductance_avg_h = 0.0001; // 100µH
//! let gains = calculate_foc_gains(&motor_params, 1000.0);
//! ```

/// Common types for detection (MotorSize, MotorParams, errors, etc.)
pub mod types;

/// Enhanced DC offset calibration for current sensors
pub mod dc_offset;

/// Flux linkage (λ) measurement via open-loop spinning
pub mod flux_linkage;

/// Inductance (Ld, Lq) measurement via HFI injection
pub mod inductance;

/// Auto PI controller tuning from measured parameters
pub mod pi_tuning;

/// Phase resistance measurement
pub mod resistance;

// Re-export commonly used types for convenience
pub use types::{
    DcOffsetParams, DcOffsets, DetectionError, FluxLinkageParams, InductanceParams, MotorParams,
    MotorSize, ResistanceParams,
};

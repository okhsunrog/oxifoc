//! Calibration and motor parameter detection for G431 platform
//!
//! This module implements async sweeps for motor parameter detection:
//! - DC offset calibration (current sensor zeros)
//! - Phase resistance measurement
//! - Inductance measurement (Ld, Lq) via rotating HFI
//! - Flux linkage measurement via open-loop spinning
//!
//! # Detection Flow
//!
//! ```text
//! 1. DC Offset Calibration (PWM off, measure current sensor zeros)
//!    ↓
//! 2. Resistance Measurement (DC current on d-axis, R = Vd/Id)
//!    ↓
//! 3. Inductance Measurement (Rotating HFI in α-β, FFT analysis)
//!    ↓
//! 4. Flux Linkage Measurement (Open-loop spin, λ = (Vq - R×Iq)/ωe)
//!    ↓
//! 5. PI Auto-Tuning (Calculate Kp/Ki from R and L)
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use oxifoc_g431::calibration::{run_full_detection, DetectionResult};
//!
//! // Run full detection sequence
//! let result = run_full_detection(params).await?;
//!
//! // Or run individual measurements
//! let resistance = measure_resistance(params).await?;
//! let inductance = measure_inductance(params).await?;
//! ```

pub mod detection;
pub mod hall;

// Re-exports
#[allow(unused_imports)]
pub use hall::{calibrate_hall, G431HallReader};

pub use detection::{
    run_full_detection, DetectionParams, DetectionResult,
    measure_resistance, measure_inductance, measure_flux_linkage,
};

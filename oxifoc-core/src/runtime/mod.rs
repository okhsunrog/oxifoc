//! Async runtime with protocol servers
//!
//! This module provides async servers that communicate via ergot protocol.
//! Servers access the global state directly - no MotorRuntime trait needed.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     oxifoc-core/runtime                     │
//! ├─────────────────────────────────────────────────────────────┤
//! │  Servers access state::STATE directly                       │
//! │  - info_server()           reads device_info config         │
//! │  - hall_sensor_server()    reads state::hall_snapshot()     │
//! │  - adc_sample_server()     reads state::adc_snapshot()      │
//! │  - motor_command_server()  sends to state::CMD_CHANNEL      │
//! └─────────────────────────────────────────────────────────────┘
//! ```

pub mod servers;
pub mod streaming;

/// Shared motor-detection server with effectively-once dedup.
///
/// Requires `delivery` (for the dedup combinator) and `storage` (to persist
/// the Hall calibration result). Platforms provide a [`detect::DetectionBackend`]
/// and call [`detect::detect_server`] instead of hand-rolling the serve loop.
#[cfg(all(feature = "delivery", feature = "storage"))]
pub mod detect;

/// UART I/O wrappers implementing embedded-io-async (requires `embassy-hal` feature)
#[cfg(feature = "embassy-hal")]
pub mod io;

pub use servers::*;
pub use streaming::*;

#[cfg(all(feature = "delivery", feature = "storage"))]
pub use detect::{DetectionBackend, detect_server};

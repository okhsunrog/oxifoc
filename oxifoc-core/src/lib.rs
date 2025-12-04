//! # oxifoc-core
//!
//! Platform-agnostic Field-Oriented Control (FOC) algorithms and motor control logic.
//!
//! This crate provides the mathematical foundation for FOC motor control:
//! - Coordinate transformations (Clarke, Park)
//! - Space Vector PWM (SVPWM) modulation
//! - PI controllers with anti-windup
//! - FOC control loops
//! - Motor parameter detection (R, L, λ)
//!
//! ## Feature Flags
//!
//! - **`algorithms`** (default): FOC math algorithms
//! - **`icd`**: Interface Control Document with ergot endpoints
//! - **`runtime`**: Async runtime with servers
//! - **`virtual-motor`**: Motor simulation for testing
//! - **`defmt`**: defmt logging support for embedded
//! - **`log`**: log crate support for std
//! - **`std`**: Standard library support
//!
//! ## Usage Examples
//!
//! ### Host application with ICD
//! ```toml
//! oxifoc-core = { version = "0.1", default-features = false, features = ["icd"] }
//! ```
//!
//! ### Embedded firmware
//! ```toml
//! oxifoc-core = { version = "0.1", features = ["runtime", "defmt"] }
//! ```
//!
//! ### Testing with virtual motor
//! ```toml
//! oxifoc-core = { version = "0.1", features = ["virtual-motor"] }
//! ```
//!
//! ## FOC Algorithm Example
//!
//! ```rust
//! use oxifoc_core::foc::{transforms, svpwm, pi_controller::PIController};
//!
//! // Example: Current control loop
//! let mut id_controller = PIController::new(0.5, 10.0).with_limits(-24.0, 24.0);
//! let mut iq_controller = PIController::new(0.5, 10.0).with_limits(-24.0, 24.0);
//!
//! // Sample feedback and setpoints
//! let ia = 1.2;
//! let ib = -0.6;
//! let theta = 0.5_f32;
//! let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
//! let id_target = 0.0;
//! let iq_target = 5.0;
//! let dt = 0.0001;
//! let max_duty = 1000;
//!
//! // Measure currents and transform to dq frame
//! let (i_alpha, i_beta) = transforms::clarke(ia, ib);
//! let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);
//!
//! // Run PI controllers
//! let vd = id_controller.update(id_target, id, dt);
//! let vq = iq_controller.update(iq_target, iq, dt);
//!
//! // Transform back and generate PWM
//! let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
//! let duties = svpwm::space_vector_pwm(v_alpha, v_beta, max_duty);
//! assert_eq!(duties.len(), 3);
//! ```

#![cfg_attr(not(any(test, feature = "std")), no_std)]

/// Logging macros abstraction (defmt/log/none)
#[macro_use]
mod fmt;

/// Timer abstraction for async delays
pub mod timer;

/// High-level motor driver combining FOC with sensors and PWM
pub mod motor;

/// Shared types for protocol communication
///
/// Contains serializable types shared between firmware and host applications:
/// - Motor state and control types (MotorState, ControlMode)
/// - Telemetry types (HallSensorData, AdcSample, MotorStatus)
/// - Device info and events
pub mod types;

/// Interface Control Document with ergot endpoints (requires `icd` feature)
///
/// Defines the communication protocol between host and device:
/// - Endpoint definitions for ergot framework
/// - Re-exports all types from the `types` module
#[cfg(feature = "icd")]
pub mod icd;

/// Motor state management (requires `runtime` feature)
///
/// Centralized state management for motor control:
/// - Global STATE with motor state, telemetry
/// - CMD_CHANNEL for protocol commands
/// - TELEMETRY watch for streaming
/// - Helper functions for ISR use
#[cfg(feature = "runtime")]
pub mod state;

/// Async runtime with servers (requires `runtime` feature)
///
/// Provides async protocol servers that access state directly:
/// - Servers for Hall, ADC, motor commands, device info
/// - No MotorRuntime trait needed - servers use state module
#[cfg(feature = "runtime")]
pub mod runtime;

/// Field-Oriented Control algorithms
pub mod foc {
    /// Board configuration and ADC utilities
    pub mod config;

    /// Mathematical constants (√3, 1/√3, etc.)
    pub mod constants;

    /// High-level FOC control loop
    pub mod controller;

    /// Fault registry shared across targets
    pub mod fault;

    /// Shunt resistor current sensing
    pub mod current_sense;

    /// Hall sensor calibration algorithm
    pub mod hall_calibration;

    /// Hall sensor angle estimation
    pub mod hall_sensor;

    /// PI controller with anti-windup
    pub mod pi_controller;

    /// Phase PWM trait for platform drivers
    pub mod pwm;

    /// Sensor trait definitions (CurrentSensor, AngleSensor)
    pub mod sensors;

    /// Space Vector PWM modulation
    pub mod svpwm;

    /// Coordinate transformations (Clarke, Park, and their inverses)
    pub mod transforms;

    /// Motor parameter detection (R, L, λ)
    pub mod detection;

    /// Phase management (PhaseProvider, PhaseManager, Observer)
    pub mod phase;
}

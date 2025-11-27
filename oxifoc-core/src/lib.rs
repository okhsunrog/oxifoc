//! # oxifoc-core
//!
//! Platform-agnostic Field-Oriented Control (FOC) algorithms and motor control logic.
//!
//! This crate provides the mathematical foundation for FOC motor control:
//! - Coordinate transformations (Clarke, Park)
//! - Space Vector PWM (SVPWM) modulation
//! - PI controllers with anti-windup
//! - FOC control loops (planned)
//! - Motor parameter detection (planned)
//!
//! ## Design Philosophy
//!
//! - **No hardware dependencies**: Pure algorithms, works on any platform
//! - **no_std compatible**: Can run on embedded systems
//! - **Testable**: All logic has comprehensive unit tests that run on host
//! - **Float-based**: Uses f32 for simplicity and FPU efficiency
//!
//! ## Usage
//!
//! ```rust
//! use oxifoc_core::foc::{transforms, svpwm, pi_controller::PIController};
//!
//! // Example: Current control loop
//! let mut id_controller = PIController::new(0.5, 10.0).with_limits(-24.0, 24.0);
//! let mut iq_controller = PIController::new(0.5, 10.0).with_limits(-24.0, 24.0);
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
//! ```

#![no_std]

/// Field-Oriented Control algorithms
pub mod foc {
    /// Coordinate transformations (Clarke, Park, and their inverses)
    pub mod transforms;

    /// Space Vector PWM modulation
    pub mod svpwm;

    /// PI controller with anti-windup
    pub mod pi_controller;
}

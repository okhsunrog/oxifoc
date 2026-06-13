//! Reusable motor control driver integrating FOC with sensors and PWM

pub mod derating;
pub mod failsafe;
pub mod foc_driver;

pub use failsafe::{FailsafeConfig, FailsafeController, FailsafePolicy};
pub use foc_driver::{ControlMode, FocDriver};

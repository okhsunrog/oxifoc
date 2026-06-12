//! Reusable motor control driver integrating FOC with sensors and PWM

pub mod derating;
pub mod failsafe;
pub mod foc_driver;
pub mod six_step;

pub use failsafe::{FailsafeConfig, FailsafeController, FailsafePolicy};
pub use foc_driver::{ControlMode, FocDriver};

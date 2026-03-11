//! Reusable motor control driver integrating FOC with sensors and PWM

pub mod foc_driver;
pub mod six_step;

pub use foc_driver::{ControlMode, FocDriver};

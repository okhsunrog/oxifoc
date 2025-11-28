//! Reusable motor control driver integrating FOC with sensors and PWM

pub mod foc_driver;

pub use foc_driver::{ControlMode, FocDriver, MotorCommand};

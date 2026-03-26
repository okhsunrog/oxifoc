//! Sensor implementations for motor control

pub mod current;
pub mod hall;

pub use current::{G474CurrentSensor, G474CurrentSensorExt};
pub use hall::{HallAngleProxy, init_hall, read_hall_state_raw};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

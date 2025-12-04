//! Sensor implementations for motor control

pub mod current;
pub mod hall;

pub use current::{G431CurrentSensor, G431CurrentSensorExt};
pub use hall::{HallAngleProxy, get_snapshot as get_hall_snapshot, init as init_hall};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

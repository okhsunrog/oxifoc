//! Sensor implementations for motor control

pub mod current;
pub mod hall;

// Consumed by control/foc.rs once the motor stack is re-enabled.
#[allow(unused_imports)]
pub use current::{G474CurrentSensor, G474CurrentSensorExt};
#[allow(unused_imports)]
pub use hall::{HallAngleProxy, init_hall, read_hall_state_raw};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

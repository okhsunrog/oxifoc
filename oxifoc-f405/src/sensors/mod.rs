//! Sensor implementations for motor control

pub mod current;
pub mod hall;

pub use current::F405CurrentSensor;
#[allow(unused_imports)] // Public API not yet wired to protocol handlers
pub use hall::{HallAngleProxy, get_snapshot as get_hall_snapshot, init_hall};

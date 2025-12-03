//! Sensor implementations for motor control

pub mod current;
pub mod hall;

pub use current::G431CurrentSensor;
pub use hall::{HallAngleProxy, get_snapshot as get_hall_snapshot, init as init_hall, read_hall_state_raw};

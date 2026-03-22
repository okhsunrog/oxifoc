//! Sensor implementations for motor control

pub mod hall {
    //! Hall sensor management — re-exported from oxifoc-g4
    pub use oxifoc_g4::hall::*;
}

pub mod current {
    //! Current sensing — re-exported from oxifoc-g4 with G431 type aliases
    pub use oxifoc_g4::current::G4CurrentSensor as G431CurrentSensor;
    pub use oxifoc_g4::current::G4CurrentSensorExt as G431CurrentSensorExt;
}

pub use current::{G431CurrentSensor, G431CurrentSensorExt};
pub use hall::{HallAngleProxy, init_hall};

// Re-export HallSnapshot from core (used by hall module but also exposed for external use)
#[allow(unused_imports)]
pub use oxifoc_core::foc::sensors::HallSnapshot;

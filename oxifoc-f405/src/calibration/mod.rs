//! Calibration routines for F405 platform

pub mod hall;

#[allow(unused_imports)] // Public API not yet wired to protocol handlers
pub use hall::{F405HallReader, calibrate_hall};

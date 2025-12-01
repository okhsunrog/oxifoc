//! Sensor trait definitions for FOC control
//!
//! Provides platform-agnostic traits for angle and current sensing.
//! Hardware implementations can be found in platform-specific crates
//! (e.g., oxifoc-g431, oxifoc-f405).

use super::hall_sensor::Direction;

/// Snapshot from an angle sensor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleSample {
    pub angle: f32,
    pub omega: f32, // Electrical angular velocity (rad/s)
    pub direction: Direction,
}

/// Platform-agnostic current sensor trait
///
/// Implementers provide 3-phase current measurements in Amperes.
/// Supports both 3-phase and 2-phase sensing (set unused phase to 0.0).
///
/// Note: Calibration is handled separately via async functions in platform code,
/// using `ShuntCurrentSense::calibrate_offsets()` for the algorithm.
pub trait CurrentSensor {
    /// Read 3-phase currents in Amperes
    ///
    /// Returns (i_a, i_b, i_c) in Amps.
    /// For 2-phase sampling, set the third phase to 0.0.
    fn read_currents(&self) -> (f32, f32, f32);

    /// Read raw ADC values (for calibration and debugging)
    ///
    /// Returns (adc_a, adc_b, adc_c) raw counts.
    fn read_raw(&self) -> (u16, u16, u16);

    /// True if calibration has been performed
    fn is_calibrated(&self) -> bool;

    /// Get current calibration offsets (in ADC counts)
    fn get_offsets(&self) -> (f32, f32, f32);
}

/// Platform-agnostic angle sensor trait
///
/// Provides electrical angle for FOC Park/Clarke transforms.
pub trait AngleSensor {
    /// Snapshot at the caller's notion of time (ticks are platform-defined).
    ///
    /// Returning `None` signals "no valid sample right now" so callers can
    /// fall back to another source.
    fn sample(&self, now_ticks: u64) -> Option<AngleSample>;

    /// Read electrical angle in radians (0..2π). Default uses `sample`.
    fn read_angle(&self) -> f32 {
        self.sample(0).map(|s| s.angle).unwrap_or(0.0)
    }

    /// Read rotation direction. Default uses `sample`.
    fn read_direction(&self) -> Direction {
        self.sample(0)
            .map(|s| s.direction)
            .unwrap_or(Direction::Stopped)
    }

    /// Error counter
    fn error_count(&self) -> u32;

    /// Reset error counter
    fn reset_errors(&mut self);
}

/// Optional velocity estimation from an angle sensor
pub trait VelocitySensor {
    /// Electrical angular velocity in rad/s
    fn read_velocity(&self) -> f32;
}

//! Sensor trait definitions for FOC control
//!
//! Provides platform-agnostic traits for angle and current sensing.
//! Hardware implementations can be found in platform-specific crates
//! (e.g., oxifoc-g431, oxifoc-f405).
//!
//! ## Trait Hierarchy
//!
//! ```text
//!                  AngleSensor
//!                  (base trait)
//!                       │
//!           ┌──────────┴──────────┐
//!           │                     │
//!           ▼                     ▼
//!    HallSensorTrait       EncoderSensorTrait
//!    (Hall-specific)       (Encoder-specific)
//! ```

use super::hall_calibration::HallCalibrationResult;
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

// ============================================================================
// Extended sensor traits
// ============================================================================

/// Hall sensor interpolation diagnostics
#[derive(Clone, Copy, Debug, Default)]
pub struct HallInterpolationInfo {
    /// Angle from Hall state calibration table
    pub base_angle: f32,
    /// Offset added by velocity extrapolation
    pub interpolation_offset: f32,
    /// Velocity used for interpolation (rad/s)
    pub estimated_velocity: f32,
    /// Time since last Hall edge (microseconds)
    pub time_since_edge_us: u32,
}

/// Hall-sensor-specific operations
///
/// Extends `AngleSensor` with Hall-specific functionality:
/// - Raw/logical state access
/// - Edge timing for velocity estimation
/// - Calibration table management
/// - Timing advance configuration
pub trait HallSensorTrait: AngleSensor {
    /// Raw 3-bit Hall state (0-7, where 0 and 7 are invalid)
    fn raw_state(&self) -> u8;

    /// Logical Hall state (0-5, normalized sequence position)
    fn logical_state(&self) -> u8;

    /// Timestamp of last Hall edge (in sensor's tick timebase)
    fn last_edge_ticks(&self) -> Option<u64>;

    /// Electrical velocity from edge timing (rad/s)
    fn electrical_velocity(&self) -> f32;

    /// Set calibration table (angles for logical states 0-5)
    /// For backwards compatibility - prefer `set_calibration_raw`
    fn set_calibration(&mut self, table: [f32; 6]);

    /// Set calibration table using raw Hall states (8-entry table)
    /// This is the preferred method as it works with any Hall sensor wiring
    fn set_calibration_raw(&mut self, raw_table: [f32; 8]);

    /// Apply calibration result from HallCalibrator
    fn apply_calibration(&mut self, result: &HallCalibrationResult) -> bool;

    /// Set timing advance (radians)
    fn set_advance(&mut self, advance_rad: f32);

    /// Get current timing advance (radians)
    fn advance(&self) -> f32;

    /// Get interpolation diagnostics
    fn interpolation_info(&self, now_ticks: u64) -> HallInterpolationInfo;
}

/// Encoder type classification
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EncoderType {
    /// Standard quadrature encoder
    #[default]
    Incremental,
    /// Quadrature with index pulse
    IncrementalWithIndex,
    /// Absolute position encoder (SPI, I2C, etc.)
    Absolute,
}

/// Encoder-specific operations
///
/// Extends `AngleSensor` with encoder-specific functionality:
/// - Raw count access
/// - Zero/offset management
/// - Index pulse handling
pub trait EncoderSensorTrait: AngleSensor {
    /// Raw encoder count
    fn counts(&self) -> i32;

    /// Set current position as electrical zero
    fn set_zero(&mut self);

    /// Set electrical angle offset (radians)
    fn set_offset(&mut self, offset_rad: f32);

    /// Get electrical angle offset (radians)
    fn offset(&self) -> f32;

    /// Counts per electrical revolution
    fn counts_per_electrical_rev(&self) -> u32;

    /// Set counts per electrical revolution
    fn set_counts_per_electrical_rev(&mut self, cpr: u32);

    /// Check if index pulse has been seen
    fn index_seen(&self) -> bool {
        false
    }

    /// Reset index flag
    fn reset_index(&mut self) {}

    /// Get encoder type
    fn encoder_type(&self) -> EncoderType {
        EncoderType::Incremental
    }
}

// ============================================================================
// Null sensor for unused slots
// ============================================================================

/// Null sensor placeholder for unused sensor slots
///
/// Used as a type parameter when a sensor is not present.
/// Always returns `None` for samples and `false` for availability.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoSensor;

impl AngleSensor for NoSensor {
    fn sample(&self, _now_ticks: u64) -> Option<AngleSample> {
        None
    }

    fn error_count(&self) -> u32 {
        0
    }

    fn reset_errors(&mut self) {}
}

impl NoSensor {
    /// Check if this is a null sensor (always true for NoSensor)
    pub fn is_null(&self) -> bool {
        true
    }
}

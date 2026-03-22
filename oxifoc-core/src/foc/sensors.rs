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

use super::current_reconstruction::ReconstructionState;
use super::current_sense::ShuntCurrentSense;
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

    /// Provide previous-cycle duty values for sector-based current reconstruction.
    ///
    /// Boards with unipolar shunt sensing (no Vref/2 bias) override this to
    /// feed duty info into the reconstruction logic. Default is a no-op for
    /// boards that don't need reconstruction (e.g., with DRV8301 Vref/2 bias).
    fn update_duties(&mut self, _duties: [u16; 3]) {}
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

// ============================================================================
// Hall sensor snapshot for telemetry
// ============================================================================

/// Snapshot of Hall sensor data for protocol/telemetry use
///
/// This is a platform-agnostic struct that can be used by any firmware
/// to report Hall sensor state to the host.
#[derive(Clone, Copy, Debug, Default)]
pub struct HallSnapshot {
    /// Electrical angle in radians (0..2π)
    pub angle_rad: f32,
    /// Electrical angular velocity in rad/s
    pub velocity_rad_s: f32,
    /// Current rotation direction
    pub direction: Direction,
    /// Raw 3-bit Hall state (0-7)
    pub state: u8,
    /// Cumulative error count
    pub error_count: u32,
}

// ============================================================================
// Generic current sensor with raw ADC reader
// ============================================================================

/// Trait for reading raw ADC values from platform-specific sources
///
/// Implement this trait in platform code to provide raw ADC readings
/// to the generic `GenericCurrentSensor`.
pub trait RawCurrentReader {
    /// Read raw ADC values for all three phases
    ///
    /// Returns (adc_a, adc_b, adc_c) in raw ADC counts.
    fn read_raw(&self) -> (u16, u16, u16);
}

/// Generic current sensor implementation
///
/// Combines a platform-specific raw ADC reader with the shared
/// `ShuntCurrentSense` converter. This eliminates code duplication
/// between platform crates.
///
/// # Usage
///
/// ```ignore
/// // In platform crate:
/// struct MyAdcReader;
///
/// impl RawCurrentReader for MyAdcReader {
///     fn read_raw(&self) -> (u16, u16, u16) {
///         (IA_SAMPLE.load(Relaxed), IB_SAMPLE.load(Relaxed), IC_SAMPLE.load(Relaxed))
///     }
/// }
///
/// type MyCurrentSensor = GenericCurrentSensor<MyAdcReader>;
/// ```
pub struct GenericCurrentSensor<R: RawCurrentReader> {
    /// Core conversion logic
    converter: ShuntCurrentSense,
    /// Platform-specific raw ADC reader
    reader: R,
    /// Optional sector-based reconstruction for unipolar shunt sensing
    reconstruction: Option<ReconstructionState>,
}

impl<R: RawCurrentReader> GenericCurrentSensor<R> {
    /// Create a new generic current sensor
    ///
    /// # Arguments
    /// * `shunt_ohms` - Shunt resistance in Ohms
    /// * `amp_gain` - OPAMP/amplifier gain
    /// * `adc_vref_mv` - ADC reference voltage in millivolts
    /// * `adc_max_counts` - Maximum ADC count value
    /// * `reader` - Platform-specific raw ADC reader
    pub fn new(
        shunt_ohms: f32,
        amp_gain: f32,
        adc_vref_mv: u32,
        adc_max_counts: u16,
        reader: R,
    ) -> Self {
        Self {
            converter: ShuntCurrentSense::new(shunt_ohms, amp_gain, adc_vref_mv, adc_max_counts),
            reader,
            reconstruction: None,
        }
    }

    /// Create from board config
    pub fn from_config(config: &super::config::BoardConfig, reader: R) -> Self {
        Self::new(
            config.shunt_ohms,
            config.amp_gain,
            config.adc_vref_mv,
            config.adc_max_counts,
            reader,
        )
    }

    /// Access the underlying converter for calibration
    pub fn converter(&self) -> &ShuntCurrentSense {
        &self.converter
    }

    /// Access the underlying converter mutably for calibration
    pub fn converter_mut(&mut self) -> &mut ShuntCurrentSense {
        &mut self.converter
    }

    /// Manually set calibration offsets
    pub fn set_offsets(&mut self, offset_a: f32, offset_b: f32, offset_c: f32) {
        self.converter.set_offsets(offset_a, offset_b, offset_c);
    }

    /// Calibrate offsets from collected samples
    pub fn calibrate_offsets(&mut self, samples: &[(u16, u16, u16)]) {
        self.converter.calibrate_offsets(samples);
    }

    /// Enable sector-based current reconstruction for unipolar shunt sensing.
    ///
    /// Call this on boards where OPAMPs have no Vref/2 bias and negative
    /// currents clip to 0V (e.g., B-G431B-ESC1 with low-side shunts).
    /// Boards with proper bias (e.g., DRV8301) should NOT call this.
    pub fn enable_reconstruction(&mut self) {
        self.reconstruction = Some(ReconstructionState::new());
    }
}

impl<R: RawCurrentReader> CurrentSensor for GenericCurrentSensor<R> {
    fn read_currents(&self) -> (f32, f32, f32) {
        let (adc_a, adc_b, adc_c) = self.reader.read_raw();
        let (ia, ib, ic) = self.converter.convert_raw(adc_a, adc_b, adc_c);
        match &self.reconstruction {
            Some(recon) => recon.reconstruct(ia, ib, ic),
            None => (ia, ib, ic),
        }
    }

    fn read_raw(&self) -> (u16, u16, u16) {
        self.reader.read_raw()
    }

    fn is_calibrated(&self) -> bool {
        self.converter.is_calibrated()
    }

    fn get_offsets(&self) -> (f32, f32, f32) {
        self.converter.get_offsets()
    }

    fn update_duties(&mut self, duties: [u16; 3]) {
        if let Some(recon) = &mut self.reconstruction {
            recon.set_duties(duties);
        }
    }
}

// ============================================================================
// ADC snapshot for telemetry
// ============================================================================

// ============================================================================
// Hall sensor polling configuration
// ============================================================================

/// Configuration for Hall sensor timer-based polling
///
/// These constants are shared across all platforms that use TIM6-based
/// polling with majority voting for Hall sensor noise immunity.
pub mod hall_polling {
    /// Polling interval in microseconds
    ///
    /// Each TIM6 interrupt reads Hall sensors with majority voting.
    /// 5µs provides good balance between noise filtering and responsiveness.
    pub const POLL_INTERVAL_US: u32 = 5;

    /// Number of GPIO reads per poll for majority voting
    ///
    /// VESC uses 7 reads for robust noise filtering.
    /// Takes ~200-300ns total on Cortex-M4F.
    pub const READS_PER_POLL: u8 = 7;

    /// Majority threshold (need more than half)
    ///
    /// With 7 reads, need 4 or more to count as HIGH.
    pub const MAJORITY_THRESHOLD: u8 = READS_PER_POLL / 2 + 1; // 4 of 7

    /// Perform majority voting on individual bit counts
    ///
    /// Given the counts of HIGH readings for each Hall channel (h1, h2, h3)
    /// out of `reads_per_poll` total reads, returns the voted 3-bit Hall state.
    ///
    /// # Arguments
    /// * `h1_count` - Number of HIGH readings for Hall 1
    /// * `h2_count` - Number of HIGH readings for Hall 2
    /// * `h3_count` - Number of HIGH readings for Hall 3
    /// * `threshold` - Minimum count to be considered HIGH (typically 4 of 7)
    ///
    /// # Returns
    /// 3-bit Hall state: H3<<2 | H2<<1 | H1
    ///
    /// # Example
    /// ```
    /// use oxifoc_core::foc::sensors::hall_polling;
    ///
    /// // 7 reads: H1=6 high, H2=2 high, H3=5 high
    /// let state = hall_polling::majority_vote(6, 2, 5, hall_polling::MAJORITY_THRESHOLD);
    /// assert_eq!(state, 0b101); // H1 and H3 are HIGH
    /// ```
    #[inline]
    pub const fn majority_vote(h1_count: u8, h2_count: u8, h3_count: u8, threshold: u8) -> u8 {
        let mut state = 0u8;
        if h1_count >= threshold {
            state |= 0b001;
        }
        if h2_count >= threshold {
            state |= 0b010;
        }
        if h3_count >= threshold {
            state |= 0b100;
        }
        state
    }

    /// Check if a Hall state is valid (not all low or all high)
    ///
    /// Valid states are 1-6 (binary: 001, 010, 011, 100, 101, 110).
    /// Invalid states are 0 (all low) and 7 (all high).
    #[inline]
    pub const fn is_valid_hall_state(state: u8) -> bool {
        state != 0 && state != 7
    }
}

// ============================================================================
// Temperature sensors
// ============================================================================

/// Temperature sensor identification
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TempSensorId {
    /// FET/MOSFET temperature
    Fet,
    /// PCB/board temperature
    Board,
    /// Motor winding temperature
    Motor,
    /// Other/custom temperature sensor
    Other(u8),
}

/// ADC sample snapshot for protocol/telemetry use
///
/// Platform-agnostic struct that can report ADC data to the host.
/// Supports variable number of temperature sensors.
#[derive(Clone, Debug)]
pub struct AdcSnapshot {
    /// Phase A current (raw ADC counts)
    pub ia: u16,
    /// Phase B current (raw ADC counts)
    pub ib: u16,
    /// Phase C current (raw ADC counts)
    pub ic: u16,
    /// DC bus voltage in millivolts
    pub vbus_mv: u32,
    /// Temperature readings: (sensor_id, value in 0.1°C)
    /// Using a fixed array to avoid heap allocation
    pub temps: [(TempSensorId, u16); 4],
    /// Number of valid temperature entries
    pub temp_count: u8,
    /// Sequence counter
    pub seq: u32,
}

impl Default for AdcSnapshot {
    fn default() -> Self {
        Self {
            ia: 0,
            ib: 0,
            ic: 0,
            vbus_mv: 0,
            temps: [(TempSensorId::Fet, 0); 4],
            temp_count: 0,
            seq: 0,
        }
    }
}

impl AdcSnapshot {
    /// Create an empty ADC snapshot (const for static initialization)
    pub const fn empty() -> Self {
        Self {
            ia: 0,
            ib: 0,
            ic: 0,
            vbus_mv: 0,
            temps: [(TempSensorId::Fet, 0); 4],
            temp_count: 0,
            seq: 0,
        }
    }

    /// Create a new ADC snapshot with currents and voltage only
    pub fn new(ia: u16, ib: u16, ic: u16, vbus_mv: u32, seq: u32) -> Self {
        Self {
            ia,
            ib,
            ic,
            vbus_mv,
            temps: [(TempSensorId::Fet, 0); 4],
            temp_count: 0,
            seq,
        }
    }

    /// Add a temperature reading
    pub fn with_temp(mut self, sensor: TempSensorId, temp_c_x10: u16) -> Self {
        if (self.temp_count as usize) < self.temps.len() {
            self.temps[self.temp_count as usize] = (sensor, temp_c_x10);
            self.temp_count += 1;
        }
        self
    }

    /// Get temperature for a specific sensor
    pub fn get_temp(&self, sensor: TempSensorId) -> Option<u16> {
        self.temps[..self.temp_count as usize]
            .iter()
            .find(|(id, _)| *id == sensor)
            .map(|(_, temp)| *temp)
    }

    /// Get FET temperature (convenience method)
    pub fn fet_temp_c_x10(&self) -> Option<u16> {
        self.get_temp(TempSensorId::Fet)
    }

    /// Get board temperature (convenience method)
    pub fn board_temp_c_x10(&self) -> Option<u16> {
        self.get_temp(TempSensorId::Board)
    }

    /// Get motor temperature (convenience method)
    pub fn motor_temp_c_x10(&self) -> Option<u16> {
        self.get_temp(TempSensorId::Motor)
    }
}

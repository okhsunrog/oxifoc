//! Hall sensor angle estimation for BLDC/FOC motor control
//!
//! Hall sensors provide 6 discrete positions per electrical revolution.
//! This module converts Hall sensor states to electrical angles and can
//! interpolate between edges using the measured electrical speed (similar
//! to VESC `mcpwm_foc` low-speed handling).
//!
//! ## VESC-compatible features:
//! - **Soft drift correction**: Gradually pulls interpolated angle back toward sector (1% per sample)
//! - **Rate limiting**: Limits angle rate of change to prevent current spikes
//! - **Explicit direction tracking**: Handles direction reversals cleanly
//! - **Low-speed threshold**: Disables interpolation below configurable velocity

use core::f32::consts::TAU;

use super::sensors::{AngleSample, AngleSensor, HallInterpolationInfo, HallSensorTrait};

/// Hall state lookup table: maps raw sensor reading (0-7) to logical state (0-5)
///
/// Valid Hall sequence (CW rotation): 1 → 3 → 2 → 6 → 4 → 5 (repeat)
/// This corresponds to: 001 → 011 → 010 → 110 → 100 → 101 in binary
///
/// The table maps raw 3-bit Hall reading to normalized state:
/// - Invalid states (0, 7) map to 0 with error flag
/// - Valid states (1,2,3,4,5,6) map to their position in the sequence
pub const HALL_STATE_TABLE: [u8; 8] = [
    0, // 000 (invalid - all sensors low)
    0, // 001 (H1 only)
    2, // 010 (H2 only)
    1, // 011 (H1+H2)
    4, // 100 (H3 only)
    5, // 101 (H3+H1)
    3, // 110 (H3+H2)
    0, // 111 (invalid - all sensors high)
];

/// Direction of rotation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Direction {
    /// Clockwise rotation
    Clockwise,
    /// Counter-clockwise rotation
    CounterClockwise,
    /// Motor stopped or direction unknown
    #[default]
    Stopped,
}

/// Default minimum eRPM for interpolation
/// Below this, use nearest Hall sector angle only.
/// VESC default is 500 eRPM
pub const DEFAULT_INTERP_MIN_ERPM: f32 = 500.0;

/// Default max angle drift before soft correction kicks in (radians)
/// VESC uses 30° = π/6
pub const DEFAULT_MAX_DRIFT_RAD: f32 = core::f32::consts::PI / 6.0;

/// Default drift correction gain (VESC uses 0.01 = 1% per cycle)
pub const DEFAULT_DRIFT_CORRECTION_GAIN: f32 = 0.01;

/// Default rate limit factor for angle changes
/// Limits angle step to max_rate = (velocity * dt * RATE_LIMIT_FACTOR)
/// VESC uses 1.5 (allows 50% overshoot for transients)
pub const DEFAULT_RATE_LIMIT_FACTOR: f32 = 1.5;

/// Default Hall sensor timeout (microseconds)
/// If no valid Hall edge is received for this duration, sensor is considered stale.
/// 100ms is reasonable for low-speed detection.
pub const DEFAULT_HALL_TIMEOUT_US: u32 = 100_000;

/// Platform-agnostic Hall sensor angle estimator
///
/// Tracks Hall sensor state transitions to estimate electrical angle
/// and direction of rotation.
///
/// Returns electrical angle (0 to 2π per electrical revolution),
/// which completes every 6 Hall states regardless of motor pole count.
///
/// ## Features (VESC-compatible):
/// - Soft drift correction (gradual pull-back toward sector angle)
/// - Rate limiting (prevents current spikes on Hall transitions)
/// - Explicit direction reversal tracking
/// - Low-speed interpolation threshold
pub struct HallSensor {
    /// Electrical angle increment per Hall state change (TAU / 6)
    angle_per_state: f32,
    /// Calibration table: electrical angle (rad) for each logical Hall state (0-5)
    calib: HallCalibration,
    /// Current electrical angle at last Hall edge (radians, 0 to 2π)
    angle: f32,
    /// Raw 3-bit Hall state (0-7)
    raw_state: u8,
    /// Logical Hall state (0-5)
    logical_state: u8,
    /// Current direction of rotation
    direction: Direction,
    /// Previous direction (for reversal detection)
    prev_direction: Direction,
    /// Direction reversal detected on last update
    direction_reversed: bool,
    /// Last edge timestamp (ticks of a caller-provided clock)
    last_edge_ticks: Option<u64>,
    /// Electrical velocity estimate (rad/s) from last edge
    elec_velocity: f32,
    /// Error counter for invalid states or transitions
    error_count: u32,
    /// Timebase (ticks per second)
    ticks_per_sec: u64,
    /// Minimum eRPM for interpolation. Below this, use sector angle only.
    interp_min_erpm: f32,
    /// Maximum drift from sector before correction (radians)
    max_drift_rad: f32,
    /// Drift correction gain (0.01 = 1% per sample, VESC default)
    drift_correction_gain: f32,
    /// Rate limit factor (1.5 = allow 50% overshoot, VESC default)
    rate_limit_factor: f32,
    /// Rate-limited angle (tracked across samples for smooth limiting)
    rate_limited_angle: f32,
    /// Last sample timestamp for rate limiting dt calculation
    last_sample_ticks: Option<u64>,
    /// Hall sensor timeout in ticks (converted from microseconds)
    timeout_ticks: u64,
}

/// Snapshot of the Hall estimator state after a valid edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HallReading {
    pub angle_rad: f32,
    pub direction: Direction,
    pub state: u8,
    pub elec_velocity: f32,
    pub t_ticks: u64,
}

/// Error when processing a Hall edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HallError {
    InvalidState(u8),
}

impl Default for HallSensor {
    fn default() -> Self {
        Self::new(1)
    }
}

impl HallSensor {
    /// Create a new Hall sensor estimator
    ///
    /// Returns **electrical angle** (0 to 2π per electrical revolution),
    /// which completes every 6 Hall states.
    ///
    /// To convert to mechanical angle, divide by pole pairs:
    /// `mechanical_angle = electrical_angle / pole_pairs`
    pub fn new(ticks_per_sec: u64) -> Self {
        // Electrical angle completes 2π every 6 Hall states (one electrical revolution)
        let angle_per_state = TAU / 6.0;

        HallSensor {
            angle_per_state,
            calib: HallCalibration::default(),
            angle: 0.0,
            raw_state: 0,
            logical_state: 0,
            direction: Direction::Stopped,
            prev_direction: Direction::Stopped,
            direction_reversed: false,
            last_edge_ticks: None,
            elec_velocity: 0.0,
            error_count: 0,
            ticks_per_sec: ticks_per_sec.max(1),
            interp_min_erpm: DEFAULT_INTERP_MIN_ERPM,
            max_drift_rad: DEFAULT_MAX_DRIFT_RAD,
            drift_correction_gain: DEFAULT_DRIFT_CORRECTION_GAIN,
            rate_limit_factor: DEFAULT_RATE_LIMIT_FACTOR,
            rate_limited_angle: 0.0,
            last_sample_ticks: None,
            timeout_ticks: Self::us_to_ticks(DEFAULT_HALL_TIMEOUT_US, ticks_per_sec),
        }
    }

    /// Convert microseconds to ticks
    #[inline]
    fn us_to_ticks(us: u32, ticks_per_sec: u64) -> u64 {
        (us as u64 * ticks_per_sec) / 1_000_000
    }

    /// Set minimum eRPM for interpolation.
    ///
    /// Below this eRPM, interpolation is disabled and the nearest
    /// Hall sector angle is used. This prevents oscillation during
    /// slow direction reversals. Default: 500 eRPM (VESC default).
    pub fn set_interp_min_erpm(&mut self, erpm: f32) {
        self.interp_min_erpm = erpm.abs();
    }

    /// Get minimum interpolation eRPM
    pub fn interp_min_erpm(&self) -> f32 {
        self.interp_min_erpm
    }

    /// Convert electrical velocity (rad/s) to eRPM
    #[inline]
    fn vel_to_erpm(vel_rad_s: f32) -> f32 {
        vel_rad_s.abs() * 60.0 / TAU
    }

    /// Set maximum drift before soft correction kicks in (radians).
    ///
    /// If interpolated angle drifts more than this from the current
    /// Hall sector, soft correction will gradually pull it back.
    /// Default: 30° (π/6).
    pub fn set_max_drift(&mut self, drift_rad: f32) {
        self.max_drift_rad = drift_rad.abs();
    }

    /// Set drift correction gain (0.0 to 1.0).
    ///
    /// When angle drifts beyond max_drift_rad, it's pulled back by
    /// `drift * gain` per sample. VESC uses 0.01 (1% per cycle).
    /// Higher = faster correction but more aggressive.
    pub fn set_drift_correction_gain(&mut self, gain: f32) {
        self.drift_correction_gain = gain.clamp(0.0, 1.0);
    }

    /// Get drift correction gain
    pub fn drift_correction_gain(&self) -> f32 {
        self.drift_correction_gain
    }

    /// Set rate limit factor for angle changes.
    ///
    /// Limits angle step per sample to `velocity * dt * factor`.
    /// VESC uses 1.5 (allows 50% overshoot for transients).
    /// Higher = less limiting, lower = smoother but slower response.
    pub fn set_rate_limit_factor(&mut self, factor: f32) {
        self.rate_limit_factor = factor.max(0.1);
    }

    /// Get rate limit factor
    pub fn rate_limit_factor(&self) -> f32 {
        self.rate_limit_factor
    }

    /// Set Hall sensor timeout in microseconds.
    ///
    /// If no valid Hall edge is received for this duration, the sensor
    /// is considered stale. Default: 100ms (100_000µs).
    pub fn set_timeout_us(&mut self, timeout_us: u32) {
        self.timeout_ticks = Self::us_to_ticks(timeout_us, self.ticks_per_sec);
    }

    /// Get Hall sensor timeout in microseconds.
    pub fn timeout_us(&self) -> u32 {
        ((self.timeout_ticks * 1_000_000) / self.ticks_per_sec) as u32
    }

    /// Check if Hall sensor data is stale (no edges for timeout period).
    ///
    /// Returns `true` if:
    /// - No edge has ever been received, or
    /// - Time since last edge exceeds the configured timeout
    pub fn is_stale(&self, now_ticks: u64) -> bool {
        match self.last_edge_ticks {
            None => true,
            Some(last) => {
                let elapsed = now_ticks.wrapping_sub(last);
                elapsed > self.timeout_ticks
            }
        }
    }

    /// Get time since last Hall edge in ticks.
    ///
    /// Returns `None` if no edge has been received yet.
    pub fn time_since_edge(&self, now_ticks: u64) -> Option<u64> {
        self.last_edge_ticks
            .map(|last| now_ticks.wrapping_sub(last))
    }

    /// Get time since last Hall edge in microseconds.
    ///
    /// Returns `None` if no edge has been received yet.
    pub fn time_since_edge_us(&self, now_ticks: u64) -> Option<u32> {
        self.time_since_edge(now_ticks)
            .map(|ticks| ((ticks * 1_000_000) / self.ticks_per_sec) as u32)
    }

    /// Check if direction reversal was detected on last update
    pub fn direction_reversed(&self) -> bool {
        self.direction_reversed
    }

    /// Get previous direction (before last update)
    pub fn prev_direction(&self) -> Direction {
        self.prev_direction
    }

    /// Update angle based on new Hall sensor reading and timestamp
    ///
    /// # Arguments
    /// * `raw_state` - 3-bit Hall sensor value (H3<<2 | H2<<1 | H1)
    /// * `t_ticks` - Monotonic time in ticks (of `ticks_per_sec`)
    ///
    /// # Returns
    /// * `Some(angle)` - Electrical angle at the edge in radians (0 to 2π)
    /// * `None` - Invalid Hall state (0 or 7)
    pub fn update(&mut self, raw_state: u8, t_ticks: u64) -> Option<f32> {
        // Store raw state
        self.raw_state = raw_state;

        // Check for invalid states (all low or all high)
        if raw_state == 0 || raw_state > 6 {
            self.error_count += 1;
            // Reset for clean recovery - next valid edge starts fresh
            // This prevents bogus velocity calculation from stale timestamps
            // after Hall sensor glitch or cable disconnect/reconnect (VESC-style)
            self.last_edge_ticks = None;
            return None;
        }

        let prev_state = self.logical_state;
        let current_state = HALL_STATE_TABLE[raw_state as usize];

        // Detect direction based on state transitions
        let is_wrap_cw = prev_state == 5 && current_state == 0;
        let is_wrap_ccw = prev_state == 0 && current_state == 5;
        if current_state.abs_diff(prev_state) > 1 && !is_wrap_cw && !is_wrap_ccw {
            // Non-adjacent transition (not wraparound) => error
            self.error_count += 1;
        }

        // Save previous direction before updating
        self.prev_direction = self.direction;

        // Determine new direction
        let new_direction = if current_state == 0 && prev_state == 5 {
            Direction::Clockwise
        } else if current_state == 5 && prev_state == 0 {
            Direction::CounterClockwise
        } else if current_state > prev_state {
            Direction::Clockwise
        } else if current_state < prev_state {
            Direction::CounterClockwise
        } else {
            self.direction // No change
        };

        // Explicit direction reversal detection (VESC-style)
        self.direction_reversed = matches!(
            (self.prev_direction, new_direction),
            (Direction::Clockwise, Direction::CounterClockwise)
                | (Direction::CounterClockwise, Direction::Clockwise)
        );

        self.direction = new_direction;
        self.logical_state = current_state;

        // Base angle from calibration table (includes user-set advance)
        // Use raw_state for direct lookup in the 8-entry table
        let angle_raw = self.calib.angle_for_state(raw_state);

        // Velocity estimate from last edge
        if let Some(last_t) = self.last_edge_ticks {
            let dt_ticks = t_ticks.wrapping_sub(last_t).max(1);
            let dt = dt_ticks as f32 / self.ticks_per_sec as f32;
            let angle_step = self.angle_step_signed(self.direction);

            // On direction reversal, negate velocity sign but keep magnitude relationship
            // This matches VESC's handling in foc_correct_hall()
            if self.direction_reversed {
                self.elec_velocity = -self.elec_velocity.signum() * (angle_step / dt).abs();
            } else {
                self.elec_velocity = angle_step / dt;
            }
        }

        self.last_edge_ticks = Some(t_ticks);
        self.angle = angle_raw;

        // Reset rate-limited angle to sector on Hall edge (fresh start)
        self.rate_limited_angle = angle_raw;

        Some(self.angle)
    }

    /// Update and return a full reading (angle, dir, velocity) or an error.
    pub fn update_sample(&mut self, raw_state: u8, t_ticks: u64) -> Result<HallReading, HallError> {
        match self.update(raw_state, t_ticks) {
            Some(angle) => Ok(HallReading {
                angle_rad: angle,
                direction: self.direction,
                state: self.logical_state,
                elec_velocity: self.elec_velocity,
                t_ticks,
            }),
            None => Err(HallError::InvalidState(raw_state)),
        }
    }

    /// Get current electrical angle in radians (0 to 2π)
    pub fn angle(&self) -> f32 {
        self.angle
    }

    /// Get current rotation direction
    pub fn direction(&self) -> Direction {
        self.direction
    }

    /// Get error count (invalid states or transitions)
    pub fn error_count(&self) -> u32 {
        self.error_count
    }

    /// Reset error counter
    pub fn reset_errors(&mut self) {
        self.error_count = 0;
    }

    /// Get the logical Hall state (0-5)
    pub fn state(&self) -> u8 {
        self.logical_state
    }

    /// Get the raw Hall state (0-7)
    pub fn raw_state(&self) -> u8 {
        self.raw_state
    }

    /// Get current timing advance (radians)
    pub fn advance(&self) -> f32 {
        self.calib.advance_rad
    }

    /// Get ticks per second
    pub fn ticks_per_sec(&self) -> u64 {
        self.ticks_per_sec
    }

    /// Calibrate Hall table using raw-state angles (8-entry table).
    ///
    /// This is the preferred method as it works with any Hall sensor wiring.
    /// Angles should be in [0, 2π). States 0 and 7 are invalid.
    pub fn set_calibration_raw(&mut self, raw_table: [f32; 8]) {
        self.calib = HallCalibration::from_raw_table(raw_table, self.calib.advance_rad);
    }

    /// Calibrate Hall table using logical-state angles (6-entry table).
    ///
    /// For backwards compatibility. Angles should be in [0, 2π) for the
    /// clockwise logical sequence 0→1→2→3→4→5.
    pub fn set_calibration(&mut self, table_rad: [f32; 6]) {
        self.calib = HallCalibration::from_logical_table(table_rad, self.calib.advance_rad);
    }

    /// Apply calibration result from `HallCalibrator`
    ///
    /// Uses the raw-state angles directly from calibration result.
    /// Returns `true` if calibration was valid and applied, `false` otherwise.
    pub fn apply_calibration(
        &mut self,
        result: &super::hall_calibration::HallCalibrationResult,
    ) -> bool {
        if result.is_valid() {
            self.set_calibration_raw(result.angles);
            true
        } else {
            false
        }
    }

    /// Set additional electrical advance (radians) applied to calibrated angles.
    pub fn set_advance(&mut self, advance_rad: f32) {
        self.calib.advance_rad = advance_rad;
    }

    /// Interpolated sample at `now_ticks` (immutable version).
    ///
    /// For the full VESC-style behavior with soft drift correction and
    /// rate limiting, use `sample_at_mut()` instead.
    ///
    /// This version applies:
    /// - Below `interp_min_vel`: returns sector angle only (no interpolation)
    /// - Above threshold: linear interpolation with hard drift clamping
    pub fn sample_at(&self, now_ticks: u64) -> Option<AngleSample> {
        let t0 = self.last_edge_ticks?;
        let dt = now_ticks.wrapping_sub(t0) as f32 / self.ticks_per_sec as f32;

        // Current sector angle from calibration (direct raw state lookup)
        let sector_angle = self.calib.angle_for_state(self.raw_state);

        // Check if velocity is below interpolation threshold (compare in eRPM)
        if Self::vel_to_erpm(self.elec_velocity) < self.interp_min_erpm {
            // Low speed: use sector angle directly, no interpolation
            // This prevents oscillation during direction reversals
            return Some(AngleSample {
                angle: sector_angle,
                omega: self.elec_velocity,
                direction: self.direction,
            });
        }

        // High speed: interpolate using velocity
        let interpolated = wrap_angle(self.angle + self.elec_velocity * dt);

        // Check drift from sector angle and apply soft correction
        let drift = angle_difference(interpolated, sector_angle);

        let final_angle = if drift.abs() <= self.max_drift_rad {
            // Within acceptable range: use interpolated angle
            interpolated
        } else {
            // Too far from sector: apply soft correction (pull back by gain %)
            // This is the VESC-style gradual correction
            let correction = drift * self.drift_correction_gain;
            wrap_angle(interpolated - correction)
        };

        Some(AngleSample {
            angle: final_angle,
            omega: self.elec_velocity,
            direction: self.direction,
        })
    }

    /// Interpolated sample at `now_ticks` with full VESC-style processing.
    ///
    /// Applies:
    /// - Below `interp_min_vel`: returns sector angle only (no interpolation)
    /// - Above threshold: linear interpolation with:
    ///   - **Soft drift correction**: Gradually pulls angle back toward sector
    ///   - **Rate limiting**: Limits angle rate of change to prevent current spikes
    ///
    /// This version mutates internal state to track rate limiting across calls.
    pub fn sample_at_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        let t0 = self.last_edge_ticks?;
        let dt_from_edge = now_ticks.wrapping_sub(t0) as f32 / self.ticks_per_sec as f32;

        // Calculate dt since last sample for rate limiting
        let dt_sample = if let Some(last) = self.last_sample_ticks {
            let dt_ticks = now_ticks.wrapping_sub(last).max(1);
            dt_ticks as f32 / self.ticks_per_sec as f32
        } else {
            dt_from_edge
        };
        self.last_sample_ticks = Some(now_ticks);

        // Current sector angle from calibration (direct raw state lookup)
        let sector_angle = self.calib.angle_for_state(self.raw_state);

        // Check if velocity is below interpolation threshold (compare in eRPM)
        if Self::vel_to_erpm(self.elec_velocity) < self.interp_min_erpm {
            // Low speed: use sector angle directly, no interpolation
            // This prevents oscillation during direction reversals
            self.rate_limited_angle = sector_angle;
            return Some(AngleSample {
                angle: sector_angle,
                omega: self.elec_velocity,
                direction: self.direction,
            });
        }

        // High speed: interpolate using velocity
        let target_angle = wrap_angle(self.angle + self.elec_velocity * dt_from_edge);

        // === SOFT DRIFT CORRECTION (VESC-style) ===
        // Check drift from sector angle
        let drift = angle_difference(target_angle, sector_angle);
        let corrected_target = if drift.abs() > self.max_drift_rad {
            // Too far from sector: apply soft correction (pull back by gain %)
            let correction = drift * self.drift_correction_gain;
            wrap_angle(target_angle - correction)
        } else {
            target_angle
        };

        // === RATE LIMITING (VESC-style) ===
        // Limit how fast angle can change per sample to prevent current spikes
        // max_step = max(|velocity|, min_vel_from_erpm) * dt * rate_limit_factor
        // Convert interp_min_erpm to rad/s for this calculation
        let min_vel_rad_s = self.interp_min_erpm * TAU / 60.0;
        let effective_vel = self.elec_velocity.abs().max(min_vel_rad_s);
        let max_step = effective_vel * dt_sample * self.rate_limit_factor;

        // Calculate desired step from current rate-limited angle
        let desired_step = angle_difference(corrected_target, self.rate_limited_angle);

        // Apply rate limiting
        let actual_step = if desired_step.abs() <= max_step {
            desired_step
        } else {
            // Clamp to max step, preserving sign
            if desired_step > 0.0 {
                max_step
            } else {
                -max_step
            }
        };

        self.rate_limited_angle = wrap_angle(self.rate_limited_angle + actual_step);

        Some(AngleSample {
            angle: self.rate_limited_angle,
            omega: self.elec_velocity,
            direction: self.direction,
        })
    }

    /// Electrical velocity estimate (rad/s) from last transition
    pub fn electrical_velocity(&self) -> f32 {
        self.elec_velocity
    }

    #[inline]
    fn angle_step_signed(&self, dir: Direction) -> f32 {
        match dir {
            Direction::Clockwise => self.angle_per_state,
            Direction::CounterClockwise => -self.angle_per_state,
            Direction::Stopped => 0.0,
        }
    }
}

/// Calibration data: electrical angle per raw Hall state (0-7) plus advance.
///
/// Uses 8-entry raw-state table for direct lookup without logical state conversion.
/// This is more flexible as it works with any Hall sensor wiring.
/// States 0 and 7 are invalid (all sensors low/high).
#[derive(Clone, Copy)]
pub struct HallCalibration {
    /// Electrical angle (rad) for each raw Hall state (0-7)
    /// Invalid states (0, 7) store 0.0 but should not be used
    raw_table: [f32; 8],
    /// Validity flags for each raw state
    valid: [bool; 8],
    /// Additional timing advance (radians)
    advance_rad: f32,
}

impl HallCalibration {
    /// Create calibration from raw-state angles (8-entry table)
    pub fn from_raw_table(raw_table: [f32; 8], advance_rad: f32) -> Self {
        let mut valid = [false; 8];
        let mut normalized = [0.0_f32; 8];
        for i in 1..=6 {
            normalized[i] = wrap_angle(raw_table[i]);
            valid[i] = true;
        }
        Self {
            raw_table: normalized,
            valid,
            advance_rad,
        }
    }

    /// Create calibration from logical-state angles (6-entry table)
    /// Maps logical states back to raw states using HALL_STATE_TABLE
    pub fn from_logical_table(logical_table: [f32; 6], advance_rad: f32) -> Self {
        let mut raw_table = [0.0_f32; 8];
        let mut valid = [false; 8];

        // Map logical state index to raw state
        // HALL_STATE_TABLE[raw] = logical, so we need the inverse
        // raw 1 -> logical 0, raw 3 -> logical 1, raw 2 -> logical 2,
        // raw 6 -> logical 3, raw 4 -> logical 4, raw 5 -> logical 5
        const LOGICAL_TO_RAW: [u8; 6] = [1, 3, 2, 6, 4, 5];

        for (logical, &raw) in LOGICAL_TO_RAW.iter().enumerate() {
            raw_table[raw as usize] = wrap_angle(logical_table[logical]);
            valid[raw as usize] = true;
        }

        Self {
            raw_table,
            valid,
            advance_rad,
        }
    }

    /// Get angle for a raw Hall state (returns None for invalid states)
    #[inline]
    pub fn angle_for_raw_state(&self, raw_state: u8) -> Option<f32> {
        let idx = (raw_state & 0x07) as usize;
        if self.valid[idx] {
            Some(wrap_angle(self.raw_table[idx] + self.advance_rad))
        } else {
            None
        }
    }

    /// Get angle for a raw Hall state, panicking on invalid (for internal use)
    #[inline]
    pub fn angle_for_state(&self, raw_state: u8) -> f32 {
        self.angle_for_raw_state(raw_state).unwrap_or(0.0) // Fallback for invalid states
    }

    /// Check if a raw state is valid
    #[inline]
    pub fn is_valid_state(&self, raw_state: u8) -> bool {
        self.valid[(raw_state & 0x07) as usize]
    }
}

impl Default for HallCalibration {
    fn default() -> Self {
        // Default: evenly spaced angles for raw states 1-6
        // Raw state sequence (CW): 1 → 3 → 2 → 6 → 4 → 5
        // Assign angles in sequence order
        let step = TAU / 6.0;
        let mut raw_table = [0.0_f32; 8];
        let mut valid = [false; 8];

        // Map raw states to their position in the CW sequence
        const RAW_TO_SEQUENCE: [(u8, u8); 6] = [
            (1, 0), // raw 1 is 1st in sequence
            (3, 1), // raw 3 is 2nd in sequence
            (2, 2), // raw 2 is 3rd in sequence
            (6, 3), // raw 6 is 4th in sequence
            (4, 4), // raw 4 is 5th in sequence
            (5, 5), // raw 5 is 6th in sequence
        ];

        for (raw, seq_pos) in RAW_TO_SEQUENCE {
            raw_table[raw as usize] = step * seq_pos as f32;
            valid[raw as usize] = true;
        }

        Self {
            raw_table,
            valid,
            advance_rad: 0.0,
        }
    }
}

#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a < 0.0 {
        a += TAU;
    }
    a
}

/// Compute signed angle difference (a - b), handling wraparound.
/// Result is in range (-π, π].
#[inline]
fn angle_difference(a: f32, b: f32) -> f32 {
    let mut diff = a - b;
    while diff > core::f32::consts::PI {
        diff -= TAU;
    }
    while diff <= -core::f32::consts::PI {
        diff += TAU;
    }
    diff
}

impl AngleSensor for HallSensor {
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        self.sample_at(now_ticks)
    }

    fn error_count(&self) -> u32 {
        self.error_count
    }

    fn reset_errors(&mut self) {
        self.error_count = 0;
    }
}

impl HallSensorTrait for HallSensor {
    fn raw_state(&self) -> u8 {
        self.raw_state
    }

    fn logical_state(&self) -> u8 {
        self.logical_state
    }

    fn last_edge_ticks(&self) -> Option<u64> {
        self.last_edge_ticks
    }

    fn electrical_velocity(&self) -> f32 {
        self.elec_velocity
    }

    fn set_calibration(&mut self, table: [f32; 6]) {
        HallSensor::set_calibration(self, table)
    }

    fn set_calibration_raw(&mut self, raw_table: [f32; 8]) {
        HallSensor::set_calibration_raw(self, raw_table)
    }

    fn apply_calibration(
        &mut self,
        result: &super::hall_calibration::HallCalibrationResult,
    ) -> bool {
        HallSensor::apply_calibration(self, result)
    }

    fn set_advance(&mut self, advance_rad: f32) {
        HallSensor::set_advance(self, advance_rad)
    }

    fn advance(&self) -> f32 {
        self.calib.advance_rad
    }

    fn interpolation_info(&self, now_ticks: u64) -> HallInterpolationInfo {
        let base_angle = self.calib.angle_for_state(self.raw_state);
        let (interpolation_offset, time_since_edge_us) = if let Some(t0) = self.last_edge_ticks {
            let dt_ticks = now_ticks.wrapping_sub(t0);
            let dt_sec = dt_ticks as f32 / self.ticks_per_sec as f32;
            let offset = self.elec_velocity * dt_sec;
            let time_us = (dt_sec * 1_000_000.0) as u32;
            (offset, time_us)
        } else {
            (0.0, 0)
        };

        HallInterpolationInfo {
            base_angle,
            interpolation_offset,
            estimated_velocity: self.elec_velocity,
            time_since_edge_us,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hall_state_table() {
        assert_eq!(HALL_STATE_TABLE[0], 0); // Invalid
        assert_eq!(HALL_STATE_TABLE[1], 0); // State 0
        assert_eq!(HALL_STATE_TABLE[2], 2); // State 2
        assert_eq!(HALL_STATE_TABLE[3], 1); // State 1
        assert_eq!(HALL_STATE_TABLE[4], 4); // State 4
        assert_eq!(HALL_STATE_TABLE[5], 5); // State 5
        assert_eq!(HALL_STATE_TABLE[6], 3); // State 3
        assert_eq!(HALL_STATE_TABLE[7], 0); // Invalid
    }

    #[test]
    fn test_hall_sensor_creation() {
        let hall = HallSensor::new(1);
        assert_eq!(hall.angle(), 0.0);
        assert_eq!(hall.direction(), Direction::Stopped);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_invalid_states() {
        let mut hall = HallSensor::new(1);

        // All low (0b000)
        assert!(hall.update(0, 0).is_none());
        assert_eq!(hall.error_count(), 1);

        // All high (0b111)
        assert!(hall.update(7, 0).is_none());
        assert_eq!(hall.error_count(), 2);
    }

    #[test]
    fn test_forward_sequence() {
        let mut hall = HallSensor::new(1_000);

        // Valid CW sequence: 1 → 3 → 2 → 6 → 4 → 5 → (wrap to 1)
        let sequence = [1, 3, 2, 6, 4, 5, 1];
        let expected_angles = [
            0.0,
            TAU / 6.0,
            TAU / 3.0,
            TAU / 2.0,
            2.0 * TAU / 3.0,
            5.0 * TAU / 6.0,
            0.0, // Wraps back
        ];

        for (i, &state) in sequence.iter().enumerate() {
            let angle = hall.update(state, i as u64).unwrap();
            assert!(
                (angle - expected_angles[i]).abs() < 1e-5,
                "Step {}: expected {}, got {}",
                i,
                expected_angles[i],
                angle
            );
        }

        // After full sequence, should be CW
        assert_eq!(hall.direction(), Direction::Clockwise);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_reverse_sequence() {
        let mut hall = HallSensor::new(1_000);

        // Start from state 5
        hall.update(5, 0).unwrap();

        // Valid CCW sequence: 5 → 4 → 6 → 2 → 3 → 1 → (wrap to 5)
        let sequence = [4, 6, 2, 3, 1, 5];

        for (i, &state) in sequence.iter().enumerate() {
            hall.update(state, (i + 1) as u64).unwrap();
        }

        // Should detect CCW direction
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_electrical_angle_increment() {
        let mut hall = HallSensor::new(1_000);

        // Electrical angle increment is TAU / 6 (completes 2π every 6 Hall states)
        let expected_increment = TAU / 6.0;

        let angle1 = hall.update(1, 0).unwrap();
        assert!((angle1 - 0.0).abs() < 1e-5);

        let angle2 = hall.update(3, 1).unwrap();
        assert!((angle2 - expected_increment).abs() < 1e-5);
    }

    #[test]
    fn test_error_detection() {
        let mut hall = HallSensor::new(1_000);

        // Valid state
        hall.update(1, 0).unwrap();

        // Invalid jump (should increment error)
        let initial_errors = hall.error_count();
        hall.update(6, 0).unwrap(); // Jumping from 1 to 6 (state 0 to 3)
        assert!(hall.error_count() > initial_errors);
    }

    #[test]
    fn test_reset_errors() {
        let mut hall = HallSensor::new(1_000);

        hall.update(0, 0).unwrap_or(0.0); // Generate error
        assert!(hall.error_count() > 0);

        hall.reset_errors();
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_interpolation_and_velocity() {
        let mut hall = HallSensor::new(10_000); // Higher tick rate for shorter dt
        hall.update(1, 0).unwrap();
        hall.update(3, 10).unwrap(); // 1 ms later with 10 kHz ticks

        let expected_vel = (TAU / 6.0) / 0.001;
        assert!((hall.electrical_velocity() - expected_vel).abs() < expected_vel * 0.01);

        // Sample shortly after the edge (0.1ms) - well within drift threshold
        let interp = hall.sample_at(11).unwrap();
        let expected_angle = (TAU / 6.0) + expected_vel * 0.0001;
        let diff = (wrap_angle(interp.angle) - wrap_angle(expected_angle)).abs();
        assert!(diff < 1e-3);
    }

    #[test]
    fn test_low_speed_no_interpolation() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz ticks
        hall.update(1, 0).unwrap();
        // Very slow: 100ms between edges = ~10 rad/s (below 52 rad/s threshold)
        hall.update(3, 100_000).unwrap();

        let sector_angle = TAU / 6.0; // Angle for logical state 1
        let sample = hall.sample_at(150_000).unwrap();

        // At low speed, should return sector angle exactly (no interpolation)
        assert!(
            (sample.angle - sector_angle).abs() < 1e-5,
            "Low speed should use sector angle, got {} expected {}",
            sample.angle,
            sector_angle
        );
    }

    #[test]
    fn test_soft_drift_correction() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0).unwrap();
        hall.update(3, 1000).unwrap(); // 1ms = 1047 rad/s (high speed)

        // Sample way in the future - interpolation would drift far
        let sample = hall.sample_at(2000).unwrap(); // 1ms after edge

        // Without correction, drift would be ~60° (TAU/6)
        // With soft correction (1% pull-back), angle should be pulled back slightly
        let sector_angle = TAU / 6.0;
        let uncorrected_angle = wrap_angle(sector_angle + (TAU / 6.0)); // ~60° drift
        let diff_from_uncorrected = angle_difference(sample.angle, uncorrected_angle).abs();

        // Should be pulled back by 1% of the drift (approximately)
        // The drift is ~60° = TAU/6, correction is ~0.6° = TAU/6 * 0.01
        assert!(
            diff_from_uncorrected > 0.005, // Should be noticeably different
            "Soft correction should pull angle back, but diff from uncorrected was only {} rad",
            diff_from_uncorrected
        );

        // Should still be reasonable (not wildly off)
        let diff_from_sector = angle_difference(sample.angle, sector_angle).abs();
        assert!(
            diff_from_sector < TAU / 4.0, // Less than 90°
            "Drift should be reasonable, got {} rad from sector",
            diff_from_sector
        );
    }

    #[test]
    fn test_rate_limiting() {
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0).unwrap();
        hall.update(3, 1000).unwrap(); // 1ms = 1047 rad/s (high speed)

        // First sample establishes baseline
        let sample1 = hall.sample_at_mut(1001).unwrap();

        // Second sample 10µs later - rate limiting should constrain step
        let sample2 = hall.sample_at_mut(1011).unwrap();

        let step = angle_difference(sample2.angle, sample1.angle).abs();

        // Max step should be: velocity * dt * rate_limit_factor
        // = 1047 * 0.00001 * 1.5 ≈ 0.0157 rad
        let expected_max_step = 1047.0 * 0.00001 * DEFAULT_RATE_LIMIT_FACTOR;

        assert!(
            step <= expected_max_step * 1.1, // Allow 10% margin for floating point
            "Rate limiting should constrain step to {}, got {} rad",
            expected_max_step,
            step
        );
    }

    #[test]
    fn test_direction_reversal_detection() {
        let mut hall = HallSensor::new(1_000);

        // Move forward: 1 → 3 (CW)
        hall.update(1, 0).unwrap();
        hall.update(3, 1).unwrap();
        assert_eq!(hall.direction(), Direction::Clockwise);
        assert!(!hall.direction_reversed());

        // Reverse: 3 → 1 (CCW) - this is a reversal
        hall.update(1, 2).unwrap();
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert!(hall.direction_reversed());
        assert_eq!(hall.prev_direction(), Direction::Clockwise);

        // Continue CCW: 1 → 5 - not a reversal
        hall.update(5, 3).unwrap();
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert!(!hall.direction_reversed());
    }

    #[test]
    fn test_timeout_detection() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz ticks

        // No edges yet - should be stale
        assert!(hall.is_stale(0));
        assert!(hall.is_stale(1_000_000));

        // Receive an edge
        hall.update(1, 0).unwrap();

        // Immediately after - not stale
        assert!(!hall.is_stale(0));
        assert!(!hall.is_stale(1000)); // 1ms later

        // Default timeout is 100ms = 100_000 ticks at 1MHz (100_000µs * 1MHz / 1_000_000)
        // Just before timeout - not stale
        assert!(!hall.is_stale(99_999));

        // At timeout - not stale (uses > not >=)
        assert!(!hall.is_stale(100_000));

        // After timeout - stale
        assert!(hall.is_stale(100_001));
        assert!(hall.is_stale(200_000));
    }

    #[test]
    fn test_is_stale_with_recent_edge() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz

        // First edge at t=0
        hall.update(1, 0).unwrap();
        assert!(hall.is_stale(100_001)); // Stale 100ms after first edge

        // Another edge at t=100_001 resets the timeout
        hall.update(3, 100_001).unwrap();
        assert!(!hall.is_stale(100_001)); // Fresh again (just received edge)
        assert!(!hall.is_stale(150_000)); // 50ms later - still fresh
        assert!(hall.is_stale(200_002)); // 100ms after second edge - stale
    }

    #[test]
    fn test_time_since_edge() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz

        // No edge yet
        assert!(hall.time_since_edge(1000).is_none());
        assert!(hall.time_since_edge_us(1000).is_none());

        // After edge
        hall.update(1, 1_000_000).unwrap(); // Edge at t=1s

        // 500ms later
        let elapsed = hall.time_since_edge(1_500_000).unwrap();
        assert_eq!(elapsed, 500_000); // 500_000 ticks

        let elapsed_us = hall.time_since_edge_us(1_500_000).unwrap();
        assert_eq!(elapsed_us, 500_000); // 500ms = 500_000µs
    }

    #[test]
    fn test_set_timeout() {
        let mut hall = HallSensor::new(1_000_000); // 1 MHz

        // Default is 100ms = 100_000µs
        assert_eq!(hall.timeout_us(), 100_000);

        // Set to 50ms = 50_000µs
        hall.set_timeout_us(50_000);
        assert_eq!(hall.timeout_us(), 50_000);

        // Verify the shorter timeout works
        // At 1MHz, 50ms = 50_000 ticks
        hall.update(1, 0).unwrap();
        assert!(!hall.is_stale(49_999)); // Just before 50ms
        assert!(hall.is_stale(50_001)); // Just after 50ms
    }
}

//! Hall sensor angle estimation for BLDC/FOC motor control
//!
//! Hall sensors provide 6 discrete positions per electrical revolution.
//! This module converts Hall sensor states to electrical angles and can
//! interpolate between edges using the measured electrical speed (similar
//! to VESC `mcpwm_foc` low-speed handling).

use core::f32::consts::TAU;

use super::sensors::{AngleSample, AngleSensor};

/// Hall state lookup table: maps raw sensor reading (0-7) to logical state (0-5)
///
/// Valid Hall sequence (CW rotation): 1 → 3 → 2 → 6 → 4 → 5 (repeat)
/// This corresponds to: 001 → 011 → 010 → 110 → 100 → 101 in binary
///
/// The table maps raw 3-bit Hall reading to normalized state:
/// - Invalid states (0, 7) map to 0 with error flag
/// - Valid states (1,2,3,4,5,6) map to their position in the sequence
const HALL_STATE_TABLE: [u8; 8] = [
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

/// Platform-agnostic Hall sensor angle estimator
///
/// Tracks Hall sensor state transitions to estimate electrical angle
/// and direction of rotation.
///
/// Returns electrical angle (0 to 2π per electrical revolution),
/// which completes every 6 Hall states regardless of motor pole count.
pub struct HallSensor {
    /// Electrical angle increment per Hall state change (TAU / 6)
    angle_per_state: f32,
    /// Calibration table: electrical angle (rad) for each logical Hall state (0-5)
    calib: HallCalibration,
    /// Current electrical angle at last Hall edge (radians, 0 to 2π)
    angle: f32,
    /// Previous Hall state (0-5)
    state_prev: u8,
    /// Current direction of rotation
    direction: Direction,
    /// Last edge timestamp (ticks of a caller-provided clock)
    last_edge_ticks: Option<u64>,
    /// Electrical velocity estimate (rad/s) from last edge
    elec_velocity: f32,
    /// Error counter for invalid states or transitions
    error_count: u32,
    /// Timebase (ticks per second)
    ticks_per_sec: u64,
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
            state_prev: 0,
            direction: Direction::Stopped,
            last_edge_ticks: None,
            elec_velocity: 0.0,
            error_count: 0,
            ticks_per_sec: ticks_per_sec.max(1),
        }
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
        // Check for invalid states (all low or all high)
        if raw_state == 0 || raw_state > 6 {
            self.error_count += 1;
            return None;
        }

        let prev_state = self.state_prev;
        let current_state = HALL_STATE_TABLE[raw_state as usize];
        // Detect direction based on state transitions
        let is_wrap_cw = prev_state == 5 && current_state == 0;
        let is_wrap_ccw = prev_state == 0 && current_state == 5;
        if current_state.abs_diff(prev_state) > 1 && !is_wrap_cw && !is_wrap_ccw {
            // Non-adjacent transition (not wraparound) => error
            self.error_count += 1;
        }
        if current_state == 0 && prev_state == 5 {
            self.direction = Direction::Clockwise;
        } else if current_state == 5 && prev_state == 0 {
            self.direction = Direction::CounterClockwise;
        } else if current_state > prev_state {
            self.direction = Direction::Clockwise;
        } else if current_state < prev_state {
            self.direction = Direction::CounterClockwise;
        }
        self.state_prev = current_state;

        // Base angle from calibration table (includes user-set advance)
        let angle_raw = self.calib.angle_for_state(current_state);

        // Velocity estimate from last edge
        if let Some(last_t) = self.last_edge_ticks {
            let dt_ticks = t_ticks.wrapping_sub(last_t).max(1);
            let dt = dt_ticks as f32 / self.ticks_per_sec as f32;
            let angle_step = self.angle_step_signed(self.direction);
            self.elec_velocity = angle_step / dt;
        }
        self.last_edge_ticks = Some(t_ticks);
        self.angle = angle_raw;

        Some(self.angle)
    }

    /// Update and return a full reading (angle, dir, velocity) or an error.
    pub fn update_sample(&mut self, raw_state: u8, t_ticks: u64) -> Result<HallReading, HallError> {
        match self.update(raw_state, t_ticks) {
            Some(angle) => Ok(HallReading {
                angle_rad: angle,
                direction: self.direction,
                state: self.state_prev,
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

    /// Get the raw Hall state index (0-5)
    pub fn state(&self) -> u8 {
        self.state_prev
    }

    /// Calibrate Hall table (electrical angles in radians for logical states 0-5).
    ///
    /// Angles should be in [0, 2π) and monotonically increasing for the
    /// clockwise sequence 0→1→2→3→4→5.
    pub fn set_calibration(&mut self, table_rad: [f32; 6]) {
        self.calib = HallCalibration::new(table_rad, self.calib.advance_rad);
    }

    /// Set additional electrical advance (radians) applied to calibrated angles.
    pub fn set_advance(&mut self, advance_rad: f32) {
        self.calib.advance_rad = advance_rad;
    }

    /// Interpolated sample at `now_ticks`.
    pub fn sample_at(&self, now_ticks: u64) -> Option<AngleSample> {
        let t0 = self.last_edge_ticks?;
        let dt = now_ticks.wrapping_sub(t0) as f32 / self.ticks_per_sec as f32;
        let angle = wrap_angle(self.angle + self.elec_velocity * dt);
        Some(AngleSample {
            angle,
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

/// Calibration data: electrical angle per logical state (0-5) plus advance.
#[derive(Clone, Copy)]
pub struct HallCalibration {
    table_rad: [f32; 6],
    advance_rad: f32,
}

impl HallCalibration {
    pub fn new(table_rad: [f32; 6], advance_rad: f32) -> Self {
        Self {
            table_rad: normalize_table(table_rad),
            advance_rad,
        }
    }

    pub fn angle_for_state(&self, state: u8) -> f32 {
        let idx = state as usize;
        let base = self.table_rad[idx.min(5)];
        wrap_angle(base + self.advance_rad)
    }
}

impl Default for HallCalibration {
    fn default() -> Self {
        let mut table = [0.0_f32; 6];
        let step = TAU / 6.0;
        for (i, v) in table.iter_mut().enumerate() {
            *v = step * i as f32;
        }
        Self {
            table_rad: table,
            advance_rad: 0.0,
        }
    }
}

fn normalize_table(mut t: [f32; 6]) -> [f32; 6] {
    for v in &mut t {
        *v = wrap_angle(*v);
    }
    t
}

#[inline]
fn wrap_angle(angle: f32) -> f32 {
    let mut a = angle % TAU;
    if a < 0.0 {
        a += TAU;
    }
    a
}

impl AngleSensor for HallSensor {
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        self.sample_at(now_ticks)
    }

    fn error_count(&self) -> u32 {
        self.error_count()
    }

    fn reset_errors(&mut self) {
        HallSensor::reset_errors(self)
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
        let mut hall = HallSensor::new(2_000);
        hall.update(1, 0).unwrap();
        hall.update(3, 2).unwrap(); // 1 ms later with 2 kHz ticks

        let expected_vel = (TAU / 6.0) / 0.001;
        assert!((hall.electrical_velocity() - expected_vel).abs() < expected_vel * 0.01);

        let interp = hall.sample_at(3).unwrap();
        let expected_angle = (TAU / 6.0) + expected_vel * 0.0005;
        let diff = (wrap_angle(interp.angle) - wrap_angle(expected_angle)).abs();
        assert!(diff < 1e-3);
    }
}

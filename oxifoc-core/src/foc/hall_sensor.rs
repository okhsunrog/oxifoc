//! Hall sensor angle estimation for BLDC/FOC motor control
//!
//! Hall sensors provide 6 discrete positions per electrical revolution.
//! This module converts Hall sensor states to electrical angles.

use core::f32::consts::TAU;

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Clockwise rotation
    Clockwise,
    /// Counter-clockwise rotation
    CounterClockwise,
    /// Motor stopped or direction unknown
    Stopped,
}

/// Platform-agnostic Hall sensor angle estimator
///
/// Tracks Hall sensor state transitions to estimate electrical angle
/// and direction of rotation.
pub struct HallSensor {
    /// Number of motor pole pairs
    pole_pairs: u8,
    /// Electrical angle increment per Hall state change
    angle_per_state: f32,
    /// Current electrical angle (radians, 0 to 2π)
    angle: f32,
    /// Previous Hall state (0-5)
    state_prev: u8,
    /// Current direction of rotation
    direction: Direction,
    /// Error counter for invalid states or transitions
    error_count: u32,
    /// Maximum Hall index (pole_pairs * 6)
    hall_idx_max: usize,
    /// Base Hall index (increments by 6 each electrical revolution)
    hall_idx_base: usize,
}

impl HallSensor {
    /// Create a new Hall sensor estimator
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of motor pole pairs (poles / 2)
    ///
    /// For a 14-pole motor: pole_pairs = 7
    pub fn new(pole_pairs: u8) -> Self {
        let hall_idx_max = pole_pairs as usize * 6;
        let angle_per_state = TAU / (hall_idx_max as f32);

        HallSensor {
            pole_pairs,
            angle_per_state,
            angle: 0.0,
            state_prev: 0,
            direction: Direction::Stopped,
            error_count: 0,
            hall_idx_max,
            hall_idx_base: 0,
        }
    }

    /// Update angle based on new Hall sensor reading
    ///
    /// # Arguments
    /// * `raw_state` - 3-bit Hall sensor value (H3<<2 | H2<<1 | H1)
    ///
    /// # Returns
    /// * `Some(angle)` - Electrical angle in radians (0 to 2π)
    /// * `None` - Invalid Hall state (0 or 7)
    pub fn update(&mut self, raw_state: u8) -> Option<f32> {
        // Check for invalid states (all low or all high)
        if raw_state == 0 || raw_state > 6 {
            self.error_count += 1;
            return None;
        }

        let prev_state = self.state_prev;
        let current_state = HALL_STATE_TABLE[raw_state as usize];
        self.state_prev = current_state;

        // Detect direction and update Hall index base
        // State 0 → 5: CW wraps to next electrical cycle
        // State 5 → 0: CCW wraps to previous electrical cycle
        match current_state {
            0 => {
                if prev_state == 5 {
                    // Forward wrap (CW)
                    self.hall_idx_base += 6;
                    if self.hall_idx_base >= self.hall_idx_max {
                        self.hall_idx_base = 0;
                    }
                    self.direction = Direction::Clockwise;
                } else if prev_state != 1 && prev_state != 0 {
                    // Invalid transition
                    self.error_count += 1;
                }
            }
            5 => {
                if prev_state == 0 {
                    // Backward wrap (CCW)
                    if self.hall_idx_base < 6 {
                        self.hall_idx_base = self.hall_idx_max - 6;
                    } else {
                        self.hall_idx_base -= 6;
                    }
                    self.direction = Direction::CounterClockwise;
                } else if prev_state != 4 && prev_state != 5 {
                    // Invalid transition
                    self.error_count += 1;
                }
            }
            _ => {
                // Normal state transitions (should differ by ±1)
                if current_state.abs_diff(prev_state) > 1 {
                    self.error_count += 1;
                }
                // Update direction based on transition
                if current_state > prev_state {
                    self.direction = Direction::Clockwise;
                } else if current_state < prev_state {
                    self.direction = Direction::CounterClockwise;
                }
            }
        }

        // Calculate electrical angle
        let hall_state_idx = self.hall_idx_base + current_state as usize;
        self.angle = self.angle_per_state * hall_state_idx as f32;

        Some(self.angle)
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

    /// Get number of pole pairs
    pub fn pole_pairs(&self) -> u8 {
        self.pole_pairs
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
        let hall = HallSensor::new(7); // 14-pole motor
        assert_eq!(hall.pole_pairs(), 7);
        assert_eq!(hall.angle(), 0.0);
        assert_eq!(hall.direction(), Direction::Stopped);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_invalid_states() {
        let mut hall = HallSensor::new(7);

        // All low (0b000)
        assert!(hall.update(0).is_none());
        assert_eq!(hall.error_count(), 1);

        // All high (0b111)
        assert!(hall.update(7).is_none());
        assert_eq!(hall.error_count(), 2);
    }

    #[test]
    fn test_forward_sequence() {
        let mut hall = HallSensor::new(1); // 2-pole motor for simplicity

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
            let angle = hall.update(state).unwrap();
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
        let mut hall = HallSensor::new(1); // 2-pole motor

        // Start from state 5
        hall.update(5).unwrap();

        // Valid CCW sequence: 5 → 4 → 6 → 2 → 3 → 1 → (wrap to 5)
        let sequence = [4, 6, 2, 3, 1, 5];

        for &state in &sequence {
            hall.update(state).unwrap();
        }

        // Should detect CCW direction
        assert_eq!(hall.direction(), Direction::CounterClockwise);
        assert_eq!(hall.error_count(), 0);
    }

    #[test]
    fn test_multi_pole_motor() {
        let mut hall = HallSensor::new(7); // 14-pole motor

        // Angle increment should be TAU / 42 (7 pole pairs * 6 states)
        let expected_increment = TAU / 42.0;

        let angle1 = hall.update(1).unwrap();
        assert!((angle1 - 0.0).abs() < 1e-5);

        let angle2 = hall.update(3).unwrap();
        assert!((angle2 - expected_increment).abs() < 1e-5);
    }

    #[test]
    fn test_error_detection() {
        let mut hall = HallSensor::new(1);

        // Valid state
        hall.update(1).unwrap();

        // Invalid jump (should increment error)
        let initial_errors = hall.error_count();
        hall.update(6).unwrap(); // Jumping from 1 to 6 (state 0 to 3)
        assert!(hall.error_count() > initial_errors);
    }

    #[test]
    fn test_reset_errors() {
        let mut hall = HallSensor::new(1);

        hall.update(0).unwrap_or(0.0); // Generate error
        assert!(hall.error_count() > 0);

        hall.reset_errors();
        assert_eq!(hall.error_count(), 0);
    }
}

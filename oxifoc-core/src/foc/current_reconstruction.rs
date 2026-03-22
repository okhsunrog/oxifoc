//! Sector-based phase current reconstruction for unipolar shunt sensing
//!
//! On boards with low-side shunts and no Vref/2 bias (e.g., B-G431B-ESC1),
//! the OPAMP output clips negative currents to 0V. Only two of three phase
//! currents are valid at any time. This module reconstructs the invalid
//! phase using ia + ib + ic = 0.
//!
//! The invalid phase is the one with the highest PWM duty — its current
//! flows in the direction that produces negative shunt voltage during the
//! V0 null vector (when all low-side FETs are ON for ADC sampling).
//!
//! Duty values from the *previous* FOC cycle are used, since the PWM pattern
//! written in cycle N is what's active when cycle N+1's ADC fires.

/// Reconstruction state, stored inside `GenericCurrentSensor`.
#[derive(Default)]
pub struct ReconstructionState {
    /// Duties from the previous FOC cycle
    prev_duties: [u16; 3],
}

impl ReconstructionState {
    /// Create with zero initial duties (safe — no reconstruction on first cycle)
    pub fn new() -> Self {
        Self {
            prev_duties: [0, 0, 0],
        }
    }

    /// Store duties for use in the next cycle's reconstruction
    pub fn set_duties(&mut self, duties: [u16; 3]) {
        self.prev_duties = duties;
    }

    /// Reconstruct the invalid phase current.
    ///
    /// Identifies which phase has the highest duty (invalid reading due to
    /// negative shunt voltage clipping), and computes it from the other two.
    #[inline]
    pub fn reconstruct(&self, ia: f32, ib: f32, ic: f32) -> (f32, f32, f32) {
        let [da, db, dc] = self.prev_duties;

        // All duties equal (startup / stopped) — no reconstruction needed
        if da == db && db == dc {
            return (ia, ib, ic);
        }

        // Phase with highest duty has clipped (negative) current reading
        if da >= db && da >= dc {
            (-(ib + ic), ib, ic)
        } else if db >= da && db >= dc {
            (ia, -(ia + ic), ic)
        } else {
            (ia, ib, -(ia + ib))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_equal_duties_passthrough() {
        let mut state = ReconstructionState::new();
        state.set_duties([1000, 1000, 1000]);
        let (ia, ib, ic) = state.reconstruct(1.0, 2.0, 3.0);
        assert_eq!((ia, ib, ic), (1.0, 2.0, 3.0));
    }

    #[test]
    fn test_reconstruct_phase_a() {
        let mut state = ReconstructionState::new();
        state.set_duties([4000, 2000, 1000]); // A has highest duty
        // Phase A reading is clipped to ~0, B=5.0, C=-2.0 (true A should be -3.0)
        let (ia, ib, ic) = state.reconstruct(0.0, 5.0, -2.0);
        assert!((ia - (-3.0)).abs() < 1e-6);
        assert_eq!(ib, 5.0);
        assert_eq!(ic, -2.0);
    }

    #[test]
    fn test_reconstruct_phase_b() {
        let mut state = ReconstructionState::new();
        state.set_duties([2000, 4000, 1000]); // B has highest duty
        let (ia, ib, ic) = state.reconstruct(5.0, 0.0, -2.0);
        assert_eq!(ia, 5.0);
        assert!((ib - (-3.0)).abs() < 1e-6);
        assert_eq!(ic, -2.0);
    }

    #[test]
    fn test_reconstruct_phase_c() {
        let mut state = ReconstructionState::new();
        state.set_duties([1000, 2000, 4000]); // C has highest duty
        let (ia, ib, ic) = state.reconstruct(5.0, -2.0, 0.0);
        assert_eq!(ia, 5.0);
        assert_eq!(ib, -2.0);
        assert!((ic - (-3.0)).abs() < 1e-6);
    }

    #[test]
    fn test_sum_to_zero_after_reconstruction() {
        let mut state = ReconstructionState::new();
        state.set_duties([3500, 2000, 1500]);
        let (ia, ib, ic) = state.reconstruct(0.0, 3.0, -1.5);
        assert!((ia + ib + ic).abs() < 1e-6);
    }

    #[test]
    fn test_zero_duties_passthrough() {
        let state = ReconstructionState::new(); // [0, 0, 0]
        let (ia, ib, ic) = state.reconstruct(0.1, -0.05, -0.05);
        assert_eq!((ia, ib, ic), (0.1, -0.05, -0.05));
    }
}

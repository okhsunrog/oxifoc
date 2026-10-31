//! Causal pairing of current samples with applied voltage and PWM timing.

use super::{DEFAULT_ACTUATION_DELAY_CYCLES, MAX_ACTUATION_DELAY_CYCLES};
use crate::foc::clamp_f32;

// A delay of up to two cycles needs four commands for fractional interpolation.
const V_HIST: usize = 4;

pub(super) struct ActuationModel {
    /// Newest command first; record only after feeding this cycle's estimator.
    history: [(f32, f32); V_HIST],
    delay_cycles: f32,
}

impl Default for ActuationModel {
    fn default() -> Self {
        Self {
            history: [(0.0, 0.0); V_HIST],
            delay_cycles: DEFAULT_ACTUATION_DELAY_CYCLES,
        }
    }
}

impl ActuationModel {
    pub(super) fn set_delay(&mut self, cycles: f32) {
        if cycles.is_finite() {
            self.delay_cycles = clamp_f32(cycles, 0.0, MAX_ACTUATION_DELAY_CYCLES);
        }
    }

    pub(super) fn angle_advance(&self, velocity: f32, dt: f32) -> f32 {
        velocity * dt * (self.delay_cycles + 0.5)
    }

    /// For d = m + f, the sampling interval sees the older command for f
    /// and the newer command for 1-f. With d=0 this is the previous command;
    /// this cycle's newly computed command has not acted on the motor yet.
    pub(super) fn applied_voltage(&self) -> (f32, f32) {
        let m = (self.delay_cycles as usize).min(V_HIST - 2);
        let f = self.delay_cycles - m as f32;
        let (newer, older) = (self.history[m], self.history[m + 1]);
        (
            f * older.0 + (1.0 - f) * newer.0,
            f * older.1 + (1.0 - f) * newer.1,
        )
    }

    pub(super) fn record_command(&mut self, alpha: f32, beta: f32) {
        self.history.rotate_right(1);
        self.history[0] = (alpha, beta);
    }
}

//! Causal pairing of current samples with applied voltage and PWM timing.

use super::{DEFAULT_ACTUATION_DELAY_CYCLES, MAX_ACTUATION_DELAY_CYCLES};
use crate::foc::clamp_f32;
use crate::units::{
    ConstZero, ElectricPotential, ElectricalAngle, ElectricalAngularVelocity, Time,
};
use core::ops::{Add, Mul};

// A delay of up to two cycles needs four commands for fractional interpolation.
const V_HIST: usize = 4;

/// The commands of the last cycles, read back with the actuation delay: αβ
/// voltages for the estimators, or duty fractions for the commissioning
/// source model.
pub(super) struct ActuationModel<T = ElectricPotential> {
    /// `head` identifies the newest command. Record only after feeding this
    /// cycle's estimator; retaining all four entries allows live delay changes.
    history: [(T, T); V_HIST],
    head: usize,
    delay: PreparedDelay,
}

struct PreparedDelay {
    whole: usize,
    older_weight: f32,
    newer_weight: f32,
    advance: f32,
}

impl PreparedDelay {
    fn new(cycles: f32) -> Self {
        let cycles = clamp_f32(cycles, 0.0, MAX_ACTUATION_DELAY_CYCLES);
        let whole = (cycles as usize).min(V_HIST - 2);
        let older_weight = cycles - whole as f32;
        Self {
            whole,
            older_weight,
            newer_weight: 1.0 - older_weight,
            advance: cycles + 0.5,
        }
    }
}

impl<T: ConstZero + Copy> Default for ActuationModel<T> {
    fn default() -> Self {
        Self {
            history: [(T::ZERO, T::ZERO); V_HIST],
            head: 0,
            delay: PreparedDelay::new(DEFAULT_ACTUATION_DELAY_CYCLES),
        }
    }
}

impl<T> ActuationModel<T>
where
    T: ConstZero + Copy + Add<Output = T>,
    f32: Mul<T, Output = T>,
{
    pub(super) fn clear(&mut self) {
        self.history = [(T::ZERO, T::ZERO); V_HIST];
        self.head = 0;
    }

    pub(super) fn delay_cycles(&self) -> f32 {
        self.delay.whole as f32 + self.delay.older_weight
    }

    pub(super) fn set_delay(&mut self, cycles: f32) {
        if cycles.is_finite() {
            self.delay = PreparedDelay::new(cycles);
        }
    }

    pub(super) fn angle_advance(
        &self,
        velocity: ElectricalAngularVelocity,
        dt: Time,
    ) -> ElectricalAngle {
        velocity * dt * self.delay.advance
    }

    /// For d = m + f, the sampling interval sees the older command for f
    /// and the newer command for 1-f. With d=0 this is the previous command;
    /// this cycle's newly computed command has not acted on the motor yet.
    pub(super) fn applied_voltage(&self) -> (T, T) {
        let m = (self.head + self.delay.whole) & (V_HIST - 1);
        let newer = self.history[m];
        let older = self.history[(m + 1) & (V_HIST - 1)];
        let f = self.delay.older_weight;
        let g = self.delay.newer_weight;
        (f * older.0 + g * newer.0, f * older.1 + g * newer.1)
    }

    pub(super) fn record_command(&mut self, alpha: T, beta: T) {
        self.head = self.head.wrapping_sub(1) & (V_HIST - 1);
        self.history[self.head] = (alpha, beta);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{ToBits, erad_per_s, seconds};

    #[test]
    fn prepared_ring_matches_shifted_history_across_live_delay_changes() {
        let mut model = ActuationModel::default();
        let mut reference = [(0.0_f32, 0.0_f32); V_HIST];
        let mut delay = DEFAULT_ACTUATION_DELAY_CYCLES;
        for n in 0..2_000 {
            let command = (n as f32 * 0.17 - 12.0, 2.5 - n as f32 * 0.013);
            model.record_command(command.0, command.1);
            reference.rotate_right(1);
            reference[0] = command;
            for request in [
                -1.0,
                -0.0,
                0.0,
                0.125,
                0.5,
                0.99,
                1.0,
                1.7,
                2.0,
                3.0,
                f32::NAN,
                f32::INFINITY,
            ] {
                model.set_delay(request);
                if request.is_finite() {
                    delay = clamp_f32(request, 0.0, MAX_ACTUATION_DELAY_CYCLES);
                }
                let m = (delay as usize).min(V_HIST - 2);
                let f = delay - m as f32;
                let expected = (
                    f * reference[m + 1].0 + (1.0 - f) * reference[m].0,
                    f * reference[m + 1].1 + (1.0 - f) * reference[m].1,
                );
                let actual = model.applied_voltage();
                assert_eq!(actual.0.to_bits(), expected.0.to_bits());
                assert_eq!(actual.1.to_bits(), expected.1.to_bits());
                assert_eq!(
                    model
                        .angle_advance(erad_per_s(-123.0), seconds(0.00005))
                        .to_bits(),
                    (-123.0_f32 * 0.00005 * (delay + 0.5)).to_bits()
                );
            }
        }
    }
}

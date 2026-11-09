//! Control velocity from the Hall commutation angle, separate from edge timing.

use crate::foc::angle_difference;
use crate::foc::fast_math::u64_to_f32;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::sensors::ControlAngleSample;
use core::f32::consts::PI;

// At 20 kHz this is alpha = 0.01, matching the corrected-angle speed
// filter used for VESC's feedforward. The time constant, not a fixed alpha,
// also applies when Stopped decimates phase updates or the loop rate changes.
const VELOCITY_TAU_S: f32 = 0.00495;

#[derive(Clone, Copy)]
enum Baseline {
    /// Construction, model/calibration replacement or a new timebase.
    Cold,
    /// A source discontinuity: seed from the currently applied control speed.
    Reenter,
    Tracking {
        angle: f32,
        ticks: u64,
    },
}

/// The filtered derivative of the rate-limited Hall angle. Raw edge velocity
/// remains in the sensor for interpolation, health checks and diagnostics.
pub(super) struct HallControlMotion {
    baseline: Baseline,
    velocity: f32,
    seconds_per_tick: f32,
    reference_epoch: u32,
}

impl Default for HallControlMotion {
    fn default() -> Self {
        Self {
            baseline: Baseline::Cold,
            velocity: 0.0,
            seconds_per_tick: 1.0e-6,
            reference_epoch: 0,
        }
    }
}

impl HallControlMotion {
    pub(super) fn reset(&mut self) {
        self.baseline = Baseline::Cold;
        self.velocity = 0.0;
    }

    pub(super) fn set_ticks_per_sec(&mut self, ticks_per_sec: u64) {
        #[allow(clippy::cast_precision_loss)]
        {
            self.seconds_per_tick = 1.0 / ticks_per_sec.max(1) as f32;
        }
        self.reset();
    }

    pub(super) fn source_changed(&mut self) {
        // A model/calibration/timebase reset dominates any source commands
        // queued before the first new observation (Stopped is decimated).
        if !matches!(self.baseline, Baseline::Cold) {
            self.baseline = Baseline::Reenter;
        }
    }

    pub(super) fn unavailable(&mut self) {
        // A cold reset stays cold even if the first observation is missing.
        if matches!(self.baseline, Baseline::Tracking { .. }) {
            self.source_changed();
        }
    }

    pub(super) fn observe(
        &mut self,
        observation: ControlAngleSample,
        now_ticks: u64,
        applied_velocity: f32,
    ) -> Option<PhaseOutput> {
        let ControlAngleSample {
            sample,
            reference_epoch,
        } = observation;
        if reference_epoch != self.reference_epoch {
            self.reset();
            self.reference_epoch = reference_epoch;
        }
        if !sample.angle.is_finite() || !sample.omega.is_finite() {
            self.unavailable();
            return None;
        }
        let applied_velocity = if applied_velocity.is_finite() {
            applied_velocity
        } else {
            0.0
        };
        match self.baseline {
            Baseline::Cold => self.velocity = 0.0,
            Baseline::Reenter => self.velocity = applied_velocity,
            Baseline::Tracking { angle, ticks } => {
                let elapsed_ticks = now_ticks.wrapping_sub(ticks);
                // Exact zero derivative: both the filter and any gap/backward
                // reanchor produce +0. Keep the ordinary path for signed zero
                // or a nonzero applied source speed; no deadband is involved.
                let exact_rest = sample.angle == angle
                    && self.velocity.to_bits() == 0
                    && applied_velocity.to_bits() == 0;
                if !exact_rest {
                    // A small wrapping delta is a genuine u64 wrap. A backwards
                    // timestamp appears in the opposite half of the counter and
                    // reanchors without inventing an angular derivative.
                    if elapsed_ticks > 0 && elapsed_ticks <= u64::MAX / 2 {
                        let elapsed_s = u64_to_f32(elapsed_ticks) * self.seconds_per_tick;
                        let possible_travel = sample
                            .omega
                            .abs()
                            .max(applied_velocity.abs())
                            .max(self.velocity.abs())
                            * elapsed_s;
                        // A long observation gap or possible half-turn aliases a
                        // wrapped angle. Hold the applied speed and reanchor;
                        // never infer missing turns. This continuity guard is
                        // separate from the driver's deadline protection.
                        if elapsed_s > 4.0 * VELOCITY_TAU_S || possible_travel >= PI {
                            self.velocity = applied_velocity;
                        } else {
                            let delta = angle_difference(sample.angle, angle);
                            self.velocity = (VELOCITY_TAU_S * self.velocity + delta)
                                / (VELOCITY_TAU_S + elapsed_s);
                        }
                    } else if elapsed_ticks != 0 {
                        self.velocity = applied_velocity;
                    }
                }
                // Duplicate time: retain speed, but reanchor the angle so an
                // untimeable change is not charged to the following interval.
            }
        }
        self.baseline = Baseline::Tracking {
            angle: sample.angle,
            ticks: now_ticks,
        };
        Some(PhaseOutput {
            angle: sample.angle,
            velocity: self.velocity,
        })
    }
}

#[cfg(test)]
mod tests;

//! Physical travel accounting, independent of the commutation-angle limiter.

use crate::foc;
use crate::foc::fast_math;
use crate::units::{ConstZero, ElectricalAngle, ElectricalAngularVelocity, Time, seconds};

/// Filter a bounded position prediction, rather than filtering edge-period
/// speed. The latter can integrate to arbitrarily many turns after a stop or
/// repeated reversals. Here each captured boundary corrects the accumulated
/// distance, including zero net distance when crossing the same boundary back.
///
/// A 20-ms time constant rejects boundary corrections on the time scale of
/// low-speed Hall sectors. It is an estimator delay, not additional knowledge
/// of motion inside a sector; the speed loop must remain slower than it.
const VELOCITY_TAU: Time = seconds(0.020);

/// Longest sampling gap over which a speed is still claimed.
const MAX_SAMPLE_GAP: Time = seconds(0.1);

#[derive(Default)]
pub(super) struct RotorMotion {
    boundary: Option<ElectricalAngle>,
    pending_travel: ElectricalAngle,
    previous_offset: ElectricalAngle,
    velocity: ElectricalAngularVelocity,
    last_ticks: Option<u64>,
}

impl RotorMotion {
    pub(super) fn reset(&mut self) {
        *self = Self::default();
    }

    /// Signed boundary-to-boundary displacement, never a centroid step.
    /// Accumulating at capture time preserves turns even if several edges
    /// arrive between control samples; no wrapped-angle differentiation.
    pub(super) fn cross_boundary(&mut self, boundary: ElectricalAngle) {
        if let Some(previous) = self.boundary {
            self.pending_travel += foc::angle_difference(boundary, previous);
        } else {
            // A boot centroid has no known displacement to this first edge.
            // Preserve an already sampled zero-speed baseline: reseeding it
            // from the first incomplete edge interval invents startup travel.
            self.previous_offset = ElectricalAngle::ZERO;
            self.pending_travel = ElectricalAngle::ZERO;
        }
        self.boundary = Some(boundary);
    }

    pub(super) fn sample(
        &mut self,
        ticks: u64,
        seconds_per_tick: Time,
        offset: ElectricalAngle,
        initial_velocity: ElectricalAngularVelocity,
    ) -> ElectricalAngularVelocity {
        let Some(last) = self.last_ticks else {
            self.velocity = initial_velocity;
            self.last_ticks = Some(ticks);
            self.previous_offset = offset;
            self.pending_travel = ElectricalAngle::ZERO;
            return self.velocity;
        };
        if ticks <= last {
            // A duplicate/stale caller must not consume pending physical travel.
            return self.velocity;
        }
        let dt = fast_math::u64_to_f32(ticks - last) * seconds_per_tick;
        if dt > MAX_SAMPLE_GAP {
            // No claim about instantaneous speed across a long sampling outage.
            self.velocity = ElectricalAngularVelocity::ZERO;
        } else {
            let travel = self.pending_travel + offset - self.previous_offset;
            self.velocity = (VELOCITY_TAU * self.velocity + travel) / (VELOCITY_TAU + dt);
        }
        self.last_ticks = Some(ticks);
        self.previous_offset = offset;
        self.pending_travel = ElectricalAngle::ZERO;
        self.velocity
    }
}

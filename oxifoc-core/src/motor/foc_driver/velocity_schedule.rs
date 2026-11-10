//! Cruise-loop scheduling in the driver's existing monotonic microsecond clock.
//!
//! Only the requested q current is held. Every native current-loop step still
//! applies its current limits, fresh torque grant, trust gate and hard trip.

use crate::foc::fast_math::u64_to_f32;
use crate::foc::phase::MotionReferenceEpoch;
use crate::foc::velocity::VelocityBlockage;

const PERIOD_US: u64 = 1_000;
// One endpoint is not a useful PI observation after four outer periods.
// Bound both native silence and the integration age; native deadline/fault
// handling remains separate.
const MAX_GAP_US: u64 = PERIOD_US * 4;

pub(super) enum VelocityTick {
    Hold,
    Update { dt: f32 },
    Reanchor,
}

#[derive(Default)]
struct OuterClock {
    previous: Option<u64>,
    updated: Option<u64>,
    deadline: u64,
}

impl OuterClock {
    fn clear_integration(&mut self) {
        self.updated = None;
    }

    fn advance(&mut self, now_us: u64) -> (u32, VelocityTick) {
        let Some(previous) = self.previous.replace(now_us) else {
            self.updated = Some(now_us);
            self.deadline = now_us.wrapping_add(PERIOD_US);
            return (0, VelocityTick::Update { dt: 0.0 });
        };
        let elapsed = now_us.wrapping_sub(previous);
        if elapsed == 0 {
            return (0, VelocityTick::Hold);
        }
        if elapsed > MAX_GAP_US {
            self.updated = None;
            self.deadline = now_us.wrapping_add(PERIOD_US);
            return (0, VelocityTick::Reanchor);
        }
        let late = now_us.wrapping_sub(self.deadline);
        if late >= 1 << 63 {
            return (elapsed as u32, VelocityTick::Hold);
        }
        if self
            .updated
            .is_some_and(|last| now_us.wrapping_sub(last) > MAX_GAP_US)
        {
            self.updated = None;
            self.deadline = now_us.wrapping_add(PERIOD_US);
            return (0, VelocityTick::Reanchor);
        }
        // Retain the periodic grid, skip missed deadlines and never replay a
        // backlog of PI steps against one physical speed observation. A late
        // update may be followed by a shorter interval to the next grid point;
        // there is still at most one evaluation in each grid period/ISR.
        // The previous deadline was ahead of the previous native sample,
        // so valid lateness cannot exceed this bounded native gap.
        debug_assert!(late <= MAX_GAP_US);
        let periods = late as u32 / PERIOD_US as u32 + 1;
        self.deadline = self.deadline.wrapping_add(u64::from(periods) * PERIOD_US);
        let dt = self
            .updated
            .replace(now_us)
            .map_or(0.0, |last| u64_to_f32(now_us.wrapping_sub(last)) * 1e-6);
        (elapsed as u32, VelocityTick::Update { dt })
    }
}

#[derive(Default)]
struct ActuatorFeedback {
    held_residual: f32,
    weighted_residual: f32,
    elapsed_us: u32,
    blockage: VelocityBlockage,
    voltage_clip: VelocityBlockage,
}

impl ActuatorFeedback {
    fn advance(&mut self, elapsed_us: u32) {
        self.weighted_residual += self.held_residual * elapsed_us as f32;
        self.elapsed_us += elapsed_us;
    }

    fn record(&mut self, requested: f32, granted: f32, clip: f32) {
        let residual = granted - requested;
        self.held_residual = if residual.is_finite() { residual } else { 0.0 };
        self.blockage.note_clip(clip);
        self.blockage.note_clip(requested - granted);
        self.voltage_clip.note_clip(clip);
    }

    fn take(&mut self) -> (f32, VelocityBlockage, VelocityBlockage) {
        let correction = if self.elapsed_us != 0 && self.weighted_residual.is_finite() {
            self.weighted_residual / self.elapsed_us as f32
        } else {
            0.0
        };
        self.weighted_residual = 0.0;
        self.elapsed_us = 0;
        (
            correction,
            core::mem::take(&mut self.blockage),
            core::mem::take(&mut self.voltage_clip),
        )
    }
}

#[derive(Default)]
pub(super) struct VelocitySchedule {
    clock: OuterClock,
    feedback: ActuatorFeedback,
    epoch: Option<MotionReferenceEpoch>,
    requested_iq: f32,
}

impl VelocitySchedule {
    pub fn invalidate(&mut self) {
        self.epoch = None;
        self.requested_iq = 0.0;
        self.feedback = ActuatorFeedback::default();
        // Do not move a live deadline: command/config floods cannot turn the
        // bounded outer controller back into a native-rate PI.
        self.clock.clear_integration();
    }

    pub fn rearm(&mut self, epoch: MotionReferenceEpoch) {
        self.invalidate();
        self.epoch = Some(epoch);
    }

    pub fn matches_epoch(&self, epoch: MotionReferenceEpoch) -> bool {
        self.epoch == Some(epoch)
    }

    pub fn begin_cycle(&mut self, now_us: u64) -> VelocityTick {
        let (elapsed, tick) = self.clock.advance(now_us);
        if matches!(tick, VelocityTick::Reanchor) {
            self.invalidate();
        } else {
            self.feedback.advance(elapsed);
        }
        tick
    }

    pub fn clear_request(&mut self) {
        self.requested_iq = 0.0;
    }

    pub fn set_request(&mut self, iq: f32) -> bool {
        if !iq.is_finite() {
            self.clear_request();
            return false;
        }
        self.requested_iq = iq;
        true
    }

    pub fn requested_iq(&self) -> f32 {
        self.requested_iq
    }

    pub fn record_delivery(&mut self, requested: f32, granted: f32, clip: f32) {
        self.feedback.record(requested, granted, clip);
    }

    pub fn take_feedback(&mut self) -> (f32, VelocityBlockage, VelocityBlockage) {
        self.feedback.take()
    }
}

#[cfg(test)]
#[path = "velocity_schedule_tests.rs"]
mod tests;

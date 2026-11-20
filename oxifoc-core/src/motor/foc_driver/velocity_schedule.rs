//! Cruise-loop service slots with monotonic microsecond timing and feedback.
//!
//! Only the requested q current is held. Every native current-loop step still
//! applies its current limits, fresh torque grant, trust gate and hard trip.

use crate::foc::fast_math::u64_to_f32;
use crate::foc::phase::MotionReferenceEpoch;
use crate::foc::velocity::VelocityBlockage;
use crate::units::Fraction;
use crate::units::{ConstZero, ElectricCurrent, Time, seconds};
use crate::units::{ElectricalAngularVelocity, Sign};

const PERIOD_US: u64 = 1_000;
// One endpoint is not a useful PI observation after four outer periods.
// Bound both native silence and the integration age; native deadline/fault
// handling remains separate.
const MAX_GAP_US: u64 = PERIOD_US * 4;

pub(super) struct ServiceSlots {
    pub slot: u8,
    frame: u8,
    length: u8,
}

impl ServiceSlots {
    pub fn new(dt: Time) -> Self {
        let length = (((seconds(0.001) / dt).fraction() + 0.5) as u8).max(2);
        Self {
            slot: length - 1,
            frame: u8::MAX,
            length,
        }
    }

    pub fn advance(&mut self) {
        self.slot += 1;
        if self.slot >= self.length {
            self.slot = 0;
            self.frame = self.frame.wrapping_add(1);
        }
    }

    #[cfg(any(test, feature = "runtime"))]
    pub fn deadman_due(&self) -> bool {
        self.next_slot() == self.length / 4
    }

    #[cfg(any(test, feature = "runtime"))]
    pub fn derating_due(&self) -> bool {
        self.next_slot() == self.length / 2 && self.frame & 7 == 0
    }

    #[cfg(any(test, feature = "runtime"))]
    fn next_slot(&self) -> u8 {
        if self.slot + 1 >= self.length {
            0
        } else {
            self.slot + 1
        }
    }

    pub fn length(&self) -> u8 {
        self.length
    }
}

/// Two native cycles at 15 kHz plus entry jitter. Older bounds are recomputed.
const MAX_AUTHORITY_AGE_US: u64 = 250;

/// Anti-windup bounds sampled one slot before the outer update. The actual
/// current grant remains fresh in every native cycle.
#[derive(Clone, Copy)]
struct PreparedAuthority {
    iq_min: ElectricCurrent,
    iq_max: ElectricCurrent,
    /// Speed direction and bus-modulation class: both select which probe
    /// is the braking or regenerating one, so the bounds jump when they change.
    direction: i8,
    bus_class: i8,
    prepared_us: u64,
}

fn direction(omega: ElectricalAngularVelocity) -> i8 {
    i8::from(omega.is_positive()) - i8::from(omega.is_negative())
}

pub(super) enum VelocityTick {
    Hold,
    /// Compute the authority bounds for the following outer update.
    Prepare,
    Update {
        dt: Time,
    },
    Reanchor,
}

/// Microseconds between two monotonic timestamps, as a time.
fn interval(now_us: u64, last_us: u64) -> Time {
    seconds(u64_to_f32(now_us.wrapping_sub(last_us)) * 1e-6)
}

#[derive(Default)]
struct OuterClock {
    previous: Option<u64>,
    updated: Option<u64>,
    deadline: u64,
    postponed: bool,
}

impl OuterClock {
    fn clear_integration(&mut self) {
        self.updated = None;
    }

    /// Elapsed-time backstop for the slot frame: one period after the last
    /// update or reanchor; it survives invalidation.
    /// A regular frame reaches it only in the scheduled slot, so it selects
    /// another slot only after missed native cycles.
    fn overdue(&self, now_us: u64) -> bool {
        now_us.wrapping_sub(self.deadline) < 1 << 63
    }
}

#[derive(Default)]
struct ActuatorFeedback {
    held_residual: ElectricCurrent,
    weighted_residual: ElectricCurrent,
    elapsed_us: u32,
    blockage: VelocityBlockage,
    voltage_clip: VelocityBlockage,
}

impl ActuatorFeedback {
    fn advance(&mut self, elapsed_us: u32) {
        self.weighted_residual += self.held_residual * elapsed_us as f32;
        self.elapsed_us += elapsed_us;
    }

    fn record(&mut self, requested: ElectricCurrent, granted: ElectricCurrent, clip: f32) {
        let residual = granted - requested;
        self.held_residual = if residual.is_finite() {
            residual
        } else {
            ElectricCurrent::ZERO
        };
        self.blockage.note_clip(clip);
        self.blockage.note_clip(requested - granted);
        self.voltage_clip.note_clip(clip);
    }

    fn take(&mut self) -> (ElectricCurrent, VelocityBlockage, VelocityBlockage) {
        let correction = if self.elapsed_us != 0 && self.weighted_residual.is_finite() {
            self.weighted_residual / self.elapsed_us as f32
        } else {
            ElectricCurrent::ZERO
        };
        self.weighted_residual = ElectricCurrent::ZERO;
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
    requested_iq: ElectricCurrent,
    defer_requested: bool,
    authority: Option<PreparedAuthority>,
    /// The prepare slot carried fresh drive activity; both stages move by one cycle.
    prepare_skipped: bool,
}

impl VelocitySchedule {
    /// Slot frame with the authority bounds one native cycle ahead of the PI.
    /// Fresh drive activity never shares a cycle with either stage, and the
    /// update is never later than slot 1.
    pub fn begin_pipelined_cycle(&mut self, now_us: u64, slot: u8, length: u8) -> VelocityTick {
        let elapsed = self
            .clock
            .previous
            .replace(now_us)
            .map_or(0, |previous| now_us.wrapping_sub(previous));
        if elapsed > MAX_GAP_US {
            self.clock.postponed = false;
            self.clock.deadline = now_us.wrapping_add(PERIOD_US);
            self.invalidate();
            return VelocityTick::Reanchor;
        }
        self.feedback.advance(elapsed as u32);
        if !self.clock.postponed {
            let overdue = slot != 0 && self.clock.overdue(now_us);
            if slot != 0 && !overdue {
                if slot + 1 == length {
                    self.prepare_skipped = self.defer_requested;
                    if !self.defer_requested {
                        return VelocityTick::Prepare;
                    }
                }
                return VelocityTick::Hold;
            }
            let prepare_skipped = core::mem::take(&mut self.prepare_skipped);
            // Scheduled or overdue: fresh drive activity still waits one cycle.
            if self.defer_requested {
                self.clock.postponed = true;
                return VelocityTick::Hold;
            }
            if !overdue && prepare_skipped && self.authority.is_none() {
                // The prepare slot carried a command. Shift both stages by one
                // cycle. Invalidated or cold bounds use the inline fallback.
                self.clock.postponed = true;
                return VelocityTick::Prepare;
            }
        }
        self.clock.postponed = false;
        self.clock.deadline = now_us.wrapping_add(PERIOD_US);
        let dt = self
            .clock
            .updated
            .replace(now_us)
            .map_or(Time::ZERO, |last| interval(now_us, last));
        VelocityTick::Update { dt }
    }

    pub fn store_authority(
        &mut self,
        iq_min: ElectricCurrent,
        iq_max: ElectricCurrent,
        omega: ElectricalAngularVelocity,
        bus_class: i8,
        now_us: u64,
    ) {
        self.authority = Some(PreparedAuthority {
            iq_min,
            iq_max,
            direction: direction(omega),
            bus_class,
            prepared_us: now_us,
        });
    }

    /// Bounds are single-use. A changed speed direction or bus-modulation
    /// class moves them discontinuously, so those bounds are recomputed.
    pub fn take_authority(
        &mut self,
        omega: ElectricalAngularVelocity,
        bus_class: i8,
        now_us: u64,
    ) -> Option<(ElectricCurrent, ElectricCurrent)> {
        let prepared = self.authority.take()?;
        (now_us.wrapping_sub(prepared.prepared_us) <= MAX_AUTHORITY_AGE_US
            && prepared.direction == direction(omega)
            && prepared.bus_class == bus_class)
            .then_some((prepared.iq_min, prepared.iq_max))
    }

    pub fn drop_authority(&mut self) {
        self.authority = None;
    }

    pub fn invalidate(&mut self) {
        self.epoch = None;
        self.requested_iq = ElectricCurrent::ZERO;
        self.authority = None;
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

    pub fn clear_request(&mut self) {
        self.requested_iq = ElectricCurrent::ZERO;
        self.authority = None;
    }

    pub fn defer_for_cycle(&mut self, requested: bool) {
        self.defer_requested = requested;
    }

    pub fn set_request(&mut self, iq: ElectricCurrent) -> bool {
        if !iq.is_finite() {
            self.clear_request();
            return false;
        }
        self.requested_iq = iq;
        true
    }

    pub fn requested_iq(&self) -> ElectricCurrent {
        self.requested_iq
    }

    pub fn record_delivery(
        &mut self,
        requested: ElectricCurrent,
        granted: ElectricCurrent,
        clip: f32,
    ) {
        self.feedback.record(requested, granted, clip);
    }

    pub fn take_feedback(&mut self) -> (ElectricCurrent, VelocityBlockage, VelocityBlockage) {
        self.feedback.take()
    }
}

#[cfg(test)]
#[path = "velocity_schedule_tests.rs"]
mod tests;

#[cfg(test)]
mod pipeline_tests {
    use super::*;
    use crate::units::{amps, erad_per_s};

    /// Command patterns: none, every cycle, only the prepare slot, only slot 0,
    /// both of them, and a pseudo-random 0.1 % stream.
    #[test]
    fn stages_never_share_a_cycle_with_commands_or_maintenance_and_updates_stay_bounded() {
        for hz in [15_000_u32, 20_000] {
            for pattern in 0..6 {
                let mut slots = ServiceSlots::new(seconds(1.0 / hz as f32));
                let mut schedule = VelocitySchedule::default();
                let length = slots.length();
                let (mut updates, mut fallbacks, mut prepares) = (0_u32, 0_u32, 0_u32);
                let mut random = 0x1234_5678_u32;
                let mut last_update = None;
                for cycle in 0..400_000_u64 {
                    let maintenance = slots.deadman_due() || slots.derating_due();
                    slots.advance();
                    let slot = slots.slot;
                    random = random.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    let command = match pattern {
                        0 => false,
                        1 => true,
                        2 => slot + 1 == length,
                        3 => slot == 0,
                        4 => slot == 0 || slot + 1 == length,
                        _ => random >> 22 == 0,
                    };
                    if command {
                        schedule.drop_authority();
                    }
                    schedule.defer_for_cycle(command);
                    let now = (cycle + 1) * 1_000_000 / u64::from(hz);
                    match schedule.begin_pipelined_cycle(now, slot, length) {
                        VelocityTick::Hold => {}
                        VelocityTick::Prepare => {
                            assert!(!command && !maintenance, "pattern {pattern} cycle {cycle}");
                            assert!(slot + 1 == length || slot == 0);
                            schedule.store_authority(
                                amps(-1.0),
                                amps(1.0),
                                erad_per_s(1.0),
                                1,
                                now,
                            );
                            prepares += 1;
                        }
                        VelocityTick::Update { .. } => {
                            assert!(slot <= 1 && !maintenance, "pattern {pattern} slot {slot}");
                            // Only a second consecutive command may accompany the bounded update.
                            assert!(!command || slot == 1);
                            if schedule.take_authority(erad_per_s(1.0), 1, now).is_none() {
                                fallbacks += 1;
                            }
                            if let Some(previous) = last_update {
                                let gap = cycle - previous;
                                assert!(
                                    (u64::from(length) - 1..=u64::from(length) + 1).contains(&gap)
                                );
                            }
                            last_update = Some(cycle);
                            updates += 1;
                        }
                        VelocityTick::Reanchor => panic!("regular cadence"),
                    }
                }
                assert!(updates >= 400_000 / u32::from(length) - 1);
                match pattern {
                    // Any command in slot 0 drops the prepared bounds: the
                    // deferred update in slot 1 recomputes them inline.
                    1 | 3 | 4 => assert_eq!(fallbacks, updates, "pattern {pattern}"),
                    // The first frame starts cold.
                    0 | 2 => assert!(fallbacks <= 1, "pattern {pattern}"),
                    _ => assert!(fallbacks * 100 < updates, "{fallbacks} of {updates}"),
                }
                assert!(prepares + fallbacks >= updates);
            }
        }
    }

    #[test]
    fn cold_or_overdue_updates_still_wait_one_cycle_for_fresh_drive_activity() {
        for first_slot in [0_u8, 7, 19] {
            let mut schedule = VelocitySchedule::default();
            schedule.defer_for_cycle(true);
            assert!(matches!(
                schedule.begin_pipelined_cycle(1_000, first_slot, 20),
                VelocityTick::Hold
            ));
            schedule.defer_for_cycle(true);
            assert!(matches!(
                schedule.begin_pipelined_cycle(1_050, (first_slot + 1) % 20, 20),
                VelocityTick::Update { .. }
            ));
        }
        // After three missed native cycles the backstop updates in slot 3.
        let mut schedule = VelocitySchedule::default();
        schedule.defer_for_cycle(false);
        assert!(matches!(
            schedule.begin_pipelined_cycle(0, 0, 20),
            VelocityTick::Update { .. }
        ));
        for slot in 1..3 {
            assert!(matches!(
                schedule.begin_pipelined_cycle(u64::from(slot) * 50, slot, 20),
                VelocityTick::Hold
            ));
        }
        assert!(matches!(
            schedule.begin_pipelined_cycle(1_000, 3, 20),
            VelocityTick::Update { .. }
        ));
    }

    #[test]
    fn prepared_bounds_are_single_use_and_expire_by_age_direction_or_invalidation() {
        let mut schedule = VelocitySchedule::default();
        schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(10.0), 1, 1_000);
        assert_eq!(
            schedule.take_authority(erad_per_s(11.0), 1, 1_100),
            Some((amps(-2.0), amps(3.0)))
        );
        assert_eq!(schedule.take_authority(erad_per_s(11.0), 1, 1_100), None);
        schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(10.0), 1, 1_000);
        assert_eq!(
            schedule.take_authority(erad_per_s(10.0), 1, 1_000 + MAX_AUTHORITY_AGE_US + 1),
            None
        );
        for (prepared, used) in [(10.0, -10.0), (10.0, 0.0), (0.0, 1.0), (-1.0, 1.0)] {
            schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(prepared), 1, 1_000);
            assert_eq!(schedule.take_authority(erad_per_s(used), 1, 1_050), None);
        }
        for (prepared, used) in [(1, -1), (1, 0), (0, 1), (-1, 1)] {
            schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(10.0), prepared, 1_000);
            assert_eq!(schedule.take_authority(erad_per_s(10.0), used, 1_050), None);
        }
        schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(0.0), 0, 1_000);
        assert_eq!(
            schedule.take_authority(erad_per_s(-0.0), 0, 1_050),
            Some((amps(-2.0), amps(3.0)))
        );
        for clear in 0..3 {
            schedule.store_authority(amps(-2.0), amps(3.0), erad_per_s(10.0), 1, 1_000);
            match clear {
                0 => schedule.invalidate(),
                1 => schedule.clear_request(),
                _ => schedule.drop_authority(),
            }
            assert_eq!(schedule.take_authority(erad_per_s(10.0), 1, 1_050), None);
        }
    }
}

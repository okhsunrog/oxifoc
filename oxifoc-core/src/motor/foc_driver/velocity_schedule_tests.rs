use super::*;
use crate::units::{Time, amps, seconds};

// 20 kHz native rate: 50 us per native cycle, 20 slots per 1 ms frame.
const LENGTH: u8 = 20;

fn slot_at(now_us: u64) -> u8 {
    (now_us / 50 % u64::from(LENGTH)) as u8
}

fn cycle(schedule: &mut VelocitySchedule, now_us: u64) -> VelocityTick {
    schedule.begin_pipelined_cycle(now_us, slot_at(now_us), LENGTH)
}

#[test]
fn postponed_cycle_retains_native_feedback_and_held_request() {
    let mut schedule = VelocitySchedule::default();
    assert!(matches!(
        cycle(&mut schedule, 0),
        VelocityTick::Update { .. }
    ));
    schedule.set_request(amps(3.0));
    schedule.record_delivery(amps(3.0), amps(1.0), 1.0);
    assert!(matches!(cycle(&mut schedule, 500), VelocityTick::Hold));
    schedule.defer_for_cycle(true);
    assert!(matches!(cycle(&mut schedule, 1_000), VelocityTick::Hold));
    assert_eq!(schedule.requested_iq(), amps(3.0));
    schedule.record_delivery(amps(3.0), amps(2.0), -1.0);
    schedule.defer_for_cycle(false);
    assert!(matches!(
        cycle(&mut schedule, 1_050),
        VelocityTick::Update { .. }
    ));
    let (residual, blocked, voltage) = schedule.take_feedback();
    assert_near!(
        residual,
        amps((-2.0 * 1_000.0 - 50.0) / 1_050.0),
        amps(1e-6)
    );
    assert!(blocked.positive() && blocked.negative());
    assert!(voltage.positive() && voltage.negative());
}

#[test]
fn feedback_retains_a_released_limit_and_both_voltage_clip_directions() {
    let mut schedule = VelocitySchedule::default();
    cycle(&mut schedule, 0);
    schedule.record_delivery(amps(2.0), amps(0.0), 1.0);
    cycle(&mut schedule, 500);
    schedule.record_delivery(amps(2.0), amps(2.0), -1.0);
    assert!(matches!(
        cycle(&mut schedule, 1_000),
        VelocityTick::Update { .. }
    ));
    let (residual, blocked, voltage) = schedule.take_feedback();
    assert_eq!(residual, amps(-1.0));
    assert!(blocked.positive() && blocked.negative());
    assert!(voltage.positive() && voltage.negative());
    let (residual, blocked, voltage) = schedule.take_feedback();
    assert_eq!(residual, amps(0.0));
    assert!(!blocked.positive() && !blocked.negative());
    assert!(!voltage.positive() && !voltage.negative());
    schedule.record_delivery(amps(-2.0), amps(-1.0), 0.0);
    cycle(&mut schedule, 1_500);
    schedule.rearm(MotionReferenceEpoch::default());
    assert_eq!(
        schedule.take_feedback().0,
        amps(0.0),
        "new reference owns no old residual"
    );
    assert!(!schedule.set_request(amps(f32::NAN)));
    assert_eq!(schedule.requested_iq(), amps(0.0));
}

#[test]
fn deferred_first_update_has_zero_dt_and_a_native_gap_reanchors() {
    let mut schedule = VelocitySchedule::default();
    schedule.defer_for_cycle(true);
    assert!(matches!(cycle(&mut schedule, 0), VelocityTick::Hold));
    let VelocityTick::Update { dt } = cycle(&mut schedule, 50) else {
        panic!("a postponed update runs on the next native cycle")
    };
    assert_eq!(dt, Time::ZERO);
    schedule.defer_for_cycle(false);
    // More than MAX_GAP_US without a native cycle: no dt is invented.
    assert!(matches!(
        cycle(&mut schedule, 50 + MAX_GAP_US + 50),
        VelocityTick::Reanchor
    ));
}

#[test]
fn wrapped_timestamps_keep_the_grid_and_the_actual_interval() {
    let mut schedule = VelocitySchedule::default();
    let start = u64::MAX - 500;
    assert!(matches!(
        schedule.begin_pipelined_cycle(start, 0, LENGTH),
        VelocityTick::Update { .. }
    ));
    assert!(matches!(
        schedule.begin_pipelined_cycle(0, 10, LENGTH),
        VelocityTick::Hold
    ));
    let VelocityTick::Update { dt } = schedule.begin_pipelined_cycle(499, 0, LENGTH) else {
        panic!("the next frame updates across the wrap")
    };
    assert_near!(dt, seconds(0.001), seconds(1e-9));
}

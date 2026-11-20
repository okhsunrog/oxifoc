use super::*;
use crate::units::{ConstZero, Time, amps, seconds};

#[test]
fn first_request_can_wait_one_native_cycle_but_keeps_zero_initial_pi_dt() {
    let mut clock = OuterClock::default();
    assert!(matches!(
        clock.advance_with_defer(0, true).1,
        VelocityTick::Hold
    ));
    let VelocityTick::Update { dt } = clock.advance_with_defer(67, true).1 else {
        panic!()
    };
    assert_eq!(dt, seconds(0.0));
    assert_eq!(clock.deadline, 1_000);
    let VelocityTick::Update { dt } = clock.advance_with_defer(1_000, false).1 else {
        panic!()
    };
    assert!((dt - seconds(0.000933)).abs() < seconds(1e-9));
}

#[test]
fn postponement_keeps_actual_dt_and_periodic_grid() {
    let mut clock = OuterClock::default();
    assert!(matches!(
        clock.advance_with_defer(0, false).1,
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));
    assert!(matches!(
        clock.advance_with_defer(950, false).1,
        VelocityTick::Hold
    ));
    assert!(matches!(
        clock.advance_with_defer(1_000, true).1,
        VelocityTick::Hold
    ));
    assert_eq!(clock.deadline, 1_000);
    let VelocityTick::Update { dt } = clock.advance_with_defer(1_067, false).1 else {
        panic!()
    };
    assert!((dt - seconds(0.001067)).abs() < seconds(1e-9));
    assert_eq!(clock.deadline, 2_000);
    let VelocityTick::Update { dt } = clock.advance_with_defer(2_000, false).1 else {
        panic!()
    };
    assert!((dt - seconds(0.000933)).abs() < seconds(1e-9));
}

#[test]
fn command_flood_cannot_starve_a_postponed_update_or_disable_gap_reanchor() {
    let mut clock = OuterClock::default();
    clock.advance_with_defer(0, false);
    assert!(matches!(
        clock.advance_with_defer(1_000, true).1,
        VelocityTick::Hold
    ));
    assert!(matches!(
        clock.advance_with_defer(1_067, true).1,
        VelocityTick::Update { .. }
    ));
    assert!(matches!(
        clock.advance_with_defer(2_000, true).1,
        VelocityTick::Hold
    ));
    assert!(matches!(
        clock.advance_with_defer(7_000, true).1,
        VelocityTick::Reanchor
    ));
    assert!(!clock.postponed);
}

#[test]
fn postponed_cycle_retains_native_feedback_and_held_request() {
    let mut schedule = VelocitySchedule::default();
    schedule.begin_cycle(0);
    schedule.set_request(amps(3.0));
    schedule.record_delivery(amps(3.0), amps(1.0), 1.0);
    schedule.begin_cycle(500);
    schedule.defer_for_cycle(true);
    assert!(matches!(schedule.begin_cycle(1_000), VelocityTick::Hold));
    assert_eq!(schedule.requested_iq(), amps(3.0));
    schedule.record_delivery(amps(3.0), amps(2.0), -1.0);
    schedule.defer_for_cycle(false);
    assert!(matches!(
        schedule.begin_cycle(1_067),
        VelocityTick::Update { .. }
    ));
    let (residual, blocked, voltage) = schedule.take_feedback();
    assert!((residual - amps((-2.0 * 1_000.0 - 67.0) / 1_067.0)).abs() < amps(1e-6));
    assert!(blocked.positive() && blocked.negative());
    assert!(voltage.positive() && voltage.negative());
}

#[test]
fn clock_keeps_its_grid_and_uses_actual_elapsed_time_at_different_native_rates() {
    for native_hz in [1_000_u64, 5_000, 15_000, 20_000] {
        let mut clock = OuterClock::default();
        let mut updates = 0;
        let mut integrated = 0.0_f64;
        for step in 0..=native_hz {
            let now = step * 1_000_000 / native_hz;
            match clock.advance(now).1 {
                VelocityTick::Update { dt } => {
                    updates += 1;
                    integrated += f64::from(dt.value);
                }
                VelocityTick::Hold => {}
                VelocityTick::Reanchor => panic!("valid native cadence"),
                #[cfg(feature = "pi-bounds-ahead")]
                VelocityTick::Prepare => panic!("the microsecond clock has no prepare stage"),
            }
        }
        assert_eq!(updates, 1_001);
        assert!((integrated - 1.0).abs() < 1e-6, "{integrated}");
    }
}

#[test]
fn clock_skips_backlog_and_reanchors_bad_time_without_inventing_dt() {
    let mut clock = OuterClock::default();
    assert!(matches!(
        clock.advance(0).1,
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));
    assert!(matches!(clock.advance(50).1, VelocityTick::Hold));
    let VelocityTick::Update { dt } = clock.advance(1_100).1 else {
        panic!()
    };
    assert!((dt - seconds(0.0011)).abs() < seconds(1e-9));
    assert!(matches!(clock.advance(1_100).1, VelocityTick::Hold));
    assert!(matches!(clock.advance(1_999).1, VelocityTick::Hold));
    let VelocityTick::Update { dt } = clock.advance(2_000).1 else {
        panic!()
    };
    assert!((dt - seconds(0.0009)).abs() < seconds(1e-9));
    assert!(matches!(clock.advance(1_999).1, VelocityTick::Reanchor));
    assert!(matches!(
        clock.advance(2_999).1,
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));
    assert!(matches!(clock.advance(10_000).1, VelocityTick::Reanchor));
    assert!(matches!(
        clock.advance(11_000).1,
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));

    let mut wrapped = OuterClock::default();
    wrapped.advance(u64::MAX - 500);
    assert!(matches!(wrapped.advance(0).1, VelocityTick::Hold));
    let VelocityTick::Update { dt } = wrapped.advance(499).1 else {
        panic!()
    };
    assert!((dt - seconds(0.001)).abs() < seconds(1e-9));
}

#[test]
fn repeated_invalidation_cannot_overclock_the_pi() {
    let mut schedule = VelocitySchedule::default();
    let epoch = MotionReferenceEpoch::default();
    schedule.rearm(epoch);
    assert!(matches!(
        schedule.begin_cycle(0),
        VelocityTick::Update { .. }
    ));
    schedule.set_request(amps(2.0));
    for now in (50..1_000).step_by(50) {
        schedule.rearm(epoch);
        assert!(matches!(schedule.begin_cycle(now), VelocityTick::Hold));
        assert_eq!(schedule.requested_iq(), amps(0.0));
    }
    assert!(matches!(
        schedule.begin_cycle(1_000),
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));
}

#[test]
fn clock_bounds_integration_age_and_keeps_short_late_grid_intervals() {
    let mut clock = OuterClock::default();
    clock.advance(0);
    assert!(matches!(clock.advance(950).1, VelocityTick::Hold));
    // The native gap is only 4 ms, but this endpoint is too old for the PI.
    assert!(matches!(clock.advance(4_950).1, VelocityTick::Reanchor));
    assert!(matches!(
        clock.advance(5_950).1,
        VelocityTick::Update { dt } if dt == Time::ZERO
    ));

    let mut clock = OuterClock::default();
    clock.advance(0);
    assert!(matches!(
        clock.advance(1_950).1,
        VelocityTick::Update { .. }
    ));
    let VelocityTick::Update { dt } = clock.advance(2_000).1 else {
        panic!()
    };
    assert!((dt - seconds(0.00005)).abs() < seconds(1e-10));
    assert!(matches!(clock.advance(2_050).1, VelocityTick::Hold));
}

#[test]
fn feedback_retains_a_released_limit_and_both_voltage_clip_directions() {
    let mut schedule = VelocitySchedule::default();
    schedule.begin_cycle(0);
    schedule.record_delivery(amps(2.0), amps(0.0), 1.0);
    schedule.begin_cycle(500);
    schedule.record_delivery(amps(2.0), amps(2.0), -1.0);
    assert!(matches!(
        schedule.begin_cycle(1_000),
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
    schedule.begin_cycle(1_500);
    schedule.rearm(MotionReferenceEpoch::default());
    assert_eq!(
        schedule.take_feedback().0,
        amps(0.0),
        "new reference owns no old residual"
    );
    assert!(!schedule.set_request(amps(f32::NAN)));
    assert_eq!(schedule.requested_iq(), amps(0.0));
}

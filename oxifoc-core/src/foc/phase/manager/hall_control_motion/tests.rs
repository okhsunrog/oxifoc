use super::*;
use crate::foc::hall_sensor::Direction;
use crate::foc::sensors::AngleSample;
use crate::foc::wrap_angle;

fn sample(angle: f32, raw_omega: f32) -> ControlAngleSample {
    ControlAngleSample {
        sample: AngleSample {
            angle: wrap_angle(angle),
            omega: raw_omega,
            direction: Direction::Clockwise,
        },
        reference_epoch: 0,
    }
}

fn observe(motion: &mut HallControlMotion, angle: f32, raw: f32, ticks: u64, applied: f32) -> f32 {
    motion
        .observe(sample(angle, raw), ticks, applied)
        .unwrap()
        .velocity
}

#[test]
fn angle_derivative_wraps_both_ways_and_ignores_raw_speed_steps() {
    for direction in [-1.0, 1.0] {
        let mut motion = HallControlMotion::default();
        let mut angle = 0.001;
        let mut velocity = observe(&mut motion, angle, 100.0, 0, 700.0);
        assert_eq!(velocity, 0.0, "cold start must not import raw speed");
        for step in 1..=2000 {
            angle = wrap_angle(angle + direction * 100.0 * 0.00005);
            let raw = if step % 200 < 100 { 50.0 } else { 500.0 };
            velocity = observe(&mut motion, angle, raw, step * 50, velocity);
        }
        assert!((velocity - direction * 100.0).abs() < 0.02, "{velocity}");
    }
}

#[test]
fn pole_uses_elapsed_ticks_across_active_and_decimated_cadences() {
    let mut responses = [0.0; 4];
    for (index, period_us) in [50_u64, 200, 1000, 4000].into_iter().enumerate() {
        let mut motion = HallControlMotion::default();
        let mut angle = 0.0;
        let mut velocity = observe(&mut motion, angle, 10.0, 0, 0.0);
        #[allow(clippy::cast_precision_loss)]
        let dt = period_us as f32 * 1.0e-6;
        let gain = VELOCITY_TAU_S / (VELOCITY_TAU_S + dt);
        let mut residual = 1.0;
        for step in 1..=20_000 / period_us {
            angle += 10.0 * dt;
            velocity = observe(&mut motion, angle, 10.0, step * period_us, velocity);
            residual *= gain;
        }
        assert!((velocity - 10.0 * (1.0 - residual)).abs() < 0.001);
        responses[index] = velocity;
    }
    assert!(responses.iter().all(|v| (*v - responses[0]).abs() < 0.4));

    // Stopped -> active: the first active visit can be only one cycle after
    // a decimated visit. Its elapsed time is 50 us, not the previous 200 us.
    let mut motion = HallControlMotion::default();
    observe(&mut motion, 0.0, 100.0, 0, 0.0);
    let stopped = observe(&mut motion, 0.02, 100.0, 200, 0.0);
    let active = observe(&mut motion, 0.025, 100.0, 250, stopped);
    let expected = (VELOCITY_TAU_S * stopped + 0.005) / (VELOCITY_TAU_S + 0.00005);
    assert!((active - expected).abs() < 0.0001);
}

#[test]
fn angle_step_is_filtered_and_reversal_is_continuous() {
    let mut motion = HallControlMotion::default();
    observe(&mut motion, 0.0, 100.0, 0, 0.0);
    let jump = observe(&mut motion, 0.1, 100.0, 50, 0.0);
    assert!((jump - 20.0).abs() < 0.001);
    let next = observe(&mut motion, 0.1, 100.0, 100, jump);
    assert!(next < jump && next > 19.0);
    let mut angle = 0.1;
    let mut velocity = next;
    for step in 3..=2000 {
        angle = wrap_angle(angle - 100.0 * 0.00005);
        let previous = velocity;
        velocity = observe(&mut motion, angle, -100.0, step * 50, previous);
        assert!((velocity - previous).abs() < 1.3);
    }
    assert!((velocity + 100.0).abs() < 0.02);
}

#[test]
fn unavailable_and_source_reentry_seed_the_applied_speed_but_reset_is_cold() {
    let mut motion = HallControlMotion::default();
    observe(&mut motion, 0.0, 100.0, 0, 0.0);
    observe(&mut motion, 0.005, 100.0, 50, 0.0);
    motion.unavailable();
    assert_eq!(observe(&mut motion, 2.0, 300.0, 100, 52.0), 52.0);
    motion.source_changed();
    assert_eq!(observe(&mut motion, 4.0, 300.0, 150, -73.0), -73.0);
    motion.reset();
    motion.unavailable();
    motion.source_changed();
    assert_eq!(observe(&mut motion, 1.0, 300.0, 200, 100.0), 0.0);
    motion.source_changed();
    assert_eq!(observe(&mut motion, 1.0, 300.0, 250, f32::NAN), 0.0);
}

#[test]
fn duplicate_backward_long_gap_and_ambiguous_angles_reanchor_without_speed_inference() {
    let mut motion = HallControlMotion::default();
    observe(&mut motion, 0.0, 100.0, 100, 0.0);
    assert_eq!(observe(&mut motion, 1.0, 100.0, 100, 55.0), 0.0);
    assert_eq!(observe(&mut motion, 2.0, 100.0, 10, 45.0), 45.0);
    let next = observe(&mut motion, 2.005, 100.0, 60, 45.0);
    assert!((next - 45.55).abs() < 0.001);
    assert_eq!(observe(&mut motion, 0.5, 100.0, 30_060, 12.0), 12.0);
    let next = observe(&mut motion, 0.505, 100.0, 30_110, 12.0);
    assert!((next - 12.88).abs() < 0.001);
    // A gap inside four tau can still contain more than a half-turn.
    assert_eq!(observe(&mut motion, 1.0, 400.0, 40_110, 25.0), 25.0);
    let next = observe(&mut motion, 1.005, 100.0, 40_160, 25.0);
    assert!((next - 25.75).abs() < 0.001);
}

#[test]
fn small_u64_wrap_is_elapsed_time_and_timebase_change_discards_history() {
    let mut motion = HallControlMotion::default();
    observe(&mut motion, 0.0, 100.0, u64::MAX - 25, 0.0);
    let wrapped = observe(&mut motion, 0.005, 100.0, 24, 0.0);
    assert!((wrapped - 1.0).abs() < 0.001);
    motion.set_ticks_per_sec(2_000_000);
    assert_eq!(observe(&mut motion, 2.0, 100.0, 1000, 700.0), 0.0);
    let velocity = observe(&mut motion, 2.005, 100.0, 1100, 0.0);
    assert!((velocity - 1.0).abs() < 0.001);
}

#[test]
fn invalid_sensor_frames_do_not_poison_control_speed() {
    for invalid in [sample(f32::NAN, 1.0), sample(1.0, f32::INFINITY)] {
        let mut motion = HallControlMotion::default();
        observe(&mut motion, 0.0, 100.0, 0, 0.0);
        assert!(motion.observe(invalid, 50, 20.0).is_none());
        assert_eq!(observe(&mut motion, 2.0, 100.0, 100, 20.0), 20.0);
    }
}

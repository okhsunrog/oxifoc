use super::*;

// Drive the sensor from an independent continuous rotor trajectory. Hall
// thresholds are half-way between nominal centroids, including the 2π wrap.
fn run_path(path: impl Fn(f32) -> f32, seconds: f32, interp: f32) -> (f32, f32, f32) {
    let mut hall = HallSensor::new(1_000_000);
    hall.set_interp_min_erpm(interp);
    const SEQUENCE: [u8; 6] = [1, 3, 2, 6, 4, 5];
    let mut last = 0;
    let mut integrated = 0.0;
    let mut old_integrated = 0.0;
    let mut final_speed = 0.0;
    for step in 0..=(seconds * 20_000.0) as u32 {
        let t = step as f32 / 20_000.0;
        let sector = ((path(t) / (TAU / 6.0) + 0.5).floor() as i32).rem_euclid(6) as usize;
        let raw = SEQUENCE[sector];
        if raw != last {
            hall.update(raw, u64::from(step) * 50);
            last = raw;
        }
        final_speed = hall
            .sample_for_control(u64::from(step) * 50)
            .unwrap()
            .sample
            .omega;
        integrated += final_speed * 0.00005;
        old_integrated += hall.sample_at(u64::from(step) * 50).unwrap().omega * 0.00005;
    }
    (integrated, old_integrated, final_speed)
}

#[test]
fn rotor_travel_stops_accumulating_after_edges_stop() {
    let path = |t: f32| 80.0 * t.min(0.5);
    let (short, _, _) = run_path(path, 1.0, 500.0);
    let (long, old_long, speed) = run_path(path, 8.0, 500.0);
    assert!((long - short).abs() < 0.002, "{short} -> {long}");
    assert!((long - 40.0).abs() < TAU / 6.0, "{long}");
    assert!(speed.abs() < 0.001, "{speed}");
    assert!(
        old_long - long > 3.0,
        "old estimator must reproduce the drift"
    );
}

#[test]
fn repeated_asymmetric_rocking_has_no_fictitious_net_rotation() {
    // A small oscillation traverses two boundaries, dwelling longer in one
    // direction. It ends at the same angle; then allow the finite filter tail.
    let path = |t: f32| 0.3 + 1.4 * (TAU * t.min(6.0) + 0.5 * (TAU * t.min(6.0)).sin()).sin();
    for sign in [-1.0, 1.0] {
        let (travel, _, speed) = run_path(|t| sign * path(t), 6.5, 500.0);
        assert!(travel.abs() < TAU / 6.0, "net travel {travel}");
        assert!(speed.abs() < 0.01, "final speed {speed}");
    }
}

#[test]
fn physical_speed_is_independent_of_commutation_interpolation_threshold() {
    let path = |t: f32| 40.0 * t + 0.4 * (TAU * 3.0 * t).sin();
    let a = run_path(path, 3.0, 0.0);
    let b = run_path(path, 3.0, 500.0);
    let c = run_path(path, 3.0, 100_000.0);
    assert_eq!(a, b);
    assert_eq!(b, c);
}

#[test]
fn constant_motion_tracks_both_directions_without_distance_bias() {
    for velocity in [-1500.0, -120.0, -40.0, 40.0, 120.0, 1500.0] {
        let (travel, _, speed) = run_path(|t| velocity * t, 2.0, 500.0);
        assert!(
            (speed - velocity).abs() < velocity.abs() * 0.015,
            "{velocity}: {speed}"
        );
        // Startup position is unknown within a sector; final filtering lags
        // physical travel by tau*speed. Neither error grows with run duration.
        assert!(
            (travel + 0.020 * speed - 2.0 * velocity).abs() < 2.0 * TAU / 6.0,
            "{velocity}: {travel}"
        );
    }
}

#[test]
fn travel_filter_retains_multiple_captures_and_ignores_duplicate_samples() {
    let mut motion = RotorMotion::default();
    motion.cross_boundary(0.0);
    assert_eq!(motion.sample(100, 1e-6, 0.0, 0.0), 0.0);
    for boundary in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.5] {
        motion.cross_boundary(boundary);
    }
    assert_eq!(motion.sample(100, 1e-6, 0.0, 0.0), 0.0);
    assert_eq!(motion.sample(99, 1e-6, 0.0, 0.0), 0.0);
    let speed = motion.sample(1100, 1e-6, 0.0, 0.0);
    assert!((speed * 0.021 - (TAU + 0.5)).abs() < 1e-5);
    assert_eq!(motion.sample(1100, 1e-6, 0.0, 0.0), speed);
    // A gap outside the control cadence reanchors, not an aliased revolution.
    assert_eq!(motion.sample(1_000_000, 1e-6, 0.0, 0.0), 0.0);
}

#[test]
fn measured_unequal_sectors_keep_speed_and_travel_in_both_directions() {
    use crate::foc::hall_calibration::{HallCalibrationResult, HallTransitionGeometry};
    let sequence = [1, 3, 2, 6, 4, 5];
    let boundaries = [0.0, 0.55, 2.15, 3.15, 3.75, 5.15, TAU];
    let mut entries = [0.0; 8];
    let mut centers = [0.0; 8];
    let mut next = [0; 8];
    for (i, &raw) in sequence.iter().enumerate() {
        entries[raw] = boundaries[i];
        centers[raw] = 0.5 * (boundaries[i] + boundaries[i + 1]);
        next[raw] = sequence[(i + 1) % 6] as u8;
    }
    let calibration = HallCalibrationResult {
        angles: centers,
        valid: [false, true, true, true, true, true, true, false],
        valid_count: 6,
        geometry: HallTransitionGeometry::try_from_raw_parts(entries, next, centers),
    };
    assert!(calibration.geometry.is_some());
    for velocity in [-120.0, -40.0, 40.0, 120.0] {
        let mut hall = HallSensor::new(1_000_000);
        assert!(hall.apply_calibration(&calibration));
        let mut previous = 0;
        let mut travel = 0.0;
        let mut speed = 0.0;
        for step in 0..=60_000_u64 {
            let angle = wrap_angle(0.25 + velocity * step as f32 * 0.00005);
            let sector = boundaries
                .windows(2)
                .position(|p| angle >= p[0] && angle < p[1])
                .unwrap();
            let raw = sequence[sector] as u8;
            if raw != previous {
                hall.update(raw, step * 50);
                previous = raw;
            }
            speed = hall.sample_for_control(step * 50).unwrap().sample.omega;
            travel += speed * 0.00005;
        }
        assert!(
            (speed - velocity).abs() < velocity.abs() * 0.01,
            "{velocity}: {speed}"
        );
        assert!(
            (travel + 0.020 * speed - 3.0 * velocity).abs() < 1.7,
            "{velocity}: {travel}"
        );
    }
}

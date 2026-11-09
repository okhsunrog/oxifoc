use super::*;
use crate::foc::hall_calibration::HallTransitionGeometry;

const RING: [u8; 6] = [3, 2, 6, 4, 5, 1];
const ENTRIES_DEG: [f32; 8] = [0.0, 300.0, 70.0, 350.0, 200.0, 240.0, 110.0, 0.0];
const CENTERS_DEG: [f32; 8] = [0.0, 325.0, 90.0, 30.0, 220.0, 270.0, 155.0, 0.0];

fn measured_hall() -> HallSensor {
    let centers = CENTERS_DEG.map(f32::to_radians);
    let mut next = [0; 8];
    for (i, raw) in RING.into_iter().enumerate() {
        next[usize::from(raw)] = RING[(i + 1) % 6];
    }
    let geometry =
        HallTransitionGeometry::try_from_raw_parts(ENTRIES_DEG.map(f32::to_radians), next, centers)
            .unwrap();
    let mut hall = HallSensor::new(1_000_000);
    assert!(
        hall.apply_calibration(&crate::foc::hall_calibration::HallCalibrationResult {
            angles: centers,
            valid: [false, true, true, true, true, true, true, false],
            valid_count: 6,
            geometry: Some(geometry),
        })
    );
    hall
}

#[test]
fn prepared_geometry_validates_the_current_frame_and_legacy_replaces_it() {
    let mut hall = measured_hall();
    hall.update(3, 100);
    let result = crate::foc::hall_calibration::HallCalibrationResult {
        angles: CENTERS_DEG.map(f32::to_radians),
        valid: [false, true, true, true, true, true, true, false],
        valid_count: 6,
        geometry: hall.geometry,
    };
    let mut inconsistent = result;
    inconsistent.angles[3] = 180_f32.to_radians();
    assert!(PreparedHallCalibration::from_result(&inconsistent).is_none());

    let prepared = PreparedHallCalibration::from_result(&result).unwrap();
    hall.geometry_resync = true;
    hall.install_prepared_calibration(&prepared);
    assert!(!hall.geometry_resync);
    assert_eq!(
        hall.current_sector_width.to_bits(),
        result.geometry.unwrap().sector_width(3).unwrap().to_bits()
    );

    let mut legacy = result;
    legacy.geometry = None;
    assert!(hall.apply_calibration(&legacy));
    assert!(hall.geometry.is_none());
    assert_eq!(
        hall.current_sector_width.to_bits(),
        hall.angle_per_state.to_bits()
    );
}

#[cfg(feature = "storage")]
#[test]
fn geometry_config_and_detection_install_the_same_reference() {
    let mut detected = measured_hall();
    detected.update(3, 100);
    let result = crate::foc::hall_calibration::HallCalibrationResult {
        angles: CENTERS_DEG.map(f32::to_radians),
        valid: [false, true, true, true, true, true, true, false],
        valid_count: 6,
        geometry: detected.geometry,
    };
    let mut from_config = HallSensor::new(1_000_000);
    from_config.update(3, 100);
    let config = crate::storage::HallCalibrationConfig::from(&result);
    from_config
        .install_prepared_calibration(&PreparedHallCalibration::from_config(&config).unwrap());
    assert_eq!(
        from_config.current_sector_width.to_bits(),
        detected.current_sector_width.to_bits()
    );
    for (state, ticks) in [(2, 8_100), (6, 12_100), (4, 21_100)] {
        from_config.update(state, ticks);
        detected.update(state, ticks);
        let actual = from_config.sample_at(ticks + 50).unwrap();
        let expected = detected.sample_at(ticks + 50).unwrap();
        assert_eq!(actual.angle.to_bits(), expected.angle.to_bits());
        assert_eq!(actual.omega.to_bits(), expected.omega.to_bits());
    }
    let legacy_config = crate::storage::HallCalibrationConfig {
        geometry: None,
        ..config
    };
    from_config.install_prepared_calibration(
        &PreparedHallCalibration::from_config(&legacy_config).unwrap(),
    );
    assert!(from_config.geometry.is_none());
}

fn advance_to_measured_motion(hall: &mut HallSensor) {
    hall.update(3, 0);
    hall.update(2, 8_000);
    assert_eq!(
        hall.electrical_velocity(),
        0.0,
        "boot interval was not a full sector"
    );
    hall.update(6, 12_000);
    assert!((hall.electrical_velocity() - 40_f32.to_radians() / 0.004).abs() < 0.001);
}

#[test]
fn unequal_sector_rotation_preserves_threshold_and_continuous_interpolation() {
    for sign in [1.0_f32, -1.0] {
        for erpm in [499.0_f32, 501.0, 1909.8593, 4000.0] {
            let omega = erpm * TAU / 60.0;
            let mut hall = measured_hall();
            let geometry = hall.geometry.unwrap();
            let start = if sign > 0.0 { 350_f32 } else { 70.0 }.to_radians();
            let mut events = Vec::new();
            let mut distance = 0.0;
            let mut index = 0_usize;
            for _ in 0..36 {
                let raw = RING[index];
                events.push(((distance / omega * 1.0e6_f32).round() as u64, raw));
                distance += geometry.sector_width(raw).unwrap();
                index = if sign > 0.0 {
                    (index + 1) % 6
                } else {
                    (index + 5) % 6
                };
            }
            let end = events.last().unwrap().0;
            let warmup = (2.0 * TAU / omega * 1.0e6) as u64;
            let mut edge = 0;
            let mut previous = None;
            for now in (0..end).step_by(50) {
                while edge < events.len() && events[edge].0 <= now {
                    hall.update(events[edge].1, events[edge].0);
                    edge += 1;
                }
                let raw = hall.sample_at(now).unwrap();
                let control = hall.sample_at_mut(now).unwrap();
                if now > warmup {
                    if erpm < 500.0 {
                        assert_eq!(
                            raw.angle.to_bits(),
                            hall.calib.angle_for_state(hall.raw_state).to_bits(),
                            "constant499eRPM must not falsely reenter interpolation"
                        );
                        if let Some(previous) = previous {
                            assert!(
                                angle_difference(control.angle, previous).abs()
                                    <= hall.interp_min_rad_s * 50.0e-6 * hall.rate_limit_factor
                                        + 1.0e-6
                            );
                        }
                        previous = Some(control.angle);
                        continue;
                    }
                    let physical = wrap_angle(start + sign * omega * now as f32 * 1.0e-6);
                    assert!(
                        angle_difference(raw.angle, physical).abs() < 0.003,
                        "raw error {} at{erpm}eRPM sign{sign}",
                        angle_difference(raw.angle, physical)
                    );
                    assert!(angle_difference(control.angle, physical).abs() < 0.003);
                    assert!((raw.omega - sign * omega).abs() < omega * 0.002);
                    if let Some(previous) = previous {
                        let progress = angle_difference(control.angle, previous) * sign;
                        assert!(
                            progress > omega * 50.0e-6 * 0.5,
                            "artificial flat/catchup at{erpm}eRPM sign{sign}"
                        );
                    }
                    previous = Some(control.angle);
                }
            }
        }
    }
}

#[test]
fn below_threshold_centroids_remain_rate_limited_in_both_directions() {
    for order in [[3, 2, 6, 4, 5, 1, 3], [3, 1, 5, 4, 6, 2, 3]] {
        let mut hall = measured_hall();
        hall.update(order[0], 0);
        hall.sample_at_mut(0).unwrap();
        for (i, raw) in order.into_iter().skip(1).enumerate() {
            let now = (i as u64 + 1) * 100_000;
            // Compare the first real50µs allowance across a sector edge.
            let previous = hall.sample_at_mut(now - 50).unwrap().angle;
            hall.update(raw, now);
            let sample = hall.sample_at_mut(now).unwrap();
            assert!(
                angle_difference(sample.angle, previous).abs()
                    <= hall.interp_min_rad_s * 50.0e-6 * hall.rate_limit_factor + 1.0e-6
            );
            assert_eq!(
                hall.sample_at(now).unwrap().angle.to_bits(),
                hall.calib.angle_for_state(raw).to_bits()
            );
        }
    }
}

#[test]
fn current_measured_width_controls_stop_decay_and_stale_deadline() {
    let mut hall = measured_hall();
    advance_to_measured_motion(&mut hall);
    let speed = hall.electrical_velocity();
    // Currentraw6 is90°, previousraw2 was40°; neither nominal60° nor the
    // previous traversed width is the correct bound for waiting onraw6's exit.
    let width = 90_f32.to_radians();
    assert!((hall.current_sector_width - width).abs() < 1.0e-6);
    let within = hall.sample_at(19_000).unwrap();
    assert!((within.omega - speed).abs() < 1.0e-5);
    let late = hall.sample_at(24_000).unwrap();
    assert!((late.omega - width / 0.012).abs() < 0.001);
    assert!(!hall.is_stale_at_speed(24_000, speed));
    assert!(hall.is_stale_at_speed(27_000, speed));
    assert!(hall.sample_at(1_000_000).unwrap().omega < 2.0);
}

#[test]
fn reversal_uses_the_sector_just_left_for_bound_and_the_new_sector_for_decay() {
    let mut hall = measured_hall();
    advance_to_measured_motion(&mut hall);
    let previous = hall.electrical_velocity();
    hall.update(2, 12_500); // turn back through the same110° boundary
    assert_eq!(hall.electrical_velocity().to_bits(), (-previous).to_bits());
    assert!((hall.current_sector_width - 40_f32.to_radians()).abs() < 1.0e-6);
    assert!((hall.angle - 110_f32.to_radians()).abs() < 1.0e-6);
    hall.update(6, 13_000);
    assert_eq!(hall.electrical_velocity().to_bits(), previous.to_bits());
    assert!((hall.current_sector_width - 90_f32.to_radians()).abs() < 1.0e-6);
}

#[test]
fn skipped_transition_withholds_frame_then_resynchronizes_on_known_adjacent_edges() {
    for (skipped, anchor, next, boundary_deg, signed_width) in
        [(1, 3, 2, 350_f32, 80_f32), (5, 4, 6, 240_f32, -40_f32)]
    {
        let mut hall = measured_hall();
        advance_to_measured_motion(&mut hall);
        let old_angle = hall.angle;
        let old_speed = hall.electrical_velocity();
        let old_ticks = hall.last_edge_ticks;
        assert_eq!(hall.update(skipped, 13_000), None);
        assert_eq!(hall.angle.to_bits(), old_angle.to_bits());
        assert_eq!(hall.last_edge_ticks, old_ticks);
        assert_eq!(hall.electrical_velocity().to_bits(), old_speed.to_bits());
        assert!(hall.snapshot_at(13_100).is_none());
        assert!(hall.sample_at_mut(13_100).is_none());
        assert_eq!(hall.update(skipped, 13_200), None);
        assert!(hall.update(anchor, 14_000).is_some());
        assert!(hall.sample_at(14_000).is_some());
        assert!((angle_difference(hall.angle, boundary_deg.to_radians())).abs() < 1.0e-6);
        let resumed_speed = old_speed * signed_width.signum();
        assert_eq!(
            hall.electrical_velocity().to_bits(),
            resumed_speed.to_bits()
        );
        hall.update(next, 18_000);
        let expected = signed_width.to_radians() / 0.004;
        assert!((hall.electrical_velocity() - expected).abs() < 0.001);
    }
}

#[test]
fn geometry_advance_and_legacy_reconfiguration_have_explicit_lifetimes() {
    let mut hall = measured_hall();
    hall.set_advance(0.2);
    advance_to_measured_motion(&mut hall);
    assert!(angle_difference(hall.angle, 110_f32.to_radians() + 0.2).abs() < 1.0e-6);
    let width = hall.current_sector_width;
    hall.set_advance(-0.3);
    assert_eq!(hall.current_sector_width.to_bits(), width.to_bits());
    hall.set_calibration_raw(CENTERS_DEG.map(f32::to_radians));
    assert!(hall.geometry.is_none());
    assert_eq!(
        hall.current_sector_width.to_bits(),
        hall.angle_per_state.to_bits()
    );
    assert!(!hall.geometry_resync);
}

#[test]
fn invalid_state_glitch_uses_measured_current_width_and_keeps_trusted_speed() {
    let mut hall = measured_hall();
    advance_to_measured_motion(&mut hall);
    let speed = hall.electrical_velocity();
    hall.update(0, 13_000);
    assert!(hall.sample_at(13_000).is_none());
    hall.update(6, 13_200); // same sector: no boundary or time change
    assert_eq!(hall.last_edge_ticks, Some(12_000));
    assert_eq!(hall.electrical_velocity().to_bits(), speed.to_bits());

    hall.update(7, 20_500);
    hall.update(4, 22_000); //90° sector predicts the hidden edge at21ms
    let recovered = hall.last_edge_ticks.unwrap();
    assert!((20_998..=21_002).contains(&recovered));
    assert!(hall.sample_at(22_000).is_some());
    assert!(!hall.geometry_resync);
    assert!((hall.angle - 200_f32.to_radians()).abs() < 1.0e-6);
    assert_eq!(hall.electrical_velocity().to_bits(), speed.to_bits());
    hall.update(5, 25_000);
    assert_eq!(hall.electrical_velocity().to_bits(), speed.to_bits());
    hall.update(1, 31_000);
    assert!((hall.electrical_velocity() - speed).abs() < 0.001);
}

#[test]
fn measured_topology_controls_direction_even_when_adjacent_centroids_are_far_apart() {
    // Both170° sectors are valid; their centroids near opposite ends have a
    // negative shortest-angle difference despite positive successor order.
    let entries = [0_f32, 355.0, 170.0, 0.0, 345.0, 350.0, 340.0, 0.0].map(f32::to_radians);
    let centers = [0_f32, 357.0, 339.0, 1.0, 347.0, 352.0, 342.0, 0.0].map(f32::to_radians);
    let mut hall = measured_hall();
    let next = hall.geometry.unwrap().next_states();
    hall.set_calibration_raw(centers);
    hall.geometry =
        Some(HallTransitionGeometry::try_from_raw_parts(entries, next, centers).unwrap());
    assert!(angle_difference(centers[2], centers[3]) < 0.0);
    hall.update(3, 0);
    hall.update(2, 17_000);
    assert_eq!(hall.direction(), Direction::Clockwise);
    hall.update(6, 34_000);
    let expected = 170_f32.to_radians() / 0.017;
    assert!((hall.electrical_velocity() - expected).abs() < 0.001);
    hall.update(2, 35_000);
    assert_eq!(hall.direction(), Direction::CounterClockwise);
    assert!(hall.electrical_velocity() < 0.0);
}

#[test]
fn reverse_edge_hidden_by_invalid_state_preserves_speed_magnitude_with_new_direction() {
    let mut hall = measured_hall();
    advance_to_measured_motion(&mut hall);
    let speed = hall.electrical_velocity();
    hall.update(0, 13_000);
    hall.update(2, 13_200);
    assert_eq!(hall.direction(), Direction::CounterClockwise);
    assert_eq!(hall.electrical_velocity().to_bits(), (-speed).to_bits());
    let sample = hall.sample_at(13_250).unwrap();
    assert!(angle_difference(sample.angle, 110_f32.to_radians()) < 0.0);
    assert_eq!(sample.omega.to_bits(), (-speed).to_bits());
    hall.update(3, 17_200);
    assert_eq!(hall.electrical_velocity().to_bits(), (-speed).to_bits());
    hall.update(1, 25_200);
    assert!((hall.electrical_velocity() + speed).abs() < 0.001);
}

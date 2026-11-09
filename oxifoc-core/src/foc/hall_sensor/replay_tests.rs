//! Differential replay against the sampling algorithm before evaluation reuse.
//! The oracle deliberately bypasses the evaluation cache and keeps an independent
//! limiter. Edge decoding is unchanged and is exercised through the same inputs.

use super::*;

#[derive(Default)]
struct LegacyLimiter {
    rate_limited_angle: f32,
    last_sample_ticks: Option<u64>,
}

fn legacy_sample(hall: &HallSensor, now_ticks: u64) -> Option<AngleSample> {
    let t0 = hall.last_edge_ticks?;
    if hall.in_glitch {
        // Invalid state right now: no angle (the caller falls back).
        return None;
    }
    // saturating: a wrapped "negative" dt as f32 would be ~2^64 and
    // collapse the decayed velocity to zero for the cycle.
    let dt = hall.timebase.seconds(now_ticks.saturating_sub(t0));

    // Current sector angle from calibration (direct raw state lookup)
    let sector_angle = hall.calib.angle_for_state(hall.raw_state);

    // Decayed velocity: see decayed_velocity() — a stopping motor must
    // not keep reporting (and integrating) the last edge speed forever.
    let velocity = hall.decayed_velocity(dt);

    // Check if velocity is below interpolation threshold (compare in eRPM)
    if HallSensor::vel_to_erpm(velocity) < hall.interp_min_erpm {
        // Low speed: use sector angle directly, no interpolation
        // This prevents oscillation during direction reversals
        return Some(AngleSample {
            angle: sector_angle,
            omega: velocity,
            direction: hall.direction,
        });
    }

    // High speed: interpolate using velocity
    Some(AngleSample {
        angle: wrap_angle(hall.angle + velocity * dt),
        omega: velocity,
        direction: hall.direction,
    })
}

fn legacy_stale(hall: &HallSensor, now_ticks: u64, omega_ref: f32) -> bool {
    let Some(dt_ticks) = hall.time_since_edge(now_ticks) else {
        // No edge ever seen: "no data yet", not "data stopped".
        return false;
    };
    let vel = hall.elec_velocity.abs().min(omega_ref.abs());
    if vel < 1.0 {
        return false;
    }
    let dt_s = hall.timebase.seconds(dt_ticks);
    let expected_sector_s = hall.angle_per_state / vel;
    dt_s > (expected_sector_s * STALE_SECTOR_MULTIPLE).max(STALE_MIN_WINDOW_S)
}

impl LegacyLimiter {
    fn sample(&mut self, hall: &HallSensor, now_ticks: u64) -> Option<AngleSample> {
        let t0 = hall.last_edge_ticks?;
        if hall.in_glitch {
            // Invalid state right now: no angle (the caller falls back).
            return None;
        }
        // saturating: see sample_at().
        let dt_from_edge = hall.timebase.seconds(now_ticks.saturating_sub(t0));

        // Calculate dt since last sample for rate limiting
        let dt_sample = if let Some(last) = self.last_sample_ticks {
            let dt_ticks = now_ticks.wrapping_sub(last).max(1);
            hall.timebase.seconds(dt_ticks)
        } else {
            dt_from_edge
        };
        self.last_sample_ticks = Some(now_ticks);

        // Current sector angle from calibration (direct raw state lookup)
        let sector_angle = hall.calib.angle_for_state(hall.raw_state);

        // Decayed velocity (see decayed_velocity()): also moves us to the
        // low-speed path once the edges stop coming.
        let velocity = hall.decayed_velocity(dt_from_edge);

        // Check if velocity is below interpolation threshold (compare in eRPM)
        if HallSensor::vel_to_erpm(velocity) < hall.interp_min_erpm {
            // Low speed: use sector angle directly, no interpolation
            // This prevents oscillation during direction reversals
            self.rate_limited_angle = sector_angle;
            return Some(AngleSample {
                angle: sector_angle,
                omega: velocity,
                direction: hall.direction,
            });
        }

        // High speed: interpolate using velocity. The interpolation starts at
        // the boundary just crossed and advances at most one sector (see
        // decayed_velocity), so it stays within the sector. (VESC's soft
        // drift correction — 1% of the drift, only past 30° from the
        // centroid — moved the angle by at most ~0.6° here and was dropped.)
        let target_angle = wrap_angle(hall.angle + velocity * dt_from_edge);

        // === RATE LIMITING (VESC-style) ===
        // Limit how fast angle can change per sample to prevent current spikes
        // max_step = max(|velocity|, min_vel_from_erpm) * dt * rate_limit_factor
        // Convert interp_min_erpm to rad/s for this calculation
        let min_vel_rad_s = hall.interp_min_erpm * TAU / 60.0;
        let effective_vel = velocity.abs().max(min_vel_rad_s);
        let max_step = effective_vel * dt_sample * hall.rate_limit_factor;

        // Calculate desired step from current rate-limited angle
        let desired_step = angle_difference(target_angle, self.rate_limited_angle);

        // Apply rate limiting
        let actual_step = if desired_step.abs() <= max_step {
            desired_step
        } else {
            // Clamp to max step, preserving sign
            if desired_step > 0.0 {
                max_step
            } else {
                -max_step
            }
        };

        self.rate_limited_angle = wrap_angle(self.rate_limited_angle + actual_step);

        Some(AngleSample {
            angle: self.rate_limited_angle,
            omega: velocity,
            direction: hall.direction,
        })
    }
}

fn assert_sample(actual: Option<AngleSample>, expected: Option<AngleSample>) {
    match (actual, expected) {
        (Some(actual), Some(expected)) => {
            assert_eq!(actual.angle.to_bits(), expected.angle.to_bits(), "angle");
            assert_eq!(actual.omega.to_bits(), expected.omega.to_bits(), "omega");
            assert_eq!(actual.direction, expected.direction);
        }
        (None, None) => {}
        pair => panic!("sample availability differs: {pair:?}"),
    }
}

fn assert_cycle(
    hall: &mut HallSensor,
    legacy: &mut LegacyLimiter,
    now: u64,
    omega_ref: Option<f32>,
) {
    let telemetry = legacy_sample(hall, now);
    // Prime twice through both immutable APIs, as telemetry/coast and other
    // readers may ask for the same tick before the control path runs.
    assert_sample(hall.sample_at(now), telemetry);
    match (hall.snapshot_at(now), telemetry) {
        (Some(snapshot), Some(sample)) => {
            assert_sample(
                Some(AngleSample {
                    angle: snapshot.angle_rad,
                    omega: snapshot.velocity_rad_s,
                    direction: snapshot.direction,
                }),
                Some(sample),
            );
            let age = hall
                .timebase
                .seconds(now.saturating_sub(hall.last_edge_ticks.unwrap()));
            assert_eq!(snapshot.edge_age_s.to_bits(), age.to_bits());
            assert_eq!(snapshot.state, hall.logical_state);
            assert_eq!(snapshot.error_count, hall.error_count);
        }
        (None, None) => {}
        _ => panic!("snapshot availability differs"),
    }

    let stale = omega_ref.is_some_and(|omega| legacy_stale(hall, now, omega));
    let expected = if stale {
        None
    } else {
        legacy.sample(hall, now)
    };
    let actual = hall.sample_for_control(now, omega_ref);
    assert_eq!(actual.stale, stale);
    assert_sample(actual.sample, expected);
    // Next-cycle prediction stays distinct from the unsmoothed present-time
    // snapshot, and must retain the same input to the existing manager step.
    let predict = |sample: AngleSample| AngleSample {
        angle: wrap_angle(sample.angle + sample.omega * 0.000_05),
        ..sample
    };
    assert_sample(actual.sample.map(predict), expected.map(predict));
    assert_eq!(hall.last_sample_ticks, legacy.last_sample_ticks);
    assert_eq!(
        hall.rate_limited_angle.to_bits(),
        legacy.rate_limited_angle.to_bits()
    );
    assert_sample(hall.sample_at(now), telemetry);
}

#[test]
fn differential_edges_reversals_glitches_and_limits() {
    let mut hall = HallSensor::new(1_000_000);
    let mut legacy = LegacyLimiter::default();
    assert_cycle(&mut hall, &mut legacy, 0, Some(1000.0));
    let mut ticks = 1_000_000;
    // Includes implausible bounce intervals, both interpolation modes,
    // acceleration, deceleration and a real reversal through each sector.
    for period in [1_000, 35, 1, 50, 200, 20_000, 100_000] {
        for raw in [1, 3, 2, 6, 4, 5, 1, 5, 4, 6, 2, 3, 1] {
            ticks += period;
            hall.update(raw, ticks);
            for offset in [0, 1, period / 2, period, period * 2, 100_000] {
                for reference in [None, Some(0.0), Some(10.0), Some(30_000.0)] {
                    assert_cycle(&mut hall, &mut legacy, ticks + offset, reference);
                }
            }
        }
    }
    // Invalid readings, recovery in the same sector, and a hidden edge
    // followed by both velocity-held edges. All reuse the same caller now.
    let now = ticks + 200;
    for (raw, edge) in [
        (0, ticks + 10),
        (1, ticks + 20),
        (7, ticks + 30),
        (3, ticks + 60),
        (2, ticks + 100),
        (6, ticks + 200),
    ] {
        hall.update(raw, edge);
        assert_cycle(&mut hall, &mut legacy, now, Some(30_000.0));
        assert_cycle(&mut hall, &mut legacy, now, None);
    }
}

#[test]
fn differential_same_tick_calibration_tuning_and_error_reset() {
    let mut hall = HallSensor::new(1_000_000);
    let mut legacy = LegacyLimiter::default();
    for (raw, ticks) in [(1, 0), (3, 1000), (2, 2000)] {
        hall.update(raw, ticks);
    }
    let now = 2500;
    let logical = core::array::from_fn(|i| wrap_angle(i as f32 * TAU / 6.0 + 0.25));
    let raw = [0.0, 0.35, 2.40, 1.25, 4.40, 5.35, 3.45, 0.0];
    let calibration = super::super::hall_calibration::HallCalibrationResult {
        angles: raw,
        valid: [false, true, true, true, true, true, true, false],
        valid_count: 6,
    };
    // Exercise immediate sector lookups at low speed and preserved boundary
    // bases at high speed, including NaN threshold behavior from the old code.
    let boundary_erpm = HallSensor::vel_to_erpm(hall.elec_velocity);
    for threshold in [
        0.0,
        boundary_erpm - 0.1,
        boundary_erpm,
        boundary_erpm + 0.1,
        f32::NAN,
    ] {
        hall.set_interp_min_erpm(threshold);
        assert_cycle(&mut hall, &mut legacy, now, None);
        hall.set_calibration(logical);
        assert_cycle(&mut hall, &mut legacy, now, None);
        hall.set_calibration_raw(raw);
        assert_cycle(&mut hall, &mut legacy, now, None);
        hall.set_advance(0.37);
        assert_cycle(&mut hall, &mut legacy, now, None);
        assert!(hall.apply_calibration(&calibration));
        assert_cycle(&mut hall, &mut legacy, now, None);
        assert!(
            !hall.apply_calibration(&super::super::hall_calibration::HallCalibrationResult {
                valid_count: 0,
                ..calibration
            })
        );
        assert_cycle(&mut hall, &mut legacy, now, None);
        for factor in [0.1, 1.5, 4.0] {
            hall.set_rate_limit_factor(factor);
            assert_cycle(&mut hall, &mut legacy, now, None);
        }
        hall.update(7, 2400);
        assert_cycle(&mut hall, &mut legacy, now, None);
        hall.reset_errors();
        assert_cycle(&mut hall, &mut legacy, now, None);
        hall.update(2, 2450);
        assert_cycle(&mut hall, &mut legacy, now, None);
    }
}

#[test]
fn differential_capture_interleavings_repeated_and_wrapped_timestamps() {
    let mut hall = HallSensor::new(1_000_000);
    let mut legacy = LegacyLimiter::default();
    for (raw, ticks) in [(1, 0), (3, 1000), (2, 2000)] {
        hall.update(raw, ticks);
    }
    for now in [2500, 2500, 2500, 2499, 2600, 3600, 3600] {
        for omega in [None, Some(0.0), Some(10.0), Some(1000.0), None] {
            assert_cycle(&mut hall, &mut legacy, now, omega);
        }
    }
    // A capture can precede the next estimator access but be timestamped
    // just ahead of the FOC ISR's fixed now. The prior cache must be gone.
    hall.update(6, 3601);
    assert_cycle(&mut hall, &mut legacy, 3600, Some(30_000.0));
    // Cover wide ages and both backward caller time and u64 clock wrap.
    for now in [u64::from(u32::MAX) + 10, u64::MAX - 100, 0, 1] {
        assert_cycle(&mut hall, &mut legacy, now, Some(1000.0));
    }
    hall.update(4, u64::MAX - 50);
    assert_cycle(&mut hall, &mut legacy, u64::MAX - 10, None);
    hall.update(5, 20);
    assert_cycle(&mut hall, &mut legacy, 50, None);
}

#[test]
fn combined_trait_default_preserves_independent_evidence_and_stateful_sampling() {
    struct Sensor {
        freshness_checks: Cell<u32>,
        samples: u32,
    }
    impl AngleSensor for Sensor {
        fn sample(&self, _: u64) -> Option<AngleSample> {
            Some(AngleSample {
                angle: 0.25,
                omega: 3.0,
                direction: Direction::Clockwise,
            })
        }

        fn sample_mut(&mut self, now: u64) -> Option<AngleSample> {
            self.samples += 1;
            self.sample(now)
        }

        fn is_stale(&self, _: u64, omega: f32) -> bool {
            self.freshness_checks.set(self.freshness_checks.get() + 1);
            omega > 5.0
        }

        fn error_count(&self) -> u32 {
            0
        }
        fn reset_errors(&mut self) {}
    }
    let mut sensor = Sensor {
        freshness_checks: Cell::new(0),
        samples: 0,
    };
    let unchecked = sensor.sample_for_control(10, None);
    assert!(!unchecked.stale);
    assert!(unchecked.sample.is_some());
    assert_eq!(sensor.freshness_checks.get(), 0);
    assert_eq!(sensor.samples, 1);
    let stale = sensor.sample_for_control(10, Some(6.0));
    assert!(stale.stale);
    assert!(stale.sample.is_none());
    assert_eq!(sensor.freshness_checks.get(), 1);
    assert_eq!(sensor.samples, 1);
    let fresh = sensor.sample_for_control(10, Some(0.0));
    assert_sample(fresh.sample, unchecked.sample);
    assert_eq!(sensor.freshness_checks.get(), 2);
    assert_eq!(sensor.samples, 2);
    let mut absent = super::super::sensors::NoSensor;
    assert_eq!(
        absent.sample_for_control(10, Some(1000.0)),
        ControlAngleSample {
            sample: None,
            stale: false,
        }
    );
}

#[test]
fn stale_verdict_keeps_raw_edge_speed_and_does_not_advance_limiter() {
    let mut hall = HallSensor::new(1_000_000);
    let mut legacy = LegacyLimiter::default();
    for (raw, ticks) in [(1, 0), (3, 1000), (2, 2000)] {
        hall.update(raw, ticks);
    }
    assert_cycle(&mut hall, &mut legacy, 2500, None);
    let limiter_before = (hall.last_sample_ticks, hall.rate_limited_angle.to_bits());
    // The decayed sample is slower than the raw edge speed. Using its
    // velocity for staleness would expand the deadline as now advances.
    assert_cycle(&mut hall, &mut legacy, 3600, Some(1000.0));
    assert!(hall.sample_for_control(3600, Some(1000.0)).stale);
    assert_eq!(
        (hall.last_sample_ticks, hall.rate_limited_angle.to_bits()),
        limiter_before
    );
    // A glitch has no sample, but the missed-edge deadline still applies.
    hall.update(7, 3500);
    assert_cycle(&mut hall, &mut legacy, 3600, Some(1000.0));
    assert!(hall.sample_for_control(3600, Some(1000.0)).stale);
    assert_eq!(
        (hall.last_sample_ticks, hall.rate_limited_angle.to_bits()),
        limiter_before
    );
    hall.update(2, 3550);
    assert_cycle(&mut hall, &mut legacy, 3600, None);
}

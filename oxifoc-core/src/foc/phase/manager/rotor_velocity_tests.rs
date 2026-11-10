use super::*;
use crate::foc::hall_calibration::{HallCalibrationResult, HallTransitionGeometry};
use crate::foc::hall_sensor::{Direction, HallSensor};
use crate::foc::phase::provider::{PhaseInput, PhaseProvider, RotorVelocityEstimate};
use crate::foc::sensors::{AngleSample, AngleSensor, ControlAngleSample};
use crate::foc::wrap_angle;

// The angle already represents the sensor's control sample. Its corrections
// deliberately differ from the independent edge-period velocity.
struct VelocitySensor {
    sample: Option<AngleSample>,
    epoch: u32,
}

impl VelocitySensor {
    fn new() -> Self {
        Self {
            sample: None,
            epoch: 0,
        }
    }

    fn set(&mut self, angle: f32, omega: f32) {
        self.sample = Some(AngleSample {
            angle: wrap_angle(angle),
            omega,
            direction: if omega < 0.0 {
                Direction::CounterClockwise
            } else {
                Direction::Clockwise
            },
        });
    }
}

impl AngleSensor for VelocitySensor {
    fn sample(&self, _now_ticks: u64) -> Option<AngleSample> {
        self.sample
    }

    fn sample_for_control(&mut self, _now_ticks: u64) -> Option<ControlAngleSample> {
        self.sample.map(|sample| ControlAngleSample {
            sample,
            reference_epoch: self.epoch,
        })
    }

    fn error_count(&self) -> u32 {
        u32::from(self.sample.is_none())
    }

    fn reset_errors(&mut self) {}
}

fn input() -> PhaseInput {
    PhaseInput {
        dt: 0.00005,
        ..Default::default()
    }
}

fn rotor_speed(manager: &impl PhaseProvider) -> Option<f32> {
    manager
        .rotor_velocity()
        .map(RotorVelocityEstimate::electrical_rad_s)
}

fn update_hall(manager: &mut PhaseManager<VelocitySensor>, ticks: u64, angle: f32, omega: f32) {
    manager.hall_mut().set(angle, omega);
    manager.update(&input(), ticks);
}

#[test]
fn rotor_velocity_estimate_accepts_only_finite_values_without_changing_them() {
    for value in [0.0, -0.0, 123.5, -123.5, f32::MAX, f32::MIN] {
        assert_eq!(
            RotorVelocityEstimate::new(value)
                .unwrap()
                .electrical_rad_s()
                .to_bits(),
            value.to_bits()
        );
    }
    for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(RotorVelocityEstimate::new(value).is_none());
    }
}

#[test]
fn hall_angle_catchup_does_not_change_constant_native_velocity() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    update_hall(&mut manager, 0, 0.2, 80.0);
    assert_eq!(rotor_speed(&manager), Some(80.0));
    assert_eq!(manager.get().velocity, 0.0);

    // Catchup at 1.5 times the native speed remains within the Hall limiter's
    // normal allowance, but must not become physical rotor acceleration.
    for step in 1..=400_u16 {
        update_hall(
            &mut manager,
            u64::from(step) * 50,
            0.2 + f32::from(step) * 0.006,
            80.0,
        );
        assert_eq!(rotor_speed(&manager), Some(80.0));
    }
    assert!(manager.get().velocity > 110.0);
    // Changing the native edge estimate still takes effect immediately while
    // the commutation-frame derivative remains continuous.
    update_hall(&mut manager, 20_050, 2.606, -250.0);
    assert_eq!(rotor_speed(&manager), Some(-250.0));
    assert!(manager.get().velocity > 0.0);
}

#[test]
fn stationary_frame_correction_does_not_report_physical_motion() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    update_hall(&mut manager, 0, 0.2, 0.0);
    // 60 rad/s is below the limiter floor at its default 500 eRPM threshold.
    for step in 1..=50_u16 {
        update_hall(
            &mut manager,
            u64::from(step) * 50,
            0.2 + f32::from(step) * 0.003,
            0.0,
        );
    }
    assert!(manager.get().velocity > 10.0);
    assert_eq!(rotor_speed(&manager), Some(0.0));
}

#[test]
fn unavailable_or_nonfinite_hall_does_not_publish_recovery_frame_speed() {
    for invalid in [
        None,
        Some((f32::NAN, 80.0)),
        Some((1.0, f32::NAN)),
        Some((1.0, f32::INFINITY)),
    ] {
        let mut manager = PhaseManager::with_hall(VelocitySensor::new());
        update_hall(&mut manager, 0, 0.2, 80.0);
        manager.hall_mut().sample = invalid.map(|(angle, omega)| AngleSample {
            angle,
            omega,
            direction: Direction::Clockwise,
        });
        manager.update(&input(), 50);
        assert!(manager.is_open_loop_override_active());
        assert_ne!(manager.get().velocity, 0.0);
        assert_eq!(rotor_speed(&manager), None);
        update_hall(&mut manager, 100, 3.0, -250.0);
        assert!(!manager.is_open_loop_override_active());
        assert_eq!(rotor_speed(&manager), Some(-250.0));
        assert_ne!(manager.get().velocity, -250.0);
    }
}

#[test]
fn accepted_source_and_timebase_changes_invalidate_the_cache() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    update_hall(&mut manager, 0, 0.2, 80.0);
    assert!(manager.set_source(PhaseSource::Observer).is_err());
    assert_eq!(rotor_speed(&manager), Some(80.0));

    let mut ticks = 50;
    for source in [PhaseSource::Manual, PhaseSource::OpenLoop] {
        manager.set_source(source).unwrap();
        assert_eq!(rotor_speed(&manager), None);
        manager.update(&input(), ticks);
        assert_eq!(rotor_speed(&manager), None);
        manager.set_source(PhaseSource::Hall).unwrap();
        assert_eq!(rotor_speed(&manager), None);
        ticks += 50;
        update_hall(&mut manager, ticks, 0.3, 80.0);
        assert_eq!(rotor_speed(&manager), Some(80.0));
        ticks += 50;
    }
    manager.set_ticks_per_sec(2_000_000);
    assert_eq!(rotor_speed(&manager), None);
    update_hall(&mut manager, ticks, 0.4, 80.0);
    assert_eq!(rotor_speed(&manager), Some(80.0));
}

#[test]
fn fresh_epoch_uses_its_native_velocity_without_differentiating_reference_jump() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    update_hall(&mut manager, 0, 0.2, 80.0);
    update_hall(&mut manager, 50, 0.204, 80.0);
    manager.hall_mut().epoch = 1;
    update_hall(&mut manager, 100, 4.0, -30.0);
    assert_eq!(manager.get().velocity, 0.0);
    assert_eq!(rotor_speed(&manager), Some(-30.0));
}

#[cfg(feature = "storage")]
#[test]
fn model_replacement_invalidates_native_velocity_until_a_fresh_sample() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    update_hall(&mut manager, 0, 0.2, 80.0);
    let mut model =
        PreparedPhaseModel::from_config(&crate::storage::RuntimeConfig::default(), 24.0);
    assert_eq!(manager.apply_prepared_model(&mut model), PhaseSource::Hall);
    assert_eq!(rotor_speed(&manager), None);
    update_hall(&mut manager, 50, 2.0, 80.0);
    assert_eq!(manager.get().velocity, 0.0);
    assert_eq!(rotor_speed(&manager), Some(80.0));
}

fn measured_hall() -> HallSensor {
    let mut angles = [0.0; 8];
    let mut entries = [0.0; 8];
    let mut next_states = [0; 8];
    for (raw, next, entry, center) in [
        (3_u8, 2, 0.0_f32, 40.0_f32),
        (2, 6, 80.0, 100.0),
        (6, 4, 120.0, 165.0),
        (4, 5, 210.0, 230.0),
        (5, 1, 250.0, 280.0),
        (1, 3, 310.0, 335.0),
    ] {
        let i = usize::from(raw);
        angles[i] = center.to_radians();
        entries[i] = entry.to_radians();
        next_states[i] = next;
    }
    let result = HallCalibrationResult {
        geometry: HallTransitionGeometry::try_from_raw_parts(entries, next_states, angles),
        angles,
        valid: [false, true, true, true, true, true, true, false],
        valid_count: 6,
    };
    assert!(result.geometry.is_some());
    let mut hall = HallSensor::new(1_000_000);
    assert!(hall.apply_calibration(&result));
    hall.update(3, 0);
    hall.update(2, 8_000);
    hall.update(6, 12_000);
    hall
}

#[test]
fn geometry_resync_invalidates_velocity_and_recovers_on_real_boundaries() {
    let mut manager = PhaseManager::with_hall(measured_hall());
    manager.update(&input(), 12_050);
    assert!(rotor_speed(&manager).unwrap() > 100.0);
    manager.hall_mut().update(5, 13_000); // A skipped boundary has no trusted angle.
    manager.update(&input(), 13_050);
    assert_eq!(rotor_speed(&manager), None);
    manager.hall_mut().update(1, 19_000);
    manager.update(&input(), 19_050);
    assert!(rotor_speed(&manager).unwrap() > 100.0);
    manager.hall_mut().update(3, 24_000);
    manager.update(&input(), 24_050);
    let expected = 50.0_f32.to_radians() / 0.005;
    assert!((rotor_speed(&manager).unwrap() - expected).abs() < 0.01);

    // Stop estimation uses growing edge age, not the non-decaying last speed.
    manager.update(&input(), 1_024_000);
    assert!(rotor_speed(&manager).unwrap() < 2.0);
    assert!(manager.hall().electrical_velocity() > 100.0);
}

#[test]
fn accepted_calibration_invalidates_velocity_but_rejection_preserves_it() {
    let mut manager = PhaseManager::with_hall(measured_hall());
    manager.update(&input(), 12_050);
    let before = rotor_speed(&manager);
    let mut calibration = HallCalibrationResult {
        geometry: None,
        angles: [-1.0; 8],
        valid: [false; 8],
        valid_count: 0,
    };
    assert!(!manager.apply_hall_calibration(&calibration));
    assert_eq!(rotor_speed(&manager), before);
    calibration.angles = [0.0, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 0.0];
    calibration.valid = [false, true, true, true, true, true, true, false];
    calibration.valid_count = 6;
    assert!(manager.apply_hall_calibration(&calibration));
    assert_eq!(rotor_speed(&manager), None);
    manager.update(&input(), 12_100);
    assert_eq!(rotor_speed(&manager), Some(0.0));
}

fn prime_observer(manager: &mut PhaseManager<VelocitySensor>) -> PhaseInput {
    use crate::foc::phase::{BackEmfObserver, Observer};
    let mut observer = BackEmfObserver::new(0.1, 100e-6, 0.02);
    observer.force_phase(0.8);
    observer.set_velocity(200.0);
    manager.set_observer(Observer::BackEmf(observer));
    manager.set_source(PhaseSource::Manual).unwrap();
    let (sin, cos) = 0.8_f32.sin_cos();
    let input = PhaseInput {
        v_alpha: -4.0 * sin,
        v_beta: 4.0 * cos,
        ..input()
    };
    manager.update(&input, 0);
    assert!(manager.observer.is_ready());
    input
}

#[test]
fn hybrid_blend_changes_frame_without_inventing_rotor_acceleration() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    let input = prime_observer(&mut manager);
    let mut min_frame = f32::MAX;
    let mut max_frame = f32::MIN;
    for (ticks, blend_low, blend_high) in
        [(50, 250.0, 350.0), (100, 100.0, 300.0), (150, 0.0, 100.0)]
    {
        manager
            .set_source(PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            })
            .unwrap();
        manager.hall_mut().set(0.4, 200.0);
        manager.update(&input, ticks);
        assert!((rotor_speed(&manager).unwrap() - 200.0).abs() < 1.0);
        min_frame = min_frame.min(manager.get().velocity);
        max_frame = max_frame.max(manager.get().velocity);
    }
    assert!(max_frame - min_frame > 100.0);
}

#[test]
fn ready_observer_fallback_publishes_its_own_velocity_but_startup_does_not() {
    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    let input = prime_observer(&mut manager);
    manager.set_source(PhaseSource::Hall).unwrap();
    manager.hall_mut().set(0.4, 80.0);
    manager.update(&input, 50);
    assert_eq!(rotor_speed(&manager), Some(80.0));
    manager.hall_mut().sample = None;
    manager.update(&input, 100);
    assert!(!manager.is_open_loop_override_active());
    assert!((rotor_speed(&manager).unwrap() - 200.0).abs() < 1.0);
    manager.startup.begin_cold_start(manager.get().angle, 1.0);
    assert!(manager.is_starting());
    assert_eq!(rotor_speed(&manager), None);
}

#[test]
fn motion_epoch_changes_on_hall_hybrid_fallback_but_not_ordinary_samples() {
    for source in [
        PhaseSource::Hall,
        PhaseSource::HallToObserver {
            blend_low: 250.0,
            blend_high: 350.0,
        },
    ] {
        let mut manager = PhaseManager::with_hall(VelocitySensor::new());
        let input = prime_observer(&mut manager);
        manager.set_source(source).unwrap();
        manager.hall_mut().set(0.4, 80.0);
        manager.update(&input, 50);
        let hall_epoch = manager.motion_reference_epoch();
        manager.hall_mut().set(0.5, 80.0);
        manager.update(&input, 100);
        assert_eq!(manager.motion_reference_epoch(), hall_epoch);
        manager.hall_mut().sample = None;
        manager.update(&input, 150);
        let fallback_epoch = manager.motion_reference_epoch();
        assert_ne!(fallback_epoch, hall_epoch);
        assert!(manager.rotor_velocity().is_some());
        manager.update(&input, 200);
        assert_eq!(manager.motion_reference_epoch(), fallback_epoch);
        manager.hall_mut().set(0.6, 80.0);
        manager.update(&input, 250);
        assert_ne!(manager.motion_reference_epoch(), fallback_epoch);
    }
}

#[test]
fn hybrid_observer_readiness_and_active_reseed_are_reference_boundaries() {
    use crate::foc::phase::{BackEmfObserver, Observer};
    for (blend_low, blend_high, contributes) in [(250.0, 350.0, false), (10.0, 50.0, true)] {
        let mut manager = PhaseManager::with_hall(VelocitySensor::new());
        update_hall(&mut manager, 0, 0.8, 200.0);
        let mut observer = BackEmfObserver::new(0.1, 100e-6, 0.02);
        observer.force_phase(0.8);
        // A coherent seed just below the speed gate becomes ready in the
        // real observer update, rather than importing readiness by a setter.
        observer.set_velocity(crate::foc::phase::observer::READY_MIN_VELOCITY.next_down());
        manager.set_observer(Observer::BackEmf(observer));
        manager
            .set_source(PhaseSource::HallToObserver {
                blend_low,
                blend_high,
            })
            .unwrap();
        assert!(!manager.observer.is_ready());
        let before = manager.motion_reference_epoch();
        let (sin, cos) = 0.8_f32.sin_cos();
        manager.update(
            &PhaseInput {
                v_alpha: -4.0 * sin,
                v_beta: 4.0 * cos,
                ..input()
            },
            50,
        );
        assert!(manager.observer.is_ready());
        assert_eq!(manager.motion_reference_epoch() != before, contributes);

        let before = manager.motion_reference_epoch();
        manager.compute_phase_with_fallback(
            Some(PhaseOutput {
                angle: 0.8 + core::f32::consts::PI,
                velocity: 30.0,
            }),
            RotorVelocityEstimate::new(30.0),
            None,
            input().dt,
            true,
        );
        assert_eq!(manager.motion_reference_epoch() != before, contributes);

        // Removing a previously ready estimator is deliberately conservative,
        // even at zero weight; it is a health boundary, not smooth blending.
        manager.observer.reset();
        let before = manager.motion_reference_epoch();
        manager.compute_phase_with_fallback(
            Some(PhaseOutput::default()),
            RotorVelocityEstimate::new(0.0),
            None,
            input().dt,
            true,
        );
        assert_ne!(manager.motion_reference_epoch(), before);
    }
}

#[cfg(feature = "hfi")]
#[test]
fn downward_hfi_reseed_does_not_publish_copied_frame_velocity() {
    use crate::foc::phase::HfiObserver;
    use core::f32::consts::TAU;

    let mut manager = PhaseManager::with_hall(VelocitySensor::new());
    let mut hfi = HfiObserver::new(1000.0, 3.0);
    hfi.set_phase(0.0); // Trusted polarity; isolate demod readiness from its probe.
    hfi.set_pll_gains(0.0, 0.0);
    manager.set_hfi_observer(hfi);
    manager.set_source(PhaseSource::Hfi).unwrap();
    for step in 0..1000_u16 {
        // An aligned d-axis carrier response earns real demod confidence.
        let carrier = f32::from(step) * TAU * 1000.0 * input().dt;
        let _injection = manager.injection();
        manager.update(
            &PhaseInput {
                i_alpha: 0.2 * libm::sinf(carrier),
                ..input()
            },
            u64::from(step) * 50,
        );
    }
    assert!(manager.hfi_observer().unwrap().is_ready());
    assert_eq!(rotor_speed(&manager), Some(0.0));

    manager
        .set_source(PhaseSource::HfiToHall { switch_vel: 100.0 })
        .unwrap();
    manager.crossover_latched = true;
    manager.output = PhaseOutput {
        angle: 0.5,
        velocity: 123.0,
    };
    let out = manager.compute_phase_with_fallback(
        Some(PhaseOutput {
            angle: 0.4,
            velocity: 10.0,
        }),
        RotorVelocityEstimate::new(10.0),
        None,
        input().dt,
        false,
    );
    assert!(!manager.crossover_latched);
    assert_eq!(out.velocity, 123.0); // The commutation handoff is unchanged.
    assert!(manager.hfi_observer().unwrap().is_ready());
    assert_eq!(rotor_speed(&manager), None);
}

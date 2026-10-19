use super::select::{blend_angles, compute_blend};
use super::*;
use crate::foc::angle_difference;
use crate::foc::hall_sensor::Direction;
#[cfg(feature = "virtual-motor")]
use crate::foc::hall_sensor::HallSensor;
use crate::foc::phase::provider::{PhaseInput, PhaseProvider};
use crate::foc::phase::source::PhaseSourceError;
use crate::foc::sensors::AngleSample;
use crate::foc::wrap_angle;
#[cfg(feature = "virtual-motor")]
use crate::virtual_motor::VirtualMotorOutput;
use core::f32::consts::TAU;

/// Phase-tracker shape (see PhaseTracker, TRACKER_KD_TAU_S), measured
/// as amplitude ratios of an injected estimate wobble: the 35-100 Hz
/// mid-band wobble is attenuated, the unavoidable low-frequency
/// resonant bump stays mild (a strong peak in the 5-25 Hz band would
/// PUMP rotor hunting), and the type-2 acceleration lag stays bounded
/// at ~omega_dot/wn^2 — never the ~90° standing angle of the freq-led
/// I/f geometry (dossier in docs/TODO.md).
#[test]
fn phase_tracker_attenuates_wobble_without_resonance_and_bounds_lag() {
    let dt = 1.0 / 20_000.0;
    // Amplitude ratio of the output wobble vs an injected angle wobble
    // riding a constant-speed ramp (measured over the last quarter).
    let ratio = |f_hz: f32| -> f32 {
        let amp = 0.5f32;
        let omega0 = 1000.0f32;
        let mut m = PhaseManager::sensorless();
        m.set_phase_tracker(60.0, 1.2);
        m.output = PhaseOutput {
            angle: 0.0,
            velocity: omega0,
        };
        let mut theta_clean = 0.0f32;
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for k in 0..16_000 {
            theta_clean = wrap_angle(theta_clean + omega0 * dt);
            #[allow(clippy::cast_precision_loss)]
            let ph = TAU * f_hz * (k as f32) * dt;
            let raw = PhaseOutput {
                angle: wrap_angle(theta_clean + amp * libm::sinf(ph)),
                velocity: omega0 + amp * TAU * f_hz * libm::cosf(ph),
            };
            let out = m.tracker_output(raw, dt);
            if k >= 12_000 {
                let d = angle_difference(out.angle, theta_clean);
                lo = lo.min(d);
                hi = hi.max(d);
            }
        }
        (hi - lo) / 2.0 / amp
    };
    let r60 = ratio(60.0);
    assert!(r60 < 0.4, "60 Hz wobble must be attenuated, ratio {r60}");
    let r90 = ratio(90.0);
    assert!(r90 < 0.2, "90 Hz wobble must be attenuated, ratio {r90}");
    let r14 = ratio(14.0);
    assert!(
        r14 < 1.5,
        "hunt-band resonant bump must stay mild, ratio {r14}"
    );

    // Constant-acceleration lag (bench 0.3 A unloaded ~1500 rad/s² el).
    let mut m = PhaseManager::sensorless();
    m.set_phase_tracker(60.0, 1.2);
    m.output = PhaseOutput {
        angle: 0.0,
        velocity: 300.0,
    };
    let mut theta_clean = 0.0f32;
    let mut vel = 300.0f32;
    let mut lag = 0.0f32;
    for k in 0..12_000 {
        vel += 1500.0 * dt;
        theta_clean = wrap_angle(theta_clean + vel * dt);
        let raw = PhaseOutput {
            angle: theta_clean,
            velocity: vel,
        };
        let out = m.tracker_output(raw, dt);
        if k >= 11_000 {
            lag = lag.max(angle_difference(raw.angle, out.angle).abs());
        }
    }
    assert!(lag < 0.6, "acceleration lag must stay bounded, got {lag}");

    // Max-torque free spin-up (bench 1.5 A: ~9 300 el rad/s²): the
    // clamp may bind during the a_est convergence transient, but must
    // RELEASE once the feedforward carries the trend — a 20 ms
    // catch-up τ rode the clamp for the entire climb (ω̇·τ ≈ 186
    // rad/s of standing frequency deficit; bench gap pinned
    // 0.62–0.67 rad all the way).
    let mut m = PhaseManager::sensorless();
    m.set_phase_tracker(30.0, 1.2);
    m.output = PhaseOutput {
        angle: 0.0,
        velocity: 300.0,
    };
    let mut theta_clean = 0.0f32;
    let mut vel = 300.0f32;
    let mut lag_late = 0.0f32;
    for k in 0..8_000 {
        vel += 9300.0 * dt;
        theta_clean = wrap_angle(theta_clean + vel * dt);
        let raw = PhaseOutput {
            angle: theta_clean,
            velocity: vel,
        };
        let out = m.tracker_output(raw, dt);
        // 8000 steps = 400 ms; the last quarter is well past the
        // a_est convergence (~3×TRACKER_FF_TAU_S = 90 ms).
        if k >= 6_000 {
            lag_late = lag_late.max(angle_difference(raw.angle, out.angle).abs());
        }
    }
    assert!(
        lag_late < 0.4,
        "the clamp must release once a_est carries the spin-up trend, \
         steady lag {lag_late}"
    );
}

#[test]
fn test_sensorless_manager() {
    let phase = PhaseManager::sensorless();
    assert!(!phase.has_hall());
    assert!(!phase.has_encoder());
    assert_eq!(phase.source(), PhaseSource::Manual);
}

#[test]
fn test_manual_angle() {
    let mut phase = PhaseManager::sensorless();
    phase.set_manual_angle(1.0);
    assert!((phase.manual_angle() - 1.0).abs() < 1e-6);
}

#[test]
fn test_open_loop() {
    let mut phase = PhaseManager::sensorless();
    phase.set_source(PhaseSource::OpenLoop).unwrap();
    phase.set_open_loop_velocity(100.0);

    let dt = 0.001;
    phase.update(
        &PhaseInput {
            dt,
            ..Default::default()
        },
        0,
    );

    // Angle should advance
    assert!(phase.open_loop_angle() > 0.0);
}

#[test]
fn test_blend_angles() {
    // Same angle
    assert!((blend_angles(0.0, 0.0, 0.5) - 0.0).abs() < 1e-6);

    // 50% blend
    let result = blend_angles(0.0, 1.0, 0.5);
    assert!((result - 0.5).abs() < 1e-6);

    // Wraparound case: blend from 6.0 to 0.5 (crossing 2π)
    let result = blend_angles(6.0, 0.5, 0.5);
    // Should interpolate through 0, not through 3
    assert!(!(1.0..=5.0).contains(&result));
}

#[test]
fn test_compute_blend() {
    assert_eq!(compute_blend(100.0, 200.0, 400.0), 0.0);
    assert_eq!(compute_blend(300.0, 200.0, 400.0), 0.5);
    assert_eq!(compute_blend(500.0, 200.0, 400.0), 1.0);
}

#[test]
#[cfg(feature = "hfi")]
fn injection_forwarded_only_in_hfi_regimes() {
    use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};

    let mut phase = PhaseManager::sensorless();
    phase.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    // HfiToObserver below also needs the fast slot configured.
    phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));

    // Manual source: HFI estimator configured but not commutating —
    // injecting would just heat the motor during calibration.
    assert_eq!(phase.injection(), (0.0, 0.0));

    // Pure HFI: carrier must flow (fresh carrier phase 0 → vd = A).
    phase.set_source(PhaseSource::Hfi).unwrap();
    let (vd, vq) = phase.injection();
    assert!(vd.abs() > 1.0, "expected carrier voltage, got vd = {vd}");
    assert_eq!(vq, 0.0, "pulsating injection is d-axis only");

    // Crossover source: inject below the latch; above the latch the
    // carrier stays on only inside the pre-heat margin
    // (speed < min_vel·(1+CROSSOVER_HYSTERESIS)) and stops above it.
    phase
        .set_source(PhaseSource::HfiToObserver {
            min_vel: 100.0,
            min_confidence: 0.5,
        })
        .unwrap();
    assert!(phase.injection().0.abs() > 1.0);
    phase.crossover_latched = true;
    // Latched but near the release threshold: pre-heat — carrier back
    // on so the demod locks before any blend weight arrives.
    phase.output.velocity = 110.0;
    assert!(
        phase.injection().0.abs() > 1.0,
        "carrier must pre-heat inside the margin above the latch"
    );
    // Latched and clear of the band: carrier off.
    phase.output.velocity = 100.0 * (1.0 + CROSSOVER_HYSTERESIS) + 10.0;
    assert_eq!(phase.injection(), (0.0, 0.0));
}

#[test]
#[cfg(feature = "hfi")]
fn hfi_to_sensor_is_untrusted_until_hfi_ready_or_sensor_latched() {
    use crate::foc::phase::HfiObserver;

    let mut phase =
        PhaseManager::with_hall(MockHallSensor::new()).with_encoder(MockHallSensor::new());
    phase.set_hfi_observer(HfiObserver::new(1000.0, 3.0));

    for source in [
        PhaseSource::HfiToHall { switch_vel: 100.0 },
        PhaseSource::HfiToEncoder { switch_vel: 100.0 },
    ] {
        phase.set_source(source).unwrap();
        assert!(
            !phase.angle_trustworthy(),
            "a cold HFI estimate must not pass the torque trust gate"
        );

        phase.crossover_latched = true;
        assert!(
            phase.angle_trustworthy(),
            "the latched hardware sensor carries a trustworthy angle"
        );
    }
}

#[test]
fn hfi_source_rejected_without_hfi_observer() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    // The HFI sources need the dedicated HFI slot: a back-EMF observer
    // can't generate a carrier, so they would silently never estimate.
    let mut phase = PhaseManager::sensorless();
    assert_eq!(
        phase.set_source(PhaseSource::Hfi),
        Err(PhaseSourceError::HfiNotConfigured)
    );
    phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));
    assert_eq!(
        phase.set_source(PhaseSource::Hfi),
        Err(PhaseSourceError::HfiNotConfigured)
    );
}

#[test]
fn test_hall_health_default() {
    // Sensorless manager should have HallHealth::NotPresent
    let phase = PhaseManager::sensorless();
    assert_eq!(phase.hall_health(), HallHealth::NotPresent);
}

#[test]
fn test_fault_tracking() {
    let mut phase = PhaseManager::sensorless();

    // No faults initially
    assert!(phase.faults().is_empty());
    assert!(!phase.has_fault(PhaseFault::HallTimeout));

    // Set a fault
    phase.set_fault(PhaseFault::HallTimeout);
    assert!(phase.has_fault(PhaseFault::HallTimeout));
    assert_eq!(phase.faults().len(), 1);

    // Setting same fault twice should not add duplicate
    phase.set_fault(PhaseFault::HallTimeout);
    assert_eq!(phase.faults().len(), 1);

    // Add another fault
    phase.set_fault(PhaseFault::ObserverNotReady);
    assert_eq!(phase.faults().len(), 2);
    assert!(phase.has_fault(PhaseFault::ObserverNotReady));

    // Clear specific fault
    phase.clear_fault(PhaseFault::HallTimeout);
    assert!(!phase.has_fault(PhaseFault::HallTimeout));
    assert!(phase.has_fault(PhaseFault::ObserverNotReady));
    assert_eq!(phase.faults().len(), 1);

    // Clear all faults
    phase.set_fault(PhaseFault::HallInvalidState);
    assert_eq!(phase.faults().len(), 2);
    phase.clear_faults();
    assert!(phase.faults().is_empty());
}

// Mock Hall sensor that can fail on demand
struct MockHallSensor {
    valid: bool,
    angle: f32,
    omega: f32,
}

impl MockHallSensor {
    fn new() -> Self {
        Self {
            valid: true,
            angle: 0.5,
            omega: 100.0,
        }
    }

    fn set_valid(&mut self, valid: bool) {
        self.valid = valid;
    }

    fn set_reading(&mut self, angle: f32, omega: f32) {
        self.angle = angle;
        self.omega = omega;
    }
}

impl AngleSensor for MockHallSensor {
    fn sample(&self, _now_ticks: u64) -> Option<AngleSample> {
        if self.valid {
            Some(AngleSample {
                angle: self.angle,
                omega: self.omega,
                direction: Direction::Clockwise,
            })
        } else {
            None
        }
    }

    fn read_angle(&self) -> f32 {
        self.angle
    }

    fn read_direction(&self) -> Direction {
        Direction::Clockwise
    }

    fn error_count(&self) -> u32 {
        if self.valid { 0 } else { 1 }
    }

    fn reset_errors(&mut self) {}
}

#[test]
fn test_hall_failure_triggers_fallback() {
    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);

    // Initially Hall is healthy
    assert_eq!(phase.hall_health(), HallHealth::Ok);

    // Update with valid Hall
    phase.update(&PhaseInput::default(), 0);
    assert_eq!(phase.hall_health(), HallHealth::Ok);
    assert!(!phase.has_fault(PhaseFault::HallInvalidState));

    // Make Hall fail
    phase.hall_mut().set_valid(false);
    phase.update(&PhaseInput::default(), 1000);

    // Hall should now be invalid
    assert_eq!(phase.hall_health(), HallHealth::Invalid);
    assert!(phase.has_fault(PhaseFault::HallInvalidState));

    // No observer configured, so also should have ObserverNotReady fault
    assert!(phase.has_fault(PhaseFault::ObserverNotReady));

    // Recover Hall
    phase.hall_mut().set_valid(true);
    phase.update(&PhaseInput::default(), 2000);

    // Hall should be healthy again and faults cleared
    assert_eq!(phase.hall_health(), HallHealth::Ok);
    assert!(!phase.has_fault(PhaseFault::HallInvalidState));
}

#[test]
fn test_hall_failure_uses_observer_when_available() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);

    // Set up observer with known angle
    let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
    observer.force_phase(1.5); // Force observer to have this phase
    observer.set_velocity(200.0);
    phase.set_observer(Observer::BackEmf(observer));

    // Make Hall fail
    phase.hall_mut().set_valid(false);
    phase.update(&PhaseInput::default(), 1000);

    // Should have fallen back to observer
    let output = phase.get();
    assert!((output.angle - 1.5).abs() < 0.01);
    assert!((output.velocity - 200.0).abs() < 0.01);

    // Should not have ObserverNotReady since observer was available
    assert!(!phase.has_fault(PhaseFault::ObserverNotReady));
}

#[test]
fn test_open_loop_override_activates_on_failure() {
    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);

    // Make Hall fail (no observer configured)
    phase.hall_mut().set_valid(false);

    // Get a first update to establish baseline
    phase.update(&PhaseInput::default(), 0);
    let _initial_output = phase.get();

    // Continue with Hall failed
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        1000,
    );

    // Open-loop override should be active
    assert!(phase.is_open_loop_override_active());
    assert!(phase.open_loop_override().timer > 0.0);

    // Output should still be valid (from open-loop override)
    let output = phase.get();
    assert!(output.velocity > 0.0); // Should have minimum velocity

    // After more updates, angle should advance
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        2000,
    );
    let output2 = phase.get();
    // Angle should have advanced
    assert!(output2.angle != output.angle || output.velocity > 0.0);
}

#[test]
fn unconverged_observer_does_not_take_over_on_hall_failure() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    // A freshly constructed observer is configured but NOT converged:
    // its phase is frozen at 0 with zero confidence. Handing it the
    // commutation on a Hall dropout swaps a good angle for garbage.
    // The manager must treat it as not-ready: raise ObserverNotReady
    // and use the open-loop override instead.
    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);
    phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));

    phase.hall_mut().set_valid(false);
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        0,
    );

    assert!(
        phase.has_fault(PhaseFault::ObserverNotReady),
        "unconverged observer must raise ObserverNotReady"
    );
    assert!(
        phase.is_open_loop_override_active(),
        "must fall back to open-loop override, not the frozen observer"
    );
}

/// Variant A of the dead-hall decision (2026-06-13, fault-overhaul.md):
/// while the open-loop recovery override fabricates the angle, the
/// manager must NOT report it trustworthy — the driver's iq gate then
/// coasts instead of pushing random-direction torque. Trust returns
/// with the sensor (and the override must actually deactivate when the
/// hall recovers — it used to outlive the glitch forever).
#[test]
fn override_makes_angle_untrusted_until_hall_recovers() {
    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);

    phase.update(&PhaseInput::default(), 0);
    assert!(phase.angle_trustworthy(), "healthy hall is trusted");

    // Hall dies, no observer: override fabricates the angle.
    phase.hall_mut().set_valid(false);
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        1_000,
    );
    assert!(phase.is_open_loop_override_active());
    assert!(
        !phase.angle_trustworthy(),
        "a fabricated angle must not be trusted"
    );

    // Hall recovers: the live sample retakes commutation, the override
    // deactivates, trust returns.
    phase.hall_mut().set_valid(true);
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        2_000,
    );
    assert!(
        !phase.is_open_loop_override_active(),
        "live hall must deactivate the recovery override"
    );
    assert!(phase.angle_trustworthy());
}

/// With a ready observer the fallback never reaches the override, so
/// the angle stays trusted through a hall dropout at speed.
#[test]
fn observer_fallback_keeps_angle_trusted() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);
    let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
    observer.force_phase(1.5);
    observer.set_velocity(200.0);
    phase.set_observer(Observer::BackEmf(observer));

    phase.hall_mut().set_valid(false);
    phase.update(&PhaseInput::default(), 1_000);

    assert!(!phase.is_open_loop_override_active());
    assert!(
        phase.angle_trustworthy(),
        "observer-carried fallback is trusted"
    );
}

/// `hall_fault()` reports the most specific degradation and only for
/// sources that consume hall data.
#[test]
fn hall_fault_reports_health_kinds() {
    use crate::foc::hall_sensor::HallFaultKind;

    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);
    assert_eq!(phase.hall_fault(), None, "healthy hall reports nothing");

    phase.hall_mut().set_valid(false);
    phase.update(&PhaseInput::default(), 1_000);
    assert_eq!(
        phase.hall_fault(),
        Some(HallFaultKind::InvalidState),
        "invalid health maps to the InvalidState kind"
    );

    // A non-hall source must not report an idle hall as a failure.
    phase.set_manual_angle(0.0);
    phase.set_source(PhaseSource::Manual).unwrap();
    phase.update(&PhaseInput::default(), 2_000);
    assert_eq!(phase.hall_fault(), None);
}

#[test]
fn blend_reseeds_pi_flipped_observer_from_sensor() {
    use crate::foc::phase::{BackEmfObserver, Observer};
    use core::f32::consts::PI;

    // Back-EMF observers started from standstill have a π ambiguity.
    // When the sensor is still trustworthy, a half-turn disagreement
    // means the observer is flipped — it must be reseeded from the
    // sensor instead of letting the blend pull the output toward the
    // inverted angle (torque collapse mid-crossover).
    let mut mock_hall = MockHallSensor::new();
    let hall_angle = 1.0f32;
    mock_hall.set_reading(hall_angle, 400.0); // mid blend band
    let mut phase = PhaseManager::with_hall(mock_hall);

    let mut observer = BackEmfObserver::new(0.1, 1e-4, 0.01);
    observer.force_phase(hall_angle + PI); // flipped
    observer.set_velocity(400.0);
    phase.set_observer(Observer::BackEmf(observer));
    phase
        .set_source(PhaseSource::HallToObserver {
            blend_low: 300.0,
            blend_high: 600.0,
        })
        .unwrap();

    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        0,
    );

    // The output predicts the next sample: hall extrapolated by ω·dt.
    let expected = hall_angle + 400.0 * 0.001;
    let out = phase.get();
    let err = angle_difference(out.angle, expected).abs();
    assert!(
        err < 0.3,
        "output {} drifted {} rad toward the flipped observer (hall predicts {})",
        out.angle,
        err,
        expected
    );
}

#[test]
fn blend_ignores_unready_observer() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    // In the blend band with an unconverged observer the sensor must
    // stay authoritative — blending with a frozen phase-0 estimate
    // drags the output toward garbage.
    let mut mock_hall = MockHallSensor::new();
    let hall_angle = 2.0f32;
    mock_hall.set_reading(hall_angle, 450.0); // mid blend band
    let mut phase = PhaseManager::with_hall(mock_hall);
    phase.set_observer(Observer::BackEmf(BackEmfObserver::new(0.1, 1e-4, 0.01)));
    phase
        .set_source(PhaseSource::HallToObserver {
            blend_low: 300.0,
            blend_high: 600.0,
        })
        .unwrap();

    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        0,
    );

    // The output predicts the next sample: hall extrapolated by ω·dt.
    let expected = hall_angle + 450.0 * 0.001;
    let out = phase.get();
    let err = angle_difference(out.angle, expected).abs();
    assert!(
        err < 0.05,
        "output {} must equal the predicted hall angle {} while observer is not ready (err {})",
        out.angle,
        expected,
        err
    );
}

/// Closed-loop sensorless harness: VirtualMotor + FocController +
/// PhaseManager(HallSensor + BackEmfObserver), HallToObserver source.
///
/// Spins from standstill on hall commutation; `hall_feed` maps the
/// plant's true hall state to what the firmware observes per step
/// (`None` = no signal at all — cable cut; a masked bit = a dead
/// wire). Deduplicates equal consecutive OBSERVED states: capture
/// hardware only fires on a pin change. Returns the manager, final
/// motor output and the largest one-cycle angle jump seen above the
/// hall interpolation regime.
#[cfg(feature = "virtual-motor")]
fn run_sensorless_sim(
    total_steps: u64,
    hall_feed: impl Fn(u64, u8) -> Option<u8>,
) -> (PhaseManager<HallSensor>, VirtualMotorOutput, f32) {
    use crate::foc::controller::FocController;
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::phase::{BackEmfObserver, Observer};
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::transforms;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    const VBUS: f32 = 24.0;
    // High friction so the motor settles around ωe ≈ 700 rad/s — well
    // above the blend band but below voltage saturation.
    let params = MotorParams {
        friction_b: 2e-3,
        ..MotorParams::default()
    };

    let mut motor = VirtualMotor::new(params);
    let mut foc =
        FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(params.r, params.ld, VBUS);

    // Prime the hall estimator with the initial rotor state so the
    // manager sees a working sensor from the start.
    let mut out = motor.step(0.0, 0.0, 0.0, DT);
    let mut hall = HallSensor::new(1_000_000); // µs timebase
    hall.update(out.hall_state, 0);
    let mut last_fed = out.hall_state;

    let mut mgr = PhaseManager::with_hall(hall);
    mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
        params.r,
        (params.ld + params.lq) / 2.0,
        params.lambda,
    )));
    mgr.set_source(PhaseSource::HallToObserver {
        blend_low: 150.0,
        blend_high: 300.0,
    })
    .unwrap();

    let iq_target = 2.0;
    let mut prev_angle: Option<f32> = None;
    let mut max_step_at_speed = 0.0f32;

    for step in 1..total_steps {
        let t_us = step * 50;

        if let Some(observed) = hall_feed(step, out.hall_state)
            && observed != last_fed
        {
            mgr.hall_mut().update(observed, t_us);
            last_fed = observed;
        }

        let angle = mgr.get().angle;
        let telem = foc.step((out.ia, out.ib, out.ic), angle, 0.0, iq_target, 1000, DT);
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

        let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
        mgr.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha: i_a,
                i_beta: i_b,
                dt: DT,
            },
            t_us,
        );

        // Continuity: above the hall interpolation regime the managed
        // angle must never jump (π flips, blend discontinuities). Below
        // it, 60° sector snaps are expected hall behavior.
        let new_angle = mgr.get().angle;
        if out.omega_e > 100.0 {
            if let Some(prev) = prev_angle {
                let jump = angle_difference(new_angle, prev).abs();
                max_step_at_speed = max_step_at_speed.max(jump);
            }
            prev_angle = Some(new_angle);
        } else {
            prev_angle = None;
        }
    }

    (mgr, out, max_step_at_speed)
}

/// Spin-up through the blend band to pure observer, halls healthy.
#[test]
#[cfg(feature = "virtual-motor")]
fn closed_loop_hall_to_observer_crossover() {
    const DT: f32 = 1.0 / 20_000.0;
    let (mgr, out, max_step_at_speed) = run_sensorless_sim(20_000, |_, s| Some(s));

    // Motor must have spun up past the blend band → pure observer.
    assert!(
        out.omega_e > 400.0,
        "motor did not spin up: ωe = {}",
        out.omega_e
    );
    assert!(
        mgr.observer().is_ready(),
        "observer must be converged at speed"
    );

    // Managed angle tracks the true rotor angle.
    let true_angle = wrap_angle(out.angle_rad);
    let err = angle_difference(mgr.get().angle, true_angle).abs();
    assert!(
        err < 0.25,
        "managed angle {} vs true {} (err {} rad)",
        mgr.get().angle,
        true_angle,
        err
    );

    // Velocity estimate agrees with the true electrical speed.
    let vel_err = (mgr.get().velocity - out.omega_e).abs() / out.omega_e;
    assert!(
        vel_err < 0.10,
        "managed velocity {} vs true {} ({}%)",
        mgr.get().velocity,
        out.omega_e,
        vel_err * 100.0
    );

    // No angle discontinuities across the whole crossover.
    let nominal_step = out.omega_e * DT;
    assert!(
        max_step_at_speed < nominal_step + 0.15,
        "angle jumped {max_step_at_speed} rad in one cycle (nominal step {nominal_step})"
    );
}

/// Hall cable cut at full speed: the manager must detect the stale
/// sensor (edges stopped arriving although the rotor demonstrably
/// spins), raise HallTimeout, and keep tracking on the observer alone.
#[test]
#[cfg(feature = "virtual-motor")]
fn closed_loop_hall_dropout_at_speed() {
    // 1 s with halls, then 0.3 s with the cable cut.
    let (mgr, out, max_step_at_speed) =
        run_sensorless_sim(26_000, |step, s| (step < 20_000).then_some(s));

    assert!(
        out.omega_e > 400.0,
        "motor must still be spinning: ωe = {}",
        out.omega_e
    );
    assert!(
        mgr.has_fault(PhaseFault::HallTimeout),
        "stale hall at speed must raise HallTimeout (health: {:?})",
        mgr.hall_health()
    );

    // Observer must be carrying the commutation accurately.
    let true_angle = wrap_angle(out.angle_rad);
    let err = angle_difference(mgr.get().angle, true_angle).abs();
    assert!(
        err < 0.25,
        "angle lost after hall dropout: managed {} vs true {} (err {})",
        mgr.get().angle,
        true_angle,
        err
    );

    // And the handoff itself must have been continuous.
    let nominal_step = out.omega_e * (1.0 / 20_000.0);
    assert!(
        max_step_at_speed < nominal_step + 0.15,
        "angle jumped {max_step_at_speed} rad during hall dropout handoff"
    );
}

/// Partial hall failure at speed: H1 goes stuck-low mid-ride. Above the
/// blend band the observer carries commutation, so the ride continues
/// smoothly — and the per-bit wire detector must name the dead wire
/// from the corrupted edge stream (one invalid state per electrical
/// revolution gates the verdict, see HallSensor::note_wire_activity).
#[test]
#[cfg(feature = "virtual-motor")]
fn closed_loop_partial_hall_failure_rides_through_and_names_wire() {
    use crate::foc::hall_sensor::HallFaultKind;

    const FAIL_AT: u64 = 20_000; // 1 s spin-up, then the wire breaks
    let (mgr, out, max_step_at_speed) = run_sensorless_sim(40_000, |step, s| {
        Some(if step < FAIL_AT { s } else { s & 0b110 })
    });

    assert!(
        out.omega_e > 400.0,
        "motor must ride through a partial hall failure: ωe = {}",
        out.omega_e
    );
    assert_eq!(
        mgr.hall().fault_kind(),
        Some(HallFaultKind::WireDead { mask: 0b001 }),
        "the dead wire must be named (health: {:?})",
        mgr.hall_health()
    );

    // Commutation stayed accurate and continuous on the observer.
    let true_angle = wrap_angle(out.angle_rad);
    let err = angle_difference(mgr.get().angle, true_angle).abs();
    assert!(err < 0.25, "angle err {err} rad with a dead hall wire");
    let nominal_step = out.omega_e * (1.0 / 20_000.0);
    assert!(
        max_step_at_speed < nominal_step + 0.15,
        "angle jumped {max_step_at_speed} rad during partial-failure ride-through"
    );
}

/// Full sensorless lifecycle on the dual estimator slots: HFI finds a
/// π-flipped rotor at standstill (saturation probe corrects it), the
/// motor accelerates on the HFI angle, and the manager crosses over to
/// the back-EMF observer at speed — continuously, with the carrier
/// injection shut off once latched.
#[test]
#[cfg(feature = "virtual-motor")]
#[cfg(feature = "hfi")]
fn closed_loop_hfi_to_observer_crossover() {
    use crate::foc::controller::FocController;
    use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::transforms;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    const MIN_VEL: f32 = 150.0;
    // Salient IPM with d-axis saturation (polarity probe needs it).
    // Light enough rotor to accelerate within the sim, enough friction
    // to settle around ωe ≈ 500 rad/s — well above the crossover.
    let params = MotorParams {
        r: 0.1,
        ld: 100e-6,
        lq: 300e-6,
        lambda: 0.02,
        pole_pairs: 4,
        j: 1e-3,
        friction_b: 2e-3,
        hall_offset: 0.0,
        sat_k: 0.05,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    // Far side: the HFI PLL's nearest saliency lock is the flipped one.
    motor.set_angle(2.5);

    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );

    let mut mgr = PhaseManager::sensorless();
    mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
        params.r,
        (params.ld + params.lq) / 2.0,
        params.lambda,
    )));
    mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    mgr.set_source(PhaseSource::HfiToObserver {
        min_vel: MIN_VEL,
        min_confidence: 0.5,
    })
    .unwrap();

    let mut out = VirtualMotorOutput {
        angle_rad: 2.5,
        ..Default::default()
    };
    let mut prev_angle: Option<f32> = None;
    let mut max_step_at_speed = 0.0f32;

    // Phase 1: standstill, zero torque — HFI must lock and resolve
    // polarity. Phase 2: torque on — spin up through the crossover.
    // Phase 3: torque off — coast back down through the band (reverse
    // handoff: observer → reseeded HFI).
    const STANDSTILL_STEPS: u64 = 10_000;
    const SPIN_STEPS: u64 = 40_000;
    const TOTAL_STEPS: u64 = 80_000;
    let mut spun_up = false;
    let mut was_latched_at_speed = false;
    // Downward-handoff instrumentation: the carrier must resume in the
    // pre-heat margin while still latched, and the demod must be locked
    // by the moment the latch releases (blend weight returns to HFI).
    let mut preheat_seen = false;
    let mut released_with_ready: Option<bool> = None;
    let mut was_latched = false;
    for step in 1..TOTAL_STEPS {
        let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
            2.0
        } else {
            0.0
        };

        let angle = mgr.get().angle;
        let (vd_inj, vq_inj) = mgr.injection();
        let telem = foc.step_with_injection(
            (out.ia, out.ib, out.ic),
            angle,
            0.0, // velocity (no decoupling configured here)
            0.0, // id_target
            iq_target,
            vd_inj,
            vq_inj,
            1000,
            DT,
        );
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

        let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
        mgr.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha: i_a,
                i_beta: i_b,
                dt: DT,
            },
            step * 50,
        );

        if step == SPIN_STEPS - 1 {
            spun_up = out.omega_e > MIN_VEL * 1.5;
            // Fully blended at speed → the carrier must be off.
            was_latched_at_speed = mgr.injection() == (0.0, 0.0);
        }

        // Downward handoff (coast phase): pre-heat + lock-before-weight.
        if step > SPIN_STEPS {
            let latched_now = mgr.crossover_latched;
            if latched_now && mgr.injection() != (0.0, 0.0) {
                preheat_seen = true;
            }
            if was_latched && !latched_now && released_with_ready.is_none() {
                released_with_ready = Some(mgr.hfi_observer().is_some_and(HfiObserver::is_ready));
            }
            was_latched = latched_now;
        } else {
            was_latched = mgr.crossover_latched;
        }

        if step == STANDSTILL_STEPS - 1 {
            // HFI must have found the flipped rotor at standstill.
            let err = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
            assert!(
                err < 0.2,
                "HFI standstill lock failed: est {} vs rotor {} (err {} full-circle)",
                mgr.get().angle,
                wrap_angle(out.angle_rad),
                err
            );
        }

        // Continuity above the crossover band (same check as the hall
        // crossover sims).
        let new_angle = mgr.get().angle;
        if out.omega_e > 100.0 {
            if let Some(prev) = prev_angle {
                let jump = angle_difference(new_angle, prev).abs();
                max_step_at_speed = max_step_at_speed.max(jump);
            }
            prev_angle = Some(new_angle);
        } else {
            prev_angle = None;
        }
    }

    assert!(spun_up, "motor did not spin up during the torque phase");
    assert!(
        was_latched_at_speed,
        "carrier injection must stop once fully blended onto the observer"
    );

    // Phase 3 outcome: coasted back down — HFI carries commutation
    // again (carrier on) and still tracks the slowing rotor.
    assert!(
        out.omega_e.abs() < MIN_VEL,
        "motor should have coasted below the band: ωe = {}",
        out.omega_e
    );
    assert_ne!(
        mgr.injection(),
        (0.0, 0.0),
        "carrier must be back on below the crossover band"
    );
    let true_angle = wrap_angle(out.angle_rad);
    let err = angle_difference(mgr.get().angle, true_angle).abs();
    assert!(
        err < 0.25,
        "managed angle {} vs true {} (err {} rad) after the down handoff",
        mgr.get().angle,
        true_angle,
        err
    );
    // Continuity through BOTH crossover directions.
    let nominal_step = 700.0 * DT; // generous bound: peak ωe during the run
    assert!(
        max_step_at_speed < nominal_step + 0.15,
        "angle jumped {max_step_at_speed} rad in one cycle through a crossover"
    );

    // Pre-heat: the carrier resumed in the margin while still latched —
    // i.e. before any blend weight could flow back to HFI…
    assert!(
        preheat_seen,
        "carrier must pre-heat inside the margin above the latch release"
    );
    // …and by the moment the latch released, the demod had re-locked
    // (confidence-before-weight). Without the pre-heat margin the demod
    // enters the band cold and this fails.
    assert_eq!(
        released_with_ready,
        Some(true),
        "HFI demod must be locked before blend weight returns to it"
    );
}

/// Mechanically-unbounded deceleration (wheel jam) crosses any finite
/// pre-heat margin instantly — that tail is covered by the TRUST GATE,
/// not the margin: the restarted demod reports zero confidence, so
/// `angle_trustworthy()` must stay false (driver keeps iq at zero)
/// until the lock is re-earned from real carrier current, and recover
/// at standstill afterwards. This is the explicit scope split: the
/// margin handles controller-bounded decel (test above), the gate
/// handles the unbounded case (this test).
#[test]
#[cfg(feature = "virtual-motor")]
#[cfg(feature = "hfi")]
fn closed_loop_hfi_jam_gates_torque_until_relock() {
    use crate::foc::controller::FocController;
    use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::transforms;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    const MIN_VEL: f32 = 150.0;
    let params = MotorParams {
        r: 0.1,
        ld: 100e-6,
        lq: 300e-6,
        lambda: 0.02,
        pole_pairs: 4,
        j: 1e-3,
        friction_b: 2e-3,
        hall_offset: 0.0,
        sat_k: 0.05,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);

    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );

    let mut mgr = PhaseManager::sensorless();
    mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
        params.r,
        (params.ld + params.lq) / 2.0,
        params.lambda,
    )));
    mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    mgr.set_source(PhaseSource::HfiToObserver {
        min_vel: MIN_VEL,
        min_confidence: 0.5,
    })
    .unwrap();

    let mut out = VirtualMotorOutput::default();

    // Phase 1: standstill HFI lock + polarity. Phase 2: spin up into
    // the latched regime. Phase 3: JAM — a brake torque far beyond the
    // motor's own, stopping the rotor within a few ms.
    const STANDSTILL_STEPS: u64 = 10_000;
    const SPIN_STEPS: u64 = 40_000;
    const TOTAL_STEPS: u64 = 60_000;

    let mut latched_at_speed = false;
    let mut cold_gate_seen = false;
    let mut trustworthy_during_cold = true;
    for step in 1..TOTAL_STEPS {
        let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
            2.0
        } else {
            0.0
        };
        // The jam: 50 N·m of external braking while the rotor still
        // turns forward (pp/J scaled, that is ~200 krad/s² — any
        // pre-heat margin is crossed in well under a millisecond).
        let load = if step >= SPIN_STEPS && out.omega_e > 2.0 {
            50.0
        } else {
            0.0
        };

        let angle = mgr.get().angle;
        let (vd_inj, vq_inj) = mgr.injection();
        let telem = foc.step_with_injection(
            (out.ia, out.ib, out.ic),
            angle,
            0.0,
            0.0,
            iq_target,
            vd_inj,
            vq_inj,
            1000,
            DT,
        );
        out = motor.step(telem.v_alpha, telem.v_beta, load, DT);

        let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
        mgr.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha: i_a,
                i_beta: i_b,
                dt: DT,
            },
            step * 50,
        );

        if step == SPIN_STEPS - 1 {
            latched_at_speed = mgr.crossover_latched && out.omega_e > MIN_VEL * 1.5;
        }

        // After the jam: the cold window is "latch released, demod not
        // yet ready". The trust gate must hold iq at zero throughout it.
        if step > SPIN_STEPS && !mgr.crossover_latched {
            let hfi_ready = mgr.hfi_observer().is_some_and(HfiObserver::is_ready);
            if !hfi_ready {
                cold_gate_seen = true;
                if mgr.angle_trustworthy() && !mgr.observer().is_ready() {
                    trustworthy_during_cold = false;
                }
            }
        }
    }

    assert!(
        latched_at_speed,
        "motor must reach the latched regime before the jam"
    );
    assert!(
        out.omega_e.abs() < 10.0,
        "jam must have stopped the rotor: ωe = {}",
        out.omega_e
    );
    assert!(
        cold_gate_seen,
        "the jam must produce a cold-demod window after latch release"
    );
    assert!(
        trustworthy_during_cold,
        "angle_trustworthy() must be false while the demod is cold \
         (the driver's iq gate is the only cover for unbounded decel)"
    );
    // Recovery: at standstill the carrier is on, the demod re-locks and
    // commutation trust returns.
    assert!(
        mgr.hfi_observer().is_some_and(HfiObserver::is_ready),
        "HFI must re-lock at standstill after the jam"
    );
    assert!(
        mgr.angle_trustworthy(),
        "trust must recover once the demod is locked"
    );
}

/// Saliency collapse under load — the classic HFI failure mode the
/// plant can now produce (`lq_sat_k`): torque current saturates the
/// q-axis iron until `Lq_eff ≈ Ld`, the demod error signal loses its
/// gradient and the angle estimate stops being corrected while the
/// rotor keeps moving.
///
/// The failure is INSIDIOUS: the carrier amplitude — and with it the
/// demod confidence — stays healthy (eps ≈ 0 reads as "no error"), so
/// no existing gate fires. This test pins both halves: the linear
/// control run keeps tracking under the identical load profile, the
/// saturating run silently loses the rotor at high confidence. The
/// detection gap (saliency monitor / dual-axis HFI45) is a TODO.
#[test]
#[cfg(feature = "virtual-motor")]
#[cfg(feature = "hfi")]
fn closed_loop_hfi_saliency_collapse_loses_tracking_silently() {
    use crate::foc::controller::FocController;
    use crate::foc::phase::HfiObserver;
    use crate::foc::phase::observer::HFI_READY_CONFIDENCE;
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::transforms;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    const LOCK_STEPS: u64 = 10_000;
    const TOTAL_STEPS: u64 = 16_000;
    const IQ_LOAD: f32 = 10.0;

    // (max tracking error during the loaded window, min HFI confidence
    // during that window)
    let run = |lq_sat_k: f32| -> (f32, f32) {
        // 3:1 IPM; at iq = 10 A with lq_sat_k = 0.3 the q-axis
        // saturation factor clamps at 4× → Lq_eff = 75 µH < Ld: the
        // saliency does not just vanish, it INVERTS, flipping the
        // PLL equilibrium to unstable (the catastrophic variant of
        // the failure).
        let params = MotorParams {
            r: 0.1,
            ld: 100e-6,
            lq: 300e-6,
            lambda: 0.02,
            pole_pairs: 4,
            j: 2e-3,
            friction_b: 1e-3,
            sat_k: 0.05,
            lq_sat_k,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);
        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r,
            (params.ld + params.lq) / 2.0,
            24.0,
        );

        let mut mgr = PhaseManager::sensorless();
        mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
        mgr.set_source(PhaseSource::Hfi).unwrap();

        let mut out = VirtualMotorOutput::default();
        let mut max_err = 0.0f32;
        let mut min_conf = 1.0f32;
        for step in 1..TOTAL_STEPS {
            // Phase 1: standstill lock + polarity. Phase 2: heavy
            // torque current against a load that nearly balances it —
            // the rotor creeps slowly (HFI regime) while iq saturates
            // the q axis.
            let (iq_target, load) = if step < LOCK_STEPS {
                (0.0, 0.0)
            } else {
                (IQ_LOAD, 1.05)
            };

            let angle = mgr.get().angle;
            let (vd_inj, vq_inj) = mgr.injection();
            let telem = foc.step_with_injection(
                (out.ia, out.ib, out.ic),
                angle,
                0.0,
                0.0,
                iq_target,
                vd_inj,
                vq_inj,
                1000,
                DT,
            );
            out = motor.step(telem.v_alpha, telem.v_beta, load, DT);

            let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
            mgr.update(
                &PhaseInput {
                    v_alpha: telem.v_alpha,
                    v_beta: telem.v_beta,
                    i_alpha: i_a,
                    i_beta: i_b,
                    dt: DT,
                },
                step * 50,
            );

            if step >= LOCK_STEPS + 1_000 {
                let err = angle_difference(mgr.get().angle, out.angle_rad).abs();
                max_err = max_err.max(err);
                if let Some(h) = mgr.hfi_observer() {
                    min_conf = min_conf.min(h.confidence());
                }
            }
        }
        (max_err, min_conf)
    };

    let (err_linear, _) = run(0.0);
    let (err_collapsed, conf_collapsed) = run(0.3);

    assert!(
        err_linear < 0.35,
        "linear plant: HFI must track the creeping rotor under load (max err {err_linear})"
    );
    assert!(
        err_collapsed > 0.7,
        "saturating plant: saliency collapse must lose the rotor (max err {err_collapsed})"
    );
    // The documented blind spot: confidence does NOT see the collapse
    // (eps ≈ 0 looks like a perfect lock). If this ever starts failing
    // because confidence drops, a saliency monitor exists — update the
    // test and close the TODO.
    assert!(
        conf_collapsed >= HFI_READY_CONFIDENCE,
        "expected the collapse to stay invisible to confidence, got {conf_collapsed}"
    );
}

/// The voltage-criterion crossover (HfiToObserverVolts) must behave
/// like the velocity one: carrier on at standstill, handoff to the
/// observer as the drive voltage (back-EMF proxy |vq − R·iq|) rises,
/// carrier off once latched, reseeded return to HFI on coast-down —
/// with the threshold in volts instead of per-motor eRPM tuning.
#[test]
#[cfg(feature = "virtual-motor")]
#[cfg(feature = "hfi")]
fn closed_loop_hfi_to_observer_volts_crossover() {
    use crate::foc::controller::FocController;
    use crate::foc::phase::{BackEmfObserver, HfiObserver, Observer};
    use crate::foc::pwm::SvpwmModulator;
    use crate::foc::transforms;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    // ωe settles ≈ 500 rad/s → bemf ≈ ω·λ = 10 V ≫ toggle; at
    // standstill the proxy is ≈ 0 V. Crossover happens around
    // ω ≈ toggle_v/λ = 150 rad/s, same regime as the velocity test.
    const TOGGLE_V: f32 = 3.0;
    let params = MotorParams {
        r: 0.1,
        ld: 100e-6,
        lq: 300e-6,
        lambda: 0.02,
        pole_pairs: 4,
        j: 1e-3,
        friction_b: 2e-3,
        hall_offset: 0.0,
        sat_k: 0.05,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    motor.set_angle(0.5);

    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );

    let mut mgr = PhaseManager::sensorless();
    mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
        params.r,
        (params.ld + params.lq) / 2.0,
        params.lambda,
    )));
    mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    mgr.set_source(PhaseSource::HfiToObserverVolts {
        toggle_v: TOGGLE_V,
        min_confidence: 0.5,
    })
    .unwrap();

    let mut out = VirtualMotorOutput {
        angle_rad: 0.5,
        ..Default::default()
    };

    const STANDSTILL_STEPS: u64 = 10_000;
    const SPIN_STEPS: u64 = 40_000;
    const TOTAL_STEPS: u64 = 70_000;
    let mut latched_at_speed = false;
    let mut err_at_speed = 0.0f32;
    for step in 1..TOTAL_STEPS {
        let iq_target = if (STANDSTILL_STEPS..SPIN_STEPS).contains(&step) {
            2.0
        } else {
            0.0
        };

        let angle = mgr.get().angle;
        let (vd_inj, vq_inj) = mgr.injection();
        let telem = foc.step_with_injection(
            (out.ia, out.ib, out.ic),
            angle,
            0.0,
            0.0,
            iq_target,
            vd_inj,
            vq_inj,
            1000,
            DT,
        );
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);

        let (i_a, i_b) = transforms::clarke(out.ia, out.ib);
        mgr.update(
            &PhaseInput {
                v_alpha: telem.v_alpha,
                v_beta: telem.v_beta,
                i_alpha: i_a,
                i_beta: i_b,
                dt: DT,
            },
            step * 50,
        );

        if step == SPIN_STEPS - 1 {
            assert!(
                out.omega_e > 300.0,
                "motor did not spin up: ωe = {}",
                out.omega_e
            );
            latched_at_speed = mgr.injection() == (0.0, 0.0);
            err_at_speed = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
        }
    }

    assert!(
        latched_at_speed,
        "carrier must be off once the drive voltage exceeds toggle_v"
    );
    assert!(
        err_at_speed < 0.25,
        "angle error {err_at_speed} rad at speed on the voltage crossover"
    );

    // Coasted down: bemf proxy below the band → back on HFI, carrier on.
    assert!(
        out.omega_e.abs() * params.lambda < TOGGLE_V - HFI_VOLTS_HYSTERESIS,
        "motor should have coasted below the voltage band: ωe = {}",
        out.omega_e
    );
    assert_ne!(
        mgr.injection(),
        (0.0, 0.0),
        "carrier must be back on below the voltage band"
    );
    let err = angle_difference(mgr.get().angle, wrap_angle(out.angle_rad)).abs();
    assert!(
        err < 0.3,
        "angle error {err} rad after the downward voltage handoff"
    );
}

/// Sources that don't use the hall sensor must not raise hall faults:
/// a manager doing Manual-angle calibration with an idle (or absent)
/// hall signal is not a failure condition.
#[test]
fn no_hall_faults_in_sources_that_ignore_hall() {
    let mut mock_hall = MockHallSensor::new();
    mock_hall.set_valid(false); // nothing on the hall lines
    let mut phase = PhaseManager::with_hall(mock_hall);
    phase.set_source(PhaseSource::Manual).unwrap();

    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        1000,
    );

    assert!(
        phase.faults().is_empty(),
        "Manual source must not raise hall faults, got {:?}",
        phase.faults()
    );
    assert!(!phase.is_open_loop_override_active());
}

#[test]
fn test_open_loop_override_deactivates_when_observer_ready() {
    use crate::foc::phase::{BackEmfObserver, Observer};

    let mock_hall = MockHallSensor::new();
    let mut phase = PhaseManager::with_hall(mock_hall);

    // Make Hall fail first (will activate open-loop override)
    phase.hall_mut().set_valid(false);
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        0,
    );

    assert!(phase.is_open_loop_override_active());

    // Now configure an observer with valid phase
    let mut observer = BackEmfObserver::new(1.0, 0.001, 0.01);
    observer.force_phase(2.0);
    observer.set_velocity(300.0);
    phase.set_observer(Observer::BackEmf(observer));

    // Update - observer should take over and deactivate open-loop override
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        1000,
    );

    // Open-loop override should be deactivated
    assert!(!phase.is_open_loop_override_active());

    // Output should come from observer (velocity matches)
    // Note: angle may have drifted slightly due to update() but velocity should match
    let output = phase.get();
    assert!((output.velocity - 300.0).abs() < 10.0); // Velocity should be from observer
}

/// Switching to a non-hall source must clear a live open-loop override:
/// the override only belongs to the hall source that armed it, and its
/// deactivation paths are all `requires_hall()`-gated. Left stranded, it
/// pins `angle_trustworthy()` false forever on a healthy encoder (whose
/// trust reads `!override.active`), and the driver coasts indefinitely.
#[test]
fn set_source_clears_stranded_open_loop_override() {
    let mut phase =
        PhaseManager::with_hall(MockHallSensor::new()).with_encoder(MockHallSensor::new());

    // Arm the override: hall dies with no observer ready.
    phase.hall_mut().set_valid(false);
    phase.update(
        &PhaseInput {
            dt: 0.001,
            ..Default::default()
        },
        0,
    );
    assert!(
        phase.is_open_loop_override_active(),
        "hall failure with no observer must arm the open-loop override"
    );

    // Host switches to the encoder — a non-hall source.
    phase
        .set_source(PhaseSource::Encoder)
        .expect("encoder is present");

    assert!(
        !phase.is_open_loop_override_active(),
        "switching to a non-hall source must clear the stranded override"
    );
    assert!(
        phase.angle_trustworthy(),
        "a healthy encoder must be trustworthy once the override is cleared"
    );
}

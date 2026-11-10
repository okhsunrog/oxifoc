use super::*;
use crate::foc::phase::{MotionReferenceEpoch, PhaseInput, PhaseOutput, RotorVelocityEstimate};
use crate::foc::velocity::VelocityLoopConfig;

struct ScheduledPhase {
    angle: f32,
    velocity: Option<f32>,
    trusted: bool,
    starting: bool,
    epoch: MotionReferenceEpoch,
    updates: usize,
}

impl PhaseProvider for ScheduledPhase {
    fn get(&self) -> PhaseOutput {
        PhaseOutput {
            angle: self.angle,
            ..Default::default()
        }
    }
    fn update(&mut self, _: &PhaseInput, _: u64) {
        self.updates += 1;
    }
    fn rotor_velocity(&self) -> Option<RotorVelocityEstimate> {
        self.velocity.and_then(RotorVelocityEstimate::new)
    }
    fn motion_reference_epoch(&self) -> MotionReferenceEpoch {
        self.epoch
    }
    fn angle_trustworthy(&self) -> bool {
        self.trusted
    }
    fn is_starting(&self) -> bool {
        self.starting
    }

    fn request_source(&mut self, source: PhaseSource) -> bool {
        matches!(source, PhaseSource::Hall)
    }
}

fn scheduled_driver() -> FocDriver<MockPwm, MockCurrentSensor, ScheduledPhase> {
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        ScheduledPhase {
            angle: 0.0,
            velocity: Some(0.0),
            trusted: true,
            starting: false,
            epoch: Default::default(),
            updates: 0,
        },
        1.0 / 20_000.0,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(3.0));
    driver.set_velocity_config(VelocityLoopConfig {
        kp: 1.0,
        ki: 1.0,
        accel_limit: 0.0,
        accel_ff: 0.0,
    });
    driver.set_mode(ControlMode::VelocityControl { target_vel: 2.0 });
    driver
}

#[test]
fn held_request_does_not_recompute_pi_on_an_ordinary_mutable_phase_sample() {
    let mut driver = scheduled_driver();
    driver.step(50).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 2.0);
    for now in (100..1_050).step_by(50) {
        driver.phase_mut().velocity = Some(1.0);
        driver.step(now).unwrap();
        assert_eq!(driver.velocity_loop.last_iq(), 2.0);
        assert_eq!(driver.last_granted_iq, 2.0);
    }
    assert_eq!(
        driver.phase().updates,
        20,
        "current/phase loop remains native-rate"
    );
    driver.step(1_050).unwrap();
    assert!(driver.velocity_loop.last_iq() < 1.01);
}

#[test]
fn driver_reanchors_clock_discontinuities_without_replaying_torque_or_elapsed_time() {
    let mut driver = scheduled_driver();
    driver.step(50).unwrap();
    assert_eq!(driver.last_granted_iq, 2.0);
    driver.step(5_000).unwrap();
    assert_eq!(
        driver.last_granted_iq, 0.0,
        "long native silence invalidates hold"
    );
    driver.step(5_050).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(6_000).unwrap();
    assert_eq!(driver.last_granted_iq, 2.0, "no integration across the gap");
    driver.step(5_999).unwrap();
    assert_eq!(
        driver.last_granted_iq, 0.0,
        "backward time also invalidates hold"
    );
    driver.step(6_999).unwrap();
    assert_eq!(driver.last_granted_iq, 2.0);
    assert_eq!(
        driver.phase().updates,
        6,
        "clock gating does not skip native FOC"
    );
}

#[test]
fn outer_integral_uses_elapsed_seconds_independent_of_native_current_cadence() {
    for native_hz in [5_000_u64, 20_000] {
        let mut driver = scheduled_driver();
        driver.dt = 1.0 / native_hz as f32;
        driver.set_current_limits(CurrentLimits::from_max_current(100.0));
        // This fixture has no electrical plant. Disable current integration
        // so artificial voltage saturation cannot constrain the outer loop.
        driver.set_current_pi_gains(0.0, 0.0);
        for step in 0..=native_hz {
            driver.step(step * 1_000_000 / native_hz).unwrap();
        }
        assert!(
            (driver.last_granted_iq - 4.0).abs() < 0.0001,
            "{native_hz}: {}",
            driver.last_granted_iq
        );
        assert_eq!(driver.phase().updates as u64, native_hz + 1);
    }
}

#[test]
fn loss_epoch_and_reversal_clear_held_torque_before_the_next_outer_tick() {
    for loss in 0..4 {
        let mut driver = scheduled_driver();
        driver.step(50).unwrap();
        match loss {
            0 => driver.phase_mut().velocity = None,
            1 => driver.phase_mut().trusted = false,
            2 => driver.phase_mut().epoch.advance(),
            _ => driver.set_mode(ControlMode::VelocityControl { target_vel: -2.0 }),
        }
        driver.step(100).unwrap();
        assert_eq!(driver.last_granted_iq, 0.0);
        assert_eq!(driver.velocity_schedule.requested_iq(), 0.0);
        driver.phase_mut().velocity = Some(0.0);
        driver.phase_mut().trusted = true;
        driver.step(150).unwrap();
        assert_eq!(
            driver.last_granted_iq, 0.0,
            "recovery cannot replay the old request"
        );
        driver.step(1_050).unwrap();
        assert_eq!(driver.last_granted_iq, if loss == 3 { -2.0 } else { 2.0 });
    }
}

#[test]
fn current_authority_and_trip_still_act_between_outer_updates() {
    let mut driver = scheduled_driver();
    driver.current_limits.bus_in_max_a = 0.1;
    driver.step(50).unwrap();
    driver.bus_mod_q_filt = 0.5;
    driver.step(100).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 2.0);
    assert_eq!(driver.last_granted_iq, 0.2);
    driver.set_derating_scales(DeratingScales {
        drive: 0.0,
        ..DeratingScales::IDENTITY
    });
    driver.step(150).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    assert_eq!(driver.velocity_loop.last_iq(), 2.0);
    driver.current_sensor_mut().currents = (7.0, -3.5, -3.5);
    assert_eq!(driver.step(200), Err(StepError::Overcurrent));
    assert_eq!(driver.velocity_schedule.requested_iq(), 0.0);
    assert_eq!(driver.last_granted_iq, 0.0);
}

#[test]
fn startup_and_stopped_never_resume_a_stale_held_request() {
    let mut driver = scheduled_driver();
    driver.step(50).unwrap();
    driver.phase_mut().starting = true;
    driver.step(100).unwrap();
    assert_eq!(driver.velocity_schedule.requested_iq(), 0.0);
    assert_eq!(driver.last_granted_iq, STARTUP_MIN_DRIVE_A);
    driver.phase_mut().starting = false;
    driver.step(150).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.set_mode(ControlMode::Stopped);
    driver.step(200).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.set_mode(ControlMode::VelocityControl { target_vel: 1.0 });
    driver.step(250).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(1_050).unwrap();
    assert_eq!(driver.last_granted_iq, 1.0);
}

#[test]
fn rejected_changes_keep_the_hold_but_accepted_source_or_limits_drop_it() {
    let mut driver = scheduled_driver();
    driver.step(50).unwrap();
    assert!(!driver.request_phase_source(PhaseSource::Encoder));
    let mut invalid = driver.velocity_config();
    invalid.ki = f32::NAN;
    driver.set_velocity_config(invalid);
    driver.step(100).unwrap();
    assert_eq!(driver.last_granted_iq, 2.0);
    assert!(driver.request_phase_source(PhaseSource::Hall));
    driver.step(150).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(1_050).unwrap();
    assert_eq!(driver.last_granted_iq, 2.0);
    driver.set_current_limits(CurrentLimits::from_max_current(0.25));
    driver.step(1_100).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(2_050).unwrap();
    assert!(driver.last_granted_iq <= 0.25);
}

#[test]
fn failsafe_takes_the_final_grant_and_never_records_against_cruise() {
    use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};
    let mut driver = scheduled_driver();
    driver.current_limits.bus_in_max_a = 0.1;
    driver.step(50).unwrap();
    driver.bus_mod_q_filt = 0.5;
    driver.step(100).unwrap();
    assert_eq!(driver.last_granted_iq, 0.2);
    driver.set_failsafe(FailsafeConfig {
        policy: FailsafePolicy::RampToZero,
        ..Default::default()
    });
    driver.enter_failsafe();
    driver.step(150).unwrap();
    assert!((driver.last_granted_iq - 0.1925).abs() < 1e-6);
    for now in (200..=900).step_by(50) {
        driver.step(now).unwrap();
        assert_eq!(driver.velocity_schedule.requested_iq(), 0.0);
        let (residual, blocked, voltage) = driver.velocity_schedule.take_feedback();
        assert_eq!(residual, 0.0);
        assert!(!blocked.positive() && !blocked.negative());
        assert!(!voltage.positive() && !voltage.negative());
    }
    driver.set_mode(ControlMode::VelocityControl { target_vel: 2.0 });
    driver.bus_mod_q_filt = 0.0;
    driver.step(950).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(1_050).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 2.0);
}

#[test]
fn released_native_clamp_is_tracked_once_with_the_correct_sign() {
    for sign in [-1.0, 1.0] {
        let mut limited = scheduled_driver();
        let mut baseline = scheduled_driver();
        for driver in [&mut limited, &mut baseline] {
            driver.set_mode(ControlMode::VelocityControl {
                target_vel: sign * 2.0,
            });
            driver.step(50).unwrap();
        }
        for now in (100..=2_050).step_by(50) {
            limited.set_derating_scales(DeratingScales {
                drive: if (100..600).contains(&now) { 0.0 } else { 1.0 },
                ..DeratingScales::IDENTITY
            });
            limited.step(now).unwrap();
            baseline.step(now).unwrap();
            if now == 1_050 || now == 2_050 {
                let correction = limited.velocity_loop.last_iq() - baseline.velocity_loop.last_iq();
                assert!((correction + sign).abs() < 1e-5, "{now}: {correction}");
            }
        }
    }
}

#[test]
fn both_voltage_clip_signs_survive_until_due_after_latest_clip_recovers() {
    for sign in [-1.0, 1.0] {
        let mut driver = scheduled_driver();
        driver.set_mode(ControlMode::VelocityControl {
            target_vel: sign * 2.0,
        });
        driver.controller.iq_pi.set_gains(100.0, 0.0);
        driver.step(50).unwrap();
        assert_eq!(driver.controller.q_voltage_clip(), sign);
        assert_eq!(driver.last_granted_iq, sign * 2.0);
        driver.current_sensor_mut().currents = (0.0, sign * 3.0, -sign * 3.0);
        driver.step(100).unwrap();
        assert_eq!(driver.controller.q_voltage_clip(), -sign);
        assert_eq!(driver.last_granted_iq, sign * 2.0);
        driver.controller.iq_pi.set_gains(0.01, 0.0);
        driver.current_sensor_mut().currents = (0.0, 0.0, 0.0);
        for now in (150..1_050).step_by(50) {
            driver.step(now).unwrap();
            assert_eq!(driver.controller.q_voltage_clip(), 0.0);
            assert_eq!(driver.last_granted_iq, sign * 2.0);
        }
        driver.step(1_050).unwrap();
        assert_eq!(driver.velocity_loop.last_iq(), sign);
    }
}

#[test]
fn position_tracks_native_angles_but_only_recomputes_its_target_on_due_ticks() {
    let mut driver = scheduled_driver();
    driver.set_mode(ControlMode::PositionControl { target_pos: 0.1 });
    driver.step(50).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 1.0);
    driver.phase_mut().angle = 0.05;
    driver.step(100).unwrap();
    assert_eq!(driver.position().as_rad(), 0.05);
    assert_eq!(driver.last_granted_iq, 1.0);
    driver.step(1_050).unwrap();
    assert!((driver.velocity_loop.last_iq() - 0.5).abs() < 0.002);
    driver.set_mode(ControlMode::PositionControl { target_pos: -0.1 });
    driver.step(1_100).unwrap();
    assert_eq!(driver.last_granted_iq, 0.0);
    driver.step(2_050).unwrap();
    assert!(driver.last_granted_iq < -1.49);
}

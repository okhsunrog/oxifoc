use super::*;

const D_GAINS: (f32, f32) = (0.37, 37.0);
const Q_GAINS: (f32, f32) = (0.61, 61.0);
const DETECTION_GAINS: (f32, f32) = (0.01, 10.0);

fn driver() -> FocDriver<MockPwm, MockCurrentSensor, PhaseManager<NoSensor, NoSensor>> {
    let mut driver = grant_driver(
        CurrentLimits::from_max_current(3.0),
        DeratingScales::IDENTITY,
    );
    driver.controller.id_pi.set_gains(D_GAINS.0, D_GAINS.1);
    driver.controller.iq_pi.set_gains(Q_GAINS.0, Q_GAINS.1);
    driver
}

fn hold(pi_gains: Option<(f32, f32)>) -> ControlMode {
    ControlMode::OpenLoop {
        angle_rad: 0.0,
        current: 1.0,
        velocity_rad_s: 0.0,
        pi_gains,
    }
}

#[test]
fn temporary_pi_scope_retains_hold_and_voltage_packets_then_restores_distinct_axes() {
    let mut driver = driver();
    driver.set_mode(hold(Some(DETECTION_GAINS)));
    driver.set_mode(hold(None));
    driver.set_mode(ControlMode::DirectVoltage {
        vd: 0.1,
        vq: 0.0,
        angle_rad: 0.0,
    });
    driver.set_mode(hold(None));
    assert_eq!(driver.controller().id_pi.gains(), DETECTION_GAINS);
    assert_eq!(driver.controller().iq_pi.gains(), DETECTION_GAINS);

    // A new temporary value updates the same sequence, never its baseline.
    driver.set_mode(hold(Some((0.02, 20.0))));
    driver.controller.id_pi.preload(0.2);
    driver.controller.iq_pi.preload(-0.3);
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.controller().id_pi.gains(), D_GAINS);
    assert_eq!(driver.controller().iq_pi.gains(), Q_GAINS);
    assert_eq!(driver.controller().id_pi.get_integral(), 0.0);
    assert_eq!(driver.controller().iq_pi.get_integral(), 0.0);
    assert!(driver.temporary_pi_restore.is_none());
    driver.set_mode(ControlMode::Stopped);
    driver.set_mode(hold(None));
    assert_eq!(driver.controller().id_pi.gains(), D_GAINS);
    assert_eq!(driver.controller().iq_pi.gains(), Q_GAINS);
}

#[test]
fn temporary_pi_scope_ends_before_every_normal_mode() {
    for mode in [
        ControlMode::Stopped,
        ControlMode::Coast,
        ControlMode::Brake,
        ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        },
        ControlMode::VelocityControl { target_vel: 100.0 },
        ControlMode::PositionControl { target_pos: 1.0 },
    ] {
        let mut driver = driver();
        driver.set_mode(hold(Some(DETECTION_GAINS)));
        driver.set_mode(mode);
        assert_eq!(driver.controller().id_pi.gains(), D_GAINS);
        assert_eq!(driver.controller().iq_pi.gains(), Q_GAINS);
        assert!(driver.temporary_pi_restore.is_none());
    }
}

#[test]
fn temporary_pi_scope_ends_on_fault_stop_brake_ramp_and_failsafe() {
    for exit in 0..4 {
        let mut driver = driver();
        driver.set_mode(hold(Some(DETECTION_GAINS)));
        match exit {
            0 => driver.safe_off(),
            1 => driver.emergency_stop(),
            2 => driver.enter_failsafe(),
            _ => assert!(driver.enter_brake_ramp()),
        }
        assert_eq!(driver.controller().id_pi.gains(), D_GAINS);
        assert_eq!(driver.controller().iq_pi.gains(), Q_GAINS);
        assert!(driver.temporary_pi_restore.is_none());
    }
    for voltage_packet in [false, true] {
        let mut driver = driver();
        driver.current_limits.overcurrent_threshold_a = 6.0;
        driver.set_mode(hold(Some(DETECTION_GAINS)));
        if voltage_packet {
            driver.set_mode(ControlMode::DirectVoltage {
                vd: 0.1,
                vq: 0.0,
                angle_rad: 0.0,
            });
        }
        driver.current_sensor.currents = (7.0, -3.5, -3.5);
        assert_eq!(driver.step(50), Err(StepError::Overcurrent));
        assert_eq!(driver.mode(), ControlMode::Stopped);
        assert_eq!(driver.controller().id_pi.gains(), D_GAINS);
        assert_eq!(driver.controller().iq_pi.gains(), Q_GAINS);
    }
}

#[test]
fn persistent_pi_tuning_supersedes_temporary_scope_without_later_rollback() {
    let mut driver = driver();
    driver.controller.id_pi.preload(0.2);
    driver.set_current_pi_gains(0.7, 70.0);
    assert_eq!(
        driver.controller().id_pi.get_integral(),
        0.2,
        "ordinary tuning keeps PI state"
    );
    driver.set_mode(hold(Some(DETECTION_GAINS)));
    driver.controller.id_pi.preload(0.3);
    driver.set_current_pi_gains(0.9, 90.0);
    assert_eq!(driver.controller().id_pi.get_integral(), 0.0);
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.controller().id_pi.gains(), (0.9, 90.0));
    assert_eq!(driver.controller().iq_pi.gains(), (0.9, 90.0));
    driver.set_mode(hold(Some(DETECTION_GAINS)));
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.controller().id_pi.gains(), (0.9, 90.0));

    driver.set_mode(hold(Some(DETECTION_GAINS)));
    driver.set_decoupling(None);
    assert_eq!(
        driver.controller().id_pi.gains(),
        DETECTION_GAINS,
        "model-only edits retain the bench PI"
    );
    let controller = driver.controller_mut();
    assert_eq!(controller.id_pi.gains(), (0.9, 90.0));
    controller.id_pi.set_gains(0.4, 40.0);
    controller.iq_pi.set_gains(0.5, 50.0);
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.controller().id_pi.gains(), (0.4, 40.0));
    assert_eq!(driver.controller().iq_pi.gains(), (0.5, 50.0));
}

#[cfg(feature = "storage")]
#[test]
fn prepared_model_pi_replaces_working_tuning_after_detection() {
    let mut driver = driver();
    let config = model_config();
    let mut prepared = PreparedMotorModel::from_config(&config, 24.0, 3.0);
    let expected = FocController::<SvpwmModulator>::from_runtime_config(&config, 24.0);
    driver.set_mode(hold(Some(DETECTION_GAINS)));
    assert_eq!(
        driver.apply_motor_model(&mut prepared),
        Err(MotorModelError::Busy)
    );
    assert_eq!(driver.controller().id_pi.gains(), DETECTION_GAINS);
    driver.set_mode(ControlMode::Stopped);
    driver.apply_motor_model(&mut prepared).unwrap();
    driver.set_mode(hold(Some(DETECTION_GAINS)));
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.controller().id_pi.gains(), expected.id_pi.gains());
    assert_eq!(driver.controller().iq_pi.gains(), expected.iq_pi.gains());
}

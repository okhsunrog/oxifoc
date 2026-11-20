use super::*;
use crate::units::{amps, erad, erad_per_s, ohms, si, volts};

fn prepared(input: f32, regen: f32) -> PreparedCommissioning {
    PreparedCommissioning::new(
        amps(60.0),
        amps(30.0),
        PowerSourceLimits::new(amps(input), amps(regen)).unwrap(),
        None,
    )
    .unwrap()
}

fn hold() -> ControlMode {
    ControlMode::OpenLoop {
        angle: erad(0.0),
        current: amps(10.0),
        velocity: erad_per_s(0.0),
        pi_gains: Some((0.01, 10.0)),
    }
}

#[test]
fn commissioning_is_independent_of_old_motor_but_retains_inverter_headroom() {
    let old = CurrentLimits::from_max_current(amps(2.0));
    let mut driver = grant_driver(old, DeratingScales::IDENTITY);
    driver.begin_commissioning(7, prepared(5.0, 0.0)).unwrap();
    assert_eq!(
        driver.current_limits.max_current,
        amps(30.0 / OVERCURRENT_HEADROOM)
    );
    assert_eq!(driver.current_limits.overcurrent_threshold, amps(30.0));
    assert_eq!(driver.current_limits.bus_in_max, amps(5.0));
    assert_eq!(
        driver.begin_commissioning(8, prepared(5.0, 0.0)),
        Err(CommissioningError::Busy)
    );
    driver.set_mode(hold());
    driver.set_mode(ControlMode::Stopped); // Intermediate stop keeps session.
    assert_eq!(driver.commissioning_generation(), Some(7));
    driver.end_commissioning();
    assert_eq!(driver.current_limits.max_current, old.max_current);
    assert_eq!(
        driver.current_limits.overcurrent_threshold,
        old.overcurrent_threshold
    );
    assert_eq!(driver.mode(), ControlMode::Stopped);
    assert_eq!(driver.commissioning_generation(), None);
}

#[test]
fn commissioning_all_exit_paths_restore_limits_and_pi_without_resuming() {
    for exit in 0..6 {
        let old = CurrentLimits::from_max_current(amps(2.0));
        let mut driver = grant_driver(old, DeratingScales::IDENTITY);
        driver.set_current_pi_gains(ohms(0.12), si(12.0));
        driver.begin_commissioning(1, prepared(5.0, 0.0)).unwrap();
        driver.set_mode(hold());
        match exit {
            0 => driver.end_commissioning(),
            1 => driver.safe_off(),
            2 => driver.emergency_stop(),
            3 => driver.enter_failsafe(),
            4 => driver.set_mode(ControlMode::CurrentControl {
                id_target: amps(0.0),
                iq_target: amps(1.0),
            }),
            _ => {
                driver.current_sensor.currents = (40.0, -20.0, -20.0);
                assert_eq!(driver.step(50), Err(StepError::Overcurrent));
            }
        }
        assert_eq!(driver.commissioning_generation(), None);
        assert_eq!(driver.mode(), ControlMode::Stopped);
        assert_eq!(driver.current_limits.max_current, old.max_current);
        assert_eq!(pi_gains(&driver.controller.id_pi), (0.12, 12.0));
        assert_eq!(driver.controller.id_pi.get_integral(), volts(0.0));
    }
}

#[test]
fn source_monitor_covers_d_axis_consumption_and_reverse_power_in_voltage_packets() {
    for (current, regen, expected_error) in
        [(4.0, 0.0, true), (-2.0, 0.0, true), (-2.0, 2.0, false)]
    {
        let mut driver = grant_driver(
            CurrentLimits::from_max_current(amps(2.0)),
            DeratingScales::IDENTITY,
        );
        driver.begin_commissioning(1, prepared(1.0, regen)).unwrap();
        driver.set_mode(ControlMode::DirectVoltage {
            vd: volts(12.0),
            vq: volts(0.0),
            angle: erad(0.0),
        });
        for _ in 0..4 {
            driver
                .commissioning
                .as_mut()
                .unwrap()
                .modulation
                .record_command(0.5, 0.0);
        }
        driver.current_sensor.currents = (current, -0.5 * current, -0.5 * current);
        let mut result = driver.step(50);
        for cycle in 2..=100 {
            if result.is_err() {
                break;
            }
            result = driver.step(cycle * 50);
        }
        if expected_error {
            assert_eq!(result, Err(StepError::SourceLimit));
            assert_eq!(driver.mode(), ControlMode::Stopped);
            assert_eq!(driver.commissioning_generation(), None);
            assert_eq!(driver.current_limits.max_current, amps(2.0));
        } else {
            assert!(result.is_ok());
        }
    }
}

#[test]
fn source_current_constraint_accounts_for_stationary_d_current() {
    let mut driver = grant_driver(CurrentLimits::default(), DeratingScales::IDENTITY);
    driver.begin_commissioning(1, prepared(5.0, 0.0)).unwrap();
    assert!(!driver.commissioning_source_constrained());
    let context = driver.commissioning.as_mut().unwrap();
    for _ in 0..4 {
        context.modulation.record_command(1.0, 0.0);
    }
    assert_eq!(
        context.constrain_current::<LibmSinCos>(erad(0.0), amps(10.0), false),
        amps(5.0)
    );
    assert_eq!(
        context.constrain_current::<LibmSinCos>(erad(0.0), amps(-10.0), false),
        amps(0.0)
    );
    assert_eq!(
        context.constrain_current::<LibmSinCos>(erad(0.0), amps(10.0), true),
        amps(10.0)
    );
    assert!(driver.commissioning_source_constrained());
    driver.end_commissioning();
    assert!(!driver.commissioning_source_constrained());
}

#[test]
fn commissioning_rejects_invalid_or_disabled_protections_and_caps() {
    let source = PowerSourceLimits::new(amps(5.0), amps(0.0)).unwrap();
    for invalid in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        assert!(PreparedCommissioning::new(amps(invalid), amps(30.0), source, None).is_err());
        assert!(PreparedCommissioning::new(amps(60.0), amps(invalid), source, None).is_err());
        assert!(
            PreparedCommissioning::new(amps(60.0), amps(30.0), source, Some(amps(invalid)))
                .is_err()
        );
    }
    assert_eq!(
        PreparedCommissioning::new(amps(60.0), amps(30.0), source, Some(amps(3.0)))
            .unwrap()
            .current_ceiling(),
        amps(3.0)
    );
}

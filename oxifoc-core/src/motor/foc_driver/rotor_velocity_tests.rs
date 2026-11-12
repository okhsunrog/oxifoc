use super::*;
use crate::foc::phase::{PhaseInput, PhaseOutput, RotorVelocityEstimate};
use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::derating::DeratingConfig;
#[cfg(feature = "storage")]
use crate::motor::drive_limits::DriveLimitsConfig;

struct SplitPhase {
    frame_velocity: f32,
    rotor_velocity: Option<f32>,
}

impl PhaseProvider for SplitPhase {
    fn get(&self) -> PhaseOutput {
        PhaseOutput {
            angle: 0.0,
            velocity: self.frame_velocity,
        }
    }

    fn rotor_velocity(&self) -> Option<RotorVelocityEstimate> {
        self.rotor_velocity.and_then(RotorVelocityEstimate::new)
    }

    fn update(&mut self, _: &PhaseInput, _: u64) {}
}

fn split_driver(
    frame_velocity: f32,
    rotor_velocity: Option<f32>,
) -> FocDriver<MockPwm, MockCurrentSensor, SplitPhase> {
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        SplitPhase {
            frame_velocity,
            rotor_velocity,
        },
        1.0 / 20_000.0,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(3.0));
    driver
}

#[test]
#[cfg(feature = "storage")]
fn inverted_directional_limits_follow_rider_and_preserve_braking() {
    let mut driver = split_driver(-90.0, Some(-90.0));
    driver.set_derating(DeratingConfig {
        max_speed_erad_s: 50.0,
        ..Default::default()
    });
    driver
        .set_drive_limits(DriveLimitsConfig {
            invert_direction: true,
            max_forward_erad_s: Some(100.0),
            max_reverse_erad_s: None,
            ..Default::default()
        })
        .unwrap();
    assert_eq!(driver.speed_ceiling(-90.0), 100.0);
    assert_eq!(driver.speed_ceiling(90.0), 50.0);
    assert_eq!(driver.grant_torque(0.0, -3.0, -90.0).1, -1.5);
    assert_eq!(driver.grant_torque(0.0, 3.0, 90.0).1, 0.0);
    assert_eq!(driver.grant_torque(0.0, 1.0, -90.0).1, 1.0);

    let mode = ControlMode::Throttle { fraction: 0.5 };
    #[cfg(feature = "runtime")]
    let mode = driver.orient_drive_command(mode);
    #[cfg(feature = "runtime")]
    {
        driver.set_mode(mode);
        driver.step(50).unwrap();
        assert_eq!(driver.last_granted_iq, -0.75);
        assert!(driver.set_drive_limits(DriveLimitsConfig::DEFAULT).is_err());
    }
    #[cfg(not(feature = "runtime"))]
    let _ = mode;
}

#[test]
#[cfg(feature = "storage")]
fn duty_taper_scales_throttle_and_clamps_only_larger_amp_requests() {
    let mut driver = split_driver(100.0, Some(100.0));
    driver
        .set_drive_limits(DriveLimitsConfig {
            duty_start_frac: DriveLimitsConfig::E_SKATE_DUTY_START,
            ..Default::default()
        })
        .unwrap();
    let voltage_limit = driver.controller.voltage_limit();
    driver.update_duty_utilization(0.0, 0.925 * voltage_limit, 0.002);
    assert!((driver.throttle_current(0.5) - 0.75).abs() < 1e-5);
    assert!((driver.grant_torque(0.0, 3.0, 100.0).1 - 1.5).abs() < 1e-5);
    assert_eq!(driver.grant_torque(0.0, 1.0, 100.0).1, 1.0);
    assert_eq!(driver.grant_torque(0.0, -3.0, 100.0).1, -3.0);
    driver.set_mode(ControlMode::Stopped);
    assert_eq!(driver.throttle_current(0.5), 1.5);
}

#[test]
fn velocity_feedback_ignores_frame_corrections_and_reenters_bumplessly() {
    let mut driver = split_driver(1000.0, Some(20.0));
    driver.set_mode(ControlMode::VelocityControl { target_vel: 20.0 });
    driver.step(50).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 0.0);
    driver.phase.frame_velocity = -1000.0;
    driver.step(100).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 0.0);

    driver.phase.rotor_velocity = None;
    driver.step(150).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 0.0);
    driver.phase.rotor_velocity = Some(20.0);
    driver.step(200).unwrap();
    assert_eq!(
        driver.velocity_loop.last_iq(),
        0.0,
        "reentry seeds actual speed, not zero or frame speed"
    );

    // The current-loop frame path and existing telemetry remain unchanged.
    driver.controller.set_decoupling(Some(Decoupling {
        ld_h: 20e-6,
        lq_h: 20e-6,
        resistance_ohm: 0.03,
        flux_linkage_wb: 0.003,
    }));
    driver
        .controller
        .set_voltage_feedforward_mode(crate::types::VoltageFeedforwardMode::BackEmf);
    let out = driver.step(250).unwrap();
    assert!((out.vq + 3.0).abs() < 1e-6, "FF still uses frame omega");
    assert_eq!(out.velocity_rad_s, -1000.0);
}

#[test]
fn velocity_authority_and_final_current_grant_share_physical_speed() {
    let mut driver = split_driver(1000.0, Some(20.0));
    driver.set_derating(DeratingConfig {
        max_speed_erad_s: 100.0,
        ..Default::default()
    });
    driver.set_velocity_config(VelocityLoopConfig {
        kp: 1.0,
        ki: 0.0,
        accel_limit: 0.0,
        accel_ff: 0.0,
    });
    driver.set_mode(ControlMode::VelocityControl { target_vel: 40.0 });
    let out = driver.step(50).unwrap();
    assert_eq!(driver.velocity_loop.last_iq(), 3.0);
    assert!(
        out.vq > 0.0,
        "final grant must not cut torque using frame speed"
    );

    let mut overspeed = split_driver(0.0, Some(200.0));
    overspeed.set_derating(*driver.derating_cfg());
    overspeed.set_mode(ControlMode::CurrentControl {
        iq_target: 3.0,
        id_target: 0.0,
    });
    assert_eq!(
        overspeed.step(50).unwrap().vq,
        0.0,
        "physical overspeed still cuts motoring torque"
    );

    driver.phase.rotor_velocity = None;
    assert_eq!(
        driver.torque_speed(),
        1000.0,
        "unknown motion preserves the explicit legacy/startup grant fallback"
    );
    driver.phase.rotor_velocity = Some(0.0);
    assert_eq!(
        driver.torque_speed(),
        0.0,
        "credible zero must not select fallback"
    );
}

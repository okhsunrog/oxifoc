//! Frozen pre-outline torque grant oracle, including the optional brake solve.

use super::*;
use crate::foc::clamp_f32;
use crate::foc::fast_math::sqrtf;
use crate::foc::phase::{PhaseInput, PhaseOutput, RotorVelocityEstimate};
use crate::motor::dissipation::{RegenBudget, WindingModel, dissipate_with_id};

type GrantDriver = FocDriver<MockPwm, MockCurrentSensor, PhaseManager<NoSensor, NoSensor>>;

struct StationaryRotor;

impl PhaseProvider for StationaryRotor {
    fn get(&self) -> PhaseOutput {
        PhaseOutput {
            angle: 0.0,
            velocity: 0.0,
        }
    }

    fn rotor_velocity(&self) -> Option<RotorVelocityEstimate> {
        RotorVelocityEstimate::new(0.0)
    }

    fn update(&mut self, _: &PhaseInput, _: u64) {}
}

#[test]
fn direct_and_reused_grants_keep_the_same_independent_current_trip() {
    for mode in [
        ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        },
        ControlMode::VelocityControl { target_vel: 100.0 },
    ] {
        let mut driver = FocDriver::new(
            FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (7.0, -3.5, -3.5),
            },
            StationaryRotor,
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(3.0));
        driver.set_mode(mode);
        assert_eq!(driver.step(50), Err(StepError::Overcurrent));
        assert_eq!(driver.mode(), ControlMode::Stopped);
    }
}

fn legacy_bus_clamp(driver: &GrantDriver, iq: f32) -> f32 {
    let in_max = driver.current_limits.bus_in_max_a;
    let regen_max = driver.current_limits.bus_regen_max_a;
    if in_max < 0.0 && regen_max < 0.0 {
        return iq;
    }
    let mod_q = driver.bus_mod_q_filt;
    if mod_q.abs() < 1e-3 {
        return iq;
    }
    let hi_bus = if in_max >= 0.0 { in_max } else { f32::INFINITY };
    let lo_bus = if regen_max >= 0.0 {
        -regen_max
    } else {
        f32::NEG_INFINITY
    };
    let (lo_iq, hi_iq) = if mod_q > 0.0 {
        (lo_bus / mod_q, hi_bus / mod_q)
    } else {
        (hi_bus / mod_q, lo_bus / mod_q)
    };
    clamp_f32(iq, lo_iq, hi_iq)
}

fn legacy_grant(driver: &GrantDriver, id_target: f32, iq_target: f32, omega: f32) -> (f32, f32) {
    let limits = &driver.current_limits;
    if !limits.is_enabled() {
        return (id_target, legacy_bus_clamp(driver, iq_target));
    }
    if iq_target * omega >= 0.0 {
        let lim = limits.max_current_a
            * driver
                .derating()
                .drive
                .min(driver.derating_cfg().speed_cut(omega));
        return (
            id_target,
            legacy_bus_clamp(driver, clamp_f32(iq_target, -lim, lim)),
        );
    }
    let brake = driver.derating().brake;
    let model = driver
        .controller
        .decoupling()
        .filter(|_| limits.max_dissipation_w > 0.0)
        .and_then(|d| {
            (d.resistance_ohm > 0.0 && d.flux_linkage_wb > 0.0).then_some(WindingModel {
                r_ohm: d.resistance_ohm,
                flux_wb: d.flux_linkage_wb,
                saliency_h: (d.lq_h - d.ld_h).abs(),
            })
        });
    let Some(model) = model else {
        let lim = limits.max_brake_a * brake.min(driver.derating().regen);
        return (
            id_target,
            legacy_bus_clamp(driver, clamp_f32(iq_target, -lim, lim)),
        );
    };
    let lim = limits.max_brake_a * brake;
    let (id_target, iq) = limits.clamp_targets(id_target, clamp_f32(iq_target, -lim, lim));
    let budget = RegenBudget {
        frac: driver.derating().regen,
        max_w: if limits.bus_regen_max_a >= 0.0 {
            driver.vbus * limits.bus_regen_max_a
        } else {
            f32::INFINITY
        },
    };
    let id_max = sqrtf(limits.max_dissipation_w * brake / (1.5 * model.r_ohm));
    match dissipate_with_id(
        id_target,
        iq,
        omega,
        &model,
        &budget,
        id_max,
        limits.max_current_a,
    ) {
        Some((id, iq)) if id * id > id_target * id_target => (id, iq),
        Some((_, iq)) => (id_target, iq),
        None => (id_target, legacy_bus_clamp(driver, iq)),
    }
}

fn check(driver: &GrantDriver, id: f32, iq: f32, omega: f32) -> (f32, f32) {
    let actual = driver.grant_torque(id, iq, omega);
    let constraints = driver.torque_constraints(omega);
    let constrained = driver.grant_torque_constrained(id, iq, &constraints);
    let expected = legacy_grant(driver, id, iq, omega);
    for (actual, expected) in [
        (actual.0, expected.0),
        (actual.1, expected.1),
        (constrained.0, expected.0),
        (constrained.1, expected.1),
    ] {
        assert!(
            (actual.is_nan() && expected.is_nan()) || actual.to_bits() == expected.to_bits(),
            "id={id:?}, iq={iq:?}, omega={omega:?}: {actual:?} != {expected:?}"
        );
    }
    actual
}

#[test]
fn local_constraints_keep_ieee_identity_and_refresh_each_cycle() {
    assert_eq!(size_of::<protection::TorqueConstraints>(), 20);
    let mut driver = grant_driver(
        CurrentLimits::from_max_current(0.0),
        DeratingScales::IDENTITY,
    );
    driver.bus_mod_q_filt = 0.5;
    for iq in [
        -0.0_f32,
        0.0,
        f32::NEG_INFINITY,
        f32::INFINITY,
        f32::from_bits(0x7fc0_1234),
    ] {
        let constraints = driver.torque_constraints(0.0);
        assert_eq!(
            driver
                .grant_torque_constrained(0.0, iq, &constraints)
                .1
                .to_bits(),
            iq.to_bits()
        );
    }
    driver.set_current_limits(CurrentLimits {
        bus_in_max_a: 1.0,
        bus_regen_max_a: 0.0,
        ..CurrentLimits::from_max_current(10.0)
    });
    for (modulation, speed) in [(0.25, 100.0), (-0.5, -100.0), (0.0, 0.0)] {
        driver.bus_mod_q_filt = modulation;
        let constraints = driver.torque_constraints(speed);
        for iq in [-10.0, 10.0, 3.0] {
            let actual = driver.grant_torque_constrained(0.0, iq, &constraints);
            let expected = legacy_grant(&driver, 0.0, iq, speed);
            assert_eq!(
                (actual.0.to_bits(), actual.1.to_bits()),
                (expected.0.to_bits(), expected.1.to_bits())
            );
        }
    }
}

#[test]
fn outlining_brake_solve_preserves_all_torque_grant_branches() {
    let values = [
        f32::NEG_INFINITY,
        -30.0,
        -10.0,
        -2.0,
        -0.0,
        0.0,
        2.0,
        30.0,
        f32::INFINITY,
        f32::NAN,
    ];
    let mut burned = false;
    let mut brake_reduced = false;
    let mut plugging = false;
    for max_a in [0.0, 3.0, 30.0] {
        for heat in [0.0, 20.0, 100.0] {
            for regen in [-1.0, 0.0, 0.1] {
                for saliency in [0.0, 8.0e-6, -8.0e-6] {
                    let mut driver = grant_driver(
                        CurrentLimits {
                            bus_in_max_a: 1.0,
                            bus_regen_max_a: regen,
                            max_dissipation_w: heat,
                            ..CurrentLimits::from_max_current(max_a)
                        },
                        DeratingScales {
                            drive: 0.7,
                            brake: 0.8,
                            regen: 0.2,
                        },
                    );
                    driver.controller.set_decoupling(Some(Decoupling {
                        ld_h: 22.0e-6,
                        lq_h: 22.0e-6 + saliency,
                        flux_linkage_wb: 2.92e-3,
                        resistance_ohm: 0.036,
                    }));
                    for modulation in [
                        -0.9_f32, -0.001, -0.000_999, -0.0, 0.0, 0.000_999, 0.001, 0.9,
                    ] {
                        driver.bus_mod_q_filt = modulation;
                        for omega in [-400.0, -50.0, -0.0, 0.0, 50.0, 400.0] {
                            for iq in values {
                                let (id, granted) = check(&driver, 0.0, iq, omega);
                                burned |= id > 0.0;
                                brake_reduced |=
                                    iq.is_finite() && iq * omega < 0.0 && granted.abs() < iq.abs();
                                plugging |= heat > 0.0
                                    && omega == 50.0
                                    && id == 0.0
                                    && iq == -10.0
                                    && granted == iq;
                                check(&driver, 4.0, iq, omega);
                                check(&driver, -30.0, iq, omega);
                            }
                        }
                    }
                }
            }
        }
    }
    assert!(
        burned && brake_reduced && plugging,
        "exercise burning, limited regeneration, and plugging"
    );
}

#[test]
fn outlined_grant_matches_legacy_for_wide_input_bits_and_missing_models() {
    let mut seed = 0x8765_4321_u32;
    let mut next = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed
    };
    let mut driver = grant_driver(
        CurrentLimits::from_max_current(30.0),
        DeratingScales::IDENTITY,
    );
    for case in 0..4096 {
        driver.current_limits.max_dissipation_w = [0.0, 20.0, 100.0][case % 3];
        driver.current_limits.bus_regen_max_a = [-1.0, 0.0, 0.2][case % 3];
        driver.current_limits.bus_in_max_a = if case % 2 == 0 { -1.0 } else { 1.0 };
        driver.bus_mod_q_filt = f32::from_bits(next());
        driver.controller.set_decoupling(match case % 5 {
            0 => None,
            variant => Some(Decoupling {
                ld_h: 22e-6,
                lq_h: 30e-6,
                flux_linkage_wb: if variant == 1 { 0.0 } else { 2.92e-3 },
                resistance_ohm: if variant == 2 { f32::NAN } else { 0.036 },
            }),
        });
        check(
            &driver,
            f32::from_bits(next()),
            f32::from_bits(next()),
            f32::from_bits(next()),
        );
    }
}

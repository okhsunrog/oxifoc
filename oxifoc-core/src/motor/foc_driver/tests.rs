use super::*;
#[cfg(feature = "virtual-motor")]
use crate::foc::angle_difference;
#[cfg(feature = "virtual-motor")]
use crate::foc::clamp_f32;
#[cfg(feature = "runtime")]
use crate::foc::config::BoardConfig;
use crate::foc::controller::Decoupling;
#[cfg(feature = "virtual-motor")]
use crate::foc::controller::FocOutput;
use crate::foc::current_offset::{CurrentOffsetMethod, CurrentOffsetRequest};
#[cfg(feature = "virtual-motor")]
use crate::foc::hall_sensor::HallSensor;
#[cfg(feature = "virtual-motor")]
use crate::foc::phase::{BackEmfObserver, Observer};
use crate::foc::phase::{PhaseManager, PhaseSource};
use crate::foc::pwm::PhaseState;
use crate::foc::sensors::NoSensor;
#[cfg(feature = "virtual-motor")]
use crate::foc::transforms::{clarke, park};
#[cfg(feature = "virtual-motor")]
use crate::foc::velocity::VelocityLoopConfig;
#[cfg(feature = "virtual-motor")]
use crate::foc::wrap_angle;
#[cfg(feature = "virtual-motor")]
use crate::motor::failsafe::{FailsafePolicy, FailsafeTerminal};
#[cfg(feature = "runtime")]
use crate::state::CMD_CHANNEL;
#[cfg(feature = "storage")]
use crate::storage::CurrentLimitsConfig;
#[cfg(feature = "virtual-motor")]
use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};
#[cfg(feature = "virtual-motor")]
use core::f32::consts::{FRAC_PI_3, FRAC_PI_6};

/// A driver for exercising the torque grant, which reads no plant
/// state — only the limits, the derating, the motor model and vbus.
fn grant_driver(
    limits: CurrentLimits,
    scales: DeratingScales,
) -> FocDriver<MockPwm, MockCurrentSensor, PhaseManager<NoSensor, NoSensor>> {
    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    foc.set_decoupling(Some(Decoupling {
        ld_h: 22e-6,
        lq_h: 22e-6,
        flux_linkage_wb: 2.92e-3,
        resistance_ohm: 0.036,
    }));
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_current_limits(limits);
    driver.set_derating_scales(scales);
    driver
}

#[test]
fn grant_derates_drive_and_brake_by_direction() {
    let iq = |d: &FocDriver<_, _, _>, iq, omega| d.grant_torque(0.0, iq, omega).1;
    // Cruising forward at the ceiling: motoring budget cut to 10%,
    // braking untouched — the velocity PI's positive bound must shrink,
    // the negative must not (this asymmetry is what kills the ceiling
    // windup: the integral can no longer bank the un-grantable 90%).
    let d = grant_driver(
        CurrentLimits::from_max_current(10.0),
        DeratingScales {
            drive: 0.1,
            ..DeratingScales::IDENTITY
        },
    );
    assert_eq!((iq(&d, -10.0, 500.0), iq(&d, 10.0, 500.0)), (-10.0, 1.0));
    // Reverse motion mirrors the orientation.
    assert_eq!((iq(&d, -10.0, -500.0), iq(&d, 10.0, -500.0)), (-1.0, 10.0));
    // Standstill: either way is pulling away — motoring.
    assert_eq!((iq(&d, -10.0, 0.0), iq(&d, 10.0, 0.0)), (-1.0, 1.0));
    // The braking side has its own limit and its own derate.
    let d = grant_driver(
        CurrentLimits {
            max_brake_a: 4.0,
            ..CurrentLimits::from_max_current(10.0)
        },
        DeratingScales {
            brake: 0.5,
            ..DeratingScales::IDENTITY
        },
    );
    assert_eq!((iq(&d, -10.0, 500.0), iq(&d, 10.0, 500.0)), (-2.0, 10.0));
    assert_eq!((iq(&d, -10.0, -500.0), iq(&d, 10.0, -500.0)), (-10.0, 2.0));
    // Unlimited when the limit is unset — mirrors CurrentLimits.
    let d = grant_driver(
        CurrentLimits::from_max_current(0.0),
        DeratingScales::IDENTITY,
    );
    assert!(iq(&d, f32::INFINITY, 100.0).is_infinite());
}

/// A full pack (regen derate at 0) on a 5065 at 400 el rad/s: without
/// dissipation the brake is cut to nothing; with it the brake keeps
/// its torque and a negative d current burns the regen.
#[test]
fn a_full_pack_brake_burns_what_it_cannot_charge() {
    let full_pack = DeratingScales {
        regen: 0.0,
        ..DeratingScales::IDENTITY
    };
    let off = grant_driver(CurrentLimits::from_max_current(30.0), full_pack);
    assert_eq!(off.grant_torque(0.0, -10.0, 400.0), (0.0, 0.0));
    let on = grant_driver(
        CurrentLimits {
            max_dissipation_w: 100.0,
            ..CurrentLimits::from_max_current(30.0)
        },
        full_pack,
    );
    let (id, iq) = on.grant_torque(0.0, -10.0, 400.0);
    assert_eq!(iq, -10.0);
    // 1.5·R·(id² + iq²) = 1.5·ω·λ·|iq|: id² = (1.168·10 − 3.6)/0.036
    // — 12.1 W of the 100 W budget.
    assert!((id + 14.98).abs() < 0.05, "id {id}");
    // Motoring and plugging are left alone.
    assert_eq!(on.grant_torque(0.0, 10.0, 400.0), (0.0, 10.0));
    assert_eq!(on.grant_torque(0.0, -10.0, 50.0), (0.0, -10.0));
    // A hot motor burns less: the budget follows the brake derate —
    // 30% of 20 W buys 10.5 A of id, which balances ~3.9 A of brake.
    let hot = grant_driver(
        CurrentLimits {
            max_dissipation_w: 20.0,
            ..CurrentLimits::from_max_current(30.0)
        },
        DeratingScales {
            brake: 0.3,
            ..full_pack
        },
    );
    let (id, iq) = hot.grant_torque(0.0, -10.0, 400.0);
    let heat_w = 1.5 * 0.036 * id * id;
    assert!((heat_w - 6.0).abs() < 0.01, "{heat_w} W");
    assert!(iq > -4.5 && iq < -3.5, "iq {iq}");
}

/// A lab PSU (`bus_regen_max_a = 0`) takes no regen: dissipation turns
/// the brake's regen into heat instead of cutting the torque.
#[test]
fn a_no_regen_supply_brakes_by_dissipation() {
    let d = grant_driver(
        CurrentLimits {
            bus_regen_max_a: 0.0,
            max_dissipation_w: 100.0,
            ..CurrentLimits::from_max_current(30.0)
        },
        DeratingScales::IDENTITY,
    );
    let (id, iq) = d.grant_torque(0.0, -10.0, 400.0);
    assert_eq!(iq, -10.0);
    assert!((id + 14.98).abs() < 0.05, "id {id}");
}

struct MockPwm {
    duties: [u16; 3],
}

impl PhasePwm for MockPwm {
    fn max_duty(&self) -> u16 {
        1000
    }
    fn set_duties(&mut self, duties: [u16; 3]) {
        self.duties = duties;
    }
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        self.duties = states.map(|state| match state {
            PhaseState::Pwm(duty) => duty,
            PhaseState::Low | PhaseState::Float => 0,
        });
    }
}

/// PWM that models the boards' channel gating: `disable()` and a
/// `Float` phase turn channels off, `set_duties` never turns them on —
/// only `enable()` or a driven `set_phase_states` does.
struct GatedPwm {
    enabled: bool,
    applied: [u16; 3],
}

impl PhasePwm for GatedPwm {
    fn max_duty(&self) -> u16 {
        1000
    }
    fn set_duties(&mut self, duties: [u16; 3]) {
        if self.enabled {
            self.applied = duties;
        }
    }
    fn disable(&mut self) {
        self.enabled = false;
        self.applied = [0; 3];
    }
    fn enable(&mut self) {
        self.enabled = true;
    }
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        self.enabled = !states.iter().all(|s| matches!(s, PhaseState::Float));
        self.applied = states.map(|state| match state {
            PhaseState::Pwm(duty) => duty,
            PhaseState::Low | PhaseState::Float => 0,
        });
    }
}

/// Every mode that drives the bridge through duties must re-enable it,
/// whatever left it off — not only Stopped. Coast → CurrentControl used
/// to write duties into the floated (disabled) bridge forever.
#[test]
fn driving_modes_reenable_the_bridge_after_coast_and_stop() {
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator>::new(24.0),
        GatedPwm {
            enabled: false,
            applied: [0; 3],
        },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.phase_mut().set_manual_angle(0.0);
    driver.phase_mut().set_source(PhaseSource::Manual).unwrap();
    let drive = ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    };
    for (step, from) in [ControlMode::Stopped, ControlMode::Coast]
        .into_iter()
        .enumerate()
    {
        driver.set_mode(from);
        let t = (step as u64 + 1) * 1_000;
        let _ = driver.step(t);
        assert!(!driver.pwm().enabled, "{from:?} must leave the bridge off");
        driver.set_mode(drive);
        let _ = driver.step(t + 50);
        assert!(
            driver.pwm().enabled && driver.pwm().applied != [0; 3],
            "{from:?} → CurrentControl must drive the bridge"
        );
    }
    // A user Brake while coasting: the brake ramp needs the bridge.
    driver.set_mode(ControlMode::Coast);
    let _ = driver.step(10_000);
    assert!(driver.enter_brake_ramp());
    assert!(
        driver.pwm().enabled,
        "the brake ramp must enable the bridge"
    );
}

struct MockCurrentSensor {
    currents: (f32, f32, f32),
}

impl CurrentSensor for MockCurrentSensor {
    fn read_currents(&self) -> (f32, f32, f32) {
        self.currents
    }
    fn read_raw(&self) -> (u16, u16, u16) {
        (0, 0, 0)
    }
    fn is_calibrated(&self) -> bool {
        true
    }
    fn get_offsets(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
    fn set_offsets(&mut self, _offsets: (f32, f32, f32)) {}
}

/// Phase-1 wiring: on a sensing board, a high-Z (`Coast`) bridge must feed
/// the *measured* terminal voltage to the back-EMF observer, so it locks
/// onto a free-spinning rotor (flying-start readiness). Mirrors MESC
/// `MOTOR_STATE_TRACKING` / VESC released-motor tracking. Synthetic coasting
/// BEMF is injected as raw ADC counts via `set_phase_voltage_raw`. A
/// no-sensing control board gets the same samples and must ignore them.
#[test]
fn coast_observer_tracks_measured_bemf() {
    use crate::foc::config::{BoardConfig, PhaseSense};
    use crate::foc::phase::{BackEmfObserver, Observer};
    use crate::foc::phase_voltage::PhaseVoltageSense;
    use crate::foc::wrap_angle;

    const BASE_BOARD: BoardConfig = BoardConfig {
        calib: crate::types::BoardCalib {
            shunt_ohms: 0.0005,
            amp_gain: 10.0,
            adc_vref_mv: 3300,
            adc_max_counts: 4095,
            invert_current_sign: false,
            vbus_divider_ratio: (39.0 + 2.2) / 2.2,
        },
        max_iq_target_a: 10.0,
        max_phase_current_a: 60.0,
        max_vbus_mv: 57_000,
        min_vbus_mv: 6_000,
        max_fet_temp_c: 100.0,
        max_motor_temp_c: 120.0,
        phase_sense: None,
    };
    const SENSE_BOARD: BoardConfig = BoardConfig {
        phase_sense: Some(PhaseSense {
            divider_ratio: (39.0 + 2.2) / 2.2,
            has_filters: false,
        }),
        ..BASE_BOARD
    };
    const RATIO: f32 = (39.0 + 2.2) / 2.2;
    // ADC counts per volt at the *terminal* (inverse of counts_to_volts).
    const COUNTS_PER_V: f32 = 4095.0 * 1000.0 / (3300.0 * RATIO);
    const SQRT3_2: f32 = 0.866_025_4;

    // Synthetic PMSM, same operating point as the observer's own no-load
    // convergence test.
    let (r, l, lambda) = (0.1f32, 50e-6f32, 0.005f32);
    let omega = 300.0f32; // electrical rad/s
    let dt = 1.0 / 20_000.0f32;

    let build = |board: &BoardConfig| {
        let mut pm = PhaseManager::sensorless();
        pm.set_observer(Observer::BackEmf(BackEmfObserver::new(r, l, lambda)));
        let mut d = FocDriver::new(
            FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            pm,
            dt,
        );
        d.set_phase_voltage_sense(PhaseVoltageSense::from_board(board));
        d.set_mode(ControlMode::Coast);
        d
    };
    let mut sensing = build(&SENSE_BOARD);
    let mut blind = build(&BASE_BOARD); // phase_sense: None

    let mut theta = 1.0f32;
    for step in 0..40_000u64 {
        theta = wrap_angle(theta + omega * dt);
        // Undriven terminals = back-EMF: e = ωλ·(−sinθ, cosθ) in αβ.
        let v_alpha = -omega * lambda * libm::sinf(theta);
        let v_beta = omega * lambda * libm::cosf(theta);
        // Inverse Clarke → per-phase terminal volts, plus a mid-scale common
        // mode (the 3-input Clarke in alpha_beta cancels it, and it keeps the
        // raw counts non-negative like real hardware).
        let va = v_alpha;
        let vb = -0.5 * v_alpha + SQRT3_2 * v_beta;
        let vc = -0.5 * v_alpha - SQRT3_2 * v_beta;
        let to_raw = |v: f32| (2048.0 + v * COUNTS_PER_V) as u16;
        let raw = [to_raw(va), to_raw(vb), to_raw(vc)];

        sensing.set_phase_voltage_raw(Some(raw));
        blind.set_phase_voltage_raw(Some(raw)); // ignored: phase_voltage is None
        let _ = sensing.step(step);
        let _ = blind.step(step);
    }

    let obs = sensing.phase().observer();
    assert!(
        (obs.velocity().unwrap_or(0.0) - omega).abs() < 0.05 * omega,
        "coast observer velocity {:?} did not lock to {omega}",
        obs.velocity()
    );
    assert!(obs.is_ready(), "coast observer should be ready (locked)");

    // The no-sensing board got the same samples but must ignore them
    // (commanded path = 0 V in Coast): no flux, no lock.
    let blind_obs = blind.phase().observer();
    // Fed 0 V in Coast (commanded path), it integrates no flux — velocity
    // stays at standstill, well below the observer's readiness floor.
    assert!(
        blind_obs.velocity().unwrap_or(0.0).abs() < 30.0,
        "no-sensing board must not track from phase samples, got {:?}",
        blind_obs.velocity()
    );
    assert!(!blind_obs.is_ready());
}

/// CMD_CHANNEL and FLASH_OP_PENDING are process-wide globals — tests
/// that touch them must not run concurrently. Also drains any stale
/// commands a previous test left behind.
#[cfg(feature = "runtime")]
fn cmd_channel_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let guard = LOCK
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    while CMD_CHANNEL.try_receive().is_ok() {}
    guard
}

/// SetPhaseSource through the command channel: a valid source switches
/// the manager (and mirrors into shared state), an invalid one is
/// rejected and leaves everything unchanged.
#[test]
#[cfg(feature = "runtime")]
#[cfg(feature = "hfi")]
fn process_commands_switches_phase_source() {
    let _serial = cmd_channel_lock();
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    #[derive(Clone, Copy, PartialEq)]
    struct TestFault;
    impl PlatformFault for TestFault {
        fn category(&self) -> FaultCategory {
            FaultCategory::OverCurrent
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
    }

    let state: CriticalSectionMutex<RefCell<MotorControlState>> =
        CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
    let registry: FaultRegistry<TestFault> = FaultRegistry::new();

    let mut mgr = PhaseManager::sensorless();
    mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        1.0 / 20_000.0,
    );
    // The driver must stay linked so process_commands doesn't force Stopped.
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

    // Valid switch: HFI estimator is configured.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hfi));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(driver.phase().source(), PhaseSource::Hfi);
    let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
    assert_eq!(mirrored, PhaseSource::Hfi);

    // Invalid switch: no hall sensor on a sensorless manager.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetPhaseSource(PhaseSource::Hall));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.phase().source(),
        PhaseSource::Hfi,
        "invalid source must be rejected, not applied"
    );
    let mirrored = critical_section::with(|cs| state.borrow(cs).borrow().phase_source);
    assert_eq!(mirrored, PhaseSource::Hfi);
}

/// Non-finite numbers must die at the command boundary: a NaN target
/// reaching the PI loop turns the SVPWM output into a garbage voltage
/// vector (bounded by the saturating casts, but still a mechanical
/// jolt until the overcurrent check reacts).
#[test]
#[cfg(feature = "runtime")]
fn process_commands_rejects_non_finite_payloads() {
    let _serial = cmd_channel_lock();
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    #[derive(Clone, Copy, PartialEq)]
    struct TestFault;
    impl PlatformFault for TestFault {
        fn category(&self) -> FaultCategory {
            FaultCategory::OverCurrent
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
    }

    let state: CriticalSectionMutex<RefCell<MotorControlState>> =
        CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
    let registry: FaultRegistry<TestFault> = FaultRegistry::new();
    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

    // NaN current target must be dropped — driver stays Stopped.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: f32::NAN,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.mode(),
        ControlMode::Stopped,
        "NaN current target must be rejected"
    );

    // Infinite direct voltage likewise.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::DirectVoltage {
        vd: f32::INFINITY,
        vq: 0.0,
        angle_rad: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(driver.mode(), ControlMode::Stopped);

    // NaN PI gains must not reach the controller.
    let gains_before = driver.controller().id_pi.gains();
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetPiGains {
        kp: f32::NAN,
        ki: 100.0,
    });
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.controller().id_pi.gains(),
        gains_before,
        "NaN gains must be rejected"
    );

    // A finite command still works.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 1.0,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.mode(),
        ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0
        }
    );
}

/// Motor start must be refused while a flash operation is in flight:
/// internal-flash erase stalls the chip, which must never overlap an
/// energized motor. This is the ISR half of the config server's
/// Busy-gate TOCTOU fix (the server arms FLASH_OP_PENDING before
/// re-checking the motor state).
#[test]
#[cfg(feature = "runtime")]
fn process_commands_blocks_start_during_flash_op() {
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::state::{
        CMD_CHANNEL, DriverCommand, FlashPendingGuard, MotorControlState, process_commands,
    };
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    let _serial = cmd_channel_lock();

    #[derive(Clone, Copy, PartialEq)]
    struct TestFault;
    impl PlatformFault for TestFault {
        fn category(&self) -> FaultCategory {
            FaultCategory::OverCurrent
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
    }

    let state: CriticalSectionMutex<RefCell<MotorControlState>> =
        CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
    let registry: FaultRegistry<TestFault> = FaultRegistry::new();
    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

    let run_cmd = DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 1.0,
        id_target: 0.0,
    });

    // Flash op in flight: the start must be refused.
    let pending = FlashPendingGuard::arm();
    let _ = CMD_CHANNEL.try_send(run_cmd);
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.mode(),
        ControlMode::Stopped,
        "start must be blocked while a flash operation is pending"
    );

    // Stop must stay allowed mid-operation — it is the safe direction.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(driver.mode(), ControlMode::Stopped);

    // Flag cleared (guard dropped): the same start goes through.
    drop(pending);
    let _ = CMD_CHANNEL.try_send(run_cmd);
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.mode(),
        ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0
        }
    );
}

/// Link loss must route through the configured failsafe policy (arm the
/// controller), not the legacy instant hard-Stop — `process_commands`
/// forces this whenever `link_active` is false and the motor is running.
/// The re-arm latch survives INTERNAL stops: the fault-kill `safe_off`
/// and step-error paths route through `set_mode(Stopped)`, and clearing
/// the latch there let a failsafe brake that tripped OC/OV mid-stop be
/// relaunched by a stale throttle replay right after `faults clear` --
/// with no "throttle back to neutral". Only the explicit host-side
/// acknowledgement may release it.
#[test]
fn internal_stops_preserve_the_failsafe_latch() {
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );

    driver.enter_failsafe();
    assert!(driver.failsafe_latched());

    // Kill path: safe_off every cycle while the fault stays latched.
    driver.safe_off();
    assert!(
        driver.failsafe_latched(),
        "safe_off must not release the re-arm latch"
    );

    // Step-error path: a direct internal Stopped.
    driver.set_mode(ControlMode::Stopped);
    assert!(
        driver.failsafe_latched(),
        "an internal set_mode(Stopped) must not release the re-arm latch"
    );

    // The host-command boundary is the only release point.
    driver.acknowledge_failsafe();
    assert!(!driver.failsafe_latched());
}

#[test]
#[cfg(feature = "runtime")]
fn link_loss_arms_failsafe_policy() {
    let _serial = cmd_channel_lock();
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};
    use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    #[derive(Clone, Copy, PartialEq)]
    struct TestFault;
    impl PlatformFault for TestFault {
        fn category(&self) -> FaultCategory {
            FaultCategory::OverCurrent
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
    }

    let state: CriticalSectionMutex<RefCell<MotorControlState>> =
        CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
    let registry: FaultRegistry<TestFault> = FaultRegistry::new();
    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_failsafe(FailsafeConfig {
        policy: FailsafePolicy::ControlledStop,
        ..FailsafeConfig::default()
    });

    // Link up, start running.
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert!(!driver.failsafe_active());

    // Link drops → next drain arms the failsafe controller (not a hard Stop).
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_inactive());
    process_commands(&state, &mut driver, &registry);
    assert!(
        driver.failsafe_active(),
        "link loss must arm the configured failsafe policy"
    );

    // …and latches: with the link back, a replayed running setpoint (a
    // reconnecting host's stale throttle) must be rejected until an
    // explicit safe-mode acknowledgement.
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
    assert!(driver.failsafe_latched());
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 5.0,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert!(
        driver.failsafe_active(),
        "running mode while latched must be rejected, not cancel the brake"
    );
    assert!(driver.failsafe_latched());

    // Stopped acknowledges ("throttle back to neutral"): latch clears…
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
    process_commands(&state, &mut driver, &registry);
    assert!(!driver.failsafe_latched());
    assert!(!driver.failsafe_active());
    assert_eq!(driver.mode(), ControlMode::Stopped);

    // …and a fresh running command is accepted again.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);
    assert!(matches!(driver.mode(), ControlMode::CurrentControl { .. }));
}

/// Bench/calibration modes are exempt from the deadman: on-device
/// detection dwells up to ~1 s between `SetMode`s, which a 150 ms
/// deadman would cut mid-measurement. Drive modes stay covered.
#[test]
fn deadman_exempts_bench_modes() {
    use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_failsafe(FailsafeConfig {
        policy: FailsafePolicy::Coast,
        staleness_timeout_us: 1_000,
        ..FailsafeConfig::default()
    });
    driver.note_command_tick(0);

    for mode in [
        ControlMode::OpenLoop {
            angle_rad: 0.0,
            current: 3.0,
            velocity_rad_s: 0.0,
            pi_gains: None,
        },
        ControlMode::DirectVoltage {
            vd: 1.0,
            vq: 0.0,
            angle_rad: 0.0,
        },
    ] {
        driver.set_mode(mode);
        assert!(
            !driver.deadman_expired(1_000_000),
            "bench mode {mode:?} must be deadman-exempt"
        );
    }

    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    });
    assert!(
        driver.deadman_expired(1_000_000),
        "drive mode stays covered"
    );
}

/// `safe_off()` must float the bridge *immediately*, not defer it to the
/// next `step()`. The Kill path returns without ever stepping, and a
/// software-only Kill (integrated OverVoltage) has no hardware break, so
/// `set_mode(Stopped)` alone would leave the phases energized at the last
/// commanded duty.
#[test]
fn safe_off_floats_bridge_immediately() {
    /// MockPwm that tracks whether the bridge is energized.
    struct OffPwm {
        enabled: bool,
    }
    impl PhasePwm for OffPwm {
        fn max_duty(&self) -> u16 {
            1000
        }
        fn set_duties(&mut self, _duties: [u16; 3]) {}
        fn disable(&mut self) {
            self.enabled = false;
        }
        fn enable(&mut self) {
            self.enabled = true;
        }
        fn set_phase_states(&mut self, states: [PhaseState; 3]) {
            self.enabled = states
                .iter()
                .any(|state| !matches!(state, PhaseState::Float));
        }
    }

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        OffPwm { enabled: true },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );

    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 1.0,
        id_target: 0.0,
    });
    assert!(driver.pwm().enabled, "a running mode arms the bridge");

    // The regression: set_mode(Stopped) alone does NOT cut the gate
    // drive (the safe-off lives in step()'s Stopped arm, which the Kill
    // path skips).
    driver.set_mode(ControlMode::Stopped);
    assert!(
        driver.pwm().enabled,
        "set_mode(Stopped) alone must not be relied on to float the bridge"
    );

    driver.safe_off();
    assert!(
        !driver.pwm().enabled,
        "safe_off must float the bridge immediately"
    );
    assert_eq!(driver.mode(), ControlMode::Stopped);

    driver.acknowledge_failsafe();
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 1.0,
        id_target: 0.0,
    });
    driver.emergency_stop();
    assert!(!driver.pwm().enabled);
    assert_eq!(driver.mode(), ControlMode::Stopped);
    assert!(
        driver.failsafe_latched(),
        "emergency stop must require an explicit safe re-arm command"
    );
}

#[test]
fn current_offset_calibration_applies_then_floats() {
    struct CalibrationPwm {
        states: [PhaseState; 3],
    }
    impl Default for CalibrationPwm {
        fn default() -> Self {
            Self {
                states: [PhaseState::Float; 3],
            }
        }
    }
    impl PhasePwm for CalibrationPwm {
        fn max_duty(&self) -> u16 {
            1000
        }
        fn set_duties(&mut self, duties: [u16; 3]) {
            self.states = duties.map(PhaseState::Pwm);
        }
        fn set_phase_states(&mut self, states: [PhaseState; 3]) {
            self.states = states;
        }
    }

    struct CalibrationSensor {
        offsets: (f32, f32, f32),
    }
    impl CurrentSensor for CalibrationSensor {
        fn read_currents(&self) -> (f32, f32, f32) {
            (0.0, 0.0, 0.0)
        }
        fn read_raw(&self) -> (u16, u16, u16) {
            (111, 222, 333)
        }
        fn is_calibrated(&self) -> bool {
            self.offsets != (0.0, 0.0, 0.0)
        }
        fn get_offsets(&self) -> (f32, f32, f32) {
            self.offsets
        }
        fn set_offsets(&mut self, offsets: (f32, f32, f32)) {
            self.offsets = offsets;
        }
    }

    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        CalibrationPwm::default(),
        CalibrationSensor {
            offsets: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver
        .start_current_offset_calibration(CurrentOffsetRequest {
            method: CurrentOffsetMethod::PerPhase50,
            samples_per_channel: 16,
            settle_cycles: 0,
            apply: true,
            stationary_confirmed: false,
        })
        .unwrap();

    for tick in 0..64 {
        let _ = driver.step(tick);
    }
    let report = driver.take_current_offset_outcome().unwrap().unwrap();
    assert_eq!(report.offsets, [111.0, 222.0, 333.0]);
    assert_eq!(driver.current_sensor().get_offsets(), (111.0, 222.0, 333.0));
    assert_eq!(driver.pwm().states, [PhaseState::Float; 3]);
    assert!(!driver.current_offset_active());
}

#[test]
fn all_phase_offset_requires_explicit_stationary_confirmation() {
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    let error = driver
        .start_current_offset_calibration(CurrentOffsetRequest {
            method: CurrentOffsetMethod::AllPhases50,
            samples_per_channel: 16,
            settle_cycles: 0,
            apply: false,
            stationary_confirmed: false,
        })
        .unwrap_err();
    assert_eq!(error, CurrentOffsetError::UnsafeWhileMoving);
    assert!(!driver.current_offset_active());
}

/// Brake (windings short) is speed-gated at the command boundary, and
/// once engaged it is a safe standing state: exempt from the deadman and
/// from the link-loss failsafe — a parked board must stay braked.
#[test]
#[cfg(feature = "runtime")]
fn brake_speed_gated_and_survives_link_loss() {
    let _serial = cmd_channel_lock();
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::phase::{PhaseInput, PhaseOutput, PhaseProvider};
    use crate::foc::trig::LibmSinCos;
    use crate::state::{CMD_CHANNEL, DriverCommand, MotorControlState, process_commands};
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    #[derive(Clone, Copy, PartialEq)]
    struct TestFault;
    impl PlatformFault for TestFault {
        fn category(&self) -> FaultCategory {
            FaultCategory::OverCurrent
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
    }

    /// Phase provider with a directly settable velocity estimate.
    struct MockPhase {
        vel: f32,
    }
    impl PhaseProvider for MockPhase {
        fn get(&self) -> PhaseOutput {
            PhaseOutput {
                angle: 0.0,
                velocity: self.vel,
            }
        }
        fn update(&mut self, _input: &PhaseInput, _now_ticks: u64) {}
    }

    let state: CriticalSectionMutex<RefCell<MotorControlState>> =
        CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
    let registry: FaultRegistry<TestFault> = FaultRegistry::new();
    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        MockPhase { vel: 300.0 },
        1.0 / 20_000.0,
    );

    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    }));
    process_commands(&state, &mut driver, &registry);

    // Spinning fast → Brake is rejected, mode unchanged.
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Brake));
    process_commands(&state, &mut driver, &registry);
    assert!(
        matches!(driver.mode(), ControlMode::CurrentControl { .. }),
        "Brake at speed must be rejected, got {:?}",
        driver.mode()
    );

    // Near standstill → accepted.
    driver.phase_mut().vel = 5.0;
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Brake));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(driver.mode(), ControlMode::Brake);

    // Safe standing state: no deadman, however stale the command link.
    assert!(!driver.deadman_expired(u64::MAX / 2));

    // Link loss must NOT kick a parked board out of the brake.
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_inactive());
    process_commands(&state, &mut driver, &registry);
    assert!(!driver.failsafe_active(), "brake is exempt from link-loss");
    assert_eq!(driver.mode(), ControlMode::Brake);

    // And the Brake step drives all-low-side without erroring.
    assert!(driver.step(1_000).is_ok());
}

#[test]
#[cfg(feature = "storage")]
fn config_limits_clamp_to_hardware_ceiling() {
    use CurrentLimitsConfig;
    // Stored config must never raise limits above the board's hardware
    // ceiling, and zero/negative config values (= "not set") fall back
    // to the board defaults instead of disabling protection.
    let hw_max = 10.0;
    let cfg = |iq: f32, ph: f32, bus_in: f32, bus_regen: f32| CurrentLimitsConfig {
        max_iq_a: iq,
        max_brake_a: 0.0,
        max_phase_current_a: ph,
        bus_in_max_a: bus_in,
        bus_regen_max_a: bus_regen,
        max_dissipation_w: 0.0,
    };

    // Normal config below the ceiling: passes through.
    let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, -1.0, -1.0), hw_max, 0.0);
    assert_eq!(l.max_current_a, 5.0);
    assert_eq!(l.overcurrent_threshold_a, 8.0);
    assert_eq!(l.bus_in_max_a, -1.0);
    assert_eq!(l.bus_regen_max_a, -1.0);

    // Config above the ceiling: clamped to the board limits. The board
    // value is the ABS trip line; the iq ceiling keeps the headroom
    // below it (NOT hw_max itself — that put full throttle exactly on
    // the per-phase Kill line).
    let l = CurrentLimits::from_config_clamped(&cfg(50.0, 100.0, -1.0, -1.0), hw_max, 0.0);
    assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
    assert_eq!(l.overcurrent_threshold_a, hw_max);

    // Zeroed config: board defaults, NOT disabled protection.
    let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0, -1.0, -1.0), hw_max, 0.0);
    assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
    assert_eq!(l.overcurrent_threshold_a, hw_max);

    // Bus limits pass through (no board ceiling — they protect the
    // supply); zero is meaningful (no regen), NaN → unlimited.
    let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, 20.0, 0.0), hw_max, 0.0);
    assert_eq!(l.bus_in_max_a, 20.0);
    assert_eq!(l.bus_regen_max_a, 0.0);
    let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0, f32::NAN, f32::NAN), hw_max, 0.0);
    assert_eq!(l.bus_in_max_a, -1.0);
    assert_eq!(l.bus_regen_max_a, -1.0);

    // The dissipation budget passes through (the current circle bounds
    // what it buys); unset, negative or NaN is off.
    let dissipation = |v: f32| {
        let c = CurrentLimitsConfig {
            max_dissipation_w: v,
            ..cfg(5.0, 8.0, -1.0, -1.0)
        };
        CurrentLimits::from_config_clamped(&c, hw_max, 0.0).max_dissipation_w
    };
    assert_eq!(dissipation(80.0), 80.0);
    assert_eq!(dissipation(0.0), 0.0);
    assert_eq!(dissipation(-5.0), 0.0);
    assert_eq!(dissipation(f32::NAN), 0.0);
}

/// The 2026-06-12 foot-gun (notes/fault-overhaul.md §4): a config that
/// places the overcurrent trip at (or inside the headroom band of) the
/// soft iq ceiling must not survive into the live limits — full
/// throttle plus PI overshoot / HFI ripple would nuisance-Kill
/// mid-ride. Protection wins over torque: iq is lowered, the trip is
/// never raised.
#[test]
#[cfg(feature = "storage")]
fn cross_field_headroom_enforced() {
    use CurrentLimitsConfig;
    // Board ABS line far above so it is not the binding constraint.
    let hw_max = 80.0;
    let l = CurrentLimits::from_config_clamped(
        &CurrentLimitsConfig {
            max_iq_a: 40.0,
            max_brake_a: 0.0,
            max_phase_current_a: 40.0,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
            max_dissipation_w: 0.0,
        },
        hw_max,
        0.0,
    );
    assert_eq!(l.overcurrent_threshold_a, 40.0, "trip is never raised");
    assert_eq!(
        l.max_current_a,
        40.0 / OVERCURRENT_HEADROOM,
        "iq must drop one headroom factor below the trip"
    );
    // The invariant holds whatever the inputs.
    assert!(l.overcurrent_threshold_a >= OVERCURRENT_HEADROOM * l.max_current_a);
}

/// Motor RATING is a ceiling above the operational config (layered
/// semantics): the session can only lower limits below it, an unset
/// config defaults to the rating itself, and the overcurrent trip
/// ceiling is 1.5× the rating (VESC `l_abs_current_max`) — all still
/// capped by the board hardware.
#[cfg(feature = "storage")]
#[test]
fn rating_caps_operational_limits() {
    let hw_max = 30.0;
    let cfg = |iq: f32, phase: f32| CurrentLimitsConfig {
        max_iq_a: iq,
        max_brake_a: 0.0,
        max_phase_current_a: phase,
        bus_in_max_a: -1.0,
        bus_regen_max_a: -1.0,
        max_dissipation_w: 0.0,
    };
    let rating = 10.0;

    // Operational asks for more than the motor tolerates: rating wins.
    let l = CurrentLimits::from_config_clamped(&cfg(25.0, 28.0), hw_max, rating);
    assert_eq!(l.max_current_a, rating);
    assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);

    // Operational below the rating: passes through untouched.
    let l = CurrentLimits::from_config_clamped(&cfg(5.0, 8.0), hw_max, rating);
    assert_eq!(l.max_current_a, 5.0);
    assert_eq!(l.overcurrent_threshold_a, 8.0);

    // Unset operational: the rating IS the default (VESC applies
    // l_current_max = i_max the same way).
    let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0), hw_max, rating);
    assert_eq!(l.max_current_a, rating);
    assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);

    // Board hardware still caps a huge rating: trip at the board ABS
    // line, iq one headroom factor below it.
    let l = CurrentLimits::from_config_clamped(&cfg(0.0, 0.0), hw_max, 500.0);
    assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
    assert_eq!(l.overcurrent_threshold_a, hw_max);

    // NaN / zero rating: no rating clamp (pre-rating config blobs).
    // 25/28 is an incoherent pair (28 < 1.3·25): the cross-field clamp
    // lowers iq to trip/headroom on top of the board iq ceiling.
    let l = CurrentLimits::from_config_clamped(&cfg(25.0, 28.0), hw_max, f32::NAN);
    assert_eq!(
        l.max_current_a,
        (28.0 / OVERCURRENT_HEADROOM).min(hw_max / OVERCURRENT_HEADROOM)
    );
    assert_eq!(l.overcurrent_threshold_a, 28.0);

    // No limits group stored at all: rating still holds at boot.
    let l = CurrentLimits::from_stored(None, hw_max, rating);
    assert_eq!(l.max_current_a, rating);
    assert_eq!(l.overcurrent_threshold_a, 1.5 * rating);
    // …and bus limits stay off (boot default).
    assert_eq!(l.bus_in_max_a, -1.0);
    assert_eq!(l.bus_regen_max_a, -1.0);

    // A fresh board: no limits group AND no motor rating. The iq budget
    // must still sit one headroom factor below the board's kill line —
    // it used to BE the kill line, with the dq trip 1.3× above it.
    let l = CurrentLimits::from_stored(None, hw_max, 0.0);
    assert_eq!(l.max_current_a, hw_max / OVERCURRENT_HEADROOM);
    assert_eq!(l.overcurrent_threshold_a, hw_max);
}

/// Bus (supply) current limits: `i_bus ≈ iq·mod_q`, so the allowed iq
/// window scales inversely with the modulation. With regen = 0 no
/// negative-power iq is allowed at all (lab-PSU safety); with both
/// limits negative (the default) nothing is clamped.
#[test]
fn bus_limit_clamps_iq() {
    use crate::foc::trig::LibmSinCos;

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );

    // Default: unlimited, pass-through whatever the modulation.
    driver.bus_mod_q_filt = 0.5;
    assert_eq!(driver.clamp_iq_for_bus(-30.0), -30.0);
    assert_eq!(driver.clamp_iq_for_bus(30.0), 30.0);

    // bus_in 10 A, regen 0, mod_q = 0.5 (driving forward at half duty):
    // draw bound = 10/0.5 = 20 A, regen bound = 0 → no negative iq.
    driver.current_limits.bus_in_max_a = 10.0;
    driver.current_limits.bus_regen_max_a = 0.0;
    assert_eq!(driver.clamp_iq_for_bus(30.0), 20.0);
    assert_eq!(driver.clamp_iq_for_bus(-5.0), 0.0, "regen must be denied");
    assert_eq!(driver.clamp_iq_for_bus(15.0), 15.0);

    // Reverse rotation (mod_q < 0): bounds mirror.
    driver.bus_mod_q_filt = -0.5;
    assert_eq!(driver.clamp_iq_for_bus(-30.0), -20.0);
    assert_eq!(driver.clamp_iq_for_bus(5.0), 0.0, "regen must be denied");

    // Near-zero modulation: dead-band, no clamp (bus current ≈ 0).
    driver.bus_mod_q_filt = 0.0;
    assert_eq!(driver.clamp_iq_for_bus(-30.0), -30.0);

    // Finite regen allowance: -2 A bus at mod_q 0.4 → iq ≥ -5 A.
    driver.bus_mod_q_filt = 0.4;
    driver.current_limits.bus_regen_max_a = 2.0;
    assert_eq!(driver.clamp_iq_for_bus(-30.0), -5.0);
}

/// Every mode that energizes the motor and can read currents must trip
/// the measured-overcurrent protection. DirectVoltage has no PI loop to
/// rein the current in, and six-step has no current loop at all — a
/// shorted phase in those modes must not cook the FETs just because
/// the mode is "simple".
#[test]
fn direct_voltage_trips_overcurrent() {
    use crate::foc::trig::LibmSinCos;

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            // ~26 A magnitude — far above the 13 A threshold below.
            currents: (20.0, -10.0, -10.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(10.0));
    driver.set_mode(ControlMode::DirectVoltage {
        vd: 1.0,
        vq: 0.0,
        angle_rad: 0.0,
    });

    let res = driver.step(0);
    assert!(
        res.is_err(),
        "overcurrent must abort the DirectVoltage step"
    );
    assert_eq!(
        driver.mode(),
        ControlMode::Stopped,
        "overcurrent must latch Stopped"
    );
}

/// Closed-loop HFI through the full runtime path: FocDriver in
/// CurrentControl with a PhaseManager(HfiObserver) source must apply
/// the estimator's carrier injection itself — no detection-mode help.
/// The rotor is parked on the far side (2.5 rad: the PLL's nearest
/// saliency lock is the π-flipped one), so this exercises BOTH the
/// carrier path and the saturation polarity probe under an active PI
/// loop that partially fights the probe pulses.
#[test]
#[cfg(feature = "virtual-motor")]
#[cfg(feature = "hfi")]
fn current_control_drives_hfi_estimator() {
    use crate::foc::phase::{HfiObserver, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    const ROTOR_ANGLE: f32 = 2.5;
    // Same IPM as the estimator-level HFI tests: 3:1 saliency, heavy
    // rotor + friction so the injection itself doesn't move it, plus
    // d-axis saturation so the polarity probe has a signal.
    let params = MotorParams {
        r: 0.1,
        ld: 100e-6,
        lq: 300e-6,
        lambda: 0.02,
        pole_pairs: 4,
        j: 1e-2,
        friction_b: 5e-2,
        hall_offset: 0.0,
        sat_k: 0.05,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    motor.set_angle(ROTOR_ANGLE);

    // 1000 rad/s current loop — well below the 1 kHz carrier, so the
    // PI regulates the fundamental without eating the carrier response.
    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );
    let mut mgr = PhaseManager::sensorless();
    mgr.set_hfi_observer(HfiObserver::new(1000.0, 3.0));
    mgr.set_source(PhaseSource::Hfi).unwrap();

    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    // Carrier + polarity-probe currents exceed the bench-safe default
    // limits — give the test the board-scale ceiling explicitly.
    driver.set_current_limits(CurrentLimits::from_max_current(40.0));
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 0.0,
        id_target: 0.0,
    });

    let mut out = VirtualMotorOutput::default();
    for step in 1..20_000u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = driver.step(step * 50).expect("FOC step failed");
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
    }

    // Full-circle match: the polarity probe must have corrected the
    // π-flipped initial lock through the driver path.
    let true_angle = wrap_angle(out.angle_rad);
    let err = angle_difference(driver.phase().get().angle, true_angle).abs();
    assert!(
        err < 0.15,
        "HFI did not converge through FocDriver: est {} vs rotor {} (err {} full-circle)",
        driver.phase().get().angle,
        true_angle,
        err
    );
    let hfi = driver.phase().hfi_observer().unwrap();
    assert!(
        hfi.is_ready(),
        "HFI must be ready (locked + polarity resolved), confidence {}",
        hfi.confidence()
    );
    assert!(
        angle_difference(out.angle_rad, ROTOR_ANGLE).abs() < 0.15,
        "injection moved the rotor: {} rad",
        out.angle_rad
    );
}

/// Sensorless cold start: a pure-`Observer` motor commanded torque from
/// standstill must spin up via the ramp→hold→confirm sequencer and hand
/// off to the back-EMF observer. The observer alone cannot commutate at
/// zero speed (no back-EMF), so without the sequencer this deadlocks; the
/// test asserts it does NOT — the rotor reaches speed forward and the
/// observer locks.
#[test]
#[cfg(feature = "virtual-motor")]
fn sensorless_cold_start_ramps_then_hands_off() {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::foc::{angle_difference, wrap_angle};
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    // Non-salient drone-class motor. The inertia keeps the 6 A
    // acceleration within the observer PLL's tracking range: the
    // original j = 1e-4 gave ω̇_el ≈ 44e3 rad/s² → steady PLL lag
    // ω̇/pll_ki ≈ 2.2 rad — the estimate trailed the rotor by over 90°
    // and the whole run was a torque-flapping limit cycle that happened
    // to satisfy the asserts at exactly step 30_000 (extending the run
    // to 40_000 failed this test on UNMODIFIED code; found 2026-07-06
    // while adding the external-validity gate). That regime is
    // reproducer material for the mid-speed limit-cycle investigation,
    // not a cold-start regression gate.
    let params = MotorParams {
        r: 0.1,
        ld: 200e-6,
        lq: 200e-6,
        lambda: 0.01,
        pole_pairs: 7,
        j: 2e-3,
        friction_b: 1e-4,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    motor.set_angle(0.5);

    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
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
    mgr.set_source(PhaseSource::Observer).unwrap();

    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(20.0));

    // Torque from standstill → the cold-start sequence engages.
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 6.0,
        id_target: 0.0,
    });
    assert!(
        driver.phase().is_starting(),
        "cold start must engage on torque-from-rest"
    );

    let mut out = VirtualMotorOutput::default();
    // 2 s: the external-validity gate holds the handoff for two coherent
    // electrical revolutions beyond the old convergence point, so give
    // the post-handoff loop the same settling room it had before.
    for step in 1..40_000u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = driver.step(step * 50).expect("FOC step failed");
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
    }

    // Handed off (not stuck open-loop), spinning forward, observer locked.
    assert!(
        !driver.phase().is_starting(),
        "must hand off the open-loop sequencer, not run it forever"
    );
    assert!(
        driver.phase().observer().is_ready(),
        "back-EMF observer must be tracking after handoff"
    );
    assert!(
        out.omega_e > 50.0,
        "motor must spin up forward, got omega_e {}",
        out.omega_e
    );
    let err = angle_difference(driver.phase().get().angle, wrap_angle(out.angle_rad)).abs();
    assert!(
        err < 0.25,
        "observer angle {} vs rotor {} (err {} rad)",
        driver.phase().get().angle,
        wrap_angle(out.angle_rad),
        err
    );
}

/// Deadshort flying restart: a sensorless motor already spinning when
/// torque is commanded must be CAUGHT by the deadshort probe and handed
/// straight to the observer within a handful of cycles — NOT dragged
/// through the full ramp cold start (which would brake/fight the
/// moving rotor). Distinguishes flying restart from cold start by the
/// handoff happening fast and the rotor never being braked to a stop.
#[test]
#[cfg(feature = "virtual-motor")]
fn deadshort_catches_a_spinning_rotor_on_start() {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::foc::{angle_difference, wrap_angle};
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams {
        // τ = L/R = 2 ms: the short current is far from settled when
        // the probe window closes (seed accuracy across τ is
        // `flying_restart_catches_low_tau_motors`'s job).
        r: 0.1,
        ld: 200e-6,
        lq: 200e-6,
        lambda: 0.01,
        pole_pairs: 7,
        j: 1e-3, // heavier rotor: the brief probe barely slows it
        friction_b: 1e-4,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    // Freewheeling forward at ~200 rad/s elec, parked at a known angle,
    // zero current (gates were off) — the kick-push case.
    motor.set_angle(1.0);
    motor.set_velocity(200.0);

    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
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
    mgr.set_source(PhaseSource::Observer).unwrap();

    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(30.0));

    // Torque from rest (driver is Stopped) while the rotor spins → probe.
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 4.0,
        id_target: 0.0,
    });

    let mut out = VirtualMotorOutput::default();
    let mut caught_cycle = None;
    for step in 1..200u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = driver.step(step * 50).expect("FOC step failed");
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        if caught_cycle.is_none() && !driver.phase().is_starting() {
            caught_cycle = Some(step);
        }
    }

    // Caught FAST — within the probe window + a few cycles, not the ~0.7 s
    // ramp (which would be >14000 cycles).
    let caught = caught_cycle.expect("flying restart must hand off");
    assert!(
        caught < 30,
        "deadshort should catch quickly, took {caught} cycles"
    );
    assert!(
        driver.phase().observer().is_ready(),
        "observer must be tracking"
    );
    // Rotor never braked to a stop and still spinning forward near ω0.
    assert!(
        out.omega_e > 120.0,
        "rotor must keep spinning, got {}",
        out.omega_e
    );
    let err = angle_difference(driver.phase().get().angle, wrap_angle(out.angle_rad)).abs();
    assert!(
        err < 0.3,
        "observer angle {} vs rotor {} (err {err} rad)",
        driver.phase().get().angle,
        wrap_angle(out.angle_rad)
    );
}

/// Run the ZD2808-like bench scenario (cold start → handoff → free
/// acceleration at 1.5 A on a 12 V bus) with the given dq-decoupling
/// parameters and observer inductance. Plant inductances are the
/// fundamental (voltage-pulse) values. PI gains are held FIXED across
/// cases at bw = 1000 rad/s against the sim plant's l_avg — the sim has
/// no eddy-current frequency-dependent L, so this mirrors the bench's
/// "bw 1000 against the (smaller) HF inductance" without conflating the
/// loop-bandwidth variable with the parameter-consistency one under
/// test. Returns the max |i_dq| seen after handoff and the final
/// electrical velocity, or the step at which the driver tripped.
#[cfg(feature = "virtual-motor")]
fn run_bench_spin_with_decoupling(
    decoupling: Decoupling,
    observer_l: f32,
    observer_salient: bool,
    actuation_delay: f32,
    initial_rotor_angle: f32,
) -> Result<(f32, f32), (u64, StepError)> {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    // ZD2808 700 KV bench values (2026-07-05 detection): pulse Ld/Lq,
    // 1/ω-extrapolated λ. Light unloaded rotor → I/f runaway + free
    // acceleration through the ω_e region where the bench tripped.
    const LD: f32 = 86e-6;
    const LQ: f32 = 129e-6;
    const LAMBDA: f32 = 1.145e-3;
    let params = MotorParams {
        r: 0.127,
        ld: LD,
        lq: LQ,
        lambda: LAMBDA,
        pole_pairs: 7,
        // Inertia from the bench acceleration (erpm 1116→7958 in 0.3 s
        // at iq≈1.4 → ω̇_e≈2400 rad/s² → J≈5e-5); friction from the
        // 0.3 A no-load plateau. A 5× lighter rotor would exaggerate
        // the observer PLL's acceleration lag (ω̇/pll_ki) well past
        // anything the bench produces.
        j: 5e-5,
        friction_b: 4e-6,
        // Realistic one-cycle actuation delay: the driver's
        // phase-advance convention is exact against it — with the
        // ideal (0-delay) plant the advance itself injects a growing
        // ω·dt lead error and destabilizes the loop near ω_e ≈ 2000.
        actuation_delay: 1.0,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    motor.set_angle(initial_rotor_angle);

    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        0.127,
        (LD + LQ) / 2.0,
        12.0,
    );
    foc.set_decoupling(Some(decoupling));

    let mut mgr = PhaseManager::sensorless();
    let mut obs = BackEmfObserver::new(0.127, observer_l, LAMBDA);
    if observer_salient {
        obs = obs.with_saliency(LD, LQ);
    }
    mgr.set_observer(Observer::BackEmf(obs));
    // Cannot fail with an observer installed; mapped instead of unwrapped
    // to satisfy unwrap_in_result in this Result-returning helper (the
    // error value is a placeholder).
    mgr.set_source(PhaseSource::Observer)
        .map_err(|_| (0u64, StepError::NotCalibrated))?;

    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(10.0));
    driver.set_actuation_delay(actuation_delay);
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 1.5,
        id_target: 0.0,
    });

    let mut out = VirtualMotorOutput::default();
    let mut max_idq = 0.0f32;
    for step in 1..40_000u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = match driver.step(step * 50) {
            Ok(t) => t,
            Err(e) => return Err((step, e)),
        };
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        if !driver.phase().is_starting() {
            let idq = (telem.id * telem.id + telem.iq * telem.iq).sqrt();
            max_idq = max_idq.max(idq);
        }
        // Stop well past the bench trip zone (~800 rad/s) but well below
        // the 12 V modulation ceiling (~5.9 krad/s at 1.5 A): above it
        // the loop saturates by physics — field weakening is not
        // implemented, and that regime is not what this probes.
        if out.omega_e >= 2500.0 {
            break;
        }
    }
    Ok((max_idq, out.omega_e))
}

/// What one instrumented ZD2808 cold start saw (see
/// [`run_zd2808_cold_start`]).
#[cfg(feature = "virtual-motor")]
#[derive(Debug, Clone, Copy, Default)]
struct ColdStartReport {
    /// Step at which the startup sequencer handed commutation to the
    /// observer (deadshort catch or ramp handoff), if it ever did.
    handoff_step: Option<u64>,
    /// TRUE rotor electrical velocity at the handoff instant.
    rotor_omega_at_handoff: f32,
    /// Observer velocity estimate at the handoff instant.
    observer_vel_at_handoff: f32,
    /// Hard trip (overcurrent etc.), if any.
    trip: Option<(u64, StepError)>,
    /// True rotor electrical velocity at the end of the run.
    final_omega: f32,
    /// Observer state at the end of the run.
    observer_ready_final: bool,
    observer_vel_final: f32,
    /// Fraction of post-handoff cycles with `angle_trustworthy()` false
    /// (iq gated to zero) — the trust-loss deadlock signature.
    untrusted_frac: f32,
}

/// One bench-mirror ZD2808 cold start: the plant carries the fundamental
/// Ld/Lq plus bench-level dead-time distortion and ADC quantization+noise,
/// while the estimation chain runs the baked-config parameters — AC
/// L = 24 µH in the observer and the current-loop gains, λ tracking on,
/// fundamental Ld/Lq only in the decoupling override — i.e. the exact
/// configuration the 2026-07-06 staircase phantom handoff ran
/// (captures/staircase-1.parquet).
/// Knobs for [`run_zd2808_cold_start`]: which parts of the bench
/// configuration/imperfections to include.
#[cfg(feature = "virtual-motor")]
#[derive(Clone, Copy)]
struct ColdStartCfg {
    iq_target: f32,
    initial_rotor_angle: f32,
    steps: u64,
    /// Observer flux-integrator inductance (bench bakes the AC 24 µH).
    observer_l: f32,
    observer_salient: bool,
    /// Current-loop gain inductance (bench bakes the AC 24 µH).
    controller_l: f32,
    /// Plant dead-time distortion + firmware comp (bench: 800 ns).
    dead_time: bool,
    /// Plant ADC quantization + noise (bench: 15 mA LSB).
    adc_noise: bool,
    /// Rotor inertia (kg·m²). The original tests assumed 5e-5; the
    /// 2026-07-06 bench climb (clean 1.5 A acceleration, e_q-verified)
    /// implies ≈3.2e-5 for the ZD2808 bell.
    j: f32,
    /// Print `t rotor ω̂ iq` every N steps (0 = off) — truth-vs-estimate
    /// trace for the sawtooth exploration; the whole point of the sim is
    /// that both are visible at once.
    trace_every: u64,
    /// Eddy-branch time constant for the plant (0 = ideal single-L).
    /// When nonzero the plant gets the ZD2808 L(f): DC 86/129 µH
    /// falling to the ~24 µH AC plateau (ΔL 62/105 µH).
    eddy_tau_s: f32,
    /// Plant imperfection scale for the sawtooth hunt: multiplies the
    /// dead-time distortion, and skews the PLANT's λ/R away from the
    /// configured values (λ ×(1+0.15·s), R ×(1+0.1·s) at s=1) — the
    /// estimation chain keeps the baked numbers. 0 = exact match.
    plant_skew: f32,
    /// Braking load-torque pulse (N·m) applied at `disturb_at_step`
    /// for `disturb_steps` — seeds a pole slip so the slip-kick
    /// ratchet is reachable in sim (a clean capture never slips).
    disturb_torque_nm: f32,
    disturb_at_step: u64,
    disturb_steps: u64,
    /// Observer phase kick (rad) applied once at `kick_at_step`
    /// (0 = off): re-seed the observer at `phase + kick` — one
    /// accumulated slip-kick, the harder ratchet seed (a load pulse
    /// rides through).
    kick_rad: f32,
    kick_at_step: u64,
    /// Observer PLL integral gain (kp scales as ki/20; default 20e3).
    pll_ki: f32,
    /// Commutation phase tracker (0.0 = off): tracker natural
    /// frequency ωn, el rad/s (`PhaseManager::set_phase_tracker`
    /// with ζ = 1.2).
    tracker_wn: f32,
}

#[cfg(feature = "virtual-motor")]
fn run_zd2808_cold_start(cfg: ColdStartCfg) -> ColdStartReport {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    const VBUS: f32 = 12.0;
    const DEAD_TIME_NS: f32 = 800.0;
    const LAMBDA: f32 = 1.145e-3;
    let sk = cfg.plant_skew;
    let params = MotorParams {
        r: 0.127 * (1.0 + 0.1 * sk),
        ld: 86e-6,
        lq: 129e-6,
        lambda: LAMBDA * (1.0 + 0.15 * sk),
        pole_pairs: 7,
        j: cfg.j,
        friction_b: 4e-6,
        actuation_delay: 1.0,
        substeps: 10,
        // Bench-level bridge distortion + sensor imperfection: the
        // firmware's dead-time comp is on below, so only the residual
        // (zero-crossing clamping the comp can't model) reaches the
        // estimators — same as hardware.
        dead_time_v: if cfg.dead_time {
            DEAD_TIME_NS * 1e-9 * 20_000.0 * VBUS * (1.0 + sk)
        } else {
            0.0
        },
        adc_lsb_a: if cfg.adc_noise { 62.0 / 4096.0 } else { 0.0 },
        adc_noise_a: if cfg.adc_noise { 62.0 / 4096.0 } else { 0.0 },
        eddy_delta_l_d: if cfg.eddy_tau_s > 0.0 { 62e-6 } else { 0.0 },
        eddy_delta_l_q: if cfg.eddy_tau_s > 0.0 { 105e-6 } else { 0.0 },
        eddy_tau_s: cfg.eddy_tau_s,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    motor.set_angle(cfg.initial_rotor_angle);

    // Baked-config current loop: bw 1000 rad/s against `controller_l`.
    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        0.127,
        cfg.controller_l,
        VBUS,
    );
    foc.set_decoupling(Some(Decoupling {
        ld_h: 86e-6,
        lq_h: 129e-6,
        flux_linkage_wb: LAMBDA,
        resistance_ohm: 0.127,
    }));
    if cfg.dead_time {
        foc.set_dead_time_comp(DEAD_TIME_NS as u32, 20_000);
    }

    let mut mgr = PhaseManager::sensorless();
    let mut obs = BackEmfObserver::new(0.127, cfg.observer_l, LAMBDA)
        .with_lambda_tracking(crate::foc::phase::DEFAULT_LAMBDA_GAIN);
    obs.set_pll_gains(cfg.pll_ki / 20.0, cfg.pll_ki);
    if cfg.observer_salient {
        obs = obs.with_saliency(86e-6, 129e-6);
    }
    mgr.set_observer(Observer::BackEmf(obs));
    if cfg.tracker_wn > 0.0 {
        mgr.set_phase_tracker(cfg.tracker_wn, 1.2);
    }
    mgr.set_source(PhaseSource::Observer)
        .expect("observer source");

    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(10.0));
    driver.set_actuation_delay(1.0); // = the plant's
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: cfg.iq_target,
        id_target: 0.0,
    });

    let mut report = ColdStartReport::default();
    let mut out = VirtualMotorOutput::default();
    let mut was_starting = false;
    let mut post_handoff_cycles = 0u64;
    let mut untrusted_cycles = 0u64;
    for step in 1..=cfg.steps {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = match driver.step(step * 50) {
            Ok(t) => t,
            Err(e) => {
                report.trip = Some((step, e));
                break;
            }
        };
        let starting = driver.phase().is_starting();
        if was_starting && !starting && report.handoff_step.is_none() {
            report.handoff_step = Some(step);
            report.rotor_omega_at_handoff = out.omega_e;
            report.observer_vel_at_handoff = driver.phase().observer().velocity().unwrap_or(0.0);
        }
        was_starting = starting;
        if report.handoff_step.is_some() {
            post_handoff_cycles += 1;
            if !driver.phase().angle_trustworthy() {
                untrusted_cycles += 1;
            }
        }
        if cfg.kick_at_step > 0 && step == cfg.kick_at_step {
            let ph = driver.phase().observer().phase().unwrap_or(0.0) + cfg.kick_rad;
            let vel = driver.phase().observer().velocity().unwrap_or(0.0);
            driver.phase_mut().seed_observer(wrap_angle(ph), vel);
        }
        if cfg.trace_every > 0 && step % cfg.trace_every == 0 {
            println!(
                "t={:6.3} rotor={:7.1} obs={:7.1} iq={:+6.2} conf={:.2} starting={}",
                step as f32 * DT,
                out.omega_e,
                driver.phase().observer().velocity().unwrap_or(0.0),
                telem.iq,
                driver.phase().observer().confidence(),
                u8::from(starting),
            );
        }
        // Drive the plant from the DUTIES, not the pre-modulation
        // command: the dead-time comp lives only in the duty path, and
        // feeding v_alpha/v_beta directly would leave the plant's
        // distortion uncompensated (the trap virtual-motor-fidelity.md
        // documents). Deadshort holds all-low duties = a shorted bridge,
        // which the αβ extraction maps to zero volts — correct.
        let scale = VBUS / f32::from(driver.pwm().max_duty());
        let va = f32::from(telem.duties[0]) * scale;
        let vb = f32::from(telem.duties[1]) * scale;
        let vc = f32::from(telem.duties[2]) * scale;
        let v_alpha = (2.0 * va - vb - vc) / 3.0;
        let v_beta = (vb - vc) * crate::foc::constants::FRAC_1_SQRT_3;
        let load = if cfg.disturb_steps > 0
            && step >= cfg.disturb_at_step
            && step < cfg.disturb_at_step + cfg.disturb_steps
        {
            cfg.disturb_torque_nm
        } else {
            0.0
        };
        out = motor.step(v_alpha, v_beta, load, DT);
    }
    report.final_omega = out.omega_e;
    report.observer_ready_final = driver.phase().observer().is_ready();
    report.observer_vel_final = driver.phase().observer().velocity().unwrap_or(0.0);
    if post_handoff_cycles > 0 {
        #[allow(clippy::cast_precision_loss)]
        {
            report.untrusted_frac = untrusted_cycles as f32 / post_handoff_cycles as f32;
        }
    }
    report
}

/// Regression gate for bench reproducer #4 (2026-07-06 staircase,
/// captures/staircase-1.parquet): the no-align catch transient + residual
/// dead-time distortion excite the observer into a PHANTOM lock — every
/// internal readiness criterion satisfied on a standing rotor — and the
/// old gate handed closed loop to it (rotor −6 rad/s, observer 60),
/// collapsing into the trust-loss deadlock (iq gated, gate flicker,
/// 70–80% untrusted cycles, audible whine).
///
/// With the external-validity chain (proxy-corroborated readiness +
/// deadshort confirm probe + hold give-up recycle) the phantom must
/// never reach closed loop: either no handoff at all, or a handoff onto
/// a rotor that is genuinely turning — and no gate-flicker deadlock
/// either way.
#[test]
#[cfg(feature = "virtual-motor")]
fn phantom_handoff_blocked_by_confirm_probe() {
    // The worst staircase leg: 0.3 A from the initial angle where the
    // ramp fails to capture the rotor at all (sim-deterministic).
    let rep = run_zd2808_cold_start(ColdStartCfg {
        iq_target: 0.3,
        initial_rotor_angle: 4.71,
        steps: 60_000,
        observer_l: (86e-6 + 129e-6) / 2.0,
        observer_salient: true,
        controller_l: (86e-6 + 129e-6) / 2.0,
        dead_time: true,
        adc_noise: true,
        j: 5e-5,
        trace_every: 0,
        eddy_tau_s: 0.0,
        plant_skew: 0.0,
        disturb_torque_nm: 0.0,
        disturb_at_step: 0,
        disturb_steps: 0,
        kick_rad: 0.0,
        kick_at_step: 0,
        pll_ki: 20e3,
        tracker_wn: 0.0,
    });
    assert!(rep.trip.is_none(), "must not trip: {:?}", rep.trip);
    if rep.handoff_step.is_some() {
        assert!(
            rep.rotor_omega_at_handoff.abs() > 30.0,
            "handoff onto a standing rotor (rotor {} rad/s, observer {})",
            rep.rotor_omega_at_handoff,
            rep.observer_vel_at_handoff
        );
    }
    assert!(
        rep.untrusted_frac < 0.05,
        "trust-loss gate flicker after handoff: untrusted_frac {}",
        rep.untrusted_frac
    );
}

/// The benign side of the same gate: a capturable cold start through
/// bench-level dead-time distortion + ADC noise must still make it to a
/// sustained spin (possibly via confirm retries / a give-up recycle),
/// not be starved by the new checks.
#[test]
#[cfg(feature = "virtual-motor")]
fn cold_start_recovers_through_dead_time_distortion() {
    let rep = run_zd2808_cold_start(ColdStartCfg {
        iq_target: 0.3,
        initial_rotor_angle: 0.0,
        steps: 60_000,
        observer_l: (86e-6 + 129e-6) / 2.0,
        observer_salient: true,
        controller_l: (86e-6 + 129e-6) / 2.0,
        dead_time: true,
        adc_noise: true,
        j: 5e-5,
        trace_every: 0,
        eddy_tau_s: 0.0,
        plant_skew: 0.0,
        disturb_torque_nm: 0.0,
        disturb_at_step: 0,
        disturb_steps: 0,
        kick_rad: 0.0,
        kick_at_step: 0,
        pll_ki: 20e3,
        tracker_wn: 0.0,
    });
    assert!(rep.trip.is_none(), "must not trip: {:?}", rep.trip);
    let handoff = rep.handoff_step.expect("must eventually hand off");
    assert!(
        rep.rotor_omega_at_handoff > 30.0,
        "handoff at step {handoff} onto rotor at {} rad/s",
        rep.rotor_omega_at_handoff
    );
    assert!(
        rep.final_omega > 500.0,
        "must reach a sustained spin, got {}",
        rep.final_omega
    );
    assert!(rep.untrusted_frac < 0.05);
}

/// Startup capture floor (see [`STARTUP_MIN_DRIVE_A`]): a cold start
/// under a weak drive command — the mid-cruise restart case, bench
/// 2026-07-07 spin-punch-15 at 0.3 A — must ramp with the floor
/// current and reach a confirmed handoff onto a genuinely spinning
/// rotor from the angle where the un-floored 0.3 A ramp fails
/// (`phantom_handoff_blocked_by_confirm_probe`'s pathological angle).
#[test]
#[cfg(feature = "virtual-motor")]
fn weak_command_cold_start_captures_via_floor() {
    let rep = run_zd2808_cold_start(ColdStartCfg {
        iq_target: 0.3,
        initial_rotor_angle: 4.71,
        steps: 60_000,
        observer_l: (86e-6 + 129e-6) / 2.0,
        observer_salient: true,
        controller_l: (86e-6 + 129e-6) / 2.0,
        dead_time: true,
        adc_noise: true,
        j: 3.2e-5,
        trace_every: 0,
        eddy_tau_s: 0.0,
        plant_skew: 0.0,
        disturb_torque_nm: 0.0,
        disturb_at_step: 0,
        disturb_steps: 0,
        kick_rad: 0.0,
        kick_at_step: 0,
        pll_ki: 20e3,
        tracker_wn: 0.0,
    });
    assert!(rep.trip.is_none(), "must not trip: {:?}", rep.trip);
    let handoff = rep.handoff_step.expect("floored ramp must hand off");
    assert!(
        rep.rotor_omega_at_handoff.abs() > 30.0,
        "handoff at step {handoff} onto rotor at {} rad/s",
        rep.rotor_omega_at_handoff
    );
    assert!(rep.untrusted_frac < 0.05);
}

/// Sawtooth exploration (2026-07-06 night): does the sim reproduce the
/// bench post-handoff ESTIMATE SAWTOOTH once it runs the ACTUAL bench
/// configuration? The regression tests above run observer_l =
/// controller_l = l_avg (107.5 µH) and j = 5e-5 — but the bench bakes
/// the AC 24 µH into BOTH (motor_params.l → observer; kp = L·bw =
/// 0.024), and the e_q-verified 1.5 A climb implies j ≈ 3.2e-5.
/// Run with:
/// `cargo test -p oxifoc-core --release explore_sawtooth -- --nocapture --ignored`
#[test]
#[ignore = "exploration harness, not a regression test"]
#[cfg(feature = "virtual-motor")]
fn explore_sawtooth_bench_config() {
    const L_AVG: f32 = (86e-6 + 129e-6) / 2.0;
    const L_AC: f32 = 24e-6;
    for (label, obs_l, ctrl_l, j, trace, eddy_tau, kick, pll_ki) in [
        (
            "bench cfg, ideal plant",
            L_AC,
            L_AC,
            3.2e-5,
            0,
            0.0,
            0.0,
            20e3,
        ),
        (
            "eddy plant, kick 0.5 rad",
            L_AC,
            L_AC,
            3.2e-5,
            400,
            0.3e-3,
            0.5,
            20e3,
        ),
        (
            "eddy plant, kick 1.0 rad",
            L_AC,
            L_AC,
            3.2e-5,
            400,
            0.3e-3,
            1.0,
            20e3,
        ),
        (
            "ideal plant, kick 1.0 rad",
            L_AC,
            L_AC,
            3.2e-5,
            0,
            0.0,
            1.0,
            20e3,
        ),
        (
            "eddy plant, kick 1.0, PLL ki/4",
            L_AC,
            L_AC,
            3.2e-5,
            400,
            0.3e-3,
            1.0,
            5e3,
        ),
        (
            "control: obs=avg ctrl=avg j=5e-5, ideal",
            L_AVG,
            L_AVG,
            5e-5,
            0,
            0.0,
            0.0,
            20e3,
        ),
    ] {
        println!("== {label} ==");
        let rep = run_zd2808_cold_start(ColdStartCfg {
            iq_target: 0.5,
            initial_rotor_angle: 0.0,
            steps: 60_000,
            observer_l: obs_l,
            observer_salient: false,
            controller_l: ctrl_l,
            dead_time: true,
            adc_noise: true,
            j,
            trace_every: trace,
            eddy_tau_s: eddy_tau,
            plant_skew: 0.0,
            disturb_torque_nm: 0.0,
            disturb_at_step: 0,
            disturb_steps: 0,
            kick_rad: kick,
            kick_at_step: if kick != 0.0 { 16_000 } else { 0 },
            pll_ki,
            tracker_wn: 0.0,
        });
        println!(
            "   handoff@{:?} rotor@handoff={:.0} obs@handoff={:.0} trip={:?} \
             final_rotor={:.0} obs_final={:.0} untrusted={:.2}",
            rep.handoff_step,
            rep.rotor_omega_at_handoff,
            rep.observer_vel_at_handoff,
            rep.trip,
            rep.final_omega,
            rep.observer_vel_final,
            rep.untrusted_frac,
        );
    }
}

/// Commutation phase tracker (PhaseManager::set_phase_tracker): the
/// full cold start must still hand off and reach a sustained spin
/// with the tracker smoothing the post-handoff commutation.
#[test]
#[cfg(feature = "virtual-motor")]
fn cold_start_with_phase_tracker_spins_up() {
    let rep = run_zd2808_cold_start(ColdStartCfg {
        iq_target: 0.5,
        initial_rotor_angle: 0.0,
        steps: 60_000,
        observer_l: 24e-6,
        observer_salient: false,
        controller_l: 24e-6,
        dead_time: true,
        adc_noise: true,
        j: 3.2e-5,
        trace_every: 0,
        eddy_tau_s: 0.3e-3,
        plant_skew: 0.0,
        disturb_torque_nm: 0.0,
        disturb_at_step: 0,
        disturb_steps: 0,
        kick_rad: 0.0,
        kick_at_step: 0,
        pll_ki: 20e3,
        tracker_wn: 60.0,
    });
    assert!(rep.trip.is_none(), "must not trip: {:?}", rep.trip);
    let handoff = rep.handoff_step.expect("must hand off with freq-led on");
    assert!(
        rep.rotor_omega_at_handoff > 30.0,
        "handoff at {handoff} onto rotor at {} rad/s",
        rep.rotor_omega_at_handoff
    );
    assert!(
        rep.final_omega > 500.0,
        "must reach a sustained spin through the filter, got {}",
        rep.final_omega
    );
    assert!(rep.untrusted_frac < 0.05);
}

/// TEMPORARY exploration of the phantom-handoff reproducer — prints one
/// line per (iq, initial-angle) case. Run with:
/// `cargo test -p oxifoc-core explore_phantom -- --nocapture --ignored`
#[test]
#[ignore = "exploration harness, not a regression test"]
#[cfg(feature = "virtual-motor")]
fn explore_phantom_handoff() {
    const L_AVG: f32 = (86e-6 + 129e-6) / 2.0;
    const L_AC: f32 = 24e-6;
    // Decomposition matrix: which ingredient breaks the post-handoff
    // closed loop / produces the phantom handoff.
    let variants: [(&str, f32, bool, f32, bool, bool); 7] = [
        (
            "ideal (existing-test config)",
            L_AVG,
            true,
            L_AVG,
            false,
            false,
        ),
        ("+noise", L_AVG, true, L_AVG, false, true),
        ("+dead-time", L_AVG, true, L_AVG, true, false),
        ("+both", L_AVG, true, L_AVG, true, true),
        (
            "bench ctrl gains (L_AC) +both",
            L_AVG,
            true,
            L_AC,
            true,
            true,
        ),
        ("bench obs L_AC +both", L_AC, false, L_AVG, true, true),
        ("full bench mirror", L_AC, false, L_AC, true, true),
    ];
    for (label, obs_l, salient, ctrl_l, dead_time, adc_noise) in variants {
        println!("== {label} ==");
        for iq in [0.3f32, 1.5] {
            for i in [0u8, 3, 6] {
                let a0 = f32::from(i) * core::f32::consts::TAU / 8.0;
                let rep = run_zd2808_cold_start(ColdStartCfg {
                    iq_target: iq,
                    initial_rotor_angle: a0,
                    steps: 60_000,
                    observer_l: obs_l,
                    observer_salient: salient,
                    controller_l: ctrl_l,
                    dead_time,
                    adc_noise,
                    j: 5e-5,
                    trace_every: 0,
                    eddy_tau_s: 0.0,
                    plant_skew: 0.0,
                    disturb_torque_nm: 0.0,
                    disturb_at_step: 0,
                    disturb_steps: 0,
                    kick_rad: 0.0,
                    kick_at_step: 0,
                    pll_ki: 20e3,
                    tracker_wn: 0.0,
                });
                println!(
                    "iq={iq:>3} a0={a0:4.2} handoff={:?} rotor@h={:7.1} obs@h={:7.1} \
                     trip={:?} final={:8.1} ready_end={} obs_end={:7.1} untrusted={:.2}",
                    rep.handoff_step,
                    rep.rotor_omega_at_handoff,
                    rep.observer_vel_at_handoff,
                    rep.trip,
                    rep.final_omega,
                    rep.observer_ready_final,
                    rep.observer_vel_final,
                    rep.untrusted_frac,
                );
            }
        }
    }
}

/// Fix #2 of the estimator session (docs/TODO.md): a mid-drive trust
/// loss must NOT deadlock. Before: estimator collapse → untrusted-angle
/// iq gate zeros torque → no motion → no back-EMF → no trust, forever
/// (bench staircase: 15 s of iq = 0.00 on a standing rotor). Now: after
/// [`SENSORLESS_RESTART_AFTER_S`] of continuous distrust under a
/// nonzero command, the driver re-enters the sensorless cold start.
///
/// Scenario: healthy sensorless spin, then a brake torque far above the
/// motor's capability stalls the rotor (trust is lost by physics);
/// after the load releases, the restart machinery must respin the
/// motor without any new host command.
#[test]
#[cfg(feature = "virtual-motor")]
fn trust_loss_mid_drive_restarts_cold_start() {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    const VBUS: f32 = 12.0;
    const LAMBDA: f32 = 1.145e-3;
    const L_AVG: f32 = (86e-6 + 129e-6) / 2.0;
    let params = MotorParams {
        r: 0.127,
        ld: 86e-6,
        lq: 129e-6,
        lambda: LAMBDA,
        pole_pairs: 7,
        j: 5e-5,
        friction_b: 4e-6,
        actuation_delay: 1.0,
        substeps: 10,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);

    let mut foc =
        FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(0.127, L_AVG, VBUS);
    foc.set_decoupling(Some(Decoupling {
        ld_h: 86e-6,
        lq_h: 129e-6,
        flux_linkage_wb: LAMBDA,
        resistance_ohm: 0.127,
    }));
    let mut mgr = PhaseManager::sensorless();
    mgr.set_observer(Observer::BackEmf(
        BackEmfObserver::new(0.127, L_AVG, LAMBDA)
            .with_saliency(86e-6, 129e-6)
            .with_lambda_tracking(crate::foc::phase::DEFAULT_LAMBDA_GAIN),
    ));
    mgr.set_source(PhaseSource::Observer).unwrap();
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(10.0));
    driver.set_actuation_delay(1.0); // = the plant's
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 1.5,
        id_target: 0.0,
    });

    // Phase 1: healthy spin-up past the first handoff.
    let mut out = VirtualMotorOutput::default();
    let mut handed_off = false;
    let mut step = 1u64;
    while step < 40_000 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = driver.step(step * 50).expect("healthy spin must not trip");
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        step += 1;
        if !driver.phase().is_starting() && driver.phase().observer().is_ready() {
            handed_off = true;
            if out.omega_e > 500.0 {
                break;
            }
        }
    }
    assert!(handed_off && out.omega_e > 500.0, "spin-up failed");

    // Phase 2: drop to a light 0.3 A cruise, then stall it with a load
    // an order of magnitude past what 0.3 A can hold — the rotor drags
    // to a stop, the estimator loses its signal, trust drops, and the
    // restart must engage (is_starting again). The current stays small
    // so the wrong-frame PI windup cannot race the validity revoke to
    // the overcurrent trip (a hard high-current stall legitimately
    // trips instead — that path belongs to protection, and its OC is
    // exactly bench reproducers #1/#2's signature).
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 0.3,
        id_target: 0.0,
    });
    let mut restarted = false;
    for _ in 0..40_000u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = match driver.step(step * 50) {
            Ok(t) => t,
            Err(e) => panic!("stall must gate torque, not trip: {e:?}"),
        };
        // Friction-style jam: always OPPOSES motion (a signed constant
        // would reverse-drive the rotor once stopped and the observer
        // would just track the backward spin — honest, but not the
        // trust-loss scenario under test).
        let load = 0.03 * out.omega_e.signum();
        out = motor.step(telem.v_alpha, telem.v_beta, load, DT);
        step += 1;
        if driver.phase().is_starting() {
            restarted = true;
            break;
        }
    }
    assert!(
        restarted,
        "trust loss under stall must re-enter the cold start (deadlock otherwise); \
         omega_e {} trustworthy {}",
        out.omega_e,
        driver.phase().angle_trustworthy()
    );

    // Phase 3: load released — the restart machinery must respin the
    // motor with no new host command (retries/recycles included).
    for _ in 0..80_000u64 {
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = match driver.step(step * 50) {
            Ok(t) => t,
            Err(e) => panic!("recovery must not trip: {e:?}"),
        };
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        step += 1;
        if out.omega_e > 500.0 && !driver.phase().is_starting() {
            break;
        }
    }
    assert!(
        out.omega_e > 500.0,
        "must respin after the load releases, got omega_e {}",
        out.omega_e
    );
}

/// Perfect-angle control run: the FocController driven directly against
/// the plant with the TRUE rotor angle/velocity (no observer, no
/// startup) — isolates the current loop + actuation delay from the
/// estimation chain.
#[cfg(feature = "virtual-motor")]
fn run_perfect_angle_spin(
    decoupling: Decoupling,
    actuation_delay: f32,
) -> Result<(f32, f32), (u64, &'static str)> {
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams {
        r: 0.127,
        ld: 86e-6,
        lq: 129e-6,
        lambda: 1.145e-3,
        pole_pairs: 7,
        j: 5e-5,
        friction_b: 4e-6,
        actuation_delay: 1.0,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);
    let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        0.127,
        (86e-6 + 129e-6) / 2.0,
        12.0,
    );
    foc.set_decoupling(Some(decoupling));

    let mut out = VirtualMotorOutput::default();
    let mut max_idq = 0.0f32;
    for step in 1..40_000u64 {
        // Same advance as FocDriver::set_actuation_delay.
        foc.set_actuation_advance(out.omega_e * DT * (actuation_delay + 0.5));
        let telem = foc.step_with_injection(
            (out.ia, out.ib, out.ic),
            wrap_angle(out.angle_rad),
            out.omega_e,
            0.0,
            1.5,
            0.0,
            0.0,
            4250,
            DT,
        );
        let idq = (telem.id * telem.id + telem.iq * telem.iq).sqrt();
        if step > 2_000 {
            max_idq = max_idq.max(idq);
        }
        if idq > 10.0 {
            return Err((step, "overcurrent"));
        }
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        if out.omega_e >= 2500.0 {
            break;
        }
    }
    Ok((max_idq, out.omega_e))
}

/// High-speed regression guard for the estimation + current-loop chain
/// on the ZD2808-like plant (fundamental Ld/Lq 86/129 µH, bench-derived
/// inertia): the full sensorless pipeline with CONSISTENT parameters
/// (decoupling and observer on the true Ld/Lq) must accelerate clean
/// through the region where the 2026-07-06 bench tripped (~800 rad/s)
/// and past 2500 rad/s with bounded currents.
///
/// What this deliberately does NOT assert: a repro of the bench trip
/// itself. The bench motor's inductance is frequency-dependent (eddy
/// currents: ~24 µH HF plateau vs 86/129 µH fundamental) and the sim
/// plant carries a single L — with bench-honest inertia even the
/// bench's mismatched configuration runs clean here, so the bench
/// mechanism is probed on the bench (gain experiments). What the sim
/// DID establish en route: the current loop by itself (perfect-angle
/// arm below) is unconditionally stable in this regime — a high-speed
/// divergence points at the estimation chain (the sim's own dominant
/// term was observer-PLL acceleration lag ω̇/pll_ki once the rotor was
/// modeled 5× too light).
#[test]
#[cfg(feature = "virtual-motor")]
fn sensorless_chain_stable_through_bench_trip_zone() {
    let consistent = run_bench_spin_with_decoupling(
        Decoupling {
            ld_h: 86e-6,
            lq_h: 129e-6,
            flux_linkage_wb: 1.145e-3,
            resistance_ohm: 0.127,
        },
        (86e-6 + 129e-6) / 2.0,
        true,
        1.0, // = the plant's actuation delay
        0.0,
    );
    let (max_idq, omega_final) =
        consistent.expect("consistent-params sensorless chain must not trip");
    // The chain runs at ~2.0 A peak for the 1.5 A command. This bound
    // was 6.5 while an earlier estimation chain peaked at 5.7–6.05 A
    // through the ~800 rad/s zone; it outlived that behavior by months
    // and would have passed a 3× regression. Same bound as
    // cold_start_captures_rotor_from_any_initial_angle.
    assert!(
        max_idq < 3.0,
        "post-handoff |i_dq| must stay bounded, got {max_idq}"
    );
    assert!(
        omega_final >= 2500.0,
        "must accelerate clean through the bench trip zone, got ω_e {omega_final}"
    );

    // Perfect-angle control arm: current loop + one-cycle actuation
    // delay, no estimation — must track the 1.5 A target tightly all
    // the way (pins any future high-speed failure on the estimation
    // chain, not the loop).
    let perfect = run_perfect_angle_spin(
        Decoupling {
            ld_h: 86e-6,
            lq_h: 129e-6,
            flux_linkage_wb: 1.145e-3,
            resistance_ohm: 0.127,
        },
        1.0, // = the plant's actuation delay
    );
    let (max_idq, omega_final) = perfect.expect("perfect-angle loop must not trip");
    assert!(
        max_idq < 2.0,
        "perfect-angle |i_dq| must track the 1.5 A target, got {max_idq}"
    );
    assert!(omega_final >= 2500.0);
}

/// No-align cold start (2026-07-06 redesign): the soft-started rotating
/// ramp must capture the rotor from ANY initial angle — including the
/// worst cases where the rotor sits ±π from the ramp's start angle. The
/// align-era failure mode was the fixed-angle dwell pumping the rotor's
/// undamped magnetic spring (dq overcurrent on 1/3+ of bench cold
/// starts, angle-dependent); this sweep is its regression gate.
#[test]
#[cfg(feature = "virtual-motor")]
fn cold_start_captures_rotor_from_any_initial_angle() {
    for frac in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let delta0 = frac * core::f32::consts::PI;
        let res = run_bench_spin_with_decoupling(
            Decoupling {
                ld_h: 86e-6,
                lq_h: 129e-6,
                flux_linkage_wb: 1.145e-3,
                resistance_ohm: 0.127,
            },
            (86e-6 + 129e-6) / 2.0,
            true,
            1.0, // = the plant's actuation delay
            delta0,
        );
        let (max_idq, omega_final) = res.unwrap_or_else(|(step, e)| {
            panic!("cold start from rotor angle {delta0} tripped at step {step}: {e:?}")
        });
        // ~2.0 A peak for the 1.5 A command from every sweep angle (see
        // sensorless_chain_stable_through_bench_trip_zone for why this
        // is no longer the old 6.5).
        assert!(
            max_idq < 3.0,
            "post-handoff |i_dq| bounded from angle {delta0}, got {max_idq}"
        );
        assert!(
            omega_final >= 2500.0,
            "must reach speed from angle {delta0}, got {omega_final}"
        );
    }
}

/// The ISR-resident deadman: a running driver with no fresh setpoint for
/// longer than the staleness timeout arms the failsafe; a fresh
/// `note_command_tick` (and `set_mode`) re-arms normal control.
#[test]
fn deadman_arms_failsafe_on_stale_command() {
    use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_failsafe(FailsafeConfig {
        policy: FailsafePolicy::Coast,
        staleness_timeout_us: 1_000, // 1 ms
        ..FailsafeConfig::default()
    });

    // Stopped: deadman never fires, however stale.
    assert!(!driver.deadman_expired(1_000_000));

    // Running + affirmed: not stale yet.
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    });
    driver.note_command_tick(0);
    assert!(!driver.deadman_expired(500));
    assert!(!driver.failsafe_active());

    // Past the timeout with no fresh command → arm.
    assert!(driver.deadman_expired(2_000));
    driver.enter_failsafe();
    assert!(driver.failsafe_active());

    // A fresh command re-arms control: set_mode clears the failsafe, and
    // the new stamp un-stales the deadman.
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 2.0,
        id_target: 0.0,
    });
    driver.note_command_tick(2_000);
    assert!(!driver.failsafe_active());
    assert!(!driver.deadman_expired(2_500));
}

/// Bench/calibration modes (OpenLoop/DirectVoltage) get the LONG deadman
/// bound, not the configured drive one and not an exemption: detection
/// legitimately dwells seconds between SetModes (the configured 150 ms
/// would cut a settle wait), but a HUNG measurement must still be cut —
/// DETECTION_ACTIVE suspends the link gate, so this is the only comm
/// failsafe covering it (until 2026-07-06 there was none).
#[test]
fn deadman_long_bound_covers_bench_modes() {
    use crate::motor::failsafe::{FailsafeConfig, FailsafePolicy};

    let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::sensorless(),
        1.0 / 20_000.0,
    );
    driver.set_failsafe(FailsafeConfig {
        policy: FailsafePolicy::Coast,
        staleness_timeout_us: 1_000, // 1 ms drive bound — must NOT apply here
        ..FailsafeConfig::default()
    });

    driver.set_mode(ControlMode::OpenLoop {
        angle_rad: 0.0,
        current: 1.0,
        velocity_rad_s: 0.0,
        pi_gains: None,
    });
    driver.note_command_tick(0);

    // Way past the drive bound but within the bench bound: still alive —
    // a detection settle/sample dwell must survive.
    assert!(!driver.deadman_expired(5_000_000));
    // Past the bench bound: a hung measurement is finally cut.
    assert!(driver.deadman_expired(BENCH_STALENESS_TIMEOUT_US + 1_000));

    // DirectVoltage behaves identically.
    driver.set_mode(ControlMode::DirectVoltage {
        vd: 0.5,
        vq: 0.0,
        angle_rad: 0.0,
    });
    driver.note_command_tick(0);
    assert!(!driver.deadman_expired(5_000_000));
    assert!(driver.deadman_expired(BENCH_STALENESS_TIMEOUT_US + 1_000));
}

/// OpenLoop's `current` rides the open-loop angle, not the angle
/// source's q axis: a stop armed from it must not seed that current as
/// q torque in the source's frame (old: 5 A of q current, direction
/// set by the open-loop sign rather than any rotor state).
#[test]
fn stop_from_open_loop_seeds_no_q_torque() {
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;

    let mut hall = HallSensor::new(1_000_000);
    hall.update(1, 0); // a valid state: rotor parked in its sector
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::with_hall(hall),
        1.0 / 20_000.0,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(20.0));
    driver.set_mode(ControlMode::OpenLoop {
        angle_rad: 0.0,
        current: 5.0,
        velocity_rad_s: 0.0,
        pi_gains: None,
    });
    assert!(driver.enter_brake_ramp());
    let out = driver.step(50).expect("stop step");
    assert!(out.vq.abs() < 1e-3, "stop seeded q torque: vq {}", out.vq);
}

/// Closed-loop: spin a VirtualMotor up on Hall, stop affirming, and the
/// ControlledStop policy must regen-brake it to standstill at the
/// configured deceleration and end Stopped. Drives the failsafe through
/// the real current-control path.
///
/// The plant coasts whenever the driver has the bridge off — it used to
/// be stepped at zero volts, i.e. with the windings SHORTED, which
/// stopped a "coasting" rotor in 36 ms; with the old heavy friction on
/// top, a stop sequence that never braked passed as well.
#[test]
#[cfg(feature = "virtual-motor")]
fn controlled_stop_brakes_spinning_motor_to_standstill() {
    const DT: f32 = 1.0 / 20_000.0;
    const DECEL: f32 = 800.0;
    let (peak, trace, driver) = controlled_stop_run(FailsafePolicy::ControlledStop, DECEL);
    assert!(
        peak > 1_000.0,
        "motor should have spun up, peak ωe = {peak}"
    );
    let below = |w: f32| {
        trace
            .iter()
            .position(|o| o.abs() < w)
            .map(|i| i as f32 * DT)
    };
    assert!(below(25.0).is_some(), "failsafe must brake to ~standstill");
    // This light rotor gets 7000+ el rad/s² per amp: the deceleration
    // bound, not the stop current, shapes the stop.
    let (t_hi, t_lo) = (below(0.75 * peak).unwrap(), below(0.25 * peak).unwrap());
    let decel = 0.5 * peak / (t_lo - t_hi);
    assert!(
        (decel / DECEL - 1.0).abs() < 0.1,
        "mid-stop deceleration {decel} el rad/s² vs bound {DECEL}"
    );
    assert_eq!(
        driver.mode(),
        ControlMode::Stopped,
        "ends Stopped after the brake"
    );
    assert!(
        !driver.failsafe_active(),
        "failsafe cleared at the terminal"
    );
    // The harness must coast an unbraked rotor for far longer.
    let (_, coasted, _) = controlled_stop_run(FailsafePolicy::Coast, DECEL);
    assert!(
        coasted.iter().all(|o| o.abs() > 25.0),
        "a coast stopped within the window"
    );
}

/// Spin up for 1.5 s, stop affirming, run 2 s more. Returns the peak
/// speed, the ωe trace after the last affirmation (one sample per
/// period) and the driver.
#[cfg(feature = "virtual-motor")]
fn controlled_stop_run(
    policy: FailsafePolicy,
    decel_rad_s2: f32,
) -> (
    f32,
    Vec<f32>,
    FocDriver<GatedPwm, MockCurrentSensor, PhaseManager<HallSensor>>,
) {
    use crate::foc::trig::LibmSinCos;
    use crate::motor::failsafe::FailsafeConfig;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams {
        friction_b: 2e-5,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);

    // Prime the hall estimator with the initial rotor state.
    let mut out = motor.step_coast(0.0, DT);
    let mut hall = HallSensor::new(1_000_000); // µs timebase
    hall.update(out.hall_state, 0);
    let mut last_hall = out.hall_state;

    let mgr = PhaseManager::with_hall(hall); // default source = Hall
    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );
    let mut driver = FocDriver::new(
        foc,
        GatedPwm {
            enabled: false,
            applied: [0; 3],
        },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(40.0));
    driver.set_failsafe(FailsafeConfig {
        policy,
        staleness_timeout_us: 5_000,
        brake_current_a: 20.0,
        slew_a_per_s: 1_000.0,
        standstill_rad_s: 20.0,
        decel_rad_s2,
        terminal: FailsafeTerminal::HighZ,
    });
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 3.0,
        id_target: 0.0,
    });

    const STOP_AFFIRMING_AT: u64 = 30_000; // 1.5 s of drive, then go silent
    let mut peak_omega = 0.0f32;
    let mut trace = Vec::new();
    for step in 1..70_000u64 {
        let now = step * 50; // µs

        if out.hall_state != last_hall {
            driver.phase_mut().hall_mut().update(out.hall_state, now);
            last_hall = out.hall_state;
        }
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);

        // Emulate run_foc_cycle's deadman (host affirms, then stops).
        if step < STOP_AFFIRMING_AT {
            driver.note_command_tick(now);
        }
        if driver.deadman_expired(now) {
            driver.enter_failsafe();
        }

        let telem = driver.step(now).unwrap_or_default();
        out = if driver.pwm.enabled {
            motor.step(telem.v_alpha, telem.v_beta, 0.0, DT)
        } else {
            motor.step_coast(0.0, DT)
        };

        if step < STOP_AFFIRMING_AT {
            peak_omega = peak_omega.max(out.omega_e.abs());
        } else {
            trace.push(out.omega_e);
        }
    }
    (peak_omega, trace, driver)
}

/// Closed-loop ramp-into-brake: a user Brake command at speed substitutes
/// the controlled-stop ramp (decel-limited, via the failsafe machinery)
/// and ends parked in `ControlMode::Brake` — without setting the
/// failsafe re-arm latch (user-commanded, no "back to neutral" owed).
#[test]
#[cfg(feature = "virtual-motor")]
fn brake_at_speed_ramps_to_standstill_then_parks() {
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams {
        friction_b: 2e-3,
        ..MotorParams::default()
    };
    let mut motor = VirtualMotor::new(params);

    let mut out = motor.step(0.0, 0.0, 0.0, DT);
    let mut hall = HallSensor::new(1_000_000);
    hall.update(out.hall_state, 0);
    let mut last_hall = out.hall_state;

    let mgr = PhaseManager::with_hall(hall);
    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(40.0));
    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 3.0,
        id_target: 0.0,
    });

    const BRAKE_AT: u64 = 20_000; // 1 s of drive, then the user brakes
    let mut peak_omega = 0.0f32;
    for step in 1..80_000u64 {
        let now = step * 50;
        if step == BRAKE_AT {
            assert!(peak_omega > 50.0, "should be spinning, peak {peak_omega}");
            // What the process_commands gate does for Brake at speed.
            assert!(driver.enter_brake_ramp());
            assert!(
                !driver.failsafe_latched(),
                "user brake must not set the re-arm latch"
            );
        }
        if out.hall_state != last_hall {
            driver.phase_mut().hall_mut().update(out.hall_state, now);
            last_hall = out.hall_state;
        }
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        if step < BRAKE_AT {
            driver.note_command_tick(now);
            peak_omega = peak_omega.max(out.omega_e.abs());
        }
        let telem = driver.step(now).unwrap_or_default();
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
    }

    assert_eq!(
        driver.mode(),
        ControlMode::Brake,
        "ends parked in Brake after the ramp"
    );
    assert!(!driver.failsafe_active());
    assert!(!driver.failsafe_latched());
    assert!(
        out.omega_e.abs() < 25.0,
        "must be ~standstill, ωe = {}",
        out.omega_e
    );
}

/// Closed-loop velocity control: spin a VirtualMotor on Hall via
/// `ControlMode::VelocityControl`, assert it tracks the target, then
/// retarget (no loop reset — the ramp carries it) and track again.
/// Exercises the full cascade: velocity loop → current loop → SVPWM.
#[test]
#[cfg(feature = "virtual-motor")]
fn velocity_control_tracks_target_on_virtual_motor() {
    use crate::foc::hall_sensor::HallSensor;
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::foc::velocity::VelocityLoopConfig;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams::default();
    let mut motor = VirtualMotor::new(params);

    // Prime the hall estimator with the initial rotor state.
    let mut out = motor.step(0.0, 0.0, 0.0, DT);
    let mut hall = HallSensor::new(1_000_000); // µs timebase
    hall.update(out.hall_state, 0);
    let mut last_hall = out.hall_state;

    let mgr = PhaseManager::with_hall(hall); // default source = Hall
    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
        params.r,
        (params.ld + params.lq) / 2.0,
        24.0,
    );
    let mut driver = FocDriver::new(
        foc,
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        mgr,
        DT,
    );
    driver.set_current_limits(CurrentLimits::from_max_current(10.0));
    // Soft gains + a moderate ramp: hall only updates the velocity
    // estimate at edges (~7 ms apart at these speeds), so the loop must
    // not change the speed much within one edge interval — aggressive
    // gains turn the stale estimate into a limit cycle (seen with
    // kp=0.05: ±100 rad/s oscillation around the target).
    driver.set_velocity_config(VelocityLoopConfig {
        kp: 0.008,
        ki: 0.2,
        accel_limit: 400.0, // erad/s²
        accel_ff: 0.0,
    });

    // Track 300 electrical rad/s from standstill (1.5 s), then retarget
    // downward within velocity mode (ramped, no reset) for another 1.5 s.
    const RETARGET_AT: u64 = 30_000;
    driver.set_mode(ControlMode::VelocityControl { target_vel: 300.0 });
    for step in 1..=60_000u64 {
        let now = step * 50; // µs
        if step == RETARGET_AT {
            assert!(
                (out.omega_e - 300.0).abs() < 30.0,
                "should track 300 erad/s, got {}",
                out.omega_e
            );
            // Stay well clear of zero: hall velocity is degenerate
            // through standstill/reversal (edge intervals blow up) —
            // a cruise loop never legitimately crosses zero anyway.
            driver.set_mode(ControlMode::VelocityControl { target_vel: 150.0 });
        }
        if out.hall_state != last_hall {
            driver.phase_mut().hall_mut().update(out.hall_state, now);
            last_hall = out.hall_state;
        }
        driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
        let telem = driver.step(now).unwrap_or_default();
        out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
    }
    assert!(
        (out.omega_e - 150.0).abs() < 25.0,
        "should track 150 erad/s after retarget, got {}",
        out.omega_e
    );
}

/// Severity gate in `run_foc_cycle` (docs/notes/fault-overhaul.md):
/// Warning never touches the motor, GracefulStop routes through the
/// failsafe machinery, Kill cuts PWM and latches Error; the deadman
/// raises CommTimeout and a fresh SetMode clears it.
#[cfg(feature = "runtime")]
mod severity_gate {
    use super::*;
    use crate::foc::fault::{FaultCategory, FaultRegistry, PlatformFault};
    use crate::foc::hall_sensor::HallFaultKind;
    use crate::foc::phase::PhaseManager;
    use crate::foc::trig::LibmSinCos;
    use crate::state::{
        CMD_CHANNEL, DriverCommand, MotorControlState, process_commands, run_foc_cycle,
    };
    use crate::types::MotorState;
    use core::cell::RefCell;
    use critical_section::Mutex as CriticalSectionMutex;

    #[derive(Clone, Copy, PartialEq, Debug)]
    enum SevFault {
        OverCurrent,
        OverVoltage,
        OverTemp,
        Hall,
        CommTimeout,
        Derating,
    }
    impl PlatformFault for SevFault {
        fn category(&self) -> FaultCategory {
            match self {
                Self::OverCurrent => FaultCategory::OverCurrent,
                Self::OverVoltage => FaultCategory::OverVoltage,
                Self::OverTemp => FaultCategory::OverTemp,
                Self::Hall => FaultCategory::HallError,
                Self::CommTimeout => FaultCategory::CommTimeout,
                Self::Derating => FaultCategory::Derating,
            }
        }
        fn details(&self) -> heapless::String<128> {
            heapless::String::new()
        }
        fn from_hall_kind(_kind: HallFaultKind) -> Option<Self> {
            Some(Self::Hall)
        }
        fn from_category(category: FaultCategory) -> Option<Self> {
            match category {
                FaultCategory::OverCurrent => Some(Self::OverCurrent),
                FaultCategory::OverVoltage => Some(Self::OverVoltage),
                FaultCategory::OverTemp => Some(Self::OverTemp),
                FaultCategory::CommTimeout => Some(Self::CommTimeout),
                FaultCategory::Derating => Some(Self::Derating),
                _ => None,
            }
        }
    }

    const BOARD: BoardConfig = BoardConfig {
        calib: crate::types::BoardCalib {
            shunt_ohms: 0.003,
            amp_gain: 16.0,
            adc_vref_mv: 3300,
            adc_max_counts: 4095,
            invert_current_sign: false,
            vbus_divider_ratio: 10.39,
        },
        max_iq_target_a: 10.0,
        max_phase_current_a: 40.0,
        max_vbus_mv: 60_000,
        min_vbus_mv: 8_000,
        max_fet_temp_c: 100.0,
        max_motor_temp_c: 120.0,
        phase_sense: None,
    };

    struct Harness {
        state: CriticalSectionMutex<RefCell<MotorControlState>>,
        registry: FaultRegistry<SevFault>,
        driver: FocDriver<MockPwm, MockCurrentSensor, PhaseManager, LibmSinCos>,
    }

    /// Driver running CurrentControl (entered via the command channel at
    /// t=0, which also stamps the deadman), link active, Manual angle.
    fn running_harness() -> Harness {
        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<SevFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
        let mut h = Harness {
            state,
            registry,
            driver,
        };
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        let out = h.cycle(0);
        assert!(out.is_some(), "harness must start cleanly");
        assert!(matches!(
            h.driver.mode(),
            ControlMode::CurrentControl { .. }
        ));
        h
    }

    impl Harness {
        fn cycle(&mut self, now_ticks: u64) -> Option<FocOutput> {
            run_foc_cycle(
                &self.state,
                &self.registry,
                &mut self.driver,
                24.0,
                now_ticks,
                &BOARD,
            )
        }

        fn motor_state(&self) -> MotorState {
            critical_section::with(|cs| self.state.borrow(cs).borrow().motor_state)
        }
    }

    #[test]
    fn warning_fault_keeps_motor_running() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();

        h.registry.set(SevFault::Hall);
        let out = h.cycle(50);

        assert!(out.is_some(), "warning must not skip the FOC step");
        assert!(matches!(
            h.driver.mode(),
            ControlMode::CurrentControl { .. }
        ));
        assert!(!h.driver.failsafe_active(), "warning must not arm failsafe");
        assert_eq!(h.motor_state(), MotorState::Running);
    }

    #[test]
    fn braking_a_coasting_stopped_rotor_keeps_flash_gate_busy() {
        use crate::foc::phase::{PhaseInput, PhaseOutput};
        use crate::state::BOOT_CURRENT_OFFSET_PENDING;
        use core::sync::atomic::Ordering;
        struct SpinningPhase;
        impl PhaseProvider for SpinningPhase {
            fn get(&self) -> PhaseOutput {
                PhaseOutput {
                    angle: 0.0,
                    velocity: 100.0,
                }
            }
            fn update(&mut self, _: &PhaseInput, _: u64) {}
        }
        let _serial = cmd_channel_lock();
        let state = CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
        let registry = FaultRegistry::<SevFault>::new();
        let mut driver = FocDriver::new(
            FocController::<SvpwmModulator>::new(24.0),
            GatedPwm {
                enabled: false,
                applied: [0; 3],
            },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            SpinningPhase,
            0.00005,
        );
        // Boot has completed; it must not mask the state bug in busy().
        let boot_pending = BOOT_CURRENT_OFFSET_PENDING.swap(false, Ordering::AcqRel);
        assert!(!critical_section::with(|cs| state
            .borrow(cs)
            .borrow()
            .actuator_busy()));
        CMD_CHANNEL
            .try_send(DriverCommand::SetMode(ControlMode::Brake))
            .unwrap();
        for n in 1..=100 {
            assert!(run_foc_cycle(&state, &registry, &mut driver, 24.0, n * 50, &BOARD).is_some());
        }
        BOOT_CURRENT_OFFSET_PENDING.store(boot_pending, Ordering::Release);
        assert!(driver.failsafe_active() && driver.pwm().enabled);
        critical_section::with(|cs| {
            let s = state.borrow(cs).borrow();
            assert_eq!(s.motor_state, MotorState::Running);
            assert_eq!(s.control_mode, ControlMode::Brake);
            assert!(s.actuator_busy());
        });
    }

    #[test]
    fn invalid_wired_temperature_trips_protection() {
        use crate::foc::config::NtcConfig;
        let _serial = cmd_channel_lock();
        let mut h = running_harness();
        h.driver.run_protection(
            &h.registry,
            &BOARD,
            24.0,
            Some(NtcConfig::INVALID_TEMP_C_X10),
            None,
        );
        assert!(h.registry.has_category(FaultCategory::OverTemp));
    }

    #[test]
    fn warning_fault_does_not_block_start() {
        let _serial = cmd_channel_lock();
        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<SevFault> = FaultRegistry::new();
        registry.set(SevFault::Hall);
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::sensorless(),
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        process_commands(&state, &mut driver, &registry);
        assert!(
            matches!(driver.mode(), ControlMode::CurrentControl { .. }),
            "a warning-class fault must not block starting"
        );
    }

    #[test]
    fn graceful_stop_fault_arms_failsafe_not_high_z() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();

        h.registry.set(SevFault::OverTemp);
        let out = h.cycle(50);

        assert!(h.driver.failsafe_active(), "GracefulStop must arm failsafe");
        assert!(out.is_some(), "failsafe drives through the normal step");
        assert_ne!(
            h.motor_state(),
            MotorState::Error,
            "GracefulStop must not latch Error"
        );

        // Restart stays blocked while the fault is active (start gate).
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
        h.cycle(100);
        assert!(matches!(h.driver.mode(), ControlMode::Stopped));
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        h.cycle(150);
        assert!(
            matches!(h.driver.mode(), ControlMode::Stopped),
            "stopping-class fault must block restart"
        );
    }

    #[test]
    fn kill_fault_cuts_pwm_and_latches_error() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();

        h.registry.set(SevFault::OverCurrent);
        let out = h.cycle(50);

        assert!(out.is_none(), "Kill must skip the FOC step");
        assert!(matches!(h.driver.mode(), ControlMode::Stopped));
        assert_eq!(h.motor_state(), MotorState::Error);
    }

    /// The hall bridge is STICKY: the warning lands in the registry on
    /// degradation, the motor keeps running (warning class), and the
    /// record survives the hall's recovery — only a host clear removes
    /// it, while the live fallback behavior recovers immediately.
    #[test]
    fn hall_degradation_bridges_sticky_warning() {
        let _serial = cmd_channel_lock();
        use crate::foc::hall_sensor::HallSensor;

        let state: CriticalSectionMutex<RefCell<MotorControlState>> =
            CriticalSectionMutex::new(RefCell::new(MotorControlState::new()));
        let registry: FaultRegistry<SevFault> = FaultRegistry::new();
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut hall = HallSensor::new(1_000_000);
        hall.update(1, 0); // boot seed: healthy, valid sample from t=0
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            PhaseManager::with_hall(hall), // default source = Hall
            1.0 / 20_000.0,
        );
        critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());

        let cycle = |driver: &mut FocDriver<_, _, _, _>, now: u64| {
            run_foc_cycle(&state, &registry, driver, 24.0, now, &BOARD)
        };

        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
            iq_target: 1.0,
            id_target: 0.0,
        }));
        cycle(&mut driver, 0);
        assert!(!registry.has_category(FaultCategory::HallError));

        // Hall dies: invalid state (cable cut, pull-ups → 0b111).
        driver.phase_mut().hall_mut().update(0b111, 10);
        cycle(&mut driver, 50);
        assert!(
            registry.has_category(FaultCategory::HallError),
            "degradation must bridge into the registry"
        );
        assert!(
            matches!(driver.mode(), ControlMode::CurrentControl { .. }),
            "warning class: the motor keeps running"
        );

        // Hall recovers: behavior follows, the record stays.
        driver.phase_mut().hall_mut().update(1, 100);
        cycle(&mut driver, 150);
        assert!(
            registry.has_category(FaultCategory::HallError),
            "the warning is sticky — only a host clear removes it"
        );
        assert!(matches!(driver.mode(), ControlMode::CurrentControl { .. }));
    }

    #[test]
    fn deadman_raises_comm_timeout_and_fresh_command_clears_it() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();

        // Quiet link: within the engage grace window (the relaxed
        // STARTUP_STALENESS_TIMEOUT_US bound after ANY engage — the
        // engage ISR transient AND the host's Start-ack round trip both
        // starve the command pump) even a >150 ms staleness must NOT
        // trip.
        h.cycle(STARTUP_STALENESS_TIMEOUT_US - 100_000);
        assert!(
            !h.registry.has_category(FaultCategory::CommTimeout),
            "engage grace must hold the deadman through the engage window"
        );

        // Past the grace window with the link still quiet → trip.
        h.cycle(STARTUP_STALENESS_TIMEOUT_US + 200_000);
        assert!(
            h.registry.has_category(FaultCategory::CommTimeout),
            "stale command link must raise CommTimeout"
        );
        assert!(
            h.driver.failsafe_active(),
            "CommTimeout severity must arm the failsafe via the gate"
        );
        assert_ne!(h.motor_state(), MotorState::Error);

        // Commands flowing again (even just an ack) clear the fault; the
        // re-arm latch still demands the explicit safe mode it received.
        let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
        h.cycle(200_050);
        assert!(
            !h.registry.has_category(FaultCategory::CommTimeout),
            "a drained SetMode proves liveness and clears CommTimeout"
        );
        assert!(matches!(h.driver.mode(), ControlMode::Stopped));
    }

    /// Voltage faults trip on INTEGRATED excursion, not single samples
    /// (run_protection): one over-voltage cycle (a regen spike / sense
    /// blip) must ride through; a sustained excursion must Kill.
    #[test]
    fn voltage_fault_integrates_instead_of_single_sample() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();

        // One 50 µs cycle at +10 V over the 60 V board limit:
        // 10 V · 50 µs = 0.5 mV·s, under the 3 mV·s trip — no fault.
        run_cycle_at_vbus(&mut h, 70.0, 50);
        assert!(
            !h.registry.has_category(FaultCategory::OverVoltage),
            "a single-sample excursion must not trip"
        );
        // Back in range, the integral decays (τ ≈ 5 ms ≫ one cycle).
        h.cycle(100);

        // Sustained excursion: trips within ~0.3 ms (VESC-equivalent).
        let mut now = 150;
        for _ in 0..20 {
            run_cycle_at_vbus(&mut h, 70.0, now);
            now += 50;
        }
        assert!(
            h.registry.has_category(FaultCategory::OverVoltage),
            "sustained overvoltage must trip"
        );
        assert!(matches!(h.driver.mode(), ControlMode::Stopped), "OV = Kill");
        assert_eq!(h.motor_state(), MotorState::Error);
    }

    fn run_cycle_at_vbus(h: &mut Harness, vbus: f32, now: u64) {
        run_foc_cycle(&h.state, &h.registry, &mut h.driver, vbus, now, &BOARD);
    }

    /// The derating warning follows the live scales with hysteresis
    /// (set < 0.8, clear > 0.95), and the warning never stops the motor.
    #[test]
    fn derating_warning_sets_and_clears_with_hysteresis() {
        let _serial = cmd_channel_lock();
        let mut h = running_harness();
        h.driver.set_derating(DeratingConfig {
            vbus_cut_start_v: 22.0,
            vbus_cut_end_v: 18.0,
            ..Default::default()
        });

        // Sagged bus (drive scale 0.25 at 19 V): the decimated update
        // fires within 256 cycles.
        let mut now = 50;
        for _ in 0..300 {
            run_cycle_at_vbus(&mut h, 19.0, now);
            now += 50;
        }
        assert!(
            h.registry.has_category(FaultCategory::Derating),
            "deep derate must raise the warning (scales {:?})",
            h.driver.derating()
        );
        assert!(
            matches!(h.driver.mode(), ControlMode::CurrentControl { .. }),
            "warning class: the motor keeps running"
        );
        assert!(h.driver.derating().drive < 0.5);

        // Bus recovered: warning auto-clears (a live state, unlike the
        // sticky hall record).
        for _ in 0..300 {
            run_cycle_at_vbus(&mut h, 24.0, now);
            now += 50;
        }
        assert!(
            !h.registry.has_category(FaultCategory::Derating),
            "derating warning must auto-clear on recovery"
        );
        assert_eq!(h.driver.derating(), DeratingScales::IDENTITY);
    }
}

/// Derating scales clamp the iq budget by DIRECTION: drive (iq·ω ≥ 0)
/// takes `drive`, opposing torque takes `brake` — a speed/sag derate
/// must never weaken the brakes.
#[test]
#[cfg(feature = "runtime")]
fn derating_clamps_drive_and_brake_separately() {
    use crate::foc::phase::{PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;

    let make = |scales: DeratingScales, iq: f32| {
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(24.0);
        let mut mgr = PhaseManager::sensorless();
        mgr.set_source(PhaseSource::OpenLoop).unwrap();
        mgr.set_open_loop_velocity(100.0);
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            1.0 / 20_000.0,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(10.0));
        driver.set_derating_scales(scales);
        // Warmup at zero target: latches the phase velocity into the
        // managed output without accumulating PI state.
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 0.0,
            id_target: 0.0,
        });
        driver.step(50).unwrap();
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: iq,
            id_target: 0.0,
        });
        driver.step(100).unwrap().vq
    };

    let full = make(DeratingScales::IDENTITY, 10.0);
    let derated = make(
        DeratingScales {
            drive: 0.5,
            ..DeratingScales::IDENTITY
        },
        10.0,
    );
    assert!(
        (derated - 0.5 * full).abs() < 0.05 * full.abs(),
        "drive iq must be halved: full vq {full}, derated vq {derated}"
    );

    // Braking (iq opposes ω): only the brake scale applies — full here.
    let full_brake = make(DeratingScales::IDENTITY, -10.0);
    let brake_with_drive_derate = make(
        DeratingScales {
            drive: 0.5,
            ..DeratingScales::IDENTITY
        },
        -10.0,
    );
    assert!(
        (brake_with_drive_derate - full_brake).abs() < 0.05 * full_brake.abs(),
        "a drive derate must not weaken the brake: {full_brake} vs {brake_with_drive_derate}"
    );
}

/// Steady-state commutation timing, measured at the driver's own
/// sampling instants.
#[cfg(feature = "virtual-motor")]
struct FrameTiming {
    /// Mean (Park angle − rotor angle at the current sample), in PWM
    /// periods of rotor travel.
    frame_err_periods: f32,
    /// Mean TRUE dq current (plant currents Park-transformed with the
    /// plant's rotor angle, both at the sample instant).
    true_id: f32,
    true_iq: f32,
}

/// Closed loop FocDriver + near-continuous plant (16 substeps) whose
/// actuation delay matches the driver's configured one, on a
/// Flipsky-5065-class motor (low R and L — the regime where frame
/// errors cost the most current). 10 A of torque current against a
/// viscous load settles at ~1500 el rad/s (~2000 rpm) — a steady
/// operating point with no speed loop in the way. Spins up on hall
/// (edges time-stamped at the true crossing, like the capture timer),
/// then switches to `source` and settles.
/// Closed-loop rig: FocDriver (hall source + a back-EMF observer + the
/// f405 phase tracker) against a near-continuous (16-substep)
/// Flipsky-5065-class plant whose actuation delay matches the
/// driver's. Hall edges are time-stamped at the true boundary crossing,
/// like the capture timer.
#[cfg(feature = "virtual-motor")]
struct HallRig {
    motor: VirtualMotor,
    out: VirtualMotorOutput,
    driver: FocDriver<GatedPwm, MockCurrentSensor, PhaseManager<HallSensor>>,
    last_hall: u8,
    prev_angle: f32,
    step: u64,
    /// Feed hall edges to the driver (false = a dead sensor).
    hall_alive: bool,
    /// Remaining steps of an injected invalid-state (0b000) glitch.
    glitch_steps: u32,
    /// Coulomb load (N·m, opposes motion).
    load: f32,
    /// Constant torque of a slope (N·m, same sign convention as a
    /// load: positive pulls toward negative ω).
    slope: f32,
}

#[cfg(feature = "virtual-motor")]
impl HallRig {
    const DT: f32 = 1.0 / 20_000.0;
    const TICKS: u64 = 50;

    fn new(delay: f32, friction_b: f32) -> Self {
        let params = MotorParams {
            r: 0.036,
            ld: 22e-6,
            lq: 22e-6,
            lambda: 2.92e-3,
            pole_pairs: 7,
            j: 4e-4,
            friction_b,
            substeps: 16,
            actuation_delay: delay,
            ..MotorParams::default()
        };
        let mut motor = VirtualMotor::new(params);
        let out = motor.step(0.0, 0.0, 0.0, Self::DT);
        let mut hall = HallSensor::new(1_000_000);
        hall.update(out.hall_state, 0);
        let mut mgr = PhaseManager::with_hall(hall);
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            params.ld,
            params.lambda,
        )));
        mgr.set_phase_tracker(100.0, 1.2); // the f405 boot configuration
        // As from_runtime_config builds it on the f405: params-derived
        // PI gains plus dq decoupling / back-EMF feedforward.
        let mut foc = FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
            params.r, params.ld, 24.0,
        );
        foc.set_decoupling(Some(Decoupling {
            ld_h: params.ld,
            lq_h: params.lq,
            flux_linkage_wb: params.lambda,
            resistance_ohm: params.r,
        }));
        let mut driver = FocDriver::new(
            foc,
            GatedPwm {
                enabled: false,
                applied: [0; 3],
            },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            Self::DT,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(30.0));
        driver.set_actuation_delay(delay);
        Self {
            motor,
            out,
            driver,
            last_hall: out.hall_state,
            prev_angle: out.angle_rad,
            step: 0,
            hall_alive: true,
            glitch_steps: 0,
            load: 0.0,
            slope: 0.0,
        }
    }

    /// Hold the hall lines at the invalid all-low state for `steps`
    /// periods (a connector bounce); the real state returns after.
    fn glitch(&mut self, steps: u32) {
        let now = (self.step + 1) * Self::TICKS;
        self.driver.phase_mut().hall_mut().update(0, now);
        self.glitch_steps = steps;
    }

    /// One FOC period: (the plant state the driver sampled, its output).
    fn step(&mut self) -> (VirtualMotorOutput, FocOutput) {
        self.step += 1;
        let now = self.step * Self::TICKS;
        if self.glitch_steps > 0 {
            self.glitch_steps -= 1;
            if self.glitch_steps == 0 {
                // The lines settle on whatever sector the rotor is in.
                self.driver
                    .phase_mut()
                    .hall_mut()
                    .update(self.out.hall_state, now);
                self.last_hall = self.out.hall_state;
            }
        } else if self.hall_alive && self.out.hall_state != self.last_hall {
            // Crossing of the hall boundary (π/6 + k·π/3) inside the step.
            let a0 = self.prev_angle;
            let a1 = a0 + angle_difference(self.out.angle_rad, a0);
            let k0 = libm::floorf((a0 - FRAC_PI_6) / FRAC_PI_3);
            let k1 = libm::floorf((a1 - FRAC_PI_6) / FRAC_PI_3);
            let frac = if k0 == k1 {
                1.0
            } else {
                clamp_f32(
                    (FRAC_PI_6 + FRAC_PI_3 * k0.max(k1) - a0) / (a1 - a0),
                    0.0,
                    1.0,
                )
            };
            let t = now - ((1.0 - frac) * Self::TICKS as f32) as u64;
            self.driver
                .phase_mut()
                .hall_mut()
                .update(self.out.hall_state, t);
            self.last_hall = self.out.hall_state;
        }
        let sample = self.out;
        self.driver.current_sensor_mut().currents = (sample.ia, sample.ib, sample.ic);
        let telem = self.driver.step(now).expect("FOC step failed");
        self.prev_angle = sample.angle_rad;
        let load = self.load * libm::tanhf(sample.omega_e / 10.0) + self.slope;
        // A disabled bridge floats the phases: the plant coasts. (Zero
        // volts would be all windings shorted — a brake.)
        self.out = if self.driver.pwm.enabled {
            self.motor.step(telem.v_alpha, telem.v_beta, load, Self::DT)
        } else {
            self.motor.step_coast(load, Self::DT)
        };
        (sample, telem)
    }

    fn run(&mut self, seconds: f32) {
        for _ in 0..(seconds / Self::DT) as u32 {
            self.step();
        }
    }
}

/// Position control moves the rotor 20 electrical turns and parks it
/// there: no overshoot past a hall sector, standing still at the end.
#[test]
#[cfg(feature = "virtual-motor")]
fn position_control_moves_and_parks() {
    for (pkp, vkp, vki, ff) in [
        (2.0, 0.01, 0.2, 0.0),
        (1.0, 0.01, 0.2, 0.0),
        (5.0, 0.05, 0.5, 1.0 / 536.0),
        (10.0, 0.05, 0.5, 1.0 / 536.0),
        (10.0, 0.1, 1.0, 1.0 / 536.0),
    ] {
        let mut rig = HallRig::new(0.5, 2e-5);
        rig.driver.set_position_config(PositionLoopConfig {
            kp: pkp,
            max_speed: 1000.0,
        });
        rig.driver.set_velocity_config(VelocityLoopConfig {
            kp: vkp,
            ki: vki,
            accel_limit: 500.0,
            accel_ff: ff,
        });
        let target = 20.0 * core::f32::consts::TAU;
        rig.driver
            .set_mode(ControlMode::PositionControl { target_pos: target });
        let mut peak = f32::NEG_INFINITY;
        let mut worst_late = 0.0f32;
        let mut true_lo = f32::INFINITY;
        let mut true_hi = f32::NEG_INFINITY;
        let mut true_pos = 0.0f32;
        let mut prev = rig.out.angle_rad;
        for i in 0..(4.0 / HallRig::DT) as u32 {
            rig.step();
            true_pos += angle_difference(rig.out.angle_rad, prev);
            prev = rig.out.angle_rad;
            let pos = rig.driver.position().as_rad();
            peak = peak.max(pos);
            if i as f32 * HallRig::DT > 3.0 {
                worst_late = worst_late.max((pos - target).abs());
                true_lo = true_lo.min(true_pos - target);
                true_hi = true_hi.max(true_pos - target);
            }
        }
        eprintln!("DBG   true rotor band over the last second: [{true_lo:.2}, {true_hi:.2}] rad");
        eprintln!(
            "DBG pkp {pkp} vkp {vkp} vki {vki} ff {ff}: overshoot {:.2} late err {:.2} ω {:.1}",
            peak - target,
            worst_late,
            rig.out.omega_e
        );
    }
}

/// Cruise control down a slope on a full pack (regen derate at 0): the
/// plain derate cuts the brake and the board runs away; dissipation
/// holds the cruise speed and pushes (next to) nothing into the pack.
#[test]
#[cfg(feature = "virtual-motor")]
fn full_pack_descent_holds_its_cruise_by_dissipation() {
    // (final ω, mean power into the supply over the last second)
    let descent = |max_dissipation_w| {
        let mut rig = HallRig::new(0.5, 2e-5);
        rig.driver.set_current_limits(CurrentLimits {
            max_dissipation_w,
            ..CurrentLimits::from_max_current(30.0)
        });
        rig.driver
            .set_mode(ControlMode::VelocityControl { target_vel: 800.0 });
        rig.run(2.0);
        // The pack is full and the road tips down: holding 800 el
        // rad/s takes ~4.9 A of brake, ~16 W of regen to burn.
        rig.driver.set_derating_scales(DeratingScales {
            regen: 0.0,
            ..DeratingScales::IDENTITY
        });
        rig.slope = -0.15;
        rig.run(1.0);
        let mut charged_j = 0.0;
        for _ in 0..(1.0 / HallRig::DT) as u32 {
            let (_, t) = rig.step();
            charged_j -= 1.5 * (t.vd * t.id + t.vq * t.iq) * HallRig::DT;
        }
        (rig.out.omega_e, charged_j)
    };
    let (omega, _) = descent(0.0);
    assert!(omega > 2_000.0, "the plain cut must run away: ω {omega}");
    let (omega, charge_w) = descent(50.0);
    assert!((omega - 800.0).abs() < 30.0, "ω {omega}");
    assert!(charge_w.abs() < 2.0, "mean charge {charge_w} W");
}

/// The parking brake entered at 1500 el rad/s on a 5065: the plain
/// short would drive toward ~90 A (λω/|Z|) and trip the 39 A
/// overcurrent, leaving the rotor to coast. It turns current-controlled
/// at `max_brake_a` instead, brakes the rotor down, and returns to the
/// short once slow.
#[test]
#[cfg(feature = "virtual-motor")]
fn park_brake_regulates_when_pushed_past_its_limit() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver.set_current_limits(CurrentLimits {
        max_brake_a: 10.0,
        ..CurrentLimits::from_max_current(30.0)
    });
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 20.0,
        id_target: 0.0,
    });
    rig.run(0.2);
    assert!(rig.out.omega_e > 1_200.0, "ω {}", rig.out.omega_e);
    rig.driver.set_mode(ControlMode::Brake);
    let mut peak = 0.0f32;
    for _ in 0..(2.0 / HallRig::DT) as u32 {
        rig.step();
        peak = peak
            .max(rig.out.ia.abs())
            .max(rig.out.ib.abs())
            .max(rig.out.ic.abs());
    }
    assert_eq!(rig.driver.mode(), ControlMode::Brake);
    assert!(rig.out.omega_e.abs() < 50.0, "ω {}", rig.out.omega_e);
    assert!(peak < 25.0, "peak phase current {peak} A");
    assert!(rig.driver.park_iq.is_none(), "back on the short");
}

/// Parked on a slope the short cannot hold within `max_brake_a` (the
/// slope needs ~13 A of short current, the limit is 10 A): the brake
/// regulates at the limit and never lets go, where the plain short
/// kept growing with the speed it let the board reach.
#[test]
#[cfg(feature = "virtual-motor")]
fn park_brake_keeps_braking_a_slope_it_cannot_short() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver.set_current_limits(CurrentLimits {
        max_brake_a: 10.0,
        ..CurrentLimits::from_max_current(30.0)
    });
    rig.driver.set_mode(ControlMode::Brake);
    rig.slope = -0.4;
    let mut peak = 0.0f32;
    for _ in 0..(1.0 / HallRig::DT) as u32 {
        let (_, t) = rig.step();
        peak = peak.max(t.iq.abs());
    }
    let (_, t) = rig.step();
    assert_eq!(rig.driver.mode(), ControlMode::Brake);
    assert!(rig.driver.park_iq.is_some(), "regulating");
    assert!((t.iq + 10.0).abs() < 1.0, "holding {} A", t.iq);
    // The hand-over from the short overshoots by what the short adds
    // during the actuation pipeline.
    assert!(peak < 14.0, "peak {peak} A");
}

/// Sensorless with no trustworthy angle (the observer never locked):
/// the short still brakes a slow rotor, and releases to high-Z — not a
/// trip — when its current says the rotor is too fast for it.
#[test]
#[cfg(feature = "virtual-motor")]
fn park_brake_without_an_angle_guards_itself_by_its_current() {
    use crate::foc::phase::{BackEmfObserver, Observer, PhaseManager, PhaseSource};
    use crate::foc::trig::LibmSinCos;
    use crate::virtual_motor::{MotorParams, VirtualMotor};

    const DT: f32 = 1.0 / 20_000.0;
    let params = MotorParams {
        r: 0.036,
        ld: 22e-6,
        lq: 22e-6,
        lambda: 2.92e-3,
        pole_pairs: 7,
        j: 4e-4,
        friction_b: 2e-5,
        substeps: 8,
        actuation_delay: 0.5,
        ..MotorParams::default()
    };
    let run = |omega0: f32| {
        let mut motor = VirtualMotor::new(params);
        motor.set_velocity(omega0);
        let mut mgr = PhaseManager::sensorless();
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            params.ld,
            params.lambda,
        )));
        mgr.set_source(PhaseSource::Observer).unwrap();
        let mut driver = FocDriver::new(
            FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(
                params.r, params.ld, 24.0,
            ),
            GatedPwm {
                enabled: false,
                applied: [0; 3],
            },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT,
        );
        driver.set_current_limits(CurrentLimits {
            max_brake_a: 15.0,
            ..CurrentLimits::from_max_current(30.0)
        });
        driver.set_mode(ControlMode::Brake);
        let mut out = motor.step_coast(0.0, DT);
        for step in 1..4_000u64 {
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            driver.step(step * 50).expect("no trip");
            out = if driver.pwm.enabled {
                motor.step(0.0, 0.0, 0.0, DT)
            } else {
                motor.step_coast(0.0, DT)
            };
        }
        assert!(!driver.phase().angle_trustworthy());
        (driver.mode(), out.omega_e)
    };
    // 100 el rad/s: ~8 A of short current — held, braked to a stop.
    let (mode, omega) = run(100.0);
    assert_eq!(mode, ControlMode::Brake);
    assert!(omega.abs() < 20.0, "ω {omega}");
    // 1500 el rad/s: the short heads for ~90 A — released at once.
    let (mode, omega) = run(1_500.0);
    assert_eq!(mode, ControlMode::Stopped);
    assert!(omega > 1_400.0, "barely touched: ω {omega}");
}

/// Link lost on a descent the failsafe brake cannot beat (the slope's
/// 0.6 N·m against the 15 A brake's 0.46): the board keeps speeding up,
/// and the stop keeps braking all the way. It used to give up to a
/// coast after 2 s without progress — and the rotor then ran away to
/// 3× top speed within a second.
#[test]
#[cfg(feature = "virtual-motor")]
fn failsafe_keeps_braking_a_descent_it_cannot_beat() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 8.0,
        id_target: 0.0,
    });
    rig.run(0.4);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 0.0,
        id_target: 0.0,
    });
    rig.load = -0.6;
    rig.driver.enter_failsafe();
    let start = rig.out.omega_e;
    for _ in 0..(5.0 / HallRig::DT) as u32 {
        rig.step();
        assert!(rig.driver.failsafe_active(), "the stop let go");
    }
    let (_, t) = rig.step();
    assert!(
        rig.out.omega_e > start,
        "the slope wins: {}",
        rig.out.omega_e
    );
    // At least the commanded 15 A (past top speed the back-EMF pushes
    // some regen beyond what the current loop holds).
    assert!(t.iq < -14.0, "braking at {} A", t.iq);
}

/// Braking has its own limit: a remote brake command past `max_brake_a`
/// is held there while the same magnitude of drive is granted in full.
#[test]
#[cfg(feature = "virtual-motor")]
fn braking_is_held_to_its_own_limit() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver.set_current_limits(CurrentLimits {
        max_brake_a: 8.0,
        ..CurrentLimits::from_max_current(30.0)
    });
    let torque = |iq_target| ControlMode::CurrentControl {
        iq_target,
        id_target: 0.0,
    };
    rig.driver.set_mode(torque(20.0));
    rig.run(0.1);
    let (_, t) = rig.step();
    assert!((t.iq - 20.0).abs() < 1.5, "drive iq {}", t.iq);
    rig.driver.set_mode(torque(-20.0));
    rig.run(0.05);
    let (_, t) = rig.step();
    assert!(rig.out.omega_e > 100.0, "still rolling forward");
    assert!((t.iq + 8.0).abs() < 1.0, "brake iq {}", t.iq);
}

/// Cruise → Brake at speed (the controlled brake ramp) → cruise again
/// at the speed the ramp had reached. The ramp runs under the unchanged
/// VelocityControl mode, so the resumed cruise used to count as a
/// retarget and keep the pre-brake loop state: the reference still at
/// the old speed drove the rotor back up toward it.
#[test]
#[cfg(feature = "virtual-motor")]
fn cruise_resumed_mid_brake_ramp_starts_from_the_rotor() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver
        .set_mode(ControlMode::VelocityControl { target_vel: 1000.0 });
    rig.run(3.0);
    assert!(rig.out.omega_e > 950.0, "cruise ω {}", rig.out.omega_e);
    assert!(rig.driver.enter_brake_ramp());
    rig.run(0.3);
    let resumed_at = rig.out.omega_e;
    assert!(resumed_at < 850.0, "brake ramp ω {resumed_at}");
    rig.driver.set_mode(ControlMode::VelocityControl {
        target_vel: resumed_at,
    });
    let mut peak = resumed_at;
    for _ in 0..(0.5 / HallRig::DT) as u32 {
        rig.step();
        peak = peak.max(rig.out.omega_e);
    }
    assert!(
        peak < resumed_at + 30.0,
        "resumed at {resumed_at} but climbed to {peak}"
    );
}

/// Cruise commanded past top speed (the bus voltage caps it at ~4700
/// el rad/s here), then told to slow down. The reference used to run on
/// to the unreachable target and had to ramp all the way back first:
/// the motor held top speed for 6.6 s after the lower command. Top
/// speed itself must not suffer for the fix.
#[test]
#[cfg(feature = "virtual-motor")]
fn cruise_past_top_speed_answers_a_lower_target_at_once() {
    // Top speed at full torque, for reference.
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 30.0,
        id_target: 0.0,
    });
    rig.run(1.5);
    let top = rig.out.omega_e;

    let mut rig = HallRig::new(0.5, 2e-5);
    rig.driver
        .set_mode(ControlMode::VelocityControl { target_vel: 8000.0 });
    rig.run(12.0);
    let cruise = rig.out.omega_e;
    assert!(cruise > 0.995 * top, "cruise {cruise} vs top speed {top}");
    rig.driver
        .set_mode(ControlMode::VelocityControl { target_vel: 2000.0 });
    rig.run(0.5);
    assert!(
        rig.out.omega_e < cruise - 100.0,
        "0.5 s after the lower command: ω {} (was {cruise})",
        rig.out.omega_e
    );
}

/// The cruise loop must see the bus-current clamp as its own bound: it
/// used to bank 10.8 A of command against the ~1 A the clamp let
/// through, then overshoot the target by 6%.
#[test]
#[cfg(feature = "virtual-motor")]
fn cruise_loop_sees_the_bus_current_clamp() {
    let mut rig = HallRig::new(0.5, 2e-5);
    let mut limits = CurrentLimits::from_max_current(30.0);
    limits.bus_in_max_a = 0.5;
    rig.driver.set_current_limits(limits);
    rig.driver
        .set_mode(ControlMode::VelocityControl { target_vel: 3000.0 });
    let (mut peak, mut banked) = (0.0f32, 0.0f32);
    for _ in 0..(10.0 / HallRig::DT) as u32 {
        let (_, telem) = rig.step();
        peak = peak.max(rig.out.omega_e);
        banked = banked.max(rig.driver.velocity_loop.last_iq() - telem.iq);
    }
    assert!(
        (rig.out.omega_e - 3000.0).abs() < 10.0,
        "ω {}",
        rig.out.omega_e
    );
    assert!(peak < 3030.0, "overshoot to {peak}");
    assert!(
        banked < 2.0,
        "commanded {banked} A beyond the delivered current"
    );
}

/// A hall-driven motor must restart after it came to a stop. The
/// sensor's staleness check compares the silence against the speed at
/// the LAST edge, so every stop used to be declared a dead sensor a few
/// sector-times later: the recovery override took commutation, the
/// angle turned untrustworthy and the torque gate held iq at zero —
/// the motor stayed put until someone turned it by hand.
#[test]
#[cfg(feature = "virtual-motor")]
fn hall_drive_restarts_after_a_stop() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.load = 0.2;
    let torque = |iq| ControlMode::CurrentControl {
        iq_target: iq,
        id_target: 0.0,
    };
    rig.driver.set_mode(torque(10.0));
    rig.run(0.5);
    rig.driver.set_mode(torque(0.0));
    rig.run(1.0); // coasts to a stop well within this
    assert!(
        rig.out.omega_e.abs() < 5.0,
        "should have stopped, ω {}",
        rig.out.omega_e
    );
    rig.driver.set_mode(torque(10.0));
    rig.run(0.3);
    assert!(
        rig.out.omega_e > 300.0,
        "must restart after the stop, ω {}",
        rig.out.omega_e
    );
    assert!(rig.driver.phase().angle_trustworthy());
}

/// Flying restart on motors whose short-circuit current does NOT settle
/// inside the probe window (τ = L/R of 0.6–0.85 ms against a 0.8 ms
/// settle) — the regime the steady-state estimator could not handle:
/// seeds 28–92% slow, and on a Flipsky-class motor at 2000 el rad/s an
/// overcurrent trip right after the catch.
#[test]
#[cfg(feature = "virtual-motor")]
fn flying_restart_catches_low_tau_motors() {
    const DT_FLY: f32 = 1.0 / 20_000.0;
    let flipsky = MotorParams {
        r: 0.036,
        ld: 22e-6,
        lq: 22e-6,
        lambda: 2.92e-3,
        pole_pairs: 7,
        j: 4e-4,
        friction_b: 2e-5,
        substeps: 8,
        actuation_delay: 0.5,
        ..MotorParams::default()
    };
    let zd2808 = MotorParams {
        r: 0.127,
        ld: 86e-6,
        lq: 129e-6,
        lambda: 1.145e-3,
        pole_pairs: 7,
        j: 5e-5,
        friction_b: 4e-6,
        substeps: 8,
        actuation_delay: 0.5,
        ..MotorParams::default()
    };
    // (name, plant, bus V, max current A, initial el rad/s, seed
    // tolerance). The 5065 runs at a current limit an e-skate would:
    // shorted at 2000 el rad/s it builds ~13 A per PWM period, so no
    // probe fits under a 10 A limit's 13 A trip. The ZD2808 is salient
    // (Lq = 1.5 Ld) and the probe has the observer's single L: at 300
    // el rad/s, where ωL·i is a quarter of the back-EMF, that costs
    // ~5%; the same plant with Ld = Lq seeds well inside 5%.
    let cases = [
        ("flipsky", flipsky, 24.0, 40.0, 300.0, 0.02),
        ("flipsky", flipsky, 24.0, 40.0, 1000.0, 0.02),
        ("flipsky", flipsky, 24.0, 40.0, 2000.0, 0.02),
        ("zd2808", zd2808, 12.0, 10.0, 300.0, 0.08),
        ("zd2808", zd2808, 12.0, 10.0, 1000.0, 0.08),
    ];
    let mut report = String::new();
    let mut ok = true;
    for (name, params, vbus, max_a, w0, tol) in cases {
        let l_avg = (params.ld + params.lq) / 2.0;
        let mut motor = VirtualMotor::new(params);
        motor.set_angle(1.0);
        motor.set_velocity(w0);
        let mut foc =
            FocController::<SvpwmModulator, LibmSinCos>::from_motor_params(params.r, l_avg, vbus);
        foc.set_decoupling(Some(Decoupling {
            ld_h: params.ld,
            lq_h: params.lq,
            flux_linkage_wb: params.lambda,
            resistance_ohm: params.r,
        }));
        let mut mgr = PhaseManager::sensorless();
        mgr.set_observer(Observer::BackEmf(BackEmfObserver::new(
            params.r,
            l_avg,
            params.lambda,
        )));
        mgr.set_source(PhaseSource::Observer).unwrap();
        mgr.set_phase_tracker(100.0, 1.2); // the f405 boot configuration
        let mut driver = FocDriver::new(
            foc,
            MockPwm { duties: [0; 3] },
            MockCurrentSensor {
                currents: (0.0, 0.0, 0.0),
            },
            mgr,
            DT_FLY,
        );
        driver.set_current_limits(CurrentLimits::from_max_current(max_a));
        driver.set_actuation_delay(params.actuation_delay);
        driver.set_mode(ControlMode::CurrentControl {
            iq_target: 3.0,
            id_target: 0.0,
        });
        // Coasting with the bridge off until the throttle.
        let mut out = motor.step_coast(0.0, DT_FLY);
        let mut peak = 0.0f32;
        let mut seed_err = f32::NAN;
        // Worst commutation-frame error over the 2 ms after the catch:
        // the handoff itself. (Later the observer PLL's lag under
        // acceleration takes over — on the light ZD2808 rotor ~0.3 rad
        // by 20 ms; a separate issue.)
        let mut frame_err = 0.0f32;
        let mut caught_at = None;
        let mut result = "ok";
        for step in 1..6_000u64 {
            driver.current_sensor_mut().currents = (out.ia, out.ib, out.ic);
            let was_starting = driver.phase().is_starting();
            let Ok(telem) = driver.step(step * 50) else {
                result = "TRIPPED";
                break;
            };
            if was_starting && !driver.phase().is_starting() && caught_at.is_none() {
                seed_err = driver.phase().get().velocity / out.omega_e - 1.0;
                caught_at = Some(step);
            }
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT_FLY);
            peak = peak.max(out.ia.abs()).max(out.ib.abs()).max(out.ic.abs());
            if caught_at.is_some_and(|c| step - c < 40) {
                // `get()` predicts the next sample instant — where the
                // plant now is.
                let e = angle_difference(driver.phase().get().angle, out.angle_rad);
                frame_err = frame_err.max(e.abs());
            }
        }
        let pass =
            result == "ok" && seed_err.abs() < tol && frame_err < 0.2 && out.omega_e > 0.7 * w0;
        ok &= pass;
        report += &format!(
            "\n  {name} w0 {w0:5.0}: {result}, seed err {:+.1}%, frame err {frame_err:.2} rad, \
             peak {peak:.1} A, final ω {:.0}{}",
            seed_err * 100.0,
            out.omega_e,
            if pass { "" } else { "  <-- FAIL" }
        );
    }
    assert!(ok, "flying restart:{report}");
}

/// Invalid-state glitches at speed (a bouncing hall connector) must not
/// trip the drive. The estimator used to zero its speed on the invalid
/// state — reporting ω = 0 at full speed when the signal came back —
/// and then took the glitch's END as an edge, overestimating the next
/// speed 1.6×: 0.2–1 ms glitches tripped overcurrent in 10 of 16
/// timings. A glitch longer than a sector also hides an edge, whose
/// boundary must be placed at the estimated crossing, not at the
/// glitch's end (−0.9 rad of frame error otherwise).
#[test]
#[cfg(feature = "virtual-motor")]
fn hall_glitches_at_speed_do_not_trip() {
    let mut rig = HallRig::new(0.5, 1.4e-3);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 10.0,
        id_target: 0.0,
    });
    rig.run(0.8);
    let cruise = rig.out.omega_e;
    let mut id_max = 0.0f32;
    for k in 0..16 {
        // 0.2 ms and 1 ms glitches, each landing at a different phase
        // of the sector (50.35 ms apart is no multiple of it).
        rig.glitch(if k % 2 == 0 { 4 } else { 20 });
        for _ in 0..1007 {
            let (sample, _) = rig.step(); // panics on an overcurrent trip
            let (i_alpha, i_beta) = clarke(sample.ia, sample.ib);
            let (s, c) = (libm::sinf(sample.angle_rad), libm::cosf(sample.angle_rad));
            id_max = id_max.max(park(i_alpha, i_beta, s, c).0.abs());
        }
    }
    assert!(id_max < 12.0, "true |id| peaked at {id_max} A");
    assert!(rig.out.omega_e > 0.9 * cruise);
}

/// Full torque into the voltage limit must settle at base speed with
/// the current under control. Uniform scaling of (vd, vq) at the limit
/// let id run away negative (−37 A at a 25 A command) until the
/// overcurrent trip killed the drive at top speed.
#[test]
#[cfg(feature = "virtual-motor")]
fn full_torque_into_the_voltage_limit_keeps_id_regulated() {
    let mut rig = HallRig::new(0.5, 2e-5);
    rig.load = 0.1;
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 25.0,
        id_target: 0.0,
    });
    rig.run(1.0); // base speed is reached at ~0.4 s
    let mut id_max = 0.0f32;
    for _ in 0..4000 {
        let (sample, _) = rig.step(); // panics on an overcurrent trip
        let (i_alpha, i_beta) = clarke(sample.ia, sample.ib);
        let (s, c) = (libm::sinf(sample.angle_rad), libm::cosf(sample.angle_rad));
        id_max = id_max.max(park(i_alpha, i_beta, s, c).0.abs());
    }
    assert!(id_max < 2.0, "true |id| {id_max} A at the voltage limit");
    assert!(rig.out.omega_e > 4000.0, "ω {}", rig.out.omega_e);
}

/// The other half of the staleness contract: hall edges stopping while
/// the back-EMF observer still sees the rotor turning IS a dead sensor —
/// commutation must move to the observer and the drive keep going. With
/// the old 8-sector window the clamped hall angle ran ~1 rad off the
/// rotor within a millisecond and the overcurrent trip fired first
/// (true id +32 A); the 1.5-sector window hands over with a ~10 A blip.
#[test]
#[cfg(feature = "virtual-motor")]
fn hall_dying_at_speed_hands_over_to_the_observer() {
    let mut rig = HallRig::new(0.5, 1.4e-3);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 10.0,
        id_target: 0.0,
    });
    rig.run(1.0);
    let cruise = rig.out.omega_e;
    assert!(cruise > 1000.0, "cruise ω {cruise}");
    rig.hall_alive = false;
    rig.run(0.5);
    assert!(
        rig.out.omega_e > 0.9 * cruise,
        "must keep driving on the observer: ω {} (was {cruise})",
        rig.out.omega_e
    );
    assert!(rig.driver.phase().angle_trustworthy());
}

/// 10 A of torque current against a viscous load settles at ~1500 el
/// rad/s (~2000 rpm) — a steady operating point with no speed loop in
/// the way. Spins up on hall, then switches to `source` and settles.
#[cfg(feature = "virtual-motor")]
fn measure_frame_timing(source: PhaseSource, delay: f32) -> FrameTiming {
    // 1.5·pp·λ·10 A = 0.31 N·m balances B·ωm at ωm ≈ 215 rad/s.
    let mut rig = HallRig::new(delay, 1.4e-3);
    rig.driver.set_mode(ControlMode::CurrentControl {
        iq_target: 10.0,
        id_target: 0.0,
    });
    let (mut err, mut w, mut tid, mut tiq) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut n = 0u32;
    for step in 1..=24_000u64 {
        if step == 12_000 {
            rig.driver.phase_mut().set_source(source).unwrap();
        }
        let (sample, telem) = rig.step();
        if step > 20_000 {
            err += angle_difference(telem.angle_rad, sample.angle_rad);
            w += sample.omega_e;
            let (i_alpha, i_beta) = clarke(sample.ia, sample.ib);
            let (s, c) = (libm::sinf(sample.angle_rad), libm::cosf(sample.angle_rad));
            let (id, iq) = park(i_alpha, i_beta, s, c);
            tid += id;
            tiq += iq;
            n += 1;
        }
    }
    let n = n as f32;
    FrameTiming {
        frame_err_periods: (err / n) / (w / n * HallRig::DT),
        true_id: tid / n,
        true_iq: tiq / n,
    }
}

/// The commutation frame must sit on the rotor at the current sample
/// instant, for every angle source and actuation delay: a frame one
/// PWM period off regulates the current vector `ωe·T` off the q axis
/// (true id = −iq·sin δ) while the controller's own frame reports
/// id = 0. Before the timing contract was made explicit, hall
/// commutated one period LATE (the angle was evaluated at the previous
/// cycle's ticks), the observer chain ~1–2 periods EARLY (the phase
/// tracker predicted a second step on top of the observer PLL's own,
/// and the observer was fed the command one period newer than the
/// voltage that actually produced the sampled currents).
#[test]
#[cfg(feature = "virtual-motor")]
fn commutation_frame_tracks_rotor_at_sample_instant() {
    let cases = [
        ("hall", PhaseSource::Hall, 0.5),
        ("observer, delay 0", PhaseSource::Observer, 0.0),
        ("observer, delay 0.5", PhaseSource::Observer, 0.5),
        ("observer, delay 1", PhaseSource::Observer, 1.0),
    ];
    let mut report = String::new();
    let mut ok = true;
    for (name, source, delay) in cases {
        let t = measure_frame_timing(source, delay);
        let pass = t.true_iq > 5.0
            && t.frame_err_periods.abs() < 0.15
            && t.true_id.abs() < 0.03 * t.true_iq;
        ok &= pass;
        report += &format!(
            "\n  {name:22} frame {:+.2} periods, true id {:+.3} A at iq {:.2} A{}",
            t.frame_err_periods,
            t.true_id,
            t.true_iq,
            if pass { "" } else { "  <-- FAIL" }
        );
    }
    assert!(ok, "commutation frame off the rotor:{report}");
}

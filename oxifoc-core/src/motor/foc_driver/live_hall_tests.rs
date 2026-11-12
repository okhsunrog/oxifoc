use super::*;
use crate::foc::hall_sensor::HallSensor;
use crate::foc::sensors::AngleSensor;
use crate::state::model::{CachedConfigAck, ConfigActionCache, ModelApplyGuard};
use crate::state::{DriverCommand, MotorControlState, process_commands};
use crate::storage::{HallCalibrationConfig, RuntimeConfig};
use crate::types::{ConfigGroupId, ConfigResponse, ReqId};
use core::cell::RefCell;
use core::task::{Context, Poll, Waker};
use critical_section::Mutex;

struct PendingTimer;
impl crate::timer::Timer for PendingTimer {
    async fn after_millis(_: u64) {
        core::future::pending::<()>().await;
    }
    async fn after_micros(_: u64) {
        core::future::pending::<()>().await;
    }
}

#[derive(Clone, Copy, PartialEq)]
struct TestFault;
impl crate::foc::fault::PlatformFault for TestFault {
    fn category(&self) -> crate::foc::fault::FaultCategory {
        crate::foc::fault::FaultCategory::OverCurrent
    }
    fn details(&self) -> heapless::String<128> {
        heapless::String::new()
    }
}

struct BootFlag(bool);
impl BootFlag {
    fn clear() -> Self {
        Self(
            crate::state::BOOT_CURRENT_OFFSET_PENDING
                .swap(false, core::sync::atomic::Ordering::AcqRel),
        )
    }
}
impl Drop for BootFlag {
    fn drop(&mut self) {
        crate::state::BOOT_CURRENT_OFFSET_PENDING
            .store(self.0, core::sync::atomic::Ordering::Release);
    }
}

#[test]
fn live_hall_transaction_commits_only_after_supported_stopped_apply() {
    let _serial = cmd_channel_lock();
    let _boot = BootFlag::clear();
    let mut initial = model_config();
    initial.hall_calibration_invalid = true;
    let config = Box::leak(Box::new(Mutex::new(RefCell::new(initial))));
    let revisions = Box::leak(Box::new(Mutex::new(RefCell::new(
        [0; ConfigGroupId::COUNT],
    ))));
    let cache = Box::leak(Box::new(Mutex::new(RefCell::new(ConfigActionCache::new()))));
    let state = Mutex::new(RefCell::new(MotorControlState::new()));
    critical_section::with(|cs| {
        let mut state = state.borrow(cs).borrow_mut();
        state.set_link_active();
        state.load_hall_calibration(&config.borrow(cs).borrow());
        assert!(!state.hall_calibration_valid);
    });
    let registry = crate::foc::fault::FaultRegistry::<TestFault>::new();
    let mut hall = HallSensor::new(1_000_000);
    hall.update(1, 0);
    let mut phase = PhaseManager::with_hall(hall);
    phase.configure_observers_from_config(&model_config(), 24.0);
    let observer_model = (phase.observer().resistance(), phase.observer().lambda());
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::from_runtime_config(&model_config(), 24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        phase,
        1.0 / 20_000.0,
    );
    let gains = driver.controller().id_pi.gains();
    let model_coefficients = |driver: &FocController<SvpwmModulator, LibmSinCos>| {
        driver.decoupling().map(|model| {
            (
                model.resistance_ohm,
                model.ld_h,
                model.lq_h,
                model.flux_linkage_wb,
            )
        })
    };
    let decoupling = model_coefficients(driver.controller());
    let limit = driver.current_limits.max_current_a;
    let mut calibration = HallCalibrationConfig::default();
    for angle in &mut calibration.angles[1..=6] {
        *angle += 0.4;
    }
    calibration.geometry = Some(crate::storage::HallTransitionGeometryConfig {
        entry_angles: calibration
            .angles
            .map(|angle| angle - core::f32::consts::TAU / 12.0),
        next_states: [0, 3, 6, 2, 5, 1, 4, 0],
    });
    let prepare = |id, expected| {
        ModelApplyGuard::prepare_hall(calibration, config, revisions, cache, ReqId(id), expected)
            .unwrap()
    };
    let mut cx = Context::from_waker(Waker::noop());

    let mut apply = Box::pin(prepare(101, 0).apply::<PendingTimer>());
    assert!(apply.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    drop(apply); // Live installation durably commits even if its waiter leaves.
    let sample = driver.phase().hall().sample(50).unwrap();
    assert_eq!(sample.angle.to_bits(), calibration.angles[1].to_bits());
    assert_eq!(
        driver.phase().get().angle.to_bits(),
        calibration.angles[1].to_bits(),
        "cached commutation frame is installed before acknowledgement"
    );
    assert_eq!(
        (
            driver.phase().observer().resistance(),
            driver.phase().observer().lambda()
        ),
        observer_model
    );
    assert_eq!(driver.controller().id_pi.gains(), gains);
    assert_eq!(model_coefficients(driver.controller()), decoupling);
    assert_eq!(driver.current_limits.max_current_a, limit);
    assert_eq!(driver.phase().source(), PhaseSource::Hall);
    critical_section::with(|cs| {
        assert_eq!(
            config.borrow(cs).borrow().hall_calibration.as_ref(),
            Some(&calibration)
        );
        assert!(!config.borrow(cs).borrow().hall_calibration_invalid);
        assert!(state.borrow(cs).borrow().hall_calibration_valid);
        assert_eq!(
            revisions.borrow(cs).borrow()[ConfigGroupId::HallCalibration.index()],
            1
        );
        assert_eq!(
            revisions.borrow(cs).borrow()[ConfigGroupId::MotorParams.index()],
            0
        );
        assert!(matches!(
            cache.borrow(cs).borrow().get(ReqId(101)),
            Some(CachedConfigAck::Applied(1))
        ));
    });

    let mut unsupported = grant_driver(CurrentLimits::default(), DeratingScales::IDENTITY);
    let mut apply = Box::pin(prepare(102, 1).apply::<PendingTimer>());
    assert!(apply.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut unsupported, &registry);
    assert_eq!(
        apply.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Unsupported)
    );
    drop(apply);

    driver.set_mode(ControlMode::CurrentControl {
        iq_target: 0.1,
        id_target: 0.0,
    });
    let mut apply = Box::pin(prepare(103, 1).apply::<PendingTimer>());
    assert!(apply.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        apply.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Busy)
    );
    drop(apply);
    driver.set_mode(ControlMode::Stopped);

    let mut apply = Box::pin(prepare(104, 0).apply::<PendingTimer>());
    assert!(apply.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        apply.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Conflict {
            current_revision: 1
        })
    );
    drop(apply);

    let mut cancelled = Box::pin(prepare(105, 1).apply::<PendingTimer>());
    assert!(cancelled.as_mut().poll(&mut cx).is_pending());
    drop(cancelled);
    process_commands(&state, &mut driver, &registry);
    critical_section::with(|cs| {
        assert_eq!(
            revisions.borrow(cs).borrow()[ConfigGroupId::HallCalibration.index()],
            1
        );
        assert_eq!(
            config.borrow(cs).borrow().hall_calibration.as_ref(),
            Some(&calibration)
        );
        for id in [102, 103, 104, 105] {
            assert!(cache.borrow(cs).borrow().get(ReqId(id)).is_none());
        }
    });
    assert!(
        NoSensor
            .apply_prepared_hall_calibration(
                &crate::foc::hall_sensor::PreparedHallCalibration::from_config(&calibration)
                    .unwrap()
            )
            .is_none()
    );
    let _ = CMD_CHANNEL.try_send(DriverCommand::SetMode(ControlMode::Stopped));
    process_commands(&state, &mut driver, &registry);

    let persisted = Box::leak(Box::new(Mutex::new(RefCell::new(
        [Some(0); ConfigGroupId::COUNT],
    ))));
    let reset = ModelApplyGuard::prepare_reset(
        PreparedMotorModel::from_config(&RuntimeConfig::default(), 24.0, 3.0),
        config,
        revisions,
        persisted,
        cache,
        0,
    )
    .unwrap();
    let mut reset = Box::pin(reset.apply::<PendingTimer>());
    assert!(reset.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        reset.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Reset)
    );
    let mut nominal = HallSensor::new(1_000_000);
    nominal.update(1, 0);
    assert_eq!(
        driver.phase().hall().sample(50).unwrap().angle.to_bits(),
        nominal.sample(50).unwrap().angle.to_bits()
    );
    assert_eq!(
        driver.phase().get().angle.to_bits(),
        nominal.sample(50).unwrap().angle.to_bits()
    );
    critical_section::with(|cs| {
        assert!(config.borrow(cs).borrow().hall_calibration.is_none());
        assert!(state.borrow(cs).borrow().hall_calibration_valid);
    });
}

#[test]
fn invalid_hall_reference_blocks_only_dependent_closed_loop_and_source_requests() {
    let _serial = cmd_channel_lock();
    let _boot = BootFlag::clear();
    let state = Mutex::new(RefCell::new(MotorControlState::new()));
    critical_section::with(|cs| {
        let mut state = state.borrow(cs).borrow_mut();
        state.set_link_active();
        state.load_hall_calibration(&RuntimeConfig::default());
        assert!(
            state.hall_calibration_valid,
            "missing legacy config permits nominal geometry"
        );
        state.load_hall_calibration(&RuntimeConfig {
            hall_calibration_invalid: true,
            ..Default::default()
        });
        assert!(!state.hall_calibration_valid);
    });
    let registry = crate::foc::fault::FaultRegistry::<TestFault>::new();
    let mut hall = HallSensor::new(1_000_000);
    hall.update(1, 0);
    let mut phase = PhaseManager::with_hall(hall);
    phase.configure_observers_from_config(&model_config(), 24.0);
    // Seed a high-speed cached frame to exercise Brake's regulated ramp gate.
    phase.set_source(PhaseSource::OpenLoop).unwrap();
    phase.set_open_loop_velocity(100.0);
    phase.update(
        &crate::foc::phase::PhaseInput {
            dt: 0.00005,
            ..Default::default()
        },
        50,
    );
    phase.set_source(PhaseSource::Hall).unwrap();
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        phase,
        1.0 / 20_000.0,
    );
    for mode in [
        ControlMode::CurrentControl {
            iq_target: 0.1,
            id_target: 0.0,
        },
        ControlMode::VelocityControl { target_vel: 10.0 },
        ControlMode::PositionControl { target_pos: 1.0 },
        ControlMode::Brake,
    ] {
        CMD_CHANNEL.try_send(DriverCommand::SetMode(mode)).unwrap();
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), ControlMode::Stopped);
        assert!(!driver.failsafe_active());
    }
    for mode in [
        ControlMode::OpenLoop {
            angle_rad: 0.0,
            current: 0.1,
            velocity_rad_s: 0.0,
            pi_gains: None,
        },
        ControlMode::DirectVoltage {
            vd: 0.01,
            vq: 0.0,
            angle_rad: 0.0,
        },
        ControlMode::Coast,
        ControlMode::Stopped,
    ] {
        CMD_CHANNEL.try_send(DriverCommand::SetMode(mode)).unwrap();
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.mode(), mode, "repair/neutral modes stay available");
    }
    for source in [PhaseSource::Manual, PhaseSource::Observer] {
        CMD_CHANNEL
            .try_send(DriverCommand::SetPhaseSource(source))
            .unwrap();
        process_commands(&state, &mut driver, &registry);
        assert_eq!(driver.phase().source(), source);
        CMD_CHANNEL
            .try_send(DriverCommand::SetPhaseSource(PhaseSource::Hall))
            .unwrap();
        process_commands(&state, &mut driver, &registry);
        assert_eq!(
            driver.phase().source(),
            source,
            "invalid stored Hall cannot be selected"
        );
        CMD_CHANNEL
            .try_send(DriverCommand::SetMode(ControlMode::CurrentControl {
                iq_target: 0.1,
                id_target: 0.0,
            }))
            .unwrap();
        process_commands(&state, &mut driver, &registry);
        assert!(matches!(driver.mode(), ControlMode::CurrentControl { .. }));
        CMD_CHANNEL
            .try_send(DriverCommand::SetMode(ControlMode::Stopped))
            .unwrap();
        process_commands(&state, &mut driver, &registry);
    }
    driver
        .phase_mut()
        .set_source(PhaseSource::OpenLoop)
        .unwrap();
    driver.phase_mut().set_open_loop_velocity(0.0);
    driver.phase_mut().update(
        &crate::foc::phase::PhaseInput {
            dt: 0.00005,
            ..Default::default()
        },
        100,
    );
    driver.phase_mut().set_source(PhaseSource::Hall).unwrap();
    CMD_CHANNEL
        .try_send(DriverCommand::SetMode(ControlMode::Brake))
        .unwrap();
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        driver.mode(),
        ControlMode::Brake,
        "standing short remains a safe operation"
    );
}

#[cfg(all(feature = "detection", feature = "delivery"))]
#[test]
fn detector_cannot_install_hall_before_explicit_apply() {
    use crate::state::DetectionActiveGuard;
    use crate::state::model::ConfigCoordinator;

    let _serial = cmd_channel_lock();
    let _boot = BootFlag::clear();
    let coordinator = Box::leak(Box::new(ConfigCoordinator::new()));
    let config = Box::leak(Box::new(Mutex::new(RefCell::new(RuntimeConfig {
        hall_calibration: Some(HallCalibrationConfig::default()),
        ..Default::default()
    }))));
    let state = Mutex::new(RefCell::new(MotorControlState::new()));
    critical_section::with(|cs| {
        state.borrow(cs).borrow_mut().set_link_active();
        coordinator.persisted.borrow(cs).borrow_mut()[ConfigGroupId::HallCalibration.index()] =
            Some(0);
    });
    let registry = crate::foc::fault::FaultRegistry::<TestFault>::new();
    let mut hall = HallSensor::new(1_000_000);
    hall.update(1, 0);
    let mut driver = FocDriver::new(
        FocController::<SvpwmModulator, LibmSinCos>::new(24.0),
        MockPwm { duties: [0; 3] },
        MockCurrentSensor {
            currents: (0.0, 0.0, 0.0),
        },
        PhaseManager::with_hall(hall),
        1.0 / 20_000.0,
    );

    let id = ReqId(201);
    let mut calibration = HallCalibrationConfig::default();
    for angle in &mut calibration.angles[1..=6] {
        *angle += 0.7;
    }
    let mut cx = Context::from_waker(Waker::noop());
    let owner = DetectionActiveGuard::try_arm().unwrap();
    assert!(DetectionActiveGuard::try_arm().is_none());

    // A regular config transaction cannot borrow the detector's maintenance
    // ownership, even though it uses the same stopped installation machinery.
    let ordinary = ModelApplyGuard::prepare_hall(
        calibration,
        config,
        &coordinator.revisions,
        &coordinator.actions,
        id,
        0,
    )
    .unwrap();
    let mut ordinary = Box::pin(ordinary.apply::<PendingTimer>());
    assert!(ordinary.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        ordinary.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Busy)
    );
    drop(ordinary);

    // Measuring Hall owns excitation, never configuration installation.
    critical_section::with(|cs| {
        assert_eq!(
            coordinator.revisions.borrow(cs).borrow()[ConfigGroupId::HallCalibration.index()],
            0
        );
        assert_eq!(
            config.borrow(cs).borrow().hall_calibration,
            Some(HallCalibrationConfig::default())
        );
    });
    drop(owner);
    let pending = ModelApplyGuard::prepare_hall(
        calibration,
        config,
        &coordinator.revisions,
        &coordinator.actions,
        id,
        0,
    )
    .unwrap();
    let mut applied = Box::pin(pending.apply::<PendingTimer>());
    assert!(applied.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert!(matches!(
        applied.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Applied { revision: 1, .. })
    ));
    assert_eq!(
        driver.phase().hall().sample(50).unwrap().angle.to_bits(),
        calibration.angles[1].to_bits()
    );
}

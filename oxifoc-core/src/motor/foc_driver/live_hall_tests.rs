use super::*;
use crate::foc::hall_sensor::HallSensor;
use crate::foc::sensors::AngleSensor;
use crate::state::model::{CachedConfigAck, ConfigActionCache, ModelApplyGuard};
use crate::state::{DriverCommand, MotorControlState, process_commands};
use crate::storage::HallCalibrationConfig;
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

#[test]
fn live_hall_transaction_commits_only_after_supported_stopped_apply() {
    let _serial = cmd_channel_lock();
    struct BootFlag(bool);
    impl Drop for BootFlag {
        fn drop(&mut self) {
            crate::state::BOOT_CURRENT_OFFSET_PENDING
                .store(self.0, core::sync::atomic::Ordering::Release);
        }
    }
    let _boot = BootFlag(
        crate::state::BOOT_CURRENT_OFFSET_PENDING.swap(false, core::sync::atomic::Ordering::AcqRel),
    );
    let config = Box::leak(Box::new(Mutex::new(RefCell::new(model_config()))));
    let revisions = Box::leak(Box::new(Mutex::new(RefCell::new(
        [0; ConfigGroupId::COUNT],
    ))));
    let cache = Box::leak(Box::new(Mutex::new(RefCell::new(ConfigActionCache::new()))));
    let state = Mutex::new(RefCell::new(MotorControlState::new()));
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
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
    let prepare = |id, expected| {
        ModelApplyGuard::prepare_hall(
            calibration.clone(),
            config,
            revisions,
            cache,
            ReqId(id),
            expected,
        )
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
}

use super::*;
use crate::foc::fault::{FaultCategory, FaultRegistry, StandardFault};
use crate::storage::VoltageLimitsConfig;

fn board() -> BoardConfig {
    BoardConfig {
        calib: Default::default(),
        current_sense_topology: crate::types::CurrentSenseTopology::ThreeShunt,
        max_iq_target_a: 10.0,
        max_phase_current_a: 70.0,
        min_vbus_mv: 6_000,
        max_vbus_mv: 57_000,
        max_fet_temp_c: 100.0,
        max_motor_temp_c: 120.0,
        phase_sense: None,
    }
}

fn limits(min: u32, max: u32) -> VoltageLimitsConfig {
    VoltageLimitsConfig {
        min_vbus_mv: min,
        max_vbus_mv: max,
    }
}

#[test]
fn boot_and_live_voltage_limits_share_fault_thresholds_and_failsafe() {
    for boot in [false, true] {
        let mut driver = grant_driver(CurrentLimits::default(), DeratingScales::IDENTITY);
        let config = limits(20_000, 26_000);
        driver
            .configure_voltage_protection(&board(), boot.then_some(&config))
            .unwrap();
        if !boot {
            driver
                .apply_voltage_limits(PreparedVoltageLimits::from_config(&config).unwrap())
                .unwrap();
        }
        assert_eq!(driver.protection.overvoltage_threshold(), 26.0);
        let registry = FaultRegistry::<StandardFault>::new();
        let prepared = PreparedBoardConfig::new(board());
        for _ in 0..100 {
            driver.run_protection(&registry, &prepared, 19.0, None, None);
        }
        assert!(registry.has_category(FaultCategory::UnderVoltage));
        driver.run_protection(&registry, &prepared, 20.1, None, None);
        assert!(
            registry.has_category(FaultCategory::UnderVoltage),
            "clear hysteresis uses configured UV"
        );
        driver.run_protection(&registry, &prepared, 21.0, None, None);
        assert!(!registry.has_category(FaultCategory::UnderVoltage));
        for _ in 0..100 {
            driver.run_protection(&registry, &prepared, 27.0, None, None);
        }
        assert!(registry.has_category(FaultCategory::OverVoltage));
        driver.reset_voltage_limits();
        assert_eq!(driver.protection.overvoltage_threshold(), 57.0);
        driver.run_protection(&registry, &prepared, 24.0, None, None);
        assert!(
            registry.has_category(FaultCategory::OverVoltage),
            "changing limits must not acknowledge a latched fault"
        );
    }
}

#[test]
fn invalid_or_running_voltage_updates_leave_previous_protection_intact() {
    let mut driver = grant_driver(CurrentLimits::default(), DeratingScales::IDENTITY);
    let valid = limits(8_000, 26_000);
    driver
        .configure_voltage_protection(&board(), Some(&valid))
        .unwrap();
    for invalid in [
        limits(0, 26_000),
        limits(26_000, 26_000),
        limits(27_000, 26_000),
        limits(5_000, 26_000),
        limits(8_000, 58_000),
    ] {
        assert_eq!(
            driver.configure_voltage_protection(&board(), Some(&invalid)),
            Err(MotorModelError::Invalid)
        );
        if let Some(prepared) = PreparedVoltageLimits::from_config(&invalid) {
            assert_eq!(
                driver.apply_voltage_limits(prepared),
                Err(MotorModelError::Invalid)
            );
        }
        assert_eq!(driver.protection.overvoltage_threshold(), 26.0);
    }
    driver.set_mode(ControlMode::CurrentControl {
        id_target: 0.0,
        iq_target: 0.2,
    });
    assert_eq!(
        driver.apply_voltage_limits(
            PreparedVoltageLimits::from_config(&limits(8_000, 30_000)).unwrap()
        ),
        Err(MotorModelError::Busy)
    );
    assert_eq!(driver.protection.overvoltage_threshold(), 26.0);
}

#[test]
fn voltage_apply_acknowledges_only_an_installed_and_validated_revision() {
    use crate::state::model::{ConfigCoordinator, ModelApplyGuard};
    use crate::state::{self, MotorControlState, process_commands};
    use crate::storage::RuntimeConfig;
    use crate::types::{ConfigGroupId, ConfigResponse, ReqId};
    use core::{
        cell::RefCell,
        future::Future,
        task::{Context, Poll, Waker},
    };
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
    let _serial = cmd_channel_lock();
    struct BootFlag(bool);
    impl Drop for BootFlag {
        fn drop(&mut self) {
            state::BOOT_CURRENT_OFFSET_PENDING.store(self.0, core::sync::atomic::Ordering::Release);
        }
    }
    let _boot = BootFlag(
        state::BOOT_CURRENT_OFFSET_PENDING.swap(false, core::sync::atomic::Ordering::AcqRel),
    );
    let coordinator = Box::leak(Box::new(ConfigCoordinator::new()));
    let config = Box::leak(Box::new(Mutex::new(RefCell::new(RuntimeConfig::default()))));
    let state = Mutex::new(RefCell::new(MotorControlState::new()));
    critical_section::with(|cs| state.borrow(cs).borrow_mut().set_link_active());
    let registry = FaultRegistry::<StandardFault>::new();
    let mut driver = grant_driver(CurrentLimits::default(), DeratingScales::IDENTITY);
    driver.configure_voltage_protection(&board(), None).unwrap();
    let mut cx = Context::from_waker(Waker::noop());

    let guard = ModelApplyGuard::prepare_voltage_limits(
        limits(8_000, 26_000),
        config,
        coordinator,
        ReqId(10),
        0,
    )
    .unwrap();
    let mut future = Box::pin(guard.apply::<PendingTimer>());
    assert!(future.as_mut().poll(&mut cx).is_pending());
    assert_eq!(driver.protection.overvoltage_threshold(), 57.0);
    assert!(critical_section::with(|cs| config
        .borrow(cs)
        .borrow()
        .voltage_limits
        .is_none()));
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        future.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Applied {
            req_id: ReqId(10),
            revision: 1
        })
    );
    drop(future);
    assert_eq!(driver.protection.overvoltage_threshold(), 26.0);
    let guard = ModelApplyGuard::prepare_voltage_limits(
        limits(8_000, 58_000),
        config,
        coordinator,
        ReqId(11),
        1,
    )
    .unwrap();
    let mut future = Box::pin(guard.apply::<PendingTimer>());
    assert!(future.as_mut().poll(&mut cx).is_pending());
    process_commands(&state, &mut driver, &registry);
    assert_eq!(
        future.as_mut().poll(&mut cx),
        Poll::Ready(ConfigResponse::Invalid)
    );
    drop(future);
    assert_eq!(driver.protection.overvoltage_threshold(), 26.0);
    critical_section::with(|cs| {
        assert_eq!(
            config.borrow(cs).borrow().voltage_limits,
            Some(limits(8_000, 26_000))
        );
        assert_eq!(
            coordinator.revisions.borrow(cs).borrow()[ConfigGroupId::VoltageLimits.index()],
            1
        );
    });
}

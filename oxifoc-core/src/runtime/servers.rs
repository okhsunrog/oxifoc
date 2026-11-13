//! Protocol servers that access state directly
//!
//! These servers handle ergot protocol requests by accessing the global
//! state module. Platforms define their own state globals using
//! `define_platform_state!` macro.
//!
//! # Usage
//!
//! ```ignore
//! // In platform - define state globals
//! oxifoc_core::define_platform_state!(MyFault);
//!
//! // In platform - single task wrapper
//! #[embassy_executor::task]
//! pub async fn protocol_servers() {
//!     let device_info = HardwareInfo { hw, sw, ..Default::default() };
//!     oxifoc_core::runtime::run_all_servers(
//!         STACK.endpoints(),
//!         device_info,
//!         &STATE,
//!         &FAULT_REGISTRY,
//!     ).await
//! }
//! ```

use core::cell::RefCell;
use core::pin::pin;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_futures::join::{join, join3};
use embassy_futures::select::{Either, select};

use crate::state::DriverCommand;
#[cfg(feature = "storage")]
use crate::state::DriverOperation;
#[cfg(feature = "storage")]
use crate::state::FlashPendingGuard;
#[cfg(feature = "storage")]
use crate::state::model::{CONFIG_COORDINATOR, CachedConfigAck, ModelApplyGuard};
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::icd::PhaseSourceEndpoint;
#[cfg(feature = "storage")]
use crate::icd::{ConfigEndpoint, ConfigRequest, ConfigResponse};
use crate::icd::{
    FaultEndpoint, FaultRequest, FaultResponse, HardwareInfo, HardwareInfoEndpoint, MotorEndpoint,
    MotorRequest, MotorStatus, SlowTelemetry, SlowTelemetryEndpoint, TelemetryConfig,
    TelemetryConfigAck, TelemetryConfigEndpoint,
};
use crate::state::{CMD_CHANNEL, MOTOR_COMMAND_DONE, MotorControlState};
use crate::types::MotorCommandOutcome;

/// Bound on the ISR-confirmation wait in [`motor_command_server`]. Normal
/// confirmation is sub-millisecond (the 20 kHz ISR drains the channel every
/// cycle); the bound exists for the pathological cases — control loop not
/// started yet, or the chip stalled by an internal-flash erase (up to ~2 s on
/// an F4 128 KB sector, but Persist guarantees the motor is stopped and no
/// motor traffic is in flight then). Must stay below the host's 2 s response
/// timeout so the host receives a typed [`MotorCommandOutcome::TimedOut`]
/// instead of an unattributable transport timeout.
pub const MOTOR_ACK_TIMEOUT_MS: u64 = 1_000;
#[cfg(feature = "storage")]
use crate::storage::{FLASH_CHANNEL, FLASH_DONE, FlashOperation, RuntimeConfig};

/// Device info server - responds to info requests from host
///
/// Returns hardware and software version information.
/// Also marks the communication link as active on first request.
pub async fn info_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    device_info: HardwareInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
) where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<HardwareInfoEndpoint, N>(Some("hardware_info"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let info = device_info.clone();
        let _result = h
            .serve(|_req: &()| {
                #[cfg(feature = "defmt")]
                defmt::info!("HardwareInfo request received");
                // Mark link as active on first request
                critical_section::with(|cs| {
                    state_mutex.borrow(cs).borrow_mut().set_link_active();
                });
                async move { info }
            })
            .await;
        #[cfg(feature = "defmt")]
        defmt::info!(
            "HardwareInfo response send result: {:?}",
            defmt::Debug2Format(&_result)
        );
    }
}

/// Hall sensor server - responds to Hall sensor data requests
///
/// Motor control server - handles motor control mode changes
///
/// Sends the sequenced request to CMD_CHANNEL and responds only after the ISR
/// has applied or rejected it — or after [`MOTOR_ACK_TIMEOUT_MS`] with
/// [`MotorCommandOutcome::TimedOut`] if the ISR never confirms (control loop
/// not running yet, or stalled by a flash erase).
pub async fn motor_command_server<NS, F, T, const N: usize>(
    endpoints: Endpoints<NS>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
) where
    NS: NetStackHandle,
    F: PlatformFault,
    T: crate::timer::Timer,
{
    let server = endpoints.bounded_server::<MotorEndpoint, N>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let result = h
            .serve(|request: &MotorRequest| {
                let request = *request;
                crate::runtime::streaming::cmd_stats::MOTOR_REQS
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                async move {
                    let confirm = async {
                        MOTOR_COMMAND_DONE.reset();
                        CMD_CHANNEL.send(DriverCommand::Motor(request)).await;
                        loop {
                            let (session, seq, outcome) = MOTOR_COMMAND_DONE.wait().await;
                            if session == request.source_session && seq == request.seq {
                                break outcome;
                            }
                            // Completion of an earlier request that timed out
                            // server-side and was drained late (e.g. the ISR
                            // resumed after a flash-erase stall). Not ours —
                            // keep waiting for this request's confirmation
                            // instead of reporting a foreign outcome.
                        }
                    };
                    let outcome = match select(confirm, T::after_millis(MOTOR_ACK_TIMEOUT_MS)).await
                    {
                        Either::First(outcome) => outcome,
                        // Timeout also covers CMD_CHANNEL.send never
                        // completing (channel full because nothing drains
                        // it): dropping the send future un-queues us and the
                        // host gets a typed outcome instead of a transport
                        // timeout with no diagnosis.
                        Either::Second(()) => MotorCommandOutcome::TimedOut,
                    };
                    critical_section::with(|cs| {
                        let state = state_mutex.borrow(cs).borrow();
                        MotorStatus {
                            outcome,
                            state: state.motor_state,
                            mode: state.control_mode,
                            fault_count: fault_registry.count() as u8,
                        }
                    })
                }
            })
            .await;
        if result.is_err() {
            crate::runtime::streaming::cmd_stats::MOTOR_ACK_SEND_ERRORS
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
    }
}

/// Fault management server - handles fault queries and clear requests
///
/// Allows host to read current fault state and clear faults.
pub async fn fault_server<NS, F, const N: usize>(
    endpoints: Endpoints<NS>,
    fault_registry: &'static FaultRegistry<F>,
) where
    NS: NetStackHandle,
    F: PlatformFault,
{
    use crate::types::FaultClearTarget;

    let server = endpoints.bounded_server::<FaultEndpoint, N>(Some("fault"));
    let server = pin!(server);
    let mut h = server.attach();
    // Compact action cache: retaining only IDs is sufficient because a retry
    // returns the *current* snapshot without executing Clear again. That is
    // stronger than caching the old snapshot: a re-asserted fault is visible
    // in the retry response but cannot be accidentally cleared.
    let clear_cache_storage = CriticalSectionMutex::new(RefCell::new([None; 4]));
    let clear_cache = &clear_cache_storage;
    let clear_cache_next_storage = CriticalSectionMutex::new(RefCell::new(0usize));
    let clear_cache_next = &clear_cache_next_storage;

    loop {
        let _ = h
            .serve(|req: &FaultRequest| {
                let response = match req {
                    FaultRequest::Query => {
                        FaultResponse::Snapshot(fault_registry.snapshot_response())
                    }
                    FaultRequest::Clear(keyed) => {
                        let duplicate = critical_section::with(|cs| {
                            clear_cache
                                .borrow(cs)
                                .borrow()
                                .iter()
                                .flatten()
                                .any(|id| *id == keyed.id)
                        });
                        if duplicate {
                            let snapshot = fault_registry.snapshot_response();
                            FaultResponse::Cleared {
                                req_id: keyed.id,
                                snapshot,
                            }
                        } else {
                            let category = match keyed.inner.target {
                                FaultClearTarget::Category(category) => Some(category),
                                FaultClearTarget::All => None,
                            };
                            if !fault_registry
                                .clear_if_generation(keyed.inner.expected_generation, category)
                            {
                                FaultResponse::Conflict(fault_registry.snapshot_response())
                            } else {
                                critical_section::with(|cs| {
                                    let mut entries = clear_cache.borrow(cs).borrow_mut();
                                    let mut next = clear_cache_next.borrow(cs).borrow_mut();
                                    entries[*next] = Some(keyed.id);
                                    *next = (*next + 1) % entries.len();
                                });
                                FaultResponse::Cleared {
                                    req_id: keyed.id,
                                    snapshot: fault_registry.snapshot_response(),
                                }
                            }
                        }
                    }
                };
                async move { response }
            })
            .await;
    }
}

/// Per-group boundary validation for config writes: every malformed write
/// fails loudly with `Invalid` instead of persisting garbage or a silent
/// no-op. (A sync helper, not match guards in the server: the float checks
/// stay out of the task's future, which flash-tight boards' flash and RAM
/// budgets ride.)
///
/// - CurrentLimits: an incoherent limits pair must fail loudly — the
///   builder would clamp it silently, and the user has to learn the
///   headroom rule, not wonder why full throttle is weak
///   (notes/fault-overhaul.md §4).
/// - Derating: a malformed ramp would silently fall back to the default
///   config in the runtime decoder.
/// - DcOffsets: raw ADC counts — finite and within the converter range.
/// - PiGains: NaN/zero/negative gains used to persist with `Ok`, get
///   silently dropped by the live-apply sanity gate (masking the bad write
///   for the session), then apply VERBATIM at the next boot — NaN vd/vq
///   with the dq overcurrent check comparing false.
/// - MotorParams: an invalid write used to return `Ok` while both
///   live-apply and boot ignored it.
/// - HallCalibration: only physical states 1-6 may be marked valid and their
///   angles must be finite; otherwise Hall commutation can ingest NaN or boot
///   with an incomplete table.
/// - VoltageLimits: ordered, positive; the ISR also verifies board bounds.
/// - PwmConfig: boards that consume this group at boot supply their limits.
#[cfg(feature = "storage")]
fn write_is_acceptable(
    w: &crate::types::ConfigValue,
    pwm_limits: Option<crate::storage::PwmConfigLimits>,
) -> bool {
    use crate::types::ConfigValue;
    let counts_ok = |v: f32| (0.0..=f32::from(u16::MAX)).contains(&v);
    match w {
        ConfigValue::BoardCalibration(v) => v.is_sane(),
        ConfigValue::CurrentLimits(v) => v.is_coherent(),
        ConfigValue::Derating(v) => v.is_sane(),
        ConfigValue::Drive(_) => true,
        ConfigValue::DcOffsets(v) => {
            counts_ok(v.phase_a) && counts_ok(v.phase_b) && counts_ok(v.phase_c)
        }
        ConfigValue::PiGains(v) => v.is_sane(),
        ConfigValue::VoltageFeedforward(_) | ConfigValue::MotorTemperature(_) => true,
        ConfigValue::MotorParams(v) => v.is_valid(),
        ConfigValue::HallCalibration(v) => v.is_calibrated(),
        ConfigValue::HallTuning(v) => v.is_sane(),
        ConfigValue::VoltageLimits(value) => value.is_sane(),
        ConfigValue::PwmConfig(config) => pwm_limits.is_some_and(|limits| limits.accepts(config)),
        ConfigValue::Failsafe(v) => v.is_sane(),
        ConfigValue::Velocity(v) => v.is_sane(),
    }
}

#[cfg(feature = "storage")]
fn write_is_live_safe_while_running(w: &crate::types::ConfigValue) -> bool {
    use crate::types::ConfigValue;
    matches!(
        w,
        ConfigValue::CurrentLimits(_) | ConfigValue::Failsafe(_) | ConfigValue::Velocity(_)
    )
}

#[cfg(feature = "storage")]
fn config_value(
    cfg: &RuntimeConfig,
    group: crate::types::ConfigGroupId,
) -> Option<crate::types::ConfigValue> {
    use crate::types::{ConfigGroupId as G, ConfigValue as V};
    match group {
        G::BoardCalibration => cfg.board_calibration.map(V::BoardCalibration),
        G::MotorParams => cfg.motor_params.clone().map(V::MotorParams),
        G::HallCalibration => cfg.hall_calibration.map(V::HallCalibration),
        G::DcOffsets => cfg.dc_offsets.clone().map(V::DcOffsets),
        G::CurrentLimits => cfg.current_limits.clone().map(V::CurrentLimits),
        G::VoltageLimits => cfg.voltage_limits.clone().map(V::VoltageLimits),
        G::PwmConfig => cfg.pwm_config.clone().map(V::PwmConfig),
        G::PiGains => cfg.pi_gains.clone().map(V::PiGains),
        G::HallTuning => cfg.hall_tuning.clone().map(V::HallTuning),
        G::Failsafe => cfg.failsafe.clone().map(V::Failsafe),
        G::Velocity => cfg.velocity.map(V::Velocity),
        G::Derating => cfg.derating.map(V::Derating),
        G::VoltageFeedforward => cfg.voltage_feedforward.map(V::VoltageFeedforward),
        G::MotorTemperature => cfg.motor_temperature.map(V::MotorTemperature),
        G::Drive => cfg.drive.map(V::Drive),
    }
}

#[cfg(feature = "storage")]
fn resolved_config(
    cfg: &RuntimeConfig,
    group: crate::types::ConfigGroupId,
    board: &crate::foc::config::BoardConfig,
) -> Option<crate::types::ConfigValue> {
    use crate::storage::{CurrentLimitsConfig, PiGainsConfig, VoltageLimitsConfig};
    use crate::types::{ConfigGroupId, ConfigValue};
    match group {
        ConfigGroupId::VoltageLimits => Some(ConfigValue::VoltageLimits(
            cfg.voltage_limits.clone().unwrap_or(VoltageLimitsConfig {
                min_vbus_mv: board.min_vbus_mv,
                max_vbus_mv: board.max_vbus_mv,
            }),
        )),
        ConfigGroupId::CurrentLimits => {
            let rating = cfg
                .motor_params
                .as_ref()
                .and_then(crate::storage::MotorParamsConfig::rating_current_a)
                .unwrap_or(0.0);
            let limits = crate::motor::foc_driver::CurrentLimits::from_stored(
                cfg.current_limits.as_ref(),
                board.max_phase_current_a,
                rating,
            );
            Some(ConfigValue::CurrentLimits(CurrentLimitsConfig {
                max_iq_a: limits.max_current_a,
                max_brake_a: limits.max_brake_a,
                max_phase_current_a: limits.overcurrent_threshold_a,
                bus_in_max_a: limits.bus_in_max_a,
                bus_regen_max_a: limits.bus_regen_max_a,
                max_dissipation_w: limits.max_dissipation_w,
            }))
        }
        ConfigGroupId::PiGains => {
            let controller = crate::foc::controller::FocController::<
                crate::foc::pwm::SvpwmModulator,
                crate::foc::trig::LibmSinCos,
            >::from_runtime_config(cfg, 24.0);
            let (kp, ki) = controller.id_pi.gains();
            Some(ConfigValue::PiGains(PiGainsConfig {
                kp,
                ki,
                bandwidth_rad_s: cfg.pi_gains.as_ref().filter(|v| v.is_sane()).map_or(
                    crate::foc::detection::pi_tuning::DEFAULT_BANDWIDTH_RAD_S,
                    |v| v.bandwidth_rad_s,
                ),
            }))
        }
        _ => None,
    }
}

/// Configuration server - handles config read/write/reset requests
///
/// Reads return the configured value, its optimistic-concurrency revision, and
/// whether flash contains that exact revision. Writes are deliberately split:
///
/// * `Apply` validates and installs live settings before acknowledging. PWM
///   is staged for the next boot. Groups whose driver update is known to be live-safe may
///   be applied while running; structural/calibration groups remain stop-only.
/// * `Persist` writes one explicitly named live revision through
///   `FLASH_CHANNEL`. It is always stop-only because internal-flash erase can
///   stall a single-bank MCU long enough to starve the FOC ISR. A concurrent
///   Apply causes `Conflict`, never persistence of an unintended newer value.
/// * baked-config firmware returns `Unsupported` for `Persist`; the volatile
///   Apply remains useful for bench tuning and `config dump --rust` extraction.
///
/// Apply/Persist carry independent request IDs. The small response cache makes
/// host retries effectively-once within the cache window and rejects reused IDs with different requests.
///
/// The board profile supplies the physical limits used to resolve effective values.
#[cfg(feature = "storage")]
#[allow(clippy::too_many_arguments)]
pub async fn config_server<NS, T: crate::timer::Timer, const N: usize>(
    endpoints: Endpoints<NS>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    board: &crate::foc::config::BoardConfig,
    boot_epoch: u64,
    drive_pwm_hz: u32,
    persist: bool,
    pwm_limits: Option<crate::storage::PwmConfigLimits>,
) where
    NS: NetStackHandle,
{
    let hw_max_current_a = board.max_phase_current_a;
    use crate::icd::ConfigGroupId;
    use crate::types::{ConfigSnapshot, ConfigValue, MotorState};

    let server = endpoints.bounded_server::<ConfigEndpoint, N>(Some("config"));
    let server = pin!(server);
    let mut h = server.attach();
    let revisions = &CONFIG_COORDINATOR.revisions;
    let persisted_revisions = &CONFIG_COORDINATOR.persisted;
    if persist {
        critical_section::with(|cs| {
            let cfg = runtime_config.borrow(cs).borrow();
            let revisions = revisions.borrow(cs).borrow();
            let mut saved = persisted_revisions.borrow(cs).borrow_mut();
            for group in ConfigGroupId::ALL {
                let index = group.index();
                if revisions[index] == 0 && config_value(&cfg, group).is_some() {
                    saved[index] = Some(0);
                }
            }
        });
    }
    let action_cache = &CONFIG_COORDINATOR.actions;
    let reset_ack = CriticalSectionMutex::new(core::cell::Cell::new(None));
    let reset_ack = &reset_ack;

    #[cfg(all(feature = "detection", feature = "delivery"))]
    let setup_ack = CriticalSectionMutex::new(RefCell::new(
        None::<(
            crate::types::Keyed<crate::types::SaveDriveSetup>,
            ConfigResponse,
        )>,
    ));
    #[cfg(all(feature = "detection", feature = "delivery"))]
    let setup_ack = &setup_ack;

    loop {
        let _ = h
            .serve(|req: &ConfigRequest| {
                let req = req.clone();
                async move {
                    let (motor_running, maintenance_busy) = critical_section::with(|cs| {
                        let state = state_mutex.borrow(cs).borrow();
                        (
                            state.motor_state == MotorState::Running,
                            state.driver_operation != DriverOperation::Idle
                                || crate::state::boot_current_offset_pending()
                                || crate::state::current_offset_request_pending()
                                || crate::state::model_apply_pending(),
                        )
                    });
                    let versioned = match &req {
                        ConfigRequest::Apply(keyed) => {
                            Some((keyed.inner.boot_epoch, keyed.inner.write.group()))
                        }
                        ConfigRequest::Persist(keyed) => {
                            Some((keyed.inner.boot_epoch, keyed.inner.group))
                        }
                        _ => None,
                    };
                    if let Some((epoch, group)) = versioned
                        && epoch != boot_epoch
                    {
                        let current_revision = critical_section::with(|cs| {
                            revisions.borrow(cs).borrow()[group.index()]
                        });
                        return ConfigResponse::Conflict { current_revision };
                    }
                    let action_id = match &req {
                        ConfigRequest::Apply(keyed) => Some(keyed.id),
                        ConfigRequest::Persist(keyed) => Some(keyed.id),
                        ConfigRequest::Read(_)
                        | ConfigRequest::ResetAll(_)
                        | ConfigRequest::SaveDriveSetup(_) => None,
                    };
                    let replay = action_id
                        .map(|id| {
                            critical_section::with(|cs| {
                                action_cache.borrow(cs).borrow_mut().begin(id, &req)
                            })
                            .map(|ack| ack.map(|ack| (id, ack)))
                        })
                        .transpose();
                    let Ok(replay) = replay else {
                        return ConfigResponse::Invalid;
                    };
                    if let Some((id, ack)) = replay.flatten() {
                        return match ack {
                            CachedConfigAck::Applied(revision) => ConfigResponse::Applied {
                                req_id: id,
                                revision,
                            },
                            CachedConfigAck::Staged(revision) => ConfigResponse::Staged {
                                req_id: id,
                                revision,
                            },
                            CachedConfigAck::Persisted(revision) => ConfigResponse::Persisted {
                                req_id: id,
                                revision,
                            },
                        };
                    }
                    match req {
                        ConfigRequest::Read(group) => {
                            let (cfg, mut snapshot) = critical_section::with(|cs| {
                                let cfg = runtime_config.borrow(cs).borrow();
                                let revision = revisions.borrow(cs).borrow()[group.index()];
                                let value = config_value(&cfg, group);
                                let default =
                                    if value.is_none() && group == ConfigGroupId::CurrentLimits {
                                        Some(ConfigValue::CurrentLimits(
                                            crate::storage::CurrentLimitsConfig::AUTO,
                                        ))
                                    } else {
                                        None
                                    };
                                let snapshot = ConfigSnapshot {
                                    group,
                                    revision,
                                    boot_epoch,
                                    valid: match group {
                                        ConfigGroupId::BoardCalibration => {
                                            cfg.board_calibration_is_valid()
                                        }
                                        ConfigGroupId::HallCalibration => {
                                            cfg.hall_calibration_is_valid()
                                        }
                                        ConfigGroupId::Drive => cfg.drive_is_valid(),
                                        _ => true,
                                    },
                                    persisted: persisted_revisions.borrow(cs).borrow()
                                        [group.index()]
                                        == Some(revision),
                                    value,
                                    effective: None,
                                    default,
                                };
                                (cfg.clone(), snapshot)
                            });
                            snapshot.effective = if group == ConfigGroupId::PwmConfig {
                                pwm_limits.map(|limits| {
                                    ConfigValue::PwmConfig(crate::storage::PwmConfigStored {
                                        freq_hz: drive_pwm_hz,
                                        max_duty_percent: limits.max_duty_percent,
                                    })
                                })
                            } else {
                                resolved_config(&cfg, group, board)
                            };
                            ConfigResponse::Snapshot(snapshot)
                        }
                        // Boundary validation, before any persistence —
                        // the per-group rules live in `write_is_acceptable`
                        // (a sync helper: it keeps the float checks out of
                        // this task's future, which flash-tight boards'
                        // flash and RAM budgets ride).
                        ConfigRequest::Apply(ref keyed)
                            if !write_is_acceptable(&keyed.inner.write, pwm_limits) =>
                        {
                            ConfigResponse::Invalid
                        }
                        ConfigRequest::Apply(_) if maintenance_busy => ConfigResponse::Busy,
                        ConfigRequest::Apply(ref keyed)
                            if motor_running
                                && !write_is_live_safe_while_running(&keyed.inner.write) =>
                        {
                            ConfigResponse::Busy
                        }
                        ConfigRequest::Apply(keyed) => {
                            let write = keyed.inner.write;
                            let group = write.group();
                            let current_revision = critical_section::with(|cs| {
                                revisions.borrow(cs).borrow()[group.index()]
                            });
                            if keyed.inner.expected_revision != current_revision {
                                return ConfigResponse::Conflict { current_revision };
                            }
                            // A measurement-source update needs no FocDriver mutation.
                            // Commit it with its mirror/revision under one CS, between
                            // complete ADC ISR executions.
                            if matches!(
                                &write,
                                ConfigValue::BoardCalibration(_) | ConfigValue::MotorTemperature(_)
                            ) {
                                return critical_section::with(|cs| {
                                    let mut state = state_mutex.borrow(cs).borrow_mut();
                                    let mut cfg = runtime_config.borrow(cs).borrow_mut();
                                    match &write {
                                        ConfigValue::BoardCalibration(value) => {
                                            if !state.can_apply_board_calibration() {
                                                return ConfigResponse::Busy;
                                            }
                                            state.board_calibration = *value;
                                            state.board_calibration_valid = true;
                                            cfg.board_calibration = Some(*value);
                                            cfg.board_calibration_invalid = false;
                                        }
                                        ConfigValue::MotorTemperature(value) => {
                                            // Permit repair after a missing expected sensor
                                            // has already raised a stop fault.
                                            if state.actuator_busy()
                                                || state.control_mode
                                                    != crate::types::ControlMode::Stopped
                                                || crate::state::DETECTION_ACTIVE
                                                    .load(core::sync::atomic::Ordering::Acquire)
                                            {
                                                return ConfigResponse::Busy;
                                            }
                                            crate::state::set_motor_temperature_input(*value);
                                            cfg.motor_temperature = Some(*value);
                                        }
                                        _ => unreachable!(),
                                    }
                                    let revision = current_revision.wrapping_add(1);
                                    revisions.borrow(cs).borrow_mut()[group.index()] = revision;
                                    action_cache
                                        .borrow(cs)
                                        .borrow_mut()
                                        .insert(keyed.id, CachedConfigAck::Applied(revision));
                                    ConfigResponse::Applied {
                                        req_id: keyed.id,
                                        revision,
                                    }
                                });
                            }
                            if let ConfigValue::HallCalibration(value) = &write {
                                let Some(guard) = ModelApplyGuard::prepare_hall(
                                    *value,
                                    runtime_config,
                                    revisions,
                                    action_cache,
                                    keyed.id,
                                    current_revision,
                                ) else {
                                    return ConfigResponse::Busy;
                                };
                                return guard.apply::<T>().await;
                            }
                            if let ConfigValue::VoltageLimits(value) = &write {
                                let Some(guard) = ModelApplyGuard::prepare_voltage_limits(
                                    value.clone(),
                                    runtime_config,
                                    &CONFIG_COORDINATOR,
                                    keyed.id,
                                    current_revision,
                                ) else {
                                    return ConfigResponse::Busy;
                                };
                                return guard.apply::<T>().await;
                            }
                            if let ConfigValue::Drive(value) = &write {
                                let Some(guard) = ModelApplyGuard::prepare_drive(
                                    *value,
                                    runtime_config,
                                    &CONFIG_COORDINATOR,
                                    keyed.id,
                                    current_revision,
                                ) else {
                                    return ConfigResponse::Busy;
                                };
                                return guard.apply::<T>().await;
                            }
                            let model_update = match &write {
                                ConfigValue::MotorParams(value) => {
                                    Some(crate::state::model::ModelConfigUpdate::MotorParams(
                                        value.clone(),
                                    ))
                                }
                                ConfigValue::VoltageFeedforward(value) => Some(
                                    crate::state::model::ModelConfigUpdate::VoltageFeedforward(
                                        *value,
                                    ),
                                ),
                                _ => None,
                            };
                            if let Some(update) = model_update {
                                let mut config = critical_section::with(|cs| {
                                    runtime_config.borrow(cs).borrow().clone()
                                });
                                update.apply_to_config(&mut config);
                                let vbus = critical_section::with(|cs| {
                                    state_mutex.borrow(cs).borrow().last_adc.vbus_mv as f32 / 1000.0
                                });
                                let model =
                                    crate::motor::foc_driver::PreparedMotorModel::from_config(
                                        &config,
                                        vbus,
                                        hw_max_current_a,
                                    );
                                let Some(guard) = ModelApplyGuard::prepare(
                                    model,
                                    update,
                                    runtime_config,
                                    revisions,
                                    action_cache,
                                    keyed.id,
                                    current_revision,
                                ) else {
                                    return ConfigResponse::Busy;
                                };
                                return guard.apply::<T>().await;
                            }
                            use crate::state::model::TuningUpdate;
                            let tuning = match write {
                                ConfigValue::CurrentLimits(v) => {
                                    Some(TuningUpdate::CurrentLimits(v))
                                }
                                ConfigValue::PiGains(v) => Some(TuningUpdate::PiGains(v)),
                                ConfigValue::DcOffsets(v) => Some(TuningUpdate::DcOffsets(v)),
                                ConfigValue::Failsafe(v) => Some(TuningUpdate::Failsafe(v)),
                                ConfigValue::Velocity(v) => Some(TuningUpdate::Velocity(v)),
                                ConfigValue::Derating(v) => Some(TuningUpdate::Derating(v)),
                                ConfigValue::HallTuning(v) => Some(TuningUpdate::HallTuning(v)),
                                ConfigValue::PwmConfig(v) => {
                                    critical_section::with(|cs| {
                                        runtime_config.borrow(cs).borrow_mut().pwm_config = Some(v);
                                    });
                                    None
                                }
                                _ => unreachable!(
                                    "handled by the model or measurement input transaction"
                                ),
                            };
                            if let Some(tuning) = tuning {
                                let Some(guard) = ModelApplyGuard::prepare_tuning(
                                    tuning,
                                    hw_max_current_a,
                                    runtime_config,
                                    &CONFIG_COORDINATOR,
                                    keyed.id,
                                    current_revision,
                                ) else {
                                    return ConfigResponse::Busy;
                                };
                                return guard.apply::<T>().await;
                            }
                            let revision = current_revision.wrapping_add(1);
                            critical_section::with(|cs| {
                                revisions.borrow(cs).borrow_mut()[group.index()] = revision;
                                action_cache
                                    .borrow(cs)
                                    .borrow_mut()
                                    .insert(keyed.id, CachedConfigAck::Staged(revision));
                            });
                            ConfigResponse::Staged {
                                req_id: keyed.id,
                                revision,
                            }
                        }
                        ConfigRequest::Persist(keyed) => {
                            if !persist {
                                return ConfigResponse::Unsupported;
                            }
                            let group = keyed.inner.group;
                            if crate::state::drive_setup_recovery_required()
                                && group.is_setup_component()
                            {
                                return ConfigResponse::Busy;
                            }
                            let current_revision = critical_section::with(|cs| {
                                revisions.borrow(cs).borrow()[group.index()]
                            });
                            if keyed.inner.expected_revision != current_revision {
                                return ConfigResponse::Conflict { current_revision };
                            }
                            let _flash_pending = FlashPendingGuard::arm();
                            if group == ConfigGroupId::BoardCalibration
                                && !critical_section::with(|cs| {
                                    state_mutex
                                        .borrow(cs)
                                        .borrow()
                                        .can_apply_board_calibration()
                                })
                            {
                                return ConfigResponse::Busy;
                            }
                            let actuator_busy = critical_section::with(|cs| {
                                state_mutex.borrow(cs).borrow().actuator_busy()
                            });
                            if actuator_busy
                                || crate::state::DETECTION_ACTIVE
                                    .load(core::sync::atomic::Ordering::Acquire)
                            {
                                return ConfigResponse::Busy;
                            }
                            let value = critical_section::with(|cs| {
                                let config = runtime_config.borrow(cs).borrow();
                                if group == ConfigGroupId::HallCalibration
                                    && !config.hall_calibration_is_valid()
                                {
                                    None
                                } else {
                                    config_value(&config, group)
                                }
                            });
                            let Some(value) = value else {
                                return ConfigResponse::Invalid;
                            };
                            if let ConfigValue::PwmConfig(config) = &value
                                && !pwm_limits.is_some_and(|limits| limits.accepts(config))
                            {
                                return ConfigResponse::Invalid;
                            }
                            FLASH_DONE.reset();
                            if FLASH_CHANNEL.try_send(FlashOperation::Save(value)).is_err() {
                                return ConfigResponse::Error;
                            }
                            if !FLASH_DONE.wait().await {
                                return ConfigResponse::Error;
                            }
                            critical_section::with(|cs| {
                                persisted_revisions.borrow(cs).borrow_mut()[group.index()] =
                                    Some(current_revision);
                                action_cache
                                    .borrow(cs)
                                    .borrow_mut()
                                    .insert(keyed.id, CachedConfigAck::Persisted(current_revision));
                            });
                            ConfigResponse::Persisted {
                                req_id: keyed.id,
                                revision: current_revision,
                            }
                        }
                        #[cfg(not(all(feature = "detection", feature = "delivery")))]
                        ConfigRequest::SaveDriveSetup(_) => ConfigResponse::Unsupported,
                        #[cfg(all(feature = "detection", feature = "delivery"))]
                        ConfigRequest::SaveDriveSetup(request) => {
                            let replay = critical_section::with(|cs| {
                                let cache = setup_ack.borrow(cs).borrow();
                                cache
                                    .as_ref()
                                    .filter(|(saved, _)| saved.id == request.id)
                                    .map(|(saved, reply)| {
                                        if saved == &request {
                                            reply.clone()
                                        } else {
                                            ConfigResponse::Invalid
                                        }
                                    })
                            });
                            if let Some(reply) = replay {
                                return reply;
                            }
                            let reply = save_drive_setup::<T>(
                                &request,
                                runtime_config,
                                state_mutex,
                                hw_max_current_a,
                                persist,
                            )
                            .await;
                            if matches!(
                                reply,
                                ConfigResponse::Persisted { .. }
                                    | ConfigResponse::SetupRequiresRestart { .. }
                                    | ConfigResponse::Error
                            ) {
                                critical_section::with(|cs| {
                                    *setup_ack.borrow(cs).borrow_mut() =
                                        Some((request, reply.clone()));
                                });
                            }
                            reply
                        }
                        ConfigRequest::ResetAll(request) => {
                            if request.inner != boot_epoch {
                                return ConfigResponse::Conflict {
                                    current_revision: critical_section::with(|cs| {
                                        revisions.borrow(cs).borrow()
                                            [ConfigGroupId::MotorParams.index()]
                                    }),
                                };
                            }
                            match critical_section::with(|cs| reset_ack.borrow(cs).get()) {
                                Some((id, true)) if id == request.id => {
                                    return ConfigResponse::Reset;
                                }
                                Some((id, false)) if id == request.id => {
                                    return ConfigResponse::ResetRequiresRestart;
                                }
                                _ => {}
                            }
                            if motor_running || maintenance_busy {
                                return ConfigResponse::Busy;
                            }
                            let _calibration_pending = FlashPendingGuard::arm();
                            if !critical_section::with(|cs| {
                                state_mutex
                                    .borrow(cs)
                                    .borrow()
                                    .can_apply_board_calibration()
                            }) {
                                return ConfigResponse::Busy;
                            }
                            let current_revision = critical_section::with(|cs| {
                                revisions.borrow(cs).borrow()[ConfigGroupId::MotorParams.index()]
                            });
                            let config = RuntimeConfig::default();
                            let vbus = critical_section::with(|cs| {
                                state_mutex.borrow(cs).borrow().last_adc.vbus_mv as f32 / 1000.0
                            });
                            let model = crate::motor::foc_driver::PreparedMotorModel::from_config(
                                &config,
                                vbus,
                                hw_max_current_a,
                            );
                            let Some(guard) = ModelApplyGuard::prepare_reset(
                                model,
                                runtime_config,
                                revisions,
                                persisted_revisions,
                                action_cache,
                                current_revision,
                            ) else {
                                return ConfigResponse::Busy;
                            };
                            if persist {
                                FLASH_DONE.reset();
                                if FLASH_CHANNEL.try_send(FlashOperation::EraseAll).is_err() {
                                    return ConfigResponse::Error;
                                }
                                crate::state::set_drive_setup_recovery(true);
                                if !FLASH_DONE.wait().await {
                                    critical_section::with(|cs| {
                                        reset_ack.borrow(cs).set(Some((request.id, false)));
                                    });
                                    return ConfigResponse::ResetRequiresRestart;
                                }
                            }
                            drop(_calibration_pending);
                            let reply = guard.apply::<T>().await;
                            #[cfg(all(feature = "detection", feature = "delivery"))]
                            if matches!(reply, ConfigResponse::Reset) {
                                critical_section::with(|cs| {
                                    *setup_ack.borrow(cs).borrow_mut() = None;
                                });
                                super::detect::invalidate_setup_candidate();
                            }
                            if matches!(reply, ConfigResponse::Reset) {
                                critical_section::with(|cs| {
                                    reset_ack.borrow(cs).set(Some((request.id, true)));
                                });
                                reply
                            } else if persist {
                                critical_section::with(|cs| {
                                    reset_ack.borrow(cs).set(Some((request.id, false)));
                                });
                                ConfigResponse::ResetRequiresRestart
                            } else {
                                reply
                            }
                        }
                    }
                }
            })
            .await;
    }
}

#[cfg(all(feature = "storage", feature = "detection", feature = "delivery"))]
async fn save_drive_setup<T: crate::timer::Timer>(
    request: &crate::types::Keyed<crate::types::SaveDriveSetup>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    board_trip_a: f32,
    persist: bool,
) -> ConfigResponse {
    use crate::state::DETECTION_ACTIVE;
    use crate::types::{ConfigGroupId, ControlMode, MotorState};
    use core::sync::atomic::Ordering;
    if !persist {
        return ConfigResponse::Unsupported;
    }
    let flash_pending = FlashPendingGuard::arm();
    let stopped = critical_section::with(|cs| {
        let state = state_mutex.borrow(cs).borrow();
        state.motor_state == MotorState::Stopped
            && state.control_mode == ControlMode::Stopped
            && !state.actuator_busy()
            && state.board_calibration_valid
            && !DETECTION_ACTIVE.load(Ordering::Acquire)
    });
    if !stopped {
        return ConfigResponse::Busy;
    }
    if !super::detect::setup_is_current(&request.inner) {
        return ConfigResponse::Invalid;
    }
    let (mut config, revision, vbus) = critical_section::with(|cs| {
        (
            runtime_config.borrow(cs).borrow().clone(),
            CONFIG_COORDINATOR.revisions.borrow(cs).borrow()[ConfigGroupId::MotorParams.index()],
            state_mutex.borrow(cs).borrow().last_adc.vbus_mv as f32 / 1000.0,
        )
    });
    let trip = config
        .current_limits
        .as_ref()
        .map_or(0.0, |limits| limits.max_phase_current_a);
    let Ok(setup) = request.inner.setup.clone().validate(board_trip_a, trip) else {
        return ConfigResponse::Invalid;
    };
    let record = crate::storage::InstalledDrive::new(&setup);
    setup.project(&mut config);
    let mut model =
        crate::motor::foc_driver::PreparedMotorModel::from_config(&config, vbus, board_trip_a);
    if model.prepare_setup_feedback(setup.hall()).is_none() {
        return ConfigResponse::Invalid;
    }
    let Some(guard) = ModelApplyGuard::prepare(
        model,
        crate::state::model::ModelConfigUpdate::DriveSetup(setup),
        runtime_config,
        &CONFIG_COORDINATOR.revisions,
        &CONFIG_COORDINATOR.actions,
        request.id,
        revision,
    ) else {
        return ConfigResponse::Busy;
    };
    FLASH_DONE.reset();
    if FLASH_CHANNEL
        .try_send(FlashOperation::SaveInstalledDrive(record))
        .is_err()
    {
        return ConfigResponse::Error;
    }
    // Once a write is queued, an error may leave a complete new record.
    // Inhibit starts until the prepared model is installed or reboot loads it.
    crate::state::set_drive_setup_recovery(true);
    if !FLASH_DONE.wait().await {
        return ConfigResponse::Error;
    }
    drop(flash_pending);
    match guard.apply::<T>().await {
        ConfigResponse::Applied { revision, .. } => ConfigResponse::Persisted {
            req_id: request.id,
            revision,
        },
        _ => ConfigResponse::SetupRequiresRestart { req_id: request.id },
    }
}

/// Telemetry config server
/// Telemetry config server - handles rate change requests from host
///
/// Telemetry config server — host sends `TelemetryConfig { fast_hz }` to start/stop streaming.
///
/// Computes decimation period from FOC frequency and stores in `FAST_TELEM_PERIOD`.
pub async fn telemetry_config_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    foc_freq_hz: u32,
    control_timing: crate::types::ControlTiming,
) where
    NS: NetStackHandle,
{
    use super::streaming::set_fast_telemetry_period;

    let server = endpoints.bounded_server::<TelemetryConfigEndpoint, N>(Some("telemetry_config"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|cfg: &TelemetryConfig| {
                let requested_fast_hz = cfg.fast_hz;
                let period = if requested_fast_hz > 0 {
                    (foc_freq_hz / u32::from(requested_fast_hz)).max(1)
                } else {
                    0 // disabled
                };

                async move {
                    let period = set_fast_telemetry_period(period).await;
                    let actual_fast_hz = control_timing.stream_hz(period);

                    #[cfg(feature = "defmt")]
                    defmt::info!(
                        "TelemetryConfig rx: fast_hz={}, period={}, actual={}Hz",
                        requested_fast_hz,
                        period,
                        actual_fast_hz
                    );

                    #[cfg(feature = "log")]
                    log::info!(
                        "Telemetry config: fast_hz={requested_fast_hz}, period={period}, actual={actual_fast_hz}Hz"
                    );

                    TelemetryConfigAck { actual_fast_hz, decimation_m: period, control_timing }
                }
            })
            .await;
    }
}

/// Slow telemetry server — responds to host poll requests
///
/// Returns current system health data (vbus, temperatures, motor state, faults).
/// Host polls this at ~10Hz, which doubles as a heartbeat for device-side
/// liveness tracking.
pub async fn slow_telemetry_server<NS, F, const N: usize>(
    endpoints: Endpoints<NS>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
) where
    NS: NetStackHandle,
    F: PlatformFault,
{
    let server = endpoints.bounded_server::<SlowTelemetryEndpoint, N>(Some("slow_telem"));
    let server = pin!(server);
    let mut h = server.attach();
    let mut seq: u32 = 0;

    loop {
        seq = seq.wrapping_add(1);
        let current_seq = seq;

        let _ = h
            .serve(|_: &()| {
                let (vbus_mv, fet_temp, motor_temp, board_temp, motor_state, control_mode) =
                    critical_section::with(|cs| {
                        let state = state_mutex.borrow(cs).borrow();
                        (
                            state.last_adc.vbus_mv,
                            state.last_adc.fet_temp_c_x10().unwrap_or(0),
                            state.last_adc.motor_temp_c_x10(),
                            state.last_adc.board_temp_c_x10().unwrap_or(0),
                            state.motor_state,
                            state.control_mode,
                        )
                    });
                let phase_source =
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow().phase_source);

                let fault_count = fault_registry.count() as u8;
                let fault_generation = fault_registry.generation();
                let derating =
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow().derating);

                async move {
                    SlowTelemetry {
                        vbus_mv,
                        fet_temp_c_x10: fet_temp,
                        motor_temp_c_x10: motor_temp,
                        board_temp_c_x10: board_temp,
                        motor_state,
                        control_mode,
                        fault_count,
                        phase_source,
                        seq: current_seq,
                        derate_drive_pct: (derating.drive * 100.0) as u8,
                        derate_brake_pct: (derating.brake * 100.0) as u8,
                        fault_generation,
                        derate_regen_pct: (derating.regen * 100.0) as u8,
                    }
                }
            })
            .await;
    }
}

/// Phase source server — host selects the angle source (hall / observer /
/// HFI / crossovers).
///
/// The command is enqueued to the control ISR; validation happens there
/// (sensor present, estimators configured), so the ack only confirms
/// enqueueing. The host reads the actually-active source back via
/// `SlowTelemetry::phase_source`.
pub async fn phase_source_server<NS, const N: usize>(endpoints: Endpoints<NS>)
where
    NS: NetStackHandle,
{
    use crate::foc::phase::PhaseSource;
    use crate::types::PhaseSourceAck;

    let server = endpoints.bounded_server::<PhaseSourceEndpoint, N>(Some("phase_source"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|source: &PhaseSource| {
                let source = *source;
                async move {
                    // Guaranteed enqueue (ISR drains every cycle); the ack
                    // still only confirms enqueueing, not application.
                    CMD_CHANNEL
                        .send(DriverCommand::SetPhaseSource(source))
                        .await;
                    PhaseSourceAck { enqueued: true }
                }
            })
            .await;
    }
}

/// Run all protocol servers concurrently in a single task
///
/// This is the recommended way to run servers - it uses `join` to run
/// all servers concurrently within a single embassy task, which:
/// - Uses less RAM (one task allocation instead of 4)
/// - Makes it easier to share state
/// - Simplifies spawning (one task instead of many)
///
/// # Arguments
/// * `endpoints` - Ergot endpoints from the net stack
/// * `device_info` - Device information (hardware/software version)
/// * `state_mutex` - Reference to platform's STATE global
/// * `fault_registry` - Reference to platform's FAULT_REGISTRY global
///
/// # Usage
///
/// ```ignore
/// // Platform code:
/// oxifoc_core::define_platform_state!(MyFault);
///
/// #[embassy_executor::task]
/// pub async fn protocol_servers() {
///     let mut hw: String<32> = String::new();
///     let mut sw: String<32> = String::new();
///     hw.push_str("My Board").ok();
///     sw.push_str("v0.1.0").ok();
///
///     oxifoc_core::runtime::run_all_servers(
///         STACK.endpoints(),
///         HardwareInfo { hw, sw, ..Default::default() },
///         &STATE,
///         &FAULT_REGISTRY,
///     ).await
/// }
/// ```
/// Run all protocol servers concurrently (without config server).
///
/// Use [`run_all_servers_with_config`] when the `storage` feature is enabled
/// to include the configuration endpoint.
pub async fn run_all_servers<NS, F, T>(
    endpoints: Endpoints<NS>,
    device_info: HardwareInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    foc_freq_hz: u32,
    control_timing: crate::types::ControlTiming,
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
    T: crate::timer::Timer,
{
    join(
        join3(
            info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
            motor_command_server::<NS, F, T, 2>(endpoints.clone(), state_mutex, fault_registry),
            fault_server::<NS, F, 2>(endpoints.clone(), fault_registry),
        ),
        join3(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            phase_source_server::<NS, 2>(endpoints.clone()),
            telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz, control_timing),
        ),
    )
    .await;
}

/// Run all protocol servers including config endpoint.
#[cfg(feature = "storage")]
#[allow(clippy::too_many_arguments)] // flat board-init facade; a struct would just move the names
pub async fn run_all_servers_with_config<NS, F, T>(
    endpoints: Endpoints<NS>,
    device_info: HardwareInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
    foc_freq_hz: u32,
    control_timing: crate::types::ControlTiming,
    board: &crate::foc::config::BoardConfig,
    boot_epoch: u64,
    persist: bool,
    pwm_limits: Option<crate::storage::PwmConfigLimits>,
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
    T: crate::timer::Timer,
{
    let drive_pwm_hz = device_info.foc_freq_hz;
    join(
        join3(
            info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
            motor_command_server::<NS, F, T, 2>(endpoints.clone(), state_mutex, fault_registry),
            fault_server::<NS, F, 2>(endpoints.clone(), fault_registry),
        ),
        join(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            join3(
                config_server::<NS, T, 2>(
                    endpoints.clone(),
                    runtime_config,
                    state_mutex,
                    board,
                    boot_epoch,
                    drive_pwm_hz,
                    persist,
                    pwm_limits,
                ),
                phase_source_server::<NS, 2>(endpoints.clone()),
                telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz, control_timing),
            ),
        ),
    )
    .await;
}

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

use crate::foc::controller::Decoupling;
use crate::foc::detection::pi_tuning::{DEFAULT_BANDWIDTH_RAD_S, calculate_current_gains};
use crate::foc::velocity::VelocityLoopConfig;
use crate::motor::derating::DeratingConfig;
use crate::motor::failsafe::FailsafeConfig;
use crate::motor::foc_driver::CurrentLimits;
use crate::state::{DriverCommand, DriverOperation, FlashPendingGuard};
use crate::types::MAX_FAULT_RESPONSE;
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
#[cfg(feature = "storage")]
use crate::storage::{
    ConfigKey, ConfigPayload, FLASH_CHANNEL, FLASH_DONE, FlashOperation, RuntimeConfig,
};

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
/// has applied or rejected it.
pub async fn motor_command_server<NS, F, const N: usize>(
    endpoints: Endpoints<NS>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
) where
    NS: NetStackHandle,
    F: PlatformFault,
{
    let server = endpoints.bounded_server::<MotorEndpoint, N>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|request: &MotorRequest| {
                let request = *request;
                crate::runtime::streaming::cmd_stats::MOTOR_REQS
                    .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                async move {
                    MOTOR_COMMAND_DONE.reset();
                    CMD_CHANNEL.send(DriverCommand::Motor(request)).await;
                    let (session, seq, outcome) = MOTOR_COMMAND_DONE.wait().await;
                    debug_assert_eq!(session, request.source_session);
                    debug_assert_eq!(seq, request.seq);
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
    let server = endpoints.bounded_server::<FaultEndpoint, N>(Some("fault"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|req: &FaultRequest| {
                // Handle requests
                match req {
                    FaultRequest::Query => {
                        // Just query, no action needed
                    }
                    FaultRequest::Clear(category) => {
                        fault_registry.clear(*category);
                    }
                    FaultRequest::ClearAll => {
                        fault_registry.clear_all();
                    }
                }

                // Build response with all faults converted to FaultInfo.
                // `total` lets the host see truncation (registry holds up to
                // MAX_FAULTS=16, the response carries at most 8).
                let fault_infos = fault_registry.to_fault_info_vec();
                let total = fault_infos.len() as u8;
                let mut response_faults = heapless::Vec::new();
                for info in fault_infos.iter().take(MAX_FAULT_RESPONSE) {
                    let _ = response_faults.push(info.clone());
                }

                async move {
                    FaultResponse {
                        faults: response_faults,
                        total,
                    }
                }
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
/// - VoltageLimits/PwmConfig: nothing consumes these groups (the UV/OV
///   thresholds and the PWM frequency are compile-time BoardConfig) —
///   accepting the write would persist a silent no-op in exactly the
///   protection domain. Rejected until a consumer exists; reads of
///   previously-stored values still work.
#[cfg(feature = "storage")]
fn write_is_acceptable(w: &crate::types::ConfigWrite) -> bool {
    use crate::types::ConfigWrite;
    let counts_ok = |v: f32| (0.0..=f32::from(u16::MAX)).contains(&v);
    match w {
        ConfigWrite::CurrentLimits(v) => v.is_coherent(),
        ConfigWrite::Derating(v) => DeratingConfig::from(v).is_sane(),
        ConfigWrite::DcOffsets(v) => {
            counts_ok(v.phase_a) && counts_ok(v.phase_b) && counts_ok(v.phase_c)
        }
        ConfigWrite::PiGains(v) => v.is_sane(),
        ConfigWrite::MotorParams(v) => v.is_valid(),
        ConfigWrite::HallCalibration(v) => v.is_calibrated(),
        ConfigWrite::VoltageLimits(_) | ConfigWrite::PwmConfig(_) => false,
        _ => true,
    }
}

#[cfg(feature = "storage")]
fn write_is_live_safe_while_running(w: &crate::types::ConfigWrite) -> bool {
    use crate::types::ConfigWrite;
    matches!(
        w,
        ConfigWrite::CurrentLimits(_)
            | ConfigWrite::Failsafe(_)
            | ConfigWrite::Velocity(_)
            | ConfigWrite::Derating(_)
    )
}

#[cfg(feature = "storage")]
fn config_value(
    cfg: &RuntimeConfig,
    group: crate::types::ConfigGroupId,
) -> Option<crate::types::ConfigValue> {
    use crate::types::{ConfigGroupId as G, ConfigValue as V};
    match group {
        G::MotorParams => cfg.motor_params.clone().map(V::MotorParams),
        G::HallCalibration => cfg.hall_calibration.clone().map(V::HallCalibration),
        G::DcOffsets => cfg.dc_offsets.clone().map(V::DcOffsets),
        G::CurrentLimits => cfg.current_limits.clone().map(V::CurrentLimits),
        G::VoltageLimits => cfg.voltage_limits.clone().map(V::VoltageLimits),
        G::PwmConfig => cfg.pwm_config.clone().map(V::PwmConfig),
        G::PiGains => cfg.pi_gains.clone().map(V::PiGains),
        G::HallTuning => cfg.hall_tuning.clone().map(V::HallTuning),
        G::Failsafe => cfg.failsafe.clone().map(V::Failsafe),
        G::Velocity => cfg.velocity.clone().map(V::Velocity),
        G::Derating => cfg.derating.clone().map(V::Derating),
    }
}

#[cfg(feature = "storage")]
fn flash_parts(value: crate::types::ConfigValue) -> (ConfigKey, ConfigPayload) {
    use crate::types::ConfigValue as V;
    match value {
        V::MotorParams(v) => (ConfigKey::MotorParams, ConfigPayload::MotorParams(v)),
        V::CurrentLimits(v) => (ConfigKey::CurrentLimits, ConfigPayload::CurrentLimits(v)),
        V::VoltageLimits(v) => (ConfigKey::VoltageLimits, ConfigPayload::VoltageLimits(v)),
        V::PwmConfig(v) => (ConfigKey::PwmConfig, ConfigPayload::PwmConfig(v)),
        V::PiGains(v) => (ConfigKey::PiGains, ConfigPayload::PiGains(v)),
        V::HallTuning(v) => (ConfigKey::HallTuning, ConfigPayload::HallTuning(v)),
        V::HallCalibration(v) => (
            ConfigKey::HallCalibration,
            ConfigPayload::HallCalibration(v),
        ),
        V::DcOffsets(v) => (ConfigKey::DcOffsets, ConfigPayload::DcOffsets(v)),
        V::Failsafe(v) => (ConfigKey::Failsafe, ConfigPayload::Failsafe(v)),
        V::Velocity(v) => (ConfigKey::Velocity, ConfigPayload::Velocity(v)),
        V::Derating(v) => (ConfigKey::Derating, ConfigPayload::Derating(v)),
    }
}

#[cfg(feature = "storage")]
#[derive(Clone, Copy)]
enum CachedConfigAck {
    Applied(u32),
    Persisted(u32),
}

#[cfg(feature = "storage")]
struct ConfigActionCache {
    entries: [Option<(crate::types::ReqId, CachedConfigAck)>; 4],
    next: usize,
}

#[cfg(feature = "storage")]
impl ConfigActionCache {
    const fn new() -> Self {
        Self {
            entries: [None; 4],
            next: 0,
        }
    }

    fn get(&self, id: crate::types::ReqId) -> Option<CachedConfigAck> {
        self.entries
            .iter()
            .flatten()
            .find_map(|(cached, ack)| (*cached == id).then_some(*ack))
    }

    fn insert(&mut self, id: crate::types::ReqId, ack: CachedConfigAck) {
        self.entries[self.next] = Some((id, ack));
        self.next = (self.next + 1) % self.entries.len();
    }
}

/// Configuration server - handles config read/write/reset requests
///
/// Reads return the live value, its optimistic-concurrency revision, and
/// whether flash contains that exact revision. Writes are deliberately split:
///
/// * `Apply` validates and changes RAM/live driver state, then increments the
///   group revision. Groups whose driver update is known to be live-safe may
///   be applied while running; structural/calibration groups remain stop-only.
/// * `Persist` writes one explicitly named live revision through
///   `FLASH_CHANNEL`. It is always stop-only because internal-flash erase can
///   stall a single-bank MCU long enough to starve the FOC ISR. A concurrent
///   Apply causes `Conflict`, never persistence of an unintended newer value.
/// * baked-config firmware returns `Unsupported` for `Persist`; the volatile
///   Apply remains useful for bench tuning and `config dump --rust` extraction.
///
/// Apply/Persist carry independent request IDs. The small response cache makes
/// host retries effectively-once without retaining full configuration payloads.
///
/// `hw_max_current_a` is the board's hardware phase-current ceiling —
/// current-limit writes are clamped to it and applied to the live driver
/// via [`DriverCommand::SetCurrentLimits`] on the command channel.
#[cfg(feature = "storage")]
pub async fn config_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    hw_max_current_a: f32,
    persist: bool,
) where
    NS: NetStackHandle,
{
    use crate::icd::ConfigGroupId;
    use crate::types::{ConfigSnapshot, ConfigWrite, MotorState};

    let server = endpoints.bounded_server::<ConfigEndpoint, N>(Some("config"));
    let server = pin!(server);
    let mut h = server.attach();
    let revisions_storage = CriticalSectionMutex::new(RefCell::new([0u32; ConfigGroupId::COUNT]));
    let revisions = &revisions_storage;
    let mut initially_persisted = [None; ConfigGroupId::COUNT];
    if persist {
        let cfg = critical_section::with(|cs| runtime_config.borrow(cs).borrow().clone());
        for group in ConfigGroupId::ALL {
            if config_value(&cfg, group).is_some() {
                initially_persisted[group.index()] = Some(0);
            }
        }
    }
    let persisted_revisions_storage = CriticalSectionMutex::new(RefCell::new(initially_persisted));
    let persisted_revisions = &persisted_revisions_storage;
    let action_cache_storage = CriticalSectionMutex::new(RefCell::new(ConfigActionCache::new()));
    let action_cache = &action_cache_storage;

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
                                || crate::state::current_offset_request_pending(),
                        )
                    });
                    let action_id = match &req {
                        ConfigRequest::Apply(keyed) => Some(keyed.id),
                        ConfigRequest::Persist(keyed) => Some(keyed.id),
                        ConfigRequest::Read(_) | ConfigRequest::ResetAll => None,
                    };
                    if let Some((id, ack)) = action_id.and_then(|id| {
                        critical_section::with(|cs| {
                            action_cache
                                .borrow(cs)
                                .borrow()
                                .get(id)
                                .map(|ack| (id, ack))
                        })
                    }) {
                        return match ack {
                            CachedConfigAck::Applied(revision) => ConfigResponse::Applied {
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
                            let cfg = critical_section::with(|cs| {
                                runtime_config.borrow(cs).borrow().clone()
                            });
                            let revision = critical_section::with(|cs| {
                                revisions.borrow(cs).borrow()[group.index()]
                            });
                            ConfigResponse::Snapshot(ConfigSnapshot {
                                group,
                                revision,
                                persisted: critical_section::with(|cs| {
                                    persisted_revisions.borrow(cs).borrow()[group.index()]
                                        == Some(revision)
                                }),
                                value: config_value(&cfg, group),
                            })
                        }
                        // Boundary validation, before any persistence —
                        // the per-group rules live in `write_is_acceptable`
                        // (a sync helper: it keeps the float checks out of
                        // this task's future, which flash-tight boards'
                        // flash and RAM budgets ride).
                        ConfigRequest::Apply(ref keyed)
                            if !write_is_acceptable(&keyed.inner.write) =>
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
                        ConfigRequest::ResetAll if motor_running || maintenance_busy => {
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
                            // Update the in-memory mirror.
                            critical_section::with(|cs| {
                                let mut cfg = runtime_config.borrow(cs).borrow_mut();
                                match &write {
                                    ConfigWrite::MotorParams(v) => {
                                        cfg.motor_params = Some(v.clone());
                                    }
                                    ConfigWrite::CurrentLimits(v) => {
                                        cfg.current_limits = Some(v.clone());
                                    }
                                    ConfigWrite::VoltageLimits(v) => {
                                        cfg.voltage_limits = Some(v.clone());
                                    }
                                    ConfigWrite::PwmConfig(v) => cfg.pwm_config = Some(v.clone()),
                                    ConfigWrite::PiGains(v) => cfg.pi_gains = Some(v.clone()),
                                    ConfigWrite::HallTuning(v) => {
                                        cfg.hall_tuning = Some(v.clone());
                                    }
                                    ConfigWrite::HallCalibration(v) => {
                                        cfg.hall_calibration = Some(v.clone());
                                    }
                                    ConfigWrite::DcOffsets(v) => {
                                        cfg.dc_offsets = Some(v.clone());
                                    }
                                    ConfigWrite::Failsafe(v) => cfg.failsafe = Some(v.clone()),
                                    ConfigWrite::Velocity(v) => cfg.velocity = Some(v.clone()),
                                    ConfigWrite::Derating(v) => cfg.derating = Some(v.clone()),
                                }
                            });
                            // Make the write take effect on the live driver,
                            // not only at the next boot. `send().await`, not
                            // `try_send`: the ISR drains the channel every
                            // cycle, and a silent drop here leaves the saved
                            // config and the live driver disagreeing (worst
                            // case: MotorParams applies its gains but loses
                            // the decoupling command on a full channel).
                            // Motor rating ceiling for the limits clamp —
                            // read back from the just-updated mirror so a
                            // simultaneous MotorParams write is reflected.
                            let rating_a = critical_section::with(|cs| {
                                runtime_config
                                    .borrow(cs)
                                    .borrow()
                                    .motor_params
                                    .as_ref()
                                    .and_then(
                                        super::super::storage::MotorParamsConfig::rating_current_a,
                                    )
                                    .unwrap_or(0.0)
                            });
                            match write {
                                // Limits: clamped to the board ceiling and
                                // the motor rating.
                                ConfigWrite::CurrentLimits(ref v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetCurrentLimits(
                                            CurrentLimits::from_config_clamped(
                                                v,
                                                hw_max_current_a,
                                                rating_a,
                                            ),
                                        ))
                                        .await;
                                }
                                // Explicit PI gains apply verbatim.
                                ConfigWrite::PiGains(v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetPiGains { kp: v.kp, ki: v.ki })
                                        .await;
                                }
                                // New motor params (post-detection write):
                                // retune the current loop the same way boot
                                // does, otherwise the driver keeps the
                                // conservative detection gains until reboot.
                                // Same precedence as boot: explicit stored PI
                                // gains win over the l_avg-derived tuning
                                // (pulse Ld/Lq are the decoupling values; the
                                // loop runs on the HF inductance — see
                                // FocController::from_runtime_config).
                                // Validity is guaranteed by the boundary
                                // guard above — no silent-skip arm left.
                                ConfigWrite::MotorParams(v) => {
                                    let stored_gains = critical_section::with(|cs| {
                                        runtime_config.borrow(cs).borrow().pi_gains.clone()
                                    });
                                    if let Some(pg) = stored_gains {
                                        CMD_CHANNEL
                                            .send(DriverCommand::SetPiGains {
                                                kp: pg.kp,
                                                ki: pg.ki,
                                            })
                                            .await;
                                    } else {
                                        let l_avg = (v.inductance_d_h + v.inductance_q_h) / 2.0;
                                        let (kp, ki) = calculate_current_gains(
                                            v.resistance_ohm,
                                            l_avg,
                                            DEFAULT_BANDWIDTH_RAD_S,
                                        );
                                        CMD_CHANNEL
                                            .send(DriverCommand::SetPiGains { kp, ki })
                                            .await;
                                    }
                                    // New inductances/flux also re-arm the
                                    // dq-decoupling feedforward, same as boot.
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetDecoupling(Decoupling {
                                            ld_h: v.inductance_d_h,
                                            lq_h: v.inductance_q_h,
                                            flux_linkage_wb: v.flux_linkage_wb,
                                        }))
                                        .await;
                                    // A new rating re-clamps the live limits
                                    // (a lower-rated motor must take effect
                                    // now, not at the next boot).
                                    let limits = critical_section::with(|cs| {
                                        runtime_config.borrow(cs).borrow().current_limits.clone()
                                    });
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetCurrentLimits(
                                            CurrentLimits::from_stored(
                                                limits.as_ref(),
                                                hw_max_current_a,
                                                rating_a,
                                            ),
                                        ))
                                        .await;
                                }
                                // Cruise velocity-loop tuning applies live.
                                ConfigWrite::Velocity(v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetVelocityConfig(
                                            VelocityLoopConfig::from_stored(Some(&v)),
                                        ))
                                        .await;
                                }
                                // Derating ramps apply to the live driver.
                                ConfigWrite::Derating(v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetDerating(
                                            DeratingConfig::from_stored(Some(&v)),
                                        ))
                                        .await;
                                }
                                // Failsafe tuning applies to the live driver.
                                ConfigWrite::Failsafe(v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetFailsafe(
                                            FailsafeConfig::from_stored(Some(&v)),
                                        ))
                                        .await;
                                }
                                // Persisted calibration must take effect now,
                                // not only after the next reboot.
                                ConfigWrite::DcOffsets(v) => {
                                    CMD_CHANNEL
                                        .send(DriverCommand::SetCurrentOffsets([
                                            v.phase_a, v.phase_b, v.phase_c,
                                        ]))
                                        .await;
                                }
                                _ => {}
                            }
                            let revision = current_revision.wrapping_add(1);
                            critical_section::with(|cs| {
                                revisions.borrow(cs).borrow_mut()[group.index()] = revision;
                                action_cache
                                    .borrow(cs)
                                    .borrow_mut()
                                    .insert(keyed.id, CachedConfigAck::Applied(revision));
                            });
                            ConfigResponse::Applied {
                                req_id: keyed.id,
                                revision,
                            }
                        }
                        ConfigRequest::Persist(keyed) => {
                            if !persist {
                                return ConfigResponse::Unsupported;
                            }
                            let group = keyed.inner.group;
                            let current_revision = critical_section::with(|cs| {
                                revisions.borrow(cs).borrow()[group.index()]
                            });
                            if keyed.inner.expected_revision != current_revision {
                                return ConfigResponse::Conflict { current_revision };
                            }
                            let _flash_pending = FlashPendingGuard::arm();
                            let actuator_busy = critical_section::with(|cs| {
                                state_mutex.borrow(cs).borrow().actuator_busy()
                            });
                            if actuator_busy {
                                return ConfigResponse::Busy;
                            }
                            let value = critical_section::with(|cs| {
                                config_value(&runtime_config.borrow(cs).borrow(), group)
                            });
                            let Some(value) = value else {
                                return ConfigResponse::Invalid;
                            };
                            let (key, payload) = flash_parts(value);
                            FLASH_DONE.reset();
                            if FLASH_CHANNEL
                                .try_send(FlashOperation::Save(key, payload))
                                .is_err()
                            {
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
                        ConfigRequest::ResetAll => {
                            if persist {
                                // Same TOCTOU guard as the Write arm above.
                                let _flash_pending = FlashPendingGuard::arm();
                                let actuator_busy = critical_section::with(|cs| {
                                    state_mutex.borrow(cs).borrow().actuator_busy()
                                });
                                if actuator_busy {
                                    return ConfigResponse::Busy;
                                }
                                FLASH_DONE.reset();
                                if FLASH_CHANNEL.try_send(FlashOperation::EraseAll).is_err() {
                                    return ConfigResponse::Error;
                                }
                                if !FLASH_DONE.wait().await {
                                    return ConfigResponse::Error;
                                }
                            }
                            critical_section::with(|cs| {
                                *runtime_config.borrow(cs).borrow_mut() = RuntimeConfig::default();
                            });
                            // Stored limits are gone — restore board defaults.
                            CMD_CHANNEL
                                .send(DriverCommand::SetCurrentLimits(
                                    CurrentLimits::from_max_current(hw_max_current_a),
                                ))
                                .await;
                            critical_section::with(|cs| {
                                *revisions.borrow(cs).borrow_mut() = [0; ConfigGroupId::COUNT];
                                *persisted_revisions.borrow(cs).borrow_mut() =
                                    [None; ConfigGroupId::COUNT];
                            });
                            ConfigResponse::Reset
                        }
                    }
                }
            })
            .await;
    }
}

/// Telemetry config server
/// Telemetry config server - handles rate change requests from host
///
/// Telemetry config server — host sends `TelemetryConfig { fast_hz }` to start/stop streaming.
///
/// Computes decimation period from FOC frequency and stores in `FAST_TELEM_PERIOD`.
pub async fn telemetry_config_server<NS, const N: usize>(endpoints: Endpoints<NS>, foc_freq_hz: u32)
where
    NS: NetStackHandle,
{
    use super::streaming::FAST_TELEM_PERIOD;
    use core::sync::atomic::Ordering;

    let server = endpoints.bounded_server::<TelemetryConfigEndpoint, N>(Some("telemetry_config"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|cfg: &TelemetryConfig| {
                let (period, actual_fast_hz) = if cfg.fast_hz > 0 {
                    let period = (foc_freq_hz / u32::from(cfg.fast_hz.max(1))).max(1);
                    let actual = (foc_freq_hz / period) as u16;
                    (period, actual)
                } else {
                    (0, 0) // disabled
                };

                FAST_TELEM_PERIOD.store(period, Ordering::Relaxed);

                #[cfg(feature = "defmt")]
                defmt::info!(
                    "TelemetryConfig rx: fast_hz={}, period={}, actual={}Hz",
                    cfg.fast_hz,
                    period,
                    actual_fast_hz
                );

                #[cfg(feature = "log")]
                log::info!(
                    "Telemetry config: fast_hz={}, period={}, actual={}Hz",
                    cfg.fast_hz,
                    period,
                    actual_fast_hz
                );

                async move { TelemetryConfigAck { actual_fast_hz } }
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
                            state.last_adc.motor_temp_c_x10().unwrap_or(0),
                            state.last_adc.board_temp_c_x10().unwrap_or(0),
                            state.motor_state,
                            state.control_mode,
                        )
                    });
                let phase_source =
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow().phase_source);

                let fault_count = fault_registry.count() as u8;
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
pub async fn run_all_servers<NS, F>(
    endpoints: Endpoints<NS>,
    device_info: HardwareInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    foc_freq_hz: u32,
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
{
    join(
        join3(
            info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
            motor_command_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            fault_server::<NS, F, 2>(endpoints.clone(), fault_registry),
        ),
        join3(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            phase_source_server::<NS, 2>(endpoints.clone()),
            telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz),
        ),
    )
    .await;
}

/// Run all protocol servers including config endpoint.
#[cfg(feature = "storage")]
#[allow(clippy::too_many_arguments)] // flat board-init facade; a struct would just move the names
pub async fn run_all_servers_with_config<NS, F>(
    endpoints: Endpoints<NS>,
    device_info: HardwareInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
    foc_freq_hz: u32,
    hw_max_current_a: f32,
    persist: bool,
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
{
    join(
        join3(
            info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
            motor_command_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            fault_server::<NS, F, 2>(endpoints.clone(), fault_registry),
        ),
        join(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            join3(
                config_server::<NS, 2>(
                    endpoints.clone(),
                    runtime_config,
                    state_mutex,
                    hw_max_current_a,
                    persist,
                ),
                phase_source_server::<NS, 2>(endpoints.clone()),
                telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz),
            ),
        ),
    )
    .await;
}

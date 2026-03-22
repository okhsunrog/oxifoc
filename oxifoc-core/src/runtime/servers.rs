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
//!     let device_info = DeviceInfo { hw: "My Board", sw: "v0.1.0" };
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
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

use crate::foc::fault::{FaultRegistry, PlatformFault};
#[cfg(feature = "storage")]
use crate::icd::{ConfigEndpoint, ConfigRequest, ConfigResponse};
use crate::icd::{
    ControlMode, DeviceInfo, FaultEndpoint, FaultRequest, FaultResponse, InfoEndpoint,
    MotorEndpoint, MotorStatus, SlowTelemetry, SlowTelemetryEndpoint, TelemetryConfig,
    TelemetryConfigAck, TelemetryConfigEndpoint,
};
use crate::state::{CMD_CHANNEL, MotorControlState};
#[cfg(feature = "storage")]
use crate::storage::{ConfigKey, ConfigPayload, FLASH_CHANNEL, FlashOperation, RuntimeConfig};

/// Device info server - responds to info requests from host
///
/// Returns hardware and software version information.
/// Also marks the communication link as active on first request.
pub async fn info_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    device_info: DeviceInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
) where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<InfoEndpoint, N>(Some("device_info"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let info = device_info.clone();
        let result = h
            .serve(|_req: &()| {
                #[cfg(feature = "defmt")]
                defmt::info!("DeviceInfo request received");
                // Mark link as active on first request
                critical_section::with(|cs| {
                    state_mutex.borrow(cs).borrow_mut().set_link_active();
                });
                async move { info }
            })
            .await;
        #[cfg(feature = "defmt")]
        defmt::info!(
            "DeviceInfo response send result: {:?}",
            defmt::Debug2Format(&result)
        );
    }
}

/// Hall sensor server - responds to Hall sensor data requests
///
/// Motor control server - handles motor control mode changes
///
/// Sends ControlMode to CMD_CHANNEL for ISR processing.
/// Returns the current motor status.
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
            .serve(|mode: &ControlMode| {
                // Send control mode to the ISR via channel
                let _ = CMD_CHANNEL.try_send(*mode);

                // Return current status
                let status = critical_section::with(|cs| {
                    let state = state_mutex.borrow(cs).borrow();
                    MotorStatus {
                        state: state.motor_state,
                        mode: state.control_mode,
                        fault_count: fault_registry.count() as u8,
                    }
                });
                async move { status }
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

                // Build response with all faults converted to FaultInfo
                let fault_infos = fault_registry.to_fault_info_vec();
                let mut response_faults = heapless::Vec::new();
                for info in fault_infos.iter().take(crate::types::MAX_FAULT_RESPONSE) {
                    let _ = response_faults.push(info.clone());
                }

                async move {
                    FaultResponse {
                        faults: response_faults,
                    }
                }
            })
            .await;
    }
}

/// Configuration server - handles config read/write/reset requests
///
/// Reads from the shared RuntimeConfig, writes go through FLASH_CHANNEL
/// to the platform's storage worker task.
#[cfg(feature = "storage")]
pub async fn config_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) where
    NS: NetStackHandle,
{
    use crate::icd::ConfigGroupId;
    use crate::types::ConfigWrite;

    let server = endpoints.bounded_server::<ConfigEndpoint, N>(Some("config"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|req: &ConfigRequest| {
                let response = match req {
                    ConfigRequest::Read(group) => {
                        let cfg =
                            critical_section::with(|cs| runtime_config.borrow(cs).borrow().clone());
                        match group {
                            ConfigGroupId::MotorParams => match cfg.motor_params {
                                Some(v) => ConfigResponse::MotorParams(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::HallCalibration => match cfg.hall_calibration {
                                Some(v) => ConfigResponse::HallCalibration(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::DcOffsets => match cfg.dc_offsets {
                                Some(v) => ConfigResponse::DcOffsets(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::CurrentLimits => match cfg.current_limits {
                                Some(v) => ConfigResponse::CurrentLimits(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::VoltageLimits => match cfg.voltage_limits {
                                Some(v) => ConfigResponse::VoltageLimits(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::PwmConfig => match cfg.pwm_config {
                                Some(v) => ConfigResponse::PwmConfig(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::PiGains => match cfg.pi_gains {
                                Some(v) => ConfigResponse::PiGains(v),
                                None => ConfigResponse::NotFound,
                            },
                            ConfigGroupId::HallTuning => match cfg.hall_tuning {
                                Some(v) => ConfigResponse::HallTuning(v),
                                None => ConfigResponse::NotFound,
                            },
                        }
                    }
                    ConfigRequest::Write(write) => {
                        let (key, payload) = match write.clone() {
                            ConfigWrite::MotorParams(v) => {
                                (ConfigKey::MotorParams, ConfigPayload::MotorParams(v))
                            }
                            ConfigWrite::CurrentLimits(v) => {
                                (ConfigKey::CurrentLimits, ConfigPayload::CurrentLimits(v))
                            }
                            ConfigWrite::VoltageLimits(v) => {
                                (ConfigKey::VoltageLimits, ConfigPayload::VoltageLimits(v))
                            }
                            ConfigWrite::PwmConfig(v) => {
                                (ConfigKey::PwmConfig, ConfigPayload::PwmConfig(v))
                            }
                            ConfigWrite::PiGains(v) => {
                                (ConfigKey::PiGains, ConfigPayload::PiGains(v))
                            }
                            ConfigWrite::HallTuning(v) => {
                                (ConfigKey::HallTuning, ConfigPayload::HallTuning(v))
                            }
                        };
                        let _ = FLASH_CHANNEL.try_send(FlashOperation::Save(key, payload));
                        // Update in-memory config
                        critical_section::with(|cs| {
                            let mut cfg = runtime_config.borrow(cs).borrow_mut();
                            match write {
                                ConfigWrite::MotorParams(v) => cfg.motor_params = Some(v.clone()),
                                ConfigWrite::CurrentLimits(v) => {
                                    cfg.current_limits = Some(v.clone())
                                }
                                ConfigWrite::VoltageLimits(v) => {
                                    cfg.voltage_limits = Some(v.clone())
                                }
                                ConfigWrite::PwmConfig(v) => cfg.pwm_config = Some(v.clone()),
                                ConfigWrite::PiGains(v) => cfg.pi_gains = Some(v.clone()),
                                ConfigWrite::HallTuning(v) => cfg.hall_tuning = Some(v.clone()),
                            }
                        });
                        ConfigResponse::Ok
                    }
                    ConfigRequest::ResetAll => {
                        let _ = FLASH_CHANNEL.try_send(FlashOperation::EraseAll);
                        critical_section::with(|cs| {
                            *runtime_config.borrow(cs).borrow_mut() = RuntimeConfig::default();
                        });
                        ConfigResponse::Ok
                    }
                };
                async move { response }
            })
            .await;
    }
}

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
                    let period = (foc_freq_hz / cfg.fast_hz.max(1) as u32).max(1);
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

                let fault_count = fault_registry.count() as u8;

                async move {
                    SlowTelemetry {
                        vbus_mv,
                        fet_temp_c_x10: fet_temp,
                        motor_temp_c_x10: motor_temp,
                        board_temp_c_x10: board_temp,
                        motor_state,
                        control_mode,
                        fault_count,
                        seq: current_seq,
                    }
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
///         DeviceInfo { hw, sw },
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
    device_info: DeviceInfo,
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
        join(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz),
        ),
    )
    .await;
}

/// Run all protocol servers including config endpoint.
#[cfg(feature = "storage")]
pub async fn run_all_servers_with_config<NS, F>(
    endpoints: Endpoints<NS>,
    device_info: DeviceInfo,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
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
        join(
            slow_telemetry_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            join(
                config_server::<NS, 2>(endpoints.clone(), runtime_config),
                telemetry_config_server::<NS, 2>(endpoints, foc_freq_hz),
            ),
        ),
    )
    .await;
}

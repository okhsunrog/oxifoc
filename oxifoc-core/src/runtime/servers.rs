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
use embassy_futures::join::{join, join5};
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::foc::hall_sensor::Direction;
use crate::icd::{
    AdcSample, AdcSampleEndpoint, ConfigEndpoint, ConfigRequest, ConfigResponse, ControlMode,
    DeviceInfo, FaultEndpoint, FaultRequest, FaultResponse, HallSensorData, HallSensorEndpoint,
    InfoEndpoint, MotorEndpoint, MotorStatus,
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
        let _ = h
            .serve(|_req: &()| {
                // Mark link as active on first request
                critical_section::with(|cs| {
                    state_mutex.borrow(cs).borrow_mut().set_link_active();
                });
                async move { info }
            })
            .await;
    }
}

/// Hall sensor server - responds to Hall sensor data requests
///
/// Returns current Hall sensor state including angle, direction,
/// raw state, and error count.
pub async fn hall_sensor_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
) where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<HallSensorEndpoint, N>(Some("hall"));
    let server = pin!(server);
    let mut h = server.attach();

    let mut seq: u32 = 0;

    loop {
        seq = seq.wrapping_add(1);
        let current_seq = seq;

        let _ = h
            .serve(|_: &()| {
                let snapshot =
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow().last_hall);

                async move {
                    match snapshot {
                        Some(s) => HallSensorData {
                            angle_rad: s.angle_rad,
                            direction: s.direction,
                            state: s.state,
                            error_count: s.error_count,
                            seq: current_seq,
                        },
                        None => HallSensorData {
                            angle_rad: 0.0,
                            direction: Direction::Stopped,
                            state: 0,
                            error_count: 0,
                            seq: current_seq,
                        },
                    }
                }
            })
            .await;
    }
}

/// ADC sample server - responds to ADC data requests
///
/// Returns current phase currents, bus voltage, and temperature.
pub async fn adc_sample_server<NS, const N: usize>(
    endpoints: Endpoints<NS>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
) where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<AdcSampleEndpoint, N>(Some("adc"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_: &()| {
                let snapshot =
                    critical_section::with(|cs| state_mutex.borrow(cs).borrow().last_adc.clone());
                async move { AdcSample::from_snapshot(&snapshot) }
            })
            .await;
    }
}

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
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
{
    join5(
        info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
        hall_sensor_server::<NS, 2>(endpoints.clone(), state_mutex),
        adc_sample_server::<NS, 2>(endpoints.clone(), state_mutex),
        motor_command_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
        fault_server::<NS, F, 2>(endpoints, fault_registry),
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
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
{
    join(
        join5(
            info_server::<NS, 2>(endpoints.clone(), device_info, state_mutex),
            hall_sensor_server::<NS, 2>(endpoints.clone(), state_mutex),
            adc_sample_server::<NS, 2>(endpoints.clone(), state_mutex),
            motor_command_server::<NS, F, 2>(endpoints.clone(), state_mutex, fault_registry),
            fault_server::<NS, F, 2>(endpoints.clone(), fault_registry),
        ),
        config_server::<NS, 2>(endpoints, runtime_config),
    )
    .await;
}

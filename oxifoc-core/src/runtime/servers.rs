//! Protocol servers that access state directly
//!
//! These servers handle ergot protocol requests by accessing the global
//! state module. No MotorRuntime trait needed.
//!
//! # Usage
//!
//! ```ignore
//! // In platform - single task wrapper
//! #[embassy_executor::task]
//! pub async fn protocol_servers() {
//!     let device_info = DeviceInfo { hw: "My Board", sw: "v0.1.0" };
//!     oxifoc_core::runtime::run_all_servers(
//!         STACK.endpoints(),
//!         device_info,
//!     ).await
//! }
//! ```

use core::pin::pin;

use embassy_futures::join::join4;
use ergot::net_stack::{NetStackHandle, endpoints::Endpoints};

use crate::foc::hall_sensor::Direction;
use crate::icd::{
    AdcSample, AdcSampleEndpoint, ControlMode, DeviceInfo, HallSensorData, HallSensorEndpoint,
    InfoEndpoint, MotorEndpoint,
};
use crate::state;

/// Device info server - responds to info requests from host
///
/// Returns hardware and software version information.
/// Also marks the communication link as active on first request.
pub async fn info_server<NS, const N: usize>(endpoints: Endpoints<NS>, device_info: DeviceInfo)
where
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
                state::set_link_active();
                async move { info }
            })
            .await;
    }
}

/// Hall sensor server - responds to Hall sensor data requests
///
/// Returns current Hall sensor state including angle, direction,
/// raw state, and error count.
pub async fn hall_sensor_server<NS, const N: usize>(endpoints: Endpoints<NS>)
where
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
                let snapshot = state::hall_snapshot();

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
pub async fn adc_sample_server<NS, const N: usize>(endpoints: Endpoints<NS>)
where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<AdcSampleEndpoint, N>(Some("adc"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_: &()| {
                let snapshot = state::adc_snapshot();
                async move { AdcSample::from_snapshot(&snapshot) }
            })
            .await;
    }
}

/// Motor control server - handles motor control mode changes
///
/// Sends ControlMode to CMD_CHANNEL for ISR processing.
/// Returns the current motor status.
pub async fn motor_command_server<NS, const N: usize>(endpoints: Endpoints<NS>)
where
    NS: NetStackHandle,
{
    let server = endpoints.bounded_server::<MotorEndpoint, N>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|mode: &ControlMode| {
                // Send control mode to the ISR via channel
                let _ = state::CMD_CHANNEL.try_send(*mode);

                // Return current status
                let status = state::motor_status();
                async move { status }
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
///
/// # Usage
///
/// ```ignore
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
///     ).await
/// }
/// ```
pub async fn run_all_servers<NS>(endpoints: Endpoints<NS>, device_info: DeviceInfo)
where
    NS: NetStackHandle + Clone,
{
    join4(
        info_server::<NS, 2>(endpoints.clone(), device_info),
        hall_sensor_server::<NS, 2>(endpoints.clone()),
        adc_sample_server::<NS, 2>(endpoints.clone()),
        motor_command_server::<NS, 2>(endpoints),
    )
    .await;
}

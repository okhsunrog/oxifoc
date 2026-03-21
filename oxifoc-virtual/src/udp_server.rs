//! Ergot UDP server — binds a UDP socket and runs protocol servers.
//!
//! Uses DirectEdge target profile. The device binds a port and waits for the
//! host to send the first packet (learns host address from recv_from).
//! No pre-configured host address needed.

use core::cell::RefCell;
use std::sync::Arc;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::interface_manager::profiles::direct_edge::tokio_udp::InterfaceKind;
use ergot::toolkits::tokio_stream::WaitQueue;
use ergot::toolkits::tokio_udp::{self as udp_kit, EdgeStack};
use heapless::String;
use tokio::net::UdpSocket;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::state::{MotorControlState, TELEMETRY};
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 512;

pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    max_current_a: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let bind_addr = format!("0.0.0.0:{port}");

    loop {
        let socket = UdpSocket::bind(&bind_addr).await?;
        info!("UDP target bound on {bind_addr}, waiting for host...");

        let queue = udp_kit::new_std_queue(4096);
        let stack: EdgeStack = udp_kit::new_target_stack(&queue, ERGOT_MTU);
        let state_notify = Arc::new(WaitQueue::new());

        udp_kit::register_edge_interface(
            &stack,
            socket,
            &queue,
            InterfaceKind::Target,
            Some(LivenessConfig { timeout_ms: 3000 }),
            Some(state_notify.clone()),
        )
        .await
        .map_err(|_| anyhow::anyhow!("UDP interface already active"))?;

        // Build device info
        let mut hw: String<32> = String::new();
        let mut sw: String<32> = String::new();
        let mut mcu: String<32> = String::new();
        let mut uuid: String<32> = String::new();
        let _ = hw.push_str("Virtual-BLDC");
        let _ = sw.push_str("oxifoc-virtual-0.1.0");
        let _ = mcu.push_str("x86_64 (virtual)");
        let _ = uuid.push_str("00000000-virtual");
        let device_info = DeviceInfo {
            hw,
            sw,
            mcu,
            uuid,
            foc_freq_hz,
            max_current_a,
        };

        // Wait for interface to become Active (host sent first packet)
        loop {
            let _ = state_notify.wait().await;
            let active = stack.manage_profile(|im| {
                matches!(im.interface_state(()), Some(InterfaceState::Active { .. }))
            });
            if active {
                info!("UDP host connected");
                break;
            }
        }

        // Cancel token — cancelled when interface goes down
        let conn_token = CancellationToken::new();

        // Monitor interface state — cancel all tasks when host disconnects
        tokio::spawn({
            let stack = stack.clone();
            let state_notify = state_notify.clone();
            let token = conn_token.clone();
            async move {
                loop {
                    let _ = state_notify.wait().await;
                    let state = stack.manage_profile(|im| im.interface_state(()));
                    if matches!(state, Some(InterfaceState::Down | InterfaceState::Inactive)) {
                        warn!("UDP host disconnected, stopping connection tasks");
                        token.cancel();
                        break;
                    }
                }
            }
        });

        // Spawn fast telemetry streaming
        tokio::spawn({
            let stack = stack.clone();
            let token = conn_token.clone();
            async move {
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = fast_telemetry_stream(stack, &TELEMETRY, state_mutex) => {}
                }
            }
        });

        // Run protocol servers until disconnect
        let endpoints = stack.endpoints();
        let token = conn_token.clone();
        tokio::select! {
            _ = token.cancelled() => {}
            _ = run_all_servers_with_config(
                endpoints,
                device_info,
                state_mutex,
                fault_registry,
                runtime_config,
                foc_freq_hz,
            ) => {}
        }

        info!("UDP session ended, ready for next connection");
    }
}

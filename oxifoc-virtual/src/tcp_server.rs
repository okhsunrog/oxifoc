//! Ergot TCP server — accepts a single host connection and runs protocol servers.
//!
//! Uses DirectEdge target profile (same topology as real embedded devices).
//! The host connects as controller (node 1), we are the target (node 2).

use core::cell::RefCell;
use std::sync::Arc;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::toolkits::tokio_stream::{
    self as stream_kit, EdgeStack, WaitQueue, register_target_stream,
};
use heapless::String;
use tokio::net::TcpListener;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 2048;

pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    max_current_a: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;

    info!("Listening on 0.0.0.0:{port}");
    loop {
        let (socket, addr) = listener.accept().await?;
        info!("Client connected: {addr}");

        // Create a fresh ergot edge stack for this connection.
        // We are the target (node 2), host is controller (node 1).
        let queue = stream_kit::new_std_queue(32768);
        let stack: EdgeStack = stream_kit::new_target_stack(&queue, ERGOT_MTU);

        let (rx, tx) = socket.into_split();
        let state_notify = Arc::new(WaitQueue::new());
        register_target_stream(
            stack.clone(),
            rx,
            tx,
            queue,
            Some(LivenessConfig {
                timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
            }),
            Some(state_notify.clone()),
        )
        .await
        .map_err(|_| anyhow::anyhow!("Interface already active"))?;

        // Cancel token for this connection — cancelled when interface goes down
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
                        warn!("Host disconnected, stopping connection tasks");
                        token.cancel();
                        break;
                    }
                }
            }
        });

        // Protocol servers for this connection
        tokio::spawn({
            let endpoints = stack.endpoints();
            let token = conn_token.clone();
            async move {
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

                tokio::select! {
                    _ = run_all_servers_with_config(
                        endpoints,
                        device_info,
                        state_mutex,
                        fault_registry,
                        runtime_config,
                        foc_freq_hz,
                    ) => {}
                    _ = token.cancelled() => {}
                }
            }
        });

        // Telemetry streaming tasks for this connection
        // Wait for Active state before broadcasting to avoid NoRouteToDest errors
        tokio::spawn({
            let stack = stack.clone();
            let state_notify = state_notify.clone();
            let token = conn_token.clone();
            async move {
                // Wait until interface is Active (has net_id from first incoming frame)
                let already_active = stack.manage_profile(|im| {
                    matches!(im.interface_state(()), Some(InterfaceState::Active { .. }))
                });
                if !already_active {
                    loop {
                        tokio::select! {
                            _ = token.cancelled() => return,
                            _ = state_notify.wait() => {
                                let active = stack.manage_profile(|im| {
                                    matches!(im.interface_state(()), Some(InterfaceState::Active { .. }))
                                });
                                if active { break; }
                            }
                        }
                    }
                }
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = fast_telemetry_stream(stack, foc_freq_hz) => {}
                }
            }
        });
        // Slow telemetry is now poll-based — served by slow_telemetry_server
        // inside run_all_servers_with_config. No separate task needed.
    }
}

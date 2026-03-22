//! Ergot UDP server — binds a UDP socket and runs protocol servers.
//!
//! Uses DirectEdge target profile. The device binds a port and waits for the
//! host to send the first packet (learns host address from recv_from).
//! No pre-configured host address needed.
//!
//! After disconnect, waits for interface to reach Down state (workers exited),
//! then re-registers on the same stack with the same socket.

use core::cell::RefCell;
use std::sync::Arc;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::interface_manager::profiles::direct_edge::tokio_udp::InterfaceKind;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
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
    let bind_addr = format!("0.0.0.0:{port}");
    info!("UDP target on {bind_addr}");

    let queue = udp_kit::new_std_queue(32768);
    let stack: EdgeStack = udp_kit::new_target_stack(&queue, ERGOT_MTU);
    let state_notify = Arc::new(WaitQueue::new());

    loop {
        // Bind a fresh socket each session (previous one is held by ergot's
        // Arc until workers exit). SO_REUSEADDR allows rebinding the port.
        let socket = {
            let sock = socket2::Socket::new(
                socket2::Domain::IPV4,
                socket2::Type::DGRAM,
                Some(socket2::Protocol::UDP),
            )?;
            sock.set_reuse_address(true)?;
            sock.set_nonblocking(true)?;
            sock.bind(&bind_addr.parse::<std::net::SocketAddr>()?.into())?;
            UdpSocket::from_std(sock.into())?
        };
        info!("Waiting for host...");

        udp_kit::register_edge_interface(
            &stack,
            socket,
            &queue,
            InterfaceKind::Target,
            Some(LivenessConfig {
                timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
            }),
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
        wait_for_state(&state_notify, &stack, |s| {
            matches!(s, Some(InterfaceState::Active { .. }))
        })
        .await;
        info!("UDP host connected");

        // Cancel token — cancelled when interface goes down
        let conn_token = CancellationToken::new();

        // Monitor interface state — cancel all tasks when host disconnects
        tokio::spawn({
            let stack = stack.clone();
            let state_notify = state_notify.clone();
            let token = conn_token.clone();
            async move {
                wait_for_state(&state_notify, &stack, |s| {
                    matches!(s, Some(InterfaceState::Down | InterfaceState::Inactive))
                })
                .await;
                warn!("UDP host disconnected, stopping connection tasks");
                token.cancel();
            }
        });

        // Spawn fast telemetry streaming
        tokio::spawn({
            let stack = stack.clone();
            let token = conn_token.clone();
            async move {
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = fast_telemetry_stream(stack, foc_freq_hz) => {}
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

        info!("UDP session ended, waiting for workers to exit...");

        // Wait for state to reach Down (workers fully exited) before re-registering
        wait_for_state(&state_notify, &stack, |s| {
            matches!(s, Some(InterfaceState::Down) | None)
        })
        .await;

        info!("Ready for next connection");
    }
}

/// Wait until the interface state matches a predicate.
async fn wait_for_state<F>(state_notify: &Arc<WaitQueue>, stack: &EdgeStack, predicate: F)
where
    F: Fn(Option<InterfaceState>) -> bool,
{
    // Check current state first
    let state = stack.manage_profile(|im| im.interface_state(()));
    if predicate(state) {
        return;
    }
    // Wait for state changes
    loop {
        let _ = state_notify.wait().await;
        let state = stack.manage_profile(|im| im.interface_state(()));
        if predicate(state) {
            return;
        }
    }
}

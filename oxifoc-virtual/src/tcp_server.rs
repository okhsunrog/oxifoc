//! Ergot TCP server — accepts a single host connection and runs protocol servers.
//!
//! Registers a `Router` profile (central, node 1) over the COBS stream, matching
//! the real device firmware: the host connects as an edge and the Router assigns
//! it a net_id, so its link-local frames are routed rather than dropped.

use core::cell::RefCell;
use std::sync::Arc;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::exports::mutex::raw_impls::cs::CriticalSectionRawMutex;
use ergot::interface_manager::interface_impls::tokio_stream::TokioStreamInterface;
use ergot::interface_manager::profiles::router::Router;
use ergot::interface_manager::transports::tokio_cobs_stream::register_router;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::net_stack::ArcNetStack;
use ergot::toolkits::tokio_stream::WaitQueue;
use heapless::String;
use rand::SeedableRng;
use rand::rngs::StdRng;
use tokio::net::TcpListener;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::HardwareInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 2048;

/// Per-connection Router sizing: one TCP client → one interface, no downstream
/// seed routes. Virtual emulates the device-side Router (central, node 1).
const ROUTER_SLOTS: usize = 2;
const ROUTER_SEEDS: usize = 0;
type RouterStack = ArcNetStack<
    CriticalSectionRawMutex,
    Router<TokioStreamInterface, StdRng, ROUTER_SLOTS, ROUTER_SEEDS>,
>;

pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    max_current_a: f32,
    vbus: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;

    info!("Listening on 0.0.0.0:{port}");
    loop {
        let (socket, addr) = listener.accept().await?;
        info!("Client connected: {addr}");

        // Fresh ergot Router stack for this connection. Virtual emulates the
        // device-side Router (central, node 1); the host connects as an edge and
        // the Router assigns it a net_id, so its link-local frames are routed
        // instead of dropped. `register_router` returns the interface `ident`
        // (used for state queries below) and only succeeds once Active.
        let stack: RouterStack =
            ArcNetStack::new_with_profile(Router::new(StdRng::seed_from_u64(0x0F0C_5EED)));

        let (rx, tx) = socket.into_split();
        let state_notify = Arc::new(WaitQueue::new());
        // No turbofish: M/SS (and CC on branches that have it) infer from the
        // `RouterStack` type alias, keeping this portable across ergot branches.
        let ident = register_router(
            stack.clone(),
            rx,
            tx,
            ERGOT_MTU,
            32768,
            Some(LivenessConfig {
                timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
            }),
            Some(state_notify.clone()),
        )
        .await
        .map_err(|_| anyhow::anyhow!("router interface registration failed"))?;

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
                    let state = stack.manage_profile(|im| im.interface_state(ident));
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
                let device_info = HardwareInfo {
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

        // Detect server for this connection
        tokio::spawn({
            let endpoints = stack.endpoints();
            let token = conn_token.clone();
            async move {
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = crate::detect::detect_server(endpoints, vbus, max_current_a, foc_freq_hz) => {}
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
                    matches!(
                        im.interface_state(ident),
                        Some(InterfaceState::Active { .. })
                    )
                });
                if !already_active {
                    loop {
                        tokio::select! {
                            _ = token.cancelled() => return,
                            _ = state_notify.wait() => {
                                let active = stack.manage_profile(|im| {
                                    matches!(im.interface_state(ident), Some(InterfaceState::Active { .. }))
                                });
                                if active { break; }
                            }
                        }
                    }
                }
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = fast_telemetry_stream::<_, { oxifoc_core::runtime::streaming::DEFAULT_BATCH_SIZE }, crate::TokioTimer>(stack, foc_freq_hz) => {}
                }
            }
        });
        // Slow telemetry is now poll-based — served by slow_telemetry_server
        // inside run_all_servers_with_config. No separate task needed.
    }
}

//! Ergot UDP server — persistent Router stack, one host session at a time.
//!
//! Registers a `Router` profile (central, node 1), matching the real device
//! firmware and the TCP server: the host reaches us as a link-local edge and
//! the Router assigns it a net_id, so its frames are routed rather than dropped
//! as spoofed (which happens if both sides are DirectEdge node-2 targets).
//!
//! The stack and its request/response endpoints are created ONCE at startup and
//! shared across every host session — exactly like real firmware. Each session
//! only binds a fresh socket and attaches a new interface; on a liveness timeout
//! the interface goes Down and we rebind. Persistent endpoints mean the host's
//! first request is always answered immediately (no per-session registration
//! race — see the TCP server for the full rationale).
//!
//! Unlike TCP there is no connection to accept: the socket is bound to a
//! well-known port and left *unconnected*. ergot's UDP `register_router` learns
//! the host's address from the first datagram it receives and replies with
//! `send_to`. After a liveness timeout the interface goes Down; we rebind a
//! fresh socket (SO_REUSEADDR) and wait for the next host.

use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use oxifoc_core::runtime::streaming::fault_topic_stream;
use oxifoc_core::virtual_motor::MotorParams;

use crate::detect::detect_server;
use core::cell::RefCell;
use std::sync::Arc;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::exports::mutex::raw_impls::cs::CriticalSectionRawMutex;
use ergot::interface_manager::interface_impls::tokio_udp::TokioUdpInterface;
use ergot::interface_manager::profiles::router::Router;
use ergot::interface_manager::transports::tokio_udp::register_router;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::net_stack::ArcNetStack;
use ergot::toolkits::tokio_stream::WaitQueue;
use heapless::String;
use rand::SeedableRng;
use rand::rngs::StdRng;
use tokio::net::UdpSocket;
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::HardwareInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::storage::RuntimeConfig;

use oxifoc_core::foc::fault::StandardFault;

const ERGOT_MTU: u16 = 2048;

/// Router sizing for the persistent stack: sessions are strictly sequential
/// (rebind waits for full teardown), so one live interface at a time — a few
/// slots give headroom and freed net_ids are recycled. No downstream seed routes.
const ROUTER_SLOTS: usize = 4;
const ROUTER_SEEDS: usize = 0;
type RouterStack = ArcNetStack<
    CriticalSectionRawMutex,
    Router<TokioUdpInterface, StdRng, ROUTER_SLOTS, ROUTER_SEEDS>,
>;

#[allow(clippy::too_many_arguments)]
pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    max_current_a: f32,
    vbus: f32,
    motor_params: MotorParams,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<StandardFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let bind_addr = format!("0.0.0.0:{port}");
    info!("UDP Router on {bind_addr}");

    // Persistent Router stack — created once, shared across all sessions.
    let stack: RouterStack =
        ArcNetStack::new_with_profile(Router::new(StdRng::seed_from_u64(0x0F0C_5EED)));

    // Device info — constant for the life of the process.
    let device_info = {
        let mut hw: String<32> = String::new();
        let mut sw: String<32> = String::new();
        let mut mcu: String<32> = String::new();
        let mut uuid: String<32> = String::new();
        let _ = hw.push_str("Virtual-BLDC");
        let _ = sw.push_str("oxifoc-virtual-0.1.0");
        let _ = mcu.push_str("x86_64 (virtual)");
        let _ = uuid.push_str("00000000-virtual");
        HardwareInfo {
            proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
            hw,
            sw,
            mcu,
            uuid,
            foc_freq_hz,
            max_current_a,
            calib: crate::VIRTUAL_CALIB,
        }
    };

    // Persistent request/response servers: bound once, always ready — the
    // host's first request is never racing endpoint registration.
    tokio::spawn({
        let endpoints = stack.endpoints();
        async move {
            // Box::pin: ~5 KB future; the 2 KB large_futures threshold is tuned
            // for firmware, on the host we just heap it.
            Box::pin(run_all_servers_with_config(
                endpoints,
                device_info,
                state_mutex,
                fault_registry,
                runtime_config,
                foc_freq_hz,
                max_current_a,
                true,
            ))
            .await;
        }
    });
    tokio::spawn({
        let endpoints = stack.endpoints();
        async move {
            detect_server(endpoints, vbus, max_current_a, foc_freq_hz, motor_params).await;
        }
    });

    let mut prev_conn_token: Option<CancellationToken> = None;
    loop {
        // Bind a fresh *unconnected* socket each session. SO_REUSEADDR lets us
        // rebind the same port after the previous session's workers exit.
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

        // Stop the previous session's telemetry tasks before attaching the new
        // interface (single-consumer fast-telemetry bbqueue — see TCP server).
        if let Some(prev) = prev_conn_token.take() {
            prev.cancel();
        }

        let state_notify = Arc::new(WaitQueue::new());
        // No turbofish: M/SS (and CC on branches that have it) infer from the
        // `RouterStack` type alias, keeping this portable across ergot branches.
        let ident = register_router(
            stack.clone(),
            socket,
            ERGOT_MTU,
            32768,
            Some(LivenessConfig {
                timeout_ms: LIVENESS_TIMEOUT_MS,
            }),
            Some(state_notify.clone()),
        )
        .await
        .map_err(|_| anyhow::anyhow!("UDP router interface registration failed"))?;

        // Cancel token for this session's telemetry.
        let conn_token = CancellationToken::new();
        prev_conn_token = Some(conn_token.clone());

        // Fast telemetry streaming. Broadcasts are gated by the host enabling
        // streaming; until the Router has learned the peer, frames simply queue
        // in the tx worker, so no Active-state wait is needed.
        tokio::spawn({
            let stack = stack.clone();
            let token = conn_token.clone();
            async move {
                tokio::select! {
                    _ = token.cancelled() => {}
                    _ = fast_telemetry_stream::<_, crate::TokioTimer>(stack.clone(), foc_freq_hz) => {}
                    _ = fault_topic_stream(stack, fault_registry) => {}
                }
            }
        });

        // Block until the host goes quiet: the interface is Active from
        // registration (net_id assigned), and on a liveness timeout ergot sets
        // it Down then *deregisters* it (state becomes None), so treat anything
        // no longer Active as a disconnect (matching Down specifically would
        // race the deregister).
        loop {
            let active = stack.manage_profile(|im| {
                matches!(
                    im.interface_state(ident),
                    Some(InterfaceState::Active { .. })
                )
            });
            if !active {
                break;
            }
            let _ = state_notify.wait().await;
        }
        warn!("Host disconnected, stopping session tasks");
        conn_token.cancel();

        info!("UDP session ended, waiting for workers to exit...");

        // Wait for the interface to fully tear down before rebinding the port.
        loop {
            let state = stack.manage_profile(|im| im.interface_state(ident));
            if matches!(state, Some(InterfaceState::Down) | None) {
                break;
            }
            let _ = state_notify.wait().await;
        }

        info!("Ready for next connection");
    }
}

//! Ergot TCP server — persistent Router stack, one host connection at a time.
//!
//! Registers a `Router` profile (central, node 1) over the COBS stream, matching
//! the real device firmware: the host connects as an edge and the Router assigns
//! it a net_id, so its link-local frames are routed rather than dropped.
//!
//! The stack and its request/response endpoints are created ONCE at startup and
//! shared across every client connection — exactly like real firmware, where the
//! stack and servers come up at boot and stay up. Each TCP connection only
//! attaches a fresh interface (and per-connection telemetry broadcasts); on
//! disconnect the interface deregisters and its net_id is recycled. The earlier
//! design rebuilt the stack and re-registered the endpoints per connection, so
//! on a reconnect the host's immediate HardwareInfo request raced the
//! per-connection endpoint registration and was dropped — forcing an 800 ms
//! handshake-retry. Persistent endpoints are always ready, so the first request
//! is answered immediately.

use oxifoc_core::icd::LIVENESS_TIMEOUT_MS;
use oxifoc_core::runtime::streaming::fault_topic_stream;
use oxifoc_core::virtual_motor::MotorParams;

use crate::detect::detect_server;
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

use oxifoc_core::foc::fault::StandardFault;

const ERGOT_MTU: u16 = 2048;

/// Router sizing for the persistent stack: at most one live client interface,
/// but reconnects can briefly overlap (the previous interface deregisters on
/// socket EOF / liveness timeout, which may lag the next `accept`). A handful of
/// slots gives that overlap headroom; freed net_ids are recycled by the Router,
/// so the pool never exhausts. No downstream seed routes.
const ROUTER_SLOTS: usize = 4;
const ROUTER_SEEDS: usize = 0;
type RouterStack = ArcNetStack<
    CriticalSectionRawMutex,
    Router<TokioStreamInterface, StdRng, ROUTER_SLOTS, ROUTER_SEEDS>,
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
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;
    info!("Listening on 0.0.0.0:{port}");

    // Persistent Router stack — created once, shared across all connections.
    // Virtual emulates the device-side Router (central, node 1); each host
    // connects as an edge and the Router assigns it a net_id.
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

    // Persistent request/response servers: bound once on the stack, always ready
    // to answer — independent of whether a client is currently attached. This is
    // the heart of the fix: the HardwareInfo (and every other) endpoint exists
    // before any frame is ever delivered, so the host's first request can never
    // race endpoint registration.
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

    let mut prev_conn: Option<(CancellationToken, u8)> = None;
    loop {
        let (socket, addr) = listener.accept().await?;
        info!("Client connected: {addr}");

        // Single-client server: cancel the previous telemetry tasks and remove
        // its interface immediately. Waiting for socket EOF/liveness lets open
        // clients accumulate until all Router slots are occupied; the fifth
        // registration used to terminate the whole virtual device.
        if let Some((token, ident)) = prev_conn.take() {
            token.cancel();
            let _ = stack.manage_profile(|router| router.deregister_interface(ident));
            tokio::task::yield_now().await;
        }

        // Attach this connection's COBS stream as a fresh Router interface. The
        // persistent endpoints above are already serving, so the host's first
        // request is delivered immediately. `register_router` returns the
        // interface `ident` and only succeeds once Active (net_id assigned).
        let (rx, tx) = socket.into_split();
        let state_notify = Arc::new(WaitQueue::new());
        // No turbofish: M/SS (and CC on branches that have it) infer from the
        // `RouterStack` type alias, keeping this portable across ergot branches.
        let Ok(ident) = register_router(
            stack.clone(),
            rx,
            tx,
            ERGOT_MTU,
            32768,
            Some(LivenessConfig {
                timeout_ms: LIVENESS_TIMEOUT_MS,
            }),
            Some(state_notify.clone()),
        )
        .await
        else {
            // A per-client resource/race failure must not take down the
            // listener and persistent endpoint tasks. Dropping the split
            // halves closes only this rejected socket.
            warn!("Router interface registration failed; rejecting client {addr}");
            continue;
        };

        // Cancel token for this connection's telemetry — cancelled when the
        // interface goes down (or when the next client connects, above).
        let conn_token = CancellationToken::new();
        prev_conn = Some((conn_token.clone(), ident));

        // Monitor interface state — cancel this connection's telemetry when the
        // host disconnects.
        tokio::spawn({
            let stack = stack.clone();
            let state_notify = state_notify.clone();
            let token = conn_token.clone();
            async move {
                loop {
                    // A COBS-stream disconnect deregisters the interface straight
                    // to `None` (not just Down/Inactive), so cancel on anything no
                    // longer Active. Matching only Down/Inactive would miss the
                    // deregister and leave the telemetry task orphaned — draining
                    // the queue and broadcasting into a dead interface until the
                    // next client connects (or forever, after the last one).
                    let active = matches!(
                        stack.manage_profile(|im| im.interface_state(ident)),
                        Some(InterfaceState::Active { .. })
                    );
                    if !active {
                        warn!("Host disconnected, stopping connection tasks");
                        token.cancel();
                        break;
                    }
                    let _ = state_notify.wait().await;
                }
            }
        });

        // Per-connection telemetry broadcasts (outbound; only while a client is
        // attached, so they don't spam NoRoute when idle). Slow telemetry is
        // poll-based, served by the persistent `slow_telemetry_server`.
        tokio::spawn({
            let stack = stack.clone();
            let state_notify = state_notify.clone();
            let token = conn_token.clone();
            async move {
                // Wait until the interface is Active (has its net_id) before
                // broadcasting, to avoid NoRouteToDest churn on the first frames.
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
                    _ = fast_telemetry_stream::<_, crate::TokioTimer>(stack.clone(), foc_freq_hz) => {}
                    _ = fault_topic_stream(stack, fault_registry) => {}
                }
            }
        });
    }
}

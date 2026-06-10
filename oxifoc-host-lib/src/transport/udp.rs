//! UDP transport for connecting to ergot devices over UDP.
//!
//! UDP is packet-framed (no COBS needed). Uses ergot's `tokio_udp` toolkit
//! directly. The stack is created once and survives reconnects; each
//! (re)connection binds a fresh socket and registers it onto the stack
//! (`register` only succeeds while the interface is Down, i.e. after a
//! teardown — same pattern as the COBS stream path).

use std::sync::Arc;

use anyhow::{Context, Result};
use ergot::interface_manager::LivenessConfig;
use ergot::interface_manager::utils::std::StdQueue;
use ergot::toolkits::tokio_stream::WaitQueue;
use ergot::toolkits::tokio_udp::{self, EdgeStack};
use tokio::net::UdpSocket;
use tracing::info;

pub fn new_stack(queue_size: usize, mtu: u16) -> (EdgeStack, StdQueue) {
    let queue = tokio_udp::new_std_queue(queue_size);
    let stack = tokio_udp::new_target_stack(&queue, mtu);
    (stack, queue)
}

pub async fn register(
    stack: &EdgeStack,
    queue: &StdQueue,
    host: &str,
    port: u16,
    state_notify: Option<Arc<WaitQueue>>,
) -> Result<()> {
    let addr = format!("{host}:{port}");
    info!("Connecting to UDP: {}", addr);

    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .context("Failed to bind UDP socket")?;
    socket
        .connect(&addr)
        .await
        .with_context(|| format!("Failed to connect UDP to {addr}"))?;

    tokio_udp::register_edge_target_interface(
        stack,
        socket,
        queue,
        Some(LivenessConfig {
            timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
        }),
        state_notify,
    )
    .await
    .map_err(|_| anyhow::anyhow!("UDP interface not in Down state"))?;

    info!("UDP connected to {}", addr);
    Ok(())
}

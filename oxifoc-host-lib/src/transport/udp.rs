//! UDP transport for connecting to ergot devices over UDP.
//!
//! UDP is packet-framed (no COBS needed). Uses ergot's `tokio_udp` toolkit directly.

use std::sync::Arc;

use anyhow::{Context, Result};
use ergot::interface_manager::LivenessConfig;
use ergot::toolkits::tokio_stream::WaitQueue;
use ergot::toolkits::tokio_udp::{self, EdgeStack};
use tokio::net::UdpSocket;
use tracing::info;

pub async fn connect(
    host: &str,
    port: u16,
    state_notify: Option<Arc<WaitQueue>>,
) -> Result<EdgeStack> {
    let addr = format!("{host}:{port}");
    info!("Connecting to UDP: {}", addr);

    let queue = tokio_udp::new_std_queue(32768);
    let stack: EdgeStack = tokio_udp::new_controller_stack(&queue, 2048);

    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .context("Failed to bind UDP socket")?;
    socket
        .connect(&addr)
        .await
        .with_context(|| format!("Failed to connect UDP to {addr}"))?;

    info!("UDP connected to {}", addr);

    tokio_udp::register_edge_controller_interface(
        &stack,
        socket,
        &queue,
        Some(LivenessConfig {
            timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
        }),
        state_notify,
    )
    .await
    .map_err(|_| anyhow::anyhow!("UDP interface already active"))?;

    Ok(stack)
}

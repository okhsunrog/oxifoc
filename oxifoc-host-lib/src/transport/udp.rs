//! UDP transport for connecting to ergot devices over UDP.
//!
//! UDP is packet-framed (no COBS needed). Uses ergot's `tokio_udp` toolkit directly.

use anyhow::{Context, Result};
use ergot::interface_manager::profiles::direct_edge::tokio_udp::InterfaceKind;
use ergot::toolkits::tokio_udp::{self, EdgeStack};
use tokio::net::UdpSocket;
use tracing::info;

pub async fn connect(host: &str, port: u16) -> Result<EdgeStack> {
    let addr = format!("{host}:{port}");
    info!("Connecting to UDP: {}", addr);

    let queue = tokio_udp::new_std_queue(4096);
    let stack: EdgeStack = tokio_udp::new_controller_stack(&queue, 512);

    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .context("Failed to bind UDP socket")?;
    socket
        .connect(&addr)
        .await
        .with_context(|| format!("Failed to connect UDP to {addr}"))?;

    info!("UDP connected to {}", addr);

    tokio_udp::register_edge_interface(&stack, socket, &queue, InterfaceKind::Controller)
        .await
        .map_err(|_| anyhow::anyhow!("UDP interface already active"))?;

    Ok(stack)
}

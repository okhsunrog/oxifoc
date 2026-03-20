//! TCP transport for connecting to oxifoc-virtual or any ergot device over TCP.

use anyhow::{Context, Result};
use tokio::net::TcpStream;
use tracing::info;

use super::Transport;

pub async fn connect(host: &str, port: u16) -> Result<Transport> {
    let addr = format!("{host}:{port}");
    info!("Connecting to TCP: {}", addr);
    let stream = TcpStream::connect(&addr)
        .await
        .with_context(|| format!("Failed to connect to {addr}"))?;

    info!("TCP connected to {}", addr);
    let (reader, writer) = stream.into_split();

    Ok(Transport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: None, // No defmt over TCP (virtual device uses tracing)
    })
}

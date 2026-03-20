//! TCP transport for connecting to oxifoc-virtual or any ergot device over TCP.

use anyhow::{Context, Result};
use tokio::net::TcpStream;
use tracing::info;

use super::CobsStreamTransport;

pub async fn connect(host: &str, port: u16) -> Result<CobsStreamTransport> {
    let addr = format!("{host}:{port}");
    info!("Connecting to TCP: {}", addr);
    let stream = TcpStream::connect(&addr)
        .await
        .with_context(|| format!("Failed to connect to {addr}"))?;

    info!("TCP connected to {}", addr);
    let (reader, writer) = stream.into_split();

    Ok(CobsStreamTransport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: None,
    })
}

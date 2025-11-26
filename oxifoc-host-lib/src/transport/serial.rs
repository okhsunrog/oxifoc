//! Serial (UART) transport implementation.

use super::Transport;
use anyhow::{Context, Result};
use tokio_serial::SerialPortBuilderExt;
use tracing::info;

/// Connect to a serial port.
pub async fn connect(path: &str, baud: u32) -> Result<Transport> {
    info!("Opening serial port {} at {} baud", path, baud);

    let port = tokio_serial::new(path, baud)
        .open_native_async()
        .with_context(|| format!("Failed to open serial port {}", path))?;

    let (reader, writer) = tokio::io::split(port);

    Ok(Transport {
        reader: Box::new(reader),
        writer: Box::new(writer),
    })
}

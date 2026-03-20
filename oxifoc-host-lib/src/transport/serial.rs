//! Serial (UART) transport implementation.
//!
//! In serial mode, defmt frames are forwarded over the ergot network,
//! so there's no separate defmt reader.

use super::CobsStreamTransport;
use anyhow::{Context, Result};
use tokio_serial::SerialPortBuilderExt;
use tracing::info;

/// Connect to a serial port.
pub async fn connect(path: &str, baud: u32) -> Result<CobsStreamTransport> {
    info!("Opening serial port {} at {} baud", path, baud);

    let port = tokio_serial::new(path, baud)
        .open_native_async()
        .with_context(|| format!("Failed to open serial port {}", path))?;

    let (reader, writer) = tokio::io::split(port);

    Ok(CobsStreamTransport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: None,
    })
}

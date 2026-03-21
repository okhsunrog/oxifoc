//! Serial (UART) transport implementation.
//!
//! In serial mode, defmt frames are forwarded over the ergot network,
//! so there's no separate defmt reader.

use super::CobsStreamTransport;
use anyhow::{Context, Result};
use tokio_serial::{SerialPort, SerialPortBuilderExt};
use tracing::{info, warn};

/// Connect to a serial port.
pub async fn connect(path: &str, baud: u32) -> Result<CobsStreamTransport> {
    info!("Opening serial port {} at {} baud", path, baud);

    let mut port = tokio_serial::new(path, baud)
        .open_native_async()
        .with_context(|| format!("Failed to open serial port {}", path))?;

    if let Err(e) = port.clear(tokio_serial::ClearBuffer::All) {
        warn!("Failed to clear serial buffers: {:?}", e);
    }
    // Send a COBS frame boundary so the device can flush any stale
    // partial frame from a previous session and sync cleanly.
    let _ = std::io::Write::write_all(&mut port, &[0]);

    let (reader, writer) = tokio::io::split(port);

    Ok(CobsStreamTransport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: None,
    })
}

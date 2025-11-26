//! RTT transport implementation via probe-rs.
//!
//! This module provides RTT (Real-Time Transfer) communication with the target
//! device through a debug probe (ST-Link, J-Link, etc.) using probe-rs.

use super::Transport;
use anyhow::{Context, Result, anyhow};
use probe_rs::{
    Permissions,
    probe::list::Lister,
    rtt::{Rtt, ScanRegion},
};
use std::io;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context as TaskContext, Poll};
use std::time::Duration;
use tokio::io::{AsyncRead, AsyncWrite, ReadBuf};
use tracing::{debug, info};

/// RTT channel indices (must match device firmware)
const RTT_UP_CHANNEL_DEFMT: usize = 0; // Device -> Host (defmt logs)
const RTT_UP_CHANNEL_ERGOT: usize = 1; // Device -> Host (ergot data)
const RTT_DOWN_CHANNEL_ERGOT: usize = 0; // Host -> Device (ergot data)

/// Shared RTT session state.
/// We store the raw Session pointer because probe-rs's Core borrows from Session,
/// making it difficult to store both together. This uses an owning approach instead.
struct RttState {
    session: probe_rs::Session,
}

impl RttState {
    /// Execute an operation with the RTT context (Core + RTT attached).
    fn with_rtt<F, T>(&mut self, channel_up: usize, f: F) -> io::Result<T>
    where
        F: FnOnce(&mut probe_rs::rtt::UpChannel, &mut probe_rs::Core) -> io::Result<T>,
    {
        let mut core = self
            .session
            .core(0)
            .map_err(|e| io::Error::other(format!("Failed to get core: {}", e)))?;

        // Re-attach to RTT each time (this is inefficient but avoids borrow issues)
        let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Ram)
            .map_err(|e| io::Error::other(format!("Failed to attach RTT: {}", e)))?;

        let channel = rtt.up_channel(channel_up).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("RTT up channel {} not found", channel_up),
            )
        })?;

        f(channel, &mut core)
    }

    fn with_rtt_down<F, T>(&mut self, channel_down: usize, f: F) -> io::Result<T>
    where
        F: FnOnce(&mut probe_rs::rtt::DownChannel, &mut probe_rs::Core) -> io::Result<T>,
    {
        let mut core = self
            .session
            .core(0)
            .map_err(|e| io::Error::other(format!("Failed to get core: {}", e)))?;

        let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Ram)
            .map_err(|e| io::Error::other(format!("Failed to attach RTT: {}", e)))?;

        let channel = rtt.down_channel(channel_down).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("RTT down channel {} not found", channel_down),
            )
        })?;

        f(channel, &mut core)
    }
}

/// Async reader wrapper for RTT up channel.
pub struct RttReader {
    state: Arc<Mutex<RttState>>,
    channel_idx: usize,
}

impl AsyncRead for RttReader {
    fn poll_read(
        self: Pin<&mut Self>,
        cx: &mut TaskContext<'_>,
        buf: &mut ReadBuf<'_>,
    ) -> Poll<io::Result<()>> {
        let mut state = self.state.lock().unwrap();
        let channel_idx = self.channel_idx;

        let result = state.with_rtt(channel_idx, |channel, core| {
            let unfilled = buf.initialize_unfilled();
            channel
                .read(core, unfilled)
                .map_err(|e| io::Error::other(e.to_string()))
        });

        match result {
            Ok(0) => {
                // No data available, wake up later
                let waker = cx.waker().clone();
                tokio::spawn(async move {
                    tokio::time::sleep(Duration::from_millis(1)).await;
                    waker.wake();
                });
                Poll::Pending
            }
            Ok(n) => {
                buf.advance(n);
                Poll::Ready(Ok(()))
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

/// Async writer wrapper for RTT down channel.
pub struct RttWriter {
    state: Arc<Mutex<RttState>>,
    channel_idx: usize,
}

impl AsyncWrite for RttWriter {
    fn poll_write(
        self: Pin<&mut Self>,
        _cx: &mut TaskContext<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        let mut state = self.state.lock().unwrap();
        let channel_idx = self.channel_idx;

        let result = state.with_rtt_down(channel_idx, |channel, core| {
            channel
                .write(core, buf)
                .map_err(|e| io::Error::other(e.to_string()))
        });

        Poll::Ready(result)
    }

    fn poll_flush(self: Pin<&mut Self>, _cx: &mut TaskContext<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }

    fn poll_shutdown(self: Pin<&mut Self>, _cx: &mut TaskContext<'_>) -> Poll<io::Result<()>> {
        Poll::Ready(Ok(()))
    }
}

/// Connect to a target via RTT through a debug probe.
pub async fn connect(probe_selector: Option<&str>, chip: &str) -> Result<Transport> {
    info!("Connecting via RTT to chip: {}", chip);

    let lister = Lister::new();

    // Find and open the probe
    let probe = if let Some(selector) = probe_selector {
        info!("Using probe selector: {}", selector);
        let probes = lister.list_all();
        debug!("Available probes: {:?}", probes);

        // Parse selector (format: "VID:PID" or "VID:PID:SERIAL")
        let parts: Vec<&str> = selector.split(':').collect();
        let (vid, pid, serial) = match parts.len() {
            2 => {
                let vid = u16::from_str_radix(parts[0], 16)
                    .with_context(|| format!("Invalid VID: {}", parts[0]))?;
                let pid = u16::from_str_radix(parts[1], 16)
                    .with_context(|| format!("Invalid PID: {}", parts[1]))?;
                (vid, pid, None)
            }
            3 => {
                let vid = u16::from_str_radix(parts[0], 16)
                    .with_context(|| format!("Invalid VID: {}", parts[0]))?;
                let pid = u16::from_str_radix(parts[1], 16)
                    .with_context(|| format!("Invalid PID: {}", parts[1]))?;
                (vid, pid, Some(parts[2]))
            }
            _ => {
                return Err(anyhow!(
                    "Invalid probe selector format. Use 'VID:PID' or 'VID:PID:SERIAL'"
                ));
            }
        };

        let probe_info = probes
            .into_iter()
            .find(|p| {
                p.vendor_id == vid
                    && p.product_id == pid
                    && (serial.is_none() || p.serial_number.as_deref() == serial)
            })
            .ok_or_else(|| anyhow!("No matching probe found for selector: {}", selector))?;

        probe_info.open().context("Failed to open probe")?
    } else {
        // Auto-detect first available probe
        let probes = lister.list_all();
        if probes.is_empty() {
            return Err(anyhow!("No debug probes found"));
        }
        info!("Auto-selecting first probe: {:?}", probes[0]);
        probes[0].open().context("Failed to open probe")?
    };

    // Attach to the target
    let mut session = probe
        .attach(chip, Permissions::default())
        .context("Failed to attach to target")?;

    info!("Attached to target, scanning for RTT...");

    // Get the core and scan for RTT to verify channels exist
    {
        let mut core = session.core(0).context("Failed to get core 0")?;
        let mut rtt =
            Rtt::attach_region(&mut core, &ScanRegion::Ram).context("Failed to attach to RTT")?;

        info!("RTT attached successfully");

        // Log available channels
        for (idx, channel) in rtt.up_channels().iter().enumerate() {
            info!(
                "RTT up channel {}: {} (size: {})",
                idx,
                channel.name().unwrap_or("unnamed"),
                channel.buffer_size()
            );
        }
        for (idx, channel) in rtt.down_channels().iter().enumerate() {
            info!(
                "RTT down channel {}: {} (size: {})",
                idx,
                channel.name().unwrap_or("unnamed"),
                channel.buffer_size()
            );
        }

        // Verify required channels exist
        if rtt.up_channel(RTT_UP_CHANNEL_DEFMT).is_none() {
            return Err(anyhow!(
                "RTT up channel {} (defmt) not found. Device may not be running or RTT not initialized.",
                RTT_UP_CHANNEL_DEFMT
            ));
        }
        if rtt.up_channel(RTT_UP_CHANNEL_ERGOT).is_none() {
            return Err(anyhow!(
                "RTT up channel {} (ergot) not found. Device may not be running or RTT not initialized.",
                RTT_UP_CHANNEL_ERGOT
            ));
        }
        if rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT).is_none() {
            return Err(anyhow!(
                "RTT down channel {} (ergot) not found. Device may not be running or RTT not initialized.",
                RTT_DOWN_CHANNEL_ERGOT
            ));
        }
    }

    let state = Arc::new(Mutex::new(RttState { session }));

    // Ergot data reader (channel 1)
    let reader = RttReader {
        state: state.clone(),
        channel_idx: RTT_UP_CHANNEL_ERGOT,
    };

    // Ergot data writer (down channel 0)
    let writer = RttWriter {
        state: state.clone(),
        channel_idx: RTT_DOWN_CHANNEL_ERGOT,
    };

    // Defmt reader (channel 0)
    let defmt_reader = RttReader {
        state,
        channel_idx: RTT_UP_CHANNEL_DEFMT,
    };

    Ok(Transport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: Some(Box::new(defmt_reader)),
    })
}

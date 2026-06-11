//! RTT transport implementation via probe-rs.
//!
//! probe-rs RTT is synchronous and Core/Rtt refs aren't Send, so all RTT I/O
//! runs in a dedicated blocking thread. Channels bridge to the async world via
//! StreamReader (AsyncRead) and SinkWriter+CopyToBytes+PollSender (AsyncWrite).

use super::CobsStreamTransport;
use anyhow::{Context, Result, anyhow};
use bytes::Bytes;
use futures::SinkExt;
use probe_rs::{
    Permissions,
    probe::list::Lister,
    rtt::{Rtt, ScanRegion},
};
use std::io;
use std::time::Duration;
use tokio_stream::wrappers::ReceiverStream;
use tokio_util::io::{CopyToBytes, SinkWriter, StreamReader};
use tokio_util::sync::PollSender;
use tracing::{debug, error, info};

/// RTT channel indices (must match device firmware)
const RTT_UP_CHANNEL_DEFMT: usize = 0; // Device -> Host (defmt logs)
const RTT_UP_CHANNEL_ERGOT: usize = 1; // Device -> Host (ergot data)
const RTT_DOWN_CHANNEL_ERGOT: usize = 0; // Host -> Device (ergot data)

/// Connect to a target via RTT through a debug probe.
///
/// Spawns a dedicated blocking thread that owns the probe-rs Session/Core/Rtt
/// and communicates with the async world via channels.
#[allow(clippy::single_range_in_vec_init)]
pub async fn connect(probe_selector: Option<&str>, chip: &str) -> Result<CobsStreamTransport> {
    info!("Connecting via RTT to chip: {}", chip);

    let lister = Lister::new();

    // Find and open the probe
    let probe = if let Some(selector) = probe_selector {
        info!("Using probe selector: {}", selector);
        let probes = lister.list_all();
        debug!("Available probes: {:?}", probes);

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
        let probes = lister.list_all();
        if probes.is_empty() {
            return Err(anyhow!("No debug probes found"));
        }
        info!("Auto-selecting first probe: {:?}", probes[0]);
        probes[0].open().context("Failed to open probe")?
    };

    // Attach to the target and verify RTT channels
    let mut session = probe
        .attach(chip, Permissions::default())
        .context("Failed to attach to target")?;

    info!("Attached to target, resetting device...");
    {
        let mut core = session.core(0).context("Failed to get core 0")?;
        core.reset().context("Failed to reset target")?;
    }

    info!("Scanning for RTT...");

    {
        let mut core = session.core(0).context("Failed to get core 0")?;
        // Scan the chip's whole RAM (per probe-rs target description) — the
        // old hardcoded 0x20000000..0x20008000 (32 KB) missed control blocks
        // on parts with more RAM (F405: 128K+CCM, G474: 128K).
        let mut rtt =
            Rtt::attach_region(&mut core, &ScanRegion::Ram).context("Failed to attach to RTT")?;

        info!("RTT attached successfully");

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

        if rtt.up_channel(RTT_UP_CHANNEL_DEFMT).is_none() {
            return Err(anyhow!(
                "RTT up channel {} (defmt) not found",
                RTT_UP_CHANNEL_DEFMT
            ));
        }
        if rtt.up_channel(RTT_UP_CHANNEL_ERGOT).is_none() {
            return Err(anyhow!(
                "RTT up channel {} (ergot) not found",
                RTT_UP_CHANNEL_ERGOT
            ));
        }
        let down_ch = rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT).ok_or_else(|| {
            anyhow!(
                "RTT down channel {} (ergot) not found",
                RTT_DOWN_CHANNEL_ERGOT
            )
        })?;

        // Send a COBS frame boundary so the device can flush any stale
        // partial frame from a previous session and sync cleanly.
        let _ = down_ch.write(&mut core, &[0]);
    }

    // Set up channels between blocking RTT thread and async world.
    //
    // Ergot RX (device→host): blocking_send → StreamReader (AsyncRead)
    // Ergot TX (host→device): SinkWriter (AsyncWrite) → try_recv
    // Defmt RX (device→host): blocking_send → StreamReader (AsyncRead)
    let (ergot_rx_tx, ergot_rx_rx) = tokio::sync::mpsc::channel::<io::Result<Bytes>>(64);
    let (ergot_tx_tx, mut ergot_tx_rx) = tokio::sync::mpsc::channel::<Bytes>(64);
    let (defmt_rx_tx, defmt_rx_rx) = tokio::sync::mpsc::channel::<io::Result<Bytes>>(64);

    // Blocking RTT I/O thread — owns Session, Core, Rtt; never re-attaches.
    // Failures must surface as a broken link (io::Error through the reader
    // channel), not a silent thread death: the old expect()s killed the
    // link with no signal to the reconnect logic.
    std::thread::spawn(move || {
        let fail = |ergot_rx_tx: &tokio::sync::mpsc::Sender<io::Result<Bytes>>, msg: String| {
            error!("{msg}");
            let _ = ergot_rx_tx.blocking_send(Err(io::Error::other(msg)));
        };
        let mut core = match session.core(0) {
            Ok(c) => c,
            Err(e) => {
                fail(
                    &ergot_rx_tx,
                    format!("RTT thread: failed to get core 0: {e}"),
                );
                return;
            }
        };
        let mut rtt = match Rtt::attach_region(&mut core, &ScanRegion::Ram) {
            Ok(r) => r,
            Err(e) => {
                fail(&ergot_rx_tx, format!("RTT thread: failed to attach: {e}"));
                return;
            }
        };

        let mut ergot_rx_buf = [0u8; 2048];
        let mut defmt_buf = [0u8; 4096];

        loop {
            let mut did_work = false;

            // 1. Read ergot data from device
            if let Some(channel) = rtt.up_channel(RTT_UP_CHANNEL_ERGOT) {
                match channel.read(&mut core, &mut ergot_rx_buf) {
                    Ok(n) if n > 0 => {
                        did_work = true;
                        if ergot_rx_tx
                            .blocking_send(Ok(Bytes::copy_from_slice(&ergot_rx_buf[..n])))
                            .is_err()
                        {
                            info!("Ergot rx channel closed, stopping RTT thread");
                            return;
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        fail(
                            &ergot_rx_tx,
                            format!("RTT ergot read error (probe gone?): {e}"),
                        );
                        return;
                    }
                }
            }

            // 2. Write ergot data to device
            while let Ok(data) = ergot_tx_rx.try_recv() {
                if let Some(channel) = rtt.down_channel(RTT_DOWN_CHANNEL_ERGOT) {
                    let mut offset = 0;
                    while offset < data.len() {
                        match channel.write(&mut core, &data[offset..]) {
                            Ok(n) => offset += n,
                            Err(e) => {
                                fail(
                                    &ergot_rx_tx,
                                    format!("RTT ergot write error (probe gone?): {e}"),
                                );
                                return;
                            }
                        }
                    }
                    did_work = true;
                }
            }

            // 3. Read defmt data from device
            if let Some(channel) = rtt.up_channel(RTT_UP_CHANNEL_DEFMT) {
                match channel.read(&mut core, &mut defmt_buf) {
                    Ok(n) if n > 0 => {
                        did_work = true;
                        if defmt_rx_tx
                            .blocking_send(Ok(Bytes::copy_from_slice(&defmt_buf[..n])))
                            .is_err()
                        {
                            info!("Defmt rx channel closed, stopping RTT thread");
                            return;
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        fail(
                            &ergot_rx_tx,
                            format!("RTT defmt read error (probe gone?): {e}"),
                        );
                        return;
                    }
                }
            }

            if !did_work {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
    });

    // Build AsyncRead/AsyncWrite from channels
    let reader = StreamReader::new(ReceiverStream::new(ergot_rx_rx));
    let writer = SinkWriter::new(CopyToBytes::new(
        PollSender::new(ergot_tx_tx).sink_map_err(|e| io::Error::new(io::ErrorKind::BrokenPipe, e)),
    ));
    let defmt_reader = StreamReader::new(ReceiverStream::new(defmt_rx_rx));

    Ok(CobsStreamTransport {
        reader: Box::new(reader),
        writer: Box::new(writer),
        defmt_reader: Some(Box::new(defmt_reader)),
    })
}

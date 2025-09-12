use anyhow::{Context, Result};
use log::{info, error};
use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::{Rtt, ScanRegion};
use std::time::Duration;
use oxifoc_protocol::{ButtonEvent, ButtonEndpoint};
use ergot::well_known::ErgotFmtRxOwned;

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();

    info!("Oxifoc Host - Ergot over RTT");
    info!("Connecting to STM32G431 via ST-Link...");

    // Get list of available probes
    let lister = Lister::new();
    let probes = lister.list_all();

    if probes.is_empty() {
        error!("No debug probes found! Make sure ST-Link is connected.");
        return Err(anyhow::anyhow!("No probes found"));
    }

    info!("Found {} probe(s)", probes.len());

    // Open the first available probe
    let probe = probes[0].open().context("Failed to open probe")?;

    // Attach to the target
    let mut session = probe
        .attach("STM32G431CBUx", Permissions::default())
        .context("Failed to attach to target")?;

    info!("Successfully attached to STM32G431");

    // Get the core
    let mut core = session.core(0)?;

    // Set up RTT - scan entire RAM
    let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Ram)
        .context("Failed to attach RTT")?;

    info!("RTT attached successfully");
    info!("Available RTT channels:");
    for (idx, channel) in rtt.up_channels().iter().enumerate() {
        info!("  Channel {}: {}", idx, channel.name().unwrap_or("unnamed"));
    }

    // Main loop - read from both channels
    let mut defmt_buf = vec![0u8; 1024];
    let mut ergot_buf = vec![0u8; 2048];

    loop {
        // Read defmt channel (channel 0) for debugging
        if let Some(channel) = rtt.up_channels().get_mut(0) {
            let count = channel.read(&mut core, &mut defmt_buf)?;
            if count > 0 {
                let text = String::from_utf8_lossy(&defmt_buf[..count]);
                if !text.is_empty() {
                    print!("[DEFMT] {}", text);
                }
            }
        }

        // Read ergot channel (channel 1)
        if let Some(channel) = rtt.up_channels().get_mut(1) {
            let count = channel.read(&mut core, &mut ergot_buf)?;
            if count > 0 {
                // Process ergot frames
                process_ergot_data(&ergot_buf[..count]);
            }
        }

        // Small delay to avoid overwhelming the probe
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn process_ergot_data(data: &[u8]) {
    // Try to decode as COBS-framed ergot packets
    // For now, just show raw data for debugging
    if !data.is_empty() {
        info!("Received ergot data: {} bytes", data.len());

        // Attempt to find COBS frames (frames end with 0x00)
        let mut pos = 0;
        while pos < data.len() {
            if let Some(frame_end) = data[pos..].iter().position(|&b| b == 0x00) {
                let frame = &data[pos..pos + frame_end];
                if !frame.is_empty() {
                    decode_ergot_frame(frame);
                }
                pos += frame_end + 1;
            } else {
                break;
            }
        }
    }
}

fn decode_ergot_frame(frame: &[u8]) {
    // Decode COBS
    let mut decoded = vec![0u8; frame.len()];
    match cobs::decode(frame, &mut decoded) {
        Ok(len) => {
            let decoded = &decoded[..len];

            // Try to decode as ergot log message
            if let Ok(log_msg) = postcard::from_bytes::<ErgotFmtRxOwned>(decoded) {
                log::info!(
                    target: "device_log",
                    "[{:?}] {}",
                    log_msg.level,
                    log_msg.inner
                );
                return;
            }

            // Try to decode as button event
            // Note: This would be in ergot endpoint format, more complex to decode
            // For now, just show we got a frame
            info!("Received ergot frame: {} bytes", decoded.len());
        }
        Err(e) => {
            error!("COBS decode error: {:?}", e);
        }
    }
}

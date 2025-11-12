use anyhow::{Context, Result};
use log::{info, error};
use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::{Rtt, ScanRegion};
use std::time::Duration;
use oxifoc_protocol::{ButtonEvent, ButtonEndpoint};
use ergot::logging::fmtlog::ErgotFmtRxOwned;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum LogSourceSel { Auto, Ergot, Defmt }

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();

    // Parse CLI flags: --log-source {auto|ergot|defmt} and optional --chip <name>
    let mut sel = LogSourceSel::Auto;
    let mut chip: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--log-source" {
            if let Some(val) = args.next() {
                sel = match val.as_str() {
                    "ergot" => LogSourceSel::Ergot,
                    "defmt" => LogSourceSel::Defmt,
                    _ => LogSourceSel::Auto,
                };
            }
        } else if arg == "--chip" {
            chip = args.next();
        }
    }

    info!("Oxifoc Host - Ergot over RTT (log-source={:?}, chip={:?})", sel, chip);
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

    // Attach to the target (auto-detect by default, or explicit --chip)
    let ts = match chip {
        Some(name) => probe_rs::config::TargetSelector::from(name),
        None => probe_rs::config::TargetSelector::Auto,
    };
    let mut session = probe
        .attach(ts, Permissions::default())
        .context("Failed to attach to target")?;

    info!("Successfully attached to STM32G431");

    // Get the core
    let mut core = session.core(0)?;

    // Set up RTT - scan entire RAM
    let mut rtt = Rtt::attach_region(&mut core, &ScanRegion::Ram)
        .context("Failed to attach RTT")?;

    info!("RTT attached successfully");
    info!("Available RTT up channels:");
    for (idx, channel) in rtt.up_channels().iter().enumerate() {
        info!("  up{}: {}", idx, channel.name().unwrap_or("unnamed"));
    }
    info!("Available RTT down channels:");
    for (idx, channel) in rtt.down_channels().iter().enumerate() {
        info!("  down{}: {}", idx, channel.name().unwrap_or("unnamed"));
    }

    // Decide the log source channel index
    let choose_log_source = |rtt: &mut Rtt, sel: LogSourceSel| -> (LogSourceSel, Option<usize>) {
        // helper: find channel by name (exact)
        let mut find_by_name = |name: &str| -> Option<usize> {
            rtt.up_channels()
                .iter()
                .enumerate()
                .find_map(|(i, ch)| {
                    if ch.name().map(|n| n == name).unwrap_or(false) { Some(i) } else { None }
                })
        };
        match sel {
            LogSourceSel::Ergot => (LogSourceSel::Ergot, find_by_name("ergot").or(Some(1))),
            LogSourceSel::Defmt => (LogSourceSel::Defmt, find_by_name("defmt").or(Some(0))),
            LogSourceSel::Auto => {
                if let Some(i) = find_by_name("ergot") { return (LogSourceSel::Ergot, Some(i)); }
                if let Some(i) = find_by_name("defmt") { return (LogSourceSel::Defmt, Some(i)); }
                // fallback to ergot on up1
                (LogSourceSel::Ergot, Some(1))
            }
        }
    };

    let (sel, log_up_idx) = choose_log_source(&mut rtt, sel);
    info!("Selected log source: {:?} on up{}", sel, log_up_idx.unwrap_or(usize::MAX));

    // Main loop - read from the selected source
    let mut buf = vec![0u8; 2048];
    loop {
        if let Some(up_idx) = log_up_idx {
            if let Some(channel) = rtt.up_channels().get_mut(up_idx) {
                let count = channel.read(&mut core, &mut buf)?;
                if count > 0 {
                    match sel {
                        LogSourceSel::Ergot => process_ergot_data(&buf[..count]),
                        LogSourceSel::Defmt => {
                            // TODO(defmt): integrate defmt-decoder with an --defmt-elf <path>.
                            // For now, stream raw bytes for debugging convenience.
                            let text = String::from_utf8_lossy(&buf[..count]);
                            if !text.is_empty() { print!("[DEFMT] {}", text); }
                        }
                        LogSourceSel::Auto => {}
                    }
                }
            }
        }
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

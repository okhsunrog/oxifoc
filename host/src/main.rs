use anyhow::{Context, Result};
use tracing::{info, error};
use probe_rs::probe::list::Lister;
use probe_rs::Permissions;
use probe_rs::rtt::{Rtt, ScanRegion};
use std::time::Duration;
// ergot stack and helpers
use defmt_decoder::{Table, DecodeError, StreamDecoder};
use std::fs;
use ergot::interface_manager::utils::std::new_std_queue;
use cobs_acc::{CobsAccumulator, FeedResult};
use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
use ergot::interface_manager::utils::std::StdQueue as ErgotStdQueue;
use ergot::net_stack::ArcNetStack;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use ergot::interface_manager::{InterfaceState, Interface};
use oxifoc_protocol::{ButtonEndpoint, ButtonEvent};
use core::pin::pin;

mod config;
use config::HostConfig;

fn init_tracing() {
    // Default INFO; allow override via RUST_LOG
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    // Do not install a log tracer here to avoid SetLoggerError; rely on tracing only.
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(true)
        .with_level(true)
        .compact()
        .try_init();
}

#[tokio::main]
async fn main() -> Result<()> {
    init_tracing();

    // Load config file
    let cfg = HostConfig::load_default().unwrap_or_default();
    let probe_sel = cfg.probe.clone();
    let chip = cfg.chip.clone();
    let elf_from_cfg = cfg.elf.clone();

    info!("Oxifoc Host - RTT (chip={:?}, probe={:?})", chip, probe_sel);
    info!("Connecting to STM32G431 via ST-Link...");

    // Get list of available probes
    let lister = Lister::new();
    let probes = lister.list_all();

    if probes.is_empty() {
        error!("No debug probes found! Make sure ST-Link is connected.");
        return Err(anyhow::anyhow!("No probes found"));
    }

    info!("Found {} probe(s)", probes.len());

    // Open specific probe if configured, otherwise first
    let probe = if let Some(sel) = probe_sel {
        let mut parts = sel.split(':');
        let vid = parts.next();
        let pid = parts.next();
        let serial = parts.next();
        let chosen = probes.iter().find(|p| {
            let ok_vid = vid.and_then(|v| u16::from_str_radix(v, 16).ok())
                .map(|v| p.vendor_id == v).unwrap_or(true);
            let ok_pid = pid.and_then(|v| u16::from_str_radix(v, 16).ok())
                .map(|v| p.product_id == v).unwrap_or(true);
            let ok_ser = serial.map(|s| p.serial_number.as_deref() == Some(s)).unwrap_or(true);
            ok_vid && ok_pid && ok_ser
        }).ok_or_else(|| anyhow::anyhow!("Configured probe not found: {}", sel))?;
        chosen.open().context("Failed to open selected probe")?
    } else {
        probes[0].open().context("Failed to open probe")?
    };

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

    // Find well-known channels by name
    let mut find_by_name = |name: &str| -> Option<usize> {
        rtt.up_channels()
            .iter()
            .enumerate()
            .find_map(|(i, ch)| {
                if ch.name().map(|n| n == name).unwrap_or(false) { Some(i) } else { None }
            })
    };
    let ergot_up_idx = if cfg.stream_ergot() { find_by_name("ergot").or(Some(1)) } else { None };
    let defmt_up_idx = if cfg.stream_defmt() { find_by_name("defmt").or(Some(0)) } else { None };
    info!("Using channels: ergot={:?}, defmt={:?}", ergot_up_idx, defmt_up_idx);

    // Build an ergot DirectEdge stack in controller mode (not router - we're directly connected to one device)
    use ergot::interface_manager::profiles::direct_edge::DirectEdge;
    struct RttInterface;
    impl Interface for RttInterface { type Sink = ErgotSink<ErgotStdQueue>; }
    type EdgeProfile = DirectEdge<RttInterface>;
    type EdgeStack = ArcNetStack<CriticalSectionRawMutex, EdgeProfile>;
    const ERGOT_MTU: u16 = 1024;
    let queue = new_std_queue(4096);

    // Create stack with DirectEdge in controller mode (network 1, node 1)
    let stack: EdgeStack = ArcNetStack::new_with_profile(
        DirectEdge::new_controller(
            ErgotSink::new_from_handle(queue.clone(), ERGOT_MTU),
            InterfaceState::Active { net_id: 1, node_id: 1 }
        )
    );

    // Spawn servers for device-originated events: button and keepalive.
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let server = stack.endpoints().bounded_server::<ButtonEndpoint, 8>(Some("button"));
            let server = pin!(server);
            let mut h = server.attach();
            loop {
                let _ = h.serve(|event: &ButtonEvent| {
                    let ev = event.clone();
                    async move {
                    match ev {
                        ButtonEvent::SingleClick => tracing::info!("Button: SINGLE"),
                        ButtonEvent::DoubleClick => tracing::info!("Button: DOUBLE"),
                        ButtonEvent::Hold => tracing::info!("Button: HOLD"),
                    }
                    }
                }).await;
            }
        }
    });

    tokio::spawn({
        let stack = stack.clone();
        async move {
            let server = stack.endpoints().bounded_server::<oxifoc_protocol::KeepAliveEndpoint, 8>(Some("keepalive"));
            let server = pin!(server);
            let mut h = server.attach();
            loop {
                let _ = h.serve(|ka: &oxifoc_protocol::KeepAlive| {
                    let seq = ka.seq;
                    async move {
                        tracing::info!("KeepAlive seq={} ", seq);
                    }
                }).await;
            }
        }
    });
    // Handshake task: retry querying device info until it succeeds (runs concurrently with I/O pump below)
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        async move {
            let device_addr = Address { network_id: 1, node_id: 2, port_id: 0 };
            let mut backoff = Duration::from_millis(100);
            for attempt in 1..=10u32 {
                let fut = stack
                    .endpoints()
                    .request::<oxifoc_protocol::InfoEndpoint>(device_addr, &(), Some("device_info"));
                match tokio::time::timeout(Duration::from_millis(800), fut).await {
                    Ok(Ok(info)) => {
                        let hw = info.hw.as_str();
                        let sw = info.sw.as_str();
                        tracing::info!("Device connected: hw='{}' sw='{}'", hw, sw);
                        return;
                    }
                    Ok(Err(e)) => {
                        tracing::warn!("DeviceInfo attempt {} failed: {:?}", attempt, e);
                    }
                    Err(_) => {
                        tracing::warn!("DeviceInfo attempt {} timed out", attempt);
                    }
                }
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(Duration::from_secs(2));
            }
            tracing::warn!("Device info not received after retries; continuing without it");
        }
    });

    // Prepare defmt decoder (ELF path)
    let default_elf = {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../device/target/thumbv7em-none-eabihf/release/oxifoc");
        p.to_string_lossy().into_owned()
    };
    let defmt_table: Option<Table> = if defmt_up_idx.is_some() {
        let elf_path = elf_from_cfg.unwrap_or(default_elf);
        let elf_bytes = fs::read(&elf_path)
            .with_context(|| format!("Failed to read ELF at {}", elf_path))?;
        Some(
            Table::parse(&elf_bytes)
                .context("Parsing defmt table from ELF failed")?
                .ok_or_else(|| anyhow::anyhow!("No .defmt section in ELF; build device with defmt"))?,
        )
    } else { None };
    let mut defmt_stream: Option<Box<dyn StreamDecoder + '_>> = defmt_table
        .as_ref()
        .map(|t| t.new_stream_decoder());

    // Main loop - read from channels (drives RTT <-> ergot)
    let mut buf = vec![0u8; 4096];
    let mut defbuf = vec![0u8; 2048];
    // Accumulator for COBS-framed ergot data across RTT reads
    let mut cobs_acc = CobsAccumulator::new_boxslice(1024 * 4);
    // Controller always has net_id=1
    let mut net_id = Some(1u16);
    // Downlink writer uses the queue's consumer to send frames to device via RTT down channel
    let down_idx = {
        let mut find_down = |name: &str| -> Option<usize> {
            rtt.down_channels()
                .iter()
                .enumerate()
                .find_map(|(i, ch)| if ch.name().map(|n| n == name).unwrap_or(false) { Some(i) } else { None })
        };
        find_down("ergot-down").or(Some(0))
    };
    let tx_consumer = queue.stream_consumer();
    loop {
        // Read ERGOT channel (COBS-framed)
        if let Some(up_idx) = ergot_up_idx
            && let Some(channel) = rtt.up_channels().get_mut(up_idx)
        {
                let count = channel.read(&mut core, &mut buf)?;
                if count > 0 {
                    let mut window = &mut buf[..count];
                    while !window.is_empty() {
                        window = match cobs_acc.feed_raw(window) {
                            FeedResult::Consumed => break,
                            FeedResult::OverFull(new_w) => new_w,
                            FeedResult::DecodeError(new_w) => new_w,
                            FeedResult::Success { data, remaining }
                            | FeedResult::SuccessInput { data, remaining } => {
                                // Process frame using DirectEdge (controller mode)
                                ergot_edge_process_frame(&mut net_id, data, &stack, ());
                                remaining
                            }
                        };
                    }
                }
        }
        // Read DEFMT channel and decode
        if let (Some(up_idx), Some(stream)) = (defmt_up_idx, defmt_stream.as_mut())
            && let Some(channel) = rtt.up_channels().get_mut(up_idx)
        {
                let count = channel.read(&mut core, &mut defbuf)?;
                if count > 0 {
                    stream.received(&defbuf[..count]);
                    loop {
                        match stream.decode() {
                            Ok(frame) => {
                                println!("{}", frame.display(true));
                            }
                            Err(DecodeError::UnexpectedEof) => break,
                            Err(DecodeError::Malformed) => { error!("Malformed defmt frame"); break; }
                        }
                    }
                }
        }
        // Flush any pending outbound ergot frames from queue to RTT down channel
        if let Some(di) = down_idx
            && let Some(channel) = rtt.down_channels().get_mut(di)
        {
            // Drain as many frames as available without blocking too long
            for _ in 0..8 {
                match tokio::time::timeout(Duration::from_millis(1), tx_consumer.wait_read()).await {
                    Ok(frame) => {
                        let len = frame.len();
                        if len == 0 { break; }
                        let data = &frame[..len];
                        let _ = channel.write(&mut core, data);
                        frame.release(len);
                    }
                    Err(_) => break,
                }
            }
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

// (no manual COBS or ad-hoc frame decoding here; ergot handles it)

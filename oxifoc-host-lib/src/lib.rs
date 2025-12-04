pub mod config;
pub mod discovery;
pub mod transport;

use anyhow::{Context, Result};
use cobs_acc::{CobsAccumulator, FeedResult};
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender, unbounded};
use defmt_decoder::{DecodeError, Table};
use defmt_parser::Level as DefmtLevel;
use ergot::interface_manager::InterfaceState;
use ergot::interface_manager::interface_impls::tokio_serial_cobs::TokioSerialInterface;
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
use ergot::interface_manager::utils::std::new_std_queue;
use ergot::net_stack::ArcNetStack;
use ergot::well_known::ErgotDefmtRxOwnedTopic;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::icd::{AdcSampleEndpoint, ButtonEndpoint, MotorEndpoint};
use oxifoc_core::types::{AdcSample, ButtonEvent, ControlMode};
use std::fs;
use std::path::Path;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::runtime::Runtime;
use tokio_util::sync::CancellationToken;
use tracing::{error, info};
use transport::Transport;

pub use config::HostConfig;
pub use discovery::{ProbeInfo, SerialPortInfo, list_probes, list_serial_ports};
pub use transport::{TransportConfig, TransportType};

pub fn init_tracing() {
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(true)
        .with_level(true)
        .compact()
        .try_init();
}

/// Default ADC polling rate in Hz
pub const DEFAULT_ADC_POLL_RATE_HZ: u32 = 60;

#[derive(Clone)]
pub enum HostCommand {
    Motor(ControlMode),
    /// Set ADC polling rate (0 = disabled, 1-255 = rate in Hz)
    SetAdcPollRate(u8),
}

pub struct HostRuntime {
    pub adc_rx: Receiver<AdcSample>,
    pub cmd_tx: Sender<HostCommand>,
    pub connected: Arc<AtomicBool>,
    cancel_token: CancellationToken,
}

impl HostRuntime {
    pub fn wait_for_connection(&self, timeout: Duration) -> bool {
        if self.connected.load(Ordering::Relaxed) {
            return true;
        }

        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.connected.load(Ordering::Relaxed) {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }

        self.connected.load(Ordering::Relaxed)
    }

    /// Signals all backend tasks to shut down gracefully.
    pub fn shutdown(&self) {
        info!("Shutting down host backend...");
        self.cancel_token.cancel();
    }
}

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (adc_tx, adc_rx) = unbounded::<AdcSample>();
    let (cmd_tx, cmd_rx) = unbounded::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let cancel_token = CancellationToken::new();

    spawn_backend(cfg, adc_tx, cmd_rx, connected.clone(), cancel_token.clone());

    HostRuntime {
        adc_rx,
        cmd_tx,
        connected,
        cancel_token,
    }
}

fn spawn_backend(
    config: HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: Receiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) {
    thread::spawn(move || {
        let rt = Runtime::new().expect("Failed to create tokio runtime");
        if let Err(e) = rt.block_on(backend_main(
            config,
            adc_tx,
            cmd_rx,
            connected_flag,
            cancel_token,
        )) {
            error!("backend_main error: {:?}", e);
        }
    });
}

async fn backend_main(
    cfg: HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: Receiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) -> Result<()> {
    const ERGOT_MTU: u16 = 512;

    let transport_type = cfg.transport_type();
    info!("Oxifoc Host backend - transport: {:?}", transport_type);

    if !cfg.stream_ergot() {
        info!("stream_ergot disabled in config; backend not starting transport");
        return Ok(());
    }

    let transport_config = cfg.transport_config()?;
    let transport = Transport::new(transport_config).await?;
    let (mut transport_rx, mut transport_tx, defmt_reader) =
        (transport.reader, transport.writer, transport.defmt_reader);

    type EdgeProfile = DirectEdge<TokioSerialInterface>;
    type EdgeStack = ArcNetStack<CriticalSectionRawMutex, EdgeProfile>;
    let queue = new_std_queue(4096);

    let stack: EdgeStack = ArcNetStack::new_with_profile(DirectEdge::new_controller(
        ErgotSink::new_from_handle(queue.clone(), ERGOT_MTU),
        InterfaceState::Active {
            net_id: 1,
            node_id: 1,
        },
    ));

    tokio::spawn({
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        let token = cancel_token.clone();
        async move {
            let mut buf = vec![0u8; 2048];
            let mut cobs_acc = CobsAccumulator::new_boxslice((ERGOT_MTU as usize) + 64);
            let mut net_id = Some(1u16);
            loop {
                tokio::select! {
                    _ = token.cancelled() => {
                        info!("Transport reader shutting down");
                        break;
                    }
                    result = transport_rx.read(&mut buf) => {
                        match result {
                            Ok(0) => {
                                error!("Transport closed");
                                connected_flag.store(false, Ordering::Relaxed);
                                break;
                            }
                            Ok(count) => {
                                let mut window = &mut buf[..count];
                                while !window.is_empty() {
                                    window = match cobs_acc.feed_raw(window) {
                                        FeedResult::Consumed => break,
                                        FeedResult::OverFull(rem) | FeedResult::DecodeError(rem) => rem,
                                        FeedResult::Success { data, remaining }
                                        | FeedResult::SuccessInput { data, remaining } => {
                                            ergot_edge_process_frame(&mut net_id, data, &stack, ());
                                            remaining
                                        }
                                    };
                                }
                            }
                            Err(e) => {
                                error!("Transport read error: {:?}", e);
                                connected_flag.store(false, Ordering::Relaxed);
                                break;
                            }
                        }
                    }
                }
            }
        }
    });

    tokio::spawn({
        let tx_queue = queue.clone();
        let connected_flag = connected_flag.clone();
        let token = cancel_token.clone();
        async move {
            let tx_consumer = tx_queue.stream_consumer();
            loop {
                tokio::select! {
                    _ = token.cancelled() => {
                        info!("Transport writer shutting down");
                        break;
                    }
                    frame = tx_consumer.wait_read() => {
                        let len = frame.len();
                        if len == 0 {
                            frame.release(len);
                            continue;
                        }

                        if let Err(e) = transport_tx.write_all(&frame[..len]).await {
                            error!("Transport write error: {:?}", e);
                            connected_flag.store(false, Ordering::Relaxed);
                            frame.release(len);
                            break;
                        }
                        frame.release(len);
                    }
                }
            }
        }
    });

    tokio::spawn({
        let stack = stack.clone();
        async move {
            let server = stack
                .endpoints()
                .bounded_server::<ButtonEndpoint, 8>(Some("button"));
            let server = pin!(server);
            let mut h = server.attach();
            loop {
                let _ = h
                    .serve(|event: &ButtonEvent| {
                        let ev = *event;
                        async move {
                            match ev {
                                ButtonEvent::SingleClick => tracing::info!("Button: SINGLE"),
                                ButtonEvent::DoubleClick => tracing::info!("Button: DOUBLE"),
                                ButtonEvent::Hold => tracing::info!("Button: HOLD"),
                            }
                        }
                    })
                    .await;
            }
        }
    });

    // ADC polling task - polls device at configurable rate
    let adc_poll_rate = Arc::new(std::sync::atomic::AtomicU8::new(
        DEFAULT_ADC_POLL_RATE_HZ as u8,
    ));
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let adc_tx = adc_tx.clone();
        let connected_flag = connected_flag.clone();
        let poll_rate = adc_poll_rate.clone();
        let token = cancel_token.clone();
        async move {
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };

            // Wait for device connection before starting to poll
            while !connected_flag.load(Ordering::Relaxed) {
                tokio::select! {
                    _ = token.cancelled() => return,
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {}
                }
            }

            tracing::info!(
                "ADC polling started at {}Hz",
                poll_rate.load(Ordering::Relaxed)
            );

            loop {
                let rate_hz = poll_rate.load(Ordering::Relaxed);
                if rate_hz == 0 {
                    // Polling disabled - check again after a delay
                    tokio::select! {
                        _ = token.cancelled() => break,
                        _ = tokio::time::sleep(Duration::from_millis(100)) => continue,
                    }
                }

                let interval = Duration::from_micros(1_000_000 / rate_hz as u64);
                let mut ticker = tokio::time::interval(interval);

                // Poll at current rate until rate changes or cancelled
                let current_rate = rate_hz;
                while poll_rate.load(Ordering::Relaxed) == current_rate {
                    tokio::select! {
                        _ = token.cancelled() => {
                            tracing::info!("ADC polling shutting down");
                            return;
                        }
                        _ = ticker.tick() => {
                            // Request ADC sample from device
                            let fut = stack
                                .endpoints()
                                .request::<AdcSampleEndpoint>(device_addr, &(), Some("adc"));
                            match tokio::time::timeout(Duration::from_millis(100), fut).await {
                                Ok(Ok(sample)) => {
                                    let _ = adc_tx.send(sample);
                                }
                                Ok(Err(_)) | Err(_) => {
                                    // Request failed or timed out - continue polling
                                }
                            }
                        }
                    }
                }
            }
        }
    });

    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let poll_rate = adc_poll_rate.clone();
        async move {
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };
            while let Ok(cmd) = cmd_rx.recv() {
                match cmd {
                    HostCommand::Motor(mc) => {
                        let res = stack
                            .endpoints()
                            .request::<MotorEndpoint>(device_addr, &mc, Some("motor"))
                            .await;
                        if let Err(e) = res {
                            tracing::warn!("Motor command failed: {:?}", e);
                        }
                    }
                    HostCommand::SetAdcPollRate(rate_hz) => {
                        let old_rate = poll_rate.load(Ordering::Relaxed);
                        poll_rate.store(rate_hz, Ordering::Relaxed);
                        tracing::info!("ADC poll rate changed: {}Hz -> {}Hz", old_rate, rate_hz);
                    }
                }
            }
        }
    });

    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        async move {
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };
            let mut backoff = Duration::from_millis(100);
            for attempt in 1..=10u32 {
                let fut = stack.endpoints().request::<oxifoc_core::icd::InfoEndpoint>(
                    device_addr,
                    &(),
                    Some("device_info"),
                );
                match tokio::time::timeout(Duration::from_millis(800), fut).await {
                    Ok(Ok(info)) => {
                        let hw = info.hw.as_str();
                        let sw = info.sw.as_str();
                        tracing::info!("Device connected: hw='{}' sw='{}'", hw, sw);
                        connected_flag.store(true, Ordering::Relaxed);
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

    if cfg.stream_defmt() {
        let default_elf = {
            // Check CARGO_TARGET_DIR first (for custom target directories)
            let target_dir = std::env::var("CARGO_TARGET_DIR")
                .ok()
                .map(std::path::PathBuf::from)
                .unwrap_or_else(|| {
                    Path::new(env!("CARGO_MANIFEST_DIR")).join("../oxifoc-g431/target")
                });
            let p = target_dir.join("thumbv7em-none-eabihf/release/oxifoc-g431");
            p.to_string_lossy().into_owned()
        };
        let elf_path = cfg.elf.clone().unwrap_or(default_elf);
        let elf_bytes =
            fs::read(&elf_path).with_context(|| format!("Failed to read ELF at {}", elf_path))?;
        let table = Table::parse(&elf_bytes)
            .context("Parsing defmt table from ELF failed")?
            .ok_or_else(|| anyhow::anyhow!("No .defmt section in ELF; build device with defmt"))?;

        if let Some(mut defmt_rx) = defmt_reader {
            // RTT mode: read defmt frames directly from RTT channel 0
            // Use a channel to send data to a blocking decoder thread (StreamDecoder is !Send)
            info!("Starting defmt decoder (RTT mode - channel 0)");
            let (tx, rx) = crossbeam_channel::bounded::<Vec<u8>>(64);

            // Decoder thread (blocking, since StreamDecoder is !Send)
            std::thread::spawn(move || {
                let mut stream = table.new_stream_decoder();
                while let Ok(data) = rx.recv() {
                    stream.received(&data);
                    loop {
                        match stream.decode() {
                            Ok(frame) => {
                                // Route defmt frames through tracing with target "device"
                                // so they appear in GUI, stdout, and logcat
                                let msg = frame.display(false).to_string();
                                match frame.level() {
                                    Some(DefmtLevel::Trace) => {
                                        tracing::trace!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Debug) => {
                                        tracing::debug!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Info) => {
                                        tracing::info!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Warn) => {
                                        tracing::warn!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Error) => {
                                        tracing::error!(target: "device", "{}", msg)
                                    }
                                    None => {
                                        // No level specified, default to info
                                        tracing::info!(target: "device", "{}", msg)
                                    }
                                }
                            }
                            Err(DecodeError::UnexpectedEof) => break,
                            Err(DecodeError::Malformed) => {
                                tracing::error!("Malformed defmt frame");
                                break;
                            }
                        }
                    }
                }
            });

            // Async reader task
            tokio::spawn(async move {
                let mut buf = vec![0u8; 1024];
                loop {
                    match defmt_rx.read(&mut buf).await {
                        Ok(0) => {
                            error!("Defmt RTT channel closed");
                            break;
                        }
                        Ok(count) => {
                            if tx.send(buf[..count].to_vec()).is_err() {
                                error!("Defmt decoder thread terminated");
                                break;
                            }
                        }
                        Err(e) => {
                            error!("Defmt RTT read error: {:?}", e);
                            break;
                        }
                    }
                }
            });
        } else {
            // Serial mode: defmt frames forwarded over ergot network
            info!("Starting defmt decoder (serial mode - ergot network)");
            tokio::spawn({
                let stack = stack.clone();
                async move {
                    let sub = stack
                        .topics()
                        .heap_bounded_receiver::<ErgotDefmtRxOwnedTopic>(32, Some("defmt"));
                    let sub = pin!(sub);
                    let mut hdl = sub.subscribe();

                    loop {
                        let msg = hdl.recv().await;
                        match table.decode(&msg.t.frame) {
                            Ok((frame, _consumed)) => {
                                // Route defmt frames through tracing with target "device"
                                let msg = frame.display(false).to_string();
                                match frame.level() {
                                    Some(DefmtLevel::Trace) => {
                                        tracing::trace!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Debug) => {
                                        tracing::debug!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Info) => {
                                        tracing::info!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Warn) => {
                                        tracing::warn!(target: "device", "{}", msg)
                                    }
                                    Some(DefmtLevel::Error) => {
                                        tracing::error!(target: "device", "{}", msg)
                                    }
                                    None => {
                                        tracing::info!(target: "device", "{}", msg)
                                    }
                                }
                            }
                            Err(DecodeError::UnexpectedEof) => {
                                error!("Unexpected EOF while decoding defmt frame");
                            }
                            Err(DecodeError::Malformed) => {
                                error!("Malformed defmt frame");
                            }
                        }
                    }
                }
            });
        }
    }

    // Wait for cancellation signal
    cancel_token.cancelled().await;
    info!("Host backend shutdown complete");
    Ok(())
}

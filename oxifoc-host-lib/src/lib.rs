pub mod config;
pub mod discovery;
pub mod transport;

use anyhow::{Context, Result};
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender, unbounded as crossbeam_unbounded};
use defmt_decoder::{DecodeError, Table};
use defmt_parser::Level as DefmtLevel;
use ergot::net_stack::NetStackHandle;
use ergot::well_known::ErgotDefmtRxOwnedTopic;
use oxifoc_core::icd::{AdcSampleEndpoint, ButtonEndpoint, MotorEndpoint};
use oxifoc_core::types::{AdcSample, ButtonEvent, ControlMode};
use std::{
    fs,
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU8, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use tokio::io::AsyncRead;
use tokio::runtime::Runtime;
use tokio_util::sync::CancellationToken;
use tracing::{error, info};

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
    pub cmd_tx: tokio::sync::mpsc::UnboundedSender<HostCommand>,
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

    pub fn shutdown(&self) {
        info!("Shutting down host backend...");
        self.cancel_token.cancel();
    }
}

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (adc_tx, adc_rx) = crossbeam_unbounded::<AdcSample>();
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
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
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
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

const ERGOT_MTU: u16 = 512;

async fn backend_main(
    cfg: HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) -> Result<()> {
    let transport_type = cfg.transport_type();
    info!("Oxifoc Host backend - transport: {:?}", transport_type);

    if !cfg.stream_ergot() {
        info!("stream_ergot disabled in config; backend not starting transport");
        return Ok(());
    }

    let transport_config = cfg.transport_config()?;

    match transport_config {
        // COBS-stream transports: TCP, Serial, RTT
        TransportConfig::Tcp { host, port } => {
            let transport = transport::tcp::connect(&host, port).await?;
            run_cobs_stream(
                transport,
                &cfg,
                adc_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        TransportConfig::Serial { path, baud } => {
            let transport = transport::serial::connect(&path, baud).await?;
            run_cobs_stream(
                transport,
                &cfg,
                adc_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        TransportConfig::Rtt { probe, chip } => {
            let transport = transport::rtt::connect(probe.as_deref(), &chip).await?;
            run_cobs_stream(
                transport,
                &cfg,
                adc_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        // Framed transports: UDP, USB
        TransportConfig::Udp { host, port } => {
            let stack = transport::udp::connect(&host, port).await?;
            spawn_protocol_tasks(
                &stack,
                adc_tx,
                cmd_rx,
                connected_flag.clone(),
                cancel_token.clone(),
            );
            if cfg.stream_defmt() {
                start_defmt_decoder(&cfg, &stack, None)?;
            }
            cancel_token.cancelled().await;
            Ok(())
        }
        TransportConfig::Usb => {
            let stack = transport::usb::connect().await?;
            spawn_protocol_tasks(
                &stack,
                adc_tx,
                cmd_rx,
                connected_flag.clone(),
                cancel_token.clone(),
            );
            if cfg.stream_defmt() {
                start_defmt_decoder(&cfg, &stack, None)?;
            }
            cancel_token.cancelled().await;
            Ok(())
        }
    }
}

/// Set up a COBS-stream transport (TCP, serial, RTT) and run the protocol.
async fn run_cobs_stream(
    transport: transport::CobsStreamTransport,
    cfg: &HostConfig,
    adc_tx: Sender<AdcSample>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) -> Result<()> {
    use ergot::toolkits::tokio_stream as stream_kit;

    let queue = stream_kit::new_std_queue(4096);
    let stack = stream_kit::new_controller_stack(&queue, ERGOT_MTU);

    stream_kit::register_controller_stream(
        stack.clone(),
        transport.reader,
        transport.writer,
        queue,
    )
    .await
    .map_err(|_| anyhow::anyhow!("Interface already active"))?;

    spawn_protocol_tasks(
        &stack,
        adc_tx,
        cmd_rx,
        connected_flag.clone(),
        cancel_token.clone(),
    );

    if cfg.stream_defmt() {
        start_defmt_decoder(cfg, &stack, transport.defmt_reader)?;
    }

    cancel_token.cancelled().await;
    info!("Host backend shutdown complete");
    Ok(())
}

// ── Protocol tasks (generic over any NetStackHandle) ─────────────────────────

fn spawn_protocol_tasks<NS>(
    stack: &NS,
    adc_tx: Sender<AdcSample>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    // Button event server
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let ns = stack.stack();
            let server = ns
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

    // ADC polling task
    let adc_poll_rate = Arc::new(AtomicU8::new(DEFAULT_ADC_POLL_RATE_HZ as u8));
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        let poll_rate = adc_poll_rate.clone();
        let token = cancel_token.clone();
        async move {
            let ns = stack.stack();
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };

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
                    tokio::select! {
                        _ = token.cancelled() => break,
                        _ = tokio::time::sleep(Duration::from_millis(100)) => continue,
                    }
                }

                let interval = Duration::from_micros(1_000_000 / rate_hz as u64);
                let mut ticker = tokio::time::interval(interval);
                let current_rate = rate_hz;

                while poll_rate.load(Ordering::Relaxed) == current_rate {
                    tokio::select! {
                        _ = token.cancelled() => return,
                        _ = ticker.tick() => {
                            let fut = ns
                                .endpoints()
                                .request::<AdcSampleEndpoint>(device_addr, &(), Some("adc"));
                            if let Ok(Ok(sample)) = tokio::time::timeout(Duration::from_millis(100), fut).await {
                                let _ = adc_tx.send(sample);
                            }
                        }
                    }
                }
            }
        }
    });

    // Motor command handler
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let poll_rate = adc_poll_rate.clone();
        async move {
            let mut cmd_rx = cmd_rx;
            let ns = stack.stack();
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };
            while let Some(cmd) = cmd_rx.recv().await {
                match cmd {
                    HostCommand::Motor(mc) => {
                        let res = ns
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

    // Device info handshake
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        async move {
            let ns = stack.stack();
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };
            let mut backoff = Duration::from_millis(100);
            for attempt in 1..=10u32 {
                let fut = ns.endpoints().request::<oxifoc_core::icd::InfoEndpoint>(
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
}

// ── Defmt decoding ───────────────────────────────────────────────────────────

fn start_defmt_decoder<NS>(
    cfg: &HostConfig,
    stack: &NS,
    defmt_reader: Option<Box<dyn AsyncRead + Send + Unpin>>,
) -> Result<()>
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    let default_elf = {
        let target_dir = std::env::var("CARGO_TARGET_DIR")
            .ok()
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../oxifoc-g431/target"));
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
        info!("Starting defmt decoder (RTT mode - channel 0)");
        let (tx, rx) = crossbeam_channel::bounded::<Vec<u8>>(64);

        std::thread::spawn(move || {
            let mut stream = table.new_stream_decoder();
            while let Ok(data) = rx.recv() {
                stream.received(&data);
                loop {
                    match stream.decode() {
                        Ok(frame) => {
                            log_defmt_frame(frame.level(), &frame.display(false).to_string())
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

        tokio::spawn(async move {
            use tokio::io::AsyncReadExt;
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
        // Non-RTT mode: defmt frames forwarded over ergot network
        info!("Starting defmt decoder (ergot network mode)");
        tokio::spawn({
            let stack = stack.clone();
            async move {
                let ns = stack.stack();
                let sub = ns
                    .topics()
                    .heap_bounded_receiver::<ErgotDefmtRxOwnedTopic>(32, Some("defmt"));
                let sub = pin!(sub);
                let mut hdl = sub.subscribe();

                loop {
                    let msg = hdl.recv().await;
                    match table.decode(&msg.t.frame) {
                        Ok((frame, _)) => {
                            log_defmt_frame(frame.level(), &frame.display(false).to_string())
                        }
                        Err(DecodeError::UnexpectedEof) => error!("Unexpected EOF decoding defmt"),
                        Err(DecodeError::Malformed) => error!("Malformed defmt frame"),
                    }
                }
            }
        });
    }

    Ok(())
}

fn log_defmt_frame(level: Option<DefmtLevel>, msg: &str) {
    match level {
        Some(DefmtLevel::Trace) => tracing::trace!(target: "device", "{}", msg),
        Some(DefmtLevel::Debug) => tracing::debug!(target: "device", "{}", msg),
        Some(DefmtLevel::Info) => tracing::info!(target: "device", "{}", msg),
        Some(DefmtLevel::Warn) => tracing::warn!(target: "device", "{}", msg),
        Some(DefmtLevel::Error) => tracing::error!(target: "device", "{}", msg),
        None => tracing::info!(target: "device", "{}", msg),
    }
}

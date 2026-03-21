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
use oxifoc_core::icd::{
    ButtonEndpoint, FastTelemetryTopic, MotorEndpoint, SlowTelemetryEndpoint,
    TelemetryConfig, TelemetryConfigEndpoint,
};
use oxifoc_core::types::{ButtonEvent, ControlMode, FastTelemetry, SlowTelemetry};
use std::{
    fs,
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use tokio::io::AsyncRead;
use tokio::runtime::Runtime;
use tokio_util::sync::CancellationToken;
use tracing::{error, info};

pub use config::{HostConfig, ReconnectPolicy};
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

#[derive(Clone)]
pub enum HostCommand {
    Motor(ControlMode),
    /// Configure telemetry streaming rates
    SetTelemetryConfig(TelemetryConfig),
}

pub struct HostRuntime {
    /// Fast telemetry receiver (currents, dq, angle, RPM — default 1kHz)
    pub fast_rx: Receiver<FastTelemetry>,
    /// Slow telemetry receiver (vbus, temps, state — polled at 10Hz)
    pub slow_rx: Receiver<SlowTelemetry>,
    /// Device info receiver (sent once on connection)
    pub device_info_rx: Receiver<oxifoc_core::types::DeviceInfo>,
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
    let (fast_tx, fast_rx) = crossbeam_unbounded::<FastTelemetry>();
    let (slow_tx, slow_rx) = crossbeam_unbounded::<SlowTelemetry>();
    let (info_tx, device_info_rx) = crossbeam_unbounded::<oxifoc_core::types::DeviceInfo>();
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let cancel_token = CancellationToken::new();

    spawn_backend(
        cfg,
        fast_tx,
        slow_tx,
        info_tx,
        cmd_rx,
        connected.clone(),
        cancel_token.clone(),
    );

    HostRuntime {
        fast_rx,
        slow_rx,
        device_info_rx,
        cmd_tx,
        connected,
        cancel_token,
    }
}

fn spawn_backend(
    config: HostConfig,
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    info_tx: Sender<oxifoc_core::types::DeviceInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) {
    thread::spawn(move || {
        let rt = Runtime::new().expect("Failed to create tokio runtime");
        if let Err(e) = rt.block_on(backend_main(
            config,
            fast_tx,
            slow_tx,
            info_tx,
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
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    info_tx: Sender<oxifoc_core::types::DeviceInfo>,
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
        // COBS-stream transports: TCP, Serial, RTT — support reconnection
        TransportConfig::Tcp { host, port } => {
            run_cobs_stream_with_reconnect(
                move || {
                    let host = host.clone();
                    async move { transport::tcp::connect(&host, port).await }
                },
                &cfg,
                fast_tx,
                slow_tx,
                info_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        TransportConfig::Serial { path, baud } => {
            run_cobs_stream_with_reconnect(
                move || {
                    let path = path.clone();
                    async move { transport::serial::connect(&path, baud).await }
                },
                &cfg,
                fast_tx,
                slow_tx,
                info_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        TransportConfig::Rtt { probe, chip } => {
            run_cobs_stream_with_reconnect(
                move || {
                    let probe = probe.clone();
                    let chip = chip.clone();
                    async move { transport::rtt::connect(probe.as_deref(), &chip).await }
                },
                &cfg,
                fast_tx,
                slow_tx,
                info_tx,
                cmd_rx,
                connected_flag,
                cancel_token,
            )
            .await
        }
        // Framed transports: UDP, USB — no reconnection loop (yet)
        TransportConfig::Udp { host, port } => {
            let stack = transport::udp::connect(&host, port).await?;
            spawn_protocol_tasks(
                &stack,
                fast_tx,
                slow_tx,
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
                fast_tx,
                slow_tx,
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

/// Set up a COBS-stream transport with automatic reconnection.
///
/// Protocol tasks (telemetry, commands) are spawned once and survive reconnections.
/// Only the transport stream is re-established when the link goes down.
async fn run_cobs_stream_with_reconnect<F, Fut>(
    connect_fn: F,
    cfg: &HostConfig,
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    info_tx: Sender<oxifoc_core::types::DeviceInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected_flag: Arc<AtomicBool>,
    cancel_token: CancellationToken,
) -> Result<()>
where
    F: Fn() -> Fut,
    Fut: std::future::Future<Output = Result<transport::CobsStreamTransport>>,
{
    use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
    use ergot::toolkits::tokio_stream as stream_kit;

    let queue = stream_kit::new_std_queue(4096);
    let stack = stream_kit::new_controller_stack(&queue, ERGOT_MTU);
    let state_notify = Arc::new(stream_kit::WaitQueue::new());

    // Protocol tasks are spawned once — they operate on the stack, not the transport
    spawn_protocol_tasks(
        &stack,
        fast_tx,
        slow_tx,
        cmd_rx,
        connected_flag.clone(),
        cancel_token.clone(),
    );

    // Defmt decoder spawned on first successful connection
    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;

    loop {
        // Try to connect
        let transport = tokio::select! {
            result = connect_fn() => {
                match result {
                    Ok(t) => {
                        connect_attempts = 0; // reset on success
                        t
                    }
                    Err(e) => {
                        connect_attempts += 1;
                        tracing::warn!("Transport connect failed (attempt {}): {:?}", connect_attempts, e);

                        // Check reconnect policy
                        match policy {
                            config::ReconnectPolicy::None => {
                                tracing::info!("Reconnect policy: none — giving up");
                                break;
                            }
                            config::ReconnectPolicy::Limited(max) if connect_attempts >= max => {
                                tracing::info!("Reconnect policy: exhausted {} attempts — giving up", max);
                                break;
                            }
                            _ => {} // Infinite or Limited with attempts remaining
                        }

                        // Wait before retrying, but respect cancel
                        tokio::select! {
                            _ = tokio::time::sleep(Duration::from_secs(2)) => continue,
                            _ = cancel_token.cancelled() => break,
                        }
                    }
                }
            }
            _ = cancel_token.cancelled() => break,
        };

        info!("Transport connected, registering stream...");

        // Register the stream on the (reusable) stack
        let reg_result = stream_kit::register_controller_stream(
            stack.clone(),
            transport.reader,
            transport.writer,
            queue.clone(),
            Some(LivenessConfig { timeout_ms: 3000 }),
            Some(state_notify.clone()),
        )
        .await;

        if reg_result.is_err() {
            tracing::warn!("Stream registration failed (interface not in Down state), retrying...");
            tokio::select! {
                _ = tokio::time::sleep(Duration::from_secs(1)) => continue,
                _ = cancel_token.cancelled() => break,
            }
        }

        // Wait for the interface to become Active before doing anything
        tokio::select! {
            _ = wait_for_active(&state_notify, &stack) => {}
            _ = cancel_token.cancelled() => break,
        }

        // DeviceInfo handshake — runs on each (re)connection
        {
            use ergot::Address;
            let device_addr = Address { network_id: 1, node_id: 2, port_id: 0 };
            let mut backoff = Duration::from_millis(100);
            for attempt in 1..=10u32 {
                let fut = stack.endpoints().request::<oxifoc_core::icd::InfoEndpoint>(
                    device_addr, &(), Some("device_info"),
                );
                match tokio::time::timeout(Duration::from_millis(800), fut).await {
                    Ok(Ok(dev_info)) => {
                        info!("Device connected: hw='{}' sw='{}' mcu='{}' uuid='{}' foc={}Hz max_i={}A",
                            dev_info.hw.as_str(), dev_info.sw.as_str(),
                            dev_info.mcu.as_str(), dev_info.uuid.as_str(),
                            dev_info.foc_freq_hz, dev_info.max_current_a);
                        let _ = info_tx.send(dev_info);
                        connected_flag.store(true, Ordering::Relaxed);
                        break;
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
        }

        // Start defmt decoder on first connection (if configured)
        if !defmt_started && cfg.stream_defmt() {
            start_defmt_decoder(cfg, &stack, transport.defmt_reader)?;
            defmt_started = true;
        }

        // Monitor interface state — update connected_flag and handle disconnect
        loop {
            tokio::select! {
                _ = state_notify.wait() => {
                    let state = stack.manage_profile(|im| im.interface_state(()));
                    let is_active = matches!(state, Some(InterfaceState::Active { .. }));
                    let was_active = connected_flag.swap(is_active, Ordering::Relaxed);

                    if !was_active && is_active {
                        info!("Interface active — device connected");
                    } else if was_active && !is_active {
                        tracing::warn!("Interface went down, waiting for recovery...");

                        // Wait for recovery (frames resume) with a deadline.
                        // If the link was just temporarily idle, it will recover.
                        // If the transport is actually dead, we'll time out and reconnect.
                        let recovered = tokio::time::timeout(
                            Duration::from_secs(10),
                            wait_for_active(&state_notify, &stack),
                        ).await.is_ok();

                        if recovered {
                            info!("Connection recovered");
                            connected_flag.store(true, Ordering::Relaxed);
                        } else {
                            tracing::warn!("Recovery timeout, reconnecting transport...");
                            connected_flag.store(false, Ordering::Relaxed);
                            break; // Exit inner loop → reconnect
                        }
                    }
                }
                _ = cancel_token.cancelled() => {
                    info!("Host backend shutdown complete");
                    return Ok(());
                }
            }
        }

        // Brief delay before reconnecting
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_secs(1)) => {}
            _ = cancel_token.cancelled() => break,
        }

        info!("Attempting reconnection...");
    }

    info!("Host backend shutdown complete");
    Ok(())
}

/// Wait until the interface is Active. Returns immediately if already Active.
async fn wait_for_active(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &ergot::toolkits::tokio_stream::EdgeStack,
) {
    use ergot::interface_manager::{InterfaceState, Profile};
    // Check current state first — may already be Active
    let state = stack.manage_profile(|im| im.interface_state(()));
    if matches!(state, Some(InterfaceState::Active { .. })) {
        return;
    }
    // Wait for state change notification
    loop {
        let _ = state_notify.wait().await;
        let state = stack.manage_profile(|im| im.interface_state(()));
        if matches!(state, Some(InterfaceState::Active { .. })) {
            return;
        }
    }
}

// ── Protocol tasks (generic over any NetStackHandle) ─────────────────────────

fn spawn_protocol_tasks<NS>(
    stack: &NS,
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
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

    // Fast telemetry subscriber (receive push-based motor data from device)
    tokio::spawn({
        let stack = stack.clone();
        let token = cancel_token.clone();
        async move {
            let ns = stack.stack();
            let receiver =
                ns.topics().single_receiver::<FastTelemetryTopic>(Some("fast_telem"));
            let mut pinned = pin!(receiver);
            let mut hdl = pinned.as_mut().subscribe();

            tracing::info!("Fast telemetry subscriber started");

            loop {
                tokio::select! {
                    _ = token.cancelled() => break,
                    msg = hdl.recv() => {
                        let _ = fast_tx.send(msg.t);
                    }
                }
            }
        }
    });

    // Slow telemetry poller (request/response — also serves as heartbeat)
    tokio::spawn({
        use ergot::Address;
        let stack = stack.clone();
        let connected_flag = connected_flag.clone();
        let token = cancel_token.clone();
        async move {
            let ns = stack.stack();
            let device_addr = Address {
                network_id: 1,
                node_id: 2,
                port_id: 0,
            };

            // Wait for initial connection
            while !connected_flag.load(Ordering::Relaxed) {
                tokio::select! {
                    _ = token.cancelled() => return,
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {}
                }
            }

            tracing::info!("Slow telemetry polling started (10Hz)");

            let mut ticker = tokio::time::interval(Duration::from_millis(100)); // 10Hz
            loop {
                tokio::select! {
                    _ = token.cancelled() => break,
                    _ = ticker.tick() => {
                        // Only poll when connected — avoid NoRouteToDest spam
                        if !connected_flag.load(Ordering::Relaxed) {
                            continue;
                        }
                        let fut = ns.endpoints()
                            .request::<SlowTelemetryEndpoint>(device_addr, &(), Some("slow_telem"));
                        if let Ok(Ok(sample)) = tokio::time::timeout(
                            Duration::from_millis(500), fut
                        ).await {
                            let _ = slow_tx.send(sample);
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
                    HostCommand::Motor(ref mc) => {
                        tracing::info!("Sending motor command: {:?}", mc);
                        let res = ns
                            .endpoints()
                            .request::<MotorEndpoint>(device_addr, mc, Some("motor"))
                            .await;
                        match &res {
                            Ok(status) => tracing::info!("Motor response: {:?}", status),
                            Err(e) => tracing::warn!("Motor command failed: {:?}", e),
                        }
                    }
                    HostCommand::SetTelemetryConfig(cfg) => {
                        tracing::info!("Setting telemetry config: {:?}", cfg);
                        let res = ns
                            .endpoints()
                            .request::<TelemetryConfigEndpoint>(device_addr, &cfg, Some("telem_cfg"))
                            .await;
                        match &res {
                            Ok(ack) => tracing::info!("Telemetry config ack: fast={}Hz",
                                ack.actual_fast_hz),
                            Err(e) => tracing::warn!("Telemetry config failed: {:?}", e),
                        }
                    }
                }
            }
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

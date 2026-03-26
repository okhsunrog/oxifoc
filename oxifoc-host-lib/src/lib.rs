pub mod config;
pub mod discovery;
pub mod transport;

use anyhow::{Context, Result};
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender};
use defmt_decoder::{DecodeError, Table};
use defmt_parser::Level as DefmtLevel;
use ergot::Address;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::net_stack::NetStackHandle;
use ergot::well_known::ErgotDefmtRxOwnedTopic;
use oxifoc_core::icd::{
    ButtonEndpoint, ConfigEndpoint, DetectEndpoint, FastTelemetryTopic, MotorEndpoint,
    SlowTelemetryEndpoint, TelemetryConfig, TelemetryConfigEndpoint,
};
use oxifoc_core::types::{ButtonEvent, ControlMode, FastTelemetry, SlowTelemetry};
use std::{
    fs,
    path::Path,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU16, Ordering},
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

// ── Constants ────────────────────────────────────────────────────────────────

/// Address of the device (Router) on the direct link.
///
/// The device is the central node (node_id=1) on net_id=1.
/// The host is the edge node (node_id=2).
const DEVICE_ADDR: Address = Address {
    network_id: 1,
    node_id: 1,
    port_id: 0,
};
const ERGOT_MTU: u16 = 2048;
const ERGOT_QUEUE_SIZE: usize = 32768;
const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(800);
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const RECOVERY_TIMEOUT: Duration = Duration::from_secs(10);
const COMMAND_TIMEOUT: Duration = Duration::from_secs(2);
const DETECT_TIMEOUT: Duration = Duration::from_secs(60);

// ── Public API ───────────────────────────────────────────────────────────────

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

/// Type alias for config response oneshot channels
pub type ConfigResponseSender =
    tokio::sync::oneshot::Sender<Result<oxifoc_core::types::ConfigResponse>>;
pub type ConfigResponseReceiver =
    tokio::sync::oneshot::Receiver<Result<oxifoc_core::types::ConfigResponse>>;

/// Create a oneshot channel pair for config request/response
pub fn config_channel() -> (ConfigResponseSender, ConfigResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Type alias for detect response oneshot channels
pub type DetectResponseSender =
    tokio::sync::oneshot::Sender<Result<oxifoc_core::types::DetectResponse>>;
pub type DetectResponseReceiver =
    tokio::sync::oneshot::Receiver<Result<oxifoc_core::types::DetectResponse>>;

/// Create a oneshot channel pair for detect request/response
pub fn detect_channel() -> (DetectResponseSender, DetectResponseReceiver) {
    tokio::sync::oneshot::channel()
}

pub enum HostCommand {
    Motor(ControlMode),
    SetTelemetryConfig(TelemetryConfig),
    ConfigRead(oxifoc_core::types::ConfigGroupId, ConfigResponseSender),
    ConfigWrite(oxifoc_core::types::ConfigWrite, ConfigResponseSender),
    Detect(oxifoc_core::types::DetectRequest, DetectResponseSender),
}

pub struct HostRuntime {
    pub fast_rx: Receiver<FastTelemetry>,
    pub slow_rx: Receiver<SlowTelemetry>,
    pub device_info_rx: Receiver<oxifoc_core::types::DeviceInfo>,
    pub cmd_tx: tokio::sync::mpsc::UnboundedSender<HostCommand>,
    pub connected: Arc<AtomicBool>,
    pub fast_hz: Arc<AtomicU16>,
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

// ── Backend context ──────────────────────────────────────────────────────────

/// Shared state passed through the backend instead of many individual arguments.
struct BackendCtx {
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    info_tx: Sender<oxifoc_core::types::DeviceInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected: Arc<AtomicBool>,
    fast_hz: Arc<AtomicU16>,
    cancel: CancellationToken,
}

// ── Entry point ──────────────────────────────────────────────────────────────

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (fast_tx, fast_rx) = crossbeam_channel::bounded::<FastTelemetry>(4096);
    let (slow_tx, slow_rx) = crossbeam_channel::bounded::<SlowTelemetry>(64);
    let (info_tx, device_info_rx) = crossbeam_channel::bounded::<oxifoc_core::types::DeviceInfo>(4);
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let fast_hz = Arc::new(AtomicU16::new(0));
    let cancel_token = CancellationToken::new();

    let ctx = BackendCtx {
        fast_tx,
        slow_tx,
        info_tx,
        cmd_rx,
        connected: connected.clone(),
        fast_hz: fast_hz.clone(),
        cancel: cancel_token.clone(),
    };

    thread::spawn(move || {
        let rt = Runtime::new().expect("Failed to create tokio runtime");
        if let Err(e) = rt.block_on(backend_main(cfg, ctx)) {
            error!("backend_main error: {:?}", e);
        }
    });

    HostRuntime {
        fast_rx,
        slow_rx,
        device_info_rx,
        cmd_tx,
        connected,
        fast_hz,
        cancel_token,
    }
}

// ── Backend dispatch ─────────────────────────────────────────────────────────

async fn backend_main(cfg: HostConfig, ctx: BackendCtx) -> Result<()> {
    let transport_type = cfg.transport_type();
    info!("Oxifoc Host backend - transport: {:?}", transport_type);

    if !cfg.stream_ergot() {
        info!("stream_ergot disabled in config; backend not starting transport");
        return Ok(());
    }

    let transport_config = cfg.transport_config()?;

    match transport_config {
        TransportConfig::Tcp { host, port } => {
            run_cobs_stream_with_reconnect(
                move || {
                    let host = host.clone();
                    async move { transport::tcp::connect(&host, port).await }
                },
                &cfg,
                ctx,
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
                ctx,
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
                ctx,
            )
            .await
        }
        TransportConfig::Udp { host, port } => {
            let state_notify = Arc::new(ergot::toolkits::tokio_stream::WaitQueue::new());
            let stack = transport::udp::connect(&host, port, Some(state_notify.clone())).await?;
            run_framed_transport(stack, state_notify, &cfg, ctx).await
        }
        TransportConfig::Usb => {
            let state_notify = Arc::new(ergot::toolkits::tokio_stream::WaitQueue::new());
            let stack = transport::usb::connect(Some(state_notify.clone())).await?;
            run_framed_transport(stack, state_notify, &cfg, ctx).await
        }
    }
}

// ── Framed transport runner (UDP, USB) ───────────────────────────────────────

async fn run_framed_transport<NS>(
    stack: NS,
    state_notify: Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    cfg: &HostConfig,
    ctx: BackendCtx,
) -> Result<()>
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>> + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    let connected = ctx.connected.clone();
    let fast_hz_flag = ctx.fast_hz.clone();
    let cancel = ctx.cancel.clone();
    let info_tx = ctx.info_tx.clone();

    device_info_handshake(&stack, &info_tx, &connected).await;
    enable_fast_telemetry(&stack, cfg.fast_hz(), &fast_hz_flag).await;
    spawn_protocol_tasks(&stack, ctx);
    if cfg.stream_defmt() {
        start_defmt_decoder(cfg, &stack, None)?;
    }
    monitor_state_until_down(&state_notify, &stack, &connected, &cancel).await;
    Ok(())
}

// ── COBS stream with reconnection ────────────────────────────────────────────

async fn run_cobs_stream_with_reconnect<F, Fut>(
    connect_fn: F,
    cfg: &HostConfig,
    ctx: BackendCtx,
) -> Result<()>
where
    F: Fn() -> Fut,
    Fut: std::future::Future<Output = Result<transport::CobsStreamTransport>>,
{
    use ergot::interface_manager::profiles::direct_edge::{EDGE_NODE_ID, EdgeFrameProcessor};
    use ergot::interface_manager::transports::tokio_cobs_stream;
    use ergot::toolkits::tokio_stream as stream_kit;

    let queue = stream_kit::new_std_queue(ERGOT_QUEUE_SIZE);
    // Host is an edge device connecting to a device-side Router
    let stack = stream_kit::new_target_stack(&queue, ERGOT_MTU);
    let state_notify = Arc::new(stream_kit::WaitQueue::new());

    let connected = ctx.connected.clone();
    let fast_hz_flag = ctx.fast_hz.clone();
    let cancel = ctx.cancel.clone();
    let info_tx = ctx.info_tx.clone();

    // Protocol tasks are spawned once — they operate on the stack, not the transport
    spawn_protocol_tasks(&stack, ctx);

    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;

    loop {
        // Try to connect
        let transport = tokio::select! {
            result = connect_fn() => {
                match result {
                    Ok(t) => {
                        connect_attempts = 0;
                        t
                    }
                    Err(e) => {
                        connect_attempts += 1;
                        tracing::warn!("Transport connect failed (attempt {}): {:?}", connect_attempts, e);

                        match policy {
                            config::ReconnectPolicy::None => {
                                info!("Reconnect policy: none — giving up");
                                break;
                            }
                            config::ReconnectPolicy::Limited(max) if connect_attempts >= max => {
                                info!("Reconnect policy: exhausted {} attempts — giving up", max);
                                break;
                            }
                            _ => {}
                        }

                        tokio::select! {
                            _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                            _ = cancel.cancelled() => break,
                        }
                    }
                }
            }
            _ = cancel.cancelled() => break,
        };

        info!("Transport connected, registering stream...");

        let reg_result = tokio_cobs_stream::register_edge::<
            _,
            ergot::interface_manager::interface_impls::tokio_stream::TokioStreamInterface,
            _,
            _,
        >(
            stack.clone(),
            transport.reader,
            transport.writer,
            queue.clone(),
            EdgeFrameProcessor::new_controller(1),
            InterfaceState::Active {
                net_id: 1,
                node_id: EDGE_NODE_ID,
            },
            Some(LivenessConfig {
                timeout_ms: oxifoc_core::icd::LIVENESS_TIMEOUT_MS,
            }),
            Some(state_notify.clone()),
        )
        .await;

        if reg_result.is_err() {
            tracing::warn!("Stream registration failed (interface not in Down state), retrying...");
            tokio::select! {
                _ = tokio::time::sleep(Duration::from_secs(1)) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // Wait for the interface to become Active
        tokio::select! {
            _ = wait_for_active(&state_notify, &stack) => {}
            _ = cancel.cancelled() => break,
        }

        // DeviceInfo handshake on each (re)connection
        let handshake_ok = device_info_handshake(&stack, &info_tx, &connected).await;

        if !handshake_ok {
            tracing::warn!("Handshake failed, reconnecting...");
            connected.store(false, Ordering::Relaxed);
            stack.manage_profile(|im| im.teardown());
            tokio::task::yield_now().await;
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => {}
                _ = cancel.cancelled() => break,
            }
            continue;
        }

        enable_fast_telemetry(&stack, cfg.fast_hz(), &fast_hz_flag).await;

        if !defmt_started && cfg.stream_defmt() {
            start_defmt_decoder(cfg, &stack, transport.defmt_reader)?;
            defmt_started = true;
        }

        // Monitor interface state with recovery
        let disconnected =
            monitor_state_with_recovery(&state_notify, &stack, &connected, &cancel).await;

        if !disconnected {
            // Cancelled
            break;
        }

        // Tear down old interface so workers release the transport
        info!("Calling teardown...");
        stack.manage_profile(|im| im.teardown());
        tokio::task::yield_now().await;
        tokio::select! {
            _ = tokio::time::sleep(RECONNECT_DELAY) => {}
            _ = cancel.cancelled() => break,
        }
        info!("Attempting reconnection...");
    }

    info!("Host backend shutdown complete");
    Ok(())
}

// ── State monitoring ─────────────────────────────────────────────────────────

/// Monitor interface state, updating the connected flag.
/// Returns when the interface goes Down or cancel fires.
async fn monitor_state_until_down<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
    connected: &Arc<AtomicBool>,
    cancel: &CancellationToken,
) where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    loop {
        tokio::select! {
            _ = state_notify.wait() => {
                let state = stack.stack().manage_profile(|im| im.interface_state(()));
                let is_active = matches!(state, Some(InterfaceState::Active { .. }));
                connected.store(is_active, Ordering::Relaxed);
                if matches!(state, Some(InterfaceState::Down)) {
                    tracing::warn!("Interface went Down");
                    break;
                }
            }
            _ = cancel.cancelled() => break,
        }
    }
}

/// Monitor interface state with recovery support (for COBS streams).
/// Returns `true` if disconnected (needs reconnect), `false` if cancelled.
async fn monitor_state_with_recovery(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &ergot::toolkits::tokio_stream::EdgeStack,
    connected: &Arc<AtomicBool>,
    cancel: &CancellationToken,
) -> bool {
    loop {
        tokio::select! {
            _ = state_notify.wait() => {
                let state = stack.manage_profile(|im| im.interface_state(()));
                let is_active = matches!(state, Some(InterfaceState::Active { .. }));
                let was_active = connected.swap(is_active, Ordering::Relaxed);

                if !was_active && is_active {
                    info!("Interface active — device connected");
                } else if was_active && !is_active {
                    tracing::warn!("Interface inactive, waiting for recovery...");

                    let recovered = tokio::time::timeout(
                        RECOVERY_TIMEOUT,
                        wait_for_active(state_notify, stack),
                    ).await.is_ok();

                    if recovered {
                        info!("Connection recovered");
                        connected.store(true, Ordering::Relaxed);
                    } else {
                        tracing::warn!("Recovery timeout, reconnecting transport...");
                        connected.store(false, Ordering::Relaxed);
                        return true; // needs reconnect
                    }
                }
            }
            _ = cancel.cancelled() => return false,
        }
    }
}

/// Wait until the interface is Active.
async fn wait_for_active<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
) where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    let state = stack.stack().manage_profile(|im| im.interface_state(()));
    if matches!(state, Some(InterfaceState::Active { .. })) {
        return;
    }
    loop {
        let _ = state_notify.wait().await;
        let state = stack.stack().manage_profile(|im| im.interface_state(()));
        if matches!(state, Some(InterfaceState::Active { .. })) {
            return;
        }
    }
}

// ── Device handshake & telemetry setup ───────────────────────────────────────

/// Run DeviceInfo handshake with retries and exponential backoff.
/// Returns `true` on success.
async fn device_info_handshake<NS>(
    stack: &NS,
    info_tx: &Sender<oxifoc_core::types::DeviceInfo>,
    connected: &Arc<AtomicBool>,
) -> bool
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let ns = stack.stack();
    let mut backoff = Duration::from_millis(100);
    for attempt in 1..=10u32 {
        let fut = ns.endpoints().request::<oxifoc_core::icd::InfoEndpoint>(
            DEVICE_ADDR,
            &(),
            Some("device_info"),
        );
        match tokio::time::timeout(HANDSHAKE_TIMEOUT, fut).await {
            Ok(Ok(dev_info)) => {
                info!(
                    "Device connected: hw='{}' sw='{}' mcu='{}' uuid='{}' foc={}Hz max_i={}A",
                    dev_info.hw.as_str(),
                    dev_info.sw.as_str(),
                    dev_info.mcu.as_str(),
                    dev_info.uuid.as_str(),
                    dev_info.foc_freq_hz,
                    dev_info.max_current_a
                );
                let _ = info_tx.send(dev_info);
                connected.store(true, Ordering::Relaxed);
                return true;
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
    connected.store(true, Ordering::Relaxed);
    false
}

/// Send TelemetryConfig to enable fast telemetry streaming.
async fn enable_fast_telemetry<NS>(stack: &NS, fast_hz: u16, fast_hz_flag: &Arc<AtomicU16>)
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    if fast_hz == 0 {
        return;
    }
    let telem_cfg = TelemetryConfig { fast_hz };
    let ns = stack.stack();
    let fut = ns
        .endpoints()
        .request::<TelemetryConfigEndpoint>(DEVICE_ADDR, &telem_cfg, None);
    match tokio::time::timeout(COMMAND_TIMEOUT, fut).await {
        Ok(Ok(ack)) => {
            info!(
                "Telemetry enabled: requested={}Hz, actual={}Hz",
                fast_hz, ack.actual_fast_hz
            );
            fast_hz_flag.store(ack.actual_fast_hz, Ordering::Relaxed);
        }
        Ok(Err(e)) => tracing::warn!("Telemetry config failed: {:?}", e),
        Err(_) => tracing::warn!("Telemetry config timed out"),
    }
}

// ── Protocol tasks ───────────────────────────────────────────────────────────

fn spawn_protocol_tasks<NS>(stack: &NS, ctx: BackendCtx)
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    spawn_button_server(stack);
    spawn_fast_telemetry_subscriber(stack, ctx.fast_tx, ctx.cancel.clone());
    spawn_slow_telemetry_poller(
        stack,
        ctx.slow_tx,
        ctx.connected.clone(),
        ctx.cancel.clone(),
    );

    // Command handler — owns cmd_rx
    tokio::spawn({
        let stack = stack.clone();
        let fast_hz_flag = ctx.fast_hz;
        let mut cmd_rx = ctx.cmd_rx;
        async move {
            while let Some(cmd) = cmd_rx.recv().await {
                handle_command(&stack, cmd, &fast_hz_flag).await;
            }
        }
    });
}

fn spawn_button_server<NS>(stack: &NS)
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
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
}

fn spawn_fast_telemetry_subscriber<NS>(
    stack: &NS,
    fast_tx: Sender<FastTelemetry>,
    cancel: CancellationToken,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let ns = stack.stack();
            let receiver = ns
                .topics()
                .heap_bounded_receiver::<FastTelemetryTopic>(128, Some("fast_telem"));
            let mut pinned = pin!(receiver);
            let mut hdl = pinned.as_mut().subscribe();
            info!("Fast telemetry subscriber started");
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    msg = hdl.recv() => {
                        for sample in &msg.t.samples {
                            let _ = fast_tx.send(*sample);
                        }
                    }
                }
            }
        }
    });
}

fn spawn_slow_telemetry_poller<NS>(
    stack: &NS,
    slow_tx: Sender<SlowTelemetry>,
    connected: Arc<AtomicBool>,
    cancel: CancellationToken,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    tokio::spawn({
        let stack = stack.clone();
        async move {
            let ns = stack.stack();

            // Wait for initial connection
            while !connected.load(Ordering::Relaxed) {
                tokio::select! {
                    _ = cancel.cancelled() => return,
                    _ = tokio::time::sleep(Duration::from_millis(100)) => {}
                }
            }

            info!("Slow telemetry polling started (10Hz)");
            let mut ticker = tokio::time::interval(Duration::from_millis(100));
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    _ = ticker.tick() => {
                        if !connected.load(Ordering::Relaxed) {
                            continue;
                        }
                        let fut = ns.endpoints()
                            .request::<SlowTelemetryEndpoint>(DEVICE_ADDR, &(), Some("slow_telem"));
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
}

async fn handle_command<NS>(ns: &NS, cmd: HostCommand, fast_hz_flag: &Arc<AtomicU16>)
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let s = ns.stack();
    match cmd {
        HostCommand::Motor(ref mc) => {
            tracing::info!("Sending motor command: {:?}", mc);
            let fut = s
                .endpoints()
                .request::<MotorEndpoint>(DEVICE_ADDR, mc, Some("motor"));
            match tokio::time::timeout(COMMAND_TIMEOUT, fut).await {
                Ok(Ok(status)) => tracing::info!("Motor response: {:?}", status),
                Ok(Err(e)) => tracing::warn!("Motor command failed: {:?}", e),
                Err(_) => tracing::warn!("Motor command timed out"),
            }
        }
        HostCommand::SetTelemetryConfig(cfg) => {
            tracing::info!("Setting telemetry config: {:?}", cfg);
            let fut = s.endpoints().request::<TelemetryConfigEndpoint>(
                DEVICE_ADDR,
                &cfg,
                Some("telemetry_config"),
            );
            match tokio::time::timeout(COMMAND_TIMEOUT, fut).await {
                Ok(Ok(ack)) => {
                    tracing::info!("Telemetry config ack: fast={}Hz", ack.actual_fast_hz);
                    fast_hz_flag.store(ack.actual_fast_hz, Ordering::Relaxed);
                }
                Ok(Err(e)) => tracing::warn!("Telemetry config failed: {:?}", e),
                Err(_) => tracing::warn!("Telemetry config timed out"),
            }
        }
        HostCommand::ConfigRead(group_id, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Reading config group: {:?}", group_id);
            let req = ConfigRequest::Read(group_id);
            let res = s
                .endpoints()
                .request::<ConfigEndpoint>(DEVICE_ADDR, &req, Some("config"))
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{:?}", e)));
        }
        HostCommand::ConfigWrite(write, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Writing config: {:?}", write);
            let req = ConfigRequest::Write(write);
            let res = s
                .endpoints()
                .request::<ConfigEndpoint>(DEVICE_ADDR, &req, Some("config"))
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{:?}", e)));
        }
        HostCommand::Detect(req, reply_tx) => {
            tracing::info!("Starting motor detection: {:?}", req);
            let res = tokio::time::timeout(
                DETECT_TIMEOUT,
                s.endpoints()
                    .request::<DetectEndpoint>(DEVICE_ADDR, &req, Some("detect")),
            )
            .await;
            let result = match res {
                Ok(inner) => inner.map_err(|e| anyhow::anyhow!("{:?}", e)),
                Err(_) => Err(anyhow::anyhow!("Detection timed out (60s)")),
            };
            let _ = reply_tx.send(result);
        }
    }
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
                            error!("Malformed defmt frame");
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

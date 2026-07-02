pub mod config;
pub mod discovery;
pub mod ops;
pub mod transport;

use anyhow::{Context, Result, anyhow};
use core::pin::pin;
use crossbeam_channel::{Receiver, Sender};
use defmt_decoder::{DecodeError, Table};
use defmt_parser::Level as DefmtLevel;
use ergot::Address;
use ergot::interface_manager::{InterfaceState, LivenessConfig, Profile};
use ergot::net_stack::NetStackHandle;
use ergot::well_known::ErgotDefmtRxOwnedTopic;
use oxifoc_core::delivery::{ReliableExt, RetryPolicy};
use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::icd::{
    ConfigEndpoint, DetectEndpoint, FastTelemetryTopic, FaultTopic, MotorEndpoint,
    SlowTelemetryEndpoint, TelemetryConfig, TelemetryConfigEndpoint,
};
use oxifoc_core::icd::{
    FaultEndpoint, HardwareInfoEndpoint, LIVENESS_TIMEOUT_MS, PhaseSourceEndpoint,
};
use oxifoc_core::timer::Timer;
use oxifoc_core::types::{
    ConfigGroupId, ConfigResponse, ConfigWrite, DetectRequest, DetectResponse, FaultRequest,
    HardwareInfo, MotorStatus,
};
use oxifoc_core::types::{ControlMode, FastTelemetry, FaultResponse, Keyed, ReqId, SlowTelemetry};
use std::{
    fs,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU16, AtomicU64, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use tokio::io::AsyncRead;
use tokio::runtime::Runtime;
use tokio_util::sync::CancellationToken;
use tracing::{error, info};

pub use config::{HostConfig, ReconnectPolicy};
pub use discovery::{BleDeviceInfo, scan_ble_devices};
#[cfg(feature = "desktop")]
pub use discovery::{ProbeInfo, SerialPortInfo, list_probes, list_serial_ports};
pub use transport::{TransportConfig, TransportType};

// ── Constants ────────────────────────────────────────────────────────────────

/// Link-local address of the directly connected device (Router).
///
/// Uses net_id=0 ("link-local"): the router rewrites this to the real net_id.
/// node_id=1 is CENTRAL_NODE_ID (the router side of the link).
const DEVICE_ADDR: Address = Address {
    network_id: 0,
    node_id: 1,
    port_id: 0,
};
const ERGOT_MTU: u16 = 4096;
const ERGOT_QUEUE_SIZE: usize = 32768;
const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(800);
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
const RECOVERY_TIMEOUT: Duration = Duration::from_secs(10);
/// Retry budget for idempotent setpoints (motor / telemetry-config / config):
/// a few attempts within a ~2 s total budget. Safe to retry on timeout because
/// these are absolute setpoints / PUT-like config writes.
const SETPOINT_POLICY: RetryPolicy = RetryPolicy {
    deadline_ms: 2_000,
    base_backoff_ms: 50,
    max_backoff_ms: 400,
    attempt_timeout_ms: 600,
};
/// Effectively-once budget for motor detection: one ~60 s attempt, plus room
/// for a single retry whose (cached) response returns near-instantly.
const DETECT_POLICY: RetryPolicy = RetryPolicy {
    deadline_ms: 70_000,
    base_backoff_ms: 500,
    max_backoff_ms: 2_000,
    attempt_timeout_ms: 60_000,
};
/// Periodic affirmation of the active drive setpoint that keeps the device's
/// ISR command-staleness deadman fed (≈150 ms threshold). One short attempt,
/// **no retry**: a persistently dropped affirmation is exactly what the
/// deadman must catch — retrying would mask a dying link (cf.
/// `oxifoc_core::delivery::policy`). The 50 ms resend cadence tolerates the
/// occasional miss.
const AFFIRM_POLICY: RetryPolicy = RetryPolicy {
    deadline_ms: 40,
    base_backoff_ms: 0,
    max_backoff_ms: 0,
    attempt_timeout_ms: 40,
};
/// Cadence at which the active drive setpoint is re-affirmed to the device.
const AFFIRM_INTERVAL: Duration = Duration::from_millis(50);

/// Tokio-backed timer for the reliable-delivery client driver.
struct TokioTimer;
impl Timer for TokioTimer {
    async fn after_millis(ms: u64) {
        tokio::time::sleep(Duration::from_millis(ms)).await;
    }
    async fn after_micros(us: u64) {
        tokio::time::sleep(Duration::from_micros(us)).await;
    }
}

/// Monotonic idempotency-key source for deduplicated requests (detection).
///
/// Seeded from wall-clock nanos: the device-side dedup cache outlives host
/// processes and matches on `ReqId`, so two runs both starting at id 1 would
/// make the device replay the previous run's cached response instead of
/// executing the new (possibly different) request.
fn next_detect_id() -> ReqId {
    static CTR: std::sync::OnceLock<AtomicU64> = std::sync::OnceLock::new();
    let ctr = CTR.get_or_init(|| {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        AtomicU64::new(nanos | 1)
    });
    ReqId(ctr.fetch_add(1, Ordering::Relaxed))
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Resolve the running firmware ELF path from `cfg.elf` (CLI `--elf`). Used both
/// for defmt decoding and for pinning the RTT control block to the firmware's
/// `_SEGGER_RTT` symbol. There is deliberately NO default: guessing the wrong
/// board's ELF silently pins RTT to the wrong address (the link never routes)
/// and decodes defmt against the wrong table, so a missing path is a hard error.
fn resolve_elf_path(cfg: &HostConfig) -> Result<String> {
    cfg.elf.clone().ok_or_else(|| {
        anyhow!(
            "no firmware ELF configured — pass --elf <path-to-elf> (or set `elf` in \
             the config file). It is required for RTT control-block pinning and \
             defmt decoding; there is no default board."
        )
    })
}

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
pub type ConfigResponseSender = tokio::sync::oneshot::Sender<Result<ConfigResponse>>;
pub type ConfigResponseReceiver = tokio::sync::oneshot::Receiver<Result<ConfigResponse>>;

/// Create a oneshot channel pair for config request/response
pub fn config_channel() -> (ConfigResponseSender, ConfigResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Type alias for detect response oneshot channels
pub type DetectResponseSender = tokio::sync::oneshot::Sender<Result<DetectResponse>>;
pub type DetectResponseReceiver = tokio::sync::oneshot::Receiver<Result<DetectResponse>>;

/// Create a oneshot channel pair for detect request/response
pub fn detect_channel() -> (DetectResponseSender, DetectResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Acknowledged motor-command channel: carries the device's MotorStatus
/// (or the delivery error) back to the caller — for CLI-style users that
/// must exit nonzero when the command did not reach the device.
pub type MotorResponseSender = tokio::sync::oneshot::Sender<Result<MotorStatus>>;
pub type MotorResponseReceiver = tokio::sync::oneshot::Receiver<Result<MotorStatus>>;

/// Create a oneshot channel pair for an acknowledged motor command
pub fn motor_channel() -> (MotorResponseSender, MotorResponseReceiver) {
    tokio::sync::oneshot::channel()
}

/// Type alias for fault query/clear response oneshot channels
pub type FaultResponseSender = tokio::sync::oneshot::Sender<Result<FaultResponse>>;
pub type FaultResponseReceiver = tokio::sync::oneshot::Receiver<Result<FaultResponse>>;

/// Create a oneshot channel pair for a fault request/response
pub fn fault_channel() -> (FaultResponseSender, FaultResponseReceiver) {
    tokio::sync::oneshot::channel()
}

pub enum HostCommand {
    Motor(ControlMode),
    /// Like [`Motor`](Self::Motor) but replies with the device's status
    /// (or the delivery error).
    MotorAck(ControlMode, MotorResponseSender),
    SetPhaseSource(PhaseSource),
    SetTelemetryConfig(TelemetryConfig),
    ConfigRead(ConfigGroupId, ConfigResponseSender),
    ConfigWrite(ConfigWrite, ConfigResponseSender),
    /// Erase every stored config group (factory reset).
    ConfigResetAll(ConfigResponseSender),
    Detect(DetectRequest, DetectResponseSender),
    /// Query or clear device faults (`FaultEndpoint`).
    Fault(FaultRequest, FaultResponseSender),
}

/// Sender half of the host→backend command channel. The `ops` helpers take
/// this rather than the whole [`HostRuntime`] so they can run from any thread
/// with a cheap clone — in particular off the GUI's runtime mutex, so a long
/// detection sequence never blocks the UI's Stop/Coast handlers.
pub type CommandSender = tokio::sync::mpsc::UnboundedSender<HostCommand>;

pub struct HostRuntime {
    pub fast_rx: Receiver<FastTelemetry>,
    pub slow_rx: Receiver<SlowTelemetry>,
    /// Fault snapshots pushed by the device on every registry change
    /// (FaultTopic). Full snapshots, not deltas — safe to miss one.
    pub fault_rx: Receiver<FaultResponse>,
    pub device_info_rx: Receiver<HardwareInfo>,
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
            thread::sleep(Duration::from_millis(50));
        }
        self.connected.load(Ordering::Relaxed)
    }

    pub fn shutdown(&self) {
        info!("Shutting down host backend...");
        self.cancel_token.cancel();
    }
}

impl Drop for HostRuntime {
    /// Cancel the backend on drop: replacing the runtime slot on a GUI
    /// reconnect must not leak the old tokio runtime + thread (which would
    /// keep holding the serial port / probe). Idempotent with `shutdown()`.
    fn drop(&mut self) {
        self.cancel_token.cancel();
    }
}

// ── Backend context ──────────────────────────────────────────────────────────

/// Shared state passed through the backend instead of many individual arguments.
struct BackendCtx {
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    fault_tx: Sender<FaultResponse>,
    info_tx: Sender<HardwareInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected: Arc<AtomicBool>,
    fast_hz: Arc<AtomicU16>,
    cancel: CancellationToken,
}

// ── Entry point ──────────────────────────────────────────────────────────────

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (fast_tx, fast_rx) = crossbeam_channel::bounded::<FastTelemetry>(4096);
    let (slow_tx, slow_rx) = crossbeam_channel::bounded::<SlowTelemetry>(64);
    let (fault_tx, fault_rx) = crossbeam_channel::bounded::<FaultResponse>(64);
    let (info_tx, device_info_rx) = crossbeam_channel::bounded::<HardwareInfo>(4);
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let fast_hz = Arc::new(AtomicU16::new(0));
    let cancel_token = CancellationToken::new();

    let ctx = BackendCtx {
        fast_tx,
        slow_tx,
        fault_tx,
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
        fault_rx,
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
        #[cfg(feature = "desktop")]
        TransportConfig::Serial { path, baud } => {
            run_cobs_stream_with_reconnect(
                move || {
                    let path = path.clone();
                    async move { transport::serial::connect(&path, baud) }
                },
                &cfg,
                ctx,
            )
            .await
        }
        #[cfg(feature = "desktop")]
        TransportConfig::Rtt { probe, chip } => {
            let elf = resolve_elf_path(&cfg)?;
            run_cobs_stream_with_reconnect(
                move || {
                    let probe = probe.clone();
                    let chip = chip.clone();
                    let elf = elf.clone();
                    async move { transport::rtt::connect(probe.as_deref(), &chip, Some(&elf)) }
                },
                &cfg,
                ctx,
            )
            .await
        }
        TransportConfig::Udp { host, port } => {
            let state_notify = Arc::new(ergot::toolkits::tokio_stream::WaitQueue::new());
            let (stack, queue) = transport::udp::new_stack(ERGOT_QUEUE_SIZE, ERGOT_MTU);
            let register_fn = {
                let stack = stack.clone();
                let notify = state_notify.clone();
                async move || {
                    transport::udp::register(&stack, &queue, &host, port, Some(notify.clone()))
                        .await
                }
            };
            run_framed_with_reconnect(stack, register_fn, state_notify, &cfg, ctx).await
        }
        TransportConfig::Usb => {
            let state_notify = Arc::new(ergot::toolkits::tokio_stream::WaitQueue::new());
            let (stack, queue) = transport::usb::new_stack();
            let register_fn = {
                let stack = stack.clone();
                let notify = state_notify.clone();
                async move || transport::usb::register(&stack, &queue, Some(notify.clone())).await
            };
            run_framed_with_reconnect(stack, register_fn, state_notify, &cfg, ctx).await
        }
        TransportConfig::Ble { device } => {
            let state_notify = Arc::new(ergot::toolkits::tokio_stream::WaitQueue::new());
            let (stack, queue) = transport::ble::new_stack();
            let register_fn = {
                let stack = stack.clone();
                let notify = state_notify.clone();
                let mut workers = Vec::new();
                async move || {
                    transport::ble::register(
                        &stack,
                        &queue,
                        &device,
                        Some(notify.clone()),
                        &mut workers,
                    )
                    .await
                }
            };
            run_framed_with_reconnect(stack, register_fn, state_notify, &cfg, ctx).await
        }
    }
}

// ── Framed transport runner (UDP, USB, BLE) ──────────────────────────────────

/// Run a packet-framed transport with the same reconnect loop the COBS
/// stream path uses: the stack lives forever, `register_fn` attaches a fresh
/// connection onto it for every attempt (it only succeeds while the
/// interface is Down — i.e. after a teardown), and a failed handshake or a
/// dead interface tears down and retries per the configured policy.
async fn run_framed_with_reconnect<NS, I>(
    stack: NS,
    mut register_fn: impl AsyncFnMut() -> Result<()>,
    state_notify: Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    cfg: &HostConfig,
    ctx: BackendCtx,
) -> Result<()>
where
    // Concrete DirectEdge profile (not just `Profile`): teardown() is an
    // inherent method on it, and all framed stacks (UDP/USB/BLE) are
    // DirectEdge targets.
    NS: NetStackHandle<Profile = ergot::interface_manager::profiles::direct_edge::DirectEdge<I>>
        + Clone
        + Send
        + Sync
        + 'static,
    I: ergot::interface_manager::Interface,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    let connected = ctx.connected.clone();
    let fast_hz_flag = ctx.fast_hz.clone();
    let cancel = ctx.cancel.clone();
    let info_tx = ctx.info_tx.clone();

    // Protocol tasks are spawned once — they operate on the stack, not the
    // per-connection interface.
    spawn_protocol_tasks(&stack, ctx);

    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;

    loop {
        let reg_result = tokio::select! {
            r = register_fn() => r,
            _ = cancel.cancelled() => break,
        };

        if let Err(e) = reg_result {
            connect_attempts += 1;
            tracing::warn!(
                "Framed transport connect failed (attempt {}): {:?}",
                connect_attempts,
                e
            );

            match policy {
                ReconnectPolicy::None => {
                    info!("Reconnect policy: none — giving up");
                    break;
                }
                ReconnectPolicy::Limited(max) if connect_attempts >= max => {
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
        connect_attempts = 0;

        // Wait for the interface to become Active — bounded like the
        // recovery path: a half-open link that registered but never goes
        // Active would otherwise pin the connect path until cancel.
        let became_active = tokio::select! {
            r = tokio::time::timeout(RECOVERY_TIMEOUT, wait_for_active(&state_notify, &stack)) => r.is_ok(),
            _ = cancel.cancelled() => break,
        };
        if !became_active {
            tracing::warn!("Interface not Active within {RECOVERY_TIMEOUT:?}, reconnecting...");
            stack
                .stack()
                .manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // HardwareInfo handshake on each (re)connection
        let handshake_ok = hardware_info_handshake(&stack, &info_tx, &connected).await;

        if !handshake_ok {
            tracing::warn!("Handshake failed, reconnecting...");
            connected.store(false, Ordering::Relaxed);
            stack
                .stack()
                .manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => {}
                _ = cancel.cancelled() => break,
            }
            continue;
        }

        enable_fast_telemetry(&stack, cfg.fast_hz(), &fast_hz_flag).await;

        if !defmt_started && cfg.stream_defmt() {
            // defmt decoding is a debugging nicety — a missing/unreadable ELF
            // (e.g. connecting to the virtual device with no firmware build)
            // must not kill an already-established connection.
            match start_defmt_decoder(cfg, &stack, None) {
                Ok(()) => defmt_started = true,
                Err(e) => {
                    tracing::warn!("defmt decoder unavailable, continuing without it: {e:#}");
                }
            }
        }

        // Monitor interface state with recovery
        let disconnected =
            monitor_state_with_recovery(&state_notify, &stack, &connected, &cancel).await;

        if !disconnected {
            // Cancelled
            break;
        }

        // Tear down the old interface so workers release the connection
        info!("Calling teardown...");
        stack
            .stack()
            .manage_profile(ergot::prelude::DirectEdge::teardown);
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

// ── COBS stream with reconnection ────────────────────────────────────────────

async fn run_cobs_stream_with_reconnect<F, Fut>(
    connect_fn: F,
    cfg: &HostConfig,
    ctx: BackendCtx,
) -> Result<()>
where
    F: Fn() -> Fut,
    Fut: Future<Output = Result<transport::CobsStreamTransport>>,
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
                            ReconnectPolicy::None => {
                                info!("Reconnect policy: none — giving up");
                                break;
                            }
                            ReconnectPolicy::Limited(max) if connect_attempts >= max => {
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
            EdgeFrameProcessor::new(),
            InterfaceState::Active {
                net_id: 0,
                node_id: EDGE_NODE_ID,
            },
            Some(LivenessConfig {
                timeout_ms: LIVENESS_TIMEOUT_MS,
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

        // Wait for the interface to become Active — bounded, same rationale
        // as the framed path above.
        let became_active = tokio::select! {
            r = tokio::time::timeout(RECOVERY_TIMEOUT, wait_for_active(&state_notify, &stack)) => r.is_ok(),
            _ = cancel.cancelled() => break,
        };
        if !became_active {
            tracing::warn!("Interface not Active within {RECOVERY_TIMEOUT:?}, reconnecting...");
            stack.manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // HardwareInfo handshake on each (re)connection
        let handshake_ok = hardware_info_handshake(&stack, &info_tx, &connected).await;

        if !handshake_ok {
            tracing::warn!("Handshake failed, reconnecting...");
            connected.store(false, Ordering::Relaxed);
            stack.manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => {}
                _ = cancel.cancelled() => break,
            }
            continue;
        }

        enable_fast_telemetry(&stack, cfg.fast_hz(), &fast_hz_flag).await;

        if !defmt_started && cfg.stream_defmt() {
            // Non-fatal: see the framed-transport path. Retried on the next
            // reconnect in case the ELF appears after a firmware build.
            match start_defmt_decoder(cfg, &stack, transport.defmt_reader) {
                Ok(()) => defmt_started = true,
                Err(e) => {
                    tracing::warn!("defmt decoder unavailable, continuing without it: {e:#}");
                }
            }
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
        stack.manage_profile(ergot::prelude::DirectEdge::teardown);
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

/// Monitor interface state with recovery support.
/// Returns `true` if disconnected (needs reconnect), `false` if cancelled.
async fn monitor_state_with_recovery<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
    connected: &Arc<AtomicBool>,
    cancel: &CancellationToken,
) -> bool
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    loop {
        tokio::select! {
            _ = state_notify.wait() => {
                let state = stack.stack().manage_profile(|im| im.interface_state(()));
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

/// Run HardwareInfo handshake with retries and exponential backoff.
/// Returns `true` on success.
async fn hardware_info_handshake<NS>(
    stack: &NS,
    info_tx: &Sender<HardwareInfo>,
    connected: &Arc<AtomicBool>,
) -> bool
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let ns = stack.stack();
    let mut backoff = Duration::from_millis(100);
    for attempt in 1..=10u32 {
        let fut =
            ns.endpoints()
                .request::<HardwareInfoEndpoint>(DEVICE_ADDR, &(), Some("hardware_info"));
        match tokio::time::timeout(HANDSHAKE_TIMEOUT, fut).await {
            Ok(Ok(dev_info)) => {
                info!(
                    "Device connected: hw='{}' sw='{}' mcu='{}' uuid='{}' foc={}Hz max_i={}A proto=v{}",
                    dev_info.hw.as_str(),
                    dev_info.sw.as_str(),
                    dev_info.mcu.as_str(),
                    dev_info.uuid.as_str(),
                    dev_info.foc_freq_hz,
                    dev_info.max_current_a,
                    dev_info.proto_version,
                );
                if dev_info.proto_version != oxifoc_core::types::ICD_PROTO_VERSION {
                    tracing::warn!(
                        "PROTOCOL VERSION MISMATCH: device proto v{}, host proto v{} — update \
                         whichever is older; some endpoints/topics may not route or may be \
                         misinterpreted",
                        dev_info.proto_version,
                        oxifoc_core::types::ICD_PROTO_VERSION,
                    );
                }
                // try_send: the consumer may have stopped reading (the GUI's
                // info listener reads exactly one message). A blocking send
                // on this bounded channel would wedge the whole backend after
                // a few reconnects. Dropping a handshake info is harmless.
                let _ = info_tx.try_send(dev_info);
                connected.store(true, Ordering::Relaxed);
                return true;
            }
            Ok(Err(e)) => {
                tracing::warn!("HardwareInfo attempt {} failed: {:?}", attempt, e);
            }
            Err(_) => {
                tracing::warn!("HardwareInfo attempt {} timed out", attempt);
            }
        }
        tokio::time::sleep(backoff).await;
        backoff = (backoff * 2).min(Duration::from_secs(2));
    }
    tracing::warn!("Device info not received after retries; giving up — caller will reconnect");
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
    let client = stack.clone().reliable::<TokioTimer>();
    match client
        .at_least_once::<TelemetryConfigEndpoint>(DEVICE_ADDR, &telem_cfg, None, &SETPOINT_POLICY)
        .await
    {
        Ok(ack) => {
            info!(
                "Telemetry enabled: requested={}Hz, actual={}Hz",
                fast_hz, ack.actual_fast_hz
            );
            fast_hz_flag.store(ack.actual_fast_hz, Ordering::Relaxed);
        }
        Err(e) => tracing::warn!("Telemetry config failed: {:?}", e),
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
    spawn_fast_telemetry_subscriber(stack, ctx.fast_tx, ctx.cancel.clone());
    spawn_fault_topic_subscriber(stack, ctx.fault_tx, ctx.cancel.clone());
    spawn_slow_telemetry_poller(
        stack,
        ctx.slow_tx,
        ctx.connected.clone(),
        ctx.cancel.clone(),
    );

    // Command handler + command-staleness affirmation, in ONE task so every
    // send is strictly ordered: an affirm of the previous setpoint can never
    // be in flight concurrently with a fresh command (in particular a Stop)
    // and land after it on the wire.
    //
    // Affirmation: while connected with a drive setpoint active, resend it
    // every AFFIRM_INTERVAL so the device's ISR deadman stays fed (absent
    // that, the device fail-safes after ~150 ms). The setpoint is idempotent
    // (MotorEndpoint is `Idempotent`), so re-sending the same absolute value
    // is safe. Fire-and-forget (no retry).
    //
    // On disconnect the setpoint is dropped: a reconnect must never resurrect
    // a stale throttle — the device latches its failsafe and waits for an
    // explicit Stopped + fresh user intent, and we must not fight that.
    //
    // Known trade-off of the single task: a long-running command (a 60 s
    // Detect, a config op on a struggling link) starves affirms, so the
    // device fail-safes. That is the safe direction — those ops don't run
    // while riding (the device refuses them with the motor running), and a
    // link bad enough to stall a 2 s setpoint send *should* trip the deadman.
    tokio::spawn({
        let stack = stack.clone();
        let fast_hz_flag = ctx.fast_hz;
        let connected = ctx.connected.clone();
        let cancel = ctx.cancel.clone();
        let mut cmd_rx = ctx.cmd_rx;
        async move {
            let mut active_setpoint: Option<ControlMode> = None;
            let mut ticker = tokio::time::interval(AFFIRM_INTERVAL);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    cmd = cmd_rx.recv() => {
                        let Some(cmd) = cmd else { break };
                        handle_command(&stack, cmd, &fast_hz_flag, &mut active_setpoint).await;
                    }
                    _ = ticker.tick() => {
                        if !connected.load(Ordering::Relaxed) {
                            if active_setpoint.take().is_some() {
                                tracing::info!(
                                    "link down: dropping active setpoint (no auto-resume on reconnect)"
                                );
                            }
                            continue;
                        }
                        if let Some(mode) = active_setpoint {
                            let client = stack.clone().reliable::<TokioTimer>();
                            let _ = client
                                .at_least_once::<MotorEndpoint>(
                                    DEVICE_ADDR,
                                    &mode,
                                    Some("affirm"),
                                    &AFFIRM_POLICY,
                                )
                                .await;
                        }
                    }
                }
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
                // N=256 capacity: the device batch size must be ≤ this or the
                // batch fails to deserialize (DeserFailed). Generous headroom so
                // growing the device batch (for bigger MTUs) needs no host change.
                // The topic KEY is N-independent (for_path uses the default N),
                // so routing still matches a device sending any batch size.
                .heap_bounded_receiver::<FastTelemetryTopic<256>>(128, Some("fast_telem"));
            let mut pinned = pin!(receiver);
            let mut hdl = pinned.as_mut().subscribe();
            info!("Fast telemetry subscriber started");
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    msg = hdl.recv() => {
                        for sample in &msg.t.samples {
                            // try_send: drop-on-full is the right semantics
                            // for telemetry. A blocking send would park this
                            // tokio worker whenever the UI stops draining
                            // (e.g. minimized window stops the render loop).
                            let _ = fast_tx.try_send(*sample);
                        }
                    }
                }
            }
        }
    });
}

fn spawn_fault_topic_subscriber<NS>(
    stack: &NS,
    fault_tx: Sender<FaultResponse>,
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
                .heap_bounded_receiver::<FaultTopic>(16, Some("faults"));
            let mut pinned = pin!(receiver);
            let mut hdl = pinned.as_mut().subscribe();
            info!("Fault topic subscriber started");
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    msg = hdl.recv() => {
                        // try_send: snapshots are self-contained, dropping
                        // one on a full queue only delays the view until
                        // the next event.
                        let _ = fault_tx.try_send(msg.t.clone());
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
                            // try_send: see fast telemetry — never block the
                            // runtime on a slow/absent consumer.
                            let _ = slow_tx.try_send(sample);
                        }
                    }
                }
            }
        }
    });
}

async fn handle_command<NS>(
    ns: &NS,
    cmd: HostCommand,
    fast_hz_flag: &Arc<AtomicU16>,
    active_setpoint: &mut Option<ControlMode>,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send,
{
    let client = ns.clone().reliable::<TokioTimer>();
    match cmd {
        HostCommand::Motor(ref mc) => {
            // Track the active drive setpoint for the affirmation tick. A
            // running mode must keep being re-affirmed to hold off the device
            // deadman; Stopped/Coast/Brake clear it (safe standing states,
            // exempt from the device deadman — no affirmation owed).
            *active_setpoint = if matches!(
                mc,
                ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
            ) {
                None
            } else {
                Some(*mc)
            };
            tracing::info!("Sending motor command: {:?}", mc);
            match client
                .at_least_once::<MotorEndpoint>(DEVICE_ADDR, mc, Some("motor"), &SETPOINT_POLICY)
                .await
            {
                Ok(status) => tracing::info!("Motor response: {:?}", status),
                Err(e) => tracing::warn!("Motor command failed: {:?}", e),
            }
        }
        HostCommand::MotorAck(ref mc, reply_tx) => {
            // Same as Motor (incl. the affirmation tracking), but the caller
            // gets the device status / delivery error back — a CLI must not
            // print "sent" and exit 0 when nothing was delivered.
            *active_setpoint = if matches!(
                mc,
                ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
            ) {
                None
            } else {
                Some(*mc)
            };
            tracing::info!("Sending motor command (acked): {:?}", mc);
            let res = client
                .at_least_once::<MotorEndpoint>(DEVICE_ADDR, mc, Some("motor"), &SETPOINT_POLICY)
                .await
                .map_err(|e| anyhow::anyhow!("{e:?}"));
            // A failed drive command must not keep being affirmed.
            if res.is_err() {
                *active_setpoint = None;
            }
            let _ = reply_tx.send(res);
        }
        HostCommand::SetPhaseSource(source) => {
            tracing::info!("Setting phase source: {:?}", source);
            match client
                .at_least_once::<PhaseSourceEndpoint>(
                    DEVICE_ADDR,
                    &source,
                    Some("phase_source"),
                    &SETPOINT_POLICY,
                )
                .await
            {
                Ok(ack) => tracing::info!("Phase source response: {:?}", ack),
                Err(e) => tracing::warn!("Phase source command failed: {:?}", e),
            }
        }
        HostCommand::SetTelemetryConfig(cfg) => {
            tracing::info!("Setting telemetry config: {:?}", cfg);
            match client
                .at_least_once::<TelemetryConfigEndpoint>(
                    DEVICE_ADDR,
                    &cfg,
                    Some("telemetry_config"),
                    &SETPOINT_POLICY,
                )
                .await
            {
                Ok(ack) => {
                    tracing::info!("Telemetry config ack: fast={}Hz", ack.actual_fast_hz);
                    fast_hz_flag.store(ack.actual_fast_hz, Ordering::Relaxed);
                }
                Err(e) => tracing::warn!("Telemetry config failed: {:?}", e),
            }
        }
        HostCommand::ConfigRead(group_id, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Reading config group: {:?}", group_id);
            let req = ConfigRequest::Read(group_id);
            let res = client
                .at_least_once::<ConfigEndpoint>(
                    DEVICE_ADDR,
                    &req,
                    Some("config"),
                    &SETPOINT_POLICY,
                )
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{e:?}")));
        }
        HostCommand::ConfigWrite(write, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Writing config: {:?}", write);
            let req = ConfigRequest::Write(write);
            let res = client
                .at_least_once::<ConfigEndpoint>(
                    DEVICE_ADDR,
                    &req,
                    Some("config"),
                    &SETPOINT_POLICY,
                )
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{e:?}")));
        }
        HostCommand::ConfigResetAll(reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Resetting all config to defaults");
            let res = client
                .at_least_once::<ConfigEndpoint>(
                    DEVICE_ADDR,
                    &ConfigRequest::ResetAll,
                    Some("config"),
                    &SETPOINT_POLICY,
                )
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{e:?}")));
        }
        HostCommand::Fault(req, reply_tx) => {
            tracing::info!("Fault request: {:?}", req);
            let res = client
                .at_least_once::<FaultEndpoint>(DEVICE_ADDR, &req, Some("fault"), &SETPOINT_POLICY)
                .await;
            let _ = reply_tx.send(res.map_err(|e| anyhow::anyhow!("{e:?}")));
        }
        HostCommand::Detect(req, reply_tx) => {
            tracing::info!("Starting motor detection: {:?}", req);
            // Detection runs up to ~60 s — spawn it off the command task so
            // a queued Stop (and the deadman affirmations) are not stuck
            // behind it. The device refuses to start detection with the
            // motor running, so the lost strict ordering is harmless.
            let client = ns.clone().reliable::<TokioTimer>();
            tokio::spawn(async move {
                // Effectively-once: a stable id across retries. If the (slow)
                // response is lost, a retry returns the device's cached
                // result instead of re-running characterization.
                let keyed = Keyed::new(next_detect_id(), req);
                let res = client
                    .effectively_once::<DetectEndpoint>(
                        DEVICE_ADDR,
                        &keyed,
                        Some("detect"),
                        &DETECT_POLICY,
                    )
                    .await;
                let result = res.map_err(|e| anyhow::anyhow!("{e:?}"));
                let _ = reply_tx.send(result);
            });
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
    let elf_path = resolve_elf_path(cfg)?;
    let elf_bytes =
        fs::read(&elf_path).with_context(|| format!("Failed to read ELF at {elf_path}"))?;
    let table = Table::parse(&elf_bytes)
        .context("Parsing defmt table from ELF failed")?
        .ok_or_else(|| anyhow::anyhow!("No .defmt section in ELF; build device with defmt"))?;

    if let Some(mut defmt_rx) = defmt_reader {
        // RTT mode: read defmt frames directly from RTT channel 0
        info!("Starting defmt decoder (RTT mode - channel 0)");
        let (tx, rx) = crossbeam_channel::bounded::<Vec<u8>>(64);

        thread::spawn(move || {
            let mut stream = table.new_stream_decoder();
            while let Ok(data) = rx.recv() {
                stream.received(&data);
                loop {
                    match stream.decode() {
                        Ok(frame) => {
                            log_defmt_frame(frame.level(), &frame.display(false));
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
                            log_defmt_frame(frame.level(), &frame.display(false));
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

fn log_defmt_frame(level: Option<DefmtLevel>, msg: &impl std::fmt::Display) {
    match level {
        Some(DefmtLevel::Trace) => tracing::trace!(target: "device", "{}", msg),
        Some(DefmtLevel::Debug) => tracing::debug!(target: "device", "{}", msg),
        Some(DefmtLevel::Info) => tracing::info!(target: "device", "{}", msg),
        Some(DefmtLevel::Warn) => tracing::warn!(target: "device", "{}", msg),
        Some(DefmtLevel::Error) => tracing::error!(target: "device", "{}", msg),
        None => tracing::info!(target: "device", "{}", msg),
    }
}

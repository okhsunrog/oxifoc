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
    ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, DetectRequest, DetectResponse,
    FaultRequest, FaultSnapshot, HardwareInfo, MotorCommand, MotorRequest, MotorStatus,
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
use tracing::{error, info, warn};

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
/// Cadence at which the active drive setpoint is re-affirmed to the device.
const AFFIRM_INTERVAL: Duration = Duration::from_millis(50);
/// How long a detached motor-response observer waits before declaring the
/// response lost (see [`send_motor_now`] — the SEND has already happened by
/// then; this bounds only the reply bookkeeping).
const MOTOR_RESPONSE_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HandshakeOutcome {
    Connected,
    RetryableFailure,
    FatalMismatch,
}

/// Record a failed connection generation and decide whether another one is
/// permitted. `Limited(n)` preserves the existing meaning: at most `n`
/// consecutive failed generations, including the one that just failed.
fn may_retry(policy: ReconnectPolicy, failures: &mut u32) -> bool {
    *failures = failures.saturating_add(1);
    match policy {
        ReconnectPolicy::None => false,
        ReconnectPolicy::Limited(max) => *failures < max,
        ReconnectPolicy::Infinite => true,
    }
}

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

fn new_drive_session() -> u64 {
    static CTR: AtomicU64 = AtomicU64::new(0);
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    seed ^ CTR.fetch_add(1, Ordering::Relaxed).rotate_left(29)
}

fn next_motor_request(session: u64, seq: &mut u32, command: MotorCommand) -> MotorRequest {
    let request = MotorRequest {
        source_session: session,
        seq: *seq,
        command,
    };
    *seq = seq.wrapping_add(1);
    request
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
        // CLI stdout is a stable data channel (`--json` emits exactly one
        // document); diagnostics and connection logs belong on stderr.
        .with_writer(std::io::stderr)
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
    /// Immediate high-Z stop through the existing motor endpoint. The device
    /// latches re-arm until a later safe mode command.
    EmergencyStop(MotorResponseSender),
    SetPhaseSource(PhaseSource),
    SetTelemetryConfig(TelemetryConfig),
    ConfigRead(ConfigGroupId, ConfigResponseSender),
    ConfigApply(Keyed<ConfigApply>, ConfigResponseSender),
    ConfigPersist(Keyed<ConfigPersist>, ConfigResponseSender),
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
    /// Full fault snapshots from FaultTopic plus generation-based query
    /// reconciliation when a topic or host handoff was lost.
    pub fault_rx: Receiver<FaultSnapshot>,
    pub device_info_rx: Receiver<HardwareInfo>,
    pub cmd_tx: tokio::sync::mpsc::UnboundedSender<HostCommand>,
    pub connected: Arc<AtomicBool>,
    pub fast_hz: Arc<AtomicU16>,
    cancel_token: CancellationToken,
    /// Backend thread handle — joined (bounded) on shutdown so the process
    /// never exits while the RTT I/O thread is mid-USB-transaction (which
    /// wedges the ST-Link for the next open). Mutex<Option<..>> so both
    /// `shutdown(&self)` and `Drop` can take it exactly once.
    backend_thread: std::sync::Mutex<Option<thread::JoinHandle<()>>>,
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
        self.join_backend(Duration::from_secs(3));
    }

    /// Wait (bounded) for the backend thread to finish its shutdown path —
    /// in particular the RTT I/O thread join that closes the probe-rs
    /// session cleanly. Skipping this and exiting the process kills that
    /// thread mid-USB-transaction and wedges the ST-Link for the next open.
    fn join_backend(&self, timeout: Duration) {
        let handle = self
            .backend_thread
            .lock()
            .expect("backend thread slot poisoned")
            .take();
        let Some(h) = handle else { return };
        let deadline = Instant::now() + timeout;
        while !h.is_finished() && Instant::now() < deadline {
            thread::sleep(Duration::from_millis(10));
        }
        if h.is_finished() {
            let _ = h.join();
        } else {
            warn!("backend thread did not finish within {timeout:?}; exiting anyway");
        }
    }
}

/// Build the raw→engineering enrichment context from the device: `BoardCalib`
/// (from the handshake `HardwareInfo`) + `dc_offsets` and `pole_pairs` (config
/// reads on `cmd`). Offsets fall back to mid-scale and `pole_pairs` to 0 when the
/// device stores neither (uncalibrated / virtual). A free function over the
/// command sender (not `&HostRuntime`) so the GUI can clone `cmd_tx` out of its
/// runtime mutex first — never holding the lock across the blocking config
/// reads. Shared by the CLI (record/watch) and the GUI so both enrich through the
/// identical core path.
pub fn build_enrich_ctx(
    cmd: &CommandSender,
    hw: Option<&HardwareInfo>,
) -> Option<oxifoc_core::foc::telemetry::EnrichCtx> {
    use oxifoc_core::types::{ConfigGroupId, ConfigValue};
    let calib = hw?.calib;
    // Fallbacks below are LOUD: a transient config-read failure (busy link,
    // reconnect window) silently degrading to mid-scale offsets skews every
    // reconstructed phase current — measured ~15 A/phase on the g431 — while
    // the columns still look plausible. The warn is the only trace.
    let offsets = ops::config::read_group(cmd, ConfigGroupId::DcOffsets)
        .ok()
        .and_then(|snapshot| match snapshot.value {
            Some(ConfigValue::DcOffsets(c)) => Some((c.phase_a, c.phase_b, c.phase_c)),
            _ => None,
        })
        .unwrap_or_else(|| {
            let mid = f32::from(calib.adc_max_counts) / 2.0;
            warn!(
                "enrich: DcOffsets group unavailable — falling back to mid-scale ({mid}); \
                 reconstructed currents may carry a large DC offset"
            );
            (mid, mid, mid)
        });
    let pole_pairs = ops::config::read_group(cmd, ConfigGroupId::MotorParams)
        .ok()
        .and_then(|snapshot| match snapshot.value {
            Some(ConfigValue::MotorParams(c)) => Some(c.pole_pairs),
            _ => None,
        })
        .unwrap_or_else(|| {
            warn!("enrich: MotorParams group unavailable — pole_pairs=0, erpm column will be 0");
            0
        });
    Some(oxifoc_core::foc::telemetry::EnrichCtx::new(
        &calib, offsets, pole_pairs,
    ))
}

impl Drop for HostRuntime {
    /// Cancel the backend on drop: replacing the runtime slot on a GUI
    /// reconnect must not leak the old tokio runtime + thread (which would
    /// keep holding the serial port / probe). Idempotent with `shutdown()`.
    /// Also joins the backend (bounded) so a plain drop-without-shutdown
    /// still closes the probe session cleanly before the process moves on.
    fn drop(&mut self) {
        self.cancel_token.cancel();
        self.join_backend(Duration::from_secs(3));
    }
}

// ── Backend context ──────────────────────────────────────────────────────────

/// Shared state passed through the backend instead of many individual arguments.
struct BackendCtx {
    fast_tx: Sender<FastTelemetry>,
    slow_tx: Sender<SlowTelemetry>,
    fault_sink: FaultSnapshotSink,
    info_tx: Sender<HardwareInfo>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<HostCommand>,
    connected: Arc<AtomicBool>,
    fast_hz: Arc<AtomicU16>,
    cancel: CancellationToken,
}

/// Ordered, loss-aware handoff from protocol tasks to synchronous consumers.
/// Topic delivery and SlowTelemetry reconciliation can race; the mutex makes
/// it impossible for an older snapshot to overwrite a newer one.
#[derive(Clone)]
struct FaultSnapshotSink {
    tx: Sender<FaultSnapshot>,
    delivered_generation: Arc<std::sync::Mutex<Option<u32>>>,
}

impl FaultSnapshotSink {
    fn new(tx: Sender<FaultSnapshot>) -> Self {
        Self {
            tx,
            delivered_generation: Arc::new(std::sync::Mutex::new(None)),
        }
    }

    fn offer(&self, snapshot: FaultSnapshot) {
        let mut delivered = self.delivered_generation.lock().unwrap();
        if delivered.is_some_and(|current| !generation_is_newer(snapshot.generation, current)) {
            return;
        }
        if self.tx.try_send(snapshot.clone()).is_ok() {
            *delivered = Some(snapshot.generation);
        }
    }

    fn needs(&self, generation: u32) -> bool {
        self.delivered_generation
            .lock()
            .unwrap()
            .is_none_or(|current| generation_is_newer(generation, current))
    }

    fn reset_generation(&self) {
        *self.delivered_generation.lock().unwrap() = None;
    }
}

fn generation_is_newer(candidate: u32, current: u32) -> bool {
    let distance = candidate.wrapping_sub(current);
    distance != 0 && distance < (1 << 31)
}

// ── Entry point ──────────────────────────────────────────────────────────────

pub fn start_host(cfg: HostConfig) -> HostRuntime {
    let (fast_tx, fast_rx) = crossbeam_channel::bounded::<FastTelemetry>(4096);
    let (slow_tx, slow_rx) = crossbeam_channel::bounded::<SlowTelemetry>(64);
    let (fault_tx, fault_rx) = crossbeam_channel::bounded::<FaultSnapshot>(64);
    let (info_tx, device_info_rx) = crossbeam_channel::bounded::<HardwareInfo>(4);
    let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel::<HostCommand>();
    let connected = Arc::new(AtomicBool::new(false));
    let fast_hz = Arc::new(AtomicU16::new(0));
    let cancel_token = CancellationToken::new();

    let ctx = BackendCtx {
        fast_tx,
        slow_tx,
        fault_sink: FaultSnapshotSink::new(fault_tx),
        info_tx,
        cmd_rx,
        connected: connected.clone(),
        fast_hz: fast_hz.clone(),
        cancel: cancel_token.clone(),
    };

    let backend_thread = thread::spawn(move || {
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
        backend_thread: std::sync::Mutex::new(Some(backend_thread)),
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
                let mut selected_identity = None;
                async move || {
                    transport::usb::register(
                        &stack,
                        &queue,
                        Some(notify.clone()),
                        &mut selected_identity,
                    )
                    .await
                }
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
    NS::Target: Send + Sync,
{
    let connected = ctx.connected.clone();
    let fast_hz_flag = ctx.fast_hz.clone();
    let cancel = ctx.cancel.clone();
    let info_tx = ctx.info_tx.clone();
    let fault_sink = ctx.fault_sink.clone();

    // Protocol tasks are spawned once — they operate on the stack, not the
    // per-connection interface.
    spawn_protocol_tasks(&stack, ctx);

    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;
    let mut expected_device_uuid: Option<String> = None;

    loop {
        let reg_result = tokio::select! {
            r = register_fn() => r,
            _ = cancel.cancelled() => break,
        };

        if let Err(e) = reg_result {
            let retry = may_retry(policy, &mut connect_attempts);
            tracing::warn!(
                "Framed transport connect failed (attempt {}): {:?}",
                connect_attempts,
                e
            );
            if !retry {
                info!("Reconnect policy exhausted — giving up");
                break;
            }

            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }
        // Wait for the interface to become Active — bounded like the
        // recovery path: a half-open link that registered but never goes
        // Active would otherwise pin the connect path until cancel.
        let became_active = tokio::select! {
            r = tokio::time::timeout(RECOVERY_TIMEOUT, wait_for_active(&state_notify, &stack)) => matches!(r, Ok(true)),
            _ = cancel.cancelled() => break,
        };
        if !became_active {
            tracing::warn!("Interface not Active within {RECOVERY_TIMEOUT:?}, reconnecting...");
            connected.store(false, Ordering::Relaxed);
            stack
                .stack()
                .manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            if !may_retry(policy, &mut connect_attempts) {
                info!("Reconnect policy exhausted after interface activation failure");
                break;
            }
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // HardwareInfo handshake on each (re)connection
        let handshake =
            hardware_info_handshake(&stack, &info_tx, &connected, &mut expected_device_uuid).await;

        match handshake {
            HandshakeOutcome::Connected => {
                // Fault generation is boot-local. A controller reset may
                // reconnect with the same UUID and generation 0; force a
                // fresh snapshot instead of treating it as older.
                fault_sink.reset_generation();
                connect_attempts = 0;
            }
            HandshakeOutcome::FatalMismatch => {
                tracing::error!("Fatal handshake mismatch — refusing to reconnect");
                connected.store(false, Ordering::Relaxed);
                stack
                    .stack()
                    .manage_profile(ergot::prelude::DirectEdge::teardown);
                break;
            }
            HandshakeOutcome::RetryableFailure => {
                tracing::warn!("Handshake failed, reconnecting...");
                connected.store(false, Ordering::Relaxed);
                stack
                    .stack()
                    .manage_profile(ergot::prelude::DirectEdge::teardown);
                tokio::task::yield_now().await;
                if !may_retry(policy, &mut connect_attempts) {
                    info!("Reconnect policy exhausted after handshake failure");
                    break;
                }
                tokio::select! {
                    _ = tokio::time::sleep(RECONNECT_DELAY) => {}
                    _ = cancel.cancelled() => break,
                }
                continue;
            }
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

        if !may_retry(policy, &mut connect_attempts) {
            info!("Reconnect policy exhausted after connection loss");
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

    // See the identical sequence in the COBS backend below — the framed
    // backend never uses RTT today, but this is a no-op then and keeps the
    // shutdown contract uniform.
    #[cfg(feature = "desktop")]
    {
        stack
            .stack()
            .manage_profile(ergot::prelude::DirectEdge::teardown);
        tokio::time::sleep(Duration::from_millis(50)).await;
        transport::rtt::join_rtt_io_thread(Duration::from_secs(2));
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
    let fault_sink = ctx.fault_sink.clone();

    // Protocol tasks are spawned once — they operate on the stack, not the transport
    spawn_protocol_tasks(&stack, ctx);

    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;
    let mut expected_device_uuid: Option<String> = None;

    loop {
        // Try to connect
        let transport = tokio::select! {
            result = connect_fn() => {
                match result {
                    Ok(t) => t,
                    Err(e) => {
                        let retry = may_retry(policy, &mut connect_attempts);
                        tracing::warn!("Transport connect failed (attempt {}): {:?}", connect_attempts, e);
                        if !retry {
                            info!("Reconnect policy exhausted — giving up");
                            break;
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

        if let Err(e) = reg_result {
            tracing::warn!("Stream registration failed: {e:?}");
            connected.store(false, Ordering::Relaxed);
            stack.manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            if !may_retry(policy, &mut connect_attempts) {
                info!("Reconnect policy exhausted after stream registration failure");
                break;
            }
            tokio::select! {
                _ = tokio::time::sleep(Duration::from_secs(1)) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // Wait for the interface to become Active — bounded, same rationale
        // as the framed path above.
        let became_active = tokio::select! {
            r = tokio::time::timeout(RECOVERY_TIMEOUT, wait_for_active(&state_notify, &stack)) => matches!(r, Ok(true)),
            _ = cancel.cancelled() => break,
        };
        if !became_active {
            tracing::warn!("Interface not Active within {RECOVERY_TIMEOUT:?}, reconnecting...");
            connected.store(false, Ordering::Relaxed);
            stack.manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            if !may_retry(policy, &mut connect_attempts) {
                info!("Reconnect policy exhausted after interface activation failure");
                break;
            }
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }

        // HardwareInfo handshake on each (re)connection
        let handshake =
            hardware_info_handshake(&stack, &info_tx, &connected, &mut expected_device_uuid).await;

        match handshake {
            HandshakeOutcome::Connected => {
                fault_sink.reset_generation();
                connect_attempts = 0;
            }
            HandshakeOutcome::FatalMismatch => {
                tracing::error!("Fatal handshake mismatch — refusing to reconnect");
                connected.store(false, Ordering::Relaxed);
                stack.manage_profile(ergot::prelude::DirectEdge::teardown);
                break;
            }
            HandshakeOutcome::RetryableFailure => {
                tracing::warn!("Handshake failed, reconnecting...");
                connected.store(false, Ordering::Relaxed);
                stack.manage_profile(ergot::prelude::DirectEdge::teardown);
                tokio::task::yield_now().await;
                if !may_retry(policy, &mut connect_attempts) {
                    info!("Reconnect policy exhausted after handshake failure");
                    break;
                }
                tokio::select! {
                    _ = tokio::time::sleep(RECONNECT_DELAY) => {}
                    _ = cancel.cancelled() => break,
                }
                continue;
            }
        }

        enable_fast_telemetry(&stack, cfg.fast_hz(), &fast_hz_flag).await;

        let defmt_reader = transport.defmt_reader;
        let transport_scoped_defmt = defmt_reader.is_some();
        if should_start_defmt(cfg.stream_defmt(), defmt_started, transport_scoped_defmt) {
            // Non-fatal: see the framed-transport path. Retried on the next
            // reconnect in case the ELF appears after a firmware build. RTT
            // readers belong to one transport generation, so every successful
            // reconnect gets a fresh decoder; network subscriptions live on
            // the persistent stack and are started only once.
            match start_defmt_decoder(cfg, &stack, defmt_reader) {
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

        if !may_retry(policy, &mut connect_attempts) {
            info!("Reconnect policy exhausted after connection loss");
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

    // RTT only (no-op otherwise): wait for the blocking I/O thread to finish
    // its current USB transaction and drop the probe-rs Session cleanly.
    // Exiting the process with that thread mid-transfer wedges the ST-Link —
    // the next `open` then times out on GET_CURRENT_MODE (~alternating
    // connect failures on back-to-back CLI runs). Order matters: the thread
    // exits when the transport READER drops, and the reader lives inside the
    // ergot interface worker — tear the interface down first (releases the
    // transport), give the worker a beat to run, THEN join. Joining before
    // the teardown just times out and leaves the session drop racing process
    // exit (measured: probe-rs `session_drop` cut off mid-way).
    #[cfg(feature = "desktop")]
    {
        stack.manage_profile(ergot::prelude::DirectEdge::teardown);
        tokio::time::sleep(Duration::from_millis(50)).await;
        transport::rtt::join_rtt_io_thread(Duration::from_secs(2));
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
            observed = state_notify.wait_for_value(|| {
                let state = stack.stack().manage_profile(|im| im.interface_state(()));
                let is_active = interface_is_active(state);
                (is_active != connected.load(Ordering::Relaxed)).then_some(state)
            }) => {
                let Ok(state) = observed else {
                    connected.store(false, Ordering::Relaxed);
                    return true;
                };
                if interface_is_active(state) {
                    connected.store(true, Ordering::Relaxed);
                    info!("Interface active — device connected");
                    continue;
                }

                connected.store(false, Ordering::Relaxed);
                if matches!(state, Some(InterfaceState::Down) | None) {
                    tracing::warn!("Interface down, reconnecting transport...");
                    return true;
                }

                tracing::warn!("Interface inactive, waiting for recovery...");
                let recovered = matches!(
                    tokio::time::timeout(
                        RECOVERY_TIMEOUT,
                        wait_for_active(state_notify, stack),
                    ).await,
                    Ok(true),
                );
                if recovered {
                    info!("Connection recovered");
                    connected.store(true, Ordering::Relaxed);
                } else {
                    tracing::warn!("Recovery failed or timed out, reconnecting transport...");
                    return true;
                }
            }
            _ = cancel.cancelled() => return false,
        }
    }
}

/// Wait until the interface is Active.
fn interface_is_active(state: Option<InterfaceState>) -> bool {
    matches!(
        state,
        Some(InterfaceState::Active { .. } | InterfaceState::ActiveLocal { .. })
    )
}

/// Wait until the interface becomes Active, or report that it reached Down.
/// The condition is registered with the waiter, so a transition immediately
/// before this call cannot be lost.
async fn wait_for_active<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
) -> bool
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    state_notify
        .wait_for_value(|| {
            let state = stack.stack().manage_profile(|im| im.interface_state(()));
            if interface_is_active(state) {
                Some(true)
            } else if matches!(state, Some(InterfaceState::Down) | None) {
                Some(false)
            } else {
                None
            }
        })
        .await
        .unwrap_or(false)
}

// ── Device handshake & telemetry setup ───────────────────────────────────────

/// Run HardwareInfo handshake with retries and exponential backoff.
/// Protocol/bootstrap and identity mismatches are fatal: retrying the same
/// transport must never silently select or accept an incompatible controller.
async fn hardware_info_handshake<NS>(
    stack: &NS,
    info_tx: &Sender<HardwareInfo>,
    connected: &Arc<AtomicBool>,
    expected_device_uuid: &mut Option<String>,
) -> HandshakeOutcome
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
                if dev_info.bootstrap_magic != oxifoc_core::types::ICD_BOOTSTRAP_MAGIC {
                    tracing::error!(
                        "INVALID PROTOCOL BOOTSTRAP: device magic={:?}, expected={:?}",
                        dev_info.bootstrap_magic,
                        oxifoc_core::types::ICD_BOOTSTRAP_MAGIC,
                    );
                    connected.store(false, Ordering::Relaxed);
                    return HandshakeOutcome::FatalMismatch;
                }
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
                if !protocol_is_compatible(dev_info.proto_version) {
                    tracing::error!(
                        "PROTOCOL VERSION MISMATCH: device proto v{}, host proto v{} — refusing \
                         the connection; update whichever side is older",
                        dev_info.proto_version,
                        oxifoc_core::types::ICD_PROTO_VERSION,
                    );
                    connected.store(false, Ordering::Relaxed);
                    return HandshakeOutcome::FatalMismatch;
                }
                let device_uuid = dev_info.uuid.as_str();
                if let Some(expected) = expected_device_uuid.as_deref() {
                    if device_uuid != expected {
                        tracing::error!(
                            "DEVICE IDENTITY MISMATCH: connected uuid='{}', expected uuid='{}' — \
                             refusing to switch controllers during reconnect",
                            device_uuid,
                            expected,
                        );
                        connected.store(false, Ordering::Relaxed);
                        return HandshakeOutcome::FatalMismatch;
                    }
                } else {
                    *expected_device_uuid = Some(device_uuid.to_owned());
                }
                // Never block the backend if a consumer stops reading.
                let _ = info_tx.try_send(dev_info);
                connected.store(true, Ordering::Relaxed);
                return HandshakeOutcome::Connected;
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
    HandshakeOutcome::RetryableFailure
}

fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
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
    NS::Target: Send + Sync,
{
    spawn_fast_telemetry_subscriber(
        stack,
        ctx.fast_tx,
        ctx.connected.clone(),
        ctx.cancel.clone(),
    );
    spawn_fault_topic_subscriber(
        stack,
        ctx.fault_sink.clone(),
        ctx.connected.clone(),
        ctx.cancel.clone(),
    );
    spawn_slow_telemetry_poller(
        stack,
        ctx.slow_tx,
        ctx.fault_sink,
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
            let drive_session = new_drive_session();
            let mut motor_seq = 0u32;
            let mut active_setpoint: Option<ControlMode> = None;
            let mut ticker = tokio::time::interval(AFFIRM_INTERVAL);
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
            // Deadman-margin diagnostics (2026-07-06 trips): the device saw
            // ≥150 ms of command silence while the RTT writer thread showed a
            // matching inter-write gap — the frames stopped ARRIVING from
            // this task. Separate the two remaining suspects: a late tick
            // (task/runtime stall) vs a slow send (stack stall inside
            // at_least_once).
            let mut last_affirm_tick: Option<Instant> = None;
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    cmd = cmd_rx.recv() => {
                        let Some(cmd) = cmd else { break };
                        if !connected.load(Ordering::Relaxed) {
                            active_setpoint = None;
                            reject_unverified_command(cmd);
                            continue;
                        }
                        handle_command(
                            &stack,
                            cmd,
                            &fast_hz_flag,
                            &mut active_setpoint,
                            drive_session,
                            &mut motor_seq,
                        ).await;
                    }
                    _ = ticker.tick() => {
                        if let Some(prev) = last_affirm_tick {
                            let gap = prev.elapsed();
                            if gap > Duration::from_millis(80) {
                                tracing::warn!("affirm tick late: gap={gap:?}");
                            }
                        }
                        last_affirm_tick = Some(Instant::now());
                        if !connected.load(Ordering::Relaxed) {
                            if active_setpoint.take().is_some() {
                                tracing::info!(
                                    "link down: dropping active setpoint (no auto-resume on reconnect)"
                                );
                            }
                            continue;
                        }
                        if let Some(mode) = active_setpoint {
                            // Truly fire-and-forget: the send is committed by
                            // `send_request` before the response future is
                            // returned (ordered with commands —
                            // the destination socket name is "motor", the
                            // device's motor server; `Some("affirm")` once
                            // pointed at a nonexistent socket and every
                            // affirm was silently dropped). The response wait
                            // is detached and swallowed — awaiting it here
                            // (even with a 40-150 ms budget) let one slow
                            // round-trip delay the NEXT affirm past the
                            // device's 150 ms deadman. The device-side
                            // `stale_max_us` counter is the margin meter.
                            let request = next_motor_request(
                                drive_session,
                                &mut motor_seq,
                                MotorCommand::SetMode(mode),
                            );
                            match send_motor_now(&stack, request) {
                                Err(e) => {
                                    tracing::warn!("setpoint affirm send failed: {e:?}");
                                }
                                Ok(fut) => {
                                    tokio::spawn(async move {
                                        match tokio::time::timeout(Duration::from_secs(1), fut)
                                            .await
                                        {
                                            Ok(Ok(_)) => {}
                                            Ok(Err(e)) => {
                                                tracing::debug!("affirm response error: {e:?}");
                                            }
                                            Err(_) => {
                                                tracing::debug!("affirm response timed out");
                                            }
                                        }
                                    });
                                }
                            }
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
            let receiver = ns
                .topics()
                // Raw-Pod batches have one fixed capacity (FAST_BATCH_BYTES in
                // core) shared by both ends — no per-side sizing to mismatch.
                .heap_bounded_receiver::<FastTelemetryTopic>(128, Some("fast_telem"));
            let mut pinned = pin!(receiver);
            let mut hdl = pinned.as_mut().subscribe();
            info!("Fast telemetry subscriber started");
            // 1 Hz pipeline diagnostics: batches/samples that reached this
            // subscriber vs samples dropped on a full fast_tx. Attributes
            // capture loss to "above the socket" (batches missing here) or
            // "below" (channel drops) without a debugger.
            use std::time::{Duration, Instant};
            let mut batches: u64 = 0;
            let mut samples_ok: u64 = 0;
            let mut dropped: u64 = 0;
            let mut last_report = Instant::now();
            loop {
                tokio::select! {
                    _ = cancel.cancelled() => break,
                    msg = hdl.recv() => {
                        if !connected.load(Ordering::Relaxed) {
                            continue;
                        }
                        batches += 1;
                        for sample in msg.t.samples() {
                            // try_send: drop-on-full is the right semantics
                            // for telemetry. A blocking send would park this
                            // tokio worker whenever the UI stops draining
                            // (e.g. minimized window stops the render loop).
                            match fast_tx.try_send(sample) {
                                Ok(()) => samples_ok += 1,
                                Err(_) => dropped += 1,
                            }
                        }
                        if last_report.elapsed() >= Duration::from_secs(1) {
                            info!(
                                "fast_telem/s: batches={batches} samples={samples_ok} chan_drops={dropped}"
                            );
                            batches = 0;
                            samples_ok = 0;
                            dropped = 0;
                            last_report = Instant::now();
                        }
                    }
                }
            }
        }
    });
}

fn spawn_fault_topic_subscriber<NS>(
    stack: &NS,
    fault_sink: FaultSnapshotSink,
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
                        if connected.load(Ordering::Relaxed) {
                            fault_sink.offer(msg.t.clone());
                        }
                    }
                }
            }
        }
    });
}

/// Refuse every externally visible command until HardwareInfo has passed the
/// bootstrap/version/UUID checks. An interface can be Active for several
/// round trips before that handshake completes; sending during that window
/// could otherwise mutate a different USB controller before UUID mismatch is
/// detected.
fn reject_unverified_command(cmd: HostCommand) {
    let error = || anyhow::anyhow!("device is not connected and identity-verified");
    match cmd {
        HostCommand::Motor(mode) => {
            tracing::warn!("Dropping motor command before verified handshake: {mode:?}");
        }
        HostCommand::MotorAck(_, reply) | HostCommand::EmergencyStop(reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::SetPhaseSource(source) => {
            tracing::warn!("Dropping phase-source command before verified handshake: {source:?}");
        }
        HostCommand::SetTelemetryConfig(config) => {
            tracing::warn!("Dropping telemetry config before verified handshake: {config:?}");
        }
        HostCommand::ConfigRead(_, reply)
        | HostCommand::ConfigApply(_, reply)
        | HostCommand::ConfigPersist(_, reply)
        | HostCommand::ConfigResetAll(reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::Detect(_, reply) => {
            let _ = reply.send(Err(error()));
        }
        HostCommand::Fault(_, reply) => {
            let _ = reply.send(Err(error()));
        }
    }
}

fn spawn_slow_telemetry_poller<NS>(
    stack: &NS,
    slow_tx: Sender<SlowTelemetry>,
    fault_sink: FaultSnapshotSink,
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
                            // A fault topic can be dropped without changing
                            // fault_count (payload refinement or clear+add).
                            // The generation in the regular poll is the
                            // reliable loss detector; query until the latest
                            // snapshot is successfully handed to consumers.
                            if fault_sink.needs(sample.fault_generation) {
                                let fault_fut = ns.endpoints().request::<FaultEndpoint>(
                                    DEVICE_ADDR,
                                    &FaultRequest::Query,
                                    Some("fault_reconcile"),
                                );
                                if let Ok(Ok(FaultResponse::Snapshot(snapshot))) =
                                    tokio::time::timeout(Duration::from_millis(500), fault_fut).await
                                {
                                    fault_sink.offer(snapshot);
                                }
                            }
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

/// A motor-command response still in flight after the send completed.
type MotorResponseFut = std::pin::Pin<
    Box<dyn Future<Output = Result<MotorStatus, ergot::net_stack::ReqRespError>> + Send>,
>;

/// Send a `MotorEndpoint` request NOW and hand back the pending response.
///
/// Ergot's split request API commits the frame synchronously, then hands the
/// owned response socket to this future. Sends therefore stay strictly ordered
/// by call order within the command task, while the caller observes the
/// response OUT of that task so response latency cannot stall the affirm
/// ticker.
///
/// Why this exists (2026-07-06 deadman hunt): the drive-`Start` used to be
/// awaited inline in the command/affirm `select!` loop. At drive engage the
/// device-side round-trip inflates to ~100-150 ms (thread-mode latency under
/// the telemetry stream), the first affirm was delayed by exactly that
/// round-trip, and the device's 150 ms deadman tripped stochastically
/// (~2/3 of spins that afternoon). The dropped 70 s inline retry is not
/// missed: for drive modes the 50 ms affirm cadence IS the retry, and for
/// stop-class commands a lost frame ends in the deadman failsafe stopping
/// the motor — the correct direction.
fn send_motor_now<NS>(
    ns: &NS,
    request: MotorRequest,
) -> Result<MotorResponseFut, ergot::net_stack::ReqRespError>
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
{
    let client =
        ergot::socket::endpoint::single::Client::<MotorEndpoint, NS>::new(ns.clone(), None);
    let mut client = Box::pin(client).attach_boxed();
    client.send_request(DEVICE_ADDR, &request, Some("motor"))?;
    Ok(Box::pin(async move {
        match client.recv().await {
            Ok(response) => Ok(response.t),
            Err(error) => Err(ergot::net_stack::ReqRespError::Remote(error.t)),
        }
    }))
}

async fn handle_command<NS>(
    ns: &NS,
    cmd: HostCommand,
    fast_hz_flag: &Arc<AtomicU16>,
    active_setpoint: &mut Option<ControlMode>,
    drive_session: u64,
    motor_seq: &mut u32,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
    NS::Mutex: Send + Sync,
    NS::Profile: Send,
    NS::Target: Send + Sync,
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
            // Send inline (ordered), observe the response detached — the
            // await must not hold up this select! loop, or the affirm ticker
            // starves and the device deadman fires (see send_motor_now).
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::SetMode(*mc));
            match send_motor_now(ns, request) {
                Err(e) => tracing::warn!("Motor command send failed: {:?}", e),
                Ok(fut) => {
                    tokio::spawn(async move {
                        match tokio::time::timeout(MOTOR_RESPONSE_TIMEOUT, fut).await {
                            Ok(Ok(status)) => tracing::info!("Motor response: {:?}", status),
                            Ok(Err(e)) => tracing::warn!("Motor command failed: {:?}", e),
                            Err(_) => tracing::warn!("Motor response lost (sent, no reply)"),
                        }
                    });
                }
            }
        }
        HostCommand::MotorAck(ref mc, reply_tx) => {
            // Same as Motor (incl. the affirmation tracking), but the caller
            // gets the device status / delivery error back — a CLI must not
            // print "sent" and exit 0 when nothing was delivered. The
            // response wait is detached like Motor's; a lost response
            // surfaces to the caller as an error after MOTOR_RESPONSE_TIMEOUT.
            //
            // The old clear-active_setpoint-on-failure is gone deliberately:
            // failure now means "no reply", not "70 s of retries exhausted".
            // The affirm keeps re-sending the human's commanded value — if
            // the original frame was lost, the next affirm IS the command
            // (idempotent); if the LINK is dead, the affirms die with it and
            // the device deadman stops the motor. Disconnect still drops the
            // setpoint in the ticker arm.
            *active_setpoint = if matches!(
                mc,
                ControlMode::Stopped | ControlMode::Coast | ControlMode::Brake
            ) {
                None
            } else {
                Some(*mc)
            };
            tracing::info!("Sending motor command (acked): {:?}", mc);
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::SetMode(*mc));
            match send_motor_now(ns, request) {
                Err(error) => {
                    let _ = reply_tx.send(Err(anyhow::anyhow!("{error:?}")));
                }
                Ok(fut) => {
                    tokio::spawn(async move {
                        let res = match tokio::time::timeout(MOTOR_RESPONSE_TIMEOUT, fut).await {
                            Ok(Ok(status)) if status.outcome.is_success() => Ok(status),
                            Ok(Ok(status)) => Err(anyhow::anyhow!(
                                "motor command rejected: {:?}",
                                status.outcome
                            )),
                            Ok(Err(e)) => Err(anyhow::anyhow!("{e:?}")),
                            Err(_) => Err(anyhow::anyhow!(
                                "motor response lost (sent, no reply in {MOTOR_RESPONSE_TIMEOUT:?})"
                            )),
                        };
                        let _ = reply_tx.send(res);
                    });
                }
            }
        }
        HostCommand::EmergencyStop(reply_tx) => {
            // Safety commands clear the local affirm immediately, before the
            // frame is sent, so an older throttle can never follow it.
            *active_setpoint = None;
            let request = next_motor_request(drive_session, motor_seq, MotorCommand::EmergencyStop);
            tracing::warn!("Sending emergency stop");
            match send_motor_now(ns, request) {
                Err(error) => {
                    let _ = reply_tx.send(Err(anyhow::anyhow!("{error:?}")));
                }
                Ok(fut) => {
                    tokio::spawn(async move {
                        let result = match tokio::time::timeout(MOTOR_RESPONSE_TIMEOUT, fut).await {
                            Ok(Ok(status)) if status.outcome.is_success() => Ok(status),
                            Ok(Ok(status)) => Err(anyhow::anyhow!(
                                "emergency stop rejected: {:?}",
                                status.outcome
                            )),
                            Ok(Err(error)) => Err(anyhow::anyhow!("{error:?}")),
                            Err(_) => Err(anyhow::anyhow!(
                                "emergency stop response lost (sent, no reply in {MOTOR_RESPONSE_TIMEOUT:?})"
                            )),
                        };
                        let _ = reply_tx.send(result);
                    });
                }
            }
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
        HostCommand::ConfigApply(request, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!("Applying volatile config: {:?}", request.inner.write);
            let req = ConfigRequest::Apply(request);
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
        HostCommand::ConfigPersist(request, reply_tx) => {
            use oxifoc_core::types::ConfigRequest;
            tracing::info!(
                "Persisting config group {:?} at revision {}",
                request.inner.group,
                request.inner.expected_revision
            );
            let req = ConfigRequest::Persist(request);
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

#[inline]
fn should_start_defmt(enabled: bool, already_started: bool, transport_scoped: bool) -> bool {
    enabled && (!already_started || transport_scoped)
}

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
                    // Each network message is one COMPLETE encoder frame
                    // (accumulated between defmt acquire/release on the
                    // device, terminator included). `Table::decode` expects
                    // an UNENCODED frame and reports rzcobs frames — the
                    // default firmware encoding — as Malformed; only the
                    // stream decoder honors the table's encoding. A fresh
                    // per-message decoder is correct because frames never
                    // split across messages (and it must not live across
                    // the await: it borrows `table`).
                    let mut stream = table.new_stream_decoder();
                    stream.received(&msg.t.frame);
                    match stream.decode() {
                        Ok(frame) => {
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

#[cfg(test)]
mod tests {
    use super::{
        FaultSnapshotSink, generation_is_newer, may_retry, protocol_is_compatible,
        reject_unverified_command, should_start_defmt,
    };
    use crate::ReconnectPolicy;
    use oxifoc_core::types::FaultSnapshot;

    #[test]
    fn transport_scoped_defmt_restarts_but_network_decoder_does_not() {
        assert!(!should_start_defmt(false, false, true));
        assert!(should_start_defmt(true, false, false));
        assert!(!should_start_defmt(true, true, false));
        assert!(should_start_defmt(true, true, true));
    }

    #[test]
    fn protocol_version_must_match_exactly() {
        let current = oxifoc_core::types::ICD_PROTO_VERSION;
        assert!(protocol_is_compatible(current));
        assert!(!protocol_is_compatible(current.wrapping_add(1)));
        assert!(!protocol_is_compatible(current.wrapping_sub(1)));
    }

    #[test]
    fn reconnect_policy_covers_every_connection_generation() {
        let mut failures = 0;
        assert!(!may_retry(ReconnectPolicy::None, &mut failures));
        assert_eq!(failures, 1);

        let mut failures = 0;
        assert!(may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert!(may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert!(!may_retry(ReconnectPolicy::Limited(3), &mut failures));
        assert_eq!(failures, 3);

        let mut failures = u32::MAX;
        assert!(may_retry(ReconnectPolicy::Infinite, &mut failures));
        assert_eq!(failures, u32::MAX);
    }

    #[test]
    fn fault_generation_order_handles_wrap() {
        assert!(generation_is_newer(2, 1));
        assert!(!generation_is_newer(1, 1));
        assert!(!generation_is_newer(1, 2));
        assert!(generation_is_newer(0, u32::MAX));
        assert!(!generation_is_newer(u32::MAX, 0));
    }

    #[test]
    fn full_fault_consumer_queue_keeps_generation_dirty() {
        let (tx, rx) = crossbeam_channel::bounded(1);
        let sink = FaultSnapshotSink::new(tx);
        let snapshot = |generation| FaultSnapshot {
            generation,
            ..Default::default()
        };

        sink.offer(snapshot(1));
        sink.offer(snapshot(2)); // full: must not claim generation 2 delivered
        assert!(sink.needs(2));
        assert_eq!(rx.recv().unwrap().generation, 1);

        sink.offer(snapshot(2));
        sink.offer(snapshot(1)); // stale: must not regress the consumer view
        assert_eq!(rx.recv().unwrap().generation, 2);
        assert!(!sink.needs(2));
    }

    #[test]
    fn fault_generation_resets_across_verified_reconnect() {
        let (tx, rx) = crossbeam_channel::bounded(2);
        let sink = FaultSnapshotSink::new(tx);
        sink.offer(FaultSnapshot {
            generation: 100,
            ..Default::default()
        });
        assert_eq!(rx.recv().unwrap().generation, 100);

        sink.reset_generation();
        sink.offer(FaultSnapshot {
            generation: 0,
            ..Default::default()
        });
        assert_eq!(rx.recv().unwrap().generation, 0);
    }

    #[test]
    fn unverified_connection_rejects_acknowledged_commands() {
        let (tx, rx) = crate::motor_channel();
        reject_unverified_command(super::HostCommand::EmergencyStop(tx));
        let error = rx
            .blocking_recv()
            .expect("rejection sender must answer")
            .expect_err("unverified command must fail");
        assert!(error.to_string().contains("identity-verified"));
    }
}

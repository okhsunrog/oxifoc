//! Persistent stack and per-connection lifecycle shared by every transport.

use super::*;

type DefmtReader = Option<Box<dyn AsyncRead + Send + Unpin>>;

/// Packet transports only attach an interface; defmt uses the persistent
/// network subscription rather than a connection-local reader.
pub(super) async fn run_framed_with_reconnect<NS, I>(
    stack: NS,
    mut register_fn: impl AsyncFnMut() -> Result<()>,
    state_notify: Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    cfg: &HostConfig,
    ctx: BackendCtx,
) -> Result<()>
where
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
    run_connection_loop(
        stack,
        async || register_fn().await.map(|()| None),
        state_notify,
        cfg,
        ctx,
    )
    .await
}

/// Stream transports build the stack once and attach fresh I/O on each
/// attempt. The common loop owns retries, verification and teardown.
pub(super) async fn run_cobs_stream_with_reconnect<F, Fut>(
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
    let stack = stream_kit::new_target_stack(&queue, ERGOT_MTU);
    let state_notify = Arc::new(stream_kit::WaitQueue::new());
    let register_fn = {
        let stack = stack.clone();
        let notify = state_notify.clone();
        async move || {
            let transport = connect_fn().await?;
            tokio_cobs_stream::register_edge::<
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
                Some(notify.clone()),
            )
            .await
            .map_err(|e| anyhow!("Stream registration failed: {e:?}"))?;
            Ok(transport.defmt_reader)
        }
    };
    run_connection_loop(stack, register_fn, state_notify, cfg, ctx).await
}

async fn run_connection_loop<NS, I>(
    stack: NS,
    mut register_fn: impl AsyncFnMut() -> Result<DefmtReader>,
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
    let requested_fast_hz = ctx.requested_fast_hz.clone();
    let device_addr = ctx.device_addr.clone();
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

        let defmt_reader = match reg_result {
            Ok(reader) => reader,
            Err(e) => {
                connected.store(false, Ordering::Relaxed);
                stack
                    .stack()
                    .manage_profile(ergot::prelude::DirectEdge::teardown);
                tokio::task::yield_now().await;
                let retry = may_retry(policy, &mut connect_attempts);
                tracing::warn!(
                    "Transport registration failed (attempt {}): {:?}",
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
        };
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

        // Resolve the controller's address before the handshake: through a
        // bridge the link peer is the bridge, not the motor controller, so
        // the handshake must target whoever actually serves MotorEndpoint.
        let resolution = tokio::select! {
            resolution = resolve_device_addr(&stack, RESOLVE_TIMEOUT) => resolution,
            _ = cancel.cancelled() => break,
        };
        let resolved = match resolution {
            AddrResolution::Resolved(addr) => {
                info!(
                    "Controller resolved at net={} node={}",
                    addr.network_id, addr.node_id
                );
                addr
            }
            AddrResolution::NoAnswer => {
                info!("No socket-query answer; using the link-local peer");
                LINK_LOCAL_PEER
            }
            AddrResolution::Ambiguous(n) => {
                tracing::error!(
                    "AMBIGUOUS CONTROLLER: {n} nodes serve MotorEndpoint on this network — \
                     refusing to pick one by luck"
                );
                connected.store(false, Ordering::Relaxed);
                break;
            }
        };
        device_addr.store(pack_addr(resolved), Ordering::Relaxed);

        // HardwareInfo handshake on each (re)connection
        let handshake = tokio::select! {
            outcome = hardware_info_handshake(
                &stack, &info_tx, &connected, &mut expected_device_uuid, resolved,
            ) => outcome,
            _ = cancel.cancelled() => break,
        };

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

        tokio::select! {
            () = enable_fast_telemetry(
                &stack, requested_fast_hz.load(Ordering::Relaxed), &fast_hz_flag, resolved,
            ) => {},
            _ = cancel.cancelled() => break,
        }

        // RTT readers belong to one connection; network subscriptions belong
        // to the persistent stack and must be started only once.
        let transport_scoped_defmt = defmt_reader.is_some();
        if should_start_defmt(cfg.stream_defmt(), defmt_started, transport_scoped_defmt) {
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

    // Always release the interface, including cancellation and exhausted
    // retry budgets. RTT I/O exits when its interface reader is dropped;
    // only then is it safe to join the blocking probe thread.
    connected.store(false, Ordering::Relaxed);
    stack
        .stack()
        .manage_profile(ergot::prelude::DirectEdge::teardown);
    #[cfg(feature = "desktop")]
    {
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
                let recovered = tokio::select! {
                    result = tokio::time::timeout(
                        RECOVERY_TIMEOUT,
                        wait_for_active(state_notify, stack),
                    ) => matches!(result, Ok(true)),
                    _ = cancel.cancelled() => return false,
                };
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

// ── Device address resolution ────────────────────────────────────────────────

enum AddrResolution {
    /// Exactly one node on the network serves `MotorEndpoint`.
    Resolved(Address),
    /// Nobody answered the socket query (device predates the handler, or a
    /// transport that cannot flood) — fall back to the link-local peer.
    NoAnswer,
    /// More than one motor controller answered. Fail closed: a controller
    /// must never be selected by luck (mirrors the USB identity pinning).
    Ambiguous(usize),
}

/// Resolve the motor controller's address by asking the whole reachable
/// network who serves `MotorEndpoint` (port-255 SocketQuery flood; routers
/// re-flood it across bridges, answers come back unicast). Returns net+node
/// with port 0 so the existing per-endpoint key/name resolution at the
/// destination keeps working unchanged.
async fn resolve_device_addr<NS>(stack: &NS, timeout: Duration) -> AddrResolution
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    use ergot::FrameKind;
    use ergot::traits::Endpoint as _;
    use ergot::well_known::{
        ErgotSocketQueryResponseTopic, ErgotSocketQueryTopic, NameRequirement, SocketQuery,
    };

    let ns = stack.stack();
    // Subscribe BEFORE sending the query; responses are unicast topic
    // messages addressed to this socket's port.
    let subber = ns
        .topics()
        .heap_bounded_receiver::<ErgotSocketQueryResponseTopic>(16, None);
    let subber = std::pin::pin!(subber);
    let mut hdl = subber.subscribe_unicast();
    let port = hdl.port();

    let query = SocketQuery {
        key: MotorEndpoint::REQ_KEY.to_bytes(),
        nash_req: NameRequirement::Any,
        frame_kind: FrameKind::ENDPOINT_REQ,
        broadcast: false,
    };

    let mut nodes: Vec<(u16, u8)> = Vec::new();
    for attempt in 1..=RESOLVE_ATTEMPTS {
        if let Err(e) = ns
            .topics()
            .broadcast_with_src_port::<ErgotSocketQueryTopic>(&query, None, port)
        {
            // Best-effort: a failed broadcast just means an empty attempt.
            tracing::debug!("socket query broadcast failed (attempt {attempt}): {e:?}");
        }
        let Ok(first) = tokio::time::timeout(timeout, hdl.recv()).await else {
            continue;
        };
        nodes.push((first.hdr.src.network_id, first.hdr.src.node_id));
        // Settle window: a duplicate answer (retried attempt) dedups away; a
        // DIFFERENT node answering means two controllers are reachable.
        let settle = async {
            loop {
                let msg = hdl.recv().await;
                let node = (msg.hdr.src.network_id, msg.hdr.src.node_id);
                if !nodes.contains(&node) {
                    nodes.push(node);
                }
            }
        };
        let _ = tokio::time::timeout(RESOLVE_SETTLE, settle).await;
        break;
    }

    match nodes.as_slice() {
        [] => AddrResolution::NoAnswer,
        [(network_id, node_id)] => AddrResolution::Resolved(Address {
            network_id: *network_id,
            node_id: *node_id,
            port_id: 0,
        }),
        many => AddrResolution::Ambiguous(many.len()),
    }
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
    device_addr: Address,
) -> HandshakeOutcome
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let ns = stack.stack();
    let mut backoff = Duration::from_millis(100);
    for attempt in 1..=10u32 {
        let fut =
            ns.endpoints()
                .request::<HardwareInfoEndpoint>(device_addr, &(), Some("hardware_info"));
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

pub(super) fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
}

/// Send TelemetryConfig to enable fast telemetry streaming.
async fn enable_fast_telemetry<NS>(
    stack: &NS,
    fast_hz: u16,
    fast_hz_flag: &Arc<AtomicU16>,
    device_addr: Address,
) where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    if fast_hz == 0 {
        return;
    }
    let telem_cfg = TelemetryConfig { fast_hz };
    let client = stack.clone().reliable::<TokioTimer>();
    match client
        .at_least_once::<TelemetryConfigEndpoint>(device_addr, &telem_cfg, None, &SETPOINT_POLICY)
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

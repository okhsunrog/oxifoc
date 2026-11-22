//! Persistent stack and per-connection lifecycle shared by every transport.

use super::*;

/// Packet transports only attach an interface.
pub(super) async fn run_framed_with_reconnect<NS, I>(
    stack: NS,
    register_fn: impl AsyncFnMut() -> Result<()>,
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
    run_connection_loop(stack, register_fn, state_notify, cfg, ctx).await
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
            Ok(())
        }
    };
    run_connection_loop(stack, register_fn, state_notify, cfg, ctx).await
}

async fn run_connection_loop<NS, I>(
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
    let device_addr = ctx.device_addr.clone();
    let cancel = ctx.cancel.clone();
    let info_tx = ctx.info_tx.clone();
    let fault_sink = ctx.fault_sink.clone();
    let raw = ctx.raw_telemetry.clone();

    // Protocol tasks are spawned once — they operate on the stack, not the
    // per-connection interface.
    let commands = spawn_protocol_tasks(&stack, ctx);
    tokio::spawn({
        let stack = stack.clone();
        async move { stack.stack().services().log_handler(32).await }
    });
    let keepalive = tokio::spawn(bridge_keepalive(
        stack.clone(),
        connected.clone(),
        cancel.clone(),
    ));

    let mut defmt_started = false;
    let policy = cfg.reconnect_policy();
    let mut connect_attempts: u32 = 0;
    let mut expected_device_uuid: Option<String> = None;
    let mut verified_peer = None;
    // True to verify the controller again on the current link instead of
    // reopening it: after the link recovered, or while a bridge waits for its
    // controller.
    let mut reuse_link = false;
    let mut waiting_for_controller = false;

    loop {
        if !reuse_link {
            let reg_result = tokio::select! {
                r = register_fn() => r,
                _ = cancel.cancelled() => break,
            };

            if let Err(e) = reg_result {
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
        }
        reuse_link = false;

        // Resolve the controller's address before the handshake: through a
        // bridge the link peer is the bridge, not the motor controller, so
        // the handshake must target whoever actually serves MotorEndpoint.
        let resolution = if cfg.bridge_only {
            AddrResolution::NoAnswer
        } else {
            tokio::select! {
                resolution = resolve_device_addr(&stack, RESOLVE_TIMEOUT) => resolution,
                _ = cancel.cancelled() => break,
            }
        };
        let (resolved, handshake_attempts) = match resolution {
            AddrResolution::Resolved(addr) => {
                info!(
                    "Controller resolved at net={} node={}",
                    addr.network_id, addr.node_id
                );
                (addr, 10)
            }
            AddrResolution::NoAnswer if cfg.bridge_only || bridge_reachable(&stack).await => {
                // Bridge services stay available even when controller
                // telemetry cannot be configured or the controller is off.
                if !cfg.bridge_only && !waiting_for_controller {
                    info!("Bridge reachable, no motor controller behind it yet; keeping the link");
                    waiting_for_controller = true;
                }
                match stay_on_link(&state_notify, &stack, &cancel, RECONNECT_DELAY).await {
                    None => break,
                    Some(true) => {
                        reuse_link = true;
                        continue;
                    }
                    Some(false) => {
                        stack
                            .stack()
                            .manage_profile(ergot::prelude::DirectEdge::teardown);
                        tokio::task::yield_now().await;
                        if !may_retry(policy, &mut connect_attempts) {
                            break;
                        }
                        continue;
                    }
                }
            }
            AddrResolution::NoAnswer => {
                // A direct legacy peer can still answer this probe. Through a
                // bridge it is the router, not the controller; do not spend the
                // full handshake retry budget on that unverified address while
                // upstream discovery/seed assignment is still settling.
                info!("No socket-query answer; probing the link-local peer once");
                (LINK_LOCAL_PEER, 1)
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
        waiting_for_controller = false;
        device_addr.store(pack_addr(resolved), Ordering::Relaxed);

        // HardwareInfo handshake on each (re)connection
        let handshake = tokio::select! {
            outcome = hardware_info_handshake(
                &stack, &info_tx, &mut expected_device_uuid, resolved, handshake_attempts,
            ) => outcome,
            _ = cancel.cancelled() => break,
        };

        match handshake {
            HandshakeOutcome::Connected => {
                verified_peer = Some(resolved);
                // Fault generation is boot-local. A controller reset may
                // reconnect with the same UUID and generation 0; force a
                // fresh snapshot instead of treating it as older.
                fault_sink.reset_generation();
                connect_attempts = 0;
            }
            HandshakeOutcome::FatalMismatch => {
                tracing::error!("Fatal handshake mismatch — refusing to reconnect");
                connected.store(false, Ordering::Relaxed);
                if bridge_reachable(&stack).await {
                    // Controller commands stay refused, but the bridge's own
                    // services (pairing) remain usable until the link drops.
                    tracing::warn!("Keeping the bridge link for bridge services");
                    hold_link(&state_notify, &stack, &cancel).await;
                }
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

        // A reconnect establishes control only. A failed capture's cookie
        // and requested rate are never resumed by a new connection.
        let setup = raw_telemetry::prepare_connection(&stack, &raw, resolved);
        let configured = match tokio::select! { result = setup => result, _ = cancel.cancelled() => break }
        {
            Ok(()) => {
                raw.reader_ready();
                true
            }
            Err(error) => {
                raw.fail(&error);
                tracing::error!("raw setup failed: {error:#}");
                false
            }
        };
        if !configured {
            verified_peer = None;
            stack
                .stack()
                .manage_profile(ergot::prelude::DirectEdge::teardown);
            tokio::task::yield_now().await;
            if !may_retry(policy, &mut connect_attempts) {
                break;
            }
            tokio::select! {
                _ = tokio::time::sleep(RECONNECT_DELAY) => continue,
                _ = cancel.cancelled() => break,
            }
        }
        // Publish readiness after stream setup, so a caller cannot race its
        // first rate change against a late connect-time fast_hz=0 request.
        connected.store(true, Ordering::Relaxed);

        // The defmt subscription belongs to the persistent stack.
        if cfg.stream_defmt() && !defmt_started {
            match start_defmt_decoder(cfg, &stack) {
                Ok(()) => defmt_started = true,
                Err(e) => {
                    tracing::warn!("defmt decoder unavailable, continuing without it: {e:#}");
                }
            }
        }

        // Monitor interface state with recovery
        let link = monitor_state_with_recovery(&state_notify, &stack, &connected, &cancel).await;

        if link == LinkOutcome::Cancelled {
            break;
        }
        raw.control_lost();
        verified_peer = None;
        if link == LinkOutcome::Recovered {
            // The link came back, but whoever answers now (a restarted
            // bridge, a different controller) is unverified until the
            // handshake has checked it again.
            info!("Link recovered; verifying the controller again");
            reuse_link = true;
            continue;
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

    // Join the command task before the final rate change: an in-flight retry
    // must not re-enable the stream after our stop was acknowledged.
    commands.abort();
    let _ = commands.await;
    keepalive.abort();

    // Cancellation stops the command task too. Send directly while the
    // transport is still alive, rather than enqueueing a best-effort command
    // that runtime teardown could discard. Never target an unverified peer.
    if let Some(peer) = verified_peer {
        let stop = raw_telemetry::configure(&stack, &raw, 0, peer);
        match tokio::time::timeout(Duration::from_millis(500), stop).await {
            Ok(Ok(ack)) if ack.actual_fast_hz == 0 => {
                fast_hz_flag.store(0, Ordering::Relaxed);
                info!("Telemetry stopped before transport shutdown (acknowledged)");
            }
            result => {
                tracing::warn!("Could not confirm telemetry stop before shutdown: {result:?}");
            }
        }
    }

    // Always release the interface, including cancellation and exhausted
    // retry budgets.
    connected.store(false, Ordering::Relaxed);
    stack
        .stack()
        .manage_profile(ergot::prelude::DirectEdge::teardown);

    info!("Host backend shutdown complete");
    Ok(())
}

// ── State monitoring ─────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LinkOutcome {
    Cancelled,
    /// The link is down; reopen it.
    Down,
    /// The link went inactive and came back. `connected` stays false: the
    /// caller must verify the controller again.
    Recovered,
}

/// Wait for the verified link to leave Active, then for it to recover.
async fn monitor_state_with_recovery<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
    connected: &Arc<AtomicBool>,
    cancel: &CancellationToken,
) -> LinkOutcome
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    let observed = tokio::select! {
        observed = state_notify.wait_for_value(|| {
            let state = stack.stack().manage_profile(|im| im.interface_state(()));
            let is_active = interface_is_active(state);
            (is_active != connected.load(Ordering::Relaxed)).then_some(state)
        }) => observed,
        _ = cancel.cancelled() => return LinkOutcome::Cancelled,
    };
    let Ok(state) = observed else {
        connected.store(false, Ordering::Relaxed);
        return LinkOutcome::Down;
    };
    if interface_is_active(state) {
        // Active while not connected: only the handshake may set `connected`.
        return LinkOutcome::Recovered;
    }

    connected.store(false, Ordering::Relaxed);
    if matches!(state, Some(InterfaceState::Down) | None) {
        tracing::warn!("Interface down, reconnecting transport...");
        return LinkOutcome::Down;
    }

    tracing::warn!("Interface inactive, waiting for recovery...");
    let recovered = tokio::select! {
        result = tokio::time::timeout(
            RECOVERY_TIMEOUT,
            wait_for_active(state_notify, stack),
        ) => matches!(result, Ok(true)),
        _ = cancel.cancelled() => return LinkOutcome::Cancelled,
    };
    if recovered {
        info!("Interface active again");
        LinkOutcome::Recovered
    } else {
        tracing::warn!("Recovery failed or timed out, reconnecting transport...");
        LinkOutcome::Down
    }
}

/// Keep a bridge link alive while no controller is verified. The liveness
/// watchdog counts received frames only, and with the controller absent
/// nothing else answers us. A bridge answers the query; a directly attached
/// controller ignores it, which costs one small frame per period.
async fn bridge_keepalive<NS>(stack: NS, connected: Arc<AtomicBool>, cancel: CancellationToken)
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>> + Clone + Send + Sync + 'static,
{
    const PERIOD: Duration = Duration::from_millis(300);
    loop {
        tokio::select! {
            _ = tokio::time::sleep(PERIOD) => {}
            _ = cancel.cancelled() => return,
        }
        let active = interface_is_active(stack.stack().manage_profile(|im| im.interface_state(())));
        if active && !connected.load(Ordering::Relaxed) {
            bridge_reachable(&stack).await;
        }
    }
}

/// True when a bridge replies on the link, even with the controller absent.
async fn bridge_reachable<NS>(stack: &NS) -> bool
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    use ergot::traits::Endpoint as _;
    let pairing = resolve_endpoint_addr(
        stack,
        oxifoc_core::icd::PairingEndpoint::REQ_KEY.to_bytes(),
        RESOLVE_TIMEOUT,
    );
    let ota = resolve_endpoint_addr(
        stack,
        oxifoc_core::icd::BridgeOtaEndpoint::REQ_KEY.to_bytes(),
        RESOLVE_TIMEOUT,
    );
    tokio::pin!(pairing, ota);
    tokio::select! {
        value = &mut pairing => value.is_some() || ota.await.is_some(),
        value = &mut ota => value.is_some() || pairing.await.is_some(),
    }
}

/// Wait `delay` on the current link. `Some(true)` if it is still Active,
/// `Some(false)` if it dropped, `None` if cancelled.
async fn stay_on_link<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
    cancel: &CancellationToken,
    delay: Duration,
) -> Option<bool>
where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    tokio::select! {
        _ = tokio::time::sleep(delay) => {
            Some(interface_is_active(stack.stack().manage_profile(|im| im.interface_state(()))))
        }
        _ = wait_for_inactive(state_notify, stack) => Some(false),
        _ = cancel.cancelled() => None,
    }
}

/// Keep the link until it stops being Active or the backend is cancelled.
async fn hold_link<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
    cancel: &CancellationToken,
) where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    tokio::select! {
        _ = wait_for_inactive(state_notify, stack) => {}
        _ = cancel.cancelled() => {}
    }
}

async fn wait_for_inactive<NS>(
    state_notify: &Arc<ergot::toolkits::tokio_stream::WaitQueue>,
    stack: &NS,
) where
    NS: NetStackHandle<Profile: Profile<InterfaceIdent = ()>>,
{
    let _ = state_notify
        .wait_for_value(|| {
            let state = stack.stack().manage_profile(|im| im.interface_state(()));
            (!interface_is_active(state)).then_some(())
        })
        .await;
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
    use ergot::traits::Endpoint as _;
    resolve_key_addr(stack, MotorEndpoint::REQ_KEY.to_bytes(), timeout).await
}

/// The single node that serves the endpoint with this request key, if exactly
/// one answers (e.g. the bridge's `PairingEndpoint`).
pub(crate) async fn resolve_endpoint_addr<NS>(
    stack: &NS,
    key: [u8; 8],
    timeout: Duration,
) -> Option<Address>
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    match resolve_key_addr(stack, key, timeout).await {
        AddrResolution::Resolved(addr) => Some(addr),
        _ => None,
    }
}

async fn resolve_key_addr<NS>(stack: &NS, key: [u8; 8], timeout: Duration) -> AddrResolution
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    use ergot::FrameKind;
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
        key,
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
    expected_device_uuid: &mut Option<String>,
    device_addr: Address,
    attempts: u32,
) -> HandshakeOutcome
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let ns = stack.stack();
    let mut backoff = Duration::from_millis(100);
    for attempt in 1..=attempts {
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
                    return HandshakeOutcome::FatalMismatch;
                }
                info!(
                    "Device connected: hw='{}' sw='{}' mcu='{}' uuid='{}' foc={}Hz max_i={}A proto=v{}",
                    dev_info.hw.as_str(),
                    dev_info.sw.as_str(),
                    dev_info.mcu.as_str(),
                    dev_info.uuid.as_str(),
                    dev_info.foc_freq_hz,
                    dev_info.max_current_a.value,
                    dev_info.proto_version,
                );
                if !protocol_is_compatible(dev_info.proto_version) {
                    tracing::error!(
                        "PROTOCOL VERSION MISMATCH: device proto v{}, host proto v{} — refusing \
                         the connection; update whichever side is older",
                        dev_info.proto_version,
                        oxifoc_core::types::ICD_PROTO_VERSION,
                    );
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
                        return HandshakeOutcome::FatalMismatch;
                    }
                } else {
                    *expected_device_uuid = Some(device_uuid.to_owned());
                }
                // Never block the backend if a consumer stops reading.
                let _ = info_tx.try_send(dev_info);
                return HandshakeOutcome::Connected;
            }
            Ok(Err(e)) => {
                tracing::warn!("HardwareInfo attempt {} failed: {:?}", attempt, e);
                // The resolved path can disappear while a parent router
                // restarts. Reopen the interface and rediscover the route;
                // retrying its old address cannot restore local addressing.
                return HandshakeOutcome::RetryableFailure;
            }
            Err(_) => {
                tracing::warn!("HardwareInfo attempt {} timed out", attempt);
            }
        }
        if attempt < attempts {
            tokio::time::sleep(backoff).await;
            backoff = (backoff * 2).min(Duration::from_secs(2));
        }
    }
    tracing::warn!("Device info not received after retries; giving up — caller will reconnect");
    HandshakeOutcome::RetryableFailure
}

pub(super) fn protocol_is_compatible(device_version: u16) -> bool {
    device_version == oxifoc_core::types::ICD_PROTO_VERSION
}

// ── Protocol tasks ───────────────────────────────────────────────────────────

//! UDP transport for connecting to ergot devices over UDP.
//!
//! UDP is packet-framed (no COBS needed). Uses ergot's `tokio_udp` toolkit
//! directly. The stack is created once and survives reconnects; each
//! (re)connection binds a fresh socket and registers it onto the stack
//! (`register` only succeeds while the interface is Down, i.e. after a
//! teardown — same pattern as the COBS stream path).

use oxifoc_core::{
    icd::{LIVENESS_TIMEOUT_MS, UDP_MTU},
    udp_link::{HEADER_BYTES, HISTORY_PACKETS, Header, Kind},
};
use std::{
    collections::BTreeMap,
    sync::{
        Arc,
        atomic::{AtomicU32, Ordering},
    },
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::{Context, Result};
use ergot::exports::bbqueue::traits::bbqhdl::BbqHandle;
use ergot::interface_manager::utils::std::StdQueue;
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, LivenessConfig, Profile,
    profiles::direct_edge::EdgeFrameProcessor, transports::tokio_udp::UdpTxWorker,
};
use ergot::toolkits::tokio_stream::WaitQueue;
use ergot::toolkits::tokio_udp::{self, EdgeStack};
use tokio::{
    net::UdpSocket,
    time::{Instant, sleep_until, timeout},
};
use tracing::info;

const QUIET_REPLAY: Duration = Duration::from_millis(35);
const REPLAY_INTERVAL: Duration = Duration::from_millis(12);
const GAP_TIMEOUT: Duration = Duration::from_millis(90);

pub fn new_stack(queue_size: usize, mtu: u16) -> (EdgeStack, StdQueue) {
    let queue = tokio_udp::new_std_queue(queue_size);
    let stack = tokio_udp::new_target_stack(&queue, mtu);
    (stack, queue)
}

pub async fn register(
    stack: &EdgeStack,
    queue: &StdQueue,
    host: &str,
    port: u16,
    state_notify: Option<Arc<WaitQueue>>,
) -> Result<()> {
    let addr = format!("{host}:{port}");
    info!("Connecting to UDP: {}", addr);

    let socket = UdpSocket::bind("0.0.0.0:0")
        .await
        .context("Failed to bind UDP socket")?;
    socket
        .connect(&addr)
        .await
        .with_context(|| format!("Failed to connect UDP to {addr}"))?;

    if let Some(token) = negotiate(&socket).await? {
        register_sequenced(stack, queue, socket, token, state_notify)?;
        info!("UDP connected to {} with frame replay", addr);
        return Ok(());
    }

    tokio_udp::register_edge_target_interface(
        stack,
        socket,
        queue,
        Some(LivenessConfig {
            timeout_ms: LIVENESS_TIMEOUT_MS,
        }),
        state_notify,
    )
    .await
    .map_err(|_| anyhow::anyhow!("UDP interface not in Down state"))?;

    info!("UDP connected to {}", addr);
    Ok(())
}

async fn negotiate(socket: &UdpSocket) -> Result<Option<u32>> {
    static NEXT_TOKEN: AtomicU32 = AtomicU32::new(0);
    let time = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u32;
    let token = time ^ NEXT_TOKEN.fetch_add(1, Ordering::Relaxed);
    let hello = Header {
        kind: Kind::Hello,
        token,
        sequence: 0,
    }
    .encode();
    let mut bytes = [0; HEADER_BYTES + 1];
    for _ in 0..3 {
        socket.send(&hello).await?;
        if let Ok(Ok(())) = timeout(Duration::from_millis(45), async {
            loop {
                let len = socket.recv(&mut bytes).await?;
                if Header::decode(&bytes[..len]).is_some_and(|(header, _)| {
                    header.kind == Kind::Ready && header.token == token && header.sequence == 0
                }) {
                    return Ok::<(), std::io::Error>(());
                }
            }
        })
        .await
        {
            return Ok(Some(token));
        }
    }
    Ok(None)
}

fn register_sequenced(
    stack: &EdgeStack,
    queue: &StdQueue,
    socket: UdpSocket,
    token: u32,
    notify: Option<Arc<WaitQueue>>,
) -> Result<()> {
    let closer = Arc::new(WaitQueue::new());
    stack.manage_profile(|profile| {
        anyhow::ensure!(
            matches!(
                profile.interface_state(()),
                Some(InterfaceState::Down) | None
            ),
            "UDP interface not in Down state"
        );
        profile.set_closer(closer.clone());
        profile
            .set_interface_state((), InterfaceState::edge_link_local())
            .map_err(|_| anyhow::anyhow!("failed to activate UDP interface"))?;
        Ok(())
    })?;
    if let Some(notify) = &notify {
        notify.wake_all();
    }
    let socket = Arc::new(socket);
    tokio::spawn(
        UdpTxWorker {
            socket: socket.clone(),
            consumer: BbqHandle::framed_consumer(queue),
            closer: closer.clone(),
            peer_rx: None,
        }
        .run(),
    );
    let stack = stack.clone();
    tokio::spawn(async move {
        tokio::select! {
            _ = closer.wait() => {},
            result = receive(&stack, &socket, token, notify.as_deref()) => {
                if let Err(error) = result {
                    tracing::warn!(%error, "sequenced UDP link closed");
                }
            },
        }
        closer.close();
        let _ =
            stack.manage_profile(|profile| profile.set_interface_state((), InterfaceState::Down));
        if let Some(notify) = notify {
            notify.wake_all();
        }
    });
    Ok(())
}

async fn receive(
    stack: &EdgeStack,
    socket: &UdpSocket,
    token: u32,
    notify: Option<&WaitQueue>,
) -> Result<()> {
    let mut pending = BTreeMap::<u32, Vec<u8>>::new();
    let mut next = 0_u32;
    let mut bytes = [0; UDP_MTU as usize + 1];
    let mut processor = EdgeFrameProcessor::new();
    let liveness = Duration::from_millis(LIVENESS_TIMEOUT_MS);
    let mut deadline = Instant::now() + liveness;
    let mut retry_at = Instant::now() + QUIET_REPLAY;
    let mut gap_deadline: Option<Instant> = None;

    loop {
        let limit = gap_deadline.map_or(deadline, |gap| gap.min(deadline));
        tokio::select! {
            result = socket.recv(&mut bytes) => {
                let len = result?;
                let Some((header, frame)) = Header::decode(&bytes[..len]) else { continue };
                if header.kind != Kind::Frame || header.token != token {
                    continue;
                }
                let offset = header.sequence.wrapping_sub(next);
                if offset >= (1_u32 << 31) {
                    continue;
                }
                anyhow::ensure!(offset < 2 * HISTORY_PACKETS as u32, "UDP frame outside receive window");
                deadline = Instant::now() + liveness;
                let mut deliver = |frame: &[u8]| {
                    if processor.process_frame(frame, stack, ()) && let Some(notify) = notify {
                        notify.wake_all();
                    }
                };
                if offset == 0 {
                    deliver(frame);
                    next = next.wrapping_add(1);
                    while let Some(ready) = pending.remove(&next) {
                        deliver(&ready);
                        next = next.wrapping_add(1);
                    }
                    gap_deadline = None;
                    retry_at = Instant::now() + QUIET_REPLAY;
                } else {
                    pending.entry(header.sequence).or_insert_with(|| frame.to_vec());
                }
                if !pending.is_empty() && gap_deadline.is_none() {
                    let now = Instant::now();
                    gap_deadline = Some(now + GAP_TIMEOUT);
                    retry_at = now;
                }
            }
            _ = sleep_until(retry_at) => {
                // Also probe for a missing last packet after traffic goes quiet.
                let request = Header { kind: Kind::Replay, token, sequence: next }.encode();
                let _ = socket.try_send(&request);
                retry_at = Instant::now() + if gap_deadline.is_some() { REPLAY_INTERVAL } else { QUIET_REPLAY };
            }
            _ = sleep_until(limit) => {
                anyhow::bail!("UDP frame {next} was not recovered before timeout");
            }
        }
    }
}

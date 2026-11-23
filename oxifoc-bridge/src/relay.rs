//! The fast telemetry session, held for a host.
//!
//! A host asks the bridge for a session instead of the controller. The bridge
//! passes the request on from its own address, so the controller sends its
//! Packed28 blocks here, and they leave for the host as Delta28, about a
//! third of the size. The controller's reader is then always present: when
//! the host's link goes down the bridge stops the session, and another host
//! can ask for one.
//!
//! The requests and replies are the controller's own, so a host follows the
//! same session rules either way.

use core::{
    cell::Cell,
    pin::pin,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_futures::join::join4;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use embassy_time::{Duration, Instant, with_timeout};
use ergot::{
    Address, FrameKind, Key,
    exports::bbqueue::traits::coordination::cas::AtomicCoord,
    interface_manager::{InterfaceSendError, profiles::direct_edge::EDGE_NODE_ID},
    net_stack::NetStackSendError,
    socket::{Attributes, HeaderMessage, borrow::Socket},
    toolkits::embedded_io_async_v0_7 as kit,
    traits::Topic,
};
use oxifoc_core::{
    icd::{
        RawRelayEndpoint, RawTelemetryBlockRef, RawTelemetryBlockRefTopic, RawTelemetryBlockTopic,
        RawTelemetryEndpoint, VelocityDebugArmEndpoint, VelocityDebugRelayEndpoint,
    },
    raw_delta::compress_envelope,
    raw_telemetry::{
        RAW_MAX_BLOCK_BYTES, RawEncoding, RawTelemetryAck, RawTelemetryRequest, RawTelemetryStatus,
        VelocityDebugArm, raw_block_header, raw_block_len,
    },
    spi_link::PAYLOAD_BYTES,
    types::{ControlTiming, TelemetryConfigAck},
};

use static_cell::StaticCell;

use crate::transport::{MTU, Stack};

/// Envelopes wait here as the frames they arrived in. One SPI slot brings at
/// most [`PAYLOAD_BYTES`] of them. Twice that keeps one slot's worth of
/// contiguous room in an empty queue wherever its pointers stand.
type Queue = kit::Queue<{ 2 * PAYLOAD_BYTES }, AtomicCoord>;
static ENVELOPES: Queue = Queue::new();
/// The envelope being sent on, outside the task's future.
static REWRITTEN: StaticCell<[u8; RAW_MAX_BLOCK_BYTES]> = StaticCell::new();

/// One attempt at a request to the controller. The host waits 600 ms for its
/// own reply, which the attempts together stay under.
const REPLY: Duration = Duration::from_millis(150);
const ATTEMPTS: usize = 3;

/// The host the session belongs to, with its port cleared.
static OWNER: Mutex<CriticalSectionRawMutex, Cell<Option<Address>>> = Mutex::new(Cell::new(None));
/// A session whose blocks have nowhere to go, for [`release`] to stop.
static ABANDONED: Signal<CriticalSectionRawMutex, u64> = Signal::new();
/// Envelopes sent on, their bytes as received and as sent, blocks that never
/// arrived, envelopes lost here, and the microseconds spent rewriting and
/// queueing them. All wrap.
pub static ENVELOPES_SENT: AtomicU32 = AtomicU32::new(0);
pub static BYTES_IN: AtomicU32 = AtomicU32::new(0);
pub static BYTES_OUT: AtomicU32 = AtomicU32::new(0);
pub static MISSED: AtomicU32 = AtomicU32::new(0);
pub static DROPPED: AtomicU32 = AtomicU32::new(0);
pub static BUSY_US: AtomicU32 = AtomicU32::new(0);

fn owner() -> Option<Address> {
    OWNER.lock(Cell::get)
}

fn set_owner(host: Option<Address>) {
    OWNER.lock(|owner| owner.set(host));
}

/// The network of the host that takes the stream, while a session is held.
pub fn stream_net() -> Option<u16> {
    owner().map(|host| host.network_id)
}

/// Wait until the queue has room for the envelopes of one more SPI slot. The
/// slot task routes its next slot after this, so no envelope is refused.
pub async fn room() {
    // The probe reserves nothing: an uncommitted grant is released.
    drop(
        ENVELOPES
            .stream_producer()
            .wait_grant_exact(PAYLOAD_BYTES)
            .await,
    );
}

fn add(counter: &AtomicU32, value: usize) {
    counter.fetch_add(value as u32, Ordering::Relaxed);
}

#[derive(Clone, Copy)]
enum Request {
    Raw(RawTelemetryRequest),
    Velocity(VelocityDebugArm),
}

async fn ask(
    stack: &'static Stack,
    controller: Address,
    request: Request,
) -> Option<RawTelemetryAck> {
    for _ in 0..ATTEMPTS {
        let endpoints = stack.endpoints();
        let reply = match request {
            Request::Raw(raw) => {
                let reply = endpoints.request::<RawTelemetryEndpoint>(controller, &raw, None);
                with_timeout(REPLY, reply).await
            }
            Request::Velocity(arm) => {
                let reply = endpoints.request::<VelocityDebugArmEndpoint>(controller, &arm, None);
                with_timeout(REPLY, reply).await
            }
        };
        if let Ok(Ok(ack)) = reply {
            return Some(ack);
        }
    }
    None
}

/// The session is another host's, or the controller did not answer. As
/// from the controller, the reply names the session the request did.
fn busy(request: Request) -> RawTelemetryAck {
    let session = match request {
        Request::Raw(RawTelemetryRequest::Arm { session, .. })
        | Request::Raw(RawTelemetryRequest::Stop { session })
        | Request::Velocity(VelocityDebugArm { session, .. }) => session,
        Request::Raw(RawTelemetryRequest::Status) => 0,
    };
    RawTelemetryAck {
        session,
        encoding: RawEncoding::Packed28,
        status: RawTelemetryStatus::Busy,
        timing: TelemetryConfigAck {
            actual_fast_hz: 0,
            decimation_m: None,
            control_timing: ControlTiming {
                clock_hz: 0,
                period_ticks: 0,
            },
        },
    }
}

async fn control(
    stack: &'static Stack,
    controller: Address,
    from: Address,
    request: Request,
) -> RawTelemetryAck {
    let host = Address { port_id: 0, ..from };
    let status = Request::Raw(RawTelemetryRequest::Status);
    let asks_status = matches!(request, Request::Raw(RawTelemetryRequest::Status));
    let arms = matches!(
        request,
        Request::Velocity(_) | Request::Raw(RawTelemetryRequest::Arm { .. })
    );
    let holder = owner();
    if !asks_status && holder.is_some_and(|holder| holder != host) {
        // The session is its host's for as long as the controller runs it.
        let running = ask(stack, controller, status).await.is_none_or(|ack| {
            matches!(
                ack.status,
                RawTelemetryStatus::Armed | RawTelemetryStatus::Stopping
            )
        });
        if running {
            return busy(request);
        }
    }
    if arms {
        // Before the request leaves: the first block can arrive in the same
        // SPI slot as the acknowledgement, and is read first.
        set_owner(Some(host));
    }
    let ack = ask(stack, controller, request).await;
    if arms && ack.is_none_or(|ack| ack.status != RawTelemetryStatus::Armed) {
        // A host whose earlier session is still draining keeps it.
        set_owner(holder.filter(|holder| *holder == host));
    }
    ack.unwrap_or_else(|| busy(request))
}

/// Stop the sessions the block loop finds abandoned. The loop itself never
/// waits: the slot task holds the SPI link while the queue here is full, and
/// the controller's reply comes over that link.
async fn release(stack: &'static Stack, controller: Address) {
    loop {
        let session = ABANDONED.wait().await;
        let stop = Request::Raw(RawTelemetryRequest::Stop { session });
        if ask(stack, controller, stop).await.is_none() {
            log::warn!("raw relay: an abandoned session was not stopped");
        }
    }
}

async fn blocks(stack: &'static Stack) {
    // The owned receiver would copy each envelope out byte by byte and keep
    // them at their full size; this one keeps the received frame.
    let socket = Socket::<&'static Queue, RawTelemetryBlockRef<'static>, &'static Stack>::new(
        stack,
        Key(RawTelemetryBlockTopic::TOPIC_KEY.to_bytes()),
        Attributes {
            kind: FrameKind::TOPIC_MSG,
            discoverable: true,
        },
        &ENVELOPES,
        MTU,
        Some("raw_relay"),
    );
    let socket = pin!(socket);
    let mut envelopes = socket.attach();
    let out = REWRITTEN.init_with(|| [0; RAW_MAX_BLOCK_BYTES]);
    let mut abandoned = None;
    let mut next = None;
    loop {
        let grant = envelopes.recv().await;
        let Some(Ok(message)) = grant.try_access() else {
            continue;
        };
        let envelope = message.t.data;
        let Some(blocks) = Blocks::of(envelope) else {
            add(&DROPPED, 1);
            continue;
        };
        if let Some((session, index)) = next
            && session == blocks.session
        {
            // Lost on the way here: a damaged SPI slot, or one the slot
            // task routed while the queue here was full.
            add(&MISSED, blocks.first.wrapping_sub(index) as usize);
        }
        next = Some((blocks.session, blocks.first.wrapping_add(blocks.count)));
        let started = Instant::now();
        let sent = match owner() {
            Some(host) => compress_envelope(envelope, out).ok().map(|len| {
                let block = RawTelemetryBlockRef { data: &out[..len] };
                let sent = stack
                    .topics()
                    .unicast_borrowed::<RawTelemetryBlockRefTopic<'_>>(host, &block);
                (len, sent)
            }),
            // The bridge restarted under a session, or its host has gone.
            None => Some((0, Err(NetStackSendError::NoRoute))),
        };
        add(&BUSY_US, started.elapsed().as_micros() as usize);
        match sent {
            Some((len, Ok(()))) => {
                add(&ENVELOPES_SENT, 1);
                add(&BYTES_IN, envelope.len());
                add(&BYTES_OUT, len);
            }
            Some((
                _,
                Err(
                    NetStackSendError::NoRoute
                    | NetStackSendError::InterfaceSend(InterfaceSendError::NoRouteToDest),
                ),
            )) => {
                // The host's link is down. The controller would otherwise
                // stream to it for good and refuse every other host.
                set_owner(None);
                if abandoned != Some(blocks.session) {
                    abandoned = Some(blocks.session);
                    ABANDONED.signal(blocks.session);
                }
                continue;
            }
            _ => add(&DROPPED, 1),
        }
        if blocks.ended {
            set_owner(None);
        }
    }
}

/// What the relay reads from the blocks of an envelope.
struct Blocks {
    session: u64,
    /// Index of the first block; a session numbers all its blocks in one
    /// sequence.
    first: u32,
    count: u32,
    /// A block ends the session: the END of the native records, which
    /// follows the END of the velocity records.
    ended: bool,
}

impl Blocks {
    fn of(mut envelope: &[u8]) -> Option<Self> {
        let first = raw_block_header(envelope).ok()?;
        let mut blocks = Self {
            session: first.session,
            first: first.block_index,
            count: 0,
            ended: false,
        };
        while !envelope.is_empty() {
            let block = envelope.get(..raw_block_len(envelope).ok()?)?;
            let header = raw_block_header(block).ok()?;
            blocks.count += 1;
            blocks.ended |= header.is_end() && !header.is_velocity();
            envelope = &envelope[block.len()..];
        }
        Some(blocks)
    }
}

#[embassy_executor::task]
pub async fn run(stack: &'static Stack, spi_ident: u8) {
    let net = stack
        .manage_profile(|p| p.net_id_of(spi_ident))
        .expect("registered SPI segment");
    let controller = Address {
        network_id: net,
        node_id: EDGE_NODE_ID,
        port_id: 0,
    };
    let raw = async {
        let server = stack
            .endpoints()
            .bounded_server::<RawRelayEndpoint, 2>(Some("raw_relay"));
        let server = pin!(server);
        let mut handle = server.attach();
        loop {
            let _ = handle
                .serve_full(async |msg: &HeaderMessage<RawTelemetryRequest>| {
                    control(stack, controller, msg.hdr.src, Request::Raw(msg.t)).await
                })
                .await;
        }
    };
    let velocity = async {
        let server = stack
            .endpoints()
            .bounded_server::<VelocityDebugRelayEndpoint, 2>(Some("raw_relay_velocity"));
        let server = pin!(server);
        let mut handle = server.attach();
        loop {
            let _ = handle
                .serve_full(async |msg: &HeaderMessage<VelocityDebugArm>| {
                    control(stack, controller, msg.hdr.src, Request::Velocity(msg.t)).await
                })
                .await;
        }
    };
    join4(raw, velocity, blocks(stack), release(stack, controller)).await;
}

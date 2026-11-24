//! The fast telemetry session, held for a host or recorded.
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
//!
//! On a board with memory for it the bridge can also keep a session itself
//! (`cmd/raw_record`): it opens the session, stores the Delta28 blocks, and
//! any host fetches them later. See [`oxifoc_core::raw_record`].

use core::{
    cell::{Cell, RefCell},
    pin::pin,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_futures::join::{join, join3};
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
    socket::{
        Attributes, HeaderMessage,
        borrow::{ResponseGrant, Socket},
    },
    toolkits::embedded_io_async_v0_7 as kit,
    traits::Topic,
};
use oxifoc_core::{
    foc::fault::FaultSeverity,
    icd::{
        FaultTopic, RawRecordEndpoint, RawRelayEndpoint, RawTelemetryBlockRef,
        RawTelemetryBlockRefTopic, RawTelemetryBlockTopic, RawTelemetryEndpoint,
        VelocityDebugArmEndpoint, VelocityDebugRelayEndpoint,
    },
    raw_delta::compress_envelope,
    raw_record::{
        RecordChunk, RecordEnd, RecordMode, RecordRefusal, RecordRequest, RecordResponse,
        RecordState, RecordStatus, Recording,
    },
    raw_telemetry::{
        RAW_MAX_BLOCK_BYTES, RawEncoding, RawTelemetryAck, RawTelemetryRequest, RawTelemetryStatus,
        VelocityDebugArm, raw_block_header, raw_block_len,
    },
    spi_link::PAYLOAD_BYTES,
    types::{ControlTiming, FaultSnapshot, TelemetryConfigAck},
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
/// The controller's fault snapshots; the largest is about a kilobyte.
type FaultQueue = kit::Queue<2048, AtomicCoord>;
static FAULTS: FaultQueue = FaultQueue::new();

/// One attempt at a request to the controller. The host waits 600 ms for its
/// own reply, which the attempts together stay under.
const REPLY: Duration = Duration::from_millis(150);
const ATTEMPTS: usize = 3;
/// What a ring still records after a fault has stopped the motor.
const AFTER_FAULT: Duration = Duration::from_secs(1);

/// Where the session's blocks go.
#[derive(Clone, Copy, PartialEq)]
enum Owner {
    /// To this host, with its port cleared.
    Host(Address),
    /// Into the bridge's own recording.
    Recorder,
}

static OWNER: Mutex<CriticalSectionRawMutex, Cell<Option<Owner>>> = Mutex::new(Cell::new(None));
/// A session whose blocks have nowhere to go, for [`release`] to stop.
static ABANDONED: Signal<CriticalSectionRawMutex, u64> = Signal::new();
/// Envelopes sent on or recorded, their bytes as received and as kept,
/// blocks that never arrived, envelopes lost here, and the microseconds
/// spent rewriting and queueing them. All wrap.
pub static ENVELOPES_SENT: AtomicU32 = AtomicU32::new(0);
pub static BYTES_IN: AtomicU32 = AtomicU32::new(0);
pub static BYTES_OUT: AtomicU32 = AtomicU32::new(0);
pub static MISSED: AtomicU32 = AtomicU32::new(0);
pub static DROPPED: AtomicU32 = AtomicU32::new(0);
pub static BUSY_US: AtomicU32 = AtomicU32::new(0);

fn owner() -> Option<Owner> {
    OWNER.lock(Cell::get)
}

fn set_owner(owner: Option<Owner>) {
    OWNER.lock(|cell| cell.set(owner));
}

/// The network of the host that takes the stream, while a host holds the
/// session.
pub fn stream_net() -> Option<u16> {
    match owner() {
        Some(Owner::Host(host)) => Some(host.network_id),
        Some(Owner::Recorder) | None => None,
    }
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

fn runs(status: RawTelemetryStatus) -> bool {
    matches!(
        status,
        RawTelemetryStatus::Armed | RawTelemetryStatus::Stopping
    )
}

const NO_TIMING: TelemetryConfigAck = TelemetryConfigAck {
    actual_fast_hz: 0,
    decimation_m: None,
    control_timing: ControlTiming {
        clock_hz: 0,
        period_ticks: 0,
    },
};

/// The session is another's, or the controller did not answer. As from the
/// controller, the reply names the session the request did.
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
        timing: NO_TIMING,
    }
}

async fn control(
    stack: &'static Stack,
    controller: Address,
    from: Address,
    request: Request,
) -> RawTelemetryAck {
    let host = Owner::Host(Address { port_id: 0, ..from });
    let status = Request::Raw(RawTelemetryRequest::Status);
    let asks_status = matches!(request, Request::Raw(RawTelemetryRequest::Status));
    let arms = matches!(
        request,
        Request::Velocity(_) | Request::Raw(RawTelemetryRequest::Arm { .. })
    );
    let holder = owner();
    if !asks_status && holder.is_some_and(|holder| holder != host) {
        // The session is its owner's for as long as the controller runs it.
        let state = ask(stack, controller, status).await;
        if state.is_none_or(|ack| runs(ack.status)) {
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

/// A recording and the session that fills it.
struct Recorder {
    recording: Recording<'static>,
    state: RecordState,
    mode: RecordMode,
    session: u64,
    started_unix_ms: u64,
    timing: TelemetryConfigAck,
    missed: u32,
    /// When a ring ends that saw a fault stop the motor.
    ends_at: Option<Instant>,
}

impl Recorder {
    fn new(memory: &'static mut [u8]) -> Self {
        Self {
            recording: Recording::new(memory),
            state: RecordState::Empty,
            mode: RecordMode::UntilFull,
            session: 0,
            started_unix_ms: 0,
            timing: NO_TIMING,
            missed: 0,
            ends_at: None,
        }
    }

    fn status(&self) -> RecordStatus {
        RecordStatus {
            state: self.state,
            mode: self.mode,
            session: self.session,
            started_unix_ms: self.started_unix_ms,
            timing: self.timing,
            bytes: self.recording.len() as u32,
            capacity: self.recording.capacity() as u32,
            blocks: self.recording.blocks(),
            missed: self.missed,
            overwritten: self.recording.overwritten(),
        }
    }

    fn clear(&mut self) {
        self.recording.clear();
        (self.state, self.session, self.started_unix_ms) = (RecordState::Empty, 0, 0);
        (self.timing, self.missed, self.ends_at) = (NO_TIMING, 0, None);
    }

    fn stop(&mut self, end: RecordEnd) {
        if self.state == RecordState::Recording {
            self.state = RecordState::Stopped(end);
        }
    }

    /// A fault has stopped the motor: a ring records a little longer and
    /// then keeps what led up to it.
    fn motor_stopped(&mut self) {
        if self.state == RecordState::Recording && self.mode == RecordMode::Ring {
            self.ends_at.get_or_insert(Instant::now() + AFTER_FAULT);
        }
    }

    /// Takes an envelope of the session. False once the recording is over.
    fn take(&mut self, envelope: &[u8], blocks: &Blocks, missing: u32) -> bool {
        if self.state != RecordState::Recording || blocks.session != self.session {
            return false;
        }
        if blocks.ended {
            self.stop(RecordEnd::SessionEnded);
        } else if self.ends_at.is_some_and(|at| Instant::now() >= at) {
            self.stop(RecordEnd::Fault);
        } else if !self.recording.push(envelope, self.mode) {
            self.stop(RecordEnd::Full);
        } else {
            self.missed = self.missed.wrapping_add(missing);
        }
        self.state == RecordState::Recording
    }
}

async fn record(
    stack: &'static Stack,
    controller: Address,
    recorder: &RefCell<Recorder>,
    request: RecordRequest,
) -> RecordResponse {
    let status = || RecordResponse::Status(recorder.borrow().status());
    let state = recorder.borrow().state;
    let kept = recorder.borrow().session;
    match request {
        RecordRequest::Status => status(),
        RecordRequest::Start {
            session,
            fast_hz,
            mode,
            started_unix_ms,
        } => {
            if recorder.borrow().recording.capacity() == 0 {
                return RecordResponse::Refused(RecordRefusal::NoMemory);
            }
            if session == 0 || fast_hz == 0 {
                return RecordResponse::Refused(RecordRefusal::Invalid);
            }
            match state {
                // A repeat of the Start that began this recording.
                _ if state != RecordState::Empty && kept == session => return status(),
                RecordState::Recording => {
                    return RecordResponse::Refused(RecordRefusal::Recording);
                }
                RecordState::Stopped(_) => return RecordResponse::Refused(RecordRefusal::Kept),
                RecordState::Empty => {}
            }
            // The Arm names the session the controller had last.
            let asked = Request::Raw(RawTelemetryRequest::Status);
            let Some(last) = ask(stack, controller, asked).await else {
                return RecordResponse::Refused(RecordRefusal::Controller);
            };
            if runs(last.status) {
                return RecordResponse::Refused(RecordRefusal::Busy);
            }
            {
                let mut recorder = recorder.borrow_mut();
                recorder.clear();
                (recorder.state, recorder.mode) = (RecordState::Recording, mode);
                (recorder.session, recorder.started_unix_ms) = (session, started_unix_ms);
            }
            // Before the request leaves, as for a host.
            set_owner(Some(Owner::Recorder));
            let arm = Request::Raw(RawTelemetryRequest::Arm {
                session,
                previous_session: last.session,
                encoding: RawEncoding::Packed28,
                fast_hz,
                max_block_bytes: RAW_MAX_BLOCK_BYTES as u16,
            });
            match ask(stack, controller, arm).await {
                Some(ack) if ack.status == RawTelemetryStatus::Armed => {
                    recorder.borrow_mut().timing = ack.timing;
                    status()
                }
                _ => {
                    set_owner(None);
                    recorder.borrow_mut().clear();
                    RecordResponse::Refused(RecordRefusal::Controller)
                }
            }
        }
        RecordRequest::Stop => {
            if state == RecordState::Recording {
                recorder.borrow_mut().stop(RecordEnd::Request);
                set_owner(None);
                ABANDONED.signal(kept);
            }
            status()
        }
        RecordRequest::Read { offset, max_bytes } => {
            if state == RecordState::Recording {
                return RecordResponse::Refused(RecordRefusal::Recording);
            }
            let mut chunk = RecordChunk {
                data: heapless::Vec::new(),
            };
            let len = usize::from(max_bytes).min(chunk.data.capacity());
            // The capacity bounds the length.
            let _ = chunk.data.resize_default(len);
            let copied = recorder
                .borrow()
                .recording
                .read(offset as usize, &mut chunk.data);
            chunk.data.truncate(copied);
            RecordResponse::Data(chunk)
        }
        RecordRequest::Clear => {
            if state == RecordState::Recording {
                return RecordResponse::Refused(RecordRefusal::Recording);
            }
            recorder.borrow_mut().clear();
            status()
        }
    }
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

/// Tell the recorder when a fault stops the motor.
async fn faults(stack: &'static Stack, recorder: &RefCell<Recorder>) {
    let socket = Socket::<&'static FaultQueue, FaultSnapshot, &'static Stack>::new(
        stack,
        Key(FaultTopic::TOPIC_KEY.to_bytes()),
        Attributes {
            kind: FrameKind::TOPIC_MSG,
            discoverable: true,
        },
        &FAULTS,
        MTU,
        Some("raw_relay_faults"),
    );
    let socket = pin!(socket);
    let mut snapshots = socket.attach_broadcast();
    loop {
        let grant = snapshots.recv().await;
        if stops_motor(&grant) {
            recorder.borrow_mut().motor_stopped();
        }
    }
}

/// True for a snapshot with a fault that stops the motor. Not async, so the
/// decoded snapshot, about a kilobyte, stays off the task's future.
fn stops_motor(grant: &ResponseGrant<'_, &'static FaultQueue, FaultSnapshot>) -> bool {
    grant.try_access().is_some_and(|snapshot| {
        snapshot.is_ok_and(|message| {
            let mut faults = message.t.faults.iter();
            faults.any(|fault| fault.severity != FaultSeverity::Warning)
        })
    })
}

async fn blocks(stack: &'static Stack, recorder: &RefCell<Recorder>) {
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
        // Lost on the way here: a damaged SPI slot, or one the slot task
        // routed while the queue here was full.
        let missing = match next {
            Some((session, index)) if session == blocks.session => blocks.first.wrapping_sub(index),
            _ => 0,
        };
        add(&MISSED, missing as usize);
        next = Some((blocks.session, blocks.first.wrapping_add(blocks.count)));

        let holder = owner();
        let started = Instant::now();
        let rewritten = match holder {
            Some(_) => compress_envelope(envelope, out).ok(),
            None => None,
        };
        // True while the session still has somewhere to go.
        let kept = match (holder, rewritten) {
            (Some(Owner::Host(host)), Some(len)) => {
                let block = RawTelemetryBlockRef { data: &out[..len] };
                let sent = stack
                    .topics()
                    .unicast_borrowed::<RawTelemetryBlockRefTopic<'_>>(host, &block);
                match sent {
                    Ok(()) => {
                        add(&ENVELOPES_SENT, 1);
                        add(&BYTES_IN, envelope.len());
                        add(&BYTES_OUT, len);
                        !blocks.ended
                    }
                    // The host's link is down. The controller would
                    // otherwise stream to it for good and refuse every
                    // other host.
                    Err(
                        NetStackSendError::NoRoute
                        | NetStackSendError::InterfaceSend(InterfaceSendError::NoRouteToDest),
                    ) => false,
                    Err(_) => {
                        add(&DROPPED, 1);
                        !blocks.ended
                    }
                }
            }
            (Some(Owner::Recorder), Some(len)) => {
                let recording = recorder.borrow_mut().take(&out[..len], &blocks, missing);
                if recording {
                    add(&ENVELOPES_SENT, 1);
                    add(&BYTES_IN, envelope.len());
                    add(&BYTES_OUT, len);
                }
                recording
            }
            (Some(_), None) => {
                add(&DROPPED, 1);
                true
            }
            // The bridge restarted under a session, or its owner has gone.
            (None, _) => false,
        };
        add(&BUSY_US, started.elapsed().as_micros() as usize);
        if !kept {
            set_owner(None);
            if !blocks.ended && abandoned != Some(blocks.session) {
                abandoned = Some(blocks.session);
                ABANDONED.signal(blocks.session);
            }
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

/// `memory` holds recordings; a board without any passes an empty slice.
#[embassy_executor::task]
pub async fn run(stack: &'static Stack, spi_ident: u8, memory: &'static mut [u8]) {
    let net = stack
        .manage_profile(|p| p.net_id_of(spi_ident))
        .expect("registered SPI segment");
    let controller = Address {
        network_id: net,
        node_id: EDGE_NODE_ID,
        port_id: 0,
    };
    let recorder = RefCell::new(Recorder::new(memory));
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
    let recording = async {
        let server = stack
            .endpoints()
            .bounded_server::<RawRecordEndpoint, 2>(Some("raw_record"));
        let server = pin!(server);
        let mut handle = server.attach();
        loop {
            let _ = handle
                .serve_full(async |msg: &HeaderMessage<RecordRequest>| {
                    record(stack, controller, &recorder, msg.t).await
                })
                .await;
        }
    };
    join(
        join3(raw, velocity, recording),
        join3(
            blocks(stack, &recorder),
            release(stack, controller),
            faults(stack, &recorder),
        ),
    )
    .await;
}

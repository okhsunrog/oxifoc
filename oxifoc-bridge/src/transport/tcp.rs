//! Ergot over one TCP connection: COBS frames on a byte stream, as on USB.
//!
//! TCP repeats lost segments and keeps the stream ordered, so neither side
//! needs a sequence or replay layer of its own. [`OUT`] holds the backlog
//! while a loss is repaired, about 170 ms of a 20 kHz stream as the relay
//! sends it; the socket keeps only the window in flight. A frame the queue
//! refuses is counted in [`DROPPED`].

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{
    Stack as NetStack,
    tcp::{Error, TcpSocket, TcpWriter},
};
use embassy_time::{Duration, with_timeout};
use ergot::interface_manager::{
    InterfaceState, LivenessConfig, Profile, profiles::router::RouterFrameProcessor,
    transports::eio::RxWorker, utils::cobs_stream,
};
use static_cell::StaticCell;

use super::{BridgeSink, Counted, LINK, MTU, STATE_NOTIFY, Stack, WIFI_CONFIGURED, WifiQueue};

const PORT: u16 = 2025;
/// Unacknowledged bytes the socket retains. smoltcp has no selective
/// acknowledgement and resends everything after a lost segment, so a large
/// window makes every loss expensive: with 56 KiB here the buffer filled and
/// native-rate captures lost frames. The backlog waits in [`OUT`] instead.
const TX_BYTES: usize = 16 * 1024;
const RX_BYTES: usize = 4096;
/// Largest COBS-encoded frame from the host, as on USB.
const FRAME_BYTES: usize = 4096;
/// A silent peer is probed every second and dropped after five.
const KEEP_ALIVE: Duration = Duration::from_secs(1);
const TIMEOUT: Duration = Duration::from_secs(5);

static OUT: WifiQueue = WifiQueue::new();
static TX_BUFFER: StaticCell<[u8; TX_BYTES]> = StaticCell::new();
// The receive side lives in RTC RAM, to leave stack room for BLE pairing.
#[esp_hal::ram(unstable(rtc_fast))]
static RX_BUFFER: StaticCell<[u8; RX_BYTES]> = StaticCell::new();
#[esp_hal::ram(unstable(rtc_fast))]
static FRAME: StaticCell<[u8; FRAME_BYTES]> = StaticCell::new();

/// The network of the TCP segment, once assigned.
pub static NET: AtomicU16 = AtomicU16::new(0);
pub static CONNECTED: AtomicBool = AtomicBool::new(false);
pub static CONNECTS: AtomicU32 = AtomicU32::new(0);
pub static DROPPED: AtomicU32 = AtomicU32::new(0);
/// Most bytes the socket's transmit buffer has held.
pub static MAX_BACKLOG: AtomicU32 = AtomicU32::new(0);

/// Wait until [`OUT`] has contiguous room for `bytes`. Returns at once
/// without a host: the router does not queue for a closed link.
pub async fn room(bytes: usize) {
    if CONNECTED.load(Ordering::Relaxed) {
        // The probe reserves nothing: an uncommitted grant is released.
        drop(OUT.stream_producer().wait_grant_exact(bytes).await);
    }
}

pub fn start(spawner: Spawner, router: &'static Stack, stack: NetStack<'static>) {
    // Without credentials the task and its buffers stay out of the image.
    if !WIFI_CONFIGURED {
        return;
    }
    let task = run(router, stack).expect("Ergot TCP task is spawned once");
    // Below the SPI slot task's priority the queue overflowed while the
    // socket was nearly empty.
    task.metadata().set_priority(LINK);
    spawner.spawn(task);
}

#[embassy_executor::task]
async fn run(router: &'static Stack, stack: NetStack<'static>) {
    let sink = BridgeSink::Wifi(Counted::new(
        cobs_stream::Sink::new(OUT.stream_producer(), MTU),
        &DROPPED,
    ));
    let Ok(ident) = super::register(router, sink) else {
        log::warn!("Ergot TCP has no router interface slot");
        return;
    };
    // Registration activates the segment; no host is connected yet.
    let _ = router.manage_profile(|p| p.set_interface_state(ident, InterfaceState::Down));
    let Some(net) = router.manage_profile(|p| p.net_id_of(ident)) else {
        return;
    };
    NET.store(net, Ordering::Relaxed);
    let rx_buffer = RX_BUFFER.init_with(|| [0; RX_BYTES]);
    let tx_buffer = TX_BUFFER.init_with(|| [0; TX_BYTES]);
    let frame = FRAME.init_with(|| [0; FRAME_BYTES]);
    let mut scratch = [0; 512];
    loop {
        stack.wait_config_up().await;
        let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
        // Command replies are single small segments.
        socket.set_nagle_enabled(false);
        socket.set_keep_alive(Some(KEEP_ALIVE));
        socket.set_timeout(Some(TIMEOUT));
        let _ = select(stack.wait_config_down(), async {
            if socket.accept(PORT).await.is_ok() {
                CONNECTS.fetch_add(1, Ordering::Relaxed);
                CONNECTED.store(true, Ordering::Relaxed);
                session(router, &mut socket, ident, net, frame, &mut scratch).await;
            }
        })
        .await;
        CONNECTED.store(false, Ordering::Relaxed);
        socket.abort();
        let _ = with_timeout(Duration::from_millis(150), socket.flush()).await;
    }
}

/// One host connection, until either direction fails or the host closes it.
async fn session(
    router: &'static Stack,
    socket: &mut TcpSocket<'_>,
    ident: u8,
    net: u16,
    frame: &mut [u8],
    scratch: &mut [u8],
) {
    // A previous connection may have left part of a frame behind.
    let stale = OUT.stream_consumer();
    while let Ok(grant) = stale.read() {
        let len = grant.len();
        grant.release(len);
    }
    let (reader, mut writer) = socket.split();
    // Dropping the worker leaves the interface Down for the next connection.
    let mut rx = RxWorker::new(router, reader, RouterFrameProcessor::new(net), ident)
        .with_liveness(LivenessConfig { timeout_ms: 5000 })
        .with_state_notify(&STATE_NOTIFY);
    let _ = select(
        rx.run(InterfaceState::Inactive, frame, scratch),
        transmit(&mut writer),
    )
    .await;
}

/// Ergot's `tx_worker`, with the socket backlog recorded after each write.
async fn transmit(writer: &mut TcpWriter<'_>) -> Result<(), Error> {
    let consumer = OUT.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let used = writer.write(&grant).await?;
        grant.release(used);
        MAX_BACKLOG.fetch_max(writer.send_queue() as u32, Ordering::Relaxed);
    }
}

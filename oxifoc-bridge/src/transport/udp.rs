//! One Ergot UDP peer on an explicitly configured Wi-Fi host address.

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::{
    select::{Either, select},
    yield_now,
};
use embassy_net::{
    IpAddress, IpEndpoint, Stack as NetStack,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_time::{Duration, Instant, Timer};
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::router::RouterFrameProcessor,
    utils::framed_stream,
};
use ergot::wire_frames::de_frame;
use oxifoc_core::{
    icd::{LIVENESS_TIMEOUT_MS, UDP_MTU},
    udp_link::{FRAME_MTU, HEADER_BYTES, HISTORY_PACKETS, Header, Kind},
};
use static_cell::StaticCell;

use super::{BridgeSink, Stack, WifiQueue};

const UDP_PORT: u16 = 2025;
const FRAME_BYTES: usize = UDP_MTU as usize;
const TX_PACKETS: usize = 7;
const RX_PACKETS: usize = 2;
static OUT: WifiQueue = WifiQueue::new();
static HISTORY: StaticCell<[Packet; HISTORY_PACKETS]> = StaticCell::new();

struct Packet {
    bytes: [u8; FRAME_BYTES],
    len: usize,
}

impl Packet {
    const EMPTY: Self = Self {
        bytes: [0; FRAME_BYTES],
        len: 0,
    };

    fn store(&mut self, header: Header, frame: &[u8]) -> &[u8] {
        self.len = HEADER_BYTES + frame.len();
        self.bytes[..HEADER_BYTES].copy_from_slice(&header.encode());
        self.bytes[HEADER_BYTES..self.len].copy_from_slice(frame);
        &self.bytes[..self.len]
    }
}

#[derive(Clone, Copy)]
struct Peer {
    addr: IpEndpoint,
    token: Option<u32>,
}

// The network stack copies these socket buffers into the driver's own
// buffers. Put them in RTC RAM to leave stack room for BLE pairing.
#[esp_hal::ram(unstable(rtc_fast))]
static TX_BUFFER: StaticCell<[u8; FRAME_BYTES * TX_PACKETS]> = StaticCell::new();
#[esp_hal::ram(unstable(rtc_fast))]
static RX_BUFFER: StaticCell<[u8; FRAME_BYTES * RX_PACKETS]> = StaticCell::new();

pub fn start(spawner: Spawner, router: &'static Stack, stack: NetStack<'static>) {
    let Some(host) =
        option_env!("OXIFOC_BRIDGE_ERGOT_PEER").and_then(|ip| ip.parse::<Ipv4Addr>().ok())
    else {
        return;
    };
    spawner
        .spawn(run(router, stack, IpAddress::Ipv4(host)).expect("Ergot UDP task is spawned once"));
}

#[embassy_executor::task]
async fn run(router: &'static Stack, stack: NetStack<'static>, host: IpAddress) {
    let history = HISTORY.init_with(|| [const { Packet::EMPTY }; HISTORY_PACKETS]);
    let mut tx_meta = [PacketMetadata::EMPTY; TX_PACKETS];
    let mut rx_meta = [PacketMetadata::EMPTY; RX_PACKETS];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        RX_BUFFER.init_with(|| [0; FRAME_BYTES * RX_PACKETS]),
        &mut tx_meta,
        TX_BUFFER.init_with(|| [0; FRAME_BYTES * TX_PACKETS]),
    );
    if socket.bind(UDP_PORT).is_err() {
        log::warn!("Ergot UDP bind failed");
        return;
    }
    loop {
        stack.wait_config_up().await;
        let sink = BridgeSink::Wifi(framed_stream::Sink::new(OUT.framed_producer(), FRAME_MTU));
        let Ok(ident) = router.manage_profile(|p| p.register_interface(sink)) else {
            log::warn!("Ergot UDP has no router interface slot");
            Timer::after_secs(1).await;
            continue;
        };
        let _ = select(
            stack.wait_config_down(),
            session(router, &mut socket, host, ident, history),
        )
        .await;
        let _ = router.manage_profile(|p| p.deregister_interface(ident));
        let consumer = OUT.framed_consumer();
        while let Ok(grant) = consumer.read() {
            grant.release();
        }
    }
}

async fn session(
    router: &'static Stack,
    socket: &mut UdpSocket<'_>,
    host: IpAddress,
    ident: u8,
    history: &mut [Packet; HISTORY_PACKETS],
) {
    let Some(net) = router.manage_profile(|p| p.net_id_of(ident)) else {
        return;
    };
    let consumer = OUT.framed_consumer();
    let timeout = Duration::from_millis(LIVENESS_TIMEOUT_MS);
    let _ = router.manage_profile(|p| p.set_interface_state(ident, InterfaceState::Inactive));
    let mut processor = RouterFrameProcessor::new(net);
    let mut peer: Option<Peer> = None;
    let mut next = 0_u32;
    let mut deadline = Instant::now() + timeout;
    let mut batch = 0;
    loop {
        match select(
            Timer::at(deadline),
            select(socket.wait_recv_ready(), consumer.wait_read()),
        )
        .await
        {
            Either::First(()) => {
                peer = None;
                next = 0;
                <RouterFrameProcessor as FrameProcessor<&'static Stack>>::reset(&mut processor);
                let _ = router
                    .manage_profile(|p| p.set_interface_state(ident, InterfaceState::Inactive));
                while let Ok(grant) = consumer.read() {
                    grant.release();
                }
                deadline = Instant::now() + timeout;
            }
            Either::Second(Either::First(())) => {
                let reply = socket
                    .recv_from_with(|data, metadata| {
                        let sender = metadata.endpoint;
                        if sender.addr != host || peer.is_some_and(|current| current.addr != sender)
                        {
                            return None;
                        }
                        if let Some((header, _)) = Header::decode(data) {
                            let token = Some(header.token);
                            match header.kind {
                                Kind::Hello
                                    if peer.is_none_or(|current| current.token == token) =>
                                {
                                    peer = Some(Peer {
                                        addr: sender,
                                        token,
                                    });
                                    deadline = Instant::now() + timeout;
                                    return Some(Header {
                                        kind: Kind::Ready,
                                        sequence: 0,
                                        ..header
                                    });
                                }
                                Kind::Replay
                                    if peer.is_some_and(|current| current.token == token) =>
                                {
                                    return Some(header);
                                }
                                _ => return None,
                            }
                        }
                        let frame = de_frame(data)?;
                        // A Wi-Fi edge may only originate frames on its own segment.
                        if frame.hdr.src.network_id != 0 && frame.hdr.src.network_id != net {
                            return None;
                        }
                        peer.get_or_insert(Peer {
                            addr: sender,
                            token: None,
                        });
                        deadline = Instant::now() + timeout;
                        processor.process_frame(data, &router, ident);
                        None
                    })
                    .await;
                if let (Some(reply), Some(peer)) = (reply, peer) {
                    if reply.kind == Kind::Ready {
                        let _ = socket.send_to(&reply.encode(), peer.addr).await;
                    } else if (1..=HISTORY_PACKETS as u32)
                        .contains(&next.wrapping_sub(reply.sequence))
                    {
                        let packet = &history[reply.sequence as usize % HISTORY_PACKETS];
                        let bytes = &packet.bytes[..packet.len];
                        if Header::decode(bytes).is_some_and(|(header, _)| {
                            header.token == reply.token && header.sequence == reply.sequence
                        }) {
                            let _ = socket.send_to(bytes, peer.addr).await;
                        }
                    }
                }
            }
            Either::Second(Either::Second(grant)) => {
                if let Some(peer) = peer {
                    if let Some(token) = peer.token {
                        if grant.len() <= usize::from(FRAME_MTU) {
                            let header = Header {
                                kind: Kind::Frame,
                                token,
                                sequence: next,
                            };
                            let packet =
                                history[next as usize % HISTORY_PACKETS].store(header, &grant);
                            next = next.wrapping_add(1);
                            let _ = socket.send_to(packet, peer.addr).await;
                        }
                    } else {
                        let _ = socket.send_to(&grant, peer.addr).await;
                    }
                }
                grant.release();
            }
        }
        // Let the network runner send a batch together, then yield even
        // when both the socket and the queue are continuously ready.
        batch += 1;
        if batch == TX_PACKETS {
            batch = 0;
            yield_now().await;
        }
    }
}

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
use oxifoc_core::icd::{LIVENESS_TIMEOUT_MS, UDP_MTU};
use static_cell::StaticCell;

use super::{BridgeSink, Stack, WifiQueue};

const UDP_PORT: u16 = 2025;
const FRAME_BYTES: usize = UDP_MTU as usize;

// These buffers are only accessed by the CPU. Keep them out of main DRAM to
// leave stack room for BLE pairing's P-256 operations.
#[esp_hal::ram(unstable(rtc_fast))]
static OUT: WifiQueue = WifiQueue::new();
#[esp_hal::ram(unstable(rtc_fast))]
static TX_BUFFER: StaticCell<[u8; FRAME_BYTES]> = StaticCell::new();
#[esp_hal::ram(unstable(rtc_fast))]
static RX_BUFFER: StaticCell<[u8; FRAME_BYTES]> = StaticCell::new();

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
    let mut tx_meta = [PacketMetadata::EMPTY; 2];
    let mut rx_meta = [PacketMetadata::EMPTY; 2];
    let mut socket = UdpSocket::new(
        stack,
        &mut rx_meta,
        RX_BUFFER.init([0; FRAME_BYTES]),
        &mut tx_meta,
        TX_BUFFER.init([0; FRAME_BYTES]),
    );
    if socket.bind(UDP_PORT).is_err() {
        defmt::warn!("Ergot UDP bind failed");
        return;
    }
    loop {
        stack.wait_config_up().await;
        let sink = BridgeSink::Wifi(framed_stream::Sink::new(OUT.framed_producer(), UDP_MTU));
        let Ok(ident) = router.manage_profile(|p| p.register_interface(sink)) else {
            defmt::warn!("Ergot UDP has no router interface slot");
            Timer::after_secs(1).await;
            continue;
        };
        let _ = select(
            stack.wait_config_down(),
            session(router, &mut socket, host, ident),
        )
        .await;
        let _ = router.manage_profile(|p| p.deregister_interface(ident));
        let consumer = OUT.framed_consumer();
        while let Ok(grant) = consumer.read() {
            grant.release();
        }
    }
}

async fn session(router: &'static Stack, socket: &mut UdpSocket<'_>, host: IpAddress, ident: u8) {
    let Some(net) = router.manage_profile(|p| p.net_id_of(ident)) else {
        return;
    };
    let consumer = OUT.framed_consumer();
    let timeout = Duration::from_millis(LIVENESS_TIMEOUT_MS);
    let _ = router.manage_profile(|p| p.set_interface_state(ident, InterfaceState::Inactive));
    let mut processor = RouterFrameProcessor::new(net);
    let mut peer: Option<IpEndpoint> = None;
    let mut deadline = Instant::now() + timeout;
    loop {
        match select(
            Timer::at(deadline),
            select(socket.wait_recv_ready(), consumer.wait_read()),
        )
        .await
        {
            Either::First(()) => {
                peer = None;
                <RouterFrameProcessor as FrameProcessor<&'static Stack>>::reset(&mut processor);
                let _ = router
                    .manage_profile(|p| p.set_interface_state(ident, InterfaceState::Inactive));
                while let Ok(grant) = consumer.read() {
                    grant.release();
                }
                deadline = Instant::now() + timeout;
            }
            Either::Second(Either::First(())) => {
                socket
                    .recv_from_with(|data, metadata| {
                        let sender = metadata.endpoint;
                        if sender.addr == host && peer.is_none_or(|current| current == sender) {
                            let Some(frame) = de_frame(data) else { return };
                            // A Wi-Fi edge may only originate frames on its own
                            // segment, including link-local bootstrap frames.
                            if frame.hdr.src.network_id != 0 && frame.hdr.src.network_id != net {
                                return;
                            }
                            peer = Some(sender);
                            deadline = Instant::now() + timeout;
                            processor.process_frame(data, &router, ident);
                        }
                    })
                    .await;
            }
            Either::Second(Either::Second(grant)) => {
                if let Some(peer) = peer {
                    let _ = socket.send_to(&grant, peer).await;
                }
                grant.release();
            }
        }
        yield_now().await;
    }
}

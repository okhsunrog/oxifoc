//! Time-limited CMSIS-DAP/TCP access to the controller's SWD pins.

use core::{pin::pin, sync::atomic::Ordering};

use embassy_executor::Spawner;
use embassy_futures::{
    select::{Either, select, select3},
    yield_now,
};
use embassy_net::{Stack as NetStack, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use esp_hal::peripherals::{GPIO4, GPIO5};
use oxifoc_core::{
    icd::SwdProbeEndpoint,
    types::swd_probe::{MAX_ARM_MINUTES, ProbeRequest, ProbeResponse},
};

use crate::transport::{Stack, spi::SPI_LINK_SLOTS};

mod framing;
mod packet;
mod pins;

use framing::{HEADER_LEN, PACKET_SIZE, PORT};
use packet::Action;
use pins::Pins;

const PAYLOAD_LEN: usize = PACKET_SIZE as usize;
const SPI_ACTIVE_WINDOW: Duration = Duration::from_secs(1);
const ACTIVE_SPI_PACKET_INTERVAL: Duration = Duration::from_millis(20);

// None disarms. Sending a new deadline replaces any existing listener or
// connection. The request handler is the only writer.
static ARM: Signal<CriticalSectionRawMutex, Option<Instant>> = Signal::new();

pub fn start(
    spawner: Spawner,
    router: &'static Stack,
    clk: GPIO4<'static>,
    dio: GPIO5<'static>,
    stack: Option<NetStack<'static>>,
) {
    let pins = Pins::new(clk, dio);
    spawner.spawn(control(router, stack).expect("probe control task is spawned once"));
    if let Some(stack) = stack {
        spawner.spawn(serve(stack, pins).expect("probe task is spawned once"));
    }
}

#[embassy_executor::task]
async fn control(router: &'static Stack, stack: Option<NetStack<'static>>) {
    let server = router
        .endpoints()
        .bounded_server::<SwdProbeEndpoint, 2>(None);
    let server = pin!(server);
    let mut handle = server.attach();

    loop {
        let _ = handle
            .serve(|request: &ProbeRequest| {
                let reply = apply(*request, stack, Instant::now());
                async move { reply }
            })
            .await;
    }
}

fn apply(request: ProbeRequest, stack: Option<NetStack<'static>>, now: Instant) -> ProbeResponse {
    match request {
        ProbeRequest::Disarm => {
            ARM.signal(None);
            log::info!("Wireless SWD disarmed");
            ProbeResponse::Disarmed
        }
        ProbeRequest::Arm { minutes } => {
            if !(1..=MAX_ARM_MINUTES).contains(&minutes) {
                return ProbeResponse::InvalidDuration;
            }
            let Some(config) = stack.and_then(|net| net.config_v4()) else {
                return ProbeResponse::WifiUnavailable;
            };
            ARM.signal(Some(now + Duration::from_secs(u64::from(minutes) * 60)));
            log::info!("Wireless SWD armed for {minutes} min");
            ProbeResponse::Armed {
                ipv4: config.address.address().octets(),
                minutes,
            }
        }
    }
}

#[embassy_executor::task]
async fn serve(stack: NetStack<'static>, mut pins: Pins) {
    let mut arm = ARM.wait().await;
    loop {
        if let Some(deadline) = arm {
            // Cancellation drops the socket and the pin adapters, including
            // when the peer is idle or has sent only part of a packet.
            if let Either::First(next) = select(
                ARM.wait(),
                select3(
                    Timer::at(deadline),
                    stack.wait_config_down(),
                    listen(stack, &mut pins),
                ),
            )
            .await
            {
                arm = next;
                continue;
            }
        }
        arm = ARM.wait().await;
    }
}

async fn listen(stack: NetStack<'static>, pins: &mut Pins) -> ! {
    let mut rx = [0; HEADER_LEN + PAYLOAD_LEN];
    let mut tx = [0; HEADER_LEN + PAYLOAD_LEN];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx, &mut tx);
        socket.set_nagle_enabled(false);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if socket.accept(PORT).await.is_ok() {
            session(&mut socket, pins).await;
            // The DAP pin adapters have already been dropped. Give TCP a
            // chance to send any queued response and its FIN.
            socket.close();
            let _ = embassy_time::with_timeout(Duration::from_millis(150), socket.flush()).await;
        }
        yield_now().await;
    }
}

async fn session(socket: &mut TcpSocket<'_>, pins: &mut Pins) {
    let clock = pins::default_delay();
    let mut dap = pins.session(&clock);
    let mut header = [0; HEADER_LEN];
    let mut request = [0; PAYLOAD_LEN];
    let mut frame = [0; HEADER_LEN + PAYLOAD_LEN];
    let mut spi_slots = SPI_LINK_SLOTS.load(Ordering::Relaxed);
    let mut last_spi = Instant::now();
    let mut last_reply = last_spi;
    loop {
        if Read::read_exact(socket, &mut header).await.is_err() {
            break;
        }
        let Some(len) = framing::request_len(header) else {
            break;
        };
        let request = &mut request[..len];
        if Read::read_exact(socket, request).await.is_err() {
            break;
        }
        let Some(action) = packet::prepare(request) else {
            break;
        };
        let now = Instant::now();
        let slots = SPI_LINK_SLOTS.load(Ordering::Relaxed);
        if slots != spi_slots {
            spi_slots = slots;
            last_spi = now;
        }
        // Even DAP_Info-only traffic delayed Stop ACKs on the live SPI route.
        // Pace the TCP client during forwarding and allow full rate once
        // SPI is idle, as it is when the target is halted for programming.
        if now - last_spi < SPI_ACTIVE_WINDOW {
            Timer::at(last_reply + ACTIVE_SPI_PACKET_INTERVAL).await;
        }
        let response = &mut frame[HEADER_LEN..];
        let response_len = match action {
            Action::Process(command) => {
                let len = dap.process_command(request, response);
                packet::mask_capabilities(command, request, response);
                len
            }
            Action::Delay(us) => {
                Timer::after_micros(u64::from(us)).await;
                packet::ok(response, request[0])
            }
            Action::Clock(hz) => {
                clock.set(pins::half_cycles(hz));
                packet::ok(response, request[0])
            }
            Action::Reply(status) => packet::short_reply(response, request[0], status),
            Action::Unsupported => {
                response[0] = 0xff;
                1
            }
        };
        frame[..HEADER_LEN].copy_from_slice(&framing::response_header(response_len as u16));
        if Write::write_all(socket, &frame[..HEADER_LEN + response_len])
            .await
            .is_err()
        {
            break;
        }
        last_reply = Instant::now();
        // TCP reads can complete immediately when the host queues requests.
        yield_now().await;
    }
}

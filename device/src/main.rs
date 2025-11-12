#![no_std]
#![no_main]

use core::pin::pin;

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_time::{Duration, Timer, with_timeout};
use ergot::{
    Address,
    exports::bbq2::traits::coordination::cas::AtomicCoord,
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{ButtonEvent, ButtonEndpoint, KeepAlive, KeepAliveEndpoint, InfoEndpoint, DeviceInfo};
use rtt_target::{rtt_init, ChannelMode::*};
use static_cell::StaticCell;

mod rtt_io;
use rtt_io::RttWriter;

// Use panic-probe for panics
use panic_probe as _;


const OUT_QUEUE_SIZE: usize = 2048;
const MAX_PACKET_SIZE: usize = 512;

// Type aliases for our application
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, rtt_io::RttReader>;

/// Statically store our outgoing packet buffer
static OUTQ: Queue = kit::Queue::new();

/// Statically store our netstack
static STACK: Stack = kit::new_target_stack(OUTQ.stream_producer(), MAX_PACKET_SIZE as u16);

/// Buffers for RX worker
static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
static SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/// RTT channel storage
static RTT_UP_CHANNEL: StaticCell<rtt_target::UpChannel> = StaticCell::new();
static RTT_DOWN_CHANNEL: StaticCell<rtt_target::DownChannel> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize RTT with defmt on channel 0 and ergot on channel 1
    // rtt-target automatically provides defmt support when defmt feature is enabled
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" } // defmt logs
            1: { size: 2048, mode: NoBlockSkip, name: "ergot" } // Ergot data channel
        }
        down: {
            0: { size: 1024, name: "ergot-down" } // Reserved for future host->device
        }
    };

    // Configure rtt-target as the defmt global logger on up channel 0
    rtt_target::set_defmt_channel(channels.up.0);

    // Get RTT channels for ergot (up: device->host, down: host->device)
    let rtt_up = channels.up.1;
    let rtt_up_static = RTT_UP_CHANNEL.init_with(|| rtt_up);
    let rtt_down = channels.down.0;
    let rtt_down_static = RTT_DOWN_CHANNEL.init_with(|| rtt_down);

    // Create RTT I/O
    let rtt_io = rtt_io::RttIo::new(rtt_up_static, rtt_down_static);
    let (rtt_rx, rtt_tx) = rtt_io.split();

    // Initialize STM32
    let p = embassy_stm32::init(Default::default());

    defmt::info!("Oxifoc starting - ergot over RTT");

    // Create RX worker for incoming ergot messages (it will set interface to Inactive, then Active after first frame)
    let rx_worker = RxWorker::new_target(&STACK, rtt_rx, ());

    // Button: PC10, external pull-up, active-low to GND
    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::None);
    defmt::info!("Button configured on PC10 (active-low)");

    // Spawn I/O workers
    spawner.spawn(run_rx(
        rx_worker,
        RECV_BUF.init_with(|| [0u8; MAX_PACKET_SIZE]),
        SCRATCH_BUF.init_with(|| [0u8; 64])
    )).unwrap();
    spawner.spawn(run_tx(rtt_tx)).unwrap();

    // Spawn application tasks (temporarily disable others to debug keepalive)
    spawner.spawn(button_handler(button)).unwrap();
    spawner.spawn(status_reporter()).unwrap();
    spawner.spawn(keepalive_task()).unwrap();
    spawner.spawn(info_server()).unwrap();

    defmt::info!("All tasks spawned, entering main loop");

    // Main heartbeat loop
    loop {
        Timer::after(Duration::from_secs(5)).await;
        defmt::info!("Heartbeat 5s - button ready, ergot active");
    }
}

/// Worker task for incoming ergot data via RTT
#[embassy_executor::task]
async fn run_rx(mut rcvr: RxWorker, recv_buf: &'static mut [u8], scratch_buf: &'static mut [u8]) {
    loop {
        let _ = rcvr.run(recv_buf, scratch_buf).await;
    }
}

/// Worker task for outgoing ergot data via RTT
#[embassy_executor::task]
async fn run_tx(mut tx: RttWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

#[embassy_executor::task]
async fn button_handler(mut button: ExtiInput<'static>) {
    const DOUBLE_CLICK_DELAY: u64 = 250;
    const HOLD_DELAY: u64 = 1000;

    defmt::info!("Button handler started");

    // Target host router at network 1, node 1 (like rp2040-serial-pair target.rs:89-95)
    let host_addr = Address {
        network_id: 1,
        node_id: 1,
        port_id: 0,
    };
    let client = STACK
        .endpoints()
        .client::<ButtonEndpoint>(host_addr, Some("button"));

    defmt::info!("Button ready (active-low)");

    loop {
        // Wait for press (active-low => falling edge)
        button.wait_for_falling_edge().await;

        // If release does not happen within HOLD_DELAY, it's a hold
        if with_timeout(
            Duration::from_millis(HOLD_DELAY),
            button.wait_for_rising_edge(),
        )
        .await
        .is_err()
        {
            defmt::info!("Button: HOLD");
            let _ = client.request(&ButtonEvent::Hold).await;
            // Ensure we're released before next iteration
            button.wait_for_rising_edge().await;
            continue;
        }

        // Released within hold window: check for a second press within DOUBLE_CLICK_DELAY
        if with_timeout(
            Duration::from_millis(DOUBLE_CLICK_DELAY),
            button.wait_for_falling_edge(),
        )
        .await
        .is_ok()
        {
            defmt::info!("Button: DOUBLE CLICK");
            let _ = client.request(&ButtonEvent::DoubleClick).await;
            // Wait for final release
            button.wait_for_rising_edge().await;
        } else {
            defmt::info!("Button: SINGLE CLICK");
            let _ = client.request(&ButtonEvent::SingleClick).await;
        }
    }
}

#[embassy_executor::task]
async fn status_reporter() {
    defmt::info!("Status reporter started");

    // Create server to handle incoming button requests from the network
    let button_socket = STACK
        .endpoints()
        .bounded_server::<ButtonEndpoint, 4>(Some("button"));
    let button_socket = pin!(button_socket);
    let mut button_hdl = button_socket.attach();

    defmt::info!("Ergot button endpoint ready");

    loop {
        // Handle button events from network with timeout
        let result = with_timeout(Duration::from_secs(5), button_hdl.serve(async |event| {
            match event {
                ButtonEvent::SingleClick => {
                    defmt::info!("Network: SINGLE CLICK");
                },
                ButtonEvent::DoubleClick => {
                    defmt::info!("Network: DOUBLE CLICK");
                },
                ButtonEvent::Hold => {
                    defmt::info!("Network: HOLD");
                },
            }
        })).await;

        // Periodic status when no network activity
        if result.is_err() {
            defmt::debug!("Waiting for network events...");
        }
    }
}

/// Periodic keepalive to host
#[embassy_executor::task]
async fn keepalive_task() {
    // Wait for interface to become active (host sends first frame)
    defmt::info!("keepalive task waiting for active interface");
    Timer::after(Duration::from_secs(2)).await;

    let mut seq: u32 = 0;
    // Target host router at network 1, node 1 (like rp2040-serial-pair target.rs:89-95)
    let host_addr = Address {
        network_id: 1,
        node_id: 1,
        port_id: 0,
    };
    let client = STACK
        .endpoints()
        .client::<KeepAliveEndpoint>(host_addr, Some("keepalive"));
    defmt::info!("keepalive task started");
    loop {
        Timer::after(Duration::from_secs(3)).await;
        defmt::info!("sending keepalive seq={}", seq);
        let msg = KeepAlive { seq };
        // Add timeout to prevent blocking forever if no host is connected
        match with_timeout(Duration::from_millis(500), client.request(&msg)).await {
            Ok(Ok(_)) => defmt::debug!("keepalive {} sent", seq),
            Ok(Err(_)) => defmt::warn!("keepalive {} failed", seq),
            Err(_) => defmt::warn!("keepalive {} timeout", seq),
        }
        seq = seq.wrapping_add(1);
    }
}

/// Respond to info requests from host
#[embassy_executor::task]
async fn info_server() {
    let server = STACK
        .endpoints()
        .bounded_server::<InfoEndpoint, 2>(Some("device_info"));
    let server = pin!(server);
    let mut h = server.attach();
    loop {
        let _ = h.serve(|_req: &()| async move {
            let mut hw: heapless::String<32> = heapless::String::new();
            let mut sw: heapless::String<32> = heapless::String::new();
            let _ = hw.push_str("B-G431B-ESC1");
            let _ = sw.push_str("oxifoc-0.1.0");
            DeviceInfo { hw, sw }
        }).await;
    }
}

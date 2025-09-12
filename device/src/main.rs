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
    logging::log_v0_4::LogSink,
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{ButtonEvent, ButtonEndpoint};
use rtt_target::rtt_init;
use static_cell::StaticCell;

mod rtt_io;
use rtt_io::RttWriter;

#[cfg(feature = "defmt")]
use panic_probe as _;

// Simple panic handler for non-defmt builds
#[cfg(not(feature = "defmt"))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf()
}

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

/// Statically store the LogSink
static LOGSINK: LogSink<&'static Stack> = LogSink::new(&STACK);

/// Buffers for RX worker
static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
static SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/// RTT channel storage
static RTT_UP_CHANNEL: StaticCell<rtt_target::UpChannel> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize RTT with two up channels
    let channels = rtt_init! {
        up: {
            0: { size: 1024 }  // Defmt/debug channel
            1: { size: 2048 }  // Ergot data channel
        }
    };

    // Get RTT channel for ergot (channel 1)
    let rtt_up = channels.up.1;
    let rtt_up_static = RTT_UP_CHANNEL.init_with(|| rtt_up);

    // Create RTT I/O
    let rtt_io = rtt_io::RttIo::new(rtt_up_static);
    let (rtt_rx, rtt_tx) = rtt_io.split();

    // Register ergot as the global logger
    LOGSINK.register_static(log::LevelFilter::Info);

    // Initialize STM32
    let p = embassy_stm32::init(Default::default());

    log::info!("Oxifoc starting - ergot over RTT");

    // Create RX worker for incoming ergot messages
    let rx_worker = RxWorker::new_target(&STACK, rtt_rx, ());

    // Button configuration
    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::Down);
    log::info!("Button configured on PC10");

    // Spawn I/O workers
    spawner.spawn(run_rx(
        rx_worker,
        RECV_BUF.init_with(|| [0u8; MAX_PACKET_SIZE]),
        SCRATCH_BUF.init_with(|| [0u8; 64])
    )).unwrap();
    spawner.spawn(run_tx(rtt_tx)).unwrap();

    // Spawn application tasks
    spawner.spawn(button_handler(button)).unwrap();
    spawner.spawn(status_reporter()).unwrap();

    log::info!("All tasks spawned, entering main loop");

    // Main heartbeat loop
    loop {
        Timer::after(Duration::from_secs(10)).await;
        log::info!("Heartbeat - button ready, ergot active");
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

    log::info!("Button handler started");

    let client = STACK
        .endpoints()
        .client::<ButtonEndpoint>(Address::unknown(), Some("button"));

    // Wait for first button press
    button.wait_for_rising_edge().await;
    log::info!("Button ready");

    loop {
        // Check for hold (button pressed for more than HOLD_DELAY)
        if with_timeout(
            Duration::from_millis(HOLD_DELAY),
            button.wait_for_falling_edge(),
        )
        .await
        .is_err()
        {
            log::info!("Button: HOLD");
            let _ = client.request(&ButtonEvent::Hold).await;
            button.wait_for_falling_edge().await;
        }
        // Check for double click
        else if with_timeout(
            Duration::from_millis(DOUBLE_CLICK_DELAY),
            button.wait_for_rising_edge(),
        )
        .await
        .is_err()
        {
            log::info!("Button: SINGLE CLICK");
            let _ = client.request(&ButtonEvent::SingleClick).await;
        } else {
            log::info!("Button: DOUBLE CLICK");
            let _ = client.request(&ButtonEvent::DoubleClick).await;
            button.wait_for_falling_edge().await;
        }

        // Wait for next button press
        button.wait_for_rising_edge().await;
    }
}

#[embassy_executor::task]
async fn status_reporter() {
    log::info!("Status reporter started");

    // Create server to handle incoming button requests from the network
    let button_socket = STACK
        .endpoints()
        .bounded_server::<ButtonEndpoint, 4>(Some("button"));
    let button_socket = pin!(button_socket);
    let mut button_hdl = button_socket.attach();

    log::info!("Ergot button endpoint ready");

    loop {
        // Handle button events from network with timeout
        let result = with_timeout(Duration::from_secs(5), button_hdl.serve(async |event| {
            match event {
                ButtonEvent::SingleClick => {
                    log::info!("Network: SINGLE CLICK");
                },
                ButtonEvent::DoubleClick => {
                    log::info!("Network: DOUBLE CLICK");
                },
                ButtonEvent::Hold => {
                    log::info!("Network: HOLD");
                },
            }
        })).await;

        // Periodic status when no network activity
        if result.is_err() {
            log::debug!("Waiting for network events...");
        }
    }
}

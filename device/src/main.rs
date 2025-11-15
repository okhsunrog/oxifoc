#![no_std]
#![no_main]

use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_time::{Duration, Timer, with_timeout};
use ergot::{
    Address,
    exports::bbq2::traits::coordination::cas::AtomicCoord,
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{
    ButtonEndpoint, ButtonEvent, DeviceInfo, InfoEndpoint,
    MotorCommand, MotorEndpoint,
};
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

mod rtt_io;
use rtt_io::RttWriter;

mod motor;
use motor::MotorController;

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

/// Link status: set true after we observe an inbound host request
static LINK_ACTIVE: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum DeviceState {
    Boot = 0,
    WaitingLink = 1,
    Linked = 2,
    Error = 3,
}

use core::sync::atomic::AtomicU8;
static DEVICE_STATE: AtomicU8 = AtomicU8::new(DeviceState::Boot as u8);

fn set_device_state(s: DeviceState) {
    DEVICE_STATE.store(s as u8, Ordering::Relaxed);
}

fn get_device_state() -> DeviceState {
    match DEVICE_STATE.load(Ordering::Relaxed) {
        0 => DeviceState::Boot,
        1 => DeviceState::WaitingLink,
        2 => DeviceState::Linked,
        _ => DeviceState::Error,
    }
}

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
            0: { size: 1024, name: "ergot-down" } // host->device
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

    // Initialize STM32 with HSE=8MHz feeding PLL to 170MHz SYSCLK
    let p = {
        let mut config = embassy_stm32::Config::default();
        {
            use embassy_stm32::rcc::*;
            use embassy_stm32::time::Hertz;
            // Use external 8MHz HSE oscillator as PLL source
            config.rcc.hse = Some(Hse {
                freq: Hertz(8_000_000),
                mode: HseMode::Oscillator,
            });
            // VCO in: 8MHz / 2 = 4MHz; VCO: 4MHz * 85 = 340MHz; SYSCLK: 340MHz / 2 = 170MHz
            config.rcc.pll = Some(Pll {
                source: PllSource::HSE,
                prediv: PllPreDiv::DIV2,
                mul: PllMul::MUL85,
                divp: None,
                divq: None,
                divr: Some(PllRDiv::DIV2),
            });
            config.rcc.sys = Sysclk::PLL1_R;
            // Above 150MHz, enable Range1 boost mode per RM0440 guidance
            config.rcc.boost = true;
        }
        embassy_stm32::init(config)
    };

    defmt::info!("Oxifoc starting - ergot over RTT");

    // Create RX worker for incoming ergot messages (it will set interface to Inactive, then Active after first frame)
    let rx_worker = RxWorker::new_target(&STACK, rtt_rx, ());

    // Button: PC10, external pull-up, active-low to GND
    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::None);
    defmt::info!("Button configured on PC10 (active-low)");

    // LED on PC6
    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);

    // Initialize motor controller with TIM1 and motor pins
    let motor_ctrl = MotorController::init(
        p.TIM1,
        p.PA8,   // Phase A high
        p.PC13,  // Phase A low
        p.PA9,   // Phase B high
        p.PA12,  // Phase B low
        p.PA10,  // Phase C high
        p.PB15,  // Phase C low
    );

    // Spawn I/O workers
    spawner
        .spawn(run_rx(
            rx_worker,
            RECV_BUF.init_with(|| [0u8; MAX_PACKET_SIZE]),
            SCRATCH_BUF.init_with(|| [0u8; 64]),
        ))
        .unwrap();
    spawner.spawn(run_tx(rtt_tx)).unwrap();

    // Initialize motor command channel
    let motor_cmd_channel = MOTOR_CMD_CHANNEL.init(embassy_sync::channel::Channel::new());
    let motor_cmd_receiver = motor_cmd_channel.receiver();
    let motor_cmd_sender = motor_cmd_channel.sender();

    spawner.spawn(button_handler(button)).unwrap();
    spawner.spawn(status_reporter()).unwrap();
    spawner.spawn(info_server()).unwrap();
    spawner.spawn(motor_control_task(motor_ctrl, motor_cmd_receiver)).unwrap();
    spawner.spawn(motor_command_server(motor_cmd_sender)).unwrap();

    // Transition to "waiting for link" once tasks are up
    set_device_state(DeviceState::WaitingLink);

    defmt::info!("All tasks spawned, entering LED status loop");

    // LED status loop - shows device state via blink patterns
    loop {
        match get_device_state() {
            DeviceState::Boot => {
                // Quick double blink
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(600)).await;
            }
            DeviceState::WaitingLink => {
                // Slow blink (1 Hz, 10% duty)
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
                led.set_low();
                Timer::after(Duration::from_millis(900)).await;
            }
            DeviceState::Linked => {
                // Solid ON with periodic short delay to allow state changes
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
            }
            DeviceState::Error => {
                // Triple blink pattern
                for _ in 0..3 {
                    led.set_high();
                    Timer::after(Duration::from_millis(120)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(120)).await;
                }
                Timer::after(Duration::from_millis(800)).await;
            }
        }
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
        let result = with_timeout(
            Duration::from_secs(5),
            button_hdl.serve(async |event| match event {
                ButtonEvent::SingleClick => {
                    defmt::info!("Network: SINGLE CLICK");
                }
                ButtonEvent::DoubleClick => {
                    defmt::info!("Network: DOUBLE CLICK");
                }
                ButtonEvent::Hold => {
                    defmt::info!("Network: HOLD");
                }
            }),
        )
        .await;

        // Periodic status when no network activity
        if result.is_err() {
            defmt::debug!("Waiting for network events...");
        }
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
        let _ = h
            .serve(|_req: &()| async move {
                // Mark link as active on first inbound request
                LINK_ACTIVE.store(true, Ordering::Relaxed);
                set_device_state(DeviceState::Linked);
                let mut hw: heapless::String<32> = heapless::String::new();
                let mut sw: heapless::String<32> = heapless::String::new();
                let _ = hw.push_str("B-G431B-ESC1");
                let _ = sw.push_str("oxifoc-0.1.0");
                DeviceInfo { hw, sw }
            })
            .await;
    }
}

/// Static channel for motor commands
static MOTOR_CMD_CHANNEL: StaticCell<
    embassy_sync::channel::Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, MotorCommand, 4>,
> = StaticCell::new();

/// Motor control task - performs 6-step commutation and handles commands
#[embassy_executor::task]
async fn motor_control_task(
    mut motor: MotorController<'static>,
    cmd_receiver: embassy_sync::channel::Receiver<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        MotorCommand,
        4,
    >,
) {
    defmt::info!("Motor control task started");

    loop {
        // Check for commands (non-blocking)
        if let Ok(cmd) = cmd_receiver.try_receive() {
            motor.handle_command(&cmd);
        }

        // Perform commutation step
        motor.commutate();

        // Wait for next commutation based on speed
        let period = motor.get_commutation_period();
        Timer::after(period).await;
    }
}

/// Motor command server - handles motor control commands via ergot
#[embassy_executor::task]
async fn motor_command_server(
    motor_cmd_sender: embassy_sync::channel::Sender<
        'static,
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        MotorCommand,
        4,
    >,
) {
    defmt::info!("Motor command server started");

    let server = STACK
        .endpoints()
        .bounded_server::<MotorEndpoint, 2>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|cmd: &MotorCommand| {
                let cmd_clone = cmd.clone();
                let sender_clone = motor_cmd_sender.clone();
                async move {
                    // Send command to motor task
                    let _ = sender_clone.try_send(cmd_clone);
                    // Return current motor status
                    motor::get_motor_status()
                }
            })
            .await;
    }
}


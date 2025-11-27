#![no_std]
#![no_main]

// Compile-time check: only one transport can be enabled
#[cfg(all(feature = "transport-uart", feature = "transport-rtt"))]
compile_error!(
    "Cannot enable both transport-uart and transport-rtt features simultaneously. Choose one transport."
);

#[cfg(not(any(feature = "transport-uart", feature = "transport-rtt")))]
compile_error!("Must enable either transport-uart or transport-rtt feature.");

use core::cell::RefCell;
use core::pin::pin;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, ConversionTrigger, Exten, InjectedAdc, SampleTime,
};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::{Peri, peripherals};
use embassy_stm32::{
    interrupt,
    interrupt::typelevel::{ADC1_2, Interrupt},
};
use embassy_time::{Duration, Timer, with_timeout};
use ergot::{
    Address,
    exports::bbq2::traits::coordination::cas::AtomicCoord,
    rtt_target::{ChannelMode::*, rtt_init},
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{
    ButtonEndpoint, ButtonEvent, DeviceInfo, InfoEndpoint, MotorCommand, MotorEndpoint,
};
use static_cell::StaticCell;

use assign_resources::assign_resources;
use embassy_sync::blocking_mutex::CriticalSectionMutex;

// Transport-specific imports
#[cfg(feature = "transport-uart")]
use embassy_stm32::bind_interrupts;
#[cfg(feature = "transport-uart")]
use embassy_stm32::usart::{
    BufferedUart, Config as UartConfig, Parity as UartParity, StopBits as UartStopBits,
};
#[cfg(feature = "transport-uart")]
mod usart_io;
#[cfg(feature = "transport-uart")]
use ergot::logging::defmt_sink;
#[cfg(feature = "transport-uart")]
use usart_io::{UartReader, UartWriter};

#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::{RttReader, RttWriter};

mod motor;
use motor::MotorController;

// Resource assignments for hardware peripherals
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pc13: PC13, // Phase A low
        pa9: PA9,   // Phase B high
        pa12: PA12, // Phase B low
        pa10: PA10, // Phase C high
        pb15: PB15, // Phase C low
    }
    hall: HallResources {
        pb6: PB6,   // H1 / Encoder A+
        pb7: PB7,   // H2 / Encoder B+
        pb8: PB8,   // H3 / Encoder Z+
    }
}

// Use panic-probe for panics
use panic_probe as _;

const OUT_QUEUE_SIZE: usize = 2048;
const MAX_PACKET_SIZE: usize = 512;

// UART transport constants
#[cfg(feature = "transport-uart")]
const UART_BAUD: u32 = 115_200;
#[cfg(feature = "transport-uart")]
const UART_BUF_LEN: usize = 1024;

// Type aliases for our application
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;

#[cfg(feature = "transport-uart")]
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, UartReader>;
#[cfg(feature = "transport-rtt")]
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, RttReader>;

/// Statically store our outgoing packet buffer
static OUTQ: Queue = kit::Queue::new();

/// Statically store our netstack
static STACK: Stack = kit::new_target_stack(OUTQ.stream_producer(), MAX_PACKET_SIZE as u16);

/// Buffers for RX worker
static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
static SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();

/// UART buffers (only for UART transport)
#[cfg(feature = "transport-uart")]
static UART_TX_BUF: StaticCell<[u8; UART_BUF_LEN]> = StaticCell::new();
#[cfg(feature = "transport-uart")]
static UART_RX_BUF: StaticCell<[u8; UART_BUF_LEN]> = StaticCell::new();

/// RTT defmt channel storage (for UART mode - hybrid defmt sink)
#[cfg(feature = "transport-uart")]
static RTT_DEFMT_UP: StaticCell<ergot::rtt_target::UpChannel> = StaticCell::new();

/// RTT channel storage (for RTT transport mode)
#[cfg(feature = "transport-rtt")]
static RTT_DEFMT_CHANNEL: StaticCell<ergot::rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_UP: StaticCell<ergot::rtt_target::UpChannel> = StaticCell::new();
#[cfg(feature = "transport-rtt")]
static RTT_ERGOT_DOWN: StaticCell<ergot::rtt_target::DownChannel> = StaticCell::new();

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

/// Handle for ADC1 injected conversions (TIM1-triggered): ia, vbus, temp.
static ADC1_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC1, 3>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
/// Handle for ADC2 injected conversions (TIM1-triggered).
static ADC2_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC2, 2>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Latest phase current samples (from ADC1/ADC2 injected sequences).
static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts (updated in ADC interrupt).
static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest measured FET temperature in 0.1°C units (updated in ADC interrupt).
static FET_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);
/// Sequence counter for ADC samples (incremented each poll).
static ADC_SEQ: AtomicU32 = AtomicU32::new(0);

// ADC conversion constants for VBUS measurement.
const ADC_MAX_COUNTS: u32 = 4095;
const ADC_VREF_MV: u32 = 3300;
// B-G431B-ESC1 VBUS divider: 169k (top, R68) and 18k (bottom, R76).
// Vsense = Vbus * 18 / 187  =>  Vbus = Vsense * 187 / 18.
const VBUS_DIV_NUM: u32 = 187;
const VBUS_DIV_DEN: u32 = 18;

fn vbus_mv_from_adc(raw: u16) -> u32 {
    let raw = raw as u32;
    let vsense_mv = raw * ADC_VREF_MV / ADC_MAX_COUNTS;
    vsense_mv * VBUS_DIV_NUM / VBUS_DIV_DEN
}

// Temperature sensing constants for PB14 NTC divider:
//  - 10k NTC to 3.3V
//  - 4.7k resistor to GND
// Using a simple Beta model with Beta = 3455 and R0 = 10k at 25°C.
const NTC_R_BOTTOM_OHM: f32 = 4700.0;
const NTC_R0_OHM: f32 = 10_000.0;
const NTC_BETA: f32 = 3455.0;
const NTC_T0_K: f32 = 273.15 + 25.0;
const NTC_KELVIN_OFFSET: f32 = 273.15;

fn fet_temp_c_from_adc(raw: u16) -> f32 {
    let adc = raw as f32;
    // Avoid divide-by-zero when ADC reading is very small.
    let eps = 0.1;
    let r_ntc = NTC_R_BOTTOM_OHM * (4096.0 / (adc + eps) - 1.0);
    // Beta-model temperature calculation.
    let ln_term = libm::logf(NTC_R0_OHM / r_ntc);
    let temp_k = NTC_BETA * NTC_T0_K / (NTC_BETA - NTC_T0_K * ln_term);
    temp_k - NTC_KELVIN_OFFSET
}

#[cfg(feature = "transport-uart")]
bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
            // ADC clock source: use SYSCLK (170MHz)
            config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        }
        embassy_stm32::init(config)
    };

    // ========== TRANSPORT-SPECIFIC RTT INITIALIZATION ==========

    // UART mode: Single RTT channel for defmt (hybrid: RTT + network forwarding)
    #[cfg(feature = "transport-uart")]
    let defmt_consumer = {
        let channels = rtt_init! {
            up: {
                0: { size: 2048, mode: NoBlockSkip, name: "defmt" }
            }
        };
        let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
        defmt_sink::init_network_and_rtt(defmt_up)
    };

    // RTT mode: Separate RTT channels for defmt and ergot transport
    #[cfg(feature = "transport-rtt")]
    let (rtt_rx, rtt_tx) = {
        use ergot::logging::defmt_sink;
        let channels = rtt_init! {
            up: {
                0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
                1: { size: 2048, mode: NoBlockSkip, name: "ergot" }
            }
            down: {
                0: { size: 1024, name: "ergot-down" }
            }
        };
        // Initialize defmt sink (RTT only, no network forwarding)
        let defmt_up = RTT_DEFMT_CHANNEL.init(channels.up.0);
        defmt_sink::init_rtt(defmt_up);
        // Store ergot RTT channels
        let ergot_up = RTT_ERGOT_UP.init(channels.up.1);
        let ergot_down = RTT_ERGOT_DOWN.init(channels.down.0);
        (RttReader::new(ergot_down), RttWriter::new(ergot_up))
    };

    #[cfg(feature = "transport-uart")]
    defmt::info!("Oxifoc starting - ergot over USART2 VCP + defmt sink");
    #[cfg(feature = "transport-rtt")]
    defmt::info!("Oxifoc starting - ergot over RTT");

    // Configure OPAMPs as PGAs for phase current shunts.
    //
    // OPAMP1: phase A current (PA1/PA3).
    // OPAMP2: phase B current (PA7/PA5).
    // OPAMP3: phase C current (PB0/PB2).
    //
    // We use internal outputs so the ADC sees the amplified shunt voltages
    // directly on dedicated internal channels.
    let mut opamp1 = OpAmp::new(p.OPAMP1, OpAmpSpeed::HighSpeed);
    opamp1.calibrate();
    let ia_chan = opamp1.pga_int(p.PA1, OpAmpGain::Mul16).degrade_adc(); // -> AnyAdcChannel<ADC1>

    let mut opamp2 = OpAmp::new(p.OPAMP2, OpAmpSpeed::HighSpeed);
    opamp2.calibrate();
    let ib_chan = opamp2.pga_int(p.PA7, OpAmpGain::Mul16).degrade_adc(); // -> AnyAdcChannel<ADC2>

    let mut opamp3 = OpAmp::new(p.OPAMP3, OpAmpSpeed::HighSpeed);
    opamp3.calibrate();
    let ic_chan = opamp3.pga_int(p.PB0, OpAmpGain::Mul16).degrade_adc(); // -> AnyAdcChannel<ADC2>

    // ========== ADC CONFIGURATION ==========
    //
    // All ADC sampling via TIM1-triggered injected conversions (no DMA).
    // Host polls for samples; all values updated in ADC interrupt.
    //
    // ADC clock: Embassy auto-selects prescaler to keep ADC clock ≤ 60MHz.
    // With SYSCLK=170MHz, prescaler=DIV4 → ADC clock = 42.5MHz.
    //
    // Conversion time = (sample_time + 12.5) cycles per channel.
    //
    // ADC1 injected sequence (3 channels, ~134 cycles total ≈ 3.2µs):
    //   - ia:   24.5 + 12.5 = 37 cycles  (phase A current, low impedance)
    //   - vbus: 47.5 + 12.5 = 60 cycles  (16kΩ divider, needs longer sample)
    //   - temp: 24.5 + 12.5 = 37 cycles  (3kΩ NTC divider)
    //
    // ADC2 injected sequence (2 channels, ~74 cycles total ≈ 1.7µs):
    //   - ib: 24.5 + 12.5 = 37 cycles  (phase B current)
    //   - ic: 24.5 + 12.5 = 37 cycles  (phase C current)
    //
    // Both ADCs triggered simultaneously by TIM1_TRGO2. Phase currents (ia, ib, ic)
    // are sampled within ~1µs of trigger. ADC1 takes longer due to vbus/temp,
    // so its interrupt signals completion of both ADCs.
    //
    // NOTE: BEMF pins (PA4/PC4/PB11) and GPIO_BEMF (PB5) are intentionally left
    // unused for now; they'll be wired up later for sensorless BEMF detection.

    let adc1 = Adc::new(p.ADC1, AdcConfig::default());
    let adc2 = Adc::new(p.ADC2, AdcConfig::default());

    let injected_trigger = ConversionTrigger {
        // TIM1_TRGO2 (routed from TIM1_CH4 compare in MotorPwm).
        channel: 8,
        edge: Exten::RISING_EDGE,
    };

    // ADC1 injected: phase A current + VBUS + temperature (finishes last → interrupt)
    let vbus_chan = p.PA0.degrade_adc();
    let temp_chan = p.PB14.degrade_adc();
    let injected_adc1 = adc1.setup_injected_conversions(
        [
            (ia_chan, SampleTime::CYCLES24_5),   // Phase A: low-Z opamp output
            (vbus_chan, SampleTime::CYCLES47_5), // VBUS: 16kΩ divider impedance
            (temp_chan, SampleTime::CYCLES24_5), // Temp: 3kΩ NTC divider
        ],
        injected_trigger,
        true, // Interrupt on ADC1 (finishes last, guarantees ADC2 is also done)
    );

    // ADC2 injected: phase B and C currents (finishes first, no interrupt)
    let injected_adc2 = adc2.setup_injected_conversions(
        [
            (ib_chan, SampleTime::CYCLES24_5), // Phase B: low-Z opamp output
            (ic_chan, SampleTime::CYCLES24_5), // Phase C: low-Z opamp output
        ],
        injected_trigger,
        false, // No interrupt - ADC1 interrupt handles both
    );

    // Store injected ADC handles for interrupt access
    ADC1_INJECTED.lock(|cell| cell.replace(Some(injected_adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(injected_adc2)));

    // Enable shared ADC1/ADC2 interrupt
    unsafe {
        ADC1_2::unpend();
        ADC1_2::enable();
    }

    // ========== TRANSPORT-SPECIFIC SETUP ==========

    // UART mode: Configure USART2 on ST-LINK VCP (PB3 TX, PB4 RX)
    #[cfg(feature = "transport-uart")]
    let (uart_tx, rx_worker) = {
        let mut uart_cfg = UartConfig::default();
        uart_cfg.baudrate = UART_BAUD;
        uart_cfg.parity = UartParity::ParityNone;
        uart_cfg.stop_bits = UartStopBits::STOP1;
        let tx_buf = UART_TX_BUF.init([0u8; UART_BUF_LEN]);
        let rx_buf = UART_RX_BUF.init([0u8; UART_BUF_LEN]);
        let uart = BufferedUart::new(p.USART2, p.PB4, p.PB3, tx_buf, rx_buf, Irqs, uart_cfg)
            .expect("USART2 init failed");
        let (uart_tx, uart_rx) = uart.split();
        let rx_worker = RxWorker::new_target(&STACK, UartReader::new(uart_rx), ());
        (uart_tx, rx_worker)
    };

    // RTT mode: Create RX worker using RTT channels
    #[cfg(feature = "transport-rtt")]
    let rx_worker = RxWorker::new_target(&STACK, rtt_rx, ());

    // Button: PC10, external pull-up, active-low to GND
    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::None);
    defmt::info!("Button configured on PC10 (active-low)");

    // LED on PC6
    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);
    // Back-EMF enable (GPIO_BEMF on PB5) - keep defined but unused for now.
    // let mut gpio_bemf = Output::new(p.PB5, Level::Low, Speed::Low);

    // Initialize motor controller with TIM1 and motor pins
    let r = split_resources!(p);

    // Initialize Hall sensor inputs with pull-ups
    let hall_h1 = Input::new(r.hall.pb6, Pull::Up);
    let hall_h2 = Input::new(r.hall.pb7, Pull::Up);
    let hall_h3 = Input::new(r.hall.pb8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

    let motor_ctrl = MotorController::init(r.motor);

    // Spawn I/O workers (transport-specific)
    spawner.spawn(
        run_rx(
            rx_worker,
            RECV_BUF.init_with(|| [0u8; MAX_PACKET_SIZE]),
            SCRATCH_BUF.init_with(|| [0u8; 64]),
        )
        .unwrap(),
    );

    #[cfg(feature = "transport-uart")]
    {
        spawner.spawn(run_tx_uart(UartWriter::new(uart_tx)).unwrap());
        // NOTE: defmt_forwarder disabled - causes feedback loop with ergot logs
        // Device logs are already visible via RTT in probe-rs terminal
        let _ = defmt_consumer; // suppress unused warning
    }

    #[cfg(feature = "transport-rtt")]
    spawner.spawn(run_tx_rtt(rtt_tx).unwrap());

    // Initialize motor command channel
    let motor_cmd_channel = MOTOR_CMD_CHANNEL.init(embassy_sync::channel::Channel::new());
    let motor_cmd_receiver = motor_cmd_channel.receiver();
    let motor_cmd_sender = motor_cmd_channel.sender();

    spawner.spawn(button_handler(button).unwrap());
    spawner.spawn(status_reporter().unwrap());
    spawner.spawn(info_server().unwrap());
    spawner.spawn(adc_sample_server().unwrap());
    spawner.spawn(motor_control_task(motor_ctrl, motor_cmd_receiver).unwrap());
    spawner.spawn(motor_command_server(motor_cmd_sender).unwrap());

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

/// Worker task for incoming ergot data (transport-agnostic)
#[embassy_executor::task]
async fn run_rx(mut rcvr: RxWorker, recv_buf: &'static mut [u8], scratch_buf: &'static mut [u8]) {
    loop {
        let _ = rcvr.run(recv_buf, scratch_buf).await;
    }
}

/// Worker task for outgoing ergot data via UART (transport-uart only)
#[cfg(feature = "transport-uart")]
#[embassy_executor::task]
async fn run_tx_uart(mut tx: UartWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// Worker task for outgoing ergot data via RTT (transport-rtt only)
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
async fn run_tx_rtt(mut tx: RttWriter) {
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
    embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        MotorCommand,
        4,
    >,
> = StaticCell::new();
/// ADC sample server - responds to host poll requests with current ADC values.
/// Host controls polling rate; device just returns latest values from atomics.
#[embassy_executor::task]
async fn adc_sample_server() {
    use oxifoc_protocol::AdcSampleEndpoint;

    defmt::info!("ADC sample server started (poll-based)");

    let server = STACK
        .endpoints()
        .bounded_server::<AdcSampleEndpoint, 2>(Some("adc"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_: &()| async {
                let seq = ADC_SEQ.fetch_add(1, Ordering::Relaxed);
                oxifoc_protocol::AdcSample {
                    ia: IA_SAMPLE.load(Ordering::Relaxed),
                    ib: IB_SAMPLE.load(Ordering::Relaxed),
                    ic: IC_SAMPLE.load(Ordering::Relaxed),
                    vbus_mv: VBUS_MV.load(Ordering::Relaxed),
                    fet_temp_c_x10: FET_TEMP_C_X10.load(Ordering::Relaxed),
                    seq,
                }
            })
            .await;
    }
}

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
        // Latest injected current samples (TIM1-synchronized via ADC1/ADC2 injected conversions).
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);
        let vbus_mv = VBUS_MV.load(Ordering::Relaxed);
        let _ = (ia_raw, ib_raw, ic_raw, vbus_mv); // placeholder for future control logic

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
                let sender_clone = motor_cmd_sender;
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

/// ADC1/ADC2 shared interrupt: read all injected ADC samples.
///
/// ADC1: ia (phase A), vbus, temp
/// ADC2: ib (phase B), ic (phase C)
///
/// Triggered by ADC1 end-of-sequence (ADC1 finishes last).
/// Stores raw phase currents; converts vbus/temp to engineering units.
#[interrupt]
unsafe fn ADC1_2() {
    // Read ADC1 injected: phase A current, VBUS voltage, FET temperature
    ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IA_SAMPLE.store(samples[0], Ordering::Relaxed);

            // Convert VBUS raw ADC to millivolts
            let vbus_mv = vbus_mv_from_adc(samples[1]);
            VBUS_MV.store(vbus_mv, Ordering::Relaxed);

            // Convert temperature raw ADC to 0.1°C units
            let temp_c = fet_temp_c_from_adc(samples[2]);
            let temp_c_x10 = if temp_c.is_finite() && temp_c >= 0.0 {
                (temp_c * 10.0) as u16
            } else {
                0
            };
            FET_TEMP_C_X10.store(temp_c_x10, Ordering::Relaxed);
        }
    });

    // Read ADC2 injected: phase B and C currents
    ADC2_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IB_SAMPLE.store(samples[0], Ordering::Relaxed);
            IC_SAMPLE.store(samples[1], Ordering::Relaxed);
        }
    });
}

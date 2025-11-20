#![no_std]
#![no_main]

use core::cell::RefCell;
use core::pin::pin;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::adc::RingBufferedAdc;
use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, ConversionTrigger, Exten, InjectedAdc, RegularConversionMode,
    SampleTime,
};
use embassy_stm32::bind_interrupts;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::peripherals;
use embassy_stm32::{
    interrupt,
    interrupt::typelevel::{ADC1_2, Interrupt},
};
use embassy_stm32::usart::{BufferedUart, Config as UartConfig, Parity as UartParity, StopBits as UartStopBits};
use embassy_time::{Duration, Timer, with_timeout};
use ergot::{
    Address,
    exports::bbq2::traits::coordination::cas::AtomicCoord,
    logging::{defmt_sink, defmtlog::ErgotDefmtTx},
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
    well_known::ErgotDefmtTxTopic,
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{
    ButtonEndpoint, ButtonEvent, DeviceInfo, InfoEndpoint, MotorCommand, MotorEndpoint,
};
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as SyncRawMutex;
use embassy_sync::channel;

mod usart_io;
use usart_io::{UartReader, UartWriter};

mod motor;
use motor::MotorController;

// Use panic-probe for panics
use panic_probe as _;

const OUT_QUEUE_SIZE: usize = 2048;
const MAX_PACKET_SIZE: usize = 512;
const UART_BAUD: u32 = 921_600;
const UART_BUF_LEN: usize = 1024;

// Type aliases for our application
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, UartReader>;

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

/// Handle for ADC1 injected conversions (TIM1-triggered).
static ADC1_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC1, 1>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
/// Handle for ADC2 injected conversions (TIM1-triggered).
static ADC2_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC2, 2>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Latest phase current samples (from ADC1/ADC2 injected sequences).
static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts.
static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest measured FET temperature in 0.1°C units.
static FET_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);

/// Sequence counter for streamed ADC samples.
static ADC_SEQ: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Simple decimation counter to reduce streaming load.
const ADC_STREAM_DECIM: u8 = 8;
static ADC_DECIM_COUNTER: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Channel used to send decimated ADC samples from ISR to telemetry task.
static ADC_SAMPLE_CH: StaticCell<channel::Channel<SyncRawMutex, oxifoc_protocol::AdcSample, 16>> =
    StaticCell::new();

/// Raw pointer to the ADC sample channel (set once in main, used in ISR).
static mut ADC_SAMPLE_CH_REF: Option<
    &'static channel::Channel<SyncRawMutex, oxifoc_protocol::AdcSample, 16>,
> = None;

// DMA buffer used for ADC1 regular conversions (VBUS measurement).
// Must be non-empty and <= 0xFFFF elements; half of this length is used as the
// measurement buffer by the RingBufferedAdc::read() API.
const VBUS_DMA_BUF_LEN: usize = 64;
const VBUS_MEAS_BUF_LEN: usize = VBUS_DMA_BUF_LEN / 2;
static VBUS_DMA_BUF: StaticCell<[u16; VBUS_DMA_BUF_LEN]> = StaticCell::new();

/// UART buffers
static UART_TX_BUF: StaticCell<[u8; UART_BUF_LEN]> = StaticCell::new();
static UART_RX_BUF: StaticCell<[u8; UART_BUF_LEN]> = StaticCell::new();

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
        }
        embassy_stm32::init(config)
    };

    // Initialize defmt sink before any logging (network output)
    let defmt_consumer = defmt_sink::init();

    defmt::info!("Oxifoc starting - ergot over USART2 VCP + defmt sink");

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

    // ADC1/ADC2: phase current feedback via injected conversions.
    //
    // NOTE: BEMF pins (PA4/PC4/PB11) and GPIO_BEMF (PB5) are intentionally left
    // unused for now; they'll be wired up later for sensorless BEMF detection.
    //
    // For ADC1 we use both:
    //   - Regular conversions (via DMA ring buffer) to sample:
    //       * DC bus voltage on the VBUS_SENS divider (PA0 -> ADC1_IN1), and
    //       * FET temperature via NTC on PB14 (ADC1_IN5).
    //   - Injected conversions (TIM1-triggered) to sample phase A current.
    //
    // ADC2 is used only for injected conversions (phases B and C).
    let adc1 = Adc::new(p.ADC1, AdcConfig::default());
    let adc2 = Adc::new(p.ADC2, AdcConfig::default());

    let injected_trigger = ConversionTrigger {
        // TIM1_TRGO2 (we route this from TIM1_CH4 compare in MotorPwm).
        channel: 8,
        edge: Exten::RISING_EDGE,
    };

    // ADC1 regular + injected:
    //
    // Regular sequence: VBUS_SENS on PA0 and NTC temperature on PB14, sampled
    // continuously via DMA into a ring buffer. A separate async task
    // (vbus_task) reads this buffer and updates VBUS_MV (mV) and
    // FET_TEMP_C_X10 (0.1°C units).
    //
    // Injected sequence: phase A current (single rank), triggered by TIM1_TRGO2.
    let vbus_dma_buf = VBUS_DMA_BUF.init([0u16; VBUS_DMA_BUF_LEN]);
    let vbus_chan = p.PA0.degrade_adc();
    let temp_chan = p.PB14.degrade_adc();
    let (adc1_ring, injected_adc1): (RingBufferedAdc<'static, peripherals::ADC1>, _) = adc1
        .into_ring_buffered_and_injected(
            p.DMA1_CH1,
            vbus_dma_buf,
            [
                (vbus_chan, SampleTime::CYCLES24_5),
                (temp_chan, SampleTime::CYCLES24_5),
            ]
            .into_iter(),
            RegularConversionMode::Continuous,
            [(ia_chan, SampleTime::CYCLES24_5)],
            injected_trigger,
            false,
        );

    // Injected ADC2: phase B and C currents (two ranks).
    let injected_adc2 = adc2.setup_injected_conversions(
        [
            (ib_chan, SampleTime::CYCLES24_5),
            (ic_chan, SampleTime::CYCLES24_5),
        ],
        injected_trigger,
        true,
    );

    // Store injected ADC handles and enable ADC1/2 interrupt.
    ADC1_INJECTED.lock(|cell| {
        cell.replace(Some(injected_adc1));
    });
    ADC2_INJECTED.lock(|cell| {
        cell.replace(Some(injected_adc2));
    });

    // Spawn VBUS sampling task (reads ADC1 regular ring buffer and updates VBUS_MV).
    spawner.spawn(vbus_task(adc1_ring).unwrap());

    // Initialize ADC streaming channel and spawn telemetry task.
    let adc_sample_ch = ADC_SAMPLE_CH.init(channel::Channel::new());
    let adc_sample_rx = adc_sample_ch.receiver();
    unsafe {
        ADC_SAMPLE_CH_REF = Some(adc_sample_ch);
    }
    spawner.spawn(adc_telemetry_task(adc_sample_rx).unwrap());

    unsafe {
        ADC1_2::unpend();
        ADC1_2::enable();
    }

    // Configure USART2 on ST-LINK VCP (PB3 TX, PB4 RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = UART_BAUD;
    uart_cfg.parity = UartParity::ParityNone;
    uart_cfg.stop_bits = UartStopBits::STOP1;
    let tx_buf = UART_TX_BUF.init([0u8; UART_BUF_LEN]);
    let rx_buf = UART_RX_BUF.init([0u8; UART_BUF_LEN]);
    let uart = BufferedUart::new(
        p.USART2,
        p.PB4,
        p.PB3,
        tx_buf,
        rx_buf,
        Irqs,
        uart_cfg,
    )
    .expect("USART2 init failed");
    let (uart_tx, uart_rx) = uart.split();

    // Create RX worker for incoming ergot messages (it will set interface to Inactive, then Active after first frame)
    let rx_worker = RxWorker::new_target(&STACK, UartReader::new(uart_rx), ());

    // Button: PC10, external pull-up, active-low to GND
    let button = ExtiInput::new(p.PC10, p.EXTI10, Pull::None);
    defmt::info!("Button configured on PC10 (active-low)");

    // LED on PC6
    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);
    // Back-EMF enable (GPIO_BEMF on PB5) - keep defined but unused for now.
    // let mut gpio_bemf = Output::new(p.PB5, Level::Low, Speed::Low);

    // Initialize motor controller with TIM1 and motor pins
    let motor_ctrl = MotorController::init(
        p.TIM1, p.PA8,  // Phase A high
        p.PC13, // Phase A low
        p.PA9,  // Phase B high
        p.PA12, // Phase B low
        p.PA10, // Phase C high
        p.PB15, // Phase C low
    );

    // Spawn I/O workers
    spawner.spawn(
        run_rx(
            rx_worker,
            RECV_BUF.init_with(|| [0u8; MAX_PACKET_SIZE]),
            SCRATCH_BUF.init_with(|| [0u8; 64]),
        )
        .unwrap(),
    );
    spawner.spawn(run_tx(UartWriter::new(uart_tx)).unwrap());
    spawner
        .spawn(defmt_forwarder(defmt_consumer).unwrap());

    // Initialize motor command channel
    let motor_cmd_channel = MOTOR_CMD_CHANNEL.init(embassy_sync::channel::Channel::new());
    let motor_cmd_receiver = motor_cmd_channel.receiver();
    let motor_cmd_sender = motor_cmd_channel.sender();

    spawner.spawn(button_handler(button).unwrap());
    spawner.spawn(status_reporter().unwrap());
    spawner.spawn(info_server().unwrap());
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

/// Worker task for incoming ergot data via USART2
#[embassy_executor::task]
async fn run_rx(mut rcvr: RxWorker, recv_buf: &'static mut [u8], scratch_buf: &'static mut [u8]) {
    loop {
        let _ = rcvr.run(recv_buf, scratch_buf).await;
    }
}

/// Worker task for outgoing ergot data via USART2
#[embassy_executor::task]
async fn run_tx(mut tx: UartWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// Forward defmt frames from the sink to ergot network
#[embassy_executor::task]
async fn defmt_forwarder(consumer: defmt_sink::DefmtConsumer) {
    loop {
        let frame = consumer.wait_read().await;
        let _ = STACK
            .topics()
            .broadcast_borrowed::<ErgotDefmtTxTopic>(&ErgotDefmtTx { frame: &frame }, None);
        frame.release();
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

/// Periodically read VBUS and FET temperature via ADC1 regular conversions and
/// update VBUS_MV / FET_TEMP_C_X10.
#[embassy_executor::task]
async fn vbus_task(mut adc: RingBufferedAdc<'static, peripherals::ADC1>) {
    defmt::info!("VBUS + temperature sampling task started");

    let mut meas = [0u16; VBUS_MEAS_BUF_LEN];

    loop {
        match adc.read(&mut meas).await {
            Ok(n) if n > 1 => {
                let mut sum_vbus: u32 = 0;
                let mut cnt_vbus: u32 = 0;
                let mut sum_temp: u32 = 0;
                let mut cnt_temp: u32 = 0;

                for (idx, &raw) in meas[..n].iter().enumerate() {
                    if idx % 2 == 0 {
                        sum_vbus += raw as u32;
                        cnt_vbus += 1;
                    } else {
                        sum_temp += raw as u32;
                        cnt_temp += 1;
                    }
                }

                if cnt_vbus > 0 {
                    let avg_vbus_raw = (sum_vbus / cnt_vbus) as u16;
                    let vbus_mv = vbus_mv_from_adc(avg_vbus_raw);
                    VBUS_MV.store(vbus_mv, Ordering::Relaxed);
                }

                if cnt_temp > 0 {
                    let avg_temp_raw = (sum_temp / cnt_temp) as u16;
                    let temp_c = fet_temp_c_from_adc(avg_temp_raw);
                    let temp_c_x10 = if temp_c.is_finite() {
                        let clamped = if temp_c < 0.0 { 0.0 } else { temp_c };
                        (clamped * 10.0) as u16
                    } else {
                        0
                    };
                    FET_TEMP_C_X10.store(temp_c_x10, Ordering::Relaxed);
                }
            }
            Ok(_) => {}
            Err(_) => {
                defmt::warn!("VBUS/temperature ADC overrun, clearing buffer");
                adc.clear();
            }
        }
    }
}

/// ADC telemetry task - sends decimated samples to host via ergot.
#[embassy_executor::task]
async fn adc_telemetry_task(
    adc_rx: channel::Receiver<'static, SyncRawMutex, oxifoc_protocol::AdcSample, 16>,
) {
    use oxifoc_protocol::AdcSampleEndpoint;

    // Host controller at network 1, node 1.
    let host_addr = Address {
        network_id: 1,
        node_id: 1,
        port_id: 0,
    };
    let client = STACK
        .endpoints()
        .client::<AdcSampleEndpoint>(host_addr, Some("adc"));

    loop {
        let sample = adc_rx.receive().await;
        let _ = client.request(&sample).await;
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

/// ADC1/ADC2 shared interrupt: read injected ADC1 samples (phase current).
#[interrupt]
unsafe fn ADC1_2() {
    // Read ADC1 injected (phase A).
    ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IA_SAMPLE.store(samples[0], Ordering::Relaxed);
        }
    });

    // Read ADC2 injected (phases B and C).
    ADC2_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IB_SAMPLE.store(samples[0], Ordering::Relaxed);
            IC_SAMPLE.store(samples[1], Ordering::Relaxed);
        }
    });

    // Decimate and enqueue ADC samples for streaming to host.
    let cnt = ADC_DECIM_COUNTER
        .fetch_add(1, Ordering::Relaxed)
        .wrapping_add(1);
    if cnt % ADC_STREAM_DECIM == 0 {
        let seq = ADC_SEQ.fetch_add(1, Ordering::Relaxed);
        let sample = oxifoc_protocol::AdcSample {
            ia: IA_SAMPLE.load(Ordering::Relaxed),
            ib: IB_SAMPLE.load(Ordering::Relaxed),
            ic: IC_SAMPLE.load(Ordering::Relaxed),
            vbus_mv: VBUS_MV.load(Ordering::Relaxed),
            fet_temp_c_x10: FET_TEMP_C_X10.load(Ordering::Relaxed),
            seq,
        };
        unsafe {
            if let Some(ch) = ADC_SAMPLE_CH_REF {
                let _ = ch.try_send(sample);
            }
        }
    }
}

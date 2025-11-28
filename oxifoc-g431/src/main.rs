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
use embassy_time::{Duration, Timer};
use ergot::{
    exports::bbq2::traits::coordination::cas::AtomicCoord,
    rtt_target::{ChannelMode::*, rtt_init},
    toolkits::embedded_io_async_v0_6::{self as kit, tx_worker},
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_protocol::{DeviceInfo, InfoEndpoint, MotorCommand, MotorEndpoint};
use static_cell::StaticCell;

use assign_resources::assign_resources;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;

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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as EmbassyCS;
use motor::current::G431CurrentSensor;
use motor::pwm::{MotorPwm, MotorPwmConfig};
use oxifoc_core::foc::controller::{FocController, FocTelemetry};
use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};
use oxifoc_core::foc::sensors::{AngleSample, AngleSensor, CurrentSensor};
use oxifoc_core::motor::{ControlMode, FocDriver};
use oxifoc_protocol::MotorState;

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
/// Conservative default bus voltage used until ADC updates arrive.
const INITIAL_VBUS_VOLTS: f32 = 12.0;
/// Maps motor duty percent (0-100) to a target q-axis current in Amps.
const MAX_IQ_TARGET_A: f32 = 10.0;
/// Timebase for Hall interpolation (match embassy_time ticks).
const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

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

/// Hall sensor data (updated by hall_sensor_task).
/// Angle stored as f32 bit-pattern in AtomicU32.
static HALL_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);
/// Hall direction: 0=Stopped, 1=Clockwise, 2=CounterClockwise
static HALL_DIRECTION: AtomicU8 = AtomicU8::new(0);
/// Hall state (0-5)
static HALL_STATE: AtomicU8 = AtomicU8::new(0);
/// Hall error count
static HALL_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
/// Sequence counter for Hall sensor samples
static HALL_SEQ: AtomicU32 = AtomicU32::new(0);

/// Keep Hall ExtiInput instances alive for EXTI interrupt handling.
static HALL_INPUTS: StaticCell<(ExtiInput<'static>, ExtiInput<'static>, ExtiInput<'static>)> =
    StaticCell::new();
static mut HALL_INPUTS_PTR: Option<&'static (
    ExtiInput<'static>,
    ExtiInput<'static>,
    ExtiInput<'static>,
)> = None;

struct HallEdgeMailbox {
    seq: AtomicU32,
    state: AtomicU8,
    ticks: AtomicU32,
}

impl HallEdgeMailbox {
    const fn new() -> Self {
        Self {
            seq: AtomicU32::new(0),
            state: AtomicU8::new(0),
            ticks: AtomicU32::new(0),
        }
    }

    fn write(&self, state: u8, ticks: u32) {
        self.state.store(state, Ordering::Relaxed);
        self.ticks.store(ticks, Ordering::Relaxed);
        self.seq.fetch_add(1, Ordering::Release);
    }

    fn load(&self) -> (u32, u8, u32) {
        let seq = self.seq.load(Ordering::Acquire);
        let state = self.state.load(Ordering::Relaxed);
        let ticks = self.ticks.load(Ordering::Relaxed);
        (seq, state, ticks)
    }
}

/// Mailbox for Hall edge updates from EXTI to ADC ISR.
static HALL_EDGE_MAILBOX: HallEdgeMailbox = HallEdgeMailbox::new();

/// Hall estimator shared between EXTI/Hall task and ADC ISR.
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Angle sensor proxy for the FOC driver; pulls snapshots from `HALL_ESTIMATOR`.
struct HallAngleProxy;

impl HallAngleProxy {
    const fn new() -> Self {
        Self
    }
}

impl AngleSensor for HallAngleProxy {
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().and_then(|h| h.sample_at(now_ticks)))
    }

    fn read_angle(&self) -> f32 {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now).map(|s| s.angle).unwrap_or(0.0)
    }

    fn read_direction(&self) -> Direction {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now)
            .map(|s| s.direction)
            .unwrap_or(Direction::Stopped)
    }

    fn error_count(&self) -> u32 {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().map(|h| h.error_count()).unwrap_or(0))
    }

    fn reset_errors(&mut self) {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                h.reset_errors();
            }
        });
    }
}

/// FOC telemetry data (updated by ADC ISR)
static FOC_TELEMETRY: Watch<EmbassyCS, FocTelemetry, 1> = Watch::new();

/// FOC command channel (tasks → ISR)
static FOC_CMD: Channel<EmbassyCS, ControlMode, 4> = Channel::new();

/// FOC driver storage (mutated only inside the ADC ISR)
type FocDriverType = FocDriver<MotorPwm<'static>, G431CurrentSensor, HallAngleProxy>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Shared fault registry
#[allow(dead_code)]
static FAULT_REGISTRY: FaultRegistry = FaultRegistry::new();

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

#[inline]
fn read_hall_state_fast() -> u8 {
    if let Some((h1, h2, h3)) = unsafe { HALL_INPUTS_PTR } {
        let mut state = 0u8;
        if h1.is_high() {
            state |= 0b001;
        }
        if h2.is_high() {
            state |= 0b010;
        }
        if h3.is_high() {
            state |= 0b100;
        }
        state
    } else {
        0
    }
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

    // LED on PC6
    let mut led = Output::new(p.PC6, Level::Low, Speed::Low);
    // Back-EMF enable (GPIO_BEMF on PB5) - keep defined but unused for now.
    // let mut gpio_bemf = Output::new(p.PB5, Level::Low, Speed::Low);

    // Initialize motor controller with TIM1 and motor pins
    let r = split_resources!(p);

    // Initialize Hall sensor inputs with pull-ups and EXTI (for async edge detection)
    let hall_h1 = ExtiInput::new(r.hall.pb6, p.EXTI6, Pull::Up);
    let hall_h2 = ExtiInput::new(r.hall.pb7, p.EXTI7, Pull::Up);
    let hall_h3 = ExtiInput::new(r.hall.pb8, p.EXTI8, Pull::Up);
    defmt::info!("Hall sensors configured: H1=PB6, H2=PB7, H3=PB8");

    // Keep Hall EXTI inputs alive to maintain configuration
    let inputs = HALL_INPUTS.init((hall_h1, hall_h2, hall_h3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }

    // Initialize Hall estimator
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(TIMEBASE_TICKS_PER_SEC)));
    });

    // Enable EXTI9_5 interrupt for Hall lines 6/7/8
    unsafe {
        interrupt::typelevel::EXTI9_5::unpend();
        interrupt::typelevel::EXTI9_5::enable();
    }

    // Build FOC driver (owns TIM1 PWM + current/hall sensors). Keep outputs off until commanded.
    let mut motor_pwm = MotorPwm::new(r.motor, MotorPwmConfig::default());
    motor_pwm.emergency_stop();

    let current_sensor = G431CurrentSensor::new();
    let angle_sensor = HallAngleProxy::new();
    let initial_vbus_v = (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(INITIAL_VBUS_VOLTS);
    let mut foc_driver = FocDriver::new(
        FocController::new(initial_vbus_v),
        motor_pwm,
        current_sensor,
        angle_sensor,
    );

    // Allow ADC injected conversions to start firing before zero-current calibration.
    Timer::after(Duration::from_millis(10)).await;
    foc_driver.current_sensor_mut().calibrate(300);

    // Install FOC driver for ISR-only access.
    FOC_DRIVER.lock(|cell| {
        cell.replace(Some(foc_driver));
    });

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

    spawner.spawn(info_server().unwrap());
    spawner.spawn(adc_sample_server().unwrap());
    spawner.spawn(hall_sensor_server().unwrap());
    spawner.spawn(motor_command_server().unwrap());

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

/// Hall sensor server - responds to host poll requests with current Hall sensor data
#[embassy_executor::task]
async fn hall_sensor_server() {
    use oxifoc_protocol::{HallDirection, HallSensorData, HallSensorEndpoint};

    defmt::info!("Hall sensor server started (poll-based)");

    let server = STACK
        .endpoints()
        .bounded_server::<HallSensorEndpoint, 2>(Some("hall"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_: &()| async {
                let seq = HALL_SEQ.fetch_add(1, Ordering::Relaxed);

                // Load angle from bit pattern
                let angle_bits = HALL_ANGLE_BITS.load(Ordering::Relaxed);
                let angle_rad = f32::from_bits(angle_bits);

                // Convert direction u8 back to enum
                let dir_u8 = HALL_DIRECTION.load(Ordering::Relaxed);
                let direction = match dir_u8 {
                    1 => HallDirection::Clockwise,
                    2 => HallDirection::CounterClockwise,
                    _ => HallDirection::Stopped,
                };

                HallSensorData {
                    angle_rad,
                    direction,
                    state: HALL_STATE.load(Ordering::Relaxed),
                    error_count: HALL_ERROR_COUNT.load(Ordering::Relaxed),
                    seq,
                }
            })
            .await;
    }
}

/// Motor command server - handles motor control commands via ergot
#[embassy_executor::task]
async fn motor_command_server() {
    defmt::info!("Motor command server started");

    let server = STACK
        .endpoints()
        .bounded_server::<MotorEndpoint, 2>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|cmd: &MotorCommand| {
                let cmd = cmd.clone();
                async move {
                    match cmd {
                        MotorCommand::Stop => {
                            motor::set_motor_state(MotorState::Stopped);
                            motor::set_motor_duty(0);
                            motor::set_motor_step(0);
                            let _ = FOC_CMD.try_send(ControlMode::Stopped);
                        }
                        MotorCommand::Start { duty } | MotorCommand::SetSpeed { duty } => {
                            let duty = duty.min(100);
                            motor::set_motor_state(MotorState::Running);
                            motor::set_motor_duty(duty);
                            motor::set_motor_step(0);

                            let iq_target = duty as f32 / 100.0 * MAX_IQ_TARGET_A;
                            let _ = FOC_CMD.try_send(ControlMode::CurrentControl {
                                iq_target,
                                id_target: 0.0,
                            });
                        }
                    }

                    motor::get_motor_status()
                }
            })
            .await;
    }
}

/// ADC1/ADC2 shared interrupt: read all injected ADC samples and run FOC control.
///
/// ADC1: ia (phase A), vbus, temp
/// ADC2: ib (phase B), ic (phase C)
///
/// Triggered by ADC1 end-of-sequence (ADC1 finishes last).
/// Stores raw phase currents; converts vbus/temp to engineering units.
/// Runs FOC control loop synchronized with PWM.
#[interrupt]
unsafe fn ADC1_2() {
    // Static state (ISR has exclusive access)
    static mut CONTROL_MODE: ControlMode = ControlMode::Stopped;
    static mut LAST_HALL_SEQ: u32 = 0;

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

    // Process FOC commands (non-blocking, ~20ns overhead)
    while let Ok(cmd) = FOC_CMD.try_receive() {
        *CONTROL_MODE = cmd;
    }

    // Incorporate latest Hall edge (from EXTI)
    let (edge_seq, edge_state, edge_ticks) = HALL_EDGE_MAILBOX.load();
    if edge_seq != *LAST_HALL_SEQ {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(edge_state, edge_ticks as u64);
            }
        });
        HALL_STATE.store(edge_state, Ordering::Relaxed);
        let err =
            HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().map(|h| h.error_count()).unwrap_or(0));
        HALL_ERROR_COUNT.store(err, Ordering::Relaxed);
        *LAST_HALL_SEQ = edge_seq;
    }

    // Snapshot current Hall data for telemetry/consumers
    let now_ticks = embassy_time::Instant::now().as_ticks();
    if let Some(sample) =
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().and_then(|h| h.sample_at(now_ticks)))
    {
        HALL_ANGLE_BITS.store(sample.angle.to_bits(), Ordering::Relaxed);
        let dir_u8 = match sample.direction {
            Direction::Stopped => 0,
            Direction::Clockwise => 1,
            Direction::CounterClockwise => 2,
        };
        HALL_DIRECTION.store(dir_u8, Ordering::Relaxed);
    }

    // Run FOC control loop
    FOC_DRIVER.lock(|cell| {
        if let Some(driver) = cell.borrow_mut().as_mut() {
            // Update bus voltage
            let vbus_mv = VBUS_MV.load(Ordering::Relaxed);
            driver.set_vbus(vbus_mv as f32 / 1000.0);

            // Update control mode
            driver.set_mode(*CONTROL_MODE);

            // Run FOC step (dt = 1/20kHz = 50µs)
            const DT: f32 = 1.0 / 20_000.0;
            match driver.step(DT, now_ticks) {
                Ok(telem) => {
                    // Broadcast telemetry to all listeners
                    FOC_TELEMETRY.sender().send(telem);
                }
                Err(_) => {
                    // Sensor not ready or other error - disable outputs
                    driver.set_mode(ControlMode::Stopped);
                }
            }
        }
    });
}

/// Handle Hall sensor edges (PB6/PB7/PB8) and timestamp them.
#[interrupt]
unsafe fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;
    HALL_EDGE_MAILBOX.write(state, ticks);

    // Clear EXTI pending bits for lines 6/7/8
    interrupt::typelevel::EXTI9_5::unpend();
}

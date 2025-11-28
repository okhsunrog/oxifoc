#![no_std]
#![no_main]

use core::cell::RefCell;
use core::pin::pin;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use assign_resources::assign_resources;
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{
    Config as StmConfig, Peri, bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    interrupt, peripherals,
    time::Hertz,
    usb,
};
use embassy_time::{Duration, Timer};
use ergot::{
    exports::bbq2::prod_cons::framed::FramedConsumer,
    exports::bbq2::traits::coordination::cas::AtomicCoord, toolkits::embassy_usb_v0_5 as kit,
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::foc::{
    controller::FocController,
    fault::FaultRegistry,
    hall_sensor::{Direction, HallSensor},
    pwm::PhasePwm,
};
use oxifoc_protocol::{DeviceInfo, InfoEndpoint};
use static_cell::StaticCell;

// Enable defmt global logger + panic probe
use panic_probe as _;

mod motor;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,
        pa9: PA9,
        pa10: PA10,
        pb13: PB13,
        pb14: PB14,
        pb15: PB15,
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT
        pc0: PC0,
        pc1: PC1,
        pc2: PC2,
        pc3: PC3,   // VBUS sense
    }
    hall: HallResources {
        pc6: PC6,
        pc7: PC7,
        pc8: PC8,
    }
}

type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, AppDriver>;

const OUT_QUEUE_SIZE: usize = 4096;
const MAX_PACKET_SIZE: usize = 512;

/// Fixed hardware scaling for Cheap FOCer 2 (STM32F405).
#[derive(Clone, Copy)]
struct BoardScaling {
    shunt_ohms: f32,
    current_amp_gain: f32,
    vbus_divider_ratio: f32,
}

impl BoardScaling {
    const fn new() -> Self {
        // Two 1 mΩ shunts in parallel => 0.5 mΩ effective.
        // DRV8301 amp gain set to 20 V/V to match external stage.
        // VBUS divider: 39k / 2.2k => ~18.7273:1 (ADC volts * ratio = bus volts).
        Self {
            shunt_ohms: 0.0005,
            current_amp_gain: 20.0,
            vbus_divider_ratio: (39.0 + 2.2) / 2.2,
        }
    }
}

static OUTQ: Queue = kit::Queue::new();
static STACK: Stack = kit::new_target_stack(OUTQ.framed_producer(), MAX_PACKET_SIZE as u16);
static USB_STORAGE: kit::WireStorage<256, 256, 64, 256> = kit::WireStorage::new();
static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
static EP_OUT_BUF: StaticCell<[u8; kit::USB_FS_MAX_PACKET_SIZE]> = StaticCell::new();
#[allow(dead_code)]
static FAULTS: FaultRegistry = FaultRegistry::new();
static HALL_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);
static HALL_DIRECTION: AtomicU8 = AtomicU8::new(0);
static HALL_STATE: AtomicU8 = AtomicU8::new(0);
static HALL_ESTIMATOR: embassy_sync::blocking_mutex::CriticalSectionMutex<
    RefCell<Option<HallSensor>>,
> = embassy_sync::blocking_mutex::CriticalSectionMutex::new(RefCell::new(None));
static HALL_INPUTS: StaticCell<(ExtiInput<'static>, ExtiInput<'static>, ExtiInput<'static>)> =
    StaticCell::new();
static mut HALL_INPUTS_PTR: Option<&'static (
    ExtiInput<'static>,
    ExtiInput<'static>,
    ExtiInput<'static>,
)> = None;
const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("oxifoc-f405 boot");
    let scaling = BoardScaling::new();

    // Clock setup for STM32F405 with 8MHz HSE (Simple FOCer 2 / VESC layouts)
    let mut config = StmConfig::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8 MHz / 4 * 168 / 2 = 168 MHz system
            divq: Some(PllQDiv::DIV7), // 8 MHz / 4 * 168 / 7 = 48 MHz for USB FS
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }

    let p = embassy_stm32::init(config);

    // Basic heartbeat LED (PC13 is convenient on many F4 boards)
    let led = Output::new(p.PC13, Level::High, Speed::Low);
    spawner.spawn(heartbeat(led).unwrap());

    // USB bulk transport for ergot
    let ep_out_buf = EP_OUT_BUF.init([0u8; kit::USB_FS_MAX_PACKET_SIZE]);
    let mut usb_cfg = embassy_stm32::usb::Config::default();
    // Simple FOCer 2 can stay powered without VBUS; disable detection for flexibility.
    usb_cfg.vbus_detection = false;
    let driver = usb::Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, ep_out_buf, usb_cfg);

    let mut usb_dev_cfg = embassy_usb::Config::new(0x16c0, 0x27DD);
    usb_dev_cfg.manufacturer = Some("oxifoc");
    usb_dev_cfg.product = Some("oxifoc-f405");
    usb_dev_cfg.serial_number = Some("simple-focer2");

    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);
    let rx_worker: RxWorker = kit::RxWorker::new(&STACK, ep_out);

    spawner.spawn(usb_task(usb_dev).unwrap());
    spawner.spawn(run_rx(rx_worker, RECV_BUF.init([0u8; MAX_PACKET_SIZE])).unwrap());
    spawner.spawn(run_tx(ep_in, OUTQ.framed_consumer()).unwrap());
    spawner.spawn(info_server().unwrap());

    // Split peripherals for motor/hall resources.
    let resources = split_resources!(p);
    let hall_resources = resources.hall;
    let _motor_resources = resources.motor;

    let hall1 = ExtiInput::new(hall_resources.pc6, p.EXTI6, Pull::Up);
    let hall2 = ExtiInput::new(hall_resources.pc7, p.EXTI7, Pull::Up);
    let hall3 = ExtiInput::new(hall_resources.pc8, p.EXTI8, Pull::Up);
    let inputs = HALL_INPUTS.init((hall1, hall2, hall3));
    unsafe {
        HALL_INPUTS_PTR = Some(inputs);
    }
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(oxifoc_core::foc::hall_sensor::HallSensor::new(
            TIMEBASE_TICKS_PER_SEC,
        )));
    });
    // Enable EXTI9_5 for PC6/7/8
    unsafe {
        embassy_stm32::interrupt::typelevel::EXTI9_5::unpend();
        embassy_stm32::interrupt::typelevel::EXTI9_5::enable();
    }
    spawner.spawn(foc_stub().unwrap());

    info!(
        "F405 pin map (planned): PWM PA8/PA9/PA10 + PB13/14/15, EN_GATE=PB5, nFAULT=PB7, SPI3 CS/SCK/MISO/MOSI=PC9/PC10/PC11/PC12, halls=PC6/7/8, ADC currents PC0-2, VBUS PC3"
    );
    info!(
        "Scaling: shunt={=f32}Ω, amp_gain={=f32} V/V, vbus_ratio={=f32}:1, faults=0x{=u32:08x}",
        scaling.shunt_ohms,
        scaling.current_amp_gain,
        scaling.vbus_divider_ratio,
        FAULTS.bits()
    );
}

#[task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

#[task]
async fn run_rx(rcvr: RxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, kit::USB_FS_MAX_PACKET_SIZE).await;
}

#[task]
async fn run_tx(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static Queue>,
) {
    kit::tx_worker::<AppDriver, OUT_QUEUE_SIZE, AtomicCoord>(
        &mut ep_in,
        rx,
        kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

/// Respond to info requests from host tooling.
#[task]
async fn info_server() {
    let server = STACK
        .endpoints()
        .bounded_server::<InfoEndpoint, 2>(Some("device_info"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_req: &()| async move {
                let mut hw: heapless::String<32> = heapless::String::new();
                let mut sw: heapless::String<32> = heapless::String::new();
                let _ = hw.push_str("Simple FOCer 2 (F405)");
                let _ = sw.push_str("oxifoc-f405@WIP");
                DeviceInfo { hw, sw }
            })
            .await;
    }
}

/// Blink a status LED so we know the scheduler is alive.
#[task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_low();
        Timer::after(Duration::from_millis(50)).await;
        led.set_high();
        Timer::after(Duration::from_millis(950)).await;
    }
}

/// Placeholder FOC loop to keep oxifoc-core integration in sync across targets.
///
/// For now we synthesize currents/angle; hardware crates will replace this
/// with real sensors and timers once the peripherals are wired up.
#[task]
async fn foc_stub() {
    let mut pwm = DummyPwm::new(1200);
    let mut foc: FocController = FocController::new(24.0);
    let mut angle = 0.0_f32;
    let mut loop_counter: u32 = 0;

    loop {
        let telemetry = foc.step((0.0, 0.0, 0.0), angle, 0.0, 0.0, pwm.max_duty(), 100e-6);
        pwm.set_duties(telemetry.duties);

        angle += 0.05;
        if angle > core::f32::consts::TAU {
            angle -= core::f32::consts::TAU;
        }

        loop_counter = loop_counter.wrapping_add(1);
        if loop_counter % 10000 == 0 {
            warn!(
                "FOC stub: duties={:?}, angle={=f32}",
                telemetry.duties, telemetry.angle_rad
            );
        }

        Timer::after(Duration::from_micros(100)).await;
    }
}

struct DummyPwm {
    duties: [u16; 3],
    max: u16,
}

impl DummyPwm {
    fn new(max: u16) -> Self {
        Self {
            duties: [max / 2; 3],
            max,
        }
    }
}

impl PhasePwm for DummyPwm {
    fn max_duty(&self) -> u16 {
        self.max
    }

    fn set_duties(&mut self, duties: [u16; 3]) {
        self.duties = duties;
    }
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

#[interrupt]
fn EXTI9_5() {
    let state = read_hall_state_fast();
    let ticks = embassy_time::Instant::now().as_ticks() as u32;
    HALL_ESTIMATOR.lock(|est| {
        if let Some(h) = est.borrow_mut().as_mut()
            && let Ok(reading) = h.update_sample(state, ticks as u64)
        {
            HALL_ANGLE_BITS.store(reading.angle_rad.to_bits(), Ordering::Relaxed);
            let dir_u8 = match reading.direction {
                Direction::Stopped => 0,
                Direction::Clockwise => 1,
                Direction::CounterClockwise => 2,
            };
            HALL_DIRECTION.store(dir_u8, Ordering::Relaxed);
            HALL_STATE.store(reading.state, Ordering::Relaxed);
        }
    });
    interrupt::typelevel::EXTI9_5::unpend();
}

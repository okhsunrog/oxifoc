#![no_std]
#![no_main]

use core::pin::pin;

use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_stm32::{
    Config as StmConfig, bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    time::Hertz,
    usb,
};
use embassy_time::{Duration, Timer};
use ergot::{
    exports::bbq2::prod_cons::framed::FramedConsumer,
    exports::bbq2::traits::coordination::cas::AtomicCoord, toolkits::embassy_usb_v0_5 as kit,
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use oxifoc_core::foc::{controller::FocController, pwm::PhasePwm};
use oxifoc_protocol::{DeviceInfo, InfoEndpoint};
use static_cell::StaticCell;

// Enable defmt global logger + panic probe
use panic_probe as _;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;
type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;
type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, AppDriver>;

const OUT_QUEUE_SIZE: usize = 4096;
const MAX_PACKET_SIZE: usize = 512;

static OUTQ: Queue = kit::Queue::new();
static STACK: Stack = kit::new_target_stack(OUTQ.framed_producer(), MAX_PACKET_SIZE as u16);
static USB_STORAGE: kit::WireStorage<256, 256, 64, 256> = kit::WireStorage::new();
static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
static EP_OUT_BUF: StaticCell<[u8; kit::USB_FS_MAX_PACKET_SIZE]> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("oxifoc-f405 boot");

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
    spawner.spawn(foc_stub().unwrap());
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
    let mut foc = FocController::new(24.0);
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

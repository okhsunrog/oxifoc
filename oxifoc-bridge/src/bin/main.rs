#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use ergot::{
    NetStack,
    interface_manager::utils::{cobs_stream, framed_stream},
};
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    timer::timg::TimerGroup,
    usb::usb_serial_jtag::UsbSerialJtag,
};
use oxifoc_bridge::transport::{self, BridgeSink, MTU};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    // Waveshare C5-Zero onboard antenna, selected before any radio initialization.
    let _antenna = Output::new(p.GPIO26, Level::Low, OutputConfig::default());
    let timer = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timer.timer0, p.FROM_CPU_INTR0);
    let rng = esp_hal::rng::Rng::new();
    let epoch = rng.random();
    let upstream = BridgeSink::Spi(framed_stream::Sink::new(
        transport::SPI_OUTQ.framed_producer(),
        MTU,
    ));
    let router = ergot::interface_manager::profiles::router::Router::new_bridge(rng, upstream);
    static STACK: static_cell::StaticCell<transport::Stack> = static_cell::StaticCell::new();
    let stack = STACK.init(NetStack::new_with_profile(router));
    let usb_ident = stack
        .manage_profile(|p| {
            p.register_interface_pending(BridgeSink::Usb(cobs_stream::Sink::new(
                transport::USB_OUTQ.stream_producer(),
                MTU,
            )))
        })
        .unwrap();
    spawner.spawn(
        transport::spi::pump(
            stack,
            transport::spi::Resources {
                bus: p.SPI2,
                sck: p.GPIO6,
                mosi: p.GPIO7,
                miso: p.GPIO9,
                cs: p.GPIO10,
                ready: p.GPIO0,
                dma: p.DMA_CH0,
            },
            epoch,
        )
        .unwrap(),
    );
    let (rx, tx) = UsbSerialJtag::new(p.USB_DEVICE).into_async().split();
    spawner.spawn(transport::usb::rx(stack, rx, usb_ident).unwrap());
    spawner.spawn(transport::usb::tx(tx).unwrap());
    spawner.spawn(transport::usb::routes(stack, usb_ident).unwrap());
    #[cfg(feature = "ble")]
    {
        esp_alloc::heap_allocator!(size: 65536);
        let ble_ident = stack
            .manage_profile(|p| {
                p.register_interface_pending(BridgeSink::Ble(framed_stream::Sink::new(
                    transport::BLE_OUTQ.framed_producer(),
                    transport::BLE_MTU,
                )))
            })
            .unwrap();
        spawner.spawn(oxifoc_bridge::ble_runtime::run(stack, p.BT, ble_ident).unwrap());
    }
    defmt::info!("OxiFOC C5: SPI DMA upstream, USB Serial/JTAG downstream");
    loop {
        Timer::after_secs(60).await;
    }
}

#![no_std]
#![no_main]

use embassy_executor::{SpawnToken, Spawner};
use embassy_time::Timer;
use ergot::{
    NetStack,
    interface_manager::utils::{cobs_stream, framed_stream},
};
use esp_hal::{
    clock::CpuClock, gpio::Pin, timer::timg::TimerGroup, usb::usb_serial_jtag::UsbSerialJtag,
};
use oxifoc_bridge::{
    bridge_logs,
    transport::{self, BridgeSink, MTU},
};

/// Record the panic, log it over RTT, then reset. esp_hal::init disables the
/// watchdogs, so a panic that only looped would leave BLE dead until a power
/// cycle. The Ergot log queue cannot drain here: the record survives the
/// reset for `req/bridge_diag`, and the message goes to RTT directly.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    oxifoc_bridge::crash::record_panic(info);
    rtt_target::rprintln!("PANIC: {}", info);
    esp_hal::system::software_reset()
}

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    bridge_logs::init();
    let p = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    oxifoc_bridge::diag::RESET_REASON.store(
        esp_hal::system::reset_reason().map_or(0, |reason| reason as u8),
        core::sync::atomic::Ordering::Relaxed,
    );
    oxifoc_bridge::crash::count_boot();
    // The board is a Seeed XIAO ESP32-C5. These are its pins as wired to
    // the controller (docs/hw/xiao-esp32c5.md): the SPI link, and SWCLK and
    // SWDIO of the wireless probe. Any pin serves any of them but the
    // strapping pins, which the chip reads at reset: D2, D3, GPIO2, GPIO3.
    let link = transport::spi::Resources {
        bus: p.SPI2,
        sck: p.GPIO10.degrade(),  // D10
        mosi: p.GPIO8.degrade(),  // D8
        miso: p.GPIO12.degrade(), // D7
        cs: p.GPIO9.degrade(),    // D9
        ready: p.GPIO0.degrade(), // D1
        dma: p.DMA_CH0,
    };
    #[cfg(feature = "swd-probe")]
    let (swclk, swdio) = (p.GPIO4.degrade(), p.GPIO5.degrade()); // the MTCK and MTDO pads
    // The PSRAM in the chip's package holds recordings of fast telemetry.
    let recording_memory: &'static mut [u8] = {
        let psram = esp_hal::psram::Psram::new(p.PSRAM, Default::default());
        match psram.raw_parts() {
            (_, 0) => &mut [],
            // SAFETY: the driver mapped this many bytes at this address for
            // the life of the firmware, and nothing else is given them.
            (start, size) => unsafe { core::slice::from_raw_parts_mut(start, size) },
        }
    };
    let timer = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timer.timer0);
    let rng = esp_hal::rng::Rng::new();
    let epoch = rng.random();
    let spi_sink = BridgeSink::Spi(framed_stream::Sink::new(
        transport::SPI_OUTQ.framed_producer(),
        MTU,
    ));
    let router = ergot::interface_manager::profiles::router::Router::new(rng);
    static STACK: static_cell::StaticCell<transport::Stack> = static_cell::StaticCell::new();
    let stack = STACK.init(NetStack::new_with_profile(router));
    spawner.spawn(bridge_logs::forward(stack).expect("bridge log task is spawned once"));
    let spi_ident = stack
        .manage_profile(|p| p.register_interface(spi_sink))
        .expect("SPI interface slot must be available at startup");
    let usb_ident = stack
        .manage_profile(|p| {
            p.register_interface(BridgeSink::Usb(transport::Counted::new(
                cobs_stream::Sink::new(transport::USB_OUTQ.stream_producer(), MTU),
                &transport::usb::USB_DROPPED,
            )))
        })
        .expect("USB interface slot must be available at startup");
    // The slot handshake preempts the thread executor and the radio tasks.
    static LINK_EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::InterruptExecutor<2>> =
        static_cell::StaticCell::new();
    let link_executor =
        LINK_EXECUTOR.init(esp_rtos::embassy::InterruptExecutor::new(p.FROM_CPU_INTR2));
    link_executor
        .start(esp_hal::interrupt::Priority::Priority1)
        .spawn(transport::spi::link(link, epoch).expect("SPI link task is spawned once"));
    spawn_bridge_task(
        spawner,
        transport::spi::slots(stack, spi_ident, epoch).expect("SPI slot task is spawned once"),
        LINK,
    );
    // GPIO14 and GPIO13 are the C5's USB D+ and D-; the driver checks them.
    let (rx, tx) = UsbSerialJtag::new(p.USB_DEVICE, p.GPIO14, p.GPIO13, Default::default())
        .expect("USB Serial/JTAG on its own pins")
        .into_async()
        .split();
    spawn_bridge_task(
        spawner,
        transport::usb::rx(stack, rx, usb_ident).expect("USB RX task is spawned once"),
        FORWARDING,
    );
    // Beside the relay that fills its queue, as the task that fills the
    // TCP socket is.
    spawn_bridge_task(
        spawner,
        transport::usb::tx(tx).expect("USB TX task is spawned once"),
        LINK,
    );
    spawn_bridge_task(
        spawner,
        transport::root_services(stack).expect("USB route task is spawned once"),
        FORWARDING,
    );
    // Beside the slot task, which waits for it to empty its queue. Below
    // it the relay took turns with the Wi-Fi stack, and a 20 kHz stream over
    // Wi-Fi lost a third (with the flash at 40 MHz).
    spawn_bridge_task(
        spawner,
        oxifoc_bridge::relay::run(stack, spi_ident, recording_memory)
            .expect("relay task is spawned once"),
        LINK,
    );
    #[cfg(any(feature = "ble", feature = "wifi"))]
    {
        // The radio heap lives in the RAM the 2nd-stage bootloader frees
        // (dram2_seg, 64 KiB), so it does not shrink the main stack. LESC
        // pairing computes P-256 in software on that stack; with the heap in
        // .bss the stack was ~16 KiB and pairing overflowed it into the BLE
        // controller's state.
        esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    }
    #[cfg(any(feature = "ble", feature = "ota"))]
    let flash = oxifoc_bridge::flash::Flash::new(p.FLASH);
    #[cfg(feature = "ble")]
    {
        let ble_idents = core::array::from_fn(|link| {
            stack
                .manage_profile(|p| {
                    p.register_interface(BridgeSink::Ble(transport::Counted::new(
                        framed_stream::Sink::new(
                            transport::BLE_OUTQ[link].framed_producer(),
                            transport::BLE_MTU,
                        ),
                        &oxifoc_bridge::diag::BLE[link].dropped,
                    )))
                })
                .expect("BLE interface slots must be available at startup")
        });
        // The remote's commands and their replies pass through this task.
        // Below the relay it waited for it: during a capture some 300 of 780
        // status replies came over 150 ms apart (with the flash at 40 MHz).
        spawn_bridge_task(
            spawner,
            oxifoc_bridge::ble_runtime::run(stack, p.BT, ble_idents, flash)
                .expect("BLE task is spawned once"),
            LINK,
        );
    }
    #[cfg(feature = "wifi")]
    let network = oxifoc_bridge::wifi::start(spawner, p.WIFI);
    #[cfg(feature = "wifi")]
    if let Some(network) = network {
        transport::tcp::start(spawner, stack, network);
    }
    #[cfg(feature = "swd-probe")]
    oxifoc_bridge::probe::start(spawner, stack, swclk, swdio, network);
    #[cfg(feature = "ota")]
    oxifoc_bridge::ota::start(spawner, stack, flash);
    log::info!("OxiFOC C5: root router: SPI DMA controller, USB Serial/JTAG host");
    loop {
        Timer::after_secs(60).await;
    }
}

use transport::{FORWARDING, LINK};

fn spawn_bridge_task<S>(spawner: Spawner, task: SpawnToken<S>, priority: u8) {
    #[cfg(feature = "wifi")]
    task.metadata().set_priority(priority);
    #[cfg(not(feature = "wifi"))]
    let _ = priority;
    spawner.spawn(task);
}

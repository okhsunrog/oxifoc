//! USB transport layer for ergot communication
//!
//! USB bulk endpoints carry ergot protocol data.
//! defmt logs go over a separate RTT channel (read via probe-rs).

use embassy_stm32::{Peri, bind_interrupts, peripherals, usb};
use ergot::{
    exports::bbqueue::traits::coordination::cas::AtomicCoord, toolkits::embassy_usb_v0_6 as kit,
};
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use rtt_target::{ChannelMode::*, rtt_init};
use static_cell::StaticCell;

use crate::config::OUT_QUEUE_SIZE;

/// RTT defmt channel storage
static RTT_DEFMT_UP: StaticCell<rtt_target::UpChannel> = StaticCell::new();

// Type aliases for USB-based ergot transport
pub type AppDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;
pub type Queue = kit::Queue<OUT_QUEUE_SIZE, AtomicCoord>;
pub type Stack = kit::Stack<&'static Queue, CriticalSectionRawMutex>;
pub type RxWorker = kit::RxWorker<&'static Queue, CriticalSectionRawMutex, AppDriver>;

// Static storage for USB transport
static USB_STORAGE: kit::WireStorage<256, 256, 64, 256> = kit::WireStorage::new();
// OTG FS endpoint buffer — must be large enough for all EP allocations (control + bulk)
static EP_OUT_BUF: StaticCell<[u8; 256]> = StaticCell::new();

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

/// Initialize RTT for defmt logging. Must be called before any defmt macros.
pub fn init_defmt_rtt() {
    use ergot::logging::defmt_sink;
    let channels = rtt_init! {
        up: {
            0: { size: 1024, mode: NoBlockSkip, name: "defmt" }
        }
    };
    let defmt_up = RTT_DEFMT_UP.init(channels.up.0);
    defmt_sink::init_rtt(defmt_up);
}

/// USB transport configuration and handles
pub struct UsbTransport {
    pub usb_dev: embassy_usb::UsbDevice<'static, AppDriver>,
    pub ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    pub rx_worker: RxWorker,
}

/// Initialize USB transport for ergot communication
pub fn init_usb(
    stack: &'static Stack,
    usb_otg_fs: Peri<'static, peripherals::USB_OTG_FS>,
    pa12: Peri<'static, peripherals::PA12>,
    pa11: Peri<'static, peripherals::PA11>,
) -> UsbTransport {
    let ep_out_buf = EP_OUT_BUF.init([0u8; 256]);

    let mut usb_cfg = embassy_stm32::usb::Config::default();
    // Simple FOCer 2 can stay powered without VBUS; disable detection for flexibility.
    usb_cfg.vbus_detection = false;

    let driver = usb::Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, ep_out_buf, usb_cfg);

    let mut usb_dev_cfg = embassy_usb::Config::new(0x16c0, 0x27DD);
    usb_dev_cfg.manufacturer = Some("oxifoc");
    usb_dev_cfg.product = Some("oxifoc-f405");
    usb_dev_cfg.serial_number = Some("simple-focer2");

    let (usb_dev, ep_in, ep_out) = USB_STORAGE.init_ergot(driver, usb_dev_cfg);
    let rx_worker = kit::RxWorker::new(stack, ep_out);

    UsbTransport {
        usb_dev,
        ep_in,
        rx_worker,
    }
}

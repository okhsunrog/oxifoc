use defmt::info;
use embassy_nrf::pac::POWER;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

#[derive(Clone, Copy)]
pub struct Reading {
    pub millivolts: u32,
    pub usb: bool,
    pub percent: Option<u32>,
}

pub static READING: Signal<CriticalSectionRawMutex, Reading> = Signal::new();

pub fn publish(millivolts: u32) {
    let usb = POWER.usbregstatus().read().vbusdetect();
    // USB can power VDDH even with no cell connected.
    let percent =
        (!usb && (2800..=4250).contains(&millivolts)).then(|| estimate_percent(millivolts));
    info!(
        "[battery] VDDH={}mV USB={} charge={:?}",
        millivolts, usb, percent
    );
    READING.signal(Reading {
        millivolts,
        usb,
        percent,
    });
}

fn estimate_percent(millivolts: u32) -> u32 {
    // Approximate resting voltage curve for a 1S cell charged to 4.20 V.
    let mut previous = (3300, 0);
    let millivolts = millivolts.max(previous.0);
    for next in [
        (3600, 10),
        (3700, 25),
        (3800, 50),
        (3900, 75),
        (4050, 90),
        (4200, 100),
    ] {
        if millivolts <= next.0 {
            return previous.1
                + (millivolts - previous.0) * (next.1 - previous.1) / (next.0 - previous.0);
        }
        previous = next;
    }
    100
}

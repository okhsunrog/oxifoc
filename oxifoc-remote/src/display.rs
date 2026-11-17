use core::fmt::Write;

use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_nrf::{gpio::Output, twim::Twim};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use heapless::String;
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};

use super::battery;

pub static BRIDGE_READY: Signal<CriticalSectionRawMutex, bool> = Signal::new();

#[embassy_executor::task]
#[allow(
    clippy::large_stack_frames,
    reason = "the SSD1306 framebuffer lives in static Embassy task storage"
)]
pub async fn run(i2c: Twim<'static>, _power: Output<'static>) {
    Timer::after_millis(100).await;
    let mut display = Ssd1306Async::new(
        I2CDisplayInterface::new(i2c),
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    let mut bridge_ready = false;
    let mut battery_reading = None;
    loop {
        if display.init().await.is_err() {
            warn!("OLED initialization failed");
            Timer::after_secs(1).await;
            continue;
        }

        loop {
            bridge_ready = BRIDGE_READY.try_take().unwrap_or(bridge_ready);
            battery_reading = battery::READING.try_take().or(battery_reading);
            display.clear_buffer();
            let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
            let status = if bridge_ready {
                "Bridge connected"
            } else {
                "Looking for bridge"
            };
            let _ = Text::new("OxiFOC Remote", Point::new(4, 16), style).draw(&mut display);
            let _ = Text::new(status, Point::new(4, 36), style).draw(&mut display);
            let mut battery_line = String::<20>::new();
            if let Some(reading) = battery_reading {
                let centivolts = (reading.millivolts + 5) / 10;
                let volts = centivolts / 100;
                let fraction = centivolts % 100;
                match reading.percent {
                    Some(percent) => {
                        let _ = write!(battery_line, "Bat ~{percent}% {volts}.{fraction:02}V");
                    }
                    None if reading.usb => {
                        let _ = write!(battery_line, "USB VDDH {volts}.{fraction:02}V");
                    }
                    None => {
                        let _ = write!(battery_line, "VDDH {volts}.{fraction:02}V");
                    }
                }
            }
            let _ = Text::new(battery_line.as_str(), Point::new(4, 56), style).draw(&mut display);
            if display.flush().await.is_err() {
                warn!("OLED frame transfer failed");
                Timer::after_secs(1).await;
                break;
            }
            info!("OLED updated: {} {}", status, battery_line.as_str());
            match select(BRIDGE_READY.wait(), battery::READING.wait()).await {
                Either::First(ready) => bridge_ready = ready,
                Either::Second(reading) => battery_reading = Some(reading),
            }
        }
    }
}

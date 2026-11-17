use defmt::{info, warn};
use embassy_nrf::{gpio::Output, twim::Twim};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};

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
    loop {
        if display.init().await.is_err() {
            warn!("OLED initialization failed");
            Timer::after_secs(1).await;
            continue;
        }

        loop {
            bridge_ready = BRIDGE_READY.try_take().unwrap_or(bridge_ready);
            display.clear_buffer();
            let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
            let status = if bridge_ready {
                "Bridge connected"
            } else {
                "Looking for bridge"
            };
            let _ = Text::new("OxiFOC Remote", Point::new(4, 16), style).draw(&mut display);
            let _ = Text::new(status, Point::new(4, 36), style).draw(&mut display);
            if display.flush().await.is_err() {
                warn!("OLED frame transfer failed");
                Timer::after_secs(1).await;
                break;
            }
            info!("OLED updated: {}", status);
            bridge_ready = BRIDGE_READY.wait().await;
        }
    }
}

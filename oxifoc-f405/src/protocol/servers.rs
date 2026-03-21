//! Ergot protocol servers and USB I/O worker tasks

use embassy_executor::Spawner;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_6 as kit,
};
use heapless::String;

use crate::protocol::STACK;
use crate::transport::{AppDriver, RxWorker};
use crate::{FAULT_REGISTRY, STATE};
use oxifoc_core::types::DeviceInfo;

// ========== Worker Tasks ==========

/// USB device task - runs USB state machine
#[embassy_executor::task]
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
#[embassy_executor::task]
pub async fn run_rx(rcvr: RxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data (USB)
#[embassy_executor::task]
pub async fn run_tx(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static crate::transport::Queue>,
) {
    kit::tx_worker::<AppDriver, { crate::config::OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
///
/// Uses join to run info, hall, adc, and motor servers together.
/// This is more RAM-efficient than separate tasks.
#[embassy_executor::task]
pub async fn protocol_servers() {
    defmt::info!("Starting protocol servers");

    // Build device info
    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("Simple FOCer 2 (F405)");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32F405RG");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = DeviceInfo {
        hw, sw, mcu, uuid,
        foc_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz,
        max_current_a: crate::config::BOARD.max_phase_current_a,
    };

    oxifoc_core::runtime::run_all_servers(
        STACK.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(protocol_servers().unwrap());
}

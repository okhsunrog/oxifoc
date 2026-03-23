//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use heapless::String;
use oxifoc_core::types::DeviceInfo;

use crate::protocol::STACK;
#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
use crate::protocol::OUTQ;
use crate::transport::RxWorker;
use crate::{FAULT_REGISTRY, STATE};

#[cfg(feature = "transport-uart")]
use {
    crate::transport::io::UartWriter,
    ergot::toolkits::embedded_io_async_v0_7::tx_worker,
};
#[cfg(feature = "transport-rtt")]
use {
    ergot::toolkits::embedded_io_async_v0_7::tx_worker,
    ergot::transport::rtt::RttWriter,
};
#[cfg(feature = "transport-usb")]
use {
    crate::transport::AppDriver,
    ergot::{
        exports::bbqueue::prod_cons::framed::FramedConsumer,
        toolkits::embassy_usb_v0_6 as usb_kit,
    },
};

// ========== Worker Tasks ==========

/// Worker task for incoming ergot data (UART / RTT)
#[cfg(any(feature = "transport-uart", feature = "transport-rtt"))]
#[embassy_executor::task]
pub async fn run_rx(
    mut rcvr: RxWorker,
    recv_buf: &'static mut [u8],
    scratch_buf: &'static mut [u8],
) {
    loop {
        let _ = rcvr.run(recv_buf, scratch_buf).await;
    }
}

/// Worker task for incoming ergot data (USB)
#[cfg(feature = "transport-usb")]
#[embassy_executor::task]
pub async fn run_rx(rcvr: RxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data via UART/LPUART
#[cfg(feature = "transport-uart")]
#[embassy_executor::task]
pub async fn run_tx_uart(mut tx: UartWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// Worker task for outgoing ergot data via RTT
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
pub async fn run_tx_rtt(mut tx: RttWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// USB device task — runs the USB state machine
#[cfg(feature = "transport-usb")]
#[embassy_executor::task]
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for outgoing ergot data via USB bulk endpoint
#[cfg(feature = "transport-usb")]
#[embassy_executor::task]
pub async fn run_tx_usb(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static crate::transport::Queue>,
) {
    usb_kit::tx_worker::<AppDriver, { crate::config::OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        usb_kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        usb_kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
#[embassy_executor::task]
pub async fn protocol_servers() {
    defmt::info!("Starting protocol servers");

    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("NUCLEO-G474RE");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32G474RE");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = DeviceInfo {
        hw,
        sw,
        mcu,
        uuid,
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

pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(protocol_servers().unwrap());
}

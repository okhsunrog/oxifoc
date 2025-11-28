//! Ergot protocol servers and USB I/O worker tasks

use core::pin::pin;

use embassy_executor::Spawner;
use ergot::{exports::bbq2::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_5 as kit};
use oxifoc_protocol::{DeviceInfo, InfoEndpoint};

use crate::protocol::STACK;
use crate::transport::{AppDriver, RxWorker};

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

/// Respond to info requests from host
#[embassy_executor::task]
pub async fn info_server() {
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

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(info_server().unwrap());
}

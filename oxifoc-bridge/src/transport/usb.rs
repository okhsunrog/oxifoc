//! USB Serial/JTAG COBS downstream; USB throughput is independent of baud.
use super::{STATE_NOTIFY, Stack, USB_OUTQ};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::Timer;
use embedded_io_async::Write;
use ergot::interface_manager::{
    InterfaceState, LivenessConfig, profiles::router::RouterFrameProcessor,
};
use esp_hal::{
    Async,
    usb::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx},
};

#[unsafe(no_mangle)]
pub static USB_DROPPED: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static USB_MAX_GRANT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static USB_WRITE_ERRORS: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn rx(stack: &'static Stack, rx: UsbSerialJtagRx<'static, Async>, ident: u8) {
    // run(Inactive) changes port state before the first frame arrives, so
    // obtain the root-assigned segment from registration, not that state.
    let net = stack
        .manage_profile(|p| p.net_id_of(ident))
        .expect("registered USB segment");
    let mut rx = ergot::interface_manager::transports::eio::RxWorker::new(
        stack,
        rx,
        RouterFrameProcessor::new(net),
        ident,
    )
    .with_liveness(LivenessConfig { timeout_ms: 5000 })
    .with_state_notify(&STATE_NOTIFY);
    let mut buffer = [0; 4096];
    let mut scratch = [0; 64];
    loop {
        let _ = rx
            .run(InterfaceState::Inactive, &mut buffer, &mut scratch)
            .await;
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
pub async fn tx(mut tx: UsbSerialJtagTx<'static, Async>) {
    let consumer = USB_OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        USB_MAX_GRANT.fetch_max(grant.len() as u32, Ordering::Relaxed);
        // Release completed prefixes promptly. Holding a large contiguous
        // grant across a slow USB write can prevent the producer wrapping,
        // even when much of that grant has already left the device.
        let len = grant.len().min(512);
        let last_chunk = len == grant.len();
        let failed = tx.write_all(&grant[..len]).await.is_err()
            || (last_chunk && len.is_multiple_of(64) && tx.flush().await.is_err());
        if failed {
            USB_WRITE_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        grant.release(len);
        embassy_futures::yield_now().await;
    }
}

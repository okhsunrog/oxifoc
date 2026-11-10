//! USB Serial/JTAG COBS downstream; USB throughput is independent of baud.
use super::{STATE_NOTIFY, Stack, USB_OUTQ};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write;
use ergot::{
    interface_manager::{
        InterfaceState, LivenessConfig, Profile,
        profiles::{
            direct_edge::CENTRAL_NODE_ID,
            router::{RouterFrameProcessor, UPSTREAM_IDENT},
        },
    },
    net_stack::services::{bridge_seed_assign, bridge_seed_refresh},
    well_known::ErgotPingEndpoint,
};
use esp_hal::{
    Async,
    usb::usb_serial_jtag::{UsbSerialJtagRx, UsbSerialJtagTx},
};

#[unsafe(no_mangle)]
pub static USB_ADMISSION_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static USB_MAX_GRANT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static USB_WRITE_ERRORS: AtomicU32 = AtomicU32::new(0);

pub struct UsbSink(ergot::interface_manager::utils::cobs_stream::Sink<&'static super::UsbQueue>);
impl UsbSink {
    pub fn new(
        sink: ergot::interface_manager::utils::cobs_stream::Sink<&'static super::UsbQueue>,
    ) -> Self {
        Self(sink)
    }
    fn counted(result: Result<(), ()>) -> Result<(), ()> {
        if result.is_err() {
            USB_ADMISSION_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        result
    }
}
impl ergot::interface_manager::InterfaceSink for UsbSink {
    fn mtu(&self) -> u16 {
        self.0.mtu()
    }
    fn send_ty<T: serde::Serialize>(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        body: &T,
    ) -> Result<(), ()> {
        Self::counted(self.0.send_ty(link, hdr, body))
    }
    fn send_raw(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        body: &[u8],
    ) -> Result<(), ()> {
        Self::counted(self.0.send_raw(link, hdr, body))
    }
    fn send_err(
        &mut self,
        link: &ergot::interface_manager::LinkMeta,
        hdr: &ergot::Header,
        error: ergot::ProtocolError,
    ) -> Result<(), ()> {
        self.0.send_err(link, hdr, error)
    }
}

#[embassy_executor::task]
pub async fn rx(stack: &'static Stack, rx: UsbSerialJtagRx<'static, Async>, ident: u8) {
    let mut rx = ergot::interface_manager::transports::eio::RxWorker::new(
        stack,
        rx,
        RouterFrameProcessor::new(0),
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

#[embassy_executor::task]
pub async fn routes(stack: &'static Stack, usb_ident: u8) {
    #[cfg(feature = "bench-delayed-upstream")]
    Timer::after_secs(3).await;
    let mut generation = super::spi::PEER_GENERATION.load(Ordering::Acquire);
    let mut lease = None;
    let mut refresh_at = embassy_time::Instant::now();
    let mut seq = 0u32;
    loop {
        seq = seq.wrapping_add(1);
        let _ = with_timeout(
            Duration::from_millis(750),
            stack.endpoints().request::<ErgotPingEndpoint>(
                ergot::Address {
                    network_id: 0,
                    node_id: CENTRAL_NODE_ID,
                    port_id: 0,
                },
                &seq,
                None,
            ),
        )
        .await;
        let current_generation = super::spi::PEER_GENERATION.load(Ordering::Acquire);
        if generation != current_generation {
            generation = current_generation;
            lease = None;
            refresh_at = embassy_time::Instant::now();
        }
        let discovered = stack.manage_profile(|p| matches!(p.interface_state(UPSTREAM_IDENT), Some(InterfaceState::Active {net_id, ..}) if net_id != 0));
        if discovered && embassy_time::Instant::now() >= refresh_at {
            let refreshed = if let Some(ref current) = lease {
                with_timeout(Duration::from_secs(2), bridge_seed_refresh(&stack, current))
                    .await
                    .ok()
                    .and_then(Result::ok)
            } else {
                None
            };
            lease = match refreshed {
                Some(l) => Some(l),
                None => with_timeout(
                    Duration::from_secs(2),
                    bridge_seed_assign(&stack, UPSTREAM_IDENT, usb_ident),
                )
                .await
                .ok()
                .and_then(Result::ok),
            };
            if let Some(ref current) = lease {
                let seconds = current
                    .expires_seconds
                    .saturating_sub(current.min_refresh_seconds)
                    .saturating_add(2)
                    .max(1);
                refresh_at = embassy_time::Instant::now() + Duration::from_secs(u64::from(seconds));
                defmt::info!("USB downstream leased net={}", current.net_id);
            }
        }
        if !discovered {
            lease = None;
            refresh_at = embassy_time::Instant::now();
        }
        Timer::after_millis(250).await;
    }
}

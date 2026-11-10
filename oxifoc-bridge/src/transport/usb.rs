//! USB Serial/JTAG COBS downstream; USB throughput is independent of baud.
use super::{STATE_NOTIFY, Stack, USB_OUTQ};
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
        let len = grant.len();
        if tx.write_all(&grant).await.is_ok() && len.is_multiple_of(64) {
            let _ = tx.flush().await;
        }
        grant.release(len);
        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::task]
pub async fn routes(stack: &'static Stack, usb_ident: u8) {
    #[cfg(feature = "bench-delayed-upstream")]
    Timer::after_secs(3).await;
    let mut generation = super::spi::PEER_GENERATION.load(core::sync::atomic::Ordering::Acquire);
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
        let current_generation =
            super::spi::PEER_GENERATION.load(core::sync::atomic::Ordering::Acquire);
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

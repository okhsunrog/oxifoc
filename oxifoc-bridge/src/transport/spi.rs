//! Demand-driven SPI slave: CS requests a slot, READY confirms armed DMA.
use super::{SPI_OUTQ, STATE_NOTIFY, Stack};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_futures::select::select;
use embassy_time::Timer;
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::router::RouterFrameProcessor,
};
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    dma_rx_buffer, dma_tx_buffer,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    peripherals::*,
    spi::{Mode, slave::Spi},
};
use oxifoc_core::spi_link::{Builder, SLOT_BYTES, parse};

pub struct Resources {
    pub bus: SPI2<'static>,
    pub sck: GPIO6<'static>,
    pub mosi: GPIO7<'static>,
    pub miso: GPIO9<'static>,
    pub cs: GPIO10<'static>,
    pub ready: GPIO0<'static>,
    pub dma: DMA_CH0<'static>,
}
#[unsafe(no_mangle)]
pub static SPI_LINK_SLOTS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_LINK_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_RX_LENGTH_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_RX_CRC_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_RX_HEADER_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_LAST_RX_LEN: AtomicU32 = AtomicU32::new(0);
fn checksum(bytes: &[u8]) -> u32 {
    !esp_hal::rom::crc::crc32_be(0, bytes)
}

#[embassy_executor::task]
pub async fn pump(stack: &'static Stack, ident: u8, r: Resources, epoch: u32) {
    defmt::assert_eq!(checksum(b"123456789"), 0x0376_e6e7);
    let mut ready = Output::new(r.ready, Level::Low, OutputConfig::default());
    let mut cs = Input::new(r.cs, InputConfig::default());
    let mut spi = Spi::new(r.bus, Mode::_0)
        .with_sck(r.sck)
        .with_mosi(r.mosi)
        .with_miso(r.miso)
        .with_cs(cs.peripheral_input())
        .with_dma(r.dma);
    let mut rx = defmt::unwrap!(dma_rx_buffer!(SLOT_BYTES));
    let mut tx = defmt::unwrap!(dma_tx_buffer!(SLOT_BYTES));
    let mut spare_rx = defmt::unwrap!(dma_rx_buffer!(SLOT_BYTES));
    let mut spare_tx = defmt::unwrap!(dma_tx_buffer!(SLOT_BYTES));
    let consumer = SPI_OUTQ.framed_consumer();
    let prepare_tx = |tx: &mut DmaTxBuf, seq| {
        tx.set_length(SLOT_BYTES);
        let mut builder =
            defmt::unwrap!(Builder::new(tx.as_mut_slice(), epoch, seq).map_err(|_| ()));
        while let Ok(grant) = consumer.read() {
            if builder.append(&grant).is_err() {
                break;
            }
            grant.release();
        }
        builder.finish(checksum);
    };
    #[cfg(feature = "bench-delayed-upstream")]
    Timer::after_secs(3).await;
    let net = defmt::unwrap!(stack.manage_profile(|p| p.net_id_of(ident)));
    let mut processor = RouterFrameProcessor::new(net);
    let mut pending = false;
    let mut seq = 0u32;
    let mut peer_epoch = None;
    loop {
        // Idle processing can generate replies. Do it before waiting for work,
        // otherwise a request which needs a reply could deadlock the transport.
        if pending && cs.is_high() && consumer.read().is_err() {
            process_slot(stack, ident, &mut processor, &mut peer_epoch, &mut spare_rx);
            pending = false;
        }
        // Dropping a read grant keeps it queued. The losing wait is cancel-safe;
        // no DMA exists yet, and both conditions remain asserted until serviced.
        if cs.is_high() && consumer.read().is_err() {
            let _ = select(cs.wait_for_low(), consumer.wait_read()).await;
        }
        rx.set_length(SLOT_BYTES);
        prepare_tx(&mut tx, seq);
        let Ok(transfer) = spi.transfer(SLOT_BYTES, rx, SLOT_BYTES, tx) else {
            defmt::panic!("SPI DMA arm failed");
        };
        ready.set_high();
        // Independent buffers allow routing the previous slot while this DMA
        // runs. Do not touch buffers owned by the transfer.
        if pending {
            process_slot(stack, ident, &mut processor, &mut peer_epoch, &mut spare_rx);
        }
        // Never cancel/drop an armed slave transfer: the master may reboot.
        while !transfer.is_done() {
            Timer::after_micros(100).await;
        }
        (spi, (rx, tx)) = transfer.wait();
        {
            use core::{
                future::{Future, poll_fn},
                pin::pin,
                task::Poll,
            };
            // Symmetric to the master's READY acknowledgement: latch CS release
            // before lowering READY. A fast master may release and assert CS
            // again before this task is scheduled, so a plain level wait loses
            // that boundary and mistakes the next request for the old transfer.
            let mut released = pin!(cs.wait_for_high());
            let already_high =
                poll_fn(|cx| Poll::Ready(released.as_mut().poll(cx).is_ready())).await;
            ready.set_low();
            if !already_high {
                released.await;
            }
        }
        seq = seq.wrapping_add(1);
        SPI_LINK_SLOTS.fetch_add(1, Ordering::Relaxed);
        core::mem::swap(&mut rx, &mut spare_rx);
        core::mem::swap(&mut tx, &mut spare_tx);
        pending = true;
        embassy_futures::yield_now().await;
    }
}

fn process_slot(
    stack: &'static Stack,
    ident: u8,
    processor: &mut RouterFrameProcessor,
    peer_epoch: &mut Option<u32>,
    rx: &mut DmaRxBuf,
) {
    if rx.number_of_received_bytes() != SLOT_BYTES {
        SPI_RX_LENGTH_ERRORS.fetch_add(1, Ordering::Relaxed);
        SPI_LAST_RX_LEN.store(rx.number_of_received_bytes() as u32, Ordering::Relaxed);
        SPI_LINK_ERRORS.fetch_add(1, Ordering::Relaxed);
        return;
    }
    match parse(&mut rx.as_mut_slice()[..SLOT_BYTES], checksum) {
        Ok(slot) => {
            if *peer_epoch != Some(slot.epoch) {
                <RouterFrameProcessor as FrameProcessor<&'static Stack>>::reset(processor);
                *peer_epoch = Some(slot.epoch);
                let _ = stack
                    .manage_profile(|p| p.set_interface_state(ident, InterfaceState::Inactive));
                STATE_NOTIFY.wake_all();
            }
            for frame in slot.frames() {
                if processor.process_frame(frame, &stack, ident) {
                    STATE_NOTIFY.wake_all();
                }
            }
        }
        Err(error) => {
            if error == oxifoc_core::spi_link::Error::Checksum {
                SPI_RX_CRC_ERRORS.fetch_add(1, Ordering::Relaxed);
            } else {
                SPI_RX_HEADER_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
            SPI_LINK_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

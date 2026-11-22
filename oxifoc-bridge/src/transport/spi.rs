//! Demand-driven SPI slave: CS requests a slot, READY confirms armed DMA.
use super::{SPI_OUTQ, STATE_NOTIFY, Stack};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
#[cfg(feature = "wifi")]
use embassy_time::with_timeout;
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
/// Controller boots seen: its link epoch changed (the first one included).
#[unsafe(no_mangle)]
pub static SPI_PEER_BOOTS: AtomicU32 = AtomicU32::new(0);
fn checksum(bytes: &[u8]) -> u32 {
    !esp_hal::rom::crc::crc32_be(0, bytes)
}

/// Receive buffers the link may arm; an empty channel holds READY low.
static FREE_RX: Channel<CriticalSectionRawMutex, DmaRxBuf, 2> = Channel::new();
/// Completed slots for [`slots`] to parse and route.
static RECEIVED: Channel<CriticalSectionRawMutex, DmaRxBuf, 2> = Channel::new();
/// The transmit buffer [`slots`] fills while the link uses its own.
static FREE_TX: Channel<CriticalSectionRawMutex, DmaTxBuf, 1> = Channel::new();
/// A slot with frames for the controller.
static PREPARED: Channel<CriticalSectionRawMutex, DmaTxBuf, 1> = Channel::new();
static SEQUENCE: AtomicU32 = AtomicU32::new(0);
/// Outbound room one slot of COBS-encoded frames needs.
#[cfg(feature = "wifi")]
const SLOT_ROOM: usize = oxifoc_core::spi_link::PAYLOAD_BYTES + 64;
/// Longest a received slot waits for that room: within the controller's
/// 150 ms deadman, so commands from another link still arrive in time.
#[cfg(feature = "wifi")]
const HOLD: embassy_time::Duration = embassy_time::Duration::from_millis(50);

fn build_slot(tx: &mut DmaTxBuf, epoch: u32, fill: impl FnOnce(&mut Builder<'_>)) {
    tx.set_length(SLOT_BYTES);
    let seq = SEQUENCE.fetch_add(1, Ordering::Relaxed);
    if let Ok(mut builder) = Builder::new(tx.as_mut_slice(), epoch, seq) {
        fill(&mut builder);
        builder.finish(checksum);
    }
}

/// The slot handshake: arm DMA, raise READY, lower it when the slot is done.
///
/// Runs on the interrupt executor. In a thread task it waited for the Wi-Fi
/// stack between steps: while a 20 kHz stream was forwarded the controller
/// spent about 5 ms of every 7 ms slot waiting for READY, against 0.5 ms on
/// a quiet link. Parsing and routing stay in [`slots`]; this task only moves
/// buffers, since it runs on the stack of whatever it interrupts.
#[embassy_executor::task]
pub async fn link(r: Resources, epoch: u32) {
    assert_eq!(checksum(b"123456789"), 0x0376_e6e7);
    let mut ready = Output::new(r.ready, Level::Low, OutputConfig::default());
    let mut cs = Input::new(r.cs, InputConfig::default());
    let mut spi = Spi::new(r.bus, Mode::_0)
        .with_sck(r.sck)
        .with_mosi(r.mosi)
        .with_miso(r.miso)
        .with_cs(cs.peripheral_input())
        .with_dma(r.dma);
    // Each macro call owns its own static buffer.
    let _ = FREE_RX.try_send(dma_rx_buffer!(SLOT_BYTES).expect("fixed SPI DMA buffer"));
    let _ = FREE_RX.try_send(dma_rx_buffer!(SLOT_BYTES).expect("fixed SPI DMA buffer"));
    let _ = FREE_TX.try_send(dma_tx_buffer!(SLOT_BYTES).expect("fixed SPI DMA buffer"));
    // Sent when the controller asks for a slot and no frames wait for it.
    let mut idle = Some(dma_tx_buffer!(SLOT_BYTES).expect("fixed SPI DMA buffer"));
    loop {
        // Either side starts a slot: the controller asserts CS, or frames
        // for it are ready. Both conditions stay asserted until serviced.
        let mut prepared = PREPARED.try_receive().ok();
        if prepared.is_none()
            && cs.is_high()
            && let Either::Second(tx) = select(cs.wait_for_low(), PREPARED.receive()).await
        {
            prepared = Some(tx);
        }
        // No free buffer means routing is behind: READY stays low and the
        // controller keeps its frames.
        let mut rx = FREE_RX.receive().await;
        rx.set_length(SLOT_BYTES);
        let (tx, lent) = match (prepared, idle.take()) {
            (Some(tx), own) => {
                idle = own;
                (tx, true)
            }
            (None, Some(mut tx)) => {
                build_slot(&mut tx, epoch, |_| {});
                (tx, false)
            }
            // The idle buffer returns after every slot that used it.
            (None, None) => {
                let _ = FREE_RX.try_send(rx);
                continue;
            }
        };
        #[expect(clippy::panic, reason = "a failed DMA arm consumes the SPI buffers")]
        let Ok(transfer) = spi.transfer(SLOT_BYTES, rx, SLOT_BYTES, tx) else {
            panic!("SPI DMA arm failed");
        };
        ready.set_high();
        // Never cancel/drop an armed slave transfer: the master may reboot.
        while !transfer.is_done() {
            Timer::after_micros(100).await;
        }
        let (rx, tx);
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
        SPI_LINK_SLOTS.fetch_add(1, Ordering::Relaxed);
        // Each channel has room for every buffer of its kind.
        let _ = RECEIVED.try_send(rx);
        if lent {
            let _ = FREE_TX.try_send(tx);
        } else {
            idle = Some(tx);
        }
    }
}

/// Parse and route received slots, and fill slots for the controller.
#[embassy_executor::task]
pub async fn slots(stack: &'static Stack, ident: u8, epoch: u32) {
    let consumer = SPI_OUTQ.framed_consumer();
    let net = stack
        .manage_profile(|p| p.net_id_of(ident))
        .expect("registered SPI segment");
    let mut processor = RouterFrameProcessor::new(net);
    let mut peer_epoch = None;
    let mut free_tx: Option<DmaTxBuf> = None;
    loop {
        if free_tx.is_none() {
            free_tx = FREE_TX.try_receive().ok();
        }
        // A dropped read grant keeps its frame queued.
        let queued = consumer.read().is_ok();
        if queued && let Some(mut tx) = free_tx.take() {
            build_slot(&mut tx, epoch, |builder| {
                while let Ok(grant) = consumer.read() {
                    if builder.append(&grant).is_err() {
                        break;
                    }
                    grant.release();
                }
            });
            let _ = PREPARED.try_send(tx);
            continue;
        }
        // Routing a slot can queue the replies the next one carries.
        let received = if free_tx.is_some() {
            match select(RECEIVED.receive(), consumer.wait_read()).await {
                Either::First(rx) => Some(rx),
                Either::Second(_) => None,
            }
        } else {
            match select(RECEIVED.receive(), FREE_TX.receive()).await {
                Either::First(rx) => Some(rx),
                Either::Second(tx) => {
                    free_tx = Some(tx);
                    None
                }
            }
        };
        if let Some(mut rx) = received {
            // Hold the buffer while the Wi-Fi queue is full: READY then stays
            // low and the backlog collects in the controller's larger queue
            // instead of being dropped here. The bound keeps a stalled host
            // from holding the link against the other interfaces.
            #[cfg(feature = "wifi")]
            let _ = with_timeout(HOLD, super::tcp::room(SLOT_ROOM)).await;
            process_slot(stack, ident, &mut processor, &mut peer_epoch, &mut rx);
            let _ = FREE_RX.try_send(rx);
            // Let the task that drains the outbound queue run between slots.
            embassy_futures::yield_now().await;
        }
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
                SPI_PEER_BOOTS.fetch_add(1, Ordering::Relaxed);
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

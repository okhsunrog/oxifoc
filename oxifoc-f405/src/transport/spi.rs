//! A single task owns SPI1 and both DMA directions; FOC never touches the bus.
use super::{SPI_OUTQ, STATE_NOTIFY, Stack};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::{
    Peri, bind_interrupts,
    crc::Crc,
    dma,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    peripherals::*,
    spi::{Config, Spi},
    time::Hertz,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::direct_edge::EdgeFrameProcessor,
};
use oxifoc_core::spi_link::{Builder, SLOT_BYTES, parse};

bind_interrupts!(struct Irqs {
    DMA2_STREAM0 =>
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::SpiRxIrqBegin,
        dma::InterruptHandler<DMA2_CH0>,
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::SpiRxIrqEnd;
    DMA2_STREAM3 =>
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::SpiTxIrqBegin,
        dma::InterruptHandler<DMA2_CH3>,
        #[cfg(feature = "bench-executor-trace")]
        crate::diagnostics::trace::SpiTxIrqEnd;
});

pub struct Resources {
    pub bus: Peri<'static, SPI1>,
    pub sck: Peri<'static, PA5>,
    pub mosi: Peri<'static, PA7>,
    pub miso: Peri<'static, PA6>,
    pub cs: Peri<'static, PB11>,
    pub ready: Peri<'static, PB6>,
    pub ready_exti: Peri<'static, EXTI6>,
    pub tx: Peri<'static, DMA2_CH3>,
    pub rx: Peri<'static, DMA2_CH0>,
    pub crc: Peri<'static, CRC>,
}

#[unsafe(no_mangle)]
pub static SPI_LINK_SLOTS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
pub static SPI_LINK_ERRORS: AtomicU32 = AtomicU32::new(0);

pub fn init(spawner: embassy_executor::Spawner, stack: &'static Stack, r: Resources, epoch: u32) {
    spawner.spawn(defmt::unwrap!(pump(stack, r, epoch)));
}

fn checksum(crc: &mut Crc<'_>, bytes: &[u8]) -> u32 {
    crc.reset();
    for word in bytes.chunks_exact(4) {
        crc.feed_word(u32::from_be_bytes([word[0], word[1], word[2], word[3]]));
    }
    crc.read()
}

#[embassy_executor::task]
async fn pump(stack: &'static Stack, r: Resources, epoch: u32) {
    #[cfg(feature = "bench-spi-pump-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(3);
    use embassy_stm32::interrupt::{InterruptExt, Priority};
    embassy_stm32::interrupt::DMA2_STREAM0.set_priority(Priority::P6);
    embassy_stm32::interrupt::DMA2_STREAM3.set_priority(Priority::P6);
    let mut cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let mut ready = ExtiInput::new(r.ready, r.ready_exti, Pull::Down, crate::ExtiIrqs);
    let mut config = Config::default();
    config.frequency = Hertz(embassy_stm32::rcc::frequency::<SPI1>().0 / 4);
    config.gpio_speed = Speed::VeryHigh;
    let mut spi = Spi::new(r.bus, r.sck, r.mosi, r.miso, r.tx, r.rx, Irqs, config);
    let mut crc = Crc::new(r.crc);
    let mut tx = [0; SLOT_BYTES];
    let mut rx = [0; SLOT_BYTES];
    let consumer = SPI_OUTQ.framed_consumer();
    let mut processor = EdgeFrameProcessor::new();
    let mut seq = 0;
    let mut peer_epoch = None;
    let mut last_rx = Instant::now();

    loop {
        // No empty SPI polling: wait for queued TX, slave READY, or the
        // link-health deadline. GPIO/queue futures retain events across races.
        if consumer.read().is_err() && ready.is_low() {
            let _ = embassy_futures::select::select3(
                consumer.wait_read(),
                ready.wait_for_high(),
                Timer::after_secs(2),
            )
            .await;
        }
        if last_rx.elapsed() > Duration::from_secs(2) {
            let _ = stack.manage_profile(|p| p.set_interface_state((), InterfaceState::Inactive));
            STATE_NOTIFY.wake_all();
            <EdgeFrameProcessor as FrameProcessor<&'static Stack>>::reset(&mut processor);
            last_rx = Instant::now();
        }
        if consumer.read().is_err() && ready.is_low() {
            continue;
        }
        cs.set_low();
        // CS is also a request to prepare RX when the slave is idle.
        if with_timeout(Duration::from_millis(30), ready.wait_for_high())
            .await
            .is_err()
        {
            cs.set_high();
            SPI_LINK_ERRORS.fetch_add(1, Ordering::Relaxed);
            Timer::after_millis(10).await;
            continue;
        }
        let mut builder = defmt::unwrap!(Builder::new(&mut tx, epoch, seq).map_err(|_| ()));
        while let Ok(grant) = consumer.read() {
            if builder.append(&grant).is_err() {
                break;
            }
            // The owned DMA slot now retains this frame until transfer completion.
            grant.release();
        }
        builder.finish(|bytes| checksum(&mut crc, bytes));
        let transfer = with_timeout(Duration::from_millis(30), spi.transfer(&mut rx, &tx)).await;
        // Latch READY's falling edge BEFORE releasing CS. C5 may lower and
        // raise READY for a queued next slot before this task runs again.
        // Registering EXTI first prevents losing that short acknowledgement.
        let acknowledged = {
            use core::{
                future::{Future, poll_fn},
                pin::pin,
                task::Poll,
            };
            let mut ack = pin!(ready.wait_for_low());
            let already_low = poll_fn(|cx| Poll::Ready(ack.as_mut().poll(cx).is_ready())).await;
            cs.set_high();
            already_low || with_timeout(Duration::from_millis(30), ack).await.is_ok()
        };
        seq = seq.wrapping_add(1);
        if !matches!(transfer, Ok(Ok(()))) || !acknowledged {
            SPI_LINK_ERRORS.fetch_add(1, Ordering::Relaxed);
            Timer::after_millis(10).await;
            continue;
        }
        SPI_LINK_SLOTS.fetch_add(1, Ordering::Relaxed);
        match parse(&mut rx, |bytes| checksum(&mut crc, bytes)) {
            Ok(slot) => {
                last_rx = Instant::now();
                if peer_epoch != Some(slot.epoch) {
                    // A new root boot may assign a different segment. Do not
                    // carry the previous root's sticky address into this epoch.
                    processor = EdgeFrameProcessor::new();
                    let _ = stack.manage_profile(|p| {
                        p.set_interface_state((), InterfaceState::edge_link_local())
                    });
                    STATE_NOTIFY.wake_all();
                    peer_epoch = Some(slot.epoch);
                }
                for frame in slot.frames() {
                    if processor.process_frame(frame, &stack, ()) {
                        STATE_NOTIFY.wake_all();
                    }
                }
            }
            Err(_) => {
                SPI_LINK_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
        // CS is high here; the next READY denotes a new armed slot.
        embassy_futures::yield_now().await;
    }
}

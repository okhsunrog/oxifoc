//! Ergot sink for the Packed28 session stream. It runs in a task, never in
//! the ADC ISR.

#[cfg(feature = "transport-spi")]
use oxifoc_core::runtime::raw_stream::SendHeadroom;
use oxifoc_core::runtime::raw_stream::{self, ErgotRawSink};
use oxifoc_core::timer::EmbassyTimer;

/// Only this task sends the raw stream. Wait for enough contiguous space for
/// its MTU and one reply MTU before enqueueing. Aborting the probe publishes
/// nothing. The send that follows is synchronous on the same executor, so a
/// different producer cannot take that headroom in between. The existing
/// FIFO still orders commands and telemetry.
#[cfg(feature = "transport-spi")]
struct SpiHeadroom;

#[cfg(feature = "transport-spi")]
impl SendHeadroom for SpiHeadroom {
    async fn wait(&mut self) {
        const MTU: u16 = crate::config::MAX_PACKET_SIZE as u16;
        const FRAME_PREFIX_BYTES: u16 = size_of::<u16>() as u16;
        drop(
            super::SPI_OUTQ
                .framed_producer()
                .wait_grant(2 * MTU + FRAME_PREFIX_BYTES)
                .await,
        );
    }
}

/// Encode in thread mode alongside the command servers. A queued Ergot send
/// can complete immediately; its self-waking stream must not keep an interrupt
/// executor runnable and starve those servers. USB RX/TX stay on their executor.
#[embassy_executor::task]
pub async fn ergot_task(stack: &'static super::Stack) {
    embassy_executor::MetadataRef::for_current_task()
        .await
        .set_priority(1);
    #[cfg(feature = "transport-spi")]
    let headroom = SpiHeadroom;
    #[cfg(not(feature = "transport-spi"))]
    let headroom = ();
    let mut sink = ErgotRawSink::<_, EmbassyTimer, _>::new(stack, headroom);
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    raw_stream::raw_telemetry_stream::<_, EmbassyTimer>(&mut sink).await;
}

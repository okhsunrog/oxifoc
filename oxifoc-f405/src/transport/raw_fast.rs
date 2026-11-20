//! Ergot sink for the Packed28 session stream. It runs in a task, never in
//! the ADC ISR.

use embassy_time::{Duration, with_timeout};
use oxifoc_core::runtime::raw_stream::{self, RawSinkError, RawTelemetrySink};
use oxifoc_core::timer::EmbassyTimer;

/// Bound a disconnected or unconsumed block without ever holding the producer gate.
const BLOCK_TIMEOUT: Duration = Duration::from_millis(250);

struct ErgotRawSink {
    stack: &'static super::Stack,
}

impl RawTelemetrySink for ErgotRawSink {
    async fn write_block(
        &mut self,
        destination: ergot::Address,
        bytes: &[u8],
    ) -> Result<(), RawSinkError> {
        use oxifoc_core::icd::{RawTelemetryBlockRef, RawTelemetryBlockRefTopic};

        with_timeout(BLOCK_TIMEOUT, async {
            #[cfg(feature = "transport-spi")]
            let producer = super::SPI_OUTQ.framed_producer();
            loop {
                // Only this task sends the raw stream. Wait for enough
                // contiguous space for its MTU and one reply MTU before
                // enqueueing. Aborting the probe publishes nothing. The
                // send below is synchronous on the same executor, so a
                // different producer cannot take that headroom in between.
                // The existing FIFO still orders commands and telemetry.
                #[cfg(feature = "transport-spi")]
                {
                    const MTU: u16 = crate::config::MAX_PACKET_SIZE as u16;
                    const FRAME_PREFIX_BYTES: u16 = size_of::<u16>() as u16;
                    drop(producer.wait_grant(2 * MTU + FRAME_PREFIX_BYTES).await);
                }
                // Route to the node which armed this session, including a host
                // behind another router. Never retry a partially successful broadcast.
                if self
                    .stack
                    .topics()
                    .unicast_borrowed::<RawTelemetryBlockRefTopic<'_>>(
                        destination,
                        &RawTelemetryBlockRef { data: bytes },
                    )
                    .is_ok()
                {
                    // Queue acceptance, not host receipt: the host still
                    // requires the matching END and exact record totals.
                    return Ok(());
                }
                embassy_time::Timer::after_micros(100).await;
            }
        })
        .await
        .map_err(|_| RawSinkError::Unavailable)?
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
    let mut sink = ErgotRawSink { stack };
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    raw_stream::raw_telemetry_stream::<_, EmbassyTimer>(&mut sink).await;
}

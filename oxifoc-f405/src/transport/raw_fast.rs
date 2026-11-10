//! Bench sinks for the raw session stream. No sink runs in the ADC ISR.

use embassy_time::{Duration, with_timeout};
use oxifoc_core::runtime::raw_stream::{RawSinkError, RawTelemetrySink};

/// Bound a disconnected or unconsumed block without ever holding the producer gate.
const BLOCK_TIMEOUT: Duration = Duration::from_millis(250);

#[cfg(feature = "bench-raw-ergot")]
struct ErgotRawSink {
    stack: &'static super::Stack,
}

#[cfg(feature = "bench-raw-ergot")]
impl RawTelemetrySink for ErgotRawSink {
    async fn write_block(
        &mut self,
        destination: ergot::Address,
        bytes: &[u8],
    ) -> Result<(), RawSinkError> {
        use oxifoc_core::icd::{RawTelemetryBlockRef, RawTelemetryBlockRefTopic};

        with_timeout(BLOCK_TIMEOUT, async {
            loop {
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
#[cfg(feature = "bench-raw-ergot")]
#[embassy_executor::task]
pub async fn ergot_task(stack: &'static super::Stack) {
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(1);
    let mut sink = ErgotRawSink { stack };
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    oxifoc_core::runtime::raw_stream::raw_telemetry_stream::<_, oxifoc_core::timer::EmbassyTimer>(
        &mut sink,
    )
    .await;
}

#[cfg(feature = "bench-raw-usb")]
type UsbIn = <super::AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn;

#[cfg(feature = "bench-raw-usb")]
struct RawStringHandler(embassy_usb::types::StringIndex);

#[cfg(feature = "bench-raw-usb")]
impl embassy_usb::Handler for RawStringHandler {
    fn get_string(&mut self, index: embassy_usb::types::StringIndex, lang_id: u16) -> Option<&str> {
        (index == self.0 && lang_id == embassy_usb::descriptor::lang_id::ENGLISH_US)
            .then_some("OxiFOC raw telemetry")
    }
}

#[cfg(feature = "bench-raw-usb")]
static RAW_STRING: static_cell::StaticCell<RawStringHandler> = static_cell::StaticCell::new();

/// Append interface 1 without changing the existing Ergot interface 0.
#[cfg(feature = "bench-raw-usb")]
pub(super) fn add_usb_interface(
    builder: &mut embassy_usb::Builder<'static, super::AppDriver>,
) -> UsbIn {
    use embassy_usb::driver::{Direction, EndpointAddress};

    let string = builder.string();
    builder.handler(RAW_STRING.init(RawStringHandler(string)));
    let mut function = builder.function(0xFF, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xFF, 0xCA, 0x7E, Some(string));
    alt.endpoint_bulk_in(Some(EndpointAddress::from_parts(2, Direction::In)), 64)
}

#[cfg(feature = "bench-raw-usb")]
struct UsbRawSink(UsbIn);

#[cfg(feature = "bench-raw-usb")]
impl RawTelemetrySink for UsbRawSink {
    async fn write_block(
        &mut self,
        _destination: ergot::Address,
        bytes: &[u8],
    ) -> Result<(), RawSinkError> {
        use embassy_usb::driver::EndpointIn;

        // The borrowed block and cursor survive every endpoint await. A failed prefix
        // aborts this session; its cookie prevents a later session accepting that prefix.
        with_timeout(BLOCK_TIMEOUT, async {
            for packet in bytes.chunks(64) {
                self.0
                    .write(packet)
                    .await
                    .map_err(|_| RawSinkError::Unavailable)?;
            }
            // Complete the host's bulk transfer even when END is the last block.
            if bytes.len().is_multiple_of(64) {
                self.0
                    .write(&[])
                    .await
                    .map_err(|_| RawSinkError::Unavailable)?;
            }
            Ok(())
        })
        .await
        .map_err(|_| RawSinkError::Unavailable)?
    }
}

#[cfg(feature = "bench-raw-usb")]
#[embassy_executor::task]
pub async fn usb_task(ep: UsbIn) {
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(1);
    let mut sink = UsbRawSink(ep);
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    oxifoc_core::runtime::raw_stream::raw_telemetry_stream::<_, oxifoc_core::timer::EmbassyTimer>(
        &mut sink,
    )
    .await;
}

#[cfg(feature = "bench-raw-rtt")]
static RTT_CHANNEL: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    rtt_target::UpChannel,
    1,
> = embassy_sync::channel::Channel::new();

#[cfg(feature = "bench-raw-rtt")]
pub(super) fn init_rtt(channel: rtt_target::UpChannel) {
    // Initialization is once, before the task is spawned. The channel then has
    // exactly one writer, with no critical section around the bulk buffer copy.
    assert!(RTT_CHANNEL.try_send(channel).is_ok());
}

#[cfg(feature = "bench-raw-rtt")]
struct RttRawSink(rtt_target::UpChannel);

#[cfg(feature = "bench-raw-rtt")]
impl RawTelemetrySink for RttRawSink {
    async fn write_block(
        &mut self,
        _destination: ergot::Address,
        bytes: &[u8],
    ) -> Result<(), RawSinkError> {
        with_timeout(BLOCK_TIMEOUT, async {
            loop {
                // A probe may overwrite RTT flags: enforce nonblocking whole-block
                // commit on each attempt. NoBlockSkip commits either all bytes or none.
                self.0.set_mode(rtt_target::ChannelMode::NoBlockSkip);
                if self.0.write(bytes) == bytes.len() {
                    return Ok(());
                }
                embassy_time::Timer::after_micros(100).await;
            }
        })
        .await
        .map_err(|_| RawSinkError::Unavailable)?
    }
}

#[cfg(feature = "bench-raw-rtt")]
#[embassy_executor::task]
pub async fn rtt_task() {
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(1);
    let mut sink = RttRawSink(RTT_CHANNEL.receive().await);
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    oxifoc_core::runtime::raw_stream::raw_telemetry_stream::<_, oxifoc_core::timer::EmbassyTimer>(
        &mut sink,
    )
    .await;
}

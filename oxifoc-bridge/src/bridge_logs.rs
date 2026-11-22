//! Send bridge log records over Ergot and keep low-level diagnostics on RTT.

use core::fmt::Write;

use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use ergot::logging::log_v0_4::{LogSink, set_ergot_internal_log_sink};
use heapless::String;
use log::{Level, LevelFilter, Log, Metadata, Record};
use rtt_target::{ChannelMode::NoBlockSkip, rprintln, rtt_init_print};

use crate::transport::Stack;

struct Line {
    level: Level,
    text: String<192>,
}

// Log calls can originate inside radio drivers or interrupt handlers. A full
// queue drops the network copy and the forwarding task owns all Ergot calls.
#[esp_hal::ram(unstable(rtc_fast))]
static OUT: Channel<CriticalSectionRawMutex, Line, 8> = Channel::new();

struct Rtt;
static RTT: Rtt = Rtt;

impl Log for Rtt {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() <= Level::Info
    }

    fn log(&self, record: &Record<'_>) {
        if self.enabled(record.metadata()) {
            rprintln!("[{} {}] {}", record.level(), record.target(), record.args());
        }
    }

    fn flush(&self) {}
}

struct BridgeLogger;
static LOGGER: BridgeLogger = BridgeLogger;

impl Log for BridgeLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        RTT.enabled(metadata)
    }

    fn log(&self, record: &Record<'_>) {
        RTT.log(record);
        if !self.enabled(record.metadata())
            || record.target().split("::").next() != Some("oxifoc_bridge")
        {
            return;
        }
        let mut text = String::new();
        let _ = write!(text, "{}", record.args());
        let _ = OUT.try_send(Line {
            level: record.level(),
            text,
        });
    }

    fn flush(&self) {}
}

pub fn init() {
    rtt_init_print!(NoBlockSkip, 1024);
    // The normal Ergot macros use their own sink, while a few direct log
    // calls still reach LOGGER. Its allowlist keeps both paths out of OUT.
    set_ergot_internal_log_sink(&RTT);
    log::set_logger(&LOGGER).expect("bridge logger installed once");
    log::set_max_level(LevelFilter::Info);
}

#[task]
pub async fn forward(stack: &'static Stack) {
    let sink = LogSink::new(stack);
    loop {
        let line = OUT.receive().await;
        sink.log(
            &Record::builder()
                .level(line.level)
                .args(format_args!("{}", line.text))
                .build(),
        );
        yield_now().await;
    }
}

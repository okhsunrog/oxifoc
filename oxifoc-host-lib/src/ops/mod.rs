//! Presentation-agnostic host operations shared by every front-end.
//!
//! The CLI and GUI must never re-derive the same host-side logic (and then
//! drift apart, as they had). Anything that turns a user intent into device
//! commands — config group (de)serialization, the simplified angle-source
//! presets, the detection sequence and its post-processing — lives here and
//! is called by both. The front-ends keep only their own rendering.

pub mod config;
pub mod detect;
pub mod phase;

/// Fast-telemetry stream rates offered to the user (Hz), shared so the
/// combo-box order, the CLI flags and the index→Hz mapping cannot diverge.
pub const STREAM_RATES: [u16; 7] = [100, 500, 1000, 2000, 5000, 10000, 20000];

/// Hz for a stream-rate selector index, falling back to 1 kHz out of range.
#[must_use]
pub fn stream_rate_hz(index: usize) -> u16 {
    STREAM_RATES.get(index).copied().unwrap_or(1000)
}

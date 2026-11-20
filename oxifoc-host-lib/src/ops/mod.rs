//! Presentation-agnostic host operations shared by every front-end.
//!
//! The CLI and GUI must never re-derive the same host-side logic (and then
//! drift apart, as they had). Anything that turns a user intent into device
//! commands — config group (de)serialization, the simplified angle-source
//! presets, the detection sequence and its post-processing — lives here and
//! is called by both. The front-ends keep only their own rendering.

pub mod config;
pub mod detect;
pub mod fault;
pub mod pairing;
pub mod phase;
pub mod telemetry;

/// One-line JSON for logs and plain-text output. Quantities serialize as
/// their SI value, like `config get`; their `Debug` form spells out the
/// base-unit dimensions (`0.0642 m^2 kg^1 s^-3 A^-2`).
#[must_use]
pub fn json_text<T: serde::Serialize + core::fmt::Debug>(value: &T) -> String {
    serde_json::to_string(value).unwrap_or_else(|_| format!("{value:?}"))
}

/// Fast-telemetry stream rates offered to the user (Hz), shared so the
/// combo-box order, the CLI flags and the index→Hz mapping cannot diverge.
pub const STREAM_RATES: [u16; 7] = [100, 500, 1000, 2000, 5000, 10000, 20000];

/// Hz for a stream-rate selector index, falling back to 1 kHz out of range.
#[must_use]
pub fn stream_rate_hz(index: usize) -> u16 {
    STREAM_RATES.get(index).copied().unwrap_or(1000)
}

#[cfg(test)]
mod tests {
    use super::json_text;
    use oxifoc_core::storage::CurrentLimitsConfig;
    use oxifoc_core::types::ConfigValue;
    use oxifoc_core::units::amps;

    #[test]
    fn json_text_prints_quantities_as_si_values() {
        let value = ConfigValue::CurrentLimits(CurrentLimitsConfig {
            max_phase_current: Some(amps(30.0)),
            ..CurrentLimitsConfig::AUTO
        });
        let text = json_text(&value);
        assert!(text.contains(r#""max_phase_current":30.0"#), "{text}");
        assert!(text.contains(r#""bus_in_max":null"#), "{text}");
        assert!(!text.contains("A^1") && !text.contains('\n'), "{text}");
    }
}

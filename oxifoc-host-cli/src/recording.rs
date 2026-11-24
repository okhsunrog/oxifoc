//! A fast telemetry session recorded on the bridge: start it, stop it, fetch
//! it into a parquet file, clear it.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::Subcommand;
use oxifoc_core::{
    raw_record::{RECORD_CHUNK_BYTES, RecordMode, RecordState, RecordStatus},
    raw_telemetry::RAW_MAX_RECORDS,
    types::ConfigGroupId,
};
use oxifoc_host_cli::{
    config_cli,
    record::{Capture, latest_hw_info},
};
use oxifoc_host_lib::{HostRuntime, build_enrich_ctx, ops::recording as ops};
use serde_json::json;

use crate::emit;

#[derive(Subcommand)]
pub enum RecordingAction {
    /// Start recording in the bridge's memory
    Start {
        #[arg(
            long,
            default_value_t = 20_000,
            help = "Rate (Hz); the controller's own rate if that is lower"
        )]
        fast_hz: u16,
        #[arg(
            long,
            help = "Keep the newest data instead of stopping when the memory is full, \
                    and stop a second after a fault that stops the motor"
        )]
        ring: bool,
    },
    /// Stop the recording; it stays in the bridge until cleared
    Stop,
    /// What the bridge records or keeps
    Status,
    /// Fetch a stopped recording into a parquet file, or as it is stored
    Fetch {
        #[arg(
            short,
            long,
            required_unless_present = "raw",
            help = "Output parquet file path; decoding needs the controller connected"
        )]
        out: Option<String>,
        #[arg(
            long,
            help = "Save the recording's bytes as the bridge holds them; needs only the bridge"
        )]
        raw: Option<String>,
    },
    /// Forget a stopped recording
    Clear,
}

impl RecordingAction {
    /// A fetch into parquet decodes with the controller's calibration; the
    /// rest needs only the bridge.
    pub fn needs_controller(&self) -> bool {
        matches!(self, Self::Fetch { out: Some(_), .. })
    }
}

/// Seconds of data in the recording's blocks: full blocks of
/// [`RAW_MAX_RECORDS`] but the last few.
fn seconds(status: &RecordStatus) -> f64 {
    match status.timing.actual_fast_hz {
        0 => 0.0,
        hz => f64::from(status.blocks) * RAW_MAX_RECORDS as f64 / f64::from(hz),
    }
}

fn describe(status: &RecordStatus) -> String {
    if status.capacity == 0 {
        return "this bridge has no memory for recordings".into();
    }
    let kept = format!(
        "{} of {} bytes ({:.0} %), {} blocks, about {:.1} s at {} Hz",
        status.bytes,
        status.capacity,
        100.0 * f64::from(status.bytes) / f64::from(status.capacity),
        status.blocks,
        seconds(status),
        status.timing.actual_fast_hz,
    );
    let losses = format!(
        "missed {} block(s), dropped {} byte(s) from the ring's old end",
        status.missed, status.overwritten
    );
    match status.state {
        RecordState::Empty => format!("empty, {} bytes free", status.capacity),
        RecordState::Recording => format!("recording ({:?}): {kept}; {losses}", status.mode),
        RecordState::Stopped(end) => {
            format!("stopped ({end:?}, {:?}): {kept}; {losses}", status.mode)
        }
    }
}

fn report(json: bool, status: &RecordStatus) -> Result<()> {
    emit(json, serde_json::to_value(status)?, describe(status));
    Ok(())
}

pub fn run(runtime: &HostRuntime, action: RecordingAction, json: bool) -> Result<()> {
    let cmd = &runtime.cmd_tx;
    match action {
        RecordingAction::Start { fast_hz, ring } => {
            let mode = if ring {
                RecordMode::Ring
            } else {
                RecordMode::UntilFull
            };
            report(json, &ops::start(cmd, fast_hz, mode)?)
        }
        RecordingAction::Stop => report(json, &ops::stop(cmd)?),
        RecordingAction::Status => report(json, &ops::status(cmd)?),
        RecordingAction::Clear => report(json, &ops::clear(cmd)?),
        RecordingAction::Fetch { out, raw } => fetch(runtime, out.as_deref(), raw.as_deref(), json),
    }
}

fn fetch(runtime: &HostRuntime, out: Option<&str>, raw: Option<&str>, json: bool) -> Result<()> {
    let cmd = &runtime.cmd_tx;
    // One reply carries a chunk, a length and a tag in a frame of the link.
    let chunk = (runtime.raw_telemetry.max_block_bytes() - 8).min(RECORD_CHUNK_BYTES as u16);
    let began = Instant::now();
    let mut shown = Instant::now();
    let (status, bytes) = ops::fetch(cmd, chunk, |done, total| {
        if !json && shown.elapsed() > Duration::from_secs(1) {
            shown = Instant::now();
            eprintln!("fetched {done} of {total} bytes");
        }
    })?;
    let fetched = began.elapsed();
    if let Some(raw) = raw {
        std::fs::write(raw, &bytes).with_context(|| format!("write {raw}"))?;
    }
    let records = ops::decode(&bytes, status.session)?;
    let recording = json!({
        "status": status,
        "bytes_fetched": bytes.len(),
        "fetch_seconds": fetched.as_secs_f64(),
        "blocks": records.blocks,
        "records": records.samples.len(),
        "missing_blocks": records.missing_blocks,
    });
    let Some(out) = out else {
        emit(
            json,
            recording,
            format!(
                "{} bytes fetched in {:.1} s: {} records in {} blocks ({} missing)",
                bytes.len(),
                fetched.as_secs_f64(),
                records.samples.len(),
                records.blocks,
                records.missing_blocks,
            ),
        );
        return Ok(());
    };
    if records.samples.is_empty() {
        bail!("the recording holds no records");
    }
    let hw = latest_hw_info(runtime).context("hardware handshake unavailable for decoding")?;
    let Some(decimation_m) = status
        .timing
        .decimation_m
        .filter(|_| status.timing.actual_fast_hz != 0 && status.timing.control_timing.is_valid())
    else {
        bail!("the recording has no valid timing: {:?}", status.timing);
    };
    let capture = Capture {
        board_calibration: config_cli::read_group(cmd, ConfigGroupId::BoardCalibration)
            .context("read the board calibration")?,
        enrich: build_enrich_ctx(cmd, Some(&hw)),
        hw: Some(hw),
        control_timing: status.timing.control_timing,
        fast_hz_requested: status.timing.actual_fast_hz,
        fast_hz_actual: status.timing.actual_fast_hz,
        decimation_m: Some(decimation_m),
        samples: records.samples,
        started: began,
        raw_telemetry: None,
        raw_session: Some(status.session),
    };
    let meta = [("oxifoc.recording".to_string(), recording.to_string())];
    let snapshot = config_cli::config_snapshot(cmd);
    let (summary, _) = capture.finish_checked(out, &snapshot, &meta)?;
    emit(
        json,
        json!({"capture": summary, "recording": recording}),
        format!(
            "{}: {} rows at {} Hz from {} blocks ({} missing), {} bytes fetched in {:.1} s",
            summary.path,
            summary.rows,
            summary.fast_hz_actual,
            records.blocks,
            records.missing_blocks,
            bytes.len(),
            fetched.as_secs_f64(),
        ),
    );
    Ok(())
}

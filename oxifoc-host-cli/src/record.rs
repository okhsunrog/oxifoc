//! Telemetry capture to a parquet file.
//!
//! Collects fast telemetry for a fixed duration and writes one parquet
//! file with full provenance in the key-value metadata: firmware identity,
//! the actual decimation factor and its anti-alias filter (group delay
//! included — phase-sensitive analysis must account for it), a JSON
//! snapshot of the device configuration, and the seq-gap statistics.
//!
//! Integrity contract: `seq` is the raw device-side FOC-cycle counter.
//! Consecutive samples must differ by exactly the decimation factor M;
//! any larger step is a dropped frame (link backpressure). The summary
//! reports gaps and the process exits nonzero when frames were lost, so
//! scripted captures cannot silently analyze hole-ridden data.

use std::fs::File;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use oxifoc_core::foc::telemetry::{EnrichCtx, RichSample};
use oxifoc_core::types::{FastTelemetry, HardwareInfo, TelemetryConfig};
use oxifoc_host_lib::{HostCommand, HostRuntime, build_enrich_ctx};
use parquet::basic::{Compression, ZstdLevel};
use parquet::data_type::{DoubleType, Int32Type, Int64Type};
use parquet::file::metadata::KeyValue;
use parquet::file::properties::WriterProperties;
use parquet::file::writer::SerializedFileWriter;
use parquet::schema::parser::parse_message_type;

// Raw ADC currents + seq are kept for provenance / gap-integrity; the
// engineering columns are reconstructed host-side by `FastTelemetry::enrich`
// (the same oxifoc-core code the firmware encodes with) and are NaN when no
// device calibration was available.
const SCHEMA: &str = "message oxifoc_telemetry {
    required double t_s;
    required int64 seq;
    required int32 ia_adc;
    required int32 ib_adc;
    required int32 ic_adc;
    required double ia_a;
    required double ib_a;
    required double ic_a;
    required double id_a;
    required double iq_a;
    required double vbus_v;
    required double vd_v;
    required double vq_v;
    required double angle_rad;
    required double mech_rpm;
    required double erpm;
}";

#[derive(serde::Serialize)]
pub struct RecordSummary {
    pub path: String,
    pub rows: usize,
    pub fast_hz_actual: u16,
    pub decimation_m: u32,
    pub duration_s: f64,
    /// Number of places where seq jumped by more than M.
    pub gaps: usize,
    /// Total samples missing across all gaps.
    pub samples_lost: u64,
}

/// Latest hardware info the backend has seen (handshake), if any.
pub fn latest_hw_info(runtime: &HostRuntime) -> Option<HardwareInfo> {
    let mut latest = None;
    while let Ok(info) = runtime.device_info_rx.try_recv() {
        latest = Some(info);
    }
    latest
}

/// A streaming capture in progress: device acked the rate, the warm-up
/// transient has been discarded, samples accumulate via [`Self::drain_until`].
pub struct Capture {
    pub hw: Option<HardwareInfo>,
    pub fast_hz_requested: u16,
    pub fast_hz_actual: u16,
    pub decimation_m: u32,
    pub samples: Vec<FastTelemetry>,
    pub started: Instant,
    /// Host-side raw→engineering context (device `BoardCalib` + `dc_offsets` +
    /// `pole_pairs`). `None` only when no handshake info was available.
    pub enrich: Option<EnrichCtx>,
}

impl Capture {
    /// Enable streaming, wait for the device ack, eat the enable transient.
    pub fn start(runtime: &HostRuntime, fast_hz: u16) -> Result<Self> {
        if fast_hz == 0 {
            bail!("fast_hz must be nonzero for a capture");
        }
        let hw = latest_hw_info(runtime);
        // Build the enrichment context up front (config reads) — while the link
        // is quiet, before the high-rate stream starts competing for it.
        let enrich = build_enrich_ctx(&runtime.cmd_tx, hw.as_ref());
        runtime
            .cmd_tx
            .send(HostCommand::SetTelemetryConfig(TelemetryConfig { fast_hz }))
            .context("send telemetry config")?;
        let ack_deadline = Instant::now() + Duration::from_secs(3);
        let fast_hz_actual = loop {
            let v = runtime.fast_hz.load(Ordering::Relaxed);
            if v != 0 {
                break v;
            }
            if Instant::now() >= ack_deadline {
                bail!("device did not acknowledge telemetry config within 3 s");
            }
            std::thread::sleep(Duration::from_millis(20));
        };
        let foc_freq_hz = hw.as_ref().map(|h| h.foc_freq_hz).unwrap_or(0);
        let decimation_m = if foc_freq_hz > 0 {
            foc_freq_hz / u32::from(fast_hz_actual)
        } else {
            0
        };

        // Warm-up: the enable transient (queue wrap before the device stream
        // task wakes) produces a guaranteed gap at the head of the stream.
        // Let it pass, then drain everything stale so the capture starts on
        // steady-state data only.
        std::thread::sleep(Duration::from_millis(150));
        while runtime.fast_rx.try_recv().is_ok() {}

        Ok(Self {
            hw,
            fast_hz_requested: fast_hz,
            fast_hz_actual,
            decimation_m,
            samples: Vec::new(),
            started: Instant::now(),
            enrich,
        })
    }

    /// Collect samples until `deadline` (host clock).
    pub fn drain_until(&mut self, runtime: &HostRuntime, deadline: Instant) -> Result<()> {
        while Instant::now() < deadline {
            let left = deadline.saturating_duration_since(Instant::now());
            let timeout = left.min(Duration::from_millis(100));
            match runtime.fast_rx.recv_timeout(timeout) {
                Ok(s) => self.samples.push(s),
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                    bail!("telemetry channel disconnected mid-capture")
                }
            }
        }
        Ok(())
    }

    /// Raw device seq of the latest sample seen (event↔sample anchor).
    pub fn last_seq(&self) -> Option<u32> {
        self.samples.last().map(|s| u32::from(s.seq))
    }

    /// Disable streaming (best effort).
    pub fn stop(&self, runtime: &HostRuntime) {
        let _ = runtime
            .cmd_tx
            .send(HostCommand::SetTelemetryConfig(TelemetryConfig {
                fast_hz: 0,
            }));
    }

    /// Expected seq step between consecutive samples.
    pub fn expected_step(&self) -> u32 {
        if self.decimation_m > 0 {
            self.decimation_m
        } else {
            // Unknown FOC frequency: infer the step as the smallest observed delta.
            self.samples
                .windows(2)
                .map(|w| u32::from(w[1].seq.wrapping_sub(w[0].seq)))
                .filter(|&d| d > 0)
                .min()
                .unwrap_or(1)
        }
    }

    /// (gap count, total samples lost) over raw seq deltas.
    pub fn analyze_gaps(&self) -> (usize, u64) {
        let expected_step = self.expected_step();
        let mut gaps = 0usize;
        let mut samples_lost = 0u64;
        for w in self.samples.windows(2) {
            let d = u32::from(w[1].seq.wrapping_sub(w[0].seq));
            if d > expected_step {
                gaps += 1;
                samples_lost += u64::from((d - expected_step) / expected_step);
            }
        }
        (gaps, samples_lost)
    }

    /// Write the capture and produce the summary. `extra_meta` lands in the
    /// parquet key-value metadata (e.g. the maneuver event log).
    pub fn finish(
        &self,
        out_path: &str,
        config_snapshot: &serde_json::Value,
        extra_meta: &[(String, String)],
    ) -> Result<RecordSummary> {
        if self.samples.is_empty() {
            bail!("no telemetry received — is the device connected and streaming?");
        }
        let (gaps, samples_lost) = self.analyze_gaps();
        write_parquet(
            out_path,
            &self.samples,
            self.hw.as_ref(),
            self.enrich.as_ref(),
            self.fast_hz_requested,
            self.fast_hz_actual,
            self.expected_step(),
            config_snapshot,
            gaps,
            samples_lost,
            extra_meta,
        )?;
        Ok(RecordSummary {
            path: out_path.to_string(),
            rows: self.samples.len(),
            fast_hz_actual: self.fast_hz_actual,
            decimation_m: self.expected_step(),
            duration_s: self.started.elapsed().as_secs_f64(),
            gaps,
            samples_lost,
        })
    }
}

pub fn record(
    runtime: &HostRuntime,
    out_path: &str,
    seconds: f64,
    fast_hz: u16,
    config_snapshot: serde_json::Value,
) -> Result<RecordSummary> {
    let mut cap = Capture::start(runtime, fast_hz)?;
    let deadline = cap.started + Duration::from_secs_f64(seconds);
    let drained = cap.drain_until(runtime, deadline);
    cap.stop(runtime);
    drained?;
    cap.finish(out_path, &config_snapshot, &[])
}

#[allow(clippy::too_many_arguments)]
fn write_parquet(
    path: &str,
    samples: &[FastTelemetry],
    hw: Option<&HardwareInfo>,
    enrich: Option<&EnrichCtx>,
    fast_hz_requested: u16,
    fast_hz_actual: u16,
    decimation_m: u32,
    config_snapshot: &serde_json::Value,
    gaps: usize,
    samples_lost: u64,
    extra_meta: &[(String, String)],
) -> Result<()> {
    let kv = |k: &str, v: String| KeyValue {
        key: k.to_string(),
        value: Some(v),
    };
    let mut meta = vec![
        kv("oxifoc.cli_version", env!("CARGO_PKG_VERSION").to_string()),
        kv(
            "oxifoc.captured_unix_s",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs().to_string())
                .unwrap_or_default(),
        ),
        kv("oxifoc.fast_hz_requested", fast_hz_requested.to_string()),
        kv("oxifoc.fast_hz_actual", fast_hz_actual.to_string()),
        kv("oxifoc.decimation_m", decimation_m.to_string()),
        kv(
            "oxifoc.telemetry_note",
            "ia_adc/ib_adc/ic_adc = uncalibrated 12-bit ADC counts (3 shunts, NO \
             anti-alias filter — decimation is plain sample-dropping at M). The \
             *_a / *_v / angle_rad / *rpm columns are reconstructed host-side by \
             oxifoc_core FastTelemetry::enrich (BoardCalib + dc_offsets + \
             pole_pairs) — the same core code the firmware encodes with. NaN only \
             when the device handshake carried no BoardCalib at all; if the \
             DcOffsets/MotorParams config reads failed, enrichment silently falls \
             back to MID-SCALE offsets / pole_pairs=0 and the columns are wrong \
             but non-NaN — cross-check the dc-offsets group in oxifoc.config \
             below (missing there = the fallback was used)."
                .to_string(),
        ),
        kv("oxifoc.config", config_snapshot.to_string()),
        kv("oxifoc.seq_gaps", gaps.to_string()),
        kv("oxifoc.samples_lost", samples_lost.to_string()),
    ];
    for (k, v) in extra_meta {
        meta.push(kv(k, v.clone()));
    }
    if let Some(h) = hw {
        meta.push(kv("oxifoc.hw", h.hw.to_string()));
        meta.push(kv("oxifoc.sw", h.sw.to_string()));
        meta.push(kv("oxifoc.mcu", h.mcu.to_string()));
        meta.push(kv("oxifoc.uuid", h.uuid.to_string()));
        meta.push(kv("oxifoc.foc_freq_hz", h.foc_freq_hz.to_string()));
        meta.push(kv("oxifoc.max_current_a", h.max_current_a.to_string()));
    }

    let schema = Arc::new(parse_message_type(SCHEMA).context("parquet schema")?);
    let props = Arc::new(
        WriterProperties::builder()
            .set_compression(Compression::ZSTD(ZstdLevel::default()))
            .set_key_value_metadata(Some(meta))
            .build(),
    );
    let file = File::create(path).with_context(|| format!("create {path}"))?;
    let mut writer = SerializedFileWriter::new(file, schema, props)?;

    // Column vectors. Time axis from raw seq against the device FOC clock —
    // immune to host-side batching jitter. The device `seq` is a u16 FOC-cycle
    // counter (wraps every 65536 cycles ≈ 3.3 s at 20 kHz), so a single
    // `wrapping_sub(seq0)` only stays monotonic within one wrap period — past
    // it `t_s` would sawtooth back to 0. Accumulate the per-sample wrapping
    // deltas into a u64 cycle count instead (each delta ≈ M ≪ 65536, so every
    // wrap is unambiguous), matching the `FastTelemetry::seq` contract. The raw
    // `seq` column stays untouched — maneuver events anchor to it by raw value.
    let foc_freq = hw.map(|h| h.foc_freq_hz).unwrap_or(0);
    let mut cycles: u64 = 0;
    let mut prev = samples[0].seq;
    let t_s: Vec<f64> = samples
        .iter()
        .map(|s| {
            cycles += u64::from(s.seq.wrapping_sub(prev));
            prev = s.seq;
            if foc_freq > 0 {
                cycles as f64 / f64::from(foc_freq)
            } else {
                f64::NAN
            }
        })
        .collect();
    let seq: Vec<i64> = samples.iter().map(|s| i64::from(s.seq)).collect();
    // Raw uncalibrated ADC counts (12-bit) for the three shunts.
    let ia: Vec<i32> = samples.iter().map(|s| i32::from(s.ia)).collect();
    let ib: Vec<i32> = samples.iter().map(|s| i32::from(s.ib)).collect();
    let ic: Vec<i32> = samples.iter().map(|s| i32::from(s.ic)).collect();

    // Engineering columns: reconstruct each sample once via the shared enrich
    // path (None → NaN so the schema stays fixed regardless of calibration).
    let rich: Vec<Option<RichSample>> = samples
        .iter()
        .map(|s| enrich.map(|c| s.enrich(c)))
        .collect();
    let eng = |sel: fn(&RichSample) -> f32| -> Vec<f64> {
        rich.iter()
            .map(|r| r.as_ref().map_or(f64::NAN, |x| f64::from(sel(x))))
            .collect()
    };
    let ia_a = eng(|r| r.ia);
    let ib_a = eng(|r| r.ib);
    let ic_a = eng(|r| r.ic);
    let id_a = eng(|r| r.id);
    let iq_a = eng(|r| r.iq);
    let vbus_v = eng(|r| r.vbus_v);
    let vd_v = eng(|r| r.vd);
    let vq_v = eng(|r| r.vq);
    let angle_rad = eng(|r| r.angle_rad);
    let mech_rpm = eng(|r| r.mech_rpm);
    let erpm = eng(|r| r.erpm);

    let doubles: [&[f64]; 11] = [
        &ia_a, &ib_a, &ic_a, &id_a, &iq_a, &vbus_v, &vd_v, &vq_v, &angle_rad, &mech_rpm, &erpm,
    ];

    let mut rg = writer.next_row_group()?;
    let mut col_idx = 0usize;
    while let Some(mut col) = rg.next_column()? {
        match col_idx {
            0 => {
                col.typed::<DoubleType>().write_batch(&t_s, None, None)?;
            }
            1 => {
                col.typed::<Int64Type>().write_batch(&seq, None, None)?;
            }
            2 => {
                col.typed::<Int32Type>().write_batch(&ia, None, None)?;
            }
            3 => {
                col.typed::<Int32Type>().write_batch(&ib, None, None)?;
            }
            4 => {
                col.typed::<Int32Type>().write_batch(&ic, None, None)?;
            }
            5..=15 => {
                col.typed::<DoubleType>()
                    .write_batch(doubles[col_idx - 5], None, None)?;
            }
            _ => bail!("schema/column mismatch"),
        }
        col.close()?;
        col_idx += 1;
    }
    rg.close()?;
    writer.close()?;
    Ok(())
}

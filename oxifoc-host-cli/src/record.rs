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
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, bail};
use oxifoc_core::foc::telemetry::{EnrichCtx, RichSample};
use oxifoc_core::types::{CAP_PWM_TIMING_FROM_ARR, ControlTiming, FastTelemetry, HardwareInfo};
use oxifoc_host_lib::{HostRuntime, build_enrich_ctx};
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
    required double i_sum_a;
    required int32 va_adc;
    required int32 vb_adc;
    required int32 vc_adc;
    required int32 pwm_a_ccr;
    required int32 pwm_b_ccr;
    required int32 pwm_c_ccr;
    required int32 pwm_arr;
    required int32 pwm_enables;
    required double va_v;
    required double vb_v;
    required double vc_v;
    required double v_alpha_measured_v;
    required double v_beta_measured_v;
    required double vd_measured_v;
    required double vq_measured_v;
    required double hall_angle_rad;
}";

#[derive(serde::Serialize)]
pub struct RecordSummary {
    pub path: String,
    pub rows: usize,
    pub fast_hz_actual: u16,
    pub fast_hz_derived: f64,
    pub pwm_timing_segments: Vec<PwmTimingSegment>,
    pub decimation_m: u32,
    pub duration_s: f64,
    /// Number of places where seq jumped by more than M.
    pub gaps: usize,
    /// Total samples missing across all gaps.
    pub samples_lost: u64,
    pub fast_source: oxifoc_host_lib::FastSource,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub raw_telemetry: Option<oxifoc_host_lib::raw_telemetry::RawCaptureReport>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub velocity_debug_path: Option<String>,
}

#[derive(serde::Serialize)]
pub struct PwmTimingSegment {
    pub first_row: usize,
    pub first_cycle: u64,
    pub first_t_s: f64,
    pub control_timing: ControlTiming,
    pub fast_hz: f64,
}

/// Integrate wrapping FOC sequence deltas. A timer restart does not advance
/// the sequence while paused. Charge the interval ending at a rate change to
/// the previous segment, since the exact wall-clock boundary is unavailable.
fn sample_timing(
    samples: &[FastTelemetry],
    initial: ControlTiming,
    decimation: u32,
    use_arr: bool,
) -> (Vec<f64>, Vec<PwmTimingSegment>) {
    let mut times = Vec::with_capacity(samples.len());
    let mut segments = Vec::new();
    let mut timing = initial;
    let mut prev = samples.first().map_or(0, |sample| sample.seq);
    let mut cycles = 0_u64;
    let mut ticks = 0_u64;
    for (row, sample) in samples.iter().enumerate() {
        let delta = u64::from(sample.seq.wrapping_sub(prev));
        cycles += delta;
        ticks += delta * u64::from(timing.period_ticks);
        prev = sample.seq;
        let time = if initial.is_valid() {
            ticks as f64 / f64::from(initial.clock_hz)
        } else {
            f64::NAN
        };
        times.push(time);
        let period = if use_arr && sample.pwm_arr != 0 {
            2 * u32::from(sample.pwm_arr)
        } else {
            timing.period_ticks
        };
        if row == 0 || period != timing.period_ticks {
            timing.period_ticks = period;
            segments.push(PwmTimingSegment {
                first_row: row,
                first_cycle: cycles,
                first_t_s: time,
                control_timing: timing,
                fast_hz: f64::from(timing.clock_hz) / f64::from(period) / f64::from(decimation),
            });
        }
    }
    (times, segments)
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
    /// Sampled after the stream ACK, while calibration changes are blocked.
    pub board_calibration: oxifoc_core::types::ConfigSnapshot,
    pub hw: Option<HardwareInfo>,
    pub control_timing: ControlTiming,
    pub fast_hz_requested: u16,
    pub fast_hz_actual: u16,
    pub decimation_m: u32,
    pub samples: Vec<FastTelemetry>,
    pub started: Instant,
    /// Host-side raw→engineering context (device `BoardCalib` + `dc_offsets` +
    /// `pole_pairs`). `None` only when no handshake info was available.
    pub enrich: Option<EnrichCtx>,
    pub raw_telemetry: Option<Arc<oxifoc_host_lib::raw_telemetry::RawTelemetryState>>,
    pub raw_session: Option<u64>,
}

impl Capture {
    /// Enable streaming, wait for the device ack, eat the enable transient.
    pub fn start(runtime: &HostRuntime, fast_hz: u16) -> Result<Self> {
        let hw = latest_hw_info(runtime).context("hardware handshake unavailable for recording")?;
        Self::start_with_hw(runtime, fast_hz, hw)
    }

    /// Keep the same handshake used by callers to select the requested rate.
    /// The info receiver is a consuming queue, so it must not be drained twice.
    pub fn start_with_hw(runtime: &HostRuntime, fast_hz: u16, hw: HardwareInfo) -> Result<Self> {
        if fast_hz == 0 {
            bail!("fast_hz must be nonzero for a capture");
        }
        let hw = Some(hw);
        // Build the enrichment context up front (config reads) — while the link
        // is quiet, before the high-rate stream starts competing for it.
        let enrich = build_enrich_ctx(&runtime.cmd_tx, hw.as_ref());
        // Correlated ack: the oneshot answers for exactly THIS request. The
        // old poll of `runtime.fast_hz` accepted any nonzero value, so a
        // rate already active before the capture produced a silently wrong
        // decimation_m (and with it a wrong time axis and loss accounting).
        if runtime.raw_telemetry.is_some() {
            while runtime.fast_rx.try_recv().is_ok() {}
        }
        let ack = oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, fast_hz)?;
        let fast_hz_actual = ack.actual_fast_hz;
        if fast_hz_actual == 0
            || ack.decimation_m == 0
            || !ack.control_timing.is_valid()
            || ack.control_timing.stream_hz(ack.decimation_m) != fast_hz_actual
        {
            let _ = oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, 0);
            bail!("device acknowledged invalid capture timing: {ack:?}");
        }
        let board_calibration = match crate::config_cli::read_group(
            &runtime.cmd_tx,
            oxifoc_core::types::ConfigGroupId::BoardCalibration,
        ) {
            Ok(snapshot) => snapshot,
            Err(error) => {
                let _ = oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, 0);
                return Err(error).context("read active board calibration after stream ACK");
            }
        };
        let decimation_m = ack.decimation_m;
        let control_timing = ack.control_timing;

        // Warm-up: the enable transient (queue wrap before the device stream
        // task wakes) produces a guaranteed gap at the head of the stream.
        // Let it pass, then drain everything stale so the capture starts on
        // steady-state data only.
        let samples = if let Some(raw) = &runtime.raw_telemetry {
            // The raw arm owns a freshly cleared device queue and token. Keep
            // its prefix: sleeping here needlessly fills the finite host FIFO.
            raw.check()?;
            runtime.fast_rx.try_iter().collect()
        } else {
            std::thread::sleep(Duration::from_millis(150));
            while runtime.fast_rx.try_recv().is_ok() {}
            Vec::new()
        };
        let raw_session = runtime
            .raw_telemetry
            .as_ref()
            .and_then(|raw| raw.report())
            .map(|report| report.session);

        Ok(Self {
            board_calibration,
            hw,
            control_timing,
            fast_hz_requested: fast_hz,
            fast_hz_actual,
            decimation_m,
            samples,
            started: Instant::now(),
            enrich,
            raw_telemetry: runtime.raw_telemetry.clone(),
            raw_session,
        })
    }

    /// Collect samples until `deadline` (host clock).
    pub fn drain_until(&mut self, runtime: &HostRuntime, deadline: Instant) -> Result<()> {
        while Instant::now() < deadline {
            if let Some(raw) = &self.raw_telemetry {
                raw.check()?;
            }
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

    /// Disable streaming and wait for the device's acknowledgement.
    pub fn stop(&mut self, runtime: &HostRuntime) -> Result<()> {
        if let Some(raw) = &self.raw_telemetry
            && raw.report().map(|report| report.session) != self.raw_session
        {
            bail!("raw capture session changed before Stop");
        }
        oxifoc_host_lib::ops::telemetry::configure(&runtime.cmd_tx, 0)?;
        if let Some(raw) = &self.raw_telemetry {
            let session = self
                .raw_session
                .context("raw capture has no owned session")?;
            let deadline = Instant::now() + Duration::from_secs(2);
            loop {
                while let Ok(sample) = runtime.fast_rx.try_recv() {
                    self.samples.push(sample);
                }
                let report = raw
                    .report()
                    .context("raw capture disappeared while draining")?;
                if report.session != session
                    || report.error.is_some()
                    || report.ended
                    || Instant::now() >= deadline
                {
                    raw.wait_end_for(session, Duration::ZERO)?;
                    // END publication follows the final FIFO send.
                    while let Ok(sample) = runtime.fast_rx.try_recv() {
                        self.samples.push(sample);
                    }
                    break;
                }
                match runtime.fast_rx.recv_timeout(Duration::from_millis(5)) {
                    Ok(sample) => self.samples.push(sample),
                    Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                    Err(error) => return Err(error.into()),
                }
            }
        }
        Ok(())
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
        let mut config_snapshot = config_snapshot.clone();
        if let Some(obj) = config_snapshot.as_object_mut() {
            let active = self
                .board_calibration
                .value
                .as_ref()
                .and_then(crate::config_cli::group_value)
                .unwrap_or_else(|| serde_json::json!({ "vbus_gain": 1.0 }));
            obj.insert("board-calibration".to_string(), active);
        }
        let mut metadata = extra_meta.to_vec();
        let raw_report = self.raw_telemetry.as_ref().and_then(|raw| raw.report());
        let velocity_debug_path = raw_report
            .as_ref()
            .and_then(|report| report.velocity_debug.as_ref())
            .map(|_| format!("{out_path}.velocity.csv"));
        let fast_source = raw_report
            .as_ref()
            .map_or(oxifoc_host_lib::FastSource::Ergot, |raw| raw.source);
        metadata.push((
            "oxifoc.fast_source".into(),
            serde_json::to_string(&fast_source)?,
        ));
        if let Some(raw) = &raw_report {
            metadata.push(("oxifoc.raw_telemetry".into(), serde_json::to_string(raw)?));
        }
        if let Some(session) = self.raw_session {
            metadata.push(("oxifoc.raw_session".into(), session.to_string()));
        }
        if let Some(path) = &velocity_debug_path {
            metadata.push((
                "oxifoc.velocity_debug_csv".into(),
                std::path::Path::new(path)
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned(),
            ));
        }
        metadata.push((
            "oxifoc.board_calibration".to_string(),
            serde_json::to_string(&self.board_calibration)?,
        ));
        let pwm_timing_segments = write_parquet(
            out_path,
            &self.samples,
            self.hw.as_ref(),
            self.enrich.as_ref(),
            self.control_timing,
            self.fast_hz_requested,
            self.fast_hz_actual,
            self.expected_step(),
            &config_snapshot,
            gaps,
            samples_lost,
            &metadata,
        )?;
        // Preserve failed captures as diagnostic artifacts, but never report
        // success from a prefix whose producer END/totals were not consumed.
        if let Some(raw) = &self.raw_telemetry {
            let session = self
                .raw_session
                .context("raw capture has no owned session")?;
            if let Some(path) = &velocity_debug_path {
                let frames = raw
                    .velocity_frames(session)?
                    .context("missing velocity debug session")?;
                write_velocity_csv(
                    path,
                    &frames,
                    self.samples[0].seq,
                    self.expected_step(),
                    &pwm_timing_segments,
                )?;
            }
            raw.wait_end_for(session, Duration::ZERO)?
                .validate_export(session, self.samples.len())?;
        }
        Ok(RecordSummary {
            path: out_path.to_string(),
            rows: self.samples.len(),
            fast_hz_actual: self.fast_hz_actual,
            fast_hz_derived: f64::from(self.control_timing.clock_hz)
                / f64::from(self.control_timing.period_ticks)
                / f64::from(self.expected_step()),
            pwm_timing_segments,
            decimation_m: self.expected_step(),
            duration_s: self.started.elapsed().as_secs_f64(),
            gaps,
            samples_lost,
            fast_source,
            raw_telemetry: raw_report,
            velocity_debug_path,
        })
    }
}

fn write_velocity_csv(
    path: &str,
    frames: &[oxifoc_core::velocity_telemetry::VelocityDebugFrame],
    first_fast_seq: u16,
    stride: u32,
    timing_segments: &[PwmTimingSegment],
) -> Result<()> {
    use std::io::{BufWriter, Write};
    // The diagnostic origin is produced before native publish in the same ISR.
    // A decimated stream's first frame may follow it by at most one stride.
    let offset = if let Some(origin) = frames.first() {
        let delta = u32::from(first_fast_seq.wrapping_sub(origin.sequence as u16));
        if delta > stride {
            bail!("velocity debug origin does not align with first fast sample");
        }
        delta
    } else {
        0
    };
    let mut file = BufWriter::new(File::create(path)?);
    writeln!(
        file,
        "t_s,foc_sequence,tick_us,target_rad_s,reference_rad_s,feedback_rad_s,requested_iq_a,granted_iq_a,granted_id_a,integral_iq_a,iq_min_a,iq_max_a,hall_raw,hall_age_us,hall_direction,flags,current_control,speed_limit,modulation_limit,temperature_limit,bus_voltage_limit,bus_current_limit,phase_current_limit,angle_limit,startup_shaping"
    )?;
    for frame in frames {
        let delta = frame.sequence.wrapping_sub(frames[0].sequence);
        let cycle = i64::from(delta) - i64::from(offset);
        let index = timing_segments
            .partition_point(|segment| segment.first_cycle as i64 <= cycle)
            .saturating_sub(1);
        let segment = &timing_segments[index];
        let dt = f64::from(segment.control_timing.period_ticks)
            / f64::from(segment.control_timing.clock_hz);
        let time = segment.first_t_s + (cycle - segment.first_cycle as i64) as f64 * dt;
        let speed = |word| f64::from(word) / 8.0;
        let current = |word| f64::from(word) / 100.0;
        use oxifoc_core::velocity_telemetry as trace;
        let current_control = frame.flags & trace::CURRENT_CONTROL != 0;
        let limited = |flag| u8::from(current_control && frame.flags & flag != 0);
        writeln!(
            file,
            "{time:.9},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            frame.sequence,
            frame.tick_us,
            speed(frame.target),
            speed(frame.reference),
            speed(frame.feedback),
            current(frame.requested_iq),
            current(frame.granted_iq),
            current(frame.granted_id),
            current(frame.integral),
            current(frame.iq_min),
            current(frame.iq_max),
            frame.hall_raw,
            u32::from(frame.hall_age_100us) * 100,
            frame.hall_direction,
            frame.flags,
            u8::from(current_control),
            limited(trace::LIMIT_SPEED),
            limited(trace::LIMIT_MODULATION),
            limited(trace::LIMIT_TEMPERATURE),
            limited(trace::LIMIT_BUS_VOLTAGE),
            limited(trace::LIMIT_BUS_CURRENT),
            limited(trace::LIMIT_PHASE_CURRENT),
            limited(trace::LIMIT_ANGLE),
            limited(trace::STARTUP_CURRENT_SHAPING),
        )?;
    }
    file.flush()?;
    Ok(())
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
    let stopped = cap.stop(runtime);
    let finished = cap.finish(out_path, &config_snapshot, &[]);
    drained?;
    stopped?;
    finished
}

#[allow(clippy::too_many_arguments)]
fn write_parquet(
    path: &str,
    samples: &[FastTelemetry],
    hw: Option<&HardwareInfo>,
    enrich: Option<&EnrichCtx>,
    control_timing: ControlTiming,
    fast_hz_requested: u16,
    fast_hz_actual: u16,
    decimation_m: u32,
    config_snapshot: &serde_json::Value,
    gaps: usize,
    samples_lost: u64,
    extra_meta: &[(String, String)],
) -> Result<Vec<PwmTimingSegment>> {
    let use_arr = hw.is_some_and(|h| h.capabilities & CAP_PWM_TIMING_FROM_ARR != 0);
    let (t_s, timing_segments) = sample_timing(samples, control_timing, decimation_m, use_arr);
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
        kv("oxifoc.control_timing", serde_json::to_string(&control_timing)?),
        kv("oxifoc.pwm_timing_segments", serde_json::to_string(&timing_segments)?),
        kv("oxifoc.foc_hz_derived", (f64::from(control_timing.clock_hz)
            / f64::from(control_timing.period_ticks)).to_string()),
        kv("oxifoc.fast_hz_derived", (f64::from(control_timing.clock_hz)
            / f64::from(control_timing.period_ticks) / f64::from(decimation_m)).to_string()),
        kv(
            "oxifoc.telemetry_note",
            "ia_adc/ib_adc/ic_adc = uncalibrated 12-bit ADC counts (valid channels \
             declared by oxifoc.current_sense_topology, NO \
             anti-alias filter — decimation is plain sample-dropping at M). The \
             *_a / *_v / angle_rad / *rpm columns are reconstructed host-side by \
             oxifoc_core FastTelemetry::enrich (declared current topology + BoardCalib + dc_offsets + \
             pole_pairs) — the same core code the firmware encodes with. NaN only \
             when the device handshake carried no BoardCalib at all; if the \
             DcOffsets/MotorParams config reads failed, enrichment silently falls \
             back to MID-SCALE offsets / pole_pairs=0 and the columns are wrong \
             but non-NaN — cross-check the dc-offsets group in oxifoc.config \
             below (missing there = the fallback was used). i_sum_a is the \
             unprojected sum of three measured currents; NaN for TwoShunt \
             or unavailable enrichment. va_adc/vb_adc/vc_adc are raw phase-terminal \
             counts (65535 = absent); phase scaling and nominal acquisition offset \
             are in oxifoc.phase_voltage_info. Measured voltage columns are not \
             delay-corrected or guaranteed PWM-averaged on unfiltered boards. \
             pwm_* are timer registers sampled BEFORE the controller update; \
             ARR=0 means unavailable. pwm_enables bits 0..2 are high-side A/B/C, \
             3..5 low-side A/B/C, bit 6 main enable. CCR may be a preload register; \
             it is not a gate-waveform measurement. vd_v/vq_v are PI commands \
             before compensation, not ADC-measured applied voltages. The top-level \
             rates describe stream arming; pwm_timing_segments records actual rates \
             by row. Sequence-derived t_s excludes any pause during timer restart."
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
        meta.push(kv("oxifoc.board_calib", serde_json::to_string(&h.calib)?));
        meta.push(kv(
            "oxifoc.phase_voltage_info",
            serde_json::to_string(&h.phase_voltage_info())?,
        ));
        meta.push(kv("oxifoc.protocol_version", h.proto_version.to_string()));
        meta.push(kv("oxifoc.capabilities", h.capabilities.to_string()));
        meta.push(kv(
            "oxifoc.current_sense_topology",
            format!(
                "{:?}",
                oxifoc_core::types::CurrentSenseTopology::from_capabilities(h.capabilities)
            ),
        ));
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

    // Keep the raw sequence for maneuver events that anchor to its value.
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
    let hall_angle_rad = eng(|r| r.hall_angle_rad.unwrap_or(f32::NAN));
    let mech_rpm = eng(|r| r.mech_rpm);
    let erpm = eng(|r| r.erpm);
    let i_sum_a = eng(|r| r.i_sum.unwrap_or(f32::NAN));
    let phase_v: [Vec<f64>; 3] = core::array::from_fn(|i| {
        rich.iter()
            .map(|r| {
                r.as_ref()
                    .and_then(|r| r.phase_voltage_v)
                    .map_or(f64::NAN, |v| f64::from(v[i]))
            })
            .collect()
    });
    let measured_abd: [Vec<f64>; 4] = [
        eng(|r| r.v_alpha_measured.unwrap_or(f32::NAN)),
        eng(|r| r.v_beta_measured.unwrap_or(f32::NAN)),
        eng(|r| r.vd_measured.unwrap_or(f32::NAN)),
        eng(|r| r.vq_measured.unwrap_or(f32::NAN)),
    ];
    let raw_capture: [Vec<i32>; 8] = core::array::from_fn(|i| {
        samples
            .iter()
            .map(|s| {
                i32::from(match i {
                    0..=2 => s.vphase_adc[i],
                    3..=5 => s.pwm_ccr[i - 3],
                    6 => s.pwm_arr,
                    _ => s.pwm_enables,
                })
            })
            .collect()
    });

    let doubles: [&[f64]; 12] = [
        &ia_a, &ib_a, &ic_a, &id_a, &iq_a, &vbus_v, &vd_v, &vq_v, &angle_rad, &mech_rpm, &erpm,
        &i_sum_a,
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
            5..=16 => {
                col.typed::<DoubleType>()
                    .write_batch(doubles[col_idx - 5], None, None)?;
            }
            17..=24 => {
                col.typed::<Int32Type>()
                    .write_batch(&raw_capture[col_idx - 17], None, None)?;
            }
            25..=27 => {
                col.typed::<DoubleType>()
                    .write_batch(&phase_v[col_idx - 25], None, None)?;
            }
            28..=31 => {
                col.typed::<DoubleType>()
                    .write_batch(&measured_abd[col_idx - 28], None, None)?;
            }
            32 => {
                col.typed::<DoubleType>()
                    .write_batch(&hall_angle_rad, None, None)?;
            }
            _ => bail!("schema/column mismatch"),
        }
        col.close()?;
        col_idx += 1;
    }
    rg.close()?;
    writer.close()?;
    Ok(timing_segments)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::storage::BoardCalibrationConfig;
    use oxifoc_core::types::{ConfigGroupId, ConfigSnapshot, ConfigValue};
    use parquet::file::reader::{FileReader, SerializedFileReader};
    use parquet::record::RowAccessor;

    #[test]
    fn pwm_timing_segments_preserve_sequence_wrap_and_rate_changes() {
        let drive = ControlTiming {
            clock_hz: 168_000_000,
            period_ticks: 11_200,
        };
        let samples =
            [(65532, 5600), (0, 4200), (4, 4200), (8, 5600), (12, 5600)].map(|(seq, pwm_arr)| {
                FastTelemetry {
                    seq,
                    pwm_arr,
                    ..Default::default()
                }
            });
        let (times, segments) = sample_timing(&samples, drive, 4, true);
        for (time, ticks) in times.into_iter().zip([0, 44_800, 78_400, 112_000, 156_800]) {
            assert!((time - f64::from(ticks) / f64::from(drive.clock_hz)).abs() < 1e-14);
        }
        assert_eq!(segments.len(), 3);
        assert_eq!(segments[1].first_row, 1);
        assert_eq!(segments[1].first_cycle, 4);
        assert_eq!(segments[1].control_timing.period_ticks, 8400);
        assert_eq!(segments[1].fast_hz, 5000.0);
        assert_eq!(segments[2].first_row, 3);
        assert_eq!(segments[2].control_timing, drive);
        assert_eq!(sample_timing(&samples, drive, 4, false).1.len(), 1);
    }

    #[test]
    fn parquet_records_projection_and_unprojected_current_residual() {
        use oxifoc_core::types::{BoardCalib, CurrentSenseTopology};
        let mut hw = HardwareInfo {
            capabilities: CurrentSenseTopology::ThreeShunt.capability_bits(),
            foc_freq_hz: 20_000,
            calib: BoardCalib {
                shunt_ohms: 0.002,
                amp_gain: 1.0,
                adc_vref_mv: 1000,
                adc_max_counts: 1000,
                invert_current_sign: false,
                vbus_divider_ratio: 1.0,
            },
            ..Default::default()
        };
        hw.set_phase_voltage_info(oxifoc_core::types::PhaseVoltageInfo {
            divider_ratio: 20.0,
            current_to_voltage_ns: 2571,
            filtered: true,
        });
        let ctx = EnrichCtx::new(
            &hw.calib,
            (500.0, 500.0, 500.0),
            0,
            CurrentSenseTopology::ThreeShunt,
        )
        .with_phase_voltage(&hw.calib, hw.phase_voltage_info());
        let frame = FastTelemetry {
            ia: 510,
            ib: 501,
            ic: 501,
            vphase_adc: [600, 300, 300],
            pwm_ccr: [1000, 2000, 3000],
            pwm_arr: 4200,
            pwm_enables: 0x7f,
            ..Default::default()
        };
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "three-shunt-{}-{nonce}.parquet",
            std::process::id()
        ));
        write_parquet(
            path.to_str().unwrap(),
            &[frame],
            Some(&hw),
            Some(&ctx),
            ControlTiming::from_hz(20_000),
            20_000,
            20_000,
            1,
            &serde_json::json!({}),
            0,
            0,
            &[],
        )
        .unwrap();
        let reader = SerializedFileReader::new(File::open(&path).unwrap()).unwrap();
        let metadata = reader
            .metadata()
            .file_metadata()
            .key_value_metadata()
            .unwrap();
        let value = |name| {
            metadata
                .iter()
                .find(|kv| kv.key == name)
                .unwrap()
                .value
                .as_deref()
                .unwrap()
        };
        assert_eq!(value("oxifoc.current_sense_topology"), "ThreeShunt");
        let phase_info: serde_json::Value =
            serde_json::from_str(value("oxifoc.phase_voltage_info")).unwrap();
        assert_eq!(phase_info["current_to_voltage_ns"], 2571);
        assert_eq!(phase_info["divider_ratio"], 20.0);
        assert!(value("oxifoc.board_calib").contains("adc_vref_mv"));
        assert_eq!(
            value("oxifoc.protocol_version"),
            hw.proto_version.to_string()
        );
        let row = reader.get_row_iter(None).unwrap().next().unwrap().unwrap();
        assert!((row.get_double(8).unwrap() - 3.0).abs() < 1e-5); // Id
        assert_eq!(row.get_double(9).unwrap(), 0.0); // Iq
        assert!((row.get_double(16).unwrap() - 6.0).abs() < 1e-5); // Raw sum
        assert_eq!(row.get_int(17).unwrap(), 600); // Phase ADC, no scaling.
        assert_eq!(row.get_int(20).unwrap(), 1000); // Pre-update CCR A.
        assert_eq!(row.get_int(23).unwrap(), 4200); // ARR.
        assert_eq!(row.get_int(24).unwrap(), 0x7f); // Output enables.
        assert!((row.get_double(25).unwrap() - 12.0).abs() < 1e-5); // Phase A volts.
        assert!((row.get_double(30).unwrap() - 4.0).abs() < 1e-5); // Measured Vd.
        drop(reader);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn parquet_time_uses_timer_ratio_across_sequence_wrap_and_decimation() {
        let timing = ControlTiming {
            clock_hz: 167_835_842,
            period_ticks: 8390,
        };
        let frames: Vec<_> = [65532, 0, 4, 8]
            .into_iter()
            .map(|seq| FastTelemetry {
                seq,
                ..Default::default()
            })
            .collect();
        let hw = HardwareInfo {
            foc_freq_hz: 20_000,
            ..Default::default()
        };
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "timer-ratio-{}-{nonce}.parquet",
            std::process::id()
        ));
        write_parquet(
            path.to_str().unwrap(),
            &frames,
            Some(&hw),
            None,
            timing,
            5000,
            timing.stream_hz(4),
            4,
            &serde_json::json!({}),
            0,
            0,
            &[],
        )
        .unwrap();
        let reader = SerializedFileReader::new(File::open(&path).unwrap()).unwrap();
        for (row, cycles) in reader.get_row_iter(None).unwrap().zip([0, 4, 8, 12]) {
            let expected = f64::from(cycles) * 8390.0 / 167_835_842.0;
            assert!((row.unwrap().get_double(0).unwrap() - expected).abs() < 1e-14);
        }
        let meta = reader
            .metadata()
            .file_metadata()
            .key_value_metadata()
            .unwrap();
        let saved: ControlTiming = serde_json::from_str(
            meta.iter()
                .find(|x| x.key == "oxifoc.control_timing")
                .unwrap()
                .value
                .as_ref()
                .unwrap(),
        )
        .unwrap();
        assert_eq!(saved, timing);
        drop(reader);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn capture_metadata_uses_calibration_read_after_stream_ack() {
        let gain = BoardCalibrationConfig { vbus_gain: 1.01 };
        let capture = Capture {
            board_calibration: ConfigSnapshot {
                effective: None,
                default: None,
                boot_epoch: 0,
                group: ConfigGroupId::BoardCalibration,
                revision: 7,
                persisted: true,
                valid: true,
                value: Some(ConfigValue::BoardCalibration(gain)),
            },
            hw: None,
            control_timing: ControlTiming::from_hz(1000),
            fast_hz_requested: 500,
            fast_hz_actual: 500,
            decimation_m: 2,
            samples: vec![
                FastTelemetry {
                    seq: 2,
                    ..Default::default()
                },
                FastTelemetry {
                    seq: 4,
                    ..Default::default()
                },
            ],
            started: Instant::now(),
            enrich: None,
            raw_telemetry: None,
            raw_session: None,
        };
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "board-calibration-{}-{nonce}.parquet",
            std::process::id()
        ));
        capture
            .finish(
                path.to_str().unwrap(),
                &serde_json::json!({"board-calibration":{"vbus_gain": 1.0}}),
                &[],
            )
            .unwrap();
        let reader = SerializedFileReader::new(File::open(&path).unwrap()).unwrap();
        let meta = reader
            .metadata()
            .file_metadata()
            .key_value_metadata()
            .unwrap();
        let get = |key| {
            meta.iter()
                .find(|kv| kv.key == key)
                .unwrap()
                .value
                .as_ref()
                .unwrap()
        };
        let config: serde_json::Value = serde_json::from_str(get("oxifoc.config")).unwrap();
        assert_eq!(
            config["board-calibration"]["vbus_gain"],
            serde_json::json!(gain.vbus_gain)
        );
        let active: ConfigSnapshot = serde_json::from_str(get("oxifoc.board_calibration")).unwrap();
        assert_eq!(active, capture.board_calibration);
        drop(reader);
        std::fs::remove_file(path).unwrap();
    }
}

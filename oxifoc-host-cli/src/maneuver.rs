//! Maneuver — a scripted experiment protocol (flight-test sense): a flat,
//! timed sequence of control commands executed against the device while
//! fast telemetry records into a parquet file.
//!
//! The point is reproducibility and sim/hardware diffing: the same maneuver
//! file runs against `oxifoc-virtual` and against the real board, producing
//! A/B captures of the identical input profile. The parquet metadata embeds
//! the maneuver itself plus an **event log** where every command carries the
//! raw device `seq` of the last sample seen before send and after the ack —
//! analysis cuts epochs by `seq`, not by wall-clock guesswork.
//!
//! Guarantees:
//! - the terminal command (default `stop`) is sent on EVERY exit path of the
//!   timeline, including errors. (A killed process is covered by the device
//!   deadman/failsafe, not by this runner.)
//! - the timeline is validated (monotonic t, finite values) and checked
//!   against the device's stored current limits before anything is sent;
//!   `--force` bypasses the limits check only.
//!
//! File format (JSON):
//! ```json
//! {
//!   "name": "iq-step-2A",
//!   "description": "current-loop step response",
//!   "capture": { "fast_hz": 5000, "tail_s": 1.0 },
//!   "terminal": "stop",
//!   "timeline": [
//!     { "t": 0.5, "cmd": { "start": { "iq": 2.0 } } },
//!     { "t": 2.5, "cmd": { "start": { "iq": 0.0 } } }
//!   ]
//! }
//! ```

use oxifoc_core::types::ConfigGroupId;

use crate::config_cli::current_value;
use crate::record::RecordSummary;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{ControlMode, FastTelemetry, MotorStatus};
use oxifoc_host_lib::{HostCommand, HostRuntime, MotorResponseReceiver, motor_channel};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::record::Capture;

const SCHEDULE_TOLERANCE: Duration = Duration::from_millis(5);
const COMMAND_ACK_TIMEOUT: Duration = Duration::from_millis(200);

/// Keep consuming the bounded telemetry channel while a command is pending.
/// Blocking on the acknowledgement alone discards samples after ~200 ms at
/// 20 kHz. Never retry an excitation and silently stretch its timeline.
fn wait_motor_reply(
    cap: &mut Capture,
    fast_rx: &crossbeam_channel::Receiver<FastTelemetry>,
    reply: &mut MotorResponseReceiver,
    deadline: Instant,
) -> Result<MotorStatus> {
    loop {
        let now = Instant::now();
        if now >= deadline {
            bail!("motor acknowledgement missed the maneuver deadline");
        }
        match reply.try_recv() {
            Ok(status) => {
                // Drain only the samples already queued at acknowledgement,
                // so a continuously producing stream cannot delay Stop.
                for _ in 0..fast_rx.len() {
                    if let Ok(s) = fast_rx.try_recv() {
                        cap.samples.push(s);
                    }
                }
                return status;
            }
            Err(_) if reply.is_terminated() => bail!("backend dropped the motor response"),
            Err(_) => {}
        }
        let timeout = deadline
            .saturating_duration_since(now)
            .min(Duration::from_millis(1));
        match fast_rx.recv_timeout(timeout) {
            Ok(s) => cap.samples.push(s),
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                bail!("telemetry channel disconnected while awaiting motor response");
            }
        }
    }
}

fn send_motor_captured(
    runtime: &HostRuntime,
    cap: &mut Capture,
    mode: ControlMode,
    deadline: Instant,
) -> Result<MotorStatus> {
    let (tx, mut reply) = motor_channel();
    runtime
        .cmd_tx
        .send(HostCommand::MotorAck(mode, tx))
        .context("send motor command")?;
    wait_motor_reply(cap, &runtime.fast_rx, &mut reply, deadline)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Maneuver {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub capture: CaptureCfg,
    /// Sent after the timeline + tail on every exit path.
    #[serde(default)]
    pub terminal: Terminal,
    pub timeline: Vec<Step>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CaptureCfg {
    pub fast_hz: u16,
    /// Seconds of capture after the last timeline command.
    #[serde(default)]
    pub tail_s: f64,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Terminal {
    #[default]
    Stop,
    Coast,
    Brake,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Step {
    /// Seconds from capture start.
    pub t: f64,
    pub cmd: Cmd,
}

/// One timeline command — the motor-facing subset of the CLI surface.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Cmd {
    Start {
        iq: f32,
        #[serde(default)]
        id: f32,
    },
    Velocity {
        rad_s: f32,
    },
    Openloop {
        current: f32,
        #[serde(default)]
        velocity: f32,
        #[serde(default)]
        angle: f32,
        /// Runtime PI override recorded with the experiment; no flash write.
        /// None retains the current gains, as on the device control mode.
        #[serde(default)]
        pi_gains: Option<(f32, f32)>,
    },
    Voltage {
        #[serde(default)]
        vd: f32,
        #[serde(default)]
        vq: f32,
        #[serde(default)]
        angle: f32,
    },
    Stop {},
    Coast {},
    Brake {},
}

impl Cmd {
    fn to_mode(&self) -> ControlMode {
        match *self {
            Self::Start { iq, id } => ControlMode::CurrentControl {
                iq_target: iq,
                id_target: id,
            },
            Self::Velocity { rad_s } => ControlMode::VelocityControl { target_vel: rad_s },
            Self::Openloop {
                current,
                velocity,
                angle,
                pi_gains,
            } => ControlMode::OpenLoop {
                angle_rad: angle,
                current,
                velocity_rad_s: velocity,
                pi_gains,
            },
            Self::Voltage { vd, vq, angle } => ControlMode::DirectVoltage {
                vd,
                vq,
                angle_rad: angle,
            },
            Self::Stop {} => ControlMode::Stopped,
            Self::Coast {} => ControlMode::Coast,
            Self::Brake {} => ControlMode::Brake,
        }
    }

    /// Peak phase-ish current this command can demand, for the limits check
    /// (None = not expressible in amps, e.g. voltage/duty modes).
    fn current_demand(&self) -> Option<f32> {
        match *self {
            Self::Start { iq, id } => Some((iq * iq + id * id).sqrt()),
            Self::Openloop { current, .. } => Some(current.abs()),
            _ => None,
        }
    }
}

impl Terminal {
    fn to_mode(self) -> ControlMode {
        match self {
            Self::Stop => ControlMode::Stopped,
            Self::Coast => ControlMode::Coast,
            Self::Brake => ControlMode::Brake,
        }
    }
}

pub fn load(path: &str) -> Result<Maneuver> {
    let text = std::fs::read_to_string(path).with_context(|| format!("read {path}"))?;
    let m: Maneuver =
        serde_json::from_str(&text).with_context(|| format!("parse maneuver {path}"))?;
    validate(&m)?;
    Ok(m)
}

/// Offline (schema-level) validation: monotonic timeline, finite values.
pub fn validate(m: &Maneuver) -> Result<()> {
    if m.timeline.is_empty() {
        bail!("timeline is empty");
    }
    if m.capture.fast_hz == 0 {
        bail!("capture.fast_hz must be nonzero");
    }
    if !m.capture.tail_s.is_finite() || m.capture.tail_s < 0.0 {
        bail!("capture.tail_s must be finite and ≥ 0");
    }
    let mut prev = 0.0f64;
    for (i, step) in m.timeline.iter().enumerate() {
        if !step.t.is_finite() || step.t < 0.0 {
            bail!("step {i}: t must be finite and ≥ 0");
        }
        if step.t < prev {
            bail!(
                "step {i}: t={} goes backwards (previous {prev}) — the timeline must be sorted",
                step.t
            );
        }
        prev = step.t;
        let mode = step.cmd.to_mode();
        if !mode.is_finite() {
            bail!("step {i}: non-finite command payload");
        }
    }
    Ok(())
}

/// Online check against the device's stored current limits.
fn check_limits(runtime: &HostRuntime, m: &Maneuver) -> Result<()> {
    let (limits, _stored) = current_value(&runtime.cmd_tx, ConfigGroupId::CurrentLimits)?;
    let max_iq = limits
        .get("max_iq_a")
        .and_then(Value::as_f64)
        .unwrap_or(0.0) as f32;
    if max_iq <= 0.0 {
        return Ok(());
    }
    for (i, step) in m.timeline.iter().enumerate() {
        if let Some(demand) = step.cmd.current_demand()
            && demand > max_iq
        {
            bail!(
                "step {i} demands {demand:.1} A > device max_iq_a {max_iq:.1} A \
                 (config set current-limits, or --force to let the device clamp)"
            );
        }
    }
    Ok(())
}

/// One executed timeline event, embedded into the parquet metadata
/// (`oxifoc.events`) — `seq_before`/`seq_after_ack` are the raw device seq
/// anchors for epoch cutting.
#[derive(Serialize)]
pub struct EventRecord {
    pub i: usize,
    pub cmd: Value,
    pub t_planned: f64,
    /// Host time of send, seconds since capture start.
    pub t_sent: f64,
    /// Host time the device ack arrived.
    pub t_acked: f64,
    /// seq of the last telemetry sample seen before the command was sent.
    pub seq_before: Option<u32>,
    /// seq of the last sample seen right after the ack.
    pub seq_after_ack: Option<u32>,
    pub ok: bool,
    pub response: String,
}

#[derive(Serialize)]
pub struct ManeuverSummary {
    pub maneuver: String,
    pub events: Vec<EventRecord>,
    pub terminal_ok: bool,
    pub record: RecordSummary,
}

pub fn run(
    runtime: &HostRuntime,
    m: &Maneuver,
    out_path: &str,
    force: bool,
    config_snapshot: Value,
) -> Result<ManeuverSummary> {
    if !force {
        check_limits(runtime, m)?;
    }

    let mut cap = Capture::start(runtime, m.capture.fast_hz)?;
    let t0 = cap.started;

    // Execute the timeline; the closure form keeps every error path flowing
    // into the terminal command below.
    let mut events: Vec<EventRecord> = Vec::new();
    let exec: Result<()> = (|| {
        for (i, step) in m.timeline.iter().enumerate() {
            let event_at = t0 + Duration::from_secs_f64(step.t);
            cap.drain_until(runtime, event_at)?;
            if Instant::now().saturating_duration_since(event_at) > SCHEDULE_TOLERANCE {
                bail!("step {i}: maneuver is behind schedule; refusing a late command");
            }

            let seq_before = cap.last_seq();
            let t_sent = t0.elapsed().as_secs_f64();
            let next_slot = m
                .timeline
                .get(i + 1)
                .map(|s| t0 + Duration::from_secs_f64(s.t) + SCHEDULE_TOLERANCE);
            let deadline = (event_at + COMMAND_ACK_TIMEOUT)
                .min(next_slot.unwrap_or(event_at + COMMAND_ACK_TIMEOUT));
            let res = send_motor_captured(runtime, &mut cap, step.cmd.to_mode(), deadline);
            let t_acked = t0.elapsed().as_secs_f64();
            // A couple of frames may have queued during the ack round-trip;
            // pull them in so seq_after_ack is fresh.
            for _ in 0..runtime.fast_rx.len() {
                if let Ok(s) = runtime.fast_rx.try_recv() {
                    cap.samples.push(s);
                }
            }
            let seq_after_ack = cap.last_seq();

            let (ok, response) = match &res {
                Ok(status) => (true, format!("{status:?}")),
                Err(e) => (false, format!("{e:#}")),
            };
            events.push(EventRecord {
                i,
                cmd: serde_json::to_value(&step.cmd)?,
                t_planned: step.t,
                t_sent,
                t_acked,
                seq_before,
                seq_after_ack,
                ok,
                response,
            });
            res.map(|_| ())
                .with_context(|| format!("step {i} ({:?}) failed", step.cmd))?;
        }
        // Tail capture after the last command.
        let tail_end = t0
            + Duration::from_secs_f64(m.timeline.last().map(|s| s.t).unwrap_or(0.0))
            + Duration::from_secs_f64(m.capture.tail_s);
        cap.drain_until(runtime, tail_end)?;
        Ok(())
    })();

    // Terminal command on EVERY path; capture a short grace window so the
    // stop itself lands in the file.
    let terminal_res = send_motor_captured(
        runtime,
        &mut cap,
        m.terminal.to_mode(),
        Instant::now() + Duration::from_secs(2),
    );
    let grace = cap.drain_until(runtime, Instant::now() + Duration::from_millis(200));
    let stream_stopped = cap.stop(runtime);
    // If the terminal response was lost under stream load, retry a safety
    // stop after quiescing telemetry. Keep the original failure in the file.
    let terminal_res = terminal_res.map_err(|error| {
        let recovery = crate::send_emergency_stop(runtime);
        error.context(format!(
            "terminal failed; emergency stop after stream shutdown: {recovery:?}"
        ))
    });

    finish_run(
        &cap,
        m,
        out_path,
        &config_snapshot,
        events,
        [
            ("execution", exec),
            ("terminal", terminal_res.map(|_| ())),
            ("terminal_capture", grace),
            ("stream_stop", stream_stopped),
        ],
    )
}

/// Save evidence before propagating operational failures, including failures
/// after the timeline. A failed terminal command must not discard the capture.
fn finish_run(
    cap: &Capture,
    m: &Maneuver,
    out_path: &str,
    config_snapshot: &Value,
    events: Vec<EventRecord>,
    outcomes: [(&str, Result<()>); 4],
) -> Result<ManeuverSummary> {
    let terminal_ok = outcomes[1].1.is_ok();
    let errors: serde_json::Map<String, Value> = outcomes
        .into_iter()
        .filter_map(|(stage, result)| {
            result
                .err()
                .map(|e| (stage.to_string(), json!(format!("{e:#}"))))
        })
        .collect();
    let extra_meta = vec![
        (
            "oxifoc.maneuver".to_string(),
            serde_json::to_string(m).unwrap_or_default(),
        ),
        (
            "oxifoc.events".to_string(),
            serde_json::to_string(&events).unwrap_or_default(),
        ),
        (
            "oxifoc.maneuver_errors".to_string(),
            json!(errors).to_string(),
        ),
    ];
    let record = cap
        .finish(out_path, config_snapshot, &extra_meta)
        .with_context(|| {
            format!("save maneuver capture to {out_path}; operational errors: {errors:?}")
        })?;
    if !errors.is_empty() {
        bail!("maneuver failed: {errors:?}; capture saved to {out_path}");
    }

    Ok(ManeuverSummary {
        maneuver: m.name.clone(),
        events,
        terminal_ok,
        record,
    })
}

pub fn summary_json(s: &ManeuverSummary) -> Value {
    json!({
        "maneuver": s.maneuver,
        "events": s.events,
        "terminal_ok": s.terminal_ok,
        "record": s.record,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::types::{ConfigSnapshot, FastTelemetry};
    use parquet::file::reader::{FileReader, SerializedFileReader};

    fn test_capture() -> Capture {
        Capture {
            raw_telemetry: None,
            raw_session: None,
            board_calibration: ConfigSnapshot {
                group: ConfigGroupId::BoardCalibration,
                revision: 0,
                persisted: false,
                valid: false,
                value: None,
            },
            hw: None,
            control_timing: oxifoc_core::types::ControlTiming::from_hz(20_000),
            fast_hz_requested: 20_000,
            fast_hz_actual: 20_000,
            decimation_m: 1,
            samples: Vec::new(),
            started: Instant::now(),
            enrich: None,
        }
    }

    #[test]
    fn pending_command_drains_telemetry_beyond_channel_capacity() {
        let mut cap = test_capture();
        let (fast_tx, fast_rx) = crossbeam_channel::bounded(4);
        let (reply_tx, mut reply_rx) = motor_channel();
        let producer = std::thread::spawn(move || {
            for seq in 0..2000 {
                fast_tx
                    .send(FastTelemetry {
                        seq,
                        ..Default::default()
                    })
                    .unwrap();
            }
            reply_tx.send(Ok(MotorStatus::default())).unwrap();
            fast_tx
        });
        wait_motor_reply(
            &mut cap,
            &fast_rx,
            &mut reply_rx,
            Instant::now() + Duration::from_secs(5),
        )
        .unwrap();
        let _fast_tx = producer.join().unwrap();
        assert_eq!(cap.samples.len(), 2000);
        assert!(
            cap.samples
                .iter()
                .enumerate()
                .all(|(i, s)| usize::from(s.seq) == i)
        );
    }

    #[test]
    fn missing_command_response_exits_at_deadline_and_keeps_samples() {
        let mut cap = test_capture();
        let (fast_tx, fast_rx) = crossbeam_channel::bounded(4);
        fast_tx.send(FastTelemetry::default()).unwrap();
        let (_reply_tx, mut reply_rx) = motor_channel();
        let error = wait_motor_reply(
            &mut cap,
            &fast_rx,
            &mut reply_rx,
            Instant::now() + Duration::from_millis(10),
        )
        .unwrap_err();
        assert!(error.to_string().contains("deadline"));
        assert_eq!(cap.samples.len(), 1);
    }

    #[test]
    fn failed_maneuver_preserves_capture_and_all_failure_stages() {
        let m = Maneuver {
            name: "failed-capture".to_string(),
            description: String::new(),
            capture: CaptureCfg {
                fast_hz: 20_000,
                tail_s: 0.0,
            },
            terminal: Terminal::Stop,
            timeline: vec![Step {
                t: 0.0,
                cmd: Cmd::Stop {},
            }],
        };
        let mut cap = test_capture();
        cap.samples.push(FastTelemetry::default());
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "failed-maneuver-{}-{nonce}.parquet",
            std::process::id()
        ));
        let error = finish_run(
            &cap,
            &m,
            path.to_str().unwrap(),
            &json!({}),
            Vec::new(),
            [
                ("execution", Err(anyhow::anyhow!("command rejected"))),
                ("terminal", Err(anyhow::anyhow!("stop timeout"))),
                ("terminal_capture", Ok(())),
                ("stream_stop", Err(anyhow::anyhow!("link disconnected"))),
            ],
        )
        .err()
        .unwrap();
        assert!(error.to_string().contains("capture saved to"));
        let reader = SerializedFileReader::new(std::fs::File::open(&path).unwrap()).unwrap();
        assert_eq!(reader.metadata().file_metadata().num_rows(), 1);
        let meta = reader
            .metadata()
            .file_metadata()
            .key_value_metadata()
            .unwrap();
        let errors: Value = serde_json::from_str(
            meta.iter()
                .find(|kv| kv.key == "oxifoc.maneuver_errors")
                .unwrap()
                .value
                .as_deref()
                .unwrap(),
        )
        .unwrap();
        assert_eq!(errors["execution"], "command rejected");
        assert_eq!(errors["terminal"], "stop timeout");
        assert_eq!(errors["stream_stop"], "link disconnected");
        assert!(errors.get("terminal_capture").is_none());
        drop(reader);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn openloop_pi_override_is_optional_and_preserved_in_the_device_command() {
        let old: Cmd = serde_json::from_str(r#"{"openloop":{"current":0.6}}"#).unwrap();
        assert!(matches!(
            old.to_mode(),
            ControlMode::OpenLoop { pi_gains: None, .. }
        ));
        let explicit: Cmd = serde_json::from_str(
            r#"{"openloop":{"current":0.6,"angle":0.0,"pi_gains":[0.01,10.0]}}"#,
        )
        .unwrap();
        let ControlMode::OpenLoop {
            current, pi_gains, ..
        } = explicit.to_mode()
        else {
            panic!("wrong control mode");
        };
        assert_eq!(current, 0.6);
        assert_eq!(pi_gains, Some((0.01, 10.0)));
        let invalid = Cmd::Openloop {
            current: 0.6,
            velocity: 0.0,
            angle: 0.0,
            pi_gains: Some((f32::NAN, 10.0)),
        };
        assert!(!invalid.to_mode().is_finite());
    }
}

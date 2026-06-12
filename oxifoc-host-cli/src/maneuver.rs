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
use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::HostRuntime;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

use crate::record::Capture;

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
    },
    Voltage {
        #[serde(default)]
        vd: f32,
        #[serde(default)]
        vq: f32,
        #[serde(default)]
        angle: f32,
    },
    Sixstep {
        duty: f32,
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
            } => ControlMode::OpenLoop {
                angle_rad: angle,
                current,
                velocity_rad_s: velocity,
                pi_gains: None,
            },
            Self::Voltage { vd, vq, angle } => ControlMode::DirectVoltage {
                vd,
                vq,
                angle_rad: angle,
            },
            Self::Sixstep { duty } => ControlMode::SixStep { duty },
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
        if let Cmd::Sixstep { duty } = step.cmd
            && !(-1.0..=1.0).contains(&duty)
        {
            bail!("step {i}: sixstep duty {duty} outside -1.0..1.0");
        }
    }
    Ok(())
}

/// Online check against the device's stored current limits.
fn check_limits(runtime: &HostRuntime, m: &Maneuver) -> Result<()> {
    let (limits, _stored) = current_value(runtime, ConfigGroupId::CurrentLimits)?;
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

            let seq_before = cap.last_seq();
            let t_sent = t0.elapsed().as_secs_f64();
            let res = crate::send_motor_acked(runtime, step.cmd.to_mode());
            let t_acked = t0.elapsed().as_secs_f64();
            // A couple of frames may have queued during the ack round-trip;
            // pull them in so seq_after_ack is fresh.
            while let Ok(s) = runtime.fast_rx.try_recv() {
                cap.samples.push(s);
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
    let terminal_res = crate::send_motor_acked(runtime, m.terminal.to_mode());
    let terminal_ok = terminal_res.is_ok();
    let _ = cap.drain_until(runtime, Instant::now() + Duration::from_millis(200));
    cap.stop(runtime);

    // A timeline failure surfaces after the terminal command went out.
    exec?;
    if let Err(e) = terminal_res {
        bail!("terminal command failed: {e:#}");
    }

    let extra_meta = vec![
        (
            "oxifoc.maneuver".to_string(),
            serde_json::to_string(m).unwrap_or_default(),
        ),
        (
            "oxifoc.events".to_string(),
            serde_json::to_string(&events).unwrap_or_default(),
        ),
    ];
    let record = cap.finish(out_path, &config_snapshot, &extra_meta)?;

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

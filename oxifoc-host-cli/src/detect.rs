//! `detect` subcommand: on-device motor detection orchestration.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{ConfigGroupId, DetectRequest};
use oxifoc_host_lib::{HostCommand, HostRuntime};
use serde_json::json;

use crate::config_cli::{self, config_snapshot};
use crate::record;
use crate::{DetectStep, OffsetMethodArg, emit};

/// Stored motor-params as JSON (defaults when not stored).
fn motor_params_value(runtime: &HostRuntime) -> Result<serde_json::Value> {
    let (v, _) = config_cli::current_value(&runtime.cmd_tx, ConfigGroupId::MotorParams)?;
    Ok(v)
}

fn f32_field(v: &serde_json::Value, key: &str) -> f32 {
    v.get(key)
        .and_then(serde_json::Value::as_f64)
        .unwrap_or(0.0) as f32
}

#[allow(clippy::too_many_arguments)]
pub fn run_detect(
    runtime: &HostRuntime,
    step: DetectStep,
    max_power_w: f32,
    resistance: Option<f32>,
    inductance: Option<f32>,
    pole_pairs: Option<u8>,
    erpm: f32,
    offset_method: OffsetMethodArg,
    samples: u16,
    stationary: bool,
    apply: bool,
    record_out: Option<String>,
    record_hz: Option<u16>,
    json: bool,
) -> Result<()> {
    use oxifoc_core::types::DetectResponse;

    if matches!(step, DetectStep::OffsetsCompare) {
        if apply {
            bail!("offset comparison is measure-only; apply one chosen offsets result separately");
        }
        if record_out.is_some() {
            bail!("--record is not supported for the three-step offset comparison");
        }
        let reports = oxifoc_host_lib::ops::detect::compare_current_offsets(
            &runtime.cmd_tx,
            samples,
            stationary,
        )?;
        let delta = |a: usize, b: usize| {
            [
                reports[b].offsets[0] - reports[a].offsets[0],
                reports[b].offsets[1] - reports[a].offsets[1],
                reports[b].offsets[2] - reports[a].offsets[2],
            ]
        };
        emit(
            json,
            json!({
                "reports": reports,
                "delta_per_phase_minus_undriven": delta(0, 1),
                "delta_all_phases_minus_per_phase": delta(1, 2),
            }),
            format!(
                "undriven={:?}\nper-phase-50={:?}\nall-phases-50={:?}\nΔ(per-undriven)={:?}\nΔ(all-per)={:?}",
                reports[0].offsets,
                reports[1].offsets,
                reports[2].offsets,
                delta(0, 1),
                delta(1, 2),
            ),
        );
        return Ok(());
    }

    // Offset measurement has no motor-parameter prerequisites; avoid an
    // unrelated config round-trip on that path.
    let stored = if matches!(step, DetectStep::Offsets) {
        serde_json::Value::Null
    } else {
        motor_params_value(runtime)?
    };
    let r = resistance.unwrap_or_else(|| f32_field(&stored, "resistance_ohm"));
    let l = inductance.unwrap_or_else(|| {
        (f32_field(&stored, "inductance_d_h") + f32_field(&stored, "inductance_q_h")) / 2.0
    });
    let pp = pole_pairs.unwrap_or_else(|| {
        stored
            .get("pole_pairs")
            .and_then(serde_json::Value::as_u64)
            .unwrap_or(0) as u8
    });

    let req = match step {
        DetectStep::Resistance => DetectRequest::MeasureResistance {
            max_power_loss_w: max_power_w,
        },
        DetectStep::Inductance => {
            if r <= 0.0 {
                bail!(
                    "inductance needs resistance: pass --resistance or run detect resistance --apply first"
                );
            }
            DetectRequest::MeasureInductance {
                max_power_loss_w: max_power_w,
                resistance_ohm: r,
            }
        }
        DetectStep::Flux => {
            if r <= 0.0 || pp == 0 {
                bail!(
                    "flux needs resistance and pole pairs: pass --resistance/--pole-pairs \
                     or store them via detect ... --apply / config set motor-params"
                );
            }
            DetectRequest::MeasureFlux {
                max_power_loss_w: max_power_w,
                resistance_ohm: r,
                inductance_h: l,
                pole_pairs: pp,
                openloop_erpm: erpm,
            }
        }
        DetectStep::Hall => {
            if r <= 0.0 {
                bail!(
                    "hall calibration needs resistance (the sweep current is derived \
                     from the power class): pass --resistance or run detect resistance \
                     --apply first"
                );
            }
            DetectRequest::CalibrateHall {
                max_power_loss_w: max_power_w,
                resistance_ohm: r,
            }
        }
        DetectStep::Offsets => {
            let method = offset_method.into();
            if method == oxifoc_core::types::CurrentOffsetMethod::AllPhases50 && !stationary {
                bail!("all-phases-50 requires --stationary");
            }
            DetectRequest::MeasureCurrentOffsets {
                method,
                samples_per_channel: samples,
                stationary_confirmed: stationary,
            }
        }
        DetectStep::OffsetsCompare => unreachable!(),
    };

    if !json {
        println!("Detection started: {req:?}");
    }

    // Raw-rate capture around the whole step by default (M=1: the CIC is the
    // identity, so the HFI carrier / pulse edges survive — decimated rates
    // null exactly the frequencies HFI lives at). `--record-hz` overrides
    // for steps where a loss-free decimated capture beats a lossy raw one.
    let mut cap = match &record_out {
        Some(_) => {
            let rate = record_hz.unwrap_or_else(|| {
                record::latest_hw_info(runtime)
                    .map(|h| h.foc_freq_hz)
                    .unwrap_or(20_000)
                    .min(u32::from(u16::MAX)) as u16
            });
            Some(record::Capture::start(runtime, rate)?)
        }
        None => None,
    };

    let (tx, mut rx) = oxifoc_host_lib::detect_channel();
    runtime
        .cmd_tx
        .send(HostCommand::Detect(req, tx))
        .context("send detect command")?;
    // The detect oneshot is async; poll it (no tokio runtime on this
    // thread). Bounded by a generous deadline (detection can be slow).
    let deadline = Instant::now() + Duration::from_secs(90);
    let detect_result: Result<DetectResponse> = loop {
        if let Ok(res) = rx.try_recv() {
            break res;
        }
        if Instant::now() >= deadline {
            break Err(anyhow::anyhow!("Detection timed out"));
        }
        // While waiting, keep draining telemetry (or just sleep).
        match &mut cap {
            Some(c) => c.drain_until(runtime, Instant::now() + Duration::from_millis(100))?,
            None => std::thread::sleep(Duration::from_millis(100)),
        }
    };

    // The capture is written BEFORE the detect outcome is propagated —
    // dissecting failures is the primary use case.
    let mut capture_summary = serde_json::Value::Null;
    if let (Some(mut c), Some(path)) = (cap.take(), record_out.as_ref()) {
        let _ = c.drain_until(runtime, Instant::now() + Duration::from_millis(200));
        c.stop(runtime);
        let outcome = match &detect_result {
            Ok(r) => serde_json::to_string(r).unwrap_or_default(),
            Err(e) => serde_json::Value::String(format!("error: {e:#}")).to_string(),
        };
        let extra = vec![
            (
                "oxifoc.detect_request".to_string(),
                serde_json::to_string(&req).unwrap_or_default(),
            ),
            ("oxifoc.detect_outcome".to_string(), outcome),
        ];
        let summary = c.finish(path, &config_snapshot(&runtime.cmd_tx), &extra)?;
        if !json || detect_result.is_err() {
            eprintln!(
                "capture: {} — {} rows at {} Hz, {} gap(s)",
                summary.path, summary.rows, summary.fast_hz_actual, summary.gaps
            );
        }
        capture_summary = serde_json::to_value(&summary)?;
    }

    let resp = detect_result?;
    if let DetectResponse::Error(e) = resp {
        bail!("detection failed: {e:?}");
    }

    let mut applied = serde_json::Value::Null;
    if apply {
        if let DetectResponse::CurrentOffsets(report) = resp {
            oxifoc_host_lib::ops::detect::apply_current_offsets(&runtime.cmd_tx, &report)?;
            applied = json!({
                "phase_a": report.offsets[0],
                "phase_b": report.offsets[1],
                "phase_c": report.offsets[2],
            });
        } else {
            let (mut mp, _) =
                config_cli::current_value(&runtime.cmd_tx, ConfigGroupId::MotorParams)?;
            let obj = mp.as_object_mut().context("motor-params not an object")?;
            match resp {
                DetectResponse::Resistance { resistance_ohm } => {
                    obj.insert("resistance_ohm".into(), json!(resistance_ohm));
                }
                DetectResponse::Inductance {
                    inductance_d_h,
                    inductance_q_h,
                } => {
                    obj.insert("inductance_d_h".into(), json!(inductance_d_h));
                    obj.insert("inductance_q_h".into(), json!(inductance_q_h));
                }
                DetectResponse::FluxLinkage {
                    flux_linkage_wb, ..
                } => {
                    obj.insert("flux_linkage_wb".into(), json!(flux_linkage_wb));
                }
                DetectResponse::HallCalibrated => {}
                DetectResponse::CurrentOffsets(_) => {}
                DetectResponse::Error(_) => {}
            }
            if matches!(resp, DetectResponse::HallCalibrated) {
                // The device parks the calibration in its in-RAM runtime config;
                // persisting is the host's job (device comment in
                // runtime/detect.rs). Read the live group back and write it —
                // the write path is what lands it in flash.
                let (hall, _) =
                    config_cli::current_value(&runtime.cmd_tx, ConfigGroupId::HallCalibration)?;
                let write =
                    config_cli::write_from_value(ConfigGroupId::HallCalibration, hall.clone())?;
                config_cli::send_write(&runtime.cmd_tx, write)?;
                applied = hall;
            } else {
                let write = config_cli::write_from_value(ConfigGroupId::MotorParams, mp.clone())?;
                config_cli::send_write(&runtime.cmd_tx, write)?;
                applied = mp;
            }
        }
    }

    emit(
        json,
        json!({"result": serde_json::to_value(resp)?, "applied": applied, "capture": capture_summary}),
        format!(
            "Detect result: {resp:?}{}",
            if apply { " (applied)" } else { "" }
        ),
    );
    Ok(())
}

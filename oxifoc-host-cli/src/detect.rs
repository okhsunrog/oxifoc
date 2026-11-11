//! `detect` subcommand: on-device motor detection orchestration.

use std::path::Path;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{ConfigGroupId, DetectRequest, DetectResponse, HardwareInfo};
use oxifoc_host_lib::ops::detect::{self, DetectionPlan};
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
    plan: DetectionPlan,
    resistance: Option<f32>,
    inductance: Option<f32>,
    offset_method: OffsetMethodArg,
    samples: u16,
    stationary: bool,
    apply: bool,
    record_out: Option<String>,
    record_hz: Option<u16>,
    json: bool,
) -> Result<()> {
    if matches!(step, DetectStep::All) {
        anyhow::ensure!(
            resistance.is_none() && inductance.is_none(),
            "full detection measures its own resistance and inductance"
        );
        return run_all(runtime, plan, record_out.as_deref(), record_hz, apply, json);
    }

    if matches!(step, DetectStep::OffsetsCompare) {
        if apply {
            bail!("offset comparison is measure-only; apply one chosen offsets result separately");
        }
        if record_out.is_some() {
            bail!("--record is not supported for the three-step offset comparison");
        }
        let reports = detect::compare_current_offsets(&runtime.cmd_tx, samples, stationary)?;
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
    let DetectionPlan {
        pole_pairs: pp,
        max_power_loss_w,
        flux_target_erpm,
        current_limit_a,
        ..
    } = plan;

    let req = match step {
        DetectStep::Resistance => DetectRequest::MeasureResistance {
            max_power_loss_w,
            current_limit_a,
        },
        DetectStep::Inductance => {
            if r <= 0.0 {
                bail!(
                    "inductance needs resistance: pass --resistance or run detect resistance --apply first"
                );
            }
            DetectRequest::MeasureInductance {
                max_power_loss_w,
                current_limit_a,
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
                max_power_loss_w,
                current_limit_a,
                resistance_ohm: r,
                inductance_h: l,
                pole_pairs: pp,
                openloop_erpm: flux_target_erpm,
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
                max_power_loss_w,
                current_limit_a,
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
        DetectStep::All | DetectStep::OffsetsCompare => unreachable!(),
    };

    if !json {
        println!("Detection started: {req:?}");
    }
    let (resp, capture) = measure(runtime, req, record_out.as_deref(), record_hz, &mut None)?;
    let applied = if apply {
        detect::apply_step(&runtime.cmd_tx, &resp)?
    } else {
        serde_json::Value::Null
    };
    emit(
        json,
        json!({"result": resp, "applied": applied, "capture": capture}),
        format!(
            "Detect result: {resp:?}{}",
            if apply { " (applied)" } else { "" }
        ),
    );
    Ok(())
}

fn run_all(
    runtime: &HostRuntime,
    plan: DetectionPlan,
    record_out: Option<&str>,
    record_hz: Option<u16>,
    apply: bool,
    json: bool,
) -> Result<()> {
    let directory = record_out.map(Path::new);
    if let Some(directory) = directory {
        std::fs::create_dir(directory).context("create new detection capture directory")?;
    }
    let mut captures = serde_json::Map::new();
    let mut hardware = None;
    let outcome = detect::run_sequence(&runtime.cmd_tx, plan, |stage, request| {
        eprintln!("Detection: {stage}");
        let file = directory.map(|dir| dir.join(format!("{stage}.parquet")));
        let (response, capture) = measure(
            runtime,
            request,
            file.as_ref().map(|p| p.to_string_lossy()).as_deref(),
            record_hz,
            &mut hardware,
        )
        .with_context(|| format!("detect {stage}"))?;
        if let Some(dir) = directory {
            let result = json!({"result": response, "capture": capture});
            std::fs::write(
                dir.join(format!("{stage}.json")),
                serde_json::to_vec_pretty(&result)?,
            )?;
        }
        captures.insert(stage.into(), capture);
        Ok(response)
    })?;
    if apply {
        detect::apply_sequence(&runtime.cmd_tx, &outcome, plan)?;
    }
    let (kp, ki) = detect::suggested_pi_gains(outcome.resistance_ohm, outcome.l_avg());
    emit(
        json,
        json!({"result": outcome, "kp": kp, "ki": ki, "applied": apply, "captures": captures}),
        format!("{outcome:?}\nCurrent PI: Kp={kp:.5}, Ki={ki:.4}"),
    );
    Ok(())
}

fn measure(
    runtime: &HostRuntime,
    req: DetectRequest,
    record_out: Option<&str>,
    record_hz: Option<u16>,
    hardware: &mut Option<HardwareInfo>,
) -> Result<(DetectResponse, serde_json::Value)> {
    // Raw-rate capture around the whole step by default (M=1 preserves the HFI
    // carrier and pulse edges; plain sample dropping can alias them).
    // `--record-hz` overrides
    // for steps where a loss-free decimated capture beats a lossy raw one.
    let mut cap = match &record_out {
        Some(_) => {
            if let Some(info) = record::latest_hw_info(runtime) {
                anyhow::ensure!(
                    hardware
                        .as_ref()
                        .is_none_or(|previous| previous.uuid == info.uuid),
                    "controller changed during detection"
                );
                *hardware = Some(info);
            }
            let hw = hardware
                .clone()
                .context("hardware handshake unavailable for detection recording")?;
            let rate = record_hz.unwrap_or_else(|| hw.foc_freq_hz.min(u32::from(u16::MAX)) as u16);
            Some(record::Capture::start_with_hw(runtime, rate, hw)?)
        }
        None => None,
    };

    let (tx, mut rx) = oxifoc_host_lib::detect_channel();
    runtime
        .cmd_tx
        .send(HostCommand::Detect(oxifoc_host_lib::new_detection(req), tx))
        .context("send detect command")?;
    // The detect oneshot is async; poll it (no tokio runtime on this
    // thread). The deadline derives from the backend's own retry budget
    // plus margin, so the CLI cannot give up before the backend does.
    let deadline =
        Instant::now() + Duration::from_millis(oxifoc_host_lib::DETECT_POLICY.deadline_ms + 5_000);
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
    let mut capture_loss = None;
    if let (Some(mut c), Some(path)) = (cap.take(), record_out.as_ref()) {
        let _ = c.drain_until(runtime, Instant::now() + Duration::from_millis(200));
        let stream_stopped = c.stop(runtime);
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
        if summary.gaps != 0 || summary.samples_lost != 0 {
            capture_loss = Some((summary.gaps, summary.samples_lost));
        }
        if detect_result.is_err() {
            eprintln!(
                "capture: {} — {} rows at {} Hz, {} gap(s)",
                summary.path, summary.rows, summary.fast_hz_actual, summary.gaps
            );
        }
        capture_summary = serde_json::to_value(&summary)?;
        if detect_result.is_ok() {
            stream_stopped?;
        } else if let Err(error) = stream_stopped {
            eprintln!("telemetry stop after failed detection: {error:#}");
        }
    }

    let resp = detect_result?;
    if let DetectResponse::Error(e) = resp {
        bail!("detection failed: {e:?}");
    }
    if let Some((gaps, lost)) = capture_loss {
        bail!(
            "detection capture has {gaps} sequence gaps ({lost} lost samples); {resp:?} retained in capture metadata"
        );
    }
    Ok((resp, capture_summary))
}

#[cfg(test)]
mod hardware_tests {
    use super::*;
    use oxifoc_core::types::{ControlMode, MotorState};
    use oxifoc_host_lib::{CommandSender, HostConfig, TransportType, motor_channel, start_host};

    fn stop(cmd: &CommandSender) -> Result<oxifoc_core::types::MotorStatus> {
        let (tx, mut rx) = motor_channel();
        cmd.send(HostCommand::MotorAck(ControlMode::Stopped, tx))?;
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Ok(result) = rx.try_recv() {
                return result;
            }
            if Instant::now() >= deadline {
                bail!("Stop acknowledgement timed out");
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    #[test]
    #[ignore = "requires the secured MK5 bench motor, explicit opt-in, and a 20 kHz capture path"]
    fn hardware_stop_cancels_inductance_at_20khz() -> Result<()> {
        anyhow::ensure!(std::env::var("OXIFOC_HARDWARE_TEST").as_deref() == Ok("1"));
        let out = std::env::var("OXIFOC_HARDWARE_CAPTURE")?;
        let serial_path = std::env::var("OXIFOC_HARDWARE_SERIAL").ok();
        let resistance = std::env::var("OXIFOC_HARDWARE_RESISTANCE")?.parse::<f32>()?;
        anyhow::ensure!(resistance.is_finite() && resistance > 0.0);
        let runtime = start_host(HostConfig {
            transport: Some(if serial_path.is_some() {
                TransportType::UsbSerialJtag
            } else {
                TransportType::Usb
            }),
            serial_path,
            fast_hz: Some(0),
            stream_defmt: Some(false),
            ..Default::default()
        });
        struct StopOnDrop(CommandSender);
        impl Drop for StopOnDrop {
            fn drop(&mut self) {
                let _ = stop(&self.0);
            }
        }
        let stop_guard = StopOnDrop(runtime.cmd_tx.clone());
        let connected_by = Instant::now() + Duration::from_secs(8);
        while !runtime.connected.load(std::sync::atomic::Ordering::Acquire) {
            anyhow::ensure!(Instant::now() < connected_by, "USB did not connect");
            std::thread::sleep(Duration::from_millis(10));
        }
        anyhow::ensure!(stop(&runtime.cmd_tx)?.fault_count == 0);
        let request = oxifoc_host_lib::new_detection(DetectRequest::MeasureInductance {
            max_power_loss_w: DetectionPlan::from(&detect::PRESETS[2]).max_power_loss_w,
            current_limit_a: None,
            resistance_ohm: resistance,
        });
        let mut capture = record::Capture::start(&runtime, 20000)?;
        let (tx, mut rx) = oxifoc_host_lib::detect_channel();
        runtime
            .cmd_tx
            .send(HostCommand::Detect(request.clone(), tx))?;
        let running_by = Instant::now() + Duration::from_secs(3);
        loop {
            capture.drain_until(&runtime, Instant::now() + Duration::from_millis(10))?;
            if runtime
                .slow_rx
                .try_iter()
                .any(|s| s.motor_state == MotorState::Running)
            {
                break;
            }
            anyhow::ensure!(
                Instant::now() < running_by,
                "detector did not enter holding ramp"
            );
        }
        capture.drain_until(&runtime, Instant::now() + Duration::from_millis(100))?;
        let sent = Instant::now();
        let ack = stop(&runtime.cmd_tx)?;
        let stop_ms = sent.elapsed().as_secs_f64() * 1000.0;
        let finish_by = Instant::now() + Duration::from_secs(4);
        let original = loop {
            if let Ok(result) = rx.try_recv() {
                break result?;
            }
            anyhow::ensure!(Instant::now() < finish_by, "cancel did not finish");
            capture.drain_until(&runtime, Instant::now() + Duration::from_millis(10))?;
        };
        anyhow::ensure!(matches!(
            original,
            DetectResponse::Error(oxifoc_core::types::DetectError::HardwareFault)
        ));
        anyhow::ensure!(ack.state == MotorState::Stopped && ack.fault_count == 0);
        runtime.slow_rx.try_iter().for_each(drop);
        // Model a lost terminal reply: resend the EXACT key and payload.
        // Then try reusing that key with different parameters.
        let mut replay_ms = Vec::new();
        for changed in [false, false, true] {
            let mut repeated = request.clone();
            if changed {
                repeated.inner = DetectRequest::MeasureResistance {
                    max_power_loss_w: 120.0,
                    current_limit_a: None,
                };
            }
            let (tx, mut rx) = oxifoc_host_lib::detect_channel();
            let sent = Instant::now();
            runtime.cmd_tx.send(HostCommand::Detect(repeated, tx))?;
            let response = loop {
                if let Ok(result) = rx.try_recv() {
                    break result?;
                }
                anyhow::ensure!(
                    sent.elapsed() < Duration::from_secs(2),
                    "cached reply timed out"
                );
                capture.drain_until(&runtime, Instant::now() + Duration::from_millis(10))?;
                for status in runtime.slow_rx.try_iter() {
                    anyhow::ensure!(
                        status.motor_state == MotorState::Stopped && status.fault_count == 0,
                        "retry resumed excitation: {status:?}"
                    );
                }
            };
            replay_ms.push(sent.elapsed().as_secs_f64() * 1000.0);
            anyhow::ensure!(
                matches!(
                    (changed, response),
                    (
                        false,
                        DetectResponse::Error(oxifoc_core::types::DetectError::HardwareFault)
                    ) | (
                        true,
                        DetectResponse::Error(oxifoc_core::types::DetectError::KeyConflict)
                    )
                ),
                "unexpected replay: {response:?}"
            );
        }
        capture.drain_until(&runtime, Instant::now() + Duration::from_millis(750))?;
        for status in runtime.slow_rx.try_iter() {
            anyhow::ensure!(status.motor_state == MotorState::Stopped && status.fault_count == 0);
        }
        capture.stop(&runtime)?;
        let summary = capture.finish(
            &out,
            &config_snapshot(&runtime.cmd_tx),
            &[
                (
                    "oxifoc.dedup_request".into(),
                    serde_json::to_string(&request)?,
                ),
                (
                    "oxifoc.dedup_replay_ms".into(),
                    serde_json::to_string(&replay_ms)?,
                ),
            ],
        )?;
        anyhow::ensure!(summary.gaps == 0 && summary.samples_lost == 0);
        println!(
            "{}",
            serde_json::json!({"stop_ack": ack, "stop_ms": stop_ms,
            "replay_ms": replay_ms, "capture": summary})
        );
        drop(stop_guard);
        runtime.shutdown();
        Ok(())
    }
}

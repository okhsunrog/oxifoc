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

    // Raw-rate capture around the whole step by default (M=1 preserves the HFI
    // carrier and pulse edges; plain sample dropping can alias them).
    // `--record-hz` overrides
    // for steps where a loss-free decimated capture beats a lossy raw one.
    let mut cap = match &record_out {
        Some(_) => {
            let hw = record::latest_hw_info(runtime)
                .context("hardware handshake unavailable for detection recording")?;
            let rate = record_hz.unwrap_or_else(|| hw.foc_freq_hz.min(u32::from(u16::MAX)) as u16);
            Some(record::Capture::start_with_hw(runtime, rate, hw)?)
        }
        None => None,
    };

    let (tx, mut rx) = oxifoc_host_lib::detect_channel();
    runtime
        .cmd_tx
        .send(HostCommand::Detect(req, tx))
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
        if !json || detect_result.is_err() {
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
        emit(
            json,
            json!({"result": serde_json::to_value(resp)?, "applied": null,
            "capture": capture_summary}),
            format!("Detect result: {resp:?}; capture incomplete"),
        );
        bail!(
            "detection capture has {gaps} sequence gaps ({lost} lost samples); result retained, not applied"
        );
    }

    // Shared per-step apply (ops::detect): the CLI used to hand-roll a copy
    // that skipped the rating recompute and could drift from the GUI path.
    let applied = if apply {
        oxifoc_host_lib::ops::detect::apply_step(&runtime.cmd_tx, &resp)?
    } else {
        serde_json::Value::Null
    };

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
        let runtime = start_host(HostConfig {
            transport: Some(TransportType::Usb),
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
        let (ack, stop_ms, error) = std::thread::scope(|scope| -> Result<_> {
            let detect = scope.spawn(|| {
                run_detect(
                    &runtime,
                    DetectStep::Inductance,
                    0.05,
                    Some(0.030444),
                    None,
                    None,
                    700.0,
                    OffsetMethodArg::PerPhase,
                    1000,
                    false,
                    false,
                    Some(out.clone()),
                    Some(20000),
                    true,
                )
            });
            let running_by = Instant::now() + Duration::from_secs(3);
            loop {
                if let Ok(status) = runtime.slow_rx.recv_timeout(Duration::from_millis(20))
                    && status.motor_state == MotorState::Running
                {
                    break;
                }
                if Instant::now() >= running_by {
                    let _ = stop(&runtime.cmd_tx);
                    bail!("detector did not enter its holding ramp");
                }
            }
            std::thread::sleep(Duration::from_millis(100));
            let sent = Instant::now();
            let ack = stop(&runtime.cmd_tx)?;
            let elapsed = sent.elapsed().as_secs_f64() * 1000.0;
            let result = detect
                .join()
                .map_err(|_| anyhow::anyhow!("detection thread panicked"))?;
            let error =
                result.expect_err("Stop must cancel the operation, not produce a measurement");
            Ok((ack, elapsed, error.to_string()))
        })?;
        anyhow::ensure!(ack.state == MotorState::Stopped && ack.fault_count == 0);
        anyhow::ensure!(error.contains("HardwareFault"), "{error}");
        runtime.slow_rx.try_iter().for_each(drop);
        let observe_until = Instant::now() + Duration::from_millis(750);
        while Instant::now() < observe_until {
            if let Ok(status) = runtime.slow_rx.recv_timeout(Duration::from_millis(50)) {
                anyhow::ensure!(
                    status.motor_state == MotorState::Stopped && status.fault_count == 0,
                    "detection resumed after Stop: {status:?}"
                );
            }
        }
        println!(
            "{}",
            serde_json::json!({"stop_ack": ack, "stop_ms": stop_ms,
            "detect_error": error, "capture": out})
        );
        drop(stop_guard);
        runtime.shutdown();
        Ok(())
    }
}

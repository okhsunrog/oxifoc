//! Stopped-only, volatile model replacement under continuous native telemetry.
//! Cleanup runs on normal errors and unwinding; killing the process bypasses it.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::storage::MotorParamsConfig;
use oxifoc_core::types::{
    ConfigGroupId, ConfigSnapshot, ConfigValue, ControlMode, MotorState, SlowTelemetry,
};
use oxifoc_host_cli::{config_cli, record::Capture};
use oxifoc_host_lib::ops::{config, telemetry};
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, ReconnectPolicy, TransportType, init_tracing,
    motor_channel, start_host,
};
use serde_json::{Value, json};

#[derive(Parser)]
struct Args {
    /// Exact firmware ELF currently flashed on the device.
    #[arg(long)]
    elf: PathBuf,
    #[arg(long)]
    out: PathBuf,
    /// Also replace pole pairs temporarily; verifies config readback only.
    #[arg(long)]
    check_pole_pairs: bool,
}

fn stopped(status: &SlowTelemetry) -> Result<()> {
    ensure!(
        status.motor_state == MotorState::Stopped
            && status.control_mode == ControlMode::Stopped
            && status.fault_count == 0,
        "device must remain Stopped and fault-free: {status:?}"
    );
    Ok(())
}

fn fresh_status(runtime: &HostRuntime) -> Result<SlowTelemetry> {
    while runtime.slow_rx.try_recv().is_ok() {}
    let status = runtime.slow_rx.recv_timeout(Duration::from_secs(3))?;
    stopped(&status)?;
    Ok(status)
}

fn stop(runtime: &HostRuntime) -> Result<()> {
    let (tx, rx) = motor_channel();
    runtime
        .cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx))?;
    let status = rx.blocking_recv().context("Stop reply dropped")??;
    ensure!(
        status.state == MotorState::Stopped
            && status.mode == ControlMode::Stopped
            && status.fault_count == 0,
        "Stop failed: {status:?}"
    );
    Ok(())
}

fn snapshots(runtime: &HostRuntime) -> Result<Vec<ConfigSnapshot>> {
    config::GROUPS
        .into_iter()
        .map(|(_, id)| config::read_group(&runtime.cmd_tx, id))
        .collect()
}

fn model(snapshot: &ConfigSnapshot) -> Result<&MotorParamsConfig> {
    match snapshot.value.as_ref() {
        Some(ConfigValue::MotorParams(value)) if snapshot.valid => Ok(value),
        _ => bail!("valid original MotorParams are required: {snapshot:?}"),
    }
}

fn apply_model(runtime: &HostRuntime, value: &MotorParamsConfig) -> Result<ConfigSnapshot> {
    let before = config::read_group(&runtime.cmd_tx, ConfigGroupId::MotorParams)?;
    let revision = config::apply_write(
        &runtime.cmd_tx,
        before.boot_epoch,
        before.revision,
        ConfigValue::MotorParams(value.clone()),
    )?;
    let after = config::read_group(&runtime.cmd_tx, ConfigGroupId::MotorParams)?;
    ensure!(
        after.revision == revision && !after.persisted,
        "unexpected apply revision: {after:?}"
    );
    ensure!(
        model(&after)? == value,
        "live model readback differs from requested model"
    );
    Ok(after)
}

fn checked_apply_model(runtime: &HostRuntime, value: &MotorParamsConfig) -> Result<ConfigSnapshot> {
    fresh_status(runtime)?;
    let applied = apply_model(runtime, value)?;
    fresh_status(runtime)?;
    Ok(applied)
}

/// Source requests only acknowledge enqueueing, so observe subsequent telemetry.
fn select_source(
    runtime: &HostRuntime,
    requested: PhaseSource,
    expected: PhaseSource,
) -> Result<SlowTelemetry> {
    while runtime.slow_rx.try_recv().is_ok() {}
    runtime
        .cmd_tx
        .send(HostCommand::SetPhaseSource(requested))?;
    let deadline = Instant::now() + Duration::from_secs(3);
    let rejected = requested != expected;
    let mut latest = None;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        let status = runtime
            .slow_rx
            .recv_timeout(left)
            .with_context(|| format!("waiting for source {requested:?} -> {expected:?}"))?;
        stopped(&status)?;
        if !rejected && status.phase_source == expected {
            return Ok(status);
        }
        ensure!(
            !rejected || status.phase_source == expected,
            "unavailable source {requested:?} became active: {status:?}"
        );
        latest = Some(status);
        // Cover the backend's two-second source-delivery retry budget.
        if rejected
            && deadline.saturating_duration_since(Instant::now()) <= Duration::from_millis(400)
        {
            return latest.context("no status after rejected source request");
        }
    }
    bail!("source {requested:?} did not settle to {expected:?}: {latest:?}")
}

struct Restore<'a> {
    runtime: &'a HostRuntime,
    original: &'a MotorParamsConfig,
    source: PhaseSource,
    armed: bool,
}

impl Restore<'_> {
    fn restore(&mut self) -> Result<()> {
        // Attempt all cleanup actions even when an earlier one fails.
        let stop_result = stop(self.runtime);
        let model_result = (|| {
            let current = config::read_group(&self.runtime.cmd_tx, ConfigGroupId::MotorParams)?;
            if !matches!(current.value.as_ref(),
                Some(ConfigValue::MotorParams(value)) if current.valid && value == self.original)
            {
                apply_model(self.runtime, self.original)?;
            }
            Ok::<_, anyhow::Error>(())
        })();
        let source_result = select_source(self.runtime, self.source, self.source);
        ensure!(
            stop_result.is_ok() && model_result.is_ok() && source_result.is_ok(),
            "restore failed: Stop={stop_result:?}, model={model_result:?}, source={source_result:?}"
        );
        self.armed = false;
        Ok(())
    }
}

impl Drop for Restore<'_> {
    fn drop(&mut self) {
        if self.armed
            && let Err(error) = self.restore()
        {
            eprintln!("MODEL RESTORATION FAILED: {error:#}");
        }
    }
}

struct DrainOnExit<'a>(&'a AtomicBool);
impl Drop for DrainOnExit<'_> {
    fn drop(&mut self) {
        self.0.store(true, Ordering::Relaxed);
    }
}

struct Events<'a> {
    start: Instant,
    seq: &'a AtomicU32,
    entries: Vec<Value>,
}

impl Events<'_> {
    fn record(&mut self, action: &str, value: impl serde::Serialize) {
        self.entries.push(json!({
            "host_elapsed_s": self.start.elapsed().as_secs_f64(),
            "last_observed_seq": self.seq.load(Ordering::Relaxed).checked_sub(1),
            "action": action, "result": value,
        }));
    }
}

fn exercise(
    runtime: &HostRuntime,
    original: &MotorParamsConfig,
    check_poles: bool,
    events: &mut Events<'_>,
) -> Result<()> {
    events.record(
        "select-observer-with-full-model",
        select_source(runtime, PhaseSource::Observer, PhaseSource::Observer)?,
    );
    let mut partial = original.clone();
    partial.flux_linkage_wb = 0.0;
    events.record("partial-model", checked_apply_model(runtime, &partial)?);
    let fallback = fresh_status(runtime)?;
    ensure!(
        fallback.phase_source == PhaseSource::Manual,
        "clearing the selected observer did not select Manual: {fallback:?}"
    );
    events.record("automatic-fallback-after-partial-model", fallback);
    for source in [PhaseSource::Observer, PhaseSource::Hfi] {
        events.record(
            "reject-source-with-partial-model",
            json!({
                "requested": source, "status": select_source(runtime, source, PhaseSource::Manual)?,
            }),
        );
    }
    events.record(
        "restore-full-model-live",
        checked_apply_model(runtime, original)?,
    );
    for source in [PhaseSource::Observer, PhaseSource::Hfi, PhaseSource::Hall] {
        events.record(
            "accept-source-with-full-model",
            json!({
                "requested": source, "status": select_source(runtime, source, source)?,
            }),
        );
    }
    if check_poles {
        let mut changed = original.clone();
        changed.pole_pairs = if original.pole_pairs == 1 {
            2
        } else {
            original.pole_pairs - 1
        };
        events.record(
            "pole-pairs-readback-only",
            checked_apply_model(runtime, &changed)?,
        );
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();
    ensure!(!args.out.exists(), "capture output already exists");
    let elf = args
        .elf
        .canonicalize()
        .context("resolve exact firmware ELF")?;
    init_tracing();
    let mut cfg = HostConfig::load_default()?.unwrap_or_default();
    cfg.transport = Some(TransportType::Usb);
    cfg.elf = Some(elf.to_string_lossy().into_owned());
    cfg.reconnect = Some(ReconnectPolicy::None);
    cfg.fast_hz = Some(0);
    let runtime = start_host(cfg);
    ensure!(
        runtime.wait_for_connection(Duration::from_secs(8)),
        "USB handshake timed out"
    );
    let initial = fresh_status(&runtime)?;
    let before = snapshots(&runtime)?;
    let original_snapshot = before
        .iter()
        .find(|s| s.group == ConfigGroupId::MotorParams)
        .context("MotorParams absent from group snapshot")?;
    let original = model(original_snapshot)?;
    ensure!(
        original.resistance_ohm > 0.0
            && original.inductance_d_h > 0.0
            && original.inductance_q_h > 0.0
            && original.flux_linkage_wb > 0.0
            && original.pole_pairs > 0,
        "a complete original model is required"
    );
    let initial_config = before
        .iter()
        .filter_map(|s| {
            Some((
                config::group_name(s.group).to_owned(),
                config_cli::group_value(s.value.as_ref()?)?,
            ))
        })
        .collect::<serde_json::Map<_, _>>();
    let mut cap = Capture::start(&runtime, 20_000)?;
    if cap.decimation_m != 1 {
        cap.stop(&runtime)?;
        bail!("device did not enable native-rate telemetry");
    }
    let quit = AtomicBool::new(false);
    let seq = AtomicU32::new(0);
    let mut events = Events {
        start: cap.started,
        seq: &seq,
        entries: Vec::new(),
    };
    let (cap, checks, restore_result, stream_stop, drain_result) = std::thread::scope(|scope| {
        let _drain_on_exit = DrainOnExit(&quit);
        let receiver = scope.spawn(|| {
            let mut cap = cap;
            let result = (|| {
                while !quit.load(Ordering::Relaxed) {
                    match runtime.fast_rx.recv_timeout(Duration::from_millis(10)) {
                        Ok(sample) => {
                            seq.store(u32::from(sample.seq) + 1, Ordering::Relaxed);
                            cap.samples.push(sample);
                        }
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                        Err(error) => return Err(anyhow::Error::from(error)),
                    }
                }
                for _ in 0..runtime.fast_rx.len() {
                    if let Ok(sample) = runtime.fast_rx.try_recv() {
                        cap.samples.push(sample);
                    }
                }
                Ok(())
            })();
            (cap, result)
        });
        let mut restore = Restore {
            runtime: &runtime,
            original,
            source: initial.phase_source,
            armed: true,
        };
        let checks = exercise(&runtime, original, args.check_pole_pairs, &mut events);
        let restore_result = restore.restore();
        events.record(
            "restore-original-model-and-source",
            json!({"ok": restore_result.is_ok()}),
        );
        drop(restore);
        let stream_stop = telemetry::configure(&runtime.cmd_tx, 0).map(|_| ());
        quit.store(true, Ordering::Relaxed);
        let (cap, drained) = receiver.join().expect("capture consumer panicked");
        (cap, checks, restore_result, stream_stop, drained)
    });
    let after = snapshots(&runtime);
    let final_status = fresh_status(&runtime);
    let unchanged = after.as_ref().is_ok_and(|after| {
        before.iter().all(|before| {
            after
                .iter()
                .find(|s| s.group == before.group)
                .is_some_and(|after| {
                    if before.group == ConfigGroupId::MotorParams {
                        before.value == after.value && after.valid
                    } else {
                        before == after
                    }
                })
        })
    });
    let report = json!({
        "elf": elf, "before": before, "after": after.as_ref().ok(),
        "capture": { "path": args.out, "rows": cap.samples.len(),
            "gaps_and_samples_lost": cap.analyze_gaps(), "decimation_m": cap.decimation_m,
            "fast_hz_actual": cap.fast_hz_actual, "control_timing": cap.control_timing },
        "initial_status": initial, "final_status": final_status.as_ref().ok(),
        "events": events.entries, "model_and_other_groups_restored": unchanged,
        "checks_error": checks.as_ref().err().map(|e| format!("{e:#}")),
        "restore_error": restore_result.as_ref().err().map(|e| format!("{e:#}")),
        "capture_stop_error": stream_stop.as_ref().err().map(|e| format!("{e:#}")),
        "drain_error": drain_result.as_ref().err().map(|e| format!("{e:#}")),
        "final_config_error": after.as_ref().err().map(|e| format!("{e:#}")),
        "final_status_error": final_status.as_ref().err().map(|e| format!("{e:#}")),
        "pole_pairs_note": "Stopped-only config readback; RPM scaling is not measured. Enrichment uses the initial pole pairs.",
        "restoration_note": "MotorParams value restored exactly; live revision advances and remains volatile. Flash is never written.",
    });
    std::fs::write(
        format!("{}.json", args.out.display()),
        serde_json::to_vec_pretty(&report)?,
    )?;
    let summary = cap.finish(
        &args.out.to_string_lossy(),
        &Value::Object(initial_config),
        &[("oxifoc.live_model_apply".into(), report.to_string())],
    )?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    runtime.shutdown();
    checks?;
    restore_result?;
    stream_stop?;
    drain_result?;
    after?;
    let status = final_status?;
    ensure!(
        unchanged && status.phase_source == initial.phase_source,
        "original configuration/source was not restored"
    );
    ensure!(
        summary.rows >= 20_000 && summary.gaps == 0 && summary.samples_lost == 0,
        "capture is too short or contains dropped samples"
    );
    ensure!(
        cap.decimation_m == 1
            && cap
                .samples
                .windows(2)
                .all(|p| p[1].seq.wrapping_sub(p[0].seq) == 1),
        "capture is not consecutive native-rate telemetry"
    );
    ensure!(
        cap.samples.iter().all(|s| s.pwm_enables & 0x3f == 0),
        "phase outputs were enabled during stopped-only test"
    );
    Ok(())
}

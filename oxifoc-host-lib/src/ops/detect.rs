//! Motor-detection orchestration shared by the front-ends.
//!
//! One sequence, one apply policy, one gains preview — so the CLI and GUI
//! can't disagree on what `detect` measures or what `apply` writes. In
//! particular: **apply writes only the measured motor parameters (plus the
//! thermal current rating); it never writes PI gains.** The device retunes
//! the current loop from the motor params on write, using
//! [`calculate_current_gains`] at [`DEFAULT_BANDWIDTH`] — the single
//! source of truth. [`suggested_pi_gains`] reproduces that math purely so a
//! front-end can *show* the operator what the device will use.

use anyhow::{Context, Result, bail};
use oxifoc_core::foc::controller::{CurrentKi, CurrentKp};
use oxifoc_core::foc::detection::pi_tuning::{DEFAULT_BANDWIDTH, calculate_current_gains};
use oxifoc_core::foc::detection::resistance::calculate_max_current;
use oxifoc_core::foc::detection::types::MotorSize;
use oxifoc_core::storage::MotorParamsConfig;
use oxifoc_core::types::commissioning::{CommissioningPlan, MotorClass, SourceLimits};
use oxifoc_core::types::measurement::{HallCalibrationConfig, Measurement, SessionId};
use oxifoc_core::types::{
    ConfigGroupId, ControlMode, CurrentOffsetMethod, CurrentOffsetReport, DetectRequest,
    DetectResponse, MotorState,
};
use oxifoc_core::units::{
    ElectricCurrent, ElectricalAngularVelocity, ElectricalResistance, Inductance, MagneticFlux,
    Power, Sign, amps, watts,
};
use serde_json::json;

use super::config;
use crate::{CommandSender, HostCommand, detect_channel, motor_channel};

mod presets;
pub use presets::{MotorPreset, PRESETS};

/// Resolved profile used throughout a commissioning run and its later apply.
/// Serialized (CLI `--json`) with quantities in SI base units.
#[derive(Clone, Copy, Debug, serde::Serialize)]
pub struct DetectionPlan {
    pub pole_pairs: u8,
    pub max_power_loss: Power,
    pub motor_class: MotorClass,
    pub source: Option<SourceLimits>,
    pub flux_target_speed: ElectricalAngularVelocity,
    pub current_limit: Option<ElectricCurrent>,
    pub calibrate_hall: bool,
}

impl From<&MotorPreset> for DetectionPlan {
    fn from(preset: &MotorPreset) -> Self {
        Self {
            pole_pairs: preset.pole_pairs,
            max_power_loss: preset.max_power_loss,
            motor_class: preset.class,
            source: None,
            // VESC's lower open-loop ERPM is for subsequent riding startup.
            flux_target_speed: preset.sensorless_speed,
            current_limit: None,
            calibrate_hall: true,
        }
    }
}

/// Collected results of a full detection sequence, serialized with
/// quantities in SI base units.
#[derive(Clone, Copy, Debug, Default, serde::Serialize)]
pub struct DetectionOutcome {
    /// Phase resistance.
    pub resistance: ElectricalResistance,
    /// d-axis inductance.
    pub inductance_d: Inductance,
    /// q-axis inductance.
    pub inductance_q: Inductance,
    /// Flux linkage.
    pub flux_linkage: MagneticFlux,
    /// Motor velocity constant (RPM/V).
    pub kv_rpm_per_v: f32,
    /// Whether Hall calibration was requested and succeeded.
    pub hall_ok: bool,
    pub hall: Option<HallCalibrationConfig>,
    pub session: Option<SessionId>,
}

impl DetectionOutcome {
    /// Construct the reviewed setup without inheriting fields from the previous motor.
    /// The device must validate it against its own inverter protection before installation.
    pub fn setup(
        &self,
        plan: DetectionPlan,
        current: oxifoc_core::drive_setup::DriveCurrent,
    ) -> Result<oxifoc_core::drive_setup::DriveSetup> {
        use oxifoc_core::drive_setup::{DriveSetup, Feedback};
        anyhow::ensure!(self.session.is_some(), "missing measurement provenance");
        anyhow::ensure!(
            plan.calibrate_hall == self.hall.is_some() && self.hall_ok == self.hall.is_some(),
            "incomplete feedback result"
        );
        anyhow::ensure!(
            plan.max_power_loss == plan.motor_class.preset().max_power_loss,
            "inconsistent motor class"
        );
        Ok(DriveSetup {
            motor: MotorParamsConfig {
                resistance: self.resistance,
                inductance_d: self.inductance_d,
                inductance_q: self.inductance_q,
                flux_linkage: self.flux_linkage,
                pole_pairs: plan.pole_pairs,
                max_power_loss: Some(plan.max_power_loss),
                max_current: rating_from_loss(self.resistance, plan.max_power_loss),
                ..Default::default()
            },
            feedback: self.hall.map_or(Feedback::Sensorless, Feedback::Hall),
            source: plan.source.context("missing confirmed source limits")?,
            current,
        })
    }

    /// Average of the d/q inductances.
    #[must_use]
    pub fn l_avg(&self) -> Inductance {
        (self.inductance_d + self.inductance_q) / 2.0
    }
}

/// The PI gains the device will compute from these params on a config write
/// (identical math: `calculate_current_gains` at the default bandwidth),
/// when both inputs are positive. For display only — the front-ends must not
/// write these back.
#[must_use]
pub fn suggested_pi_gains(
    resistance: ElectricalResistance,
    l_avg: Inductance,
) -> Option<(CurrentKp, CurrentKi)> {
    (l_avg.is_positive() && resistance.is_positive())
        .then(|| calculate_current_gains(resistance, l_avg, DEFAULT_BANDWIDTH))
}

/// Continuous-current rating from the VESC thermal solve (√(P/R/1.5)), when
/// both inputs are positive; otherwise unknown and the board defaults apply.
#[must_use]
pub fn rating_from_loss(
    resistance: ElectricalResistance,
    max_power_loss: Power,
) -> Option<ElectricCurrent> {
    (resistance.is_positive() && max_power_loss.is_positive())
        .then(|| calculate_max_current(resistance, MotorSize::Custom(max_power_loss)))
}

/// Run one detection step and wait for the device's response (blocking).
pub fn run_step(cmd: &CommandSender, req: DetectRequest) -> Result<DetectResponse> {
    run_measurement(cmd, crate::new_detection(cmd, req)?)
}

pub fn run_measurement(cmd: &CommandSender, measurement: Measurement) -> Result<DetectResponse> {
    let (tx, rx) = detect_channel();
    cmd.send(HostCommand::Detect(measurement, tx))
        .context("send detect command")?;
    rx.blocking_recv()
        .context("backend dropped the detect command")?
        .context("detection failed")
}

/// Require acknowledged Stop and a healthy controller before a wizard run.
pub fn preflight(cmd: &CommandSender) -> Result<()> {
    let (tx, rx) = motor_channel();
    cmd.send(HostCommand::MotorAck(ControlMode::Stopped, tx))
        .context("send Stop before detection")?;
    let status = rx.blocking_recv().context("Stop acknowledgement lost")??;
    anyhow::ensure!(
        status.state == MotorState::Stopped && status.mode == ControlMode::Stopped,
        "controller did not acknowledge Stop: {status:?}"
    );
    let faults = super::fault::query(cmd)?;
    anyhow::ensure!(
        faults.total == 0,
        "active faults before detection: {:?}",
        faults.faults
    );
    Ok(())
}

/// Run one non-mutating current-offset diagnostic.
pub fn measure_current_offsets(
    cmd: &CommandSender,
    method: CurrentOffsetMethod,
    samples_per_channel: u16,
    stationary_confirmed: bool,
) -> Result<CurrentOffsetReport> {
    match run_step(
        cmd,
        DetectRequest::MeasureCurrentOffsets {
            method,
            samples_per_channel,
            stationary_confirmed,
        },
    )? {
        DetectResponse::CurrentOffsets(report) => Ok(report),
        DetectResponse::Error(error) => bail!("offset calibration failed: {error:?}"),
        other => bail!("unexpected offset response: {other:?}"),
    }
}

/// Measure all supported topologies without applying any of them.
pub fn compare_current_offsets(
    cmd: &CommandSender,
    samples_per_channel: u16,
    stationary_confirmed: bool,
) -> Result<[CurrentOffsetReport; 3]> {
    if !stationary_confirmed {
        bail!("offset comparison includes all-phases-50; confirm the rotor is stationary");
    }
    Ok([
        measure_current_offsets(
            cmd,
            CurrentOffsetMethod::Undriven,
            samples_per_channel,
            true,
        )?,
        measure_current_offsets(
            cmd,
            CurrentOffsetMethod::PerPhase50,
            samples_per_channel,
            true,
        )?,
        measure_current_offsets(
            cmd,
            CurrentOffsetMethod::AllPhases50,
            samples_per_channel,
            true,
        )?,
    ])
}

/// Persist a measured report through the normal config write-through path;
/// the device also applies that write to the live sensor immediately.
pub fn apply_current_offsets(cmd: &CommandSender, report: &CurrentOffsetReport) -> Result<()> {
    let (mut value, state) = config::current_value(cmd, ConfigGroupId::DcOffsets)?;
    let obj = value
        .as_object_mut()
        .context("dc-offsets is not a JSON object")?;
    obj.insert("phase_a".into(), json!(report.offsets[0]));
    obj.insert("phase_b".into(), json!(report.offsets[1]));
    obj.insert("phase_c".into(), json!(report.offsets[2]));
    let write = config::write_from_value(ConfigGroupId::DcOffsets, value)
        .context("patched dc-offsets no longer deserializes")?;
    let revision = config::apply_write(cmd, state.boot_epoch, state.revision, write)?;
    config::persist_group(cmd, ConfigGroupId::DcOffsets, state.boot_epoch, revision)?;
    Ok(())
}

/// Run the same steps for both front-ends, allowing the CLI to record each one.
pub fn run_sequence(
    cmd: &CommandSender,
    plan: DetectionPlan,
    mut measure: impl FnMut(&'static str, Measurement) -> Result<DetectResponse>,
) -> Result<DetectionOutcome> {
    let DetectionPlan {
        pole_pairs,
        max_power_loss,
        flux_target_speed,
        current_limit,
        calibrate_hall,
        motor_class,
        source,
    } = plan;
    anyhow::ensure!(
        pole_pairs > 0
            && max_power_loss.is_finite()
            && max_power_loss.is_positive()
            && flux_target_speed.is_finite()
            && flux_target_speed.is_positive()
            && current_limit.is_none_or(|a| a.is_finite() && a >= amps(0.5)),
        "invalid detection profile or override"
    );
    preflight(cmd)?;
    anyhow::ensure!(
        max_power_loss == motor_class.preset().max_power_loss,
        "motor class does not match excitation profile"
    );
    let confirmed = CommissioningPlan {
        source: source.context("confirm input and regenerative current limits before detection")?,
        motor_class,
        pole_pairs,
        flux_target_speed,
        current_limit,
        calibrate_hall,
    };
    let mut session = crate::DetectionSession::open(cmd, Some(confirmed))?;
    let mut out = DetectionOutcome {
        session: Some(session.id()),
        ..Default::default()
    };
    let mut measure = |stage, request| measure(stage, session.measurement(request)?);

    // Step 1: resistance.
    match measure(
        "resistance",
        DetectRequest::MeasureResistance {
            max_power_loss,
            current_limit,
        },
    )? {
        DetectResponse::Resistance { resistance } => out.resistance = resistance,
        DetectResponse::Error(e) => bail!("R: {e:?}"),
        other => bail!("R: unexpected response {other:?}"),
    }

    // Step 2: inductance (needs R).
    match measure(
        "inductance",
        DetectRequest::MeasureInductance {
            max_power_loss,
            current_limit,
            resistance: out.resistance,
        },
    )? {
        DetectResponse::Inductance {
            inductance_d,
            inductance_q,
        } => {
            out.inductance_d = inductance_d;
            out.inductance_q = inductance_q;
        }
        DetectResponse::Error(e) => bail!("L: {e:?}"),
        other => bail!("L: unexpected response {other:?}"),
    }

    // Step 3: flux linkage (needs R, L, pole pairs).
    match measure(
        "flux",
        DetectRequest::MeasureFlux {
            max_power_loss,
            current_limit,
            resistance: out.resistance,
            inductance: out.l_avg(),
            pole_pairs,
            openloop_speed: flux_target_speed,
        },
    )? {
        DetectResponse::FluxLinkage {
            flux_linkage,
            kv_rpm_per_v,
        } => {
            out.flux_linkage = flux_linkage;
            out.kv_rpm_per_v = kv_rpm_per_v;
        }
        DetectResponse::Error(e) => bail!("Flux: {e:?}"),
        other => bail!("Flux: unexpected response {other:?}"),
    }

    if calibrate_hall {
        match measure(
            "hall",
            DetectRequest::CalibrateHall {
                max_power_loss,
                current_limit,
                resistance: out.resistance,
            },
        )? {
            DetectResponse::HallCalibrated(hall) => {
                anyhow::ensure!(hall.is_calibrated(), "invalid Hall candidate");
                out.hall_ok = true;
                out.hall = Some(hall);
            }
            DetectResponse::Error(e) => bail!("Hall: {e:?}"),
            other => bail!("Hall: unexpected response {other:?}"),
        }
    }

    preflight(cmd)?;
    Ok(out)
}

/// Use the same current resolution as firmware for the review screen.
pub fn preview_setup(
    cmd: &CommandSender,
    setup: oxifoc_core::drive_setup::DriveSetup,
    board_trip_a: f32,
) -> Result<oxifoc_core::drive_setup::ValidatedDriveSetup> {
    let limits = config::read_group(cmd, ConfigGroupId::CurrentLimits)?;
    let trip = match limits.value {
        Some(oxifoc_core::types::ConfigValue::CurrentLimits(value)) => value.max_phase_current,
        None => None,
        _ => bail!("controller returned an unexpected current-limits value"),
    };
    setup
        .validate(amps(board_trip_a), trip)
        .map_err(|e| anyhow::anyhow!("invalid drive setup: {e:?}"))
}

/// Install and persist the reviewed setup with one addressed request.
pub fn apply_sequence(
    cmd: &CommandSender,
    outcome: &DetectionOutcome,
    plan: DetectionPlan,
    current: oxifoc_core::drive_setup::DriveCurrent,
) -> Result<()> {
    use oxifoc_core::types::{ConfigResponse, Keyed, SaveDriveSetup};
    let setup = outcome.setup(plan, current)?;
    let session = outcome.session.context("missing measurement session")?;
    let id = config::next_config_id();
    let (tx, rx) = crate::config_channel();
    cmd.send(HostCommand::SaveDriveSetup(
        Keyed::new(id, SaveDriveSetup { session, setup }),
        tx,
    ))
    .context("send setup save")?;
    match rx
        .blocking_recv()
        .context("setup save outcome unknown")?
        .context("setup save outcome unknown; check the controller before retrying")?
    {
        ConfigResponse::Persisted { req_id, .. } if req_id == id => Ok(()),
        ConfigResponse::SetupRequiresRestart { req_id } if req_id == id => {
            bail!("setup was saved to flash; restart the controller to load it")
        }
        ConfigResponse::Invalid => {
            bail!("setup rejected: check the measured values, current limits and session freshness")
        }
        ConfigResponse::Busy => bail!("stop the controller before saving"),
        ConfigResponse::Unsupported => bail!("this firmware does not support setup persistence"),
        ConfigResponse::Error => {
            bail!("setup flash write failed; restart and check the stored configuration")
        }
        other => bail!("unexpected setup save response: {other:?}"),
    }
}

/// Apply a SINGLE detection step's result into config, patching only what
/// the step measured. Returns the JSON that was written (for display).
///
/// This is the shared per-step apply used by every front-end (the CLI used
/// to hand-roll its own copy that skipped the rating recompute below):
/// - measured motor params patch the `motor-params` group field-by-field;
///   a new resistance also recomputes the thermal current rating from the
///   stored dissipation class (rating = √(P/R/1.5) tracks R);
/// - Hall calibration applies and persists the returned candidate; measurement
///   itself never changes the installed table;
/// - current offsets persist through [`apply_current_offsets`].
pub fn apply_step(cmd: &CommandSender, resp: &DetectResponse) -> Result<serde_json::Value> {
    match resp {
        DetectResponse::CurrentOffsets(report) => {
            apply_current_offsets(cmd, report)?;
            Ok(json!({
                "phase_a": report.offsets[0],
                "phase_b": report.offsets[1],
                "phase_c": report.offsets[2],
            }))
        }
        DetectResponse::HallCalibrated(calibration) => {
            let (_, state) = config::current_value(cmd, ConfigGroupId::HallCalibration)?;
            let hall = serde_json::to_value(calibration)?;
            let write = config::write_from_value(ConfigGroupId::HallCalibration, hall.clone())
                .context("Hall candidate no longer deserializes")?;
            let revision = config::apply_write(cmd, state.boot_epoch, state.revision, write)?;
            config::persist_group(
                cmd,
                ConfigGroupId::HallCalibration,
                state.boot_epoch,
                revision,
            )?;
            Ok(hall)
        }
        DetectResponse::Error(e) => bail!("cannot apply a failed detection: {e:?}"),
        measured => {
            let (mut value, state) = config::current_value(cmd, ConfigGroupId::MotorParams)?;
            let obj = value
                .as_object_mut()
                .context("motor-params is not a JSON object")?;
            match measured {
                DetectResponse::Resistance { resistance } => {
                    obj.insert("resistance".into(), json!(resistance));
                    let loss = obj
                        .get("max_power_loss")
                        .and_then(serde_json::Value::as_f64)
                        .map(|w| watts(w as f32));
                    if let Some(rating) = loss.and_then(|loss| rating_from_loss(*resistance, loss))
                    {
                        obj.insert("max_current".into(), json!(rating));
                    }
                }
                DetectResponse::Inductance {
                    inductance_d,
                    inductance_q,
                } => {
                    obj.insert("inductance_d".into(), json!(inductance_d));
                    obj.insert("inductance_q".into(), json!(inductance_q));
                }
                DetectResponse::FluxLinkage { flux_linkage, .. } => {
                    obj.insert("flux_linkage".into(), json!(flux_linkage));
                }
                _ => bail!("unexpected detect response: {measured:?}"),
            }
            let write = config::write_from_value(ConfigGroupId::MotorParams, value.clone())
                .context("patched motor-params no longer deserializes")?;
            let revision = config::apply_write(cmd, state.boot_epoch, state.revision, write)?;
            config::persist_group(cmd, ConfigGroupId::MotorParams, state.boot_epoch, revision)?;
            Ok(value)
        }
    }
}

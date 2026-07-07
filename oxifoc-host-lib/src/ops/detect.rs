//! Motor-detection orchestration shared by the front-ends.
//!
//! One sequence, one apply policy, one gains preview — so the CLI and GUI
//! can't disagree on what `detect` measures or what `apply` writes. In
//! particular: **apply writes only the measured motor parameters (plus the
//! thermal current rating); it never writes PI gains.** The device retunes
//! the current loop from the motor params on write, using
//! [`calculate_current_gains`] at [`DEFAULT_BANDWIDTH_RAD_S`] — the single
//! source of truth. [`suggested_pi_gains`] reproduces that math purely so a
//! front-end can *show* the operator what the device will use.

use anyhow::{Context, Result, bail};
use oxifoc_core::foc::detection::pi_tuning::{DEFAULT_BANDWIDTH_RAD_S, calculate_current_gains};
use oxifoc_core::foc::detection::resistance::calculate_max_current;
use oxifoc_core::foc::detection::types::MotorSize;
use oxifoc_core::types::{ConfigGroupId, DetectRequest, DetectResponse};
use serde_json::json;

use super::config;
use crate::{CommandSender, HostCommand, detect_channel};

/// Collected results of a full detection sequence.
#[derive(Clone, Copy, Debug, Default)]
pub struct DetectionOutcome {
    /// Phase resistance (Ω).
    pub resistance_ohm: f32,
    /// d-axis inductance (H).
    pub inductance_d_h: f32,
    /// q-axis inductance (H).
    pub inductance_q_h: f32,
    /// Flux linkage (Wb).
    pub flux_linkage_wb: f32,
    /// Motor velocity constant (RPM/V).
    pub kv_rpm_per_v: f32,
    /// Whether Hall calibration succeeded (false = no hall / failed; non-fatal).
    pub hall_ok: bool,
}

impl DetectionOutcome {
    /// Average of the d/q inductances.
    #[must_use]
    pub fn l_avg(&self) -> f32 {
        (self.inductance_d_h + self.inductance_q_h) / 2.0
    }
}

/// The PI gains the device will compute from these params on a config write
/// (identical math: `calculate_current_gains` at the default bandwidth).
/// For display only — the front-ends must not write these back.
#[must_use]
pub fn suggested_pi_gains(resistance_ohm: f32, l_avg_h: f32) -> (f32, f32) {
    if l_avg_h > 0.0 && resistance_ohm > 0.0 {
        calculate_current_gains(resistance_ohm, l_avg_h, DEFAULT_BANDWIDTH_RAD_S)
    } else {
        (0.0, 0.0)
    }
}

/// Continuous-current rating from the VESC thermal solve (√(P/R/1.5)).
/// Returns 0.0 (= "unknown", board defaults apply) when inputs are invalid.
#[must_use]
pub fn rating_from_loss(resistance_ohm: f32, max_power_loss_w: f32) -> f32 {
    if resistance_ohm > 0.0 && max_power_loss_w > 0.0 {
        calculate_max_current(resistance_ohm, MotorSize::Custom(max_power_loss_w))
    } else {
        0.0
    }
}

/// Run one detection step and wait for the device's response (blocking).
pub fn run_step(cmd: &CommandSender, req: DetectRequest) -> Result<DetectResponse> {
    let (tx, rx) = detect_channel();
    cmd.send(HostCommand::Detect(req, tx))
        .context("send detect command")?;
    rx.blocking_recv()
        .context("backend dropped the detect command")?
        .context("detection failed")
}

/// Run the full R → L → flux → hall sequence and collect the results.
///
/// R/L/flux failures abort with an error; a Hall-calibration failure is
/// non-fatal (motor may have no hall sensors) and leaves `hall_ok = false`.
pub fn run_sequence(
    cmd: &CommandSender,
    pole_pairs: u8,
    max_power_loss_w: f32,
    openloop_erpm: f32,
) -> Result<DetectionOutcome> {
    let mut out = DetectionOutcome::default();

    // Step 1: resistance.
    match run_step(cmd, DetectRequest::MeasureResistance { max_power_loss_w })? {
        DetectResponse::Resistance { resistance_ohm } => out.resistance_ohm = resistance_ohm,
        DetectResponse::Error(e) => bail!("R: {e:?}"),
        other => bail!("R: unexpected response {other:?}"),
    }

    // Step 2: inductance (needs R).
    match run_step(
        cmd,
        DetectRequest::MeasureInductance {
            max_power_loss_w,
            resistance_ohm: out.resistance_ohm,
        },
    )? {
        DetectResponse::Inductance {
            inductance_d_h,
            inductance_q_h,
        } => {
            out.inductance_d_h = inductance_d_h;
            out.inductance_q_h = inductance_q_h;
        }
        DetectResponse::Error(e) => bail!("L: {e:?}"),
        other => bail!("L: unexpected response {other:?}"),
    }

    // Step 3: flux linkage (needs R, L, pole pairs).
    match run_step(
        cmd,
        DetectRequest::MeasureFlux {
            max_power_loss_w,
            resistance_ohm: out.resistance_ohm,
            inductance_h: out.l_avg(),
            pole_pairs,
            openloop_erpm,
        },
    )? {
        DetectResponse::FluxLinkage {
            flux_linkage_wb,
            kv_rpm_per_v,
        } => {
            out.flux_linkage_wb = flux_linkage_wb;
            out.kv_rpm_per_v = kv_rpm_per_v;
        }
        DetectResponse::Error(e) => bail!("Flux: {e:?}"),
        other => bail!("Flux: unexpected response {other:?}"),
    }

    // Step 4: hall calibration (best-effort). Sweep current derives from
    // the same power class as the other steps (device-side √(P/R/1.5)).
    if let Ok(DetectResponse::HallCalibrated) = run_step(
        cmd,
        DetectRequest::CalibrateHall {
            max_power_loss_w,
            resistance_ohm: out.resistance_ohm,
        },
    ) {
        out.hall_ok = true;
    }

    Ok(out)
}

/// Write the measured parameters into the `motor-params` group: the four
/// measured fields, the supplied pole pairs, and the thermal current rating.
/// PI gains are intentionally *not* written — the device retunes them.
pub fn apply_motor_params(
    cmd: &CommandSender,
    outcome: &DetectionOutcome,
    pole_pairs: u8,
    max_power_loss_w: f32,
) -> Result<()> {
    let (mut value, _stored) = config::current_value(cmd, ConfigGroupId::MotorParams)?;
    let obj = value
        .as_object_mut()
        .context("motor-params is not a JSON object")?;
    obj.insert("resistance_ohm".into(), json!(outcome.resistance_ohm));
    obj.insert("inductance_d_h".into(), json!(outcome.inductance_d_h));
    obj.insert("inductance_q_h".into(), json!(outcome.inductance_q_h));
    obj.insert("flux_linkage_wb".into(), json!(outcome.flux_linkage_wb));
    obj.insert("pole_pairs".into(), json!(pole_pairs));
    obj.insert(
        "max_current_a".into(),
        json!(rating_from_loss(outcome.resistance_ohm, max_power_loss_w)),
    );
    obj.insert("max_power_loss_w".into(), json!(max_power_loss_w));

    let write = config::write_from_value(ConfigGroupId::MotorParams, value)
        .context("patched motor-params no longer deserializes")?;
    config::send_write(cmd, write)
}

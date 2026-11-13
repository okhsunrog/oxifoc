//! Config get/set — generic over every group via a JSON round-trip.
//!
//! All stored-config structs derive `Serialize + Deserialize + Default`, so
//! field-level writes are implemented without per-field code: read the
//! current group (or its default when nothing is stored), patch keys on the
//! JSON object, deserialize back into the typed struct and send a
//! `ConfigValue`. Unknown field names are rejected against the actual
//! object's keys (serde would silently ignore them otherwise).
//!
//! Lives in host-lib (not the CLI) so the GUI shares the exact same group
//! schema, defaults and write/validation handling. CLI-only presentation
//! (`config dump --rust`) stays in the CLI.

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{
    ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, ConfigSnapshot, ConfigValue, Keyed,
    ReqId,
};
use serde_json::Value;

use crate::{CommandSender, HostCommand, config_channel};

/// kebab-case CLI name ↔ group id, in protocol order.
pub const GROUPS: [(&str, ConfigGroupId); ConfigGroupId::COUNT] = [
    ("motor-params", ConfigGroupId::MotorParams),
    ("hall-calibration", ConfigGroupId::HallCalibration),
    ("dc-offsets", ConfigGroupId::DcOffsets),
    ("current-limits", ConfigGroupId::CurrentLimits),
    ("voltage-limits", ConfigGroupId::VoltageLimits),
    ("pwm-config", ConfigGroupId::PwmConfig),
    ("pi-gains", ConfigGroupId::PiGains),
    ("hall-tuning", ConfigGroupId::HallTuning),
    ("failsafe", ConfigGroupId::Failsafe),
    ("velocity", ConfigGroupId::Velocity),
    ("derating", ConfigGroupId::Derating),
    ("board-calibration", ConfigGroupId::BoardCalibration),
    ("voltage-feedforward", ConfigGroupId::VoltageFeedforward),
    ("motor-temperature", ConfigGroupId::MotorTemperature),
    ("drive", ConfigGroupId::Drive),
    ("motor-feedback", ConfigGroupId::MotorFeedback),
];

/// Parse a kebab-case group name into its id.
pub fn parse_group(s: &str) -> Result<ConfigGroupId> {
    GROUPS
        .iter()
        .find(|(name, _)| *name == s)
        .map(|(_, g)| *g)
        .with_context(|| {
            let names: Vec<&str> = GROUPS.iter().map(|(n, _)| *n).collect();
            format!(
                "unknown config group '{s}'; available: {}",
                names.join(", ")
            )
        })
}

/// kebab-case name for a group id (`"?"` if somehow unknown).
#[must_use]
pub fn group_name(group: ConfigGroupId) -> &'static str {
    GROUPS
        .iter()
        .find(|(_, g)| *g == group)
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

pub(super) fn next_config_id() -> ReqId {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CTR: std::sync::OnceLock<AtomicU64> = std::sync::OnceLock::new();
    let ctr = CTR.get_or_init(|| {
        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        AtomicU64::new(seed | 1)
    });
    ReqId(ctr.fetch_add(1, Ordering::Relaxed))
}

#[derive(Clone, Debug)]
pub struct ConfigState {
    pub valid: bool,
    pub revision: u32,
    pub boot_epoch: u64,
    pub persisted: bool,
    pub present: bool,
    pub effective: Option<Value>,
}

#[derive(Clone, Copy, Debug)]
pub struct ConfigCommit {
    pub revision: u32,
    pub persisted: bool,
}

/// Read one config group with its optimistic-concurrency revision.
pub fn read_group(cmd: &CommandSender, group: ConfigGroupId) -> Result<ConfigSnapshot> {
    let (tx, rx) = config_channel();
    cmd.send(HostCommand::ConfigRead(group, tx))
        .context("send config read")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config read")?
        .context("config read failed")?;
    match resp {
        ConfigResponse::Snapshot(snapshot) if snapshot.group == group => Ok(snapshot),
        other => bail!("device answered {other:?} to a read of {group:?}"),
    }
}

/// JSON value of a group response payload (None for non-payload variants).
#[must_use]
pub fn group_value(value: &ConfigValue) -> Option<Value> {
    match value {
        ConfigValue::BoardCalibration(v) => serde_json::to_value(v).ok(),
        ConfigValue::MotorParams(v) => serde_json::to_value(v).ok(),
        ConfigValue::CurrentLimits(v) => serde_json::to_value(v).ok(),
        ConfigValue::VoltageLimits(v) => serde_json::to_value(v).ok(),
        ConfigValue::PwmConfig(v) => serde_json::to_value(v).ok(),
        ConfigValue::PiGains(v) => serde_json::to_value(v).ok(),
        ConfigValue::HallTuning(v) => serde_json::to_value(v).ok(),
        ConfigValue::HallCalibration(v) => serde_json::to_value(v).ok(),
        ConfigValue::DcOffsets(v) => serde_json::to_value(v).ok(),
        ConfigValue::Failsafe(v) => serde_json::to_value(v).ok(),
        ConfigValue::Velocity(v) => serde_json::to_value(v).ok(),
        ConfigValue::Derating(v) => serde_json::to_value(v).ok(),
        ConfigValue::VoltageFeedforward(v) => serde_json::to_value(v).ok(),
        ConfigValue::MotorTemperature(v) => serde_json::to_value(v).ok(),
        ConfigValue::Drive(v) => serde_json::to_value(v).ok(),
        ConfigValue::MotorFeedback(v) => serde_json::to_value(v).ok(),
    }
}

/// Default JSON for a group (used when the device has nothing stored).
#[must_use]
pub fn group_default_value(group: ConfigGroupId) -> Value {
    use ConfigGroupId as G;
    use oxifoc_core::storage as st;
    let v = match group {
        G::BoardCalibration => serde_json::to_value(st::BoardCalibrationConfig::default()),
        G::MotorParams => serde_json::to_value(st::MotorParamsConfig::default()),
        G::HallCalibration => serde_json::to_value(st::HallCalibrationConfig::default()),
        G::DcOffsets => serde_json::to_value(st::DcOffsetsConfig::default()),
        G::CurrentLimits => serde_json::to_value(st::CurrentLimitsConfig::default()),
        G::VoltageLimits => serde_json::to_value(st::VoltageLimitsConfig::default()),
        G::PwmConfig => serde_json::to_value(st::PwmConfigStored::default()),
        G::PiGains => serde_json::to_value(st::PiGainsConfig::default()),
        G::HallTuning => serde_json::to_value(st::HallTuningConfig::default()),
        G::Failsafe => serde_json::to_value(st::FailsafeConfigStored::default()),
        G::Velocity => serde_json::to_value(st::VelocityLoopConfig::default()),
        G::Derating => serde_json::to_value(st::DeratingConfig::default()),
        G::VoltageFeedforward => serde_json::to_value(st::VoltageFeedforwardConfig::default()),
        G::MotorTemperature => serde_json::to_value(st::MotorTemperatureConfig::default()),
        G::Drive => serde_json::to_value(st::DriveConfig::default()),
        G::MotorFeedback => serde_json::to_value(st::MotorFeedbackConfig::default()),
    };
    v.expect("stored-config structs always serialize")
}

/// Build a typed `ConfigValue` back from a patched JSON object.
pub fn write_from_value(group: ConfigGroupId, v: Value) -> Result<ConfigValue> {
    use ConfigGroupId as G;
    Ok(match group {
        G::BoardCalibration => ConfigValue::BoardCalibration(serde_json::from_value(v)?),
        G::MotorParams => ConfigValue::MotorParams(serde_json::from_value(v)?),
        G::HallCalibration => ConfigValue::HallCalibration(serde_json::from_value(v)?),
        G::DcOffsets => ConfigValue::DcOffsets(serde_json::from_value(v)?),
        G::CurrentLimits => ConfigValue::CurrentLimits(serde_json::from_value(v)?),
        G::VoltageLimits => ConfigValue::VoltageLimits(serde_json::from_value(v)?),
        G::PwmConfig => ConfigValue::PwmConfig(serde_json::from_value(v)?),
        G::PiGains => ConfigValue::PiGains(serde_json::from_value(v)?),
        G::HallTuning => ConfigValue::HallTuning(serde_json::from_value(v)?),
        G::Failsafe => ConfigValue::Failsafe(serde_json::from_value(v)?),
        G::Velocity => ConfigValue::Velocity(serde_json::from_value(v)?),
        G::Derating => ConfigValue::Derating(serde_json::from_value(v)?),
        G::VoltageFeedforward => ConfigValue::VoltageFeedforward(serde_json::from_value(v)?),
        G::MotorTemperature => ConfigValue::MotorTemperature(serde_json::from_value(v)?),
        G::Drive => ConfigValue::Drive(serde_json::from_value(v)?),
        G::MotorFeedback => ConfigValue::MotorFeedback(serde_json::from_value(v)?),
    })
}

/// Current JSON of a group: the live value, or the typed default when the
/// firmware has no value for this group. `ConfigState` distinguishes absence
/// from a volatile value that has not yet reached flash.
pub fn current_value(cmd: &CommandSender, group: ConfigGroupId) -> Result<(Value, ConfigState)> {
    let snapshot = read_group(cmd, group)?;
    let present = snapshot.value.is_some();
    let value = match snapshot.value.as_ref() {
        Some(value) => group_value(value).context("config value is not JSON-serializable")?,
        None => snapshot
            .default
            .as_ref()
            .or(snapshot.effective.as_ref())
            .and_then(group_value)
            .unwrap_or_else(|| group_default_value(group)),
    };
    Ok((
        value,
        ConfigState {
            valid: snapshot.valid,
            revision: snapshot.revision,
            boot_epoch: snapshot.boot_epoch,
            persisted: snapshot.persisted,
            present,
            effective: snapshot.effective.as_ref().and_then(group_value),
        },
    ))
}

pub fn apply_write(
    cmd: &CommandSender,
    boot_epoch: u64,
    expected_revision: u32,
    write: ConfigValue,
) -> Result<u32> {
    write_revision(cmd, boot_epoch, expected_revision, write, false)
}

fn write_revision(
    cmd: &CommandSender,
    boot_epoch: u64,
    expected_revision: u32,
    write: ConfigValue,
    allow_stage: bool,
) -> Result<u32> {
    if !allow_stage && matches!(write.group(), ConfigGroupId::PwmConfig) {
        bail!(
            "{} takes effect after restart and must be saved",
            group_name(write.group())
        );
    }
    let (tx, rx) = config_channel();
    let id = next_config_id();
    let group = write.group();
    cmd.send(HostCommand::ConfigApply(
        Keyed::new(
            id,
            ConfigApply {
                boot_epoch,
                expected_revision,
                write,
            },
        ),
        tx,
    ))
    .context("send volatile config apply")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config apply")?
        .context("config apply failed")?;
    match resp {
        ConfigResponse::Applied { req_id, revision } if req_id == id => Ok(revision),
        ConfigResponse::Staged { req_id, revision } if allow_stage && req_id == id => Ok(revision),
        ConfigResponse::Busy => {
            bail!("device refused apply: stop the motor and wait for active maintenance to finish")
        }
        ConfigResponse::Invalid if group == ConfigGroupId::PwmConfig => bail!(
            "device rejected PWM configuration. Supported on MK5: freq_hz=15000 or 20000, \
             max_duty_percent=95"
        ),
        ConfigResponse::Invalid if group == ConfigGroupId::Derating => bail!(
            "device rejected derating configuration. Speed ceilings must be finite and non-negative, \
             speed_start_frac strictly between 0 and 1, and duty_start_frac in 0..=1"
        ),
        ConfigResponse::Invalid if group == ConfigGroupId::MotorFeedback => bail!(
            "feedback requires the selected sensors and a valid motor model for observer modes. Blend thresholds must be positive and increasing"
        ),
        ConfigResponse::Invalid => bail!(
            "device refused the write: value fails validation. Rules: all fields \
             finite; current_limits: max_phase_current_a >= 1.3 x max_iq_a (the \
             overcurrent trip needs headroom above full throttle); derating: every \
             enabled ramp well-formed (temp/regen end > start, battery cut start > \
             end, accel_dec in 0..=1); board-calibration: \
             vbus_gain must be finite and within 0.95..=1.05; voltage-limits: \
             0 < min_vbus_mv < max_vbus_mv, within board limits"
        ),
        ConfigResponse::Conflict { current_revision } => bail!(
            "config changed or controller restarted (expected revision {expected_revision}, current {current_revision}); re-read and retry"
        ),
        other => bail!("config apply rejected: {other:?}"),
    }
}

pub fn persist_group(
    cmd: &CommandSender,
    group: ConfigGroupId,
    boot_epoch: u64,
    expected_revision: u32,
) -> Result<bool> {
    let (tx, rx) = config_channel();
    let id = next_config_id();
    cmd.send(HostCommand::ConfigPersist(
        Keyed::new(
            id,
            ConfigPersist {
                group,
                boot_epoch,
                expected_revision,
            },
        ),
        tx,
    ))
    .context("send config persist")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config persist")?
        .context("config persist failed")?;
    match resp {
        ConfigResponse::Persisted { req_id, revision }
            if req_id == id && revision == expected_revision =>
        {
            Ok(true)
        }
        ConfigResponse::Unsupported => Ok(false),
        ConfigResponse::Busy => bail!(
            "device refused persistence: motor/maintenance is active; volatile value remains applied"
        ),
        ConfigResponse::Conflict { current_revision } => bail!(
            "config changed before persistence (expected revision {expected_revision}, current {current_revision}); current live value was not persisted"
        ),
        other => bail!("config persist rejected: {other:?}"),
    }
}

/// Apply a value, then persist that exact revision when flash is available.
pub fn send_write(cmd: &CommandSender, write: ConfigValue) -> Result<ConfigCommit> {
    let group = write.group();
    let snapshot = read_group(cmd, group)?;
    write_at_revision(cmd, snapshot.boot_epoch, snapshot.revision, write)
}

/// Apply against the snapshot the caller edited, then persist that revision.
pub fn write_at_revision(
    cmd: &CommandSender,
    boot_epoch: u64,
    expected_revision: u32,
    write: ConfigValue,
) -> Result<ConfigCommit> {
    let group = write.group();
    let revision = write_revision(cmd, boot_epoch, expected_revision, write, true)?;
    let persisted = persist_group(cmd, group, boot_epoch, revision)
        .with_context(|| format!("revision {revision} accepted in RAM; flash save failed"))?;
    Ok(ConfigCommit {
        revision,
        persisted,
    })
}

/// Patch fields in an already-read group value and build the typed write.
/// Kept separate from transport so GUI/CLI preservation semantics are directly
/// testable: fields absent from `kvs` must survive byte-for-byte.
pub fn patch_fields(
    group: ConfigGroupId,
    mut value: Value,
    kvs: &[String],
) -> Result<(Value, ConfigValue)> {
    for kv in kvs {
        let (key, raw) = kv
            .split_once('=')
            .with_context(|| format!("expected field=value, got '{kv}'"))?;
        let mut field = &mut value;
        for segment in key.split('.') {
            field = field
                .get_mut(segment)
                .with_context(|| format!("group {} has no field '{key}'", group_name(group)))?;
        }
        let parsed =
            serde_json::from_str::<Value>(raw).unwrap_or_else(|_| Value::String(raw.to_string()));
        *field = parsed;
    }

    let write = write_from_value(group, value.clone())
        .with_context(|| format!("patched {} no longer deserializes", group_name(group)))?;
    Ok((value, write))
}

/// Volatile read-modify-write of a config group.
///
/// Values are parsed as JSON (numbers, booleans, arrays); anything that
/// fails to parse is taken as a string. Returns the resulting group JSON and
/// the new live revision; this function never requests flash persistence.
pub fn apply_fields(
    cmd: &CommandSender,
    group: ConfigGroupId,
    kvs: &[String],
) -> Result<(Value, u32)> {
    let (value, state) = current_value(cmd, group)?;
    let (value, write) = patch_fields(group, value, kvs)?;
    let revision = apply_write(cmd, state.boot_epoch, state.revision, write)?;
    Ok((value, revision))
}

/// Read-modify-write and persist. Use `apply_fields` for a temporary live test.
pub fn set_fields(
    cmd: &CommandSender,
    group: ConfigGroupId,
    kvs: &[String],
) -> Result<(Value, ConfigCommit)> {
    let (value, state) = current_value(cmd, group)?;
    let (value, write) = patch_fields(group, value, kvs)?;
    let commit = write_at_revision(cmd, state.boot_epoch, state.revision, write)?;
    Ok((value, commit))
}

/// Snapshot of every config group as one JSON object (stored groups only).
#[must_use]
pub fn config_snapshot(cmd: &CommandSender) -> Value {
    let mut obj = serde_json::Map::new();
    for (name, group) in GROUPS {
        if let Ok(snapshot) = read_group(cmd, group)
            && let Some(value) = snapshot.value.as_ref()
            && let Some(v) = group_value(value)
        {
            obj.insert(name.to_string(), v);
        }
    }
    Value::Object(obj)
}

/// Factory reset: erase every stored config group.
pub fn reset_all(cmd: &CommandSender) -> Result<()> {
    let (tx, rx) = config_channel();
    let boot_epoch = read_group(cmd, ConfigGroupId::Drive)?.boot_epoch;
    cmd.send(HostCommand::ConfigResetAll(
        Keyed::new(next_config_id(), boot_epoch),
        tx,
    ))
    .context("send config reset")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config reset")?
        .context("config reset failed")?;
    match resp {
        ConfigResponse::Reset => Ok(()),
        ConfigResponse::ResetRequiresRestart => {
            bail!("reset did not complete; restart the controller and read back its configuration")
        }
        other => bail!("config reset rejected: {other:?}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::storage::{MotorParamsConfig, VelocityLoopConfig, VoltageFeedforwardConfig};
    use oxifoc_core::types::VoltageFeedforwardMode;

    #[test]
    fn hall_json_preserves_complete_optional_geometry_and_accepts_legacy_omission() {
        use oxifoc_core::storage::{HallCalibrationConfig, HallTransitionGeometryConfig};
        let group = ConfigGroupId::HallCalibration;
        let default = group_default_value(group);
        assert!(default["geometry"].is_null());
        let mut legacy = default.clone();
        legacy.as_object_mut().unwrap().remove("geometry");
        assert_eq!(
            write_from_value(group, legacy).unwrap(),
            ConfigValue::HallCalibration(HallCalibrationConfig::default())
        );
        let tau = core::f32::consts::TAU;
        let config = HallCalibrationConfig {
            geometry: Some(HallTransitionGeometryConfig {
                entry_angles: [
                    0.0,
                    0.0,
                    tau / 3.0,
                    tau / 6.0,
                    2.0 * tau / 3.0,
                    5.0 * tau / 6.0,
                    tau / 2.0,
                    0.0,
                ],
                next_states: [0, 3, 6, 2, 5, 1, 4, 0],
            }),
            ..Default::default()
        };
        assert!(config.is_calibrated());
        let json = group_value(&ConfigValue::HallCalibration(config)).unwrap();
        assert_eq!(
            write_from_value(group, json.clone()).unwrap(),
            ConfigValue::HallCalibration(config)
        );
        let (_, write) =
            patch_fields(group, default, &[format!("geometry={}", json["geometry"])]).unwrap();
        assert_eq!(write, ConfigValue::HallCalibration(config));
    }

    #[test]
    fn voltage_feedforward_defaults_and_modes_use_the_typed_wire_schema() {
        let group = ConfigGroupId::VoltageFeedforward;
        let default = group_default_value(group);
        assert_eq!(default, serde_json::json!({ "mode": "Disabled" }));
        for mode in [
            VoltageFeedforwardMode::Disabled,
            VoltageFeedforwardMode::CrossCoupling,
            VoltageFeedforwardMode::BackEmf,
            VoltageFeedforwardMode::Both,
        ] {
            let config = VoltageFeedforwardConfig { mode };
            let (json, write) =
                patch_fields(group, default.clone(), &[format!("mode={mode:?}")]).unwrap();
            assert_eq!(write, ConfigValue::VoltageFeedforward(config));
            assert_eq!(
                group_value(&ConfigValue::VoltageFeedforward(config)),
                Some(json)
            );
        }
        assert!(patch_fields(group, default.clone(), &["mode=enabled".into()]).is_err());
        assert!(patch_fields(group, default, &["enabled=true".into()]).is_err());
    }

    #[test]
    fn volatile_feedforward_apply_uses_snapshot_revision_and_never_persists() {
        let (cmd, mut requests) = tokio::sync::mpsc::unbounded_channel();
        let backend = std::thread::spawn(move || {
            let Some(HostCommand::ConfigRead(group, reply)) = requests.blocking_recv() else {
                panic!("expected config snapshot request");
            };
            assert_eq!(group, ConfigGroupId::VoltageFeedforward);
            reply
                .send(Ok(ConfigResponse::Snapshot(ConfigSnapshot {
                    effective: None,
                    default: None,
                    boot_epoch: 12,
                    group,
                    revision: 7,
                    persisted: false,
                    valid: true,
                    value: None,
                })))
                .unwrap();
            let Some(HostCommand::ConfigApply(request, reply)) = requests.blocking_recv() else {
                panic!("expected volatile config apply");
            };
            assert_eq!(request.inner.expected_revision, 7);
            assert_eq!(request.inner.boot_epoch, 12);
            assert_eq!(
                request.inner.write,
                ConfigValue::VoltageFeedforward(VoltageFeedforwardConfig {
                    mode: VoltageFeedforwardMode::Both,
                })
            );
            reply
                .send(Ok(ConfigResponse::Applied {
                    req_id: request.id,
                    revision: 8,
                }))
                .unwrap();
            assert!(
                requests.blocking_recv().is_none(),
                "volatile apply must not persist"
            );
        });
        let (value, revision) = apply_fields(
            &cmd,
            ConfigGroupId::VoltageFeedforward,
            &["mode=Both".into()],
        )
        .unwrap();
        assert_eq!(value, serde_json::json!({ "mode": "Both" }));
        assert_eq!(revision, 8);
        drop(cmd);
        backend.join().unwrap();
    }

    #[test]
    fn field_patches_preserve_unmentioned_motor_and_velocity_values() {
        let motor = MotorParamsConfig {
            resistance_ohm: 0.1,
            ld_fundamental_h: 81e-6,
            lq_fundamental_h: 127e-6,
            ..Default::default()
        };
        let (_, write) = patch_fields(
            ConfigGroupId::MotorParams,
            serde_json::to_value(motor).unwrap(),
            &["resistance_ohm=0.2".into()],
        )
        .unwrap();
        let ConfigValue::MotorParams(patched) = write else {
            panic!("wrong write variant");
        };
        assert_eq!(patched.resistance_ohm, 0.2);
        assert_eq!(patched.ld_fundamental_h, 81e-6);
        assert_eq!(patched.lq_fundamental_h, 127e-6);

        let velocity = VelocityLoopConfig {
            accel_ff: 0.75,
            ..Default::default()
        };
        let (_, write) = patch_fields(
            ConfigGroupId::Velocity,
            serde_json::to_value(velocity).unwrap(),
            &["kp=0.3".into()],
        )
        .unwrap();
        let ConfigValue::Velocity(patched) = write else {
            panic!("wrong write variant");
        };
        assert_eq!(patched.kp, 0.3);
        assert_eq!(patched.accel_ff, 0.75);
    }

    /// Enforcer: `GROUPS` is a hand-maintained mirror of the wire enum and
    /// this is the only thing that catches a newly appended group — the
    /// typed matches (`write_from_value` & co) are compile-enforced, an
    /// array of a stale length is not.
    #[test]
    fn groups_table_mirrors_protocol_order() {
        assert_eq!(GROUPS.len(), ConfigGroupId::COUNT);
        for (i, (name, group)) in GROUPS.iter().enumerate() {
            assert_eq!(
                *group,
                ConfigGroupId::ALL[i],
                "GROUPS[{i}] ({name}) out of protocol order"
            );
        }
    }

    /// Every group id resolves to a unique kebab-case name that parses back.
    #[test]
    fn group_names_round_trip() {
        for group in ConfigGroupId::ALL {
            let name = group_name(group);
            assert_ne!(name, "?", "{group:?} missing from GROUPS");
            assert_eq!(parse_group(name).unwrap(), group);
        }
    }
}

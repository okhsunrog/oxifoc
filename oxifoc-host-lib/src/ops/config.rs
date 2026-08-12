//! Config get/set — generic over every group via a JSON round-trip.
//!
//! All stored-config structs derive `Serialize + Deserialize + Default`, so
//! field-level writes are implemented without per-field code: read the
//! current group (or its default when nothing is stored), patch keys on the
//! JSON object, deserialize back into the typed struct and send a
//! `ConfigWrite`. Unknown field names are rejected against the actual
//! object's keys (serde would silently ignore them otherwise).
//!
//! Lives in host-lib (not the CLI) so the GUI shares the exact same group
//! schema, defaults and write/validation handling. CLI-only presentation
//! (`config dump --rust`) stays in the CLI.

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{
    ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, ConfigSnapshot, ConfigValue,
    ConfigWrite, Keyed, ReqId,
};
use serde_json::Value;

use crate::{CommandSender, HostCommand, config_channel};

/// kebab-case CLI name ↔ group id, in protocol order.
pub const GROUPS: [(&str, ConfigGroupId); 11] = [
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
        .find(|(_, g)| format!("{g:?}") == format!("{group:?}"))
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

fn next_config_id() -> ReqId {
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

#[derive(Clone, Copy, Debug)]
pub struct ConfigState {
    pub revision: u32,
    pub persisted: bool,
    pub present: bool,
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
    }
}

/// Default JSON for a group (used when the device has nothing stored).
#[must_use]
pub fn group_default_value(group: ConfigGroupId) -> Value {
    use ConfigGroupId as G;
    use oxifoc_core::storage as st;
    let v = match group {
        G::MotorParams => serde_json::to_value(st::MotorParamsConfig::default()),
        G::HallCalibration => serde_json::to_value(st::HallCalibrationConfig::default()),
        G::DcOffsets => serde_json::to_value(st::DcOffsetsConfig::default()),
        G::CurrentLimits => serde_json::to_value(st::CurrentLimitsConfig::default()),
        G::VoltageLimits => serde_json::to_value(st::VoltageLimitsConfig::default()),
        G::PwmConfig => serde_json::to_value(st::PwmConfigStored::default()),
        G::PiGains => serde_json::to_value(st::PiGainsConfig::default()),
        G::HallTuning => serde_json::to_value(st::HallTuningConfig::default()),
        G::Failsafe => serde_json::to_value(st::FailsafeConfigStored::default()),
        G::Velocity => serde_json::to_value(st::VelocityConfigStored::default()),
        G::Derating => serde_json::to_value(st::DeratingConfigStored::default()),
    };
    v.expect("stored-config structs always serialize")
}

/// Build a typed `ConfigWrite` back from a patched JSON object.
pub fn write_from_value(group: ConfigGroupId, v: Value) -> Result<ConfigWrite> {
    use ConfigGroupId as G;
    Ok(match group {
        G::MotorParams => ConfigWrite::MotorParams(serde_json::from_value(v)?),
        G::HallCalibration => ConfigWrite::HallCalibration(serde_json::from_value(v)?),
        G::DcOffsets => ConfigWrite::DcOffsets(serde_json::from_value(v)?),
        G::CurrentLimits => ConfigWrite::CurrentLimits(serde_json::from_value(v)?),
        G::VoltageLimits => ConfigWrite::VoltageLimits(serde_json::from_value(v)?),
        G::PwmConfig => ConfigWrite::PwmConfig(serde_json::from_value(v)?),
        G::PiGains => ConfigWrite::PiGains(serde_json::from_value(v)?),
        G::HallTuning => ConfigWrite::HallTuning(serde_json::from_value(v)?),
        G::Failsafe => ConfigWrite::Failsafe(serde_json::from_value(v)?),
        G::Velocity => ConfigWrite::Velocity(serde_json::from_value(v)?),
        G::Derating => ConfigWrite::Derating(serde_json::from_value(v)?),
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
        None => group_default_value(group),
    };
    Ok((
        value,
        ConfigState {
            revision: snapshot.revision,
            persisted: snapshot.persisted,
            present,
        },
    ))
}

pub fn apply_write(cmd: &CommandSender, expected_revision: u32, write: ConfigWrite) -> Result<u32> {
    let (tx, rx) = config_channel();
    let id = next_config_id();
    cmd.send(HostCommand::ConfigApply(
        Keyed::new(
            id,
            ConfigApply {
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
        ConfigResponse::Busy => {
            bail!("device refused volatile apply: this group is not live-safe while running")
        }
        ConfigResponse::Invalid => bail!(
            "device refused the write: value fails validation. Rules: all fields \
             finite; current_limits: max_phase_current_a >= 1.3 x max_iq_a (the \
             overcurrent trip needs headroom above full throttle); derating: every \
             enabled ramp well-formed (temp/regen end > start, battery cut start > \
             end, accel_dec and speed_start_frac in 0..=1)"
        ),
        ConfigResponse::Conflict { current_revision } => bail!(
            "config changed concurrently (expected revision {expected_revision}, current {current_revision}); re-read and retry"
        ),
        other => bail!("config apply rejected: {other:?}"),
    }
}

pub fn persist_group(
    cmd: &CommandSender,
    group: ConfigGroupId,
    expected_revision: u32,
) -> Result<bool> {
    let (tx, rx) = config_channel();
    let id = next_config_id();
    cmd.send(HostCommand::ConfigPersist(
        Keyed::new(
            id,
            ConfigPersist {
                group,
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
pub fn send_write(cmd: &CommandSender, write: ConfigWrite) -> Result<ConfigCommit> {
    let group = write.group();
    let snapshot = read_group(cmd, group)?;
    let revision = apply_write(cmd, snapshot.revision, write)?;
    let persisted = persist_group(cmd, group, revision)?;
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
) -> Result<(Value, ConfigWrite)> {
    let obj = value
        .as_object_mut()
        .context("config group is not a JSON object")?;

    for kv in kvs {
        let (key, raw) = kv
            .split_once('=')
            .with_context(|| format!("expected field=value, got '{kv}'"))?;
        if !obj.contains_key(key) {
            let fields: Vec<&str> = obj.keys().map(String::as_str).collect();
            bail!(
                "group {} has no field '{key}'; fields: {}",
                group_name(group),
                fields.join(", ")
            );
        }
        let parsed =
            serde_json::from_str::<Value>(raw).unwrap_or_else(|_| Value::String(raw.to_string()));
        obj.insert(key.to_string(), parsed);
    }

    let write = write_from_value(group, value.clone())
        .with_context(|| format!("patched {} no longer deserializes", group_name(group)))?;
    Ok((value, write))
}

/// `config set GROUP field=value ...` — read-modify-write.
///
/// Values are parsed as JSON (numbers, booleans, arrays); anything that
/// fails to parse is taken as a string. Returns the resulting group JSON and
/// whether its new revision was persisted.
pub fn set_fields(
    cmd: &CommandSender,
    group: ConfigGroupId,
    kvs: &[String],
) -> Result<(Value, ConfigCommit)> {
    let (value, state) = current_value(cmd, group)?;
    let (value, write) = patch_fields(group, value, kvs)?;
    let revision = apply_write(cmd, state.revision, write)?;
    let persisted = persist_group(cmd, group, revision)?;
    Ok((
        value,
        ConfigCommit {
            revision,
            persisted,
        },
    ))
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
    cmd.send(HostCommand::ConfigResetAll(tx))
        .context("send config reset")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config reset")?
        .context("config reset failed")?;
    match resp {
        ConfigResponse::Reset => Ok(()),
        other => bail!("config reset rejected: {other:?}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::storage::{MotorParamsConfig, VelocityConfigStored};

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
        let ConfigWrite::MotorParams(patched) = write else {
            panic!("wrong write variant");
        };
        assert_eq!(patched.resistance_ohm, 0.2);
        assert_eq!(patched.ld_fundamental_h, 81e-6);
        assert_eq!(patched.lq_fundamental_h, 127e-6);

        let velocity = VelocityConfigStored {
            accel_ff: 0.75,
            ..Default::default()
        };
        let (_, write) = patch_fields(
            ConfigGroupId::Velocity,
            serde_json::to_value(velocity).unwrap(),
            &["kp=0.3".into()],
        )
        .unwrap();
        let ConfigWrite::Velocity(patched) = write else {
            panic!("wrong write variant");
        };
        assert_eq!(patched.kp, 0.3);
        assert_eq!(patched.accel_ff, 0.75);
    }
}

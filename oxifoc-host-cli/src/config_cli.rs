//! Config get/set/dump — generic over every group via a JSON round-trip.
//!
//! All stored-config structs derive `Serialize + Deserialize + Default`, so
//! field-level writes are implemented without per-field code: read the
//! current group (or its default when nothing is stored), patch keys on the
//! JSON object, deserialize back into the typed struct and send a
//! `ConfigWrite`. Unknown field names are rejected against the actual
//! object's keys (serde would silently ignore them otherwise).

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{ConfigGroupId, ConfigResponse, ConfigWrite};
use oxifoc_host_lib::{HostCommand, HostRuntime, config_channel};
use serde_json::Value;

/// kebab-case CLI name ↔ group id, in protocol order.
pub const GROUPS: [(&str, ConfigGroupId); 10] = [
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
];

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

pub fn group_name(group: ConfigGroupId) -> &'static str {
    GROUPS
        .iter()
        .find(|(_, g)| format!("{g:?}") == format!("{group:?}"))
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

/// Read one config group (None when the device has nothing stored for it).
pub fn read_group(runtime: &HostRuntime, group: ConfigGroupId) -> Result<Option<ConfigResponse>> {
    let (tx, rx) = config_channel();
    runtime
        .cmd_tx
        .send(HostCommand::ConfigRead(group, tx))
        .context("send config read")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config read")?
        .context("config read failed")?;
    Ok(match resp {
        ConfigResponse::NotFound => None,
        other => Some(other),
    })
}

/// JSON value of a group response payload (None for non-payload variants).
pub fn group_value(resp: &ConfigResponse) -> Option<Value> {
    use ConfigResponse as R;
    match resp {
        R::MotorParams(v) => serde_json::to_value(v).ok(),
        R::CurrentLimits(v) => serde_json::to_value(v).ok(),
        R::VoltageLimits(v) => serde_json::to_value(v).ok(),
        R::PwmConfig(v) => serde_json::to_value(v).ok(),
        R::PiGains(v) => serde_json::to_value(v).ok(),
        R::HallTuning(v) => serde_json::to_value(v).ok(),
        R::HallCalibration(v) => serde_json::to_value(v).ok(),
        R::DcOffsets(v) => serde_json::to_value(v).ok(),
        R::Failsafe(v) => serde_json::to_value(v).ok(),
        R::Velocity(v) => serde_json::to_value(v).ok(),
        R::Ok | R::NotFound | R::Error | R::Busy | R::Invalid => None,
    }
}

/// Default JSON for a group (used when the device has nothing stored).
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
    })
}

/// Current JSON of a group: stored value, or the typed default.
pub fn current_value(runtime: &HostRuntime, group: ConfigGroupId) -> Result<(Value, bool)> {
    Ok(match read_group(runtime, group)? {
        Some(resp) => match group_value(&resp) {
            Some(v) => (v, true),
            None => bail!("device answered {resp:?} to a read of {group:?}"),
        },
        None => (group_default_value(group), false),
    })
}

/// Send a `ConfigWrite` and require the `Ok` ack.
pub fn send_write(runtime: &HostRuntime, write: ConfigWrite) -> Result<()> {
    let (tx, rx) = config_channel();
    runtime
        .cmd_tx
        .send(HostCommand::ConfigWrite(write, tx))
        .context("send config write")?;
    let resp = rx
        .blocking_recv()
        .context("backend dropped the config write")?
        .context("config write failed")?;
    match resp {
        ConfigResponse::Ok => Ok(()),
        ConfigResponse::Busy => bail!(
            "device refused the write: motor is running (flash writes stall the control loop)"
        ),
        ConfigResponse::Invalid => bail!(
            "device refused the write: value fails validation (current_limits: every \
             field must be finite and max_phase_current_a >= 1.3 x max_iq_a — the \
             overcurrent trip needs headroom above full throttle)"
        ),
        other => bail!("config write rejected: {other:?}"),
    }
}

/// `config set GROUP field=value ...` — read-modify-write.
///
/// Values are parsed as JSON (numbers, booleans, arrays); anything that
/// fails to parse is taken as a string. Returns the resulting group JSON.
pub fn set_fields(runtime: &HostRuntime, group: ConfigGroupId, kvs: &[String]) -> Result<Value> {
    let (mut value, _stored) = current_value(runtime, group)?;
    let obj = value
        .as_object_mut()
        .context("config group is not a JSON object")?;

    for kv in kvs {
        let (key, raw) = kv
            .split_once('=')
            .with_context(|| format!("expected field=value, got '{kv}'"))?;
        if !obj.contains_key(key) {
            let fields: Vec<&str> = obj.keys().map(|s| s.as_str()).collect();
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
    send_write(runtime, write)?;
    Ok(value)
}

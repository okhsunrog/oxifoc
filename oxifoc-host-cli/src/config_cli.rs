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
        R::Derating(v) => serde_json::to_value(v).ok(),
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
            "device refused the write: value fails validation. Rules: all fields \
             finite; current_limits: max_phase_current_a >= 1.3 x max_iq_a (the \
             overcurrent trip needs headroom above full throttle); derating: every \
             enabled ramp well-formed (temp/regen end > start, battery cut start > \
             end, accel_dec and speed_start_frac in 0..=1)"
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
    send_write(runtime, write)?;
    Ok(value)
}

/// Snapshot of every config group as one JSON object (stored groups only).
pub fn config_snapshot(runtime: &HostRuntime) -> Value {
    let mut obj = serde_json::Map::new();
    for (name, group) in GROUPS {
        if let Ok(Some(resp)) = read_group(runtime, group)
            && let Some(v) = group_value(&resp)
        {
            obj.insert(name.to_string(), v);
        }
    }
    Value::Object(obj)
}

/// Dump every config group; `--rust` renders a ready-to-paste
/// `baked_config.rs` body for the baked firmware profile.
pub fn dump_config(runtime: &HostRuntime, rust: bool, json_mode: bool) -> Result<()> {
    if json_mode && !rust {
        let mut obj = serde_json::Map::new();
        for (name, group) in GROUPS {
            let v = match read_group(runtime, group)? {
                Some(resp) => group_value(&resp).unwrap_or(Value::Null),
                None => Value::Null,
            };
            obj.insert(name.to_string(), v);
        }
        println!("{:#}", Value::Object(obj));
        return Ok(());
    }

    let mut read = Vec::new();
    for (_, g) in GROUPS {
        read.push((g, read_group(runtime, g)?));
    }

    if !rust {
        for (g, resp) in &read {
            match resp {
                Some(r) => println!("{g:?}: {r:?}"),
                None => println!("{g:?}: (not stored)"),
            }
        }
        return Ok(());
    }

    // Rust emission: via the JSON representation so the output stays in
    // sync with the structs without per-field format strings here.
    println!("//! Compiled-in configuration for the baked profile (`storage` feature off).");
    println!("//!");
    println!("//! Generated by `oxifoc-host-cli config dump --rust`. Regenerate after");
    println!("//! re-detection or re-tuning, then rebuild and reflash.");
    println!();
    println!("use oxifoc_core::storage::*;");
    println!();
    println!("/// The baked configuration.");
    println!("pub fn baked() -> RuntimeConfig {{");
    println!("    RuntimeConfig {{");
    for (name, group) in GROUPS {
        let field = name.replace('-', "_");
        let resp = read
            .iter()
            .find(|(g, _)| format!("{g:?}") == format!("{group:?}"))
            .and_then(|(_, r)| r.as_ref());
        match resp.and_then(group_value) {
            Some(v) => {
                let struct_name = rust_struct_name(name);
                println!("        {field}: Some({struct_name} {{");
                if let Some(obj) = v.as_object() {
                    for (k, val) in obj {
                        println!("            {k}: {},", rust_literal(val));
                    }
                }
                println!("        }}),");
            }
            None => println!("        {field}: None,"),
        }
    }
    println!("    }}");
    println!("}}");
    Ok(())
}

pub fn rust_struct_name(group: &str) -> &'static str {
    match group {
        "motor-params" => "MotorParamsConfig",
        "hall-calibration" => "HallCalibrationConfig",
        "dc-offsets" => "DcOffsetsConfig",
        "current-limits" => "CurrentLimitsConfig",
        "voltage-limits" => "VoltageLimitsConfig",
        "pwm-config" => "PwmConfigStored",
        "pi-gains" => "PiGainsConfig",
        "hall-tuning" => "HallTuningConfig",
        "failsafe" => "FailsafeConfigStored",
        "velocity" => "VelocityConfigStored",
        "derating" => "DeratingConfigStored",
        _ => unreachable!(),
    }
}

/// Render a JSON value as a Rust literal for baked_config emission.
pub fn rust_literal(v: &Value) -> String {
    match v {
        Value::Number(n) => {
            // f32 fields must keep a float literal even for round values.
            if n.is_f64() {
                let f = n.as_f64().unwrap() as f32;
                format!("{f:?}")
            } else {
                format!("{n}")
            }
        }
        Value::Bool(b) => format!("{b}"),
        Value::Array(items) => {
            let inner: Vec<String> = items.iter().map(rust_literal).collect();
            format!("[{}]", inner.join(", "))
        }
        other => format!("{other}"),
    }
}

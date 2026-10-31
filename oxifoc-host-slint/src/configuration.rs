//! Configuration UI integration.

use super::{App, SharedRuntime, parse_field};
use oxifoc_core::types::{ConfigResponse, ConfigValue};
use oxifoc_host_lib::{HostCommand, config_channel, ops};
use slint::ComponentHandle;
use slint::SharedString;
use std::thread;

/// Config groups exposed by the GUI, indexed by the `config-group` ComboBox
/// in `ui/app.slint` — MUST stay in the same order as that ComboBox `model`
/// (the write path's per-group forms are keyed by the same index). This
/// const is the single Rust-side copy of that mapping.
const GUI_CONFIG_GROUPS: [oxifoc_core::types::ConfigGroupId; 6] = {
    use oxifoc_core::types::ConfigGroupId as G;
    [
        G::MotorParams,   // "Motor Params"
        G::CurrentLimits, // "Current Limits"
        G::VoltageLimits, // "Voltage Limits" (read-only)
        G::PiGains,       // "PI Gains"
        G::Velocity,      // "Velocity Loop"
        G::Failsafe,      // "Failsafe"
    ]
};

pub(super) fn bind(app: &App, runtime: &SharedRuntime) {
    // ── Config read ──────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_read(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                tracing::warn!("Config read clicked but no runtime");
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();
            let group_idx = app.get_config_group();

            let Some(group_id) = usize::try_from(group_idx)
                .ok()
                .and_then(|i| GUI_CONFIG_GROUPS.get(i).copied())
            else {
                return;
            };

            let (tx, rx) = config_channel();
            if runtime
                .cmd_tx
                .send(HostCommand::ConfigRead(group_id, tx))
                .is_err()
            {
                return;
            }

            thread::spawn(move || {
                let result = rx.blocking_recv();
                let _ = weak.upgrade_in_event_loop(move |app| match result {
                    Ok(Ok(ConfigResponse::Snapshot(snapshot))) => {
                        match snapshot.value {
                            Some(ConfigValue::MotorParams(p)) => {
                                app.set_cfg_resistance(SharedString::from(format!(
                                    "{}",
                                    p.resistance_ohm
                                )));
                                app.set_cfg_inductance_d(SharedString::from(format!(
                                    "{}",
                                    p.inductance_d_h
                                )));
                                app.set_cfg_inductance_q(SharedString::from(format!(
                                    "{}",
                                    p.inductance_q_h
                                )));
                                app.set_cfg_flux_linkage(SharedString::from(format!(
                                    "{}",
                                    p.flux_linkage_wb
                                )));
                                app.set_cfg_pole_pairs(SharedString::from(format!(
                                    "{}",
                                    p.pole_pairs
                                )));
                                app.set_cfg_motor_rating(SharedString::from(format!(
                                    "{}",
                                    p.max_current_a
                                )));
                                app.set_cfg_motor_power_loss(SharedString::from(format!(
                                    "{}",
                                    p.max_power_loss_w
                                )));
                            }
                            Some(ConfigValue::CurrentLimits(c)) => {
                                app.set_cfg_max_iq(SharedString::from(format!("{}", c.max_iq_a)));
                                app.set_cfg_max_brake(SharedString::from(format!(
                                    "{}",
                                    c.max_brake_a
                                )));
                                app.set_cfg_max_phase_current(SharedString::from(format!(
                                    "{}",
                                    c.max_phase_current_a
                                )));
                                app.set_cfg_bus_in_max(SharedString::from(format!(
                                    "{}",
                                    c.bus_in_max_a
                                )));
                                app.set_cfg_bus_regen_max(SharedString::from(format!(
                                    "{}",
                                    c.bus_regen_max_a
                                )));
                                app.set_cfg_max_dissipation(SharedString::from(format!(
                                    "{}",
                                    c.max_dissipation_w
                                )));
                            }
                            Some(ConfigValue::VoltageLimits(v)) => {
                                app.set_cfg_min_vbus(SharedString::from(format!(
                                    "{}",
                                    v.min_vbus_mv
                                )));
                                app.set_cfg_max_vbus(SharedString::from(format!(
                                    "{}",
                                    v.max_vbus_mv
                                )));
                            }
                            Some(ConfigValue::PiGains(g)) => {
                                app.set_cfg_kp(SharedString::from(format!("{}", g.kp)));
                                app.set_cfg_ki(SharedString::from(format!("{}", g.ki)));
                                app.set_cfg_bandwidth(SharedString::from(format!(
                                    "{}",
                                    g.bandwidth_rad_s
                                )));
                            }
                            Some(ConfigValue::Velocity(v)) => {
                                app.set_cfg_vel_kp(SharedString::from(format!("{}", v.kp)));
                                app.set_cfg_vel_ki(SharedString::from(format!("{}", v.ki)));
                                app.set_cfg_vel_accel(SharedString::from(format!(
                                    "{}",
                                    v.accel_limit
                                )));
                            }
                            Some(ConfigValue::Failsafe(f)) => {
                                app.set_cfg_fs_staleness(SharedString::from(format!(
                                    "{}",
                                    f.staleness_timeout_ms
                                )));
                                app.set_cfg_fs_policy(SharedString::from(format!("{}", f.policy)));
                                app.set_cfg_fs_brake_current(SharedString::from(format!(
                                    "{}",
                                    f.brake_current_a
                                )));
                                app.set_cfg_fs_slew(SharedString::from(format!(
                                    "{}",
                                    f.slew_a_per_s
                                )));
                                app.set_cfg_fs_standstill(SharedString::from(format!(
                                    "{}",
                                    f.standstill_rad_s
                                )));
                                app.set_cfg_fs_decel(SharedString::from(format!(
                                    "{}",
                                    f.decel_rad_s2
                                )));
                                app.set_cfg_fs_terminal(SharedString::from(format!(
                                    "{}",
                                    f.terminal
                                )));
                            }
                            None => {
                                app.set_config_status(SharedString::from(format!(
                                    "Revision {}: not configured",
                                    snapshot.revision
                                )));
                                return;
                            }
                            _ => {}
                        }
                        app.set_config_status(SharedString::from(format!(
                            "Revision {}: {}",
                            snapshot.revision,
                            if snapshot.persisted {
                                "persisted"
                            } else {
                                "volatile"
                            }
                        )));
                    }
                    Ok(Ok(other)) => app.set_config_status(SharedString::from(format!(
                        "Unexpected response: {other:?}"
                    ))),
                    Ok(Err(e)) => {
                        app.set_config_status(SharedString::from(format!("Error: {e}")));
                    }
                    Err(_) => {
                        app.set_config_status(SharedString::from("No response"));
                    }
                });
            });
        });
    }

    // ── Config write ─────────────────────────────────────────────────────────
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_write(move || {
            let guard = rt.lock().unwrap();
            let Some(ref runtime) = *guard else {
                tracing::warn!("Config write clicked but no runtime");
                return;
            };
            let weak = weak.clone();
            let app = weak.unwrap();
            let group_idx = app.get_config_group();

            use oxifoc_core::storage::*;
            use oxifoc_core::types::{ConfigGroupId, ConfigWrite};

            enum ConfigAction {
                Replace(ConfigWrite),
                Patch {
                    group: ConfigGroupId,
                    fields: Vec<String>,
                },
            }

            let mut parse_err: Option<String> = None;
            let err = &mut parse_err;
            let write = match group_idx {
                0 => {
                    let r: f32 = parse_field("resistance", &app.get_cfg_resistance(), err);
                    let ld: f32 = parse_field("inductance d", &app.get_cfg_inductance_d(), err);
                    let lq: f32 = parse_field("inductance q", &app.get_cfg_inductance_q(), err);
                    let fl: f32 = parse_field("flux linkage", &app.get_cfg_flux_linkage(), err);
                    let pp: u8 = parse_field("pole pairs", &app.get_cfg_pole_pairs(), err);
                    let rating: f32 = parse_field("motor rating", &app.get_cfg_motor_rating(), err);
                    let ploss: f32 =
                        parse_field("motor power loss", &app.get_cfg_motor_power_loss(), err);
                    ConfigAction::Patch {
                        group: ConfigGroupId::MotorParams,
                        fields: vec![
                            format!("resistance_ohm={r}"),
                            format!("inductance_d_h={ld}"),
                            format!("inductance_q_h={lq}"),
                            format!("flux_linkage_wb={fl}"),
                            format!("pole_pairs={pp}"),
                            format!("max_current_a={rating}"),
                            format!("max_power_loss_w={ploss}"),
                        ],
                    }
                }
                1 => {
                    let iq: f32 = parse_field("max iq", &app.get_cfg_max_iq(), err);
                    let brake: f32 = parse_field("max brake", &app.get_cfg_max_brake(), err);
                    let ph: f32 =
                        parse_field("max phase current", &app.get_cfg_max_phase_current(), err);
                    let bus_in: f32 = parse_field("bus in max", &app.get_cfg_bus_in_max(), err);
                    let bus_regen: f32 =
                        parse_field("bus regen max", &app.get_cfg_bus_regen_max(), err);
                    let dissipation: f32 =
                        parse_field("dissipation max", &app.get_cfg_max_dissipation(), err);
                    ConfigAction::Replace(ConfigWrite::CurrentLimits(CurrentLimitsConfig {
                        max_iq_a: iq,
                        max_brake_a: brake,
                        max_phase_current_a: ph,
                        bus_in_max_a: bus_in,
                        bus_regen_max_a: bus_regen,
                        max_dissipation_w: dissipation,
                    }))
                }
                2 => {
                    app.set_config_status(SharedString::from(
                        "Voltage Limits are read-only until firmware runtime support is added",
                    ));
                    return;
                }
                3 => {
                    let kp: f32 = parse_field("kp", &app.get_cfg_kp(), err);
                    let ki: f32 = parse_field("ki", &app.get_cfg_ki(), err);
                    let bw: f32 = parse_field("bandwidth", &app.get_cfg_bandwidth(), err);
                    ConfigAction::Replace(ConfigWrite::PiGains(PiGainsConfig {
                        kp,
                        ki,
                        bandwidth_rad_s: bw,
                    }))
                }
                4 => {
                    let kp: f32 = parse_field("velocity kp", &app.get_cfg_vel_kp(), err);
                    let ki: f32 = parse_field("velocity ki", &app.get_cfg_vel_ki(), err);
                    let accel: f32 = parse_field("accel limit", &app.get_cfg_vel_accel(), err);
                    ConfigAction::Patch {
                        group: ConfigGroupId::Velocity,
                        fields: vec![
                            format!("kp={kp}"),
                            format!("ki={ki}"),
                            format!("accel_limit={accel}"),
                        ],
                    }
                }
                5 => {
                    let staleness: u32 = parse_field("staleness", &app.get_cfg_fs_staleness(), err);
                    let policy: u8 = parse_field("policy", &app.get_cfg_fs_policy(), err);
                    let brake: f32 =
                        parse_field("brake current", &app.get_cfg_fs_brake_current(), err);
                    let slew: f32 = parse_field("slew", &app.get_cfg_fs_slew(), err);
                    let standstill: f32 =
                        parse_field("standstill", &app.get_cfg_fs_standstill(), err);
                    let decel: f32 = parse_field("decel", &app.get_cfg_fs_decel(), err);
                    let terminal: u8 = parse_field("terminal", &app.get_cfg_fs_terminal(), err);
                    ConfigAction::Replace(ConfigWrite::Failsafe(FailsafeConfigStored {
                        staleness_timeout_ms: staleness,
                        policy,
                        brake_current_a: brake,
                        slew_a_per_s: slew,
                        standstill_rad_s: standstill,
                        decel_rad_s2: decel,
                        terminal,
                    }))
                }
                _ => return,
            };

            // A typo must never reach flash as a zero — abort the write.
            if let Some(msg) = parse_err {
                app.set_config_status(SharedString::from(msg));
                return;
            }

            let cmd_tx = runtime.cmd_tx.clone();
            drop(guard);
            app.set_config_status(SharedString::from("Writing..."));

            thread::spawn(move || {
                let result = match write {
                    ConfigAction::Replace(write) => ops::config::send_write(&cmd_tx, write),
                    ConfigAction::Patch { group, fields } => {
                        ops::config::set_fields(&cmd_tx, group, &fields).map(|(_, commit)| commit)
                    }
                };
                let _ = weak.upgrade_in_event_loop(move |app| match result {
                    Ok(commit) => {
                        app.set_config_status(SharedString::from(format!(
                            "Revision {}: {}",
                            commit.revision,
                            if commit.persisted {
                                "persisted"
                            } else {
                                "volatile"
                            }
                        )));
                    }
                    Err(e) => {
                        app.set_config_status(SharedString::from(format!("Error: {e}")));
                    }
                });
            });
        });
    }

    // ── Config reset (factory reset) ─────────────────────────────────────────
    // Guarded by the UI "confirm" toggle (the button is disabled otherwise).
    {
        let rt = runtime.clone();
        let weak = app.as_weak();
        app.on_config_reset(move || {
            let app = weak.unwrap();
            if !app.get_config_reset_confirm() {
                return;
            }
            let cmd_tx = {
                let guard = rt.lock().unwrap();
                let Some(ref runtime) = *guard else {
                    app.set_config_status(SharedString::from("Not connected"));
                    return;
                };
                runtime.cmd_tx.clone()
            };
            app.set_config_status(SharedString::from("Resetting..."));
            let weak = weak.clone();
            thread::spawn(move || {
                let result = ops::config::reset_all(&cmd_tx);
                let _ = weak.upgrade_in_event_loop(move |app| {
                    app.set_config_reset_confirm(false);
                    match result {
                        Ok(()) => app.set_config_status(SharedString::from("Reset to defaults")),
                        Err(e) => app.set_config_status(SharedString::from(format!("Error: {e}"))),
                    }
                });
            });
        });
    }
}

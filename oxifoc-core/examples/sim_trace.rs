//! Closed-loop trace of the firmware control stack against VirtualMotor,
//! written as CSV for plotting (`scripts/simplot.py`). The rig (f405 boot
//! configuration, hall edge timing, matched actuation delay) lives in
//! `common/mod.rs`.
//!
//! ```sh
//! cargo run --release -p oxifoc-core --example sim_trace \
//!     --features virtual-motor,std,storage -- \
//!     source=observer mode=rpm set=2000 load=0.3 secs=4 out=trace.csv
//! uv run scripts/simplot.py trace.csv --out trace.png --fft w_true
//! ```
//!
//! Arguments (`key=value`, all optional):
//! - `motor=flipsky|zd2808` — plant + config preset (default flipsky)
//! - `source=hall|observer|cold` — `observer` spins up on hall and switches
//!   at `switch=` s; `cold` runs the sensorless cold start from rest
//! - `mode=iq|rpm|pos|lock|anchor`, `set=` — torque current (A),
//!   mechanical rpm target, position target (electrical turns from the
//!   start), the current (A) of a magnetic lock at the angle read at the
//!   start (OpenLoop, the stepper-style hold prototype), or the base
//!   current (A) of an anchored park brake (MESC SLAMBRAKE-style
//!   prototype: the windings short within `db=` rad of the start
//!   position, `set + akp·|err|` A pushing back beyond it, minus
//!   `akd·ω̂` A of damping on the driver's velocity estimate)
//! - `pkp=` position-loop gain (1/s); `vkp=`, `vki=`, `ff=` velocity-loop
//!   gains and acceleration feedforward (default: the loop's defaults)
//! - `load=` Coulomb load (N·m, opposes motion), `visc=` extra viscous
//!   load (N·m·s/rad, mechanical), `slope=` constant torque (N·m,
//!   positive pulls toward negative speed), `j=` rotor inertia (kg·m²,
//!   mechanical; a rider on a 5065 board reflects ~0.024)
//! - `delay=` actuation delay in PWM periods (plant and driver, default 0.5)
//! - `substeps=` plant sub-steps per period (default 8)
//! - `tracker=` phase-tracker ωn (rad/s, 0 = off; default 100 like f405)
//! - `accel=` velocity-loop acceleration limit (el rad/s², default: the
//!   loop's default)
//! - `step=T:V[,T:V…]` setpoint changes, `loadstep=T:NM[,…]` load changes,
//!   `slopestep=T:NM[,…]` slope changes, `coast=T[,…]` switch to Coast
//!   (the next `step=` re-engages)
//! - `secs=` duration, `decim=` write every Nth step, `out=` CSV path

#[allow(dead_code)] // shared across the sim examples; each uses a subset
mod common;

use std::collections::HashMap;
use std::env;
use std::f32::consts::{FRAC_PI_2, TAU};
use std::fs::File;
use std::io::{BufWriter, Write};

use oxifoc_core::foc::phase::{PhaseProvider, PhaseSource};
use oxifoc_core::foc::position::PositionLoopConfig;
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::foc::{angle_difference, wrap_angle};
use oxifoc_core::motor::ControlMode;

use common::{DT, FOC_HZ, Rig, RigConfig};

fn arg<T: std::str::FromStr>(args: &HashMap<String, String>, key: &str, default: T) -> T {
    args.get(key)
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// `T:V,T:V…` → [(step index, value)]; a bare `T` gets value 0.
fn events(args: &HashMap<String, String>, key: &str) -> Vec<(u64, f32)> {
    args.get(key).map_or_else(Vec::new, |s| {
        s.split(',')
            .filter_map(|e| {
                let (t, v) = e.split_once(':').unwrap_or((e, "0"));
                Some((
                    (t.parse::<f32>().ok()? * FOC_HZ as f32) as u64,
                    v.parse().ok()?,
                ))
            })
            .collect()
    })
}

/// `pos0`: the driver's position at the start (the `pos` targets' origin).
fn mode_for(rig: &Rig, mode: &str, set: f32, pos0: f32) -> ControlMode {
    match mode {
        "rpm" => ControlMode::VelocityControl {
            target_vel: rig.rpm_to_el(set),
        },
        "pos" => ControlMode::PositionControl {
            target_pos: pos0 + set * TAU,
        },
        // The OpenLoop current rides the q axis of its frame: a frame a
        // quarter turn back puts the vector — and the rotor's rest — on
        // the angle read now.
        "lock" => ControlMode::OpenLoop {
            angle_rad: wrap_angle(rig.driver.phase().get().angle - FRAC_PI_2),
            current: set,
            velocity_rad_s: 0.0,
            pi_gains: None,
        },
        "anchor" => ControlMode::Brake,
        _ => ControlMode::CurrentControl {
            iq_target: set,
            id_target: 0.0,
        },
    }
}

fn main() -> std::io::Result<()> {
    let args: HashMap<String, String> = env::args()
        .skip(1)
        .filter_map(|a| a.split_once('=').map(|(k, v)| (k.to_owned(), v.to_owned())))
        .collect();
    let source: String = arg(&args, "source", "hall".to_owned());
    let mode: String = arg(&args, "mode", "iq".to_owned());
    let set: f32 = arg(&args, "set", 5.0);
    let secs: f32 = arg(&args, "secs", 3.0);
    let switch_s: f32 = arg(&args, "switch", 1.0);
    let decim: u64 = arg(&args, "decim", 4).max(1);
    let out_path: String = arg(&args, "out", "trace.csv".to_owned());

    let mut rig = Rig::new(&RigConfig {
        motor: arg(&args, "motor", "flipsky".to_owned()),
        visc: arg(&args, "visc", 0.0),
        substeps: arg(&args, "substeps", 8),
        delay: arg(&args, "delay", 0.5),
        tracker: arg(&args, "tracker", 100.0),
        j: arg(&args, "j", 0.0),
    });
    rig.load = arg(&args, "load", 0.0);
    rig.torque_disturbance = arg(&args, "slope", 0.0);
    if source == "cold" {
        rig.set_source(PhaseSource::Observer);
    }
    let vel_default = VelocityLoopConfig::default();
    rig.driver.set_velocity_config(VelocityLoopConfig {
        kp: arg(&args, "vkp", vel_default.kp),
        ki: arg(&args, "vki", vel_default.ki),
        accel_limit: arg(&args, "accel", vel_default.accel_limit),
        accel_ff: arg(&args, "ff", vel_default.accel_ff),
    });
    let pos_default = PositionLoopConfig::default();
    rig.driver.set_position_config(PositionLoopConfig {
        kp: arg(&args, "pkp", pos_default.kp),
        ..pos_default
    });
    // One standing step: latches the driver's position frame (and the
    // angle a lock captures), and the rotor's true position in that frame.
    let first = rig.step();
    let pos0 = rig.driver.position().as_rad();
    let mut pos_true = pos0 + angle_difference(first.sample.angle_rad, pos0);
    let mut prev_angle = first.sample.angle_rad;
    let initial = mode_for(&rig, &mode, set, pos0);
    rig.driver.set_mode(initial);

    let setpoints = events(&args, "step");
    let loads = events(&args, "loadstep");
    let slopes = events(&args, "slopestep");
    let coasts = events(&args, "coast");
    let switch_step = (switch_s * FOC_HZ as f32) as u64;
    let anchor_db: f32 = arg(&args, "db", 1.2);
    let anchor_kp: f32 = arg(&args, "akp", 5.0);
    let anchor_kd: f32 = arg(&args, "akd", 0.0);

    let mut w = BufWriter::new(File::create(&out_path)?);
    writeln!(
        w,
        "t,theta,frame_err,obs_raw_err,w_true,w_est,w_obs,id_meas,iq_meas,id_true,iq_true,vd,vq,torque,starting,trusted,iq_target,pos_true,pos_est,pos_target"
    )?;
    let steps = (secs * FOC_HZ as f32) as u64;
    for step in 1..=steps {
        if source == "observer" && step == switch_step {
            rig.set_source(PhaseSource::Observer);
        }
        if let Some(&(_, v)) = setpoints.iter().find(|(s, _)| *s == step) {
            let m = mode_for(&rig, &mode, v, pos0);
            rig.driver.set_mode(m);
        }
        if let Some(&(_, v)) = loads.iter().find(|(s, _)| *s == step) {
            rig.load = v;
        }
        if let Some(&(_, v)) = slopes.iter().find(|(s, _)| *s == step) {
            rig.torque_disturbance = v;
        }
        if coasts.iter().any(|(s, _)| *s == step) {
            rig.driver.set_mode(ControlMode::Coast);
        }
        if mode == "anchor" {
            let err = rig.driver.position().as_rad() - pos0;
            let want = if err.abs() < anchor_db {
                ControlMode::Brake
            } else {
                ControlMode::CurrentControl {
                    iq_target: -err.signum() * (set + anchor_kp * (err.abs() - anchor_db))
                        - anchor_kd * rig.driver.phase().get().velocity,
                    id_target: 0.0,
                }
            };
            // Re-entering Brake every cycle would restart it as the plain
            // short; a current retarget is side-effect free.
            if !(matches!(want, ControlMode::Brake)
                && matches!(rig.driver.mode(), ControlMode::Brake))
            {
                rig.driver.set_mode(want);
            }
        }
        let rec = rig.step();
        pos_true += angle_difference(rec.sample.angle_rad, prev_angle);
        prev_angle = rec.sample.angle_rad;
        if step % decim == 0 {
            let t = step as f32 * DT;
            let (id_true, iq_true) = rec.true_dq();
            let obs = rig.driver.phase().observer();
            let obs_raw_err = obs
                .phase_raw()
                .map_or(f32::NAN, |p| angle_difference(p, rec.sample.angle_rad));
            let iq_target = match rig.driver.mode() {
                ControlMode::CurrentControl { iq_target, .. } => iq_target,
                _ => f32::NAN,
            };
            let pos_target = match rig.driver.mode() {
                ControlMode::PositionControl { target_pos } => target_pos,
                ControlMode::OpenLoop { angle_rad, .. } => {
                    // The lock's rest angle, unwrapped next to the rotor.
                    let rest = wrap_angle(angle_rad + FRAC_PI_2);
                    pos_true + angle_difference(rest, rec.sample.angle_rad)
                }
                _ => f32::NAN,
            };
            let pos_est = rig.driver.position().as_rad();
            writeln!(
                w,
                "{t:.6},{:.5},{:.5},{obs_raw_err:.5},{:.2},{:.2},{:.2},{:.4},{:.4},{id_true:.4},{iq_true:.4},{:.4},{:.4},{:.5},{},{},{iq_target:.3},{pos_true:.4},{pos_est:.4},{pos_target:.4}",
                wrap_angle(rec.sample.angle_rad),
                rec.frame_err(),
                rec.sample.omega_e,
                rec.telem.velocity_rad_s,
                obs.velocity().unwrap_or(f32::NAN),
                rec.telem.id,
                rec.telem.iq,
                rec.telem.vd,
                rec.telem.vq,
                rec.sample.torque,
                u8::from(rig.driver.phase().is_starting()),
                u8::from(rig.driver.phase().angle_trustworthy()),
            )?;
        }
    }
    w.flush()?;
    eprintln!("wrote {out_path}");
    Ok(())
}

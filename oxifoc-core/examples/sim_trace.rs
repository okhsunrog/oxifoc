//! Closed-loop trace of the firmware control stack against VirtualMotor,
//! written as CSV for plotting (`scripts/simplot.py`).
//!
//! The driver is built the way the f405 boots it: `RuntimeConfig` motor
//! params → `FocController::from_runtime_config` +
//! `configure_observers_from_config` + the commutation phase tracker. Hall
//! edges are time-stamped at the true boundary crossing (like the capture
//! timer), and the plant's actuation delay matches the driver's.
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
//! - `mode=iq|rpm`, `set=` — torque current (A) or mechanical rpm target
//! - `load=` Coulomb load (N·m, opposes motion), `visc=` extra viscous
//!   load (N·m·s/rad, mechanical)
//! - `delay=` actuation delay in PWM periods (plant and driver, default 0.5)
//! - `substeps=` plant sub-steps per period (default 8)
//! - `tracker=` phase-tracker ωn (rad/s, 0 = off; default 100 like f405)
//! - `accel=` velocity-loop acceleration limit (el rad/s², default: the
//!   loop's default)
//! - `step=T:VALUE` setpoint change, `loadstep=T:NM` load change
//! - `secs=` duration, `decim=` write every Nth step, `out=` CSV path

use std::collections::HashMap;
use std::env;
use std::f32::consts::{FRAC_PI_3, FRAC_PI_6, TAU};
use std::fs::File;
use std::io::{BufWriter, Write};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::hall_sensor::HallSensor;
use oxifoc_core::foc::phase::{PhaseManager, PhaseProvider, PhaseSource};
use oxifoc_core::foc::pwm::{PhasePwm, PhaseState, SvpwmModulator};
use oxifoc_core::foc::sensors::CurrentSensor;
use oxifoc_core::foc::transforms::{clarke, park};
use oxifoc_core::foc::trig::LibmSinCos;
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::foc::{angle_difference, clamp_f32, wrap_angle};
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::motor::{ControlMode, FocDriver};
use oxifoc_core::storage::{MotorParamsConfig, RuntimeConfig};
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor};

const FOC_HZ: u32 = 20_000;
const DT: f32 = 1.0 / FOC_HZ as f32;
const TICKS: u64 = 1_000_000 / FOC_HZ as u64;

struct Pwm([u16; 3]);

impl PhasePwm for Pwm {
    fn max_duty(&self) -> u16 {
        1000
    }
    fn set_duties(&mut self, duties: [u16; 3]) {
        self.0 = duties;
    }
    fn set_phase_states(&mut self, states: [PhaseState; 3]) {
        self.0 = states.map(|s| match s {
            PhaseState::Pwm(d) => d,
            PhaseState::Low | PhaseState::Float => 0,
        });
    }
}

struct Currents((f32, f32, f32));

impl CurrentSensor for Currents {
    fn read_currents(&self) -> (f32, f32, f32) {
        self.0
    }
    fn read_raw(&self) -> (u16, u16, u16) {
        (0, 0, 0)
    }
    fn is_calibrated(&self) -> bool {
        true
    }
    fn get_offsets(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
    fn set_offsets(&mut self, _offsets: (f32, f32, f32)) {}
}

/// (plant, bus voltage, current limit)
fn preset(name: &str) -> (MotorParams, f32, f32) {
    match name {
        // ZD2808 700 KV bench motor (2026-07-05 detection values).
        "zd2808" => (
            MotorParams {
                r: 0.127,
                ld: 86e-6,
                lq: 129e-6,
                lambda: 1.145e-3,
                pole_pairs: 7,
                j: 5e-5,
                friction_b: 4e-6,
                ..MotorParams::default()
            },
            12.0,
            10.0,
        ),
        // Flipsky 5065 270 KV: Y-equivalent of the LCR line-to-line
        // values, λ from KV.
        _ => (
            MotorParams {
                r: 0.036,
                ld: 22e-6,
                lq: 22e-6,
                lambda: 60.0 / (3f32.sqrt() * TAU * 270.0 * 7.0),
                pole_pairs: 7,
                j: 4e-4,
                friction_b: 2e-5,
                ..MotorParams::default()
            },
            24.0,
            30.0,
        ),
    }
}

fn arg<T: std::str::FromStr>(args: &HashMap<String, String>, key: &str, default: T) -> T {
    args.get(key)
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// `T:VALUE` → (seconds, value)
fn timed(args: &HashMap<String, String>, key: &str) -> Option<(f32, f32)> {
    let (t, v) = args.get(key)?.split_once(':')?;
    Some((t.parse().ok()?, v.parse().ok()?))
}

fn mode_for(mode: &str, set: f32, pole_pairs: u8) -> ControlMode {
    if mode == "rpm" {
        ControlMode::VelocityControl {
            target_vel: set * TAU / 60.0 * f32::from(pole_pairs),
        }
    } else {
        ControlMode::CurrentControl {
            iq_target: set,
            id_target: 0.0,
        }
    }
}

fn main() -> std::io::Result<()> {
    let args: HashMap<String, String> = env::args()
        .skip(1)
        .filter_map(|a| a.split_once('=').map(|(k, v)| (k.to_owned(), v.to_owned())))
        .collect();
    let motor_name: String = arg(&args, "motor", "flipsky".to_owned());
    let source: String = arg(&args, "source", "hall".to_owned());
    let mode: String = arg(&args, "mode", "iq".to_owned());
    let set: f32 = arg(&args, "set", 5.0);
    let mut load: f32 = arg(&args, "load", 0.0);
    let delay: f32 = arg(&args, "delay", 0.5);
    let tracker: f32 = arg(&args, "tracker", 100.0);
    let secs: f32 = arg(&args, "secs", 3.0);
    let switch_s: f32 = arg(&args, "switch", 1.0);
    let decim: u64 = arg(&args, "decim", 4).max(1);
    let out_path: String = arg(&args, "out", "trace.csv".to_owned());

    let (mut params, vbus, i_max) = preset(&motor_name);
    params.friction_b += arg(&args, "visc", 0.0);
    params.substeps = arg(&args, "substeps", 8);
    params.actuation_delay = delay;

    let config = RuntimeConfig {
        motor_params: Some(MotorParamsConfig {
            resistance_ohm: params.r,
            inductance_d_h: params.ld,
            inductance_q_h: params.lq,
            flux_linkage_wb: params.lambda,
            pole_pairs: params.pole_pairs,
            ..MotorParamsConfig::default()
        }),
        ..RuntimeConfig::default()
    };
    let mut motor = VirtualMotor::new(params);
    let mut out = motor.step(0.0, 0.0, 0.0, DT);
    let mut hall = HallSensor::new(1_000_000);
    hall.update(out.hall_state, 0);
    let mut last_hall = out.hall_state;
    let mut phase = PhaseManager::with_hall(hall);
    phase.configure_observers_from_config(&config, vbus);
    phase.set_phase_tracker(tracker, 1.2);
    if source == "cold" {
        phase
            .set_source(PhaseSource::Observer)
            .expect("observer configured");
    }
    let foc = FocController::<SvpwmModulator, LibmSinCos>::from_runtime_config(&config, vbus);
    let mut driver = FocDriver::new(foc, Pwm([0; 3]), Currents((0.0, 0.0, 0.0)), phase, DT);
    driver.set_current_limits(CurrentLimits::from_max_current(i_max));
    driver.set_actuation_delay(delay);
    let vel_default = VelocityLoopConfig::default();
    driver.set_velocity_config(VelocityLoopConfig {
        accel_limit: arg(&args, "accel", vel_default.accel_limit),
        ..vel_default
    });
    driver.set_mode(mode_for(&mode, set, params.pole_pairs));

    let mut w = BufWriter::new(File::create(&out_path)?);
    writeln!(
        w,
        "t,theta,frame_err,obs_raw_err,w_true,w_est,w_obs,id_meas,iq_meas,id_true,iq_true,vd,vq,torque,starting,trusted,iq_target"
    )?;
    let steps = (secs * FOC_HZ as f32) as u64;
    let switch_step = (switch_s * FOC_HZ as f32) as u64;
    let setpoint_step = timed(&args, "step");
    let load_step = timed(&args, "loadstep");
    let mut prev_angle = out.angle_rad;
    for step in 1..=steps {
        let t = step as f32 * DT;
        let now = step * TICKS;
        if source == "observer" && step == switch_step {
            driver
                .phase_mut()
                .set_source(PhaseSource::Observer)
                .expect("observer configured");
        }
        if let Some((ts, v)) = setpoint_step
            && step == (ts * FOC_HZ as f32) as u64
        {
            driver.set_mode(mode_for(&mode, v, params.pole_pairs));
        }
        if let Some((ts, v)) = load_step
            && step == (ts * FOC_HZ as f32) as u64
        {
            load = v;
        }
        if out.hall_state != last_hall {
            // Crossing of the hall boundary (π/6 + k·π/3) inside the step.
            let a1 = prev_angle + angle_difference(out.angle_rad, prev_angle);
            let k0 = ((prev_angle - FRAC_PI_6) / FRAC_PI_3).floor();
            let k1 = ((a1 - FRAC_PI_6) / FRAC_PI_3).floor();
            let frac = if k0 == k1 {
                1.0
            } else {
                clamp_f32(
                    (FRAC_PI_6 + FRAC_PI_3 * k0.max(k1) - prev_angle) / (a1 - prev_angle),
                    0.0,
                    1.0,
                )
            };
            let edge = now - ((1.0 - frac) * TICKS as f32) as u64;
            driver.phase_mut().hall_mut().update(out.hall_state, edge);
            last_hall = out.hall_state;
        }
        let sample = out;
        driver.current_sensor_mut().0 = (sample.ia, sample.ib, sample.ic);
        let telem = driver.step(now).unwrap_or_default();
        prev_angle = sample.angle_rad;
        let omega_m = sample.omega_e / f32::from(params.pole_pairs);
        out = motor.step(
            telem.v_alpha,
            telem.v_beta,
            load * (omega_m / 2.0).tanh(),
            DT,
        );
        if step % decim == 0 {
            let (i_alpha, i_beta) = clarke(sample.ia, sample.ib);
            let (s, c) = sample.angle_rad.sin_cos();
            let (id_true, iq_true) = park(i_alpha, i_beta, s, c);
            let obs = driver.phase().observer();
            let obs_raw_err = obs
                .phase_raw()
                .map_or(f32::NAN, |p| angle_difference(p, sample.angle_rad));
            let iq_target = match driver.mode() {
                ControlMode::CurrentControl { iq_target, .. } => iq_target,
                _ => f32::NAN,
            };
            writeln!(
                w,
                "{t:.6},{:.5},{:.5},{obs_raw_err:.5},{:.2},{:.2},{:.2},{:.4},{:.4},{id_true:.4},{iq_true:.4},{:.4},{:.4},{:.5},{},{},{iq_target:.3}",
                wrap_angle(sample.angle_rad),
                angle_difference(telem.angle_rad, sample.angle_rad),
                sample.omega_e,
                telem.velocity_rad_s,
                obs.velocity().unwrap_or(f32::NAN),
                telem.id,
                telem.iq,
                telem.vd,
                telem.vq,
                sample.torque,
                u8::from(driver.phase().is_starting()),
                u8::from(driver.phase().angle_trustworthy()),
            )?;
        }
    }
    w.flush()?;
    eprintln!("wrote {out_path}");
    Ok(())
}

//! Stepped-sine frequency response of the closed control loops, measured on
//! the simulated firmware stack (see `common/mod.rs`) with lock-in
//! demodulation over whole periods. Writes CSV for `scripts/bode.py`.
//!
//! ```sh
//! cargo run --release -p oxifoc-core --example loop_response \
//!     --features virtual-motor,std,storage -- loop=estimator source=observer out=est.csv
//! uv run scripts/bode.py est.csv --out est.png
//! ```
//!
//! Loops (`loop=`):
//! - `current`: iq target + sine at a viscous-load cruise → H = iq_true/iq_ref
//!   (and iq_meas/iq_ref)
//! - `velocity`: velocity target + sine → H = ω_true/ω_ref
//! - `estimator`: shaft-torque sine at an iq cruise, sized for a constant
//!   rotor-angle wobble → E = frame_err/θ_rotor, the share of the rotor's
//!   motion the commutation frame does NOT follow (0 = perfect tracking,
//!   1 = the frame ignores it, > 1 = amplifies it)
//!
//! Other args: `source=hall|observer`, `tracker=` ωn (0 = off),
//! `delay=`, `motor=`, `fmin= fmax= points=`, `out=`.

#[allow(dead_code)] // shared across the sim examples; each uses a subset
mod common;

use std::collections::HashMap;
use std::env;
use std::f32::consts::TAU;
use std::fs::File;
use std::io::{BufWriter, Write};

use oxifoc_core::foc::phase::PhaseSource;
use oxifoc_core::motor::ControlMode;

use common::{DT, FOC_HZ, Rig, RigConfig};

fn arg<T: std::str::FromStr>(args: &HashMap<String, String>, key: &str, default: T) -> T {
    args.get(key)
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

/// Lock-in accumulator for one frequency: phasors of several signals.
struct LockIn {
    w: f32,
    re: Vec<f64>,
    im: Vec<f64>,
    n: u32,
}

impl LockIn {
    fn new(f: f32, signals: usize) -> Self {
        Self {
            w: TAU * f,
            re: vec![0.0; signals],
            im: vec![0.0; signals],
            n: 0,
        }
    }

    /// `t` since the excitation started; the excitation is `sin(w·t)`, so
    /// a signal in phase with it has phasor angle 0.
    fn add(&mut self, t: f32, ys: &[f32]) {
        let (s, c) = (self.w * t).sin_cos();
        for (k, y) in ys.iter().enumerate() {
            self.re[k] += f64::from(y * s);
            self.im[k] += f64::from(y * c);
        }
        self.n += 1;
    }

    /// Complex amplitude (re, im) of signal `k`.
    fn phasor(&self, k: usize) -> (f64, f64) {
        let g = 2.0 / f64::from(self.n);
        (self.re[k] * g, self.im[k] * g)
    }
}

fn ratio(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let den = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / den, (a.1 * b.0 - a.0 * b.1) / den)
}

fn main() -> std::io::Result<()> {
    let args: HashMap<String, String> = env::args()
        .skip(1)
        .filter_map(|a| a.split_once('=').map(|(k, v)| (k.to_owned(), v.to_owned())))
        .collect();
    let lp: String = arg(&args, "loop", "estimator".to_owned());
    let source: String = arg(&args, "source", "observer".to_owned());
    let cfg = RigConfig {
        motor: arg(&args, "motor", "flipsky".to_owned()),
        // Viscous load: 10 A of torque current settles near 2000 rpm.
        visc: arg(&args, "visc", 1.4e-3),
        substeps: arg(&args, "substeps", 8),
        delay: arg(&args, "delay", 0.5),
        tracker: arg(&args, "tracker", 100.0),
    };
    let (fmin, fmax) = match lp.as_str() {
        "current" => (arg(&args, "fmin", 20.0f32), arg(&args, "fmax", 4000.0f32)),
        _ => (arg(&args, "fmin", 1.0f32), arg(&args, "fmax", 300.0f32)),
    };
    let points: u32 = arg(&args, "points", 24);
    let out_path: String = arg(&args, "out", "response.csv".to_owned());

    let mut w = BufWriter::new(File::create(&out_path)?);
    writeln!(w, "loop,source,tracker,f,mag,phase_deg,mag2,phase2_deg")?;
    for i in 0..points {
        let f = fmin * (fmax / fmin).powf(i as f32 / (points - 1).max(1) as f32);
        let mut rig = Rig::new(&cfg);
        // Spin up on hall at 10 A against the viscous load, then hand over.
        rig.driver.set_mode(ControlMode::CurrentControl {
            iq_target: 10.0,
            id_target: 0.0,
        });
        for _ in 0..FOC_HZ {
            rig.step();
        }
        if source == "observer" {
            rig.set_source(PhaseSource::Observer);
        }
        let w0 = rig.out.omega_e;
        if lp == "velocity" {
            rig.driver
                .set_mode(ControlMode::VelocityControl { target_vel: w0 });
        }
        for _ in 0..FOC_HZ / 2 {
            rig.step();
        }

        // Excitation: settle 0.2 s or 3 periods, then measure over whole
        // periods (≥ 4, ≥ 0.3 s).
        let period = 1.0 / f;
        let settle = (0.2f32).max(3.0 * period);
        let cycles = (4.0f32).max((0.3 * f).ceil());
        let total = settle + cycles * period;
        let pp = f32::from(rig.params.pole_pairs);
        // Estimator: torque sized for a 0.05 rad el rotor-angle wobble.
        let wf = TAU * f;
        let torque_amp = 0.05 * rig.params.j * wf * wf / pp;
        let mut lock = LockIn::new(f, 3);
        let mut t = 0.0f32;
        while t < total {
            let s = (wf * t).sin();
            match lp.as_str() {
                "current" => rig.driver.set_mode(ControlMode::CurrentControl {
                    iq_target: 10.0 + 1.0 * s,
                    id_target: 0.0,
                }),
                "velocity" => rig.driver.set_mode(ControlMode::VelocityControl {
                    target_vel: w0 + 30.0 * s,
                }),
                _ => rig.torque_disturbance = torque_amp * s,
            }
            let rec = rig.step();
            if t >= settle {
                let tm = t - settle;
                let (_, iq_true) = rec.true_dq();
                match lp.as_str() {
                    "current" => lock.add(tm, &[iq_true, rec.telem.iq, s]),
                    "velocity" => lock.add(tm, &[rec.sample.omega_e, rec.telem.velocity_rad_s, s]),
                    // Rotor speed (its constant part cancels over whole
                    // periods) and the frame error.
                    _ => lock.add(tm, &[rec.frame_err(), rec.sample.omega_e, s]),
                }
            }
            t += DT;
        }
        let excitation = lock.phasor(2);
        let (h1, h2) = match lp.as_str() {
            "current" => {
                let r = (excitation.0 * 1.0, excitation.1 * 1.0);
                (ratio(lock.phasor(0), r), ratio(lock.phasor(1), r))
            }
            "velocity" => {
                let r = (excitation.0 * 30.0, excitation.1 * 30.0);
                (ratio(lock.phasor(0), r), ratio(lock.phasor(1), r))
            }
            _ => {
                // θ = ∫ω → Θ = Ω/(jω); E/Θ = E·jω/Ω.
                let (er, ei) = lock.phasor(0);
                let (or, oi) = lock.phasor(1);
                let wf = f64::from(wf);
                let theta = (oi / wf, -or / wf);
                (ratio((er, ei), theta), (0.0, 0.0))
            }
        };
        let polar = |h: (f64, f64)| (h.0.hypot(h.1), h.1.atan2(h.0).to_degrees());
        let (m1, p1) = polar(h1);
        let (m2, p2) = polar(h2);
        writeln!(
            w,
            "{lp},{source},{},{f:.3},{m1:.5},{p1:.2},{m2:.5},{p2:.2}",
            cfg.tracker
        )?;
        eprintln!("{lp} {source} f={f:8.2} Hz  |H|={m1:.4}  ∠{p1:+7.1}°");
    }
    w.flush()?;
    eprintln!("wrote {out_path}");
    Ok(())
}

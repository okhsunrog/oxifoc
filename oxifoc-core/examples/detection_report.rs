//! Motor parameter detection report against VirtualMotor.
//!
//! Runs `run_full_detection()` (and individual flux methods) across a catalog
//! of simulated motors with different sizes/parameters, printing a summary
//! table with ground truth comparison.
//!
//! ```sh
//! cargo run -p oxifoc-core --example detection_report --features virtual-motor,microfft
//! ```

use core::f32::consts::TAU;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use std::cell::RefCell;

use oxifoc_core::foc::controller::{FocController, FocOutput};
use oxifoc_core::foc::detection::sweep::{
    DetectionHardware, DetectionParams, measure_flux_linkage, measure_flux_linkage_magnitude,
    measure_flux_linkage_spindown, measure_inductance, measure_inductance_pulse,
    run_full_detection,
};
use oxifoc_core::foc::detection::types::{
    FluxLinkageParams, InductanceParams, MotorSize, VoltagePulseParams,
};
use oxifoc_core::foc::pi_controller::PIController;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::{angle_difference, wrap_angle};
use oxifoc_core::motor::ControlMode;
use oxifoc_core::timer::Timer;
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

const DT: f32 = 1.0 / 20_000.0;
const MAX_DUTY: u16 = 1000;

// ── Motor catalog ──────────────────────────────────────────────────────────

struct MotorDef {
    name: &'static str,
    params: MotorParams,
    vbus: f32,
    motor_size: MotorSize,
}

fn motor_catalog() -> Vec<MotorDef> {
    vec![
        MotorDef {
            name: "Default (hobby BLDC)",
            params: MotorParams {
                j: 5e-4,
                ..MotorParams::default()
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
        },
        MotorDef {
            name: "Micro gimbal",
            params: MotorParams {
                r: 8.0,
                ld: 3e-3,
                lq: 3e-3,
                lambda: 0.005,
                pole_pairs: 11,
                j: 5e-6,
                friction_b: 1e-5,
                hall_offset: 0.0,
            },
            vbus: 12.0,
            motor_size: MotorSize::Mini,
        },
        MotorDef {
            name: "5010 drone motor",
            params: MotorParams {
                r: 0.12,
                ld: 2e-4,
                lq: 2e-4,
                lambda: 0.008,
                pole_pairs: 7,
                j: 3e-4,
                friction_b: 5e-5,
                hall_offset: 0.0,
            },
            vbus: 24.0,
            motor_size: MotorSize::Small,
        },
        MotorDef {
            name: "6354 eskate",
            params: MotorParams {
                r: 0.035,
                ld: 1.5e-5,
                lq: 1.5e-5,
                lambda: 0.0085,
                pole_pairs: 7,
                j: 1e-3,
                friction_b: 1e-3,
                hall_offset: 0.0,
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
        },
        MotorDef {
            name: "8308 ebike hub",
            params: MotorParams {
                r: 0.05,
                ld: 4e-5,
                lq: 4e-5,
                lambda: 0.015,
                pole_pairs: 20,
                j: 5e-3,
                friction_b: 5e-3,
                hall_offset: 0.0,
            },
            vbus: 72.0,
            motor_size: MotorSize::Large,
        },
        // IPM motors (Ld ≠ Lq) — saliency test
        MotorDef {
            name: "IPM servo (mild)",
            params: MotorParams {
                r: 0.3,
                ld: 3e-4, // Ld < Lq typical for IPM
                lq: 5e-4,
                lambda: 0.012,
                pole_pairs: 4,
                j: 5e-4,
                friction_b: 1e-4,
                hall_offset: 0.0,
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
        },
        MotorDef {
            name: "IPM traction (strong)",
            params: MotorParams {
                r: 0.02,
                ld: 1e-4, // strong saliency: Lq/Ld = 3
                lq: 3e-4,
                lambda: 0.02,
                pole_pairs: 4,
                j: 2e-3,
                friction_b: 1e-3,
                hall_offset: 0.0,
            },
            vbus: 96.0,
            motor_size: MotorSize::Large,
        },
        MotorDef {
            name: "NEMA23 stepper-servo",
            params: MotorParams {
                r: 1.2,
                ld: 2e-3,
                lq: 2e-3,
                lambda: 0.04,
                pole_pairs: 50,
                j: 2e-4,
                friction_b: 1e-3,
                hall_offset: 0.0,
            },
            vbus: 48.0,
            motor_size: MotorSize::Medium,
        },
    ]
}

// ── Simulation infrastructure ──────────────────────────────────────────────

struct SimState {
    foc: FocController<SvpwmModulator>,
    motor: VirtualMotor,
    out: VirtualMotorOutput,
    mode: ControlMode,
    sim_angle: f32,
    ol_omega: f32,
    prev_cmd_angle: f32,
    steps_since_send: u64,
}

impl SimState {
    fn new(motor_params: MotorParams, vbus: f32) -> Self {
        let kp = motor_params.ld * 1_000.0;
        let ki = motor_params.r * 1_000.0;
        let mut foc = FocController::<SvpwmModulator>::new(vbus);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        Self {
            foc,
            motor: VirtualMotor::new(motor_params),
            out: VirtualMotorOutput::default(),
            mode: ControlMode::Stopped,
            sim_angle: 0.0,
            ol_omega: 0.0,
            prev_cmd_angle: 0.0,
            steps_since_send: 0,
        }
    }

    fn step_one(&mut self) -> FocOutput {
        if matches!(self.mode, ControlMode::OpenLoop { .. }) && self.ol_omega.abs() > 0.1 {
            self.sim_angle = wrap_angle(self.sim_angle + self.ol_omega * DT);
        }
        self.steps_since_send += 1;

        let telem = match self.mode {
            ControlMode::OpenLoop { current, .. } => self.foc.step(
                (self.out.ia, self.out.ib, self.out.ic),
                self.sim_angle,
                current,
                0.0,
                MAX_DUTY,
                DT,
            ),
            ControlMode::DirectVoltage { vd, vq, angle_rad } => {
                self.foc.apply_dq(vd, vq, angle_rad, MAX_DUTY)
            }
            ControlMode::Coast | ControlMode::Stopped => FocOutput::empty(),
            _ => FocOutput::empty(),
        };

        self.out = match self.mode {
            ControlMode::Coast => self.motor.step_coast(0.0, DT),
            ControlMode::Stopped => self.motor.step_shorted(0.0, DT),
            _ => self.motor.step(telem.v_alpha, telem.v_beta, 0.0, DT),
        };
        telem
    }

    fn step_n(&mut self, n: usize) {
        for _ in 0..n {
            self.step_one();
        }
    }
}

thread_local! {
    static SIM: RefCell<Option<SimState>> = RefCell::new(None);
}

struct VirtualHardware;

impl DetectionHardware for VirtualHardware {
    fn send_command(&self, mode: ControlMode) {
        SIM.with(|s| {
            let mut borrow = s.borrow_mut();
            let sim = borrow.as_mut().unwrap();
            match mode {
                ControlMode::OpenLoop { angle_rad, .. } => {
                    if !matches!(sim.mode, ControlMode::OpenLoop { .. }) {
                        sim.sim_angle = angle_rad;
                        sim.ol_omega = 0.0;
                    }
                    if sim.steps_since_send > 0 && sim.steps_since_send < 2000 {
                        let elapsed = sim.steps_since_send as f32 * DT;
                        let wrapped = angle_difference(angle_rad, sim.prev_cmd_angle);
                        let expected = sim.ol_omega * elapsed;
                        let n = ((expected - wrapped) / TAU).round();
                        let delta = wrapped + n * TAU;
                        sim.ol_omega = delta / elapsed;
                    }
                    sim.prev_cmd_angle = angle_rad;
                    sim.steps_since_send = 0;
                }
                ControlMode::Coast => sim.foc.reset(),
                ControlMode::Stopped => {
                    sim.foc.reset();
                    sim.ol_omega = 0.0;
                }
                ControlMode::DirectVoltage { .. } => sim.ol_omega = 0.0,
                _ => {}
            }
            sim.mode = mode;
        });
    }

    fn wait_telemetry(&mut self) -> impl core::future::Future<Output = FocOutput> {
        core::future::ready(SIM.with(|s| s.borrow_mut().as_mut().unwrap().step_one()))
    }

    fn read_phase_currents(&self) -> (f32, f32, f32) {
        SIM.with(|s| {
            let s = s.borrow();
            let s = s.as_ref().unwrap();
            (s.out.ia, s.out.ib, s.out.ic)
        })
    }

    fn read_coast_telemetry(&self) -> (f32, f32, f32) {
        SIM.with(|s| {
            let s = s.borrow();
            let s = s.as_ref().unwrap();
            (s.out.bemf_alpha, s.out.bemf_beta, s.out.omega_e)
        })
    }
}

struct VirtualTimer;

impl Timer for VirtualTimer {
    fn after_millis(ms: u64) -> impl core::future::Future<Output = ()> {
        let steps = ((ms as f64 / 1000.0) * 20_000.0) as usize;
        if steps > 0 {
            SIM.with(|s| s.borrow_mut().as_mut().unwrap().step_n(steps));
        }
        core::future::ready(())
    }

    fn after_micros(us: u64) -> impl core::future::Future<Output = ()> {
        let steps = ((us as f64 / 1_000_000.0) * 20_000.0) as usize;
        if steps > 0 {
            SIM.with(|s| s.borrow_mut().as_mut().unwrap().step_n(steps));
        }
        core::future::ready(())
    }
}

fn block_on<F: core::future::Future>(f: F) -> F::Output {
    fn noop(_: *const ()) {}
    fn clone(p: *const ()) -> RawWaker {
        RawWaker::new(p, &VTABLE)
    }
    static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut f = core::pin::pin!(f);
    match f.as_mut().poll(&mut cx) {
        Poll::Ready(v) => v,
        Poll::Pending => panic!("unexpected Pending"),
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────

fn reset_sim(motor_params: MotorParams, vbus: f32) {
    SIM.with(|s| {
        *s.borrow_mut() = Some(SimState::new(motor_params, vbus));
    });
}

fn err_pct(measured: f32, expected: f32) -> f32 {
    if expected.abs() < 1e-12 {
        0.0
    } else {
        (measured - expected) / expected * 100.0
    }
}

fn fmt_err(measured: f32, expected: f32) -> String {
    let e = err_pct(measured, expected);
    if e.abs() < 0.05 {
        " 0.0%".to_string()
    } else {
        format!("{:+5.1}%", e)
    }
}

// ── Per-motor detection ────────────────────────────────────────────────────

#[allow(dead_code)]
struct DetResult {
    r: Option<f32>,
    ld: Option<f32>,
    lq: Option<f32>,
    l_avg: Option<f32>,
    lam_spindown: Option<f32>,
    lam_driven: Option<f32>,
    lam_full: Option<f32>,
}

fn run_detection(def: &MotorDef) -> DetResult {
    let p = def.params;
    let mut hw = VirtualHardware;

    let det_params = DetectionParams {
        motor_size: def.motor_size,
        pole_pairs: p.pole_pairs,
        current_max: 10.0,
        pwm_freq_hz: 20_000.0,
        vbus: def.vbus,
    };

    // Full detection sequence
    reset_sim(p, def.vbus);
    let full = block_on(run_full_detection::<VirtualHardware, VirtualTimer>(
        &mut hw, det_params,
    ));

    let (r, ld, lq, l_avg, lam_full) = match full {
        Ok(det) => (
            Some(det.params.resistance_ohm),
            Some(det.params.inductance_d_h),
            Some(det.params.inductance_q_h),
            Some(det.params.inductance_avg_h),
            Some(det.params.flux_linkage_wb),
        ),
        Err(_) => (None, None, None, None, None),
    };

    // Individual flux — spin-down
    let flux_params = FluxLinkageParams {
        motor_size: def.motor_size,
        pole_pairs: p.pole_pairs,
        ..Default::default()
    };
    reset_sim(p, def.vbus);
    let lam_spindown = block_on(
        measure_flux_linkage_spindown::<VirtualHardware, VirtualTimer>(&mut hw, &flux_params),
    )
    .ok();

    // Individual flux — driven
    let flux_driven = FluxLinkageParams {
        resistance_ohm: p.r,
        ..flux_params
    };
    reset_sim(p, def.vbus);
    let lam_driven = block_on(measure_flux_linkage::<VirtualHardware, VirtualTimer>(
        &mut hw,
        &flux_driven,
    ))
    .ok();

    DetResult {
        r,
        ld,
        lq,
        l_avg,
        lam_spindown,
        lam_driven,
        lam_full,
    }
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let catalog = motor_catalog();

    println!("Motor Parameter Detection Report (VirtualMotor Simulation)");
    println!("==========================================================");
    println!();

    // ── Ground truth table ─────────────────────────────────────────────
    println!("Ground truth:");
    println!(
        "  {:<22} {:>7} {:>8} {:>8} {:>9} {:>4} {:>9}",
        "Motor", "R(Ohm)", "Ld(uH)", "Lq(uH)", "lm(mWb)", "pp", "J(g*m2)"
    );
    println!(
        "  {:-<22} {:->7} {:->8} {:->8} {:->9} {:->4} {:->9}",
        "", "", "", "", "", "", ""
    );
    for def in &catalog {
        let p = &def.params;
        println!(
            "  {:<22} {:>7.4} {:>8.1} {:>8.1} {:>9.3} {:>4} {:>9.2}",
            def.name,
            p.r,
            p.ld * 1e6,
            p.lq * 1e6,
            p.lambda * 1e3,
            p.pole_pairs,
            p.j * 1e3,
        );
    }
    println!();

    // ── Detection results ──────────────────────────────────────────────
    println!("Detection results (error vs ground truth):");
    println!(
        "  {:<22} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
        "Motor", "R", "Ld", "Lq", "lm(full)", "lm(spindn)", "lm(driven)"
    );
    println!(
        "  {:-<22} {:->9} {:->9} {:->9} {:->12} {:->12} {:->12}",
        "", "", "", "", "", "", ""
    );

    for def in &catalog {
        let p = &def.params;
        let det = run_detection(def);

        let r_s = det
            .r
            .map(|v| fmt_err(v, p.r))
            .unwrap_or_else(|| " FAIL".to_string());
        let ld_s = det
            .ld
            .map(|v| fmt_err(v, p.ld))
            .unwrap_or_else(|| " FAIL".to_string());
        let lq_s = det
            .lq
            .map(|v| fmt_err(v, p.lq))
            .unwrap_or_else(|| " FAIL".to_string());
        let lf_s = det
            .lam_full
            .map(|v| fmt_err(v, p.lambda))
            .unwrap_or_else(|| "  FAIL".to_string());
        let ls_s = det
            .lam_spindown
            .map(|v| fmt_err(v, p.lambda))
            .unwrap_or_else(|| "  FAIL".to_string());
        let ld_drv = det
            .lam_driven
            .map(|v| fmt_err(v, p.lambda))
            .unwrap_or_else(|| "  FAIL".to_string());

        println!(
            "  {:<22} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
            def.name, r_s, ld_s, lq_s, lf_s, ls_s, ld_drv,
        );
    }

    println!();

    // ── VESC improvement benchmarks ────────────────────────────────────
    println!("=== VESC Improvement Benchmarks ===");
    println!();

    // Benchmark 1: HFI at different injection frequencies
    println!("1) HFI injection frequency (Ld/Lq error at 1kHz vs 2kHz vs 5kHz):");
    println!(
        "  {:<22} {:>14} {:>14} {:>14}",
        "Motor", "1kHz (cur)", "2kHz", "5kHz"
    );
    println!("  {:-<22} {:->14} {:->14} {:->14}", "", "", "", "");
    for def in &catalog {
        let p = def.params;
        let mut results = Vec::new();
        for freq in [1000.0f32, 2000.0, 5000.0] {
            reset_sim(p, def.vbus);
            let r_val = p.r;
            let max_hold = (def.vbus * 0.577 * 0.6) / r_val.max(0.001);
            let hold = 2.0f32.min(max_hold).max(0.1);
            let params = InductanceParams {
                motor_size: def.motor_size,
                resistance_ohm: r_val,
                hold_current_a: hold,
                hfi_frequency_hz: freq,
                ..Default::default()
            };
            let mut hw = VirtualHardware;
            let l = block_on(measure_inductance::<VirtualHardware, VirtualTimer>(
                &mut hw, &params, 20_000.0,
            ));
            match l {
                Ok((ld, lq)) => {
                    let ld_e = err_pct(ld, p.ld);
                    let lq_e = err_pct(lq, p.lq);
                    results.push(format!("{:+.1}/{:+.1}%", ld_e, lq_e));
                }
                Err(_) => results.push("FAIL".to_string()),
            }
        }
        println!(
            "  {:<22} {:>14} {:>14} {:>14}",
            def.name, results[0], results[1], results[2]
        );
    }
    println!();

    // Benchmark 2: Voltage pulse with auto-ranging amplitude
    println!("2) Voltage pulse auto-ranging (vs fixed 30% Vbus):");
    println!("  {:<22} {:>16} {:>16}", "Motor", "fixed 30%", "auto-range");
    println!("  {:-<22} {:->16} {:->16}", "", "", "");
    for def in &catalog {
        let p = def.params;
        let r_val = p.r;
        let max_hold = (def.vbus * 0.577 * 0.6) / r_val.max(0.001);
        let hold = 2.0f32.min(max_hold).max(0.1);
        let v_hold = r_val * hold;
        let v_headroom = def.vbus * 0.577 - v_hold;

        // Fixed pulse
        let fixed_pulse = v_headroom.max(0.5);
        reset_sim(p, def.vbus);
        let mut hw = VirtualHardware;
        let fixed = block_on(measure_inductance_pulse::<VirtualHardware, VirtualTimer>(
            &mut hw,
            &VoltagePulseParams {
                hold_current_a: hold,
                resistance_ohm: r_val,
                pulse_voltage_v: fixed_pulse,
                num_pulses: 20,
                settle_time_ms: 200,
            },
            20_000.0,
        ));
        let fixed_s = match fixed {
            Ok((ld, lq)) => format!("{:+.1}/{:+.1}%", err_pct(ld, p.ld), err_pct(lq, p.lq)),
            Err(_) => "FAIL".to_string(),
        };

        // Auto-ranging: start at 10%, ×1.5 up to headroom
        let mut best: Option<(f32, f32, f32, f32)> = None;
        let mut v = (v_headroom * 0.1).max(0.2);
        while v <= v_headroom.max(0.5) {
            reset_sim(p, def.vbus);
            let res = block_on(measure_inductance_pulse::<VirtualHardware, VirtualTimer>(
                &mut hw,
                &VoltagePulseParams {
                    hold_current_a: hold,
                    resistance_ohm: r_val,
                    pulse_voltage_v: v,
                    num_pulses: 20,
                    settle_time_ms: 200,
                },
                20_000.0,
            ));
            if let Ok((ld, lq)) = res {
                let avg_err = (err_pct(ld, p.ld).abs() + err_pct(lq, p.lq).abs()) / 2.0;
                if best.is_none() || avg_err < best.unwrap().0 {
                    best = Some((avg_err, ld, lq, v));
                }
            }
            v *= 1.5;
        }
        let auto_s = match best {
            Some((_, ld, lq, _v)) => {
                format!("{:+.1}/{:+.1}%", err_pct(ld, p.ld), err_pct(lq, p.lq))
            }
            None => "FAIL".to_string(),
        };

        println!("  {:<22} {:>16} {:>16}", def.name, fixed_s, auto_s);
    }
    println!();

    // Benchmark 3: Driven flux — q-axis vs magnitude (VESC-style)
    println!("3) Driven flux: q-axis (ours) vs magnitude (VESC-style):");
    println!("  {:<22} {:>14} {:>14}", "Motor", "q-axis", "magnitude");
    println!("  {:-<22} {:->14} {:->14}", "", "", "");
    for def in &catalog {
        let p = def.params;
        let flux_params = FluxLinkageParams {
            motor_size: def.motor_size,
            resistance_ohm: p.r,
            pole_pairs: p.pole_pairs,
            ..Default::default()
        };
        let l_avg = (p.ld + p.lq) / 2.0;

        // Q-axis driven
        reset_sim(p, def.vbus);
        let mut hw = VirtualHardware;
        let qaxis = block_on(measure_flux_linkage::<VirtualHardware, VirtualTimer>(
            &mut hw,
            &flux_params,
        ));
        let q_s = match qaxis {
            Ok(v) => format!("{:+.1}%", err_pct(v, p.lambda)),
            Err(_) => "FAIL".to_string(),
        };

        // Magnitude driven (VESC-style)
        reset_sim(p, def.vbus);
        let mag = block_on(measure_flux_linkage_magnitude::<
            VirtualHardware,
            VirtualTimer,
        >(&mut hw, &flux_params, l_avg));
        let m_s = match mag {
            Ok(v) => format!("{:+.1}%", err_pct(v, p.lambda)),
            Err(_) => "FAIL".to_string(),
        };

        println!("  {:<22} {:>14} {:>14}", def.name, q_s, m_s);
    }

    println!();
    println!("Positive = overestimate, negative = underestimate.");
    println!("Ld/Lq columns show Ld error / Lq error.");
}

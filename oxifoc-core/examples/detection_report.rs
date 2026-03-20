//! Motor parameter detection report against VirtualMotor.
//!
//! Runs both the full detection sequence (spin-down primary) and the
//! individual measurement functions, printing a comparison table.
//!
//! ```sh
//! cargo run -p oxifoc-core --example detection_report --features virtual-motor,microfft
//! ```

use core::f32::consts::TAU;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use std::cell::RefCell;

use oxifoc_core::foc::controller::{FocController, FocOutput};
use oxifoc_core::foc::detection::sweep::{
    DetectionHardware, DetectionParams, measure_flux_linkage, measure_flux_linkage_spindown,
    measure_inductance, measure_resistance, run_full_detection,
};
use oxifoc_core::foc::detection::types::{
    FluxLinkageParams, InductanceParams, MotorSize, ResistanceParams,
};
use oxifoc_core::foc::pi_controller::PIController;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::{angle_difference, wrap_angle};
use oxifoc_core::motor::ControlMode;
use oxifoc_core::timer::Timer;
use oxifoc_core::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

const DT: f32 = 1.0 / 20_000.0;
const MAX_DUTY: u16 = 1000;
const VBUS: f32 = 24.0;

// ── Simulation infrastructure (same as E2E test) ───────────────────────────

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
    fn new(motor_params: MotorParams) -> Self {
        let kp = motor_params.ld * 1_000.0;
        let ki = motor_params.r * 1_000.0;
        let mut foc = FocController::<SvpwmModulator>::new(VBUS);
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
                ControlMode::Coast => {
                    sim.foc.reset();
                }
                ControlMode::Stopped => {
                    sim.foc.reset();
                    sim.ol_omega = 0.0;
                }
                ControlMode::DirectVoltage { .. } => {
                    sim.ol_omega = 0.0;
                }
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

fn reset_sim(motor_params: MotorParams) {
    SIM.with(|s| {
        *s.borrow_mut() = Some(SimState::new(motor_params));
    });
}

fn err_pct(measured: f32, expected: f32) -> f32 {
    (measured - expected).abs() / expected * 100.0
}

fn print_row(name: &str, measured: f32, expected: f32, unit: &str) {
    println!(
        "  {:<12} {:>10.4} {:<4}  (expected {:>10.4}, err {:>5.1}%)",
        name,
        measured,
        unit,
        expected,
        err_pct(measured, expected)
    );
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    let motor_params = MotorParams {
        j: 5e-4,
        ..MotorParams::default()
    };

    println!("Motor parameter detection report (VirtualMotor simulation)");
    println!("===========================================================");
    println!();
    println!("Ground truth:");
    println!("  R  = {:.4} Ohm", motor_params.r);
    println!("  Ld = {:.1} uH", motor_params.ld * 1e6);
    println!("  Lq = {:.1} uH", motor_params.lq * 1e6);
    println!("  lm = {:.5} Wb", motor_params.lambda);
    println!("  pp = {}", motor_params.pole_pairs);
    println!("  J  = {:.1e} kg*m^2", motor_params.j);
    println!();

    let det_params = DetectionParams {
        motor_size: MotorSize::Small,
        pole_pairs: motor_params.pole_pairs,
        current_max: 10.0,
        pwm_freq_hz: 20_000.0,
    };
    let flux_params = FluxLinkageParams {
        motor_size: MotorSize::Small,
        pole_pairs: motor_params.pole_pairs,
        ..Default::default()
    };

    // ── Individual measurements ────────────────────────────────────────

    println!("--- Individual measurements ---");
    println!();

    // Resistance
    reset_sim(motor_params);
    let mut hw = VirtualHardware;
    let r = block_on(measure_resistance::<VirtualHardware, VirtualTimer>(
        &mut hw,
        &ResistanceParams {
            motor_size: MotorSize::Small,
            current_max: 10.0,
            ..Default::default()
        },
    ));
    match r {
        Ok(v) => print_row("R", v, motor_params.r, "Ohm"),
        Err(e) => println!("  R:           FAILED ({e:?})"),
    }

    // Inductance
    reset_sim(motor_params);
    let l = block_on(measure_inductance::<VirtualHardware, VirtualTimer>(
        &mut hw,
        &InductanceParams {
            motor_size: MotorSize::Small,
            resistance_ohm: motor_params.r,
            ..Default::default()
        },
        20_000.0,
    ));
    match l {
        Ok((ld, lq)) => {
            print_row("Ld", ld * 1e6, motor_params.ld * 1e6, "uH");
            print_row("Lq", lq * 1e6, motor_params.lq * 1e6, "uH");
            print_row("L_avg", (ld + lq) / 2.0 * 1e6, motor_params.ld * 1e6, "uH");
        }
        Err(e) => println!("  L:           FAILED ({e:?})"),
    }

    // Flux linkage — spin-down
    reset_sim(motor_params);
    let lam_sd = block_on(
        measure_flux_linkage_spindown::<VirtualHardware, VirtualTimer>(&mut hw, &flux_params),
    );
    match lam_sd {
        Ok(v) => print_row("lm(spindown)", v, motor_params.lambda, "Wb"),
        Err(e) => println!("  lm(spindown) FAILED ({e:?})"),
    }

    // Flux linkage — driven
    let flux_driven = FluxLinkageParams {
        resistance_ohm: motor_params.r,
        ..flux_params
    };
    reset_sim(motor_params);
    let lam_dr = block_on(measure_flux_linkage::<VirtualHardware, VirtualTimer>(
        &mut hw,
        &flux_driven,
    ));
    match lam_dr {
        Ok(v) => print_row("lm(driven)", v, motor_params.lambda, "Wb"),
        Err(e) => println!("  lm(driven)   FAILED ({e:?})"),
    }

    println!();

    // ── Full detection sequence ────────────────────────────────────────

    println!("--- Full detection sequence (run_full_detection) ---");
    println!();

    reset_sim(motor_params);
    let result = block_on(run_full_detection::<VirtualHardware, VirtualTimer>(
        &mut hw, det_params,
    ));
    match result {
        Ok(det) => {
            print_row("R", det.params.resistance_ohm, motor_params.r, "Ohm");
            print_row(
                "Ld",
                det.params.inductance_d_h * 1e6,
                motor_params.ld * 1e6,
                "uH",
            );
            print_row(
                "Lq",
                det.params.inductance_q_h * 1e6,
                motor_params.lq * 1e6,
                "uH",
            );
            print_row(
                "L_avg",
                det.params.inductance_avg_h * 1e6,
                motor_params.ld * 1e6,
                "uH",
            );
            print_row("lm", det.params.flux_linkage_wb, motor_params.lambda, "Wb");
            println!();
            println!("  Kv     = {:.1} RPM/V", det.params.kv_rpm_per_v);
            println!("  I_max  = {:.1} A", det.params.max_current_a);
            println!("  kp     = {:.4}", det.kp_current);
            println!("  ki     = {:.4}", det.ki_current);
        }
        Err(e) => println!("  FAILED: {e:?}"),
    }
}

//! Virtual motor harness for detection testing and benchmarking.
//!
//! Provides a complete `DetectionHardware` + `Timer` implementation backed
//! by [`VirtualMotor`], running a faithful 20 kHz FOC simulation.  Used by
//! both the E2E integration test and the `detection_report` example.
//!
//! # Quick usage
//!
//! ```ignore
//! use oxifoc_core::foc::detection::virtual_harness::VirtualHarness;
//!
//! let motor = MotorParams { j: 5e-4, ..MotorParams::default() };
//! let det = DetectionParams { vbus: 24.0, ..Default::default() };
//! let result = VirtualHarness::run_detection(motor, 24.0, det);
//! ```

use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use std::cell::RefCell;

use crate::foc::controller::{FocController, FocOutput};
use crate::foc::detection::sweep::{
    DetectionHardware, DetectionParams, DetectionResult, run_full_detection,
};
use crate::foc::detection::types::DetectionError;
use crate::foc::pi_controller::PIController;
use crate::foc::pwm::SvpwmModulator;
use crate::foc::wrap_angle;
use crate::motor::ControlMode;
use crate::timer::Timer;
use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

const DT: f32 = 1.0 / 20_000.0;
const MAX_DUTY: u16 = 1000;

// ── Simulation state ───────────────────────────────────────────────────────

struct SimState {
    foc: FocController<SvpwmModulator>,
    motor: VirtualMotor,
    out: VirtualMotorOutput,
    mode: ControlMode,
    sim_angle: f32,
    ol_omega: f32,
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
        }
    }

    fn step_one(&mut self) -> FocOutput {
        // Mirror FocDriver::step_open_loop: a nonzero velocity integrates the
        // angle every FOC cycle.
        if matches!(self.mode, ControlMode::OpenLoop { .. }) && self.ol_omega != 0.0 {
            self.sim_angle = wrap_angle(self.sim_angle + self.ol_omega * DT);
        }

        let telem = match self.mode {
            ControlMode::OpenLoop { current, .. } => {
                // Same current placement as the firmware: d-axis when locked
                // (velocity 0, holds the rotor), q-axis when spinning
                // (produces torque). The harness used to put it on d in both
                // cases, which hid the open-loop load-angle geometry from
                // every flux-method comparison.
                let (id_t, iq_t) = if self.ol_omega == 0.0 {
                    (current, 0.0)
                } else {
                    (0.0, current)
                };
                self.foc.step(
                    (self.out.ia, self.out.ib, self.out.ic),
                    self.sim_angle,
                    id_t,
                    iq_t,
                    MAX_DUTY,
                    DT,
                )
            }
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
    static SIM: RefCell<Option<SimState>> = const { RefCell::new(None) };
}

// ── DetectionHardware ──────────────────────────────────────────────────────

/// Virtual hardware implementation for detection testing.
///
/// Uses a thread-local `SimState` shared with [`VirtualTimer`].
pub struct VirtualHardware;

impl DetectionHardware for VirtualHardware {
    async fn send_command(&self, mode: ControlMode) {
        SIM.with(|s| {
            let mut borrow = s.borrow_mut();
            let sim = borrow.as_mut().unwrap();

            match mode {
                ControlMode::OpenLoop {
                    angle_rad,
                    velocity_rad_s,
                    pi_gains,
                    ..
                } => {
                    // Apply PI gains override if provided
                    if let Some((kp, ki)) = pi_gains {
                        sim.foc.id_pi.set_gains(kp, ki);
                        sim.foc.iq_pi.set_gains(kp, ki);
                    }
                    // Mirror FocDriver::step_open_loop exactly: with a
                    // velocity the angle keeps integrating from wherever it
                    // is (set_mode does not reset it); with velocity 0 the
                    // commanded angle is used directly. The old harness
                    // reconstructed a velocity from successive commanded
                    // angles, i.e. it simulated a smooth rotation the real
                    // firmware never produced for host-paced angle steps.
                    sim.ol_omega = velocity_rad_s;
                    if velocity_rad_s == 0.0 {
                        sim.sim_angle = wrap_angle(angle_rad);
                    }
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

    fn supports_coast_telemetry(&self) -> bool {
        true
    }

    fn read_coast_telemetry(&self) -> (f32, f32, f32) {
        SIM.with(|s| {
            let s = s.borrow();
            let s = s.as_ref().unwrap();
            (s.out.bemf_alpha, s.out.bemf_beta, s.out.omega_e)
        })
    }
}

// ── Timer ──────────────────────────────────────────────────────────────────

/// Virtual timer that advances the simulation by the requested duration.
pub struct VirtualTimer;

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

// ── Executor ───────────────────────────────────────────────────────────────

/// Minimal single-poll executor for running async detection in tests.
pub fn block_on<F: core::future::Future>(f: F) -> F::Output {
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
        Poll::Pending => panic!("unexpected Pending in virtual harness"),
    }
}

// ── Public API ─────────────────────────────────────────────────────────────

/// High-level entry point: set up simulation and run a single async future.
///
/// Resets the simulation state before running.  The future has access to
/// [`VirtualHardware`] and [`VirtualTimer`] via the thread-local.
pub fn with_sim<F, R>(motor_params: MotorParams, vbus: f32, f: F) -> R
where
    F: FnOnce(&mut VirtualHardware) -> R,
{
    SIM.with(|s| {
        *s.borrow_mut() = Some(SimState::new(motor_params, vbus));
    });
    let mut hw = VirtualHardware;
    f(&mut hw)
}

/// Virtual Hall sensor reader — reads the current Hall state from the simulation.
pub struct VirtualHallReader;

impl crate::foc::hall_calibration::HallReader for VirtualHallReader {
    fn read_hall_state(&self) -> u8 {
        SIM.with(|s| s.borrow().as_ref().unwrap().out.hall_state)
    }
}

/// Convenience: run `run_full_detection` against a virtual motor.
pub fn run_detection(
    motor_params: MotorParams,
    vbus: f32,
    det_params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    with_sim(motor_params, vbus, |hw| {
        block_on(run_full_detection::<
            VirtualHardware,
            VirtualTimer,
            crate::foc::trig::LibmSinCos,
        >(hw, det_params))
    })
}

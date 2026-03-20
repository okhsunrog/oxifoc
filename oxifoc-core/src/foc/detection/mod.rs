//! Motor parameter detection algorithms.
//!
//! This module provides VESC-style motor parameter detection for:
//! - Phase resistance (R)
//! - Inductance (Ld, Lq) via HFI injection
//! - Flux linkage (λ)
//! - DC offset calibration
//! - Auto PI controller tuning
//!
//! # Usage
//!
//! The detection functions are platform-agnostic. Platform crates (like oxifoc-f405)
//! implement the actual measurement sweeps using async functions, while this module
//! provides the core algorithms and accumulators.
//!
//! ## Typical Detection Flow
//!
//! 1. **DC Offset Calibration** - Measure current sensor offsets
//! 2. **Resistance Measurement** - Apply DC current, measure V/I
//! 3. **Inductance Measurement** - HFI injection + FFT analysis
//! 4. **Flux Linkage Measurement** - Open-loop spin, measure Vq/ω
//! 5. **PI Tuning** - Calculate Kp/Ki from R and L
//!
//! ## Motor Size
//!
//! Test currents are determined by motor size to prevent overheating:
//!
//! | Size | max_power_loss | Typical motors |
//! |------|----------------|----------------|
//! | Mini | 20W | ~75g outrunners |
//! | Small | 50W | ~200g motors |
//! | Medium | 120W | ~750g motors |
//! | Large | 400W | ~2kg motors |
//!
//! ## Example
//!
//! ```ignore
//! use oxifoc_core::foc::detection::{
//!     types::{MotorSize, MotorParams, ResistanceParams},
//!     resistance::ResistanceMeasurement,
//!     pi_tuning::calculate_foc_gains,
//! };
//!
//! // Configure for medium motor
//! let params = ResistanceParams {
//!     motor_size: MotorSize::Medium,
//!     ..Default::default()
//! };
//!
//! // Create measurement accumulator
//! let mut measurement = ResistanceMeasurement::new(100);
//!
//! // Platform code collects samples during measurement sweep
//! // measurement.record(vd, id);
//!
//! // Get result
//! let resistance = measurement.finish()?;
//!
//! // Auto-tune PI gains
//! let mut motor_params = MotorParams::default();
//! motor_params.resistance_ohm = resistance;
//! motor_params.inductance_avg_h = 0.0001; // 100µH
//! let gains = calculate_foc_gains(&motor_params, 1000.0);
//! ```

/// Common types for detection (MotorSize, MotorParams, errors, etc.)
pub mod types;

/// Enhanced DC offset calibration for current sensors
pub mod dc_offset;

/// Flux linkage (λ) measurement via open-loop spinning
pub mod flux_linkage;

/// Inductance (Ld, Lq) measurement via HFI injection
pub mod inductance;

/// Auto PI controller tuning from measured parameters
pub mod pi_tuning;

/// Phase resistance measurement
pub mod resistance;

/// Async detection sweeps (requires platform implementation)
pub mod sweep;

// Re-export commonly used types for convenience
pub use types::{
    DcOffsetParams, DcOffsets, DetectionError, FluxLinkageParams, InductanceParams, MotorParams,
    MotorSize, ResistanceParams,
};

/// Integration tests: feed VirtualMotor output into detection accumulators
/// and verify the detected values match the known motor parameters.
#[cfg(all(test, feature = "virtual-motor"))]
mod integration_tests {
    use crate::foc::controller::FocController;
    use crate::foc::pi_controller::PIController;
    use crate::foc::pwm::SvpwmModulator;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    use super::flux_linkage::FluxLinkageMeasurement;
    #[cfg(feature = "microfft")]
    use super::inductance::{HfiInjector, InductanceMeasurement};
    use super::resistance::ResistanceMeasurement;
    #[cfg(feature = "microfft")]
    use super::types::InductanceParams;

    /// Verify resistance detection against the virtual motor's known R.
    ///
    /// Strategy: drive id_target = 1 A, iq_target = 0.  With iq = 0 the
    /// electromagnetic torque is zero, so the motor stays locked at angle 0.
    /// At steady state: Vd = R × Id, so R = Vd / Id ≈ 0.5 Ω.
    #[test]
    fn detect_resistance_matches_virtual_motor() {
        const DT: f32 = 1.0 / 20_000.0;
        let params = MotorParams::default(); // R = 0.5 Ω
        let kp = params.ld * 1_000.0;
        let ki = params.r * 1_000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Settle for 2 000 steps (0.1 s ≈ 100 × Ld/R time constants).
        for _ in 0..2_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 1.0, 0.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        // Collect 500 steady-state (Vd, Id) samples.
        let mut meas = ResistanceMeasurement::new(500);
        for _ in 0..500 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 1.0, 0.0, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
            meas.record(telem.vd, telem.id);
        }

        let r_measured = meas.finish().unwrap();
        let error = (r_measured - params.r).abs() / params.r;
        assert!(
            error < 0.10,
            "Resistance error too large: measured = {r_measured:.4} Ω, \
             expected = {:.4} Ω, error = {:.1}%",
            params.r,
            error * 100.0
        );
    }

    /// Verify flux-linkage detection against the virtual motor's known λ.
    ///
    /// Strategy: spin the motor with a small iq_target and high friction so it
    /// reaches steady state quickly.  At steady state with id ≈ 0:
    ///   Vq ≈ R × Iq + ωe × λ  →  λ = (Vq − R × Iq) / ωe ≈ 0.01 Wb.
    #[test]
    fn detect_flux_linkage_matches_virtual_motor() {
        const DT: f32 = 1.0 / 20_000.0;
        // friction_b = 1e-3 → time constant J/friction_b = 0.1 s → 5× by 0.5 s
        let params = MotorParams {
            friction_b: 1e-3,
            ..MotorParams::default()
        };
        let kp = params.ld * 1_000.0;
        let ki = params.r * 1_000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);
        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Spin with iq_target = 0.5 A for 0.5 s (10 000 steps ≈ 5 time constants).
        for _ in 0..10_000 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 0.5, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
        }

        assert!(
            out.omega_e > 10.0,
            "motor must be spinning before measurement: ωe = {}",
            out.omega_e
        );

        // Collect 500 steady-state (Vq, Iq, ωe) samples.
        let mut meas = FluxLinkageMeasurement::new(params.r, 500);
        for _ in 0..500 {
            let telem = foc.step((out.ia, out.ib, out.ic), out.angle_rad, 0.0, 0.5, 1000, DT);
            out = motor.step(telem.v_alpha, telem.v_beta, 0.0, DT);
            meas.record(telem.vq, telem.iq, out.omega_e);
        }

        let lambda_measured = meas.finish().unwrap();
        let error = (lambda_measured - params.lambda).abs() / params.lambda;
        assert!(
            error < 0.15,
            "Flux linkage error too large: measured = {lambda_measured:.5} Wb, \
             expected = {:.5} Wb, error = {:.1}%",
            params.lambda,
            error * 100.0
        );
    }

    /// End-to-end HFI inductance measurement using VirtualMotor.
    ///
    /// Unlike the unit test in inductance.rs which uses a pure-inductor model
    /// (di = V·dt/L), this drives VirtualMotor with full PMSM dynamics:
    /// resistance, inductance, back-EMF, and mechanical model. The rotor is
    /// locked at angle 0 with a DC holding voltage, then HFI injection is
    /// applied on top and the current response is fed to InductanceMeasurement.
    #[cfg(feature = "microfft")]
    #[test]
    fn detect_inductance_matches_virtual_motor() {
        use crate::foc::transforms;

        const DT: f32 = 1.0 / 20_000.0;
        const PWM_FREQ: f32 = 20_000.0;
        let params = MotorParams::default(); // Ld = Lq = 0.5 mH (SPM)

        let mut motor = VirtualMotor::new(params);
        let mut out = VirtualMotorOutput::default();

        // Phase 1: lock rotor at angle 0 with DC holding voltage
        // At steady state: V = R × I, so V_hold = R × I_hold
        let hold_current = 2.0;
        let hold_v = hold_current * params.r; // 1.0V for 2A × 0.5Ω
        for _ in 0..4_000 {
            // 200ms settle — alpha axis only (angle 0)
            out = motor.step(hold_v, 0.0, 0.0, DT);
        }

        // Phase 2: inject HFI and collect inductance samples
        let ind_params = InductanceParams {
            hfi_frequency_hz: 1000.0,
            hfi_voltage_v: 3.0,
            num_cycles: 10,
            hold_current_a: hold_current,
            ..Default::default()
        };

        let ind_params = InductanceParams {
            hfi_frequency_hz: 1000.0,
            hfi_voltage_v: 3.0,
            num_cycles: 10,
            hold_current_a: hold_current,
            resistance_ohm: params.r, // use known R for compensation
            ..Default::default()
        };

        let mut injector = HfiInjector::new(
            ind_params.hfi_frequency_hz,
            ind_params.hfi_voltage_v,
            PWM_FREQ,
        );
        let mut measurement = InductanceMeasurement::new(&ind_params, PWM_FREQ);

        let mut prev_v_a = 0.0f32;
        let mut prev_v_b = 0.0f32;

        while !measurement.is_complete() {
            let injection_angle = injector.injection_angle();
            let (v_inj_a, v_inj_b) = injector.step(DT);

            // Apply holding voltage + HFI injection
            out = motor.step(hold_v + v_inj_a, v_inj_b, 0.0, DT);

            let (i_alpha, i_beta) = transforms::clarke(out.ia, out.ib);
            measurement.record(i_alpha, i_beta, injection_angle, prev_v_a, prev_v_b);
            prev_v_a = v_inj_a;
            prev_v_b = v_inj_b;
        }

        let result = measurement.finish().unwrap();

        // With carrier demodulation + R compensation, both axes should
        // be close to the true value for an SPM motor.
        let expected_l = params.ld;
        let ld_error = (result.ld - expected_l).abs() / expected_l;
        let lq_error = (result.lq - expected_l).abs() / expected_l;

        assert!(
            ld_error < 0.30,
            "Ld error too large: {:.1}% (measured {:.2} µH, expected {:.2} µH)",
            ld_error * 100.0,
            result.ld * 1e6,
            expected_l * 1e6
        );
        assert!(
            lq_error < 0.30,
            "Lq error too large: {:.1}% (measured {:.2} µH, expected {:.2} µH)",
            lq_error * 100.0,
            result.lq * 1e6,
            expected_l * 1e6
        );

        // SPM motor: Ld ≈ Lq, ratio should be close to 1.0.
        // The holding current on the d-axis creates some residual asymmetry
        // in the PMSM simulation, so allow a wider band than ideal.
        let ratio = result.ld / result.lq;
        assert!(
            (0.5..=2.0).contains(&ratio),
            "Ld/Lq ratio {:.2} too far from 1.0 for SPM motor",
            ratio
        );
    }

    /// End-to-end test of the full async detection orchestrator.
    ///
    /// Runs [`run_full_detection()`] from `sweep.rs` against a `VirtualMotor`
    /// parameterised as a 6354-class electric skateboard outrunner.  The
    /// higher rotor inertia (~0.001 kg·m²) is representative of a real
    /// motor+wheel load and ensures the motor coasts through the
    /// flux-linkage settle phase — just as it does on real hardware.
    ///
    /// Every simulation step is one 50 µs FOC cycle at 20 kHz, faithfully
    /// reproducing the control loop timing of real firmware.  Between
    /// `send_command` calls the estimated open-loop angular velocity is
    /// used to continuously advance the commutation angle so the motor
    /// sees smooth rotation rather than discrete step-and-hold.
    #[cfg(feature = "microfft")]
    #[test]
    fn run_full_detection_e2e() {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        use std::cell::RefCell;

        use super::sweep::{DetectionHardware, DetectionParams, run_full_detection};
        use super::types::MotorSize;
        use crate::foc::controller::{FocController, FocOutput};
        use core::f32::consts::TAU;

        use crate::foc::{angle_difference, wrap_angle};
        use crate::motor::ControlMode;
        use crate::timer::Timer;

        const DT: f32 = 1.0 / 20_000.0;
        const MAX_DUTY: u16 = 1000;

        // ── Shared simulation state ────────────────────────────────────────
        struct SimState {
            foc: FocController<SvpwmModulator>,
            motor: VirtualMotor,
            out: VirtualMotorOutput,
            mode: ControlMode,
            /// Smoothly-advancing angle for OpenLoop mode.  Between
            /// `send_command` calls the Timer advances this at `ol_omega`
            /// so the motor sees continuous rotation.
            sim_angle: f32,
            /// Estimated open-loop angular velocity (rad/s).
            ol_omega: f32,
            /// Commanded angle from the last `send_command` (for ω estimation).
            prev_cmd_angle: f32,
            /// Steps elapsed since the last `send_command`.
            steps_since_send: u64,
        }

        impl SimState {
            /// One FOC cycle (50 µs).
            fn step_one(&mut self) -> FocOutput {
                // Advance the simulation angle for OpenLoop mode so the
                // motor sees continuous rotation between send_command calls.
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
                    ControlMode::Coast => FocOutput::empty(),
                    ControlMode::Stopped => FocOutput::empty(),
                    _ => FocOutput::empty(),
                };

                // Advance the motor with the appropriate terminal model.
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

        // ── DetectionHardware backed by VirtualMotor ───────────────────────
        struct VirtualHardware;

        impl DetectionHardware for VirtualHardware {
            fn send_command(&self, mode: ControlMode) {
                SIM.with(|s| {
                    let mut borrow = s.borrow_mut();
                    let sim = borrow.as_mut().unwrap();

                    match mode {
                        ControlMode::OpenLoop { angle_rad, .. } => {
                            // On entry from a non-OpenLoop mode, snap the
                            // simulation angle to the first commanded angle.
                            if !matches!(sim.mode, ControlMode::OpenLoop { .. }) {
                                sim.sim_angle = angle_rad;
                                sim.ol_omega = 0.0;
                            }
                            // Update ω estimate from consecutive commanded
                            // angles, but only for short gaps (< 100 ms).
                            // Long gaps are settle/pause periods where the
                            // previous ω should be kept.
                            if sim.steps_since_send > 0 && sim.steps_since_send < 2000 {
                                let elapsed = sim.steps_since_send as f32 * DT;
                                let wrapped = angle_difference(angle_rad, sim.prev_cmd_angle);
                                // Unwrap: the open-loop ramp can advance by
                                // more than π per step at high speeds, so
                                // angle_difference may alias.  Use the previous
                                // ω estimate to resolve the ambiguity.
                                let expected = sim.ol_omega * elapsed;
                                let n = ((expected - wrapped) / TAU).round();
                                let delta = wrapped + n * TAU;
                                sim.ol_omega = delta / elapsed;
                            }
                            sim.prev_cmd_angle = angle_rad;
                            sim.steps_since_send = 0;
                        }
                        ControlMode::Coast => {
                            // Motor coasts freely — keep ol_omega for
                            // Timer stepping but reset FOC integrators.
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

        // ── Timer: each ms/µs → exact number of 50 µs FOC steps ───────────
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

        // ── Minimal single-poll executor ───────────────────────────────────
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
                Poll::Pending => panic!("unexpected Pending in virtual motor sweep test"),
            }
        }

        // ── Motor parameters ────────────────────────────────────────────────
        // Default electrical params (R=0.5 Ω, Ld=Lq=0.5 mH, λ=0.01 Wb, 7pp)
        // with higher inertia so the 2 A open-loop current can track the ramp
        // (max accel = 0.21/5e-4 = 420 rad/s² > ramp's 183 rad/s²) and the
        // motor coasts through the 1 s settle with minimal speed loss.
        let motor_params = MotorParams {
            j: 5e-4,
            ..MotorParams::default()
        };
        let kp = motor_params.ld * 1_000.0;
        let ki = motor_params.r * 1_000.0;

        let mut foc = FocController::<SvpwmModulator>::new(24.0);
        foc.id_pi = PIController::new(kp, ki);
        foc.iq_pi = PIController::new(kp, ki);

        SIM.with(|s| {
            *s.borrow_mut() = Some(SimState {
                foc,
                motor: VirtualMotor::new(motor_params),
                out: VirtualMotorOutput::default(),
                mode: ControlMode::Stopped,
                sim_angle: 0.0,
                ol_omega: 0.0,
                prev_cmd_angle: 0.0,
                steps_since_send: 0,
            });
        });

        // ── Run the full detection sequence ────────────────────────────────
        let det_params = DetectionParams {
            motor_size: MotorSize::Small,
            pole_pairs: motor_params.pole_pairs,
            current_max: 10.0,
            pwm_freq_hz: 20_000.0,
        };

        let mut hw = VirtualHardware;
        let result = block_on(run_full_detection::<VirtualHardware, VirtualTimer>(
            &mut hw, det_params,
        ));
        let result = result.expect("full detection sequence should succeed");

        // ── Verify detected parameters against ground truth ────────────────
        assert!(
            result.params.is_complete(),
            "all motor parameters should be detected"
        );

        // Resistance (R = 0.5 Ω)
        let r_err = (result.params.resistance_ohm - motor_params.r).abs() / motor_params.r;
        assert!(
            r_err < 0.20,
            "R error {:.1}%: measured {:.4} Ω, expected {:.4} Ω",
            r_err * 100.0,
            result.params.resistance_ohm,
            motor_params.r,
        );

        // Inductance (Ld = Lq = 0.5 mH)
        let l_err = (result.params.inductance_avg_h - motor_params.ld).abs() / motor_params.ld;
        assert!(
            l_err < 0.15,
            "L error {:.1}%: measured {:.2} µH, expected {:.2} µH",
            l_err * 100.0,
            result.params.inductance_avg_h * 1e6,
            motor_params.ld * 1e6,
        );

        // Flux linkage (λ = 10 mWb)
        let lam_err =
            (result.params.flux_linkage_wb - motor_params.lambda).abs() / motor_params.lambda;
        assert!(
            lam_err < 0.05,
            "λ error {:.1}%: measured {:.5} Wb, expected {:.5} Wb",
            lam_err * 100.0,
            result.params.flux_linkage_wb,
            motor_params.lambda,
        );

        // PI gains should be positive and finite
        assert!(
            result.kp_current > 0.0 && result.kp_current.is_finite(),
            "kp should be positive: {}",
            result.kp_current,
        );
        assert!(
            result.ki_current > 0.0 && result.ki_current.is_finite(),
            "ki should be positive: {}",
            result.ki_current,
        );

        // Derived values
        assert!(
            result.params.kv_rpm_per_v > 50.0,
            "Kv should be reasonable: {}",
            result.params.kv_rpm_per_v,
        );
        assert!(
            result.params.max_current_a > 0.0,
            "max current should be calculated: {}",
            result.params.max_current_a,
        );
    }
}

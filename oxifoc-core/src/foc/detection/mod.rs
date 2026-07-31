//! Motor parameter detection algorithms.
//!
//! This module provides VESC-style motor parameter detection for:
//! - Phase resistance (R)
//! - Inductance (Ld, Lq) via HFI injection
//! - Flux linkage (λ)
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
//! 1. **Resistance Measurement** - Apply DC current, measure V/I
//! 2. **Inductance Measurement** - HFI injection + FFT analysis
//! 3. **Flux Linkage Measurement** - Open-loop spin, measure Vq/ω
//! 4. **PI Tuning** - Calculate Kp/Ki from R and L
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

/// Flux linkage (λ) measurement via open-loop spinning
#[cfg(feature = "detection")]
pub mod flux_linkage;

/// Inductance (Ld, Lq) measurement via HFI injection (rotating injection + FFT)
#[cfg(feature = "hfi-detect")]
pub mod inductance;

/// Auto PI controller tuning from measured parameters
pub mod pi_tuning;

/// Phase resistance measurement
#[cfg(feature = "detection")]
pub mod resistance;

/// Async detection sweeps (requires platform implementation)
#[cfg(feature = "detection")]
pub mod sweep;

/// Inductance measurement via voltage pulse (fallback for HFI)
#[cfg(feature = "detection")]
pub mod voltage_pulse;

/// Virtual motor harness for detection testing and benchmarking
#[cfg(all(
    feature = "detection",
    feature = "virtual-motor",
    any(test, feature = "std")
))]
pub mod virtual_harness;

/// Embassy-based detection hardware (timer, hardware abstraction, hall reader)
#[cfg(all(feature = "detection", feature = "embassy", feature = "runtime"))]
pub mod embassy_hw;

// Re-export commonly used types for convenience
pub use types::{
    DcOffsetParams, DcOffsets, DetectionError, FluxLinkageParams, InductanceParams, MotorParams,
    MotorSize, ResistanceParams, VoltagePulseParams,
};

/// Integration tests: feed VirtualMotor output into detection accumulators
/// and verify the detected values match the known motor parameters.
#[cfg(all(test, feature = "detection", feature = "virtual-motor"))]
mod integration_tests {
    use crate::foc::controller::FocController;
    use crate::foc::pi_controller::PIController;
    use crate::foc::pwm::SvpwmModulator;
    use crate::virtual_motor::{MotorParams, VirtualMotor, VirtualMotorOutput};

    use super::flux_linkage::FluxLinkageMeasurement;
    #[cfg(feature = "hfi-detect")]
    use super::inductance::{HfiInjector, InductanceMeasurement};
    use super::resistance::ResistanceMeasurement;
    #[cfg(feature = "hfi-detect")]
    use super::types::InductanceParams;

    /// Full-pipeline L tolerance, shared by the HFI and voltage-pulse configs
    /// (both now resolve L to a few percent on these motors). The worst case
    /// is the gimbal on a 12 V bus (`run_full_detection_high_r_low_vbus`,
    /// ~6.5%): there the bus barely covers the holding voltage, leaving the
    /// pulse little headroom, and the ideal sub-step-1 plant adds its
    /// forward-Euler R·dt/2L discretization bias on top. HFI precision is
    /// asserted separately by `detect_inductance_matches_virtual_motor`.
    const E2E_L_TOL: f32 = 0.10;

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
    #[test]
    #[cfg(feature = "hfi-detect")]
    fn detect_inductance_matches_virtual_motor() {
        use crate::foc::transforms;

        const DT: f32 = 1.0 / 20_000.0;
        const PWM_FREQ: f32 = 20_000.0;
        let params = MotorParams::default(); // Ld = Lq = 0.5 mH (SPM)

        let mut motor = VirtualMotor::new(params);

        // Phase 1: lock rotor at angle 0 with DC holding voltage
        // At steady state: V = R × I, so V_hold = R × I_hold
        let hold_current = 2.0;
        let hold_v = hold_current * params.r; // 1.0V for 2A × 0.5Ω
        for _ in 0..4_000 {
            // 200ms settle — alpha axis only (angle 0)
            motor.step(hold_v, 0.0, 0.0, DT);
        }

        // Phase 2: inject HFI and collect inductance samples
        let ind_params = InductanceParams {
            hfi_frequency_hz: 1000.0,
            hfi_voltage_v: 3.0,
            num_cycles: 10,
            hold_current_a: hold_current,
            resistance_ohm: params.r, // use known R for compensation
            ..Default::default()
        };

        use crate::foc::trig::LibmSinCos;
        let mut injector = HfiInjector::<LibmSinCos>::new(
            ind_params.hfi_frequency_hz,
            ind_params.hfi_voltage_v,
            PWM_FREQ,
        );
        let mut measurement = InductanceMeasurement::<LibmSinCos>::new(&ind_params, PWM_FREQ);

        while !measurement.is_complete() {
            let injection_angle = injector.injection_angle();
            let (v_inj_a, v_inj_b) = injector.step(DT);

            // Apply holding voltage + HFI injection — the plant integrates
            // this voltage in the SAME step (zero pipeline), so the di seen
            // by record() is driven by exactly this command: pass it as-is
            // (record's pairing contract).
            let out = motor.step(hold_v + v_inj_a, v_inj_b, 0.0, DT);

            let (i_alpha, i_beta) = transforms::clarke(out.ia, out.ib);
            measurement.record(i_alpha, i_beta, injection_angle, v_inj_a, v_inj_b);
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
            "Ld/Lq ratio {ratio:.2} too far from 1.0 for SPM motor"
        );
    }

    /// End-to-end test of the full async detection orchestrator.
    ///
    /// Uses the shared [`virtual_harness`] to run `run_full_detection()`
    /// against a VirtualMotor and verify all parameters match ground truth.
    #[test]
    fn run_full_detection_e2e() {
        use super::sweep::DetectionParams;
        use super::types::MotorSize;
        use super::virtual_harness::run_detection;

        let motor_params = MotorParams {
            j: 5e-4,
            ..MotorParams::default()
        };

        let det_params = DetectionParams {
            motor_size: MotorSize::Small,
            pole_pairs: motor_params.pole_pairs,
            current_max: 10.0,
            max_power_loss_w: 50.0,
            pwm_freq_hz: 20_000.0,
            vbus: 24.0,
            openloop_erpm: 1400.0,
        };

        let result = run_detection(motor_params, 24.0, det_params)
            .expect("full detection sequence should succeed");

        assert!(result.params.is_complete());

        let r_err = (result.params.resistance_ohm - motor_params.r).abs() / motor_params.r;
        assert!(r_err < 0.20, "R error {:.1}%", r_err * 100.0);

        let l_err = (result.params.inductance_avg_h - motor_params.ld).abs() / motor_params.ld;
        assert!(l_err < E2E_L_TOL, "L error {:.1}%", l_err * 100.0);

        let lam_err =
            (result.params.flux_linkage_wb - motor_params.lambda).abs() / motor_params.lambda;
        assert!(lam_err < 0.05, "λ error {:.1}%", lam_err * 100.0);

        assert!(result.kp_current > 0.0 && result.kp_current.is_finite());
        assert!(result.ki_current > 0.0 && result.ki_current.is_finite());
        assert!(result.params.kv_rpm_per_v > 50.0);
        assert!(result.params.max_current_a > 0.0);
    }

    /// Regression: high-R motor on a low bus (gimbal-class).
    ///
    /// The thermal safe-current formula alone (√(P/R/1.5) = 1.29 A at 8 Ω /
    /// 20 W) demands 10.3 V of the ~6.9 V a 12 V bus can drive — without the
    /// bus-voltage clamp the resistance step saturates short of its setpoint
    /// and aborts with `UnexpectedMotion`, failing the whole sequence.
    // Runs on both configs (HFI when built, else the voltage-pulse fallback).
    // High-R + low-vbus is the pulse method's hardest case — v_hold = R·I eats
    // most of the bus, leaving a floored pulse step — yet the discharge-anchored
    // absolute-current accumulator still lands within E2E_L_TOL (~6.5%).
    #[test]
    fn run_full_detection_high_r_low_vbus() {
        use super::sweep::DetectionParams;
        use super::types::MotorSize;
        use super::virtual_harness::run_detection;

        let motor_params = MotorParams {
            r: 8.0,
            ld: 3e-3,
            lq: 3e-3,
            lambda: 0.005,
            pole_pairs: 11,
            j: 5e-6,
            friction_b: 1e-5,
            hall_offset: 0.0,
            ..MotorParams::default()
        };

        let det_params = DetectionParams {
            motor_size: MotorSize::Mini,
            pole_pairs: motor_params.pole_pairs,
            current_max: 10.0,
            max_power_loss_w: 20.0,
            pwm_freq_hz: 20_000.0,
            vbus: 12.0,
            openloop_erpm: 1400.0,
        };

        let result = run_detection(motor_params, 12.0, det_params)
            .expect("gimbal-class detection must survive the low bus");

        let r_err = (result.params.resistance_ohm - motor_params.r).abs() / motor_params.r;
        assert!(r_err < 0.05, "R error {:.1}%", r_err * 100.0);

        let l_err = (result.params.inductance_avg_h - motor_params.ld).abs() / motor_params.ld;
        assert!(l_err < E2E_L_TOL, "L error {:.1}%", l_err * 100.0);

        let lam_err =
            (result.params.flux_linkage_wb - motor_params.lambda).abs() / motor_params.lambda;
        assert!(lam_err < 0.05, "λ error {:.1}%", lam_err * 100.0);
    }

    /// Full detection on the non-ideal plant: sub-stepped integration
    /// (breaks the sim/estimator discretization lockstep), g431-class
    /// dead-time distortion (with the matching compensation the firmware
    /// configures), and a 12-bit ±31 A current sensor with 1 LSB of noise.
    ///
    /// The low-R eskate-class motor is the adversarial case: its entire
    /// R·I holding voltage (~0.3 V) is smaller than the dead-time
    /// distortion (~0.29 V), so this pins the chain that used to fail —
    /// probe-R retry, settled hold-voltage capture, and dead-time comp in
    /// `apply_dq` (an uncompensated DirectVoltage hold collapses and trips
    /// the open-circuit gate).
    #[test]
    fn run_full_detection_nonideal_plant() {
        use super::sweep::DetectionParams;
        use super::types::MotorSize;
        use super::virtual_harness::run_detection;

        const VBUS: f32 = 48.0;
        const ADC_LSB_A: f32 = 62.0 / 4096.0;
        let motor_params = MotorParams {
            r: 0.035,
            ld: 1.5e-5,
            lq: 1.5e-5,
            lambda: 0.0085,
            pole_pairs: 7,
            j: 1e-3,
            friction_b: 1e-3,
            substeps: 10,
            dead_time_v: 300e-9 * 20_000.0 * VBUS,
            adc_lsb_a: ADC_LSB_A,
            adc_noise_a: ADC_LSB_A,
            ..MotorParams::default()
        };

        let det_params = DetectionParams {
            motor_size: MotorSize::Medium,
            pole_pairs: motor_params.pole_pairs,
            current_max: 10.0,
            max_power_loss_w: MotorSize::Medium.max_power_loss_w(),
            pwm_freq_hz: 20_000.0,
            vbus: VBUS,
            openloop_erpm: 700.0,
        };

        let result = run_detection(motor_params, VBUS, det_params)
            .expect("low-R detection must survive dead-time distortion");

        let r_err = (result.params.resistance_ohm - motor_params.r).abs() / motor_params.r;
        assert!(r_err < 0.05, "R error {:.1}%", r_err * 100.0);

        let l_err = (result.params.inductance_avg_h - motor_params.ld).abs() / motor_params.ld;
        assert!(l_err < E2E_L_TOL, "L error {:.1}%", l_err * 100.0);

        let lam_err =
            (result.params.flux_linkage_wb - motor_params.lambda).abs() / motor_params.lambda;
        assert!(lam_err < 0.02, "λ error {:.1}%", lam_err * 100.0);
    }

    /// THE pipeline-skew regression: same adversarial low-R motor, same
    /// non-idealities, plus a one-cycle actuation pipeline. Before the fix
    /// the L step failed at +1000% (the demod paired currents with the
    /// injection one cycle off = 90° of carrier phase); now the lag probe
    /// measures the depth in place, the history ring pairs explicitly and
    /// the |Z| cross-check guards the result.
    // Runs on both configs. For HFI this is the pipeline-skew regression (the
    // demod once paired currents one cycle off = +1000%). For the voltage-pulse
    // fallback the 1-cycle delay is exactly what the per-pulse discharge +
    // argmax edge-find absorbs — it lands within ~1% here.
    #[test]
    fn run_full_detection_nonideal_plant_with_delay() {
        use super::sweep::DetectionParams;
        use super::types::MotorSize;
        use super::virtual_harness::run_detection;

        const VBUS: f32 = 48.0;
        const ADC_LSB_A: f32 = 62.0 / 4096.0;
        let motor_params = MotorParams {
            r: 0.035,
            ld: 1.5e-5,
            lq: 1.5e-5,
            lambda: 0.0085,
            pole_pairs: 7,
            j: 1e-3,
            friction_b: 1e-3,
            substeps: 10,
            dead_time_v: 300e-9 * 20_000.0 * VBUS,
            adc_lsb_a: ADC_LSB_A,
            adc_noise_a: ADC_LSB_A,
            actuation_delay_steps: 1,
            ..MotorParams::default()
        };

        let det_params = DetectionParams {
            motor_size: MotorSize::Medium,
            pole_pairs: motor_params.pole_pairs,
            current_max: 10.0,
            max_power_loss_w: MotorSize::Medium.max_power_loss_w(),
            pwm_freq_hz: 20_000.0,
            vbus: VBUS,
            openloop_erpm: 700.0,
        };

        let result = run_detection(motor_params, VBUS, det_params)
            .expect("detection must survive a one-cycle actuation pipeline");

        let r_err = (result.params.resistance_ohm - motor_params.r).abs() / motor_params.r;
        assert!(r_err < 0.05, "R error {:.1}%", r_err * 100.0);

        let l_err = (result.params.inductance_avg_h - motor_params.ld).abs() / motor_params.ld;
        assert!(l_err < E2E_L_TOL, "L error {:.1}%", l_err * 100.0);

        let lam_err =
            (result.params.flux_linkage_wb - motor_params.lambda).abs() / motor_params.lambda;
        assert!(lam_err < 0.02, "λ error {:.1}%", lam_err * 100.0);
    }

    /// The |Z| magnitude cross-check must catch a mispaired demod: force a
    /// WRONG pipeline lag (1) on a plant whose true depth is 2 — the
    /// phase-sensitive demod corrupts silently, the latency-immune
    /// magnitude does not, and the disagreement must surface as an error
    /// (LowConfidence → the auto ladder would fall back to the pulse
    /// method). The auto-probed run on the identical plant must succeed.
    #[test]
    #[cfg(feature = "hfi-detect")]
    fn hfi_mispairing_caught_by_magnitude_cross_check() {
        use super::sweep::measure_inductance;
        use super::virtual_harness::{VirtualHardware, VirtualTimer, block_on, with_sim};
        use crate::foc::trig::LibmSinCos;

        const VBUS: f32 = 48.0;
        let plant = MotorParams {
            r: 0.035,
            ld: 1.5e-5,
            lq: 1.5e-5,
            lambda: 0.0085,
            pole_pairs: 7,
            j: 1e-3,
            friction_b: 1e-3,
            substeps: 10,
            actuation_delay_steps: 1, // harness depth becomes 2 cycles
            ..MotorParams::default()
        };
        let run = |pipeline_lag: i8| {
            with_sim(plant, VBUS, |hw| {
                let ip = InductanceParams {
                    resistance_ohm: plant.r,
                    hold_current_a: 8.0,
                    vbus: VBUS,
                    pipeline_lag,
                    ..Default::default()
                };
                block_on(measure_inductance::<
                    VirtualHardware,
                    VirtualTimer,
                    LibmSinCos,
                >(hw, &ip, 20_000.0))
            })
        };

        let auto = run(-1).expect("auto-probed lag must measure accurately");
        let l_err = ((auto.0 + auto.1) / 2.0 - plant.ld).abs() / plant.ld;
        assert!(l_err < 0.10, "auto-lag L error {:.1}%", l_err * 100.0);

        let forced_wrong = run(1);
        assert!(
            forced_wrong.is_err(),
            "a mispaired demod must not return a plausible-looking L: {forced_wrong:?}"
        );
    }
}

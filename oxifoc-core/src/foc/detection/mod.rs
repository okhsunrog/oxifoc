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
    use super::resistance::ResistanceMeasurement;

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
}

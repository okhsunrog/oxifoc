//! Async motor parameter detection sweeps.
//!
//! This module provides platform-agnostic async functions for motor detection.
//! Platforms implement the `DetectionHardware` trait to provide hardware access,
//! and the `Timer` trait for async delays.
//!
//! # Example
//!
//! ```ignore
//! use oxifoc_core::foc::detection::sweep::{DetectionHardware, measure_resistance};
//! use oxifoc_core::timer::Timer;
//!
//! struct MyHardware;
//! struct MyTimer;
//!
//! impl DetectionHardware for MyHardware {
//!     // ... implement trait methods ...
//! }
//!
//! impl Timer for MyTimer {
//!     async fn after_millis(ms: u64) { /* ... */ }
//!     async fn after_micros(us: u64) { /* ... */ }
//! }
//!
//! async fn detect() {
//!     let hw = MyHardware;
//!     let resistance = measure_resistance::<_, MyTimer>(&hw, &params).await?;
//! }
//! ```

use core::future::Future;

use super::flux_linkage::FluxLinkageMeasurement;
use super::inductance::{HfiInjector, InductanceMeasurement};
use super::pi_tuning::{calculate_foc_gains, estimate_bandwidth};
use super::resistance::ResistanceMeasurement;
use super::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorParams, MotorSize, ResistanceParams,
};
use crate::foc::controller::FocOutput;
use crate::foc::transforms;
use crate::motor::ControlMode;
use crate::timer::Timer;

// ============================================================================
// Hardware Abstraction Trait
// ============================================================================

/// Hardware abstraction for motor parameter detection.
///
/// Platforms implement this trait to provide access to FOC control,
/// telemetry, and raw ADC readings needed for detection sweeps.
///
/// Async delays are provided separately via the [`Timer`] trait.
pub trait DetectionHardware {
    /// Send a control mode command to the FOC driver.
    fn send_command(&self, mode: ControlMode);

    /// Wait for the next telemetry update and return it.
    ///
    /// This should block until new telemetry is available from the FOC ISR.
    fn wait_telemetry(&mut self) -> impl Future<Output = FocOutput>;

    /// Read raw phase currents in Amps (ia, ib, ic).
    ///
    /// Used for HFI inductance measurement where we need α-β currents
    /// without going through the full FOC telemetry path.
    fn read_phase_currents(&self) -> (f32, f32, f32);
}

// ============================================================================
// Detection Parameters and Result
// ============================================================================

/// Parameters for full motor detection sequence.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DetectionParams {
    /// Motor size classification
    pub motor_size: MotorSize,
    /// Number of pole pairs (required for flux linkage)
    pub pole_pairs: u8,
    /// Maximum hardware current limit (Amps)
    pub current_max: f32,
    /// PWM frequency in Hz
    pub pwm_freq_hz: f32,
}

impl Default for DetectionParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            pole_pairs: 7,
            current_max: 10.0,
            pwm_freq_hz: 20000.0,
        }
    }
}

/// Result of full motor detection sequence.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DetectionResult {
    /// Detected motor parameters
    pub params: MotorParams,
    /// Proportional gain for current PI controller
    pub kp_current: f32,
    /// Integral gain for current PI controller
    pub ki_current: f32,
}

// ============================================================================
// Individual Measurement Functions
// ============================================================================

/// Measure motor phase resistance.
///
/// Applies DC current on d-axis and measures voltage drop.
/// Motor must be stationary (rotor locks to d-axis).
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `params` - Resistance measurement parameters
///
/// # Returns
/// * `Ok(f32)` - Measured resistance in Ohms
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_resistance<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &ResistanceParams,
) -> Result<f32, DetectionError> {
    info!("Starting resistance measurement...");

    // Calculate safe test current based on motor size
    let test_current = params.current_max / 10.0; // Start conservative
    let test_current = test_current.max(0.5).min(params.current_max);

    // Ramp up current at angle 0 (d-axis)
    let ramp_steps = 50u32;
    let ramp_delay_ms = params.ramp_time_ms / ramp_steps;

    for i in 1..=ramp_steps {
        let current = test_current * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    // Wait for settling
    T::after_millis(params.settle_time_ms as u64).await;

    // Collect samples
    let mut measurement = ResistanceMeasurement::new(params.num_samples);

    for _ in 0..params.num_samples {
        // Wait for new telemetry
        let telem = hw.wait_telemetry().await;

        // Record Vd and Id from telemetry
        // In open-loop at angle 0, vd/id are the relevant values
        measurement.record(telem.vd, telem.id);

        T::after_micros(params.sample_interval_us as u64).await;
    }

    // Ramp down and stop
    for i in (0..ramp_steps).rev() {
        let current = test_current * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    // Compute result
    let resistance = measurement.finish()?;
    info!("Resistance measurement complete");

    Ok(resistance)
}

/// Measure motor inductance using rotating HFI.
///
/// Injects a rotating high-frequency voltage vector in α-β frame
/// and analyzes current response using FFT.
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `params` - Inductance measurement parameters
/// * `pwm_freq_hz` - PWM frequency in Hz
///
/// # Returns
/// * `Ok((ld, lq))` - Measured d-axis and q-axis inductance in Henries
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_inductance<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    info!("Starting inductance measurement (rotating HFI)...");

    // Create HFI injector and measurement
    let mut injector = HfiInjector::new(params.hfi_frequency_hz, params.hfi_voltage_v, pwm_freq_hz);
    let mut measurement = InductanceMeasurement::new(params, pwm_freq_hz);

    let dt = 1.0 / pwm_freq_hz;

    // First, lock rotor at angle 0 with holding current
    let ramp_steps = 50u32;

    for i in 1..=ramp_steps {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        T::after_millis(10).await;
    }

    // Wait for rotor to settle, then capture steady-state holding voltage
    T::after_millis(params.settle_time_ms as u64).await;
    let telem = hw.wait_telemetry().await;
    let vd_hold = telem.vd;

    info!("Starting HFI injection (vd_hold={}V)...", vd_hold);

    // Switch to DirectVoltage mode — no PI interference during measurement.
    // The captured vd_hold maintains the holding force, HFI injection is added on top.
    let mut first_iteration = true;
    let mut prev_injection_angle = 0.0f32;

    while !measurement.is_complete() {
        // Wait for current PWM cycle to complete (synced to ADC ISR)
        let _telem = hw.wait_telemetry().await;

        // Read currents from THIS cycle (response to previous voltage)
        let (ia, ib, _ic) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        // Record sample (skip first iteration - no previous injection yet)
        if !first_iteration {
            measurement.record(i_alpha, i_beta, prev_injection_angle);
        }

        // Calculate and send NEXT injection command
        let injection_angle = injector.injection_angle();
        let (v_alpha_inj, v_beta_inj) = injector.step(dt);

        // At angle 0, α-β = d-q: vd_hold holds rotor, injection rides on top
        hw.send_command(ControlMode::DirectVoltage {
            vd: vd_hold + v_alpha_inj,
            vq: v_beta_inj,
            angle_rad: 0.0,
        });

        prev_injection_angle = injection_angle;
        first_iteration = false;
    }

    // Ramp down holding voltage
    info!("HFI measurement complete, ramping down...");

    for i in (0..ramp_steps).rev() {
        let vd = vd_hold * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::DirectVoltage {
            vd,
            vq: 0.0,
            angle_rad: 0.0,
        });
        T::after_millis(10).await;
    }

    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    // Compute result
    let result = measurement.finish()?;

    Ok((result.ld, result.lq))
}

/// Measure motor flux linkage via open-loop spinning.
///
/// Spins motor in open-loop mode and measures back-EMF.
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `params` - Flux linkage measurement parameters
///
/// # Returns
/// * `Ok(f32)` - Measured flux linkage in Weber
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_flux_linkage<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    info!("Starting flux linkage measurement...");

    if params.resistance_ohm <= 0.0 {
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement = FluxLinkageMeasurement::from_params(params)?;

    // Calculate target electrical angular velocity
    let target_omega_e = params.spin_rpm * core::f32::consts::TAU * params.pole_pairs as f32 / 60.0;

    // Ramp up to target speed (open-loop)
    let ramp_steps = 100u32;
    let ramp_delay_ms = params.ramp_time_ms / ramp_steps;
    let mut current_angle = 0.0f32;

    info!("Ramping up to target speed...");

    for i in 1..=ramp_steps {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;

        // Advance angle
        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
        });

        T::after_millis(ramp_delay_ms as u64).await;
    }

    // Wait for settling at target speed
    info!("Settling at target speed...");
    T::after_millis(params.settle_time_ms as u64).await;

    // Collect samples while spinning
    info!("Collecting flux linkage samples...");

    let sample_delay_us = 500u32; // 2kHz sampling
    let dt = 1.0 / 2000.0;

    for _ in 0..params.num_samples {
        // Advance angle at target speed
        current_angle += target_omega_e * dt;
        current_angle %= core::f32::consts::TAU;

        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
        });

        T::after_micros(sample_delay_us as u64).await;

        // Get telemetry
        let telem = hw.wait_telemetry().await;

        // Record Vq, Iq, and angular velocity
        measurement.record(telem.vq, telem.iq, target_omega_e);
    }

    // Ramp down and stop
    info!("Ramping down...");
    for i in (0..ramp_steps).rev() {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;

        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        let current = params.current_a * progress;
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current,
        });

        T::after_millis(ramp_delay_ms as u64).await;
    }

    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    // Compute result
    let flux_linkage = measurement.finish()?;
    info!("Flux linkage measurement complete");

    Ok(flux_linkage)
}

// ============================================================================
// Full Detection Sequence
// ============================================================================

/// Run full motor parameter detection sequence.
///
/// Performs all measurements in order:
/// 1. Resistance
/// 2. Inductance (Ld, Lq)
/// 3. Flux linkage
/// 4. PI auto-tuning
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `params` - Detection parameters
///
/// # Returns
/// * `Ok(DetectionResult)` - All detected parameters and gains
/// * `Err(DetectionError)` - If any measurement failed
pub async fn run_full_detection<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    info!("Starting full motor detection sequence");

    let mut result = DetectionResult::default();
    result.params.pole_pairs = params.pole_pairs;

    // Step 1: Measure resistance
    info!("Step 1/4: Resistance measurement");
    let resistance_params = ResistanceParams {
        motor_size: params.motor_size,
        current_max: params.current_max,
        ..Default::default()
    };
    result.params.resistance_ohm = measure_resistance::<H, T>(hw, &resistance_params).await?;

    T::after_millis(500).await;

    // Step 2: Measure inductance
    info!("Step 2/4: Inductance measurement");
    let inductance_params = InductanceParams {
        motor_size: params.motor_size,
        ..Default::default()
    };
    let (ld, lq) = measure_inductance::<H, T>(hw, &inductance_params, params.pwm_freq_hz).await?;
    result.params.inductance_d_h = ld;
    result.params.inductance_q_h = lq;
    result.params.inductance_avg_h = (ld + lq) / 2.0;
    result.params.inductance_diff_h = lq - ld;

    T::after_millis(500).await;

    // Step 3: Measure flux linkage
    info!("Step 3/4: Flux linkage measurement");
    let flux_params = FluxLinkageParams {
        motor_size: params.motor_size,
        resistance_ohm: result.params.resistance_ohm,
        pole_pairs: params.pole_pairs,
        ..Default::default()
    };
    result.params.flux_linkage_wb = measure_flux_linkage::<H, T>(hw, &flux_params).await?;
    result.params.calculate_kv();

    // Step 4: Calculate PI gains
    info!("Step 4/4: PI auto-tuning");
    let bandwidth = estimate_bandwidth(result.params.inductance_avg_h, params.pwm_freq_hz);
    if let Some(gains) = calculate_foc_gains(&result.params, bandwidth) {
        // Use average of d/q gains for simplicity
        result.kp_current = (gains.kp_d + gains.kp_q) / 2.0;
        result.ki_current = (gains.ki_d + gains.ki_q) / 2.0;
    }

    // Calculate max current
    result.params.calculate_max_current(params.motor_size);

    info!("Detection complete!");

    Ok(result)
}

// ============================================================================
// Hall Sensor Calibration
// ============================================================================

/// Hall sensor reader trait for calibration.
///
/// This is re-exported from hall_calibration for convenience.
pub use crate::foc::hall_calibration::HallReader;

use crate::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult, HallCalibrator};

/// Run Hall sensor calibration sweep.
///
/// Sweeps motor through electrical angles while recording Hall sensor states.
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `hall_reader` - Hall sensor reader implementation
/// * `params` - Calibration parameters
///
/// # Returns
/// * `Ok(HallCalibrationResult)` - Calibration result with angle mappings
/// * `Err(DetectionError)` - If calibration failed
pub async fn calibrate_hall<H: DetectionHardware, T: Timer, R: HallReader>(
    hw: &mut H,
    hall_reader: &R,
    params: HallCalibrationParams,
) -> Result<HallCalibrationResult, DetectionError> {
    let mut calibrator = HallCalibrator::new();

    info!("Starting Hall calibration...");

    // Step 1: Ramp up current at angle 0 to lock rotor
    let ramp_steps = 100u32;
    let ramp_delay_ms = params.ramp_time_ms / ramp_steps;

    for i in 1..=ramp_steps {
        let current = params.current_amps * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    // Hold at full current briefly to let rotor settle
    T::after_millis(200).await;

    // Step 2: Perform sweeps
    let degrees_per_sweep = 360u32;

    for sweep in 0..params.sweep_count {
        let forward = sweep % 2 == 0;

        for deg in 0..degrees_per_sweep {
            let actual_deg = if forward {
                deg
            } else {
                degrees_per_sweep - 1 - deg
            };
            let angle_rad = actual_deg as f32 * core::f32::consts::TAU / 360.0;

            // Command motor to this angle
            hw.send_command(ControlMode::OpenLoop {
                angle_rad,
                current: params.current_amps,
            });

            // Wait for rotor to settle
            T::after_micros(params.step_delay_us as u64).await;

            // Read and record Hall state
            let hall_state = hall_reader.read_hall_state();
            calibrator.record(angle_rad, hall_state);
        }
    }

    // Step 3: Ramp down and stop
    for i in (0..ramp_steps).rev() {
        let current = params.current_amps * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    // Stop motor
    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    // Step 4: Compute result
    info!("Computing calibration result...");
    let result = calibrator
        .finish()
        .map_err(|_| DetectionError::LowConfidence)?;

    if result.is_valid() {
        info!("Hall calibration successful!");
    }

    Ok(result)
}

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

use super::flux_linkage::{
    FluxLinkageMeasurement, MagnitudeFluxMeasurement, SpinDownFluxMeasurement,
};
use super::inductance::{HfiInjector, InductanceMeasurement, validate_inductance};
use super::pi_tuning::{calculate_foc_gains, estimate_bandwidth};
use super::resistance::ResistanceMeasurement;
use super::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorParams, MotorSize, ResistanceParams,
    VoltagePulseParams,
};
use super::voltage_pulse::VoltagePulseMeasurement;
use crate::foc::controller::FocOutput;
use crate::foc::transforms;
use crate::foc::trig::SinCos;
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

    /// Read coast-down telemetry: back-EMF voltages and angular velocity.
    ///
    /// Returns `(v_alpha, v_beta, omega_e)` where:
    /// - `v_alpha`, `v_beta` are open-circuit back-EMF in the αβ frame (V)
    /// - `omega_e` is electrical angular velocity (rad/s)
    ///
    /// Called during spin-down flux linkage measurement when all FETs are
    /// off.  On real hardware: ADC reads phase voltage dividers, Hall or
    /// observer provides ωe.  Default returns zeros (triggers fallback to
    /// driven measurement).
    fn read_coast_telemetry(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }
}

// ============================================================================
// Detection Parameters and Result
// ============================================================================

/// Parameters for full motor detection sequence.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DetectionParams {
    /// Motor size classification (used for validation ranges).
    /// Set to `MotorSize::Custom(max_power_loss_w)` when the power
    /// limit comes from a host command instead of a hardcoded preset.
    pub motor_size: MotorSize,
    /// Number of pole pairs (required for flux linkage)
    pub pole_pairs: u8,
    /// Maximum hardware current limit (Amps)
    pub current_max: f32,
    /// Maximum acceptable power dissipation in the motor during
    /// detection (Watts).  Controls the safe test current.
    pub max_power_loss_w: f32,
    /// PWM frequency in Hz
    pub pwm_freq_hz: f32,
    /// DC bus voltage (Volts) — used for voltage pulse fallback
    pub vbus: f32,
    /// Open-loop ERPM for flux linkage spin-up.
    /// Converted to mechanical RPM: `spin_rpm = openloop_erpm / pole_pairs`.
    /// When 0, uses the motor_size default.
    pub openloop_erpm: f32,
}

impl Default for DetectionParams {
    fn default() -> Self {
        Self {
            motor_size: MotorSize::Medium,
            pole_pairs: 7,
            current_max: 10.0,
            max_power_loss_w: 120.0,
            pwm_freq_hz: 20000.0,
            vbus: 24.0,
            openloop_erpm: 0.0,
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
            velocity_rad_s: 0.0,
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
            velocity_rad_s: 0.0,
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
pub async fn measure_inductance<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    info!("Starting inductance measurement (rotating HFI)...");

    // Create HFI injector and measurement
    let mut injector =
        HfiInjector::<S>::new(params.hfi_frequency_hz, params.hfi_voltage_v, pwm_freq_hz);
    let mut measurement = InductanceMeasurement::<S>::new(params, pwm_freq_hz);

    let dt = 1.0 / pwm_freq_hz;

    // First, lock rotor at angle 0 with holding current
    let ramp_steps = 50u32;

    for i in 1..=ramp_steps {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
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
    let mut prev_v_alpha_inj = 0.0f32;
    let mut prev_v_beta_inj = 0.0f32;

    while !measurement.is_complete() {
        // Wait for current PWM cycle to complete (synced to ADC ISR)
        let _telem = hw.wait_telemetry().await;

        // Read currents from THIS cycle (response to previous voltage)
        let (ia, ib, _ic) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        // Record sample with the injection voltage that caused this current
        if !first_iteration {
            measurement.record(
                i_alpha,
                i_beta,
                prev_injection_angle,
                prev_v_alpha_inj,
                prev_v_beta_inj,
            );
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
        prev_v_alpha_inj = v_alpha_inj;
        prev_v_beta_inj = v_beta_inj;
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

/// Measure inductance via voltage pulse (di/dt).
///
/// Locks the rotor at angle 0 (d-axis), applies a voltage step, measures
/// the current change over one PWM period, then repeats at angle π/2
/// (q-axis).  Works reliably on high-resistance motors where HFI fails.
///
/// Requires previously measured resistance for compensation.
pub async fn measure_inductance_pulse<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: &VoltagePulseParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    info!("Starting voltage-pulse inductance measurement...");

    let ramp_steps = 50u32;
    let mut results = [(0.0f32, 0.0f32); 2]; // (angle, measured_L)
    let angles = [0.0f32, core::f32::consts::FRAC_PI_2];

    for (axis, &angle) in angles.iter().enumerate() {
        // Lock rotor at this angle
        for i in 1..=ramp_steps {
            let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
            hw.send_command(ControlMode::OpenLoop {
                angle_rad: angle,
                current,
                velocity_rad_s: 0.0,
            });
            T::after_millis(10).await;
        }
        T::after_millis(params.settle_time_ms as u64).await;

        // Capture steady-state holding voltage
        let telem = hw.wait_telemetry().await;
        let vd_hold = telem.vd;

        // Pulse measurement
        let mut meas = VoltagePulseMeasurement::new(params, pwm_freq_hz);

        for _ in 0..params.num_pulses * 2 {
            // guard against skipped pulses
            if meas.is_complete() {
                break;
            }

            // Read current before pulse
            let (ia, ib, _) = hw.read_phase_currents();
            let (i_alpha, i_beta) = transforms::clarke(ia, ib);
            let (sin_a, cos_a) = S::sin_cos(angle);
            let id_before = i_alpha * cos_a + i_beta * sin_a;

            // Apply voltage step
            hw.send_command(ControlMode::DirectVoltage {
                vd: vd_hold + params.pulse_voltage_v,
                vq: 0.0,
                angle_rad: angle,
            });
            hw.wait_telemetry().await; // one PWM period

            // Read current after pulse
            let (ia, ib, _) = hw.read_phase_currents();
            let (i_alpha, i_beta) = transforms::clarke(ia, ib);
            let id_after = i_alpha * cos_a + i_beta * sin_a;

            meas.record_pulse(id_before, id_after);

            // Restore holding voltage and wait one cycle
            hw.send_command(ControlMode::DirectVoltage {
                vd: vd_hold,
                vq: 0.0,
                angle_rad: angle,
            });
            hw.wait_telemetry().await;
        }

        results[axis] = (angle, meas.finish()?);

        // Ramp down
        for i in (0..ramp_steps).rev() {
            let vd = vd_hold * (i as f32 / ramp_steps as f32);
            hw.send_command(ControlMode::DirectVoltage {
                vd,
                vq: 0.0,
                angle_rad: angle,
            });
            T::after_millis(10).await;
        }
        hw.send_command(ControlMode::Stopped);
        T::after_millis(200).await;
    }

    let ld = results[0].1; // angle 0 = d-axis
    let lq = results[1].1; // angle π/2 = q-axis
    info!("Voltage-pulse inductance measurement complete");
    Ok((ld, lq))
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
            velocity_rad_s: 0.0,
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
            velocity_rad_s: 0.0,
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
            velocity_rad_s: 0.0,
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

/// Measure flux linkage using magnitude-based VESC formula.
///
/// Same open-loop spin procedure as [`measure_flux_linkage`] but uses
/// voltage and current **magnitudes** instead of q-axis components:
///
///   `λ = (|V| − R·|I|) / ωe − |I|·L`
///
/// This is rotation-invariant — angle tracking lag does not affect the
/// result.  Requires both R and L from earlier detection steps.
pub async fn measure_flux_linkage_magnitude<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
    inductance_h: f32,
) -> Result<f32, DetectionError> {
    info!("Starting magnitude-based flux linkage measurement...");

    if params.resistance_ohm <= 0.0 {
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement =
        MagnitudeFluxMeasurement::new(params.resistance_ohm, inductance_h, params.num_samples);

    let target_omega_e = params.spin_rpm * core::f32::consts::TAU * params.pole_pairs as f32 / 60.0;

    // Ramp up (identical to driven method)
    let ramp_steps = 100u32;
    let ramp_delay_ms = params.ramp_time_ms / ramp_steps;
    let mut current_angle = 0.0f32;

    for i in 1..=ramp_steps {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;
        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
            velocity_rad_s: 0.0,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    T::after_millis(params.settle_time_ms as u64).await;

    // Collect samples — record all 4 dq components
    let sample_delay_us = 500u32;
    let dt = 1.0 / 2000.0;

    for _ in 0..params.num_samples {
        current_angle += target_omega_e * dt;
        current_angle %= core::f32::consts::TAU;

        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
            velocity_rad_s: 0.0,
        });
        T::after_micros(sample_delay_us as u64).await;

        let telem = hw.wait_telemetry().await;
        measurement.record(telem.vd, telem.vq, telem.id, telem.iq, target_omega_e);
    }

    // Ramp down
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
            velocity_rad_s: 0.0,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    let flux = measurement.finish()?;
    info!("Magnitude-based flux linkage measurement complete");
    Ok(flux)
}

/// Measure flux linkage using spin-down (undriven) back-EMF.
///
/// Spins the motor to target speed, releases all FETs (coast), and
/// measures the open-circuit back-EMF during deceleration.
///
///   `λ = |V_bemf| / |ωe|`
///
/// This method does **not** depend on resistance or inductance.
///
/// Returns `Err(InsufficientSamples)` if the motor decelerates too
/// quickly for enough valid samples — the caller should fall back to
/// the driven [`measure_flux_linkage`] in that case.
pub async fn measure_flux_linkage_spindown<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    info!("Starting spin-down flux linkage measurement...");

    let target_omega_e = params.spin_rpm * core::f32::consts::TAU * params.pole_pairs as f32 / 60.0;

    // ── Spin-up (open-loop ramp, same as driven method) ────────────────
    let ramp_steps = 100u32;
    let ramp_delay_ms = params.ramp_time_ms / ramp_steps;
    let mut current_angle = 0.0f32;

    for i in 1..=ramp_steps {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;
        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        hw.send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
            velocity_rad_s: 0.0,
        });
        T::after_millis(ramp_delay_ms as u64).await;
    }

    // Hold at speed briefly to ensure steady state
    T::after_millis(params.settle_time_ms as u64).await;

    // ── Release: coast with all FETs off ───────────────────────────────
    hw.send_command(ControlMode::Coast);

    // Wait for currents to decay (a few L/R time constants)
    T::after_millis(20).await;

    // ── Sample back-EMF during coast-down ──────────────────────────────
    let mut measurement = SpinDownFluxMeasurement::from_params(params);

    let max_coast_samples = 10_000u32; // safety limit
    for _ in 0..max_coast_samples {
        hw.wait_telemetry().await; // advance one FOC cycle
        T::after_micros(500).await; // ~2 kHz effective sample rate

        let (v_alpha, v_beta, omega_e) = hw.read_coast_telemetry();
        let v_bemf = libm::sqrtf(v_alpha * v_alpha + v_beta * v_beta);

        if !measurement.record(v_bemf, omega_e) {
            // omega below threshold — motor has slowed too much
            break;
        }
        if measurement.has_enough_samples() {
            break;
        }
    }

    // ── Stop ───────────────────────────────────────────────────────────
    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    let flux = measurement.finish()?;
    info!("Spin-down flux linkage measurement complete");
    Ok(flux)
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
pub async fn run_full_detection<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: DetectionParams,
) -> Result<DetectionResult, DetectionError> {
    info!("Starting full motor detection sequence");

    let mut result = DetectionResult::default();
    result.params.pole_pairs = params.pole_pairs;

    // Step 1: Measure resistance with safe current finding.
    // First pass at low current to estimate R, then compute the safe
    // test current from the power limit, then full measurement.
    info!("Step 1/4: Resistance measurement");
    let probe_current = (params.current_max / 50.0).max(0.5);
    let probe_params = ResistanceParams {
        motor_size: params.motor_size,
        current_max: probe_current,
        num_samples: 20,
        ramp_time_ms: 200,
        settle_time_ms: 100,
        ..Default::default()
    };
    let r_probe = measure_resistance::<H, T>(hw, &probe_params).await?;
    T::after_millis(200).await;

    // Safe current: I = sqrt(max_power_loss / R / 1.5), capped to hardware limit
    let safe_current = libm::sqrtf(params.max_power_loss_w / r_probe / 1.5)
        .min(params.current_max)
        .max(probe_current);
    info!("Safe test current found");

    let resistance_params = ResistanceParams {
        motor_size: params.motor_size,
        current_max: safe_current,
        ..Default::default()
    };
    result.params.resistance_ohm = measure_resistance::<H, T>(hw, &resistance_params).await?;

    T::after_millis(500).await;

    // Step 2: Measure inductance (using measured R for compensation)
    info!("Step 2/4: Inductance measurement");

    // Limit holding current to both the power-safe limit and what the
    // bus can deliver (with 40% headroom for HFI/pulse voltage).
    let r = result.params.resistance_ohm;
    let max_bus_current = (params.vbus * 0.577 * 0.6) / r.max(0.001);
    let hold_current = safe_current.min(max_bus_current).max(0.1);

    let inductance_params = InductanceParams {
        motor_size: params.motor_size,
        resistance_ohm: r,
        hold_current_a: hold_current,
        ..Default::default()
    };
    let (mut ld, mut lq) =
        measure_inductance::<H, T, S>(hw, &inductance_params, params.pwm_freq_hz).await?;

    // If HFI result looks suspicious, fall back to voltage pulse method
    if validate_inductance(ld, lq).is_err() {
        info!("HFI inductance suspicious, falling back to voltage pulse");
        T::after_millis(500).await;
        let v_hold = r * hold_current;
        let v_headroom = params.vbus * 0.577 - v_hold;
        let pulse_params = VoltagePulseParams {
            hold_current_a: hold_current,
            resistance_ohm: r,
            pulse_voltage_v: v_headroom.max(0.5),
            num_pulses: 20,
            settle_time_ms: 200,
        };
        (ld, lq) =
            measure_inductance_pulse::<H, T, S>(hw, &pulse_params, params.pwm_freq_hz).await?;
    }

    result.params.inductance_d_h = ld;
    result.params.inductance_q_h = lq;
    result.params.inductance_avg_h = (ld + lq) / 2.0;
    result.params.inductance_diff_h = lq - ld;

    T::after_millis(500).await;

    // Step 3: Measure flux linkage — try spin-down (R-independent) first,
    // fall back to driven method if the motor decelerates too quickly.
    // Use openloop_erpm to set spin RPM, fall back to motor_size default
    let spin_rpm = if params.openloop_erpm > 0.0 {
        params.openloop_erpm / params.pole_pairs as f32
    } else {
        params.motor_size.suggested_open_loop_erpm() / params.pole_pairs as f32
    };
    let flux_params = FluxLinkageParams {
        motor_size: params.motor_size,
        resistance_ohm: result.params.resistance_ohm,
        pole_pairs: params.pole_pairs,
        spin_rpm,
        current_a: safe_current.min(2.0), // cap to safe level
        ..Default::default()
    };
    info!("Step 3/4: Flux linkage measurement (spin-down)");
    match measure_flux_linkage_spindown::<H, T>(hw, &flux_params).await {
        Ok(flux) => result.params.flux_linkage_wb = flux,
        Err(DetectionError::InsufficientSamples) => {
            // Motor can't coast — fall back to driven measurement.
            // Try both q-axis and magnitude methods, pick the better one.
            info!("Spin-down failed (motor stopped too fast), falling back to driven methods");
            T::after_millis(500).await;

            // Try magnitude first (angle-invariant, better on most motors).
            // Fall back to q-axis if magnitude fails (high-R motors).
            let l_avg = result.params.inductance_avg_h;
            let lam_m = measure_flux_linkage_magnitude::<H, T>(hw, &flux_params, l_avg).await;
            result.params.flux_linkage_wb = match lam_m {
                Ok(m) => {
                    info!("Using magnitude flux (angle-invariant)");
                    m
                }
                Err(_) => {
                    info!("Magnitude flux failed, falling back to q-axis");
                    T::after_millis(500).await;
                    measure_flux_linkage::<H, T>(hw, &flux_params).await?
                }
            };
        }
        Err(e) => return Err(e),
    }
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
            velocity_rad_s: 0.0,
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
                velocity_rad_s: 0.0,
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
            velocity_rad_s: 0.0,
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

//! Motor parameter detection implementation for F405
//!
//! Async sweeps for resistance, inductance, and flux linkage measurement.

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::sync::atomic::Ordering;

use embassy_time::{Duration, Timer};

use oxifoc_core::foc::detection::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorParams, MotorSize, ResistanceParams,
};
use oxifoc_core::foc::detection::flux_linkage::{FluxLinkageMeasurement, calculate_kv};
use oxifoc_core::foc::detection::inductance::{HfiInjector, InductanceMeasurement};
use oxifoc_core::foc::detection::pi_tuning::{calculate_foc_gains, estimate_bandwidth};
use oxifoc_core::foc::detection::resistance::ResistanceMeasurement;
use oxifoc_core::foc::transforms;
use oxifoc_core::motor::ControlMode;

use crate::config::BOARD;
use crate::control::foc::{send_command, FOC_TELEMETRY, IA_SAMPLE, IB_SAMPLE, IC_SAMPLE};

// ============================================================================
// Detection Parameters
// ============================================================================

/// Parameters for full motor detection
#[derive(Clone, Debug)]
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
            current_max: 20.0,
            pwm_freq_hz: 20000.0,
        }
    }
}

/// Result of full motor detection
#[derive(Clone, Debug, Default, defmt::Format)]
pub struct DetectionResult {
    /// Detected motor parameters
    pub params: MotorParams,
    /// Current PI gains
    pub kp_current: f32,
    /// Current PI integral gain
    pub ki_current: f32,
}

// ============================================================================
// Individual Measurement Functions
// ============================================================================

/// Measure motor phase resistance
///
/// Applies DC current on d-axis and measures voltage drop.
/// Motor must be stationary (rotor locks to d-axis).
///
/// # Arguments
/// * `params` - Resistance measurement parameters
///
/// # Returns
/// * `Ok(f32)` - Measured resistance in Ohms
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_resistance(params: &ResistanceParams) -> Result<f32, DetectionError> {
    defmt::info!("Starting resistance measurement...");

    // Calculate safe test current based on motor size
    let test_current = params.current_max / 10.0; // Start conservative
    let test_current = test_current.max(0.5).min(params.current_max);

    defmt::info!("Using test current: {} A", test_current);

    // Ramp up current at angle 0 (d-axis)
    let ramp_steps = 50u32;
    let ramp_delay = Duration::from_millis(params.ramp_time_ms as u64 / ramp_steps as u64);

    for i in 1..=ramp_steps {
        let current = test_current * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    // Wait for settling
    Timer::after(Duration::from_millis(params.settle_time_ms as u64)).await;

    // Collect samples
    let mut measurement = ResistanceMeasurement::new(params.num_samples);
    let sample_delay = Duration::from_micros(params.sample_interval_us as u64);

    // Get telemetry receiver
    let mut telem_rx = FOC_TELEMETRY.receiver().unwrap();

    for _ in 0..params.num_samples {
        // Wait for new telemetry
        let telem = telem_rx.changed().await;

        // Record Vd and Id from telemetry
        // In open-loop at angle 0, vd/id are the relevant values
        measurement.record(telem.vd, telem.id);

        Timer::after(sample_delay).await;
    }

    // Ramp down and stop
    for i in (0..ramp_steps).rev() {
        let current = test_current * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    send_command(ControlMode::Stopped);
    Timer::after(Duration::from_millis(100)).await;

    // Compute result
    let resistance = measurement.finish()?;
    defmt::info!("Measured resistance: {} mΩ", (resistance * 1000.0) as i32);

    Ok(resistance)
}

/// Measure motor inductance using rotating HFI
///
/// Injects a rotating high-frequency voltage vector in α-β frame
/// and analyzes current response using FFT.
///
/// # Arguments
/// * `params` - Inductance measurement parameters
/// * `pwm_freq_hz` - PWM frequency in Hz
///
/// # Returns
/// * `Ok((ld, lq))` - Measured d-axis and q-axis inductance in Henries
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_inductance(
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    defmt::info!("Starting inductance measurement (VESC-style rotating HFI)...");
    defmt::info!(
        "HFI freq: {} Hz, voltage: {} V, hold current: {} A",
        params.hfi_frequency_hz,
        params.hfi_voltage_v,
        params.hold_current_a
    );

    // Create HFI injector and measurement
    let mut injector = HfiInjector::new(
        params.hfi_frequency_hz,
        params.hfi_voltage_v,
        pwm_freq_hz,
    );
    let mut measurement = InductanceMeasurement::new(params, pwm_freq_hz);

    let dt = 1.0 / pwm_freq_hz;

    // First, lock rotor at angle 0 with holding current
    defmt::info!("Locking rotor with {} A holding current...", params.hold_current_a);

    let ramp_steps = 50u32;
    let ramp_delay = Duration::from_millis(10);

    for i in 1..=ramp_steps {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    // Wait for rotor to settle
    Timer::after(Duration::from_millis(params.settle_time_ms as u64)).await;

    defmt::info!("Starting HFI injection...");

    // Calculate timing
    // We need to sync with PWM cycles - each PWM cycle we get new ADC samples
    let pwm_period_us = (1_000_000.0 / pwm_freq_hz) as u64;
    let sample_delay = Duration::from_micros(pwm_period_us);

    // Run HFI measurement
    // For each sample, we:
    // 1. Get injection voltages from HfiInjector
    // 2. Send them to FOC (as HFI injection mode)
    // 3. Read currents from ADC
    // 4. Record in measurement

    while !measurement.is_complete() {
        // Get injection voltages (α-β frame)
        let injection_angle = injector.injection_angle();
        let (v_alpha_inj, v_beta_inj) = injector.step(dt);

        // Convert α-β injection to d-q for the current HfiInjection mode
        // Since we're locked at angle 0, the transform is simple:
        // vd = v_alpha * cos(0) + v_beta * sin(0) = v_alpha
        // vq = -v_alpha * sin(0) + v_beta * cos(0) = v_beta
        let (vd_inject, vq_inject) = (v_alpha_inj, v_beta_inj);

        // Send HFI injection command
        send_command(ControlMode::HfiInjection {
            hold_current: params.hold_current_a,
            vd_inject,
            vq_inject,
        });

        // Wait for next PWM cycle
        Timer::after(sample_delay).await;

        // Read currents (α-β frame)
        // At angle 0: i_alpha = ia, i_beta = (ia + 2*ib) / sqrt(3)
        let ia_raw = IA_SAMPLE.load(Ordering::Relaxed);
        let ib_raw = IB_SAMPLE.load(Ordering::Relaxed);
        let ic_raw = IC_SAMPLE.load(Ordering::Relaxed);

        // Convert raw ADC to currents
        let (ia, ib, _ic) = convert_raw_currents(ia_raw, ib_raw, ic_raw);

        // Clarke transform to get α-β currents
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        // Record sample
        let cycle_done = measurement.record(i_alpha, i_beta, injection_angle);

        if cycle_done {
            defmt::debug!(
                "FFT cycle {} complete",
                measurement.cycles_completed()
            );
        }
    }

    // Stop HFI injection, ramp down holding current
    defmt::info!("HFI measurement complete, ramping down...");

    for i in (0..ramp_steps).rev() {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
        });
        Timer::after(ramp_delay).await;
    }

    send_command(ControlMode::Stopped);
    Timer::after(Duration::from_millis(100)).await;

    // Compute result
    let result = measurement.finish()?;

    defmt::info!(
        "Measured inductance: Ld={} µH, Lq={} µH",
        (result.ld * 1e6) as i32,
        (result.lq * 1e6) as i32
    );

    Ok((result.ld, result.lq))
}

/// Measure motor flux linkage via open-loop spinning
///
/// Spins motor in open-loop mode and measures back-EMF.
///
/// # Arguments
/// * `params` - Flux linkage measurement parameters
///
/// # Returns
/// * `Ok(f32)` - Measured flux linkage in Weber
/// * `Err(DetectionError)` - If measurement failed
pub async fn measure_flux_linkage(params: &FluxLinkageParams) -> Result<f32, DetectionError> {
    defmt::info!("Starting flux linkage measurement...");

    if params.resistance_ohm <= 0.0 {
        defmt::error!("Resistance must be measured first!");
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement = FluxLinkageMeasurement::from_params(params)?;

    // Calculate target electrical angular velocity
    let target_omega_e = params.spin_rpm * core::f32::consts::TAU * params.pole_pairs as f32 / 60.0;

    defmt::info!(
        "Target: {} RPM mechanical, {} rad/s electrical",
        params.spin_rpm as i32,
        target_omega_e as i32
    );

    // Ramp up to target speed (open-loop)
    let ramp_steps = 100u32;
    let ramp_delay = Duration::from_millis(params.ramp_time_ms as u64 / ramp_steps as u64);
    let mut current_angle = 0.0f32;

    defmt::info!("Ramping up to target speed...");

    for i in 1..=ramp_steps {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;

        // Advance angle
        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
        });

        Timer::after(ramp_delay).await;
    }

    // Wait for settling at target speed
    defmt::info!("Settling at target speed...");
    Timer::after(Duration::from_millis(params.settle_time_ms as u64)).await;

    // Collect samples while spinning
    defmt::info!("Collecting flux linkage samples...");
    let mut telem_rx = FOC_TELEMETRY.receiver().unwrap();

    let sample_delay = Duration::from_micros(500); // 2kHz sampling
    let dt = 1.0 / 2000.0;

    for i in 0..params.num_samples {
        // Advance angle at target speed
        current_angle += target_omega_e * dt;
        current_angle %= core::f32::consts::TAU;

        send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current: params.current_a,
        });

        Timer::after(sample_delay).await;

        // Get telemetry
        let telem = telem_rx.changed().await;

        // Record Vq, Iq, and angular velocity
        measurement.record(telem.vq, telem.iq, target_omega_e);

        if i % 50 == 0 {
            if let Some(estimate) = measurement.current_estimate() {
                defmt::debug!(
                    "Flux estimate at {}: {} mWb",
                    i,
                    (estimate * 1000.0) as i32
                );
            }
        }
    }

    // Ramp down and stop
    defmt::info!("Ramping down...");
    for i in (0..ramp_steps).rev() {
        let progress = i as f32 / ramp_steps as f32;
        let omega = target_omega_e * progress;

        let dt = params.ramp_time_ms as f32 / 1000.0 / ramp_steps as f32;
        current_angle += omega * dt;
        current_angle %= core::f32::consts::TAU;

        let current = params.current_a * progress;
        send_command(ControlMode::OpenLoop {
            angle_rad: current_angle,
            current,
        });

        Timer::after(ramp_delay).await;
    }

    send_command(ControlMode::Stopped);
    Timer::after(Duration::from_millis(100)).await;

    // Compute result
    let flux_linkage = measurement.finish()?;

    let kv = calculate_kv(flux_linkage, params.pole_pairs);
    defmt::info!(
        "Measured flux linkage: {} mWb, Kv: {} RPM/V",
        (flux_linkage * 1000.0) as i32,
        kv as i32
    );

    Ok(flux_linkage)
}

// ============================================================================
// Full Detection Sequence
// ============================================================================

/// Run full motor parameter detection sequence
///
/// Performs all measurements in order:
/// 1. Resistance
/// 2. Inductance (Ld, Lq)
/// 3. Flux linkage
/// 4. PI auto-tuning
///
/// # Arguments
/// * `params` - Detection parameters
///
/// # Returns
/// * `Ok(DetectionResult)` - All detected parameters and gains
/// * `Err(DetectionError)` - If any measurement failed
pub async fn run_full_detection(params: DetectionParams) -> Result<DetectionResult, DetectionError> {
    defmt::info!("========================================");
    defmt::info!("Starting full motor detection sequence");
    defmt::info!("========================================");

    let mut result = DetectionResult::default();
    result.params.pole_pairs = params.pole_pairs;

    // Step 1: Measure resistance
    defmt::info!("Step 1/4: Resistance measurement");
    let resistance_params = ResistanceParams {
        motor_size: params.motor_size,
        current_max: params.current_max,
        ..Default::default()
    };
    result.params.resistance_ohm = measure_resistance(&resistance_params).await?;

    Timer::after(Duration::from_millis(500)).await;

    // Step 2: Measure inductance
    defmt::info!("Step 2/4: Inductance measurement");
    let inductance_params = InductanceParams {
        motor_size: params.motor_size,
        ..Default::default()
    };
    let (ld, lq) = measure_inductance(&inductance_params, params.pwm_freq_hz).await?;
    result.params.inductance_d_h = ld;
    result.params.inductance_q_h = lq;
    result.params.inductance_avg_h = (ld + lq) / 2.0;
    result.params.inductance_diff_h = lq - ld;

    Timer::after(Duration::from_millis(500)).await;

    // Step 3: Measure flux linkage
    defmt::info!("Step 3/4: Flux linkage measurement");
    let flux_params = FluxLinkageParams {
        motor_size: params.motor_size,
        resistance_ohm: result.params.resistance_ohm,
        pole_pairs: params.pole_pairs,
        ..Default::default()
    };
    result.params.flux_linkage_wb = measure_flux_linkage(&flux_params).await?;
    result.params.calculate_kv();

    // Step 4: Calculate PI gains
    defmt::info!("Step 4/4: PI auto-tuning");
    let bandwidth = estimate_bandwidth(result.params.inductance_avg_h, params.pwm_freq_hz);
    if let Some(gains) = calculate_foc_gains(&result.params, bandwidth) {
        // Use average of d/q gains for simplicity
        result.kp_current = (gains.kp_d + gains.kp_q) / 2.0;
        result.ki_current = (gains.ki_d + gains.ki_q) / 2.0;
    } else {
        defmt::warn!("Could not calculate PI gains from motor parameters");
    }

    // Calculate max current
    result.params.calculate_max_current(params.motor_size);

    defmt::info!("========================================");
    defmt::info!("Detection complete!");
    defmt::info!("  R = {} mΩ", (result.params.resistance_ohm * 1000.0) as i32);
    defmt::info!("  Ld = {} µH", (result.params.inductance_d_h * 1e6) as i32);
    defmt::info!("  Lq = {} µH", (result.params.inductance_q_h * 1e6) as i32);
    defmt::info!("  λ = {} mWb", (result.params.flux_linkage_wb * 1000.0) as i32);
    defmt::info!("  Kv = {} RPM/V", result.params.kv_rpm_per_v as i32);
    defmt::info!("  Kp = {}, Ki = {}", result.kp_current, result.ki_current);
    defmt::info!("========================================");

    Ok(result)
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert raw ADC values to currents in Amps
///
/// Uses board configuration for conversion.
fn convert_raw_currents(raw_a: u16, raw_b: u16, raw_c: u16) -> (f32, f32, f32) {
    // Get offsets from current sensor (stored during calibration)
    // For now, use mid-scale as default offset
    let offset = BOARD.adc_max_counts as f32 / 2.0;

    let scale = BOARD.adc_vref_mv as f32 / 1000.0 / BOARD.adc_max_counts as f32
        / BOARD.shunt_ohms
        / BOARD.amp_gain;

    let ia = (raw_a as f32 - offset) * scale;
    let ib = (raw_b as f32 - offset) * scale;
    let ic = (raw_c as f32 - offset) * scale;

    (ia, ib, ic)
}

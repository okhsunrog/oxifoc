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

/// Conservative PI gains for detection (VESC-style).
/// Motor parameters are unknown at detection time, so these must be safe
/// for any motor. Kp=0.01, Ki=10.0 (scaled for 20kHz loop).
pub const DETECTION_PI_KP: f32 = 0.01;
pub const DETECTION_PI_KI: f32 = 10.0;

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

    // 2-point differential measurement (MESC-style):
    // Measure Vd/Id at two steady-state current levels, compute R = ΔV/ΔI.
    // This eliminates offset errors and inductance contamination (dI/dt=0 at SS).
    let i_high = params.current_max.max(0.5);
    let i_low = i_high * 0.2;
    let settle_cycles = 1000_u64; // 1s settle — ensure PI fully converges and dI/dt→0
    let sample_count = 2000_u32; // Average over 2000 FOC cycles (~100ms at 20kHz)
    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));

    debug!(
        "R meas: i_low={}, i_high={}, settle={}ms, samples={}",
        i_low, i_high, settle_cycles, sample_count
    );

    // --- Ramp to low setpoint ---
    // First command carries PI gains override; subsequent commands use None
    // since gains persist until explicitly changed.
    let ramp_steps = 50u32;
    for i in 1..=ramp_steps {
        let current = i_low * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: if i == 1 { det_gains } else { None },
        });
        T::after_millis(4).await;
    }
    T::after_millis(settle_cycles).await;

    // Sample at low setpoint
    let mut vd_low_sum = 0.0f32;
    let mut id_low_sum = 0.0f32;
    for _ in 0..sample_count {
        let t = hw.wait_telemetry().await;
        vd_low_sum += t.vd;
        id_low_sum += t.id;
    }
    let vd_low = vd_low_sum / sample_count as f32;
    let id_low = id_low_sum / sample_count as f32;
    debug!("R meas: low point: vd={}, id={}", vd_low, id_low);

    // --- Ramp to high setpoint ---
    for i in 1..=ramp_steps {
        let current = i_low + (i_high - i_low) * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: None,
        });
        T::after_millis(4).await;
    }
    T::after_millis(settle_cycles).await;

    // Sample at high setpoint
    let mut vd_high_sum = 0.0f32;
    let mut id_high_sum = 0.0f32;
    for _ in 0..sample_count {
        let t = hw.wait_telemetry().await;
        vd_high_sum += t.vd;
        id_high_sum += t.id;
    }
    let vd_high = vd_high_sum / sample_count as f32;
    let id_high = id_high_sum / sample_count as f32;
    debug!("R meas: high point: vd={}, id={}", vd_high, id_high);

    // --- Ramp down and stop ---
    for i in (0..ramp_steps).rev() {
        let current = i_high * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: None,
        });
        T::after_millis(4).await;
    }
    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;

    // --- Compute R = ΔV / ΔI ---
    let delta_i = id_high - id_low;
    let delta_v = vd_high - vd_low;

    debug!("R meas: dV={}, dI={}", delta_v, delta_i);

    if delta_i.abs() < 0.1 {
        return Err(DetectionError::MotorNotResponding);
    }

    let resistance = (delta_v / delta_i).abs();

    if resistance < 0.001 {
        return Err(DetectionError::OutOfRange);
    }
    if resistance > 100.0 {
        return Err(DetectionError::MotorNotResponding);
    }

    info!(
        "Resistance: {} Ohm (dV={}, dI={})",
        resistance, delta_v, delta_i
    );

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
    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));

    for i in 1..=ramp_steps {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: if i == 1 { det_gains } else { None },
        });
        T::after_millis(10).await;
    }

    // Wait for rotor to settle, then compute holding voltage from R×I
    // (Don't use PI output — it includes dead time compensation voltage)
    T::after_millis(params.settle_time_ms as u64).await;
    let vd_hold = params.resistance_ohm * params.hold_current_a;

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
    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));
    let mut first_cmd = true;

    for (axis, &angle) in angles.iter().enumerate() {
        // Lock rotor at this angle
        for i in 1..=ramp_steps {
            let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
            hw.send_command(ControlMode::OpenLoop {
                angle_rad: angle,
                current,
                velocity_rad_s: 0.0,
                pi_gains: if first_cmd {
                    first_cmd = false;
                    det_gains
                } else {
                    None
                },
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

/// Maximum electrical angular velocity for open-loop spin-up ramps,
/// independent of the (mechanical) `spin_rpm` cap. Mirrors VESC's
/// 12000 ERPM ceiling in the flux-linkage wizard.
const SPINUP_MAX_OMEGA_E: f32 = 12_000.0 * core::f32::consts::TAU / 60.0;

/// Fraction of the running |V| maximum below which the rotor is considered
/// desynchronized during spin-up. A synced rotor contributes ω·λ of
/// back-EMF; on sync loss that contribution disappears and |V| collapses.
/// Same criterion as VESC (`duty_now < duty_max * 0.7`).
const SPINUP_SYNC_LOSS_RATIO: f32 = 0.7;

/// Lock the rotor, then spin it up in open loop and return the electrical
/// angular velocity the firmware is left integrating at (rad/s).
///
/// Uses `OpenLoop { velocity_rad_s != 0 }`, where the *firmware* advances
/// the angle every FOC cycle (`FocDriver::step_open_loop`) — the host only
/// ramps the velocity setpoint. The previous approach stepped the angle
/// from this async task, which at speed meant near-π jumps per command
/// that a real rotor cannot follow (it only ever worked against the
/// simulator, which smoothed the steps).
///
/// The ramp runs until one of (VESC conf_general flux wizard behavior):
/// * `|V| ≥ params.v_target` (if nonzero) — fast enough that back-EMF
///   dominates the resistive drop;
/// * the `spin_rpm` / [`SPINUP_MAX_OMEGA_E`] speed cap is reached;
/// * |V| collapses below [`SPINUP_SYNC_LOSS_RATIO`] of its running max
///   after the early ramp → `Err(MotorNotResponding)`.
async fn spin_up_open_loop<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    let omega_cap = (params.spin_rpm * core::f32::consts::TAU * params.pole_pairs as f32 / 60.0)
        .min(SPINUP_MAX_OMEGA_E);

    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));

    // ── Capture: bring the current up on a slowly creeping frame ──────
    // Locking with d-axis current (velocity 0) and then starting the ramp
    // would jump the current vector 90° to the q axis in one FOC cycle,
    // kicking the rotor into a poorly damped swing that corrupts the first
    // seconds of |V|. Instead the current grows from zero with the command
    // frame already advancing slowly, so the rotor is captured gently —
    // the same effect as VESC's lock via set_openloop_current.
    info!("Capturing rotor...");
    const CAPTURE_OMEGA_E: f32 = core::f32::consts::TAU; // 1 elec rev/s
    const CAPTURE_STEPS: u32 = 20;
    const CAPTURE_TIME_MS: u64 = 400;
    for i in 1..=CAPTURE_STEPS {
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0, // ignored: velocity mode
            current: params.current_a * i as f32 / CAPTURE_STEPS as f32,
            velocity_rad_s: CAPTURE_OMEGA_E,
            pi_gains: if i == 1 { det_gains } else { None },
        });
        T::after_millis(CAPTURE_TIME_MS / CAPTURE_STEPS as u64).await;
    }

    // Resistive |V| baseline at near-zero speed. The v_target criterion
    // must measure the back-EMF *rise* above this: for high-R motors the
    // R·I drop alone can exceed any absolute voltage target (e.g. a gimbal
    // motor at 8 Ω × 1.3 A = 10 V on a 12 V bus), which would end the ramp
    // on its first step.
    let mut v_baseline = 0.0f32;
    const BASELINE_SAMPLES: u32 = 10;
    for _ in 0..BASELINE_SAMPLES {
        let telem = hw.wait_telemetry().await;
        v_baseline += libm::sqrtf(telem.vd * telem.vd + telem.vq * telem.vq);
        T::after_micros(500).await;
    }
    v_baseline /= BASELINE_SAMPLES as f32;

    info!("Ramping up (velocity mode)...");
    let ramp_steps = 100u32;
    let step_ms = (params.ramp_time_ms / ramp_steps).max(1) as u64;
    // Low-passed |V| for the sync check: rotor swing after disturbances
    // modulates the back-EMF at a few Hz, and small motors run this whole
    // ramp at well under a volt — raw samples would trip the threshold on
    // ripple alone.
    let mut v_filt = 0.0f32;
    let mut v_filt_max = 0.0f32;
    // Below this |V| the sync check is meaningless: nothing but resistive
    // drop and measurement noise. R may be unknown (spin-down path), hence
    // the absolute floor.
    let v_check_floor = (3.0 * params.resistance_ohm * params.current_a).max(0.25);
    let mut omega = CAPTURE_OMEGA_E;

    for i in 1..=ramp_steps {
        omega = CAPTURE_OMEGA_E + (omega_cap - CAPTURE_OMEGA_E) * i as f32 / ramp_steps as f32;
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0, // ignored: firmware integrates velocity
            current: params.current_a,
            velocity_rad_s: omega,
            pi_gains: None,
        });
        T::after_millis(step_ms).await;

        let telem = hw.wait_telemetry().await;
        let v_mag = libm::sqrtf(telem.vd * telem.vd + telem.vq * telem.vq);
        v_filt = if i == 1 {
            v_mag
        } else {
            0.85 * v_filt + 0.15 * v_mag
        };
        v_filt_max = v_filt_max.max(v_filt);

        // Sync loss: the back-EMF contribution vanished (VESC checks
        // duty_now < 0.7 × duty_max the same way). Only meaningful once
        // |V| has risen clear of the resistive-drop floor and past the
        // early ramp transients.
        if i > ramp_steps / 2
            && v_filt_max > v_check_floor
            && v_filt < SPINUP_SYNC_LOSS_RATIO * v_filt_max
        {
            hw.send_command(ControlMode::Stopped);
            return Err(DetectionError::MotorNotResponding);
        }

        // Back-EMF rise above the resistive baseline reached the target —
        // fast enough for the flux formulas.
        if params.v_target > 0.0 && v_filt - v_baseline >= params.v_target {
            break;
        }
    }

    Ok(omega)
}

/// Ramp the open-loop velocity (and current) back to zero, then stop.
async fn ramp_down_open_loop<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    current_a: f32,
    omega_e: f32,
    ramp_time_ms: u32,
) {
    let ramp_steps = 50u32;
    let step_ms = (ramp_time_ms / ramp_steps).max(1) as u64;
    for i in (0..ramp_steps).rev() {
        let progress = i as f32 / ramp_steps as f32;
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current: current_a * progress,
            velocity_rad_s: omega_e * progress,
            pi_gains: None,
        });
        T::after_millis(step_ms).await;
    }
    hw.send_command(ControlMode::Stopped);
    T::after_millis(100).await;
}

/// Measure motor flux linkage via open-loop spinning (q-axis components).
///
/// `λ = (Vq − R·Iq) / ωe` in the **command** frame.
///
/// # Accuracy warning
///
/// In open-loop drive the rotor's d axis pulls onto the current vector, so
/// the rotor leads the command frame by up to 90° and the back-EMF is not
/// aligned with the command q axis — this method underestimates λ by the
/// load-angle cosine. It is kept for comparison/diagnostics;
/// [`measure_flux_linkage_magnitude`] (back-EMF vector, load-angle
/// invariant) is what [`run_full_detection`] uses as the driven fallback.
pub async fn measure_flux_linkage<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    info!("Starting flux linkage measurement (q-axis)...");

    if params.resistance_ohm <= 0.0 {
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement = FluxLinkageMeasurement::from_params(params)?;

    let omega_e = spin_up_open_loop::<H, T>(hw, params).await?;
    T::after_millis(params.settle_time_ms as u64).await;

    // The firmware integrates the angle at the FOC rate, so the actual
    // electrical speed IS the commanded one (synchronous machine; sync
    // loss is detected during the ramp).
    info!("Collecting flux linkage samples...");
    for _ in 0..params.num_samples {
        T::after_micros(500).await; // ~2 kHz sampling
        let telem = hw.wait_telemetry().await;
        measurement.record(telem.vq, telem.iq, omega_e);
    }

    ramp_down_open_loop::<H, T>(hw, params.current_a, omega_e, params.ramp_time_ms).await;

    let flux_linkage = measurement.finish()?;
    info!("Flux linkage measurement complete");
    Ok(flux_linkage)
}

/// Measure flux linkage via the back-EMF vector (driven, load-angle
/// invariant).
///
/// Same open-loop spin as [`measure_flux_linkage`], but solves the full
/// steady-state dq equations for the back-EMF vector:
///
///   `e⃗ = V⃗ − R·i⃗ − jωL·i⃗`,  `λ = |e⃗| / ωe`
///
/// Exact at steady state for any load angle (see
/// [`MagnitudeFluxMeasurement`]), unlike both the q-axis method and
/// VESC's scalar `(|V| − R|I|)/ω − |I|L` approximation. Requires both R
/// and L from earlier detection steps.
pub async fn measure_flux_linkage_magnitude<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
    inductance_h: f32,
) -> Result<f32, DetectionError> {
    info!("Starting back-EMF-vector flux linkage measurement...");

    if params.resistance_ohm <= 0.0 {
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement =
        MagnitudeFluxMeasurement::new(params.resistance_ohm, inductance_h, params.num_samples);

    let omega_e = spin_up_open_loop::<H, T>(hw, params).await?;
    T::after_millis(params.settle_time_ms as u64).await;

    info!("Collecting flux linkage samples...");
    for _ in 0..params.num_samples {
        T::after_micros(500).await; // ~2 kHz sampling
        let telem = hw.wait_telemetry().await;
        measurement.record(telem.vd, telem.vq, telem.id, telem.iq, omega_e);
    }

    ramp_down_open_loop::<H, T>(hw, params.current_a, omega_e, params.ramp_time_ms).await;

    let flux = measurement.finish()?;
    info!("Back-EMF-vector flux linkage measurement complete");
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

    // ── Spin-up (shared open-loop ramp, firmware-integrated angle) ─────
    let _omega_e = spin_up_open_loop::<H, T>(hw, params).await?;

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
        // Ramp until the phase voltage reaches ~20% of vbus (VESC spins its
        // flux wizard to duty 0.3 ≈ the same), so back-EMF dominates R·I.
        v_target: 0.2 * params.vbus,
        ..Default::default()
    };
    info!("Step 3/4: Flux linkage measurement (spin-down)");
    match measure_flux_linkage_spindown::<H, T>(hw, &flux_params).await {
        Ok(flux) => result.params.flux_linkage_wb = flux,
        Err(DetectionError::InsufficientSamples) => {
            // Motor can't coast (high friction / geared) — fall back to the
            // driven back-EMF-vector method. The q-axis method is NOT used
            // here: in open loop the rotor leads the command frame by up to
            // 90°, which biases it by the load-angle cosine.
            info!("Spin-down failed (motor stopped too fast), falling back to driven method");
            T::after_millis(500).await;

            let l_avg = result.params.inductance_avg_h;
            result.params.flux_linkage_wb =
                measure_flux_linkage_magnitude::<H, T>(hw, &flux_params, l_avg).await?;
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
    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));

    for i in 1..=ramp_steps {
        let current = params.current_amps * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: if i == 1 { det_gains } else { None },
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
                pi_gains: None,
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
            pi_gains: None,
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

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
#[cfg(feature = "hfi-detect")]
use super::inductance::{HfiInjector, InductanceMeasurement, validate_inductance};
use super::pi_tuning::{calculate_foc_gains, estimate_bandwidth};
use super::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorParams, MotorSize, ResistanceParams,
    VoltagePulseParams,
};
use super::voltage_pulse::VoltagePulseMeasurement;
use crate::foc::clamp_f32;
use crate::foc::controller::FocOutput;
use crate::foc::fast_math::sqrtf;
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
    ///
    /// Must guarantee delivery (await channel space if needed): a silently
    /// dropped command mid-sweep desynchronizes the measurement — e.g. the
    /// HFI loop pairs each recorded current with the voltage it commanded
    /// one cycle earlier, and a dropped `DirectVoltage` corrupts that
    /// pairing with no trace. The FOC ISR drains the command channel every
    /// cycle (50 µs), so awaiting is cheap and bounded.
    fn send_command(&self, mode: ControlMode) -> impl Future<Output = ()>;

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
    /// observer provides ωe.  Default returns zeros.
    fn read_coast_telemetry(&self) -> (f32, f32, f32) {
        (0.0, 0.0, 0.0)
    }

    /// Whether [`read_coast_telemetry`](Self::read_coast_telemetry) is
    /// actually wired up. The default is `false` — `run_full_detection`
    /// then skips the spin-down flux method entirely (honestly, with a log)
    /// instead of spinning the motor up, coasting, reading zeros and
    /// "falling back" as if the rotor had stopped too fast.
    fn supports_coast_telemetry(&self) -> bool {
        false
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

/// Average `vd`/`id` over up to `count` telemetry frames, bounded by a
/// deadline.
///
/// The deadline is a sampling-window cap, not a health check per se: under a
/// concurrent full-rate telemetry stream the executor schedules this task far
/// below the FOC rate (measured ~750 frames/s on g431 at 20 kHz streaming),
/// so the full `count` may not fit in the window. Each frame is an
/// independent snapshot of a DC steady state, so a *partial* average over
/// enough frames is just as unbiased — on deadline we return it as long as a
/// statistical floor was reached.
///
/// If the FOC ISR stops producing telemetry mid-detection, an unbounded
/// sample loop would await a dead channel forever (the ISR is IWDG-covered,
/// but on hosts/tests there is no watchdog at all). A silent control loop
/// collects ~nothing by the deadline and still maps to `MotorNotResponding`.
async fn sample_vd_id<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    count: u32,
    timeout_ms: u64,
) -> Result<(f32, f32), DetectionError> {
    use core::cell::Cell;
    use embassy_futures::select::{Either, select};
    // Cells, not locals in the async block: on deadline the accumulated sums
    // must survive the sample future being dropped by `select`.
    let got = Cell::new(0u32);
    let vd_sum = Cell::new(0.0f32);
    let id_sum = Cell::new(0.0f32);
    let sample = async {
        for _ in 0..count {
            let t = hw.wait_telemetry().await;
            vd_sum.set(vd_sum.get() + t.vd);
            id_sum.set(id_sum.get() + t.id);
            got.set(got.get() + 1);
        }
    };
    let n = match select(sample, T::after_millis(timeout_ms)).await {
        Either::First(()) => count,
        Either::Second(()) => {
            // Enough frames for a trustworthy DC average despite the timeout?
            let floor = (count / 8).max(64).min(count);
            let n = got.get();
            if n >= floor {
                info!(
                    "telemetry sampling window closed early: averaging {}/{} frames",
                    n, count
                );
                n
            } else {
                warn!(
                    "telemetry sampling starved: {}/{} frames in {} ms (floor {})",
                    n, count, timeout_ms, floor
                );
                return Err(DetectionError::MotorNotResponding);
            }
        }
    };
    Ok((vd_sum.get() / n as f32, id_sum.get() / n as f32))
}

/// Settled DirectVoltage holding voltage for `hold_current_a` at the
/// current rotor lock.
///
/// Computed `R·I` plus the *measured* make-up voltage `avg_vd − R·avg_id`
/// over a short telemetry window. With a converged PI this reduces to the
/// averaged PI output; with a PI still converging (high-R motors on the
/// soft detection gains) it degrades gracefully to `R·I`. In both cases it
/// carries whatever voltage the bridge loses to uncompensated dead-time
/// distortion — which the open-loop DirectVoltage hold must keep supplying
/// or the hold collapses: a g431 loses `800 ns × f_pwm × vbus ≈ 0.38 V`,
/// more than the entire `R·I` holding voltage of a low-resistance
/// outrunner, and the avg-current open-circuit gate then (correctly)
/// reports `MotorNotResponding`. A naked `R·I` reproduced exactly that on
/// the simulated non-ideal plant.
async fn settled_hold_voltage<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    resistance_ohm: f32,
    hold_current_a: f32,
) -> f32 {
    let computed = resistance_ohm * hold_current_a;
    match sample_vd_id::<H, T>(hw, 200, 1000).await {
        Ok((avg_vd, avg_id)) => computed + (avg_vd - resistance_ohm * avg_id),
        Err(_) => computed,
    }
}

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
        })
        .await;
        T::after_millis(4).await;
    }
    T::after_millis(settle_cycles).await;

    // Sample at low setpoint (~100 ms nominal at 20 kHz; 2 s deadline).
    // On timeout, command Stopped before bailing — the motor was left at the
    // setpoint and the normal ramp-down below never runs on this path.
    let (vd_low, id_low) = match sample_vd_id::<H, T>(hw, sample_count, 2000).await {
        Ok(avg) => avg,
        Err(e) => {
            hw.send_command(ControlMode::Stopped).await;
            return Err(e);
        }
    };
    debug!("R meas: low point: vd={}, id={}", vd_low, id_low);

    // --- Ramp to high setpoint ---
    for i in 1..=ramp_steps {
        let current = i_low + (i_high - i_low) * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: None,
        })
        .await;
        T::after_millis(4).await;
    }
    T::after_millis(settle_cycles).await;

    // Sample at high setpoint (same deadline and bail-out)
    let (vd_high, id_high) = match sample_vd_id::<H, T>(hw, sample_count, 2000).await {
        Ok(avg) => avg,
        Err(e) => {
            hw.send_command(ControlMode::Stopped).await;
            return Err(e);
        }
    };
    debug!("R meas: high point: vd={}, id={}", vd_high, id_high);

    // --- Ramp down and stop ---
    for i in (0..ramp_steps).rev() {
        let current = i_high * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: None,
        })
        .await;
        T::after_millis(4).await;
    }
    hw.send_command(ControlMode::Stopped).await;
    T::after_millis(100).await;

    // --- Compute R = ΔV / ΔI ---
    let delta_i = id_high - id_low;
    let delta_v = vd_high - vd_low;

    debug!("R meas: dV={}, dI={}", delta_v, delta_i);

    if delta_i.abs() < 0.1 {
        return Err(DetectionError::MotorNotResponding);
    }

    // The current loop must actually have converged on the setpoints: a
    // rotor that isn't locked (cogging past detents, oscillating PI) still
    // averages to *some* vd/id, yielding a plausible-but-wrong R that then
    // poisons everything downstream (inductance comp, PI tuning).
    //
    // Tolerance is 30% of the setpoint but never tighter than SETTLE_TOL_FLOOR_A:
    // at a low probe setpoint (e.g. 0.1 A) the inverter dead-time biases the
    // settled current by a fixed ~0.04 A on the g431, which alone exceeds a bare
    // 30% (0.03 A) and rejects a perfectly good measurement. The 2-point ΔV/ΔI is
    // offset-robust — a constant current bias cancels in ΔI — so a loose landing
    // at the low point does NOT bias R; only gross motion (≫ the floor) should
    // fail here, and the floor still catches that.
    const SETTLE_TOL_FLOOR_A: f32 = 0.15;
    let tol_low = (0.3 * i_low).max(SETTLE_TOL_FLOOR_A);
    let tol_high = (0.3 * i_high).max(SETTLE_TOL_FLOOR_A);
    if (id_low - i_low).abs() > tol_low || (id_high - i_high).abs() > tol_high {
        debug!(
            "R meas: current didn't settle (id {} vs {} / {} vs {})",
            id_low, i_low, id_high, i_high
        );
        return Err(DetectionError::UnexpectedMotion);
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

/// Amplitude adaptation context for the HFI loop.
#[cfg(feature = "hfi-detect")]
struct HfiAdapt {
    /// Carrier angular frequency (rad/s) for the |Z| solve.
    omega: f32,
    /// Phase resistance (Ω) for the |Z| solve.
    r: f32,
    /// Target ripple current (A).
    i_target: f32,
    /// Amplitude clamp range (V).
    v_min: f32,
    v_max: f32,
}

/// Maximum command→apply pipeline depth the probe scans for (FOC cycles).
const PIPELINE_LAG_MAX: usize = 4;

/// Minimum single-period current rise (A) the voltage-pulse edge detector
/// treats as a real pulse; below it the winding looks open / unresponsive.
const PULSE_EDGE_THRESHOLD_A: f32 = 0.02;

/// Frame cap on the inter-pulse discharge wait (~200 ms at 20 kHz; covers an
/// L/R decay constant up to ~10 ms). The winding settles back to the holding
/// current at the open-loop `vd_hold` equilibrium before the next pulse.
const DISCHARGE_MAX_FRAMES: usize = 4000;

/// Measure the command→apply pipeline depth by cross-correlation.
///
/// Injects the rotating carrier for a few periods and correlates the
/// measured current against the carrier reference at candidate lags
/// `1..=PIPELINE_LAG_MAX`. Through an inductor the current response to
/// `v = A·sin(φ)·dir(θ)` is `≈ −(A/ω_c L)·cos(φ)·dir(θ)` — at the true
/// lag the `−cos` correlation peaks (at the default 5 kHz/20 kHz the
/// carrier advances 90° per cycle, so adjacent lags are orthogonal and
/// the argmax is sharp). The loop structure (read → send) is identical
/// to `hfi_collect`, so the returned lag plugs straight into its history
/// pairing: lag 1 = a command sent at iteration k drives the current
/// change read at iteration k+1 (the classic single-cycle pipeline).
///
/// Also returns a latency-immune magnitude estimate of L: the response
/// NORM is invariant under pairing rotation, so
/// `|Z| = A / |i_response|`, `L = √(|Z|² − R²)/ω_c` — used downstream as
/// a cross-check against the phase-sensitive demod result.
///
/// Returns `(lag, l_magnitude_estimate)`; `l_magnitude_estimate = 0.0`
/// when the response was too weak to correlate (open motor — let the
/// main measurement fail with its own diagnostics).
#[cfg(feature = "hfi-detect")]
async fn probe_hfi_pipeline_lag<H: DetectionHardware, S: SinCos>(
    hw: &mut H,
    injector: &mut HfiInjector<S>,
    vd_hold: f32,
    dt: f32,
    resistance_ohm: f32,
) -> (u32, f32) {
    let omega_c = injector.omega_hfi();
    let amp = injector.voltage_amplitude();
    let samples_per_period = ((core::f32::consts::TAU / (omega_c * dt)) + 0.5).max(2.0) as usize;
    // Settle the transient, then accumulate over an integer number of
    // carrier periods (the DC hold projection cancels over full periods).
    let warmup = 3 * samples_per_period + PIPELINE_LAG_MAX;
    let accum = 16 * samples_per_period;

    let mut hist = [(0.0f32, 0.0f32); 8]; // (direction angle, carrier phase)
    let mut corr_q = [0.0f32; PIPELINE_LAG_MAX + 1];
    let mut corr_i = [0.0f32; PIPELINE_LAG_MAX + 1];

    for k in 0..(warmup + accum) {
        let _telem = hw.wait_telemetry().await;
        let (ia, ib, _ic) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        if k >= warmup {
            for d in 1..=PIPELINE_LAG_MAX {
                let (theta, phase) = hist[(k + 8 - d) % 8];
                let (sin_t, cos_t) = S::sin_cos(theta);
                let (sin_p, cos_p) = S::sin_cos(phase);
                let i_dir = i_alpha * cos_t + i_beta * sin_t;
                corr_q[d] += i_dir * (-cos_p);
                corr_i[d] += i_dir * sin_p;
            }
        }

        let theta = injector.injection_angle();
        let phase = injector.carrier_phase();
        let (v_a, v_b) = injector.step(dt);
        hw.send_command(ControlMode::DirectVoltage {
            vd: vd_hold + v_a,
            vq: v_b,
            angle_rad: 0.0,
        })
        .await;
        hist[k % 8] = (theta, phase);
    }

    // Each command holds for one full period, so the discrete response
    // phase sits at the period CENTER: i ∝ −cos(φ + ω_c·dt/2). Score each
    // lag by projecting onto that half-step-rotated reference — with the
    // default 90°-per-cycle carrier the raw −cos correlation alone splits
    // 45°/45° between adjacent bins and cannot discriminate them.
    let (half_sin, half_cos) = S::sin_cos(omega_c * dt * 0.5);
    let score = |d: usize| corr_q[d] * half_cos + corr_i[d] * half_sin;
    let mut lag = 1usize;
    for d in 2..=PIPELINE_LAG_MAX {
        if score(d) > score(lag) {
            lag = d;
        }
    }
    debug!(
        "lag probe bins (q,i): 1=({},{}) 2=({},{}) 3=({},{}) 4=({},{})",
        corr_q[1], corr_i[1], corr_q[2], corr_i[2], corr_q[3], corr_i[3], corr_q[4], corr_i[4]
    );

    // Latency-immune |Z| from the winning bin's response norm.
    let n = accum as f32;
    let i_amp = 2.0 * sqrtf(corr_q[lag] * corr_q[lag] + corr_i[lag] * corr_i[lag]) / n;
    let l_mag = if i_amp > 1e-4 && amp > 0.0 {
        let z = amp / i_amp;
        let zl_sq = z * z - resistance_ohm * resistance_ohm;
        if zl_sq > 0.0 {
            sqrtf(zl_sq) / omega_c
        } else {
            0.0
        }
    } else {
        0.0
    };

    info!(
        "HFI pipeline lag probe: lag={} cycles, |Z|-estimate L={} H",
        lag, l_mag
    );
    (lag as u32, l_mag)
}

/// Run the HFI collection loop: rotating injection riding on `vd_hold`,
/// recording into `measurement` until it reports complete.
///
/// With `adapt` set, the first [`HFI_PROBE_CYCLES`] windows act as an
/// amplitude scout: the interim L estimate solves `V = I_target · |Z|`,
/// the injector amplitude is re-scaled and the accumulators restart, so
/// only properly-scaled windows reach the final result.
///
/// Leaves the motor in `DirectVoltage { vd: vd_hold }` — the caller ramps
/// down.
#[cfg(feature = "hfi-detect")]
async fn hfi_collect<H: DetectionHardware, S: SinCos>(
    hw: &mut H,
    injector: &mut HfiInjector<S>,
    measurement: &mut InductanceMeasurement<S>,
    vd_hold: f32,
    dt: f32,
    mut adapt: Option<HfiAdapt>,
    lag: u32,
) {
    // DirectVoltage mode — no PI interference during measurement.
    // The captured vd_hold maintains the holding force, HFI injection is
    // added on top.
    //
    // Pairing: the current change read at iteration k was driven by the
    // command sent at iteration k − lag (lag = the measured command→apply
    // pipeline depth, see probe_hfi_pipeline_lag). A ring buffer of sent
    // commands resolves the pairing explicitly; `record_from` gates the
    // first records until the post-(re)start history is deep enough.
    let lag = (lag as usize).clamp(1, PIPELINE_LAG_MAX);
    let mut hist = [(0.0f32, 0.0f32, 0.0f32); 8]; // (angle, v_alpha, v_beta)
    let mut iter: usize = 0;
    let mut record_from = lag;

    while !measurement.is_complete() {
        if measurement.cycles_completed() >= HFI_PROBE_CYCLES {
            // On a failed scout (no interim estimate) keep the configured
            // amplitude — finish() reports the real error downstream.
            // `adapt.take()` only fires once an interim estimate exists.
            if let Some(l_avg) = measurement.interim_l_avg()
                && let Some(a) = adapt.take()
            {
                let z = sqrtf(a.r * a.r + a.omega * l_avg * a.omega * l_avg);
                let v_run = clamp_f32(a.i_target * z, a.v_min, a.v_max);
                info!("HFI amplitude adapted: {}V", v_run);
                injector.set_amplitude(v_run);
                injector.reset();
                measurement.restart(v_run);
                // Old-amplitude commands are still in flight for `lag`
                // cycles; let them flush before recording resumes.
                record_from = iter + lag;
            } else {
                adapt = None;
            }
        }

        // Wait for current PWM cycle to complete (synced to ADC ISR)
        let _telem = hw.wait_telemetry().await;

        // Read currents from THIS cycle (the response to the command sent
        // `lag` iterations ago)
        let (ia, ib, _ic) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        if iter >= record_from {
            let (angle, v_a, v_b) = hist[(iter + 8 - lag) % 8];
            measurement.record(i_alpha, i_beta, angle, v_a, v_b);
        }

        // Calculate and send NEXT injection command
        let injection_angle = injector.injection_angle();
        let (v_alpha_inj, v_beta_inj) = injector.step(dt);

        // At angle 0, α-β = d-q: vd_hold holds rotor, injection rides on top
        hw.send_command(ControlMode::DirectVoltage {
            vd: vd_hold + v_alpha_inj,
            vq: v_beta_inj,
            angle_rad: 0.0,
        })
        .await;

        hist[iter % 8] = (injection_angle, v_alpha_inj, v_beta_inj);
        iter += 1;
    }
}

/// Number of FFT windows for the amplitude-scouting probe phase.
#[cfg(feature = "hfi-detect")]
const HFI_PROBE_CYCLES: u32 = 8;

/// Target HFI ripple current as a fraction of the holding current — large
/// enough to clear the ADC noise floor, small enough that the rotor stays
/// firmly locked and the total current stays inside the thermal budget.
#[cfg(feature = "hfi-detect")]
const HFI_RIPPLE_FRACTION: f32 = 0.25;

/// Hard cap on the HFI injection current (amps) — both the lag/|Z| probe and
/// the adaptive collection ripple. At the carrier frequency |Z| ≥ R, so the
/// worst-case (low-inductance) ripple is `V/R`; capping the probe at `I·R`
/// keeps even a near-short within budget. The fixed `hfi_voltage_v` (3 V) drew
/// tens of amps on a low-impedance outrunner and tripped the bench PSU; the
/// voltage-pulse path is current-limited the same way (`calibrate_pulse_voltage`).
#[cfg(feature = "hfi-detect")]
const HFI_PROBE_CURRENT_A: f32 = 2.0;

/// Measure motor inductance using rotating HFI.
///
/// Injects a rotating high-frequency voltage vector in α-β frame and
/// analyzes the current response using FFT. Runs in two passes: a short
/// probe at `params.hfi_voltage_v` scouts L, then the main pass re-solves
/// the amplitude for a target ripple current (a fraction of the holding
/// current), clamped to the bus-voltage headroom when `params.vbus` is set.
///
/// # Arguments
/// * `hw` - Hardware abstraction implementation
/// * `params` - Inductance measurement parameters
/// * `pwm_freq_hz` - PWM frequency in Hz
///
/// # Returns
/// * `Ok((ld, lq))` - Measured d-axis and q-axis inductance in Henries
/// * `Err(DetectionError)` - If measurement failed
#[cfg(feature = "hfi-detect")]
pub async fn measure_inductance<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    info!("Starting inductance measurement (rotating HFI)...");

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
        })
        .await;
        T::after_millis(10).await;
    }

    // Wait for the rotor to settle, then derive the DirectVoltage holding
    // voltage (see `settled_hold_voltage` — `R·I` alone collapses under
    // uncompensated dead-time distortion). Telemetry vd is the
    // PRE-modulation command, so configured dead-time comp (a duty-domain
    // adjustment) is not double-counted.
    T::after_millis(u64::from(params.settle_time_ms)).await;
    let vd_hold =
        settled_hold_voltage::<H, T>(hw, params.resistance_ohm, params.hold_current_a).await;

    // Voltage headroom above the holding voltage (when vbus is known):
    // commanding beyond this saturates against the bus mid-carrier and
    // distorts the injection waveform.
    let headroom = if params.vbus > 0.0 {
        ((params.vbus * 0.577 - vd_hold) * 0.9).max(0.1)
    } else {
        f32::INFINITY
    };

    // ── Current-budgeted injection ──────────────────────────────────────
    // A fixed amplitude gives a ripple V/(ω·L) spanning two orders of magnitude
    // across motors (amps on a 15 µH outrunner, mA on a 3 mH gimbal), so the
    // amplitude is solved for a target ripple current — VESC-style, the same way
    // the voltage-pulse path is current-limited (`calibrate_pulse_voltage`).
    //
    // The PROBE that seeds it must be safe too: at the carrier |Z| ≥ R, so `I·R`
    // caps the worst-case (low-L) probe current. The old fixed `hfi_voltage_v`
    // (3 V) instead drew tens of amps on a low-impedance outrunner and tripped
    // the bench PSU before the adaptive collection could re-scale.
    let omega = params.hfi_frequency_hz * core::f32::consts::TAU;
    let i_cap = (HFI_PROBE_CURRENT_A * params.resistance_ohm).max(0.2);
    let probe_v = i_cap.min(headroom).min(params.hfi_voltage_v);
    info!(
        "HFI injection (probe v={}V, vd_hold={}V, I_cap={}A)...",
        probe_v, vd_hold, HFI_PROBE_CURRENT_A
    );

    let mut injector = HfiInjector::<S>::new(params.hfi_frequency_hz, probe_v, pwm_freq_hz);

    // Command→apply pipeline depth. The probe ALWAYS runs: besides the
    // lag it yields the latency-immune |Z| magnitude estimate of L
    // (pairing rotations preserve the response norm) that guards the
    // phase-sensitive demod below — an override must not silently disable
    // that safety net. `pipeline_lag ≥ 1` overrides the lag value only.
    let (probed_lag, l_probe) =
        probe_hfi_pipeline_lag::<H, S>(hw, &mut injector, vd_hold, dt, params.resistance_ohm).await;
    injector.reset();
    let lag = if params.pipeline_lag >= 1 {
        params.pipeline_lag as u32
    } else {
        probed_lag
    };

    // Now |Z| is known from the safe probe, so the adaptive collection can scale
    // up to the ripple target without overcurrent: the most it can command is
    // `v_max`, drawing `v_max/|Z| ≈ HFI_PROBE_CURRENT_A` at any inductance.
    let z_probe = sqrtf(
        params.resistance_ohm * params.resistance_ohm + (omega * l_probe) * (omega * l_probe),
    )
    .max(params.resistance_ohm);
    let adapt = HfiAdapt {
        omega,
        r: params.resistance_ohm,
        i_target: clamp_f32(
            HFI_RIPPLE_FRACTION * params.hold_current_a,
            0.05,
            HFI_PROBE_CURRENT_A,
        ),
        v_min: 0.2,
        v_max: clamp_f32(HFI_PROBE_CURRENT_A * z_probe, probe_v, headroom),
    };

    let mut measurement = InductanceMeasurement::<S>::new(params, pwm_freq_hz);
    measurement.restart(probe_v);
    hfi_collect::<H, S>(
        hw,
        &mut injector,
        &mut measurement,
        vd_hold,
        dt,
        Some(adapt),
        lag,
    )
    .await;

    // Ramp down holding voltage
    info!("HFI measurement complete, ramping down...");

    for i in (0..ramp_steps).rev() {
        let vd = vd_hold * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::DirectVoltage {
            vd,
            vq: 0.0,
            angle_rad: 0.0,
        })
        .await;
        T::after_millis(10).await;
    }

    hw.send_command(ControlMode::Stopped).await;
    T::after_millis(100).await;

    // Compute result
    let result = measurement.finish()?;

    // Latency-immune cross-check: the demod L is phase-sensitive (a wrong
    // pairing corrupts it while looking plausible), the probe's |Z|
    // magnitude is not. A gross mismatch means the demod cannot be
    // trusted — LowConfidence sends the auto-ladder to the pulse method.
    if l_probe > 0.0 {
        let ratio = result.l_avg / l_probe;
        if !(0.4..=2.5).contains(&ratio) {
            info!(
                "HFI demod L={} disagrees with |Z| magnitude L={} — low confidence",
                result.l_avg, l_probe
            );
            return Err(DetectionError::LowConfidence);
        }
    }

    Ok((result.ld, result.lq))
}

/// Fixed context for a voltage-pulse session on one locked axis: where the
/// rotor is held (`angle` + its sin/cos), the steady d-axis voltage that holds
/// it there, and the current it settles back to between pulses.
struct LockedAxis {
    angle: f32,
    sin_a: f32,
    cos_a: f32,
    vd_hold: f32,
    hold_current_a: f32,
    /// Settle band around `hold_current_a` (kept above the ADC noise floor).
    settle_tol: f32,
}

impl LockedAxis {
    /// d-axis current at this lock angle, from the live phase currents.
    fn read_id<H: DetectionHardware>(&self, hw: &H) -> f32 {
        let (ia, ib, _) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);
        i_alpha * self.cos_a + i_beta * self.sin_a
    }
}

/// One discharge-anchored pulse on a locked axis.
///
/// Settles the winding back to the holding current at `vd_hold` (the open-loop
/// equilibrium — polled, not a fixed wait, because the L/R decay outlasts the
/// inter-pulse gap), then applies `vd_hold + pulse_v` and returns the
/// `(i_before, i_after)` of the *largest* single-period current rise across the
/// next `PIPELINE_LAG_MAX + 1` frames. That argmax is the application edge:
/// immune to the command→apply latency and to ADC noise (the real step dwarfs
/// it). The accumulator works from the absolute current, so a loose settle does
/// not bias the result.
async fn pulse_once<H: DetectionHardware>(hw: &mut H, ax: &LockedAxis, pulse_v: f32) -> (f32, f32) {
    // Discharge to the holding current.
    hw.send_command(ControlMode::DirectVoltage {
        vd: ax.vd_hold,
        vq: 0.0,
        angle_rad: ax.angle,
    })
    .await;
    for _ in 0..DISCHARGE_MAX_FRAMES {
        hw.wait_telemetry().await;
        if (ax.read_id(hw) - ax.hold_current_a).abs() < ax.settle_tol {
            break;
        }
    }

    // Apply the step, capture the application window, take the max rise.
    let id_before = ax.read_id(hw);
    hw.send_command(ControlMode::DirectVoltage {
        vd: ax.vd_hold + pulse_v,
        vq: 0.0,
        angle_rad: ax.angle,
    })
    .await;
    let mut prev = id_before;
    let mut best_before = id_before;
    let mut best_after = id_before;
    let mut best_di = f32::NEG_INFINITY;
    for _ in 0..=PIPELINE_LAG_MAX {
        hw.wait_telemetry().await;
        let id_now = ax.read_id(hw);
        let di = id_now - prev;
        if di > best_di {
            best_di = di;
            best_before = prev;
            best_after = id_now;
        }
        prev = id_now;
    }
    (best_before, best_after)
}

/// Size the pulse step to a target current excursion — VESC's
/// `mcpwm_foc_measure_inductance_current` idea, applied to the di/dt pulse.
///
/// Ramps the step up from small until one period's di reaches `target_di`, so
/// the peak current (`I_hold + di`) is bounded *regardless of L*. A fixed step
/// drives `di = V·dt/L`, which explodes on a low-inductance motor — a 24 µH
/// drone winding spikes ~14 A in one period off a 12 V bus, and the bigger the
/// bus the worse it gets. Once bracketed, trims to the target exactly
/// (`di ∝ pulse_v` for a linear winding); caps at the bus-headroom `ceiling`
/// for high-L motors that can't reach the target at all (there it just uses all
/// the headroom it has — still the best available SNR).
async fn calibrate_pulse_voltage<H: DetectionHardware>(
    hw: &mut H,
    ax: &LockedAxis,
    ceiling: f32,
    target_di: f32,
) -> f32 {
    // Start small (≈ VESC's 0.02 duty) and grow geometrically; the early
    // probes barely move the current, so they are inherently safe.
    let mut v = (ceiling * 0.02).max(0.02);
    loop {
        let (before, after) = pulse_once(hw, ax, v).await;
        let di = after - before;
        if di >= target_di && di > 0.0 {
            // Bracketed — trim down to hit the target exactly.
            return clamp_f32(v * target_di / di, 0.02, ceiling);
        }
        if v >= ceiling {
            return ceiling; // high-L: target unreachable, use all headroom
        }
        v = (v * 1.5).min(ceiling);
    }
}

/// Measure inductance via voltage pulse (di/dt).
///
/// Locks the rotor at angle 0 (d-axis), applies a voltage step, measures
/// the current change over one PWM period, then repeats at angle π/2
/// (q-axis).  Works reliably on high-resistance motors where HFI fails.
///
/// With `params.current_target_a > 0` the step amplitude is current-limited
/// (see [`calibrate_pulse_voltage`]) so the peak current stays bounded on a
/// low-inductance motor; otherwise `params.pulse_voltage_v` is used as-is.
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
    // Step amplitude. Calibrated once (on the d axis) and reused: for a salient
    // motor Lq > Ld, so the q-axis di at the same step is smaller — i.e. still
    // within the current budget. `0` target ⇒ use the ceiling as-is.
    let mut pulse_voltage = params.pulse_voltage_v;
    let mut calibrated = false;

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
            })
            .await;
            T::after_millis(10).await;
        }
        T::after_millis(u64::from(params.settle_time_ms)).await;

        // Steady-state holding voltage (averaged; robust to a PI that is
        // still converging and to dead-time make-up — see
        // `settled_hold_voltage`).
        let vd_hold =
            settled_hold_voltage::<H, T>(hw, params.resistance_ohm, params.hold_current_a).await;

        let (sin_a, cos_a) = S::sin_cos(angle);
        let ax = LockedAxis {
            angle,
            sin_a,
            cos_a,
            vd_hold,
            hold_current_a: params.hold_current_a,
            // Settle band around i_hold (kept above the ADC noise floor).
            settle_tol: (0.1 * params.hold_current_a).max(0.05),
        };

        // Size the step to the current budget once (current-limited pulse).
        if !calibrated && params.current_target_a > 0.0 {
            pulse_voltage =
                calibrate_pulse_voltage(hw, &ax, params.pulse_voltage_v, params.current_target_a)
                    .await;
            calibrated = true;
            debug!("Pulse step calibrated to the current target");
        }

        // Average num_pulses discharge-anchored pulses (see `pulse_once`): each
        // settles the winding back to i_hold, applies the step, and takes the
        // largest one-period rise as the application edge. The accumulator
        // turns that (i_before, i_after) into L from the absolute current, so a
        // ratcheting baseline cannot bias it.
        let cal_params = VoltagePulseParams {
            pulse_voltage_v: pulse_voltage,
            ..*params
        };
        let mut meas = VoltagePulseMeasurement::new(&cal_params, pwm_freq_hz, vd_hold);
        // Residual dead-time the firmware's compensation did not cancel
        // (vd_hold − R·I_hold). Removed from the pulse so L is dead-time-immune;
        // also the signal a self-calibrating comp factor would key off.
        info!(
            "axis {}: vd_hold={} V, residual dead-time={} V, pulse={} V",
            axis,
            vd_hold,
            meas.dead_time_v(),
            pulse_voltage
        );

        for _ in 0..params.num_pulses * 2 {
            // guard against skipped pulses
            if meas.is_complete() {
                break;
            }

            let (before, after) = pulse_once(hw, &ax, pulse_voltage).await;

            // A real pulse dwarfs the noise; below the floor means an open
            // winding (no rise) — record a skip so finish() reports it.
            if after - before > PULSE_EDGE_THRESHOLD_A {
                meas.record_pulse(before, after);
            } else {
                meas.record_pulse(before, before);
            }
        }

        results[axis] = (angle, meas.finish()?);

        // Ramp down
        for i in (0..ramp_steps).rev() {
            let vd = vd_hold * (i as f32 / ramp_steps as f32);
            hw.send_command(ControlMode::DirectVoltage {
                vd,
                vq: 0.0,
                angle_rad: angle,
            })
            .await;
            T::after_millis(10).await;
        }
        hw.send_command(ControlMode::Stopped).await;
        T::after_millis(200).await;
    }

    let ld = results[0].1; // angle 0 = d-axis
    let lq = results[1].1; // angle π/2 = q-axis
    info!("Voltage-pulse inductance measurement complete");
    Ok((ld, lq))
}

/// Carrier frequencies swept by [`measure_impedance_sweep`], as **fractions of
/// the PWM frequency** so the list self-scales to the loop and stays inside the
/// synchronous-demod limit. The carrier is sampled once per PWM period, so
/// `1/frac` is the samples-per-carrier-period: clean sine demod needs `≥4`, hence
/// the top fraction is `~f_sw/4.3` (not `f_sw/3` — 3 samples/period is below a
/// usable sine, and `f_sw/2` degenerates the carrier to a sign flip, killing the
/// quadrature → L entirely). The fractions are also deliberately **off integer
/// reciprocals** (no `1/4`, `1/8`, …): at an exact `1/N` ratio the carrier phase
/// locks to `N` fixed points and the in-phase (`corr_i`→R) and quadrature
/// (`corr_q`→ωL) projections fall on disjoint sample sets, so any even/odd
/// measurement asymmetry biases R against L — detuning precesses the phase grid
/// and interleaves them. On the 20 kHz g431 loop this is
/// `[500, 900, 1680, 2700, 3700, 4640] Hz`: it brackets the bench LCR 1 kHz point
/// and the ~5 kHz HFI band, but **cannot reach the bench 10 kHz point** (that is
/// `f_sw/2` here) — compare the R(f)/L(f) trend, not that single point.
#[cfg(feature = "impedance-sweep")]
const SWEEP_FREQ_FRACTIONS: [f32; 6] = [
    0.025, // f_sw/40 ≈ 500 Hz  (40 samples/carrier period)
    0.045, //         ≈ 900 Hz  (22.2)
    0.084, //         ≈ 1680 Hz (11.9)
    0.135, //         ≈ 2700 Hz (7.4)
    0.185, //         ≈ 3700 Hz (5.4)
    0.232, //         ≈ 4640 Hz (4.3) — top: off f_sw/4 so the phase grid precesses
];

/// Accumulate the in-phase and quadrature current response to the rotating
/// carrier at a **known** pipeline lag, and solve the *complex* impedance.
///
/// Identical inner loop to [`probe_hfi_pipeline_lag`] (read → correlate → send),
/// but at one fixed `lag` instead of scanning, and it keeps both projections:
/// `corr_i` (onto `sin φ`) carries the in-phase / resistive response, `corr_q`
/// (onto `−cos φ`) the quadrature / inductive one. Each held command drives the
/// current at the period *centre*, so the carrier has advanced `ω_c·dt/2` by then;
/// de-rotating the complex sum `corr_i + j·corr_q` by that half step makes its
/// argument the true impedance angle `ψ = atan2(ωL, R)`. With `|Z| = A/|i|` this
/// yields `R = |Z|·cos ψ` and `L = |Z|·sin ψ / ω_c` directly — no assumed R,
/// unlike the magnitude-only `|Z|` solve the production HFI path uses.
///
/// Returns `(r_ac, l, |Z|)`; `r_ac` falls back to the supplied `resistance_ohm`
/// and `l` to `0.0` when the response is too weak to resolve.
#[cfg(feature = "impedance-sweep")]
async fn measure_impedance_at<H: DetectionHardware, S: SinCos>(
    hw: &mut H,
    injector: &mut HfiInjector<S>,
    vd_hold: f32,
    dt: f32,
    lag: u32,
    resistance_ohm: f32,
) -> (f32, f32, f32) {
    let omega_c = injector.omega_hfi();
    let amp = injector.voltage_amplitude();
    let lag = (lag as usize).clamp(1, PIPELINE_LAG_MAX);
    // Accumulate over a whole number of carrier periods. Use the EXACT
    // samples-per-period (not rounded): `round(N·spp_exact)` spans ≈ N whole
    // periods even at a non-integer ratio, so the DC-hold projection and the
    // fundamental both close cleanly. `N·round(spp)` instead leaves a partial
    // period whose leakage corrupts the *phase* (the R/L split) while |Z| — the
    // norm — survives; the detuned sweep frequencies are non-integer ratios, so
    // this is exactly where it bites.
    let spp_exact = (core::f32::consts::TAU / (omega_c * dt)).max(2.0);
    let warmup = (3.0 * spp_exact + 0.5) as usize + lag;
    let accum = (16.0 * spp_exact + 0.5) as usize;

    let mut hist = [(0.0f32, 0.0f32); 8]; // (direction angle, carrier phase)
    let mut corr_q = 0.0f32;
    let mut corr_i = 0.0f32;

    for k in 0..(warmup + accum) {
        let _telem = hw.wait_telemetry().await;
        let (ia, ib, _ic) = hw.read_phase_currents();
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);

        if k >= warmup {
            let (theta, phase) = hist[(k + 8 - lag) % 8];
            let (sin_t, cos_t) = S::sin_cos(theta);
            let (sin_p, cos_p) = S::sin_cos(phase);
            let i_dir = i_alpha * cos_t + i_beta * sin_t;
            corr_q += i_dir * (-cos_p);
            corr_i += i_dir * sin_p;
        }

        let theta = injector.injection_angle();
        let phase = injector.carrier_phase();
        let (v_a, v_b) = injector.step(dt);
        hw.send_command(ControlMode::DirectVoltage {
            vd: vd_hold + v_a,
            vq: v_b,
            angle_rad: 0.0,
        })
        .await;
        hist[k % 8] = (theta, phase);
    }

    let n = accum as f32;
    let mag = sqrtf(corr_q * corr_q + corr_i * corr_i);
    let i_amp = 2.0 * mag / n;
    if i_amp < 1e-5 || amp <= 0.0 || mag < 1e-9 {
        return (resistance_ohm, 0.0, resistance_ohm);
    }
    let z = amp / i_amp;
    // De-rotate (corr_i + j·corr_q) by the half-step dwell ω_c·dt/2 so the
    // argument becomes the impedance angle: real part ∝ R, imag part ∝ ωL.
    let (sin_d, cos_d) = S::sin_cos(omega_c * dt * 0.5);
    let re = corr_i * cos_d - corr_q * sin_d; // ∝ R
    let im = corr_i * sin_d + corr_q * cos_d; // ∝ ωL
    let r_ac = (z * re / mag).max(0.0);
    let l = (z * im / mag / omega_c).max(0.0);
    (r_ac, l, z)
}

/// **Experiment (feature `impedance-sweep`):** map R(f) and L(f) on the d axis.
///
/// Locks the rotor on the d axis **once**, then sweeps the HFI carrier across
/// [`SWEEP_FREQ_FRACTIONS`] of `f_sw`, extracting the complex impedance with
/// [`measure_impedance_at`] — so both the AC resistance and the inductance fall
/// out of a single locked measurement. Logs `(f, V, |Z|, R, L)` per row so the
/// on-device curve can be overlaid on a bench LCR sweep (read it from RTT).
///
/// Reuses the production safety path verbatim: the same rotor lock, the same
/// [`settled_hold_voltage`], the same command→apply [`probe_hfi_pipeline_lag`],
/// and the same current budget — the amplitude is `I_target·|Z(f)|` predicted
/// from one seed L estimate, so the ripple stays ≈ the target at every frequency
/// (`|Z| ≥ R` keeps it within budget) and never trips the bench PSU.
///
/// Returns the d-axis L at the highest swept frequency (the AC inductance the
/// current loop actually sees) so the caller still yields a normal inductance
/// result; the full R(f)/L(f) table lives in the log.
#[cfg(feature = "impedance-sweep")]
pub async fn measure_impedance_sweep<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    info!("Starting impedance sweep R(f)/L(f) (experiment)...");
    let dt = 1.0 / pwm_freq_hz;
    let ramp_steps = 50u32;
    let det_gains = Some((DETECTION_PI_KP, DETECTION_PI_KI));

    // Lock the rotor on the d axis (angle 0) at the holding current.
    for i in 1..=ramp_steps {
        let current = params.hold_current_a * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current,
            velocity_rad_s: 0.0,
            pi_gains: if i == 1 { det_gains } else { None },
        })
        .await;
        T::after_millis(10).await;
    }
    T::after_millis(u64::from(params.settle_time_ms)).await;
    let vd_hold =
        settled_hold_voltage::<H, T>(hw, params.resistance_ohm, params.hold_current_a).await;

    // Voltage headroom above the hold (so the injection never saturates the bus).
    let headroom = if params.vbus > 0.0 {
        ((params.vbus * 0.577 - vd_hold) * 0.9).max(0.1)
    } else {
        f32::INFINITY
    };

    // Probe the command→apply pipeline lag (and a seed |Z|→L) once at the
    // default carrier. The lag is the FOC-cycle pipeline depth — frequency
    // independent — so it is reused for every swept frequency; the seed L sizes
    // the per-frequency amplitude. The probe current is capped at I·R (|Z| ≥ R).
    let probe_v = (HFI_PROBE_CURRENT_A * params.resistance_ohm)
        .max(0.2)
        .min(headroom);
    let mut injector = HfiInjector::<S>::new(params.hfi_frequency_hz, probe_v, pwm_freq_hz);
    let (lag, l_seed) =
        probe_hfi_pipeline_lag::<H, S>(hw, &mut injector, vd_hold, dt, params.resistance_ohm).await;

    let i_target = clamp_f32(
        HFI_RIPPLE_FRACTION * params.hold_current_a,
        0.05,
        HFI_PROBE_CURRENT_A,
    );
    info!(
        "=== impedance sweep (single rotor lock, vd_hold={}V, lag={}) ===",
        vd_hold, lag
    );
    // The returned L is the highest-frequency (best ωL-SNR) point, from the
    // phase-invariant |Z| magnitude — the AC inductance the current loop sees.
    // The full curve is in the log.
    let mut l_return = 0.0f32;
    for &frac in SWEEP_FREQ_FRACTIONS.iter() {
        let f = frac * pwm_freq_hz;
        let omega = core::f32::consts::TAU * f;
        // Predict |Z(f)| from the seed L so the amplitude draws ≈ i_target at
        // every frequency without a per-frequency adaptation loop.
        let z_pred = sqrtf(
            params.resistance_ohm * params.resistance_ohm + (omega * l_seed) * (omega * l_seed),
        )
        .max(params.resistance_ohm);
        let v = clamp_f32(i_target * z_pred, 0.2, headroom);
        let mut inj = HfiInjector::<S>::new(f, v, pwm_freq_hz);
        let (r_f, l_f, z) =
            measure_impedance_at::<H, S>(hw, &mut inj, vd_hold, dt, lag, params.resistance_ohm)
                .await;
        // Robust L from the |Z| magnitude and the known DC R: phase-invariant,
        // trustworthy where the phase-sensitive R/L split is not. At low f, |Z|≈R
        // so this is noisy (small √ of a difference); at mid/high f, ωL dominates
        // and it is the reliable inductance. The phase split is logged alongside
        // as a diagnostic — a clean curve there means the carrier phase is well
        // calibrated; ragged R/L means it is not (then |Z|·L is what to trust).
        let l_from_z = if z > params.resistance_ohm {
            sqrtf(z * z - params.resistance_ohm * params.resistance_ohm) / omega
        } else {
            0.0
        };
        info!(
            "Zsweep f={} Hz V={} |Z|={} L|Z|={} H  (phase-split R={} L={})",
            f, v, z, l_from_z, r_f, l_f
        );
        l_return = l_from_z;
    }

    // Ramp the hold back down and stop.
    for i in (0..ramp_steps).rev() {
        let vd = vd_hold * (i as f32 / ramp_steps as f32);
        hw.send_command(ControlMode::DirectVoltage {
            vd,
            vq: 0.0,
            angle_rad: 0.0,
        })
        .await;
        T::after_millis(10).await;
    }
    hw.send_command(ControlMode::Stopped).await;
    T::after_millis(100).await;

    if l_return <= 0.0 {
        return Err(DetectionError::LowConfidence);
    }
    Ok((l_return, l_return))
}

/// Measure inductance: HFI first, voltage-pulse fallback when HFI fails.
///
/// This is the production entry point (both `run_full_detection` and the
/// per-step detect server route through it): the rotating-HFI method is
/// the accurate one, but on high-resistance motors the inductive ripple
/// can sink below what the ADC resolves even at full bus amplitude — the
/// pulse method (di/dt over one PWM period) still works there. The
/// fallback fires when HFI returns an implausible result
/// ([`validate_inductance`]) or errors out — except `MotorNotResponding`
/// (open circuit / dead control loop), where pulsing would see the same
/// nothing and only waste a spin of the motor leads.
pub async fn measure_inductance_auto<H: DetectionHardware, T: Timer, S: SinCos>(
    hw: &mut H,
    params: &InductanceParams,
    pwm_freq_hz: f32,
) -> Result<(f32, f32), DetectionError> {
    // HFI saliency measurement (rotating injection + FFT), when built in.
    // Off → straight to the voltage-pulse method below (the only path on
    // non-salient / flash-tight boards).
    #[cfg(feature = "hfi-detect")]
    {
        let hfi = measure_inductance::<H, T, S>(hw, params, pwm_freq_hz).await;
        match &hfi {
            Ok((ld, lq)) if validate_inductance(*ld, *lq).is_ok() => return hfi,
            Err(DetectionError::MotorNotResponding) => return hfi,
            _ => {}
        }
        info!("HFI inductance suspicious, falling back to voltage pulse");
        T::after_millis(500).await;
    }

    // Current-limited voltage pulse (the only L method with hfi-detect off).
    // `params.hold_current_a` is the power/bus-safe current run_full_detection
    // settled on; split that budget between a modest locking hold and the pulse
    // excursion, and let measure_inductance_pulse size the step to that
    // excursion. The peak (hold + di) then stays within the safe current for
    // ANY inductance — a fixed vbus·0.577 step instead drives di = V·dt/L,
    // which spikes a low-L drone winding tens of amps in a single period and
    // only gets worse on a bigger bus.
    let safe_current = params.hold_current_a.max(0.2);
    let lock_current = (safe_current * 0.4).max(0.1);
    let current_target = (safe_current * 0.5).max(0.1);
    let v_hold = params.resistance_ohm * lock_current;
    let pulse_ceiling = if params.vbus > 0.0 {
        (params.vbus * 0.577 - v_hold).max(0.5)
    } else {
        // Bus voltage unknown — keep the conservative default step.
        VoltagePulseParams::default().pulse_voltage_v
    };
    let pulse_params = VoltagePulseParams {
        hold_current_a: lock_current,
        resistance_ohm: params.resistance_ohm,
        pulse_voltage_v: pulse_ceiling,
        current_target_a: current_target,
        ..Default::default()
    };
    measure_inductance_pulse::<H, T, S>(hw, &pulse_params, pwm_freq_hz).await
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
    let omega_cap = (params.spin_rpm * core::f32::consts::TAU * f32::from(params.pole_pairs)
        / 60.0)
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
        })
        .await;
        T::after_millis(CAPTURE_TIME_MS / u64::from(CAPTURE_STEPS)).await;
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
        v_baseline += sqrtf(telem.vd * telem.vd + telem.vq * telem.vq);
        T::after_micros(500).await;
    }
    v_baseline /= BASELINE_SAMPLES as f32;

    info!("Ramping up (velocity mode)...");
    let ramp_steps = 100u32;
    let step_ms = u64::from((params.ramp_time_ms / ramp_steps).max(1));
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
        })
        .await;
        T::after_millis(step_ms).await;

        let telem = hw.wait_telemetry().await;
        let v_mag = sqrtf(telem.vd * telem.vd + telem.vq * telem.vq);
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
            hw.send_command(ControlMode::Stopped).await;
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
    let step_ms = u64::from((ramp_time_ms / ramp_steps).max(1));
    for i in (0..ramp_steps).rev() {
        let progress = i as f32 / ramp_steps as f32;
        hw.send_command(ControlMode::OpenLoop {
            angle_rad: 0.0,
            current: current_a * progress,
            velocity_rad_s: omega_e * progress,
            pi_gains: None,
        })
        .await;
        T::after_millis(step_ms).await;
    }
    hw.send_command(ControlMode::Stopped).await;
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
    T::after_millis(u64::from(params.settle_time_ms)).await;

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
/// VESC's scalar `(|V| − R|I|)/ω − |I|L` approximation. Requires R from an
/// earlier detection step; `params.inductance_h` trims the `ωL·i`
/// reactance term and may be 0.0 when L is unknown.
pub async fn measure_flux_linkage_magnitude<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    info!("Starting back-EMF-vector flux linkage measurement...");

    if params.resistance_ohm <= 0.0 {
        return Err(DetectionError::MissingPrerequisite);
    }

    let mut measurement = MagnitudeFluxMeasurement::new(
        params.resistance_ohm,
        params.inductance_h,
        params.num_samples,
    );

    let omega_e = spin_up_open_loop::<H, T>(hw, params).await?;
    T::after_millis(u64::from(params.settle_time_ms)).await;

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
    T::after_millis(u64::from(params.settle_time_ms)).await;

    // ── Release: coast with all FETs off ───────────────────────────────
    hw.send_command(ControlMode::Coast).await;

    // Wait for currents to decay (a few L/R time constants)
    T::after_millis(20).await;

    // ── Sample back-EMF during coast-down ──────────────────────────────
    let mut measurement = SpinDownFluxMeasurement::from_params(params);

    let max_coast_samples = 10_000u32; // safety limit
    for _ in 0..max_coast_samples {
        hw.wait_telemetry().await; // advance one FOC cycle
        T::after_micros(500).await; // ~2 kHz effective sample rate

        let (v_alpha, v_beta, omega_e) = hw.read_coast_telemetry();
        let v_bemf = sqrtf(v_alpha * v_alpha + v_beta * v_beta);

        if !measurement.record(v_bemf, omega_e) {
            // omega below threshold — motor has slowed too much
            break;
        }
        if measurement.has_enough_samples() {
            break;
        }
    }

    // ── Stop ───────────────────────────────────────────────────────────
    hw.send_command(ControlMode::Stopped).await;
    T::after_millis(100).await;

    let flux = measurement.finish()?;
    info!("Spin-down flux linkage measurement complete");
    Ok(flux)
}

/// Measure flux linkage: spin-down first (when the hardware can read
/// phase voltages during coast), back-EMF-vector driven method otherwise.
///
/// This is the production entry point (both `run_full_detection` and the
/// per-step detect server route through it). The ladder mirrors
/// [`measure_inductance_auto`]:
///
/// - **spin-down** is the most direct method (open-circuit back-EMF, no R
///   or L in the formula) but needs coast telemetry —
///   [`DetectionHardware::supports_coast_telemetry`] gates it honestly
///   instead of reading zeros.
/// - **back-EMF vector** ([`measure_flux_linkage_magnitude`]) is the
///   driven fallback: load-angle invariant, needs R (and optionally L)
///   from earlier steps. Also the fallback when the motor decelerates too
///   fast to collect a coast window (`InsufficientSamples`).
///
/// The q-axis method is *not* in the ladder — in open loop it is biased
/// by the load-angle cosine; it stays available for diagnostics only.
pub async fn measure_flux_linkage_auto<H: DetectionHardware, T: Timer>(
    hw: &mut H,
    params: &FluxLinkageParams,
) -> Result<f32, DetectionError> {
    if hw.supports_coast_telemetry() {
        info!("Flux: spin-down method (R-independent)");
        match measure_flux_linkage_spindown::<H, T>(hw, params).await {
            Err(DetectionError::InsufficientSamples) => {
                // Motor can't coast (high friction / geared) — drive it.
                info!("Spin-down failed (motor stopped too fast), falling back to driven method");
                T::after_millis(500).await;
            }
            done => return done,
        }
    } else {
        info!("Flux: driven method (no coast telemetry on this hardware)");
    }
    measure_flux_linkage_magnitude::<H, T>(hw, params).await
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
    let r_probe = match measure_resistance::<H, T>(hw, &probe_params).await {
        Ok(r) => r,
        // A very-low-R motor can sit below the duty resolution at the gentle
        // probe current (ΔV under one timer count averages out as ΔV ≈ 0 →
        // R ≈ 0 → OutOfRange; ADC noise normally dithers this away, a quiet
        // system may not). One retry at half the hardware limit resolves it,
        // and only near-short readings get here: anything ≥ ~0.1 Ω resolves
        // at the gentle probe already, so the retry power stays at watts.
        Err(DetectionError::OutOfRange) => {
            info!("R probe read near-short, retrying at higher current");
            let retry_params = ResistanceParams {
                current_max: (params.current_max * 0.5).max(probe_current),
                ..probe_params
            };
            measure_resistance::<H, T>(hw, &retry_params).await?
        }
        Err(e) => return Err(e),
    };
    T::after_millis(200).await;

    // Safe current: I = sqrt(max_power_loss / R / 1.5), capped to the
    // hardware limit AND to what the bus can actually drive through R —
    // the thermal formula alone asks a high-R motor for more voltage than
    // the bus has (a 20 W gimbal at 8 Ω on 12 V needs 10.3 V of the 6.9 V
    // available), the PI saturates short of the setpoint and the settle
    // check aborts the measurement.
    let max_bus_current = (params.vbus * 0.577 * 0.85) / r_probe.max(0.001);
    let safe_current = sqrtf(params.max_power_loss_w / r_probe / 1.5)
        .min(params.current_max)
        .min(max_bus_current)
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
        vbus: params.vbus,
        ..Default::default()
    };
    let (ld, lq) =
        measure_inductance_auto::<H, T, S>(hw, &inductance_params, params.pwm_freq_hz).await?;

    result.params.inductance_d_h = ld;
    result.params.inductance_q_h = lq;
    result.params.inductance_avg_h = (ld + lq) / 2.0;
    result.params.inductance_diff_h = lq - ld;

    T::after_millis(500).await;

    // Step 3: Measure flux linkage — try spin-down (R-independent) first,
    // fall back to driven method if the motor decelerates too quickly.
    // Use openloop_erpm to set spin RPM, fall back to motor_size default
    let spin_rpm = if params.openloop_erpm > 0.0 {
        params.openloop_erpm / f32::from(params.pole_pairs)
    } else {
        params.motor_size.suggested_open_loop_erpm() / f32::from(params.pole_pairs)
    };
    let flux_params = FluxLinkageParams {
        motor_size: params.motor_size,
        resistance_ohm: result.params.resistance_ohm,
        pole_pairs: params.pole_pairs,
        spin_rpm,
        current_a: safe_current.min(2.0), // cap to safe level
        inductance_h: result.params.inductance_avg_h,
        // Ramp until the phase voltage reaches ~20% of vbus (VESC spins its
        // flux wizard to duty 0.3 ≈ the same), so back-EMF dominates R·I.
        v_target: 0.2 * params.vbus,
        ..Default::default()
    };
    info!("Step 3/4: Flux linkage measurement");
    result.params.flux_linkage_wb = measure_flux_linkage_auto::<H, T>(hw, &flux_params).await?;
    result.params.calculate_kv();

    // Calculate max current (needed below for the gain voltage limit)
    result.params.calculate_max_current(params.motor_size);

    // Step 4: Calculate PI gains
    info!("Step 4/4: PI auto-tuning");
    let mut bandwidth = estimate_bandwidth(result.params.inductance_avg_h, params.pwm_freq_hz);
    // Voltage limit: a worst-case current step of i_max must not demand
    // more than the bus can give (V ≈ Kp·ΔI, Kp = L·ω_bw). Without this a
    // high-R/high-L motor gets gains that slam the PI into saturation on
    // every transient.
    let i_max = result.params.max_current_a.min(params.current_max);
    let l_avg = result.params.inductance_avg_h;
    if i_max > 0.0 && l_avg > 0.0 {
        let bw_voltage_limit = (params.vbus * 0.577 / i_max) / l_avg;
        bandwidth = bandwidth.min(bw_voltage_limit);
    }
    if let Some(gains) = calculate_foc_gains(&result.params, bandwidth) {
        // Use average of d/q gains for simplicity
        result.kp_current = (gains.kp_d + gains.kp_q) / 2.0;
        result.ki_current = (gains.ki_d + gains.ki_q) / 2.0;
    }

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
        })
        .await;
        T::after_millis(u64::from(ramp_delay_ms)).await;
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
            })
            .await;

            // Wait for rotor to settle
            T::after_micros(u64::from(params.step_delay_us)).await;

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
        })
        .await;
        T::after_millis(u64::from(ramp_delay_ms)).await;
    }

    // Stop motor
    hw.send_command(ControlMode::Stopped).await;
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

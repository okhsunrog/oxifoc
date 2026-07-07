//! Per-section DWT cycle profiling for the shared ISR path (feature
//! `isr-profiling`, device-only — reads the Cortex-M cycle counter).
//!
//! All statics are cycle SUMS since the last reset; the platform's 1 Hz
//! stats task divides by its own ISR count and swap-resets them. With the
//! feature off, [`now`] is a constant and [`add`] a no-op, so the marks in
//! the shared code cost nothing on host builds.

use core::sync::atomic::AtomicU32;
#[cfg(feature = "isr-profiling")]
use core::sync::atomic::Ordering;

/// `run_foc_cycle`: command drain + deadman/link/fault gates.
pub static CYCLE_CMD: AtomicU32 = AtomicU32::new(0);
/// `run_foc_cycle`: `run_protection` (voltage/temp integrators, derating).
pub static CYCLE_PROT: AtomicU32 = AtomicU32::new(0);
/// `run_foc_cycle`: `FocDriver::step` (mode arm + phase update + control).
pub static CYCLE_STEP: AtomicU32 = AtomicU32::new(0);
/// `run_foc_cycle`: tail (per-phase OC check, hall bridge, state mirrors).
pub static CYCLE_TAIL: AtomicU32 = AtomicU32::new(0);
/// `step`, Stopped arm: platform `pwm.disable()`.
pub static STEP_PWMOFF: AtomicU32 = AtomicU32::new(0);
/// `step`, Stopped arm: `update_phase_with_prev_voltage` (estimators).
pub static STEP_PHASE: AtomicU32 = AtomicU32::new(0);
/// `step_current_control`: pre-work before the current loop (target clamps,
/// trust/derating/bus gates, `phase.get`, injection read, currents read).
pub static STEP_GATE: AtomicU32 = AtomicU32::new(0);
/// `step_current_control`: `FocController::step_with_injection` (Clarke/Park,
/// PI + decoupling, circular limit, inverse Park, dead-time comp, SVPWM).
pub static STEP_CTRL: AtomicU32 = AtomicU32::new(0);
/// `step_with_injection`: the `S::sin_cos` call — hardware CORDIC write +
/// result wait on device builds. Subset of [`STEP_CTRL`].
pub static CTRL_TRIG: AtomicU32 = AtomicU32::new(0);
/// `step_current_control`: post-work after the current loop (OC trip check,
/// bus-mod filter, PWM duty write, sensor duty feed).
pub static STEP_POST: AtomicU32 = AtomicU32::new(0);
/// `step_current_control`: `update_phase_with_prev_voltage` (phase manager:
/// observer flux integrator + PLL, startup machine, telemetry cache).
pub static STEP_EST: AtomicU32 = AtomicU32::new(0);
/// `PhaseManager::update`: `BackEmfObserver::update` (flux integrator,
/// atan2, PLL). Subset of [`STEP_EST`].
pub static EST_OBS: AtomicU32 = AtomicU32::new(0);
/// `PhaseManager::update`: the startup-sequencer block (deadshort feed or
/// `SensorlessStartup::tick` + |i| sqrt). Subset of [`STEP_EST`].
pub static EST_STARTUP: AtomicU32 = AtomicU32::new(0);
/// `PhaseManager::update`: `compute_phase_with_fallback` (source dispatch,
/// crossover blends, velocity, output cache). Subset of [`STEP_EST`].
pub static EST_OUT: AtomicU32 = AtomicU32::new(0);
/// `step_current_control` gate: `CurrentSensor::read_currents`. Subset of
/// [`STEP_GATE`].
pub static GATE_CURR: AtomicU32 = AtomicU32::new(0);
/// `compute_phase_with_fallback`: `tracker_output` alone. Subset of
/// [`EST_OUT`] — splits the phase-tracker math from the source-dispatch
/// glue around it (2026-07-07 drive-state cost forensics: EST_OUT runs
/// ~1 000 cy in ANY drive state while the tracker body is ~200
/// instructions — find where the other ~700 live).
pub static OUT_TRK: AtomicU32 = AtomicU32::new(0);
/// `BackEmfObserver::update`: the readiness tail (velocity-slope filter,
/// cached lag-compensated error, Schmitt latch). Subset of [`EST_OBS`] —
/// isolates the 2026-07-07 readiness additions (+280 cy A/B against
/// 87afa17, ~3× their arithmetic).
pub static OBS_TAIL: AtomicU32 = AtomicU32::new(0);

/// Current DWT cycle count (0 when profiling is compiled out).
#[inline(always)]
pub fn now() -> u32 {
    #[cfg(feature = "isr-profiling")]
    {
        cortex_m::peripheral::DWT::cycle_count()
    }
    #[cfg(not(feature = "isr-profiling"))]
    {
        0
    }
}

/// Accumulate the `t0..t1` span into `sum` (no-op when compiled out).
#[inline(always)]
pub fn add(sum: &AtomicU32, t0: u32, t1: u32) {
    #[cfg(feature = "isr-profiling")]
    sum.fetch_add(t1.wrapping_sub(t0), Ordering::Relaxed);
    #[cfg(not(feature = "isr-profiling"))]
    {
        let _ = (sum, t0, t1);
    }
}

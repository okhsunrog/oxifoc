//! Per-section DWT cycle profiling for the shared ISR path (feature
//! `isr-profiling`, device-only — reads the Cortex-M cycle counter).
//!
//! All statics are cycle SUMS since the last reset; the platform's 1 Hz
//! stats task divides by its own ISR count and swap-resets them. With the
//! feature off, [`now`](crate::isr_prof::now) is a constant and [`add`](crate::isr_prof::add) a no-op, so the marks in
//! the shared code cost nothing on host builds.

use core::sync::atomic::AtomicU32;
#[cfg(any(feature = "isr-profiling", feature = "runtime"))]
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
/// Due-only outer velocity update, including both torque-authority probes.
pub static VELOCITY_REQUEST: AtomicU32 = AtomicU32::new(0);
/// PhaseManager sensor sampling, Hall health and prediction before estimators.
pub static EST_SENSE: AtomicU32 = AtomicU32::new(0);
/// Applied-voltage selection/conversion before PhaseManager::update.
pub static EST_VOLTAGE: AtomicU32 = AtomicU32::new(0);
/// `BackEmfObserver::update`: the readiness tail (velocity-slope filter,
/// cached lag-compensated error, Schmitt latch). Subset of [`EST_OBS`] —
/// isolates the 2026-07-07 readiness additions (+280 cy A/B against
/// 87afa17, ~3× their arithmetic).
pub static OBS_TAIL: AtomicU32 = AtomicU32::new(0);

/// Named sections of one coherent profiling window. Nested sections (e.g.
/// trigonometry inside control) are subsets, not additional execution time.
#[cfg(feature = "runtime")]
#[derive(Clone, Copy)]
pub enum Section {
    Command,
    Protection,
    Step,
    Tail,
    PwmOff,
    StoppedPhase,
    Gate,
    Control,
    Trig,
    Post,
    Estimator,
    Observer,
    Startup,
    PhaseOutput,
    CurrentRead,
    PhaseTracker,
    VelocityRequest,
    SensorSampling,
    PhaseVoltage,
    // Keep last: adding a section above this also grows Snapshot's array.
    ObserverTail,
}

#[cfg(feature = "runtime")]
pub struct Snapshot {
    cycles: [u32; Section::ObserverTail as usize + 1],
}

#[cfg(feature = "runtime")]
impl Snapshot {
    fn counters() -> [&'static AtomicU32; Section::ObserverTail as usize + 1] {
        [
            &CYCLE_CMD,
            &CYCLE_PROT,
            &CYCLE_STEP,
            &CYCLE_TAIL,
            &STEP_PWMOFF,
            &STEP_PHASE,
            &STEP_GATE,
            &STEP_CTRL,
            &CTRL_TRIG,
            &STEP_POST,
            &STEP_EST,
            &EST_OBS,
            &EST_STARTUP,
            &EST_OUT,
            &GATE_CURR,
            &OUT_TRK,
            &VELOCITY_REQUEST,
            &EST_SENSE,
            &EST_VOLTAGE,
            &OBS_TAIL,
        ]
    }

    /// Non-destructive observation for an ISR-owned before/after pair.
    /// The caller must exclude both other writers and the reporting reset
    /// between observations. On the single-core target this is true within
    /// one highest-priority ADC handler; this is not a cross-task snapshot.
    pub fn observe() -> Self {
        Self {
            cycles: Self::counters().map(|counter| counter.load(Ordering::Relaxed)),
        }
    }

    /// Per-section increments, including counters that wrapped once.
    pub fn increments_since(&self, earlier: &Self) -> [u32; Section::ObserverTail as usize + 1] {
        core::array::from_fn(|i| self.cycles[i].wrapping_sub(earlier.cycles[i]))
    }

    /// The caller must capture its ISR invocation count and platform sections
    /// in this SAME critical section. Logging afterwards cannot split a window.
    pub fn take(_cs: critical_section::CriticalSection<'_>) -> Self {
        Self {
            cycles: Self::counters().map(|counter| counter.swap(0, Ordering::Relaxed)),
        }
    }

    pub fn average(&self, section: Section, samples: core::num::NonZeroU32) -> u32 {
        self.cycles[section as usize] / samples
    }
}

#[cfg(all(test, feature = "runtime"))]
mod tests {
    use super::*;
    use core::{num::NonZeroU32, sync::atomic::Ordering};

    #[test]
    fn snapshot_keeps_its_window_when_live_counters_advance() {
        // Only this test writes the global counters; observation arithmetic
        // below uses local snapshots so parallel tests cannot interfere.
        CYCLE_CMD.store(u32::MAX - 3, Ordering::Relaxed);
        let before = Snapshot::observe();
        CYCLE_CMD.store(8, Ordering::Relaxed);
        let after = Snapshot::observe();
        assert_eq!(
            after.increments_since(&before)[Section::Command as usize],
            12
        );
        assert_eq!(CYCLE_CMD.load(Ordering::Relaxed), 8);
        CYCLE_CMD.store(40, Ordering::Relaxed);
        STEP_CTRL.store(100, Ordering::Relaxed);
        VELOCITY_REQUEST.store(12, Ordering::Relaxed);
        let first = critical_section::with(Snapshot::take);
        CYCLE_CMD.store(80, Ordering::Relaxed);
        let second = critical_section::with(Snapshot::take);
        let samples = NonZeroU32::new(4).unwrap();
        assert_eq!(first.average(Section::Command, samples), 10);
        assert_eq!(first.average(Section::Control, samples), 25);
        assert_eq!(second.average(Section::Command, samples), 20);
        assert_eq!(second.average(Section::Control, samples), 0);
        assert_eq!(first.average(Section::VelocityRequest, samples), 3);
        assert_eq!(second.average(Section::VelocityRequest, samples), 0);
    }
}

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

/// Selected phase/outer-loop group, allowing native-rate live attribution
/// without enabling all nested ISR markers simultaneously.
#[inline(always)]
pub fn phase_now() -> u32 {
    #[cfg(any(feature = "isr-profiling", feature = "isr-profile-phase"))]
    {
        cortex_m::peripheral::DWT::cycle_count()
    }
    #[cfg(not(any(feature = "isr-profiling", feature = "isr-profile-phase")))]
    {
        0
    }
}

#[inline(always)]
pub fn phase_add(sum: &AtomicU32, t0: u32, t1: u32) {
    #[cfg(any(feature = "isr-profiling", feature = "isr-profile-phase"))]
    sum.fetch_add(t1.wrapping_sub(t0), core::sync::atomic::Ordering::Relaxed);
    #[cfg(not(any(feature = "isr-profiling", feature = "isr-profile-phase")))]
    {
        let _ = (sum, t0, t1);
    }
}

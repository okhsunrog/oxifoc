//! Shared Hall sensor state management for embassy-based platforms
//!
//! Provides the static `HALL_ESTIMATOR`, update/query functions, and
//! `HallAngleProxy` implementing `AngleSensor`. Platform crates keep
//! GPIO init, TIM6 setup, and the ISR itself — the ISR body simply
//! calls [`update_hall_state`] with the already-voted state.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::CriticalSectionMutex;

use super::hall_sensor::{Direction, HallSensor};
use super::sensors::{AngleSample, AngleSensor, HallSensorTrait, HallSnapshot};

// ========== Hall Estimator (shared state) ==========

/// Hall estimator — the single source of truth for Hall sensor state.
/// Accessed by TIM6 ISR (write via [`update_hall_state`]) and telemetry tasks (read).
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Initialization ==========

/// Create the Hall estimator in the global static.
///
/// Must be called once during platform init, before enabling the TIM6 interrupt.
///
/// # Arguments
/// * `timebase_ticks_per_sec` — Timebase frequency for Hall interpolation
///   (typically `embassy_time::TICK_HZ`).
pub fn init_estimator(timebase_ticks_per_sec: u64) {
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(timebase_ticks_per_sec)));
    });
}

// ========== ISR Entry Point ==========

/// Update Hall state from the platform ISR.
///
/// Call this from the TIM6 ISR after performing majority voting on GPIO reads.
/// Handles edge detection, timestamp capture, and estimator update internally.
///
/// Transitions into invalid states (0 and 7) are forwarded to the estimator
/// too: they bump its error counter and reset its edge tracking, which is
/// how a disconnected cable (pull-ups read 0b111 / 0b000) becomes visible to
/// the phase manager as `HallInvalidState` instead of being silently
/// swallowed here while the phase free-runs on the last velocity. VESC does
/// the same (invalid reading → `m_ang_hall_int_prev = -1` → fallback).
#[inline]
pub fn update_hall_state(state: u8) {
    // Edge detection via static mutable — safe because this is called only from a single ISR
    static mut LAST_STATE: u8 = 0;

    // SAFETY: called only from a single ISR context (TIM6_DAC)
    let last = unsafe { LAST_STATE };

    if state != last {
        let ticks = embassy_time::Instant::now().as_ticks();

        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(state, ticks);
            }
        });

        // SAFETY: called only from a single ISR context (TIM6_DAC)
        unsafe {
            LAST_STATE = state;
        }
    }
}

// ========== Public API for Telemetry ==========

/// Get current Hall sensor snapshot (for telemetry, polled at low rate).
pub fn get_snapshot(now_ticks: u64) -> Option<HallSnapshot> {
    HALL_ESTIMATOR.lock(|est| {
        est.borrow().as_ref().and_then(|h| {
            h.sample_at(now_ticks).map(|sample| HallSnapshot {
                angle_rad: sample.angle,
                velocity_rad_s: sample.omega,
                direction: sample.direction,
                state: h.logical_state(),
                error_count: h.error_count(),
            })
        })
    })
}

// ========== Apply Stored Config at Boot ==========

/// Apply stored Hall calibration and tuning parameters to the estimator.
///
/// Must be called after [`init_estimator`]. Skips silently if no config is stored.
#[cfg(feature = "storage")]
pub fn apply_stored_config(config: &crate::storage::RuntimeConfig) {
    HALL_ESTIMATOR.lock(|est| {
        if let Some(h) = est.borrow_mut().as_mut() {
            if let Some(ref cal) = config.hall_calibration
                && cal.is_calibrated()
            {
                h.set_calibration_raw(cal.angles);
                info!("Applied stored Hall calibration");
            }
            if let Some(ref tuning) = config.hall_tuning {
                h.set_interp_min_erpm(tuning.interp_min_erpm);
                h.set_drift_correction_gain(tuning.drift_correction_gain);
                h.set_rate_limit_factor(tuning.rate_limit_factor);
                h.set_timeout_us(tuning.timeout_us);
                info!("Applied stored Hall tuning params");
            }
        }
    });
}

// ========== Hall Angle Proxy for FOC ==========

/// Angle sensor proxy for the FOC driver; pulls snapshots from `HALL_ESTIMATOR`.
pub struct HallAngleProxy;

impl HallAngleProxy {
    pub const fn new() -> Self {
        Self
    }
}

impl AngleSensor for HallAngleProxy {
    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().and_then(|h| h.sample_at(now_ticks)))
    }

    fn read_angle(&self) -> f32 {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now).map(|s| s.angle).unwrap_or(0.0)
    }

    fn read_direction(&self) -> Direction {
        let now = embassy_time::Instant::now().as_ticks();
        self.sample(now)
            .map(|s| s.direction)
            .unwrap_or(Direction::Stopped)
    }

    fn error_count(&self) -> u32 {
        HALL_ESTIMATOR.lock(|est| est.borrow().as_ref().map(|h| h.error_count()).unwrap_or(0))
    }

    fn reset_errors(&mut self) {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                h.reset_errors();
            }
        });
    }
}

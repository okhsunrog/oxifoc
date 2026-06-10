//! Shared Hall sensor state management for embassy-based platforms
//!
//! Provides the static `HALL_ESTIMATOR`, update/query functions, and
//! `HallAngleProxy` implementing `AngleSensor`. Platform crates keep
//! GPIO/timer init and the capture ISR itself — the ISR body calls
//! [`update_hall_edge`] with the GPIO state and the hardware-latched
//! capture timestamp.
//!
//! All timestamps live in the hall capture timer's tick domain (µs on the
//! current boards), NOT `embassy_time` ticks. The platform registers its
//! tick source via [`set_tick_source`] so the convenience `AngleSensor`
//! methods that need "now" stay in the same domain as the edge timestamps.

use core::cell::{Cell, RefCell};

use embassy_sync::blocking_mutex::CriticalSectionMutex;

use super::hall_sensor::{Direction, HallSensor};
use super::sensors::{AngleSample, AngleSensor, HallSensorTrait, HallSnapshot};

// ========== Hall Estimator (shared state) ==========

/// Hall estimator — the single source of truth for Hall sensor state.
/// Accessed by the capture ISR (write via [`update_hall_edge`]) and telemetry tasks (read).
static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<Option<HallSensor>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Platform tick source for "now" in the hall tick domain, registered at init.
/// Used by the convenience `AngleSensor` methods (`read_angle`/`read_direction`);
/// the control path always receives explicit ticks from the FOC ISR.
static TICK_SOURCE: CriticalSectionMutex<Cell<Option<fn() -> u64>>> =
    CriticalSectionMutex::new(Cell::new(None));

// ========== Initialization ==========

/// Create the Hall estimator in the global static.
///
/// Must be called once during platform init, before enabling the capture interrupt.
///
/// # Arguments
/// * `timebase_ticks_per_sec` — Tick rate of the hall capture timer
///   (1 MHz on the current boards).
pub fn init_estimator(timebase_ticks_per_sec: u64) {
    HALL_ESTIMATOR.lock(|est| {
        est.replace(Some(HallSensor::new(timebase_ticks_per_sec)));
    });
}

/// Register the platform's hall-domain tick source ("now" in the same ticks
/// as the edge timestamps fed to [`update_hall_edge`]).
pub fn set_tick_source(f: fn() -> u64) {
    TICK_SOURCE.lock(|c| c.set(Some(f)));
}

fn hall_now_ticks() -> u64 {
    TICK_SOURCE.lock(|c| c.get()).map(|f| f()).unwrap_or(0)
}

// ========== ISR Entry Point ==========

/// Feed one hall edge from the platform capture ISR.
///
/// `ticks` is the hardware-latched capture timestamp (extended to 64 bits),
/// so ISR latency does not affect edge timing. Same-state calls are ignored:
/// a glitch that bounces through the input filter produces two captures that
/// land back on the previous state.
///
/// Transitions into invalid states (0 and 7) are forwarded to the estimator
/// too: they bump its error counter and reset its edge tracking, which is
/// how a disconnected cable (pull-ups read 0b111 / 0b000) becomes visible to
/// the phase manager as `HallInvalidState` instead of being silently
/// swallowed here while the phase free-runs on the last velocity. VESC does
/// the same (invalid reading → `m_ang_hall_int_prev = -1` → fallback).
#[inline]
pub fn update_hall_edge(state: u8, ticks: u64) {
    // Edge memory via static mutable — safe because this is called only from
    // a single ISR (the hall capture timer's).
    static mut LAST_STATE: u8 = 0;

    // SAFETY: called only from a single ISR context
    let last = unsafe { LAST_STATE };

    if state != last {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().as_mut() {
                let _ = h.update_sample(state, ticks);
            }
        });

        // SAFETY: called only from a single ISR context
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

    // The trait defaults would silently drop the estimator's stateful
    // behavior: sample_mut carries the VESC-style rate limiter (smooths
    // sector-edge angle jumps for the control path) and is_stale the
    // velocity-adaptive dead-sensor detection that drives the HallTimeout
    // fault + observer fallback. Both must reach the shared estimator.
    fn sample_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow_mut()
                .as_mut()
                .and_then(|h| h.sample_at_mut(now_ticks))
        })
    }

    fn is_stale(&self, now_ticks: u64) -> bool {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow()
                .as_ref()
                .is_some_and(|h| h.is_stale_at_speed(now_ticks))
        })
    }

    fn read_angle(&self) -> f32 {
        self.sample(hall_now_ticks())
            .map(|s| s.angle)
            .unwrap_or(0.0)
    }

    fn read_direction(&self) -> Direction {
        self.sample(hall_now_ticks())
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

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

use crate::foc::hall_sensor::HallFaultKind;
#[cfg(feature = "storage")]
use crate::storage::RuntimeConfig;
use core::cell::{Cell, RefCell};

use embassy_sync::blocking_mutex::CriticalSectionMutex;

use super::hall_sensor::{Direction, HallSensor, PreparedHallCalibration};
use super::sensors::{AngleSample, AngleSensor, ControlAngleSample, HallSnapshot};

// ========== Hall Estimator (shared state) ==========

/// Hall estimator — the single source of truth for Hall sensor state.
/// Accessed by the capture ISR (write via [`update_hall_edge`]) and telemetry tasks (read).
struct HallEstimatorState {
    sensor: Option<HallSensor>,
    reference_epoch: u32,
}

static HALL_ESTIMATOR: CriticalSectionMutex<RefCell<HallEstimatorState>> =
    CriticalSectionMutex::new(RefCell::new(HallEstimatorState {
        sensor: None,
        reference_epoch: 0,
    }));

/// Platform tick source for "now" in the hall tick domain, registered at init.
type TickSourceFn = fn() -> u64;

/// Used by the convenience `AngleSensor` methods (`read_angle`/`read_direction`);
/// the control path always receives explicit ticks from the FOC ISR.
static TICK_SOURCE: CriticalSectionMutex<Cell<Option<TickSourceFn>>> =
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
        let mut state = est.borrow_mut();
        state.sensor = Some(HallSensor::new(timebase_ticks_per_sec));
        state.reference_epoch = state.reference_epoch.wrapping_add(1);
    });
}

/// Register the platform's hall-domain tick source ("now" in the same ticks
/// as the edge timestamps fed to [`update_hall_edge`]).
pub fn set_tick_source(f: fn() -> u64) {
    TICK_SOURCE.lock(|c| c.set(Some(f)));
}

fn hall_now_ticks() -> u64 {
    TICK_SOURCE.lock(Cell::get).map(|f| f()).unwrap_or(0)
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
    // a single ISR (the hall capture timer's). 0xFF = "no edge seen yet":
    // 0 is also a real (invalid, all-low) reading, and using it as the
    // sentinel swallowed a first edge into state 0 — the estimator never
    // learned of it and its error counter stayed flat.
    const NO_EDGE_YET: u8 = 0xFF;
    static mut LAST_STATE: u8 = NO_EDGE_YET;

    // SAFETY: called only from a single ISR context
    let last = unsafe { LAST_STATE };

    if state != last {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().sensor.as_mut() {
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
        est.borrow()
            .sensor
            .as_ref()
            .and_then(|h| h.snapshot_at(now_ticks))
    })
}

// ========== Apply Stored Config at Boot ==========

/// Apply a successful detection result to the live estimator without a reboot.
/// This changes RAM only; persistence remains an explicit host operation.
#[cfg(feature = "detection")]
pub fn apply_detected_calibration(result: &super::hall_calibration::HallCalibrationResult) -> bool {
    let Some(prepared) = PreparedHallCalibration::from_result(result) else {
        return false;
    };
    apply_prepared_calibration(&prepared).is_some()
}

fn apply_prepared_calibration(
    prepared: &PreparedHallCalibration,
) -> Option<super::sensors::HallReferenceUpdate> {
    HALL_ESTIMATOR.lock(|est| {
        let mut state = est.borrow_mut();
        let sensor = state.sensor.as_mut()?;
        let update = sensor.install_prepared_calibration(prepared);
        state.reference_epoch = state.reference_epoch.wrapping_add(1);
        Some(update)
    })
}

/// Apply stored Hall calibration and tuning parameters to the estimator.
///
/// Must be called after [`init_estimator`]. Skips silently if no config is stored.
#[cfg(feature = "storage")]
pub fn apply_stored_config(config: &RuntimeConfig) {
    let prepared = config
        .hall_calibration
        .as_ref()
        .and_then(PreparedHallCalibration::from_config);
    HALL_ESTIMATOR.lock(|est| {
        let mut state = est.borrow_mut();
        let mut changed = false;
        if let Some(h) = state.sensor.as_mut() {
            if let Some(prepared) = &prepared {
                h.install_prepared_calibration(prepared);
                changed = true;
                info!("Applied stored Hall calibration");
            }
            if let Some(ref tuning) = config.hall_tuning
                && tuning.is_sane()
            {
                h.set_interp_min_erpm(tuning.interp_min_erpm);
                h.set_rate_limit_factor(tuning.rate_limit_factor);
                changed = true;
                info!("Applied stored Hall tuning params");
            }
        }
        if changed {
            state.reference_epoch = state.reference_epoch.wrapping_add(1);
        }
    });
}

// ========== Hall Angle Proxy for FOC ==========

/// Angle sensor proxy for the FOC driver; pulls snapshots from `HALL_ESTIMATOR`.
#[derive(Default)]
pub struct HallAngleProxy;

impl HallAngleProxy {
    pub const fn new() -> Self {
        Self
    }
}

impl AngleSensor for HallAngleProxy {
    fn apply_prepared_hall_calibration(
        &mut self,
        prepared: &PreparedHallCalibration,
    ) -> Option<super::sensors::HallReferenceUpdate> {
        apply_prepared_calibration(prepared)
    }

    fn sample(&self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow()
                .sensor
                .as_ref()
                .and_then(|h| h.sample_at(now_ticks))
        })
    }

    // The trait defaults would silently drop the estimator's stateful
    // behavior: sample_mut carries the VESC-style rate limiter (smooths
    // sector-edge angle jumps for the control path) and is_stale the
    // velocity-adaptive dead-sensor detection that drives the HallTimeout
    // fault + observer fallback. Both must reach the shared estimator.
    fn sample_mut(&mut self, now_ticks: u64) -> Option<AngleSample> {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow_mut()
                .sensor
                .as_mut()
                .and_then(|h| h.sample_at_mut(now_ticks))
        })
    }

    fn sample_for_control(&mut self, now_ticks: u64) -> Option<ControlAngleSample> {
        HALL_ESTIMATOR.lock(|est| {
            let mut state = est.borrow_mut();
            let reference_epoch = state.reference_epoch;
            state
                .sensor
                .as_mut()
                .and_then(|h| h.sample_at_mut(now_ticks))
                .map(|sample| ControlAngleSample {
                    sample,
                    reference_epoch,
                })
        })
    }

    fn is_stale(&self, now_ticks: u64, omega_ref: f32) -> bool {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow()
                .sensor
                .as_ref()
                .is_some_and(|h| h.is_stale_at_speed(now_ticks, omega_ref))
        })
    }

    fn hall_edge_observation(&self) -> Option<super::sensors::HallEdgeObservation> {
        HALL_ESTIMATOR.lock(|est| {
            let state = est.borrow();
            let mut edge = state.sensor.as_ref()?.hall_edge_observation()?;
            edge.reference_epoch = state.reference_epoch;
            Some(edge)
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
        HALL_ESTIMATOR.lock(|est| {
            est.borrow()
                .sensor
                .as_ref()
                .map(HallSensor::error_count)
                .unwrap_or(0)
        })
    }

    fn reset_errors(&mut self) {
        HALL_ESTIMATOR.lock(|est| {
            if let Some(h) = est.borrow_mut().sensor.as_mut() {
                h.reset_errors();
            }
        });
    }

    /// Present once the platform created the estimator (`init_estimator`),
    /// i.e. the hall hardware is wired up — even before the first edge.
    fn is_present(&self) -> bool {
        HALL_ESTIMATOR.lock(|est| est.borrow().sensor.is_some())
    }

    // The trait default (None) would hide the shared estimator's wire
    // verdicts from the fault bridge, same trap as sample_mut/is_stale.
    fn fault_kind(&self) -> Option<HallFaultKind> {
        HALL_ESTIMATOR.lock(|est| {
            est.borrow()
                .sensor
                .as_ref()
                .and_then(HallSensor::fault_kind)
        })
    }
}

#[cfg(all(test, feature = "detection"))]
mod tests {
    use super::*;
    use crate::foc::hall_calibration::HallCalibrationResult;
    use crate::foc::hall_sensor::HALL_STATE_TABLE;
    use crate::foc::phase::{PhaseInput, PhaseManager, PhaseProvider};

    // This is the only host test that owns the shared estimator. Exercise the
    // same live-apply API as embassy_hw::calibrate_hall, rather than a direct
    // PhaseManager setter that hardware proxies never call.
    #[test]
    fn live_proxy_calibration_epochs_reset_control_motion_only_after_success() {
        init_estimator(1_000_000);
        update_hall_edge(1, 0);
        let mut manager = PhaseManager::with_hall(HallAngleProxy::new());
        let input = PhaseInput {
            dt: 0.00005,
            ..Default::default()
        };
        let sequence = [1, 3, 2, 6, 4, 5];
        for step in 0..=600_u64 {
            if step.is_multiple_of(100) {
                let index = usize::try_from((step / 100) % 6).unwrap();
                update_hall_edge(sequence[index], step * 50);
            }
            manager.update(&input, step * 50);
        }
        assert!(manager.get().velocity > 10.0);
        let mut proxy = HallAngleProxy::new();
        let epoch = proxy.sample_for_control(30_000).unwrap().reference_epoch;
        let mut calibration = HallCalibrationResult {
            geometry: None,
            angles: [0.0; 8],
            valid: [false; 8],
            valid_count: 0,
        };
        assert!(!apply_detected_calibration(&calibration));
        assert_eq!(
            proxy.sample_for_control(30_000).unwrap().reference_epoch,
            epoch
        );
        manager.update(&input, 30_050);
        assert!(
            manager.get().velocity > 10.0,
            "rejected calibration keeps motion history"
        );

        calibration.valid = [false, true, true, true, true, true, true, false];
        calibration.valid_count = 6;
        for (angle, logical) in calibration.angles[1..7]
            .iter_mut()
            .zip(&HALL_STATE_TABLE[1..7])
        {
            *angle = f32::from(*logical) * core::f32::consts::TAU / 6.0 + 0.3;
        }
        assert!(apply_detected_calibration(&calibration));
        let frame = proxy.sample_for_control(30_100).unwrap();
        assert_eq!(frame.reference_epoch, epoch.wrapping_add(1));
        manager.update(&input, 30_100);
        assert_eq!(
            manager.get().velocity,
            0.0,
            "new table starts a fresh derivative"
        );

        // An epoch change while samples are unavailable must survive the
        // fallback: the first recovered frame still cold-resets the filter.
        update_hall_edge(7, 30_150);
        assert!(apply_detected_calibration(&calibration));
        manager.update(&input, 30_150);
        assert!(proxy.sample_for_control(30_150).is_none());
        update_hall_edge(1, 30_200);
        manager.update(&input, 30_200);
        assert_eq!(manager.get().velocity, 0.0);

        HALL_ESTIMATOR.lock(|est| est.borrow_mut().reference_epoch = u32::MAX);
        assert!(apply_detected_calibration(&calibration));
        assert_eq!(proxy.sample_for_control(30_250).unwrap().reference_epoch, 0);
        manager.update(&input, 30_250);
        assert_eq!(
            manager.get().velocity,
            0.0,
            "epoch rollover is a reference change too"
        );
        let prepared = PreparedHallCalibration::from_result(&calibration).unwrap();
        assert!(proxy.apply_prepared_hall_calibration(&prepared).is_some());
        assert_eq!(proxy.sample_for_control(30_300).unwrap().reference_epoch, 1);
        assert_eq!(proxy.sample(30_300).unwrap().omega, 0.0);
        manager.update(&input, 30_300);
        assert_eq!(manager.get().velocity, 0.0);
        HALL_ESTIMATOR.lock(|est| est.borrow_mut().sensor = None);
        assert!(proxy.apply_prepared_hall_calibration(&prepared).is_none());
        assert_eq!(HALL_ESTIMATOR.lock(|est| est.borrow().reference_epoch), 1);
        #[cfg(feature = "storage")]
        {
            let stored = crate::storage::HallCalibrationConfig {
                angles: calibration.angles,
                valid: calibration.valid,
                geometry: None,
            };
            let config = RuntimeConfig {
                hall_calibration: Some(stored),
                ..Default::default()
            };
            init_estimator(1_000_000);
            apply_stored_config(&config);
            update_hall_edge(3, 40_000);
            let boot = proxy.sample(40_000).unwrap();
            let prepared =
                PreparedHallCalibration::from_config(config.hall_calibration.as_ref().unwrap())
                    .unwrap();
            assert!(proxy.apply_prepared_hall_calibration(&prepared).is_some());
            let live = proxy.sample(40_000).unwrap();
            assert_eq!(
                boot.angle.to_bits(),
                live.angle.to_bits(),
                "boot and live share the same installed reference"
            );
            assert_eq!(live.omega, 0.0);
        }
    }
}

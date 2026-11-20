//! Session-scoped motor measurements with bounded replay state.
//!
//! The task services status/cancel/retry requests while a measurement future
//! runs. Admission is owned by `session`, independent of actuator execution.
//! Terminal outcomes are published only after ISR cleanup. Session IDs include
//! a platform-provided random boot epoch; retired identities never re-energize.
//! Hosts retain orchestration of R/L/flux/Hall and may run each stage separately.

use crate::drive_setup::{DriveCurrent, DriveSetup};
use crate::foc::detection::flux_linkage;
use crate::state;
use crate::timer::Timer;
use crate::types::commissioning::{CommissioningPlan, SourceLimits};
use crate::types::measurement::SessionId;
use crate::types::{ConfigGroupId, SaveDriveSetup};
use core::cell::RefCell;

pub mod operations;
mod session;

use critical_section::Mutex as CriticalSectionMutex;
use ergot::net_stack::NetStackHandle;
use ergot::net_stack::endpoints::Endpoints;

#[cfg(feature = "offset-diagnostics")]
use crate::foc::current_offset::{
    CurrentOffsetError, CurrentOffsetMethod, CurrentOffsetReport, CurrentOffsetRequest,
};
use crate::foc::detection::types::{
    DetectionCurrents, DetectionError, FluxLinkageParams, InductanceParams, MotorSize,
    ResistanceParams,
};
use crate::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use crate::icd::DetectEndpoint;
use crate::motor::foc_driver::{PowerSourceLimits, PreparedCommissioning};
use crate::state::DetectionActiveGuard;
use crate::state::model::CONFIG_COORDINATOR;
use crate::storage::{HallCalibrationConfig, RuntimeConfig};
use crate::types::{DetectError, DetectRequest, DetectResponse};
use crate::units::{
    ConstZero, ElectricCurrent, ElectricPotential, Electrical, ElectricalResistance, Frequency,
    Inductance, MagneticFlux, Sign, amps,
};
use uom::si::{Dimension, Quantity, Units};

struct ReadySetup {
    session: SessionId,
    revisions: [u32; ConfigGroupId::COUNT],
    setup: DriveSetup,
}
static READY_SETUP: CriticalSectionMutex<RefCell<Option<ReadySetup>>> =
    CriticalSectionMutex::new(RefCell::new(None));

pub(super) fn setup_is_current(request: &SaveDriveSetup) -> bool {
    critical_section::with(|cs| {
        let ready = READY_SETUP.borrow(cs).borrow();
        ready.as_ref().is_some_and(|ready| {
            let mut offered = request.setup.clone();
            offered.current = DriveCurrent::Auto;
            offered.motor.max_current.value = 0.0;
            ready.session == request.session
                && offered == ready.setup
                && ready.revisions == *CONFIG_COORDINATOR.revisions.borrow(cs).borrow()
        })
    })
}

pub(super) fn invalidate_setup_candidate() {
    critical_section::with(|cs| *READY_SETUP.borrow(cs).borrow_mut() = None);
}

/// The platform-specific half of detection: the raw measurements, bound to the
/// hardware (or to the virtual-motor sim). Everything else — parameter
/// computation, dedup, error mapping, candidate validation — is handled by
/// [`detect_server`].
#[allow(async_fn_in_trait)]
pub trait DetectionBackend {
    /// Wall-clock timer for the bounded ISR installation acknowledgement.
    /// Simulation's instantaneous measurement clock is unsuitable here.
    type Timer: Timer;

    /// Present DC bus voltage (read from the platform ADC / sim).
    fn vbus(&self) -> ElectricPotential;

    /// Select the stage timebase while the commissioning lease owns the
    /// stopped actuator. Platforms with fixed timing use the boot rate.
    async fn prepare_timing(
        &mut self,
        _request: &DetectRequest,
        drive_foc: Frequency,
    ) -> Result<Frequency, DetectError> {
        Ok(drive_foc)
    }

    /// Restore the drive timebase after the ISR has disarmed commissioning.
    async fn restore_timing(&mut self) -> bool {
        true
    }

    /// Measure phase-to-neutral resistance.
    async fn measure_resistance(
        &mut self,
        params: &ResistanceParams,
    ) -> Result<ElectricalResistance, DetectionError>;

    /// Measure d/q-axis inductance via rotating HFI.
    async fn measure_inductance(
        &mut self,
        params: &InductanceParams,
        pwm_freq: Frequency,
    ) -> Result<(Inductance, Inductance), DetectionError>;

    /// Measure flux linkage via open-loop spin.
    async fn measure_flux(
        &mut self,
        params: &FluxLinkageParams,
    ) -> Result<MagneticFlux, DetectionError>;

    /// Measure Hall sensors without installing the result.
    async fn calibrate_hall(
        &mut self,
        params: HallCalibrationParams,
    ) -> Result<HallCalibrationResult, DetectionError>;

    /// Measure current-sensor offsets through the platform's ISR-owned driver
    /// (or return a deterministic equivalent in simulation).
    #[cfg(feature = "offset-diagnostics")]
    async fn measure_current_offsets(
        &mut self,
        request: CurrentOffsetRequest,
    ) -> Result<CurrentOffsetReport, CurrentOffsetError>;
}

/// Serve `DetectEndpoint` with effectively-once dedup, forever.
///
/// `max_current` is the per-platform current ceiling fed into the safe-test-
/// current formulas. Inverter protection and an optional diagnostic ceiling
/// further bound it. Finite source limits must be configured explicitly.
/// `foc_freq` is the PWM/FOC loop frequency (used by the inductance HFI).
/// Hall calibration returns candidate data without changing the installed setup.
pub async fn detect_server<NS, B>(
    endpoints: Endpoints<NS>,
    mut backend: B,
    boot_epoch: u64,
    max_current: ElectricCurrent,
    foc_freq: Frequency,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
) where
    NS: NetStackHandle + Clone,
    B: DetectionBackend,
{
    use crate::types::measurement::Outcome;
    use embassy_futures::select::{Either, select};
    let mut session = session::Session::new(boot_epoch);
    let server = endpoints
        .clone()
        .bounded_server::<DetectEndpoint, 4>(Some("detect"));
    let server = core::pin::pin!(server);
    let mut h = server.attach();
    loop {
        let Ok(msg) = h.recv_manual().await else {
            continue;
        };
        let dispatch = session.handle_with_revisions(msg.t, config_revisions());
        session.publish_candidate();
        let _ = endpoints
            .clone()
            .respond_owned::<DetectEndpoint>(&msg.hdr, &dispatch.reply);
        let Some(measurement) = dispatch.start else {
            continue;
        };
        let Some(guard) = DetectionActiveGuard::try_arm() else {
            session.finish(measurement.id, Outcome::Failed(DetectError::Busy));
            continue;
        };
        let plan = session.plan();
        let operation = async {
            let result = match select(
                run_step(
                    &mut backend,
                    measurement.request,
                    max_current,
                    foc_freq,
                    runtime_config,
                    &guard,
                    plan,
                ),
                <B::Timer as Timer>::after_millis(65_000),
            )
            .await
            {
                Either::First(result) => result,
                Either::Second(()) => {
                    guard.cancel();
                    DetectResponse::Error(DetectError::Timeout)
                }
            };
            // Do not publish a terminal outcome before ISR cleanup. If the ISR
            // is unavailable, retain ownership; the hardware watchdog remains
            // responsible for a hung controller.
            while !guard.end::<B::Timer>().await {
                guard.cancel();
            }
            // Hold the lease through restoration, including after timeout or
            // cancellation. Never advertise completion with the stage rate.
            while !backend.restore_timing().await {
                guard.cancel();
                <B::Timer as Timer>::after_millis(1).await;
            }
            if state::detection_cancelled()
                && !matches!(
                    result,
                    DetectResponse::Error(DetectError::SourceLimited | DetectError::Timeout)
                )
            {
                Outcome::Cancelled
            } else {
                match result {
                    DetectResponse::Error(e) => Outcome::Failed(e),
                    value => Outcome::Completed(value),
                }
            }
        };
        let mut operation = core::pin::pin!(operation);
        loop {
            match select(operation.as_mut(), h.recv_manual()).await {
                Either::First(outcome) => {
                    session.finish(measurement.id, outcome);
                    session.publish_candidate();
                    break;
                }
                Either::Second(Ok(msg)) => {
                    let dispatch = session.handle_with_revisions(msg.t, config_revisions());
                    if dispatch.cancel_active {
                        guard.cancel();
                    }
                    // The ledger never admits another operation while cleanup
                    // is outstanding. Replies remain available during a sweep.
                    debug_assert!(dispatch.start.is_none());
                    let _ = endpoints
                        .clone()
                        .respond_owned::<DetectEndpoint>(&msg.hdr, &dispatch.reply);
                }
                Either::Second(Err(_)) => {}
            }
        }
    }
}

/// Positive-and-finite gate for detection request payloads. Wire input is
/// arbitrary bits and the safe-current formulas are NaN-transparent —
/// `f32::min` discards a NaN operand, so `sqrt(NaN).min(max_current)`
/// SELECTS the platform's maximum test current instead of failing.
#[inline]
fn pos<D, U>(v: Quantity<D, U, f32>) -> bool
where
    D: Dimension + ?Sized,
    U: Units<f32> + ?Sized,
{
    v.is_finite() && v.is_positive()
}

/// Validate a physical resistance result before it reaches current-limit math.
/// Backend implementations are a trust boundary just like wire requests: a
/// non-finite result would otherwise make `f32::min` select the platform limit.
#[inline]
fn measured_resistance(r: ElectricalResistance) -> Result<ElectricalResistance, DetectError> {
    if pos(r) {
        Ok(r)
    } else {
        Err(DetectError::HardwareFault)
    }
}

/// Run a single detection step: compute safe parameters, call the backend, map
/// the result. Kept as one concrete (non-closure) `async fn` so the enclosing
/// server future stays `Send`.
async fn run_step<B: DetectionBackend>(
    backend: &mut B,
    req: DetectRequest,
    max_current: ElectricCurrent,
    foc_freq: Frequency,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
    detection: &DetectionActiveGuard,
    plan: Option<CommissioningPlan>,
) -> DetectResponse {
    if runtime_config.is_some_and(|config| {
        critical_section::with(|cs| !config.borrow(cs).borrow().board_calibration_is_valid())
    }) {
        return DetectResponse::Error(DetectError::Unsafe);
    }
    if matches!(req, DetectRequest::MeasureCurrentOffsets { .. }) {
        return measure_step(backend, req, max_current, foc_freq).await;
    }
    let prepared = match runtime_config {
        Some(config) => critical_section::with(|cs| {
            prepare_commissioning(
                max_current,
                Some(&config.borrow(cs).borrow()),
                req.current_limit(),
                plan.map(|p| p.source),
            )
        }),
        None => Err(DetectError::Unsafe),
    };
    let prepared = match prepared {
        Ok(prepared) => prepared,
        Err(error) => return DetectResponse::Error(error),
    };
    let foc_freq = match backend.prepare_timing(&req, foc_freq).await {
        Ok(freq) => freq,
        Err(error) => return DetectResponse::Error(error),
    };
    if !detection.begin::<B::Timer>(prepared).await {
        return DetectResponse::Error(DetectError::Busy);
    }
    let result = measure_step(backend, req, prepared.current_ceiling(), foc_freq).await;
    let limited = state::detection_source_limited();
    if !detection.end::<B::Timer>().await {
        return DetectResponse::Error(DetectError::HardwareFault);
    }
    if limited {
        DetectResponse::Error(DetectError::SourceLimited)
    } else if state::detection_cancelled() {
        DetectResponse::Error(DetectError::HardwareFault)
    } else if matches!(result, DetectResponse::Error(_)) && state::detection_source_constrained() {
        DetectResponse::Error(DetectError::SourceLimited)
    } else {
        result
    }
}

async fn measure_step<B: DetectionBackend>(
    backend: &mut B,
    req: DetectRequest,
    max_current: ElectricCurrent,
    foc_freq: Frequency,
) -> DetectResponse {
    match req {
        DetectRequest::MeasureResistance { max_power_loss, .. } => {
            if !pos(max_power_loss) {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            let motor_size = MotorSize::Custom(max_power_loss);
            // Probe at low current to find a safe test current, then measure.
            let probe = ResistanceParams {
                motor_size,
                current_max: (max_current / 50.0).max(amps(0.5)),
                num_samples: 20,
                ramp_time_ms: 200,
                settle_time_ms: 100,
                ..Default::default()
            };
            let r_probe = match backend.measure_resistance(&probe).await {
                Ok(r) => r,
                Err(e) => return DetectResponse::Error(map_err(e)),
            };
            let r_probe = match measured_resistance(r_probe) {
                Ok(r) => r,
                Err(e) => return DetectResponse::Error(e),
            };
            let currents = match DetectionCurrents::new(
                max_power_loss,
                r_probe,
                backend.vbus(),
                max_current,
            ) {
                Ok(currents) if currents.resistance >= amps(0.5) => currents,
                Ok(_) => return DetectResponse::Error(DetectError::OutOfRange),
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            let params = ResistanceParams {
                motor_size,
                current_max: currents.resistance,
                ..Default::default()
            };
            match backend.measure_resistance(&params).await {
                Ok(r) => match measured_resistance(r) {
                    Ok(resistance) => DetectResponse::Resistance { resistance },
                    Err(e) => DetectResponse::Error(e),
                },
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureInductance {
            max_power_loss,
            resistance,
            ..
        } => {
            if !pos(max_power_loss) || !pos(resistance) {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            let params = match InductanceParams::commissioning(
                MotorSize::Custom(max_power_loss),
                resistance,
                backend.vbus(),
                max_current,
            ) {
                Ok(params) => params,
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            match backend.measure_inductance(&params, foc_freq).await {
                Ok((inductance_d, inductance_q)) => DetectResponse::Inductance {
                    inductance_d,
                    inductance_q,
                },
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureFlux {
            max_power_loss,
            resistance,
            inductance,
            pole_pairs,
            openloop_speed,
            ..
        } => {
            // pole_pairs = 0 would turn the open-loop speed into an infinite
            // mechanical spin target.
            if !pos(max_power_loss)
                || !pos(resistance)
                || !pos(inductance)
                || pole_pairs == 0
                || !pos(openloop_speed)
            {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            let currents = match DetectionCurrents::new(
                max_power_loss,
                resistance,
                backend.vbus(),
                max_current,
            ) {
                Ok(currents) => currents,
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            let params = FluxLinkageParams {
                motor_size: MotorSize::Custom(max_power_loss),
                resistance,
                inductance,
                pole_pairs,
                spin_speed: openloop_speed.to_mechanical(pole_pairs),
                current: currents.flux,
                // Ramp until back-EMF dominates R·I (VESC's duty-0.3 idea);
                // spin_speed above remains the hard speed cap.
                v_target: 0.2 * backend.vbus(),
                ..Default::default()
            };
            match backend.measure_flux(&params).await {
                Ok(flux_linkage) => {
                    // Line-to-line Kv (carries the √3 phase→line factor — see
                    // flux_linkage::calculate_kv); inlining it here is what kept
                    // the detection result √3-high.
                    let kv_rpm_per_v = if flux_linkage.is_positive() {
                        flux_linkage::calculate_kv(flux_linkage, pole_pairs)
                    } else {
                        0.0
                    };
                    DetectResponse::FluxLinkage {
                        flux_linkage,
                        kv_rpm_per_v,
                    }
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::CalibrateHall {
            max_power_loss,
            resistance,
            ..
        } => {
            let currents = match DetectionCurrents::new(
                max_power_loss,
                resistance,
                backend.vbus(),
                max_current,
            ) {
                Ok(currents) => currents,
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            let params = HallCalibrationParams {
                current: currents.hall,
                ..Default::default()
            };
            match backend.calibrate_hall(params).await {
                Ok(result) => {
                    let calibration = HallCalibrationConfig::from(&result);
                    if !result.is_valid() || !calibration.is_calibrated() {
                        return DetectResponse::Error(DetectError::HardwareFault);
                    }
                    DetectResponse::HallCalibrated(calibration)
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureCurrentOffsets {
            method,
            samples_per_channel,
            stationary_confirmed,
        } => {
            #[cfg(not(feature = "offset-diagnostics"))]
            {
                let _ = (method, samples_per_channel, stationary_confirmed);
                return DetectResponse::Error(DetectError::MissingPrerequisite);
            }
            #[cfg(feature = "offset-diagnostics")]
            {
                if method == CurrentOffsetMethod::AllPhases50 && !stationary_confirmed {
                    return DetectResponse::Error(DetectError::Unsafe);
                }
                let request = CurrentOffsetRequest::diagnostic(
                    method,
                    samples_per_channel,
                    stationary_confirmed,
                );
                if !request.is_valid() {
                    return DetectResponse::Error(DetectError::OutOfRange);
                }
                match backend.measure_current_offsets(request).await {
                    Ok(report) => DetectResponse::CurrentOffsets(report),
                    Err(error) => DetectResponse::Error(map_current_offset_err(error)),
                }
            }
        }
    }
}

/// Prepare from inverter and source constraints only. The installed motor's
/// thermal rating and ride-current settings have no authority over detection.
/// Wizard sessions supply confirmed limits; individual diagnostics retain the
/// explicit finite source limits configured on the device.
fn prepare_commissioning(
    platform_max: ElectricCurrent,
    config: Option<&RuntimeConfig>,
    test_ceiling: Option<ElectricCurrent>,
    source: Option<SourceLimits>,
) -> Result<PreparedCommissioning, DetectError> {
    let limits = config.and_then(|c| c.current_limits.as_ref());
    let source = source
        .or_else(|| {
            limits.map(|limits| SourceLimits {
                input: limits.bus_in_max,
                regen: limits.bus_regen_max,
            })
        })
        .ok_or(DetectError::Unsafe)?;
    let source =
        PowerSourceLimits::new(source.input, source.regen).map_err(|_| DetectError::Unsafe)?;
    // A fresh board has no user override. Its independent platform trip still
    // applies; an explicitly stored stricter protection always survives.
    let configured_trip = limits.map_or(ElectricCurrent::ZERO, |limits| limits.max_phase_current);
    if !configured_trip.is_finite() || configured_trip.is_negative() {
        return Err(DetectError::Unsafe);
    }
    let trip = if configured_trip == ElectricCurrent::ZERO {
        platform_max
    } else {
        configured_trip
    };
    let prepared = PreparedCommissioning::new(platform_max, trip, source, test_ceiling)
        .map_err(|_| DetectError::Unsafe)?;
    if prepared.current_ceiling() < amps(0.5) {
        return Err(DetectError::OutOfRange);
    }
    Ok(prepared)
}

#[cfg(feature = "offset-diagnostics")]
fn map_current_offset_err(error: CurrentOffsetError) -> DetectError {
    match error {
        CurrentOffsetError::Busy => DetectError::Busy,
        CurrentOffsetError::InvalidRequest => DetectError::OutOfRange,
        CurrentOffsetError::UnsafeWhileMoving
        | CurrentOffsetError::RequiresExistingCalibration
        | CurrentOffsetError::OverCurrent => DetectError::Unsafe,
        CurrentOffsetError::Cancelled => DetectError::HardwareFault,
    }
}

/// Map an internal [`DetectionError`] to the wire [`DetectError`].
fn map_err(e: DetectionError) -> DetectError {
    match e {
        DetectionError::MotorNotResponding => DetectError::MotorNotResponding,
        DetectionError::OutOfRange => DetectError::OutOfRange,
        DetectionError::Timeout => DetectError::Timeout,
        DetectionError::HardwareFault => DetectError::HardwareFault,
        DetectionError::InsufficientSamples => DetectError::InsufficientSamples,
        DetectionError::LowConfidence => DetectError::LowConfidence,
        DetectionError::MissingPrerequisite => DetectError::MissingPrerequisite,
        _ => DetectError::HardwareFault,
    }
}

fn config_revisions() -> [u32; ConfigGroupId::COUNT] {
    critical_section::with(|cs| *CONFIG_COORDINATOR.revisions.borrow(cs).borrow())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::foc_driver::OVERCURRENT_HEADROOM;
    use crate::types::commissioning::SourceLimits;
    use crate::units::{amps, erpm, henries, ohms, watts};

    #[test]
    fn commissioning_uses_inverter_and_source_without_old_motor_limits() {
        use crate::storage::{CurrentLimitsConfig, MotorParamsConfig};
        let mut cfg = RuntimeConfig::default();
        assert!(prepare_commissioning(amps(70.0), None, None, None).is_err());
        cfg.current_limits = Some(CurrentLimitsConfig {
            max_iq: amps(3.0),
            max_phase_current: amps(30.0),
            bus_in_max: amps(5.0),
            bus_regen_max: amps(0.0),
            ..Default::default()
        });
        cfg.motor_params = Some(MotorParamsConfig {
            max_current: amps(2.0),
            ..Default::default()
        });
        assert_eq!(
            prepare_commissioning(amps(70.0), Some(&cfg), None, None)
                .unwrap()
                .current_ceiling(),
            amps(30.0 / OVERCURRENT_HEADROOM),
        );
        assert_eq!(
            prepare_commissioning(amps(70.0), Some(&cfg), Some(amps(4.0)), None)
                .unwrap()
                .current_ceiling(),
            amps(4.0),
        );
        assert!(prepare_commissioning(amps(70.0), Some(&cfg), Some(amps(f32::NAN)), None).is_err());
        cfg.current_limits.as_mut().unwrap().bus_regen_max.value = -1.0;
        assert!(prepare_commissioning(amps(70.0), Some(&cfg), None, None).is_err());
        let confirmed = SourceLimits {
            input: amps(5.0),
            regen: amps(0.0),
        };
        assert_eq!(
            prepare_commissioning(amps(70.0), Some(&cfg), None, Some(confirmed))
                .unwrap()
                .current_ceiling(),
            amps(30.0 / OVERCURRENT_HEADROOM),
        );
        assert_eq!(
            prepare_commissioning(
                amps(70.0),
                Some(&RuntimeConfig::default()),
                None,
                Some(confirmed)
            )
            .unwrap()
            .current_ceiling(),
            amps(70.0 / OVERCURRENT_HEADROOM),
        );
    }

    fn request_is_sane(req: &DetectRequest) -> bool {
        // Mirror of the per-arm guards in `run_step`, kept in one place so
        // the boundary rules stay unit-testable without an async backend.
        match *req {
            DetectRequest::MeasureResistance {
                max_power_loss: max_power_loss_w,
                ..
            } => pos(max_power_loss_w),
            DetectRequest::MeasureInductance {
                max_power_loss: max_power_loss_w,
                resistance: resistance_ohm,
                ..
            } => pos(max_power_loss_w) && pos(resistance_ohm),
            DetectRequest::MeasureFlux {
                max_power_loss: max_power_loss_w,
                resistance: resistance_ohm,
                inductance: inductance_h,
                pole_pairs,
                openloop_speed: openloop_erpm,
                ..
            } => {
                pos(max_power_loss_w)
                    && pos(resistance_ohm)
                    && pos(inductance_h)
                    && pole_pairs >= 1
                    && pos(openloop_erpm)
            }
            DetectRequest::CalibrateHall {
                max_power_loss: max_power_loss_w,
                resistance: resistance_ohm,
                ..
            } => pos(max_power_loss_w) && pos(resistance_ohm),
            DetectRequest::MeasureCurrentOffsets { .. } => true,
        }
    }

    #[test]
    fn detect_requests_reject_nan_and_degenerate_payloads() {
        // NaN power: sqrtf(NaN).min(max) would SELECT the platform maximum.
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            current_limit: None,
            max_power_loss: watts(f32::NAN),
        }));
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            current_limit: None,
            max_power_loss: watts(0.0),
        }));
        assert!(request_is_sane(&DetectRequest::MeasureResistance {
            current_limit: None,
            max_power_loss: watts(25.0),
        }));
        // Zero resistance divides the bus-current clamp.
        assert!(!request_is_sane(&DetectRequest::MeasureInductance {
            current_limit: None,
            max_power_loss: watts(25.0),
            resistance: ohms(0.0),
        }));
        // pole_pairs = 0 turns openloop_erpm into an infinite spin target.
        assert!(!request_is_sane(&DetectRequest::MeasureFlux {
            current_limit: None,
            max_power_loss: watts(25.0),
            resistance: ohms(0.03),
            inductance: henries(10e-6),
            pole_pairs: 0,
            openloop_speed: erpm(3000.0),
        }));
        assert!(request_is_sane(&DetectRequest::MeasureFlux {
            current_limit: None,
            max_power_loss: watts(25.0),
            resistance: ohms(0.03),
            inductance: henries(10e-6),
            pole_pairs: 7,
            openloop_speed: erpm(3000.0),
        }));
        assert!(!request_is_sane(&DetectRequest::CalibrateHall {
            current_limit: None,
            max_power_loss: watts(0.0),
            resistance: ohms(0.0),
        }));
        assert!(!request_is_sane(&DetectRequest::CalibrateHall {
            current_limit: None,
            max_power_loss: watts(f32::NAN),
            resistance: ohms(0.0),
        }));
    }

    #[test]
    fn resistance_results_reject_non_finite_and_non_positive_values() {
        assert_eq!(measured_resistance(ohms(0.25)).unwrap(), ohms(0.25));
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.0, -0.1] {
            assert!(matches!(
                measured_resistance(ohms(invalid)),
                Err(DetectError::HardwareFault)
            ));
        }
    }
}

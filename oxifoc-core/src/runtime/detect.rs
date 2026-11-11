//! Session-scoped motor measurements with bounded replay state.
//!
//! The task services status/cancel/retry requests while a measurement future
//! runs. Admission is owned by `session`, independent of actuator execution.
//! Terminal outcomes are published only after ISR cleanup. Session IDs include
//! a platform-provided random boot epoch; retired identities never re-energize.
//! Hosts retain orchestration of R/L/flux/Hall and may run each stage separately.

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
use crate::motor::ControlMode;
use crate::motor::foc_driver::{PowerSourceLimits, PreparedCommissioning};
use crate::state::DriverCommand;
use crate::state::model::{CONFIG_COORDINATOR, ModelApplyGuard};
use crate::state::{CMD_CHANNEL, DetectionActiveGuard};
use crate::storage::{HallCalibrationConfig, RuntimeConfig};
use crate::types::{DetectError, DetectRequest, DetectResponse, ReqId};

/// The platform-specific half of detection: the raw measurements, bound to the
/// hardware (or to the virtual-motor sim). Everything else — parameter
/// computation, dedup, error mapping, live config installation — is handled by
/// [`detect_server`].
#[allow(async_fn_in_trait)]
pub trait DetectionBackend {
    /// Wall-clock timer for the bounded ISR installation acknowledgement.
    /// Simulation's instantaneous measurement clock is unsuitable here.
    type Timer: crate::timer::Timer;

    /// Present DC bus voltage in volts (read from the platform ADC / sim).
    fn vbus(&self) -> f32;

    /// Measure phase-to-neutral resistance (Ω).
    async fn measure_resistance(
        &mut self,
        params: &ResistanceParams,
    ) -> Result<f32, DetectionError>;

    /// Measure d/q-axis inductance (H) via rotating HFI.
    async fn measure_inductance(
        &mut self,
        params: &InductanceParams,
        pwm_freq_hz: f32,
    ) -> Result<(f32, f32), DetectionError>;

    /// Measure flux linkage (Wb) via open-loop spin.
    async fn measure_flux(&mut self, params: &FluxLinkageParams) -> Result<f32, DetectionError>;

    /// Measure Hall sensors without installing the result. The server applies
    /// it at an acknowledged control boundary and updates the shared revision.
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
/// `max_current_a` is the per-platform current ceiling fed into the safe-test-
/// current formulas. Inverter protection and an optional diagnostic ceiling
/// further bound it. Finite source limits must be configured explicitly.
/// `foc_freq_hz` is the PWM/FOC loop frequency (used by the inductance HFI).
/// The Hall calibration result is written into `runtime_config` in memory; the
/// host persists it to flash via the config endpoint.
pub async fn detect_server<NS, B>(
    endpoints: Endpoints<NS>,
    mut backend: B,
    boot_epoch: u64,
    max_current_a: f32,
    foc_freq_hz: f32,
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
        let dispatch = session.handle(msg.t);
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
        // IDs are unique within this boot; config's ISR acknowledgement cache
        // is volatile too. Session checking happens before this internal key.
        let key = ReqId(
            (u64::from(measurement.id.session.generation) << 32)
                | u64::from(measurement.id.sequence),
        );
        let operation = async {
            let result = match select(
                run_step(
                    &mut backend,
                    measurement.request,
                    max_current_a,
                    foc_freq_hz,
                    runtime_config,
                    &guard,
                    key,
                ),
                <B::Timer as crate::timer::Timer>::after_millis(65_000),
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
            if crate::state::detection_cancelled()
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
                    break;
                }
                Either::Second(Ok(msg)) => {
                    let dispatch = session.handle(msg.t);
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
/// `f32::min` discards a NaN operand, so `sqrtf(NaN).min(max_current_a)`
/// SELECTS the platform's maximum test current instead of failing.
#[inline]
fn pos(v: f32) -> bool {
    v.is_finite() && v > 0.0
}

/// Validate a physical resistance result before it reaches current-limit math.
/// Backend implementations are a trust boundary just like wire requests: a
/// non-finite result would otherwise make `f32::min` select the platform limit.
#[inline]
fn measured_resistance(v: f32) -> Result<f32, DetectError> {
    if pos(v) {
        Ok(v)
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
    max_current_a: f32,
    foc_freq_hz: f32,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
    detection: &DetectionActiveGuard,
    request_id: ReqId,
) -> DetectResponse {
    if runtime_config.is_some_and(|config| {
        critical_section::with(|cs| !config.borrow(cs).borrow().board_calibration_is_valid())
    }) {
        return DetectResponse::Error(DetectError::Unsafe);
    }
    if matches!(req, DetectRequest::MeasureCurrentOffsets { .. }) {
        return measure_step(
            backend,
            req,
            max_current_a,
            foc_freq_hz,
            runtime_config,
            detection,
            request_id,
        )
        .await;
    }
    let prepared = match runtime_config {
        Some(config) => critical_section::with(|cs| {
            prepare_commissioning(
                max_current_a,
                Some(&config.borrow(cs).borrow()),
                req.current_limit_a(),
            )
        }),
        None => Err(DetectError::Unsafe),
    };
    let prepared = match prepared {
        Ok(prepared) => prepared,
        Err(error) => return DetectResponse::Error(error),
    };
    if !detection.begin::<B::Timer>(prepared).await {
        return DetectResponse::Error(DetectError::Busy);
    }
    let result = measure_step(
        backend,
        req,
        prepared.current_ceiling_a(),
        foc_freq_hz,
        runtime_config,
        detection,
        request_id,
    )
    .await;
    let limited = crate::state::detection_source_limited();
    if !detection.end::<B::Timer>().await {
        return DetectResponse::Error(DetectError::HardwareFault);
    }
    if limited {
        DetectResponse::Error(DetectError::SourceLimited)
    } else if crate::state::detection_cancelled() {
        DetectResponse::Error(DetectError::HardwareFault)
    } else if matches!(result, DetectResponse::Error(_))
        && crate::state::detection_source_constrained()
    {
        DetectResponse::Error(DetectError::SourceLimited)
    } else {
        result
    }
}

async fn measure_step<B: DetectionBackend>(
    backend: &mut B,
    req: DetectRequest,
    max_current_a: f32,
    foc_freq_hz: f32,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
    detection: &DetectionActiveGuard,
    request_id: ReqId,
) -> DetectResponse {
    match req {
        DetectRequest::MeasureResistance {
            max_power_loss_w, ..
        } => {
            if !pos(max_power_loss_w) {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            // Probe at low current to find a safe test current, then measure.
            let probe_current = (max_current_a / 50.0).max(0.5);
            let probe = ResistanceParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                current_max: probe_current,
                num_samples: 20,
                ramp_time_ms: 200,
                settle_time_ms: 100,
                ..Default::default()
            };
            match backend.measure_resistance(&probe).await {
                Ok(r_probe) => {
                    let r_probe = match measured_resistance(r_probe) {
                        Ok(r) => r,
                        Err(e) => return DetectResponse::Error(e),
                    };
                    let currents = match DetectionCurrents::new(
                        max_power_loss_w,
                        r_probe,
                        backend.vbus(),
                        max_current_a,
                    ) {
                        Ok(currents) if currents.resistance_a >= 0.5 => currents,
                        Ok(_) => return DetectResponse::Error(DetectError::OutOfRange),
                        Err(error) => return DetectResponse::Error(map_err(error)),
                    };
                    let params = ResistanceParams {
                        motor_size: MotorSize::Custom(max_power_loss_w),
                        current_max: currents.resistance_a,
                        ..Default::default()
                    };
                    match backend.measure_resistance(&params).await {
                        Ok(r) => match measured_resistance(r) {
                            Ok(resistance_ohm) => DetectResponse::Resistance { resistance_ohm },
                            Err(e) => DetectResponse::Error(e),
                        },
                        Err(e) => DetectResponse::Error(map_err(e)),
                    }
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureInductance {
            max_power_loss_w,
            resistance_ohm: r,
            ..
        } => {
            if !pos(max_power_loss_w) || !pos(r) {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            let params = match InductanceParams::commissioning(
                MotorSize::Custom(max_power_loss_w),
                r,
                backend.vbus(),
                max_current_a,
            ) {
                Ok(params) => params,
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            match backend.measure_inductance(&params, foc_freq_hz).await {
                Ok((ld, lq)) => DetectResponse::Inductance {
                    inductance_d_h: ld,
                    inductance_q_h: lq,
                },
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureFlux {
            max_power_loss_w,
            resistance_ohm: r,
            inductance_h,
            pole_pairs,
            openloop_erpm,
            ..
        } => {
            // pole_pairs = 0 would turn openloop_erpm into an infinite
            // spin target via the erpm/pole_pairs division below.
            if !pos(max_power_loss_w)
                || !pos(r)
                || !pos(inductance_h)
                || pole_pairs == 0
                || !pos(openloop_erpm)
            {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            let currents =
                match DetectionCurrents::new(max_power_loss_w, r, backend.vbus(), max_current_a) {
                    Ok(currents) => currents,
                    Err(error) => return DetectResponse::Error(map_err(error)),
                };
            let spin_rpm = openloop_erpm / f32::from(pole_pairs);
            let params = FluxLinkageParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                resistance_ohm: r,
                inductance_h,
                pole_pairs,
                spin_rpm,
                current_a: currents.flux_a,
                // Ramp until back-EMF dominates R·I (VESC's duty-0.3 idea);
                // spin_rpm above remains the hard speed cap.
                v_target: 0.2 * backend.vbus(),
                ..Default::default()
            };
            match backend.measure_flux(&params).await {
                Ok(flux) => {
                    // Line-to-line Kv (carries the √3 phase→line factor — see
                    // flux_linkage::calculate_kv); inlining it here is what kept
                    // the detection result √3-high.
                    let kv = if flux > 0.0 {
                        crate::foc::detection::flux_linkage::calculate_kv(flux, pole_pairs)
                    } else {
                        0.0
                    };
                    DetectResponse::FluxLinkage {
                        flux_linkage_wb: flux,
                        kv_rpm_per_v: kv,
                    }
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::CalibrateHall {
            max_power_loss_w,
            resistance_ohm,
            ..
        } => {
            let Some(runtime_config) = runtime_config else {
                return DetectResponse::Error(DetectError::MissingPrerequisite);
            };
            let currents = match DetectionCurrents::new(
                max_power_loss_w,
                resistance_ohm,
                backend.vbus(),
                max_current_a,
            ) {
                Ok(currents) => currents,
                Err(error) => return DetectResponse::Error(map_err(error)),
            };
            let params = HallCalibrationParams {
                current_amps: currents.hall_a,
                ..Default::default()
            };
            match backend.calibrate_hall(params).await {
                Ok(result) => {
                    let calibration = HallCalibrationConfig::from(&result);
                    if !result.is_valid() || !calibration.is_calibrated() {
                        return DetectResponse::Error(DetectError::HardwareFault);
                    }
                    CMD_CHANNEL
                        .send(DriverCommand::SetMode(ControlMode::Stopped))
                        .await;
                    let Some(pending) = ModelApplyGuard::prepare_detected_hall(
                        calibration,
                        runtime_config,
                        &CONFIG_COORDINATOR,
                        detection,
                        request_id,
                    ) else {
                        return DetectResponse::Error(DetectError::Busy);
                    };
                    match pending.apply::<B::Timer>().await {
                        crate::types::ConfigResponse::Applied { .. } => {
                            DetectResponse::HallCalibrated
                        }
                        crate::types::ConfigResponse::Busy
                        | crate::types::ConfigResponse::Conflict { .. } => {
                            DetectResponse::Error(DetectError::Busy)
                        }
                        _ => DetectResponse::Error(DetectError::HardwareFault),
                    }
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
/// Legacy endpoints require explicitly configured finite source limits until
/// the wizard carries the source selection in its session request.
fn prepare_commissioning(
    platform_max: f32,
    config: Option<&RuntimeConfig>,
    test_ceiling_a: Option<f32>,
) -> Result<PreparedCommissioning, DetectError> {
    let limits = config
        .and_then(|c| c.current_limits.as_ref())
        .ok_or(DetectError::Unsafe)?;
    let source = PowerSourceLimits::new(limits.bus_in_max_a, limits.bus_regen_max_a)
        .map_err(|_| DetectError::Unsafe)?;
    // Zero is the legacy "use board protection" value; reject negative/NaN.
    if !limits.max_phase_current_a.is_finite() || limits.max_phase_current_a < 0.0 {
        return Err(DetectError::Unsafe);
    }
    let trip = if limits.max_phase_current_a == 0.0 {
        platform_max
    } else {
        limits.max_phase_current_a
    };
    let prepared = PreparedCommissioning::new(platform_max, trip, source, test_ceiling_a)
        .map_err(|_| DetectError::Unsafe)?;
    if prepared.current_ceiling_a() < 0.5 {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commissioning_uses_inverter_and_source_without_old_motor_limits() {
        use crate::storage::{CurrentLimitsConfig, MotorParamsConfig};
        let mut cfg = RuntimeConfig::default();
        assert!(prepare_commissioning(70.0, None, None).is_err());
        cfg.current_limits = Some(CurrentLimitsConfig {
            max_iq_a: 3.0,
            max_phase_current_a: 30.0,
            bus_in_max_a: 5.0,
            bus_regen_max_a: 0.0,
            ..Default::default()
        });
        cfg.motor_params = Some(MotorParamsConfig {
            max_current_a: 2.0,
            ..Default::default()
        });
        assert_eq!(
            prepare_commissioning(70.0, Some(&cfg), None)
                .unwrap()
                .current_ceiling_a(),
            30.0 / crate::motor::foc_driver::OVERCURRENT_HEADROOM
        );
        assert_eq!(
            prepare_commissioning(70.0, Some(&cfg), Some(4.0))
                .unwrap()
                .current_ceiling_a(),
            4.0
        );
        assert!(prepare_commissioning(70.0, Some(&cfg), Some(f32::NAN)).is_err());
        cfg.current_limits.as_mut().unwrap().bus_regen_max_a = -1.0;
        assert!(prepare_commissioning(70.0, Some(&cfg), None).is_err());
    }

    fn request_is_sane(req: &DetectRequest) -> bool {
        // Mirror of the per-arm guards in `run_step`, kept in one place so
        // the boundary rules stay unit-testable without an async backend.
        match *req {
            DetectRequest::MeasureResistance {
                max_power_loss_w, ..
            } => pos(max_power_loss_w),
            DetectRequest::MeasureInductance {
                max_power_loss_w,
                resistance_ohm,
                ..
            } => pos(max_power_loss_w) && pos(resistance_ohm),
            DetectRequest::MeasureFlux {
                max_power_loss_w,
                resistance_ohm,
                inductance_h,
                pole_pairs,
                openloop_erpm,
                ..
            } => {
                pos(max_power_loss_w)
                    && pos(resistance_ohm)
                    && pos(inductance_h)
                    && pole_pairs >= 1
                    && pos(openloop_erpm)
            }
            DetectRequest::CalibrateHall {
                max_power_loss_w,
                resistance_ohm,
                ..
            } => pos(max_power_loss_w) && pos(resistance_ohm),
            DetectRequest::MeasureCurrentOffsets { .. } => true,
        }
    }

    #[test]
    fn detect_requests_reject_nan_and_degenerate_payloads() {
        // NaN power: sqrtf(NaN).min(max) would SELECT the platform maximum.
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            current_limit_a: None,
            max_power_loss_w: f32::NAN,
        }));
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            current_limit_a: None,
            max_power_loss_w: 0.0,
        }));
        assert!(request_is_sane(&DetectRequest::MeasureResistance {
            current_limit_a: None,
            max_power_loss_w: 25.0,
        }));
        // Zero resistance divides the bus-current clamp.
        assert!(!request_is_sane(&DetectRequest::MeasureInductance {
            current_limit_a: None,
            max_power_loss_w: 25.0,
            resistance_ohm: 0.0,
        }));
        // pole_pairs = 0 turns openloop_erpm into an infinite spin target.
        assert!(!request_is_sane(&DetectRequest::MeasureFlux {
            current_limit_a: None,
            max_power_loss_w: 25.0,
            resistance_ohm: 0.03,
            inductance_h: 10e-6,
            pole_pairs: 0,
            openloop_erpm: 3000.0,
        }));
        assert!(request_is_sane(&DetectRequest::MeasureFlux {
            current_limit_a: None,
            max_power_loss_w: 25.0,
            resistance_ohm: 0.03,
            inductance_h: 10e-6,
            pole_pairs: 7,
            openloop_erpm: 3000.0,
        }));
        assert!(!request_is_sane(&DetectRequest::CalibrateHall {
            current_limit_a: None,
            max_power_loss_w: 0.0,
            resistance_ohm: 0.0,
        }));
        assert!(!request_is_sane(&DetectRequest::CalibrateHall {
            current_limit_a: None,
            max_power_loss_w: f32::NAN,
            resistance_ohm: 0.0,
        }));
    }

    #[test]
    fn resistance_results_reject_non_finite_and_non_positive_values() {
        assert_eq!(measured_resistance(0.25).unwrap(), 0.25);
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 0.0, -0.1] {
            assert!(matches!(
                measured_resistance(invalid),
                Err(DetectError::HardwareFault)
            ));
        }
    }
}

//! Shared motor-detection server with effectively-once dedup.
//!
//! Detection is the one non-idempotent action in the ICD: re-running a step
//! re-energises the motor and (for Hall calibration) replaces the live config. So
//! `DetectEndpoint` carries a [`ReqId`](crate::types::ReqId)
//! (`Keyed<DetectRequest>`): the server caches the last `(id, response)` and a
//! retry with the same id replays the cached response instead of measuring
//! again — effectively-once.
//!
//! This module owns everything that used to be copy-pasted across the platform
//! detect servers — the serve loop, the dedup, the safe-current formulas, the
//! error mapping, motor-stop bracketing, and acknowledged Hall installation —
//! so all platforms behave identically. Each platform supplies only a
//! [`DetectionBackend`]: the raw measurements bound to its hardware (or to the
//! virtual-motor simulation).
//!
//! The loop is driven with `recv_manual` + `respond_owned` (owned request in,
//! owned response out) and contains no generic closures, so the future stays
//! `Send` and the virtual platform can spawn it on a multi-threaded runtime.

use core::cell::RefCell;

use critical_section::Mutex as CriticalSectionMutex;
use ergot::net_stack::NetStackHandle;
use ergot::net_stack::endpoints::Endpoints;

use crate::foc::clamp_f32;
#[cfg(feature = "offset-diagnostics")]
use crate::foc::current_offset::{
    CurrentOffsetError, CurrentOffsetMethod, CurrentOffsetReport, CurrentOffsetRequest,
};
use crate::foc::detection::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorSize, ResistanceParams,
};
use crate::foc::fast_math::sqrtf;
use crate::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use crate::icd::DetectEndpoint;
use crate::motor::ControlMode;
use crate::motor::foc_driver::CurrentLimits;
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
/// current formulas. Explicit configured driver/motor limits further bound it;
/// absent explicit limits, detection retains a 3-A commissioning ceiling.
/// `foc_freq_hz` is the PWM/FOC loop frequency (used by the inductance HFI).
/// The Hall calibration result is written into `runtime_config` in memory; the
/// host persists it to flash via the config endpoint.
pub async fn detect_server<NS, B>(
    endpoints: Endpoints<NS>,
    mut backend: B,
    max_current_a: f32,
    foc_freq_hz: f32,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
) where
    NS: NetStackHandle + Clone,
    B: DetectionBackend,
{
    // A separate handle for sending responses (`respond_owned` consumes it).
    let responder = endpoints.clone();
    let server = endpoints.bounded_server::<DetectEndpoint, 2>(Some("detect"));
    let server = core::pin::pin!(server);
    let mut h = server.attach();

    // Single-entry dedup cache: a retry with the same id AND the same request
    // replays this response. The payload must be compared too — host ids are
    // only unique within one process, so a freshly started host can reuse an
    // old id for a *different* step and would otherwise get the stale answer
    // replayed (e.g. a Resistance result for a CalibrateHall request).
    let mut cache: Option<(ReqId, DetectRequest, DetectResponse)> = None;

    loop {
        // Owned request + header — nothing borrowed across the measurement await.
        let Ok(msg) = h.recv_manual().await else {
            continue;
        };
        let id = msg.t.id;

        // A Hall commit is durable in the ISR, including when its waiter was
        // cancelled after claim. Replay it without re-energising the motor.
        if let Some(response) = CONFIG_COORDINATOR.detected(id, msg.t.inner) {
            let _ = responder
                .clone()
                .respond_owned::<DetectEndpoint>(&msg.hdr, &response);
            continue;
        }

        let resp = match &cache {
            // Hit: replay without re-running the (physical) measurement.
            Some((cached_id, cached_req, resp))
                if *cached_id == id && *cached_req == msg.t.inner =>
            {
                *resp
            }
            // Miss: stop the motor, measure, leave the motor stopped, cache.
            _ => {
                // send().await, not try_send: the bracketing Stops must not
                // be droppable on a full channel (the ISR drains every
                // cycle, so this resolves within one FOC period).
                // Suspend the link-loss failsafe while we drive: the host is
                // blocked on our response and sends no liveness frames (see
                // DETECTION_ACTIVE). RAII: the flag clears on EVERY exit path,
                // including a future cancellation of this future — a stranded
                // flag would leave the link-loss failsafe disabled forever.
                // The command-staleness deadman still covers the measurement
                // (long BENCH_STALENESS_TIMEOUT_US bound).
                let resp = if let Some(detection_guard) = DetectionActiveGuard::try_arm() {
                    CMD_CHANNEL
                        .send(DriverCommand::SetMode(ControlMode::Stopped))
                        .await;
                    let resp = run_step(
                        &mut backend,
                        msg.t.inner,
                        max_current_a,
                        foc_freq_hz,
                        runtime_config,
                        &detection_guard,
                        id,
                    )
                    .await;
                    CMD_CHANNEL
                        .send(DriverCommand::SetMode(ControlMode::Stopped))
                        .await;
                    resp
                } else {
                    DetectResponse::Error(DetectError::Busy)
                };
                // Success-only cache: a transient error stays retryable.
                if !matches!(resp, DetectResponse::Error(_)) {
                    cache = Some((id, msg.t.inner, resp));
                }
                resp
            }
        };

        let _ = responder
            .clone()
            .respond_owned::<DetectEndpoint>(&msg.hdr, &resp);
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
    let ceiling = match runtime_config {
        Some(config) => critical_section::with(|cs| {
            configured_current_ceiling(max_current_a, Some(&config.borrow(cs).borrow()))
        }),
        None => configured_current_ceiling(max_current_a, None),
    };
    let max_current_a = match ceiling {
        Ok(current) => current,
        Err(error) => return DetectResponse::Error(error),
    };
    match req {
        DetectRequest::MeasureResistance { max_power_loss_w } => {
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
                    // Clamp by the bus too: the thermal formula alone asks a
                    // high-R motor for more voltage than the bus has, the PI
                    // saturates short of the setpoint and the settle check
                    // aborts the measurement (same clamp as run_full_detection).
                    let max_bus_current = (backend.vbus() * 0.577 * 0.85) / r_probe.max(0.001);
                    let safe_current = sqrtf(max_power_loss_w / r_probe / 1.5)
                        .min(max_current_a)
                        .min(max_bus_current)
                        .max(probe_current);
                    let params = ResistanceParams {
                        motor_size: MotorSize::Custom(max_power_loss_w),
                        current_max: safe_current,
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
            let safe_current = sqrtf(max_power_loss_w / r / 1.5)
                .min(max_current_a)
                .min((backend.vbus() * 0.577 * 0.85) / r.max(0.001))
                .max(0.5);
            let spin_rpm = openloop_erpm / f32::from(pole_pairs);
            let params = FluxLinkageParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                resistance_ohm: r,
                inductance_h,
                pole_pairs,
                spin_rpm,
                current_a: safe_current.min(2.0),
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
        } => {
            let Some(runtime_config) = runtime_config else {
                return DetectResponse::Error(DetectError::MissingPrerequisite);
            };
            // Non-positive means "use the default sweep current" by design,
            // but non-finite bits are still rejected.
            if !max_power_loss_w.is_finite() || !resistance_ohm.is_finite() {
                return DetectResponse::Error(DetectError::OutOfRange);
            }
            // Sweep current from the power class, same formula as the other
            // steps (√(P/R/1.5)), capped by the platform ceiling. Falls back
            // to the conservative default when R is unknown.
            let mut params = HallCalibrationParams::default();
            if resistance_ohm > 0.0 && max_power_loss_w > 0.0 {
                let i = sqrtf(max_power_loss_w / resistance_ohm / 1.5);
                params.current_amps = clamp_f32(i, params.current_amps, max_current_a);
            }
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
                        req,
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

/// Share the driver's actual clamping rules instead of silently requesting a
/// measurement current above configured limits. Raising the platform ceiling
/// alone must not opt an unconfigured board into a high-current detection.
fn configured_current_ceiling(
    platform_max: f32,
    config: Option<&RuntimeConfig>,
) -> Result<f32, DetectError> {
    if !platform_max.is_finite() || platform_max < 0.5 {
        return Err(DetectError::Unsafe);
    }
    let Some((config, limits)) = config.and_then(|c| c.current_limits.as_ref().map(|l| (c, l)))
    else {
        return Ok(platform_max.min(3.0));
    };
    if !limits.is_coherent() {
        return Err(DetectError::Unsafe);
    }
    let rating = config
        .motor_params
        .as_ref()
        .and_then(crate::storage::MotorParamsConfig::rating_current_a)
        .unwrap_or(0.0);
    let live = CurrentLimits::from_config_clamped(limits, platform_max, rating);
    let ceiling = if limits.max_iq_a > 0.0 {
        live.max_current_a
    } else {
        live.max_current_a.min(3.0)
    };
    // Existing detection routines have a 0.5-A probe floor.
    if !ceiling.is_finite() || ceiling < 0.5 {
        return Err(DetectError::Unsafe);
    }
    Ok(ceiling)
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
    fn detection_ceiling_requires_explicit_limits_and_respects_driver_clamps() {
        use crate::storage::{CurrentLimitsConfig, MotorParamsConfig};
        let mut cfg = RuntimeConfig::default();
        assert!(configured_current_ceiling(0.1, None).is_err());
        assert_eq!(configured_current_ceiling(70.0, Some(&cfg)).unwrap(), 3.0);
        cfg.current_limits = Some(CurrentLimitsConfig {
            max_iq_a: 30.7,
            max_phase_current_a: 45.0,
            ..Default::default()
        });
        assert_eq!(configured_current_ceiling(70.0, Some(&cfg)).unwrap(), 30.7);
        assert_eq!(
            configured_current_ceiling(20.0, Some(&cfg)).unwrap(),
            20.0 / crate::motor::foc_driver::OVERCURRENT_HEADROOM
        );
        cfg.motor_params = Some(MotorParamsConfig {
            max_current_a: 12.0,
            ..Default::default()
        });
        assert_eq!(configured_current_ceiling(70.0, Some(&cfg)).unwrap(), 12.0);
        cfg.current_limits.as_mut().unwrap().max_iq_a = 0.1;
        assert!(matches!(
            configured_current_ceiling(70.0, Some(&cfg)),
            Err(DetectError::Unsafe)
        ));
        cfg.current_limits.as_mut().unwrap().max_iq_a = 0.0;
        assert_eq!(configured_current_ceiling(70.0, Some(&cfg)).unwrap(), 3.0);
    }

    fn request_is_sane(req: &DetectRequest) -> bool {
        // Mirror of the per-arm guards in `run_step`, kept in one place so
        // the boundary rules stay unit-testable without an async backend.
        match *req {
            DetectRequest::MeasureResistance { max_power_loss_w } => pos(max_power_loss_w),
            DetectRequest::MeasureInductance {
                max_power_loss_w,
                resistance_ohm,
            } => pos(max_power_loss_w) && pos(resistance_ohm),
            DetectRequest::MeasureFlux {
                max_power_loss_w,
                resistance_ohm,
                inductance_h,
                pole_pairs,
                openloop_erpm,
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
            } => max_power_loss_w.is_finite() && resistance_ohm.is_finite(),
            DetectRequest::MeasureCurrentOffsets { .. } => true,
        }
    }

    #[test]
    fn detect_requests_reject_nan_and_degenerate_payloads() {
        // NaN power: sqrtf(NaN).min(max) would SELECT the platform maximum.
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            max_power_loss_w: f32::NAN,
        }));
        assert!(!request_is_sane(&DetectRequest::MeasureResistance {
            max_power_loss_w: 0.0,
        }));
        assert!(request_is_sane(&DetectRequest::MeasureResistance {
            max_power_loss_w: 25.0,
        }));
        // Zero resistance divides the bus-current clamp.
        assert!(!request_is_sane(&DetectRequest::MeasureInductance {
            max_power_loss_w: 25.0,
            resistance_ohm: 0.0,
        }));
        // pole_pairs = 0 turns openloop_erpm into an infinite spin target.
        assert!(!request_is_sane(&DetectRequest::MeasureFlux {
            max_power_loss_w: 25.0,
            resistance_ohm: 0.03,
            inductance_h: 10e-6,
            pole_pairs: 0,
            openloop_erpm: 3000.0,
        }));
        assert!(request_is_sane(&DetectRequest::MeasureFlux {
            max_power_loss_w: 25.0,
            resistance_ohm: 0.03,
            inductance_h: 10e-6,
            pole_pairs: 7,
            openloop_erpm: 3000.0,
        }));
        // CalibrateHall: non-positive means "use defaults" by design, but
        // non-finite bits are still rejected.
        assert!(request_is_sane(&DetectRequest::CalibrateHall {
            max_power_loss_w: 0.0,
            resistance_ohm: 0.0,
        }));
        assert!(!request_is_sane(&DetectRequest::CalibrateHall {
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

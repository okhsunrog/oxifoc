//! Shared motor-detection server with effectively-once dedup.
//!
//! Detection is the one non-idempotent action in the ICD: re-running a step
//! re-energises the motor and (for Hall calibration) rewrites stored config. So
//! `DetectEndpoint` carries a [`ReqId`](crate::types::ReqId)
//! (`Keyed<DetectRequest>`): the server caches the last `(id, response)` and a
//! retry with the same id replays the cached response instead of measuring
//! again — effectively-once.
//!
//! This module owns everything that used to be copy-pasted across the platform
//! detect servers — the serve loop, the dedup, the safe-current formulas, the
//! error mapping, the motor-stop bracketing, and persisting the Hall result —
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

use crate::foc::detection::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorSize, ResistanceParams,
};
use crate::foc::fast_math::sqrtf;
use crate::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use crate::icd::DetectEndpoint;
use crate::motor::ControlMode;
use crate::state::CMD_CHANNEL;
use crate::state::DriverCommand;
use crate::storage::{HallCalibrationConfig, RuntimeConfig};
use crate::types::{DetectError, DetectRequest, DetectResponse, ReqId};

/// The platform-specific half of detection: the raw measurements, bound to the
/// hardware (or to the virtual-motor sim). Everything else — parameter
/// computation, dedup, error mapping, config persistence — is handled by
/// [`detect_server`].
#[allow(async_fn_in_trait)]
pub trait DetectionBackend {
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

    /// Calibrate Hall sensors; the result is persisted by [`detect_server`].
    async fn calibrate_hall(
        &mut self,
        params: HallCalibrationParams,
    ) -> Result<HallCalibrationResult, DetectionError>;
}

/// Serve `DetectEndpoint` with effectively-once dedup, forever.
///
/// `max_current_a` is the per-platform current ceiling fed into the safe-test-
/// current formulas (pass e.g. `min(board.max_phase_current_a, 3.0)` to clamp).
/// `foc_freq_hz` is the PWM/FOC loop frequency (used by the inductance HFI).
/// The Hall calibration result is written into `runtime_config` in memory; the
/// host persists it to flash via the config endpoint.
pub async fn detect_server<NS, B>(
    endpoints: Endpoints<NS>,
    mut backend: B,
    max_current_a: f32,
    foc_freq_hz: u32,
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
                CMD_CHANNEL
                    .send(DriverCommand::SetMode(ControlMode::Stopped))
                    .await;
                // Suspend the link-loss failsafe while we drive: the host is
                // blocked on our response and sends no liveness frames (see
                // DETECTION_ACTIVE). RAII: the flag clears on EVERY exit path,
                // including a future cancellation of this future — a stranded
                // flag would leave the link-loss failsafe disabled forever.
                // The command-staleness deadman still covers the measurement
                // (long BENCH_STALENESS_TIMEOUT_US bound).
                let _detection_guard = crate::state::DetectionActiveGuard::arm();
                let resp = run_step(
                    &mut backend,
                    msg.t.inner,
                    max_current_a,
                    foc_freq_hz,
                    runtime_config,
                )
                .await;
                drop(_detection_guard);
                CMD_CHANNEL
                    .send(DriverCommand::SetMode(ControlMode::Stopped))
                    .await;
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

/// Run a single detection step: compute safe parameters, call the backend, map
/// the result. Kept as one concrete (non-closure) `async fn` so the enclosing
/// server future stays `Send`.
async fn run_step<B: DetectionBackend>(
    backend: &mut B,
    req: DetectRequest,
    max_current_a: f32,
    foc_freq_hz: u32,
    runtime_config: Option<&'static CriticalSectionMutex<RefCell<RuntimeConfig>>>,
) -> DetectResponse {
    match req {
        DetectRequest::MeasureResistance { max_power_loss_w } => {
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
                        Ok(r) => DetectResponse::Resistance { resistance_ohm: r },
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
            let safe_current = sqrtf(max_power_loss_w / r / 1.5)
                .min(max_current_a)
                .max(0.5);
            let max_bus_current = (backend.vbus() * 0.577 * 0.6) / r.max(0.001);
            let hold_current = safe_current.min(max_bus_current).max(0.1);
            let params = InductanceParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                resistance_ohm: r,
                hold_current_a: hold_current,
                vbus: backend.vbus(),
                ..Default::default()
            };
            match backend
                .measure_inductance(&params, foc_freq_hz as f32)
                .await
            {
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

        DetectRequest::CalibrateHall => {
            match backend
                .calibrate_hall(HallCalibrationParams::default())
                .await
            {
                Ok(result) => {
                    // Persist in memory if the platform has a config store; the
                    // host then writes it to flash via the config endpoint.
                    if let Some(rc) = runtime_config {
                        critical_section::with(|cs| {
                            rc.borrow(cs).borrow_mut().hall_calibration =
                                Some(HallCalibrationConfig {
                                    angles: result.angles,
                                    valid: result.valid,
                                });
                        });
                    }
                    DetectResponse::HallCalibrated
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }
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

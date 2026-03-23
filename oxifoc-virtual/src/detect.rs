//! Virtual detect server — runs real detection algorithms against VirtualMotor.
//!
//! Each `DetectRequest` step is executed by running the actual sweep functions
//! from `oxifoc_core::foc::detection::sweep` against a freshly initialised
//! `VirtualHardware` / `VirtualTimer` simulation via `tokio::task::spawn_blocking`.

use core::pin::pin;

use ergot::net_stack::NetStackHandle;
use ergot::net_stack::endpoints::Endpoints;
use oxifoc_core::foc::detection::sweep::{
    calibrate_hall, measure_flux_linkage, measure_inductance, measure_resistance,
};
use oxifoc_core::foc::detection::types::{
    DetectionError, FluxLinkageParams, InductanceParams, MotorSize, ResistanceParams,
};
use oxifoc_core::foc::detection::virtual_harness::{
    VirtualHallReader, VirtualTimer, block_on, with_sim,
};
use oxifoc_core::foc::hall_calibration::HallCalibrationParams;
use oxifoc_core::icd::DetectEndpoint;
use oxifoc_core::types::{DetectError, DetectRequest, DetectResponse};
use oxifoc_core::virtual_motor::MotorParams;

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

pub async fn detect_server<NS: NetStackHandle>(
    endpoints: Endpoints<NS>,
    vbus: f32,
    max_current_a: f32,
    foc_freq_hz: u32,
) {
    let server = endpoints.bounded_server::<DetectEndpoint, 2>(Some("detect"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|req: &DetectRequest| {
                let req = *req;
                async move {
                    tokio::task::spawn_blocking(move || {
                        run_detect_step(req, vbus, max_current_a, foc_freq_hz)
                    })
                    .await
                    .unwrap_or(DetectResponse::Error(DetectError::HardwareFault))
                }
            })
            .await;
    }
}

/// Run one detection step synchronously on a blocking thread.
///
/// Each step creates a fresh `VirtualMotor` simulation via `with_sim`, then
/// runs the real detection algorithm synchronously (all timer delays
/// immediately advance the simulation — no real time elapses).
fn run_detect_step(
    req: DetectRequest,
    vbus: f32,
    max_current_a: f32,
    foc_freq_hz: u32,
) -> DetectResponse {
    let motor_params = MotorParams::default();

    match req {
        DetectRequest::MeasureResistance { max_power_loss_w } => {
            // Probe at low current first to find safe test current
            let probe_current = (max_current_a / 50.0).max(0.5);
            let probe_params = ResistanceParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                current_max: probe_current,
                num_samples: 20,
                ramp_time_ms: 200,
                settle_time_ms: 100,
                ..Default::default()
            };
            let r_probe = with_sim(motor_params, vbus, |hw| {
                block_on(measure_resistance::<_, VirtualTimer>(hw, &probe_params))
            });
            match r_probe {
                Ok(r_probe) => {
                    let safe_current = (max_power_loss_w / r_probe / 1.5)
                        .sqrt()
                        .min(max_current_a)
                        .max(probe_current);
                    let params = ResistanceParams {
                        motor_size: MotorSize::Custom(max_power_loss_w),
                        current_max: safe_current,
                        ..Default::default()
                    };
                    with_sim(motor_params, vbus, |hw| {
                        block_on(measure_resistance::<_, VirtualTimer>(hw, &params))
                    })
                    .map(|r| DetectResponse::Resistance { resistance_ohm: r })
                    .unwrap_or_else(|e| DetectResponse::Error(map_err(e)))
                }
                Err(e) => DetectResponse::Error(map_err(e)),
            }
        }

        DetectRequest::MeasureInductance {
            max_power_loss_w,
            resistance_ohm: r,
        } => {
            let safe_current = (max_power_loss_w / r / 1.5)
                .sqrt()
                .min(max_current_a)
                .max(0.5);
            let max_bus_current = (vbus * 0.577 * 0.6) / r.max(0.001);
            let hold_current = safe_current.min(max_bus_current).max(0.1);
            let params = InductanceParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                resistance_ohm: r,
                hold_current_a: hold_current,
                ..Default::default()
            };
            with_sim(motor_params, vbus, |hw| {
                block_on(measure_inductance::<
                    _,
                    VirtualTimer,
                    oxifoc_core::foc::trig::LibmSinCos,
                >(hw, &params, foc_freq_hz as f32))
            })
            .map(|(ld, lq)| DetectResponse::Inductance {
                inductance_d_h: ld,
                inductance_q_h: lq,
            })
            .unwrap_or_else(|e| DetectResponse::Error(map_err(e)))
        }

        DetectRequest::MeasureFlux {
            max_power_loss_w,
            resistance_ohm: r,
            pole_pairs,
            openloop_erpm,
        } => {
            let safe_current = (max_power_loss_w / r / 1.5)
                .sqrt()
                .min(max_current_a)
                .max(0.5);
            let spin_rpm = openloop_erpm / pole_pairs as f32;
            let params = FluxLinkageParams {
                motor_size: MotorSize::Custom(max_power_loss_w),
                resistance_ohm: r,
                pole_pairs,
                spin_rpm,
                current_a: safe_current.min(2.0),
                ..Default::default()
            };
            with_sim(motor_params, vbus, |hw| {
                block_on(measure_flux_linkage::<_, VirtualTimer>(hw, &params))
            })
            .map(|flux| {
                let kv = if flux > 0.0 {
                    60.0 / (core::f32::consts::TAU * flux * pole_pairs as f32)
                } else {
                    0.0
                };
                DetectResponse::FluxLinkage {
                    flux_linkage_wb: flux,
                    kv_rpm_per_v: kv,
                }
            })
            .unwrap_or_else(|e| DetectResponse::Error(map_err(e)))
        }

        DetectRequest::CalibrateHall => {
            let params = HallCalibrationParams::default();
            let hall_reader = VirtualHallReader;
            with_sim(motor_params, vbus, |hw| {
                block_on(calibrate_hall::<_, VirtualTimer, _>(
                    hw,
                    &hall_reader,
                    params,
                ))
            })
            .map(|hall_result| {
                use oxifoc_core::storage::HallCalibrationConfig;
                critical_section::with(|cs| {
                    crate::RUNTIME_CONFIG
                        .borrow(cs)
                        .borrow_mut()
                        .hall_calibration = Some(HallCalibrationConfig {
                        angles: hall_result.angles,
                        valid: hall_result.valid,
                    });
                });
                DetectResponse::HallCalibrated
            })
            .unwrap_or_else(|e| DetectResponse::Error(map_err(e)))
        }
    }
}

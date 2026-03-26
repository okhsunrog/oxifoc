//! Ergot protocol servers and USB I/O worker tasks

use embassy_executor::Spawner;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_6 as kit,
};
use heapless::String;

use crate::protocol::STACK;
use crate::transport::{AppDriver, RxWorker};
use crate::{FAULT_REGISTRY, STATE};
use oxifoc_core::types::DeviceInfo;

// ========== Worker Tasks ==========

/// USB device task - runs USB state machine
#[embassy_executor::task]
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
#[embassy_executor::task]
pub async fn run_rx(rcvr: RxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data (USB)
#[embassy_executor::task]
pub async fn run_tx(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static crate::transport::Queue>,
) {
    kit::tx_worker::<AppDriver, { crate::config::OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
///
/// Uses join to run info, hall, adc, and motor servers together.
/// This is more RAM-efficient than separate tasks.
#[embassy_executor::task]
pub async fn protocol_servers() {
    defmt::info!("Starting protocol servers");

    // Build device info
    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("Simple FOCer 2 (F405)");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32F405RG");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = DeviceInfo {
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz,
        max_current_a: crate::config::BOARD.max_phase_current_a,
    };

    oxifoc_core::runtime::run_all_servers(
        STACK.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// State monitor — watches interface state transitions and reacts to disconnect.
/// On disconnect: stops motor, disables fast telemetry, drains stale queue data.
#[embassy_executor::task]
pub async fn state_monitor() {
    use crate::transport::STATE_NOTIFY;
    use core::sync::atomic::Ordering;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::{FAST_TELEM_PERIOD, FAST_TELEM_Q};

    let mut was_active = false;

    loop {
        STATE_NOTIFY.wait().await.unwrap();
        let state = STACK.manage_profile(|im| im.interface_state(()));
        let is_active = matches!(state, Some(InterfaceState::Active { .. }));

        // Only react on actual transitions
        if is_active && !was_active {
            defmt::info!("Interface active — linked");
            was_active = true;
        } else if !is_active && was_active {
            defmt::info!("Interface down — stopping motor, disabling telemetry");
            was_active = false;

            // Stop the motor
            let _ =
                oxifoc_core::state::CMD_CHANNEL.try_send(oxifoc_core::motor::ControlMode::Stopped);

            // Stop fast telemetry streaming
            FAST_TELEM_PERIOD.store(0, Ordering::Relaxed);

            // Drain stale data from the fast telemetry bbqueue
            let cons = FAST_TELEM_Q.framed_consumer();
            while let Ok(grant) = cons.read() {
                grant.release();
            }

            // Yield to let the TX worker drain the outgoing queue
            for _ in 0..64 {
                embassy_futures::yield_now().await;
            }
        }
    }
}

/// Motor detection server — handles individual measurement steps.
#[embassy_executor::task]
pub async fn detect_server() {
    use core::pin::pin;

    use crate::calibration::{self, FluxLinkageParams, InductanceParams, ResistanceParams};
    use oxifoc_core::foc::detection::MotorSize;
    use oxifoc_core::foc::trig::FastSinCos;
    use oxifoc_core::icd::DetectEndpoint;
    use oxifoc_core::types::{DetectRequest, DetectResponse};

    let endpoints = STACK.endpoints();
    let server = endpoints.bounded_server::<DetectEndpoint, 2>(Some("detect"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|req: &DetectRequest| {
                let req = *req;
                async move {
                    // Stop motor before any measurement
                    let _ = oxifoc_core::state::CMD_CHANNEL
                        .try_send(oxifoc_core::motor::ControlMode::Stopped);

                    let vbus = crate::control::foc::VBUS_MV
                        .load(core::sync::atomic::Ordering::Relaxed)
                        as f32
                        / 1000.0;
                    let board = &crate::config::BOARD;
                    let pwm_hz = crate::config::PWM_CONFIG.pwm_freq_hz as f32;

                    let resp = match req {
                        DetectRequest::MeasureResistance { max_power_loss_w } => {
                            defmt::info!("Detect: measuring resistance");
                            let probe_current = (board.max_phase_current_a / 50.0).max(0.5);
                            let probe_params = ResistanceParams {
                                motor_size: MotorSize::Custom(max_power_loss_w),
                                current_max: probe_current,
                                num_samples: 20,
                                ramp_time_ms: 200,
                                settle_time_ms: 100,
                                ..Default::default()
                            };
                            match calibration::measure_resistance_ez(&probe_params).await {
                                Ok(r_probe) => {
                                    let safe_current =
                                        libm::sqrtf(max_power_loss_w / r_probe / 1.5)
                                            .min(board.max_phase_current_a)
                                            .min(3.0)
                                            .max(probe_current);
                                    let params = ResistanceParams {
                                        motor_size: MotorSize::Custom(max_power_loss_w),
                                        current_max: safe_current,
                                        ..Default::default()
                                    };
                                    match calibration::measure_resistance_ez(&params).await {
                                        Ok(r) => {
                                            defmt::info!("Resistance: {}Ω", r);
                                            DetectResponse::Resistance { resistance_ohm: r }
                                        }
                                        Err(e) => {
                                            defmt::error!("Resistance measurement failed");
                                            DetectResponse::Error(map_err(e))
                                        }
                                    }
                                }
                                Err(e) => {
                                    defmt::error!("Resistance probe failed");
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }

                        DetectRequest::MeasureInductance {
                            max_power_loss_w,
                            resistance_ohm: r,
                        } => {
                            defmt::info!("Detect: measuring inductance (R={})", r);
                            let safe_current = oxifoc_core::foc::clamp_f32(
                                libm::sqrtf(max_power_loss_w / r / 1.5)
                                    .min(board.max_phase_current_a),
                                0.5,
                                3.0,
                            );
                            let max_bus_current = (vbus * 0.577 * 0.6) / r.max(0.001);
                            let hold_current = oxifoc_core::foc::clamp_f32(
                                safe_current.min(max_bus_current),
                                0.1,
                                3.0,
                            );
                            let params = InductanceParams {
                                motor_size: MotorSize::Custom(max_power_loss_w),
                                resistance_ohm: r,
                                hold_current_a: hold_current,
                                ..Default::default()
                            };
                            match calibration::measure_inductance_ez::<FastSinCos>(&params, pwm_hz)
                                .await
                            {
                                Ok((ld, lq)) => {
                                    defmt::info!("Inductance: Ld={}H Lq={}H", ld, lq);
                                    DetectResponse::Inductance {
                                        inductance_d_h: ld,
                                        inductance_q_h: lq,
                                    }
                                }
                                Err(e) => {
                                    defmt::error!("Inductance measurement failed");
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }

                        DetectRequest::MeasureFlux {
                            max_power_loss_w,
                            resistance_ohm: r,
                            pole_pairs,
                            openloop_erpm,
                        } => {
                            defmt::info!("Detect: measuring flux linkage");
                            let safe_current = oxifoc_core::foc::clamp_f32(
                                libm::sqrtf(max_power_loss_w / r / 1.5)
                                    .min(board.max_phase_current_a),
                                0.5,
                                3.0,
                            );
                            let spin_rpm = openloop_erpm / pole_pairs as f32;
                            let params = FluxLinkageParams {
                                motor_size: MotorSize::Custom(max_power_loss_w),
                                resistance_ohm: r,
                                pole_pairs,
                                spin_rpm,
                                current_a: safe_current.min(2.0),
                                ..Default::default()
                            };
                            match calibration::measure_flux_linkage_ez(&params).await {
                                Ok(flux) => {
                                    let kv = if flux > 0.0 {
                                        60.0 / (core::f32::consts::TAU * flux * pole_pairs as f32)
                                    } else {
                                        0.0
                                    };
                                    defmt::info!("Flux: {}Wb Kv={}RPM/V", flux, kv);
                                    DetectResponse::FluxLinkage {
                                        flux_linkage_wb: flux,
                                        kv_rpm_per_v: kv,
                                    }
                                }
                                Err(e) => {
                                    defmt::error!("Flux linkage measurement failed");
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }

                        DetectRequest::CalibrateHall => {
                            defmt::info!("Detect: calibrating hall sensors");
                            match calibration::calibrate_hall_default_ez().await {
                                Ok(_hall_result) => {
                                    defmt::info!("Hall calibration OK");
                                    DetectResponse::HallCalibrated
                                }
                                Err(e) => {
                                    defmt::error!("Hall calibration failed");
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }
                    };

                    let _ = oxifoc_core::state::CMD_CHANNEL
                        .try_send(oxifoc_core::motor::ControlMode::Stopped);
                    resp
                }
            })
            .await;
    }
}

fn map_err(e: oxifoc_core::foc::detection::DetectionError) -> oxifoc_core::types::DetectError {
    use oxifoc_core::foc::detection::DetectionError;
    use oxifoc_core::types::DetectError;
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

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
#[embassy_executor::task]
pub async fn fast_telemetry_task() {
    oxifoc_core::runtime::streaming::fast_telemetry_stream::<_, 8>(
        &STACK,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(protocol_servers().unwrap());
    spawner.spawn(fast_telemetry_task().unwrap());
    spawner.spawn(state_monitor().unwrap());
    spawner.spawn(detect_server().unwrap());
}

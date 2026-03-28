//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use embedded_io_async::Write;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_6 as usb_kit,
};
use heapless::String;

use crate::transport::{AppDriver, Stack, UartRxWorker, UartWriter, UsbQueue, UsbRxWorker};
use crate::{FAULT_REGISTRY, STATE};
use oxifoc_core::types::HardwareInfo;

// ========== Worker Tasks ==========

/// USB device task — runs USB state machine
#[embassy_executor::task]
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
#[embassy_executor::task]
pub async fn run_usb_rx(rcvr: UsbRxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data (USB framed)
#[embassy_executor::task]
pub async fn run_usb_tx(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static UsbQueue>,
) {
    usb_kit::tx_worker::<AppDriver, { crate::config::USB_OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        usb_kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        usb_kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

/// Worker task for incoming ergot data (UART)
#[embassy_executor::task]
pub async fn run_uart_rx(
    mut rcvr: UartRxWorker,
    recv_buf: &'static mut [u8],
    scratch_buf: &'static mut [u8],
) {
    use ergot::interface_manager::InterfaceState;
    loop {
        let _ = rcvr
            .run(InterfaceState::Inactive, recv_buf, scratch_buf)
            .await;
    }
}

/// Maximum COBS-encoded frame size
const MAX_WIRE_BYTES: usize =
    crate::config::MAX_PACKET_SIZE + crate::config::MAX_PACKET_SIZE / 254 + 1;

/// Time to transmit one max-sized frame at the configured baud rate.
/// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
const TX_TIMEOUT_US: u64 =
    (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (crate::config::UART_BAUD as u64) * 3;

/// Worker task for outgoing ergot data (UART COBS stream)
///
/// When the interface is not Active, frames are discarded without writing to UART.
#[embassy_executor::task]
pub async fn run_uart_tx(mut tx: UartWriter, stack: &'static Stack, uart_ident: u8) {
    use ergot::interface_manager::{InterfaceState, Profile};

    let consumer = crate::transport::UART_OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let len = grant.len();

        let is_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(uart_ident),
                Some(InterfaceState::Active { .. })
            )
        });

        if is_active {
            let mut remaining = &grant[..];
            while !remaining.is_empty() {
                match embassy_time::with_timeout(
                    embassy_time::Duration::from_micros(TX_TIMEOUT_US),
                    tx.write(remaining),
                )
                .await
                {
                    Ok(Ok(n)) => remaining = &remaining[n..],
                    _ => break,
                }
            }
        }
        grant.release(len);
    }
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
#[embassy_executor::task]
pub async fn protocol_servers(stack: &'static Stack) {
    defmt::info!("Starting protocol servers");

    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("Simple FOCer 2 (F405)");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32F405RG");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = HardwareInfo {
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz,
        max_current_a: crate::config::BOARD.max_phase_current_a,
    };

    oxifoc_core::runtime::run_all_servers(
        stack.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// State monitor — watches interface state transitions and reacts to disconnect.
/// Stops motor and disables telemetry when ALL interfaces go down.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, usb_ident: u8, uart_ident: u8) {
    use crate::transport::STATE_NOTIFY;
    use core::sync::atomic::Ordering;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::{FAST_TELEM_PERIOD, FAST_TELEM_Q};

    let mut any_was_active = false;

    loop {
        STATE_NOTIFY.wait().await.unwrap();

        let usb_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(usb_ident),
                Some(InterfaceState::Active { .. })
            )
        });
        let uart_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(uart_ident),
                Some(InterfaceState::Active { .. })
            )
        });
        let any_active = usb_active || uart_active;

        if any_active && !any_was_active {
            defmt::info!(
                "Interface active — USB={}, UART={}",
                usb_active,
                uart_active
            );
            any_was_active = true;
        } else if !any_active && any_was_active {
            defmt::info!("All interfaces down — stopping motor, disabling telemetry");
            any_was_active = false;

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

            // Yield to let TX workers drain
            for _ in 0..64 {
                embassy_futures::yield_now().await;
            }
        }
    }
}

/// Motor detection server — handles individual measurement steps.
#[embassy_executor::task]
pub async fn detect_server(stack: &'static Stack) {
    use core::pin::pin;

    use crate::calibration::{self, FluxLinkageParams, InductanceParams, ResistanceParams};
    use oxifoc_core::foc::detection::MotorSize;
    use oxifoc_core::foc::trig::FastSinCos;
    use oxifoc_core::icd::DetectEndpoint;
    use oxifoc_core::types::{DetectRequest, DetectResponse};

    let endpoints = stack.endpoints();
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

/// Forward defmt frames from the network bbqueue to the ergot defmt topic.
/// Connected hosts receive these on `ErgotDefmtRxOwnedTopic`.
#[embassy_executor::task]
pub async fn defmt_forwarder(
    consumer: ergot::logging::defmt_sink::DefmtConsumer,
    stack: &'static Stack,
) {
    ergot::logging::defmt_sink::forward_to_ergot_topic(&consumer, stack, None).await
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
#[embassy_executor::task]
pub async fn fast_telemetry_task(stack: &'static Stack) {
    oxifoc_core::runtime::streaming::fast_telemetry_stream::<_, 8>(
        stack,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(
    spawner: &Spawner,
    stack: &'static Stack,
    usb_ident: u8,
    uart_ident: u8,
    defmt_consumer: ergot::logging::defmt_sink::DefmtConsumer,
) {
    spawner.spawn(protocol_servers(stack).unwrap());
    spawner.spawn(fast_telemetry_task(stack).unwrap());
    spawner.spawn(state_monitor(stack, usb_ident, uart_ident).unwrap());
    spawner.spawn(detect_server(stack).unwrap());
    spawner.spawn(defmt_forwarder(defmt_consumer, stack).unwrap());
}

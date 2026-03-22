//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
use heapless::String;
use oxifoc_core::types::DeviceInfo;

use crate::protocol::{OUTQ, STACK};
use crate::transport::RxWorker;
use crate::{FAULT_REGISTRY, RUNTIME_CONFIG, STATE};

#[cfg(feature = "transport-uart")]
use crate::transport::io::UartWriter;
#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::RttWriter;

// ========== Worker Tasks ==========

/// Worker task for incoming ergot data (transport-agnostic)
#[embassy_executor::task]
pub async fn run_rx(
    mut rcvr: RxWorker,
    recv_buf: &'static mut [u8],
    scratch_buf: &'static mut [u8],
) {
    loop {
        let _ = rcvr.run(recv_buf, scratch_buf).await;
    }
}

/// Worker task for outgoing ergot data via UART (transport-uart only)
#[cfg(feature = "transport-uart")]
#[embassy_executor::task]
pub async fn run_tx_uart(mut tx: UartWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// Worker task for outgoing ergot data via RTT (transport-rtt only)
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
pub async fn run_tx_rtt(mut tx: RttWriter) {
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
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
    let _ = hw.push_str("B-G431B-ESC1");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32G431CB");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = DeviceInfo {
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz,
        max_current_a: crate::config::BOARD.max_phase_current_a,
    };

    oxifoc_core::runtime::run_all_servers_with_config(
        STACK.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        &RUNTIME_CONFIG,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
/// Uses batch size of 8 to reduce stack usage (~360B vs ~1.4KB for 32).
#[embassy_executor::task]
pub async fn fast_telemetry_task() {
    oxifoc_core::runtime::streaming::fast_telemetry_stream::<_, 8>(
        &STACK,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// State monitor — watches interface state transitions and updates DeviceState.
/// On disconnect, disables fast telemetry streaming and drains the bbqueue
/// so the device doesn't waste cycles broadcasting to nobody.
#[embassy_executor::task]
pub async fn state_monitor() {
    use crate::protocol::{DeviceState, set_device_state};
    use crate::transport::STATE_NOTIFY;
    use core::sync::atomic::Ordering;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::{FAST_TELEM_PERIOD, FAST_TELEM_Q};

    loop {
        STATE_NOTIFY.wait().await.unwrap();
        let state = STACK.manage_profile(|im| im.interface_state(()));
        match state {
            Some(InterfaceState::Active { .. }) => {
                defmt::info!("Interface active — linked");
                set_device_state(DeviceState::Linked);
            }
            Some(InterfaceState::Inactive) | Some(InterfaceState::Down) | None => {
                defmt::info!("Interface inactive/down — waiting for link");
                set_device_state(DeviceState::WaitingLink);

                // Stop the motor
                defmt::info!("Interface is down — stopping the motor");
                let _ = oxifoc_core::state::CMD_CHANNEL
                    .try_send(oxifoc_core::motor::ControlMode::Stopped);

                // Stop fast telemetry streaming
                FAST_TELEM_PERIOD.store(0, Ordering::Relaxed);

                // Drain stale data from the bbqueue
                let cons = FAST_TELEM_Q.framed_consumer();
                while let Ok(grant) = cons.read() {
                    grant.release();
                }
            }
            _ => {}
        }
    }
}

/// Motor detection server — runs full detection sequence on request.
/// Separate task because detection takes several seconds and would block
/// all other protocol servers if joined into protocol_servers().
#[embassy_executor::task]
pub async fn detect_server() {
    use core::pin::pin;

    use crate::calibration::{DetectionParams, run_full_detection};
    use crate::cordic::CordicSinCos;
    use oxifoc_core::foc::detection::DetectionError;
    use oxifoc_core::foc::detection::MotorSize;
    use oxifoc_core::icd::DetectEndpoint;
    use oxifoc_core::types::{DetectError, DetectRequest, DetectResponse};

    let endpoints = STACK.endpoints();
    let server = endpoints.bounded_server::<DetectEndpoint, 2>(Some("detect"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|req: &DetectRequest| {
                let req = *req; // Copy before async block
                async move {
                    defmt::info!(
                        "Detection requested: pp={} loss={}W erpm={}",
                        req.pole_pairs,
                        req.max_power_loss_w,
                        req.openloop_erpm
                    );

                    // Stop motor before detection
                    let _ = oxifoc_core::state::CMD_CHANNEL
                        .try_send(oxifoc_core::motor::ControlMode::Stopped);

                    let params = DetectionParams {
                        motor_size: MotorSize::Custom(req.max_power_loss_w),
                        pole_pairs: req.pole_pairs,
                        current_max: crate::config::BOARD.max_phase_current_a,
                        max_power_loss_w: req.max_power_loss_w,
                        pwm_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz as f32,
                        vbus: 24.0, // TODO: read actual VBUS from ADC
                        openloop_erpm: req.openloop_erpm,
                    };

                    let response = match run_full_detection::<CordicSinCos>(params).await {
                        Ok(result) => {
                            defmt::info!(
                                "Detection OK: R={}Ω Ld={}H Lq={}H λ={}Wb",
                                result.params.resistance_ohm,
                                result.params.inductance_d_h,
                                result.params.inductance_q_h,
                                result.params.flux_linkage_wb,
                            );
                            DetectResponse::Ok {
                                resistance_ohm: result.params.resistance_ohm,
                                inductance_d_h: result.params.inductance_d_h,
                                inductance_q_h: result.params.inductance_q_h,
                                flux_linkage_wb: result.params.flux_linkage_wb,
                                kv_rpm_per_v: result.params.kv_rpm_per_v,
                                max_current_a: result.params.max_current_a,
                                kp_current: result.kp_current,
                                ki_current: result.ki_current,
                            }
                        }
                        Err(e) => {
                            defmt::warn!("Detection failed: {}", e);
                            let err = match e {
                                DetectionError::MotorNotResponding => {
                                    DetectError::MotorNotResponding
                                }
                                DetectionError::OutOfRange => DetectError::OutOfRange,
                                DetectionError::Timeout => DetectError::Timeout,
                                DetectionError::HardwareFault => DetectError::HardwareFault,
                                DetectionError::InsufficientSamples => {
                                    DetectError::InsufficientSamples
                                }
                                DetectionError::LowConfidence => DetectError::LowConfidence,
                                DetectionError::MissingPrerequisite => {
                                    DetectError::MissingPrerequisite
                                }
                                _ => DetectError::HardwareFault,
                            };
                            DetectResponse::Error(err)
                        }
                    };

                    // Stop motor after detection
                    let _ = oxifoc_core::state::CMD_CHANNEL
                        .try_send(oxifoc_core::motor::ControlMode::Stopped);

                    response
                }
            })
            .await;
    }
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(protocol_servers().unwrap());
    spawner.spawn(fast_telemetry_task().unwrap());
    spawner.spawn(state_monitor().unwrap());
    spawner.spawn(detect_server().unwrap());
}

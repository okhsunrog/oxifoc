//! Protocol layer for ergot communication and device management

use core::sync::atomic::{AtomicU8, Ordering};
use static_cell::StaticCell;

use crate::config::MAX_PACKET_SIZE;
#[cfg(feature = "transport-uart")]
use crate::config::UART_BAUD;
use crate::transport::{Queue, Stack};
use embedded_io_async::Write;
use ergot::interface_manager::{InterfaceState, Profile};

// ========== Ergot Stack ==========

/// Statically store our outgoing packet buffer
pub static OUTQ: Queue = Queue::new();

/// Statically store our netstack
pub static STACK: Stack = ergot::toolkits::embedded_io_async_v0_7::new_target_stack(
    OUTQ.stream_producer(),
    MAX_PACKET_SIZE as u16,
);

/// Buffers for RX worker
pub static RECV_BUF: StaticCell<[u8; MAX_PACKET_SIZE]> = StaticCell::new();
pub static SCRATCH_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// ========== Device State Management ==========

/// Device operational state
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DeviceState {
    Boot = 0,
    WaitingLink = 1,
    Linked = 2,
    Error = 3,
}

static DEVICE_STATE: AtomicU8 = AtomicU8::new(DeviceState::Boot as u8);

pub fn set_device_state(s: DeviceState) {
    DEVICE_STATE.store(s as u8, Ordering::Relaxed);
}

pub fn get_device_state() -> DeviceState {
    match DEVICE_STATE.load(Ordering::Relaxed) {
        0 => DeviceState::Boot,
        1 => DeviceState::WaitingLink,
        2 => DeviceState::Linked,
        _ => DeviceState::Error,
    }
}

// ========== Worker Tasks ==========

use embassy_executor::Spawner;
#[cfg(feature = "transport-rtt")]
use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
use heapless::String;
use oxifoc_core::types::DeviceInfo;

use crate::transport::RxWorker;
use crate::{FAULT_REGISTRY, RUNTIME_CONFIG, STATE};

#[cfg(feature = "transport-uart")]
use crate::transport::UartWriter;
#[cfg(feature = "transport-rtt")]
use ergot::transport::rtt::RttWriter;

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

/// Maximum COBS-encoded frame size (the largest grant the sink can produce).
/// Formula: n + n/254 + 1 (same as cobs::max_encoding_length)
#[cfg(feature = "transport-uart")]
const MAX_WIRE_BYTES: usize = MAX_PACKET_SIZE + MAX_PACKET_SIZE / 254 + 1;

/// Time to transmit one max-sized frame at the configured baud rate.
/// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
#[cfg(feature = "transport-uart")]
const TX_TIMEOUT_US: u64 = (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (UART_BAUD as u64) * 3;

/// Worker task for outgoing ergot data via UART (transport-uart only)
///
/// When the interface is not Active, frames are discarded from the queue
/// without writing to UART — this prevents stale telemetry frames from
/// blocking new protocol responses after a disconnect.
///
/// Writes have a timeout derived from the maximum frame size and baud rate,
/// so a stuck UART TX cannot block the queue permanently.
#[cfg(feature = "transport-uart")]
#[embassy_executor::task]
pub async fn run_tx_uart(mut tx: UartWriter) {
    let consumer = OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let len = grant.len();

        let is_active = STACK.manage_profile(|im| {
            matches!(im.interface_state(()), Some(InterfaceState::Active { .. }))
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
                    _ => break, // Timeout or error — drop this frame
                }
            }
        }
        grant.release(len);
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

                // Drain stale data from the fast telemetry bbqueue
                let cons = FAST_TELEM_Q.framed_consumer();
                while let Ok(grant) = cons.read() {
                    grant.release();
                }

                // Yield to let the TX worker drain the outgoing queue.
                // It discards frames since the interface is not Active.
                // Yield enough times to drain worst case (~50 frames in 2KB queue).
                for _ in 0..64 {
                    embassy_futures::yield_now().await;
                }
            }
            _ => {}
        }
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

/// Motor detection server — handles individual measurement steps.
///
/// GUI sends steps sequentially: MeasureResistance → MeasureInductance →
/// MeasureFlux → CalibrateHall. Cached R/L are used by subsequent steps.
/// PI gains are computed on the host side.
#[embassy_executor::task]
pub async fn detect_server() {
    use core::pin::pin;

    use crate::calibration::{
        self, FluxLinkageParams, InductanceParams, ResistanceParams, calibrate_hall_default,
    };
    use crate::cordic::CordicSinCos;
    use oxifoc_core::foc::detection::MotorSize;
    use oxifoc_core::icd::DetectEndpoint;
    use oxifoc_core::types::{DetectRequest, DetectResponse};

    // Cached results from previous steps. Single-task access, no sync needed.
    // Using statics because the serve closure is 'static.
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

                    let vbus = crate::foc::VBUS_MV.load(core::sync::atomic::Ordering::Relaxed)
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
                            match calibration::measure_resistance(&probe_params).await {
                                Ok(r_probe) => {
                                    // Limit detection current to 3A — safe for any PSU,
                                    // sufficient for accurate 2-point differential measurement.
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
                                    match calibration::measure_resistance(&params).await {
                                        Ok(r) => {
                                            defmt::info!("Resistance: {}Ω", r);
                                            DetectResponse::Resistance { resistance_ohm: r }
                                        }
                                        Err(e) => {
                                            defmt::error!("Resistance measurement failed: {}", e);
                                            DetectResponse::Error(map_err(e))
                                        }
                                    }
                                }
                                Err(e) => {
                                    defmt::error!("Resistance probe failed: {}", e);
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }

                        DetectRequest::MeasureInductance {
                            max_power_loss_w,
                            resistance_ohm: r,
                        } => {
                            defmt::info!("Detect: measuring inductance (R={})", r);
                            let safe_current = libm::sqrtf(max_power_loss_w / r / 1.5)
                                .min(board.max_phase_current_a)
                                .max(0.5);
                            let max_bus_current = (vbus * 0.577 * 0.6) / r.max(0.001);
                            let hold_current = safe_current.min(max_bus_current).max(0.1);
                            let params = InductanceParams {
                                motor_size: MotorSize::Custom(max_power_loss_w),
                                resistance_ohm: r,
                                hold_current_a: hold_current,
                                ..Default::default()
                            };
                            match calibration::measure_inductance::<CordicSinCos>(&params, pwm_hz)
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
                                    defmt::error!("Inductance measurement failed: {}", e);
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
                            let safe_current = libm::sqrtf(max_power_loss_w / r / 1.5)
                                .min(board.max_phase_current_a)
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
                            match calibration::measure_flux_linkage(&params).await {
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
                                    defmt::error!("Flux linkage measurement failed: {}", e);
                                    DetectResponse::Error(map_err(e))
                                }
                            }
                        }

                        DetectRequest::CalibrateHall => {
                            defmt::info!("Detect: calibrating hall sensors");
                            match calibrate_hall_default().await {
                                Ok(hall_result) => {
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
                                    defmt::info!("Hall calibration OK");
                                    DetectResponse::HallCalibrated
                                }
                                Err(e) => {
                                    defmt::error!("Hall calibration failed: {}", e);
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

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(protocol_servers().unwrap());
    spawner.spawn(fast_telemetry_task().unwrap());
    spawner.spawn(state_monitor().unwrap());
    spawner.spawn(detect_server().unwrap());
}

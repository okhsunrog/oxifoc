//! Ergot protocol servers and I/O worker tasks

use core::pin::pin;
use core::sync::atomic::Ordering;

use embassy_executor::Spawner;
use ergot::toolkits::embedded_io_async_v0_6::tx_worker;
use oxifoc_protocol::{DeviceInfo, InfoEndpoint, MotorCommand, MotorEndpoint, MotorState};

use crate::control::{duty_to_iq, get_adc_snapshot, send_command};
use crate::motor;
use crate::protocol::{DeviceState, LINK_ACTIVE, OUTQ, STACK, set_device_state};
use crate::sensors::get_hall_snapshot;
use crate::transport::RxWorker;
use oxifoc_core::motor::ControlMode;

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

/// Respond to info requests from host
#[embassy_executor::task]
pub async fn info_server() {
    let server = STACK
        .endpoints()
        .bounded_server::<InfoEndpoint, 2>(Some("device_info"));
    let server = pin!(server);
    let mut h = server.attach();
    loop {
        let _ = h
            .serve(|_req: &()| async move {
                // Mark link as active on first inbound request
                LINK_ACTIVE.store(true, Ordering::Relaxed);
                set_device_state(DeviceState::Linked);
                let mut hw: heapless::String<32> = heapless::String::new();
                let mut sw: heapless::String<32> = heapless::String::new();
                let _ = hw.push_str("B-G431B-ESC1");
                let _ = sw.push_str("oxifoc-0.1.0");
                DeviceInfo { hw, sw }
            })
            .await;
    }
}

/// ADC sample server - responds to host poll requests with current ADC values.
/// Host controls polling rate; device just returns latest values from atomics.
#[embassy_executor::task]
pub async fn adc_sample_server() {
    use oxifoc_protocol::AdcSampleEndpoint;

    defmt::info!("ADC sample server started (poll-based)");

    let server = STACK
        .endpoints()
        .bounded_server::<AdcSampleEndpoint, 2>(Some("adc"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|_: &()| async {
                let snapshot = get_adc_snapshot();
                oxifoc_protocol::AdcSample {
                    ia: snapshot.ia,
                    ib: snapshot.ib,
                    ic: snapshot.ic,
                    vbus_mv: snapshot.vbus_mv,
                    fet_temp_c_x10: snapshot.fet_temp_c_x10().unwrap_or(0),
                    seq: snapshot.seq,
                }
            })
            .await;
    }
}

/// Hall sensor server - responds to host poll requests with current Hall sensor data
#[embassy_executor::task]
pub async fn hall_sensor_server() {
    use oxifoc_protocol::{HallDirection, HallSensorData, HallSensorEndpoint};

    defmt::info!("Hall sensor server started (poll-based)");

    let server = STACK
        .endpoints()
        .bounded_server::<HallSensorEndpoint, 2>(Some("hall"));
    let server = pin!(server);
    let mut h = server.attach();

    // Sequence counter for protocol (since we removed it from HallSnapshot)
    let mut seq: u32 = 0;

    loop {
        let _ = h
            .serve(|_: &()| {
                seq = seq.wrapping_add(1);
                let current_seq = seq;
                async move {
                    let now_ticks = embassy_time::Instant::now().as_ticks();
                    let snapshot = get_hall_snapshot(now_ticks);

                    match snapshot {
                        Some(s) => {
                            // Convert core Direction to protocol HallDirection
                            let direction = match s.direction {
                                oxifoc_core::foc::hall_sensor::Direction::Clockwise => {
                                    HallDirection::Clockwise
                                }
                                oxifoc_core::foc::hall_sensor::Direction::CounterClockwise => {
                                    HallDirection::CounterClockwise
                                }
                                oxifoc_core::foc::hall_sensor::Direction::Stopped => {
                                    HallDirection::Stopped
                                }
                            };

                            HallSensorData {
                                angle_rad: s.angle_rad,
                                direction,
                                state: s.state,
                                error_count: s.error_count,
                                seq: current_seq,
                            }
                        }
                        None => {
                            // Hall sensor not initialized yet
                            HallSensorData {
                                angle_rad: 0.0,
                                direction: HallDirection::Stopped,
                                state: 0,
                                error_count: 0,
                                seq: current_seq,
                            }
                        }
                    }
                }
            })
            .await;
    }
}

/// Motor command server - handles motor control commands via ergot
#[embassy_executor::task]
pub async fn motor_command_server() {
    defmt::info!("Motor command server started");

    let server = STACK
        .endpoints()
        .bounded_server::<MotorEndpoint, 2>(Some("motor"));
    let server = pin!(server);
    let mut h = server.attach();

    loop {
        let _ = h
            .serve(|cmd: &MotorCommand| {
                let cmd = cmd.clone();
                async move {
                    match cmd {
                        MotorCommand::Stop => {
                            motor::set_motor_state(MotorState::Stopped);
                            motor::set_motor_duty(0);
                            motor::set_motor_step(0);
                            send_command(ControlMode::Stopped);
                        }
                        MotorCommand::Start { duty } | MotorCommand::SetSpeed { duty } => {
                            let duty = duty.min(100);
                            motor::set_motor_state(MotorState::Running);
                            motor::set_motor_duty(duty);
                            motor::set_motor_step(0);

                            let iq_target = duty_to_iq(duty);
                            send_command(ControlMode::CurrentControl {
                                iq_target,
                                id_target: 0.0,
                            });
                        }
                    }

                    motor::get_motor_status()
                }
            })
            .await;
    }
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(spawner: &Spawner) {
    spawner.spawn(info_server().unwrap());
    spawner.spawn(adc_sample_server().unwrap());
    spawner.spawn(hall_sensor_server().unwrap());
    spawner.spawn(motor_command_server().unwrap());
}

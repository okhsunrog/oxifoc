//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
use heapless::String;
use oxifoc_core::types::DeviceInfo;

use crate::protocol::{OUTQ, STACK};
use crate::transport::RxWorker;
use crate::{FAULT_REGISTRY, STATE};

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

    oxifoc_core::runtime::run_all_servers(
        STACK.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches
#[embassy_executor::task]
pub async fn fast_telemetry_task() {
    oxifoc_core::runtime::streaming::fast_telemetry_stream(
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
}

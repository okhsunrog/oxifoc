//! Protocol layer for ergot communication and device management

use core::sync::atomic::{AtomicU8, Ordering};
use static_cell::StaticCell;

use crate::config::MAX_PACKET_SIZE;
#[cfg(feature = "transport-uart")]
use crate::config::UART_BAUD;
use crate::transport::Stack;
use embedded_io_async::Write;
use ergot::interface_manager::{InterfaceState, Profile};

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
use heapless::String;
use oxifoc_core::types::HardwareInfo;

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
        let _ = rcvr
            .run(InterfaceState::Inactive, recv_buf, scratch_buf)
            .await;
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
pub async fn run_tx_uart(mut tx: UartWriter, stack: &'static Stack, ident: u8) {
    let consumer = crate::transport::OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let len = grant.len();

        let is_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(ident),
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
pub async fn run_tx_rtt(mut tx: RttWriter, stack: &'static Stack, ident: u8) {
    use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
    // TODO: add active check like UART if needed
    let _ = (stack, ident);
    loop {
        let _ = tx_worker(&mut tx, crate::transport::OUTQ.stream_consumer()).await;
    }
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
///
/// Uses join to run info, hall, adc, and motor servers together.
/// This is more RAM-efficient than separate tasks.
#[embassy_executor::task]
pub async fn protocol_servers(stack: &'static Stack) {
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
    let device_info = HardwareInfo {
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: crate::config::PWM_CONFIG.pwm_freq_hz,
        max_current_a: crate::config::BOARD.max_phase_current_a,
    };

    oxifoc_core::runtime::run_all_servers_with_config(
        stack.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        &RUNTIME_CONFIG,
        crate::config::PWM_CONFIG.pwm_freq_hz,
        crate::config::BOARD.max_phase_current_a,
        // No flash persistence on this board — the config server reports
        // persist-capable = false and serves the RAM copy only.
        false,
    )
    .await
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
/// Uses batch size of 8 to reduce stack usage (~360B vs ~1.4KB for 32).
#[embassy_executor::task]
pub async fn fast_telemetry_task(stack: &'static Stack) {
    oxifoc_core::runtime::streaming::fast_telemetry_stream::<_, 8, oxifoc_core::timer::EmbassyTimer>(
        stack,
        crate::config::PWM_CONFIG.pwm_freq_hz,
    )
    .await
}

/// State monitor — watches interface state transitions and updates DeviceState.
/// On disconnect, disables fast telemetry streaming and drains the bbqueue
/// so the device doesn't waste cycles broadcasting to nobody.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, ident: u8) {
    use crate::transport::STATE_NOTIFY;
    use core::sync::atomic::Ordering;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::{FAST_TELEM_PERIOD, FAST_TELEM_Q};

    loop {
        defmt::unwrap!(STATE_NOTIFY.wait().await.ok());
        let state = stack.manage_profile(|im| im.interface_state(ident));
        match state {
            Some(InterfaceState::Active { .. }) => {
                defmt::info!("Interface active — linked");
                set_device_state(DeviceState::Linked);
                critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_active());
            }
            Some(InterfaceState::Inactive) | Some(InterfaceState::Down) | None => {
                defmt::info!("Interface inactive/down — waiting for link");
                set_device_state(DeviceState::WaitingLink);

                // Drop link_active — the ISR link gate routes a running
                // motor through the configured failsafe policy. Deliberately
                // NO SetMode(Stopped) here: a queued Stopped would be applied
                // by process_commands and `set_mode` cancels an in-progress
                // failsafe brake (and clears the re-arm latch) — turning the
                // ControlledStop into a coast one liveness-timeout after the
                // deadman armed it.
                defmt::info!("Interface is down — failsafe via link gate");
                critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_inactive());

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

/// Detection backend for the G431 platform: the raw measurements bound to the
/// shared calibration code (which uses the platform ADC statics + board config).
#[cfg(feature = "detection")]
struct G431Backend;

#[cfg(feature = "detection")]
impl oxifoc_core::runtime::DetectionBackend for G431Backend {
    fn vbus(&self) -> f32 {
        crate::foc::VBUS_MV.load(core::sync::atomic::Ordering::Relaxed) as f32 / 1000.0
    }
    async fn measure_resistance(
        &mut self,
        params: &oxifoc_core::foc::detection::types::ResistanceParams,
    ) -> Result<f32, oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_resistance(params).await
    }
    async fn measure_inductance(
        &mut self,
        params: &oxifoc_core::foc::detection::types::InductanceParams,
        pwm_freq_hz: f32,
    ) -> Result<(f32, f32), oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_inductance::<crate::cordic::CordicSinCos>(params, pwm_freq_hz)
            .await
    }
    async fn measure_flux(
        &mut self,
        params: &oxifoc_core::foc::detection::types::FluxLinkageParams,
    ) -> Result<f32, oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_flux_linkage(params).await
    }
    async fn calibrate_hall(
        &mut self,
        params: oxifoc_core::foc::hall_calibration::HallCalibrationParams,
    ) -> Result<
        oxifoc_core::foc::hall_calibration::HallCalibrationResult,
        oxifoc_core::foc::detection::DetectionError,
    > {
        crate::calibration::calibrate_hall(params).await
    }
}

#[cfg(feature = "detection")]
#[embassy_executor::task]
pub async fn detect_server(stack: &'static Stack) {
    oxifoc_core::runtime::detect_server(
        stack.endpoints(),
        G431Backend,
        crate::config::BOARD.max_phase_current_a.min(3.0),
        crate::config::PWM_CONFIG.pwm_freq_hz,
        Some(&RUNTIME_CONFIG),
    )
    .await
}

// ========== Task Spawning ==========

pub fn spawn_servers(spawner: &Spawner, stack: &'static Stack, ident: u8) {
    spawner.spawn(defmt::unwrap!(protocol_servers(stack)));
    spawner.spawn(defmt::unwrap!(fast_telemetry_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, ident)));
    #[cfg(feature = "detection")]
    spawner.spawn(defmt::unwrap!(detect_server(stack)));
}

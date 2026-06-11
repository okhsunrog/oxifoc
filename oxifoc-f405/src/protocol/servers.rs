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

    oxifoc_core::runtime::run_all_servers_with_config(
        stack.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        &crate::RUNTIME_CONFIG,
        crate::config::PWM_CONFIG.pwm_freq_hz,
        crate::config::BOARD.max_phase_current_a,
        true,
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
        defmt::unwrap!(STATE_NOTIFY.wait().await.ok());

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
            critical_section::with(|cs| crate::STATE.borrow(cs).borrow_mut().set_link_active());
        } else if !any_active && any_was_active {
            defmt::info!("All interfaces down — failsafe via link gate, disabling telemetry");
            any_was_active = false;

            // Drop link_active — the ISR link gate routes a running motor
            // through the configured failsafe policy. Deliberately NO
            // SetMode(Stopped) here: a queued Stopped would be applied by
            // process_commands and `set_mode` cancels an in-progress
            // failsafe brake (and clears the re-arm latch) — turning the
            // ControlledStop into a coast one liveness-timeout after the
            // deadman armed it.
            critical_section::with(|cs| crate::STATE.borrow(cs).borrow_mut().set_link_inactive());

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

/// Motor detection server — delegates to the shared, deduplicating server.
///
/// F405 has no `RUNTIME_CONFIG`/config endpoint yet, so the Hall result is not
/// persisted (`None`) — same as before this was unified.
#[embassy_executor::task]
pub async fn detect_server(stack: &'static Stack) {
    oxifoc_core::runtime::detect_server(
        stack.endpoints(),
        F405Backend,
        crate::config::BOARD.max_phase_current_a.min(3.0),
        crate::config::PWM_CONFIG.pwm_freq_hz,
        None,
    )
    .await
}

/// Detection backend for the F405 platform: the raw measurements bound to the
/// shared calibration `*_ez` wrappers (which use the platform ADC statics).
struct F405Backend;

impl oxifoc_core::runtime::DetectionBackend for F405Backend {
    fn vbus(&self) -> f32 {
        crate::control::foc::VBUS_MV.load(core::sync::atomic::Ordering::Relaxed) as f32 / 1000.0
    }
    async fn measure_resistance(
        &mut self,
        params: &oxifoc_core::foc::detection::types::ResistanceParams,
    ) -> Result<f32, oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_resistance_ez(params).await
    }
    async fn measure_inductance(
        &mut self,
        params: &oxifoc_core::foc::detection::types::InductanceParams,
        pwm_freq_hz: f32,
    ) -> Result<(f32, f32), oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_inductance_ez::<oxifoc_core::foc::trig::FastSinCos>(
            params,
            pwm_freq_hz,
        )
        .await
    }
    async fn measure_flux(
        &mut self,
        params: &oxifoc_core::foc::detection::types::FluxLinkageParams,
    ) -> Result<f32, oxifoc_core::foc::detection::DetectionError> {
        crate::calibration::measure_flux_linkage_ez(params).await
    }
    async fn calibrate_hall(
        &mut self,
        _params: oxifoc_core::foc::hall_calibration::HallCalibrationParams,
    ) -> Result<
        oxifoc_core::foc::hall_calibration::HallCalibrationResult,
        oxifoc_core::foc::detection::DetectionError,
    > {
        crate::calibration::calibrate_hall_default_ez().await
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
    oxifoc_core::runtime::streaming::fast_telemetry_stream::<_, 8, oxifoc_core::timer::EmbassyTimer>(
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
    spawner.spawn(defmt::unwrap!(protocol_servers(stack)));
    spawner.spawn(defmt::unwrap!(fast_telemetry_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, usb_ident, uart_ident)));
    spawner.spawn(defmt::unwrap!(detect_server(stack)));
    spawner.spawn(defmt::unwrap!(defmt_forwarder(defmt_consumer, stack)));
}

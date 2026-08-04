//! Ergot protocol servers and I/O worker tasks

use embedded_io_async::Write;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_6 as usb_kit,
};
use heapless::String;

use crate::transport::{AppDriver, Stack, UartRxWorker, UartWriter, UsbQueue, UsbRxWorker};
use crate::{FAULT_REGISTRY, STATE};
use core::sync::atomic::Ordering;
use oxifoc_core::types::HardwareInfo;

use oxifoc_core::foc::detection::DetectionError;
use oxifoc_core::foc::detection::types::{FluxLinkageParams, InductanceParams, ResistanceParams};
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
use oxifoc_core::foc::trig::FastSinCos;
use oxifoc_core::runtime::streaming::{fast_telemetry_stream, fault_topic_stream};
use oxifoc_core::runtime::{DetectionBackend, run_all_servers_with_config};
use rtic_monotonics::Monotonic;
use rtic_monotonics::fugit::ExtU64;

use crate::time::{Mono, MonoTimer};

use crate::RUNTIME_CONFIG;
use crate::calibration::{
    calibrate_hall_ez, measure_flux_linkage_ez, measure_inductance_ez, measure_resistance_ez,
};
use crate::config::{BOARD, MAX_PACKET_SIZE, PWM_CONFIG, UART_BAUD, USB_OUT_QUEUE_SIZE};
use crate::control::foc::VBUS_MV;
use crate::transport::UART_OUTQ;

// ========== Worker Tasks ==========

/// USB device task — runs USB state machine
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
pub async fn run_usb_rx(rcvr: UsbRxWorker, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data (USB framed)
pub async fn run_usb_tx(
    mut ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static UsbQueue>,
) {
    usb_kit::tx_worker::<AppDriver, { USB_OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        usb_kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        usb_kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

/// Worker task for incoming ergot data (UART)
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
const MAX_WIRE_BYTES: usize = MAX_PACKET_SIZE + MAX_PACKET_SIZE / 254 + 1;

/// Time to transmit one max-sized frame at the configured baud rate.
/// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
const TX_TIMEOUT_US: u64 = (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (UART_BAUD as u64) * 3;

/// Worker task for outgoing ergot data (UART COBS stream)
///
/// When the interface is not Active, frames are discarded without writing to UART.
pub async fn run_uart_tx(mut tx: UartWriter, stack: &'static Stack, uart_ident: u8) {
    use ergot::interface_manager::{InterfaceState, Profile};

    let consumer = UART_OUTQ.stream_consumer();
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
                match Mono::timeout_after(TX_TIMEOUT_US.micros(), tx.write(remaining)).await {
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
pub async fn protocol_servers(stack: &'static Stack) {
    defmt::info!("Starting protocol servers");

    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    #[cfg(feature = "board-cf2")]
    let _ = hw.push_str("Simple FOCer 2 (F405)");
    #[cfg(feature = "board-vesc6-mk5")]
    let _ = hw.push_str("VESC 6 MK5 (F405)");
    let _ = sw.push_str(concat!("oxifoc-", env!("CARGO_PKG_VERSION")));
    let _ = mcu.push_str("STM32F405RG");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = HardwareInfo {
        proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: PWM_CONFIG.pwm_freq_hz,
        max_current_a: BOARD.max_phase_current_a,
        calib: BOARD.calib,
    };

    // This future IS the protocol-servers task (all endpoint servers
    // joined); embassy arena-allocates it statically, so its size is the
    // task's intended footprint, not an accident the lint should flag.
    #[expect(clippy::large_futures, reason = "the joined servers are the task")]
    run_all_servers_with_config(
        stack.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        &RUNTIME_CONFIG,
        PWM_CONFIG.pwm_freq_hz,
        BOARD.max_phase_current_a,
        true,
    )
    .await;
}

/// Ergot well-known services for downstream bridges.
///
/// The seed-router handler answers net-id assignment/refresh/release requests
/// (the BLE bridge leases a net for its radio segment here — without it a
/// bridge's pending downstream never gets a routable net). The ping handler
/// is the bridge's upstream-discovery bootstrap: the bridge pings the root
/// link-local, and the *reply* is the first frame addressed to the bridge,
/// which is what its edge processor learns the upstream net_id from.
pub async fn seed_router_task(stack: &'static Stack) {
    embassy_futures::join::join(
        stack.services().seed_router_request_handler::<2>(),
        stack.services().ping_handler::<2>(),
    )
    .await;
}

/// State monitor — watches interface state transitions and reacts to disconnect.
/// Stops motor and disables telemetry when ALL interfaces go down.
pub async fn state_monitor(stack: &'static Stack, idents: heapless::Vec<u8, 3>) {
    use crate::transport::STATE_NOTIFY;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::{FAST_TELEM_PERIOD, FAST_TELEM_Q};

    let mut any_was_active = false;

    loop {
        defmt::unwrap!(STATE_NOTIFY.wait().await.ok());

        let any_active = idents.iter().any(|&id| {
            stack.manage_profile(|im| {
                matches!(im.interface_state(id), Some(InterfaceState::Active { .. }))
            })
        });

        if any_active && !any_was_active {
            defmt::info!("Interface active — link up");
            any_was_active = true;
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_active());
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
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_inactive());

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
/// The Hall calibration result lands in `RUNTIME_CONFIG` (in RAM); the host
/// persists it to flash through the config endpoint (the CLI's
/// `detect hall --apply` does exactly that). The stale claim that the F405
/// "has no RUNTIME_CONFIG yet" silently discarded every calibration
/// (found on the Flipsky bring-up, 2026-07-08).
pub async fn detect_server(stack: &'static Stack) {
    oxifoc_core::runtime::detect_server(
        stack.endpoints(),
        F405Backend,
        BOARD.max_phase_current_a.min(3.0),
        PWM_CONFIG.pwm_freq_hz,
        Some(&RUNTIME_CONFIG),
    )
    .await;
}

/// Detection backend for the F405 platform: the raw measurements bound to the
/// shared calibration `*_ez` wrappers (which use the platform ADC statics).
struct F405Backend;

impl DetectionBackend for F405Backend {
    fn vbus(&self) -> f32 {
        VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0
    }
    // Pure pass-throughs return the inner future directly (`fn -> impl
    // Future` instead of `async fn`) — an `async` body here would wrap the
    // already-large detection futures in one more generated state machine
    // for zero benefit.
    fn measure_resistance(
        &mut self,
        params: &ResistanceParams,
    ) -> impl Future<Output = Result<f32, DetectionError>> {
        measure_resistance_ez(params)
    }
    fn measure_inductance(
        &mut self,
        params: &InductanceParams,
        pwm_freq_hz: f32,
    ) -> impl Future<Output = Result<(f32, f32), DetectionError>> {
        measure_inductance_ez::<FastSinCos>(params, pwm_freq_hz)
    }
    fn measure_flux(
        &mut self,
        params: &FluxLinkageParams,
    ) -> impl Future<Output = Result<f32, DetectionError>> {
        measure_flux_linkage_ez(params)
    }
    fn calibrate_hall(
        &mut self,
        params: HallCalibrationParams,
    ) -> impl Future<Output = Result<HallCalibrationResult, DetectionError>> {
        calibrate_hall_ez(params)
    }
    fn measure_current_offsets(
        &mut self,
        request: oxifoc_core::foc::current_offset::CurrentOffsetRequest,
    ) -> impl Future<
        Output = Result<
            oxifoc_core::foc::current_offset::CurrentOffsetReport,
            oxifoc_core::foc::current_offset::CurrentOffsetError,
        >,
    > {
        oxifoc_core::state::measure_current_offsets(request)
    }
}

/// Forward defmt frames from the network bbqueue to the ergot defmt topic.
/// Connected hosts receive these on `ErgotDefmtRxOwnedTopic`.
pub async fn defmt_forwarder(
    consumer: ergot::logging::defmt_sink::DefmtConsumer,
    stack: &'static Stack,
) {
    // (2026-07-08: defmt-over-USB works end-to-end. It was broken by TWO
    // stacked bugs, fixed the same day: ergot's defmt sink granted half
    // the ring per frame and could park its pointer in a geometric dead
    // zone — permanent silent drop of every frame (fixed in ergot
    // b59fd1d: exact-size grants via a scratch accumulator); and the
    // host decoded the rzcobs frames with Table::decode, which only
    // accepts unencoded frames (fixed in oxifoc-host-lib: per-message
    // stream decoder).)
    ergot::logging::defmt_sink::forward_to_ergot_topic(&consumer, stack, None).await;
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
pub async fn fast_telemetry_task(stack: &'static Stack) {
    fast_telemetry_stream::<_, MonoTimer>(stack, PWM_CONFIG.pwm_freq_hz).await;
}

/// Fault topic publisher — pushes the full fault snapshot on every
/// registry change (the remote's vibration/UI path; FaultEndpoint stays
/// the pull/clear side).
pub async fn fault_topic_task(stack: &'static Stack) {
    fault_topic_stream(stack, &FAULT_REGISTRY).await;
}

// Task spawning lives in main.rs: every worker/server here is a plain
// `async fn` wrapped by an RTIC software task (see the `app` module's
// priority map).

// ========== RTT Transport workers (feature = "transport-rtt") ==========

/// Worker task for incoming ergot data (RTT down channel)
#[cfg(feature = "transport-rtt")]

pub async fn run_rtt_rx(
    mut rcvr: crate::transport::RttRxWorker,
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

/// Worker task for outgoing ergot data (RTT up channel)
#[cfg(feature = "transport-rtt")]

pub async fn run_rtt_tx(mut tx: ergot::transport::rtt::RttWriter) {
    use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
    loop {
        let _ = tx_worker(&mut tx, crate::transport::RTT_OUTQ.stream_consumer()).await;
    }
}

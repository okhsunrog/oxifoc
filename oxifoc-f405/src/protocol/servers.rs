//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
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
#[cfg(not(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
)))]
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::runtime::streaming::fault_topic_stream;
use oxifoc_core::runtime::{DetectionBackend, run_all_servers_with_config};
use oxifoc_core::timer::EmbassyTimer;

use crate::RUNTIME_CONFIG;
use crate::calibration::{
    calibrate_hall_ez, measure_flux_linkage_ez, measure_inductance_ez, measure_resistance_ez,
};
use crate::config::{
    BOARD, MAX_PACKET_SIZE, PWM_CONFIG, UART_BAUD, USB_OUT_QUEUE_SIZE, measured_board,
};
use crate::control::foc::VBUS_MV;
use crate::transport::UART_OUTQ;

// ========== Worker Tasks ==========

/// USB device task — runs USB state machine
#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    #[cfg(feature = "bench-scheduler-deadline")]
    crate::diagnostics::deadline::register(0, 125).await;
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(3);
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
#[embassy_executor::task]
async fn run_usb_rx(rcvr: UsbRxWorker, recv_buf: &'static mut [u8]) {
    #[cfg(feature = "bench-scheduler-deadline")]
    crate::diagnostics::deadline::register(1, 125).await;
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(3);
    rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data (USB framed)
#[embassy_executor::task]
async fn run_usb_tx(
    ep_in: <AppDriver as embassy_usb::driver::Driver<'static>>::EndpointIn,
    rx: FramedConsumer<&'static UsbQueue>,
) {
    #[cfg(feature = "bench-scheduler-deadline")]
    crate::diagnostics::deadline::register(2, 125).await;
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(3);
    #[cfg(feature = "bench-usb-metrics")]
    let mut ep_in = super::usb_metrics::MeasuredIn(ep_in);
    #[cfg(not(feature = "bench-usb-metrics"))]
    let mut ep_in = ep_in;
    #[cfg(feature = "bench-usb-metrics")]
    type TxDriver = super::usb_metrics::MeasuredDriver<AppDriver>;
    #[cfg(not(feature = "bench-usb-metrics"))]
    type TxDriver = AppDriver;
    usb_kit::tx_worker::<TxDriver, { USB_OUT_QUEUE_SIZE }, _>(
        &mut ep_in,
        rx,
        usb_kit::DEFAULT_TIMEOUT_MS_PER_FRAME,
        usb_kit::USB_FS_MAX_PACKET_SIZE,
    )
    .await;
}

/// Build non-Send USB handlers on their owning executor. Ergot I/O has higher
/// priority than the optional raw telemetry endpoint.
#[cfg(feature = "transport-usb")]
#[embassy_executor::task]
pub async fn usb_io_task(
    stack: &'static Stack,
    usb: embassy_stm32::Peri<'static, embassy_stm32::peripherals::USB_OTG_FS>,
    pa12: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA12>,
    pa11: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PA11>,
) {
    let (transport, ident) = crate::transport::init_usb(stack, usb, pa12, pa11);
    defmt::unwrap!(crate::protocol::USB_IDENT.init(ident).map_err(|_| ()));
    // SAFETY: Embassy polls this task with its own task waker. The local
    // spawner and newly constructed non-Send handlers stay on this same
    // executor (USB_EXECUTOR normally, thread mode in the bench A/B build).
    let spawner = unsafe { Spawner::for_current_executor().await };
    spawner.spawn(defmt::unwrap!(usb_task(transport.usb_dev)));
    spawner.spawn(defmt::unwrap!(run_usb_rx(
        transport.rx_worker,
        crate::protocol::USB_RECV_BUF.init_with(|| [0; MAX_PACKET_SIZE])
    )));
    spawner.spawn(defmt::unwrap!(run_usb_tx(
        transport.ep_in,
        crate::transport::USB_OUTQ.framed_consumer()
    )));
    #[cfg(feature = "bench-raw-usb")]
    spawner.spawn(defmt::unwrap!(crate::transport::raw_fast::usb_task(
        transport.raw_ep_in
    )));
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
const MAX_WIRE_BYTES: usize = MAX_PACKET_SIZE + MAX_PACKET_SIZE / 254 + 1;

/// Time to transmit one max-sized frame at the configured baud rate.
/// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
const TX_TIMEOUT_US: u64 = (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (UART_BAUD as u64) * 3;

/// Worker task for outgoing ergot data (UART COBS stream)
///
/// When the interface is not Active, frames are discarded without writing to UART.
#[embassy_executor::task]
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
    #[cfg(feature = "bench-scheduler-deadline")]
    crate::diagnostics::deadline::register(3, 500).await;
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(2);
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
    #[cfg(feature = "bench-no-phase-outputs")]
    let _ = sw.push_str("+outputs-inhibited");
    #[cfg(feature = "bench-only-phase-a")]
    let _ = sw.push_str("+phase-a-only");
    let _ = mcu.push_str("STM32F405RG");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let mut device_info = HardwareInfo {
        bootstrap_magic: oxifoc_core::types::ICD_BOOTSTRAP_MAGIC,
        proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
        capabilities: BOARD.current_sense_topology.capability_bits(),
        reserved: [0; 8],
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: PWM_CONFIG.pwm_freq_hz,
        max_current_a: BOARD.max_phase_current_a,
        calib: measured_board().calib,
    };
    if let Some(phase) = measured_board().phase_sense {
        device_info.set_phase_voltage_info(oxifoc_core::types::PhaseVoltageInfo {
            divider_ratio: phase.divider_ratio,
            current_to_voltage_ns: crate::hardware::peripherals::current_to_phase_sample_ns(),
            filtered: phase.has_filters,
        });
    }

    // This future IS the protocol-servers task (all endpoint servers
    // joined); embassy arena-allocates it statically, so its size is the
    // task's intended footprint, not an accident the lint should flag.
    #[expect(clippy::large_futures, reason = "the joined servers are the task")]
    run_all_servers_with_config::<_, _, EmbassyTimer>(
        stack.endpoints(),
        device_info,
        &STATE,
        &FAULT_REGISTRY,
        &RUNTIME_CONFIG,
        PWM_CONFIG.pwm_freq_hz,
        *crate::config::CONTROL_TIMING.get().await,
        BOARD.max_phase_current_a,
        true,
    )
    .await;
}

/// Ergot well-known services for downstream bridges and host discovery.
///
/// The seed-router handler answers net-id assignment/refresh/release requests
/// (the BLE bridge leases a net for its radio segment here — without it a
/// bridge's pending downstream never gets a routable net). The ping handler
/// is the bridge's upstream-discovery bootstrap: the bridge pings the root
/// link-local, and the *reply* is the first frame addressed to the bridge,
/// which is what its edge processor learns the upstream net_id from. The
/// socket-query handler answers the host's connect-time address resolution
/// (SocketQuery by endpoint key) so a host behind a bridge can find this
/// controller instead of assuming its peer is the controller.
#[embassy_executor::task]
pub async fn seed_router_task(stack: &'static Stack) {
    embassy_futures::join::join3(
        stack.services().seed_router_request_handler::<2>(),
        stack.services().ping_handler::<2>(),
        stack.services().socket_query_handler::<2>(),
    )
    .await;
}

/// State monitor — watches interface state transitions and reacts to disconnect.
/// Stops motor and disables telemetry when ALL interfaces go down.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, idents: heapless::Vec<u8, 3>) {
    use crate::transport::STATE_NOTIFY;
    use ergot::interface_manager::{InterfaceState, Profile};
    use oxifoc_core::runtime::streaming::set_fast_telemetry_period;

    let mut any_was_active = false;

    loop {
        let any_active = defmt::unwrap!(
            STATE_NOTIFY
                .wait_for_value(|| {
                    let any_active = idents.iter().any(|&id| {
                        stack.manage_profile(|im| {
                            matches!(
                                im.interface_state(id),
                                Some(
                                    InterfaceState::Active { .. }
                                        | InterfaceState::ActiveLocal { .. }
                                )
                            )
                        })
                    });
                    (any_active != any_was_active).then_some(any_active)
                })
                .await
                .ok()
        );

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

            set_fast_telemetry_period(0).await;

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
#[embassy_executor::task]
#[allow(
    clippy::large_futures,
    reason = "Embassy statically allocates this task including the finite detection packet buffers"
)]
pub async fn detect_server(stack: &'static Stack) {
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(1);
    oxifoc_core::runtime::detect_server(
        stack.endpoints(),
        F405Backend,
        BOARD.max_phase_current_a,
        crate::config::CONTROL_TIMING.get().await.frequency_hz(),
        Some(&RUNTIME_CONFIG),
    )
    .await;
}

/// Detection backend for the F405 platform: the raw measurements bound to the
/// shared calibration `*_ez` wrappers (which use the platform ADC statics).
struct F405Backend;

impl DetectionBackend for F405Backend {
    type Timer = oxifoc_core::timer::EmbassyTimer;

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
#[embassy_executor::task]
pub async fn defmt_forwarder(
    consumer: ergot::logging::defmt_sink::DefmtConsumer,
    stack: &'static Stack,
) {
    // Profiling windows are useless if a backlogged stream starves delivery
    // until the diagnostic ring has dropped the active-state measurements.
    // This worker waits for new frames; it never polls an empty ring.
    #[cfg(all(feature = "isr-timing", feature = "task-priority"))]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(2);
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
#[cfg(not(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
)))]
#[embassy_executor::task]
pub async fn fast_telemetry_task(stack: &'static Stack) {
    #[cfg(feature = "bench-scheduler-deadline")]
    crate::diagnostics::deadline::register(4, 1000).await;
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(1);
    let hz = crate::config::CONTROL_TIMING.get().await.frequency_hz() as u32;
    fast_telemetry_stream::<_, EmbassyTimer>(stack, hz).await;
}

/// Bench session control stays on Ergot while its sole queue consumer owns the
/// selected raw sink. Normal images do not expose this endpoint server.
#[cfg(any(
    feature = "bench-raw-usb",
    feature = "bench-raw-rtt",
    feature = "bench-raw-ergot"
))]
#[embassy_executor::task]
async fn raw_telemetry_server_task(stack: &'static Stack) {
    #[cfg(feature = "task-priority")]
    embassy_executor::Metadata::for_current_task()
        .await
        .set_priority(2);
    let timing = *crate::config::CONTROL_TIMING.get().await;
    oxifoc_core::runtime::raw_stream::raw_telemetry_server::<_, 2>(
        stack.endpoints(),
        timing.frequency_hz() as u32,
        timing,
    )
    .await;
}

/// Fault topic publisher — pushes the full fault snapshot on every
/// registry change (the remote's vibration/UI path; FaultEndpoint stays
/// the pull/clear side).
#[embassy_executor::task]
pub async fn fault_topic_task(stack: &'static Stack) {
    fault_topic_stream::<_, _, EmbassyTimer>(stack, &FAULT_REGISTRY).await;
}

// ========== Task Spawning ==========

/// Spawn all protocol server tasks
pub fn spawn_servers(
    spawner: &Spawner,
    stack: &'static Stack,
    idents: &heapless::Vec<u8, 3>,
    defmt_consumer: ergot::logging::defmt_sink::DefmtConsumer,
) {
    spawner.spawn(defmt::unwrap!(protocol_servers(stack)));
    #[cfg(feature = "bench-raw-codec")]
    spawner.spawn(defmt::unwrap!(
        crate::diagnostics::codec_bench::report_task(stack)
    ));
    #[cfg(not(any(
        feature = "bench-raw-usb",
        feature = "bench-raw-rtt",
        feature = "bench-raw-ergot"
    )))]
    spawner.spawn(defmt::unwrap!(fast_telemetry_task(stack)));
    #[cfg(any(
        feature = "bench-raw-usb",
        feature = "bench-raw-rtt",
        feature = "bench-raw-ergot"
    ))]
    spawner.spawn(defmt::unwrap!(raw_telemetry_server_task(stack)));
    #[cfg(feature = "bench-raw-rtt")]
    spawner.spawn(defmt::unwrap!(crate::transport::raw_fast::rtt_task()));
    #[cfg(feature = "bench-raw-ergot")]
    spawner.spawn(defmt::unwrap!(crate::transport::raw_fast::ergot_task(
        stack
    )));
    spawner.spawn(defmt::unwrap!(fault_topic_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, idents.clone())));
    spawner.spawn(defmt::unwrap!(detect_server(stack)));
    spawner.spawn(defmt::unwrap!(seed_router_task(stack)));
    spawner.spawn(defmt::unwrap!(defmt_forwarder(defmt_consumer, stack)));
}

// ========== RTT Transport workers (feature = "transport-rtt") ==========

/// Worker task for incoming ergot data (RTT down channel)
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
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
#[embassy_executor::task]
pub async fn run_rtt_tx(mut tx: ergot::transport::rtt::RttWriter) {
    use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
    loop {
        let _ = tx_worker(&mut tx, crate::transport::RTT_OUTQ.stream_consumer()).await;
    }
}

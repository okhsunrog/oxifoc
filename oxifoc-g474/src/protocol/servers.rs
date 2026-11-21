//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use embedded_io_async::Write;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer,
    toolkits::embassy_usb_v0_6_preview as usb_kit,
};
use heapless::String;
use oxifoc_core::runtime::raw_stream::{self, ErgotRawSink};
use oxifoc_core::runtime::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fault_topic_stream;
use oxifoc_core::timer::EmbassyTimer;
use oxifoc_core::types;
use oxifoc_core::types::{ControlTiming, HardwareInfo};

use crate::RUNTIME_CONFIG;
use crate::config::{
    BOARD, LINK_BLOCK_BYTES, MAX_PACKET_SIZE, PWM_CONFIG, UART_BAUD, USB_OUT_QUEUE_SIZE,
};
use crate::transport::UART_OUTQ;

use crate::transport::{AppDriver, Stack, UartRxWorkerType, UartWriter, UsbQueue, UsbRxWorkerType};
use crate::{FAULT_REGISTRY, STATE};

#[cfg(feature = "transport-rtt")]
use {
    crate::transport::{RTT_OUTQ, RttRxWorkerType},
    ergot::transport::rtt::RttWriter,
};

// ========== Worker Tasks ==========

/// USB device task — runs the USB state machine
#[embassy_executor::task]
pub async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, AppDriver>) {
    usb.run().await;
}

/// Worker task for incoming ergot data (USB)
#[embassy_executor::task]
pub async fn run_usb_rx(rcvr: UsbRxWorkerType, recv_buf: &'static mut [u8]) {
    rcvr.run(recv_buf, usb_kit::USB_FS_MAX_PACKET_SIZE).await;
}

/// Worker task for outgoing ergot data via USB bulk endpoint
#[embassy_executor::task]
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

/// Worker task for incoming ergot data (UART / LPUART)
#[embassy_executor::task]
pub async fn run_uart_rx(
    mut rcvr: UartRxWorkerType,
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

/// Worker task for outgoing ergot data via UART/LPUART
///
/// When the interface is not Active, frames are discarded from the queue
/// without writing to UART — this prevents stale telemetry frames from
/// blocking new protocol responses after a disconnect.
#[embassy_executor::task]
pub async fn run_uart_tx(mut tx: UartWriter, stack: &'static Stack, uart_ident: u8) {
    use ergot::interface_manager::{InterfaceState, Profile};

    /// Maximum COBS-encoded frame size (the largest grant the sink can produce).
    /// Formula: n + n/254 + 1 (same as cobs::max_encoding_length)
    const MAX_WIRE_BYTES: usize = MAX_PACKET_SIZE + MAX_PACKET_SIZE / 254 + 1;

    /// Time to transmit one max-sized frame at the configured baud rate.
    /// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
    const TX_TIMEOUT_US: u64 = (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (UART_BAUD as u64) * 3;

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
                    _ => break, // Timeout or error — drop this frame
                }
            }
        }
        grant.release(len);
    }
}

// ========== RTT Transport workers (feature = "transport-rtt") ==========

/// Worker task for incoming ergot data (RTT down channel)
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
pub async fn run_rtt_rx(
    mut rcvr: RttRxWorkerType,
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
pub async fn run_rtt_tx(mut tx: RttWriter) {
    use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
    loop {
        let _ = tx_worker(&mut tx, RTT_OUTQ.stream_consumer()).await;
    }
}

/// Packed28 session stream: the only consumer of the FOC ISR's fast
/// telemetry queue, sending blocks to the node that armed the session.
#[embassy_executor::task]
pub async fn raw_telemetry_task(stack: &'static Stack) {
    let mut sink = ErgotRawSink::<_, EmbassyTimer, _>::new(stack, ());
    #[expect(
        clippy::large_futures,
        reason = "the fixed raw block belongs to this statically allocated Embassy task"
    )]
    raw_stream::raw_telemetry_stream::<_, EmbassyTimer>(&mut sink).await;
}

// ========== Protocol Servers ==========

/// All protocol servers running concurrently in a single task
#[embassy_executor::task]
pub async fn protocol_servers(stack: &'static Stack, boot_epoch: u64) {
    defmt::info!("Starting protocol servers");

    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("NUCLEO-G474RE");
    let _ = sw.push_str(concat!("oxifoc-", env!("CARGO_PKG_VERSION")));
    let _ = mcu.push_str("STM32G474RE");
    let _ = uuid.push_str(embassy_stm32::uid::uid_hex());
    let device_info = HardwareInfo {
        bootstrap_magic: types::ICD_BOOTSTRAP_MAGIC,
        proto_version: types::ICD_PROTO_VERSION,
        capabilities: BOARD.current_sense_topology.capability_bits(),
        reserved: [0; 8],
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz: PWM_CONFIG.pwm_freq_hz,
        max_current_a: BOARD.max_phase_current,
        calib: BOARD.calib,
    };

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
        ControlTiming::from_hz(PWM_CONFIG.pwm_freq_hz),
        &BOARD,
        boot_epoch,
        true,
        None,
        LINK_BLOCK_BYTES,
    )
    .await;
}

/// State monitor — watches interface state transitions and updates DeviceState.
/// Linked when ANY registered interface is active; stops motor and disables
/// telemetry when ALL of them go down. Watches every transport that was
/// registered (USB / UART / RTT), passed as their idents.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, idents: heapless::Vec<u8, 3>) {
    use crate::protocol::{DeviceState, set_device_state};
    use crate::transport::STATE_NOTIFY;
    use ergot::interface_manager::{InterfaceState, Profile};

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
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_active());
            set_device_state(DeviceState::Linked);
            any_was_active = true;
        } else if !any_active && any_was_active {
            defmt::info!("All interfaces down — stopping motor, waiting for link");
            // Fail-safe: drop link_active so the FOC loop forces ControlMode::Stopped.
            // A raw session stops on its own: its blocks time out unconsumed
            // and the stream fails the session.
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_inactive());
            set_device_state(DeviceState::WaitingLink);
            any_was_active = false;
        }
    }
}

// ========== Task Spawning ==========

/// Fault topic publisher — pushes the full fault snapshot on every
/// registry change (the remote's vibration/UI path; FaultEndpoint stays
/// the pull/clear side).
#[embassy_executor::task]
pub async fn fault_topic_task(stack: &'static Stack) {
    fault_topic_stream::<_, _, EmbassyTimer>(stack, &FAULT_REGISTRY).await;
}

/// Ergot well-known services (mirrors the f405's rationale, minus the seed
/// router — no bridge is expected downstream of this board yet): ping is the
/// bridge/upstream discovery bootstrap, socket-query answers the host's
/// connect-time address resolution.
#[embassy_executor::task]
pub async fn well_known_services_task(stack: &'static Stack) {
    embassy_futures::join::join(
        stack.services().ping_handler::<2>(),
        stack.services().socket_query_handler::<2>(),
    )
    .await;
}

pub fn spawn_servers(
    spawner: &Spawner,
    stack: &'static Stack,
    idents: &heapless::Vec<u8, 3>,
    boot_epoch: u64,
) {
    spawner.spawn(defmt::unwrap!(protocol_servers(stack, boot_epoch)));
    spawner.spawn(defmt::unwrap!(fault_topic_task(stack)));
    spawner.spawn(defmt::unwrap!(well_known_services_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, idents.clone())));
    spawner.spawn(defmt::unwrap!(raw_telemetry_task(stack)));
}

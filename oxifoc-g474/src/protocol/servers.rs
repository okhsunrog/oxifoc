//! Ergot protocol servers and I/O worker tasks

use embassy_executor::Spawner;
use embedded_io_async::Write;
use ergot::{
    exports::bbqueue::prod_cons::framed::FramedConsumer, toolkits::embassy_usb_v0_6 as usb_kit,
};
use heapless::String;
use oxifoc_core::types::HardwareInfo;

use crate::transport::{AppDriver, Stack, UartRxWorkerType, UartWriter, UsbQueue, UsbRxWorkerType};
use crate::{FAULT_REGISTRY, STATE};

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
    usb_kit::tx_worker::<AppDriver, { crate::config::USB_OUT_QUEUE_SIZE }, _>(
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
    const MAX_WIRE_BYTES: usize =
        crate::config::MAX_PACKET_SIZE + crate::config::MAX_PACKET_SIZE / 254 + 1;

    /// Time to transmit one max-sized frame at the configured baud rate.
    /// 10 bits per byte (8N1). 3x safety margin for interrupt latency.
    const TX_TIMEOUT_US: u64 =
        (MAX_WIRE_BYTES as u64 * 10 * 1_000_000) / (crate::config::UART_BAUD as u64) * 3;

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
                    _ => break, // Timeout or error — drop this frame
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
    let _ = hw.push_str("NUCLEO-G474RE");
    let _ = sw.push_str("oxifoc-0.1.0");
    let _ = mcu.push_str("STM32G474RE");
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

/// State monitor — watches interface state transitions and updates DeviceState.
/// Stops motor and disables telemetry when ALL interfaces go down.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, usb_ident: u8, uart_ident: u8) {
    use crate::protocol::{DeviceState, set_device_state};
    use crate::transport::STATE_NOTIFY;
    use ergot::interface_manager::{InterfaceState, Profile};

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
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_active());
            set_device_state(DeviceState::Linked);
            any_was_active = true;
        } else if !any_active && any_was_active {
            defmt::info!("All interfaces down — stopping motor, waiting for link");
            // Fail-safe: drop link_active so the FOC loop forces ControlMode::Stopped.
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_inactive());
            set_device_state(DeviceState::WaitingLink);
            any_was_active = false;
        }
    }
}

// ========== Task Spawning ==========

pub fn spawn_servers(spawner: &Spawner, stack: &'static Stack, usb_ident: u8, uart_ident: u8) {
    spawner.spawn(protocol_servers(stack).unwrap());
    spawner.spawn(state_monitor(stack, usb_ident, uart_ident).unwrap());
}

//! Ergot protocol servers and I/O worker tasks (RTT transport).

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use ergot::interface_manager::InterfaceState;
use ergot::transport::rtt::RttWriter;
use heapless::String;
use oxifoc_core::runtime::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::{
    FAST_TELEM_PERIOD, fast_telemetry_stream, fault_topic_stream, push_fast_telemetry,
};
use oxifoc_core::timer::EmbassyTimer;
use oxifoc_core::types::{FastTelemetry, HardwareInfo};

use crate::RUNTIME_CONFIG;
use crate::config::{BOARD, MAX_PACKET_SIZE, PWM_CONFIG};
use crate::transport::{OUTQ, RxWorker, Stack};
use crate::{FAULT_REGISTRY, STATE};

// ========== Worker Tasks ==========

/// Worker task for incoming ergot data (RTT down channel)
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

/// Worker task for outgoing ergot data (RTT up channel)
#[embassy_executor::task]
pub async fn run_tx_rtt(mut tx: RttWriter) {
    use ergot::toolkits::embedded_io_async_v0_7::tx_worker;
    loop {
        let _ = tx_worker(&mut tx, OUTQ.stream_consumer()).await;
    }
}

/// Fast telemetry streaming task — drains the bbqueue and broadcasts batches.
/// Batch 64 of the 8 B raw frame (512 B/batch) amortises ergot/COBS overhead.
#[embassy_executor::task]
pub async fn fast_telemetry_task(stack: &'static Stack) {
    fast_telemetry_stream::<_, 64, EmbassyTimer>(stack, 400000).await;
}

/// Synthetic telemetry generator (this board has no FOC ISR producing samples).
///
/// Saturates the telemetry bbqueue so the RTT path is the only bottleneck:
/// the host-measured samples/s then equals the achievable RTT throughput.
/// Each loop pushes a burst of frames (overflow is silently dropped by the
/// queue) then yields, keeping the queue fed faster than RTT can drain it.
/// Disabled until the host sends `TelemetryConfig` (sets `FAST_TELEM_PERIOD`).
#[embassy_executor::task]
pub async fn fake_telemetry_gen() {
    use core::sync::atomic::Ordering;
    let mut seq: u32 = 0;
    loop {
        if FAST_TELEM_PERIOD.load(Ordering::Relaxed) == 0 {
            // Streaming off — poll for the host to enable it.
            Timer::after(Duration::from_millis(10)).await;
            continue;
        }
        // Push a burst, then yield so the stream/tx tasks run. The burst size
        // exceeds the bbqueue depth, so this reliably keeps it full.
        for _ in 0..32 {
            seq = seq.wrapping_add(1);
            let s = seq as u16;
            // Realistic value ranges so postcard varint sizing matches a real
            // device (12-bit ADC currents, small dq volts, ~24 V bus); angle and
            // seq genuinely span the full u16 (worst-case 3-byte varints).
            let t = FastTelemetry {
                ia: 2048u16.wrapping_add(s & 0x3FF), // ~12-bit ADC counts
                ib: 2048u16.wrapping_sub(s & 0x3FF),
                ic: 2048u16.wrapping_add(s & 0x1FF),
                vbus: 6_000, // 12 V in 2-mV units
                angle: s,    // full electrical sweep
                vd: (s & 0x3FF) as i16, // small dq volts
                vq: (s & 0x1FF) as i16,
                rpm: (s & 0x7FF) as i16,
                seq: s,
            };
            push_fast_telemetry(&t);
        }
        embassy_futures::yield_now().await;
    }
}

/// Fault topic publisher — pushes the full fault snapshot on every
/// registry change.
#[embassy_executor::task]
pub async fn fault_topic_task(stack: &'static Stack) {
    fault_topic_stream(stack, &FAULT_REGISTRY).await;
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
        foc_freq_hz: PWM_CONFIG.pwm_freq_hz,
        max_current_a: BOARD.max_phase_current_a,
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

/// State monitor — watches interface state transitions and updates DeviceState.
/// Disables telemetry when the link goes down.
#[embassy_executor::task]
pub async fn state_monitor(stack: &'static Stack, ident: u8) {
    use crate::protocol::{DeviceState, set_device_state};
    use crate::transport::STATE_NOTIFY;
    use ergot::interface_manager::Profile;

    let mut was_active = false;

    loop {
        defmt::unwrap!(STATE_NOTIFY.wait().await.ok());

        let active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(ident),
                Some(InterfaceState::Active { .. })
            )
        });

        if active && !was_active {
            defmt::info!("RTT interface active — link up");
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_active());
            set_device_state(DeviceState::Linked);
            was_active = true;
        } else if !active && was_active {
            defmt::info!("RTT interface down — stopping telemetry, waiting for link");
            critical_section::with(|cs| STATE.borrow(cs).borrow_mut().set_link_inactive());
            FAST_TELEM_PERIOD.store(0, core::sync::atomic::Ordering::Relaxed);
            set_device_state(DeviceState::WaitingLink);
            was_active = false;
        }
    }
}

// ========== Task Spawning ==========

pub fn spawn_servers(spawner: &Spawner, stack: &'static Stack, ident: u8) {
    spawner.spawn(defmt::unwrap!(protocol_servers(stack)));
    spawner.spawn(defmt::unwrap!(fast_telemetry_task(stack)));
    spawner.spawn(defmt::unwrap!(fake_telemetry_gen()));
    spawner.spawn(defmt::unwrap!(fault_topic_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, ident)));
}

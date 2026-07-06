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
#[cfg(feature = "detection")]
use oxifoc_core::foc::detection::DetectionError;
#[cfg(feature = "detection")]
use oxifoc_core::foc::detection::types::{FluxLinkageParams, InductanceParams, ResistanceParams};
#[cfg(feature = "detection")]
use oxifoc_core::foc::hall_calibration::{HallCalibrationParams, HallCalibrationResult};
#[cfg(feature = "detection")]
use oxifoc_core::runtime::DetectionBackend;
use oxifoc_core::runtime::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::{fast_telemetry_stream, fault_topic_stream};
use oxifoc_core::timer::EmbassyTimer;
use oxifoc_core::types::HardwareInfo;

#[cfg(feature = "detection")]
use crate::calibration::{
    calibrate_hall, measure_flux_linkage, measure_inductance, measure_resistance,
};
use crate::config::{BOARD, PWM_CONFIG};
#[cfg(feature = "detection")]
use crate::cordic::CordicSinCos;
#[cfg(feature = "detection")]
use crate::foc::VBUS_MV;
use crate::transport::OUTQ;

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
    let consumer = OUTQ.stream_consumer();
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
///
/// Thread executor, NOT ergot's tx_worker: that loop (wait_read → write →
/// release) has no Pending await while the queue holds data — RTT writes
/// complete synchronously, so on this single cooperative executor it
/// monopolizes the CPU until OUTQ fully drains. Yield after every full
/// chunk; back off 500 µs after a partial one (RTT ring full to the byte,
/// host frees space only per SWD poll — sleeping costs no throughput).
///
/// NOTE (2026-07-05): running this on a medium-priority InterruptExecutor
/// (SAI1) was tried and REVERTED. It fixed the TX starvation (host rate
/// 10.5k → 14.6k under detection load), but with tx off the thread executor
/// ALL embassy-time thread timers froze for a deterministic ~44.93 s during
/// detection+streaming (1 Hz stats task, the detect ramp's 4 ms timers, a
/// dedicated 2 ms keeper task — all dead until unrelated RX traffic revived
/// them). This loop's frequent short backoff timers were evidently what kept
/// the time-driver alarm re-armed. Root cause in the embassy-stm32 gp16 time
/// driver (or our use of it) not yet found — needs an rtos-trace session;
/// see docs/TODO.md.
#[cfg(feature = "transport-rtt")]
#[embassy_executor::task]
pub async fn run_tx_rtt(mut tx: RttWriter, stack: &'static Stack, ident: u8) {
    let _ = (stack, ident);
    let consumer = OUTQ.stream_consumer();
    // Self-reported 1 Hz loop stats (the thread-mode stats task can starve —
    // this task must be able to testify about itself).
    let mut iters: u32 = 0;
    let mut sleeps: u32 = 0;
    let mut bytes: u32 = 0;
    let mut last_report = embassy_time::Instant::now();
    loop {
        let data = consumer.wait_read().await;
        let len = data.len();
        let used = tx.write(&data).await.unwrap_or(0);
        data.release(used);
        iters = iters.wrapping_add(1);
        bytes = bytes.wrapping_add(used as u32);
        if used < len {
            sleeps = sleeps.wrapping_add(1);
            embassy_time::Timer::after_micros(500).await;
        } else {
            // Made progress; let the stream/rx/server tasks run before the
            // next chunk so a long queue drain can't monopolize the executor.
            embassy_futures::yield_now().await;
        }
        let now = embassy_time::Instant::now();
        if now.duration_since(last_report).as_millis() >= 1000 {
            defmt::info!("tx/s: iters={} sleeps={} bytes={}", iters, sleeps, bytes);
            iters = 0;
            sleeps = 0;
            bytes = 0;
            last_report = now;
        }
    }
}

/// 1 Hz fast-telemetry pipeline stats over defmt (drops attributed per stage).
/// Only logs while the stream is active (any counter moved).
#[embassy_executor::task]
pub async fn telem_stats_task() {
    use core::sync::atomic::Ordering;
    use oxifoc_core::runtime::streaming::fast_telem_stats as s;
    loop {
        embassy_time::Timer::after_secs(1).await;
        let pok = s::PUSH_OK.swap(0, Ordering::Relaxed);
        let drops = s::PUSH_DROPS.swap(0, Ordering::Relaxed);
        let rok = s::READ_OK.swap(0, Ordering::Relaxed);
        let rbad = s::READ_BADLEN.swap(0, Ordering::Relaxed);
        let bfail = s::BCAST_FAILS.swap(0, Ordering::Relaxed);
        let bok = s::BCAST_OK.swap(0, Ordering::Relaxed);
        if pok != 0 || drops != 0 || bfail != 0 || bok != 0 {
            defmt::info!(
                "telem/s: push_ok={} push_drops={} read_ok={} read_badlen={} bcast_fail={} bcast_ok={}",
                pok,
                drops,
                rok,
                rbad,
                bfail,
                bok
            );
        }
        // Host→device command path: MotorEndpoint requests seen by the
        // server vs SetModes drained by the ISR. Printed whenever the fast
        // stream is up (zeros included — a silent RX outage under host
        // affirms is exactly the case this line exists to catch).
        {
            use oxifoc_core::runtime::streaming::cmd_stats as c;
            let reqs = c::MOTOR_REQS.swap(0, Ordering::Relaxed);
            let drained = c::SETMODE_DRAINED.swap(0, Ordering::Relaxed);
            if pok != 0 || reqs != 0 || drained != 0 {
                defmt::info!("rx/s: motor_reqs={} setmode_drained={}", reqs, drained);
            }
        }
        // ISR cost (DWT cycles at 170 MHz): avg/max per cycle + CPU share.
        let cyc_sum = crate::foc::ISR_CYC_SUM.swap(0, Ordering::Relaxed);
        let cyc_max = crate::foc::ISR_CYC_MAX.swap(0, Ordering::Relaxed);
        let cyc_n = crate::foc::ISR_CYC_N.swap(0, Ordering::Relaxed);
        if cyc_n != 0 {
            defmt::info!(
                "isr/s: n={} avg={} max={} load_pct={}",
                cyc_n,
                cyc_sum / cyc_n,
                cyc_max,
                cyc_sum / 1_700_000
            );
        }
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
    let _ = sw.push_str(concat!("oxifoc-", env!("CARGO_PKG_VERSION")));
    let _ = mcu.push_str("STM32G431CB");
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
        // No flash persistence on this board — the config server reports
        // persist-capable = false and serves the RAM copy only.
        false,
    )
    .await;
}

/// Fast telemetry streaming task — drains bbqueue and broadcasts batches.
/// Batch capacity is fixed in core (raw-Pod, compile-time wire size); the
/// static MTU fit is asserted in config.rs.
#[embassy_executor::task]
pub async fn fast_telemetry_task(stack: &'static Stack) {
    fast_telemetry_stream::<_, EmbassyTimer>(stack, PWM_CONFIG.pwm_freq_hz).await;
}

/// Fault topic publisher — pushes the full fault snapshot on every
/// registry change (the remote's vibration/UI path; FaultEndpoint stays
/// the pull/clear side).
#[embassy_executor::task]
pub async fn fault_topic_task(stack: &'static Stack) {
    fault_topic_stream(stack, &FAULT_REGISTRY).await;
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
impl DetectionBackend for G431Backend {
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
        measure_resistance(params)
    }
    fn measure_inductance(
        &mut self,
        params: &InductanceParams,
        pwm_freq_hz: f32,
    ) -> impl Future<Output = Result<(f32, f32), DetectionError>> {
        measure_inductance::<CordicSinCos>(params, pwm_freq_hz)
    }
    fn measure_flux(
        &mut self,
        params: &FluxLinkageParams,
    ) -> impl Future<Output = Result<f32, DetectionError>> {
        measure_flux_linkage(params)
    }
    fn calibrate_hall(
        &mut self,
        params: HallCalibrationParams,
    ) -> impl Future<Output = Result<HallCalibrationResult, DetectionError>> {
        calibrate_hall(params)
    }
}

#[cfg(feature = "detection")]
#[embassy_executor::task]
pub async fn detect_server(stack: &'static Stack) {
    oxifoc_core::runtime::detect_server(
        stack.endpoints(),
        G431Backend,
        BOARD.max_phase_current_a.min(3.0),
        PWM_CONFIG.pwm_freq_hz,
        Some(&RUNTIME_CONFIG),
    )
    .await;
}

// ========== Task Spawning ==========

pub fn spawn_servers(spawner: &Spawner, stack: &'static Stack, ident: u8) {
    spawner.spawn(defmt::unwrap!(protocol_servers(stack)));
    spawner.spawn(defmt::unwrap!(fast_telemetry_task(stack)));
    spawner.spawn(defmt::unwrap!(fault_topic_task(stack)));
    spawner.spawn(defmt::unwrap!(state_monitor(stack, ident)));
    #[cfg(feature = "detection")]
    spawner.spawn(defmt::unwrap!(detect_server(stack)));
}

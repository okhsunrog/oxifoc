//! Telemetry streaming (push-based via Topics)
//!
//! Fast telemetry is pushed continuously via ergot Topics.
//! Slow telemetry is poll-based (endpoint server in servers.rs).

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use ergot::net_stack::NetStackHandle;

use crate::foc::controller::FocOutput;
use crate::icd::{FastTelemetry, FastTelemetryTopic};
use crate::state::MotorControlState;

use super::servers::FAST_TELEM_DIVIDER;

/// Run fast telemetry streaming task.
///
/// Subscribes to the TELEMETRY watch (updated every FOC cycle by ISR),
/// decimates according to `FAST_TELEM_DIVIDER`, and broadcasts
/// `FastTelemetry` via the topic.
///
/// # Arguments
/// * `stack` - Ergot net stack for topic broadcasting
/// * `telemetry_watch` - Watch that ISR writes FocOutput to every FOC cycle
/// * `state_mutex` - For reading hall state
pub async fn fast_telemetry_stream<NS>(
    stack: NS,
    telemetry_watch: &'static Watch<CriticalSectionRawMutex, FocOutput, 2>,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
) where
    NS: NetStackHandle + Clone,
{
    let mut receiver = match telemetry_watch.receiver() {
        Some(r) => r,
        None => {
            #[cfg(feature = "log")]
            log::error!("Failed to create TELEMETRY watch receiver (max receivers reached)");
            return;
        }
    };
    let mut cycle_count: u32 = 0;
    let mut seq: u32 = 0;

    #[cfg(feature = "log")]
    log::info!("fast_telemetry_stream: waiting for first FOC output...");

    loop {
        let foc = receiver.changed().await;
        cycle_count = cycle_count.wrapping_add(1);

        #[cfg(feature = "log")]
        if cycle_count == 1 {
            log::info!("fast_telemetry_stream: received first FOC output");
        }

        let divider = FAST_TELEM_DIVIDER.load(Ordering::Relaxed) as u32;
        if divider == 0 || cycle_count % divider != 0 {
            continue;
        }

        // Read hall state from shared state
        let hall_state = critical_section::with(|cs| {
            state_mutex
                .borrow(cs)
                .borrow()
                .last_hall
                .map(|h| h.state)
                .unwrap_or(0)
        });

        // Read velocity for ERPM calculation
        let velocity_rad_s = critical_section::with(|cs| {
            state_mutex
                .borrow(cs)
                .borrow()
                .last_hall
                .map(|h| h.velocity_rad_s)
                .unwrap_or(0.0)
        });

        seq = seq.wrapping_add(1);

        let msg = FastTelemetry {
            ia_ma: (foc.ia * 1000.0) as i32,
            ib_ma: (foc.ib * 1000.0) as i32,
            ic_ma: (foc.ic * 1000.0) as i32,
            id_ma: (foc.id * 1000.0) as i32,
            iq_ma: (foc.iq * 1000.0) as i32,
            vd_mv: (foc.vd * 1000.0) as i32,
            vq_mv: (foc.vq * 1000.0) as i32,
            angle_mrad: (foc.angle_rad * 1000.0) as i32,
            erpm: (velocity_rad_s * 60.0 / core::f32::consts::TAU) as i32,
            duty_x10: 0, // TODO: compute from duties when available
            hall_state,
            seq,
        };

        let result = stack.stack().topics().broadcast::<FastTelemetryTopic>(&msg, None);

        #[cfg(feature = "log")]
        if seq <= 3 {
            log::info!("fast_telemetry broadcast seq={} result={:?}", seq, result);
        }
    }
}


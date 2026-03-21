//! Telemetry streaming tasks (push-based via Topics)
//!
//! These tasks read from the global state and broadcast telemetry
//! via ergot Topics. Unlike request/response servers, these push
//! data continuously at configurable rates.

use core::cell::RefCell;
use core::sync::atomic::Ordering;

use critical_section::Mutex as CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use ergot::net_stack::NetStackHandle;

use crate::foc::controller::FocOutput;
use crate::foc::fault::{FaultRegistry, PlatformFault};
use crate::icd::{FastTelemetry, FastTelemetryTopic, SlowTelemetry, SlowTelemetryTopic};
use crate::state::MotorControlState;

use super::servers::{FAST_TELEM_DIVIDER, SLOW_TELEM_RATE_HZ};

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
    let mut receiver = telemetry_watch.receiver().unwrap();
    let mut cycle_count: u32 = 0;
    let mut seq: u32 = 0;

    loop {
        let foc = receiver.changed().await;
        cycle_count = cycle_count.wrapping_add(1);

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

        let _ = stack.stack().topics().broadcast::<FastTelemetryTopic>(&msg, None);
    }
}

/// Run slow telemetry streaming task.
///
/// Periodically reads system health data from state and broadcasts
/// `SlowTelemetry` via the topic at `SLOW_TELEM_RATE_HZ`.
///
/// # Arguments
/// * `stack` - Ergot net stack for topic broadcasting
/// * `state_mutex` - For reading ADC/temperature/motor state
/// * `fault_registry` - For fault count
pub async fn slow_telemetry_stream<NS, F>(
    stack: NS,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<F>,
) where
    NS: NetStackHandle + Clone,
    F: PlatformFault,
{
    let mut seq: u32 = 0;

    loop {
        let rate_hz = SLOW_TELEM_RATE_HZ.load(Ordering::Relaxed).max(1);
        let interval_ms = 1000u64 / rate_hz as u64;

        embassy_time::Timer::after(embassy_time::Duration::from_millis(interval_ms)).await;

        let (vbus_mv, fet_temp, motor_temp, board_temp, motor_state, control_mode) =
            critical_section::with(|cs| {
                let state = state_mutex.borrow(cs).borrow();
                (
                    state.last_adc.vbus_mv,
                    state.last_adc.fet_temp_c_x10().unwrap_or(0),
                    state.last_adc.motor_temp_c_x10().unwrap_or(0),
                    state.last_adc.board_temp_c_x10().unwrap_or(0),
                    state.motor_state,
                    state.control_mode,
                )
            });

        seq = seq.wrapping_add(1);

        let msg = SlowTelemetry {
            vbus_mv,
            fet_temp_c_x10: fet_temp,
            motor_temp_c_x10: motor_temp,
            board_temp_c_x10: board_temp,
            motor_state,
            control_mode,
            fault_count: fault_registry.count() as u8,
            seq,
        };

        let _ = stack.stack().topics().broadcast::<SlowTelemetryTopic>(&msg, None);
    }
}

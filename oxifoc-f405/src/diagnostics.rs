//! Hall crossover tuning diagnostics (`hall-diagnostics`): finite SWD
//! captures and a low-rate Hall log. No per-event logging or USB traffic.

pub mod hall_edges;

/// Low-rate observations tied to the same ADC sequence as fast telemetry.
/// No GPIO/USB logging is added to the control or Hall capture interrupts.
#[embassy_executor::task]
pub async fn hall_task() {
    use core::sync::atomic::Ordering;
    use oxifoc_core::motor::ControlMode;
    loop {
        // These are fresh observations, not a history buffer. If streaming
        // delays this task, skip missed observations instead of logging a
        // burst of nearly identical snapshots while catching up.
        embassy_time::Timer::after_millis(20).await;
        let snapshot = critical_section::with(|cs| {
            let state = crate::STATE.borrow(cs).borrow();
            (state.last_adc.seq, state.last_hall, state.control_mode)
        });
        if let (seq, Some(hall), mode) = snapshot
            && mode != ControlMode::Stopped
        {
            defmt::info!(
                "hall: seq={} raw={} logical={} angle={} omega={} errors={} over={}",
                seq,
                crate::sensors::hall::read_hall_state_raw(),
                hall.state,
                hall.angle,
                hall.velocity,
                hall.error_count,
                crate::sensors::hall::OVERCAPTURES.load(Ordering::Relaxed),
            );
        }
    }
}

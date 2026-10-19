//! Reactive open-loop override used when a hall source drops out.

use super::PhaseManager;
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    /// Update open-loop override state (advance angle, decrement timer).
    ///
    /// The override stays active until a real source (hall or a ready
    /// observer) takes over — `try_observer_fallback` deactivates it then.
    /// The timer is purely informational dwell time; it does NOT terminate
    /// the override, because dropping to a dead sensor at its expiry would
    /// be strictly worse than continuing open loop (VESC keeps its
    /// observer-override running for the same reason).
    pub(super) fn update_open_loop_override(&mut self, dt: f32) {
        if self.open_loop_override.active {
            // Advance angle based on velocity
            self.open_loop_override.angle += self.open_loop_override.velocity * dt;
            self.open_loop_override.angle = wrap_angle(self.open_loop_override.angle);

            self.open_loop_override.timer = (self.open_loop_override.timer - dt).max(0.0);
        }
    }
}

//! Position control: an unwrapped rotor position and the P law that turns a
//! position error into a velocity target for the velocity loop (cascade:
//! position P → [`VelocityLoop`](crate::foc::velocity::VelocityLoop) →
//! current).
//!
//! Units: electrical radians, like everything the phase provider measures;
//! hosts convert mechanical positions via pole pairs.
//!
//! The resolution is the angle source's: a hall sensor reports its sector
//! centroid below the interpolation speed, so a standing rotor is known to
//! ±30° electrical (±2.6 mm of travel on a 5065 / 36:15 / 83 mm board).

use core::f32::consts::TAU;

use crate::foc::fast_math::sqrtf;
use crate::foc::{angle_difference, wrap_angle};

/// An unwrapped electrical position: whole turns plus the angle within the
/// turn. A plain f32 accumulator loses the fraction over a ride (at 10⁵ rad
/// its resolution is already 8 mrad); here only differences of nearby
/// positions ever become f32.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Position {
    turns: i32,
    /// Angle within the turn, `[0, 2π)`
    angle: f32,
}

impl Position {
    /// `rad` electrical radians from the origin.
    pub fn from_rad(rad: f32) -> Self {
        Self::default().advanced(rad)
    }

    /// Radians from the origin — lossy far from it (telemetry, host
    /// commands).
    pub fn as_rad(self) -> f32 {
        self.turns as f32 * TAU + self.angle
    }

    /// This position moved by `delta` radians.
    pub fn advanced(self, delta: f32) -> Self {
        let a = self.angle + delta;
        // Hot path (the tracker's per-cycle step): within this turn or the
        // neighbouring one — no floorf in the ISR.
        if (0.0..TAU).contains(&a) {
            return Self {
                turns: self.turns,
                angle: a,
            };
        }
        if (TAU..2.0 * TAU).contains(&a) {
            return Self {
                turns: self.turns.wrapping_add(1),
                angle: a - TAU,
            };
        }
        if (-TAU..0.0).contains(&a) && a + TAU < TAU {
            return Self {
                turns: self.turns.wrapping_sub(1),
                angle: a + TAU,
            };
        }
        let whole = core::f32::math::floor(a / TAU);
        let turns = self.turns.wrapping_add(whole as i32);
        let angle = a - whole * TAU;
        // `a − whole·2π` can round to exactly 2π for a tiny negative `a`.
        if angle >= TAU {
            Self {
                turns: turns.wrapping_add(1),
                angle: 0.0,
            }
        } else {
            Self { turns, angle }
        }
    }

    /// `self − other` in radians.
    pub fn minus(self, other: Self) -> f32 {
        self.turns.wrapping_sub(other.turns) as f32 * TAU + (self.angle - other.angle)
    }
}

/// Follows the wrapped angle source into a [`Position`].
#[derive(Debug, Default)]
pub struct PositionTracker {
    position: Position,
    last_angle: Option<f32>,
}

impl PositionTracker {
    /// Feed this cycle's angle. Assumes less than half a turn per cycle
    /// (π·20 kHz ≈ 63 000 el rad/s — far past any motor here).
    pub fn update(&mut self, angle: f32) {
        if let Some(last) = self.last_angle {
            self.position = self.position.advanced(angle_difference(angle, last));
        } else {
            self.position = Position {
                turns: self.position.turns,
                angle: wrap_angle(angle),
            };
        }
        self.last_angle = Some(angle);
    }

    pub fn position(&self) -> Position {
        self.position
    }
}

/// Position-loop tuning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionLoopConfig {
    /// Proportional gain: el rad/s of velocity target per el rad of
    /// position error (1/s — the inverse of the approach time constant).
    pub kp: f32,
    /// Travel speed ceiling (el rad/s); `0` = none (the velocity loop and
    /// the derating speed ceiling still bound it).
    pub max_speed: f32,
}

impl Default for PositionLoopConfig {
    fn default() -> Self {
        Self {
            kp: 10.0,
            max_speed: 1_000.0,
        }
    }
}

impl PositionLoopConfig {
    /// Finite, `kp > 0`, `max_speed >= 0`.
    pub fn is_sane(&self) -> bool {
        self.kp.is_finite() && self.max_speed.is_finite() && self.kp > 0.0 && self.max_speed >= 0.0
    }

    /// The velocity target for position error `err` (rad): `kp·err`, no
    /// faster than a stop at `decel` (el rad/s², the velocity loop's ramp)
    /// can still end at the target — `√(2·decel·|err|)` — and no faster
    /// than `max_speed`. `decel <= 0` = unbounded.
    pub fn speed_for(&self, err: f32, decel: f32) -> f32 {
        let e = err.abs();
        let mut v = self.kp * e;
        if decel > 0.0 {
            v = v.min(sqrtf(2.0 * decel * e));
        }
        if self.max_speed > 0.0 {
            v = v.min(self.max_speed);
        }
        if err < 0.0 { -v } else { v }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn position_keeps_turns_and_fraction_apart() {
        let p = Position::from_rad(5.0 * TAU + 1.0);
        assert_eq!(p.turns, 5);
        assert!((p.angle - 1.0).abs() < 1e-5);
        let q = Position::from_rad(-0.5);
        assert_eq!(q.turns, -1);
        assert!((q.angle - (TAU - 0.5)).abs() < 1e-6);
        assert!((q.minus(p) - (-0.5 - 5.0 * TAU - 1.0)).abs() < 1e-4);
        // Far from the origin the difference of neighbours stays exact to
        // f32 angle precision, where a plain accumulator would not.
        let far = Position::from_rad(0.0).advanced(1e6 * TAU);
        let near = far.advanced(0.001);
        assert!(
            (near.minus(far) - 0.001).abs() < 1e-6,
            "{}",
            near.minus(far)
        );
    }

    #[test]
    fn tracker_unwraps_across_the_seam() {
        let mut t = PositionTracker::default();
        let mut angle = 0.0f32;
        t.update(angle);
        // Three turns forward in 0.3 rad steps, then one back.
        for _ in 0..63 {
            angle = wrap_angle(angle + 0.3);
            t.update(angle);
        }
        assert!((t.position().as_rad() - 63.0 * 0.3).abs() < 1e-3);
        for _ in 0..21 {
            angle = wrap_angle(angle - 0.3);
            t.update(angle);
        }
        assert!((t.position().as_rad() - 42.0 * 0.3).abs() < 1e-3);
        // A step just under half a turn is still read the right way.
        t.update(wrap_angle(angle - (PI - 0.01)));
        assert!((t.position().as_rad() - (42.0 * 0.3 - PI + 0.01)).abs() < 1e-3);
    }

    #[test]
    fn speed_law_stops_within_the_ramp() {
        let cfg = PositionLoopConfig {
            kp: 10.0,
            max_speed: 1_000.0,
        };
        // Close in: the P term.
        assert!((cfg.speed_for(0.5, 2_000.0) - 5.0).abs() < 1e-4);
        assert!((cfg.speed_for(-0.5, 2_000.0) + 5.0).abs() < 1e-4);
        // Mid-range: the stopping-distance bound √(2·a·e).
        assert!((cfg.speed_for(100.0, 2_000.0) - 632.46).abs() < 0.01);
        // Far: the travel ceiling.
        assert_eq!(cfg.speed_for(1e4, 2_000.0), 1_000.0);
        // No decel bound: P then ceiling.
        assert_eq!(cfg.speed_for(50.0, 0.0), 500.0);
    }
}

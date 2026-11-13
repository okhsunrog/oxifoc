//! Current-limit configuration and pure current constraints.

use crate::foc::clamp_f32;
use crate::foc::fast_math::sqrtf;
#[cfg(feature = "storage")]
use crate::storage::CurrentLimitsConfig;

/// Explicit source capability for an energized commissioning session.
/// Unlike legacy configuration, this value has no unlimited sentinel.
/// Construction is required after decoding a request; do not deserialize it
/// directly. These limits concern DC current, not phase-current amplitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowerSourceLimits {
    input_a: f32,
    regen_a: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidSourceLimits {
    Input,
    Regeneration,
}

impl PowerSourceLimits {
    pub fn new(input_a: f32, regen_a: f32) -> Result<Self, InvalidSourceLimits> {
        if !input_a.is_finite() || input_a <= 0.0 {
            return Err(InvalidSourceLimits::Input);
        }
        if !regen_a.is_finite() || regen_a < 0.0 {
            return Err(InvalidSourceLimits::Regeneration);
        }
        Ok(Self { input_a, regen_a })
    }

    pub fn input_a(self) -> f32 {
        self.input_a
    }
    pub fn regen_a(self) -> f32 {
        self.regen_a
    }
}

/// Independent measured dq-magnitude trip. The board's per-phase trip is
/// separate and remains enforced by the platform control-cycle boundary.
#[derive(Clone, Copy, Debug)]
pub(super) struct DqCurrentProtection {
    threshold_a: f32,
}

impl DqCurrentProtection {
    fn exceeded(self, id: f32, iq: f32) -> bool {
        if !id.is_finite() || !iq.is_finite() {
            return true;
        }
        self.threshold_a > 0.0 && id * id + iq * iq > self.threshold_a * self.threshold_a
    }
}

/// Current limiting configuration for the FOC driver.
///
/// Three layers of protection:
/// 1. **Target clamp**: limits what the PI controller is asked to do (prevents
///    absurd commands). Uses circular clamp with d-axis priority.
/// 2. **Bus (supply) current clamp**: VESC-style `i_bus ≈ iq·mod_q` bound on
///    the iq target — phase and bus current differ by the duty factor, so a
///    battery/PSU limit cannot be expressed as a phase limit. Regen side is
///    separate: `bus_regen_max_a = 0` forbids pushing energy into the supply
///    at all (a lab PSU cannot absorb reverse current).
/// 3. **Measured overcurrent**: checks actual dq current magnitude after the
///    FOC step. If it exceeds `overcurrent_threshold`, PWM is disabled and
///    an error is returned. This is the software equivalent of hardware
///    overcurrent protection for boards that lack it.
#[derive(Clone, Copy, Debug)]
pub struct CurrentLimits {
    /// Maximum current target magnitude (A). The PI controller will never
    /// be asked to produce more than this. Set from BoardConfig or
    /// CurrentLimitsConfig. 0 = no limit.
    pub max_current_a: f32,
    /// Maximum braking q current (A): the limit whenever iq opposes the
    /// rotation. At most `max_current_a`.
    pub max_brake_a: f32,
    /// Hard overcurrent threshold on measured current (A). If actual
    /// sqrt(id² + iq²) exceeds this, PWM is immediately disabled.
    /// Typically set to 1.2-1.5× max_current_a. 0 = no limit.
    pub overcurrent_threshold_a: f32,
    /// Maximum supply (bus) current draw (A). `< 0` = unlimited.
    /// Note the different "off" sentinel from the phase limits: here 0 is a
    /// meaningful value (no draw allowed), so "unlimited" must be negative.
    pub bus_in_max_a: f32,
    /// Maximum regen (charge) current into the supply (A, positive
    /// magnitude). `< 0` = unlimited; **0 = no regen** (lab-PSU safe:
    /// current braking degrades to what winding losses absorb, the
    /// windings-short parking brake is unaffected — it never touches the
    /// bus).
    pub bus_regen_max_a: f32,
    /// Heat budget (W, `1.5·R·id²`) for burning refused regen in the
    /// windings (`motor::dissipation`); the d current it buys stays inside
    /// the `max_current_a` circle. `0` = off.
    pub max_dissipation_w: f32,
}

impl Default for CurrentLimits {
    fn default() -> Self {
        // Conservative bench-safe fallback, NOT "protection off": a platform
        // that forgets set_current_limits() gets a 5 A motor, not an
        // unprotected one. Explicit zeros still mean "no limit" where a
        // test or sim genuinely wants that.
        Self::from_max_current(5.0)
    }
}

/// Required ratio between the overcurrent trip and the soft iq ceiling.
/// The band between them absorbs what a legitimate full-throttle command
/// adds on top of its target: PI overshoot on steps, the HFI carrier
/// ripple (~2 A target), measurement noise. A config that places the Kill
/// line inside that band turns full throttle into a nuisance trip — the
/// config server rejects such writes (`CurrentLimitsConfig::is_coherent`)
/// and `from_config_clamped` clamps whatever arrives by other paths.
pub const OVERCURRENT_HEADROOM: f32 = 1.3;

impl CurrentLimits {
    /// Install an explicitly validated source capability without changing
    /// torque or inverter protection. Kept here as the adapter to the legacy
    /// flattened driver/config representation during the commissioning migration.
    pub fn with_power_source(mut self, source: PowerSourceLimits) -> Self {
        self.bus_in_max_a = source.input_a();
        self.bus_regen_max_a = source.regen_a();
        self
    }

    /// Create current limits from a maximum current value. Sets the
    /// overcurrent threshold [`OVERCURRENT_HEADROOM`] above the max;
    /// bus limits and dissipation off.
    pub fn from_max_current(max_a: f32) -> Self {
        Self {
            max_current_a: max_a,
            max_brake_a: max_a,
            overcurrent_threshold_a: max_a * OVERCURRENT_HEADROOM,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
            max_dissipation_w: 0.0,
        }
    }

    /// Build limits from a host-written config, clamped to the board's
    /// hardware ceiling AND the motor's continuous-current rating.
    ///
    /// Layered semantics (the VESC override-matrix idea): the RATING is a
    /// ceiling owned by the motor (detection's thermal solve, stored in
    /// the MotorParams group), the OPERATIONAL config is the session's
    /// choice below it — it can lower limits but never raise them above
    /// either the board hardware or what the motor tolerates. Zero or
    /// negative config values mean "not set" and fall back to the ceiling
    /// itself (VESC applies `l_current_max = i_max` the same way), so a
    /// detected motor gets sane defaults with no extra configuration. The
    /// overcurrent trip ceiling is 1.5× the rating (VESC's
    /// `l_abs_current_max = i_max·1.5`), still capped by the board.
    ///
    /// `hw_max_a` is the board's ABS trip line — the same line the
    /// per-phase OC check in `run_foc_cycle` kills at — NOT an iq
    /// budget: the iq ceiling sits [`OVERCURRENT_HEADROOM`] below it.
    /// (Until 2026-06-12 the iq ceiling WAS the line, so a board-limit
    /// config met the per-phase Kill exactly at full throttle.) The same
    /// headroom is enforced across the config fields: protection wins
    /// over torque, an incoherent pair lowers iq rather than raising the
    /// trip.
    ///
    /// # Arguments
    /// * `cfg` - the host-written limits config
    /// * `hw_max_a` - board hardware phase-current ABS trip (A)
    /// * `rating_a` - motor continuous-current rating (A); `<= 0`/NaN =
    ///   unknown (no rating clamp)
    #[cfg(feature = "storage")]
    pub fn from_config_clamped(cfg: &CurrentLimitsConfig, hw_max_a: f32, rating_a: f32) -> Self {
        let rating_ok = rating_a.is_finite() && rating_a > 0.0;
        let iq_ceiling_board = hw_max_a / OVERCURRENT_HEADROOM;
        let iq_ceiling = if rating_ok {
            iq_ceiling_board.min(rating_a)
        } else {
            iq_ceiling_board
        };
        // Bus limits have no board ceiling (they protect the supply, not
        // the board); NaN → unlimited (the boundary sanity check rejects
        // NaN commands, this is boot-path defense).
        let bus = |v: f32| if v.is_finite() { v } else { -1.0 };
        let trip = if cfg.max_phase_current_a > 0.0 {
            cfg.max_phase_current_a.min(hw_max_a)
        } else if rating_ok {
            hw_max_a.min(1.5 * rating_a)
        } else {
            hw_max_a
        };
        let iq = if cfg.max_iq_a > 0.0 {
            cfg.max_iq_a.min(iq_ceiling)
        } else {
            iq_ceiling
        };
        let max_current_a = iq.min(trip / OVERCURRENT_HEADROOM);
        Self {
            max_current_a,
            max_brake_a: if cfg.max_brake_a > 0.0 {
                cfg.max_brake_a.min(max_current_a)
            } else {
                max_current_a
            },
            overcurrent_threshold_a: trip,
            bus_in_max_a: bus(cfg.bus_in_max_a),
            bus_regen_max_a: bus(cfg.bus_regen_max_a),
            // `> 0` is false for NaN too: off. No ceiling needed here: the
            // current circle bounds what the budget can buy.
            max_dissipation_w: if cfg.max_dissipation_w > 0.0 {
                cfg.max_dissipation_w
            } else {
                0.0
            },
        }
    }

    /// Limits for boot: the stored config (clamped) when present, board
    /// defaults otherwise. `rating_a` as in
    /// [`from_config_clamped`](Self::from_config_clamped) — pass the
    /// stored motor rating (or `0.0` when absent) so a detected motor's
    /// thermal ceiling holds even with no limits group written.
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&CurrentLimitsConfig>, hw_max_a: f32, rating_a: f32) -> Self {
        match cfg {
            Some(c) => Self::from_config_clamped(c, hw_max_a, rating_a),
            // No limits group stored: the ceilings ARE the limits (an
            // all-unset config, NOT CurrentLimitsConfig::default() — that
            // one carries concrete 10 A/40 A values). Also without a motor
            // rating: `from_max_current(hw_max_a)` here put the iq budget
            // ON the board's kill line (and the dq trip 1.3× above it) on
            // every fresh board — the exact state the headroom fix above
            // was meant to end.
            None => Self::from_config_clamped(&CurrentLimitsConfig::AUTO, hw_max_a, rating_a),
        }
    }

    /// Check if current limiting is enabled
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.max_current_a > 0.0
    }

    /// Clamp id/iq targets to the current limit circle.
    ///
    /// Uses d-axis priority: id is clamped first to the full budget,
    /// then iq gets the remaining circular margin. This is correct for
    /// IPM motors where id is used for field weakening.
    #[inline]
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn clamp_targets(&self, id_target: f32, iq_target: f32) -> (f32, f32) {
        if !(self.max_current_a > 0.0) {
            return (id_target, iq_target);
        }
        let limit = self.max_current_a;
        // D-axis priority: clamp id first
        let id = clamp_f32(id_target, -limit, limit);
        // Q-axis gets the remaining circular budget
        let iq_budget_sq = limit * limit - id * id;
        let iq_budget = if iq_budget_sq > 0.0 {
            // .max(0.0) lets the compiler prove -iq_budget <= iq_budget,
            // eliminating clamp's panic branch (sqrtf can't return NaN here,
            // but LLVM can't prove it).
            sqrtf(iq_budget_sq).max(0.0)
        } else {
            0.0
        };
        let iq = clamp_f32(iq_target, -iq_budget, iq_budget);
        (id, iq)
    }

    /// Check if measured current exceeds the overcurrent threshold.
    /// Returns true if overcurrent is detected.
    #[inline]
    pub fn is_overcurrent(&self, id: f32, iq: f32) -> bool {
        // Invalid current feedback must not bypass the trip through NaN
        // comparisons, including when a numeric threshold is disabled.
        DqCurrentProtection {
            threshold_a: self.overcurrent_threshold_a,
        }
        .exceeded(id, iq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_replacement_preserves_motor_and_trip_limits() {
        let original = CurrentLimits {
            max_current_a: 20.0,
            max_brake_a: 12.0,
            overcurrent_threshold_a: 30.0,
            bus_in_max_a: -1.0,
            bus_regen_max_a: -1.0,
            max_dissipation_w: 40.0,
        };
        let limits = original.with_power_source(PowerSourceLimits::new(5.0, 0.0).unwrap());
        assert_eq!(limits.max_current_a, original.max_current_a);
        assert_eq!(limits.max_brake_a, original.max_brake_a);
        assert_eq!(
            limits.overcurrent_threshold_a,
            original.overcurrent_threshold_a
        );
        assert_eq!(limits.max_dissipation_w, original.max_dissipation_w);
        assert_eq!(limits.bus_in_max_a, 5.0);
        assert_eq!(limits.bus_regen_max_a, 0.0);
    }

    #[test]
    fn source_requires_explicit_finite_capability() {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -1.0, 0.0] {
            assert_eq!(
                PowerSourceLimits::new(invalid, 0.0),
                Err(InvalidSourceLimits::Input)
            );
        }
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -1.0] {
            assert_eq!(
                PowerSourceLimits::new(5.0, invalid),
                Err(InvalidSourceLimits::Regeneration)
            );
        }
        let source = PowerSourceLimits::new(5.0, 0.0).unwrap();
        assert_eq!(source.input_a(), 5.0);
        assert_eq!(source.regen_a(), 0.0);
    }
}

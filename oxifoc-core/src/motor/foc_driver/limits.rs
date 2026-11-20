//! Current-limit configuration and pure current constraints.

use crate::foc::fast_math::sqrtf;
#[cfg(feature = "storage")]
use crate::storage::CurrentLimitsConfig;
#[cfg(feature = "storage")]
use crate::units::watts;
use crate::units::{ConstZero, ElectricCurrent, Power, Scalar, Sign, amps};

/// "Unlimited" for the bus-current limits, where zero is a real value.
const UNLIMITED: ElectricCurrent = amps(-1.0);

/// Explicit source capability for an energized commissioning session.
/// Unlike legacy configuration, this value has no unlimited sentinel.
/// Construction is required after decoding a request; do not deserialize it
/// directly. These limits concern DC current, not phase-current amplitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowerSourceLimits {
    input: ElectricCurrent,
    regen: ElectricCurrent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidSourceLimits {
    Input,
    Regeneration,
}

impl PowerSourceLimits {
    pub fn new(
        input: ElectricCurrent,
        regen: ElectricCurrent,
    ) -> Result<Self, InvalidSourceLimits> {
        if !input.is_finite() || input.is_non_positive() {
            return Err(InvalidSourceLimits::Input);
        }
        if !regen.is_finite() || regen.is_negative() {
            return Err(InvalidSourceLimits::Regeneration);
        }
        Ok(Self { input, regen })
    }

    pub fn input(self) -> ElectricCurrent {
        self.input
    }
    pub fn regen(self) -> ElectricCurrent {
        self.regen
    }
}

/// Independent measured dq-magnitude trip. The board's per-phase trip is
/// separate and remains enforced by the platform control-cycle boundary.
#[derive(Clone, Copy, Debug)]
pub(super) struct DqCurrentProtection {
    threshold: ElectricCurrent,
}

impl DqCurrentProtection {
    fn exceeded(self, id: ElectricCurrent, iq: ElectricCurrent) -> bool {
        if !id.is_finite() || !iq.is_finite() {
            return true;
        }
        self.threshold.is_positive() && id * id + iq * iq > self.threshold * self.threshold
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
///    separate: `bus_regen_max = 0` forbids pushing energy into the supply
///    at all (a lab PSU cannot absorb reverse current).
/// 3. **Measured overcurrent**: checks actual dq current magnitude after the
///    FOC step. If it exceeds `overcurrent_threshold`, PWM is disabled and
///    an error is returned. This is the software equivalent of hardware
///    overcurrent protection for boards that lack it.
#[derive(Clone, Copy, Debug)]
pub struct CurrentLimits {
    /// Maximum current target magnitude. The PI controller will never
    /// be asked to produce more than this. Set from BoardConfig or
    /// CurrentLimitsConfig. 0 = no limit.
    pub max_current: ElectricCurrent,
    /// Maximum braking q current: the limit whenever iq opposes the
    /// rotation. At most `max_current`.
    pub max_brake: ElectricCurrent,
    /// Hard overcurrent threshold on measured current. If actual
    /// sqrt(id² + iq²) exceeds this, PWM is immediately disabled.
    /// Typically set to 1.2-1.5× max_current. 0 = no limit.
    pub overcurrent_threshold: ElectricCurrent,
    /// Maximum supply (bus) current draw. `< 0` = unlimited.
    /// Note the different "off" sentinel from the phase limits: here 0 is a
    /// meaningful value (no draw allowed), so "unlimited" must be negative.
    pub bus_in_max: ElectricCurrent,
    /// Maximum regen (charge) current into the supply (positive
    /// magnitude). `< 0` = unlimited; **0 = no regen** (lab-PSU safe:
    /// current braking degrades to what winding losses absorb, the
    /// windings-short parking brake is unaffected — it never touches the
    /// bus).
    pub bus_regen_max: ElectricCurrent,
    /// Heat budget (`1.5·R·id²`) for burning refused regen in the
    /// windings (`motor::dissipation`); the d current it buys stays inside
    /// the `max_current` circle. `0` = off.
    pub max_dissipation: Power,
}

impl Default for CurrentLimits {
    fn default() -> Self {
        // Conservative bench-safe fallback, NOT "protection off": a platform
        // that forgets set_current_limits() gets a 5 A motor, not an
        // unprotected one. Explicit zeros still mean "no limit" where a
        // test or sim genuinely wants that.
        Self::from_max_current(amps(5.0))
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
        self.bus_in_max = source.input();
        self.bus_regen_max = source.regen();
        self
    }

    /// Create current limits from a maximum current value. Sets the
    /// overcurrent threshold [`OVERCURRENT_HEADROOM`] above the max;
    /// bus limits and dissipation off.
    pub fn from_max_current(max: ElectricCurrent) -> Self {
        Self {
            max_current: max,
            max_brake: max,
            overcurrent_threshold: max * OVERCURRENT_HEADROOM,
            bus_in_max: UNLIMITED,
            bus_regen_max: UNLIMITED,
            max_dissipation: Power::ZERO,
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
    /// `hw_max` is the board's ABS trip line — the same line the
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
    /// * `hw_max` - board hardware phase-current ABS trip
    /// * `rating` - motor continuous-current rating, when known (a
    ///   non-finite or non-positive value counts as unknown)
    #[cfg(feature = "storage")]
    pub fn from_config_clamped(
        cfg: &CurrentLimitsConfig,
        hw_max: ElectricCurrent,
        rating: Option<ElectricCurrent>,
    ) -> Self {
        let rating = rating.filter(|r| r.is_finite() && r.is_positive());
        let iq_ceiling_board = hw_max / OVERCURRENT_HEADROOM;
        let iq_ceiling = rating.map_or(iq_ceiling_board, |r| iq_ceiling_board.min(r));
        // A set limit counts when positive; NaN and non-positive values
        // fall back to the inherited ceiling (the boundary check rejects
        // them, this is boot-path defense).
        let set = |v: Option<ElectricCurrent>| v.filter(Sign::is_positive);
        // Bus limits have no board ceiling (they protect the supply, not
        // the board); unset, negative or NaN → unlimited, zero is a real
        // limit.
        let bus = |v: Option<ElectricCurrent>| {
            v.filter(|a| a.is_finite() && a.is_non_negative())
                .unwrap_or(UNLIMITED)
        };
        let trip = match (set(cfg.max_phase_current), rating) {
            (Some(trip), _) => trip.min(hw_max),
            (None, Some(rating)) => hw_max.min(1.5 * rating),
            (None, None) => hw_max,
        };
        let iq = set(cfg.max_iq).map_or(iq_ceiling, |iq| iq.min(iq_ceiling));
        let max_current = iq.min(trip / OVERCURRENT_HEADROOM);
        Self {
            max_current,
            max_brake: set(cfg.max_brake).map_or(max_current, |brake| brake.min(max_current)),
            overcurrent_threshold: trip,
            bus_in_max: bus(cfg.bus_in_max),
            bus_regen_max: bus(cfg.bus_regen_max),
            // `> 0` is false for NaN too: off. No ceiling needed here: the
            // current circle bounds what the budget can buy.
            max_dissipation: if cfg.max_dissipation.is_positive() {
                cfg.max_dissipation
            } else {
                watts(0.0)
            },
        }
    }

    /// Limits for boot: the stored config (clamped) when present, board
    /// defaults otherwise. `rating` as in
    /// [`from_config_clamped`](Self::from_config_clamped) — pass the
    /// stored motor rating so a detected motor's thermal ceiling holds
    /// even with no limits group written.
    #[cfg(feature = "storage")]
    pub fn from_stored(
        cfg: Option<&CurrentLimitsConfig>,
        hw_max: ElectricCurrent,
        rating: Option<ElectricCurrent>,
    ) -> Self {
        match cfg {
            Some(c) => Self::from_config_clamped(c, hw_max, rating),
            // No limits group stored: the ceilings ARE the limits (an
            // all-unset config, NOT CurrentLimitsConfig::default() — that
            // one carries concrete 10 A/40 A values). Also without a motor
            // rating: `from_max_current(hw_max)` here put the iq budget
            // ON the board's kill line (and the dq trip 1.3× above it) on
            // every fresh board — the exact state the headroom fix above
            // was meant to end.
            None => Self::from_config_clamped(&CurrentLimitsConfig::AUTO, hw_max, rating),
        }
    }

    /// Check if current limiting is enabled
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.max_current.is_positive()
    }

    /// Clamp id/iq targets to the current limit circle.
    ///
    /// Uses d-axis priority: id is clamped first to the full budget,
    /// then iq gets the remaining circular margin. This is correct for
    /// IPM motors where id is used for field weakening.
    #[inline]
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn clamp_targets(
        &self,
        id_target: ElectricCurrent,
        iq_target: ElectricCurrent,
    ) -> (ElectricCurrent, ElectricCurrent) {
        if !self.max_current.is_positive() {
            return (id_target, iq_target);
        }
        let limit = self.max_current;
        // D-axis priority: clamp id first
        let id = id_target.clamp_to(-limit, limit);
        // Q-axis gets the remaining circular budget
        let iq_budget_sq = limit * limit - id * id;
        // Strict comparison keeps the square-root path at the rounding
        // boundary, including zero/subnormal budgets and underflowed squares.
        if iq_target * iq_target < iq_budget_sq {
            return (id, iq_target);
        }
        let iq_budget = if iq_budget_sq.is_positive() {
            // .max(0.0) lets the compiler prove -iq_budget <= iq_budget,
            // eliminating clamp's panic branch (sqrtf can't return NaN here,
            // but LLVM can't prove it).
            iq_budget_sq.sqrt_with(sqrtf).max(ElectricCurrent::ZERO)
        } else {
            ElectricCurrent::ZERO
        };
        let iq = iq_target.clamp_to(-iq_budget, iq_budget);
        (id, iq)
    }

    /// Check if measured current exceeds the overcurrent threshold.
    /// Returns true if overcurrent is detected.
    #[inline]
    pub fn is_overcurrent(&self, id: ElectricCurrent, iq: ElectricCurrent) -> bool {
        // Invalid current feedback must not bypass the trip through NaN
        // comparisons, including when a numeric threshold is disabled.
        DqCurrentProtection {
            threshold: self.overcurrent_threshold,
        }
        .exceeded(id, iq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::clamp_f32;
    use crate::units::{ToBits, amps, watts};

    #[test]
    fn current_circle_fast_path_matches_sqrt_at_boundaries_and_extreme_inputs() {
        let check = |limit: f32, id: f32, iq: f32| {
            let limits = CurrentLimits::from_max_current(amps(limit));
            let actual = limits.clamp_targets(amps(id), amps(iq));
            let expected = if limit > 0.0 {
                let id = clamp_f32(id, -limit, limit);
                let squared = limit * limit - id * id;
                let room = if squared > 0.0 {
                    sqrtf(squared).max(0.0)
                } else {
                    0.0
                };
                (id, clamp_f32(iq, -room, room))
            } else {
                (id, iq)
            };
            assert_eq!(
                actual.0.to_bits(),
                expected.0.to_bits(),
                "limit={limit}, id={id}, iq={iq}"
            );
            assert_eq!(
                actual.1.to_bits(),
                expected.1.to_bits(),
                "limit={limit}, id={id}, iq={iq}"
            );
        };
        for limit in [
            0.0,
            f32::from_bits(1),
            1e-20,
            1e-10,
            1.0,
            10.0,
            100.0,
            1e20,
            f32::MAX,
            f32::INFINITY,
        ] {
            for id in [0.0, -0.0, limit * 0.5, limit, -limit, f32::NAN] {
                let room = sqrtf((limit * limit - id * id).max(0.0));
                for iq in [
                    0.0,
                    -0.0,
                    f32::from_bits(1),
                    room.next_down(),
                    room,
                    room.next_up(),
                    -room.next_up(),
                    f32::INFINITY,
                    f32::NAN,
                ] {
                    check(limit, id, iq);
                }
            }
        }
        let mut bits = 0x1234_5678_u32;
        for _ in 0..100_000 {
            let mut next = || {
                bits = bits.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                f32::from_bits(bits)
            };
            check(next().abs(), next(), next());
        }
    }

    #[test]
    fn source_replacement_preserves_motor_and_trip_limits() {
        let original = CurrentLimits {
            max_current: amps(20.0),
            max_brake: amps(12.0),
            overcurrent_threshold: amps(30.0),
            bus_in_max: amps(-1.0),
            bus_regen_max: amps(-1.0),
            max_dissipation: watts(40.0),
        };
        let limits =
            original.with_power_source(PowerSourceLimits::new(amps(5.0), amps(0.0)).unwrap());
        assert_eq!(limits.max_current, original.max_current);
        assert_eq!(limits.max_brake, original.max_brake);
        assert_eq!(limits.overcurrent_threshold, original.overcurrent_threshold);
        assert_eq!(limits.max_dissipation, original.max_dissipation);
        assert_eq!(limits.bus_in_max, amps(5.0));
        assert_eq!(limits.bus_regen_max, amps(0.0));
    }

    #[test]
    fn source_requires_explicit_finite_capability() {
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -1.0, 0.0] {
            assert_eq!(
                PowerSourceLimits::new(amps(invalid), amps(0.0)),
                Err(InvalidSourceLimits::Input)
            );
        }
        for invalid in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, -1.0] {
            assert_eq!(
                PowerSourceLimits::new(amps(5.0), amps(invalid)),
                Err(InvalidSourceLimits::Regeneration)
            );
        }
        let source = PowerSourceLimits::new(amps(5.0), amps(0.0)).unwrap();
        assert_eq!(source.input(), amps(5.0));
        assert_eq!(source.regen(), amps(0.0));
    }
}

//! Six-step (trapezoidal) commutation for BLDC motor control
//!
//! Provides the commutation table and helpers for voltage-mode six-step drive.
//! This is the simplest motor drive mode, useful for board bringup, hall sensor
//! validation, and phase wiring verification without requiring current sensor
//! calibration.
//!
//! ## Commutation Sequence
//!
//! Standard six-step with one phase high (PWM), one low (sink), one floating:
//!
//! | Sector | Angle Range | Phase A | Phase B | Phase C |
//! |--------|-------------|---------|---------|---------|
//! | 0      | 0° - 60°    | High    | Low     | Float   |
//! | 1      | 60° - 120°  | High    | Float   | Low     |
//! | 2      | 120° - 180° | Float   | High    | Low     |
//! | 3      | 180° - 240° | Low     | High    | Float   |
//! | 4      | 240° - 300° | Low     | Float   | High    |
//! | 5      | 300° - 360° | Float   | Low     | High    |
//!
//! For reverse direction, High and Low roles are swapped (current flows opposite).

use core::f32::consts::TAU;

use crate::foc::pwm::PhaseState;

/// Role of a phase in a given commutation sector
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PhaseRole {
    /// PWM-modulated high-side drive
    High,
    /// Low-side sink (current return path)
    Low,
    /// Floating / high-impedance (both FETs OFF)
    Float,
}

/// Commutation table: maps sector (0-5) to phase roles [A, B, C].
///
/// For reverse direction, swap High ↔ Low (handled by [`commutate`]).
pub const COMMUTATION_TABLE: [[PhaseRole; 3]; 6] = [
    [PhaseRole::High, PhaseRole::Low, PhaseRole::Float], // Sector 0: 0°-60°
    [PhaseRole::High, PhaseRole::Float, PhaseRole::Low], // Sector 1: 60°-120°
    [PhaseRole::Float, PhaseRole::High, PhaseRole::Low], // Sector 2: 120°-180°
    [PhaseRole::Low, PhaseRole::High, PhaseRole::Float], // Sector 3: 180°-240°
    [PhaseRole::Low, PhaseRole::Float, PhaseRole::High], // Sector 4: 240°-300°
    [PhaseRole::Float, PhaseRole::Low, PhaseRole::High], // Sector 5: 300°-360°
];

/// Convert an electrical angle (radians, 0 to 2π) to a commutation sector (0-5).
///
/// Sector boundaries are at multiples of 60 electrical degrees (TAU/6).
#[inline]
pub fn angle_to_sector(angle_rad: f32) -> u8 {
    let mut a = angle_rad % TAU;
    if a < 0.0 {
        a += TAU;
    }
    let sector = (a / (TAU / 6.0)) as u8;
    // Clamp to 0-5 in case of floating-point edge at exactly TAU
    if sector > 5 { 5 } else { sector }
}

/// Generate phase states for a given commutation sector and duty.
///
/// # Arguments
/// * `sector` - Commutation sector (0-5), from [`angle_to_sector`]
/// * `duty` - PWM duty count for the active (High) phase
/// * `forward` - `true` for forward rotation, `false` for reverse
///
/// For reverse, High and Low roles are swapped (current flows opposite direction).
#[inline]
pub fn commutate(sector: u8, duty: u16, forward: bool) -> [PhaseState; 3] {
    let roles = &COMMUTATION_TABLE[sector.min(5) as usize];
    roles.map(|role| match (role, forward) {
        (PhaseRole::High, true) | (PhaseRole::Low, false) => PhaseState::Pwm(duty),
        (PhaseRole::Low, true) | (PhaseRole::High, false) => PhaseState::Low,
        (PhaseRole::Float, _) => PhaseState::Float,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_to_sector() {
        assert_eq!(angle_to_sector(0.0), 0);
        assert_eq!(angle_to_sector(0.5), 0);
        assert_eq!(angle_to_sector(TAU / 6.0 + 0.01), 1);
        assert_eq!(angle_to_sector(TAU / 3.0 + 0.01), 2);
        assert_eq!(angle_to_sector(TAU / 2.0 + 0.01), 3);
        assert_eq!(angle_to_sector(2.0 * TAU / 3.0 + 0.01), 4);
        assert_eq!(angle_to_sector(5.0 * TAU / 6.0 + 0.01), 5);
    }

    #[test]
    fn test_angle_to_sector_wraps_negative() {
        assert_eq!(angle_to_sector(-0.1), 5);
    }

    #[test]
    fn test_angle_to_sector_boundary() {
        let sector = angle_to_sector(TAU);
        assert!(sector <= 5);
    }

    #[test]
    fn test_commutate_forward_sector0() {
        let states = commutate(0, 500, true);
        assert_eq!(states[0], PhaseState::Pwm(500)); // A = High
        assert_eq!(states[1], PhaseState::Low); // B = Low
        assert_eq!(states[2], PhaseState::Float); // C = Float
    }

    #[test]
    fn test_commutate_reverse_swaps_high_low() {
        let fwd = commutate(0, 500, true);
        let rev = commutate(0, 500, false);
        // Forward: A=High(Pwm), B=Low, C=Float
        // Reverse: A=Low, B=High(Pwm), C=Float
        assert_eq!(fwd[0], PhaseState::Pwm(500));
        assert_eq!(fwd[1], PhaseState::Low);
        assert_eq!(rev[0], PhaseState::Low);
        assert_eq!(rev[1], PhaseState::Pwm(500));
        // Float stays Float
        assert_eq!(fwd[2], PhaseState::Float);
        assert_eq!(rev[2], PhaseState::Float);
    }

    #[test]
    fn test_each_sector_has_one_high_one_low_one_float() {
        for sector in 0..6 {
            let states = commutate(sector, 1000, true);
            let has_pwm = states.iter().any(|s| matches!(s, PhaseState::Pwm(_)));
            let has_low = states.iter().any(|s| *s == PhaseState::Low);
            let has_float = states.iter().any(|s| *s == PhaseState::Float);
            assert!(has_pwm, "sector {sector} missing Pwm");
            assert!(has_low, "sector {sector} missing Low");
            assert!(has_float, "sector {sector} missing Float");
        }
    }

    #[test]
    fn test_each_sector_reverse_has_one_high_one_low_one_float() {
        for sector in 0..6 {
            let states = commutate(sector, 1000, false);
            let has_pwm = states.iter().any(|s| matches!(s, PhaseState::Pwm(_)));
            let has_low = states.iter().any(|s| *s == PhaseState::Low);
            let has_float = states.iter().any(|s| *s == PhaseState::Float);
            assert!(has_pwm, "sector {sector} rev missing Pwm");
            assert!(has_low, "sector {sector} rev missing Low");
            assert!(has_float, "sector {sector} rev missing Float");
        }
    }

    #[test]
    fn test_sector_out_of_range_clamped() {
        // Sector > 5 should be clamped to 5
        let states = commutate(10, 500, true);
        let expected = commutate(5, 500, true);
        assert_eq!(states, expected);
    }
}

//! Dissipative braking: burn in the windings the regen the supply refuses.
//!
//! A braking q current turns the rotor's kinetic energy into electrical
//! power; what the copper does not burn flows back into the supply. The
//! supply can refuse it — the regen-overvoltage derate on a full pack, the
//! battery's charge-current limit, a lab PSU (`bus_regen_max_a = 0`).
//! Cutting the brake torque there (the plain derate) fails exactly on a
//! long descent. A d current makes no torque on a surface-magnet motor but
//! heats the windings like any other, so it can take the surplus instead:
//!
//! ```text
//! P_supply = 1.5·(|ω|·λ·|iq| − R·(iq² + id²))      (braking, W, >0 charges)
//! ```
//!
//! The torque stays what the rider asked for as long as the current circle
//! (`i_max`) and the dissipation budget (`id_max` — the driver's heat
//! budget `max_dissipation_w` buys `√(W / 1.5·R)` of it) can carry the
//! surplus; beyond that |iq| shrinks to the largest value they can still
//! balance — never below what the plain derate would grant. The burn
//! capacity is small on low-resistance motors: `1.5·R·I²` ≈ 50 W at 30 A,
//! 190 W at 60 A on a 5065 (R 36 mΩ).
//!
//! id is negative (flux-weakening direction): it lowers the q voltage the
//! brake needs instead of raising it.
//!
//! The balance runs on the configured R and λ, not on a measurement. A
//! motor warmer than at detection has a higher R and burns more than
//! modelled, and the inverter's own losses add to it — both on the safe
//! (less charge) side.

use crate::foc::fast_math::sqrtf;

/// The winding parameters the power balance runs on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindingModel {
    /// Phase resistance (Ω, Y-equivalent), `> 0`
    pub r_ohm: f32,
    /// Permanent-magnet flux linkage (Wb), `> 0`
    pub flux_wb: f32,
}

/// What the supply accepts this cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RegenBudget {
    /// Share of the brake's own regen the supply still takes, 0..=1 (the
    /// regen-overvoltage derate).
    pub frac: f32,
    /// Ceiling on the charging power (W): vbus × the bus regen-current
    /// limit, `INFINITY` when unlimited.
    pub max_w: f32,
}

/// Power the brake pushes into the supply (W, negative = draws from it).
fn charge_w(w: f32, r: f32, iq_abs: f32, id_sq: f32) -> f32 {
    1.5 * (w * iq_abs - r * (iq_abs * iq_abs + id_sq))
}

/// Shape a braking current (`iq` opposing `omega`) to the regen budget.
///
/// Returns `Some((id, iq))` when the brake regenerates: `iq` keeps its
/// sign, is at most the request in magnitude and as large as `id_max` /
/// `i_max` allow while the charge stays within `budget`; `id <= 0` burns
/// the surplus. `None` when the brake does not charge the supply at this
/// speed (plugging — it draws), so there is nothing to shape.
pub fn dissipate(
    iq: f32,
    omega: f32,
    model: &WindingModel,
    budget: &RegenBudget,
    id_max: f32,
    i_max: f32,
) -> Option<(f32, f32)> {
    let r = model.r_ohm;
    let w = omega.abs() * model.flux_wb;
    let q = iq.abs();
    let p0 = charge_w(w, r, q, 0.0);
    if p0 <= 0.0 || p0.is_nan() {
        return None;
    }
    let allow = (budget.frac * p0).min(budget.max_w).max(0.0);
    if p0 <= allow {
        return Some((0.0, iq));
    }
    let a = allow / 1.5;
    let id_max_sq = id_max * id_max;
    let i_max_sq = i_max * i_max;
    // id² that brings the charge down to the budget at |iq| = q.
    let need = |q: f32| (w * q - r * q * q - a) / r;
    let cap = |q: f32| id_max_sq.min(i_max_sq - q * q).max(0.0);
    let q = if need(q) <= cap(q) {
        q
    } else {
        // Largest q' < q the caps can still balance, per cap:
        // - id_max: R·q'² − w·q' + (R·id_max² + a) ≥ 0 holds below the
        //   lower root (q sits between the roots, it violates this);
        // - circle (id² = i_max² − q'²): w·q' ≤ R·i_max² + a.
        let c = r * id_max_sq + a;
        let disc = w * w - 4.0 * r * c;
        let q_dissipation = if disc >= 0.0 {
            // Lower root, in the cancellation-free form.
            (2.0 * c / (w + sqrtf(disc))).min(q)
        } else {
            q
        };
        let q_circle = ((r * i_max_sq + a) / w).min(q);
        q_dissipation.min(q_circle)
    };
    let id_sq = need(q).min(cap(q)).max(0.0);
    let iq = if iq < 0.0 { -q } else { q };
    Some((-sqrtf(id_sq), iq))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Flipsky 5065 (the HallRig plant).
    const M5065: WindingModel = WindingModel {
        r_ohm: 0.036,
        flux_wb: 2.92e-3,
    };

    fn charge(omega: f32, (id, iq): (f32, f32)) -> f32 {
        charge_w(omega.abs() * M5065.flux_wb, M5065.r_ohm, iq.abs(), id * id)
    }

    const NONE: RegenBudget = RegenBudget {
        frac: 0.0,
        max_w: f32::INFINITY,
    };

    #[test]
    fn full_budget_passes_the_brake_through() {
        let open = RegenBudget {
            frac: 1.0,
            max_w: f32::INFINITY,
        };
        assert_eq!(
            dissipate(-15.0, 1500.0, &M5065, &open, 30.0, 30.0),
            Some((0.0, -15.0))
        );
    }

    #[test]
    fn plugging_draws_and_is_left_alone() {
        // Below |ω|·λ = R·|iq| the brake draws from the supply.
        assert_eq!(dissipate(-15.0, 100.0, &M5065, &NONE, 30.0, 30.0), None);
        assert_eq!(dissipate(15.0, 0.0, &M5065, &NONE, 30.0, 30.0), None);
    }

    #[test]
    fn a_refusing_supply_keeps_the_torque_while_the_copper_can_burn_it() {
        // 400 el rad/s, 10 A: 1.5·(1.17·10 − 3.6) = 12.1 W to burn.
        let (id, iq) = dissipate(-10.0, 400.0, &M5065, &NONE, 30.0, 30.0).unwrap();
        assert_eq!(iq, -10.0);
        assert!(
            id < 0.0 && id * id + iq * iq <= 900.0 + 1e-3,
            "({id}, {iq})"
        );
        assert!(
            charge(400.0, (id, iq)).abs() < 0.05,
            "{}",
            charge(400.0, (id, iq))
        );
        // Reverse rotation: same magnitudes, iq keeps its sign, id < 0.
        let (id_r, iq_r) = dissipate(10.0, -400.0, &M5065, &NONE, 30.0, 30.0).unwrap();
        assert_eq!(iq_r, 10.0);
        assert!((id_r - id).abs() < 1e-4);
    }

    #[test]
    fn past_the_circle_the_torque_shrinks_to_what_it_can_balance() {
        // 1500 el rad/s: holding 15 A would need ~40 A of id. On the 30 A
        // circle the balance holds at |iq| = R·I²/(ωλ) ≈ 7.4 A.
        let omega = 1500.0;
        let (id, iq) = dissipate(-15.0, omega, &M5065, &NONE, 30.0, 30.0).unwrap();
        assert!((iq + 7.4).abs() < 0.05, "iq {iq}");
        assert!(
            (id * id + iq * iq - 900.0).abs() < 0.5,
            "on the circle: ({id}, {iq})"
        );
        assert!(charge(omega, (id, iq)).abs() < 0.2);
        // Maximal: a little more torque at the same total current charges.
        let more = (-sqrtf(900.0 - 7.6 * 7.6), -7.6);
        assert!(charge(omega, more) > 0.5);
    }

    #[test]
    fn the_dissipation_cap_binds_before_the_circle() {
        let omega = 1500.0;
        let (id, iq) = dissipate(-15.0, omega, &M5065, &NONE, 20.0, 30.0).unwrap();
        assert!((id + 20.0).abs() < 1e-3, "id {id}");
        assert!(
            charge(omega, (id, iq)).abs() < 0.2,
            "{}",
            charge(omega, (id, iq))
        );
        // Still better than the plain derate, which grants nothing.
        assert!(iq < -3.0, "iq {iq}");
    }

    #[test]
    fn a_partial_budget_charges_its_share() {
        let omega = 400.0;
        let half = RegenBudget {
            frac: 0.5,
            max_w: f32::INFINITY,
        };
        let p0 = charge(omega, (0.0, -10.0));
        let out = dissipate(-10.0, omega, &M5065, &half, 30.0, 30.0).unwrap();
        assert_eq!(out.1, -10.0);
        assert!((charge(omega, out) - 0.5 * p0).abs() < 0.05);
    }

    #[test]
    fn the_bus_regen_ceiling_is_a_power_cap() {
        let omega = 400.0;
        let capped = RegenBudget {
            frac: 1.0,
            max_w: 5.0,
        };
        let out = dissipate(-10.0, omega, &M5065, &capped, 30.0, 30.0).unwrap();
        assert_eq!(out.1, -10.0);
        assert!((charge(omega, out) - 5.0).abs() < 0.05);
    }

    #[test]
    fn no_dissipation_budget_degrades_to_the_plain_cut() {
        // id_max = 0: nothing but the brake's own copper loss burns, far
        // too little at this speed — the torque drops to zero, the plain
        // derate's answer.
        let omega = 1500.0;
        let (id, iq) = dissipate(-15.0, omega, &M5065, &NONE, 0.0, 30.0).unwrap();
        assert_eq!(id, 0.0);
        assert!(charge(omega, (id, iq)) <= 1e-3);
        assert!(iq.abs() < 0.5, "iq {iq}");
    }
}

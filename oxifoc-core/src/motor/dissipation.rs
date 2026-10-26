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
//! P_supply ≤ 1.5·(|ω|·(λ + |Lq−Ld|·|id|)·|iq| − R·(iq² + id²))
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
    /// Absolute d/q inductance difference (H). This gives the exact
    /// reluctance power for Lq >= Ld and negative id, and a conservative
    /// upper bound for inverse saliency or a caller's positive id.
    pub saliency_h: f32,
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
fn charge_w(w: f32, k: f32, r: f32, q: f32, d: f32) -> f32 {
    1.5 * ((w + k * d) * q - r * (q * q + d * d))
}

/// Largest q <= ceiling satisfying R q² - w q + c >= 0.
/// Both intervals matter: a request above the upper root is feasible.
fn quadratic_ceiling(ceiling: f32, w: f32, r: f32, c: f32) -> f32 {
    if (w - r * ceiling) * ceiling <= c {
        return ceiling;
    }
    let disc = (w * w - 4.0 * r * c).max(0.0);
    2.0 * c / (w + sqrtf(disc))
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
    dissipate_with_id(0.0, iq, omega, model, budget, id_max, i_max)
}

/// As [`dissipate`], retaining the caller's existing d-current magnitude.
/// Its current-circle share and reluctance power must be included BEFORE
/// solving; clipping iq afterwards can move it into an infeasible interval.
pub(super) fn dissipate_with_id(
    id: f32,
    iq: f32,
    omega: f32,
    model: &WindingModel,
    budget: &RegenBudget,
    id_max: f32,
    i_max: f32,
) -> Option<(f32, f32)> {
    let r = model.r_ohm;
    let w = omega.abs() * model.flux_wb;
    let k = omega.abs() * model.saliency_h;
    let i_sq = i_max * i_max;
    let d_min = id.abs().min(i_max);
    let q = iq.abs().min(sqrtf((i_sq - d_min * d_min).max(0.0)));
    let p0 = charge_w(w, k, r, q, d_min);
    if p0 <= 0.0 || p0.is_nan() {
        return None;
    }
    let allow = (budget.frac * p0).min(budget.max_w).max(0.0);
    if p0 <= allow {
        return Some((-d_min, iq.signum() * q));
    }
    let a = allow / 1.5;
    let d_max = id_max.max(d_min).min(i_max);
    let boundary = sqrtf((i_sq - d_max * d_max).max(0.0));

    // For fixed q, charging power is concave in d. Its minimum over
    // [d_min, cap(q)] is therefore at one of the ENDPOINTS. Search each
    // endpoint's feasible q intervals, then take the largest candidate.
    let mut granted = quadratic_ceiling(q, w + k * d_min, r, r * d_min * d_min + a);
    granted = granted.max(quadratic_ceiling(
        q.min(boundary),
        w + k * d_max,
        r,
        r * d_max * d_max + a,
    ));
    if q > boundary {
        let circle_charge = |q: f32| (w + k * sqrtf((i_sq - q * q).max(0.0))) * q - r * i_sq;
        let circle = if k == 0.0 {
            // The common surface-PM case stays analytic in the ISR.
            ((r * i_sq + a) / w).min(q)
        } else if circle_charge(q) <= a {
            q
        } else if circle_charge(boundary) <= a {
            // q*sqrt(I²-q²) is concave for q>=0. With a feasible left
            // endpoint and an infeasible right one, there is exactly one
            // lower crossing. Retain the feasible side of the bracket.
            let (mut lo, mut hi) = (boundary, q);
            for _ in 0..16 {
                let mid = 0.5 * (lo + hi);
                if circle_charge(mid) <= a {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            lo
        } else {
            // Both endpoints violate a concave inequality: no solution
            // in this part of the current circle.
            0.0
        };
        if circle >= boundary {
            granted = granted.max(circle);
        }
    }

    let q = granted.min(q);
    let d = if charge_w(w, k, r, q, d_min) <= allow {
        d_min
    } else {
        // Positive root of R d² - k q d - (w q - R q² - a) = 0.
        let b = k * q;
        let need = (w - r * q) * q - a;
        (b + sqrtf((b * b + 4.0 * r * need).max(0.0))) / (2.0 * r)
    };
    Some((-d, iq.signum() * q))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Flipsky 5065 (the HallRig plant).
    const M5065: WindingModel = WindingModel {
        r_ohm: 0.036,
        flux_wb: 2.92e-3,
        saliency_h: 0.0,
    };

    fn charge(omega: f32, (id, iq): (f32, f32)) -> f32 {
        charge_w(
            omega.abs() * M5065.flux_wb,
            0.0,
            M5065.r_ohm,
            iq.abs(),
            id.abs(),
        )
    }

    const NONE: RegenBudget = RegenBudget {
        frac: 0.0,
        max_w: f32::INFINITY,
    };

    #[test]
    fn current_circle_keeps_the_upper_feasible_interval() {
        let omega = 1.09 / M5065.flux_wb;
        let (id, iq) = dissipate(-30.0, omega, &M5065, &NONE, 5.0, 30.0).unwrap();
        assert!(iq < -29.7, "braking collapsed: ({id}, {iq})");
        assert!(id.abs() <= 5.0 && id * id + iq * iq <= 900.01);
        assert!(charge(omega, (id, iq)).abs() < 0.001);
    }

    #[test]
    fn salient_braking_respects_physical_power_and_current_caps() {
        // Check the returned currents with independent steady-state dq
        // voltage equations, including both normal and inverse saliency.
        for (ld, lq) in [(100e-6f32, 300e-6f32), (300e-6, 100e-6)] {
            let model = WindingModel {
                saliency_h: (lq - ld).abs(),
                ..M5065
            };
            for omega in [100.0f32, 373.29, 400.0, 1500.0, -400.0] {
                for request in [1.0f32, 10.0, 30.0] {
                    for d_cap in [0.0f32, 5.0, 30.0] {
                        let q = -omega.signum() * request;
                        let Some((id, iq)) = dissipate(q, omega, &model, &NONE, d_cap, 30.0) else {
                            continue;
                        };
                        let vd = model.r_ohm * id - omega * lq * iq;
                        let vq = model.r_ohm * iq + omega * (ld * id + model.flux_wb);
                        let charge = -1.5 * (vd * id + vq * iq);
                        assert!(charge <= 0.002, "regen {charge}: ({id}, {iq}) at {omega}");
                        assert!(id.abs() <= d_cap + 0.001);
                        assert!(id * id + iq * iq <= 900.02);
                    }
                }
            }
        }
    }

    #[test]
    fn existing_d_current_is_accounted_before_granting_braking() {
        let model = WindingModel {
            saliency_h: 200e-6,
            ..M5065
        };
        for id in [-25.0f32, -10.0, 10.0, 25.0] {
            let (d, q) = dissipate_with_id(id, -30.0, 400.0, &model, &NONE, 5.0, 30.0).unwrap();
            assert!(d.abs() >= id.abs());
            assert!(d * d + q * q <= 900.02);
            assert!(
                charge_w(
                    400.0 * model.flux_wb,
                    400.0 * model.saliency_h,
                    model.r_ohm,
                    q.abs(),
                    d.abs()
                ) <= 0.002
            );
        }
    }

    #[test]
    fn grant_preserves_every_sampled_feasible_braking_current() {
        // Independent exhaustive grid of physical dq operating points:
        // a feasible sampled point must never brake harder than the grant.
        for delta in [0.0f32, 200e-6] {
            let model = WindingModel {
                saliency_h: delta,
                ..M5065
            };
            for omega in [373.29f32, 400.0, 1500.0] {
                for d_cap in [5.0f32, 20.0, 30.0] {
                    for max_w in [0.0f32, 5.0] {
                        let budget = RegenBudget { frac: 1.0, max_w };
                        let (id, iq) =
                            dissipate(-30.0, omega, &model, &budget, d_cap, 30.0).unwrap();
                        for di in 0..=300 {
                            let d = f64::from(di) * 0.1;
                            if d > f64::from(d_cap) {
                                continue;
                            }
                            for qi in 0..=300 {
                                let q = f64::from(qi) * 0.1;
                                if d * d + q * q > 900.0 {
                                    continue;
                                }
                                let power = 1.5
                                    * (f64::from(omega)
                                        * (f64::from(model.flux_wb) + f64::from(delta) * d)
                                        * q
                                        - f64::from(model.r_ohm) * (d * d + q * q));
                                if power <= f64::from(max_w) {
                                    assert!(
                                        q <= f64::from(iq.abs()) + 0.002,
                                        "grant ({id}, {iq}) missed ({d}, {q}), delta={delta}, omega={omega}, cap={d_cap}, budget={max_w}"
                                    );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

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

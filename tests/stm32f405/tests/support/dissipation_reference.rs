//! Frozen scalar solver from c43a9d3 for numerical and cycle comparisons.
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
//! Additional heat uses positive id unless the caller already requests
//! negative id. With a coarse angle estimate (for example Hall sectors),
//! positive id produces a restoring alignment torque; negative id can
//! amplify angle error and rock the rotor. Flux weakening must therefore
//! be an explicit d-axis request, not a side effect of asking for heat.
//! The power bound uses |id| and covers either polarity. It is a steady-state
//! winding model, not proof of zero regeneration during tracking transients.
//!
//! The balance runs on the configured R and λ, not on a measurement. A
//! motor warmer than at detection has a higher R and burns more than
//! modelled, and the inverter's own losses add to it — both on the safe
//! (less charge) side.

use oxifoc_core::foc::fast_math::sqrtf;

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
/// `i_max` allow while the charge stays within `budget`; `id >= 0` burns
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

/// As [`dissipate`], retaining the caller's existing d-current direction and
/// minimum magnitude. A zero request selects the aligning (positive) direction.
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
    let (d, q) = dissipate_magnitude(id.abs(), iq, omega, model, budget, id_max, i_max)?;
    Some((if id < 0.0 { -d } else { d }, q))
}

/// Solve the polarity-independent power bound. Keep the choice of magnetic
/// alignment outside the scalar heat/current-circle calculation.
fn dissipate_magnitude(
    id_abs: f32,
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
    let d_min = id_abs.min(i_max);
    let q_cap = if d_min == 0.0 {
        i_max
    } else {
        sqrtf((i_sq - d_min * d_min).max(0.0))
    };
    let q = iq.abs().min(q_cap);
    let p0 = charge_w(w, k, r, q, d_min);
    if p0 <= 0.0 || p0.is_nan() {
        return None;
    }
    let allow = (budget.frac * p0).min(budget.max_w).max(0.0);
    if p0 <= allow {
        return Some((d_min, iq.signum() * q));
    }
    let a = allow / 1.5;
    let d_max = id_max.max(d_min).min(i_max);

    // Most low-speed braking requests fit both budgets. If the high-d
    // endpoint is feasible, the requested q is already maximal; finding
    // other feasible q intervals cannot improve it. Solve only the d root.
    // The concave charge curve starts above `allow` at d_min, so this
    // crossing lies inside [d_min, d_max]. Keep the full search for requests
    // which actually reach a heat/current-circle boundary.
    if q * q + d_max * d_max <= i_sq && charge_w(w, k, r, q, d_max) <= allow {
        let d = required_d(q, w, k, r, a).max(d_min);
        return Some((d, iq.signum() * q));
    }
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
        required_d(q, w, k, r, a).max(d_min)
    };
    Some((d, iq.signum() * q))
}

/// Positive crossing of R d² - k q d - (w q - R q² - a) = 0.
#[inline]
fn required_d(q: f32, w: f32, k: f32, r: f32, a: f32) -> f32 {
    let b = k * q;
    let need = (w - r * q) * q - a;
    (b + sqrtf((b * b + 4.0 * r * need).max(0.0))) / (2.0 * r)
}

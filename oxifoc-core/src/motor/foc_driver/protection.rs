//! Phase-current limits, the bus clamp, derating, and the shared protection update.

use super::FocDriver;
use crate::foc::clamp_f32;
#[cfg(feature = "runtime")]
use crate::foc::config::BoardConfig;
use crate::foc::fast_math::sqrtf;
#[cfg(feature = "runtime")]
use crate::foc::fault::FaultCategory;
#[cfg(feature = "runtime")]
use crate::foc::fault::FaultRegistry;
#[cfg(feature = "runtime")]
use crate::foc::fault::PlatformFault;
#[cfg(feature = "runtime")]
use crate::foc::fault::VOLTAGE_HYSTERESIS_MV;
use crate::foc::phase::PhaseProvider;
use crate::foc::pwm::PhasePwm;
use crate::foc::sensors::CurrentSensor;
use crate::foc::trig::SinCos;
use crate::motor::derating::DeratingConfig;
use crate::motor::derating::DeratingScales;
use crate::motor::dissipation::RegenBudget;
use crate::motor::dissipation::WindingModel;
use crate::motor::dissipation::dissipate_with_id;
#[cfg(feature = "storage")]
use crate::storage::CurrentLimitsConfig;

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
        let trip_ceiling = if rating_ok {
            hw_max_a.min(1.5 * rating_a)
        } else {
            hw_max_a
        };
        // Bus limits have no board ceiling (they protect the supply, not
        // the board); NaN → unlimited (the boundary sanity check rejects
        // NaN commands, this is boot-path defense).
        let bus = |v: f32| if v.is_finite() { v } else { -1.0 };
        let trip = if cfg.max_phase_current_a > 0.0 {
            cfg.max_phase_current_a.min(trip_ceiling)
        } else {
            trip_ceiling
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
            None => Self::from_config_clamped(
                &CurrentLimitsConfig {
                    max_iq_a: 0.0,
                    max_brake_a: 0.0,
                    max_phase_current_a: 0.0,
                    bus_in_max_a: -1.0,
                    bus_regen_max_a: -1.0,
                    max_dissipation_w: 0.0,
                },
                hw_max_a,
                rating_a,
            ),
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
        if self.overcurrent_threshold_a <= 0.0 {
            return false;
        }
        let mag_sq = id * id + iq * iq;
        let threshold_sq = self.overcurrent_threshold_a * self.overcurrent_threshold_a;
        mag_sq > threshold_sq
    }
}

impl<P, C, Phase, S> FocDriver<P, C, Phase, S>
where
    P: PhasePwm,
    C: CurrentSensor,
    Phase: PhaseProvider,
    S: SinCos,
{
    /// Apply derating configuration (boot / live config write).
    pub fn set_derating(&mut self, cfg: DeratingConfig) {
        self.derating_cfg = cfg;
    }

    /// Current derating configuration.
    pub fn derating_cfg(&self) -> &DeratingConfig {
        &self.derating_cfg
    }

    /// Live derating scales (1.0/1.0 = no derate).
    pub fn derating(&self) -> DeratingScales {
        self.derating
    }

    /// Store freshly computed derating scales (protection update).
    pub fn set_derating_scales(&mut self, scales: DeratingScales) {
        self.derating = scales;
    }

    /// Shared protection update, called by `run_foc_cycle` every cycle
    /// (moved here from per-platform ISR copies — one implementation for
    /// every board):
    ///
    /// - **Voltage faults via excursion integrals** (V·s) instead of
    ///   single samples (VESC `wrong_voltage_integrator`): a regen spike
    ///   or an EMI blip on the vbus sense must not cost torque mid-ride.
    ///   Trip threshold ≈ VESC-equivalent `5e-5 · max_vbus` V·s (≈3 ms at
    ///   1 V over on a 57 V board); in-range the integral decays with
    ///   τ ≈ 5 ms, so oscillation around the limit still accumulates.
    ///   UnderVoltage keeps its hysteresis auto-clear.
    /// - **Temperature faults**: single-sample against the board
    ///   thresholds — the readings are already slow and filtered.
    /// - **Graduated derating** (decimated /256 ≈ 78 Hz at 20 kHz —
    ///   temps and vbus are slow): recomputes the live scales and
    ///   maintains the `Derating` warning with set/clear hysteresis
    ///   (set < 0.8, clear > 0.95) so the rider learns WHY the board
    ///   feels weak.
    ///
    /// Faults are raised through [`PlatformFault::from_category`]; a
    /// platform returning `None` for a category silently skips it.
    #[cfg(feature = "runtime")]
    pub fn run_protection<F: PlatformFault>(
        &mut self,
        registry: &FaultRegistry<F>,
        board: &BoardConfig,
        vbus_v: f32,
        fet_temp_c_x10: Option<i16>,
        motor_temp_c_x10: Option<i16>,
    ) {
        // --- Voltage excursion integrals (every cycle) ---
        // Leaky-bucket trip: integrate the excursion, fire past `trip_vs`,
        // bleed it off in-range with τ≈5 ms. The integral is capped at a few
        // × `trip_vs` so a long excursion can't bank unbounded "credit" and
        // make a later marginal spike trip instantly; the cap sits above the
        // trip point, so first-trip latency is unchanged.
        //
        // A non-finite `vbus_v` leaves both comparisons false → neither fault
        // trips and the integrals just decay. Acceptable here: vbus is a
        // linear ADC divider with no division, so NaN is not physically
        // reachable — mirrors the host-boundary "reject NaN" philosophy (the
        // ISR trusts its own sense path).
        let max_v = board.max_vbus_mv as f32 / 1000.0;
        let min_v = board.min_vbus_mv as f32 / 1000.0;
        let trip_vs = 5e-5 * max_v;
        let integral_ceiling = trip_vs * 4.0;
        let decay = 1.0 - (self.dt / 0.005).min(1.0);
        if vbus_v > max_v {
            self.ov_integral_vs =
                (self.ov_integral_vs + (vbus_v - max_v) * self.dt).min(integral_ceiling);
            if self.ov_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::OverVoltage)
                && let Some(f) = F::from_category(FaultCategory::OverVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("OverVoltage FAULT: vbus = {} V (integrated)", vbus_v);
            }
        } else {
            self.ov_integral_vs *= decay;
        }
        if vbus_v < min_v {
            self.uv_integral_vs =
                (self.uv_integral_vs + (min_v - vbus_v) * self.dt).min(integral_ceiling);
            if self.uv_integral_vs > trip_vs
                && !registry.has_category(FaultCategory::UnderVoltage)
                && let Some(f) = F::from_category(FaultCategory::UnderVoltage)
            {
                registry.set(f);
                #[cfg(feature = "defmt")]
                defmt::error!("UnderVoltage FAULT: vbus = {} V (integrated)", vbus_v);
            }
        } else {
            self.uv_integral_vs *= decay;
            if vbus_v * 1000.0 > (board.min_vbus_mv + VOLTAGE_HYSTERESIS_MV) as f32
                && registry.has_category(FaultCategory::UnderVoltage)
            {
                registry.clear(FaultCategory::UnderVoltage);
            }
        }

        // --- Temperatures (board thresholds; <= 0 = sensor not wired) ---
        let fet_c = fet_temp_c_x10.map(|t| f32::from(t) / 10.0);
        let motor_c = motor_temp_c_x10.map(|t| f32::from(t) / 10.0);
        let fet_over =
            fet_c.is_some_and(|t| board.max_fet_temp_c > 0.0 && t > board.max_fet_temp_c);
        let motor_over =
            motor_c.is_some_and(|t| board.max_motor_temp_c > 0.0 && t > board.max_motor_temp_c);
        if (fet_over || motor_over)
            && !registry.has_category(FaultCategory::OverTemp)
            && let Some(f) = F::from_category(FaultCategory::OverTemp)
        {
            registry.set(f);
            #[cfg(feature = "defmt")]
            defmt::error!(
                "OverTemp FAULT (fet_over={}, motor_over={})",
                fet_over,
                motor_over
            );
        }

        // --- Graduated derating (decimated) ---
        self.protection_tick = self.protection_tick.wrapping_add(1);
        if self.protection_tick.is_multiple_of(256) {
            let omega = self.phase.get().velocity;
            self.derating = self.derating_cfg.compute(fet_c, motor_c, vbus_v, omega);
            let worst = self.derating.worst();
            if worst < 0.8 {
                if let Some(f) = F::from_category(FaultCategory::Derating) {
                    registry.set(f);
                }
            } else if worst > 0.95 && registry.has_category(FaultCategory::Derating) {
                registry.clear(FaultCategory::Derating);
            }
        }
    }

    /// Low-pass the q-axis modulation (`mod_q = 1.5·vq/vbus`, the duty
    /// factor in `i_bus ≈ iq·mod_q`). τ ≈ 2 ms — slow enough to ignore
    /// per-cycle voltage ripple, fast enough to track real speed changes.
    pub(super) fn update_bus_mod_q(&mut self, vq: f32, dt: f32) {
        const TAU_S: f32 = 0.002;
        let mod_q = 1.5 * vq / self.vbus.max(0.5);
        let alpha = (dt / TAU_S).min(1.0);
        self.bus_mod_q_filt += alpha * (mod_q - self.bus_mod_q_filt);
    }

    /// The (id, iq) targets a torque request is granted this cycle. Every
    /// torque path funnels through here (remote, velocity loop, failsafe,
    /// the regulated parking brake), and the velocity loop takes its
    /// anti-windup bounds from it.
    ///
    /// - iq·ω ≥ 0 is motoring (standstill counts: pulling away is
    ///   acceleration), limited by `max_current_a` × the drive derate
    ///   (heat, sag, speed). The speed cut is re-evaluated FRESH each cycle
    ///   on top of the decimated scales — a 78 Hz-stale cut bang-bangs the
    ///   torque around the ceiling on a fast free rotor (see
    ///   [`DeratingConfig::speed_cut`]).
    /// - Opposing is braking: `max_brake_a` × the brake derate (heat only,
    ///   never speed). The regen the supply refuses (the regen-OV derate,
    ///   `bus_regen_max_a`) is burned in the windings when dissipation is
    ///   on ([`dissipation`](crate::motor::dissipation)); with it off, the
    ///   regen derate scales the torque instead.
    /// - The bus-current clamp last — except for a brake the dissipation
    ///   already shaped: its bus estimate ignores the d current.
    pub(super) fn grant_torque(&self, id_target: f32, iq_target: f32, omega: f32) -> (f32, f32) {
        let limits = &self.current_limits;
        if !limits.is_enabled() {
            return (id_target, self.clamp_iq_for_bus(iq_target));
        }
        if iq_target * omega >= 0.0 {
            let lim =
                limits.max_current_a * self.derating.drive.min(self.derating_cfg.speed_cut(omega));
            return (
                id_target,
                self.clamp_iq_for_bus(clamp_f32(iq_target, -lim, lim)),
            );
        }
        let brake = self.derating.brake;
        let Some(model) = self.dissipation_model() else {
            let lim = limits.max_brake_a * brake.min(self.derating.regen);
            return (
                id_target,
                self.clamp_iq_for_bus(clamp_f32(iq_target, -lim, lim)),
            );
        };
        let lim = limits.max_brake_a * brake;
        let (id_target, iq) = limits.clamp_targets(id_target, clamp_f32(iq_target, -lim, lim));
        let budget = RegenBudget {
            frac: self.derating.regen,
            max_w: if limits.bus_regen_max_a >= 0.0 {
                self.vbus * limits.bus_regen_max_a
            } else {
                f32::INFINITY
            },
        };
        // The heat budget buys id_max = √(W / 1.5·R), and follows the
        // thermal brake derate: a hot motor or board burns less (and so
        // brakes less on a full pack).
        let id_max = sqrtf(limits.max_dissipation_w * brake / (1.5 * model.r_ohm));
        match dissipate_with_id(
            id_target,
            iq,
            omega,
            &model,
            &budget,
            id_max,
            limits.max_current_a,
        ) {
            // The solve includes the caller's existing |id|. Preserve its
            // sign unless additional negative d current is required.
            Some((id, iq)) if id * id > id_target * id_target => (id, iq),
            Some((_, iq)) => (id_target, iq),
            // Plugging: the brake draws from the supply, nothing to shape.
            None => (id_target, self.clamp_iq_for_bus(iq)),
        }
    }

    /// The winding model dissipative braking runs on: `Some` when it is on
    /// (`max_dissipation_w > 0`) and the motor params carry R and λ.
    fn dissipation_model(&self) -> Option<WindingModel> {
        let d = self
            .controller
            .decoupling()
            .filter(|_| self.current_limits.max_dissipation_w > 0.0)?;
        (d.resistance_ohm > 0.0 && d.flux_linkage_wb > 0.0).then_some(WindingModel {
            r_ohm: d.resistance_ohm,
            flux_wb: d.flux_linkage_wb,
            saliency_h: (d.lq_h - d.ld_h).abs(),
        })
    }

    /// Clamp the iq target so the implied bus current stays inside
    /// `[-bus_regen_max_a, bus_in_max_a]` (VESC mcpwm_foc.c:3629-3637).
    ///
    /// `i_bus ≈ iq·mod_q` (electrical power over vbus, q-axis only — the
    /// d-axis share is losses/HFI carrier, small and consumption-side).
    /// With `bus_regen_max_a = 0` no energy is ever pushed into the supply:
    /// without dissipation, current braking degrades to ~zero torque in the
    /// regen zone (a lab PSU cannot absorb reverse current). The
    /// windings-short parking brake is unaffected — it never sees the bus.
    pub(super) fn clamp_iq_for_bus(&self, iq_target: f32) -> f32 {
        let in_max = self.current_limits.bus_in_max_a;
        let regen_max = self.current_limits.bus_regen_max_a;
        if in_max < 0.0 && regen_max < 0.0 {
            return iq_target; // both unlimited (the default)
        }
        let mod_q = self.bus_mod_q_filt;
        // Near-zero modulation: bus current is negligible whatever iq is,
        // and dividing by it would explode. Same dead-band as VESC.
        if mod_q.abs() < 1e-3 {
            return iq_target;
        }
        let hi_bus = if in_max >= 0.0 { in_max } else { f32::INFINITY };
        let lo_bus = if regen_max >= 0.0 {
            -regen_max
        } else {
            f32::NEG_INFINITY
        };
        let (lo_iq, hi_iq) = if mod_q > 0.0 {
            (lo_bus / mod_q, hi_bus / mod_q)
        } else {
            (hi_bus / mod_q, lo_bus / mod_q)
        };
        clamp_f32(iq_target, lo_iq, hi_iq)
    }
}

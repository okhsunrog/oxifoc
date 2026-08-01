//! High-level Field-Oriented Control loop
//!
//! This module keeps the full FOC math inside `oxifoc-core` so platform
//! crates only need to provide sensors and PWM drivers implementing the
//! shared traits. The controller is intentionally minimal: it runs one
//! current loop step and returns the computed PWM duties plus telemetry.

use super::{
    detection::pi_tuning::{self, DEFAULT_BANDWIDTH_RAD_S},
    pi_controller::PIController,
    pwm::{Modulator, SvpwmModulator},
    transforms,
    trig::{LibmSinCos, SinCos},
};
use crate::foc::clamp_f32;
use crate::foc::fast_math::sqrtf;

/// Output of a single FOC current-loop step
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct FocOutput {
    /// Raw phase currents (A)
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    /// Electrical angle used for this step (radians)
    pub angle_rad: f32,
    /// Clarke-transformed currents (α, β)
    pub i_alpha: f32,
    pub i_beta: f32,
    /// Park-transformed currents (d, q)
    pub id: f32,
    pub iq: f32,
    /// PI controller outputs in dq frame
    pub vd: f32,
    pub vq: f32,
    /// Commanded stationary-frame voltages
    pub v_alpha: f32,
    pub v_beta: f32,
    /// Duty cycles to apply (already clamped)
    pub duties: [u16; 3],
    /// Electrical velocity of the ACTIVE angle source (rad/s) — hall,
    /// observer, HFI or the startup ramp, whichever commutates right now.
    /// Stamped by `FocDriver::step` (the controller itself only sees the
    /// angle); the virtual sim stamps it from the plant. Feeds the fast
    /// telemetry `rpm` field, which used to read the hall estimator
    /// unconditionally and showed 0 on sensorless boards while spinning.
    pub velocity_rad_s: f32,
}

impl FocOutput {
    /// Create an empty telemetry struct (const for static initialization)
    pub const fn empty() -> Self {
        Self {
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            angle_rad: 0.0,
            i_alpha: 0.0,
            i_beta: 0.0,
            id: 0.0,
            iq: 0.0,
            vd: 0.0,
            vq: 0.0,
            v_alpha: 0.0,
            v_beta: 0.0,
            duties: [0; 3],
            velocity_rad_s: 0.0,
        }
    }
}

/// Motor parameters for dq decoupling and back-EMF feedforward.
///
/// In the rotor frame the axes are cross-coupled through the speed
/// voltages: −ω·Lq·iq disturbs the d axis, +ω·(Ld·id + λ) the q axis.
/// Feeding these terms forward lifts the disturbance off the PI loops,
/// whose pole-placement tuning assumes a decoupled R-L plant — without
/// it the loop bandwidth degrades as speed rises.
#[derive(Clone, Copy, Debug)]
pub struct Decoupling {
    /// d-axis inductance (H)
    pub ld_h: f32,
    /// q-axis inductance (H)
    pub lq_h: f32,
    /// Permanent-magnet flux linkage (Wb) — the back-EMF feedforward term
    pub flux_linkage_wb: f32,
}

impl Decoupling {
    /// Finite, physically meaningful values only — this multiplies the
    /// measured velocity straight into the output voltage.
    pub fn is_valid(&self) -> bool {
        self.ld_h.is_finite()
            && self.lq_h.is_finite()
            && self.flux_linkage_wb.is_finite()
            && self.ld_h > 0.0
            && self.lq_h > 0.0
            && self.flux_linkage_wb >= 0.0
    }
}

/// Field-Oriented Controller (current loop)
///
/// The controller is hardware-agnostic: it consumes measured currents and
/// electrical angle, then returns duty cycles that can be written through a
/// platform `PhasePwm` implementation.
pub struct FocController<M: Modulator = SvpwmModulator, S: SinCos = LibmSinCos> {
    /// d-axis PI controller (flux)
    pub id_pi: PIController,
    /// q-axis PI controller (torque)
    pub iq_pi: PIController,
    /// DC bus voltage (Volts)
    vbus: f32,
    /// Modulation limit as a fraction of `vbus` (0.0–1.0)
    modulation_limit: f32,
    /// Dead time compensation factor = dead_time_s × pwm_freq_hz (0.0 = disabled)
    dead_time_comp: f32,
    /// Actuation-frame advance (electrical rad, 0.0 = disabled): the output
    /// voltage vector is rotated by this much AFTER the inverse Park.
    /// Compensates the one-PWM-period pipeline delay (the rotor moves
    /// `ωe·T_pwm` between current sampling and voltage application) without
    /// touching the measurement frame — advancing the angle used for the
    /// current Park displaces the regulated current vector off the q axis
    /// by the same amount (`id_true = −iq·sin(δ)`).
    actuation_advance: f32,
    /// dq decoupling + back-EMF feedforward; None = plain PI (motor
    /// parameters unknown, e.g. before detection)
    decoupling: Option<Decoupling>,
    /// Modulator + SinCos phantom (both are ZSTs)
    _phantom: core::marker::PhantomData<(M, S)>,
}

impl<M: Modulator, S: SinCos> FocController<M, S> {
    /// Conservative default modulation limit (keeps us inside linear SVPWM).
    ///
    /// This limit is in *volt* space: |V| ≤ vbus × limit. 1/√3 is the maximum
    /// sinusoidal phase-to-neutral amplitude SVPWM can produce without
    /// overmodulation (VESC uses the same bound: `max_v_mag = ONE_BY_SQRT3 *
    /// v_bus`, mcpwm_foc.c:4660). After the volts→modulation conversion this
    /// corresponds to a modulation magnitude of 1.5 × 1/√3 = √3/2.
    pub const DEFAULT_MODULATION_LIMIT: f32 = 0.577; // ≈ 1/√3

    /// Volts → modulation conversion: m = VOLTS_TO_MOD × v / vbus.
    ///
    /// The VESC-style SVPWM produces a phase-to-neutral voltage of
    /// (2/3)·m·vbus for a modulation input m (derivable from sector 1:
    /// ta−tb = t1, tb−tc = t2 ⟹ va_n = (2t1+t2)/(3·ARR)·vbus = (2/3)·α·vbus),
    /// so converting volts to modulation requires the reciprocal factor 1.5
    /// (VESC: `voltage_normalize = 1.5 / v_bus`, mcpwm_foc.c:4684).
    const VOLTS_TO_MOD: f32 = 1.5;

    /// Minimum bus voltage to avoid divide-by-zero (used in both `new` and `set_vbus`).
    const MIN_VBUS: f32 = 0.5;

    /// Create a new controller with reasonable default gains.
    ///
    /// The default gains (kp=0.4, ki=40) are a conservative starting point
    /// but won't be optimal for most motors. Prefer [`from_motor_params`](Self::from_motor_params)
    /// when resistance and inductance are known.
    ///
    /// PI controllers have no individual output limits — voltage is constrained
    /// by circular clamping in [`step_with_injection`](Self::step_with_injection)
    /// which tracks `vbus × modulation_limit` every cycle.
    pub fn new(vbus: f32) -> Self {
        Self {
            id_pi: PIController::new(0.4, 40.0),
            iq_pi: PIController::new(0.4, 40.0),
            vbus: vbus.max(Self::MIN_VBUS),
            modulation_limit: Self::DEFAULT_MODULATION_LIMIT,
            dead_time_comp: 0.0,
            actuation_advance: 0.0,
            decoupling: None,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Create a controller with PI gains computed from motor parameters.
    ///
    /// Uses the standard pole-placement formula:
    /// - `Kp = L × ω_bw`
    /// - `Ki = R × ω_bw`
    ///
    /// Voltage is constrained by circular clamping in
    /// [`step_with_injection`](Self::step_with_injection) using
    /// `vbus × modulation_limit`, recomputed every cycle.
    ///
    /// # Arguments
    /// * `resistance` - Phase resistance in Ohms
    /// * `inductance` - Phase inductance in Henries (use Ld≈Lq for SPMSM)
    /// * `vbus` - DC bus voltage in Volts
    ///
    /// Uses the default bandwidth of 1000 rad/s (~160 Hz). For custom
    /// bandwidth, use [`from_motor_params_with_bw`](Self::from_motor_params_with_bw).
    pub fn from_motor_params(resistance: f32, inductance: f32, vbus: f32) -> Self {
        Self::from_motor_params_with_bw(resistance, inductance, vbus, DEFAULT_BANDWIDTH_RAD_S)
    }

    /// Create a controller with PI gains computed from motor parameters and custom bandwidth.
    ///
    /// # Arguments
    /// * `resistance` - Phase resistance in Ohms
    /// * `inductance` - Phase inductance in Henries
    /// * `vbus` - DC bus voltage in Volts
    /// * `bandwidth_rad_s` - Desired current loop bandwidth in rad/s
    pub fn from_motor_params_with_bw(
        resistance: f32,
        inductance: f32,
        vbus: f32,
        bandwidth_rad_s: f32,
    ) -> Self {
        let (kp, ki) = pi_tuning::calculate_current_gains(resistance, inductance, bandwidth_rad_s);
        let vbus = vbus.max(Self::MIN_VBUS);

        Self {
            id_pi: PIController::new(kp, ki),
            iq_pi: PIController::new(kp, ki),
            vbus,
            modulation_limit: Self::DEFAULT_MODULATION_LIMIT,
            dead_time_comp: 0.0,
            actuation_advance: 0.0,
            decoupling: None,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Enable (or disable with `None`) dq decoupling + back-EMF
    /// feedforward. Invalid parameters (non-finite, non-positive
    /// inductances) disable it — feedforward multiplies velocity straight
    /// into output voltage, so garbage here is worse than none.
    pub fn set_decoupling(&mut self, decoupling: Option<Decoupling>) {
        self.decoupling = decoupling.filter(Decoupling::is_valid);
    }

    /// Active decoupling parameters, if any.
    pub fn decoupling(&self) -> Option<Decoupling> {
        self.decoupling
    }

    /// Build a controller from the stored runtime config, the way every
    /// board boots. Precedence: motor params arm the decoupling/observer
    /// model, but EXPLICIT stored PI gains override the params-derived
    /// (l_avg·bw) tuning when both are present — the fundamental Ld/Lq the
    /// pulse detection measures are the right inductances for the ω·L·i
    /// decoupling terms, while the current loop's per-cycle di/dt runs on
    /// the (smaller) high-frequency inductance on an eddy-current-heavy
    /// motor, so one L cannot serve both consumers (ZD2808: 86/129 µH
    /// fundamental vs ~24 µH AC plateau).
    #[cfg(feature = "storage")]
    pub fn from_runtime_config(config: &crate::storage::RuntimeConfig, vbus: f32) -> Self {
        if let Some(ref mp) = config.motor_params
            && mp.is_valid()
        {
            let l_avg = (mp.inductance_d_h + mp.inductance_q_h) / 2.0;
            #[cfg(feature = "defmt")]
            defmt::info!(
                "Using stored motor params: R={=f32}, L={=f32}, λ={=f32}, pp={}",
                mp.resistance_ohm,
                l_avg,
                mp.flux_linkage_wb,
                mp.pole_pairs
            );
            let mut foc = Self::from_motor_params(mp.resistance_ohm, l_avg, vbus);
            if let Some(ref pg) = config.pi_gains {
                foc.id_pi.set_gains(pg.kp, pg.ki);
                foc.iq_pi.set_gains(pg.kp, pg.ki);
                #[cfg(feature = "defmt")]
                defmt::info!(
                    "PI gains overridden by stored config: kp={=f32}, ki={=f32}",
                    pg.kp,
                    pg.ki
                );
            }
            // Decoupling wants the FUNDAMENTAL Ld/Lq when the config has
            // them (two-inductance rule: the AC plateau under-compensates
            // the cross-coupling 4–5× on eddy-heavy motors — the bench
            // 800 rad/s OC class); fall back to the AC values otherwise.
            let (ld, lq) = mp
                .fundamental_ld_lq()
                .unwrap_or((mp.inductance_d_h, mp.inductance_q_h));
            foc.set_decoupling(Some(Decoupling {
                ld_h: ld,
                lq_h: lq,
                flux_linkage_wb: mp.flux_linkage_wb,
            }));
            foc
        } else if let Some(ref pg) = config.pi_gains {
            let mut foc = Self::new(vbus);
            foc.id_pi.set_gains(pg.kp, pg.ki);
            foc.iq_pi.set_gains(pg.kp, pg.ki);
            #[cfg(feature = "defmt")]
            defmt::info!("Using stored PI gains: kp={=f32}, ki={=f32}", pg.kp, pg.ki);
            foc
        } else {
            Self::new(vbus)
        }
    }

    /// Override the modulation limit (fraction of `vbus`).
    ///
    /// Values above 1.0 request over-modulation; keep within 0.0–1.0 for
    /// predictable behavior.
    pub fn with_modulation_limit(mut self, limit: f32) -> Self {
        self.modulation_limit = clamp_f32(limit, 0.0, 1.2);
        self
    }

    /// Update the cached DC bus voltage.
    pub fn set_vbus(&mut self, vbus: f32) {
        // Avoid divide-by-zero while tolerating brief brownouts.
        self.vbus = vbus.max(Self::MIN_VBUS);
    }

    /// Configure dead time compensation from PWM parameters.
    ///
    /// Compensates for voltage distortion caused by FET dead time by adjusting
    /// modulation values based on phase current direction (VESC-style).
    pub fn set_dead_time_comp(&mut self, dead_time_ns: u32, pwm_freq_hz: u32) {
        self.dead_time_comp = dead_time_ns as f32 * 1e-9 * pwm_freq_hz as f32;
    }

    /// Configure dead time compensation directly as a modulation factor
    /// (`t_dt × f_pwm`, dimensionless). Prefer [`Self::set_dead_time_comp`]
    /// on hardware; this entry point serves hosts/tests that know the
    /// distortion voltage rather than the timer setting
    /// (`factor = dead_time_v / vbus`).
    pub fn set_dead_time_comp_factor(&mut self, factor: f32) {
        self.dead_time_comp = factor.max(0.0);
    }

    /// Apply dead time compensation to normalized modulation values.
    ///
    /// During dead time the body diode of the leg carrying positive current
    /// conducts to the negative rail, so the phase *loses* `t_dt × f_pwm` of
    /// duty in the current direction (and gains it for negative current). The
    /// command is therefore *increased* along the phase current sign, like
    /// MESC's `CCR += deadtime_comp` for positive current (MESCpwm.c:172-177).
    ///
    /// Note: VESC instead leaves the command uncompensated and subtracts this
    /// same term when estimating the actually-applied voltage for its observer
    /// (update_valpha_vbeta, mcpwm_foc.c). Compensating the command keeps the
    /// applied voltage close to the commanded one, so our telemetry vαβ stays
    /// valid as observer input without a separate estimate.
    fn apply_dead_time_comp(
        mod_alpha: f32,
        mod_beta: f32,
        i_alpha: f32,
        i_beta: f32,
        comp_factor: f32,
    ) -> (f32, f32) {
        if comp_factor == 0.0 {
            return (mod_alpha, mod_beta);
        }

        // Reconstruct phase current signs from αβ
        let (ia, ib, ic) = transforms::inverse_clarke(i_alpha, i_beta);
        let sign_a = if ia >= 0.0 { 1.0f32 } else { -1.0 };
        let sign_b = if ib >= 0.0 { 1.0f32 } else { -1.0 };
        let sign_c = if ic >= 0.0 { 1.0f32 } else { -1.0 };

        // Per-phase ±comp_factor mapped to the αβ frame through the Clarke
        // transform: α gets (2·sa − sb − sc)/3, β gets (sb − sc)/√3.
        let comp_alpha = (1.0 / 3.0) * (2.0 * sign_a - sign_b - sign_c) * comp_factor;
        let comp_beta = super::constants::FRAC_1_SQRT_3 * (sign_b - sign_c) * comp_factor;

        (mod_alpha + comp_alpha, mod_beta + comp_beta)
    }

    /// Hard bound on the actuation advance (electrical rad).
    ///
    /// The small-angle rotation in `apply_actuation_advance` is only a
    /// rotation for small δ — beyond ~2 rad the truncated series becomes an
    /// AMPLIFIER (gain ≈160 at δ=10), applied AFTER the circular voltage
    /// limit, so an estimator velocity spike (e.g. a hall edge bounce)
    /// would rail all three phases at full bus in an arbitrary direction.
    /// 0.5 rad is well above the ~0.3 rad a Flipsky-class motor needs at
    /// full speed and still inside the series' accuracy envelope (δ⁴/24 ≈
    /// 2.6·10⁻³).
    pub const MAX_ACTUATION_ADVANCE_RAD: f32 = 0.5;

    /// Set the actuation-frame advance for the next step (electrical rad).
    ///
    /// The driver recomputes this every cycle as
    /// `velocity × dt × phase_advance_cycles`. Applied as a cheap
    /// small-angle rotation of the output voltage vector — accurate to
    /// `δ⁴/24` (3·10⁻⁴ at the ~0.3 rad of a Flipsky-class motor at full
    /// speed), with no second SinCos evaluation in the ISR. Non-finite
    /// values reset to 0; magnitude saturates at
    /// [`MAX_ACTUATION_ADVANCE_RAD`](Self::MAX_ACTUATION_ADVANCE_RAD).
    pub fn set_actuation_advance(&mut self, advance_rad: f32) {
        self.actuation_advance = if advance_rad.is_finite() {
            clamp_f32(
                advance_rad,
                -Self::MAX_ACTUATION_ADVANCE_RAD,
                Self::MAX_ACTUATION_ADVANCE_RAD,
            )
        } else {
            0.0
        };
    }

    /// Rotate an αβ vector by the configured actuation advance.
    #[inline]
    fn apply_actuation_advance(&self, v_alpha: f32, v_beta: f32) -> (f32, f32) {
        let d = self.actuation_advance;
        if d == 0.0 {
            return (v_alpha, v_beta);
        }
        // Small-angle sin/cos (3rd/2nd order): exact enough for pipeline
        // delays and far cheaper than a second CORDIC/LUT evaluation.
        let sd = d - d * d * d * (1.0 / 6.0);
        let cd = 1.0 - d * d * 0.5;
        (v_alpha * cd - v_beta * sd, v_alpha * sd + v_beta * cd)
    }

    /// Current DC bus voltage cached in the controller.
    pub fn vbus(&self) -> f32 {
        self.vbus
    }

    /// Fractional modulation limit (0.0–1.2).
    pub fn modulation_limit(&self) -> f32 {
        self.modulation_limit
    }

    /// Reset both PI controller integrators.
    pub fn reset(&mut self) {
        self.id_pi.reset();
        self.iq_pi.reset();
    }

    /// Convert dq-frame voltages to PWM duties without PI control.
    ///
    /// Applies circular voltage clamping, inverse Park transform, dead-time
    /// compensation (when configured) and SVPWM. No PI feedback — pure
    /// voltage-to-duty conversion.
    ///
    /// Use this for measurement modes (HFI inductance detection) and direct
    /// voltage control where PI interference is undesirable.
    ///
    /// `i_alpha`/`i_beta` are the latest measured stator currents, used only
    /// for the dead-time compensation phase signs. This matters precisely
    /// here: with no PI to absorb the distortion, an uncompensated
    /// DirectVoltage hold loses `t_dt·f_pwm·vbus` per phase — on a g431
    /// (800 ns) driving a 24 V bus that is 0.38 V, more than the entire
    /// `R·I` holding voltage of a low-resistance outrunner. Passing zeros
    /// (currents unknown) degrades gracefully: equal signs cancel in the
    /// Clarke map and no compensation is applied.
    pub fn apply_dq(
        &self,
        vd: f32,
        vq: f32,
        angle_rad: f32,
        i_alpha: f32,
        i_beta: f32,
        max_duty: u16,
    ) -> FocOutput {
        let (sin_theta, cos_theta) = S::sin_cos(angle_rad);

        // Circular voltage limiting
        let v_limit = self.vbus * self.modulation_limit;
        let v_mag_sq = vd * vd + vq * vq;
        let v_limit_sq = v_limit * v_limit;

        let (vd, vq) = if v_mag_sq > v_limit_sq {
            let scale = v_limit / sqrtf(v_mag_sq);
            (vd * scale, vq * scale)
        } else {
            (vd, vq)
        };

        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
        let volts_to_mod = Self::VOLTS_TO_MOD / self.vbus;
        let (mod_a, mod_b) = Self::apply_dead_time_comp(
            v_alpha * volts_to_mod,
            v_beta * volts_to_mod,
            i_alpha,
            i_beta,
            self.dead_time_comp,
        );
        let duties = M::to_duties(mod_a, mod_b, max_duty);

        FocOutput {
            angle_rad,
            vd,
            vq,
            v_alpha,
            v_beta,
            duties,
            ..Default::default()
        }
    }

    /// Run one FOC current loop step.
    ///
    /// # Arguments
    /// * `currents`   - (ia, ib, ic) phase currents in Amps
    /// * `angle_rad`  - electrical angle in radians
    /// * `id_target`  - d-axis current target (flux), typically 0
    /// * `iq_target`  - q-axis current target (torque)
    /// * `max_duty`   - timer ARR value used for PWM normalization
    /// * `dt`         - loop period in seconds
    ///
    /// # Returns
    /// Telemetry containing intermediate values and final duty cycles.
    pub fn step(
        &mut self,
        currents: (f32, f32, f32),
        angle_rad: f32,
        id_target: f32,
        iq_target: f32,
        max_duty: u16,
        dt: f32,
    ) -> FocOutput {
        // Zero velocity = decoupling feedforward off; fine for the
        // detection/test contexts this entry point serves.
        self.step_with_injection(
            currents, angle_rad, 0.0, id_target, iq_target, 0.0, 0.0, max_duty, dt,
        )
    }

    /// Run one FOC current loop step with voltage injection.
    ///
    /// Used for sensorless HFI angle tracking where PI must remain active.
    /// The injection voltages are added to the PI controller outputs before
    /// inverse Park transform.
    ///
    /// # Arguments
    /// * `currents`   - (ia, ib, ic) phase currents in Amps
    /// * `angle_rad`  - electrical angle in radians
    /// * `vel_rad_s`  - electrical velocity in rad/s (decoupling feedforward;
    ///   pass 0.0 to disable for this step)
    /// * `id_target`  - d-axis current target (flux)
    /// * `iq_target`  - q-axis current target (torque)
    /// * `vd_inject`  - d-axis voltage to inject (V)
    /// * `vq_inject`  - q-axis voltage to inject (V)
    /// * `max_duty`   - timer ARR value used for PWM normalization
    /// * `dt`         - loop period in seconds
    ///
    /// # Returns
    /// Telemetry containing intermediate values and final duty cycles.
    /// The `vd` and `vq` fields include the injected voltages.
    #[allow(clippy::too_many_arguments)]
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub fn step_with_injection(
        &mut self,
        currents: (f32, f32, f32),
        angle_rad: f32,
        vel_rad_s: f32,
        id_target: f32,
        iq_target: f32,
        vd_inject: f32,
        vq_inject: f32,
        max_duty: u16,
        dt: f32,
    ) -> FocOutput {
        let (ia, ib, ic) = currents;
        let prof_t0 = crate::isr_prof::now();
        let (sin_theta, cos_theta) = S::sin_cos(angle_rad);
        crate::isr_prof::add(&crate::isr_prof::CTRL_TRIG, prof_t0, crate::isr_prof::now());

        // Phase currents -> stationary frame
        let (i_alpha, i_beta) = transforms::clarke(ia, ib);
        // Stationary -> rotating frame
        let (id, iq) = transforms::park(i_alpha, i_beta, sin_theta, cos_theta);

        // dq decoupling + back-EMF feedforward (rotor-frame speed voltages):
        //   vd_ff = −ω·Lq·iq*        vq_ff = +ω·(Ld·id* + λ)
        // Added before the circular limit so the limit sees the true total
        // demand; anti-windup below charges each PI only its own share.
        //
        // REFERENCE currents, not measured: the measured-current form is a
        // feedback path with gain ω·L through the one-cycle actuation delay,
        // and once ω·L becomes comparable to kp it turns the loop into a
        // delayed positive-feedback oscillator (sim 2026-07-06: divergence
        // at ω_e·Ts ≈ 0.09 regardless of how accurate the decoupling L was).
        // Reference-based terms are pure feedforward — identical in steady
        // state (i → i*), outside the loop dynamically.
        let (vd_ff, vq_ff) = match self.decoupling {
            Some(d) => (
                -vel_rad_s * d.lq_h * iq_target,
                vel_rad_s * (d.ld_h * id_target + d.flux_linkage_wb),
            ),
            None => (0.0, 0.0),
        };

        // Current controllers (dq frame) + feedforward + optional HFI injection
        let vd_raw = self.id_pi.update(id_target, id, dt) + vd_ff + vd_inject;
        let vq_raw = self.iq_pi.update(iq_target, iq, dt) + vq_ff + vq_inject;

        // Circular voltage limiting: constrain |V| ≤ vbus × modulation_limit
        // Preserves voltage vector direction (unlike independent axis clamping)
        let v_limit = self.vbus * self.modulation_limit;
        let v_mag_sq = vd_raw * vd_raw + vq_raw * vq_raw;
        let v_limit_sq = v_limit * v_limit;

        let (vd, vq) = if v_mag_sq > v_limit_sq {
            let scale = v_limit / sqrtf(v_mag_sq);
            let vd = vd_raw * scale;
            let vq = vq_raw * scale;
            // Coordinated anti-windup, charged to the PI's own share only.
            // The circular limit scales the whole vector uniformly, so the
            // PI's realizable output is pi·scale and the back-calculation
            // term is pi·(scale−1). Using the full `v − v_raw` (which
            // includes feedforward + injection) unwound the integrator for
            // saturation the feedforward caused, costing a recovery
            // transient once the demand dropped back inside the circle.
            // With ff = inject = 0 this reduces to the classic v − v_raw.
            let pi_d = vd_raw - vd_ff - vd_inject;
            let pi_q = vq_raw - vq_ff - vq_inject;
            self.id_pi.apply_anti_windup(pi_d * (scale - 1.0));
            self.iq_pi.apply_anti_windup(pi_q * (scale - 1.0));
            (vd, vq)
        } else {
            (vd_raw, vq_raw)
        };

        // dq -> stationary frame, then advance the actuation frame: the
        // voltage acts ~one PWM period after the currents were sampled and
        // the rotor will have moved (see `set_actuation_advance`).
        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
        let (v_alpha, v_beta) = self.apply_actuation_advance(v_alpha, v_beta);
        let volts_to_mod = Self::VOLTS_TO_MOD / self.vbus;
        let (mod_a, mod_b) = Self::apply_dead_time_comp(
            v_alpha * volts_to_mod,
            v_beta * volts_to_mod,
            i_alpha,
            i_beta,
            self.dead_time_comp,
        );
        let duties = M::to_duties(mod_a, mod_b, max_duty);

        FocOutput {
            ia,
            ib,
            ic,
            angle_rad,
            i_alpha,
            i_beta,
            id,
            iq,
            vd,
            vq,
            v_alpha,
            v_beta,
            duties,
            // The controller only sees the angle; the driver (or the sim)
            // stamps the active source's velocity on top.
            velocity_rad_s: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::foc::trig::FastSinCos;

    const DT: f32 = 0.0001;

    /// Generate FOC controller tests for a given SinCos implementation.
    macro_rules! foc_controller_tests {
        ($mod_name:ident, $sincos:ty) => {
            mod $mod_name {
                use super::*;

                #[test]
                fn zero_currents_zero_setpoint_centered_pwm() {
                    let mut foc = FocController::<SvpwmModulator, $sincos>::new(24.0);
                    let telem = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 1000, DT);

                    for duty in telem.duties {
                        assert!((490..=510).contains(&duty));
                    }
                    assert!((telem.id).abs() < 1e-6);
                    assert!((telem.iq).abs() < 1e-6);
                }

                #[test]
                fn positive_q_axis_command_generates_voltage() {
                    let mut foc = FocController::<SvpwmModulator, $sincos>::new(48.0);
                    let telem = foc.step((0.0, 0.0, 0.0), 1.0, 0.0, 5.0, 1200, DT);

                    assert!(telem.vq > 0.0);
                    assert!(telem.duties.iter().all(|d| *d <= 1200));
                }

                #[test]
                fn modulation_limit_is_respected() {
                    let mut foc = FocController::<SvpwmModulator, $sincos>::new(30.0)
                        .with_modulation_limit(0.25);
                    let telem = foc.step((2.0, -1.0, -1.0), 0.7, 0.0, 20.0, 800, DT);

                    let v_limit = foc.vbus() * foc.modulation_limit();
                    let v_mag =
                        $crate::foc::fast_math::sqrtf(telem.vd * telem.vd + telem.vq * telem.vq);
                    assert!(
                        v_mag <= v_limit + 1e-6,
                        "voltage magnitude {} exceeds limit {}",
                        v_mag,
                        v_limit
                    );
                }

                #[test]
                fn decoupling_feedforward_produces_speed_voltages() {
                    // Zero-gain PI isolates the feedforward path: the output
                    // must be exactly the rotor-frame speed voltages from the
                    // REFERENCE currents, vd = −ω·Lq·iq*, vq = ω·(Ld·id* + λ)
                    // — reference-based on purpose (the measured-current form
                    // is a delayed feedback path with gain ω·L, see the
                    // step_with_injection decoupling comment).
                    let mut foc = FocController::<SvpwmModulator, $sincos>::new(24.0);
                    foc.id_pi.set_gains(0.0, 0.0);
                    foc.iq_pi.set_gains(0.0, 0.0);
                    foc.set_decoupling(Some(Decoupling {
                        ld_h: 100e-6,
                        lq_h: 300e-6,
                        flux_linkage_wb: 0.02,
                    }));

                    let omega = 200.0; // electrical rad/s
                    let (id_target, iq_target) = (0.5, 2.0);
                    let telem = foc.step_with_injection(
                        (1.0, 0.5, -1.5),
                        0.0,
                        omega,
                        id_target,
                        iq_target,
                        0.0,
                        0.0,
                        1000,
                        DT,
                    );

                    let vd_expected = -omega * 300e-6 * iq_target;
                    let vq_expected = omega * (100e-6 * id_target + 0.02);
                    assert!(
                        (telem.vd - vd_expected).abs() < 1e-4,
                        "vd {} != expected {}",
                        telem.vd,
                        vd_expected
                    );
                    assert!(
                        (telem.vq - vq_expected).abs() < 1e-4,
                        "vq {} != expected {}",
                        telem.vq,
                        vq_expected
                    );

                    // Garbage params must disable the feedforward, not feed
                    // NaN·ω into the output voltage.
                    foc.set_decoupling(Some(Decoupling {
                        ld_h: f32::NAN,
                        lq_h: 300e-6,
                        flux_linkage_wb: 0.02,
                    }));
                    assert!(foc.decoupling().is_none());
                }

                #[test]
                fn from_motor_params_computes_correct_gains() {
                    let foc = FocController::<SvpwmModulator, $sincos>::from_motor_params(
                        0.5, 5e-4, 24.0,
                    );

                    let mut foc = foc;
                    let telem = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, 0.001);
                    assert!(
                        telem.vq > 0.4,
                        "vq should reflect computed gains, got {}",
                        telem.vq
                    );
                    assert!(telem.vq < 1.5, "vq should be reasonable, got {}", telem.vq);

                    let v_limit =
                        24.0 * FocController::<SvpwmModulator, $sincos>::DEFAULT_MODULATION_LIMIT;
                    let telem_saturated = foc.step((0.0, 0.0, 0.0), 0.0, 0.0, 100.0, 1000, 0.001);
                    let v_mag = $crate::foc::fast_math::sqrtf(
                        telem_saturated.vd * telem_saturated.vd
                            + telem_saturated.vq * telem_saturated.vq,
                    );
                    assert!(
                        v_mag <= v_limit + 1e-3,
                        "voltage magnitude should be limited to {}, got {}",
                        v_limit,
                        v_mag
                    );
                }

                #[test]
                fn from_motor_params_with_custom_bandwidth() {
                    let foc_default = FocController::<SvpwmModulator, $sincos>::from_motor_params(
                        0.5, 5e-4, 24.0,
                    );
                    let foc_fast =
                        FocController::<SvpwmModulator, $sincos>::from_motor_params_with_bw(
                            0.5, 5e-4, 24.0, 2000.0,
                        );

                    let mut foc_d = foc_default;
                    let mut foc_f = foc_fast;

                    let telem_d = foc_d.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, DT);
                    let telem_f = foc_f.step((0.0, 0.0, 0.0), 0.0, 0.0, 1.0, 1000, DT);

                    assert!(
                        telem_f.vq > telem_d.vq * 1.5,
                        "Higher bandwidth should give stronger response: {} vs {}",
                        telem_f.vq,
                        telem_d.vq
                    );
                }
            }
        };
    }

    foc_controller_tests!(libm, LibmSinCos);
    foc_controller_tests!(fast, FastSinCos);

    /// The small-angle advance rotation diverges beyond ~2 rad (gain ~160 at
    /// delta = 10) AFTER the circular voltage limit -- an estimator velocity
    /// spike must saturate at a value where it is still a rotation.
    #[test]
    fn actuation_advance_clamps_to_a_rotation() {
        type Foc = FocController<SvpwmModulator, FastSinCos>;
        let mut foc = Foc::new(24.0);

        foc.set_actuation_advance(10.0);
        assert_eq!(foc.actuation_advance, Foc::MAX_ACTUATION_ADVANCE_RAD);
        foc.set_actuation_advance(-10.0);
        assert_eq!(foc.actuation_advance, -Foc::MAX_ACTUATION_ADVANCE_RAD);
        foc.set_actuation_advance(f32::NAN);
        assert_eq!(foc.actuation_advance, 0.0);

        // At the clamp the rotation must preserve vector magnitude.
        foc.set_actuation_advance(Foc::MAX_ACTUATION_ADVANCE_RAD);
        let (a, b) = foc.apply_actuation_advance(1.0, 0.5);
        let gain2 = (a * a + b * b) / (1.0 + 0.25);
        assert!((gain2 - 1.0).abs() < 5e-3, "gain^2 = {gain2}");
    }

    /// Average phase-to-neutral αβ voltage that a duty triple applies over one
    /// PWM period, assuming ideal half-bridges (leg voltage = duty/max × vbus).
    fn alpha_beta_from_duties(duties: [u16; 3], max_duty: u16, vbus: f32) -> (f32, f32) {
        let leg: [f32; 3] =
            core::array::from_fn(|i| f32::from(duties[i]) / f32::from(max_duty) * vbus);
        let neutral = (leg[0] + leg[1] + leg[2]) / 3.0;
        transforms::clarke(leg[0] - neutral, leg[1] - neutral)
    }

    #[test]
    fn duty_path_applies_commanded_voltage() {
        // Regression test for the volts→modulation normalization.
        //
        // The VESC SVPWM algorithm produces a phase-to-neutral voltage of
        // (2/3)·m·vbus for a modulation input m, so the conversion from volts
        // must be m = 1.5·v/vbus (VESC mcpwm_foc.c:4684,
        // "voltage_normalize = 1/(2/3*V_bus)"). With a plain 1/vbus the motor
        // receives only 2/3 of the commanded voltage and every quantity derived
        // from commanded volts (R/L/λ detection, observer input) is off by 1.5×.
        let vbus = 24.0;
        let foc = FocController::<SvpwmModulator, LibmSinCos>::new(vbus);
        let max_duty = 1000u16;
        // One duty LSB is vbus/max_duty of leg voltage; allow a few for truncation.
        let tol = 3.0 * vbus / f32::from(max_duty);
        for angle_deg in (0..360).step_by(30) {
            let angle = (angle_deg as f32).to_radians();
            // |V| ≈ 5.1 V, far below the 13.8 V modulation limit — no clamping.
            let out = foc.apply_dq(1.0, 5.0, angle, 0.0, 0.0, max_duty);
            let (va, vb) = alpha_beta_from_duties(out.duties, max_duty, vbus);
            assert!(
                (va - out.v_alpha).abs() < tol,
                "applied v_alpha {} != commanded {} at {}°",
                va,
                out.v_alpha,
                angle_deg
            );
            assert!(
                (vb - out.v_beta).abs() < tol,
                "applied v_beta {} != commanded {} at {}°",
                vb,
                out.v_beta,
                angle_deg
            );
        }
    }

    #[test]
    fn dead_time_comp_pushes_voltage_in_current_direction() {
        // During dead time the body diode of the leg carrying positive current
        // conducts to the negative rail, so the phase *loses* voltage in the
        // current direction. The command must therefore be increased along the
        // current sign — MESC does exactly `CCR += deadtime_comp` for positive
        // phase current (MESCpwm.c:172-177). Subtracting doubles the distortion
        // instead of cancelling it.
        let comp = 0.02; // 1 µs dead time × 20 kHz PWM
        // Current along +α: ia > 0, ib < 0, ic < 0.
        let (ma, _) = FocController::<SvpwmModulator, LibmSinCos>::apply_dead_time_comp(
            0.5, 0.0, 10.0, 0.0, comp,
        );
        assert!(ma > 0.5, "mod_alpha must increase with ia > 0, got {ma}");
        // Current along +β: ib > 0, ic < 0.
        let (_, mb) = FocController::<SvpwmModulator, LibmSinCos>::apply_dead_time_comp(
            0.0, 0.5, 0.0, 10.0, comp,
        );
        assert!(mb > 0.5, "mod_beta must increase with iβ > 0, got {mb}");
    }
}

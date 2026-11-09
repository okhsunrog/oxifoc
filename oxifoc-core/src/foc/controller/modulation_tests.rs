//! Differential oracle retaining the controller math before modulation preparation.
use super::*;
use crate::foc::trig::FastSinCos;

impl<M: Modulator, S: SinCos> FocController<M, S> {
    fn legacy_voltage_limit(&self) -> f32 {
        self.vbus
            * self
                .modulation
                .requested
                .min(self.modulation.duty_fraction * FRAC_1_SQRT_3)
    }

    fn legacy_duties_for(&self, mod_a: f32, mod_b: f32, period: u16) -> [u16; 3] {
        let duties = M::to_duties(mod_a, mod_b, period);
        let cap = (self.modulation.duty_fraction * f32::from(period)) as u16;
        let top = duties[0].max(duties[1]).max(duties[2]);
        if top <= cap {
            return duties;
        }
        let shift = top - cap;
        duties.map(|d| d.saturating_sub(shift))
    }

    #[allow(clippy::too_many_arguments)]
    fn legacy_step_with_injection(
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
        let (i_alpha, i_beta) = self.current_sense_topology.clarke(currents);
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

        // Circular voltage limit (modulation limit ∧ duty cap) with d-axis
        // PRIORITY: the d loop keeps the current vector on its axis
        // (id → id*) and the torque axis gets the voltage that remains.
        // Scaling (vd, vq) uniformly instead handed the vector's direction
        // to whichever demand was larger: at top speed vq (≈ ω·λ) dwarfs
        // the vd the d loop needs (≈ −ω·Lq·iq), the scaling took that vd
        // away and id ran negative uncontrolled — sim, 5065 at 24 V with a
        // 25 A command: −37 A of id, then the overcurrent trip. With d
        // priority the torque current just falls off at base speed.
        let v_limit = self.legacy_voltage_limit();
        let vd = clamp_f32(vd_raw, -v_limit, v_limit);
        let vq_room = sqrtf((v_limit * v_limit - vd * vd).max(0.0));
        let vq = clamp_f32(vq_raw, -vq_room, vq_room);
        // Back-calculation anti-windup per axis, charged the whole clipped
        // amount: the integrator then absorbs a feedforward that overshoots
        // (the old PI-share rule peaked at 15 A vs 6 A on a 2 V back-EMF
        // feedforward error).
        if vd != vd_raw {
            self.id_pi.apply_anti_windup(vd - vd_raw);
        }
        if vq != vq_raw {
            self.iq_pi.apply_anti_windup(vq - vq_raw);
        }
        // Within 2% of the limit counts too: at the limit the q demand
        // hovers across it cycle to cycle, and an outer loop must not
        // regain its authority on every cycle it dips under.
        self.q_clip = if vq_raw > 0.0 && vq_raw >= 0.98 * vq_room {
            1.0
        } else if vq_raw < 0.0 && vq_raw <= -0.98 * vq_room {
            -1.0
        } else {
            0.0
        };

        // dq -> stationary frame, then advance the actuation frame: the
        // voltage acts ~one PWM period after the currents were sampled and
        // the rotor will have moved (see `set_actuation_advance`).
        let (v_alpha, v_beta) = transforms::inverse_park(vd, vq, sin_theta, cos_theta);
        let (v_alpha, v_beta) = self.apply_actuation_advance(v_alpha, v_beta);
        let volts_to_mod = Self::VOLTS_TO_MOD / self.vbus;
        // Dead-time signs from the REFERENCE current vector: in closed loop
        // the current follows it, and unlike the measured current it does
        // not flip sign on ADC noise at low current (sim: with ±0.15 A of
        // sensor noise at 0.3 A the measured-sign compensation made the
        // current error worse than none).
        let (i_ref_alpha, i_ref_beta) =
            transforms::inverse_park(id_target, iq_target, sin_theta, cos_theta);
        let (mod_a, mod_b) = Self::apply_dead_time_comp(
            v_alpha * volts_to_mod,
            v_beta * volts_to_mod,
            i_ref_alpha,
            i_ref_beta,
            self.dead_time_comp,
            self.dead_time_current_inv_a,
        );
        let duties = self.legacy_duties_for(mod_a, mod_b, max_duty);
        self.bus_mod_q = QAxisModulation::from_pwm(duties, max_duty, sin_theta, cos_theta);

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

    #[allow(clippy::too_many_arguments)]
    fn legacy_apply_dq(
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
        let v_limit = self.legacy_voltage_limit();
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
            self.dead_time_current_inv_a,
        );
        let duties = self.legacy_duties_for(mod_a, mod_b, max_duty);
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
}

type Controller = FocController<SvpwmModulator, FastSinCos>;

const _: () = {
    fn assert_send_sync<T: Send + Sync>() {}
    let _ = assert_send_sync::<Controller>;
    let _ = assert_send_sync::<FocController>;
};

fn assert_output(actual: FocOutput, expected: FocOutput) {
    let floats = |out: FocOutput| {
        [
            out.ia,
            out.ib,
            out.ic,
            out.angle_rad,
            out.i_alpha,
            out.i_beta,
            out.id,
            out.iq,
            out.vd,
            out.vq,
            out.v_alpha,
            out.v_beta,
            out.velocity_rad_s,
        ]
        .map(f32::to_bits)
    };
    assert_eq!(floats(actual), floats(expected));
    assert_eq!(actual.duties, expected.duties);
}

#[test]
fn prepared_cap_preserves_rounding_for_every_period_and_invalid_setting() {
    let mut limits = ModulationLimits::new(Controller::DEFAULT_MODULATION_LIMIT);
    for fraction in [
        0.5,
        0.500_001,
        0.949_999_9,
        0.95,
        0.950_000_1,
        1.0,
        0.0,
        -0.0,
        -1.0,
        2.0,
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
    ] {
        limits.set_duty_fraction(fraction);
        let legacy_fraction = if fraction.is_finite() {
            clamp_f32(fraction, 0.5, 1.0)
        } else {
            1.0
        };
        for period in 0..=u16::MAX {
            let expected = (legacy_fraction * f32::from(period)) as u16;
            assert_eq!(limits.pwm_cap(period), expected);
            assert_eq!(limits.pwm_cap(period), expected, "cache hit");
        }
    }
    // Driver boot derives the fraction from timer counts. Preserve its
    // float round trip rather than replacing the truncated cap with `duty`.
    for period in [1, 2, 999, 1000, 4199, 8399, u16::MAX] {
        for duty in [0, period / 2, period.saturating_sub(1), period] {
            let fraction = f32::from(duty) / f32::from(period);
            limits.set_duty_fraction(fraction);
            let expected = (clamp_f32(fraction, 0.5, 1.0) * f32::from(period)) as u16;
            assert_eq!(limits.pwm_cap(period), expected);
        }
    }
}

#[test]
fn setters_keep_effective_limit_coherent_at_equal_and_nonfinite_boundaries() {
    for fraction in [0.5, 0.95, 1.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut foc = Controller::new(24.0);
        foc.set_duty_cap(fraction);
        let cap_limit = foc.modulation.duty_fraction * FRAC_1_SQRT_3;
        for requested in [
            f32::from_bits(cap_limit.to_bits() - 1),
            cap_limit,
            f32::from_bits(cap_limit.to_bits() + 1),
            -0.0,
            0.0,
            -1.0,
            1.0,
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
        ] {
            foc = foc.with_modulation_limit(requested);
            for vbus in [0.0, -1.0, 0.5, 12.0, 24.0, 60.0, f32::NAN, f32::INFINITY] {
                foc.set_vbus(vbus);
                assert_eq!(
                    foc.voltage_limit().to_bits(),
                    foc.legacy_voltage_limit().to_bits()
                );
            }
        }
        // Reverse the setter order, including a previously cached period.
        for period in [0, 4199, 8399, u16::MAX] {
            foc.modulation.pwm_cap(period);
            foc.set_duty_cap(0.73);
            assert_eq!(
                foc.modulation.pwm_cap(period),
                (0.73 * f32::from(period)) as u16
            );
            assert_eq!(
                foc.voltage_limit().to_bits(),
                foc.legacy_voltage_limit().to_bits()
            );
        }
    }
    for foc in [
        Controller::new(24.0),
        Controller::from_motor_params(0.05, 0.0001, 24.0),
        Controller::from_motor_params_with_bw(0.05, 0.0001, 24.0, 1500.0),
    ] {
        assert_eq!(
            foc.voltage_limit().to_bits(),
            foc.legacy_voltage_limit().to_bits()
        );
        assert_eq!(foc.modulation.pwm_cap(4199), 4199);
    }
}

fn configure_cycle(foc: Controller, cycle: usize) -> Controller {
    let mut foc = foc;
    if cycle.is_multiple_of(17) {
        let limits = [0.0, 0.25, 0.577, FRAC_1_SQRT_3, -1.0, 1.0, f32::NAN];
        foc = foc.with_modulation_limit(limits[(cycle / 17) % limits.len()]);
    }
    if cycle.is_multiple_of(19) {
        let caps = [
            0.95,
            0.5,
            0.72,
            1.0,
            0.0,
            -1.0,
            2.0,
            f32::NAN,
            f32::INFINITY,
        ];
        foc.set_duty_cap(caps[(cycle / 19) % caps.len()]);
    }
    let buses = [0.0, 0.49, 0.5, 12.0, 24.0, 48.0, f32::NAN, f32::INFINITY];
    foc.set_vbus(buses[(cycle / 23) % buses.len()]);
    if cycle.is_multiple_of(31) {
        foc.reset();
    }
    if cycle.is_multiple_of(37) {
        foc.set_dead_time_comp(800, 20_000);
        foc.set_dead_time_comp_factor(if cycle.is_multiple_of(74) { 0.0 } else { 0.016 });
        foc.set_dead_time_comp_linear_region(1.0).unwrap();
        assert!(foc.set_dead_time_comp_linear_region(f32::NAN).is_err());
    }
    foc.set_actuation_advance(match cycle % 3 {
        0 => 0.2,
        1 => -0.2,
        _ => f32::NAN,
    });
    foc.set_decoupling(if cycle.is_multiple_of(3) {
        None
    } else {
        Some(Decoupling {
            ld_h: 0.00008,
            lq_h: 0.00012,
            flux_linkage_wb: 0.004,
            resistance_ohm: 0.05,
        })
    });
    foc.set_current_sense_topology(if cycle.is_multiple_of(5) {
        crate::types::CurrentSenseTopology::ThreeShunt
    } else {
        crate::types::CurrentSenseTopology::TwoShunt
    });
    foc
}

#[test]
fn differential_trajectories_keep_priority_antiwindup_duties_and_fresh_vbus() {
    let mut cached = Controller::new(24.0);
    let mut legacy = Controller::new(24.0);
    for cycle in 0..4_000 {
        cached = configure_cycle(cached, cycle);
        legacy = configure_cycle(legacy, cycle);
        // Exercise stable periods as well as immediate changes, including
        // zero and the full u16 range. Both control entry points share them.
        let period = [0, 1, 2, 999, 4199, 8399, u16::MAX][(cycle / 7) % 7];
        let angle = (cycle % 101) as f32 * 0.1;
        let currents = (
            (cycle % 37) as f32 - 18.0,
            (cycle % 29) as f32 - 14.0,
            (cycle % 23) as f32 - 11.0,
        );
        let id = if cycle.is_multiple_of(2) { 80.0 } else { -30.0 };
        let iq = if cycle.is_multiple_of(3) {
            -120.0
        } else {
            90.0
        };
        let (vd_inj, vq_inj) = if cycle.is_multiple_of(5) {
            (2.0, -3.0)
        } else {
            (0.0, 0.0)
        };
        let dt = if cycle.is_multiple_of(2) {
            0.00005
        } else {
            0.0001
        };
        let actual =
            cached.step_with_injection(currents, angle, 1500.0, id, iq, vd_inj, vq_inj, period, dt);
        let expected = legacy.legacy_step_with_injection(
            currents, angle, 1500.0, id, iq, vd_inj, vq_inj, period, dt,
        );
        assert_output(actual, expected);
        assert_eq!(
            cached.id_pi.get_integral().to_bits(),
            legacy.id_pi.get_integral().to_bits()
        );
        assert_eq!(
            cached.iq_pi.get_integral().to_bits(),
            legacy.iq_pi.get_integral().to_bits()
        );
        assert_eq!(
            cached.q_voltage_clip().to_bits(),
            legacy.q_voltage_clip().to_bits()
        );
        assert_eq!(
            cached.bus_mod_q().coefficient().to_bits(),
            legacy.bus_mod_q().coefficient().to_bits()
        );
        assert_output(
            cached.apply_dq(id, iq, angle, currents.0, currents.1, period),
            legacy.legacy_apply_dq(id, iq, angle, currents.0, currents.1, period),
        );
    }
}

#[test]
fn concurrent_shared_voltage_calls_keep_distinct_periods_coherent() {
    let mut foc = Controller::new(24.0).with_modulation_limit(0.4);
    foc.set_duty_cap(0.73);
    foc.set_dead_time_comp_factor(0.016);
    let periods = [0, 1, 2, 999, 4199, 8399, 32767, u16::MAX];
    let barrier = std::sync::Barrier::new(periods.len());
    std::thread::scope(|scope| {
        for period in periods {
            let foc = &foc;
            let barrier = &barrier;
            scope.spawn(move || {
                barrier.wait();
                for cycle in 0..10_000 {
                    let angle = (cycle % 67) as f32 * 0.1;
                    let (vd, vq) = if cycle % 2 == 0 {
                        (30.0, -40.0)
                    } else {
                        (-40.0, 30.0)
                    };
                    assert_output(
                        foc.apply_dq(vd, vq, angle, 2.0, -3.0, period),
                        foc.legacy_apply_dq(vd, vq, angle, 2.0, -3.0, period),
                    );
                }
            });
        }
    });
}

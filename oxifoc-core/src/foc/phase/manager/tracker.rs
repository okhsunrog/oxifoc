//! Second-order commutation tracker over the observer estimate.

use super::{
    PhaseManager, TRACKER_CATCHUP_TAU_S, TRACKER_FF_TAU_S, TRACKER_HUNT_DAMP, TRACKER_HUNT_TAU_S,
    TRACKER_KD_TAU_S, TRACKER_MAX_LAG_RAD,
};
use crate::foc::angle_difference;
use crate::foc::phase::provider::PhaseOutput;
use crate::foc::sensors::AngleSensor;
use crate::foc::trig::SinCos;
use crate::foc::wrap_angle;

impl<H: AngleSensor, E: AngleSensor, S: SinCos> PhaseManager<H, E, S> {
    // ========================================================================
    // Internal phase computation
    // ========================================================================

    /// One step of the commutation phase tracker (see [`PhaseTracker`](super::PhaseTracker)).
    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    pub(super) fn tracker_output(&mut self, raw: PhaseOutput, dt: f32) -> PhaseOutput {
        let prof_t0 = crate::isr_prof::now();
        let out = self.tracker_output_inner(raw, dt);
        crate::isr_prof::add(&crate::isr_prof::OUT_TRK, prof_t0, crate::isr_prof::now());
        out
    }

    #[cfg_attr(feature = "isr-speed", optimize(speed))]
    fn tracker_output_inner(&mut self, raw: PhaseOutput, dt: f32) -> PhaseOutput {
        let tr = &mut self.tracker;
        if !tr.active {
            // Engage on the estimate: zero initial gap (the same angle step
            // a plain confirmed handoff performs) at its frequency. Not the
            // last output's: after a deadshort catch that is the probe's
            // standstill hold, ω = 0 — the tracker then trailed a
            // 1000 rad/s rotor at the lag clamp and tripped OC at 2000 on a
            // 5065 (flying_restart_catches_low_tau_motors). The freq-led
            // predecessor's `self.output` angle seed froze the startup
            // ramp's ~90° load angle into cruise — see the PhaseTracker doc
            // and docs/TODO.md dossier.
            tr.theta = raw.angle;
            tr.omega = raw.velocity;
            tr.d_filt = 0.0;
            tr.v_slow = raw.velocity;
            tr.a_est = 0.0;
            tr.hunt_filt = 0.0;
            tr.active = true;
        } else {
            // Timing contract: `raw` already predicts the NEXT sampling
            // instant (the observer PLL advances one step itself), and the
            // previous output predicted THIS one — so move the frame one
            // period first, THEN compare. The original compare-then-advance
            // order (a PLL's own step) predicted a second period on top:
            // commutation ran one PWM period early.
            tr.theta = wrap_angle(tr.theta + tr.omega * dt);
        }
        let d = angle_difference(raw.angle, tr.theta);
        // Acceleration feedforward (see TRACKER_FF_TAU_S): the filtered
        // trend of the estimate's own velocity.
        let a_ff = (dt / TRACKER_FF_TAU_S).min(1.0);
        let v_slow_prev = tr.v_slow;
        tr.v_slow += a_ff * (raw.velocity - tr.v_slow);
        tr.a_est += a_ff * ((tr.v_slow - v_slow_prev) / dt - tr.a_est);
        // Frequency integrator: type-2 + acceleration feedforward — Δ → 0
        // at constant speed AND under slow acceleration trends; only the
        // trend ERROR (unmodeled load steps, the first ~2τ of a punch)
        // produces lag, and the hard clamp below bounds that.
        tr.omega += (tr.kp * d + tr.a_est) * dt;
        // Damping path on the low-passed Δ (see TRACKER_KD_TAU_S): the
        // LPF removes the PI zero's high-frequency leakage, NOT the
        // restoring force — the lagged-pull negative-damping trap of the
        // freq-led era applied lag to the whole stiffness; here the
        // un-lagged kp path carries it.
        let a_d = (dt / TRACKER_KD_TAU_S).min(1.0);
        tr.d_filt += a_d * (d - tr.d_filt);
        // Hunting damper (see TRACKER_HUNT_DAMP): yield with the rotor's
        // swing velocity so the quasi-synchronous frame extracts energy
        // from the hunt mode instead of storing it.
        let a_h = (dt / TRACKER_HUNT_TAU_S).min(1.0);
        tr.hunt_filt += a_h * (TRACKER_HUNT_DAMP * (raw.velocity - tr.omega) - tr.hunt_filt);
        tr.theta = wrap_angle(tr.theta + (tr.kd * tr.d_filt + tr.hunt_filt) * dt);
        // Hard load-angle bound. A soft tracker that filters the 35–100 Hz
        // mid-band wobble cannot also follow an unloaded max-torque punch
        // (ω̇ up to ~5.6 k rad/s² at 1.5 A): the type-2 lag ω̇/ωn²
        // self-consistently settles near the torque-curve flat top —
        // bench 2026-07-08: ωn=30 climbed at 1.36 k rad/s² with an ~77°
        // standing lag and died in the same dq OC as the freq-led
        // geometry. The clamp makes the failure mode impossible instead
        // of tuning it away: the frame never trails the estimate by more
        // than TRACKER_MAX_LAG_RAD (cos 34° = 0.83 of full torque, far
        // from the flat top). While it binds the output slides WITH the
        // raw estimate (wobble passes — acceptable: hard transients are
        // loud, the mid-band cycle is a light-load phenomenon) and the
        // frequency catches up on a fast τ so the clamp releases into
        // soft tracking.
        let d2 = angle_difference(raw.angle, tr.theta);
        if d2.abs() > TRACKER_MAX_LAG_RAD {
            let sign = if d2 > 0.0 { 1.0 } else { -1.0 };
            tr.theta = wrap_angle(raw.angle - sign * TRACKER_MAX_LAG_RAD);
            let a_v = (dt / TRACKER_CATCHUP_TAU_S).min(1.0);
            // Ramp-lag-compensated target: pulling toward a ramping
            // velocity alone leaves a standing ω̇·τ deficit and the clamp
            // never releases during a sustained spin-up (see
            // TRACKER_CATCHUP_TAU_S).
            let target = raw.velocity + tr.a_est * TRACKER_CATCHUP_TAU_S;
            tr.omega += a_v * (target - tr.omega);
        }
        PhaseOutput {
            angle: tr.theta,
            velocity: tr.omega,
        }
    }
}

//! Velocity control loop building block: slew-limited reference + clamped PI.
//!
//! One *type*, several *instances* (share the mechanism, not the instance —
//! see docs/safety.md):
//!
//! - [`ControlMode::VelocityControl`](crate::motor::ControlMode) — the
//!   normal, host-tunable cruise loop owned by `FocDriver`.
//! - the failsafe's ControlledStop (`motor::failsafe`): a decel-limited ramp
//!   to zero through a *separate* instance with fixed conservative gains, so
//!   a mis-tuned cruise loop can never become the link-loss safety net.
//! - (planned) position control: a position P term feeds `omega_target`
//!   into the same block (cascade).
//!
//! The loop is pure math — no hardware, channels, or async. The caller
//! (driver/failsafe) pushes in the measured velocity and `dt` every cycle
//! and routes the returned q-current through the normal current loop, so
//! the current-limit clamp and the overcurrent trip still apply.
//!
//! Units: electrical rad/s in, Amps (q-axis) out. Electrical because that
//! is what [`PhaseProvider::get`](crate::foc::phase::PhaseProvider::get)
//! measures; hosts convert mechanical targets via pole pairs.

use crate::foc::pi_controller::ClampedPI;
#[cfg(feature = "storage")]
use crate::storage::VelocityConfigStored;

/// Slew-rate limiter: a value that moves toward a target at a bounded rate.
///
/// Used as the velocity-reference ramp (accel/decel limit). `rate <= 0` or
/// non-finite disables limiting (the value jumps straight to the target).
#[derive(Debug)]
pub struct SlewLimiter {
    value: f32,
    /// Max |d(value)/dt| in units per second.
    rate: f32,
}

impl SlewLimiter {
    pub const fn new(rate: f32) -> Self {
        Self { value: 0.0, rate }
    }

    /// Jump the internal value (bumpless re-arm: seed from the measurement).
    pub fn reset_to(&mut self, value: f32) {
        self.value = value;
    }

    /// Current (ramped) value.
    pub fn value(&self) -> f32 {
        self.value
    }

    pub fn set_rate(&mut self, rate: f32) {
        self.rate = rate;
    }

    /// Advance one cycle toward `target`, moving at most `rate * dt`.
    pub fn step(&mut self, target: f32, dt: f32) -> f32 {
        if self.rate <= 0.0 || !self.rate.is_finite() {
            self.value = target;
            return self.value;
        }
        let max_step = self.rate * dt;
        let d = target - self.value;
        self.value = if d.abs() <= max_step {
            target
        } else if d > 0.0 {
            self.value + max_step
        } else {
            self.value - max_step
        };
        self.value
    }
}

/// Velocity-loop tuning. Motor- and load-dependent (the right gains scale
/// with inertia, which the firmware cannot know) — host-tunable for the
/// cruise loop, fixed-conservative for safety instances.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityLoopConfig {
    /// Proportional gain, A per (electrical rad/s) of velocity error.
    pub kp: f32,
    /// Integral gain, A per (electrical rad/s · s) of accumulated error.
    pub ki: f32,
    /// Reference ramp limit, electrical rad/s² (both accel and decel).
    /// `<= 0` disables the ramp (step targets go straight to the PI).
    pub accel_limit: f32,
}

impl Default for VelocityLoopConfig {
    /// Deliberately soft starting point. The hard constraint is the velocity
    /// *estimate* update rate, not the current loop: a hall source only
    /// learns velocity at edges (6 per electrical rev), so the torque the
    /// loop commands must not change the speed significantly within one edge
    /// interval, or the stale estimate turns into a self-reinforcing
    /// oscillation. Soft kp/ki + the accel ramp keep the per-edge speed
    /// change small on an untuned motor; raise per motor + load (and per
    /// estimator — an encoder/observer source tolerates much more).
    fn default() -> Self {
        Self {
            kp: 0.01,
            ki: 0.2,
            accel_limit: 500.0,
        }
    }
}

impl VelocityLoopConfig {
    /// Build from the stored (host-writable) form. A missing or non-sane
    /// stored value falls back to [`Default`].
    #[cfg(feature = "storage")]
    pub fn from_stored(cfg: Option<&VelocityConfigStored>) -> Self {
        match cfg {
            Some(c) => {
                let candidate = Self {
                    kp: c.kp,
                    ki: c.ki,
                    accel_limit: c.accel_limit,
                };
                if candidate.is_sane() {
                    candidate
                } else {
                    Self::default()
                }
            }
            None => Self::default(),
        }
    }

    /// All fields finite, gains non-negative, at least one gain positive.
    pub fn is_sane(&self) -> bool {
        self.kp.is_finite()
            && self.ki.is_finite()
            && self.accel_limit.is_finite()
            && self.kp >= 0.0
            && self.ki >= 0.0
            && (self.kp > 0.0 || self.ki > 0.0)
    }
}

/// Velocity loop: ω_target → slew-limited ω_ref → PI(ω_ref − ω_meas) → iq.
///
/// The PI output is clamped to the caller-supplied current limit each cycle
/// with back-calculation anti-windup ([`ClampedPI`]), so saturating into the
/// limit (steep hill, step target) doesn't wind the integrator up.
#[derive(Debug)]
pub struct VelocityLoop {
    pi: ClampedPI,
    ramp: SlewLimiter,
    cfg: VelocityLoopConfig,
    /// Last commanded iq (A) — lets the failsafe arm bumpless from this loop.
    last_iq: f32,
}

impl VelocityLoop {
    pub fn new(cfg: VelocityLoopConfig) -> Self {
        Self {
            pi: ClampedPI::new(cfg.kp, cfg.ki, 0.0, 0.0),
            ramp: SlewLimiter::new(cfg.accel_limit),
            cfg,
            last_iq: 0.0,
        }
    }

    /// Apply new tuning (sane values only) without disturbing the state.
    pub fn set_config(&mut self, cfg: VelocityLoopConfig) {
        if cfg.is_sane() {
            self.cfg = cfg;
            self.pi.set_gains(cfg.kp, cfg.ki);
            self.ramp.set_rate(cfg.accel_limit);
        }
    }

    pub fn config(&self) -> VelocityLoopConfig {
        self.cfg
    }

    /// Re-arm bumpless: seed the reference ramp at the measured velocity and
    /// clear the integrator. Call on (re-)entry into velocity mode so the
    /// loop ramps *from where the rotor is*, not from zero.
    pub fn reset(&mut self, omega_meas: f32) {
        self.ramp.reset_to(if omega_meas.is_finite() {
            omega_meas
        } else {
            0.0
        });
        self.pi.reset();
        self.last_iq = 0.0;
    }

    /// Last commanded q-current (A) from the previous [`step`](Self::step).
    pub fn last_iq(&self) -> f32 {
        self.last_iq
    }

    /// One cycle. `iq_limit` is the current-limit magnitude (A); `<= 0` or
    /// non-finite means unlimited (mirrors `CurrentLimits` semantics — the
    /// downstream `clamp_targets` is still the real ceiling).
    pub fn step(&mut self, omega_target: f32, omega_meas: f32, iq_limit: f32, dt: f32) -> f32 {
        let limit = if iq_limit > 0.0 && iq_limit.is_finite() {
            iq_limit
        } else {
            f32::INFINITY
        };
        self.step_clamped(omega_target, omega_meas, -limit, limit, dt)
    }

    /// One cycle with explicit asymmetric output bounds — for unidirectional
    /// users like the failsafe brake, which must only ever oppose the
    /// original rotation. Anti-windup tracks these exact bounds, so pinning
    /// one side at zero doesn't wind the integrator.
    pub fn step_clamped(
        &mut self,
        omega_target: f32,
        omega_meas: f32,
        iq_min: f32,
        iq_max: f32,
        dt: f32,
    ) -> f32 {
        self.pi.set_limits(iq_min, iq_max);
        let omega_ref = self.ramp.step(omega_target, dt);
        self.last_iq = self.pi.update(omega_ref, omega_meas, dt);
        self.last_iq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 20_000.0;

    #[test]
    fn slew_limiter_bounds_rate() {
        let mut s = SlewLimiter::new(100.0); // 100 units/s
        s.reset_to(0.0);
        // One cycle moves at most rate*dt.
        let v = s.step(50.0, DT);
        assert!((v - 100.0 * DT).abs() < 1e-6, "step {v}");
        // Converges to the target and stays there.
        for _ in 0..20_000 {
            s.step(50.0, DT);
        }
        assert_eq!(s.value(), 50.0);
        // Symmetric downward.
        let v = s.step(0.0, DT);
        assert!((v - (50.0 - 100.0 * DT)).abs() < 1e-4);
    }

    #[test]
    fn slew_limiter_unlimited_when_rate_unset() {
        let mut s = SlewLimiter::new(0.0);
        assert_eq!(s.step(123.0, DT), 123.0);
        let mut s = SlewLimiter::new(f32::NAN);
        assert_eq!(s.step(-7.0, DT), -7.0);
    }

    /// First-order mechanical plant: J·dω/dt = kt·iq − b·ω.
    fn run_plant(
        loop_: &mut VelocityLoop,
        omega0: f32,
        target: f32,
        iq_limit: f32,
        cycles: u32,
    ) -> (f32, f32) {
        let (j, kt, b) = (5e-5, 0.02, 1e-5);
        let mut omega = omega0;
        let mut max_iq = 0.0f32;
        for _ in 0..cycles {
            let iq = loop_.step(target, omega, iq_limit, DT);
            max_iq = max_iq.max(iq.abs());
            omega += (kt * iq - b * omega) / j * DT;
        }
        (omega, max_iq)
    }

    #[test]
    fn tracks_target_velocity() {
        let mut vl = VelocityLoop::new(VelocityLoopConfig {
            kp: 0.05,
            ki: 2.0,
            accel_limit: 5_000.0,
        });
        vl.reset(0.0);
        let (omega, _) = run_plant(&mut vl, 0.0, 300.0, 40.0, 40_000); // 2 s
        assert!(
            (omega - 300.0).abs() < 10.0,
            "should track 300 rad/s, got {omega}"
        );
        // Retarget downward without reset — the ramp carries it.
        let (omega, _) = run_plant(&mut vl, omega, 100.0, 40.0, 40_000);
        assert!(
            (omega - 100.0).abs() < 10.0,
            "should track 100 rad/s, got {omega}"
        );
    }

    #[test]
    fn output_clamped_to_iq_limit_without_windup() {
        let mut vl = VelocityLoop::new(VelocityLoopConfig {
            kp: 0.5,
            ki: 50.0,
            accel_limit: 0.0, // step target straight in → guaranteed saturation
        });
        vl.reset(0.0);
        let (_omega, max_iq) = run_plant(&mut vl, 0.0, 2_000.0, 10.0, 10_000);
        assert!(max_iq <= 10.0 + 1e-3, "iq must respect the limit: {max_iq}");
        // After the long saturated stretch, the integrator must not have
        // wound up: retargeting to the current velocity must not overshoot
        // the limit-relaxed response wildly. (Back-calculation keeps the
        // integral pinned near the limit.)
        assert!(vl.last_iq() <= 10.0 + 1e-3);
    }

    #[test]
    fn reset_is_bumpless() {
        let mut vl = VelocityLoop::new(VelocityLoopConfig::default());
        vl.reset(250.0);
        // First cycle with ω_meas == ramp seed: zero error, ~zero output.
        let iq = vl.step(250.0, 250.0, 40.0, DT);
        assert!(iq.abs() < 0.1, "bumpless entry, got iq {iq}");
    }

    #[test]
    #[cfg(feature = "storage")]
    fn from_stored_maps_and_falls_back() {
        use crate::storage::VelocityConfigStored;
        // Stored default mirrors the runtime default.
        assert_eq!(
            VelocityLoopConfig::from_stored(Some(&VelocityConfigStored::default())),
            VelocityLoopConfig::default()
        );
        // Missing or non-sane → default.
        assert_eq!(
            VelocityLoopConfig::from_stored(None),
            VelocityLoopConfig::default()
        );
        let bad = VelocityConfigStored {
            kp: f32::NAN,
            ..VelocityConfigStored::default()
        };
        assert_eq!(
            VelocityLoopConfig::from_stored(Some(&bad)),
            VelocityLoopConfig::default()
        );
    }

    #[test]
    fn config_sanity() {
        assert!(VelocityLoopConfig::default().is_sane());
        assert!(
            !VelocityLoopConfig {
                kp: 0.0,
                ki: 0.0,
                accel_limit: 100.0
            }
            .is_sane()
        );
        assert!(
            !VelocityLoopConfig {
                kp: f32::NAN,
                ..VelocityLoopConfig::default()
            }
            .is_sane()
        );
        // set_config ignores garbage.
        let mut vl = VelocityLoop::new(VelocityLoopConfig::default());
        let before = vl.config();
        vl.set_config(VelocityLoopConfig {
            kp: -1.0,
            ..VelocityLoopConfig::default()
        });
        assert_eq!(vl.config(), before);
    }
}

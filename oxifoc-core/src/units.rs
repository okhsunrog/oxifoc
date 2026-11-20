//! Typed physical quantities for motor-model arithmetic.
//!
//! Thin layer over [`uom`] (the `uom-ng` fork) with `f32` storage in SI base
//! units. Every quantity is a `#[repr(transparent)]`-like wrapper over one
//! `f32`: arithmetic compiles to the same FPU instructions as plain floats,
//! while the compiler checks dimensions, for example that `ω·λ` is a voltage
//! and `√(P / R)` a current.
//!
//! Rotor angles and speeds are electrical by default
//! ([`ElectricalAngularVelocity`], the mechanical speed times the pole pairs).
//! Mechanical ones use [`AngularVelocity`]. The two are different uom kinds:
//! they do not add, compare or convert into each other except through
//! [`Electrical::to_mechanical`] and [`Mechanical::to_electrical`], which
//! take the pole pairs. Control bandwidths are plain [`AngularVelocity`].
//!
//! Constructors here take the value in the SI base unit (ampere, volt, ohm,
//! henry, weber, watt, rad/s, second) and are `const`, so model constants can
//! be quantities too.
//!
//! `no_std`: `uom` uses `FloatCore` here and has no `sqrt`; take roots with
//! [`Quantity::sqrt_with`](uom::si::Quantity::sqrt_with) and
//! [`crate::foc::fast_math::sqrtf`], which keeps `libm` out of the firmware.
//!
//! When converting f32 code, keep its operations as they were: a clamp it
//! did not have (`sqrt_current` clamps at zero) or moved statements change
//! the ISR code. Compare the disassembly of the hot functions before and
//! after.

use core::marker::PhantomData;
use core::ops::{Div, Mul};

pub use uom::ConstZero;
pub use uom::si::f32::{
    AngularVelocity, ElectricCurrent, ElectricPotential, ElectricalAngle,
    ElectricalAngularAcceleration, ElectricalAngularVelocity, ElectricalResistance, Frequency,
    Inductance, MagneticFlux, Power, Ratio, Time,
};

/// Unit markers for reading a quantity back, e.g. `q.get::<ampere>()`.
pub mod unit {
    pub use uom::si::angular_velocity::radian_per_second;
    /// Electrical radians, rad/s and eRPM.
    pub mod electrical {
        pub use uom::si::electrical_angle::radian;
        pub use uom::si::electrical_angular_velocity::{
            radian_per_second, revolution_per_minute as erpm,
        };
    }
    /// Mechanical revolutions per minute.
    pub use uom::si::angular_velocity::revolution_per_minute as rpm;
    pub use uom::si::electric_current::ampere;
    pub use uom::si::electric_potential::volt;
    pub use uom::si::electrical_resistance::ohm;
    pub use uom::si::frequency::hertz;
    pub use uom::si::inductance::henry;
    pub use uom::si::magnetic_flux::weber;
    pub use uom::si::power::watt;
    pub use uom::si::time::second;
}

/// `A/s`, the rate at which a current changes (failsafe and ramp slews).
pub type CurrentRate = <ElectricCurrent as Div<Time>>::Output;

/// `A²`, the dimension of `i²` terms in copper-loss and current-circle
/// bounds.
pub type CurrentSquared = <ElectricCurrent as Mul<ElectricCurrent>>::Output;

/// `1/s²`, the integral gain of a phase-locked loop: angle error in, angular
/// acceleration out.
pub type PllIntegralGain = <Frequency as Mul<Frequency>>::Output;

/// Any quantity from its value in SI base units, in a `const` context (the
/// `const` counterpart of [`si`] for quantities without a named constructor).
#[inline(always)]
#[must_use]
pub const fn quantity<D, U>(value: f32) -> uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    uom::si::Quantity {
        dimension: PhantomData,
        units: PhantomData,
        value,
    }
}

/// Electrical angle of the vector `(x, y)` of one quantity (a flux or current
/// vector in the αβ frame), through the firmware's polynomial atan2.
#[inline(always)]
#[must_use]
pub fn vector_angle<D, U>(
    x: uom::si::Quantity<D, U, f32>,
    y: uom::si::Quantity<D, U, f32>,
) -> ElectricalAngle
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    erad(crate::foc::fast_math::atan2f(y.value, x.value))
}

macro_rules! si_constructor {
    ($(#[$doc:meta])* $name:ident -> $quantity:ident) => {
        $(#[$doc])*
        #[inline(always)]
        #[must_use]
        pub const fn $name(value: f32) -> $quantity {
            $quantity {
                dimension: PhantomData,
                units: PhantomData,
                value,
            }
        }
    };
}

si_constructor!(
    /// Current in amperes.
    amps -> ElectricCurrent
);
si_constructor!(
    /// Voltage in volts.
    volts -> ElectricPotential
);
si_constructor!(
    /// Resistance in ohms.
    ohms -> ElectricalResistance
);
si_constructor!(
    /// Inductance in henries.
    henries -> Inductance
);
si_constructor!(
    /// Flux linkage in webers (V·s).
    webers -> MagneticFlux
);
si_constructor!(
    /// Power in watts.
    watts -> Power
);
si_constructor!(
    /// Angular velocity in rad/s: a mechanical speed or a control bandwidth.
    rad_per_s -> AngularVelocity
);
si_constructor!(
    /// Electrical angular velocity in electrical rad/s.
    erad_per_s -> ElectricalAngularVelocity
);
si_constructor!(
    /// Electrical angle in electrical radians.
    erad -> ElectricalAngle
);
si_constructor!(
    /// Electrical angular acceleration in electrical rad/s².
    erad_per_s2 -> ElectricalAngularAcceleration
);
si_constructor!(
    /// Current slew rate in A/s.
    amps_per_s -> CurrentRate
);
si_constructor!(
    /// Time in seconds.
    seconds -> Time
);
si_constructor!(
    /// Frequency in hertz.
    hertz -> Frequency
);
si_constructor!(
    /// A rate or gain in 1/s: a filter or loop gain, or an angular frequency
    /// in rad/s (a loop's natural frequency). Same storage as [`hertz`].
    per_second -> Frequency
);

/// Electrical angular velocity from eRPM.
#[inline(always)]
#[must_use]
pub const fn erpm(value: f32) -> ElectricalAngularVelocity {
    erad_per_s(value * (core::f32::consts::PI / 30.0))
}

/// A plain `f32` or any quantity, built from a value in SI base units.
///
/// For generated code that only knows a JSON number, such as
/// `config dump --rust`: the field's type picks the implementation.
pub trait FromSi {
    #[must_use]
    fn from_si(value: f32) -> Self;
}

impl FromSi for f32 {
    #[inline(always)]
    fn from_si(value: f32) -> Self {
        value
    }
}

impl<D, U> FromSi for uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    #[inline(always)]
    fn from_si(value: f32) -> Self {
        Self {
            dimension: PhantomData,
            units: PhantomData,
            value,
        }
    }
}

/// [`FromSi::from_si`] as a function: `max_iq: si(30.0)`.
#[inline(always)]
#[must_use]
pub fn si<T: FromSi>(value: f32) -> T {
    T::from_si(value)
}

/// Sign tests on a quantity's value, with the same NaN behavior as the
/// matching `f32` comparison (`is_non_positive` is `value <= 0.0`, so NaN is
/// neither positive nor non-positive).
pub trait Sign {
    #[must_use]
    fn is_positive(&self) -> bool;
    #[must_use]
    fn is_negative(&self) -> bool;
    #[must_use]
    fn is_non_negative(&self) -> bool;
    #[must_use]
    fn is_non_positive(&self) -> bool;
}

impl<D, U> Sign for uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    #[inline(always)]
    fn is_positive(&self) -> bool {
        self.value > 0.0
    }
    #[inline(always)]
    fn is_negative(&self) -> bool {
        self.value < 0.0
    }
    #[inline(always)]
    fn is_non_negative(&self) -> bool {
        self.value >= 0.0
    }
    #[inline(always)]
    fn is_non_positive(&self) -> bool {
        self.value <= 0.0
    }
}

impl Sign for f32 {
    #[inline(always)]
    fn is_positive(&self) -> bool {
        *self > 0.0
    }
    #[inline(always)]
    fn is_negative(&self) -> bool {
        *self < 0.0
    }
    #[inline(always)]
    fn is_non_negative(&self) -> bool {
        *self >= 0.0
    }
    #[inline(always)]
    fn is_non_positive(&self) -> bool {
        *self <= 0.0
    }
}

/// What generic control code (PI regulators, slew limiters) needs from a
/// plain `f32` or a quantity. `clamp_to` has the semantics of
/// [`crate::foc::clamp_f32`] (no assert, NaN passes through), so the `f32`
/// instance compiles to the same code as before.
pub trait Scalar: Copy + PartialOrd + ConstZero {
    const INFINITY: Self;
    const NEG_INFINITY: Self;
    #[must_use]
    fn is_nan(self) -> bool;
    #[must_use]
    fn is_finite(self) -> bool;
    #[must_use]
    fn abs(self) -> Self;
    #[must_use]
    fn clamp_to(self, min: Self, max: Self) -> Self;
    /// The magnitude of `self` with the sign of `sign` (`f32::copysign`).
    #[must_use]
    fn copysign(self, sign: f32) -> Self;
}

#[allow(
    clippy::use_self,
    reason = "`f32::INFINITY` reads as the float constant, `Self::INFINITY` as this trait's own"
)]
impl Scalar for f32 {
    const INFINITY: Self = f32::INFINITY;
    const NEG_INFINITY: Self = f32::NEG_INFINITY;
    #[inline(always)]
    fn is_nan(self) -> bool {
        f32::is_nan(self)
    }
    #[inline(always)]
    fn is_finite(self) -> bool {
        f32::is_finite(self)
    }
    #[inline(always)]
    fn abs(self) -> Self {
        f32::abs(self)
    }
    #[inline(always)]
    fn clamp_to(self, min: Self, max: Self) -> Self {
        crate::foc::clamp_f32(self, min, max)
    }
    #[inline(always)]
    fn copysign(self, sign: f32) -> Self {
        f32::copysign(self, sign)
    }
}

impl<D, U> Scalar for uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    const INFINITY: Self = Self {
        dimension: PhantomData,
        units: PhantomData,
        value: f32::INFINITY,
    };
    const NEG_INFINITY: Self = Self {
        dimension: PhantomData,
        units: PhantomData,
        value: f32::NEG_INFINITY,
    };
    #[inline(always)]
    fn is_nan(self) -> bool {
        self.value.is_nan()
    }
    #[inline(always)]
    fn is_finite(self) -> bool {
        self.value.is_finite()
    }
    #[inline(always)]
    fn abs(self) -> Self {
        Self::from_si(f32::abs(self.value))
    }
    #[inline(always)]
    fn clamp_to(self, min: Self, max: Self) -> Self {
        Self::from_si(crate::foc::clamp_f32(self.value, min.value, max.value))
    }
    #[inline(always)]
    fn copysign(self, sign: f32) -> Self {
        Self::from_si(self.value.copysign(sign))
    }
}

/// The raw bit pattern of a quantity's value, for caches keyed on an exact
/// input (signed zeros and NaN payloads stay distinct) and bit-exact tests.
pub trait ToBits {
    #[must_use]
    fn to_bits(self) -> u32;
}

impl<D, U> ToBits for uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    #[inline(always)]
    fn to_bits(self) -> u32 {
        self.value.to_bits()
    }
}

/// A dimensionless plain quantity (a [`Ratio`]) as `f32`: a modulation
/// index from `V / V_bus`, or a current over a reference current.
pub trait Fraction {
    #[must_use]
    fn fraction(self) -> f32;
}

impl Fraction for Ratio {
    #[inline(always)]
    fn fraction(self) -> f32 {
        self.value
    }
}

/// The ratio of two values of the same quantity (same dimension and kind) as
/// a plain `f32`: a fraction, a scale or a position along a ramp.
pub trait Linear: Copy {
    #[must_use]
    fn ratio_to(self, denominator: Self) -> f32;
}

impl Linear for f32 {
    #[inline(always)]
    fn ratio_to(self, denominator: Self) -> f32 {
        self / denominator
    }
}

impl<D, U> Linear for uom::si::Quantity<D, U, f32>
where
    D: uom::si::Dimension + ?Sized,
    U: uom::si::Units<f32> + ?Sized,
{
    #[inline(always)]
    fn ratio_to(self, denominator: Self) -> f32 {
        self.value / denominator.value
    }
}

/// One electrical radian. A plain `1/s` that is an electrical speed by its
/// physics, such as `|e|/λ`, becomes an [`ElectricalAngularVelocity`] when
/// multiplied by it (exact: the factor is `1.0`). Division cannot recover the
/// kind on its own.
pub const ELECTRICAL_RADIAN: ElectricalAngle = erad(1.0);

/// An angle in radians as the FPU helpers (`wrap_angle`, `angle_difference`,
/// `SinCos`) see it: a plain `f32` or an [`ElectricalAngle`].
pub trait AngleRepr: Copy {
    #[must_use]
    fn radians(self) -> f32;
    #[must_use]
    fn from_radians(radians: f32) -> Self;
}

impl AngleRepr for f32 {
    #[inline(always)]
    fn radians(self) -> f32 {
        self
    }
    #[inline(always)]
    fn from_radians(radians: f32) -> Self {
        radians
    }
}

impl AngleRepr for ElectricalAngle {
    #[inline(always)]
    fn radians(self) -> f32 {
        self.value
    }
    #[inline(always)]
    fn from_radians(radians: f32) -> Self {
        erad(radians)
    }
}

/// Electrical to mechanical rotation: divide by the pole pairs.
pub trait Electrical {
    type Mechanical;
    #[must_use]
    fn to_mechanical(self, pole_pairs: u8) -> Self::Mechanical;
}

/// Mechanical to electrical rotation: multiply by the pole pairs.
pub trait Mechanical {
    type Electrical;
    #[must_use]
    fn to_electrical(self, pole_pairs: u8) -> Self::Electrical;
}

impl Electrical for ElectricalAngularVelocity {
    type Mechanical = AngularVelocity;
    #[inline(always)]
    fn to_mechanical(self, pole_pairs: u8) -> AngularVelocity {
        rad_per_s(self.value / f32::from(pole_pairs))
    }
}

impl Mechanical for AngularVelocity {
    type Electrical = ElectricalAngularVelocity;
    #[inline(always)]
    fn to_electrical(self, pole_pairs: u8) -> ElectricalAngularVelocity {
        erad_per_s(self.value * f32::from(pole_pairs))
    }
}

/// `√max(x, 0)` for an `A²` quantity, through the firmware's square root.
#[inline(always)]
#[must_use]
pub fn sqrt_current(x: CurrentSquared) -> ElectricCurrent {
    x.max(CurrentSquared::ZERO)
        .sqrt_with(crate::foc::fast_math::sqrtf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::si::electric_current::ampere;
    use uom::si::electric_potential::volt;
    use uom::si::power::watt;

    #[test]
    fn constructors_store_si_base_values() {
        assert_eq!(amps(2.5).get::<ampere>(), 2.5);
        assert_eq!(volts(-12.0).get::<volt>(), -12.0);
        assert_eq!(watts(0.0).get::<watt>(), 0.0);
    }

    #[test]
    fn derived_dimensions_check_out() {
        let emf: ElectricPotential = erad_per_s(400.0) * webers(2.92e-3);
        assert!((emf.get::<volt>() - 1.168).abs() < 1e-6);
        let loss: Power = ohms(0.036) * amps(10.0) * amps(10.0);
        assert!((loss.get::<watt>() - 3.6).abs() < 1e-6);
        assert_eq!(sqrt_current(amps(3.0) * amps(3.0)).get::<ampere>(), 3.0);
        assert_eq!(sqrt_current(-(amps(3.0) * amps(3.0))), amps(0.0));
    }

    #[test]
    fn electrical_and_mechanical_speeds_convert_through_pole_pairs() {
        use super::unit::electrical::erpm;
        use super::unit::rpm;
        let electrical = ElectricalAngularVelocity::new::<erpm>(7000.0);
        let mechanical = electrical.to_mechanical(7);
        assert!((mechanical.get::<rpm>() - 1000.0).abs() < 1e-3);
        assert_eq!(mechanical.to_electrical(7), electrical);
        assert!((super::erpm(7000.0).get::<erpm>() - 7000.0).abs() < 1e-3);
    }

    #[test]
    fn si_builds_plain_floats_and_quantities() {
        let plain: f32 = si(1.5);
        let current: ElectricCurrent = si(1.5);
        let speeds: [ElectricalAngularVelocity; 2] = [si(1.0), si(2.0)];
        assert_eq!(plain, 1.5);
        assert_eq!(current, amps(1.5));
        assert_eq!(speeds[1], erad_per_s(2.0));
    }
}

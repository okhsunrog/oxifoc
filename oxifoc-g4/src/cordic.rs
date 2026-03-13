//! CORDIC hardware-accelerated sin/cos for STM32G4
//!
//! Provides [`CordicSinCos`] implementing [`SinCos`] using the CORDIC
//! coprocessor for ~10x faster sin/cos than software `libm`.
//!
//! The CORDIC is configured once at init, then the ISR hot path is just
//! 1 register write + 2 register reads (~24 clock cycles at 170 MHz).

use embassy_stm32::pac;
use embassy_stm32::pac::cordic::vals;
use oxifoc_core::foc::trig::{SinCos, angle_to_cordic_q31, q31_to_f32};

/// CORDIC hardware-accelerated sin/cos
///
/// Zero-sized type — all state lives in the CORDIC peripheral registers.
/// Must call [`init()`](Self::init) once before first use.
pub struct CordicSinCos;

impl CordicSinCos {
    /// Initialize CORDIC peripheral for sin/cos computation.
    ///
    /// Call once during board init, before the FOC ISR starts.
    /// Configures: Cosine function, 6 iterations (≈20-bit precision),
    /// 1 argument input, 2 result outputs, q1.31 format.
    pub fn init() {
        // Enable CORDIC clock
        pac::RCC.ahb1enr().modify(|w| w.set_cordicen(true));

        // Configure CSR in one write
        pac::CORDIC.csr().write(|w| {
            w.set_func(vals::Func::from_bits(0)); // Cosine function
            w.set_precision(vals::Precision::from_bits(6)); // 6 iterations = 24 cycles ≈ 20-bit
            w.set_scale(vals::Scale::from_bits(0)); // No scaling
            w.set_nargs(vals::Num::NUM1); // 1 argument (angle only, modulus defaults to 1.0)
            w.set_nres(vals::Num::NUM2); // 2 results (cos + sin)
            w.set_argsize(vals::Size::BITS32); // q1.31 input
            w.set_ressize(vals::Size::BITS32); // q1.31 output
            w.set_ien(false); // No interrupt
            w.set_dmaren(false); // No DMA
            w.set_dmawen(false); // No DMA
        });
    }
}

impl SinCos for CordicSinCos {
    #[inline(always)]
    fn sin_cos(angle: f32) -> (f32, f32) {
        let angle_q31 = angle_to_cordic_q31(angle);

        // Write angle — CORDIC starts computing immediately
        pac::CORDIC.wdata().write_value(angle_q31 as u32);

        // Wait for result (typically ~24 cycles at 6 iterations)
        while !pac::CORDIC.csr().read().rrdy() {}

        // First read = cos (Cosine function primary result)
        let cos_q31 = pac::CORDIC.rdata().read() as i32;
        // Second read = sin (Cosine function secondary result)
        let sin_q31 = pac::CORDIC.rdata().read() as i32;

        (q31_to_f32(sin_q31), q31_to_f32(cos_q31))
    }
}

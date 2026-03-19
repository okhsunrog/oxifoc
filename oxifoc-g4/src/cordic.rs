//! CORDIC hardware-accelerated sin/cos for STM32G4
//!
//! Provides [`CordicSinCos`] implementing [`SinCos`] using the embassy CORDIC
//! driver for ~10x faster sin/cos than software `libm`.
//!
//! The CORDIC is configured once at init via the embassy driver, then the
//! hot path uses `blocking_calc_32bit` (~22 cycles at 170 MHz with O3+LTO).

use core::cell::RefCell;

use embassy_stm32::cordic::{self, Cordic};
use embassy_stm32::peripherals;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use oxifoc_core::foc::trig::{SinCos, angle_to_cordic_q31, q31_to_f32};

/// CORDIC driver instance, initialized once and accessed from the FOC ISR.
static CORDIC_INSTANCE: CriticalSectionMutex<RefCell<Option<Cordic<'static, peripherals::CORDIC>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// CORDIC hardware-accelerated sin/cos
///
/// Must call [`init()`](Self::init) once before first use.
/// Uses the embassy CORDIC driver with Cosine function (primary=cos, secondary=sin),
/// 1 argument input, 2 result outputs, q1.31 format.
pub struct CordicSinCos;

impl CordicSinCos {
    /// Initialize CORDIC peripheral for sin/cos computation.
    ///
    /// Call once during board init, before the FOC ISR starts.
    /// Configures: Cosine function, 24 iterations (≈20-bit precision),
    /// 1 argument input, 2 result outputs, q1.31 format.
    pub fn init(peri: Peri<'static, peripherals::CORDIC>) {
        let config = cordic::Config::new(
            cordic::Function::Cos,
            cordic::Precision::Iters24,
            Default::default(),
        )
        .unwrap()
        .res_count(cordic::AccessCount::Two);

        let cordic = Cordic::new(peri, config);
        CORDIC_INSTANCE.lock(|cell| cell.replace(Some(cordic)));
    }
}

impl SinCos for CordicSinCos {
    #[inline(always)]
    fn sin_cos(angle: f32) -> (f32, f32) {
        let angle_q31 = angle_to_cordic_q31(angle);
        let input = [angle_q31 as u32];
        let mut output = [0u32; 2];

        CORDIC_INSTANCE.lock(|cell| {
            let mut cordic = cell.borrow_mut();
            let cordic = cordic.as_mut().unwrap();
            cordic.blocking_calc_32bit(&input, &mut output).unwrap();
        });

        // Cosine function: primary result = cos, secondary = sin
        let cos_q31 = output[0] as i32;
        let sin_q31 = output[1] as i32;

        (q31_to_f32(sin_q31), q31_to_f32(cos_q31))
    }
}

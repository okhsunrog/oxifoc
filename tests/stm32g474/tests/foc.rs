#![no_std]
#![no_main]
#![allow(unused_imports)]

use defmt_rtt as _;

#[embedded_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI};

    use oxifoc_core::foc::{
        controller::FocController,
        pwm::SvpwmModulator,
        transforms,
        trig::{SinCos, angle_to_cordic_q31, f32_to_q31, q31_to_f32},
    };
    use oxifoc_core::foc::trig::CordicSinCos;

    const MAX_TRIG_ERR: f32 = 1e-4; // ~20-bit CORDIC precision
    const MAX_Q31_ERR: f32 = 1e-6; // q31 round-trip precision
    const SYSCLK_HZ: u32 = 170_000_000;

    struct TestState;

    /// Enable DWT cycle counter for benchmarking
    fn enable_dwt() {
        unsafe {
            // DCB DEMCR: set TRCENA bit to enable DWT
            let demcr = 0xE000_EDFC as *mut u32;
            core::ptr::write_volatile(demcr, core::ptr::read_volatile(demcr) | (1 << 24));
            // DWT CYCCNT: reset counter
            core::ptr::write_volatile(0xE000_1004 as *mut u32, 0);
            // DWT CTRL: set CYCCNTENA bit
            let ctrl = 0xE000_1000 as *mut u32;
            core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 1);
        }
    }

    fn dwt_cycles() -> u32 {
        unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
    }

    #[init]
    fn init() -> TestState {
        use embassy_stm32::rcc::*;
        use embassy_stm32::time::Hertz;

        let mut config = embassy_stm32::Config::default();
        // 24MHz HSE → PLL → 170MHz SYSCLK
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.boost = true;

        let dp = embassy_stm32::init(config);
        CordicSinCos::init(dp.CORDIC);
        enable_dwt();
        TestState
    }

    // ========== q31 fixed-point round-trip (inline ASM on ARM) ==========

    #[test]
    fn q31_round_trip(_state: TestState) {
        let values: [f32; 7] = [-0.999, -0.5, -0.25, 0.0, 0.25, 0.5, 0.999];
        for val in values {
            let q31 = f32_to_q31(val);
            let back = q31_to_f32(q31);
            let err = libm::fabsf(val - back);
            defmt::assert!(
                err < MAX_Q31_ERR,
                "q31 round-trip failed for {}: got {}, err {}",
                val,
                back,
                err
            );
        }
        defmt::info!("q31 round-trip: PASS");
    }

    // ========== CORDIC sin/cos accuracy at known angles ==========

    #[test]
    fn cordic_accuracy(_state: TestState) {
        // (angle, expected_sin, expected_cos)
        let cases: [(f32, f32, f32); 6] = [
            (0.0, 0.0, 1.0),
            (FRAC_PI_6, 0.5, 0.866_025_4),
            (FRAC_PI_4, core::f32::consts::FRAC_1_SQRT_2, core::f32::consts::FRAC_1_SQRT_2),
            (FRAC_PI_2, 1.0, 0.0),
            (PI, 0.0, -1.0),
            (-FRAC_PI_4, -core::f32::consts::FRAC_1_SQRT_2, core::f32::consts::FRAC_1_SQRT_2),
        ];

        for (angle, exp_sin, exp_cos) in cases {
            let (sin, cos) = CordicSinCos::sin_cos(angle);
            let sin_err = libm::fabsf(sin - exp_sin);
            let cos_err = libm::fabsf(cos - exp_cos);
            defmt::assert!(
                sin_err < MAX_TRIG_ERR,
                "sin({}) = {}, expected {}, err {}",
                angle,
                sin,
                exp_sin,
                sin_err
            );
            defmt::assert!(
                cos_err < MAX_TRIG_ERR,
                "cos({}) = {}, expected {}, err {}",
                angle,
                cos,
                exp_cos,
                cos_err
            );
        }
        defmt::info!("CORDIC accuracy: PASS");
    }

    // ========== Clarke → Park → inverse Park → inverse Clarke round-trip ==========

    #[test]
    fn transform_round_trip(_state: TestState) {
        let test_angles: [f32; 4] = [0.0, FRAC_PI_6, FRAC_PI_2, 2.5];
        let ia = 1.0_f32;
        let ib = -0.5_f32;
        let ic = -ia - ib; // balanced 3-phase

        for angle in test_angles {
            let (sin_t, cos_t) = CordicSinCos::sin_cos(angle);
            let (alpha, beta) = transforms::clarke(ia, ib);
            let (d, q) = transforms::park(alpha, beta, sin_t, cos_t);
            let (alpha2, beta2) = transforms::inverse_park(d, q, sin_t, cos_t);
            let (a2, b2, c2) = transforms::inverse_clarke(alpha2, beta2);

            defmt::assert!(
                libm::fabsf(a2 - ia) < 1e-3,
                "angle {}: ia round-trip {} vs {}",
                angle,
                a2,
                ia
            );
            defmt::assert!(
                libm::fabsf(b2 - ib) < 1e-3,
                "angle {}: ib round-trip {} vs {}",
                angle,
                b2,
                ib
            );
            defmt::assert!(
                libm::fabsf(c2 - ic) < 1e-3,
                "angle {}: ic round-trip {} vs {}",
                angle,
                c2,
                ic
            );
        }
        defmt::info!("Transform round-trip: PASS");
    }

    // ========== Full FocController with zero currents (no motor) ==========

    #[test]
    fn foc_zero_currents(_state: TestState) {
        let vbus = 12.0_f32;
        let max_duty: u16 = 1000;
        let dt = 50e-6_f32; // 20kHz

        let mut foc = FocController::<SvpwmModulator, CordicSinCos>::new(vbus);

        // With zero currents and zero targets, duties should be centered
        let angles: [f32; 4] = [0.0, FRAC_PI_4, PI, 5.0];
        for angle in angles {
            let telem = foc.step((0.0, 0.0, 0.0), angle, 0.0, 0.0, max_duty, dt);

            // PI controllers with zero error should output ~zero voltage → centered duties
            let center = max_duty / 2;
            let tolerance = max_duty / 10; // 10% tolerance for PI transient
            for (i, &duty) in telem.duties.iter().enumerate() {
                let diff = if duty > center {
                    duty - center
                } else {
                    center - duty
                };
                defmt::assert!(
                    diff <= tolerance,
                    "angle {}: duty[{}] = {}, expected ~{} (±{})",
                    angle,
                    i,
                    duty,
                    center,
                    tolerance
                );
            }
        }
        defmt::info!("FOC zero currents: PASS");
    }

    // ========== Full FocController with synthetic d-axis current ==========

    #[test]
    fn foc_synthetic_current(_state: TestState) {
        let vbus = 24.0_f32;
        let max_duty: u16 = 2000;
        let dt = 50e-6_f32;

        let mut foc = FocController::<SvpwmModulator, CordicSinCos>::new(vbus);

        // Simulate: angle = 0, inject known current on d-axis
        // At angle = 0: sin=0, cos=1, so clarke(ia, ib) with ia=1.0, ib=-0.5
        // maps to park(1.0, 0.0, 0.0, 1.0) = (d=1.0, q=0.0)
        // PI should react to push id towards target=0
        let angle = 0.0_f32;
        let ia = 1.0_f32;
        let ib = -0.5_f32;

        // Run multiple steps to let PI respond
        let mut last_telem = foc.step((ia, ib, -ia - ib), angle, 0.0, 0.0, max_duty, dt);
        for _ in 0..10 {
            last_telem = foc.step((ia, ib, -ia - ib), angle, 0.0, 0.0, max_duty, dt);
        }

        // With persistent d-axis current and id_target=0, vd should be negative
        // (PI tries to push id towards zero)
        defmt::assert!(
            last_telem.vd < 0.0,
            "Expected negative vd to counteract positive id, got vd={}",
            last_telem.vd
        );

        // Duties should be non-trivially different from center (PI is acting)
        let all_same = last_telem.duties[0] == last_telem.duties[1]
            && last_telem.duties[1] == last_telem.duties[2];
        defmt::assert!(
            !all_same,
            "Duties should differ when PI is active: {:?}",
            last_telem.duties
        );

        defmt::info!(
            "FOC synthetic current: PASS (vd={}, vq={}, duties={:?})",
            last_telem.vd,
            last_telem.vq,
            last_telem.duties
        );
    }

    // ========== SVPWM sector coverage ==========

    #[test]
    fn svpwm_all_sectors(_state: TestState) {
        use oxifoc_core::foc::svpwm::space_vector_pwm;

        let max_duty: u16 = 1000;
        let amplitude = 0.5_f32; // 50% modulation

        // Test 6 angles covering all SVPWM sectors (60° apart)
        for i in 0..6 {
            let angle = (i as f32) * PI / 3.0 + PI / 6.0; // 30°, 90°, 150°, ...
            let (sin, cos) = CordicSinCos::sin_cos(angle);
            let alpha = amplitude * cos;
            let beta = amplitude * sin;

            let duties = space_vector_pwm(alpha, beta, max_duty);

            // All duties should be within valid range
            for (j, &d) in duties.iter().enumerate() {
                defmt::assert!(
                    d <= max_duty,
                    "sector {}: duty[{}] = {} > max {}",
                    i + 1,
                    j,
                    d,
                    max_duty
                );
            }

            // At least one duty should be non-zero (we're not at zero voltage)
            let any_nonzero = duties[0] > 0 || duties[1] > 0 || duties[2] > 0;
            defmt::assert!(any_nonzero, "sector {}: all duties zero", i + 1);
        }
        defmt::info!("SVPWM all sectors: PASS");
    }

    // ========== Benchmark: full FOC step at 170MHz ==========

    #[test]
    fn bench_foc_step(_state: TestState) {
        let vbus = 24.0_f32;
        let max_duty: u16 = 2000;
        let dt = 50e-6_f32; // 20kHz

        let mut foc = FocController::<SvpwmModulator, CordicSinCos>::new(vbus);

        // Warm up: let PI settle
        for i in 0..50u32 {
            let angle = (i as f32) * 0.1;
            foc.step((1.0, -0.5, -0.5), angle, 0.5, 2.0, max_duty, dt);
        }

        // Benchmark N iterations of the full FOC step
        // (Clarke + CORDIC sin/cos + Park + 2×PI + inverse Park + SVPWM)
        const N: u32 = 1000;
        let mut min_cycles = u32::MAX;
        let mut max_cycles = 0u32;
        let mut total_cycles = 0u64;

        for i in 0..N {
            // Vary angle to sweep all SVPWM sectors; vary currents slightly
            let angle = (i as f32) * 0.00628; // full rotation over 1000 steps
            let ia = 1.5 + (i as f32) * 0.001;
            let ib = -0.75 - (i as f32) * 0.0005;
            let ic = -ia - ib;

            let start = dwt_cycles();
            let result = foc.step((ia, ib, ic), angle, 0.5, 2.0, max_duty, dt);
            let end = dwt_cycles();
            core::hint::black_box(&result);

            let elapsed = end.wrapping_sub(start);
            total_cycles += elapsed as u64;
            if elapsed < min_cycles {
                min_cycles = elapsed;
            }
            if elapsed > max_cycles {
                max_cycles = elapsed;
            }
        }

        let avg_cycles = (total_cycles / N as u64) as u32;
        let avg_ns = (avg_cycles as u64 * 1_000_000_000) / SYSCLK_HZ as u64;
        let min_ns = (min_cycles as u64 * 1_000_000_000) / SYSCLK_HZ as u64;
        let max_ns = (max_cycles as u64 * 1_000_000_000) / SYSCLK_HZ as u64;
        let budget_cycles = SYSCLK_HZ / 20_000; // 50µs at 170MHz = 8500 cycles
        let utilization = (avg_cycles as u64 * 100) / budget_cycles as u64;

        defmt::info!("=== FOC step benchmark ({} iterations @ {}MHz) ===", N, SYSCLK_HZ / 1_000_000);
        defmt::info!("  avg: {} cycles ({} ns)", avg_cycles, avg_ns);
        defmt::info!("  min: {} cycles ({} ns)", min_cycles, min_ns);
        defmt::info!("  max: {} cycles ({} ns)", max_cycles, max_ns);
        defmt::info!("  50us budget: {} cycles, utilization: {}%", budget_cycles, utilization);
    }
}

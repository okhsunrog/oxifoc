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
        trig::{FastSinCos, LibmSinCos, SinCos},
    };

    const MAX_TRIG_ERR: f32 = 1e-5;
    const SYSCLK_HZ: u32 = 168_000_000;

    struct TestState;

    fn enable_dwt() {
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            core::ptr::write_volatile(demcr, core::ptr::read_volatile(demcr) | (1 << 24));
            core::ptr::write_volatile(0xE000_1004 as *mut u32, 0);
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
        // 8MHz HSE crystal → PLL → 168MHz SYSCLK (max for STM32F405)
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;

        let _dp = embassy_stm32::init(config);
        enable_dwt();
        TestState
    }

    // ========== libm sin/cos accuracy ==========

    #[test]
    fn libm_sincos_accuracy(_state: TestState) {
        let cases: [(f32, f32, f32); 6] = [
            (0.0, 0.0, 1.0),
            (FRAC_PI_6, 0.5, 0.866_025_4),
            (
                FRAC_PI_4,
                core::f32::consts::FRAC_1_SQRT_2,
                core::f32::consts::FRAC_1_SQRT_2,
            ),
            (FRAC_PI_2, 1.0, 0.0),
            (PI, 0.0, -1.0),
            (
                -FRAC_PI_4,
                -core::f32::consts::FRAC_1_SQRT_2,
                core::f32::consts::FRAC_1_SQRT_2,
            ),
        ];

        for (angle, exp_sin, exp_cos) in cases {
            let (sin, cos) = LibmSinCos::sin_cos(angle);
            defmt::assert!(
                libm::fabsf(sin - exp_sin) < MAX_TRIG_ERR,
                "sin({}) err",
                angle
            );
            defmt::assert!(
                libm::fabsf(cos - exp_cos) < MAX_TRIG_ERR,
                "cos({}) err",
                angle
            );
        }
        defmt::info!("libm sin/cos accuracy: PASS");
    }

    // ========== Clarke/Park round-trip ==========

    #[test]
    fn transform_round_trip(_state: TestState) {
        let test_angles: [f32; 4] = [0.0, FRAC_PI_6, FRAC_PI_2, 2.5];
        let ia = 1.0_f32;
        let ib = -0.5_f32;
        let ic = -ia - ib;

        for angle in test_angles {
            let (sin_t, cos_t) = LibmSinCos::sin_cos(angle);
            let (alpha, beta) = transforms::clarke(ia, ib);
            let (d, q) = transforms::park(alpha, beta, sin_t, cos_t);
            let (alpha2, beta2) = transforms::inverse_park(d, q, sin_t, cos_t);
            let (a2, b2, c2) = transforms::inverse_clarke(alpha2, beta2);

            defmt::assert!(libm::fabsf(a2 - ia) < 1e-3, "ia round-trip at {}", angle);
            defmt::assert!(libm::fabsf(b2 - ib) < 1e-3, "ib round-trip at {}", angle);
            defmt::assert!(libm::fabsf(c2 - ic) < 1e-3, "ic round-trip at {}", angle);
        }
        defmt::info!("Transform round-trip: PASS");
    }

    // ========== FOC zero currents ==========

    #[test]
    fn foc_zero_currents(_state: TestState) {
        let mut foc = FocController::<SvpwmModulator, FastSinCos>::new(12.0);
        let max_duty: u16 = 1000;
        let dt = 50e-6_f32;

        for &angle in &[0.0_f32, FRAC_PI_4, PI, 5.0] {
            let telem = foc.step((0.0, 0.0, 0.0), angle, 0.0, 0.0, max_duty, dt);
            let center = max_duty / 2;
            let tolerance = max_duty / 10;
            for (i, &duty) in telem.duties.iter().enumerate() {
                let diff = if duty > center {
                    duty - center
                } else {
                    center - duty
                };
                defmt::assert!(diff <= tolerance, "angle {}: duty[{}] = {}", angle, i, duty);
            }
        }
        defmt::info!("FOC zero currents: PASS");
    }

    // ========== FOC synthetic current ==========

    #[test]
    fn foc_synthetic_current(_state: TestState) {
        let mut foc = FocController::<SvpwmModulator, FastSinCos>::new(24.0);
        let max_duty: u16 = 2000;
        let dt = 50e-6_f32;

        let mut last = foc.step((1.0, -0.5, -0.5), 0.0, 0.0, 0.0, max_duty, dt);
        for _ in 0..10 {
            last = foc.step((1.0, -0.5, -0.5), 0.0, 0.0, 0.0, max_duty, dt);
        }

        defmt::assert!(last.vd < 0.0, "Expected negative vd, got {}", last.vd);
        let all_same = last.duties[0] == last.duties[1] && last.duties[1] == last.duties[2];
        defmt::assert!(!all_same, "Duties should differ: {:?}", last.duties);
        defmt::info!(
            "FOC synthetic current: PASS (vd={}, duties={:?})",
            last.vd,
            last.duties
        );
    }

    // ========== SVPWM sector coverage ==========

    #[test]
    fn svpwm_all_sectors(_state: TestState) {
        use oxifoc_core::foc::svpwm::space_vector_pwm;
        let max_duty: u16 = 1000;

        for i in 0..6 {
            let angle = (i as f32) * PI / 3.0 + PI / 6.0;
            let (sin, cos) = LibmSinCos::sin_cos(angle);
            let duties = space_vector_pwm(0.5 * cos, 0.5 * sin, max_duty);
            for (j, &d) in duties.iter().enumerate() {
                defmt::assert!(d <= max_duty, "sector {}: duty[{}] = {}", i + 1, j, d);
            }
            defmt::assert!(
                duties[0] > 0 || duties[1] > 0 || duties[2] > 0,
                "sector {}: all zero",
                i + 1
            );
        }
        defmt::info!("SVPWM all sectors: PASS");
    }

    // ========== Benchmark: full FOC step at 168MHz ==========

    fn bench_foc<S: SinCos>(label: &str) {
        let mut foc = FocController::<SvpwmModulator, S>::new(24.0);
        let max_duty: u16 = 2000;
        let dt = 50e-6_f32;

        for i in 0..20u32 {
            foc.step((1.0, -0.5, -0.5), (i as f32) * 0.1, 0.5, 2.0, max_duty, dt);
        }

        const N: u32 = 1000;
        let mut min_cycles = u32::MAX;
        let mut max_cycles = 0u32;
        let mut total_cycles = 0u64;

        for i in 0..N {
            let angle = (i as f32) * 0.00628;
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
        let budget_cycles = SYSCLK_HZ / 20_000;
        let utilization = (avg_cycles as u64 * 100) / budget_cycles as u64;

        defmt::info!(
            "=== FOC {} ({} iters @ {}MHz) ===",
            label,
            N,
            SYSCLK_HZ / 1_000_000
        );
        defmt::info!("  avg: {} cycles ({} ns)", avg_cycles, avg_ns);
        defmt::info!("  min: {} cycles ({} ns)", min_cycles, min_ns);
        defmt::info!("  max: {} cycles ({} ns)", max_cycles, max_ns);
        defmt::info!(
            "  50us budget: {} cycles, utilization: {}%",
            budget_cycles,
            utilization
        );
    }

    #[test]
    fn z_bench_fast_sincos(_state: TestState) {
        bench_foc::<FastSinCos>("FastSinCos");
    }

    #[test]
    fn z_bench_libm_sincos(_state: TestState) {
        bench_foc::<LibmSinCos>("LibmSinCos");
    }
}

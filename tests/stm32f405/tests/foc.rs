#![no_std]
#![no_main]
#![allow(unused_imports)]

use defmt_rtt as _;

#[embedded_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_6, PI};
    use core::mem::ManuallyDrop;

    use oxifoc_core::foc::{
        controller::FocController,
        pwm::SvpwmModulator,
        transforms,
        trig::{FastSinCos, LibmSinCos, SinCos},
    };

    const MAX_TRIG_ERR: f32 = 1e-5;
    const SYSCLK_HZ: u32 = 168_000_000;

    struct TestState {
        // Keep the MK5 powered and its driver off until reset, including the
        // harness epilogue: dropping Output would disconnect these pins.
        _shutdown_latch: ManuallyDrop<embassy_stm32::gpio::Output<'static>>,
        _gate_disable: ManuallyDrop<embassy_stm32::gpio::Output<'static>>,
    }

    fn enable_dwt() {
        // SAFETY: these are Cortex-M4 debug registers; init runs once per test.
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            core::ptr::write_volatile(demcr, core::ptr::read_volatile(demcr) | (1 << 24));
            core::ptr::write_volatile(0xE000_1004 as *mut u32, 0);
            let ctrl = 0xE000_1000 as *mut u32;
            core::ptr::write_volatile(ctrl, core::ptr::read_volatile(ctrl) | 1);
        }
    }

    fn dwt_cycles() -> u32 {
        // SAFETY: DWT CYCCNT is a readable Cortex-M4 register, enabled in init.
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
        config.rcc.pll_src = PllSource::Hse;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::Div4,
            mul: PllMul::Mul168,
            divp: Some(PllPDiv::Div2),
            divq: Some(PllQDiv::Div7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::Div1;
        config.rcc.apb1_pre = APBPrescaler::Div4;
        config.rcc.apb2_pre = APBPrescaler::Div2;
        config.rcc.sys = Sysclk::Pll1P;

        let dp = embassy_stm32::init(config);
        use embassy_stm32::gpio::{Level, Output, Speed};
        let state = TestState {
            _shutdown_latch: ManuallyDrop::new(Output::new(dp.PC5, Level::High, Speed::Low)),
            _gate_disable: ManuallyDrop::new(Output::new(dp.PB5, Level::Low, Speed::Low)),
        };
        // Match shipping F405 flash acceleration after RCC sets wait states.
        // TIM1 and all six phase PWM pins remain at their reset defaults.
        embassy_stm32::pac::FLASH.acr().modify(|w| {
            w.set_prften(true);
            w.set_icen(true);
            w.set_dcen(true);
        });
        enable_dwt();
        state
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

    // ========== Benchmark: old/shipping angle wrapping, DWT cycles only ==========

    #[inline(never)]
    fn previous_wrap_angle(angle: f32) -> f32 {
        use core::f32::consts::TAU;
        let mut a = angle;
        if !(-TAU..=TAU).contains(&a) {
            a %= TAU;
        }
        if a < 0.0 {
            a += TAU;
        } else if a >= TAU {
            a -= TAU;
        }
        a
    }

    #[inline(never)]
    fn shipping_wrap_angle(angle: f32) -> f32 {
        oxifoc_core::foc::wrap_angle(angle)
    }

    #[cold]
    #[inline(never)]
    fn wrap_remainder(angle: f32) -> f32 {
        angle % core::f32::consts::TAU
    }

    #[inline(never)]
    fn outlined_wrap_angle(angle: f32) -> f32 {
        use core::f32::consts::TAU;
        let mut a = angle;
        if !(-TAU..2.0 * TAU).contains(&a) {
            a = wrap_remainder(a);
        }
        if a < 0.0 {
            a += TAU;
        } else if a >= TAU {
            a -= TAU;
        }
        a
    }

    #[inline(never)]
    fn wrap_identity(angle: f32) -> f32 {
        angle
    }

    const WRAP_BATCH_CALLS: u32 = 64;
    const WRAP_ROUNDS: u32 = 24;

    #[inline(never)]
    fn measure_wrap_batch(function: fn(f32) -> f32, inputs: &[f32; 8]) -> u32 {
        let function = core::hint::black_box(function);
        // Each bounded window covers 64 calls, with no logging or awaiting.
        // Masking IRQs measures helper cost independently of the time driver.
        cortex_m::interrupt::free(|_| {
            let start = dwt_cycles();
            for _ in 0..WRAP_BATCH_CALLS / 8 {
                for &angle in inputs {
                    core::hint::black_box(function(core::hint::black_box(angle)));
                }
            }
            dwt_cycles().wrapping_sub(start)
        })
    }

    #[derive(Clone, Copy)]
    struct WrapCycles {
        min: u32,
        max: u32,
        total: u64,
    }

    impl WrapCycles {
        const fn new() -> Self {
            Self {
                min: u32::MAX,
                max: 0,
                total: 0,
            }
        }

        fn record(&mut self, cycles: u32) {
            // Far below the controller watchdog's three-second timeout;
            // no watchdog is started by this test image.
            defmt::assert!(cycles > 0 && cycles < 1_000_000, "unexpected batch length");
            self.min = self.min.min(cycles);
            self.max = self.max.max(cycles);
            self.total += u64::from(cycles);
        }

        fn report(&self, label: &str) {
            let mean = self.total * 1_000 / u64::from(WRAP_ROUNDS * WRAP_BATCH_CALLS);
            defmt::info!(
                "{}: mean={} milli-cycles/call, batch min={} max={} cycles",
                label,
                mean,
                self.min,
                self.max
            );
        }
    }

    #[test]
    fn z_bench_wrap_angle(_state: TestState) {
        use core::f32::consts::TAU;
        // Opaque pointers keep one shared indirect-call loop for all variants.
        let functions = core::hint::black_box([
            wrap_identity as fn(f32) -> f32,
            previous_wrap_angle,
            shipping_wrap_angle,
            outlined_wrap_angle,
        ]);
        // Validate the experimental fallback at the semantic boundaries too.
        for angle in [
            -0.0,
            0.0,
            -f32::from_bits(1),
            -TAU,
            TAU,
            2.0 * TAU,
            f32::NEG_INFINITY,
            f32::INFINITY,
            f32::NAN,
        ] {
            let expected = functions[1](core::hint::black_box(angle)).to_bits();
            for function in &functions[2..] {
                defmt::assert_eq!(function(core::hint::black_box(angle)).to_bits(), expected);
            }
        }
        let cases = [
            (
                "ordinary positive",
                [0.0, 0.001, 0.1, 1.0, PI, 5.0, TAU - 0.1, TAU - 1e-6],
            ),
            (
                "positive crossing",
                [
                    TAU + 1e-6,
                    TAU + 0.001,
                    TAU + 0.1,
                    TAU + 1.0,
                    1.5 * TAU,
                    TAU + 5.0,
                    2.0 * TAU - 0.1,
                    2.0 * TAU - 1e-6,
                ],
            ),
            (
                "negative crossing",
                [-1e-6, -0.001, -0.1, -1.0, -PI, -5.0, -TAU + 1e-6, -TAU],
            ),
            (
                "cold large",
                [
                    2.0 * TAU,
                    -2.0 * TAU,
                    100.0 * TAU,
                    -100.0 * TAU,
                    1e6,
                    -1e6,
                    f32::MAX,
                    -f32::MAX,
                ],
            ),
        ];
        defmt::info!(
            "wrap benchmark: ART on, {} calls/batch, {} rounds; includes equal call/loop overhead",
            WRAP_BATCH_CALLS,
            WRAP_ROUNDS
        );
        for (label, inputs) in cases {
            let inputs = core::hint::black_box(inputs);
            for &angle in &inputs {
                let expected = functions[1](core::hint::black_box(angle)).to_bits();
                for function in &functions[2..] {
                    defmt::assert_eq!(function(core::hint::black_box(angle)).to_bits(), expected);
                }
            }
            // Warm each code/data path before collecting any samples.
            for function in functions {
                for _ in 0..4 {
                    core::hint::black_box(measure_wrap_batch(function, &inputs));
                }
            }
            let mut stats = [WrapCycles::new(); 4];
            for round in 0..WRAP_ROUNDS {
                // Mixed-radix selection visits all 24 permutations once,
                // balancing positions and every preceding/following pair.
                let mut available = [0, 1, 2, 3];
                let mut permutation = round as usize;
                for remaining in (1..=available.len()).rev() {
                    let selected = permutation % remaining;
                    permutation /= remaining;
                    let index = available[selected];
                    available[selected] = available[remaining - 1];
                    stats[index].record(measure_wrap_batch(functions[index], &inputs));
                }
            }
            defmt::info!("wrap input: {}", label);
            stats[0].report("identity");
            stats[1].report("old");
            stats[2].report("shipping");
            stats[3].report("outlined cold");
            let delta = (stats[1].total as i64 - stats[2].total as i64) * 1_000
                / i64::from(WRAP_ROUNDS * WRAP_BATCH_CALLS);
            defmt::info!("old minus shipping: {} milli-cycles/call", delta);
            let delta = (stats[1].total as i64 - stats[3].total as i64) * 1_000
                / i64::from(WRAP_ROUNDS * WRAP_BATCH_CALLS);
            defmt::info!("old minus outlined cold: {} milli-cycles/call", delta);
        }
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

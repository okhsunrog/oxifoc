//! Peripheral initialization for STM32F405

use embassy_stm32::adc::{Adc, AdcChannel, Exten, InjectedAdc, InjectedAdcTrigger, SampleTime};
use embassy_stm32::{
    Config as StmConfig, Peri, Peripherals,
    gpio::{Level, Output, Speed},
    mode::Blocking,
    peripherals,
    time::Hertz,
};

/// Initialize system clock for STM32F405 with 8MHz HSE
/// Configured for Simple FOCer 2 / VESC layouts with external crystal
pub fn init_clock() -> Peripherals {
    defmt::info!("oxifoc-f405 boot");

    let mut config = StmConfig::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::Hse;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::Div4,
            mul: PllMul::Mul168,
            divp: Some(PllPDiv::Div2), // 8 MHz / 4 * 168 / 2 = 168 MHz system (= config::CPU_HZ)
            divq: Some(PllQDiv::Div7), // 8 MHz / 4 * 168 / 7 = 48 MHz for USB FS
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::Div1;
        config.rcc.apb1_pre = APBPrescaler::Div4;
        config.rcc.apb2_pre = APBPrescaler::Div2;
        config.rcc.sys = Sysclk::Pll1P;
        config.rcc.mux.clk48sel = mux::Clk48sel::Pll1Q;
    }

    let p = embassy_stm32::init(config);

    // ART accelerator: embassy's F4 RCC init WRITES flash ACR with only the
    // latency field (rcc/f247.rs), and the F405 reset value is all-zero —
    // so prefetch, I-cache and D-cache are all OFF at 5 wait states /
    // 168 MHz unless we turn them on here. Same trap class as the G4
    // PRFTEN find (2026-07-07, docs/decisions.md): without these bits the
    // branchy FOC hot path pays a multi-x CPI penalty on every fetch.
    // Bench A/B 2026-07-07 (Stopped state, 20 kHz ISR): ART off = 2868 cy
    // avg / 34% load; ART on = 2236 cy / 26%. The gap widens on the
    // branchier drive path — never remove these bits without re-measuring.
    embassy_stm32::pac::FLASH.acr().modify(|w| {
        w.set_prften(true);
        w.set_icen(true);
        w.set_dcen(true);
    });

    p
}

/// Initialize green LED on PB0 (heartbeat)
pub fn init_green_led(pb0: Peri<'static, peripherals::PB0>) -> Output<'static> {
    Output::new(pb0, Level::High, Speed::Low)
}

/// Initialize red LED on PB1 (fault indicator)
pub fn init_red_led(pb1: Peri<'static, peripherals::PB1>) -> Output<'static> {
    Output::new(pb1, Level::High, Speed::Low)
}

/// ADC handles for injected conversions
pub struct AdcHandles {
    pub adc1: InjectedAdc<'static, embassy_stm32::pac::adc::Adc, Blocking>,
    pub adc2: InjectedAdc<'static, embassy_stm32::pac::adc::Adc, Blocking>,
    pub adc3: InjectedAdc<'static, embassy_stm32::pac::adc::Adc, Blocking>,
}

/// Initialize ADC1/ADC2/ADC3 with injected conversions triggered by TIM1_CC4
///
/// # ADC Configuration
///
/// All ADC sampling via TIM1-triggered injected conversions (no DMA).
/// - ADC1 injected: Phase A current (PC0, ch10) + Board temp (PA3, ch3)
/// - ADC2 injected: Phase B current (PC1, ch11) + Motor temp (PC4, ch14)
/// - ADC3 injected: Phase C current (PC2, ch12) + VBUS (PC3, ch13)
///
/// All triggered simultaneously by TIM1_CC4 rising edge.
/// ADC3 generates JEOC interrupt (signals all ADCs done).
#[allow(clippy::too_many_arguments)]
pub fn init_adc(
    adc1_peri: Peri<'static, peripherals::ADC1>,
    adc2_peri: Peri<'static, peripherals::ADC2>,
    adc3_peri: Peri<'static, peripherals::ADC3>,
    ia_pin: Peri<'static, peripherals::PC0>,
    board_temp_pin: Peri<'static, peripherals::PA3>,
    ib_pin: Peri<'static, peripherals::PC1>,
    motor_temp_pin: Peri<'static, peripherals::PC4>,
    ic_pin: Peri<'static, peripherals::PC2>,
    vbus_pin: Peri<'static, peripherals::PC3>,
) -> AdcHandles {
    let adc1 = Adc::new(adc1_peri);
    let adc2 = Adc::new(adc2_peri);
    let adc3 = Adc::new(adc3_peri);

    // ADC1 injected: phase A current + board temperature
    let injected_adc1 = adc1.setup_injected_conversions(
        (),
        [
            (ia_pin.degrade_adc(), SampleTime::Cycles15), // Phase A current (PC0, ch10)
            (board_temp_pin.degrade_adc(), SampleTime::Cycles15), // Board temp (PA3, ch3)
        ],
        InjectedAdcTrigger::from(embassy_stm32::triggers::TIM1_CH4, Exten::RisingEdge),
        Blocking,
    );

    // ADC2 injected: phase B current + motor temperature
    let injected_adc2 = adc2.setup_injected_conversions(
        (),
        [
            (ib_pin.degrade_adc(), SampleTime::Cycles15), // Phase B current (PC1, ch11)
            (motor_temp_pin.degrade_adc(), SampleTime::Cycles15), // Motor temp (PC4, ch14)
        ],
        InjectedAdcTrigger::from(embassy_stm32::triggers::TIM1_CH4, Exten::RisingEdge),
        Blocking,
    );

    // ADC3 injected: phase C current + VBUS (interrupt on ADC3 — signals all done)
    let injected_adc3 = adc3.setup_injected_conversions(
        (),
        [
            (ic_pin.degrade_adc(), SampleTime::Cycles15), // Phase C current (PC2, ch12)
            (vbus_pin.degrade_adc(), SampleTime::Cycles15), // VBUS (PC3, ch13)
        ],
        InjectedAdcTrigger::from(embassy_stm32::triggers::TIM1_CH4, Exten::RisingEdge),
        Blocking,
    );
    embassy_stm32::pac::ADC3
        .cr1()
        .modify(|w| w.set_jeocie(true));

    // NOTE: ADC interrupt is NOT enabled here.
    // foc::init() enables it after installing ADC handles into statics.

    defmt::info!("ADC1/ADC2/ADC3 initialized with TIM1_CC4-triggered injected conversions");

    AdcHandles {
        adc1: injected_adc1,
        adc2: injected_adc2,
        adc3: injected_adc3,
    }
}

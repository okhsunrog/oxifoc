//! Peripheral initialization for STM32F405

use embassy_stm32::adc::{Adc, AdcChannel, Exten, InjectedAdc, SampleTime};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{
    Config as StmConfig, Peri, Peripherals, interrupt, peripherals,
    gpio::{Level, Output, Speed},
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
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8 MHz / 4 * 168 / 2 = 168 MHz system
            divq: Some(PllQDiv::DIV7), // 8 MHz / 4 * 168 / 7 = 48 MHz for USB FS
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }

    embassy_stm32::init(config)
}

/// Initialize heartbeat LED on PC13
pub fn init_led(
    pc13: Peri<'static, peripherals::PC13>,
) -> Output<'static> {
    Output::new(pc13, Level::High, Speed::Low)
}

/// ADC handles for injected conversions
pub struct AdcHandles {
    pub adc1: InjectedAdc<'static, peripherals::ADC1, 2>,
    pub adc2: InjectedAdc<'static, peripherals::ADC2, 2>,
    pub adc3: InjectedAdc<'static, peripherals::ADC3, 2>,
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
        [
            (ia_pin.degrade_adc(), SampleTime::CYCLES15),         // Phase A current (PC0, ch10)
            (board_temp_pin.degrade_adc(), SampleTime::CYCLES15), // Board temp (PA3, ch3)
        ],
        embassy_stm32::triggers::TIM1_CH4,
        Exten::RISING_EDGE,
        false, // No interrupt on ADC1
    );

    // ADC2 injected: phase B current + motor temperature
    let injected_adc2 = adc2.setup_injected_conversions(
        [
            (ib_pin.degrade_adc(), SampleTime::CYCLES15),          // Phase B current (PC1, ch11)
            (motor_temp_pin.degrade_adc(), SampleTime::CYCLES15),  // Motor temp (PC4, ch14)
        ],
        embassy_stm32::triggers::TIM1_CH4,
        Exten::RISING_EDGE,
        false, // No interrupt on ADC2
    );

    // ADC3 injected: phase C current + VBUS (interrupt on ADC3 — signals all done)
    let injected_adc3 = adc3.setup_injected_conversions(
        [
            (ic_pin.degrade_adc(), SampleTime::CYCLES15),   // Phase C current (PC2, ch12)
            (vbus_pin.degrade_adc(), SampleTime::CYCLES15), // VBUS (PC3, ch13)
        ],
        embassy_stm32::triggers::TIM1_CH4,
        Exten::RISING_EDGE,
        true, // Interrupt on ADC3 (all ADCs finish simultaneously)
    );

    // Enable ADC interrupt
    unsafe {
        interrupt::typelevel::ADC::unpend();
        interrupt::typelevel::ADC::enable();
    }

    defmt::info!("ADC1/ADC2/ADC3 initialized with TIM1_CC4-triggered injected conversions");

    AdcHandles {
        adc1: injected_adc1,
        adc2: injected_adc2,
        adc3: injected_adc3,
    }
}

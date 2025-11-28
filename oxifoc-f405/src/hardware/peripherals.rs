//! Peripheral initialization for STM32F405

use embassy_stm32::{
    Config as StmConfig, Peripherals,
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
    pc13: embassy_stm32::Peri<'static, embassy_stm32::peripherals::PC13>,
) -> Output<'static> {
    Output::new(pc13, Level::High, Speed::Low)
}

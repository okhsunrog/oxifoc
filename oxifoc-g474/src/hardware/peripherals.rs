//! Hardware peripheral initialization for NUCLEO-G474RE

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{Peri, Peripherals, peripherals};

// ADC initialization for the IHM08M1 shield is written when the motor
// stack is enabled — see the plan at the bottom of this file.

/// Initialize STM32G474 clocks and return peripherals
///
/// Clock configuration for NUCLEO-G474RE:
/// - HSE: 24MHz crystal oscillator
/// - LSE: 32.768kHz crystal oscillator (for RTC/accurate timekeeping)
/// - SYSCLK: 170MHz via PLL (24MHz / 6 * 85 / 2 = 170MHz)
/// - Boost mode enabled for >150MHz operation
pub fn init_clock() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        use embassy_stm32::time::Hertz;

        // Configure LSE: 32.768kHz crystal oscillator
        config.rcc.ls = LsConfig {
            rtc: RtcClockSource::Lse,
            lsi: false,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::MediumHigh),
            }),
        };

        // Configure HSE: 24MHz crystal oscillator
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        });

        // Configure PLL for 170MHz SYSCLK
        // VCO in: 24MHz / 6 = 4MHz
        // VCO: 4MHz * 85 = 340MHz
        // SYSCLK: 340MHz / 2 = 170MHz
        config.rcc.pll = Some(Pll {
            source: PllSource::Hse,
            prediv: PllPreDiv::Div6,
            mul: PllMul::Mul85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::Div2),
        });

        config.rcc.sys = Sysclk::Pll1R;

        // Above 150MHz, enable Range1 boost mode per RM0440 guidance
        config.rcc.boost = true;

        // ADC clock source: use SYSCLK (170MHz) - will be needed for motor control
        config.rcc.mux.adc12sel = mux::Adcsel::Sys;

        // Note: LPUART1 clock is configured automatically by Embassy based on the
        // peripheral clock. For 115200 baud, the default PCLK is sufficient.

        // USB 48MHz clock: enable HSI48 with CRS synchronization from USB SOF frames
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        config.rcc.mux.clk48sel = mux::Clk48sel::Hsi48;
    }
    embassy_stm32::init(config)
}

// ============================================================================
// ADC plan for the IHM08M1 shield (write when the motor stack is enabled)
// ============================================================================
//
// The shield conditions all analog signals with its own TSV994 op-amps —
// the MCU internal OPAMPs are NOT used on this board (unlike B-G431B-ESC1,
// where raw shunt voltages reach the MCU). A previous internal-OPAMP/PGA
// plan lived here and was removed: its pins (PA1/PA7/PB0) actually carry
// VBUS / UL / VL on this shield.
//
// Injected sequences, TIM1_TRGO2-triggered (Mms2::COMPARE_OC4, like g431),
// pins per docs/hw/nucleo-g474re-ihm08m1.md:
//   ADC1: ia   = PA0 (ADC12_IN1),
//         vbus = PA1 (ADC12_IN2),
//         temp = PC2 (ADC12_IN8)
//   ADC2: ib   = PC1 (ADC12_IN7),
//         ic   = PC0 (ADC12_IN6)
// ADC1 finishes last (3 conversions) → ADC1_2 interrupt runs the FOC loop.

// ============================================================================
// NUCLEO-G474RE on-board peripherals
// ============================================================================

/// Initialize user LED on PA5 (NUCLEO-G474RE LD2 - active high)
pub fn init_led(pa5: Peri<'static, peripherals::PA5>) -> Output<'static> {
    Output::new(pa5, Level::Low, Speed::Low)
}

// User button B1 is on PC13 (directly connected to GND when pressed, active low)
// To use it, configure as Input with internal pull-up:
//
// use embassy_stm32::gpio::{Input, Pull};
//
// pub fn init_button(pc13: Peri<'static, peripherals::PC13>) -> Input<'static> {
//     Input::new(pc13, Pull::Up)
// }
//
// Usage: button.is_low() returns true when pressed

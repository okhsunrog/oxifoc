//! Hardware peripheral initialization for NUCLEO-G474RE

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{peripherals, Peri, Peripherals};

// Motor-related imports (commented out until IHM08M1 shield is connected)
// use embassy_stm32::adc::{
//     Adc, AdcChannel, AdcConfig, ConversionTrigger, Exten, InjectedAdc, SampleTime,
// };
// use embassy_stm32::interrupt::typelevel::{ADC1_2, Interrupt};
// use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};

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
            rtc: RtcClockSource::LSE,
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
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV6,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });

        config.rcc.sys = Sysclk::PLL1_R;

        // Above 150MHz, enable Range1 boost mode per RM0440 guidance
        config.rcc.boost = true;

        // ADC clock source: use SYSCLK (170MHz) - will be needed for motor control
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;

        // Note: LPUART1 clock is configured automatically by Embassy based on the
        // peripheral clock. For 115200 baud, the default PCLK is sufficient.
    }
    embassy_stm32::init(config)
}

// ============================================================================
// Motor-related peripheral initialization (commented out until IHM08M1 connected)
// ============================================================================

// /// OPAMP channels returned from initialization
// pub struct OpAmpChannels {
//     pub ia_chan: embassy_stm32::adc::AnyAdcChannel<peripherals::ADC1>,
//     pub ib_chan: embassy_stm32::adc::AnyAdcChannel<peripherals::ADC2>,
//     pub ic_chan: embassy_stm32::adc::AnyAdcChannel<peripherals::ADC2>,
// }

// /// Initialize OPAMPs as PGAs for phase current shunts
// ///
// /// Pin assignments will need to be updated for IHM08M1 shield:
// /// OPAMP1: phase A current -> ADC1
// /// OPAMP2: phase B current -> ADC2
// /// OPAMP3: phase C current -> ADC2
// ///
// /// All configured for 16x gain with high-speed mode and calibrated.
// pub fn init_opamps(
//     opamp1: Peri<'static, peripherals::OPAMP1>,
//     opamp2: Peri<'static, peripherals::OPAMP2>,
//     opamp3: Peri<'static, peripherals::OPAMP3>,
//     pa1: Peri<'static, peripherals::PA1>,
//     pa7: Peri<'static, peripherals::PA7>,
//     pb0: Peri<'static, peripherals::PB0>,
// ) -> OpAmpChannels {
//     let mut opamp1 = OpAmp::new(opamp1, OpAmpSpeed::HighSpeed);
//     opamp1.calibrate();
//     let ia_chan = opamp1.pga_int(pa1, OpAmpGain::Mul16).degrade_adc();
//
//     let mut opamp2 = OpAmp::new(opamp2, OpAmpSpeed::HighSpeed);
//     opamp2.calibrate();
//     let ib_chan = opamp2.pga_int(pa7, OpAmpGain::Mul16).degrade_adc();
//
//     let mut opamp3 = OpAmp::new(opamp3, OpAmpSpeed::HighSpeed);
//     opamp3.calibrate();
//     let ic_chan = opamp3.pga_int(pb0, OpAmpGain::Mul16).degrade_adc();
//
//     OpAmpChannels {
//         ia_chan,
//         ib_chan,
//         ic_chan,
//     }
// }

// /// ADC handles for injected conversions
// pub struct AdcHandles {
//     pub adc1: InjectedAdc<peripherals::ADC1, 3>,
//     pub adc2: InjectedAdc<peripherals::ADC2, 2>,
// }

// /// Initialize ADC1 and ADC2 with injected conversions triggered by TIM1
// ///
// /// Pin assignments will need to be updated for IHM08M1 shield.
// /// ADC configuration will be similar to G431 but with different pin mappings.
// pub fn init_adc(
//     adc1_periph: Peri<'static, peripherals::ADC1>,
//     adc2_periph: Peri<'static, peripherals::ADC2>,
//     opamp_channels: OpAmpChannels,
//     vbus_pin: Peri<'static, peripherals::PA0>,
//     temp_pin: Peri<'static, peripherals::PB14>,
// ) -> AdcHandles {
//     // ... ADC initialization code will go here when IHM08M1 is connected
//     todo!("ADC initialization for IHM08M1")
// }

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

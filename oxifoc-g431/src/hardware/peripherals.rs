//! Hardware peripheral initialization for B-G431B-ESC1

use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, ConversionTrigger, Exten, InjectedAdc, SampleTime,
};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt::typelevel::{ADC1_2, Interrupt};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::{Peri, Peripherals, peripherals};
use static_cell::StaticCell;

/// Initialize STM32G431 clocks and return peripherals
///
/// Configures HSE=8MHz feeding PLL to 170MHz SYSCLK with boost mode enabled.
pub fn init_clock() -> Peripherals {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        use embassy_stm32::time::Hertz;
        // Use external 8MHz HSE oscillator as PLL source
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        // VCO in: 8MHz / 2 = 4MHz; VCO: 4MHz * 85 = 340MHz; SYSCLK: 340MHz / 2 = 170MHz
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        // Above 150MHz, enable Range1 boost mode per RM0440 guidance
        config.rcc.boost = true;
        // ADC clock source: use SYSCLK (170MHz)
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    }
    embassy_stm32::init(config)
}

/// OPAMP channels returned from initialization
pub struct OpAmpChannels<'a> {
    pub ia_chan: embassy_stm32::adc::AnyAdcChannel<'a, peripherals::ADC1>,
    pub ib_chan: embassy_stm32::adc::AnyAdcChannel<'a, peripherals::ADC2>,
    pub ic_chan: embassy_stm32::adc::AnyAdcChannel<'a, peripherals::ADC2>,
}

// Static storage for OpAmps (they must outlive the ADC channels)
static OPAMP1_CELL: StaticCell<OpAmp<'static, peripherals::OPAMP1>> = StaticCell::new();
static OPAMP2_CELL: StaticCell<OpAmp<'static, peripherals::OPAMP2>> = StaticCell::new();
static OPAMP3_CELL: StaticCell<OpAmp<'static, peripherals::OPAMP3>> = StaticCell::new();

/// Initialize OPAMPs as PGAs for phase current shunts
///
/// OPAMP1: phase A current (PA1/PA3) -> ADC1
/// OPAMP2: phase B current (PA7/PA5) -> ADC2
/// OPAMP3: phase C current (PB0/PB2) -> ADC2
///
/// All configured for 16x gain with high-speed mode and calibrated.
pub fn init_opamps(
    opamp1: Peri<'static, peripherals::OPAMP1>,
    opamp2: Peri<'static, peripherals::OPAMP2>,
    opamp3: Peri<'static, peripherals::OPAMP3>,
    pa1: Peri<'static, peripherals::PA1>,
    pa7: Peri<'static, peripherals::PA7>,
    pb0: Peri<'static, peripherals::PB0>,
) -> OpAmpChannels<'static> {
    let mut opamp1_inst = OpAmp::new(opamp1, OpAmpSpeed::HighSpeed);
    opamp1_inst.calibrate();
    let opamp1_ref = OPAMP1_CELL.init(opamp1_inst);
    let ia_chan = opamp1_ref.pga_int(pa1, OpAmpGain::Mul16).degrade_adc();

    let mut opamp2_inst = OpAmp::new(opamp2, OpAmpSpeed::HighSpeed);
    opamp2_inst.calibrate();
    let opamp2_ref = OPAMP2_CELL.init(opamp2_inst);
    let ib_chan = opamp2_ref.pga_int(pa7, OpAmpGain::Mul16).degrade_adc();

    let mut opamp3_inst = OpAmp::new(opamp3, OpAmpSpeed::HighSpeed);
    opamp3_inst.calibrate();
    let opamp3_ref = OPAMP3_CELL.init(opamp3_inst);
    let ic_chan = opamp3_ref.pga_int(pb0, OpAmpGain::Mul16).degrade_adc();

    OpAmpChannels {
        ia_chan,
        ib_chan,
        ic_chan,
    }
}

/// ADC handles for injected conversions
pub struct AdcHandles<'a> {
    pub adc1: InjectedAdc<'a, peripherals::ADC1, 3>,
    pub adc2: InjectedAdc<'a, peripherals::ADC2, 2>,
}

/// Initialize ADC1 and ADC2 with injected conversions triggered by TIM1
///
/// # ADC Configuration
///
/// All ADC sampling via TIM1-triggered injected conversions (no DMA).
/// Host polls for samples; all values updated in ADC interrupt.
///
/// ADC clock: Embassy auto-selects prescaler to keep ADC clock ≤ 60MHz.
/// With SYSCLK=170MHz, prescaler=DIV4 → ADC clock = 42.5MHz.
///
/// Conversion time = (sample_time + 12.5) cycles per channel.
///
/// ADC1 injected sequence (3 channels, ~134 cycles total ≈ 3.2µs):
///   - ia:   24.5 + 12.5 = 37 cycles  (phase A current, low impedance)
///   - vbus: 47.5 + 12.5 = 60 cycles  (16kΩ divider, needs longer sample)
///   - temp: 24.5 + 12.5 = 37 cycles  (3kΩ NTC divider)
///
/// ADC2 injected sequence (2 channels, ~74 cycles total ≈ 1.7µs):
///   - ib: 24.5 + 12.5 = 37 cycles  (phase B current)
///   - ic: 24.5 + 12.5 = 37 cycles  (phase C current)
///
/// Both ADCs triggered simultaneously by TIM1_TRGO2. Phase currents (ia, ib, ic)
/// are sampled within ~1µs of trigger. ADC1 takes longer due to vbus/temp,
/// so its interrupt signals completion of both ADCs.
pub fn init_adc(
    adc1_periph: Peri<'static, peripherals::ADC1>,
    adc2_periph: Peri<'static, peripherals::ADC2>,
    opamp_channels: OpAmpChannels<'static>,
    vbus_pin: Peri<'static, peripherals::PA0>,
    temp_pin: Peri<'static, peripherals::PB14>,
) -> AdcHandles<'static> {
    let adc1 = Adc::new(adc1_periph, AdcConfig::default());
    let adc2 = Adc::new(adc2_periph, AdcConfig::default());

    let injected_trigger = ConversionTrigger {
        // TIM1_TRGO2 (routed from TIM1_CH4 compare in MotorPwm).
        channel: 8,
        edge: Exten::RISING_EDGE,
    };

    // ADC1 injected: phase A current + VBUS + temperature (finishes last → interrupt)
    let vbus_chan = vbus_pin.degrade_adc();
    let temp_chan = temp_pin.degrade_adc();
    let injected_adc1 = adc1.setup_injected_conversions(
        [
            (opamp_channels.ia_chan, SampleTime::CYCLES24_5), // Phase A: low-Z opamp output
            (vbus_chan, SampleTime::CYCLES47_5),              // VBUS: 16kΩ divider impedance
            (temp_chan, SampleTime::CYCLES24_5),              // Temp: 3kΩ NTC divider
        ],
        injected_trigger,
        true, // Interrupt on ADC1 (finishes last, guarantees ADC2 is also done)
    );

    // ADC2 injected: phase B and C currents (finishes first, no interrupt)
    let injected_adc2 = adc2.setup_injected_conversions(
        [
            (opamp_channels.ib_chan, SampleTime::CYCLES24_5), // Phase B: low-Z opamp output
            (opamp_channels.ic_chan, SampleTime::CYCLES24_5), // Phase C: low-Z opamp output
        ],
        injected_trigger,
        false, // No interrupt - ADC1 interrupt handles both
    );

    // Enable shared ADC1/ADC2 interrupt
    // SAFETY: Called during single-threaded initialization after ADC1/ADC2 are configured.
    // The ADC1_2 ISR (in control/foc.rs) will have valid ADC handles stored in the global
    // statics before this interrupt fires. Enabling and unpending is safe here because
    // the ISR is designed to handle being called immediately after enable.
    unsafe {
        ADC1_2::unpend();
        ADC1_2::enable();
    }

    defmt::info!("ADC1/ADC2 initialized with TIM1-triggered injected conversions");

    AdcHandles {
        adc1: injected_adc1,
        adc2: injected_adc2,
    }
}

/// Initialize LED on PC6
pub fn init_led(pc6: Peri<'static, peripherals::PC6>) -> Output<'static> {
    Output::new(pc6, Level::Low, Speed::Low)
}

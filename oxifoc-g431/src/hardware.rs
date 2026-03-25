//! Hardware abstraction layer for B-G431B-ESC1 board

use assign_resources::assign_resources;
use embassy_stm32::adc::{Adc, AdcChannel, AdcConfig, Exten, InjectedAdc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::opamp::{OpAmp, OpAmpGain, OpAmpSpeed};
use embassy_stm32::{Peri, Peripherals, peripherals};
use static_cell::StaticCell;

// ========== Resource Assignments ==========

// Resource assignments for hardware peripherals
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pc13: PC13, // Phase A low
        pa9: PA9,   // Phase B high
        pa12: PA12, // Phase B low
        pa10: PA10, // Phase C high
        pb15: PB15, // Phase C low
    }
    hall: HallResources {
        pb6: PB6,   // H1 / Encoder A+
        pb7: PB7,   // H2 / Encoder B+
        pb8: PB8,   // H3 / Encoder Z+
    }
    storage: StorageResources {
        flash: FLASH,
    }
}

// ========== Peripheral Initialization ==========

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
/// OPAMP1: phase A current VINP=PA1, VINM0=PA3 (bias) -> ADC1_IN13 (internal)
/// OPAMP2: phase B current VINP=PA7, VINM0=PA5 (bias) -> ADC2_IN16 (internal)
/// OPAMP3: phase C current VINP=PB0, VINM0=PB2 (bias) -> ADC2_IN18 (internal)
///
/// PGA mode with VINM0 bias input (PGA_IO0_BIAS): the VINM0 pin provides
/// the ground reference for the PGA. All configured for 16x gain, high-speed,
/// calibrated.
///
/// OpAmpInternalOutput handles are stored in statics to prevent embassy's Drop
/// from disabling the OPAMPs.
#[allow(clippy::too_many_arguments)]
pub fn init_opamps(
    opamp1: Peri<'static, peripherals::OPAMP1>,
    opamp2: Peri<'static, peripherals::OPAMP2>,
    opamp3: Peri<'static, peripherals::OPAMP3>,
    pa1: Peri<'static, peripherals::PA1>,
    pa3: Peri<'static, peripherals::PA3>,
    pa7: Peri<'static, peripherals::PA7>,
    pa5: Peri<'static, peripherals::PA5>,
    pb0: Peri<'static, peripherals::PB0>,
    pb2: Peri<'static, peripherals::PB2>,
) -> OpAmpChannels<'static> {
    // Workaround: embassy's OpAmpInternalOutput::drop() clears OPAEN,
    // and degrade_adc() consumes self triggering Drop. Re-enable via PAC.
    // See: https://github.com/embassy-rs/embassy/issues/4269
    let mut opamp1_inst = OpAmp::new(opamp1, OpAmpSpeed::HighSpeed);
    opamp1_inst.calibrate();
    let opamp1_ref = OPAMP1_CELL.init(opamp1_inst);
    let ia_chan = opamp1_ref
        .pga_biased_int(pa1, pa3, OpAmpGain::Mul16)
        .degrade_adc();
    embassy_stm32::pac::OPAMP1
        .csr()
        .modify(|w| w.set_opampen(true));

    let mut opamp2_inst = OpAmp::new(opamp2, OpAmpSpeed::HighSpeed);
    opamp2_inst.calibrate();
    let opamp2_ref = OPAMP2_CELL.init(opamp2_inst);
    let ib_chan = opamp2_ref
        .pga_biased_int(pa7, pa5, OpAmpGain::Mul16)
        .degrade_adc();
    embassy_stm32::pac::OPAMP2
        .csr()
        .modify(|w| w.set_opampen(true));

    let mut opamp3_inst = OpAmp::new(opamp3, OpAmpSpeed::HighSpeed);
    opamp3_inst.calibrate();
    let opamp3_ref = OPAMP3_CELL.init(opamp3_inst);
    let ic_chan = opamp3_ref
        .pga_biased_int(pb0, pb2, OpAmpGain::Mul16)
        .degrade_adc();
    embassy_stm32::pac::OPAMP3
        .csr()
        .modify(|w| w.set_opampen(true));

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
/// ADC1 injected sequence (3 channels, ~116 cycles total ≈ 2.7µs):
///   - ia:   6.5 + 12.5 = 19 cycles  (phase A current, low-Z opamp output)
///   - vbus: 47.5 + 12.5 = 60 cycles (16kΩ divider, needs longer sample)
///   - temp: 24.5 + 12.5 = 37 cycles (3kΩ NTC divider)
///
/// ADC2 injected sequence (2 channels, ~38 cycles total ≈ 0.9µs):
///   - ib: 6.5 + 12.5 = 19 cycles  (phase B current, low-Z opamp output)
///   - ic: 6.5 + 12.5 = 19 cycles  (phase C current, low-Z opamp output)
///
/// Both ADCs triggered simultaneously by TIM1_TRGO2. Phase currents (ia, ib, ic)
/// are sampled within ~0.45µs of trigger. ADC1 takes longer due to vbus/temp,
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

    // ADC1 injected: phase A current + VBUS + temperature (finishes last → interrupt)
    let vbus_chan = vbus_pin.degrade_adc();
    let temp_chan = temp_pin.degrade_adc();
    let injected_adc1 = adc1.setup_injected_conversions(
        [
            (opamp_channels.ia_chan, SampleTime::CYCLES6_5), // Phase A: low-Z opamp output
            (vbus_chan, SampleTime::CYCLES47_5),             // VBUS: 16kΩ divider impedance
            (temp_chan, SampleTime::CYCLES24_5),             // Temp: 3kΩ NTC divider
        ],
        embassy_stm32::triggers::TIM1_TRGO2,
        Exten::RISING_EDGE,
        true, // Interrupt on ADC1 (finishes last, guarantees ADC2 is also done)
    );

    // ADC2 injected: phase B and C currents (finishes first, no interrupt)
    let injected_adc2 = adc2.setup_injected_conversions(
        [
            (opamp_channels.ib_chan, SampleTime::CYCLES6_5), // Phase B: low-Z opamp output
            (opamp_channels.ic_chan, SampleTime::CYCLES6_5), // Phase C: low-Z opamp output
        ],
        embassy_stm32::triggers::TIM1_TRGO2,
        Exten::RISING_EDGE,
        false, // No interrupt - ADC1 interrupt handles both
    );

    // NOTE: ADC1_2 interrupt is NOT enabled here.
    // It's enabled in foc::init() after ADC handles and PWM outputs are set up.

    defmt::info!("ADC1/ADC2 initialized (interrupt deferred to FOC init)");

    AdcHandles {
        adc1: injected_adc1,
        adc2: injected_adc2,
    }
}

/// Initialize hardware overcurrent protection using COMP1/2/4 + DAC3.
///
/// Comparators monitor raw shunt voltage (shared pad with OPAMP inputs):
///   COMP1 INP0 = PA1 (phase A shunt, shared with OPAMP1_VINP)
///   COMP2 INP0 = PA7 (phase B shunt, shared with OPAMP2_VINP)
///   COMP4 INP0 = PB0 (phase C shunt, shared with OPAMP3_VINP)
///
/// DAC3 (internal-only, no pins) provides programmable threshold on INM:
///   COMP1 INMSEL=DACA → DAC3_CH1
///   COMP2 INMSEL=DACA → DAC3_CH2
///   COMP4 INMSEL=DACA → DAC3_CH2
///
/// COMP outputs route to TIM1 BKIN (configured in motor.rs) for
/// hardware PWM shutdown in nanoseconds — no software in the loop.
///
/// Must be called AFTER init_opamps() (pins already in analog mode).
#[allow(dead_code)] // Temporarily disabled while debugging COMP false triggers
pub fn init_overcurrent_protection(threshold_amps: f32) {
    use embassy_stm32::pac;
    use embassy_stm32::pac::comp::vals as comp_vals;
    let dac_counts = crate::config::overcurrent_dac_counts(threshold_amps);

    // Enable SYSCFG clock (shared by all COMPs) and DAC3 clock
    pac::RCC.apb2enr().modify(|w| w.set_syscfgen(true));
    pac::RCC.ahb2enr().modify(|w| w.set_dac3en(true));

    // --- DAC3: set threshold on both channels ---
    // Mode 0b011: on-chip peripherals only, buffer disabled (RM0440 §22.7.16)
    // DAC3 has no external pins — only feeds COMP inverting inputs internally.
    pac::DAC3.mcr().modify(|w| {
        w.set_mode(0, 0b011.into());
        w.set_mode(1, 0b011.into());
    });
    // Enable both channels
    pac::DAC3.cr().modify(|w| {
        w.set_en(0, true);
        w.set_en(1, true);
    });
    // Set threshold value (both channels to same value)
    pac::DAC3.dhr12r(0).write(|w| w.set_dhr(dac_counts));
    pac::DAC3.dhr12r(1).write(|w| w.set_dhr(dac_counts));

    // --- COMP1: phase A (PA1 = INP0) ---
    pac::COMP1.csr().write(|w| {
        w.set_en(true);
        w.set_inpsel(false); // INP0 = PA1
        w.set_inmsel(comp_vals::Inm::DACA); // DAC3_CH1
        w.set_hyst(comp_vals::Hysteresis::NONE);
        w.set_polarity(comp_vals::Polarity::NOT_INVERTED);
        w.set_blanksel(comp_vals::Blanking::NO_BLANKING);
    });

    // --- COMP2: phase B (PA7 = INP0) ---
    pac::COMP2.csr().write(|w| {
        w.set_en(true);
        w.set_inpsel(false); // INP0 = PA7
        w.set_inmsel(comp_vals::Inm::DACA); // DAC3_CH2
        w.set_hyst(comp_vals::Hysteresis::NONE);
        w.set_polarity(comp_vals::Polarity::NOT_INVERTED);
        w.set_blanksel(comp_vals::Blanking::NO_BLANKING);
    });

    // --- COMP4: phase C (PB0 = INP0) ---
    pac::COMP4.csr().write(|w| {
        w.set_en(true);
        w.set_inpsel(false); // INP0 = PB0
        w.set_inmsel(comp_vals::Inm::DACA); // DAC3_CH2
        w.set_hyst(comp_vals::Hysteresis::NONE);
        w.set_polarity(comp_vals::Polarity::NOT_INVERTED);
        w.set_blanksel(comp_vals::Blanking::NO_BLANKING);
    });

    defmt::info!(
        "HW overcurrent: COMP1/2/4 + DAC3 @ {}A ({}counts)",
        threshold_amps,
        dac_counts,
    );
}

/// Initialize LED on PC6
pub fn init_led(pc6: Peri<'static, peripherals::PC6>) -> Output<'static> {
    Output::new(pc6, Level::Low, Speed::Low)
}

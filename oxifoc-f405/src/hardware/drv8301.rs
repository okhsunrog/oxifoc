//! DRV8301 gate driver configuration for Simple FOCer 2

use drv8301_dd::{Drv8301, DrvError, GateCurrent, OcpMode, ShuntAmplifierGain};

// Re-export FaultStatus for use by other modules
pub use drv8301_dd::FaultStatus;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
    peripherals,
    spi::{self, Spi},
    time::Hertz,
};
use embedded_hal_bus::spi::ExclusiveDevice;

/// DRV8301 configuration matching VESC Simple FOCer 2 settings
pub struct Drv8301Config<'d> {
    pub spi: Spi<'d, embassy_stm32::mode::Blocking, embassy_stm32::spi::mode::Master>,
    pub cs: Output<'static>,
    pub en_gate: Output<'static>,
    #[allow(dead_code)]
    pub fault: embassy_stm32::gpio::Input<'static>,
}

/// Initialize SPI3 for DRV8301 communication
///
/// Simple FOCer 2 pinout:
/// - SPI3_SCK:  PC10
/// - SPI3_MISO: PC11
/// - SPI3_MOSI: PC12
/// - SPI3_CS:   PC9
pub fn init_spi(
    spi3: Peri<'static, peripherals::SPI3>,
    pc10: Peri<'static, peripherals::PC10>,
    pc11: Peri<'static, peripherals::PC11>,
    pc12: Peri<'static, peripherals::PC12>,
    pc9: Peri<'static, peripherals::PC9>,
    pb5: Peri<'static, peripherals::PB5>,
    pb7: Peri<'static, peripherals::PB7>,
) -> Drv8301Config<'static> {
    // Configure SPI3 - DRV8301: CPOL=0, CPHA=1 (Mode 1), max 10MHz
    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnSecondTransition,
    };
    spi_config.frequency = Hertz(1_000_000); // Start at 1MHz for safety

    let spi = Spi::new_blocking(
        spi3, pc10, // SCK
        pc12, // MOSI
        pc11, // MISO
        spi_config,
    );

    // CS pin (active low)
    let cs = Output::new(pc9, Level::High, Speed::VeryHigh);

    // EN_GATE pin (enable gate driver, active high)
    let en_gate = Output::new(pb5, Level::Low, Speed::Low);

    // nFAULT pin (fault indicator, active low)
    let fault = embassy_stm32::gpio::Input::new(pb7, embassy_stm32::gpio::Pull::Up);

    defmt::info!("DRV8301 SPI3 initialized");

    Drv8301Config {
        spi,
        cs,
        en_gate,
        fault,
    }
}

/// DRV8301 SPI error type alias for convenience
pub type Drv8301Error = DrvError<
    embedded_hal_bus::spi::DeviceError<embassy_stm32::spi::Error, core::convert::Infallible>,
>;

/// Configure DRV8301 according to VESC Cheap FOCer 2 v1.0 settings
///
/// Matches VESC firmware: drv8301_write_reg(2, 0x0430) + drv8301_set_current_amp_gain(10)
pub fn configure_drv8301(drv_config: &mut Drv8301Config<'_>) -> Result<(), Drv8301Error> {
    let spi_device =
        ExclusiveDevice::new_no_delay(&mut drv_config.spi, &mut drv_config.cs).unwrap();

    let mut drv = Drv8301::new(spi_device);

    defmt::info!("Configuring DRV8301...");

    // Read device ID to verify communication
    match drv.get_device_id() {
        Ok(id) => defmt::info!("DRV8301 Device ID: {:#x}", id),
        Err(e) => {
            defmt::error!("Failed to read DRV8301 device ID");
            return Err(e);
        }
    }

    // Check for existing faults and log details
    let fault_status = drv.get_fault_status()?;
    if !fault_status.is_ok() {
        log_fault_status(&fault_status);
        defmt::warn!("DRV8301 has faults, resetting...");
        drv.reset_gate_faults()?;
    }

    // VESC configuration (0x0430): OC disabled, 6-PWM, low gate current
    drv.set_ocp_mode(OcpMode::OcDisabled)?;
    drv.set_pwm_mode(false)?;
    drv.set_gate_current(GateCurrent::Low)?;

    // Cheap FOCer 2 v1.0: CURRENT_AMP_GAIN = 10
    drv.set_shunt_amplifier_gain(ShuntAmplifierGain::Gain10)?;

    // Reset any latched faults
    drv.reset_gate_faults()?;

    defmt::info!("DRV8301 configured (Cheap FOCer 2 v1.0)");

    Ok(())
}

/// Log detailed fault status information
fn log_fault_status(status: &FaultStatus) {
    defmt::warn!("DRV8301 Fault Status:");
    if status.has_voltage_fault() {
        defmt::warn!(
            "  Voltage faults - GVDD_UV: {}, GVDD_OV: {}, PVDD_UV: {}",
            status.gvdd_uv,
            status.gvdd_ov,
            status.pvdd_uv
        );
    }
    if status.has_thermal() {
        defmt::warn!(
            "  Thermal - Shutdown: {}, Warning: {}",
            status.otsd,
            status.otw
        );
    }
    if status.has_overcurrent() {
        defmt::warn!(
            "  Phase A OC - High: {}, Low: {}",
            status.fetha_oc,
            status.fetla_oc
        );
        defmt::warn!(
            "  Phase B OC - High: {}, Low: {}",
            status.fethb_oc,
            status.fetlb_oc
        );
        defmt::warn!(
            "  Phase C OC - High: {}, Low: {}",
            status.fethc_oc,
            status.fetlc_oc
        );
    }
}

/// Enable the DRV8301 gate driver
pub fn enable_gate_driver(drv_config: &mut Drv8301Config<'_>) {
    drv_config.en_gate.set_high();
    defmt::info!("DRV8301 gate driver enabled");
}

/// Disable the DRV8301 gate driver
#[allow(dead_code)]
pub fn disable_gate_driver(drv_config: &mut Drv8301Config<'_>) {
    drv_config.en_gate.set_low();
    defmt::info!("DRV8301 gate driver disabled");
}

/// Check if DRV8301 is reporting a fault via hardware pin (fast check)
#[allow(dead_code)]
pub fn is_fault_active(drv_config: &Drv8301Config<'_>) -> bool {
    drv_config.fault.is_low()
}

/// Get detailed fault status from DRV8301 via SPI
///
/// This reads both status registers to provide a complete fault picture.
/// Use `is_fault_active()` for a fast hardware-level check, or this function
/// when you need to know specifically what fault occurred.
#[allow(dead_code)]
pub fn get_fault_status(drv_config: &mut Drv8301Config<'_>) -> Result<FaultStatus, Drv8301Error> {
    let spi_device =
        ExclusiveDevice::new_no_delay(&mut drv_config.spi, &mut drv_config.cs).unwrap();
    let mut drv = Drv8301::new(spi_device);
    drv.get_fault_status()
}

/// Check and log any active faults, returning true if faults are present
#[allow(dead_code)]
pub fn check_and_log_faults(drv_config: &mut Drv8301Config<'_>) -> Result<bool, Drv8301Error> {
    let fault_status = get_fault_status(drv_config)?;
    if !fault_status.is_ok() {
        log_fault_status(&fault_status);
    }
    Ok(!fault_status.is_ok())
}

/// Reset gate driver faults
#[allow(dead_code)]
pub fn reset_faults(drv_config: &mut Drv8301Config<'_>) -> Result<(), Drv8301Error> {
    let spi_device =
        ExclusiveDevice::new_no_delay(&mut drv_config.spi, &mut drv_config.cs).unwrap();
    let mut drv = Drv8301::new(spi_device);
    drv.reset_gate_faults()
}

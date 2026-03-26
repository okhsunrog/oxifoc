//! DRV8301 gate driver configuration for Simple FOCer 2

use core::cell::RefCell;

use drv8301_dd::{Drv8301, DrvError, GateCurrent, OcAdjSet, OcpMode, ShuntAmplifierGain};

// Re-export FaultStatus for use by other modules
pub use drv8301_dd::FaultStatus;
use embassy_stm32::{
    Peri,
    exti::{self, ExtiInput},
    gpio::{Level, Output, Pull, Speed},
    interrupt::{self, typelevel::Binding},
    peripherals,
    spi::{self, Spi},
    time::Hertz,
};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::FAULT_REGISTRY;
use crate::fault::F405Fault;

/// DRV8301 configuration matching VESC Simple FOCer 2 settings
pub struct Drv8301Config<'d> {
    pub spi: Spi<'d, embassy_stm32::mode::Blocking, embassy_stm32::spi::mode::Master>,
    pub cs: Output<'static>,
    pub en_gate: Output<'static>,
}

/// nFAULT EXTI input for interrupt-driven fault detection
pub type NfaultInput = ExtiInput<'static, embassy_stm32::mode::Async>;

/// Global DRV8301 config for fault status reading from ISR/tasks
static DRV_CONFIG: CriticalSectionMutex<RefCell<Option<Drv8301Config<'static>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// Initialize SPI3 for DRV8301 communication
///
/// Simple FOCer 2 pinout:
/// - SPI3_SCK:  PC10
/// - SPI3_MISO: PC11
/// - SPI3_MOSI: PC12
/// - SPI3_CS:   PC9
///
/// Returns (Drv8301Config, NfaultInput) - the nFAULT input should be passed
/// to `nfault_monitor_task` for interrupt-driven fault detection.
#[allow(clippy::too_many_arguments)]
pub fn init_spi(
    spi3: Peri<'static, peripherals::SPI3>,
    pc10: Peri<'static, peripherals::PC10>,
    pc11: Peri<'static, peripherals::PC11>,
    pc12: Peri<'static, peripherals::PC12>,
    pc9: Peri<'static, peripherals::PC9>,
    pb5: Peri<'static, peripherals::PB5>,
    pb7: Peri<'static, peripherals::PB7>,
    exti7: Peri<'static, peripherals::EXTI7>,
    exti_irq: impl Binding<
        interrupt::typelevel::EXTI9_5,
        exti::InterruptHandler<interrupt::typelevel::EXTI9_5>,
    >,
) -> (Drv8301Config<'static>, NfaultInput) {
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
    // Start HIGH — DRV8301 must be enabled before SPI communication
    let en_gate = Output::new(pb5, Level::High, Speed::Low);

    // nFAULT pin with EXTI for interrupt-driven fault detection (active low, pull-up)
    let nfault = ExtiInput::new(pb7, exti7, Pull::Up, exti_irq);

    defmt::info!("DRV8301 SPI3 initialized");

    (Drv8301Config { spi, cs, en_gate }, nfault)
}

/// DRV8301 SPI error type alias for convenience
pub type Drv8301Error = DrvError<
    embedded_hal_bus::spi::DeviceError<embassy_stm32::spi::Error, core::convert::Infallible>,
>;

/// Configure DRV8301 according to VESC Cheap FOCer 2 v1.0 settings
///
/// Matches VESC firmware: drv8301_write_reg(2, 0x0430) + drv8301_set_current_amp_gain(10)
/// After successful configuration, stores the config globally for fault reading.
pub fn configure_and_store_drv8301(
    mut drv_config: Drv8301Config<'static>,
) -> Result<(), Drv8301Error> {
    let spi_device =
        ExclusiveDevice::new_no_delay(&mut drv_config.spi, &mut drv_config.cs).unwrap();

    let mut drv = Drv8301::new(spi_device);

    defmt::info!("Configuring DRV8301...");

    // DRV8301 needs time after EN_GATE goes high (tWAKE ≈ 1ms)
    cortex_m::asm::delay(168_000 * 2); // ~2ms at 168MHz

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

    // Hardware overcurrent protection: VDS sensing with latched shutdown.
    // IRFS7530 Rds_on ≈ 1.5mΩ (25°C), ~3mΩ (150°C).
    // At 60A nominal: VDS ≈ 90–180mV. Threshold 511mV gives margin for
    // transients while catching dead shorts (~340A cold) before FET failure.
    drv.set_oc_threshold(OcAdjSet::Vds511mV)?;
    drv.set_ocp_mode(OcpMode::OcLatchShutdown)?;
    drv.set_pwm_mode(false)?;
    drv.set_gate_current(GateCurrent::Low)?;

    // Cheap FOCer 2 v1.0: CURRENT_AMP_GAIN = 10
    drv.set_shunt_amplifier_gain(ShuntAmplifierGain::Gain10)?;

    // Reset any latched faults
    drv.reset_gate_faults()?;

    defmt::info!("DRV8301 configured (OCP: latch shutdown, VDS threshold: 511mV)");

    // Store config globally for fault status reading from tasks
    DRV_CONFIG.lock(|cell| {
        cell.replace(Some(drv_config));
    });

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
pub fn enable_gate_driver() {
    DRV_CONFIG.lock(|cell| {
        if let Some(config) = cell.borrow_mut().as_mut() {
            config.en_gate.set_high();
            defmt::info!("DRV8301 gate driver enabled");
        }
    });
}

/// Disable the DRV8301 gate driver
#[allow(dead_code)]
pub fn disable_gate_driver() {
    DRV_CONFIG.lock(|cell| {
        if let Some(config) = cell.borrow_mut().as_mut() {
            config.en_gate.set_low();
            defmt::info!("DRV8301 gate driver disabled");
        }
    });
}

/// Check if DRV8301 nFAULT is asserted (active low)
#[allow(dead_code)]
pub fn is_fault_active(nfault: &NfaultInput) -> bool {
    nfault.is_low()
}

/// nFAULT monitor task - reads DRV8301 fault details and sets DriverFault
///
/// This task monitors the DRV8301 nFAULT pin using EXTI interrupts.
/// When nFAULT goes low (fault condition), it:
/// 1. Reads detailed fault status from DRV8301 via SPI
/// 2. Logs the specific fault details (OC, thermal, voltage, etc.)
/// 3. Sets the DriverFault flag in the global fault registry
///
/// The task then waits for the rising edge (fault cleared) before
/// monitoring for the next fault.
///
/// # Note
/// This does NOT auto-clear the fault - the host must clear it via
/// the fault management protocol after investigating the cause.
#[embassy_executor::task]
pub async fn nfault_monitor_task(mut nfault: NfaultInput) {
    defmt::info!("DRV8301 nFAULT monitor started");

    loop {
        // Wait for falling edge (nFAULT asserted - fault condition)
        nfault.wait_for_falling_edge().await;

        // Read detailed fault status from DRV8301 via SPI
        let fault_status = DRV_CONFIG.lock(|cell| {
            if let Some(config) = cell.borrow_mut().as_mut() {
                let spi_device =
                    ExclusiveDevice::new_no_delay(&mut config.spi, &mut config.cs).unwrap();
                let mut drv = Drv8301::new(spi_device);
                drv.get_fault_status().ok()
            } else {
                None
            }
        });

        // Log detailed fault information and set fault with DRV status
        if let Some(status) = fault_status {
            defmt::error!("DRV8301 FAULT detected!");
            log_fault_status(&status);
            // Set the driver fault with full status details
            FAULT_REGISTRY.set(F405Fault::DrvFault(status));
        } else {
            defmt::error!("DRV8301 FAULT detected (failed to read details)!");
            // Set a default DRV fault (couldn't read details)
            FAULT_REGISTRY.set(F405Fault::DrvFault(FaultStatus::default()));
        }

        // Wait for rising edge (fault condition cleared in hardware)
        // Note: Software fault flag remains set until host clears it
        nfault.wait_for_rising_edge().await;
        defmt::info!("DRV8301 nFAULT deasserted (hardware clear)");
    }
}

/// Get detailed fault status from DRV8301 via SPI
///
/// This reads both status registers to provide a complete fault picture.
/// Use `is_fault_active()` for a fast hardware-level check, or this function
/// when you need to know specifically what fault occurred.
#[allow(dead_code)]
pub fn get_fault_status() -> Option<FaultStatus> {
    DRV_CONFIG.lock(|cell| {
        if let Some(config) = cell.borrow_mut().as_mut() {
            let spi_device =
                ExclusiveDevice::new_no_delay(&mut config.spi, &mut config.cs).unwrap();
            let mut drv = Drv8301::new(spi_device);
            drv.get_fault_status().ok()
        } else {
            None
        }
    })
}

/// Check and log any active faults, returning true if faults are present
#[allow(dead_code)]
pub fn check_and_log_faults() -> bool {
    if let Some(fault_status) = get_fault_status()
        && !fault_status.is_ok()
    {
        log_fault_status(&fault_status);
        return true;
    }
    false
}

/// Reset gate driver faults
#[allow(dead_code)]
pub fn reset_faults() -> Result<(), ()> {
    DRV_CONFIG.lock(|cell| {
        if let Some(config) = cell.borrow_mut().as_mut() {
            let spi_device =
                ExclusiveDevice::new_no_delay(&mut config.spi, &mut config.cs).unwrap();
            let mut drv = Drv8301::new(spi_device);
            drv.reset_gate_faults().map_err(|_| ())
        } else {
            Err(())
        }
    })
}

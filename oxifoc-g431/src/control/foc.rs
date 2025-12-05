//! FOC (Field-Oriented Control) management and ADC interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::fault::{FaultKind, VOLTAGE_HYSTERESIS_MV};
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::sensors::NoSensor;
use oxifoc_core::motor::{ControlMode, FocDriver};
use oxifoc_core::state::{self, FAULT_REGISTRY};

use crate::config::{BOARD, NTC, PWM_CONFIG};
use crate::motor::MotorPwm;
use crate::sensors::{G431CurrentSensor, G431CurrentSensorExt, HallAngleProxy};

// ========== ADC Sample Storage (Global Atomics) ==========

/// Latest phase current samples (from ADC1/ADC2 injected sequences).
pub static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts (updated in ADC interrupt).
pub static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest measured FET temperature in 0.1°C units (updated in ADC interrupt).
pub static FET_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);

// ========== ADC Handles ==========

/// Handle for ADC1 injected conversions (TIM1-triggered): ia, vbus, temp.
pub static ADC1_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC1, 3>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
/// Handle for ADC2 injected conversions (TIM1-triggered).
pub static ADC2_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC2, 2>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== FOC Control ==========

/// FOC driver storage (mutated only inside the ADC ISR)
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor>;
type FocDriverType = FocDriver<MotorPwm<'static>, G431CurrentSensor, PhaseManagerType>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Initialization ==========

/// Initialize FOC driver with motor PWM and sensors
pub async fn init(
    mut motor_pwm: MotorPwm<'static>,
    adc1: InjectedAdc<peripherals::ADC1, 3>,
    adc2: InjectedAdc<peripherals::ADC2, 2>,
) {
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();

    // Build current sensor and phase manager
    let current_sensor = G431CurrentSensor::from_board(&BOARD);
    let hall_proxy = HallAngleProxy::new();
    let phase_manager = PhaseManager::with_hall(hall_proxy);
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);

    // Build FOC driver with dt from PWM config
    let mut foc_driver = FocDriver::new(
        FocController::new(initial_vbus_v),
        motor_pwm,
        current_sensor,
        phase_manager,
        PWM_CONFIG.dt_s(),
    );

    // Store ADC handles for ISR access
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc2)));

    // Allow ADC injected conversions to start firing before zero-current calibration.
    Timer::after(Duration::from_millis(10)).await;
    foc_driver.current_sensor_mut().calibrate().await;

    // Install FOC driver for ISR-only access.
    FOC_DRIVER.lock(|cell| {
        cell.replace(Some(foc_driver));
    });

    defmt::info!("FOC driver initialized and calibrated");
}

// ========== Fault Detection ==========

/// Check voltage faults (overvoltage / undervoltage)
///
/// Sets fault if out of range. Clears undervoltage (recoverable) if back in range with hysteresis.
#[inline]
fn check_voltage_faults(vbus_mv: u32) {
    // Overvoltage check
    if vbus_mv > BOARD.max_vbus_mv && !FAULT_REGISTRY.is_set(FaultKind::OverVoltage) {
        state::set_fault(FaultKind::OverVoltage);
    }

    // Undervoltage check (recoverable with hysteresis)
    if vbus_mv < BOARD.min_vbus_mv && !FAULT_REGISTRY.is_set(FaultKind::UnderVoltage) {
        state::set_fault(FaultKind::UnderVoltage);
    } else if vbus_mv > BOARD.min_vbus_mv + VOLTAGE_HYSTERESIS_MV
        && FAULT_REGISTRY.is_set(FaultKind::UnderVoltage)
    {
        // Auto-recover undervoltage (it's recoverable)
        state::clear_fault(FaultKind::UnderVoltage);
    }
}

/// Check temperature fault (FET overtemperature)
#[inline]
fn check_temperature_fault(temp_c_x10: u16) {
    let temp_c = temp_c_x10 as f32 / 10.0;
    if temp_c > BOARD.max_fet_temp_c && !FAULT_REGISTRY.is_set(FaultKind::OverTemp) {
        state::set_fault(FaultKind::OverTemp);
    }
}

/// Check phase current faults (overcurrent)
///
/// Instantaneous trip if any phase exceeds limit.
#[inline]
fn check_current_faults(ia: f32, ib: f32, ic: f32) {
    let limit = BOARD.max_phase_current_a;
    if (ia.abs() > limit || ib.abs() > limit || ic.abs() > limit)
        && !FAULT_REGISTRY.is_set(FaultKind::OverCurrent)
    {
        state::set_fault(FaultKind::OverCurrent);
    }
}

// ========== ADC Interrupt Handler ==========

/// ADC1/ADC2 shared interrupt: read all injected ADC samples and run FOC control.
///
/// ADC1: ia (phase A), vbus, temp
/// ADC2: ib (phase B), ic (phase C)
///
/// Triggered by ADC1 end-of-sequence (ADC1 finishes last).
/// Stores raw phase currents; converts vbus/temp to engineering units.
/// Runs FOC control loop synchronized with PWM.
#[interrupt]
fn ADC1_2() {
    static mut SEQ: u32 = 0;

    use oxifoc_core::foc::sensors::{AdcSnapshot, TempSensorId};

    // Local storage for ADC readings
    let mut ia_raw: u16 = 0;
    let mut ib_raw: u16 = 0;
    let mut ic_raw: u16 = 0;
    let mut vbus_mv: u32 = 0;
    let mut temp_c_x10: u16 = 0;

    // Read ADC1 injected: phase A current, VBUS voltage, FET temperature
    ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            ia_raw = samples[0];
            IA_SAMPLE.store(ia_raw, Ordering::Relaxed);

            // Convert VBUS raw ADC to millivolts
            vbus_mv = BOARD.vbus_mv_from_adc(samples[1]);
            VBUS_MV.store(vbus_mv, Ordering::Relaxed);

            // Convert temperature raw ADC to 0.1°C units
            let temp_c = NTC.temp_c_from_adc(samples[2], BOARD.adc_max_counts);
            temp_c_x10 = if temp_c.is_finite() && temp_c >= 0.0 {
                (temp_c * 10.0) as u16
            } else {
                0
            };
            FET_TEMP_C_X10.store(temp_c_x10, Ordering::Relaxed);
        }
    });

    // Read ADC2 injected: phase B and C currents
    ADC2_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            ib_raw = samples[0];
            ic_raw = samples[1];
            IB_SAMPLE.store(ib_raw, Ordering::Relaxed);
            IC_SAMPLE.store(ic_raw, Ordering::Relaxed);
        }
    });

    // === Fault detection (voltage and temperature) ===
    check_voltage_faults(vbus_mv);
    check_temperature_fault(temp_c_x10);

    // Get current timestamp for FOC and phase manager
    let now_ticks = embassy_time::Instant::now().as_ticks();

    // Build ADC snapshot
    *SEQ = SEQ.wrapping_add(1);
    let adc_snapshot = AdcSnapshot::new(ia_raw, ib_raw, ic_raw, vbus_mv, *SEQ)
        .with_temp(TempSensorId::Fet, temp_c_x10);

    // Get Hall snapshot
    let hall_snapshot = crate::sensors::hall::get_snapshot(now_ticks);

    // Run FOC control loop (skip if faulted with non-recoverable fault)
    let foc_telem = FOC_DRIVER.lock(|cell| {
        if let Some(driver) = cell.borrow_mut().as_mut() {
            // Update bus voltage
            driver.set_vbus(vbus_mv as f32 / 1000.0);

            // Process commands from core state channel
            let mode = state::process_commands(driver);

            // If faulted, disable outputs and skip FOC step
            if FAULT_REGISTRY.any() {
                if mode != ControlMode::Stopped {
                    driver.set_mode(ControlMode::Stopped);
                }
                return None;
            }

            // Run FOC step (dt is stored in driver from PWM_CONFIG)
            match driver.step(now_ticks) {
                Ok(telem) => {
                    // Check phase currents for overcurrent (instantaneous)
                    check_current_faults(telem.ia, telem.ib, telem.ic);
                    Some(telem)
                }
                Err(_) => {
                    // Sensor not ready or other error - disable outputs
                    if mode != ControlMode::Stopped {
                        driver.set_mode(ControlMode::Stopped);
                    }
                    None
                }
            }
        } else {
            None
        }
    });

    // Update global state with telemetry
    if let Some(foc) = foc_telem {
        state::update_telemetry(adc_snapshot, hall_snapshot, foc);
    }
}

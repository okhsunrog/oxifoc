//! FOC (Field-Oriented Control) management and ADC interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::{FocController, FocTelemetry};
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::sensors::NoSensor;
use oxifoc_core::motor::{ControlMode, FocDriver};

use crate::config::{BOARD, NTC};
use crate::motor::pwm::MotorPwm;
use crate::sensors::{G431CurrentSensor, HallAngleProxy};

// ========== ADC Sample Storage (Global Atomics) ==========

/// Latest phase current samples (from ADC1/ADC2 injected sequences).
pub static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts (updated in ADC interrupt).
pub static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest measured FET temperature in 0.1°C units (updated in ADC interrupt).
pub static FET_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);
/// Sequence counter for ADC samples (incremented each poll).
pub static ADC_SEQ: AtomicU32 = AtomicU32::new(0);

// ========== ADC Handles ==========

/// Handle for ADC1 injected conversions (TIM1-triggered): ia, vbus, temp.
pub static ADC1_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC1, 3>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
/// Handle for ADC2 injected conversions (TIM1-triggered).
pub static ADC2_INJECTED: CriticalSectionMutex<RefCell<Option<InjectedAdc<peripherals::ADC2, 2>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== FOC Control ==========

/// FOC telemetry data (updated by ADC ISR)
pub static FOC_TELEMETRY: Watch<CriticalSectionRawMutex, FocTelemetry, 1> = Watch::new();

/// FOC command channel (tasks → ISR)
pub static FOC_CMD: Channel<CriticalSectionRawMutex, ControlMode, 4> = Channel::new();

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
    let current_sensor = G431CurrentSensor::new(&BOARD);
    let hall_proxy = HallAngleProxy::new();
    let phase_manager = PhaseManager::with_hall(hall_proxy);
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);

    // Build FOC driver
    let mut foc_driver = FocDriver::new(
        FocController::new(initial_vbus_v),
        motor_pwm,
        current_sensor,
        phase_manager,
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

/// Send a control mode command to the FOC driver
pub fn send_command(mode: ControlMode) {
    let _ = FOC_CMD.try_send(mode);
}

/// Map duty percent to target q-axis current
pub fn duty_to_iq(duty: u8) -> f32 {
    BOARD.duty_to_iq(duty)
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
unsafe fn ADC1_2() {
    // Static state (ISR has exclusive access)
    static mut CONTROL_MODE: ControlMode = ControlMode::Stopped;
    static mut LAST_HALL_SEQ: u32 = 0;

    // Read ADC1 injected: phase A current, VBUS voltage, FET temperature
    ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            IA_SAMPLE.store(samples[0], Ordering::Relaxed);

            // Convert VBUS raw ADC to millivolts
            let vbus_mv = BOARD.vbus_mv_from_adc(samples[1]);
            VBUS_MV.store(vbus_mv, Ordering::Relaxed);

            // Convert temperature raw ADC to 0.1°C units
            let temp_c = NTC.temp_c_from_adc(samples[2], BOARD.adc_max_counts);
            let temp_c_x10 = if temp_c.is_finite() && temp_c >= 0.0 {
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
            IB_SAMPLE.store(samples[0], Ordering::Relaxed);
            IC_SAMPLE.store(samples[1], Ordering::Relaxed);
        }
    });

    // Process FOC commands (non-blocking, ~20ns overhead)
    while let Ok(cmd) = FOC_CMD.try_receive() {
        *CONTROL_MODE = cmd;
    }

    // Incorporate latest Hall edge (from EXTI)
    crate::sensors::hall::process_edge(LAST_HALL_SEQ);

    // Snapshot current Hall data for telemetry/consumers
    let now_ticks = embassy_time::Instant::now().as_ticks();
    crate::sensors::hall::update_snapshot(now_ticks);

    // Run FOC control loop
    FOC_DRIVER.lock(|cell| {
        if let Some(driver) = cell.borrow_mut().as_mut() {
            // Update bus voltage
            let vbus_mv = VBUS_MV.load(Ordering::Relaxed);
            driver.set_vbus(vbus_mv as f32 / 1000.0);

            // Update control mode
            driver.set_mode(*CONTROL_MODE);

            // Run FOC step (dt = 1/20kHz = 50µs)
            const DT: f32 = 1.0 / 20_000.0;
            match driver.step(DT, now_ticks) {
                Ok(telem) => {
                    // Broadcast telemetry to all listeners
                    FOC_TELEMETRY.sender().send(telem);
                }
                Err(_) => {
                    // Sensor not ready or other error - disable outputs
                    driver.set_mode(ControlMode::Stopped);
                }
            }
        }
    });
}

// ========== Public API for Protocol Servers ==========

/// Get ADC sample snapshot
pub struct AdcSnapshot {
    pub ia: u16,
    pub ib: u16,
    pub ic: u16,
    pub vbus_mv: u32,
    pub fet_temp_c_x10: u16,
    pub seq: u32,
}

pub fn get_adc_snapshot() -> AdcSnapshot {
    let seq = ADC_SEQ.fetch_add(1, Ordering::Relaxed);
    AdcSnapshot {
        ia: IA_SAMPLE.load(Ordering::Relaxed),
        ib: IB_SAMPLE.load(Ordering::Relaxed),
        ic: IC_SAMPLE.load(Ordering::Relaxed),
        vbus_mv: VBUS_MV.load(Ordering::Relaxed),
        fet_temp_c_x10: FET_TEMP_C_X10.load(Ordering::Relaxed),
        seq,
    }
}

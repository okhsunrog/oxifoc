//! FOC (Field-Oriented Control) management and ADC interrupt handling for F405
//!
//! # ADC Configuration
//!
//! All ADC sampling via TIM1-triggered injected conversions (no DMA for currents).
//! - ADC1 injected: Phase A current (PC0, ch10) + Board temp (PA3, ch3)
//! - ADC2 injected: Phase B current (PC1, ch11) + Motor temp (PC4, ch14)
//! - ADC3 injected: Phase C current (PC2, ch12) + VBUS (PC3, ch13)
//!
//! Triggered by TIM1_CH4 compare event (at PWM center).

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::fault;
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::sensors::{AdcSnapshot, NoSensor, TempSensorId};
use oxifoc_core::foc::trig::FastSinCos;
use oxifoc_core::motor::{ControlMode, FocDriver};
use oxifoc_core::storage::RuntimeConfig;

use crate::config::{BOARD, NTC_BOARD, NTC_MOTOR, PWM_CONFIG};
use crate::fault::F405Fault;
use crate::motor::MotorPwm;
use crate::sensors::{F405CurrentSensor, F405CurrentSensorExt, hall::HallAngleProxy};
use crate::{FAULT_REGISTRY, STATE};

// ========== ADC Sample Storage (Global Atomics) ==========

/// Latest phase current samples (from ADC1/ADC2/ADC3 injected sequences).
pub static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts (updated in ADC interrupt).
pub static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest board temperature in 0.1°C units (updated in ADC interrupt).
pub static BOARD_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);
/// Latest motor temperature in 0.1°C units (updated in ADC interrupt).
pub static MOTOR_TEMP_C_X10: AtomicU16 = AtomicU16::new(0);

// ========== ADC Handles ==========

/// ADC1 injected handle: phase A current + board temperature
pub static ADC1_INJECTED: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, peripherals::ADC1, 2>>>,
> = CriticalSectionMutex::new(RefCell::new(None));
/// ADC2 injected handle: phase B current + motor temperature
pub static ADC2_INJECTED: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, peripherals::ADC2, 2>>>,
> = CriticalSectionMutex::new(RefCell::new(None));
/// ADC3 injected handle: phase C current + VBUS
pub static ADC3_INJECTED: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, peripherals::ADC3, 2>>>,
> = CriticalSectionMutex::new(RefCell::new(None));

// ========== FOC Control ==========

/// FOC driver storage (mutated only inside the ADC ISR)
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor>;
type FocDriverType = FocDriver<MotorPwm<'static>, F405CurrentSensor, PhaseManagerType, FastSinCos>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Initialization ==========

/// Initialize FOC driver with motor PWM, sensors, and stored config.
pub async fn init(
    mut motor_pwm: MotorPwm<'static>,
    adc_handles: crate::hardware::peripherals::AdcHandles,
    config: &RuntimeConfig,
) {
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();

    // Store ADC handles for ISR access (before enabling PWM triggers)
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc2)));
    ADC3_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc3)));

    // Enable ADC interrupt and PWM outputs (CH4 trigger + phase channels).
    // Order: install ADC handles → enable interrupt → enable PWM triggers.
    unsafe {
        use embassy_stm32::interrupt::typelevel::Interrupt;
        <embassy_stm32::interrupt::typelevel::ADC as Interrupt>::unpend();
        <embassy_stm32::interrupt::typelevel::ADC as Interrupt>::enable();
    }
    motor_pwm.enable_outputs();

    // Build current sensor and phase manager
    let current_sensor = F405CurrentSensor::from_board(&BOARD, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE);
    crate::sensors::hall::apply_stored_config(config);
    let hall_proxy = HallAngleProxy::new();
    let phase_manager = PhaseManager::with_hall(hall_proxy);
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);

    // Build FOC controller — use stored motor params for PI tuning if available
    let mut foc_controller = if let Some(ref mp) = config.motor_params {
        if mp.is_valid() {
            let l_avg = (mp.inductance_d_h + mp.inductance_q_h) / 2.0;
            defmt::info!(
                "Using stored motor params: R={=f32}, L={=f32}, λ={=f32}, pp={}",
                mp.resistance_ohm,
                l_avg,
                mp.flux_linkage_wb,
                mp.pole_pairs
            );
            FocController::<_, FastSinCos>::from_motor_params(
                mp.resistance_ohm,
                l_avg,
                initial_vbus_v,
            )
        } else {
            FocController::<_, FastSinCos>::new(initial_vbus_v)
        }
    } else if let Some(ref pg) = config.pi_gains {
        let mut foc = FocController::<_, FastSinCos>::new(initial_vbus_v);
        foc.id_pi.set_gains(pg.kp, pg.ki);
        foc.iq_pi.set_gains(pg.kp, pg.ki);
        defmt::info!("Using stored PI gains: kp={=f32}, ki={=f32}", pg.kp, pg.ki);
        foc
    } else {
        FocController::<_, FastSinCos>::new(initial_vbus_v)
    };

    // Configure dead time compensation
    foc_controller.set_dead_time_comp(PWM_CONFIG.dead_time_ns, PWM_CONFIG.pwm_freq_hz);

    // Build FOC driver with dt from PWM config
    let mut foc_driver = FocDriver::new(
        foc_controller,
        motor_pwm,
        current_sensor,
        phase_manager,
        PWM_CONFIG.dt_s(),
    );

    // Set current limits from board config
    foc_driver.set_current_limits(
        oxifoc_core::motor::foc_driver::CurrentLimits::from_max_current(BOARD.max_phase_current_a),
    );

    // Allow ADC injected conversions to start firing before zero-current calibration.
    Timer::after(Duration::from_millis(10)).await;
    foc_driver.current_sensor_mut().calibrate().await;

    // Install FOC driver for ISR-only access.
    FOC_DRIVER.lock(|cell| {
        cell.replace(Some(foc_driver));
    });

    defmt::info!("F405 FOC driver initialized and calibrated");
}

/// Map duty percent to target q-axis current
pub fn duty_to_iq(duty: u8) -> f32 {
    BOARD.duty_to_iq(duty)
}

// ========== ADC Interrupt Handler ==========

/// ADC interrupt: read all injected ADC samples and run FOC control.
///
/// Triggered by ADC3 JEOC (end of injected conversion sequence).
/// ADC1, ADC2, ADC3 all start conversion simultaneously on TIM1_CC4.
#[interrupt]
fn ADC() {
    static mut SEQ: u32 = 0;

    // Read ADC1 injected data (phase A current + board temp)
    let (ia_raw, board_temp_raw) = ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            (samples[0], samples[1])
        } else {
            (0, 0)
        }
    });
    IA_SAMPLE.store(ia_raw, Ordering::Relaxed);

    // Convert board temperature raw ADC to 0.1°C units
    let board_temp_c = NTC_BOARD.temp_c_from_adc(board_temp_raw, BOARD.adc_max_counts);
    let board_temp_c_x10 = if board_temp_c.is_finite() && board_temp_c >= 0.0 {
        (board_temp_c * 10.0) as u16
    } else {
        0
    };
    BOARD_TEMP_C_X10.store(board_temp_c_x10, Ordering::Relaxed);

    // Read ADC2 injected data (phase B current + motor temp)
    let (ib_raw, motor_temp_raw) = ADC2_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            (samples[0], samples[1])
        } else {
            (0, 0)
        }
    });
    IB_SAMPLE.store(ib_raw, Ordering::Relaxed);

    // Convert motor temperature raw ADC to 0.1°C units
    let motor_temp_c = NTC_MOTOR.temp_c_from_adc(motor_temp_raw, BOARD.adc_max_counts);
    let motor_temp_c_x10 = if motor_temp_c.is_finite() && motor_temp_c >= 0.0 {
        (motor_temp_c * 10.0) as u16
    } else {
        0
    };
    MOTOR_TEMP_C_X10.store(motor_temp_c_x10, Ordering::Relaxed);

    // Read ADC3 injected data (phase C current + VBUS)
    let (ic_raw, vbus_raw) = ADC3_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let samples = injected.read_injected_samples();
            (samples[0], samples[1])
        } else {
            (0, 0)
        }
    });
    IC_SAMPLE.store(ic_raw, Ordering::Relaxed);

    // Convert VBUS raw ADC to millivolts
    let vbus_mv = BOARD.vbus_mv_from_adc(vbus_raw);
    VBUS_MV.store(vbus_mv, Ordering::Relaxed);

    // === Fault detection (voltage and temperature) ===
    fault::check_voltage_faults(
        vbus_mv,
        &BOARD,
        &FAULT_REGISTRY,
        F405Fault::OverVoltage,
        F405Fault::UnderVoltage,
    );
    fault::check_temperature_fault(
        board_temp_c_x10,
        &BOARD,
        &FAULT_REGISTRY,
        F405Fault::OverTemp,
    );

    // Get current timestamp for FOC and phase manager
    let now_ticks = embassy_time::Instant::now().as_ticks();

    // Build ADC snapshot
    *SEQ = SEQ.wrapping_add(1);
    let adc_snapshot = AdcSnapshot::new(ia_raw, ib_raw, ic_raw, vbus_mv, *SEQ)
        .with_temp(TempSensorId::Board, board_temp_c_x10)
        .with_temp(TempSensorId::Motor, motor_temp_c_x10);

    // Get Hall snapshot
    let hall_snapshot = crate::sensors::hall::get_snapshot(now_ticks);

    // Run FOC control loop (skip if faulted)
    let foc_telem = FOC_DRIVER.lock(|cell| {
        if let Some(driver) = cell.borrow_mut().as_mut() {
            // Update bus voltage
            driver.set_vbus(vbus_mv as f32 / 1000.0);

            // Process commands from core state channel
            let mode = oxifoc_core::state::process_commands(&STATE, driver);

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
                    fault::check_current_faults(
                        telem.ia,
                        telem.ib,
                        telem.ic,
                        &BOARD,
                        &FAULT_REGISTRY,
                        F405Fault::OverCurrent,
                    );
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
    let foc_telem = foc_telem.unwrap_or_default();
    {
        let foc = foc_telem;
        oxifoc_core::state::update_telemetry(&STATE, adc_snapshot, hall_snapshot, foc);

        // Fast telemetry: decimation at the source, write to bbqueue
        use oxifoc_core::runtime::streaming::{
            FAST_TELEM_PERIOD, build_fast_telemetry, push_fast_telemetry,
        };
        let period = FAST_TELEM_PERIOD.load(Ordering::Relaxed);
        if period != 0 && (*SEQ).is_multiple_of(period) {
            let (hall_state, velocity_rad_s) = hall_snapshot
                .map(|h| (h.state, h.velocity_rad_s))
                .unwrap_or((0, 0.0));
            let telem = build_fast_telemetry(&foc, hall_state, velocity_rad_s, *SEQ);
            push_fast_telemetry(&telem);
        }
    }
}

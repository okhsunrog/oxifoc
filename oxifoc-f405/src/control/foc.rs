//! FOC (Field-Oriented Control) management and ADC interrupt handling for F405
//!
//! Uses raw PAC access for injected ADC since embassy-stm32 doesn't support
//! injected channels on STM32F4.
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

use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::{interrupt, pac};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::fault;
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::sensors::{AdcSnapshot, NoSensor, TempSensorId};
use oxifoc_core::motor::{ControlMode, FocDriver};

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

// ========== FOC Control ==========

/// FOC driver storage (mutated only inside the ADC ISR)
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor>;
type FocDriverType = FocDriver<MotorPwm<'static>, F405CurrentSensor, PhaseManagerType>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Injected ADC External Trigger Selection ==========

// Use PAC types for ADC configuration
use pac::adc::vals::{Exten, SampleTime};
use pac::timer::vals::Ocm;

// ========== Initialization ==========

/// Initialize ADCs for injected conversions triggered by TIM1_CH4
///
/// This configures:
/// - ADC1: Phase A current (PC0, ch10) + Board temp (PA3, ch3) - 2 injected channels
/// - ADC2: Phase B current (PC1, ch11) + Motor temp (PC4, ch14) - 2 injected channels
/// - ADC3: Phase C current (PC2, ch12) + VBUS (PC3, ch13) - 2 injected channels
///
/// All triggered by TIM1_CC4 rising edge.
/// ADC3 generates interrupt on JEOC (all ADCs now have 2 channels each).
pub fn init_adc_injected() {
    // Enable ADC clocks (APB2)
    pac::RCC.apb2enr().modify(|w| {
        w.set_adc1en(true);
        w.set_adc2en(true);
        w.set_adc3en(true);
    });

    // Configure ADC common settings
    // Prescaler: PCLK2/4 = 84MHz/4 = 21MHz (max 36MHz per datasheet)
    pac::ADC123_COMMON.ccr().modify(|w| {
        w.set_adcpre(pac::adccommon::vals::Adcpre::DIV4);
    });

    // ========== ADC1: Phase A current (PC0, ch10) + Board temp (PA3, ch3) ==========
    let adc1 = pac::ADC1;

    // Power on ADC1
    adc1.cr2().modify(|w| w.set_adon(true));

    // Configure sample time for channels
    // ch3 (PA3) is in SMPR2[3], ch10 (PC0) is in SMPR1[0]
    adc1.smpr2().modify(|w| w.set_smp(3, SampleTime::CYCLES15)); // ch3 board temp
    adc1.smpr1().modify(|w| w.set_smp(0, SampleTime::CYCLES15)); // ch10 phase A

    // Configure injected sequence: 2 channels
    // JL = 1 means 2 conversions
    // When JL=1, JSQ3 and JSQ4 are used (JSQ3 first, then JSQ4)
    adc1.jsqr().write(|w| {
        w.set_jl(1); // 2 conversions
        w.set_jsq(2, 10); // JSQ3 = channel 10 (phase A current)
        w.set_jsq(3, 3); // JSQ4 = channel 3 (board temp)
    });

    // Configure external trigger for injected: TIM1_CC4, rising edge
    // TIM1_CC4 = 0b0000 for injected trigger selection
    adc1.cr2().modify(|w| {
        w.set_jextsel(0b0000); // TIM1_CC4
        w.set_jexten(Exten::RISING_EDGE);
    });

    // ========== ADC2: Phase B current (PC1, ch11) + Motor temp (PC4, ch14) ==========
    let adc2 = pac::ADC2;

    // Power on ADC2
    adc2.cr2().modify(|w| w.set_adon(true));

    // Configure sample time for channels
    // ch11 (PC1) is in SMPR1[1], ch14 (PC4) is in SMPR1[4]
    adc2.smpr1().modify(|w| {
        w.set_smp(1, SampleTime::CYCLES15); // ch11 phase B
        w.set_smp(4, SampleTime::CYCLES15); // ch14 motor temp
    });

    // Configure injected sequence: 2 channels
    // JL = 1 means 2 conversions
    // When JL=1, JSQ3 and JSQ4 are used (JSQ3 first, then JSQ4)
    adc2.jsqr().write(|w| {
        w.set_jl(1); // 2 conversions
        w.set_jsq(2, 11); // JSQ3 = channel 11 (phase B current)
        w.set_jsq(3, 14); // JSQ4 = channel 14 (motor temp)
    });

    // Configure external trigger for injected: TIM1_CC4, rising edge
    adc2.cr2().modify(|w| {
        w.set_jextsel(0b0000); // TIM1_CC4
        w.set_jexten(Exten::RISING_EDGE);
    });

    // ========== ADC3: Phase C current (PC2, ch12) + VBUS (PC3, ch13) ==========
    let adc3 = pac::ADC3;

    // Power on ADC3
    adc3.cr2().modify(|w| w.set_adon(true));

    // Configure sample time for channels 12 and 13 - 15 cycles
    adc3.smpr1().modify(|w| {
        w.set_smp(2, SampleTime::CYCLES15); // ch12 is in SMPR1[2]
        w.set_smp(3, SampleTime::CYCLES15); // ch13 is in SMPR1[3]
    });

    // Configure injected sequence: 2 channels
    // JL = 1 means 2 conversions
    // When JL=1, JSQ3 and JSQ4 are used (JSQ3 first, then JSQ4)
    adc3.jsqr().write(|w| {
        w.set_jl(1); // 2 conversions
        w.set_jsq(2, 12); // JSQ3 = channel 12 (phase C)
        w.set_jsq(3, 13); // JSQ4 = channel 13 (VBUS)
    });

    // Configure external trigger for injected: TIM1_CC4, rising edge
    adc3.cr2().modify(|w| {
        w.set_jextsel(0b0000); // TIM1_CC4
        w.set_jexten(Exten::RISING_EDGE);
    });

    // Enable JEOC interrupt on ADC3 (it finishes last with 2 channels)
    adc3.cr1().modify(|w| w.set_jeocie(true));

    // SAFETY: All ADCs are configured above and FOC_DRIVER will be initialized
    // before motor control starts. The ISR only accesses ADC registers and
    // atomics/mutexes that are safely initialized.
    unsafe {
        interrupt::typelevel::ADC::unpend();
        interrupt::typelevel::ADC::enable();
    }

    defmt::info!("F405 ADC injected channels initialized (TIM1_CC4 trigger)");
}

/// Configure TIM1 CH4 to trigger ADC at peak of center-aligned PWM
///
/// Sets TIM1_CH4 compare value near ARR (peak of triangle wave).
/// In center-aligned mode, CNT=ARR is the V0 point where all low-side
/// FETs are ON for any duty < 100%, making it optimal for low-side shunt sampling.
pub fn configure_tim1_adc_trigger() {
    let tim1 = pac::TIM1;

    // Read current ARR value
    let arr = tim1.arr().read().arr();

    // Sample at peak of triangle wave (V0 - all low-side ON)
    // Small offset ensures ADC completes before any switching edges
    let peak_offset = arr / 50; // ~2% margin
    let trigger_point = arr.saturating_sub(peak_offset);

    // Set CH4 compare value to peak
    tim1.ccr(3).write(|w| w.set_ccr(trigger_point));

    // Enable CH4 output compare (not for pin output, just for internal trigger)
    // We need to enable the CC4 event generation
    tim1.ccmr_output(1).modify(|w| {
        // CH4 in PWM mode 1 (active when CNT < CCR4)
        w.set_ocm(1, Ocm::PWM_MODE1);
    });

    // Enable CC4 preload
    tim1.ccmr_output(1).modify(|w| {
        w.set_ocpe(1, true);
    });

    // Note: We don't enable CH4 output on a pin, just the compare event
    // The compare event will trigger ADC via the internal connection

    defmt::info!(
        "TIM1 CH4 configured for ADC trigger at ARR-offset={}",
        trigger_point
    );
}

/// Initialize FOC driver with motor PWM and sensors
pub async fn init(mut motor_pwm: MotorPwm<'static>) {
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();

    // Initialize ADC for injected conversions
    init_adc_injected();

    // Configure TIM1 CH4 for ADC triggering
    configure_tim1_adc_trigger();

    // Build current sensor and phase manager
    let current_sensor = F405CurrentSensor::from_board(&BOARD);
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
/// ADC3 finishes last (2 channels) and generates the interrupt.
#[interrupt]
fn ADC() {
    static mut SEQ: u32 = 0;

    let adc1 = pac::ADC1;
    let adc2 = pac::ADC2;
    let adc3 = pac::ADC3;

    // Check if ADC3 JEOC flag is set
    if !adc3.sr().read().jeoc() {
        return;
    }

    // Read ADC1 injected data (phase A current + board temp)
    // For JL=1: JDR1 = first conversion (ch10), JDR2 = second conversion (ch3)
    let ia_raw = adc1.jdr(0).read().jdata();
    let board_temp_raw = adc1.jdr(1).read().jdata();
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
    // For JL=1: JDR1 = first conversion (ch11), JDR2 = second conversion (ch14)
    let ib_raw = adc2.jdr(0).read().jdata();
    let motor_temp_raw = adc2.jdr(1).read().jdata();
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
    // For JL=1: JDR1 = first conversion (ch12), JDR2 = second conversion (ch13)
    let ic_raw = adc3.jdr(0).read().jdata();
    let vbus_raw = adc3.jdr(1).read().jdata();
    IC_SAMPLE.store(ic_raw, Ordering::Relaxed);

    // Convert VBUS raw ADC to millivolts
    let vbus_mv = BOARD.vbus_mv_from_adc(vbus_raw);
    VBUS_MV.store(vbus_mv, Ordering::Relaxed);

    // Clear JEOC flags on all ADCs
    adc1.sr().modify(|w| w.set_jeoc(false));
    adc2.sr().modify(|w| w.set_jeoc(false));
    adc3.sr().modify(|w| w.set_jeoc(false));

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
    if let Some(foc) = foc_telem {
        oxifoc_core::state::update_telemetry(&STATE, adc_snapshot, hall_snapshot, foc);
    }
}

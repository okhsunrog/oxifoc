//! FOC (Field-Oriented Control) management and ADC interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{Peri, interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::fault;
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::sensors::NoSensor;
use oxifoc_core::motor::{ControlMode, FocDriver};
use oxifoc_core::storage::RuntimeConfig;

use crate::config::{BOARD, NTC, PWM_CONFIG};
use crate::cordic::CordicSinCos;
use crate::fault::G431Fault;
use crate::motor::MotorPwm;
use crate::sensors::{G431CurrentSensor, G431CurrentSensorExt, HallAngleProxy};
use crate::{FAULT_REGISTRY, STATE};

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
pub static ADC1_INJECTED: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, peripherals::ADC1, 3>>>,
> = CriticalSectionMutex::new(RefCell::new(None));
/// Handle for ADC2 injected conversions (TIM1-triggered).
pub static ADC2_INJECTED: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, peripherals::ADC2, 2>>>,
> = CriticalSectionMutex::new(RefCell::new(None));

// ========== FOC Control ==========

/// FOC driver storage (mutated only inside the ADC ISR)
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor>;
type FocDriverType =
    FocDriver<MotorPwm<'static>, G431CurrentSensor, PhaseManagerType, CordicSinCos>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Initialization ==========

/// Initialize FOC driver with motor PWM, sensors, and stored config.
pub async fn init(
    mut motor_pwm: MotorPwm<'static>,
    adc1: InjectedAdc<'static, peripherals::ADC1, 3>,
    adc2: InjectedAdc<'static, peripherals::ADC2, 2>,
    cordic_peri: Peri<'static, peripherals::CORDIC>,
    config: &RuntimeConfig,
) {
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();

    // Build current sensor with reconstruction (unipolar shunts, no Vref/2 bias)
    let current_sensor = G431CurrentSensor::from_board(&BOARD, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE);
    // Reconstruction disabled: bias network provides bidirectional sensing
    // on all three phases (~2573 counts at zero current). Reconstruction
    // would replace a valid measurement with a noisier computed value.
    // current_sensor.enable_reconstruction();
    crate::sensors::hall::apply_stored_config(config);
    let hall_proxy = HallAngleProxy::new();
    let phase_manager = PhaseManager::with_hall(hall_proxy);
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);

    // Initialize CORDIC hardware for fast sin/cos in FOC loop
    CordicSinCos::init(cordic_peri);

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
            FocController::<SvpwmModulator, CordicSinCos>::from_motor_params(
                mp.resistance_ohm,
                l_avg,
                initial_vbus_v,
            )
        } else {
            FocController::<SvpwmModulator, CordicSinCos>::new(initial_vbus_v)
        }
    } else if let Some(ref pg) = config.pi_gains {
        // No motor params but explicit PI gains stored
        let mut foc = FocController::<SvpwmModulator, CordicSinCos>::new(initial_vbus_v);
        foc.id_pi.set_gains(pg.kp, pg.ki);
        foc.iq_pi.set_gains(pg.kp, pg.ki);
        defmt::info!("Using stored PI gains: kp={=f32}, ki={=f32}", pg.kp, pg.ki);
        foc
    } else {
        FocController::<SvpwmModulator, CordicSinCos>::new(initial_vbus_v)
    };

    // Configure dead time compensation
    foc_controller.set_dead_time_comp(PWM_CONFIG.dead_time_ns, PWM_CONFIG.pwm_freq_hz);

    // Store ADC handles for ISR access (before enabling interrupt/PWM)
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc2)));

    // Enable ADC interrupt and PWM outputs.
    // Order: install handles → enable interrupt → enable PWM triggers.
    // ADC1_2 at priority 0 (highest) — FOC loop is the most time-critical ISR.
    // TIM6 (hall polling) runs at default priority and can be preempted by this.
    unsafe {
        use embassy_stm32::interrupt::typelevel::Interrupt;
        let irq = embassy_stm32::interrupt::ADC1_2;
        cortex_m::peripheral::NVIC::unmask(irq);
        cortex_m::peripheral::NVIC::set_priority(&mut cortex_m::Peripherals::steal().NVIC, irq, 0);
        <embassy_stm32::interrupt::typelevel::ADC1_2 as Interrupt>::unpend();
        <embassy_stm32::interrupt::typelevel::ADC1_2 as Interrupt>::enable();
    }
    motor_pwm.enable_outputs();

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

    // Allow ADC injected conversions to settle before zero-current calibration.
    defmt::info!("Waiting 10ms for ADC to settle...");
    Timer::after(Duration::from_millis(10)).await;

    // Diagnostic: dump raw ADC values before calibration
    defmt::info!(
        "Pre-cal raw ADC: ia={} ib={} ic={} vbus_mv={} temp_x10={}",
        IA_SAMPLE.load(Ordering::Relaxed),
        IB_SAMPLE.load(Ordering::Relaxed),
        IC_SAMPLE.load(Ordering::Relaxed),
        VBUS_MV.load(Ordering::Relaxed),
        FET_TEMP_C_X10.load(Ordering::Relaxed),
    );

    defmt::info!("Starting current sensor calibration...");
    foc_driver.current_sensor_mut().calibrate().await;
    defmt::info!("Current sensor calibration done");

    // Install FOC driver for ISR-only access.
    FOC_DRIVER.lock(|cell| {
        cell.replace(Some(foc_driver));
    });

    defmt::info!("FOC driver initialized and calibrated");
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

    // Detect hardware overcurrent break event (COMP → TIM1 BKIN cleared MOE)
    {
        let sr = embassy_stm32::pac::TIM1.sr().read();
        if sr.bif(0) {
            // Clear break interrupt flag
            embassy_stm32::pac::TIM1
                .sr()
                .modify(|w| w.set_bif(0, false));
            if !FAULT_REGISTRY.any() {
                defmt::error!("HW overcurrent FAULT: COMP triggered TIM1 BKIN");
            }
            FAULT_REGISTRY.set(G431Fault::OverCurrent);
        }
    }

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
    let had_fault = FAULT_REGISTRY.any();
    fault::check_voltage_faults(
        vbus_mv,
        &BOARD,
        &FAULT_REGISTRY,
        G431Fault::OverVoltage,
        G431Fault::UnderVoltage,
    );
    fault::check_temperature_fault(temp_c_x10, &BOARD, &FAULT_REGISTRY, G431Fault::OverTemp);
    if !had_fault && FAULT_REGISTRY.any() {
        // Log only once per fault episode (not every ISR cycle)
        static mut LAST_FAULT_SEQ: u32 = 0;
        if *SEQ > unsafe { LAST_FAULT_SEQ } + 1000 {
            defmt::error!("FAULT: vbus={}mV, temp={}", vbus_mv, temp_c_x10);
            unsafe { LAST_FAULT_SEQ = *SEQ };
        }
    }

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
            let prev_mode = driver.mode();
            let mode = oxifoc_core::state::process_commands(&STATE, driver);

            // Clear faults on transition from Stopped to active mode
            // (spurious COMP trips during PWM enable are expected)
            if matches!(prev_mode, ControlMode::Stopped) && mode != ControlMode::Stopped {
                FAULT_REGISTRY.clear_all();
            }

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
                    let before = FAULT_REGISTRY.any();
                    fault::check_current_faults(
                        telem.ia,
                        telem.ib,
                        telem.ic,
                        &BOARD,
                        &FAULT_REGISTRY,
                        G431Fault::OverCurrent,
                    );
                    if !before && FAULT_REGISTRY.any() {
                        defmt::error!(
                            "SW overcurrent FAULT: ia={}, ib={}, ic={}",
                            telem.ia,
                            telem.ib,
                            telem.ic
                        );
                    }
                    Some(telem)
                }
                Err(e) => {
                    defmt::error!("FOC step error: {}", e);
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
    // TODO: remove this fallback once motor PSU is connected for testing
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

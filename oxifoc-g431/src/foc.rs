//! FOC (Field-Oriented Control) management and ADC interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicI16, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{Peri, interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::clear_rc_w0;
use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::phase::PhaseManager;
use oxifoc_core::foc::pwm::SvpwmModulator;
use oxifoc_core::foc::sensors::NoSensor;
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::derating::DeratingConfig;
use oxifoc_core::motor::failsafe::FailsafeConfig;
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use oxifoc_core::state::run_foc_cycle;
use oxifoc_core::storage::RuntimeConfig;

use crate::safety::feed_watchdog;
use crate::sensors::hall;

use crate::config::{BOARD, NTC, PWM_CONFIG};
use crate::cordic::CordicSinCos;
use crate::motor::MotorPwm;
use crate::sensors::{G431CurrentSensor, G431CurrentSensorExt, HallAngleProxy};
use crate::{FAULT_REGISTRY, STATE};
use oxifoc_core::foc::fault::StandardFault;

// ========== ADC Sample Storage (Global Atomics) ==========

/// Latest phase current samples (from ADC1/ADC2 injected sequences).
pub static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

/// Latest measured DC bus voltage in millivolts (updated in ADC interrupt).
pub static VBUS_MV: AtomicU32 = AtomicU32::new(0);
/// Latest measured FET temperature in 0.1°C units (updated in ADC interrupt).
pub static FET_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);

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
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor, CordicSinCos>;
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
    hall::apply_stored_config(config);
    let hall_proxy = HallAngleProxy::new();
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);
    let mut phase_manager = PhaseManager::with_hall(hall_proxy).with_sincos::<CordicSinCos>();
    // Arm the sensorless estimators (back-EMF + HFI) from detected motor
    // params; the angle source stays Hall until the host switches it.
    phase_manager.configure_observers_from_config(config, initial_vbus_v);

    // Initialize CORDIC hardware for fast sin/cos in FOC loop
    CordicSinCos::init(cordic_peri);

    // Build FOC controller from stored config (motor params → PI gains → defaults)
    let mut foc_controller =
        FocController::<SvpwmModulator, CordicSinCos>::from_runtime_config(config, initial_vbus_v);

    // Configure dead time compensation
    foc_controller.set_dead_time_comp(PWM_CONFIG.dead_time_ns, PWM_CONFIG.pwm_freq_hz);

    // Store ADC handles for ISR access (before enabling interrupt/PWM)
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc2)));

    // Enable ADC interrupt and PWM outputs.
    // Order: install handles → enable interrupt → enable PWM triggers.
    // ADC1_2 at priority 0 (highest) — FOC loop is the most time-critical ISR.
    // TIM4 (hall capture) runs below it; edge timestamps are latched in
    // hardware, so that delay is harmless.
    #[expect(
        clippy::multiple_unsafe_ops_per_block,
        reason = "single logical operation: FOC ADC IRQ bring-up"
    )]
    // SAFETY: one-time IRQ bring-up during init, before the PWM trigger is
    // enabled (so the ISR cannot fire mid-setup); Peripherals::steal() only
    // touches NVIC priority registers nothing else owns at this point.
    unsafe {
        use embassy_stm32::interrupt::typelevel::Interrupt;
        let irq = interrupt::ADC1_2;
        cortex_m::peripheral::NVIC::unmask(irq);
        cortex_m::peripheral::NVIC::set_priority(&mut cortex_m::Peripherals::steal().NVIC, irq, 0);
        <interrupt::typelevel::ADC1_2 as Interrupt>::unpend();
        <interrupt::typelevel::ADC1_2 as Interrupt>::enable();
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

    // Current limits: stored config (clamped to the board ceiling) or board defaults
    foc_driver.set_current_limits(CurrentLimits::from_stored(
        config.current_limits.as_ref(),
        BOARD.max_phase_current_a,
        // Motor rating ceiling (detection's thermal solve), 0 = unknown.
        config
            .motor_params
            .as_ref()
            .and_then(oxifoc_core::storage::MotorParamsConfig::rating_current_a)
            .unwrap_or(0.0),
    ));

    // Failsafe: command-staleness deadman + reaction policy from stored config
    // (or board defaults); the OV trip feeds the regen-brake derate.
    foc_driver.set_failsafe(FailsafeConfig::from_stored(config.failsafe.as_ref()));
    foc_driver.set_ov_threshold(BOARD.max_vbus_mv as f32 / 1000.0);

    // Cruise velocity-loop tuning from stored config (or soft defaults).
    foc_driver.set_velocity_config(VelocityLoopConfig::from_stored(config.velocity.as_ref()));

    // Graduated derating ramps from stored config (default = FET thermal
    // rolloff only; see motor::derating).
    foc_driver.set_derating(DeratingConfig::from_stored(config.derating.as_ref()));

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
            // Clear the break flag (race-free rc_w0 complement write).
            clear_rc_w0!(embassy_stm32::pac::TIM1.sr(), |w| w.set_bif(0, false));
            if !FAULT_REGISTRY.any() {
                defmt::error!("HW overcurrent FAULT: COMP triggered TIM1 BKIN");
            }
            FAULT_REGISTRY.set(StandardFault::OverCurrent);
        }
    }

    // Local storage for ADC readings
    let mut ia_raw: u16 = 0;
    let mut ib_raw: u16 = 0;
    let mut ic_raw: u16 = 0;
    let mut vbus_mv: u32 = 0;
    let mut temp_c_x10: i16 = 0;

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
            temp_c_x10 = NTC.temp_c_x10_from_adc(samples[2], BOARD.adc_max_counts);
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

    // Voltage/temperature protection moved into core: run_foc_cycle's
    // run_protection covers them (with excursion integrators) for every
    // board.

    // Hall-domain timestamp (TIM4 µs ticks) for FOC and phase manager —
    // must match the tick domain of the hall edge timestamps.
    let now_ticks = hall::now_ticks();

    // Build ADC snapshot
    *SEQ = SEQ.wrapping_add(1);
    let adc_snapshot = AdcSnapshot::new(ia_raw, ib_raw, ic_raw, vbus_mv, *SEQ)
        .with_temp(TempSensorId::Fet, temp_c_x10);

    // Get Hall snapshot
    let hall_snapshot = hall::get_snapshot(now_ticks);

    // Run FOC control loop (shared cycle logic in core)
    let foc_telem = FOC_DRIVER.lock(|cell| {
        cell.borrow_mut().as_mut().and_then(|driver| {
            run_foc_cycle(
                &STATE,
                &FAULT_REGISTRY,
                driver,
                vbus_mv as f32 / 1000.0,
                now_ticks,
                &BOARD,
            )
        })
    });

    // Update global state + fast telemetry stream
    // TODO: remove this fallback once motor PSU is connected for testing
    publish_cycle_telemetry(
        &STATE,
        adc_snapshot,
        hall_snapshot,
        foc_telem.unwrap_or_default(),
        *SEQ,
    );

    // Feed the IWDG: a completed FOC cycle is the board's liveness signal.
    feed_watchdog();
}

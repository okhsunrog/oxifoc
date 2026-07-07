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
use core::sync::atomic::{AtomicI16, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
use oxifoc_core::foc::sensors::{AdcSnapshot, NoSensor, TempSensorId};
use oxifoc_core::foc::trig::FastSinCos;
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::derating::DeratingConfig;
use oxifoc_core::motor::failsafe::FailsafeConfig;
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use oxifoc_core::state::run_foc_cycle;
use oxifoc_core::storage::RuntimeConfig;

use crate::hardware::peripherals::AdcHandles;
use crate::safety::feed_watchdog;
use crate::sensors::hall;

use crate::config::{BOARD, NTC_BOARD, NTC_MOTOR, PWM_CONFIG};
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
pub static BOARD_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);
/// Latest motor temperature in 0.1°C units (updated in ADC interrupt).
pub static MOTOR_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);
static MOTOR_POLE_PAIRS: AtomicU8 = AtomicU8::new(0);

// ========== ISR cost instrumentation (DWT cycle counter) ==========
// Ported from the g431 (same layout so the isr/s log lines and the
// bench-suite parser stay identical across boards).

/// Sum of ADC ISR durations in CPU cycles since last stats swap.
pub static ISR_CYC_SUM: AtomicU32 = AtomicU32::new(0);
/// Per-section DWT cycle sums (reset each 1 Hz report; avg = sum / N).
/// adc = all three injected reads + NTC/vbus conversions, snap = snapshot
/// + hall sampling, foc = run_foc_cycle, pub = telemetry publish.
pub static ISR_PROF_ADC: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_SNAP: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_FOC: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_PUB: AtomicU32 = AtomicU32::new(0);
/// Max single ISR duration in CPU cycles since last stats swap.
pub static ISR_CYC_MAX: AtomicU32 = AtomicU32::new(0);
/// ISR cycles that exceeded the 8400-cycle 20 kHz budget (168 MHz).
pub static ISR_CYC_OVER: AtomicU32 = AtomicU32::new(0);
/// Number of ADC ISR executions since last stats swap.
pub static ISR_CYC_N: AtomicU32 = AtomicU32::new(0);

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
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor, FastSinCos>;
type FocDriverType = FocDriver<MotorPwm<'static>, F405CurrentSensor, PhaseManagerType, FastSinCos>;
static FOC_DRIVER: CriticalSectionMutex<RefCell<Option<FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== Initialization ==========

/// Initialize FOC driver with motor PWM, sensors, and stored config.
pub async fn init(
    mut motor_pwm: MotorPwm<'static>,
    adc_handles: AdcHandles,
    config: &RuntimeConfig,
) {
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();

    // Store ADC handles for ISR access (before enabling PWM triggers)
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc2)));
    ADC3_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc3)));
    MOTOR_POLE_PAIRS.store(
        config.motor_params.as_ref().map_or(0, |m| m.pole_pairs),
        Ordering::Relaxed,
    );

    // Enable ADC interrupt and PWM outputs (CH4 trigger + phase channels).
    // Order: install ADC handles → enable interrupt → enable PWM triggers.
    // ADC at priority 0 (highest) — the FOC loop is the actuator's most
    // time-critical ISR; comms ISRs (USB/UART) must never preempt or
    // jitter it (mirrors the G431 setup).
    #[expect(
        clippy::multiple_unsafe_ops_per_block,
        reason = "single logical operation: FOC ADC IRQ bring-up"
    )]
    // SAFETY: one-time IRQ bring-up during init, before the PWM trigger is
    // enabled (so the ISR cannot fire mid-setup); Peripherals::steal() only
    // touches NVIC priority registers nothing else owns at this point.
    unsafe {
        use embassy_stm32::interrupt::typelevel::Interrupt;
        // DWT cycle counter for ISR-cost stats (ISR_CYC_* atomics).
        let mut cp = cortex_m::Peripherals::steal();
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        let irq = interrupt::ADC;
        cortex_m::peripheral::NVIC::set_priority(&mut cortex_m::Peripherals::steal().NVIC, irq, 0);
        <interrupt::typelevel::ADC as Interrupt>::unpend();
        <interrupt::typelevel::ADC as Interrupt>::enable();
    }
    motor_pwm.enable_outputs();

    // Build current sensor and phase manager
    let current_sensor = F405CurrentSensor::from_board(&BOARD, &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE);
    hall::apply_stored_config(config);
    let hall_proxy = HallAngleProxy::new();
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);
    let mut phase_manager = PhaseManager::with_hall(hall_proxy).with_sincos::<FastSinCos>();
    // Arm the sensorless estimators (back-EMF + HFI) from detected motor
    // params; the angle source stays Hall until the host switches it.
    phase_manager.configure_observers_from_config(config, initial_vbus_v);

    // Boot angle source is CONFIG-driven (this board persists config, so no
    // compile-time SENSORLESS switch like the g431): a stored hall
    // calibration means a sensored motor — keep the Hall boot. No hall
    // calibration means sensorless — boot on the back-EMF observer once
    // detection has filled motor params (before that, Manual: commutation
    // stays inert until the host drives it, and unwired hall inputs never
    // get consulted, so they can't spam HallError).
    if config.hall_calibration.is_none() {
        let boot_source = if config.motor_params.is_some() {
            PhaseSource::Observer
        } else {
            PhaseSource::Manual
        };
        let applied = if phase_manager.set_source(boot_source).is_err() {
            // Observer rejects a PARTIAL param set (R/L present, flux step
            // never ran). Fall back to Manual, not Hall — same rationale as
            // the g431 sensorless boot.
            defmt::warn!(
                "sensorless boot source rejected (partial motor params?); falling back to Manual"
            );
            let _ = phase_manager.set_source(PhaseSource::Manual);
            PhaseSource::Manual
        } else {
            boot_source
        };
        // Mirror into shared state: the runtime only syncs phase_source on
        // host-commanded changes (DriverCommand::SetPhaseSource), so without
        // this the status endpoint reports the struct default (Hall) forever.
        critical_section::with(|cs| {
            STATE.borrow(cs).borrow_mut().phase_source = applied;
        });
    }
    if config.motor_params.is_some() {
        // Commutation phase tracker: 2nd-order PLL on the observer angle
        // (ωn/ζ carried over from the g431 ZD2808 bench, 2026-07-07 —
        // see docs/decisions.md "minimal load-bearing set"). The accel
        // prior is deliberately NOT set here: its per_amp is a per-motor
        // measurement (free-rotor spin-up) that has not been taken on
        // this board yet.
        phase_manager.set_phase_tracker(100.0, 1.2);
    }

    // Build FOC controller from stored config (motor params → PI gains → defaults)
    let mut foc_controller =
        FocController::<_, FastSinCos>::from_runtime_config(config, initial_vbus_v);

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

    // Allow ADC injected conversions to start firing before zero-current calibration.
    Timer::after(Duration::from_millis(10)).await;
    foc_driver.current_sensor_mut().calibrate().await;

    // Publish the measured zero-current offsets into the DcOffsets config
    // group: the boot measurement is the ground truth for this power-up, and
    // the host's telemetry enrichment reads this group (falling back to
    // mid-scale counts when it's absent, which shifts reconstructed phase
    // currents by amps).
    {
        let (oa, ob, oc) = foc_driver.current_sensor().converter().get_offsets();
        critical_section::with(|cs| {
            crate::RUNTIME_CONFIG.borrow(cs).borrow_mut().dc_offsets =
                Some(oxifoc_core::storage::DcOffsetsConfig {
                    phase_a: oa,
                    phase_b: ob,
                    phase_c: oc,
                });
        });
    }

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

    let isr_t0 = cortex_m::peripheral::DWT::cycle_count();

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
    let board_temp_c_x10 =
        NTC_BOARD.temp_c_x10_from_adc(board_temp_raw, BOARD.calib.adc_max_counts);
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
    let motor_temp_c_x10 =
        NTC_MOTOR.temp_c_x10_from_adc(motor_temp_raw, BOARD.calib.adc_max_counts);
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

    let prof_t1 = cortex_m::peripheral::DWT::cycle_count();

    // Voltage/temperature protection moved into core: run_foc_cycle's
    // run_protection covers them (with excursion integrators) for every
    // board — incl. the motor winding NTC, which reaches it through the
    // AdcSnapshot in shared state.

    // Get current timestamp for FOC and phase manager
    // Hall-domain timestamp (capture-timer us ticks) for FOC and phase
    // manager - must match the tick domain of the hall edge timestamps.
    let now_ticks = hall::now_ticks();

    // Build ADC snapshot
    *SEQ = SEQ.wrapping_add(1);
    let adc_snapshot = AdcSnapshot::new(ia_raw, ib_raw, ic_raw, vbus_mv, *SEQ)
        .with_temp(TempSensorId::Fet, board_temp_c_x10)
        .with_temp(TempSensorId::Motor, motor_temp_c_x10);

    // Get Hall snapshot
    let hall_snapshot = hall::get_snapshot(now_ticks);

    let prof_t2 = cortex_m::peripheral::DWT::cycle_count();

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

    let prof_t3 = cortex_m::peripheral::DWT::cycle_count();

    // Update global state + fast telemetry stream
    publish_cycle_telemetry(
        &STATE,
        adc_snapshot,
        hall_snapshot,
        foc_telem.unwrap_or_default(),
        MOTOR_POLE_PAIRS.load(Ordering::Relaxed),
        *SEQ,
    );

    let prof_t4 = cortex_m::peripheral::DWT::cycle_count();

    // Feed the IWDG: a completed FOC cycle is the board's liveness signal.
    feed_watchdog();

    ISR_PROF_ADC.fetch_add(prof_t1.wrapping_sub(isr_t0), Ordering::Relaxed);
    ISR_PROF_SNAP.fetch_add(prof_t2.wrapping_sub(prof_t1), Ordering::Relaxed);
    ISR_PROF_FOC.fetch_add(prof_t3.wrapping_sub(prof_t2), Ordering::Relaxed);
    ISR_PROF_PUB.fetch_add(prof_t4.wrapping_sub(prof_t3), Ordering::Relaxed);

    let isr_dt = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(isr_t0);
    ISR_CYC_SUM.fetch_add(isr_dt, Ordering::Relaxed);
    ISR_CYC_MAX.fetch_max(isr_dt, Ordering::Relaxed);
    ISR_CYC_N.fetch_add(1, Ordering::Relaxed);
    // Budget-overrun counter: cycles that ate the whole 8400-cycle period
    // (168 MHz / 20 kHz) — thread mode got nothing.
    if isr_dt > 8_400 {
        ISR_CYC_OVER.fetch_add(1, Ordering::Relaxed);
    }
}

/// 1 Hz ISR-cost stats — same line format as the g431 so the bench-suite
/// parser works unchanged: `isr/s: n= avg= max= over= load_pct=` plus the
/// per-section and core-bucket breakdowns.
#[embassy_executor::task]
pub async fn isr_stats_task() {
    use embassy_time::Ticker;
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        let cyc_sum = ISR_CYC_SUM.swap(0, Ordering::Relaxed);
        let cyc_max = ISR_CYC_MAX.swap(0, Ordering::Relaxed);
        let cyc_n = ISR_CYC_N.swap(0, Ordering::Relaxed);
        if cyc_n == 0 {
            continue;
        }
        let adc = ISR_PROF_ADC.swap(0, Ordering::Relaxed) / cyc_n;
        let sn = ISR_PROF_SNAP.swap(0, Ordering::Relaxed) / cyc_n;
        let fo = ISR_PROF_FOC.swap(0, Ordering::Relaxed) / cyc_n;
        let pb = ISR_PROF_PUB.swap(0, Ordering::Relaxed) / cyc_n;
        let avg = cyc_sum / cyc_n;
        defmt::info!(
            "isr/s: n={} avg={} max={} over={} load_pct={}",
            cyc_n,
            avg,
            cyc_max,
            ISR_CYC_OVER.swap(0, Ordering::Relaxed),
            cyc_sum / 1_680_000, // 168 MHz → percent of one second
        );
        defmt::info!(
            "isrp/s: adc={} snap={} foc={} pub={} tail={}",
            adc,
            sn,
            fo,
            pb,
            avg.saturating_sub(adc + sn + fo + pb)
        );
        // run_foc_cycle internals (core isr_prof buckets) — same lines as
        // the g431 protocol stats task.
        {
            use oxifoc_core::isr_prof as p;
            defmt::info!(
                "isrc/s: cmd={} prot={} step={} ctail={} | stopped: pwmoff={} phase={}",
                p::CYCLE_CMD.swap(0, Ordering::Relaxed) / cyc_n,
                p::CYCLE_PROT.swap(0, Ordering::Relaxed) / cyc_n,
                p::CYCLE_STEP.swap(0, Ordering::Relaxed) / cyc_n,
                p::CYCLE_TAIL.swap(0, Ordering::Relaxed) / cyc_n,
                p::STEP_PWMOFF.swap(0, Ordering::Relaxed) / cyc_n,
                p::STEP_PHASE.swap(0, Ordering::Relaxed) / cyc_n,
            );
            defmt::info!(
                "isrd/s: gate={} (curr={}) ctrl={} (trig={}) post={} est={}",
                p::STEP_GATE.swap(0, Ordering::Relaxed) / cyc_n,
                p::GATE_CURR.swap(0, Ordering::Relaxed) / cyc_n,
                p::STEP_CTRL.swap(0, Ordering::Relaxed) / cyc_n,
                p::CTRL_TRIG.swap(0, Ordering::Relaxed) / cyc_n,
                p::STEP_POST.swap(0, Ordering::Relaxed) / cyc_n,
                p::STEP_EST.swap(0, Ordering::Relaxed) / cyc_n,
            );
            defmt::info!(
                "isre/s: obs={} (otail={}) startup={} out={} (trk={})",
                p::EST_OBS.swap(0, Ordering::Relaxed) / cyc_n,
                p::OBS_TAIL.swap(0, Ordering::Relaxed) / cyc_n,
                p::EST_STARTUP.swap(0, Ordering::Relaxed) / cyc_n,
                p::EST_OUT.swap(0, Ordering::Relaxed) / cyc_n,
                p::OUT_TRK.swap(0, Ordering::Relaxed) / cyc_n,
            );
        }
    }
}

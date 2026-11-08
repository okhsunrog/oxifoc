//! FOC (Field-Oriented Control) management and ADC interrupt handling for F405
//!
//! # ADC Configuration
//!
//! All ADC sampling via TIM1-triggered injected conversions (no DMA for currents).
//! - ADC1: current A (PC0) + board temp (PA3) + voltage A (PA0)
//! - ADC2: current B (PC1) + motor temp (PC4) + voltage B (PA1)
//! - ADC3: current C (PC2) + VBUS (PC3) + voltage C (PA2)
//!
//! Triggered by TIM1_CH4 compare event (at PWM center).

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicI16, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::interrupt;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::current_offset::CurrentOffsetRequest;
use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
use oxifoc_core::foc::sensors::{AdcSnapshot, NoSensor, PwmSnapshot, TempSensorId};
use oxifoc_core::foc::trig::FastSinCos;
use oxifoc_core::foc::velocity::VelocityLoopConfig;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::derating::DeratingConfig;
use oxifoc_core::motor::failsafe::FailsafeConfig;
use oxifoc_core::motor::foc_driver::CurrentLimits;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use oxifoc_core::state::{
    BOOT_CURRENT_OFFSET_PENDING, CURRENT_OFFSET_DONE, DriverOperation, run_foc_cycle,
};
use oxifoc_core::storage::RuntimeConfig;

use crate::hardware::peripherals::AdcHandles;
use crate::safety::feed_watchdog;
use crate::sensors::hall;

use crate::config::{BOARD, CPU_HZ, NTC_BOARD, NTC_MOTOR, measured_board};
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
/// Set by the FOC ISR once `VBUS_MV` holds a real conversion — `init` waits
/// on this so the observers/controller are configured from a MEASURED bus
/// voltage, never a compile-time assumption (a measured 0 V is still the
/// truth; `FocController` floors vbus at `MIN_VBUS` internally).
static VBUS_MEASURED: AtomicBool = AtomicBool::new(false);
/// Latest board temperature in 0.1°C units (updated in ADC interrupt).
pub static BOARD_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);
/// Latest motor temperature in 0.1°C units (updated in ADC interrupt).
pub static MOTOR_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);
static MOTOR_POLE_PAIRS: AtomicU8 = AtomicU8::new(0);

/// DIAG TEMP: raw motor-NTC ADC counts (PC4) for the hall-connector hunt.
pub static MOTOR_TEMP_RAW: AtomicU16 = AtomicU16::new(0);

/// DIAG TEMP: per-port IDR activity masks (bits that changed since the
/// previous FOC cycle, OR-accumulated per stats window) — pin-hunting for
/// the hall inputs. Ports A, B, C, D.
pub static PIN_ACT: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];
static PIN_PREV: [AtomicU32; 4] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

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
/// ISR executions that exceeded [`ISR_BUDGET_CYCLES`].
pub static ISR_CYC_OVER: AtomicU32 = AtomicU32::new(0);
/// Per-ISR cycle budget: one full PWM period of core cycles
/// (168 MHz / 20 kHz = 8400).
static ISR_BUDGET_CYCLES: AtomicU32 = AtomicU32::new(0);
/// Number of ADC ISR executions since last stats swap.
pub static ISR_CYC_N: AtomicU32 = AtomicU32::new(0);
// Entry-to-entry timing is independent of the software FOC sequence counter.
// Execution duration alone cannot expose an interrupt that started late or a
// conversion whose interrupt was never serviced. These counters diagnose that
// distinction; a long interval is not by itself a proven missed ADC trigger.
static ISR_INTERVAL_SUM: AtomicU32 = AtomicU32::new(0);
static ISR_INTERVAL_MIN: AtomicU32 = AtomicU32::new(u32::MAX);
static ISR_INTERVAL_MAX: AtomicU32 = AtomicU32::new(0);
static ISR_INTERVAL_N: AtomicU32 = AtomicU32::new(0);
static ISR_INTERVAL_LONG: AtomicU32 = AtomicU32::new(0);
static ISR_INTERVAL_SHORT: AtomicU32 = AtomicU32::new(0);

// ========== ADC Handles ==========

/// ADC1 injected handle: phase A current + board temperature
pub static ADC1_INJECTED: CriticalSectionMutex<
    RefCell<
        Option<InjectedAdc<'static, embassy_stm32::pac::adc::Adc, embassy_stm32::mode::Blocking>>,
    >,
> = CriticalSectionMutex::new(RefCell::new(None));
/// ADC2 injected handle: phase B current + motor temperature
pub static ADC2_INJECTED: CriticalSectionMutex<
    RefCell<
        Option<InjectedAdc<'static, embassy_stm32::pac::adc::Adc, embassy_stm32::mode::Blocking>>,
    >,
> = CriticalSectionMutex::new(RefCell::new(None));
/// ADC3 injected handle: phase C current + VBUS
pub static ADC3_INJECTED: CriticalSectionMutex<
    RefCell<
        Option<InjectedAdc<'static, embassy_stm32::pac::adc::Adc, embassy_stm32::mode::Blocking>>,
    >,
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
    let timing = motor_pwm.control_timing();
    let budget =
        (u64::from(CPU_HZ) * u64::from(timing.period_ticks) / u64::from(timing.clock_hz)) as u32;
    ISR_BUDGET_CYCLES.store(budget, Ordering::Relaxed);

    // Store ADC handles for ISR access (before enabling PWM triggers)
    ADC1_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc1)));
    ADC2_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc2)));
    ADC3_INJECTED.lock(|cell| cell.replace(Some(adc_handles.adc3)));
    MOTOR_POLE_PAIRS.store(
        config.motor_params.as_ref().map_or(0, |m| m.pole_pairs),
        Ordering::Relaxed,
    );

    // Enable the ADC interrupt and its internal PWM trigger. Phase channels
    // stay high-Z until the calibration state machine selects one.
    // Order: install ADC handles → enable interrupt → enable PWM triggers.
    // ADC at priority 0 (highest) — the FOC loop is the actuator's most
    // time-critical ISR; comms ISRs (USB/UART) must never preempt or
    // jitter it (mirrors the G431 setup; main.rs demotes every other
    // vector below the control ISRs for exactly this reason).
    //
    // DELIBERATE: the ISR free-runs with FOC_DRIVER == None until the
    // driver is installed below — first_vbus_v() and the boot current-
    // offset calibration are fed by the ISR's atomics, so it MUST start
    // first. The window is benign by construction: run_foc_cycle is
    // skipped (None), the bridge stays high-Z, host commands are rejected
    // by BOOT_CURRENT_OFFSET_PENDING and bounded by the motor server's
    // ack timeout, and publish_cycle_telemetry emits a default FocOutput
    // (streaming is off until the host enables it anyway).
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
    motor_pwm.enable_adc_trigger();

    // Build current sensor and phase manager
    let current_sensor =
        F405CurrentSensor::from_board(measured_board(), &IA_SAMPLE, &IB_SAMPLE, &IC_SAMPLE);
    hall::apply_stored_config(config);
    let hall_proxy = HallAngleProxy::new();
    let initial_vbus_v = first_vbus_v().await;
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
    if !config
        .hall_calibration
        .as_ref()
        .is_some_and(oxifoc_core::storage::HallCalibrationConfig::is_calibrated)
    {
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
    foc_controller
        .set_dead_time_comp_factor(crate::config::DEAD_TIME_COMP_NS as f32 * 1e-9 / timing.dt_s());
    defmt::unwrap!(
        foc_controller
            .set_dead_time_comp_linear_region(crate::config::DEAD_TIME_COMP_LINEAR_CURRENT_A)
    );

    // Use the same timer-derived period sent to the host and detection backend.
    let mut foc_driver = FocDriver::new(
        foc_controller,
        motor_pwm,
        current_sensor,
        phase_manager,
        timing.dt_s(),
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

    CURRENT_OFFSET_DONE.reset();
    let calibration_started = foc_driver
        .start_current_offset_calibration(CurrentOffsetRequest::boot())
        .is_ok();
    if calibration_started {
        critical_section::with(|cs| {
            STATE.borrow(cs).borrow_mut().driver_operation =
                DriverOperation::CurrentOffsetCalibration;
        });
    } else {
        defmt::error!("failed to start boot current-offset calibration");
    }

    // Install FOC driver for ISR-only access; it now advances calibration.
    FOC_DRIVER.lock(|cell| {
        cell.replace(Some(foc_driver));
    });

    if calibration_started {
        match CURRENT_OFFSET_DONE.wait().await {
            Ok(report) => {
                critical_section::with(|cs| {
                    crate::RUNTIME_CONFIG.borrow(cs).borrow_mut().dc_offsets =
                        Some(oxifoc_core::storage::DcOffsetsConfig {
                            phase_a: report.offsets[0],
                            phase_b: report.offsets[1],
                            phase_c: report.offsets[2],
                        });
                });
            }
            Err(_) => defmt::error!("current-offset calibration failed"),
        }
    }
    BOOT_CURRENT_OFFSET_PENDING.store(false, Ordering::Release);

    defmt::info!("F405 FOC driver initialized and calibrated");
}

/// Map duty percent to target q-axis current
pub fn duty_to_iq(duty: u8) -> f32 {
    BOARD.duty_to_iq(duty)
}

/// Wait for the FOC ISR to publish its first VBUS conversion and return it
/// in volts. The ADC trigger + interrupt are already armed when this runs,
/// so the first sample lands within one PWM period (~50 µs); the 100 ms
/// ceiling only trips if the ADC/ISR pipeline is dead — motor control is
/// impossible then anyway, so log loudly and return 0 (the controller
/// floors vbus at MIN_VBUS and the board stays alive for host debugging).
async fn first_vbus_v() -> f32 {
    for _ in 0..1000 {
        if VBUS_MEASURED.load(Ordering::Acquire) {
            return VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0;
        }
        Timer::after(Duration::from_micros(100)).await;
    }
    defmt::error!("no VBUS measurement within 100ms - ADC/ISR pipeline dead?");
    0.0
}

// ========== ADC Interrupt Handler ==========

/// ADC interrupt: read all injected ADC samples and run FOC control.
///
/// Triggered by ADC3 JEOC (end of injected conversion sequence).
/// ADC1, ADC2, ADC3 all start conversion simultaneously on TIM1_CC4.
#[interrupt]
fn ADC() {
    static mut SEQ: u32 = 0;
    static mut LAST_START: u32 = 0;

    let isr_t0 = cortex_m::peripheral::DWT::cycle_count();
    #[cfg(feature = "bench-executor-trace")]
    crate::diagnostics::trace::record(8, 0);
    if *LAST_START != 0 {
        let interval = isr_t0.wrapping_sub(*LAST_START);
        ISR_INTERVAL_SUM.fetch_add(interval, Ordering::Relaxed);
        ISR_INTERVAL_MIN.fetch_min(interval, Ordering::Relaxed);
        ISR_INTERVAL_MAX.fetch_max(interval, Ordering::Relaxed);
        ISR_INTERVAL_N.fetch_add(1, Ordering::Relaxed);
        if interval > ISR_BUDGET_CYCLES.load(Ordering::Relaxed) * 3 / 2 {
            ISR_INTERVAL_LONG.fetch_add(1, Ordering::Relaxed);
        }
        if interval < ISR_BUDGET_CYCLES.load(Ordering::Relaxed) / 2 {
            ISR_INTERVAL_SHORT.fetch_add(1, Ordering::Relaxed);
        }
    }
    *LAST_START = isr_t0;
    let board = measured_board();

    // NTC Beta-model conversion runs libm::logf + several float divides
    // (~200 cycles per sensor, two sensors here = ~5-7% of the cycle
    // budget) — decimate to every 128th cycle (156 Hz); the thermal time
    // constants are seconds, and consumers read the atomics. Same scheme
    // as the g431 ISR.
    let convert_temp = *SEQ & 127 == 0;

    // Read ADC1 injected data (phase A current + board temp)
    let (ia_raw, board_temp_raw, va_raw) = ADC1_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let mut samples = [0; 3];
            injected.read_latest(&mut samples);
            (samples[0], samples[1], samples[2])
        } else {
            (0, 0, 0)
        }
    });
    IA_SAMPLE.store(ia_raw, Ordering::Relaxed);

    // Convert board temperature raw ADC to 0.1°C units
    let board_temp_c_x10 = if convert_temp {
        let t = NTC_BOARD.temp_c_x10_from_adc(board_temp_raw, board.calib.adc_max_counts);
        BOARD_TEMP_C_X10.store(t, Ordering::Relaxed);
        t
    } else {
        BOARD_TEMP_C_X10.load(Ordering::Relaxed)
    };

    // Read ADC2 injected data (phase B current + motor temp)
    let (ib_raw, motor_temp_raw, vb_raw) = ADC2_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let mut samples = [0; 3];
            injected.read_latest(&mut samples);
            (samples[0], samples[1], samples[2])
        } else {
            (0, 0, 0)
        }
    });
    IB_SAMPLE.store(ib_raw, Ordering::Relaxed);

    MOTOR_TEMP_RAW.store(motor_temp_raw, Ordering::Relaxed);
    // Convert motor temperature raw ADC to 0.1°C units
    let motor_temp_c_x10 = if convert_temp {
        let t = NTC_MOTOR.temp_c_x10_from_adc(motor_temp_raw, board.calib.adc_max_counts);
        MOTOR_TEMP_C_X10.store(t, Ordering::Relaxed);
        t
    } else {
        MOTOR_TEMP_C_X10.load(Ordering::Relaxed)
    };

    // Read ADC3 injected data (phase C current + VBUS)
    let (ic_raw, vbus_raw, vc_raw) = ADC3_INJECTED.lock(|cell| {
        if let Some(injected) = cell.borrow_mut().as_mut() {
            let mut samples = [0; 3];
            injected.read_latest(&mut samples);
            (samples[0], samples[1], samples[2])
        } else {
            (0, 0, 0)
        }
    });
    IC_SAMPLE.store(ic_raw, Ordering::Relaxed);

    // Convert VBUS raw ADC to millivolts
    let vbus_mv = oxifoc_core::state::calibrated_vbus_mv(&STATE, board.vbus_mv_from_adc(vbus_raw));
    VBUS_MV.store(vbus_mv, Ordering::Relaxed);
    VBUS_MEASURED.store(true, Ordering::Release);

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
    // Read the compare/preload registers BEFORE run_foc_cycle can program
    // the next vector. Do not pair ADC measurements with that next command.
    let tim = embassy_stm32::pac::TIM1;
    let ccer = tim.ccer().read();
    let mut enables = u16::from(tim.bdtr().read().moe()) << 6;
    for phase in 0..3 {
        enables |= u16::from(ccer.cce(phase)) << phase;
        enables |= u16::from(ccer.ccne(phase)) << (phase + 3);
    }
    let pwm_snapshot = PwmSnapshot {
        ccr: core::array::from_fn(|i| tim.ccr(i).read().ccr()),
        arr: tim.arr().read().arr(),
        enables,
    };
    let adc_snapshot = AdcSnapshot::new(ia_raw, ib_raw, ic_raw, vbus_mv, *SEQ)
        .with_phase_voltages([va_raw, vb_raw, vc_raw])
        .with_pwm(pwm_snapshot)
        .with_temp(TempSensorId::Fet, board_temp_c_x10)
        .with_temp(TempSensorId::Motor, motor_temp_c_x10);

    // Get Hall snapshot
    let hall_snapshot = hall::get_snapshot(now_ticks);

    let prof_t2 = cortex_m::peripheral::DWT::cycle_count();

    // Run FOC control loop (shared cycle logic in core)
    let foc_telem = FOC_DRIVER.lock(|cell| {
        cell.borrow_mut().as_mut().and_then(|driver| {
            let output = run_foc_cycle(
                &STATE,
                &FAULT_REGISTRY,
                driver,
                vbus_mv as f32 / 1000.0,
                now_ticks,
                board,
            );
            #[cfg(feature = "bench-executor-trace")]
            if !matches!(driver.mode(), oxifoc_core::types::ControlMode::Stopped) {
                crate::diagnostics::trace::trigger_active();
            } else {
                crate::diagnostics::trace::trigger_stopped();
            }
            output
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

    #[cfg(feature = "bench-executor-trace")]
    crate::diagnostics::trace::record(9, 0);
    let isr_dt = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(isr_t0);
    ISR_CYC_SUM.fetch_add(isr_dt, Ordering::Relaxed);
    ISR_CYC_MAX.fetch_max(isr_dt, Ordering::Relaxed);
    ISR_CYC_N.fetch_add(1, Ordering::Relaxed);
    // Budget-overrun counter: an ISR that ate the whole PWM period —
    // thread mode got nothing.
    if isr_dt > ISR_BUDGET_CYCLES.load(Ordering::Relaxed) {
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
    #[cfg(feature = "transport-usb")]
    let mut usb_clock_previous = None;
    #[cfg(feature = "transport-usb")]
    let mut usb_clock_totals = (0u64, 0u64);
    loop {
        ticker.next().await;
        #[cfg(feature = "transport-usb")]
        if crate::protocol::USB_IDENT.is_set() {
            // DSTS is read-only. SOF frame numbers come from the USB host,
            // providing a clock independent of TIM1/TIM2/DWT on this board.
            // FS frames have an 11-bit frame number, wrapping every 2048 ms.
            let frame = embassy_stm32::pac::USB_OTG_FS.dsts().read().fnsof() & 0x7ff;
            let cycles = cortex_m::peripheral::DWT::cycle_count();
            if let Some((previous_frame, previous_cycles)) = usb_clock_previous {
                let frames = frame.wrapping_sub(previous_frame) & 0x7ff;
                let elapsed = cycles.wrapping_sub(previous_cycles);
                // Reject suspend/reset/long reporting gaps rather than infer
                // missing wraps or fold them into the reference-clock fit.
                if (900..=1100).contains(&frames)
                    && (CPU_HZ * 4 / 5..=CPU_HZ * 6 / 5).contains(&elapsed)
                {
                    usb_clock_totals.0 += u64::from(frames);
                    usb_clock_totals.1 += u64::from(elapsed);
                    defmt::info!(
                        "usbclk/s: frames={} cycles={} total_frames={} total_cycles={} avg_cycles_per_frame={}",
                        frames,
                        elapsed,
                        usb_clock_totals.0,
                        usb_clock_totals.1,
                        usb_clock_totals.1 / usb_clock_totals.0,
                    );
                } else {
                    usb_clock_totals = (0, 0);
                }
            }
            usb_clock_previous = Some((frame, cycles));
        }
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
            cyc_sum / (CPU_HZ / 100), // cycles per 1% of a second
        );
        defmt::info!(
            "isrp/s: adc={} snap={} foc={} pub={} tail={}",
            adc,
            sn,
            fo,
            pb,
            avg.saturating_sub(adc + sn + fo + pb)
        );
        // Swap the timing tuple atomically relative to the ADC IRQ so average
        // and extrema describe one window. Logging stays outside the section.
        let (interval_n, interval_sum, interval_min, interval_max, long, short) =
            critical_section::with(|_| {
                (
                    ISR_INTERVAL_N.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_SUM.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_MIN.swap(u32::MAX, Ordering::Relaxed),
                    ISR_INTERVAL_MAX.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_LONG.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_SHORT.swap(0, Ordering::Relaxed),
                )
            });
        if interval_n != 0 {
            defmt::info!(
                "isri/s: n={} avg={} min={} max={} long={} short={} expected={}",
                interval_n,
                interval_sum / interval_n,
                interval_min,
                interval_max,
                long,
                short,
                ISR_BUDGET_CYCLES.load(Ordering::Relaxed),
            );
        }
        use oxifoc_core::runtime::streaming::fast_telem_stats as ft;
        let pushed = ft::PUSH_OK.swap(0, Ordering::Relaxed);
        let dropped = ft::PUSH_DROPS.swap(0, Ordering::Relaxed);
        let read = ft::READ_OK.swap(0, Ordering::Relaxed);
        let badlen = ft::READ_BADLEN.swap(0, Ordering::Relaxed);
        let batches = ft::BCAST_OK.swap(0, Ordering::Relaxed);
        let payload_bytes = ft::BCAST_BYTES.swap(0, Ordering::Relaxed);
        let failed = ft::BCAST_FAILS.swap(0, Ordering::Relaxed);
        if pushed != 0 || dropped != 0 || read != 0 || failed != 0 {
            defmt::info!(
                "telem/s: push={} drop={} read={} badlen={} batches={} failed={} payload_bytes={}",
                pushed,
                dropped,
                read,
                badlen,
                batches,
                failed,
                payload_bytes
            );
        }
        // Hall diagnostics: raw pin state + edge/overcapture rates. A
        // sensored motor at standstill shows a constant valid state (1-6)
        // and 0 edges; 0b111/0b000 = unpowered/disconnected sensors.
        // Hall + motor-NTC diagnostics (raw GPIO/ADC — the honest bottom
        // of the stack). A sensored motor at standstill shows a constant
        // VALID state (1-6) and 0 edges; 0b111 = unpowered/disconnected
        // sensors; t_raw 4095 = open TEMP line. Flaky hall wiring shows as
        // state flips + edge bursts synchronized with cable movement
        // (Flipsky connector hunt, 2026-07-08).
        defmt::info!(
            "hall/s: state={=u8:b} edges={} overcap={} t_raw={}",
            hall::read_hall_state_raw(),
            hall::EDGES.swap(0, Ordering::Relaxed),
            hall::OVERCAPTURES.swap(0, Ordering::Relaxed),
            MOTOR_TEMP_RAW.load(Ordering::Relaxed),
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

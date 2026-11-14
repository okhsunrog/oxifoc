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
use core::sync::atomic::{AtomicBool, AtomicI16, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::interrupt;
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::current_offset::CurrentOffsetRequest;
use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
use oxifoc_core::foc::sensors::{AdcSnapshot, NoSensor, PwmSnapshot, TempSensorId};
use oxifoc_core::foc::trig::FastSinCos;
use oxifoc_core::motor::FocDriver;
use oxifoc_core::motor::failsafe::FailsafeConfig;
use oxifoc_core::runtime::streaming::publish_cycle_telemetry;
use oxifoc_core::state::{
    BOOT_CURRENT_OFFSET_PENDING, CURRENT_OFFSET_DONE, DriverOperation, run_foc_cycle,
};
use oxifoc_core::storage::RuntimeConfig;
#[cfg(feature = "board-vesc6-mk5")]
use oxifoc_core::types::ControlTiming;

use crate::hardware::peripherals::AdcHandles;
use crate::safety::feed_watchdog;
use crate::sensors::{hall, temperature};

use crate::config::{BOARD, CPU_HZ, measured_board, prepared_board};
use crate::motor::MotorPwm;
use crate::sensors::{F405CurrentSensor, F405CurrentSensorExt, hall::HallAngleProxy};
use crate::{FAULT_REGISTRY, STATE};

#[path = "profiling_window.rs"]
pub(crate) mod profiling_window;
use profiling_window::{DwtSample, DwtWindow};

// ========== ADC Sample Storage (Global Atomics) ==========

/// Latest phase current samples (from ADC1/ADC2/ADC3 injected sequences).
pub static IA_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IB_SAMPLE: AtomicU16 = AtomicU16::new(0);
pub static IC_SAMPLE: AtomicU16 = AtomicU16::new(0);

// The bounded ADC conversion cannot produce this sentinel.
const NO_VBUS_SAMPLE: u32 = u32::MAX;
static VBUS_MV: AtomicU32 = AtomicU32::new(NO_VBUS_SAMPLE);

/// Latest calibrated bus voltage in millivolts, once the ADC has sampled it.
pub(crate) fn measured_vbus_mv() -> Option<u32> {
    let mv = VBUS_MV.load(Ordering::Relaxed);
    (mv != NO_VBUS_SAMPLE).then_some(mv)
}

/// Latest board temperature in 0.1°C units (updated in ADC interrupt).
pub static BOARD_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);
/// Latest motor temperature in 0.1°C units (updated in ADC interrupt).
pub static MOTOR_TEMP_C_X10: AtomicI16 = AtomicI16::new(0);

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
// Split active current/velocity loops with a nonzero control-speed estimate
// from stationary holds. This selects a computational path, not physical
// rotation evidence. Bookkeeping follows the ISR end timestamp.
#[cfg(feature = "isr-timing")]
mod motion_timing {
    use super::*;
    static N: AtomicU32 = AtomicU32::new(0);
    static SUM: AtomicU32 = AtomicU32::new(0);
    static MAX: AtomicU32 = AtomicU32::new(0);

    pub fn record(cycles: u32) {
        N.fetch_add(1, Ordering::Relaxed);
        SUM.fetch_add(cycles, Ordering::Relaxed);
        MAX.fetch_max(cycles, Ordering::Relaxed);
    }

    // Called in the same existing stats CS as the whole-ISR counters.
    pub fn take() -> (u32, u32, u32) {
        (
            N.swap(0, Ordering::Relaxed),
            SUM.swap(0, Ordering::Relaxed),
            MAX.swap(0, Ordering::Relaxed),
        )
    }
}
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
// The stats CS resets this baseline when DWT was stopped by a debugger.
// Relaxed loads/stores are the same plain word accesses as the ISR-local
// value; only the stats task writes outside the ADC ISR, with IRQs masked.
static ISR_LAST_START: AtomicU32 = AtomicU32::new(0);
static DWT_GENERATION: AtomicU32 = AtomicU32::new(0);

// ========== ADC Handles ==========

/// One-time transfer from boot into the ADC interrupt's local state.
static ADC_HANDOFF: CriticalSectionMutex<RefCell<Option<AdcHandles>>> =
    CriticalSectionMutex::new(RefCell::new(None));

// ========== FOC Control ==========

/// FOC driver storage (mutated only inside the ADC ISR)
type PhaseManagerType = PhaseManager<HallAngleProxy, NoSensor, FastSinCos>;
type FocDriverType = FocDriver<MotorPwm<'static>, F405CurrentSensor, PhaseManagerType, FastSinCos>;

#[cfg(feature = "fast-telemetry")]
#[inline(never)]
fn publish_velocity_debug(
    driver: &FocDriverType,
    sequence: u32,
    tick_us: u64,
    hall_snapshot: Option<oxifoc_core::foc::sensors::HallSnapshot>,
) {
    use oxifoc_core::{runtime::velocity_debug, velocity_telemetry as trace};
    let update = driver.velocity_update_at(tick_us);
    let current = driver.current_debug_sample();
    if update.is_none() && current.is_none() && !velocity_debug::start_pending() {
        return;
    }
    let mut frame = trace::VelocityDebugFrame {
        sequence,
        tick_us: tick_us as u32,
        hall_raw: hall::read_hall_state_raw(),
        ..current.unwrap_or_default()
    };
    if let Some(hall) = hall_snapshot {
        use oxifoc_core::foc::hall_sensor::Direction;
        frame.hall_direction = match hall.direction {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
            Direction::Stopped => 0,
        };
        let age = hall.edge_age_s * 10_000.0;
        if age > f32::from(u16::MAX) {
            frame.flags |= trace::HALL_AGE_CLIPPED;
        }
        frame.hall_age_100us = age as u16;
    } else {
        frame.flags |= trace::HALL_UNAVAILABLE;
    }
    if let (Some(update), Some(snapshot)) = (update, driver.velocity_control_snapshot()) {
        frame.flags |= trace::ACTIVE | update.flags;
        frame.target = frame.quantize(update.target_rad_s, 8.0);
        frame.reference = frame.quantize(snapshot.reference_rad_s, 8.0);
        frame.feedback = frame.quantize(update.feedback_rad_s, 8.0);
        frame.requested_iq = frame.quantize(snapshot.requested_iq_a, 100.0);
        frame.granted_iq = frame.quantize(snapshot.granted_iq_a, 100.0);
        frame.granted_id = frame.quantize(snapshot.granted_id_a, 100.0);
        frame.integral = frame.quantize(driver.velocity_integral_a(), 100.0);
        frame.iq_min = frame.quantize(update.iq_min_a, 100.0);
        frame.iq_max = frame.quantize(update.iq_max_a, 100.0);
        if (snapshot.requested_iq_a - snapshot.granted_iq_a).abs() > 1e-4 {
            frame.flags |= trace::GRANT_CLIPPED;
        }
    } else if current.is_none() && !velocity_debug::start_pending() {
        // Position-control reuses the loop, but this stream describes velocity mode.
        return;
    }
    velocity_debug::push(frame);
}
#[cfg_attr(feature = "ccm", unsafe(link_section = ".ccm.bss"))]
static FOC_DRIVER: static_cell::StaticCell<FocDriverType> = static_cell::StaticCell::new();
static FOC_HANDOFF: CriticalSectionMutex<RefCell<Option<&'static mut FocDriverType>>> =
    CriticalSectionMutex::new(RefCell::new(None));
static FOC_READY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "board-vesc6-mk5")]
mod detection_timing {
    use super::{AtomicU32, CPU_HZ, ControlTiming, Duration, FocDriverType, ISR_BUDGET_CYCLES};
    use super::{ISR_LAST_START, MotorPwm, Ordering, Timer, interrupt};
    use embassy_stm32::{interrupt::typelevel::Interrupt, pac};

    const IDLE: u32 = 0;
    const AWAITING_SAMPLE: u32 = 0x1_0000;
    const APPLIED: u32 = AWAITING_SAMPLE + 1;
    const REJECTED: u32 = APPLIED + 1;
    // 1..=65535 carries the requested ARR. The detection task owns the
    // request through acknowledgement or cancellation. Only the ADC ISR
    // changes it to an outcome. The task cannot run during either ISR step.
    static REQUEST: AtomicU32 = AtomicU32::new(IDLE);

    pub(super) struct Request;

    impl Request {
        pub(super) fn submit(arr: u16) -> Option<Self> {
            REQUEST
                .compare_exchange(IDLE, u32::from(arr), Ordering::Release, Ordering::Relaxed)
                .ok()
                .map(|_| Self)
        }

        pub(super) async fn wait(self) -> bool {
            embassy_time::with_timeout(Duration::from_millis(500), async {
                loop {
                    match REQUEST.load(Ordering::Acquire) {
                        APPLIED => return true,
                        REJECTED => return false,
                        _ => Timer::after_millis(1).await,
                    }
                }
            })
            .await
            .unwrap_or(false)
        }
    }

    impl Drop for Request {
        fn drop(&mut self) {
            // An outer detection timeout may drop the waiter. Restoration
            // can then submit its own request while still holding the lease.
            REQUEST.store(IDLE, Ordering::Release);
        }
    }

    #[inline(always)]
    pub(super) fn after_sample(driver: &mut Option<&'static mut FocDriverType>) {
        // The word contains the complete request. There is no separate
        // payload to synchronize, and the task cannot run during this ISR.
        let state = REQUEST.load(Ordering::Relaxed);
        if state != IDLE {
            service(driver, state);
        }
    }

    #[inline(never)]
    fn service(driver: &mut Option<&'static mut FocDriverType>, state: u32) {
        if state == AWAITING_SAMPLE {
            REQUEST.store(APPLIED, Ordering::Release);
        } else if let Ok(arr) = u16::try_from(state)
            && let Some(driver) = driver.as_deref_mut()
        {
            apply(driver, arr);
        }
    }

    fn apply(driver: &mut FocDriverType, arr: u16) {
        let Some(drive) = crate::config::CONTROL_TIMING.try_get() else {
            REQUEST.store(REJECTED, Ordering::Release);
            return;
        };
        if !oxifoc_core::state::DETECTION_ACTIVE.load(Ordering::Acquire)
            || oxifoc_core::state::FLASH_OP_PENDING.load(Ordering::Acquire)
        {
            REQUEST.store(REJECTED, Ordering::Release);
            return;
        }
        let tim = pac::TIM1;
        if tim.arr().read().arr() == arr {
            REQUEST.store(APPLIED, Ordering::Release);
            return;
        }
        let timing = ControlTiming {
            clock_hz: drive.clock_hz,
            period_ticks: 2 * u32::from(arr),
        };
        if !driver.retime_stopped(timing, crate::config::DEAD_TIME_COMP_NS, |pwm| {
            pwm.install_stopped_period(arr);
        }) {
            REQUEST.store(REJECTED, Ordering::Release);
            return;
        }
        // The old sample has already been published. TIM1 is stopped, so
        // drain any in-flight conversion before restarting the trigger.
        cortex_m::asm::delay(CPU_HZ / 100_000);
        for adc in [pac::ADC1, pac::ADC2, pac::ADC3] {
            adc.sr().write(|_| {});
        }
        interrupt::typelevel::ADC::unpend();
        let budget = (u64::from(CPU_HZ) * u64::from(timing.period_ticks)
            / u64::from(timing.clock_hz)) as u32;
        ISR_BUDGET_CYCLES.store(budget, Ordering::Relaxed);
        ISR_LAST_START.store(0, Ordering::Relaxed);
        REQUEST.store(AWAITING_SAMPLE, Ordering::Release);
        MotorPwm::resume_control_timer();
    }
}

/// Request the stopped ISR to change TIM1 and acknowledge the first fresh
/// sample. The commissioning lease excludes drive commands until restore.
#[cfg(feature = "board-vesc6-mk5")]
pub(crate) async fn retime_for_detection(timing: ControlTiming) -> bool {
    let Some(drive) = crate::config::CONTROL_TIMING.try_get() else {
        return false;
    };
    if timing.clock_hz != drive.clock_hz
        || timing.period_ticks == 0
        || !timing.period_ticks.is_multiple_of(2)
    {
        return false;
    }
    let Ok(arr) = u16::try_from(timing.period_ticks / 2) else {
        return false;
    };
    let Some(request) = detection_timing::Request::submit(arr) else {
        return false;
    };
    request.wait().await
}

// ========== Initialization ==========

/// Initialize FOC driver with motor PWM, sensors, and stored config.
pub async fn init(
    mut motor_pwm: MotorPwm<'static>,
    adc_handles: AdcHandles,
    config: &RuntimeConfig,
) {
    #[cfg(feature = "fast-telemetry")]
    oxifoc_core::runtime::velocity_debug::register_producer();
    // Ensure PWM outputs are off initially
    motor_pwm.emergency_stop();
    let timing = motor_pwm.control_timing();
    let budget =
        (u64::from(CPU_HZ) * u64::from(timing.period_ticks) / u64::from(timing.clock_hz)) as u32;
    ISR_BUDGET_CYCLES.store(budget, Ordering::Relaxed);

    // Publish the initialized handles before enabling the ADC interrupt.
    ADC_HANDOFF.lock(|cell| cell.replace(Some(adc_handles)));

    // Enable the ADC interrupt and its internal PWM trigger. Phase channels
    // stay high-Z until the calibration state machine selects one.
    // Order: install ADC handles → enable interrupt → enable PWM triggers.
    // ADC at priority 0 (highest) — the FOC loop is the actuator's most
    // time-critical ISR; comms ISRs (USB) must never preempt or
    // jitter it (mirrors the G431 setup; main.rs demotes every other
    // vector below the control ISRs for exactly this reason).
    //
    // DELIBERATE: the ISR free-runs without a FOC driver until the
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
    let phase_manager = PhaseManager::with_hall(hall_proxy).with_sincos::<FastSinCos>();

    // Hardware/controller setup is independent of the electrical model.
    // Boot and live writes install the same prepared model below.
    let mut foc_controller = FocController::<_, FastSinCos>::new(initial_vbus_v);

    // Configure dead time compensation
    foc_controller
        .set_dead_time_comp_factor(crate::config::DEAD_TIME_COMP_NS as f32 * 1e-9 / timing.dt_s());
    defmt::unwrap!(
        foc_controller
            .set_dead_time_comp_linear_region(crate::config::DEAD_TIME_COMP_LINEAR_CURRENT_A)
    );

    // Use the same timer-derived period sent to the host and detection backend.
    let actuation_delay_cycles = motor_pwm.actuation_delay_cycles();
    let mut foc_driver = FocDriver::new(
        foc_controller,
        motor_pwm,
        current_sensor,
        phase_manager,
        timing.dt_s(),
    );
    foc_driver.set_actuation_delay(actuation_delay_cycles);

    let mut model = oxifoc_core::motor::foc_driver::PreparedMotorModel::from_config(
        config,
        initial_vbus_v,
        BOARD.max_phase_current_a,
    );
    let _ = defmt::unwrap!(foc_driver.apply_motor_model(&mut model));
    let boot_source = config.boot_feedback_source();
    let source = if foc_driver.request_phase_source(boot_source) {
        boot_source
    } else {
        let _ = foc_driver.request_phase_source(PhaseSource::Manual);
        PhaseSource::Manual
    };
    critical_section::with(|cs| {
        let mut state = STATE.borrow(cs).borrow_mut();
        state.motor_pole_pairs = model.pole_pairs;
        state.phase_source = source;
    });
    // Failsafe: command-staleness deadman + reaction policy from stored config
    // (or board defaults); the OV trip feeds the regen-brake derate.
    foc_driver.set_failsafe(FailsafeConfig::from_stored(config.failsafe.as_ref()));
    foc_driver
        .configure_voltage_protection(&BOARD, config.voltage_limits.as_ref())
        .expect("stored voltage limits must fit the board");

    // Cruise velocity-loop tuning from stored config (or soft defaults).
    foc_driver.set_velocity_config(config.effective_velocity());

    // Graduated derating ramps from stored config (default = FET thermal
    // rolloff only; see motor::derating).
    foc_driver.set_derating(config.effective_derating());
    foc_driver.load_drive_config(config);

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
    let driver = FOC_DRIVER.init(foc_driver);
    FOC_HANDOFF.lock(|cell| cell.replace(Some(driver)));
    FOC_READY.store(true, Ordering::Release);

    if calibration_started {
        match CURRENT_OFFSET_DONE.wait().await {
            Ok(report) => oxifoc_core::state::model::publish_boot_current_offsets(
                &crate::RUNTIME_CONFIG,
                report.offsets,
            ),
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
        if let Some(mv) = measured_vbus_mv() {
            return mv as f32 / 1000.0;
        }
        Timer::after(Duration::from_micros(100)).await;
    }
    defmt::error!("no VBUS measurement within 100ms - ADC/ISR pipeline dead?");
    0.0
}

// ========== ADC Interrupt Handler ==========

/// Read the profiling clock; compile away when timing is disabled.
#[inline(always)]
fn isr_timing_now() -> u32 {
    #[cfg(feature = "isr-timing")]
    {
        cortex_m::peripheral::DWT::cycle_count()
    }
    #[cfg(not(feature = "isr-timing"))]
    {
        0
    }
}

/// Read all injected ADC samples and run FOC control.
/// Triggered by ADC3 JEOC (end of injected conversion sequence).
/// ADC1, ADC2, ADC3 all start conversion simultaneously on TIM1_CC4.
#[interrupt]
#[cfg_attr(feature = "ram-foc", unsafe(link_section = ".data.ram_foc"))]
fn ADC() {
    static mut ADCS: Option<AdcHandles> = None;
    static mut DRIVER: Option<&'static mut FocDriverType> = None;
    static mut SEQ: u32 = 0;
    #[cfg(all(
        feature = "bench-foc-distribution",
        not(feature = "bench-foc-deadline")
    ))]
    static mut DISTRIBUTION: crate::diagnostics::foc_distribution::Recorder =
        crate::diagnostics::foc_distribution::Recorder::new();
    #[cfg(feature = "bench-cycle-outliers")]
    static mut CYCLE_OUTLIERS: crate::diagnostics::cycle_outliers::Recorder =
        crate::diagnostics::cycle_outliers::Recorder::new();
    #[cfg(feature = "bench-hall-phase")]
    static mut HALL_PHASE: crate::diagnostics::hall_edges::PhaseRecorder =
        crate::diagnostics::hall_edges::PhaseRecorder::new();
    #[cfg(feature = "bench-raw-codec")]
    static mut CODEC_BENCH: crate::diagnostics::codec_bench::Bench =
        crate::diagnostics::codec_bench::Bench::new();

    #[cfg(feature = "bench-netstack-trace")]
    let trace_adc = crate::diagnostics::trace::is_recording();
    #[cfg(feature = "bench-netstack-trace")]
    if trace_adc {
        // First MMIO observations in this IRQ, before DWT or trace appends.
        // A turn between CR1 reads makes DIR/CNT phase reconstruction ambiguous.
        let dir_before = embassy_stm32::pac::TIM1.cr1().read().dir()
            == embassy_stm32::pac::timer::vals::Dir::Down;
        let count = embassy_stm32::pac::TIM1.cnt().read().cnt();
        let dir_after = embassy_stm32::pac::TIM1.cr1().read().dir()
            == embassy_stm32::pac::timer::vals::Dir::Down;
        let phase = u32::from(count)
            | (u32::from(dir_after) << 16)
            | (u32::from(dir_before != dir_after) << 17);
        // Timestamp remains the serialized append time, while id is the
        // earlier entry-phase snapshot: CNT[15:0], DIR16, transition17.
        crate::diagnostics::trace::record(crate::diagnostics::trace::ADC_ENTRY_PHASE, phase);
    }
    #[cfg(feature = "bench-foc-deadline")]
    crate::diagnostics::pwm_deadline::begin();
    let isr_t0 = isr_timing_now();
    #[cfg(feature = "bench-cycle-outliers")]
    let core_before = oxifoc_core::isr_prof::Snapshot::observe();
    #[cfg(feature = "bench-executor-trace")]
    crate::diagnostics::trace::record(8, 0);
    #[cfg(all(
        feature = "bench-netstack-trace",
        not(feature = "bench-executor-trace")
    ))]
    if trace_adc {
        crate::diagnostics::trace::record(8, 0);
    }
    let last_start = if cfg!(feature = "isr-profiling") {
        ISR_LAST_START.load(Ordering::Relaxed)
    } else {
        0
    };
    if cfg!(feature = "isr-profiling") && last_start != 0 {
        let interval = isr_t0.wrapping_sub(last_start);
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
    if cfg!(feature = "isr-profiling") {
        ISR_LAST_START.store(isr_t0, Ordering::Relaxed);
    }
    let protection_board = prepared_board();
    let board = protection_board.board();

    let adcs = ADCS.get_or_insert_with(|| {
        defmt::unwrap!(
            ADC_HANDOFF.lock(|cell| cell.take()),
            "ADC interrupt enabled before handles were installed"
        )
    });

    // Preserve the 128-cycle thermal sampling cadence. Exact flash lookups
    // replace the two Beta-model logarithms that previously spiked this cycle.
    let convert_temp = *SEQ & 127 == 0;

    // Read ADC1 injected data (phase A current + board temp)
    let mut phase_a = [0; 3];
    adcs.adc1.read_latest(&mut phase_a);
    let [ia_raw, board_temp_raw, va_raw] = phase_a;
    IA_SAMPLE.store(ia_raw, Ordering::Relaxed);

    // Convert board temperature raw ADC to 0.1°C units
    let board_temp_c_x10 = if convert_temp {
        let t = temperature::board_c_x10(board_temp_raw);
        BOARD_TEMP_C_X10.store(t, Ordering::Relaxed);
        t
    } else {
        BOARD_TEMP_C_X10.load(Ordering::Relaxed)
    };

    // Read ADC2 injected data (phase B current + motor temp)
    let mut phase_b = [0; 3];
    adcs.adc2.read_latest(&mut phase_b);
    let [ib_raw, motor_temp_raw, vb_raw] = phase_b;
    IB_SAMPLE.store(ib_raw, Ordering::Relaxed);

    MOTOR_TEMP_RAW.store(motor_temp_raw, Ordering::Relaxed);
    let motor_temp_c_x10 = oxifoc_core::state::motor_temperature_input_enabled().then(|| {
        if convert_temp {
            let t = temperature::motor_c_x10(motor_temp_raw);
            MOTOR_TEMP_C_X10.store(t, Ordering::Relaxed);
            t
        } else {
            MOTOR_TEMP_C_X10.load(Ordering::Relaxed)
        }
    });

    // Read ADC3 injected data (phase C current + VBUS)
    let mut phase_c = [0; 3];
    adcs.adc3.read_latest(&mut phase_c);
    let [ic_raw, vbus_raw, vc_raw] = phase_c;
    IC_SAMPLE.store(ic_raw, Ordering::Relaxed);

    // Convert VBUS raw ADC to millivolts
    let vbus_mv = oxifoc_core::state::calibrated_vbus_mv(&STATE, board.vbus_mv_from_adc(vbus_raw));
    VBUS_MV.store(vbus_mv, Ordering::Relaxed);

    let prof_t1 = oxifoc_core::isr_prof::now();

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
    #[cfg(feature = "bench-hall")]
    crate::diagnostics::hall_edges::observe_adc(*SEQ);
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
        .with_temp(TempSensorId::Fet, board_temp_c_x10);
    let adc_snapshot = match motor_temp_c_x10 {
        Some(temp) => adc_snapshot.with_temp(TempSensorId::Motor, temp),
        None => adc_snapshot,
    };

    // Get Hall snapshot
    let hall_snapshot = hall::get_snapshot(now_ticks);

    let prof_t2 = oxifoc_core::isr_prof::now();

    // Run FOC control loop (shared cycle logic in core)
    #[cfg(feature = "isr-timing")]
    let mut time_control_motion = false;
    #[cfg(any(feature = "bench-cycle-outliers", feature = "bench-foc-distribution"))]
    let mut outlier_active = false;
    #[cfg(feature = "bench-cycle-outliers")]
    let mut outlier_velocity = 0.0_f32;
    #[cfg(feature = "bench-raw-codec")]
    let mut codec_bench_stopped = false;
    // The release flag avoids touching the boot handoff while the task
    // waits for its first VBUS reading. Once taken, DRIVER stays ISR-local.
    if DRIVER.is_none() && FOC_READY.load(Ordering::Acquire) {
        *DRIVER = FOC_HANDOFF.lock(|cell| cell.take());
    }
    let foc_telem = DRIVER.as_deref_mut().and_then(|driver| {
        #[cfg(feature = "fast-telemetry")]
        let velocity_debug = oxifoc_core::runtime::velocity_debug::enabled();
        #[cfg(feature = "fast-telemetry")]
        {
            driver.record_velocity_updates(velocity_debug);
            // Sample the current grant once per 16 native cycles (~938 Hz at 15 kHz).
            driver.arm_current_debug(velocity_debug && *SEQ & 15 == 0);
        }
        let output = run_foc_cycle(
            &STATE,
            &FAULT_REGISTRY,
            driver,
            vbus_mv as f32 / 1000.0,
            now_ticks,
            protection_board,
        );
        #[cfg(feature = "fast-telemetry")]
        if velocity_debug {
            publish_velocity_debug(driver, *SEQ, now_ticks, hall_snapshot);
        }
        #[cfg(feature = "bench-hall-phase")]
        HALL_PHASE.observe(*SEQ, driver.phase(), hall_snapshot);
        #[cfg(feature = "bench-hall-phase")]
        HALL_PHASE.observe_velocity(*SEQ, || driver.velocity_control_snapshot());
        #[cfg(feature = "bench-hall-crossover")]
        if matches!(
            driver.mode(),
            oxifoc_core::types::ControlMode::VelocityControl { .. }
        ) {
            HALL_PHASE.observe_crossover(*SEQ, driver.phase());
        }
        #[cfg(feature = "bench-hall-phase")]
        HALL_PHASE.observe_faults(*SEQ, || {
            use oxifoc_core::foc::phase::PhaseProvider;
            driver
                .phase()
                .rotor_velocity()
                .map_or(f32::NAN, |v| v.electrical_rad_s())
        });
        #[cfg(feature = "bench-raw-codec")]
        {
            codec_bench_stopped = matches!(driver.mode(), oxifoc_core::types::ControlMode::Stopped);
        }
        #[cfg(any(feature = "bench-cycle-outliers", feature = "bench-foc-distribution"))]
        {
            outlier_active = matches!(
                driver.mode(),
                oxifoc_core::types::ControlMode::CurrentControl { .. }
                    | oxifoc_core::types::ControlMode::VelocityControl { .. }
            );
            #[cfg(feature = "bench-cycle-outliers")]
            {
                outlier_velocity = output.map_or(0.0, |sample| sample.velocity_rad_s);
            }
        }
        #[cfg(feature = "isr-timing")]
        {
            use oxifoc_core::types::ControlMode;
            time_control_motion = matches!(
                driver.mode(),
                ControlMode::CurrentControl { .. } | ControlMode::VelocityControl { .. }
            ) && output.is_some_and(|sample| sample.velocity_rad_s != 0.0);
        }
        #[cfg(any(
            feature = "bench-executor-trace",
            feature = "bench-pwm-commit",
            feature = "bench-netstack-trace"
        ))]
        let trace_active = if !cfg!(feature = "bench-netstack-trace")
            && cfg!(any(
                feature = "bench-trace-excitation",
                feature = "bench-pwm-commit"
            )) {
            matches!(
                driver.mode(),
                oxifoc_core::types::ControlMode::DirectVoltage { .. }
            )
        } else {
            !matches!(driver.mode(), oxifoc_core::types::ControlMode::Stopped)
        };
        #[cfg(any(
            feature = "bench-executor-trace",
            feature = "bench-pwm-commit",
            feature = "bench-netstack-trace"
        ))]
        if trace_active {
            crate::diagnostics::trace::trigger_active();
        } else {
            crate::diagnostics::trace::trigger_stopped();
        }
        output
    });

    let prof_t3 = oxifoc_core::isr_prof::now();

    // Update global state + fast telemetry stream
    publish_cycle_telemetry(
        &STATE,
        adc_snapshot,
        hall_snapshot,
        foc_telem.unwrap_or_default(),
        *SEQ,
    );

    let prof_t4 = oxifoc_core::isr_prof::now();

    // Feed the IWDG: a completed FOC cycle is the board's liveness signal.
    feed_watchdog();

    oxifoc_core::isr_prof::add(&ISR_PROF_ADC, isr_t0, prof_t1);
    oxifoc_core::isr_prof::add(&ISR_PROF_SNAP, prof_t1, prof_t2);
    oxifoc_core::isr_prof::add(&ISR_PROF_FOC, prof_t2, prof_t3);
    oxifoc_core::isr_prof::add(&ISR_PROF_PUB, prof_t3, prof_t4);

    #[cfg(all(
        feature = "bench-netstack-trace",
        not(feature = "bench-executor-trace")
    ))]
    if trace_adc {
        crate::diagnostics::trace::record(9, 0);
    }
    #[cfg(feature = "board-vesc6-mk5")]
    detection_timing::after_sample(DRIVER);
    #[cfg(feature = "bench-raw-codec")]
    CODEC_BENCH.on_adc(
        codec_bench_stopped,
        enables,
        DWT_GENERATION.load(Ordering::Relaxed),
    );
    #[cfg(feature = "bench-cycle-outliers")]
    {
        let core_after = oxifoc_core::isr_prof::Snapshot::observe();
        let body_end = isr_timing_now();
        CYCLE_OUTLIERS.observe(
            outlier_active,
            *SEQ,
            isr_t0,
            DWT_GENERATION.load(Ordering::Relaxed),
            body_end.wrapping_sub(isr_t0),
            ISR_BUDGET_CYCLES.load(Ordering::Relaxed),
            u32::from(convert_temp) | (u32::from(time_control_motion) << 1),
            outlier_velocity.to_bits(),
            [
                prof_t1.wrapping_sub(isr_t0),
                prof_t2.wrapping_sub(prof_t1),
                prof_t3.wrapping_sub(prof_t2),
                prof_t4.wrapping_sub(prof_t3),
            ],
            core_after.increments_since(&core_before),
        );
    }
    let isr_end = isr_timing_now();
    #[cfg(feature = "bench-foc-deadline")]
    crate::diagnostics::foc_distribution::observe_deadline_session(outlier_active);
    #[cfg(all(
        feature = "bench-foc-distribution",
        not(feature = "bench-foc-deadline")
    ))]
    DISTRIBUTION.observe(
        outlier_active,
        *SEQ,
        isr_t0,
        DWT_GENERATION.load(Ordering::Relaxed),
        isr_end.wrapping_sub(isr_t0),
        ISR_BUDGET_CYCLES.load(Ordering::Relaxed),
    );
    #[cfg(feature = "isr-timing")]
    if time_control_motion {
        motion_timing::record(isr_end.wrapping_sub(isr_t0));
    }
    if cfg!(feature = "isr-timing") {
        ISR_CYC_SUM.fetch_add(isr_end.wrapping_sub(isr_t0), Ordering::Relaxed);
    }
    ISR_CYC_N.fetch_add(1, Ordering::Relaxed);
    if cfg!(feature = "isr-timing") {
        let isr_dt = isr_end.wrapping_sub(isr_t0);
        ISR_CYC_MAX.fetch_max(isr_dt, Ordering::Relaxed);
        // An ISR that ate the whole PWM period leaves no thread time.
        if isr_dt > ISR_BUDGET_CYCLES.load(Ordering::Relaxed) {
            ISR_CYC_OVER.fetch_add(1, Ordering::Relaxed);
        }
    }
    #[cfg(feature = "bench-foc-deadline")]
    crate::diagnostics::pwm_deadline::end();
    // Include timing/statistics bookkeeping in the traced ADC context;
    // ending earlier would charge that higher-priority work to USB.
    #[cfg(feature = "bench-executor-trace")]
    crate::diagnostics::trace::record(9, 0);
}

fn dwt_sample() -> DwtSample {
    DwtSample {
        cycles: cortex_m::peripheral::DWT::cycle_count(),
        generation: DWT_GENERATION.load(Ordering::Relaxed),
    }
}

/// A clock reference must not bridge a frozen counter or a stats-task rearm.
pub(crate) fn dwt_clock_sample() -> Option<DwtSample> {
    critical_section::with(|_| {
        // SAFETY: this short-lived handle only reads the DCB/DWT registers;
        // no peripheral reference escapes and the stats writer uses this CS.
        let cp = unsafe { cortex_m::Peripherals::steal() };
        profiling_window::dwt_enabled(cp.DCB.demcr.read(), cp.DWT.ctrl.read()).then(dwt_sample)
    })
}

/// Re-enable only missing enable bits. Never reset CYCCNT or other debug state.
/// The same CS must then discard all platform/core counters and interval history.
fn rearm_dwt(_cs: critical_section::CriticalSection<'_>) -> bool {
    // SAFETY: the stats task is the sole runtime DCB/DWT writer, under the
    // snapshot CS. This handle does not access any other core peripheral.
    let cp = unsafe { cortex_m::Peripherals::steal() };
    let demcr = cp.DCB.demcr.read();
    let ctrl = cp.DWT.ctrl.read();
    if profiling_window::dwt_enabled(demcr, ctrl) {
        return false;
    }
    if demcr & profiling_window::TRACE_ENABLE == 0 {
        // SAFETY: read-modify-write sets TRCENA and preserves all other bits.
        unsafe { cp.DCB.demcr.modify(profiling_window::enable_trace) };
    }
    if ctrl & profiling_window::COUNTER_ENABLE == 0 {
        // SAFETY: read-modify-write sets CYCCNTENA, preserving other DWT bits.
        unsafe { cp.DWT.ctrl.modify(profiling_window::enable_counter) };
    }
    DWT_GENERATION.fetch_add(1, Ordering::Relaxed);
    ISR_LAST_START.store(0, Ordering::Relaxed);
    true
}

/// 1 Hz ISR-cost stats — same line format as the g431 so the bench-suite
/// parser works unchanged: `isr/s: n= avg= max= over= load_pct=` plus the
/// per-section and core-bucket breakdowns.
struct IsrStatistics {
    samples: u32,
    cycles: u32,
    max_cycles: u32,
    overruns: u32,
    #[cfg(feature = "isr-timing")]
    motion: (u32, u32, u32),
    adc: u32,
    snapshot: u32,
    control: u32,
    publish: u32,
    intervals: (u32, u32, u32, u32, u32, u32),
    core: oxifoc_core::isr_prof::Snapshot,
    clock: DwtSample,
    rearmed: bool,
}

impl IsrStatistics {
    fn take() -> Self {
        critical_section::with(|cs| {
            let rearmed = rearm_dwt(cs);
            Self {
                samples: ISR_CYC_N.swap(0, Ordering::Relaxed),
                cycles: ISR_CYC_SUM.swap(0, Ordering::Relaxed),
                max_cycles: ISR_CYC_MAX.swap(0, Ordering::Relaxed),
                overruns: ISR_CYC_OVER.swap(0, Ordering::Relaxed),
                #[cfg(feature = "isr-timing")]
                motion: motion_timing::take(),
                adc: ISR_PROF_ADC.swap(0, Ordering::Relaxed),
                snapshot: ISR_PROF_SNAP.swap(0, Ordering::Relaxed),
                control: ISR_PROF_FOC.swap(0, Ordering::Relaxed),
                publish: ISR_PROF_PUB.swap(0, Ordering::Relaxed),
                intervals: (
                    ISR_INTERVAL_N.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_SUM.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_MIN.swap(u32::MAX, Ordering::Relaxed),
                    ISR_INTERVAL_MAX.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_LONG.swap(0, Ordering::Relaxed),
                    ISR_INTERVAL_SHORT.swap(0, Ordering::Relaxed),
                ),
                core: oxifoc_core::isr_prof::Snapshot::take(cs),
                clock: dwt_sample(),
                rearmed,
            }
        })
    }
}

#[embassy_executor::task]
pub async fn isr_stats_task() {
    use embassy_time::Ticker;
    // A perpetually backlogged stream yields between batches but stays ready.
    // Profiling must run before that stream so windows do not span states.
    #[cfg(all(feature = "isr-timing", feature = "task-priority"))]
    embassy_executor::MetadataRef::for_current_task()
        .await
        .set_priority(2);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut timing_window = DwtWindow::default();
    #[cfg(feature = "transport-usb")]
    let mut usb_clock_previous = None;
    #[cfg(feature = "transport-usb")]
    let mut usb_clock_totals = (0u64, 0u64);
    loop {
        ticker.next().await;
        let stats = IsrStatistics::take();
        let elapsed_cycles = timing_window.observe(Some(stats.clock));
        if stats.rearmed {
            defmt::warn!("DWT restarted; discarded partial profiling window");
            #[cfg(feature = "transport-usb")]
            {
                usb_clock_previous = None;
                usb_clock_totals = (0, 0);
            }
        }
        #[cfg(feature = "transport-usb")]
        if crate::protocol::USB_IDENT.is_set() {
            // DSTS is read-only. SOF frame numbers come from the USB host,
            // providing a clock independent of TIM1/TIM2/DWT on this board.
            // FS frames have an 11-bit frame number, wrapping every 2048 ms.
            let frame = embassy_stm32::pac::USB_OTG_FS.dsts().read().fnsof() & 0x7ff;
            if let Some(clock) = dwt_clock_sample() {
                if let Some((previous_frame, previous_clock)) = usb_clock_previous {
                    let frames = frame.wrapping_sub(previous_frame) & 0x7ff;
                    // Reject suspend/reset/long reporting gaps rather than infer
                    // missing wraps or fold them into the reference-clock fit.
                    if let Some(elapsed) = clock.elapsed_since(previous_clock)
                        && (900..=1100).contains(&frames)
                        && (CPU_HZ * 4 / 5..=CPU_HZ * 6 / 5).contains(&elapsed.get())
                    {
                        usb_clock_totals.0 += u64::from(frames);
                        usb_clock_totals.1 += u64::from(elapsed.get());
                        defmt::info!(
                            "usbclk/s: frames={} cycles={} total_frames={} total_cycles={} avg_cycles_per_frame={}",
                            frames,
                            elapsed.get(),
                            usb_clock_totals.0,
                            usb_clock_totals.1,
                            usb_clock_totals.1 / usb_clock_totals.0,
                        );
                    } else {
                        usb_clock_totals = (0, 0);
                    }
                }
                usb_clock_previous = Some((frame, clock));
            } else {
                usb_clock_previous = None;
                usb_clock_totals = (0, 0);
            }
        }
        let Some(sample_count) = core::num::NonZeroU32::new(stats.samples) else {
            continue;
        };
        let cyc_n = sample_count.get();
        #[cfg(feature = "bench-foc-sections-ram")]
        {
            use crate::diagnostics::foc_distribution as distribution;
            use oxifoc_core::isr_prof::Section as S;
            if distribution::needs_sections(cortex_m::peripheral::DWT::cycle_count()) {
                let a = |section| stats.core.average(section, sample_count);
                distribution::publish_sections([
                    1,
                    CPU_HZ,
                    cyc_n,
                    stats.cycles / cyc_n,
                    stats.max_cycles,
                    stats.overruns,
                    stats.adc / cyc_n,
                    stats.snapshot / cyc_n,
                    stats.control / cyc_n,
                    stats.publish / cyc_n,
                    a(S::Command),
                    a(S::Protection),
                    a(S::Step),
                    a(S::Tail),
                    a(S::Gate),
                    a(S::Control),
                    a(S::Trig),
                    a(S::Post),
                    a(S::Estimator),
                    a(S::Observer),
                    a(S::Startup),
                    a(S::PhaseOutput),
                    a(S::SensorSampling),
                    a(S::PhaseVoltage),
                    a(S::VelocityRequest),
                ]);
            }
            // Timing counters were already consumed coherently. Avoid all
            // reporter text I/O during this diagnostic, including through RTT.
            if cfg!(feature = "bench-foc-sections-ram") {
                continue;
            }
        }
        #[cfg(feature = "isr-timing")]
        if stats.motion.0 != 0 {
            defmt::info!(
                "isr-motion/s: n={} avg={} max={}",
                stats.motion.0,
                stats.motion.1 / stats.motion.0,
                stats.motion.2
            );
        }
        if cfg!(feature = "isr-timing")
            && let Some(elapsed_cycles) = elapsed_cycles
        {
            let cyc_sum = stats.cycles;
            let cyc_max = stats.max_cycles;
            let adc = stats.adc / cyc_n;
            let sn = stats.snapshot / cyc_n;
            let fo = stats.control / cyc_n;
            let pb = stats.publish / cyc_n;
            let avg = cyc_sum / cyc_n;
            defmt::info!(
                "isr/s: n={} avg={} max={} over={} load_pct={}",
                cyc_n,
                avg,
                cyc_max,
                stats.overruns,
                u64::from(cyc_sum) * 100 / u64::from(elapsed_cycles.get()),
            );
            if cfg!(feature = "isr-profiling") {
                defmt::info!(
                    "isrp/s: adc={} snap={} foc={} pub={} tail={}",
                    adc,
                    sn,
                    fo,
                    pb,
                    avg.saturating_sub(adc + sn + fo + pb)
                );
            }
            let (interval_n, interval_sum, interval_min, interval_max, long, short) =
                stats.intervals;
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
        } else if !cfg!(feature = "isr-timing") {
            defmt::info!("adc/s: n={} profiling=off", cyc_n);
        }
        use oxifoc_core::runtime::streaming::fast_telem_stats as ft;
        let pushed = ft::PUSH_OK.swap(0, Ordering::Relaxed);
        let dropped = ft::PUSH_DROPS.swap(0, Ordering::Relaxed);
        let read = ft::READ_OK.swap(0, Ordering::Relaxed);
        let badlen = ft::READ_BADLEN.swap(0, Ordering::Relaxed);
        let batches = ft::BCAST_OK.swap(0, Ordering::Relaxed);
        let payload_bytes = ft::BCAST_BYTES.swap(0, Ordering::Relaxed);
        let failed = ft::BCAST_FAILS.swap(0, Ordering::Relaxed);
        #[cfg(feature = "bench-hall")]
        crate::diagnostics::hall_edges::observe_stream_stats(dropped, badlen, failed);
        let full = ft::BCAST_FULL.swap(0, Ordering::Relaxed);
        let busy = ft::BCAST_BUSY.swap(0, Ordering::Relaxed);
        let unrouted = ft::BCAST_UNROUTED.swap(0, Ordering::Relaxed);
        let other = ft::BCAST_OTHER.swap(0, Ordering::Relaxed);
        let drain_gap = ft::MAX_DRAIN_GAP_US.swap(0, Ordering::Relaxed);
        let batch_cost = ft::MAX_BATCH_US.swap(0, Ordering::Relaxed);
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
            if cfg!(feature = "isr-profiling") {
                defmt::info!(
                    "streamdiag/s: full={} busy={} unrouted={} other={} drain_gap_us={} batch_us={} encode_us={} broadcast_us={}",
                    full,
                    busy,
                    unrouted,
                    other,
                    drain_gap,
                    batch_cost,
                    ft::ENCODE_US.swap(0, Ordering::Relaxed),
                    ft::BROADCAST_US.swap(0, Ordering::Relaxed)
                );
            } else {
                defmt::info!(
                    "streamdiag/s: full={} busy={} unrouted={} other={} profiling=off",
                    full,
                    busy,
                    unrouted,
                    other,
                );
            }
        }
        #[cfg(feature = "bench-usb-metrics")]
        {
            use crate::protocol::usb_metrics as um;
            defmt::info!(
                "usbdiag/s: writes={} max_write_ticks={} long={}",
                um::WRITES.swap(0, Ordering::Relaxed),
                um::MAX_WRITE_TICKS.swap(0, Ordering::Relaxed),
                um::LONG_WRITES.swap(0, Ordering::Relaxed)
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
        #[cfg(feature = "isr-profiling")]
        if elapsed_cycles.is_some() {
            use oxifoc_core::isr_prof::Section as S;
            let average = |section| stats.core.average(section, sample_count);
            defmt::info!(
                "isrc/s: cmd={} prot={} step={} ctail={} | stopped: pwmoff={} phase={}",
                average(S::Command),
                average(S::Protection),
                average(S::Step),
                average(S::Tail),
                average(S::PwmOff),
                average(S::StoppedPhase),
            );
            defmt::info!(
                "isrd/s: gate={} (curr={}) ctrl={} (trig={}) post={} est={}",
                average(S::Gate),
                average(S::CurrentRead),
                average(S::Control),
                average(S::Trig),
                average(S::Post),
                average(S::Estimator),
            );
            defmt::info!(
                "isre/s: obs={} (otail={}) startup={} out={} (trk={})",
                average(S::Observer),
                average(S::ObserverTail),
                average(S::Startup),
                average(S::PhaseOutput),
                average(S::PhaseTracker),
            );
            defmt::info!(
                "isrh/s: sense={} voltage={} velocity_request={} ",
                average(S::SensorSampling),
                average(S::PhaseVoltage),
                average(S::VelocityRequest)
            );
        }
    }
}

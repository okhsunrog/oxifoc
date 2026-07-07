//! FOC (Field-Oriented Control) management and ADC interrupt handling

use core::cell::RefCell;
use core::sync::atomic::{AtomicI16, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::adc::InjectedAdc;
use embassy_stm32::{Peri, interrupt, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Timer};

use oxifoc_core::clear_rc_w0;
use oxifoc_core::foc::controller::FocController;
use oxifoc_core::foc::phase::{PhaseManager, PhaseSource};
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
static MOTOR_POLE_PAIRS: AtomicU8 = AtomicU8::new(0);

// ========== ISR cost instrumentation (DWT cycle counter) ==========

/// Sum of ADC1_2 ISR durations in CPU cycles since last stats swap.
/// u32 headroom: 20 kHz × ~4000 cycles = 80 M/s, swapped at 1 Hz.
pub static ISR_CYC_SUM: AtomicU32 = AtomicU32::new(0);

/// Per-section DWT cycle sums for the ADC1_2 ISR (reset each 1 Hz report;
/// avg = sum / ISR_CYC_N). Sections in ISR order — the boundaries are the
/// timestamps in the handler, so each atomic costs one RMW (~6 cycles);
/// total instrumentation overhead ~40 cycles, charged to `tail`.
pub static ISR_PROF_ADC1: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_ADC2: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_SNAP: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_FOC: AtomicU32 = AtomicU32::new(0);
pub static ISR_PROF_PUB: AtomicU32 = AtomicU32::new(0);
/// Max single ADC1_2 ISR duration in CPU cycles since last stats swap.
pub static ISR_CYC_MAX: AtomicU32 = AtomicU32::new(0);
/// ISR cycles that exceeded the 8500-cycle 20 kHz budget (per stats window).
pub static ISR_CYC_OVER: AtomicU32 = AtomicU32::new(0);
/// Number of ADC1_2 ISR executions since last stats swap.
pub static ISR_CYC_N: AtomicU32 = AtomicU32::new(0);

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
    MOTOR_POLE_PAIRS.store(
        config.motor_params.as_ref().map_or(0, |m| m.pole_pairs),
        Ordering::Relaxed,
    );
    let hall_proxy = HallAngleProxy::new();
    let initial_vbus_v =
        (VBUS_MV.load(Ordering::Relaxed) as f32 / 1000.0).max(BOARD.initial_vbus_volts);
    let mut phase_manager = PhaseManager::with_hall(hall_proxy).with_sincos::<CordicSinCos>();
    // Arm the sensorless estimators (back-EMF + HFI) from detected motor
    // params; the angle source stays Hall until the host switches it.
    phase_manager.configure_observers_from_config(config, initial_vbus_v);

    // Sensorless board (config::SENSORLESS): keep the boot angle source off
    // Hall so the unwired hall inputs don't spam a HallError every cycle. Ride
    // on the back-EMF observer once it has params; before detection bakes them
    // hold a Manual angle (also non-hall) — commutation stays inert until the
    // host drives it, and detection bypasses the source anyway.
    if crate::config::SENSORLESS {
        let boot_source = if config.motor_params.is_some() {
            PhaseSource::Observer
        } else {
            PhaseSource::Manual
        };
        if phase_manager.set_source(boot_source).is_err() {
            // Observer can be rejected on a PARTIAL bake: `is_some()` above is
            // weaker than the observer's own gate (`is_valid() && flux > 0`),
            // e.g. R/L present but the flux step never ran. Fall back to
            // Manual, NOT Hall — Hall on this hall-less board spams a
            // HallError every cycle, which is the exact failure SENSORLESS
            // exists to avoid. Manual cannot be rejected (needs nothing).
            defmt::warn!(
                "sensorless boot source rejected (partial motor params?); falling back to Manual"
            );
            let _ = phase_manager.set_source(PhaseSource::Manual);
        }
    }

    // Physics acceleration prior for the observer PLL (ZD2808 interim
    // numbers, like the decoupling override below): |ω̂| growth capped at
    // floor + per_amp·|iq| el rad/s². per_amp = 1.3 × 1.5·pp²·λ/J with
    // pp = 7, λ = 1.145 mWb, J ≈ 3.2e-5 (measured from the e_q-verified
    // 1.5 A climb, 2026-07-06) ⇒ ≈ 3400; floor 500 covers load-driven
    // acceleration. Catches the slow-phantom escape the slip gate cannot
    // (see BackEmfObserver::set_accel_prior); belongs in config once
    // detection measures J.
    if config.motor_params.is_some() {
        phase_manager.set_observer_accel_prior(500.0, 3400.0);
        // Observer eddy L(f) ladder with MEASURED parameters (2026-07-08,
        // on-device impedance sweep, 100–1680 Hz grid at 0.8 V carrier —
        // scripts session log): single-pole fit L_hf = 15 µH,
        // ΔL = 155 µH, τ = 1.39 ms (all six points within ~15%). The
        // mid-band slip transients are 100–300 Hz events where the true
        // stator flux follows L(f) ≈ 35–103 µH — the flat 24 µH the
        // integrator subtracts under-removes stator flux and every slip
        // kicks the flux vector (the ratchet dossier). The earlier GUESSED
        // ladder (ΔL 105 µH, τ 0.3 ms) failed because its corner sat 5×
        // too high: at 200–300 Hz it over-compensated ~3×. ΔL here bridges
        // from the baked AC L (24 µH) to the measured DC value
        // (~170 µH at the probe bias): 146 µH.
        phase_manager.set_observer_eddy_ladder(146e-6, 1.4e-3);
        // Commutation phase tracker (freq-led REDESIGN, 2026-07-08): a
        // critically-ish damped 2nd-order PLL on the observer angle —
        // torque axis stays with the observer (the frequency-led
        // predecessor was structurally an I/f drive riding a ~90°
        // standing load angle; docs/TODO.md dossier). ωn 60 el rad/s:
        // mid-band wobble (35–100 Hz) attenuated, hunting band followed,
        // acceleration lag ≈ ω̇/ωn² (~8° at cruise drag-limited accel).
        phase_manager.set_phase_tracker(30.0, 1.2);
    }

    // Initialize CORDIC hardware for fast sin/cos in FOC loop
    CordicSinCos::init(cordic_peri);

    // Build FOC controller from stored config (motor params → PI gains → defaults)
    let mut foc_controller =
        FocController::<SvpwmModulator, CordicSinCos>::from_runtime_config(config, initial_vbus_v);

    // Two-inductance override (ZD2808, 2026-07-06): the ω·L·i dq-decoupling
    // needs the FUNDAMENTAL (voltage-pulse) Ld/Lq — with the AC value the
    // cross-coupling was ~4.5× undercompensated and the sustained-1.5 A run
    // diverged into the dq overcurrent at ~800 rad/s. motor_params carries
    // the AC value for the estimation chain (observer, deadshort — both
    // hardware-validated there), so the decoupling gets its own numbers
    // here until MotorParamsConfig grows an explicit second-inductance
    // field (TODO "two-inductance model").
    if config.motor_params.is_some() {
        foc_controller.set_decoupling(Some(oxifoc_core::foc::controller::Decoupling {
            ld_h: 85.7e-6,
            lq_h: 129.4e-6,
            flux_linkage_wb: 1.145e-3,
        }));
    }

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
        // DWT cycle counter for ISR-cost stats (ISR_CYC_* atomics).
        let mut cp = cortex_m::Peripherals::steal();
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
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

    // Publish the measured zero-current offsets into the DcOffsets config
    // group. This board has no flash storage (baked config leaves the group
    // None), and the host's telemetry enrichment falls back to mid-scale
    // ADC counts for a missing group — shifting reconstructed phase currents
    // by ~15 A. The boot measurement is the ground truth for this power-up.
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

    let isr_t0 = cortex_m::peripheral::DWT::cycle_count();

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

    // NTC Beta-model conversion runs libm::logf + several float divides
    // (~200 cycles) — decimate to every 128th cycle (156 Hz); the FET
    // thermal time constant is seconds, and consumers read the atomic.
    let convert_temp = *SEQ & 127 == 0;

    // Read the injected sequences straight from the JDR registers — the
    // ISR is the only reader, and going through the CS+RefCell handle
    // locks cost ~370 cycles/cycle of pure locking (2026-07-06 tier-2
    // profiling). The `InjectedAdc` handles stay parked in
    // ADC1_INJECTED/ADC2_INJECTED to keep the peripherals alive (their
    // Drop stops the ADC); this IRQ is only unmasked after init stores
    // them and starts the trigger, so the registers are always live here.
    // Semantics match `read_injected_samples` (JDR reads, then JEOS
    // clear), with a plain w1c write instead of embassy's read-modify-
    // write (which could eat other pending flags).
    let adc1 = embassy_stm32::pac::ADC1;
    let adc2 = embassy_stm32::pac::ADC2;
    // ADC1 injected: phase A current, VBUS voltage, FET temperature
    let ia_raw: u16 = adc1.jdr(0).read().jdata();
    let vbus_raw: u16 = adc1.jdr(1).read().jdata();
    let temp_raw: u16 = adc1.jdr(2).read().jdata();
    adc1.isr().write(|w| w.set_jeos(true));
    // ADC2 injected: phase B and C currents
    let ib_raw: u16 = adc2.jdr(0).read().jdata();
    let ic_raw: u16 = adc2.jdr(1).read().jdata();
    adc2.isr().write(|w| w.set_jeos(true));

    IA_SAMPLE.store(ia_raw, Ordering::Relaxed);
    // Convert VBUS raw ADC to millivolts
    let vbus_mv: u32 = BOARD.vbus_mv_from_adc(vbus_raw);
    VBUS_MV.store(vbus_mv, Ordering::Relaxed);
    // Convert temperature raw ADC to 0.1°C units
    let temp_c_x10: i16 = if convert_temp {
        let t = NTC.temp_c_x10_from_adc(temp_raw, BOARD.calib.adc_max_counts);
        FET_TEMP_C_X10.store(t, Ordering::Relaxed);
        t
    } else {
        FET_TEMP_C_X10.load(Ordering::Relaxed)
    };

    let prof_t1 = cortex_m::peripheral::DWT::cycle_count();
    IB_SAMPLE.store(ib_raw, Ordering::Relaxed);
    IC_SAMPLE.store(ic_raw, Ordering::Relaxed);

    let prof_t2 = cortex_m::peripheral::DWT::cycle_count();

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

    // Get Hall snapshot. On the sensorless build the hall inputs are not
    // wired: the estimator would interpolate garbage every cycle (CS +
    // RefCell + sample math) for a snapshot nothing consumes — skip it at
    // compile time.
    let hall_snapshot = if crate::config::SENSORLESS {
        None
    } else {
        hall::get_snapshot(now_ticks)
    };

    let prof_t3 = cortex_m::peripheral::DWT::cycle_count();

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

    let prof_t4 = cortex_m::peripheral::DWT::cycle_count();

    // Update global state + fast telemetry stream
    // TODO: remove this fallback once motor PSU is connected for testing
    publish_cycle_telemetry(
        &STATE,
        adc_snapshot,
        hall_snapshot,
        foc_telem.unwrap_or_default(),
        MOTOR_POLE_PAIRS.load(Ordering::Relaxed),
        *SEQ,
    );

    let prof_t5 = cortex_m::peripheral::DWT::cycle_count();

    // Feed the IWDG: a completed FOC cycle is the board's liveness signal.
    feed_watchdog();

    ISR_PROF_ADC1.fetch_add(prof_t1.wrapping_sub(isr_t0), Ordering::Relaxed);
    ISR_PROF_ADC2.fetch_add(prof_t2.wrapping_sub(prof_t1), Ordering::Relaxed);
    ISR_PROF_SNAP.fetch_add(prof_t3.wrapping_sub(prof_t2), Ordering::Relaxed);
    ISR_PROF_FOC.fetch_add(prof_t4.wrapping_sub(prof_t3), Ordering::Relaxed);
    ISR_PROF_PUB.fetch_add(prof_t5.wrapping_sub(prof_t4), Ordering::Relaxed);

    let isr_dt = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(isr_t0);
    ISR_CYC_SUM.fetch_add(isr_dt, Ordering::Relaxed);
    ISR_CYC_MAX.fetch_max(isr_dt, Ordering::Relaxed);
    ISR_CYC_N.fetch_add(1, Ordering::Relaxed);
    // Budget-overrun counter: cycles that ate the whole 8500-cycle period
    // (thread mode got nothing). A burst of these at drive engage = the
    // executor-stall mechanism behind the 2026-07-06 deadman trips.
    if isr_dt > 8_500 {
        ISR_CYC_OVER.fetch_add(1, Ordering::Relaxed);
    }
}

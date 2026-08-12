//! Hall sensor management for NUCLEO-G474RE + X-NUCLEO-IHM08M1
//!
//! Hall acquisition via the TIM2 hall-sensor interface. The shield's J3
//! hall/encoder connector lands on PA15/PB3/PB10 (see
//! docs/hw/nucleo-g474re-ihm08m1.md), which are exactly TIM2_CH1/CH2/CH3
//! (AF1). `CR2.TI1S` XORs the three inputs into TI1; IC1 captures the
//! filtered XOR signal on both edges, so every hall transition latches its
//! timestamp in hardware — the capture ISR (below the FOC ISR in priority)
//! only picks the latched value up. The ICF input filter (~6 µs stable
//! window) provides hardware debounce.
//!
//! TIM2 is 32-bit: at 1 MHz the counter wraps every ~71.6 minutes (vs
//! 65.5 ms for the 16-bit timers on G431/F405); the same `CaptureTimebase`
//! extends it to 64 bits. The embassy time driver was moved to TIM5 to
//! free TIM2 for this (Cargo.toml `time-driver-tim5`).
//!
//! Hall pull-ups come from the shield (JP3 closed); MCU pull-ups are
//! enabled too — both is fine.

#![allow(dead_code)] // Motor stack dormant until the IHM08M1 shield is connected

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::timer::input_capture::CaptureInput;
use embassy_stm32::timer::low_level::{InputCaptureMode, Timer};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Channel};
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;

use oxifoc_core::foc::capture_timebase::CaptureTimebase;
use oxifoc_core::foc::hall_embassy::{set_tick_source, update_hall_edge};

// Re-export shared items from core (consumed by control/foc.rs once the
// motor stack is re-enabled).
#[allow(unused_imports)]
pub use oxifoc_core::foc::hall_embassy::{
    HallAngleProxy, apply_stored_config, get_snapshot, init_estimator,
};

/// Hall timebase tick rate: TIM2 counts at 1 MHz.
pub const HALL_TICKS_PER_SEC: u64 = 1_000_000;

/// Keeps TIM2 alive (RCC enabled / not reused); the ISR talks to
/// `pac::TIM2` directly and never takes this lock.
static TIM_DRIVER: CriticalSectionMutex<RefCell<Option<Timer<'static, peripherals::TIM2>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// 32-bit CCR/CNT → 64-bit tick extension (overflow accounting).
static TIMEBASE: CaptureTimebase<u32> = CaptureTimebase::new();

/// Capture overruns: an edge arrived before the previous one was picked up,
/// so one timestamp was lost (the estimator then sees a wider sector).
/// Diagnostics only.
pub static OVERCAPTURES: AtomicU32 = AtomicU32::new(0);

/// "Now" in hall ticks (µs), assembled from TIM2 CNT + overflow count.
///
/// Valid after [`init_hall`]. Safe from any context: the FOC ISR cannot
/// observe a torn overflow update (the writer holds a critical section),
/// thread context retries on concurrent updates.
pub fn now_ticks() -> u64 {
    TIMEBASE.now(|| {
        // CNT before UIF: if the wrap lands between the two reads, UIF is
        // seen with an upper-half CNT and correctly not re-counted.
        let cnt = pac::TIM2.cnt().read();
        let uif = pac::TIM2.sr().read().uif();
        (cnt, uif)
    })
}

/// Clear selected TIM2 status flags (race-free rc_w0 complement write,
/// see [`oxifoc_core::clear_rc_w0`]).
fn clear_flags(uif: bool, cc1of: bool) {
    oxifoc_core::clear_rc_w0!(pac::TIM2.sr(), |w| {
        if uif {
            w.set_uif(false);
        }
        if cc1of {
            w.set_ccof(0, false);
        }
    });
}

/// Initialize hall acquisition: pins to TIM2 AF, XOR + capture setup.
pub fn init_hall(
    pa15: Peri<'static, peripherals::PA15>,
    pb3: Peri<'static, peripherals::PB3>,
    pb10: Peri<'static, peripherals::PB10>,
    tim2: Peri<'static, peripherals::TIM2>,
) {
    // Pins to TIM2 channel inputs with pull-ups (CapturePin sets AF).
    // GPIO IDR still reflects pin levels in AF mode, so raw state reads
    // for calibration keep working.
    let ch1: CaptureInput<'static, peripherals::TIM2, Ch1> =
        CaptureInput::from_pin(pa15, Pull::Up).expect("PA15 supports TIM2_CH1");
    let ch2: CaptureInput<'static, peripherals::TIM2, Ch2> =
        CaptureInput::from_pin(pb3, Pull::Up).expect("PB3 supports TIM2_CH2");
    let ch3: CaptureInput<'static, peripherals::TIM2, Ch3> =
        CaptureInput::from_pin(pb10, Pull::Up).expect("PB10 supports TIM2_CH3");
    // Pins must stay configured for the lifetime of the firmware: dropping
    // a CapturePin reverts the pin from AF mode and kills hall capture.
    #[expect(clippy::mem_forget, reason = "deliberate leak keeps AF pin config")]
    core::mem::forget((ch1, ch2, ch3));

    init_estimator(HALL_TICKS_PER_SEC);
    set_tick_source(now_ticks);

    let timer = Timer::new(tim2);
    let clk = u64::from(timer.get_clock_frequency().0);
    // Derive PSC from the actual RCC config — TIM2 is on APB1, whose timer
    // clock doubles when the APB prescaler is > 1. Hardcoding would
    // silently skew every hall velocity by that factor.
    let psc = clk / HALL_TICKS_PER_SEC - 1;
    defmt::assert!(
        clk.is_multiple_of(HALL_TICKS_PER_SEC) && psc <= u64::from(u16::MAX),
        "TIM2 clock {} not divisible to 1 MHz",
        clk
    );
    // PSC/CR1/CR2/CCMR/CCER/EGR/SR share the GP16 register layout; CNT and
    // ARR are the 32-bit registers (regs_gp32).
    let regs16 = timer.regs_gp16();
    let regs32 = timer.regs_gp32();
    regs16.psc().write_value(psc as u16);
    regs32.arr().write_value(0xFFFF_FFFF);
    // f_DTS = timer clock / 4; ICF = f_DTS/32, 8 samples: the XOR signal
    // must be stable ~6 µs before an edge is accepted. Hardware debounce —
    // the filter delay is identical for every edge, so it cancels in
    // velocity math.
    regs16
        .cr1()
        .modify(|w| w.set_ckd(pac::timer::vals::Ckd::Div4));
    // XOR CH1^CH2^CH3 → TI1: any single hall transition flips parity.
    regs16
        .cr2()
        .modify(|w| w.set_ti1s(pac::timer::vals::Ti1s::Xor));
    timer.set_input_ti_seletion(Channel::Ch1, 0);
    timer.set_input_capture_filter(Channel::Ch1, pac::timer::vals::FilterValue::FdtsDiv32N8);
    timer.set_input_capture_mode(Channel::Ch1, InputCaptureMode::BothEdges);
    // Latch PSC into the shadow register (UG sets UIF as a side effect —
    // clear before enabling interrupts).
    regs16.egr().write(|w| w.set_ug(true));
    clear_flags(true, true);
    timer.enable_channel(Channel::Ch1, true);
    timer.enable_input_interrupt(Channel::Ch1, true);
    timer.enable_update_interrupt(true);
    timer.start();

    TIM_DRIVER.lock(|cell| cell.replace(Some(timer)));

    // Seed the estimator with the boot-time hall state: the sector is known
    // from the pin levels before any edge arrives. Without this, `sample()`
    // is None until the rotor moves — on a sensored cold start the first
    // torque command would commutate via the open-loop recovery override
    // instead of the actual angle. A disconnected cable (pull-ups read
    // 0b111) surfaces immediately as an invalid state. Safe: the capture
    // interrupt is not unmasked yet.
    update_hall_edge(read_hall_state(), now_ticks());

    #[expect(
        clippy::multiple_unsafe_ops_per_block,
        reason = "single logical operation: hall-capture IRQ bring-up"
    )]
    // SAFETY: one-time IRQ bring-up during init, before the capture
    // interrupt can fire; Peripherals::steal() only touches NVIC priority
    // registers nothing else owns at this point.
    unsafe {
        interrupt::typelevel::TIM2::unpend();
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
        let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
        // NVIC::set_priority takes the RAW 8-bit IPR value; STM32 implements
        // only the upper 4 bits. Below the FOC ADC ISR is fine: edge
        // timestamps are latched in hardware, so delaying this handler only
        // delays when the estimator learns of the edge, not the timestamp.
        nvic.set_priority(interrupt::TIM2, 1 << 4);
    }

    defmt::info!(
        "Hall: TIM2 XOR capture @ 1 MHz (clk {} Hz, psc {}), H1=PA15 H2=PB3 H3=PB10",
        clk,
        psc
    );
}

/// Read Hall state from GPIO. Unlike G431/F405 the three inputs span two
/// ports (shield routing): H1=PA15 (bit 0), H2=PB3 (bit 1), H3=PB10 (bit 2).
#[inline(always)]
fn read_hall_state() -> u8 {
    let a = pac::GPIOA.idr().read().0;
    let b = pac::GPIOB.idr().read().0;
    (((a >> 15) & 1) | (((b >> 3) & 1) << 1) | (((b >> 10) & 1) << 2)) as u8
}

/// Read raw Hall sensor state from GPIO (public for calibration).
#[inline]
pub fn read_hall_state_raw() -> u8 {
    read_hall_state()
}

/// TIM2 capture/update interrupt: extend the hardware-latched edge
/// timestamp and feed the estimator.
#[interrupt]
fn TIM2() {
    let regs = pac::TIM2;
    let sr = regs.sr().read();

    if sr.ccof(0) {
        OVERCAPTURES.fetch_add(1, Ordering::Relaxed);
        clear_flags(false, true);
    }

    if sr.ccif(0) {
        // Reading CCR1 clears CC1IF.
        let captured = regs.ccr(0).read();
        let ticks = TIMEBASE.capture(captured, sr.uif(), || clear_flags(true, false));
        update_hall_edge(read_hall_state(), ticks);
    } else if sr.uif() {
        TIMEBASE.overflow(|| clear_flags(true, false));
    }
}

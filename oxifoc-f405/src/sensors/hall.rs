//! Hall sensor management for Simple FOCer 2 (STM32F405)
//!
//! Hall acquisition via the TIM3 hall-sensor interface: PC6/PC7/PC8 are
//! TIM3_CH1/2/3 (AF2). `CR2.TI1S` XORs the three inputs into TI1; IC1
//! captures the filtered XOR signal on both edges, so every hall transition
//! latches its timestamp in hardware. The capture ISR (below the FOC ISR in
//! priority) only picks the latched value up — its latency no longer affects
//! hall timing, unlike the 200 kHz TIM6 polling this replaces. The ICF input
//! filter replaces the 7-read majority vote, and the free-running 1 MHz
//! counter replaces 32768 Hz embassy timestamps.
//!
//! (PC6-8 also map to TIM8_CH1/2/3; TIM8 — the second advanced timer — is
//! deliberately left free.)

#![allow(dead_code)] // Public API not yet wired to protocol handlers

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::timer::input_capture::CapturePin;
use embassy_stm32::timer::low_level::{InputCaptureMode, InputTISelection, Timer};
use embassy_stm32::timer::{Ch1, Ch2, Ch3, Channel};
use embassy_stm32::{Peri, interrupt, pac, peripherals};
use embassy_sync::blocking_mutex::CriticalSectionMutex;

use oxifoc_core::foc::capture_timebase::CaptureTimebase;
use oxifoc_core::foc::hall_embassy::{set_tick_source, update_hall_edge};

// Re-export shared items from core
pub use oxifoc_core::foc::hall_embassy::{
    HallAngleProxy, apply_stored_config, get_snapshot, init_estimator,
};

/// Hall timebase tick rate: TIM3 counts at 1 MHz.
pub const HALL_TICKS_PER_SEC: u64 = 1_000_000;

/// Keeps TIM3 alive (RCC enabled / not reused); the ISR talks to
/// `pac::TIM3` directly and never takes this lock.
static TIM_DRIVER: CriticalSectionMutex<RefCell<Option<Timer<'static, peripherals::TIM3>>>> =
    CriticalSectionMutex::new(RefCell::new(None));

/// 16-bit CCR/CNT → 64-bit tick extension (overflow accounting).
static TIMEBASE: CaptureTimebase = CaptureTimebase::new();

/// Capture overruns: an edge arrived before the previous one was picked up,
/// so one timestamp was lost (the estimator then sees a wider sector).
/// Diagnostics only.
pub static OVERCAPTURES: AtomicU32 = AtomicU32::new(0);

/// "Now" in hall ticks (µs), assembled from TIM3 CNT + overflow count.
///
/// Valid after [`init_hall`]. Safe from any context: the FOC ISR cannot
/// observe a torn overflow update (the writer holds a critical section),
/// thread context retries on concurrent updates.
pub fn now_ticks() -> u64 {
    TIMEBASE.now(|| {
        // CNT before UIF: if the wrap lands between the two reads, UIF is
        // seen with an upper-half CNT and correctly not re-counted.
        let cnt = pac::TIM3.cnt().read().cnt();
        let uif = pac::TIM3.sr().read().uif();
        (cnt, uif)
    })
}

/// Clear selected TIM3 status flags. SR is rc_w0: writing 1 leaves a flag
/// untouched, 0 clears it — write the complement mask (ST LL does the same)
/// instead of read-modify-write, which would also clear any flag that set
/// between the read and the write.
fn clear_flags(uif: bool, cc1of: bool) {
    let mut v = pac::TIM3.sr().read();
    v.0 = u32::MAX;
    if uif {
        v.set_uif(false);
    }
    if cc1of {
        v.set_ccof(0, false);
    }
    pac::TIM3.sr().write_value(v);
}

/// Initialize hall acquisition: pins to TIM3 AF, XOR + capture setup.
pub fn init_hall(
    pc6: Peri<'static, peripherals::PC6>,
    pc7: Peri<'static, peripherals::PC7>,
    pc8: Peri<'static, peripherals::PC8>,
    tim3: Peri<'static, peripherals::TIM3>,
) {
    // Pins to TIM3 channel inputs with pull-ups (CapturePin sets AF).
    // GPIO IDR still reflects pin levels in AF mode, so raw state reads
    // for calibration keep working.
    let ch1: CapturePin<'static, peripherals::TIM3, Ch1> = CapturePin::new(pc6, Pull::Up);
    let ch2: CapturePin<'static, peripherals::TIM3, Ch2> = CapturePin::new(pc7, Pull::Up);
    let ch3: CapturePin<'static, peripherals::TIM3, Ch3> = CapturePin::new(pc8, Pull::Up);
    core::mem::forget((ch1, ch2, ch3));

    init_estimator(HALL_TICKS_PER_SEC);
    set_tick_source(now_ticks);

    let timer = Timer::new(tim3);
    let clk = timer.get_clock_frequency().0 as u64;
    // Derive PSC from the actual RCC config — TIM3 is on APB1, whose timer
    // clock doubles when the APB prescaler is > 1 (84 MHz here at 168 MHz
    // sysclk). Hardcoding would silently skew every hall velocity.
    let psc = clk / HALL_TICKS_PER_SEC - 1;
    defmt::assert!(
        clk.is_multiple_of(HALL_TICKS_PER_SEC) && psc <= u16::MAX as u64,
        "TIM3 clock {} not divisible to 1 MHz",
        clk
    );
    let regs = timer.regs_gp16();
    regs.psc().write_value(psc as u16);
    regs.arr().write(|w| w.set_arr(0xFFFF));
    // f_DTS = timer clock / 4; ICF = f_DTS/32, 8 samples: the XOR signal
    // must be stable ~12 µs (at 84 MHz) before an edge is accepted.
    // Hardware debounce replacing the 7-read majority vote — and unlike
    // software voting, the filter delay is identical for every edge, so it
    // cancels in velocity math.
    regs.cr1()
        .modify(|w| w.set_ckd(pac::timer::vals::Ckd::DIV4));
    // XOR CH1^CH2^CH3 → TI1: any single hall transition flips parity.
    regs.cr2()
        .modify(|w| w.set_ti1s(pac::timer::vals::Ti1s::XOR));
    timer.set_input_ti_selection(Channel::Ch1, InputTISelection::Normal);
    timer.set_input_capture_filter(Channel::Ch1, pac::timer::vals::FilterValue::FDTS_DIV32_N8);
    timer.set_input_capture_mode(Channel::Ch1, InputCaptureMode::BothEdges);
    // Latch PSC into the shadow register (UG sets UIF as a side effect —
    // clear before enabling interrupts).
    regs.egr().write(|w| w.set_ug(true));
    clear_flags(true, true);
    timer.enable_channel(Channel::Ch1, true);
    timer.enable_input_interrupt(Channel::Ch1, true);
    timer.enable_update_interrupt(true);
    timer.start();

    TIM_DRIVER.lock(|cell| cell.replace(Some(timer)));

    unsafe {
        interrupt::typelevel::TIM3::unpend();
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
        let mut nvic = cortex_m::peripheral::Peripherals::steal().NVIC;
        // NVIC::set_priority takes the RAW 8-bit IPR value; STM32 implements
        // only the upper 4 bits. Below the FOC ADC ISR is fine: edge
        // timestamps are latched in hardware, so delaying this handler only
        // delays when the estimator learns of the edge, not the timestamp.
        nvic.set_priority(interrupt::TIM3, 1 << 4);
    }

    defmt::info!(
        "Hall: TIM3 XOR capture @ 1 MHz (clk {} Hz, psc {}), H1=PC6 H2=PC7 H3=PC8",
        clk,
        psc
    );
}

// ========== Fast Hall State Reading ==========

/// Read Hall state from GPIOC IDR in a single register access.
/// PC6=H1 (bit 0), PC7=H2 (bit 1), PC8=H3 (bit 2)
#[inline(always)]
fn read_hall_idr() -> u8 {
    // Single 32-bit read, extract bits 6-8, shift to bits 0-2
    ((pac::GPIOC.idr().read().0 >> 6) & 0b111) as u8
}

/// Read raw Hall sensor state (public API for calibration).
///
/// Returns 3-bit Hall state (0-7): H3<<2 | H2<<1 | H1
///
/// INIT ORDER: init_hall() must be called before any use of this function.
#[inline]
pub fn read_hall_state_raw() -> u8 {
    read_hall_idr()
}

// ========== TIM3 Interrupt Handler ==========

/// TIM3 capture/update interrupt: extend the hardware-latched edge
/// timestamp and feed the estimator.
#[interrupt]
fn TIM3() {
    let regs = pac::TIM3;
    let sr = regs.sr().read();

    if sr.ccof(0) {
        OVERCAPTURES.fetch_add(1, Ordering::Relaxed);
        clear_flags(false, true);
    }

    if sr.ccif(0) {
        // Reading CCR1 clears CC1IF.
        let captured = regs.ccr(0).read().0 as u16;
        let ticks = TIMEBASE.capture(captured, sr.uif(), || clear_flags(true, false));
        update_hall_edge(read_hall_idr(), ticks);
    } else if sr.uif() {
        TIMEBASE.overflow(|| clear_flags(true, false));
    }
}

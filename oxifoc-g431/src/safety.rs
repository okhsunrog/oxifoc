//! Last-resort safety: gate kill on panic/HardFault and the independent
//! watchdog.
//!
//! Replaces `panic_probe`: a panic must not leave TIM1 commutating with
//! the last duty cycles, so the handlers below clear BDTR.MOE (all six
//! gate outputs drop to their inactive state) before any reporting.
//!
//! The IWDG covers what a panic handler can't: hard lockups and a wedged
//! FOC ISR. The ADC ISR feeds the dog every PWM cycle; if it stops, the
//! chip resets and reboots with PWM outputs disabled.

use embassy_stm32::{Peri, pac, peripherals, wdg::IndependentWatchdog};

/// Watchdog timeout. Must outlive the longest CPU stall with no ISR
/// running: a flash page erase (~25 ms on G4) stalls the core, since
/// code executes from the same flash bank.
const IWDG_TIMEOUT_US: u32 = 100_000;

/// Drop all gate outputs at the register level. No ownership of TIM1
/// needed — callable from panic/fault context.
pub fn kill_gates() {
    // MOE=0 puts every TIM1 output into its inactive state immediately.
    pac::TIM1.bdtr().modify(|w| w.set_moe(false));
}

/// Configure and start the IWDG. Call only once the FOC ISR is running —
/// it is the sole feeder.
pub fn arm_watchdog(iwdg: Peri<'static, peripherals::IWDG>) {
    // Stop the IWDG while a debugger has the core halted, otherwise
    // every breakpoint ends in a watchdog reset.
    pac::DBGMCU.apb1lfzr().modify(|w| w.set_iwdg(true));
    let mut wdg = IndependentWatchdog::new(iwdg, IWDG_TIMEOUT_US);
    wdg.unleash();
}

/// Feed the watchdog. Raw register write so the ISR doesn't need to own
/// the peripheral. A no-op until [`arm_watchdog`] runs.
#[inline]
pub fn feed_watchdog() {
    pac::IWDG
        .kr()
        .write(|w| w.set_key(pac::iwdg::vals::Key::RESET));
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    kill_gates();
    defmt::error!("PANIC: {}", defmt::Display2Format(info));
    // Same exit as panic-probe: UDF halts the core under a debugger
    // (vector catch); standalone it escalates to lockup and the IWDG
    // resets the board.
    cortex_m::asm::udf();
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    kill_gates();
    defmt::error!(
        "HARD FAULT: pc={=u32:#x} lr={=u32:#x}",
        frame.pc(),
        frame.lr()
    );
    // Under a debugger the vector catch halted us before this handler
    // ran; standalone, spin until the IWDG resets the board.
    loop {
        core::hint::spin_loop();
    }
}

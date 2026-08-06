//! Last-resort safety: gate kill on panic/HardFault.
//!
//! Replaces `panic_probe`: a panic must not leave TIM1 commutating with
//! the last duty cycles, so the handlers below clear BDTR.MOE (all six
//! gate outputs drop to their inactive state) before any reporting.
//!
//! No IWDG here yet: the motor modules (and the FOC ISR that would feed
//! the dog) are dormant until the IHM08M1 shield is connected. When
//! control/foc.rs comes back to life, mirror the f405 safety.rs —
//! arm_watchdog(p.IWDG) after foc::init, feed from the ADC ISR,
//! DBGMCU.apb1lfzr().set_iwdg(true) for debugging.

use embassy_stm32::pac;

/// Drop all gate outputs at the register level. No ownership of TIM1
/// needed — callable from panic/fault context. Harmless while the motor
/// stage is dormant (TIM1 isn't even running).
pub fn kill_gates() {
    // MOE=0 puts every TIM1 output into its inactive state immediately.
    pac::TIM1.bdtr().modify(|w| w.set_moe(false));
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    kill_gates();
    defmt::error!("PANIC: {}", defmt::Display2Format(info));
    // Same exit as panic-probe: UDF halts the core under a debugger
    // (vector catch); standalone it escalates to lockup.
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
    loop {
        core::hint::spin_loop();
    }
}

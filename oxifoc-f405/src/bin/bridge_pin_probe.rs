//! MK5 connector continuity test. PB10 stays high impedance.
//! No FOC, USART, SPI or flash-storage code. Repeats a finite slow pattern,
//! or observes input levels with bench-bridge-pins-rx.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
// Supply the existing defmt sink used by HAL diagnostics; no network is started.
use ergot as _;

/// 0: initializing, 2: running, 3: complete.
#[unsafe(no_mangle)]
static BRIDGE_PIN_STATE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_COUNT: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_ERRORS: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_SEEN_HIGH: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_SEEN_LOW: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_EDGES: AtomicU32 = AtomicU32::new(0);

#[cortex_m_rt::entry]
fn main() -> ! {
    // Default HSI avoids any dependence on the bench HSE calibration.
    let p = embassy_stm32::init(Default::default());
    let _latch = Output::new(p.PC5, Level::High, Speed::Low);
    let _gate = Output::new(p.PB5, Level::Low, Speed::Low);
    let _phase_outputs = [
        Output::new(p.PA8, Level::Low, Speed::Low),
        Output::new(p.PA9, Level::Low, Speed::Low),
        Output::new(p.PA10, Level::Low, Speed::Low),
        Output::new(p.PB13, Level::Low, Speed::Low),
        Output::new(p.PB14, Level::Low, Speed::Low),
        Output::new(p.PB15, Level::Low, Speed::Low),
    ];
    let _leds = [
        Output::new(p.PB0, Level::Low, Speed::Low),
        Output::new(p.PB1, Level::Low, Speed::Low),
    ];
    let shared = Input::new(p.PB10, Pull::None);
    // Bit order: MOSI, SCK, MISO, CS, IRQ.
    let mut lines = [
        Flex::new(p.PA7),
        Flex::new(p.PA5),
        Flex::new(p.PA6),
        Flex::new(p.PB11),
        Flex::new(p.PB6),
    ];
    #[cfg(feature = "bench-bridge-pins-rx")]
    for line in &mut lines {
        line.set_as_input(Pull::Down);
    }
    #[cfg(not(feature = "bench-bridge-pins-rx"))]
    for line in &mut lines {
        line.set_low();
        line.set_as_output(Speed::Low);
    }
    BRIDGE_PIN_STATE.store(2, Ordering::Release);
    #[cfg(not(feature = "bench-bridge-pins-rx"))]
    for word in (0u8..32).cycle().take(256) {
        for (bit, line) in lines.iter_mut().enumerate() {
            line.set_level(if word & (1 << bit) != 0 {
                Level::High
            } else {
                Level::Low
            });
        }
        cortex_m::asm::delay(1_600_000);
        if shared.is_high() != (word & 1 != 0) {
            BRIDGE_PIN_ERRORS.fetch_add(1, Ordering::Relaxed);
        }
        BRIDGE_PIN_COUNT.fetch_add(1, Ordering::Relaxed);
        cortex_m::asm::delay(1_600_000);
    }
    #[cfg(feature = "bench-bridge-pins-rx")]
    {
        let mut previous = 0;
        for _ in 0..20_000 {
            let mut mask = 0;
            for (bit, line) in lines.iter().enumerate() {
                mask |= u32::from(line.is_high()) << bit;
            }
            mask |= u32::from(shared.is_high()) << 5;
            BRIDGE_PIN_SEEN_HIGH.fetch_or(mask, Ordering::Relaxed);
            BRIDGE_PIN_SEEN_LOW.fetch_or(!mask & 0x3f, Ordering::Relaxed);
            if mask != previous {
                BRIDGE_PIN_EDGES.fetch_add(1, Ordering::Relaxed);
            }
            previous = mask;
            BRIDGE_PIN_COUNT.fetch_add(1, Ordering::Relaxed);
            cortex_m::asm::delay(16_000);
        }
    }
    #[cfg(not(feature = "bench-bridge-pins-rx"))]
    for line in &mut lines {
        line.set_low();
    }
    BRIDGE_PIN_STATE.store(3, Ordering::Release);
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    embassy_stm32::pac::TIM1.bdtr().modify(|w| w.set_moe(false));
    embassy_stm32::pac::GPIOB
        .bsrr()
        .write(|w| w.set_br(5, true));
    loop {
        cortex_m::asm::wfi();
    }
}

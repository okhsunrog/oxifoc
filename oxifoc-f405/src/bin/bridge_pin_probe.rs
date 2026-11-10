//! Finite MK5 connector capture. No FOC, USART, SPI or flash-storage code.
//! Feed a slow GPIO pattern from the bridge only after this image is running.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
// Supply the existing defmt sink used by HAL diagnostics; no network is started.
use ergot as _;

const SAMPLES: usize = 8192;

/// 0: initializing, 1: waiting for PB6 high, 2: recording, 3: frozen.
#[unsafe(no_mangle)]
static BRIDGE_PIN_STATE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static BRIDGE_PIN_COUNT: AtomicU32 = AtomicU32::new(0);
/// Bits 0..5: PB6, PA5, PA6, PB10, PB11, PA7. Read only after state == 3.
#[unsafe(no_mangle)]
static mut BRIDGE_PIN_SAMPLES: [u8; SAMPLES] = [0; SAMPLES];

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
    let inputs = [
        Input::new(p.PB6, Pull::Down),
        Input::new(p.PA5, Pull::Down),
        Input::new(p.PA6, Pull::Down),
        Input::new(p.PB10, Pull::Down),
        Input::new(p.PB11, Pull::Down),
        Input::new(p.PA7, Pull::Down),
    ];
    BRIDGE_PIN_STATE.store(1, Ordering::Release);
    while !inputs[0].is_high() {
        cortex_m::asm::nop();
    }
    BRIDGE_PIN_STATE.store(2, Ordering::Release);
    for index in 0..SAMPLES {
        let mut mask = 0u8;
        for (bit, pin) in inputs.iter().enumerate() {
            mask |= u8::from(pin.is_high()) << bit;
        }
        // SAFETY: sole writer; the host reads only after the frozen state.
        unsafe {
            core::ptr::addr_of_mut!(BRIDGE_PIN_SAMPLES)
                .cast::<u8>()
                .add(index)
                .write_volatile(mask);
        }
        BRIDGE_PIN_COUNT.store((index + 1) as u32, Ordering::Relaxed);
        // Approximate 1 ms sampling, sufficient for a 40-ms GPIO codeword.
        cortex_m::asm::delay(16_000);
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

//! Finite SPI1 DMA qualification; no motor control or persistent writes.
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull, Speed},
    peripherals,
    rcc::*,
    spi::{Config, Spi},
    time::Hertz,
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use ergot as _;

bind_interrupts!(struct Irqs {
    DMA2_STREAM0 => dma::InterruptHandler<peripherals::DMA2_CH0>;
    DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
});

// Per case: SPI clock Hz, length, completed frames, byte errors, HAL errors,
// elapsed us (includes ready handshake), DMA transfer us, first bad byte + 1.
#[unsafe(no_mangle)]
static SPI_BENCH_RESULTS: [AtomicU32; 26 * 8] = [const { AtomicU32::new(0) }; 26 * 8];
/// 0: boot, 1: waiting, 2: running, 3: complete, 4: timeout/error.
#[unsafe(no_mangle)]
static SPI_BENCH_STATE: AtomicU32 = AtomicU32::new(0);
#[unsafe(no_mangle)]
static SPI_BENCH_CASE: AtomicU32 = AtomicU32::new(0);

fn pattern(seq: u32, index: usize, salt: u8) -> u8 {
    if index < 4 {
        seq.to_le_bytes()[index] ^ salt
    } else {
        (seq.rotate_left((index % 31) as u32) ^ (index as u32 * 29)) as u8 ^ salt
    }
}

async fn wait_level(pin: &Input<'_>, high: bool, timeout_s: u64) -> bool {
    with_timeout(Duration::from_secs(timeout_s), async {
        while pin.is_high() != high {
            Timer::after_micros(10).await;
        }
    })
    .await
    .is_ok()
}

async fn park() -> ! {
    // Keep latch, CS, gate inhibit and DMA ownership stable for inspection.
    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // HSI PLL deliberately isolates this electrical test from the board's HSE
    // calibration. Clock rates in the result are nominal HSI-based rates.
    let mut clock = embassy_stm32::Config::default();
    clock.rcc.pll_src = PllSource::Hsi;
    clock.rcc.pll = Some(Pll {
        prediv: PllPreDiv::Div16,
        mul: PllMul::Mul336,
        divp: Some(PllPDiv::Div2),
        divq: Some(PllQDiv::Div7),
        divr: None,
    });
    clock.rcc.sys = Sysclk::Pll1P;
    clock.rcc.apb1_pre = APBPrescaler::Div4;
    clock.rcc.apb2_pre = APBPrescaler::Div2;
    let p = embassy_stm32::init(clock);
    let _latch = Output::new(p.PC5, Level::High, Speed::Low);
    let _gate = Output::new(p.PB5, Level::Low, Speed::Low);
    let _phases = [
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
    let _shared = Input::new(p.PB10, Pull::None);
    let ready = Input::new(p.PB6, Pull::Down);
    let mut cs = Output::new(p.PB11, Level::High, Speed::VeryHigh);
    let mut config = Config::default();
    config.frequency = Hertz(328_125);
    let mut spi = Spi::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA2_CH3,
        p.DMA2_CH0,
        Irqs,
        config.clone(),
    );
    let mut tx = [0u8; 512];
    let mut rx = [0u8; 512];
    let mut seq = 0;
    #[cfg(feature = "bench-spi-cs-request")]
    Timer::after_secs(3).await;
    for case in 0..26 {
        let hz = [328_125, 1_312_500, 5_250_000, 10_500_000, 21_000_000][(case / 5).min(4)];
        let len = if case == 25 {
            512
        } else {
            [8, 64, 137, 256, 512][case % 5]
        };
        let frames = if case == 25 { 10_000 } else { 16 };
        let row = &SPI_BENCH_RESULTS[case * 8..case * 8 + 8];
        row[0].store(hz, Ordering::Relaxed);
        row[1].store(len as u32, Ordering::Relaxed);
        SPI_BENCH_CASE.store(case as u32, Ordering::Release);
        config.frequency = Hertz(hz);
        if spi.set_config(&config).is_err() {
            SPI_BENCH_STATE.store(4, Ordering::Release);
            park().await;
        }
        let start = Instant::now();
        for _ in 0..frames {
            for (i, byte) in tx[..len].iter_mut().enumerate() {
                *byte = pattern(seq, i, 0x39);
            }
            rx.fill(0xee);
            SPI_BENCH_STATE.store(1, Ordering::Release);
            // Even slots: master requests by asserting CS before the slave arms.
            // Odd slots: slave initiates by arming DMA and asserting READY first.
            #[cfg(feature = "bench-spi-cs-request")]
            if seq % 2 == 0 {
                cs.set_low();
            }
            if !wait_level(&ready, true, if seq == 0 { 60 } else { 5 }).await {
                SPI_BENCH_STATE.store(4, Ordering::Release);
                park().await;
            }
            SPI_BENCH_STATE.store(2, Ordering::Release);
            cs.set_low();
            let transfer_start = Instant::now();
            let result = with_timeout(
                Duration::from_secs(1),
                spi.transfer(&mut rx[..len], &tx[..len]),
            )
            .await;
            row[6].fetch_add(
                transfer_start.elapsed().as_micros() as u32,
                Ordering::Relaxed,
            );
            cs.set_high();
            if !matches!(result, Ok(Ok(()))) {
                row[4].fetch_add(1, Ordering::Relaxed);
                SPI_BENCH_STATE.store(4, Ordering::Release);
                park().await;
            }
            for (i, byte) in rx[..len].iter().enumerate() {
                if *byte != pattern(seq, i, 0xc6) {
                    row[3].fetch_add(1, Ordering::Relaxed);
                    let _ = row[7].compare_exchange(
                        0,
                        i as u32 + 1,
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    );
                }
            }
            row[2].fetch_add(1, Ordering::Relaxed);
            seq += 1;
            if !wait_level(&ready, false, 5).await {
                SPI_BENCH_STATE.store(4, Ordering::Release);
                park().await;
            }
        }
        row[5].store(start.elapsed().as_micros() as u32, Ordering::Release);
    }
    SPI_BENCH_STATE.store(3, Ordering::Release);
    park().await;
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    embassy_stm32::pac::GPIOB
        .bsrr()
        .write(|w| w.set_br(5, true));
    loop {
        cortex_m::asm::wfi();
    }
}

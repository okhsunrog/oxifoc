//! Bounded clock experiments only: SRAM retention, flash reads, FPU and die
//! temperature alongside the real ADC ISR. Phase outputs are compile-time off.
use core::{
    hint::black_box,
    ptr::{read_volatile, write_volatile},
};
use embassy_stm32::pac;
use embassy_time::{Duration, Instant, with_timeout};

const WORDS: usize = 2048;
const CHUNK: usize = 64;

const fn pattern(index: u32, round: u32) -> u32 {
    index.wrapping_mul(0x9e37_79b9).rotate_left(13) ^ round.wrapping_mul(0x85eb_ca6b)
}

const fn flash_pattern() -> [u32; 512] {
    let mut data = [0; 512];
    let mut i = 0;
    while i < data.len() {
        data[i] = pattern(i as u32, 0x1234);
        i += 1;
    }
    data
}
static FLASH_PATTERN: [u32; 512] = flash_pattern();

const fn expected_fpu(seed: u32) -> u32 {
    let mut x = seed as f32;
    let mut i = 0;
    while i < 32 {
        x = (x * 1.5 + 0.25) * 0.5;
        i += 1;
    }
    x.to_bits()
}
const fn fpu_table() -> [u32; 16] {
    let mut result = [0; 16];
    let mut i = 0;
    while i < 16 {
        result[i] = expected_fpu(i as u32);
        i += 1;
    }
    result
}
static FPU_EXPECTED: [u32; 16] = fpu_table();

/// Owns ADC1's otherwise unused *regular* group after initialization. The
/// injected group stays owned by FOC. Injection preempts regular conversions;
/// no JSQR/JEXTEN, shared prescaler or current sample-time register is changed.
async fn die_temperature_x10() -> i32 {
    let adc = pac::ADC1;
    let _ = adc.dr().read(); // clear a previous regular EOC, never touch JEOC
    adc.cr2().modify(|w| w.set_swstart(true));
    defmt::unwrap!(
        with_timeout(Duration::from_millis(5), async {
            while !adc.sr().read().eoc() {
                embassy_futures::yield_now().await;
            }
        })
        .await,
        "oc: temperature conversion timeout"
    );
    let raw = adc.dr().read().data() as i32;
    // STM32F405 factory TS_CAL1/TS_CAL2: 30/110 degrees at VDDA=3300 mV.
    // SAFETY: aligned, read-only calibration halfwords in system memory.
    let (cal30, cal110) = unsafe {
        (
            read_volatile(0x1fff_7a2c as *const u16) as i32,
            read_volatile(0x1fff_7a2e as *const u16) as i32,
        )
    };
    defmt::assert!(cal30 != cal110, "oc: invalid temperature calibration");
    let normalized = raw * crate::config::measured_board().calib.adc_vref_mv as i32 / 3300;
    300 + (normalized - cal30) * 800 / (cal110 - cal30)
}

#[embassy_executor::task]
pub async fn run() {
    // The static task future owns this buffer; no additional stack allocation
    // or alias with DMA/FOC storage. Alternate writing and delayed verification.
    let mut ram = [0u32; WORDS];
    pac::ADC1.sqr1().modify(|w| w.set_l(0));
    pac::ADC1.sqr3().modify(|w| w.set_sq(0, 16));
    pac::ADC1
        .smpr1()
        .modify(|w| w.set_smp(6, pac::adc::vals::SampleTime::Cycles480));
    let mut round = 1u32;
    let mut index = 0usize;
    let mut verify = false;
    let mut checks = 0u32;
    let mut next_report = Instant::now() + Duration::from_secs(1);
    loop {
        for i in index..index + CHUNK {
            let expected = pattern(i as u32, round);
            // SAFETY: i is in bounds, and this future exclusively owns ram.
            unsafe {
                let p = ram.as_mut_ptr().add(i);
                if verify {
                    defmt::assert_eq!(read_volatile(p), expected, "oc: SRAM failure");
                } else {
                    write_volatile(p, expected);
                }
                let j = i % FLASH_PATTERN.len();
                defmt::assert_eq!(
                    read_volatile(FLASH_PATTERN.as_ptr().add(j)),
                    pattern(j as u32, 0x1234),
                    "oc: flash failure"
                );
            }
        }
        let seed = checks % 16;
        let mut x = black_box(seed as f32);
        for _ in 0..32 {
            x = black_box(x * 1.5);
            x = black_box(x + 0.25);
            x = black_box(x * 0.5);
        }
        defmt::assert_eq!(x.to_bits(), FPU_EXPECTED[seed as usize], "oc: FPU failure");
        checks = checks.wrapping_add(1);
        index += CHUNK;
        if index == WORDS {
            index = 0;
            if verify {
                round = round.wrapping_add(1);
            }
            verify = !verify;
        }
        if Instant::now() >= next_report {
            let temp = die_temperature_x10().await;
            defmt::warn!(
                "oc: cpu={} tim1={} checks={} die_x10={}",
                crate::config::CPU_HZ,
                crate::config::CLOCK_PROFILE.tim1_hz(),
                checks,
                temp
            );
            defmt::assert!((-200..700).contains(&temp), "oc: die temperature limit");
            next_report = Instant::now() + Duration::from_secs(1);
        }
        // Saturate spare CPU with deterministic work, but leave a poll
        // boundary for SPI, protocol and telemetry after every short chunk.
        #[cfg(not(feature = "bench-overclock-light"))]
        embassy_futures::yield_now().await;
        #[cfg(feature = "bench-overclock-light")]
        embassy_time::Timer::after_millis(1).await;
    }
}

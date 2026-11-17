use defmt::info;
use embassy_nrf::saadc::Saadc;
use embassy_time::{Duration, Ticker};

use super::battery;

const SCANS_PER_TICK: i32 = 8;
const TICKS_PER_THROTTLE_LOG: usize = 20;
const TICKS_PER_BATTERY_READING: usize = 200;

#[embassy_executor::task]
pub async fn sample(mut adc: Saadc<'static, 2>) {
    adc.calibrate().await;
    let mut ticker = Ticker::every(Duration::from_millis(5));
    let mut raw = [0i16; 2];

    loop {
        let mut min = i32::MAX;
        let mut max = i32::MIN;
        let mut battery_sum = 0u64;
        for tick in 1..=TICKS_PER_BATTERY_READING {
            ticker.next().await;
            let mut throttle_sum = 0;
            // SAADC hardware oversampling is for single-channel conversions.
            for _ in 0..SCANS_PER_TICK {
                adc.sample(&mut raw).await;
                let [throttle_raw, battery_raw] = raw;
                throttle_sum += i32::from(throttle_raw);
                battery_sum += battery_raw.max(0) as u64;
            }
            let throttle_raw = throttle_sum / SCANS_PER_TICK;
            min = min.min(throttle_raw);
            max = max.max(throttle_raw);
            if tick % TICKS_PER_THROTTLE_LOG == 0 {
                info!("[throttle] raw={} min={} max={}", throttle_raw, min, max);
                min = i32::MAX;
                max = i32::MIN;
            }
        }

        // Voyager's conversion: 3.6 V ADC full scale, 12 bits, VDDH divided by 5.
        let divisor = 4096 * (TICKS_PER_BATTERY_READING as u64) * (SCANS_PER_TICK as u64);
        let millivolts = ((battery_sum * 18_000 + divisor / 2) / divisor) as u32;
        battery::publish(millivolts);
    }
}

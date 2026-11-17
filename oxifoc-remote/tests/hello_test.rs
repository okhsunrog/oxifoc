//! Demo test suite using embedded-test
//!
//! You can run this using `cargo test` as usual.

#![no_std]
#![no_main]
// embedded_test::tests expands to `cfg(rust_analyzer)`, unknown to check-cfg.
#![allow(unexpected_cfgs)]

use nrf_mpsl as _;

#[cfg(test)]
#[embedded_test::tests(executor = embassy_executor::Executor::new())]
mod tests {
    use defmt::assert_eq;

    #[init]
    fn init() {
        embassy_nrf::init(Default::default());
        rtt_target::rtt_init_defmt!();
    }

    #[test]
    async fn hello_test() {
        defmt::info!("Running test!");

        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        assert_eq!(1 + 1, 2);
    }
}

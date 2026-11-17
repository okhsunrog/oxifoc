#![no_std]

// Uptime on every defmt line, so logs from several chips can be correlated.
defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

#[allow(clippy::needless_borrows_for_generic_args)]
pub mod ble_nus;
pub mod transport;

#[cfg(feature = "ble")]
pub mod ble_runtime;

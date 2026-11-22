#![no_std]

// Uptime on every defmt line, so logs from several chips can be correlated.
defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

#[allow(clippy::needless_borrows_for_generic_args)]
pub mod ble_nus;
pub mod diag;
pub mod transport;

#[cfg(feature = "ble")]
pub mod ble_runtime;
#[cfg(feature = "ble")]
pub mod bonds;
#[cfg(feature = "ble")]
mod csprng;
#[cfg(feature = "ble")]
use embassy_crypto_rustcrypto as _;
#[cfg(feature = "ble")]
pub mod pairing;

#[cfg(feature = "swd-probe")]
pub mod probe;

#[cfg(any(feature = "ble", feature = "ota"))]
pub mod flash;
#[cfg(feature = "ota")]
pub mod ota;
#[cfg(feature = "wifi")]
pub mod wifi;

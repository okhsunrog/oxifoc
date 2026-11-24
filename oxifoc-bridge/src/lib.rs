#![no_std]

#[cfg(not(any(feature = "board-c5-zero", feature = "board-xiao-c5")))]
compile_error!("select a board: board-c5-zero or board-xiao-c5");
#[cfg(all(feature = "board-c5-zero", feature = "board-xiao-c5"))]
compile_error!("board-c5-zero and board-xiao-c5 are mutually exclusive");

#[allow(clippy::needless_borrows_for_generic_args)]
pub mod ble_nus;
pub mod bridge_logs;
pub mod crash;
pub mod diag;
pub mod relay;
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

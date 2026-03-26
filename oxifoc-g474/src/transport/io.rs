//! I/O wrappers — re-exported from oxifoc-core (UART transport only)

#[cfg(feature = "transport-uart")]
pub use oxifoc_core::runtime::io::*;

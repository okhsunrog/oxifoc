//! Timer abstraction for async delays.
//!
//! Provides a platform-agnostic timer trait that platforms implement
//! to provide async delay functionality.
//!
//! # Example
//!
//! ```ignore
//! use oxifoc_core::timer::Timer;
//!
//! struct EmbassyTimer;
//!
//! impl Timer for EmbassyTimer {
//!     async fn after_millis(ms: u64) {
//!         embassy_time::Timer::after_millis(ms).await
//!     }
//!
//!     async fn after_micros(us: u64) {
//!         embassy_time::Timer::after_micros(us).await
//!     }
//! }
//! ```

use core::future::Future;

/// Timer trait for async delays.
///
/// Platforms implement this to provide delay functionality
/// using their native timer implementation (e.g., embassy_time::Timer).
pub trait Timer {
    /// Delay for the specified number of milliseconds.
    fn after_millis(ms: u64) -> impl Future<Output = ()>;

    /// Delay for the specified number of microseconds.
    fn after_micros(us: u64) -> impl Future<Output = ()>;
}

/// Embassy timer implementation for async delays.
#[cfg(feature = "embassy")]
pub struct EmbassyTimer;

#[cfg(feature = "embassy")]
impl Timer for EmbassyTimer {
    async fn after_millis(ms: u64) {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(ms)).await;
    }

    async fn after_micros(us: u64) {
        embassy_time::Timer::after(embassy_time::Duration::from_micros(us)).await;
    }
}

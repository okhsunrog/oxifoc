//! Phase management for FOC control
//!
//! Provides a unified interface for electrical phase angle estimation,
//! supporting multiple sources (Hall, Encoder, Observer) with runtime switching.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      PhaseManager<H, E>                      │
//! │  • Manages Hall sensor (H)                                   │
//! │  • Manages Encoder (E)                                       │
//! │  • Manages Observer (sensorless)                             │
//! │  • Handles source selection & blending                       │
//! │                                                              │
//! │  implements PhaseProvider                                    │
//! └──────────────────────────┬──────────────────────────────────┘
//!                            │
//!                            ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     PhaseProvider trait                      │
//! │  • get() → PhaseOutput (angle, velocity)                    │
//! │  • update(PhaseInput, now_ticks)                            │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! use oxifoc_core::foc::phase::{PhaseManager, PhaseSource, Observer, BackEmfObserver};
//!
//! // Create phase manager with Hall sensor
//! let mut phase = PhaseManager::with_hall(hall_sensor);
//!
//! // Add sensorless observer for high-speed operation
//! phase.set_observer(Observer::BackEmf(BackEmfObserver::new(r, l, lambda)));
//!
//! // Configure hybrid mode
//! phase.set_source(PhaseSource::HallToObserver {
//!     blend_low: 300.0,   // Start blending at 300 rad/s
//!     blend_high: 600.0,  // Full observer at 600 rad/s
//! })?;
//!
//! // Use with FocDriver
//! let driver = FocDriver::new(pwm, current_sensor, phase, vbus);
//! ```

mod manager;
mod observer;
mod provider;
mod source;

pub use manager::{HallHealth, OpenLoopOverride, PhaseFault, PhaseManager};
pub use observer::{
    BackEmfObserver, DEFAULT_CENTERING_GAIN, DEFAULT_LAMBDA_GAIN, HFI_DEFAULT_AMPLITUDE_RATIO,
    HFI_DEFAULT_FREQ_HZ, HfiObserver, Observer, ObserverInput,
};
pub use provider::{PhaseInput, PhaseOutput, PhaseProvider};
pub use source::{PhaseSource, PhaseSourceError};

//! Logging abstraction for defmt/log feature-gated output.
//!
//! Provides macros that work with either `defmt` or `log` crate,
//! or compile to no-ops when neither is enabled.
//!
//! The features are additive (no `compile_error!` on the combination):
//! cargo unifies features across a build graph, so a host crate enabling
//! `log` must not break a sibling enabling `defmt`. When both are enabled,
//! `defmt` wins (the defmt-or-log convention).

#![allow(unused)]

#[collapse_debuginfo(yes)]
macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(all(feature = "log", not(feature = "defmt")))]
            ::log::trace!($s $(, $x)*);
            #[cfg(feature = "defmt")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(any(feature = "log", feature="defmt")))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(all(feature = "log", not(feature = "defmt")))]
            ::log::debug!($s $(, $x)*);
            #[cfg(feature = "defmt")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(any(feature = "log", feature="defmt")))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(all(feature = "log", not(feature = "defmt")))]
            ::log::info!($s $(, $x)*);
            #[cfg(feature = "defmt")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(any(feature = "log", feature="defmt")))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! warn {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(all(feature = "log", not(feature = "defmt")))]
            ::log::warn!($s $(, $x)*);
            #[cfg(feature = "defmt")]
            ::defmt::warn!($s $(, $x)*);
            #[cfg(not(any(feature = "log", feature="defmt")))]
            let _ = ($( & $x ),*);
        }
    };
}

#[collapse_debuginfo(yes)]
macro_rules! error {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(all(feature = "log", not(feature = "defmt")))]
            ::log::error!($s $(, $x)*);
            #[cfg(feature = "defmt")]
            ::defmt::error!($s $(, $x)*);
            #[cfg(not(any(feature = "log", feature="defmt")))]
            let _ = ($( & $x ),*);
        }
    };
}

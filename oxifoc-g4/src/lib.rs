//! Shared hardware abstraction for STM32G4-based motor controllers
//!
//! This crate provides platform-agnostic implementations shared between
//! G431 (B-G431B-ESC1) and G474 (NUCLEO-G474RE + X-NUCLEO-IHM08M1).
//!
//! **Note:** This crate declares `embassy-stm32` without a chip feature.
//! The chip feature is provided by the consuming crate (oxifoc-g431 or
//! oxifoc-g474) via Cargo feature unification.

#![no_std]

pub mod calibration;
pub mod cordic;
pub mod current;
pub mod hall;
pub mod io;

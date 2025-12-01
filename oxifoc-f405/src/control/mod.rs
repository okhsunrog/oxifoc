//! FOC control module for STM32F405
//!
//! Provides ISR-based FOC control synchronized with PWM via TIM1-triggered
//! injected ADC conversions.

pub mod foc;

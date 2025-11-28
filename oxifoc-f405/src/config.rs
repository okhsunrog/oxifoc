//! Configuration constants for oxifoc-f405 (Simple FOCer 2 / VESC hardware)

/// Fixed hardware scaling for Cheap FOCer 2 (STM32F405).
#[derive(Clone, Copy)]
pub struct BoardScaling {
    pub shunt_ohms: f32,
    pub current_amp_gain: f32,
    pub vbus_divider_ratio: f32,
}

impl BoardScaling {
    pub const fn new() -> Self {
        // Two 1 mΩ shunts in parallel => 0.5 mΩ effective.
        // DRV8301 amp gain set to 20 V/V to match external stage.
        // VBUS divider: 39k / 2.2k => ~18.7273:1 (ADC volts * ratio = bus volts).
        Self {
            shunt_ohms: 0.0005,
            current_amp_gain: 20.0,
            vbus_divider_ratio: (39.0 + 2.2) / 2.2,
        }
    }
}

// ========== USB/Ergot Configuration ==========

/// Size of outgoing packet queue for ergot over USB
pub const OUT_QUEUE_SIZE: usize = 4096;

/// Maximum packet size for ergot framing
pub const MAX_PACKET_SIZE: usize = 512;

// ========== Timing Configuration ==========

/// Embassy timebase ticks per second
pub const TIMEBASE_TICKS_PER_SEC: u64 = embassy_time::TICK_HZ;

//! Hardware resource assignments for NUCLEO-G474RE + X-NUCLEO-IHM08M1 board
//!
//! X-NUCLEO-IHM08M1 connects via Morpho connectors.
//! PWM signals directly connected to TIM1 channels.

use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

// Resource assignments for hardware peripherals
assign_resources! {
    // Motor control resources for X-NUCLEO-IHM08M1
    // TIM1 PWM via Morpho connectors:
    motor: MotorResources {
        tim1: TIM1,
        // Phase U: TIM1_CH1/CH1N
        pa8: PA8,   // UH - Phase U high (TIM1_CH1) - CN10-21
        pa7: PA7,   // UL - Phase U low (TIM1_CH1N) - CN10-15
        // Phase V: TIM1_CH2/CH2N
        pa9: PA9,   // VH - Phase V high (TIM1_CH2) - CN10-19
        pb0: PB0,   // VL - Phase V low (TIM1_CH2N) - CN7-34
        // Phase W: TIM1_CH3/CH3N
        pa10: PA10, // WH - Phase W high (TIM1_CH3) - CN10-33
        pb1: PB1,   // WL - Phase W low (TIM1_CH3N) - CN7-30
    }

    // Hall sensor inputs for X-NUCLEO-IHM08M1
    // Active low, directly from motor Hall sensors
    hall: HallResources {
        pb6: PB6,   // H1 - CN10-17
        pb7: PB7,   // H2 - CN7-21
        pb8: PB8,   // H3 - CN10-3
    }

    // Persistent storage (always available)
    storage: StorageResources {
        flash: FLASH,
    }
}

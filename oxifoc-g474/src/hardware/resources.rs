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
        pa8: PA8,   // UH - Phase U high (TIM1_CH1) - CN10-23
        pa7: PA7,   // UL - Phase U low (TIM1_CH1N) - CN10-15
        // Phase V: TIM1_CH2/CH2N
        pa9: PA9,   // VH - Phase V high (TIM1_CH2) - CN10-21
        pb0: PB0,   // VL - Phase V low (TIM1_CH2N) - CN7-34
        // Phase W: TIM1_CH3/CH3N
        pa10: PA10, // WH - Phase W high (TIM1_CH3) - CN10-33
        pb1: PB1,   // WL - Phase W low (TIM1_CH3N) - CN10-24
    }

    // Hall sensor inputs for X-NUCLEO-IHM08M1 (J3, pull-ups via shield JP3)
    // All three are TIM2 CH1/CH2/CH3 (AF1) — see docs/nucleo-g474re-ihm08m1.md
    hall: HallResources {
        pa15: PA15, // H1 / Enc A - CN7-17  (TIM2_CH1)
        pb3: PB3,   // H2 / Enc B - CN10-31 (TIM2_CH2)
        pb10: PB10, // H3 / Enc Z - CN10-25 (TIM2_CH3)
    }

    // Persistent storage (always available)
    storage: StorageResources {
        flash: FLASH,
    }
}

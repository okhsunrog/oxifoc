//! Hardware resource assignments for NUCLEO-G474RE board
//!
//! Motor control resources are commented out until IHM08M1 shield is connected.
//! Pin assignments for IHM08M1 shield will need to be verified against the
//! shield documentation and NUCLEO morpho connector pinout.

use assign_resources::assign_resources;
use embassy_stm32::{peripherals, Peri};

// Resource assignments for hardware peripherals
assign_resources! {
    // Motor control resources (commented out until IHM08M1 shield is connected)
    // The IHM08M1 shield uses TIM1 for PWM generation:
    // motor: MotorResources {
    //     tim1: TIM1,
    //     // Phase A: TIM1_CH1/CH1N - verify pins for IHM08M1
    //     pa8: PA8,   // Phase A high (TIM1_CH1)
    //     pa7: PA7,   // Phase A low (TIM1_CH1N) - verify for IHM08M1
    //     // Phase B: TIM1_CH2/CH2N
    //     pa9: PA9,   // Phase B high (TIM1_CH2)
    //     pb0: PB0,   // Phase B low (TIM1_CH2N) - verify for IHM08M1
    //     // Phase C: TIM1_CH3/CH3N
    //     pa10: PA10, // Phase C high (TIM1_CH3)
    //     pb1: PB1,   // Phase C low (TIM1_CH3N) - verify for IHM08M1
    // }

    // Hall sensor inputs (commented out until IHM08M1 shield is connected)
    // hall: HallResources {
    //     // Hall sensor pins - verify for IHM08M1 shield
    //     pb6: PB6,   // H1 / Encoder A+
    //     pb7: PB7,   // H2 / Encoder B+
    //     pb8: PB8,   // H3 / Encoder Z+
    // }

    // Persistent storage (always available)
    storage: StorageResources {
        flash: FLASH,
    }
}

//! Resource assignment for Simple FOCer 2 hardware

use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pa9: PA9,   // Phase B high
        pa10: PA10, // Phase C high
        pb13: PB13, // Phase A low
        pb14: PB14, // Phase B low
        pb15: PB15, // Phase C low
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT
        pc0: PC0,   // Current sense A
        pc1: PC1,   // Current sense B
        pc2: PC2,   // Current sense C
        pc3: PC3,   // VBUS sense
    }
    hall: HallResources {
        pc6: PC6,   // Hall sensor 1
        pc7: PC7,   // Hall sensor 2
        pc8: PC8,   // Hall sensor 3
    }
}

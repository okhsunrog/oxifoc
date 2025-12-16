//! Hardware resource assignments for B-G431B-ESC1 board

use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

// Resource assignments for hardware peripherals
assign_resources! {
    motor: MotorResources {
        tim1: TIM1,
        pa8: PA8,   // Phase A high
        pc13: PC13, // Phase A low
        pa9: PA9,   // Phase B high
        pa12: PA12, // Phase B low
        pa10: PA10, // Phase C high
        pb15: PB15, // Phase C low
    }
    hall: HallResources {
        pb6: PB6,   // H1 / Encoder A+
        pb7: PB7,   // H2 / Encoder B+
        pb8: PB8,   // H3 / Encoder Z+
    }
    storage: StorageResources {
        flash: FLASH,
    }
}

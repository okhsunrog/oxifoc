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
    }
    drv: DrvResources {
        spi3: SPI3,
        pc9: PC9,   // SPI3_CS
        pc10: PC10, // SPI3_SCK
        pc11: PC11, // SPI3_MISO
        pc12: PC12, // SPI3_MOSI
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT pin
        exti7: EXTI7, // EXTI for nFAULT interrupt
    }
    hall: HallResources {
        pc6: PC6,   // Hall sensor 1
        pc7: PC7,   // Hall sensor 2
        pc8: PC8,   // Hall sensor 3
    }
}

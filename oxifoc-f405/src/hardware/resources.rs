//! Resource assignment — one `assign_resources!` block per supported board.
//!
//! Both boards follow the VESC reference layout, so motor/hall/uart blocks
//! are identical; only the DRV8301 bus wiring (and MK5's extra board-control
//! pins) differ. See docs/hw/cheap-focer2-notes.md and docs/hw/vesc6-mk5.md.

use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

#[cfg(feature = "board-cf2")]
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
    uart: UartResources {
        usart3: USART3,
        pb10: PB10,  // USART3 TX
        pb11: PB11,  // USART3 RX
    }
}

#[cfg(feature = "board-vesc6-mk5")]
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
    // DRV8301 on bit-bang SPI: PB3/PB4 are not a valid hardware-SPI mapping
    // (PC11/PC12 are the NRF51 UART on MK5), VESC bit-bangs it too.
    drv: DrvResources {
        pc9: PC9,   // CS
        pc10: PC10, // SCK
        pb3: PB3,   // MISO (JTDO after reset; GPIO reconfig leaves SWD alone)
        pb4: PB4,   // MOSI (NJTRST after reset)
        pb5: PB5,   // EN_GATE
        pb7: PB7,   // nFAULT pin
        exti7: EXTI7, // EXTI for nFAULT interrupt
    }
    hall: HallResources {
        pc6: PC6,   // Hall sensor 1
        pc7: PC7,   // Hall sensor 2
        pc8: PC8,   // Hall sensor 3
    }
    uart: UartResources {
        usart3: USART3,
        pb10: PB10,  // USART3 TX
        pb11: PB11,  // USART3 RX
    }
    board_ctrl: BoardCtrlResources {
        pc5: PC5,   // Shutdown latch — hold high or the button powers us off
        pd2: PD2,   // CURRENT_FILTER enable (active high)
        pc13: PC13, // PHASE_FILTER enable (active high, MK5+)
    }
}

//! Bit-bang SPI master for the DRV8301 on VESC 6 MK5 boards.
//!
//! MK5 routes the DRV8301 to SCK PC10 / MOSI PB4 / MISO PB3 — not a valid
//! hardware-SPI mapping on the F405 (VESC bit-bangs it as well). The bus is
//! only touched at boot (register config) and on nFAULT (status read), both
//! outside the FOC ISR, so a blocking software implementation costs nothing.
//!
//! Mode 1 (CPOL=0, CPHA=1), MSB first, ~500 kHz: data is driven on the
//! rising edge and sampled on the falling edge — DRV8301 timing per its
//! datasheet (max 10 MHz).

use embassy_stm32::gpio::{Input, Output};
use embedded_hal::spi::{ErrorType, SpiBus};

/// Half a bit period in CPU cycles: 168 cycles ≈ 1 µs at 168 MHz → ~500 kHz.
const HALF_PERIOD_CYCLES: u32 = 168;

pub struct SoftSpi {
    sck: Output<'static>,
    mosi: Output<'static>,
    miso: Input<'static>,
}

impl SoftSpi {
    /// `sck` and `mosi` must be initialized low (idle state for Mode 1).
    pub fn new(sck: Output<'static>, mosi: Output<'static>, miso: Input<'static>) -> Self {
        Self { sck, mosi, miso }
    }

    fn xfer_byte(&mut self, write: u8) -> u8 {
        let mut read = 0u8;
        for bit in (0..8).rev() {
            // Leading (rising) edge: both sides launch their data bit.
            self.mosi.set_level(((write >> bit) & 1 == 1).into());
            self.sck.set_high();
            cortex_m::asm::delay(HALF_PERIOD_CYCLES);
            // Trailing (falling) edge: both sides sample.
            self.sck.set_low();
            if self.miso.is_high() {
                read |= 1 << bit;
            }
            cortex_m::asm::delay(HALF_PERIOD_CYCLES);
        }
        read
    }
}

impl ErrorType for SoftSpi {
    type Error = core::convert::Infallible;
}

impl SpiBus<u8> for SoftSpi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words {
            *w = self.xfer_byte(0);
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for w in words {
            self.xfer_byte(*w);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let common = read.len().min(write.len());
        for i in 0..common {
            read[i] = self.xfer_byte(write[i]);
        }
        // SpiBus contract: the longer side keeps clocking (0x00 fill).
        for w in &mut read[common..] {
            *w = self.xfer_byte(0);
        }
        for w in &write[common..] {
            self.xfer_byte(*w);
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words {
            *w = self.xfer_byte(*w);
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

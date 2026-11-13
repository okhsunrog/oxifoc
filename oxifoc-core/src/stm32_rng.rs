//! Adapt the STM32 hardware RNG to the rand_core 0.9 interface used by Ergot.

use embassy_stm32::{mode::Async, rng::Rng};
use rand_core::RngCore;

pub struct ErgotRng(Rng<'static, Async>);

impl ErgotRng {
    pub fn new(rng: Rng<'static, Async>) -> Self {
        Self(rng)
    }
}

impl RngCore for ErgotRng {
    fn next_u32(&mut self) -> u32 {
        self.0.blocking_next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.0.blocking_next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.0.blocking_fill_bytes(dest);
    }
}

//! Hardware Hall sensor driver for STM32G431
//!
//! Uses ExtiInput for async edge detection on Hall sensor pins.

use embassy_stm32::exti::ExtiInput;

/// Hardware Hall sensor driver
///
/// Wraps the platform-agnostic `HallSensor` logic with STM32-specific
/// GPIO interrupt handling.
#[allow(dead_code)]
pub struct HallSensorDriver {
    h1: ExtiInput<'static>,
    h2: ExtiInput<'static>,
    h3: ExtiInput<'static>,
}

impl HallSensorDriver {
    /// Create a new Hall sensor driver
    ///
    /// # Arguments
    /// * `h1`, `h2`, `h3` - ExtiInput pins for the three Hall sensors
    #[allow(dead_code)]
    pub fn new(h1: ExtiInput<'static>, h2: ExtiInput<'static>, h3: ExtiInput<'static>) -> Self {
        Self { h1, h2, h3 }
    }

    /// Wait for any Hall sensor edge and return raw state + timestamp ticks.
    #[allow(dead_code)]
    pub async fn wait_for_edge(&mut self) -> (u8, u64) {
        // Wait for any Hall sensor edge using select3
        embassy_futures::select::select3(
            self.h1.wait_for_any_edge(),
            self.h2.wait_for_any_edge(),
            self.h3.wait_for_any_edge(),
        )
        .await;

        // Read all three Hall sensors to get current state
        let raw_state = self.read_hall_state();
        // Timestamping handled elsewhere; return 0 here to avoid ambiguity
        (raw_state, 0)
    }

    /// Read current Hall sensor state as 3-bit value
    ///
    /// Returns: H3<<2 | H2<<1 | H1
    #[allow(dead_code)]
    fn read_hall_state(&self) -> u8 {
        let mut state = 0u8;
        if self.h1.is_high() {
            state |= 0b001;
        }
        if self.h2.is_high() {
            state |= 0b010;
        }
        if self.h3.is_high() {
            state |= 0b100;
        }
        state
    }
}

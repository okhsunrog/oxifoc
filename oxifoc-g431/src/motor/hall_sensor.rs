//! Hardware Hall sensor driver for STM32G431
//!
//! Uses ExtiInput for async edge detection on Hall sensor pins.
//! Publishes electrical angle updates via Embassy Signal.

use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use oxifoc_core::foc::hall_sensor::{Direction, HallSensor};

/// Global signal for publishing Hall angle updates
///
/// Other tasks can wait on this signal to get the latest electrical angle
pub static HALL_ANGLE: Signal<CriticalSectionRawMutex, f32> = Signal::new();

/// Global signal for publishing Hall direction updates
pub static HALL_DIRECTION: Signal<CriticalSectionRawMutex, Direction> = Signal::new();

/// Hardware Hall sensor driver
///
/// Wraps the platform-agnostic `HallSensor` logic with STM32-specific
/// GPIO interrupt handling.
pub struct HallSensorDriver {
    h1: ExtiInput<'static>,
    h2: ExtiInput<'static>,
    h3: ExtiInput<'static>,
    sensor: HallSensor,
}

impl HallSensorDriver {
    /// Create a new Hall sensor driver
    ///
    /// # Arguments
    /// * `h1`, `h2`, `h3` - ExtiInput pins for the three Hall sensors
    /// * `pole_pairs` - Number of motor pole pairs
    pub fn new(
        h1: ExtiInput<'static>,
        h2: ExtiInput<'static>,
        h3: ExtiInput<'static>,
        pole_pairs: u8,
    ) -> Self {
        Self {
            h1,
            h2,
            h3,
            sensor: HallSensor::new(pole_pairs),
        }
    }

    /// Get current error count
    pub fn error_count(&self) -> u32 {
        self.sensor.error_count()
    }

    /// Reset error counter
    pub fn reset_errors(&mut self) {
        self.sensor.reset_errors();
    }

    /// Wait for any Hall sensor edge and update angle
    ///
    /// This is the main async loop - waits for any of the three Hall
    /// sensors to change state, then updates the angle estimate.
    pub async fn wait_and_update(&mut self) {
        // Wait for any Hall sensor edge using select3
        embassy_futures::select::select3(
            self.h1.wait_for_any_edge(),
            self.h2.wait_for_any_edge(),
            self.h3.wait_for_any_edge(),
        )
        .await;

        // Read all three Hall sensors to get current state
        let raw_state = self.read_hall_state();

        // Update angle estimate
        match self.sensor.update(raw_state) {
            Ok(angle) => {
                // Publish angle and direction to signals
                HALL_ANGLE.signal(angle);
                HALL_DIRECTION.signal(self.sensor.direction());
            }
            Err(_) => {
                // Invalid state - error already incremented in sensor.update()
                defmt::warn!(
                    "Invalid Hall state: {:03b} (errors: {})",
                    raw_state,
                    self.sensor.error_count()
                );
            }
        }
    }

    /// Read current Hall sensor state as 3-bit value
    ///
    /// Returns: H3<<2 | H2<<1 | H1
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

    /// Get current electrical angle (for synchronous access)
    pub fn angle(&self) -> f32 {
        self.sensor.angle()
    }

    /// Get current direction (for synchronous access)
    pub fn direction(&self) -> Direction {
        self.sensor.direction()
    }

    /// Get current Hall state (0-5)
    pub fn state(&self) -> u8 {
        self.sensor.state()
    }
}


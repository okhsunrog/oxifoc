//! ESP HAL adapter for dap-rs's GPIO bit-banger.

use core::cell::Cell;

use dap_rs::{
    dap::{Dap, DapLeds, DelayNs, HostStatus},
    driver::bitbang::{BitbangAdapter, DelayCycles, InputOutputPin, JtagAdapter, SwdAdapter},
    swo::NoSwo,
};
use esp_hal::{
    clock,
    delay::Delay,
    gpio::{AnyPin, Flex, InputConfig, Level, OutputConfig},
};
use riscv::register::mcycle;

use super::{framing::PACKET_SIZE, packet::DEFAULT_CLOCK_HZ};

pub struct Pins {
    clk: Flex<'static>,
    dio: Flex<'static>,
}

impl Pins {
    pub fn new(clk: AnyPin<'static>, dio: AnyPin<'static>) -> Self {
        let mut clk = Flex::new(clk);
        let mut dio = Flex::new(dio);
        for pin in [&mut clk, &mut dio] {
            pin.set_output_enable(false);
            pin.apply_input_config(&InputConfig::default());
            pin.apply_output_config(&OutputConfig::default());
            pin.set_input_enable(true);
        }
        clk.set_low();
        Self { clk, dio }
    }

    pub fn session<'p>(&'p mut self, clock: &'p Cell<u32>) -> Session<'p> {
        // SWCLK idles low until the client starts the SWD link.
        self.clk.set_low();
        let adapter = BitbangAdapter::new(
            IoPin(None), // NRST is unavailable. Use the target's SYSRESETREQ.
            IoPin(None),
            IoPin(Some(&mut self.dio)),
            IoPin(Some(&mut self.clk)),
            IoPin(None),
            SckDelay(clock),
            &mut [],
        );
        Dap::new(adapter, NoLeds, Delay::new(), NoSwo, "2.1.0", PACKET_SIZE)
    }
}

pub type Session<'p> = Dap<
    'static,
    BitbangAdapter<IoPin<'p>, SckDelay<'p>>,
    NoLeds,
    Delay,
    JtagAdapter<IoPin<'p>, SckDelay<'p>>,
    SwdAdapter<IoPin<'p>, SckDelay<'p>>,
    NoSwo,
>;

pub struct IoPin<'p>(Option<&'p mut Flex<'static>>);

impl InputOutputPin for IoPin<'_> {
    fn set_as_output(&mut self) {
        if let Some(pin) = &mut self.0 {
            pin.set_output_enable(true);
        }
    }

    fn set_high(&mut self, high: bool) {
        if let Some(pin) = &mut self.0 {
            pin.set_level(Level::from(high));
        }
    }

    fn set_as_input(&mut self) {
        if let Some(pin) = &mut self.0 {
            pin.set_output_enable(false);
        }
    }

    fn is_high(&mut self) -> bool {
        self.0.as_ref().is_none_or(|pin| pin.is_high())
    }
}

impl Drop for IoPin<'_> {
    fn drop(&mut self) {
        // A dropped TCP future releases the bus even without DAP_Disconnect.
        self.set_as_input();
    }
}

pub struct SckDelay<'p>(&'p Cell<u32>);

pub fn half_cycles(hz: u32) -> u32 {
    clock::cpu_clock().as_hz().div_ceil(2 * hz)
}

impl DelayNs for SckDelay<'_> {
    fn delay_ns(&mut self, ns: u32) {
        Delay::new().delay_ns(ns);
    }
}

impl DelayCycles for SckDelay<'_> {
    fn cpu_clock(&self) -> u32 {
        clock::cpu_clock().as_hz()
    }

    #[inline(always)]
    fn delay_cycles(&mut self, _cycles: u32) {
        // Apply a DAP_SWJ_Clock change to the next edge. The generic adapter
        // updates its own delay only when it processes a sequence command.
        let start = mcycle::read();
        let cycles = self.0.get() as usize;
        while mcycle::read().wrapping_sub(start) < cycles {
            core::hint::spin_loop();
        }
    }
}

pub fn default_delay() -> Cell<u32> {
    Cell::new(half_cycles(DEFAULT_CLOCK_HZ))
}

pub struct NoLeds;

impl DapLeds for NoLeds {
    fn react_to_host_status(&mut self, _status: HostStatus) {}
}

//! One flash driver shared by the bridge's thread-mode tasks.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embedded_storage::nor_flash::{ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{Error, FlashRegion, PartitionEntry};
use esp_hal::{
    Blocking,
    flash::{Config, Flash as Driver},
    peripherals::FLASH,
};
use static_cell::StaticCell;

#[derive(Clone, Copy)]
pub struct Flash(&'static Mutex<NoopRawMutex, RefCell<Driver<'static, Blocking>>>);

impl Flash {
    pub fn new(peripheral: FLASH<'static>) -> Self {
        static DRIVER: StaticCell<Mutex<NoopRawMutex, RefCell<Driver<'static, Blocking>>>> =
            StaticCell::new();
        let driver =
            Driver::new(peripheral, Config::default()).expect("the flash chip is identified");
        Self(DRIVER.init(Mutex::new(RefCell::new(driver))))
    }

    pub fn with<T>(&self, access: impl FnOnce(&mut Driver<'static, Blocking>) -> T) -> T {
        self.0.lock(|flash| access(&mut flash.borrow_mut()))
    }

    /// One partition as NOR flash. Offsets count from the partition's start.
    pub fn partition(self, entry: PartitionEntry) -> Partition {
        Partition { flash: self, entry }
    }
}

/// A partition of the shared flash. Each call takes the driver for itself,
/// so several tasks can hold partitions at once. Writes and erases outside
/// the partition, or into the running application, are refused.
#[derive(Clone, Copy)]
pub struct Partition {
    flash: Flash,
    entry: PartitionEntry,
}

impl Partition {
    fn region<T>(
        &self,
        access: impl FnOnce(&mut FlashRegion<'_, '_>) -> Result<T, Error>,
    ) -> Result<T, Error> {
        self.flash
            .with(|flash| access(&mut self.entry.as_flash_region(flash)?))
    }
}

impl ErrorType for Partition {
    type Error = Error;
}

impl ReadNorFlash for Partition {
    const READ_SIZE: usize = <FlashRegion<'static, 'static> as ReadNorFlash>::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.region(|region| region.read(offset, bytes))
    }

    fn capacity(&self) -> usize {
        self.entry.len() as usize
    }
}

impl NorFlash for Partition {
    const WRITE_SIZE: usize = <FlashRegion<'static, 'static> as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <FlashRegion<'static, 'static> as NorFlash>::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.region(|region| region.erase(from, to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.region(|region| region.write(offset, bytes))
    }
}

impl MultiwriteNorFlash for Partition {}

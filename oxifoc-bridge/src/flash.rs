//! One flash peripheral shared by the bridge's thread-mode tasks.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embedded_storage::nor_flash::{ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash};
use esp_hal::peripherals::FLASH;
use esp_storage::{FlashStorage, FlashStorageError};
use static_cell::StaticCell;

#[derive(Clone, Copy)]
pub struct Flash(&'static Mutex<NoopRawMutex, RefCell<FlashStorage<'static>>>);

impl Flash {
    pub fn new(peripheral: FLASH<'static>) -> Self {
        static STORAGE: StaticCell<Mutex<NoopRawMutex, RefCell<FlashStorage<'static>>>> =
            StaticCell::new();
        Self(STORAGE.init(Mutex::new(RefCell::new(FlashStorage::new(peripheral)))))
    }

    pub fn with<T>(&self, access: impl FnOnce(&mut FlashStorage<'static>) -> T) -> T {
        self.0.lock(|flash| access(&mut flash.borrow_mut()))
    }
}

impl ErrorType for Flash {
    type Error = FlashStorageError;
}

impl ReadNorFlash for Flash {
    const READ_SIZE: usize = FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.with(|flash| flash.read_nor(offset, bytes))
    }

    fn capacity(&self) -> usize {
        self.with(|flash| flash.capacity())
    }
}

impl NorFlash for Flash {
    const WRITE_SIZE: usize = FlashStorage::WRITE_SIZE;
    const ERASE_SIZE: usize = FlashStorage::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.with(|flash| flash.erase(from, to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.with(|flash| flash.write_nor(offset, bytes))
    }
}

impl MultiwriteNorFlash for Flash {}

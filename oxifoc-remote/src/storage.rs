//! Persistent remote settings in the two flash pages below the bootloader.
//!
//! A `sequential-storage` map over the MPSL flash driver: MPSL schedules every
//! erase and write in a radio timeslot, so the BLE link keeps its timing.
//! memory.x ends the firmware at [`STORAGE_START`].

use defmt::{info, warn};
use nrf_mpsl::Flash;
use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};

use super::GattHandles;

/// Two 4 KiB pages: sequential-storage needs at least two for wear levelling.
pub const STORAGE_START: u32 = 0xF2000;
const STORAGE_END: u32 = 0xF4000; // Adafruit bootloader starts here
const BUFFER_SIZE: usize = 64;

const _: () = assert!(
    STORAGE_START >= const_parse_u32(env!("FIRMWARE_END")),
    "STORAGE_START overlaps the firmware FLASH region - update memory.x"
);

const fn const_parse_u32(s: &str) -> u32 {
    let bytes = s.as_bytes();
    let mut i = 0;
    let mut result: u32 = 0;
    while i < bytes.len() {
        result = result * 10 + (bytes[i] - b'0') as u32;
        i += 1;
    }
    result
}

/// Keys of the settings map. Keep numbers stable across firmware versions.
#[repr(u8)]
#[derive(Clone, Copy)]
enum Key {
    /// Attribute handles of the bridge's OxiFOC service.
    BridgeHandles = 1,
}

impl PostcardValue<'_> for GattHandles {}

type Storage<'d> = MapStorage<
    u8,
    Flash<'d>,
    Cache<
        sequential_storage::cache::Uncached,
        sequential_storage::cache::Uncached,
        sequential_storage::cache::Uncached,
        u8,
    >,
>;

pub struct Settings<'d> {
    storage: Storage<'d>,
    buf: [u8; BUFFER_SIZE],
}

impl<'d> Settings<'d> {
    pub fn new(flash: Flash<'d>) -> Self {
        Self {
            storage: MapStorage::new(
                flash,
                MapConfig::new(STORAGE_START..STORAGE_END),
                Cache::new_uncached(),
            ),
            buf: [0; BUFFER_SIZE],
        }
    }

    /// Handles from the last successful discovery, if any.
    pub async fn bridge_handles(&mut self) -> Option<GattHandles> {
        match self
            .storage
            .fetch_item::<GattHandles>(&mut self.buf, &(Key::BridgeHandles as u8))
            .await
        {
            Ok(handles) => handles,
            Err(e) => {
                warn!("[storage] reading bridge handles failed: {:?}", e);
                None
            }
        }
    }

    pub async fn set_bridge_handles(&mut self, handles: &GattHandles) {
        match self
            .storage
            .store_item(&mut self.buf, &(Key::BridgeHandles as u8), handles)
            .await
        {
            Ok(()) => info!("[storage] bridge handles saved"),
            Err(e) => warn!("[storage] saving bridge handles failed: {:?}", e),
        }
    }
}

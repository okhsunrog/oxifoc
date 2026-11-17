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
    /// Our static random identity address and IRK, generated once.
    OwnIdentity = 2,
    /// Bond with the bridge (docs/notes/ble-pairing.md).
    BridgeBond = 3,
}

impl PostcardValue<'_> for GattHandles {}

/// The remote's BLE identity. The bridge stores it with the bond and resolves
/// our private addresses with the IRK, so both must survive reboots.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct OwnIdentity {
    pub address: [u8; 6],
    pub irk: u128,
}

impl PostcardValue<'_> for OwnIdentity {}

/// Bond with the bridge. Our own type, independent of trouble's.
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct BridgeBond {
    pub address: [u8; 6],
    pub address_random: bool,
    pub irk: Option<u128>,
    pub ltk: u128,
}

impl PostcardValue<'_> for BridgeBond {}

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

    pub async fn own_identity(&mut self) -> Option<OwnIdentity> {
        self.fetch(Key::OwnIdentity).await
    }

    pub async fn set_own_identity(&mut self, identity: &OwnIdentity) {
        self.store(Key::OwnIdentity, identity).await;
    }

    pub async fn bridge_bond(&mut self) -> Option<BridgeBond> {
        self.fetch(Key::BridgeBond).await
    }

    pub async fn set_bridge_bond(&mut self, bond: &BridgeBond) {
        self.store(Key::BridgeBond, bond).await;
    }

    pub async fn clear_bridge_bond(&mut self) {
        if let Err(e) = self
            .storage
            .remove_item(&mut self.buf, &(Key::BridgeBond as u8))
            .await
        {
            warn!("[storage] removing bridge bond failed: {:?}", e);
        }
    }

    async fn fetch<V: for<'a> PostcardValue<'a>>(&mut self, key: Key) -> Option<V> {
        match self
            .storage
            .fetch_item::<V>(&mut self.buf, &(key as u8))
            .await
        {
            Ok(value) => value,
            Err(e) => {
                warn!("[storage] reading key {} failed: {:?}", key as u8, e);
                None
            }
        }
    }

    async fn store<V: for<'a> PostcardValue<'a>>(&mut self, key: Key, value: &V) {
        match self
            .storage
            .store_item(&mut self.buf, &(key as u8), value)
            .await
        {
            Ok(()) => info!("[storage] key {} saved", key as u8),
            Err(e) => warn!("[storage] saving key {} failed: {:?}", key as u8, e),
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

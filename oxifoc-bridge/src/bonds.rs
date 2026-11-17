//! Persistent BLE bonds in the NVS partition (docs/notes/ble-pairing.md).
//!
//! A `sequential-storage` map keyed by slot id `0..MAX_BONDS`. The record is our
//! own type, so the flash format does not depend on the trouble version.

use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use esp_bootloader_esp_idf::partitions::{
    DataPartitionSubType, PARTITION_TABLE_MAX_LEN, PartitionType, read_partition_table,
};
use esp_storage::FlashStorage;
use heapless::{String, Vec};
use oxifoc_core::types::pairing::{BOND_NAME_LEN, BondEntry, BondRole, MAX_BONDS, PairingError};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};
use trouble_host::prelude::{
    AddrKind, Address, BdAddr, BondInformation, Identity, IdentityResolvingKey, LongTermKey,
    SecurityLevel,
};

type Flash = BlockingAsync<FlashStorage<'static>>;
type Storage = MapStorage<u8, Flash, Cache<Uncached, Uncached, Uncached, u8>>;

/// Flash record of one bond. Keys stay on the bridge; only [`BondEntry`] is
/// reported to clients.
#[derive(Clone, Serialize, Deserialize)]
pub struct StoredBond {
    pub role: BondRole,
    pub name: String<BOND_NAME_LEN>,
    pub added_by: Option<u8>,
    pub address: [u8; 6],
    pub address_random: bool,
    pub irk: Option<u128>,
    pub ltk: u128,
}

impl PostcardValue<'_> for StoredBond {}

impl StoredBond {
    pub fn from_trouble(
        bond: &BondInformation,
        role: BondRole,
        name: String<BOND_NAME_LEN>,
        added_by: Option<u8>,
    ) -> Self {
        let mut address = [0; 6];
        address.copy_from_slice(bond.identity.addr.addr.raw());
        Self {
            role,
            name,
            added_by,
            address,
            address_random: bond.identity.addr.kind == AddrKind::RANDOM,
            irk: bond.identity.irk.map(|irk| irk.0.get()),
            ltk: bond.ltk.0,
        }
    }

    pub fn identity(&self) -> Identity {
        Identity {
            addr: Address {
                kind: if self.address_random {
                    AddrKind::RANDOM
                } else {
                    AddrKind::PUBLIC
                },
                addr: BdAddr::new(self.address),
            },
            irk: self.irk.and_then(IdentityResolvingKey::new),
        }
    }

    pub fn to_trouble(&self) -> BondInformation {
        BondInformation::new(
            self.identity(),
            LongTermKey(self.ltk),
            SecurityLevel::EncryptedAuthenticated,
            true,
        )
    }

    fn entry(&self, id: u8) -> BondEntry {
        BondEntry {
            id,
            role: self.role,
            name: self.name.clone(),
            address: self.address,
            address_random: self.address_random,
            added_by: self.added_by,
        }
    }
}

pub struct BondStore {
    storage: Storage,
    buf: [u8; 128],
    slots: [Option<StoredBond>; MAX_BONDS],
}

impl BondStore {
    /// Open the map in the NVS partition and load every bond.
    pub async fn open(mut flash: FlashStorage<'static>) -> Self {
        let mut table_buf = [0u8; PARTITION_TABLE_MAX_LEN];
        let range = {
            let table =
                read_partition_table(&mut flash, &mut table_buf).expect("partition table readable");
            let nvs = table
                .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
                .expect("partition table parses")
                .expect("NVS partition present");
            nvs.offset()..nvs.offset() + nvs.len()
        };
        info!("[bonds] NVS partition {:#x}..{:#x}", range.start, range.end);
        let mut store = Self {
            storage: MapStorage::new(
                BlockingAsync::new(flash),
                MapConfig::new(range),
                Cache::new_uncached(),
            ),
            buf: [0; 128],
            slots: [const { None }; MAX_BONDS],
        };
        store.load().await;
        store
    }

    async fn load(&mut self) {
        for id in 0..MAX_BONDS as u8 {
            match self
                .storage
                .fetch_item::<StoredBond>(&mut self.buf, &id)
                .await
            {
                Ok(bond) => self.slots[id as usize] = bond,
                Err(e) => {
                    // Foreign or corrupt data: the partition is ours alone, so
                    // start over rather than run without bond storage.
                    warn!("[bonds] map unreadable ({:?}), erasing", e);
                    if self.storage.erase_all().await.is_err() {
                        warn!("[bonds] erase failed");
                    }
                    self.slots = [const { None }; MAX_BONDS];
                    return;
                }
            }
        }
        info!("[bonds] {} stored", self.slots.iter().flatten().count());
    }

    pub fn iter(&self) -> impl Iterator<Item = (u8, &StoredBond)> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(id, slot)| slot.as_ref().map(|bond| (id as u8, bond)))
    }

    pub fn entries(&self) -> Vec<BondEntry, MAX_BONDS> {
        self.iter().map(|(id, bond)| bond.entry(id)).collect()
    }

    pub fn entry(&self, id: u8) -> Option<BondEntry> {
        self.get(id).map(|bond| bond.entry(id))
    }

    pub fn get(&self, id: u8) -> Option<&StoredBond> {
        self.slots.get(id as usize).and_then(Option::as_ref)
    }

    pub fn is_full(&self) -> bool {
        self.slots.iter().all(Option::is_some)
    }

    /// Slot of the bond with this identity.
    pub fn find(&self, identity: &Identity) -> Option<u8> {
        self.iter()
            .find(|(_, bond)| bond.identity().match_identity(identity))
            .map(|(id, _)| id)
    }

    /// Store a new bond, replacing an older one with the same identity.
    pub async fn insert(&mut self, bond: StoredBond) -> Result<u8, PairingError> {
        let id = match self.find(&bond.identity()) {
            Some(id) => id,
            None => self
                .slots
                .iter()
                .position(Option::is_none)
                .ok_or(PairingError::TableFull)? as u8,
        };
        self.storage
            .store_item(&mut self.buf, &id, &bond)
            .await
            .map_err(|_| PairingError::Storage)?;
        self.slots[id as usize] = Some(bond);
        Ok(id)
    }

    pub async fn remove(&mut self, id: u8) -> Result<StoredBond, PairingError> {
        let bond = self.get(id).cloned().ok_or(PairingError::NoSuchBond)?;
        self.storage
            .remove_item(&mut self.buf, &id)
            .await
            .map_err(|_| PairingError::Storage)?;
        self.slots[id as usize] = None;
        Ok(bond)
    }

    pub async fn rename(
        &mut self,
        id: u8,
        name: String<BOND_NAME_LEN>,
    ) -> Result<(), PairingError> {
        let mut bond = self.get(id).cloned().ok_or(PairingError::NoSuchBond)?;
        bond.name = name;
        self.storage
            .store_item(&mut self.buf, &id, &bond)
            .await
            .map_err(|_| PairingError::Storage)?;
        self.slots[id as usize] = Some(bond);
        Ok(())
    }
}

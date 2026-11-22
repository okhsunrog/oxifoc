//! Boot selection records live in separate erase sectors. Preserve the record
//! for the running app when publishing the next slot to the ESP-IDF bootloader.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_bootloader_esp_idf::partitions::{
    AppPartitionSubType, DataPartitionSubType, PARTITION_TABLE_MAX_LEN, PartitionEntry,
    PartitionType, read_partition_table,
};
use esp_hal::rom::crc;
use oxifoc_core::types::bridge_ota::OtaError;

use crate::flash::Flash;

pub const SECTOR: u32 = 4096;
const RECORD_SIZE: usize = 32;
const OTA_UNDEFINED: u32 = u32::MAX;
const OTA_VALID: u32 = 2;

#[derive(Clone, Copy)]
pub struct Slots {
    apps: [PartitionEntry; 2],
    metadata: u32,
    pub booted: u8,
}

impl Slots {
    pub fn read(flash: Flash) -> Result<Self, OtaError> {
        if esp_storage::flash_encryption() {
            return Err(OtaError::PartitionLayout);
        }
        let mut buffer = [0; PARTITION_TABLE_MAX_LEN];
        flash.with(|flash| {
            let table =
                read_partition_table(flash, &mut buffer).map_err(|_| OtaError::PartitionLayout)?;
            let find = |kind| {
                table
                    .find_partition(kind)
                    .ok()
                    .flatten()
                    .ok_or(OtaError::PartitionLayout)
            };
            let apps = [
                find(PartitionType::App(AppPartitionSubType::Ota0))?,
                find(PartitionType::App(AppPartitionSubType::Ota1))?,
            ];
            let data = find(PartitionType::Data(DataPartitionSubType::Ota))?;
            let running = table
                .booted_partition()
                .ok()
                .flatten()
                .ok_or(OtaError::PartitionLayout)?;
            let booted = apps
                .iter()
                .position(|part| part.offset() == running.offset())
                .ok_or(OtaError::PartitionLayout)? as u8;

            // Accept aligned, writable, non-overlapping slots and metadata.
            // A damaged table must never send a write into the running app or NVS.
            for part in [apps[0], apps[1], data] {
                let start = part.offset();
                let end = start
                    .checked_add(part.len())
                    .ok_or(OtaError::PartitionLayout)?;
                if start < 0x10000
                    || end as usize > flash.capacity()
                    || part.is_empty()
                    || !start.is_multiple_of(SECTOR)
                    || !end.is_multiple_of(SECTOR)
                    || part.is_read_only()
                    || table.iter().any(|other| {
                        (other.offset(), other.partition_type()) != (start, part.partition_type())
                            && other.offset() < end
                            && other.offset().saturating_add(other.len()) > start
                    })
                {
                    return Err(OtaError::PartitionLayout);
                }
            }
            if data.len() != 2 * SECTOR {
                return Err(OtaError::PartitionLayout);
            }
            Ok(Self {
                apps,
                metadata: data.offset(),
                booted,
            })
        })
    }

    pub fn inactive(self) -> PartitionEntry {
        self.apps[usize::from(self.booted ^ 1)]
    }

    fn newest(self, flash: &mut Flash) -> Result<Option<(u32, u32)>, OtaError> {
        let mut newest = None;
        for at in [self.metadata, self.metadata + SECTOR] {
            let mut record = [0; RECORD_SIZE];
            flash.read(at, &mut record).map_err(|_| OtaError::Flash)?;
            if let Some(seq) = sequence(record)
                && newest.is_none_or(|(_, previous)| seq > previous)
            {
                newest = Some((at, seq));
            }
        }
        Ok(newest)
    }

    pub fn check_running_selection(self, flash: &mut Flash) -> Result<(), OtaError> {
        let expected = self.newest(flash)?.map_or(0, |(_, seq)| app_index(seq));
        if expected == self.booted {
            Ok(())
        } else {
            // The bootloader fell back to the other app. Wired recovery
            // restores the boot selection before that app can be overwritten.
            Err(OtaError::PartitionLayout)
        }
    }

    pub fn select_inactive(self, flash: &mut Flash) -> Result<u8, OtaError> {
        self.check_running_selection(flash)?;
        let (at, last) = self.newest(flash)?.unwrap_or((self.metadata + SECTOR, 0));
        let target = self.booted ^ 1;
        let mut seq = last.checked_add(1).ok_or(OtaError::Flash)?;
        if app_index(seq) != target {
            seq = seq.checked_add(1).ok_or(OtaError::Flash)?;
        }
        if seq == u32::MAX {
            return Err(OtaError::Flash);
        }
        let other = if at == self.metadata {
            at + SECTOR
        } else {
            self.metadata
        };
        let seq_bytes = seq.to_le_bytes();
        let mut record = [0xff; RECORD_SIZE];
        record[..4].copy_from_slice(&seq_bytes);
        record[28..].copy_from_slice(&crc::crc32_le(u32::MAX, &seq_bytes).to_le_bytes());
        flash
            .erase(other, other + SECTOR)
            .map_err(|_| OtaError::Flash)?;
        flash.write(other, &record).map_err(|_| OtaError::Flash)?;
        let mut readback = [0; RECORD_SIZE];
        flash
            .read(other, &mut readback)
            .map_err(|_| OtaError::Flash)?;
        if readback != record {
            return Err(OtaError::Flash);
        }
        Ok(target)
    }
}

fn app_index(sequence: u32) -> u8 {
    ((sequence - 1) % 2) as u8
}

fn sequence(record: [u8; RECORD_SIZE]) -> Option<u32> {
    let seq_bytes = record[..4].try_into().ok()?;
    let seq = u32::from_le_bytes(seq_bytes);
    let state = u32::from_le_bytes(record[24..28].try_into().ok()?);
    let crc_stored = u32::from_le_bytes(record[28..].try_into().ok()?);
    (seq != 0
        && seq != u32::MAX
        && matches!(state, OTA_UNDEFINED | OTA_VALID)
        && crc_stored == crc::crc32_le(u32::MAX, &seq_bytes))
    .then_some(seq)
}

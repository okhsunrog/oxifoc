//! Persistent configuration storage for NUCLEO-G474RE.
//!
//! Uses the last 4KB of bank 2 (2 pages x 2KB).
//! True async flash — non-blocking operations from bank 2
//! while firmware runs from bank 1.

use defmt::info;
use embassy_executor::task;
use embassy_stm32::flash::{Async, Flash as StmFlash, WRITE_SIZE};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};
use static_cell::ConstStaticCell;

// Re-export config types, channels, and signals from core
pub use oxifoc_core::storage::*;

// ============================================================================
// Flash Layout
// ============================================================================

/// Storage region: last 4KB of 512KB flash (bank 2, 2 x 2KB pages)
const STORAGE_START: u32 = 0x7F000; // 508KB offset
const STORAGE_SIZE: u32 = 4 * 1024;
const BUFFER_SIZE: usize = 128;

// Compile-time check: storage must not overlap firmware.
// FIRMWARE_END_OFFSET is parsed from memory.x by build.rs (ORIGIN - 0x08000000 + LENGTH).
const _: () = assert!(
    STORAGE_START >= const_parse_u32(env!("FIRMWARE_END_OFFSET")),
    "STORAGE_START overlaps firmware FLASH region - update memory.x or STORAGE_START"
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

// ============================================================================
// Storage Types
// ============================================================================

static BUFFER: ConstStaticCell<[u8; BUFFER_SIZE]> = ConstStaticCell::new([0u8; BUFFER_SIZE]);

pub type AsyncFlash = StmFlash<'static, Async>;
type Storage = MapStorage<ConfigKey, AsyncFlash, NoCache>;

// ============================================================================
// Storage Worker Task
// ============================================================================

/// Storage worker: loads all configs at boot, then handles write operations.
///
/// Signals `CONFIG_LOADED` after boot-time reads, then loops on `FLASH_CHANNEL`.
#[task]
pub async fn storage_worker(flash: AsyncFlash) {
    let buf = BUFFER.take();
    let config: MapConfig<AsyncFlash> =
        MapConfig::new(STORAGE_START..(STORAGE_START + STORAGE_SIZE));
    let mut storage: Storage = MapStorage::new(flash, config, NoCache::new());

    info!(
        "Storage worker started, range: {:#x}..{:#x}, write_size: {}",
        STORAGE_START,
        STORAGE_START + STORAGE_SIZE,
        WRITE_SIZE
    );

    run_storage_worker(&mut storage, buf).await
}

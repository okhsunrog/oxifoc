//! Persistent configuration storage for Simple FOCer 2 (STM32F405).
//!
//! Uses the last 256KB of flash (sectors 10-11, each 128KB).
//! Flash is blocking (single-bank), wrapped with BlockingAsync.
//!
//! memory.x must limit firmware FLASH to 768K to avoid overlap.

use defmt::info;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::task;
use embassy_stm32::flash::{Blocking, Flash as StmFlash, WRITE_SIZE};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};
use static_cell::ConstStaticCell;

// Re-export config types, channels, and signals from core
pub use oxifoc_core::storage::*;

// ============================================================================
// Flash Layout
// ============================================================================

/// Storage region: last 256KB of 1MB flash (sectors 10-11, each 128KB)
const STORAGE_START: u32 = 0xC0000; // 768KB offset
const STORAGE_SIZE: u32 = 256 * 1024;
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

pub type AsyncFlash = BlockingAsync<StmFlash<'static, Blocking>>;
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

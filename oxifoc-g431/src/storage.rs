//! Persistent configuration storage for B-G431B-ESC1.
//!
//! Uses the last 4KB of internal flash (2 pages x 2KB).
//! Flash is blocking (single-bank), wrapped with BlockingAsync.

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

/// Storage region: last 4KB of 128KB flash (2 x 2KB pages)
const STORAGE_START: u32 = 0x1F000; // 124KB offset
const STORAGE_SIZE: u32 = 4 * 1024;
const BUFFER_SIZE: usize = 128;

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

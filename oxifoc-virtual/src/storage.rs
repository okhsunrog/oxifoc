//! Mock flash storage backing the shared core storage worker.

use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
use tracing::info;

use oxifoc_core::storage::*;

/// Mock flash: 4 pages, 4-byte words, 256 words per page = 4KB total
type MockFlash = MockFlashBase<4, 4, 256>;
type Storage = MapStorage<ConfigKey, MockFlash, UncachedStorage<ConfigKey>>;

pub async fn storage_worker(board_trip_a: f32) {
    let flash = MockFlash::new(WriteCountCheck::TwiceWithZero, None, true);
    let config: MapConfig<MockFlash> = MapConfig::new(MockFlash::FULL_FLASH_RANGE);
    let mut storage: Storage = MapStorage::new(flash, config, Cache::new_uncached());
    let mut buf = [0u8; 512];

    info!("Mock storage worker started");
    // Ride the shared core worker instead of a local copy: the hand-written
    // load_all here had drifted (it silently skipped the Derating group, so
    // a persisted derating config was lost on every restart).
    Box::pin(run_storage_worker(&mut storage, &mut buf, board_trip_a)).await;
}

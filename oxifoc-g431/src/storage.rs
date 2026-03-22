//! Persistent configuration storage for B-G431B-ESC1.
//!
//! Uses the last 4KB of internal flash (2 pages x 2KB).
//! Flash is blocking (single-bank), wrapped with BlockingAsync.

#![allow(dead_code)]

use defmt::{debug, error, info};
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

    // Boot-time: load all stored configs
    let cfg = load_all(&mut storage, buf).await;
    CONFIG_LOADED.signal(cfg);

    // Runtime: handle write operations
    loop {
        let op = FLASH_CHANNEL.receive().await;
        debug!("Flash operation received");

        let success = match op {
            FlashOperation::Save(key, payload) => {
                let result = match payload {
                    ConfigPayload::MotorParams(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallCalibration(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::DcOffsets(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::CurrentLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::VoltageLimits(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PwmConfig(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::PiGains(v) => storage.store_item(buf, &key, &v).await,
                    ConfigPayload::HallTuning(v) => storage.store_item(buf, &key, &v).await,
                };
                match result {
                    Ok(_) => true,
                    Err(_) => {
                        error!("Failed to save config");
                        false
                    }
                }
            }
            FlashOperation::EraseAll => match storage.erase_all().await {
                Ok(_) => true,
                Err(_) => {
                    error!("Failed to erase storage");
                    false
                }
            },
        };

        if success {
            debug!("Flash operation succeeded");
        }
        FLASH_DONE.signal(success);
    }
}

/// Load all stored configs. Missing keys return None.
async fn load_all(storage: &mut Storage, buf: &mut [u8]) -> RuntimeConfig {
    let mut cfg = RuntimeConfig::default();

    macro_rules! load {
        ($field:ident, $key:ident) => {
            cfg.$field = storage
                .fetch_item(buf, &ConfigKey::$key)
                .await
                .ok()
                .flatten();
        };
    }

    load!(motor_params, MotorParams);
    load!(hall_calibration, HallCalibration);
    load!(dc_offsets, DcOffsets);
    load!(current_limits, CurrentLimits);
    load!(voltage_limits, VoltageLimits);
    load!(pwm_config, PwmConfig);
    load!(pi_gains, PiGains);
    load!(hall_tuning, HallTuning);

    info!("Loaded config from flash");
    cfg
}

// ============================================================================
// Public Save Helpers
// ============================================================================

pub async fn save_motor_params(config: MotorParamsConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::Save(
            ConfigKey::MotorParams,
            ConfigPayload::MotorParams(config),
        ))
        .await;
}

pub async fn save_hall_calibration(config: HallCalibrationConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::Save(
            ConfigKey::HallCalibration,
            ConfigPayload::HallCalibration(config),
        ))
        .await;
}

pub async fn save_dc_offsets(config: DcOffsetsConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::Save(
            ConfigKey::DcOffsets,
            ConfigPayload::DcOffsets(config),
        ))
        .await;
}

pub async fn erase_all_config() {
    FLASH_CHANNEL.send(FlashOperation::EraseAll).await;
}

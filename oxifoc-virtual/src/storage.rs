//! Mock flash storage worker using sequential-storage's MockFlashBase.

use sequential_storage::cache::Cache;
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
use tracing::{debug, error, info};

use oxifoc_core::storage::*;

/// Mock flash: 4 pages, 4-byte words, 256 words per page = 4KB total
type MockFlash = MockFlashBase<4, 4, 256>;
type Storage = MapStorage<ConfigKey, MockFlash, UncachedStorage<ConfigKey>>;

pub async fn storage_worker() {
    let flash = MockFlash::new(WriteCountCheck::TwiceWithZero, None, true);
    let config: MapConfig<MockFlash> = MapConfig::new(MockFlash::FULL_FLASH_RANGE);
    let mut storage: Storage = MapStorage::new(flash, config, Cache::new_uncached());
    let mut buf = [0u8; 128];

    // Boot-time: load all stored configs (empty on first run)
    let cfg = load_all(&mut storage, &mut buf).await;
    CONFIG_LOADED.signal(cfg);

    info!("Mock storage worker started");

    // Runtime: handle write operations
    loop {
        let op = FLASH_CHANNEL.receive().await;
        debug!("Flash operation received");

        let success = match op {
            FlashOperation::Save(key, payload) => {
                let result = match payload {
                    ConfigPayload::MotorParams(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::HallCalibration(v) => {
                        storage.store_item(&mut buf, &key, &v).await
                    }
                    ConfigPayload::DcOffsets(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::CurrentLimits(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::VoltageLimits(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::PwmConfig(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::PiGains(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::HallTuning(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::Failsafe(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::Velocity(v) => storage.store_item(&mut buf, &key, &v).await,
                    ConfigPayload::Derating(v) => storage.store_item(&mut buf, &key, &v).await,
                };
                match result {
                    Ok(_) => true,
                    Err(e) => {
                        error!("Failed to save config: {:?}", e);
                        false
                    }
                }
            }
            FlashOperation::EraseAll => match storage.erase_all().await {
                Ok(_) => true,
                Err(e) => {
                    error!("Failed to erase storage: {:?}", e);
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

async fn load_all(storage: &mut Storage, buf: &mut [u8]) -> RuntimeConfig {
    let mut cfg = RuntimeConfig::default();

    macro_rules! load {
        ($field:ident, $key:ident, $ty:ty) => {
            cfg.$field = storage
                .fetch_item::<$ty>(buf, &ConfigKey::$key)
                .await
                .ok()
                .flatten();
        };
    }

    load!(motor_params, MotorParams, MotorParamsConfig);
    load!(hall_calibration, HallCalibration, HallCalibrationConfig);
    load!(dc_offsets, DcOffsets, DcOffsetsConfig);
    load!(current_limits, CurrentLimits, CurrentLimitsConfig);
    load!(voltage_limits, VoltageLimits, VoltageLimitsConfig);
    load!(pwm_config, PwmConfig, PwmConfigStored);
    load!(pi_gains, PiGains, PiGainsConfig);
    load!(hall_tuning, HallTuning, HallTuningConfig);
    load!(failsafe, Failsafe, FailsafeConfigStored);
    load!(velocity, Velocity, VelocityConfigStored);

    cfg
}

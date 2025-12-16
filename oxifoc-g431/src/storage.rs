//! Persistent configuration storage using sequential_storage::map
//!
//! Stores motor parameters and calibration data in flash memory.
//! Uses the last 4KB of internal flash (2 pages × 2KB).

// Allow unused code - this is a public API that will be integrated with calibration routines
#![allow(dead_code)]

use defmt::{debug, error, info};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::task;
use embassy_stm32::flash::{Blocking, Flash as StmFlash, WRITE_SIZE};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage, SerializationError, Value};
use serde::{Deserialize, Serialize};
use static_cell::ConstStaticCell;

// ============================================================================
// Flash Storage Configuration
// ============================================================================

/// Start address of storage region (offset from flash base)
/// Flash base is 0x08000000, storage starts at 0x0801F000
const STORAGE_START: u32 = 0x1F000; // 124KB offset

/// Size of storage region in bytes (4KB = 2 pages × 2KB)
const STORAGE_SIZE: u32 = 4 * 1024;

/// Buffer size for flash operations (must fit largest item + overhead)
const BUFFER_SIZE: usize = 128;

// ============================================================================
// Storage Keys
// ============================================================================

/// Storage keys for different configuration items
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageKey {
    /// Motor electrical parameters
    MotorParams = 1,
    /// Hall sensor calibration
    HallCalibration = 2,
    /// Current sensor DC offsets
    DcOffsets = 3,
}

impl sequential_storage::map::Key for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = *self as u8;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        let key = match buffer[0] {
            1 => StorageKey::MotorParams,
            2 => StorageKey::HallCalibration,
            3 => StorageKey::DcOffsets,
            _ => return Err(SerializationError::InvalidFormat),
        };
        Ok((key, 1))
    }
}

// ============================================================================
// Persistable Configuration Types
// ============================================================================

/// Motor electrical parameters for persistent storage.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct MotorParamsConfig {
    /// Phase-to-neutral resistance in Ohms
    pub resistance_ohm: f32,
    /// d-axis inductance in Henries
    pub inductance_d_h: f32,
    /// q-axis inductance in Henries
    pub inductance_q_h: f32,
    /// Flux linkage (lambda) in Weber
    pub flux_linkage_wb: f32,
    /// Number of pole pairs
    pub pole_pairs: u8,
}

impl MotorParamsConfig {
    pub fn is_valid(&self) -> bool {
        self.resistance_ohm > 0.0 && self.pole_pairs > 0
    }
}

impl Value<'_> for MotorParamsConfig {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer)
            .map(|b| b.len())
            .map_err(|_| SerializationError::BufferTooSmall)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        postcard::take_from_bytes(buffer)
            .map(|(val, remaining)| (val, buffer.len() - remaining.len()))
            .map_err(|_| SerializationError::InvalidFormat)
    }
}

/// Hall sensor calibration data for persistent storage.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct HallCalibrationConfig {
    /// Electrical angle (radians) for each raw Hall state (0-7)
    pub angles: [f32; 8],
    /// Validity flags for each Hall state
    pub valid: [bool; 8],
}

impl Default for HallCalibrationConfig {
    fn default() -> Self {
        use core::f32::consts::TAU;
        Self {
            angles: [
                0.0,               // state 0 - invalid
                TAU / 12.0,        // state 1 - 30°
                5.0 * TAU / 12.0,  // state 2 - 150°
                TAU / 4.0,         // state 3 - 90°
                3.0 * TAU / 4.0,   // state 4 - 270°
                11.0 * TAU / 12.0, // state 5 - 330°
                7.0 * TAU / 12.0,  // state 6 - 210°
                0.0,               // state 7 - invalid
            ],
            valid: [false, true, true, true, true, true, true, false],
        }
    }
}

impl HallCalibrationConfig {
    pub fn is_calibrated(&self) -> bool {
        self.valid.iter().filter(|&&v| v).count() == 6
    }
}

impl Value<'_> for HallCalibrationConfig {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer)
            .map(|b| b.len())
            .map_err(|_| SerializationError::BufferTooSmall)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        postcard::take_from_bytes(buffer)
            .map(|(val, remaining)| (val, buffer.len() - remaining.len()))
            .map_err(|_| SerializationError::InvalidFormat)
    }
}

/// Current sensor DC offset calibration data.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, defmt::Format)]
pub struct DcOffsetsConfig {
    /// Phase A offset (ADC counts)
    pub phase_a: f32,
    /// Phase B offset (ADC counts)
    pub phase_b: f32,
    /// Phase C offset (ADC counts)
    pub phase_c: f32,
}

impl Value<'_> for DcOffsetsConfig {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer)
            .map(|b| b.len())
            .map_err(|_| SerializationError::BufferTooSmall)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        postcard::take_from_bytes(buffer)
            .map(|(val, remaining)| (val, buffer.len() - remaining.len()))
            .map_err(|_| SerializationError::InvalidFormat)
    }
}

// ============================================================================
// Flash Operation Messages
// ============================================================================

/// Messages for flash operations
#[derive(Clone, Debug)]
pub enum FlashOperation {
    /// Save motor parameters
    SaveMotorParams(MotorParamsConfig),
    /// Save Hall calibration
    SaveHallCalibration(HallCalibrationConfig),
    /// Save DC offsets
    SaveDcOffsets(DcOffsetsConfig),
    /// Erase all storage
    EraseAll,
}

/// Channel for sending flash operations to the storage task
pub static FLASH_CHANNEL: Channel<CriticalSectionRawMutex, FlashOperation, 4> = Channel::new();

/// Signal indicating flash operation completion (true = success)
pub static FLASH_DONE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// ============================================================================
// Static Buffers
// ============================================================================

static BUFFER: ConstStaticCell<[u8; BUFFER_SIZE]> = ConstStaticCell::new([0u8; BUFFER_SIZE]);

// ============================================================================
// Storage Task
// ============================================================================

/// Flash type alias for convenience
pub type AsyncFlash = BlockingAsync<StmFlash<'static, Blocking>>;

/// Map storage type alias
pub type Storage = MapStorage<StorageKey, AsyncFlash, NoCache>;

/// Storage worker task that handles flash I/O operations.
#[task]
pub async fn storage_worker(flash: AsyncFlash) {
    let buf = BUFFER.take();

    // Create storage configuration
    let config: MapConfig<AsyncFlash> =
        MapConfig::new(STORAGE_START..(STORAGE_START + STORAGE_SIZE));

    // Create map storage instance
    let mut storage: Storage = MapStorage::new(flash, config, NoCache::new());

    info!(
        "Storage worker started, range: {:#x}..{:#x}, write_size: {}",
        STORAGE_START,
        STORAGE_START + STORAGE_SIZE,
        WRITE_SIZE
    );

    loop {
        let op = FLASH_CHANNEL.receive().await;
        debug!("Flash operation received");

        let success = match op {
            FlashOperation::SaveMotorParams(config) => {
                match storage
                    .store_item(buf, &StorageKey::MotorParams, &config)
                    .await
                {
                    Ok(_) => true,
                    Err(_) => {
                        error!("Failed to save motor params");
                        false
                    }
                }
            }
            FlashOperation::SaveHallCalibration(config) => {
                match storage
                    .store_item(buf, &StorageKey::HallCalibration, &config)
                    .await
                {
                    Ok(_) => true,
                    Err(_) => {
                        error!("Failed to save hall calibration");
                        false
                    }
                }
            }
            FlashOperation::SaveDcOffsets(config) => {
                match storage
                    .store_item(buf, &StorageKey::DcOffsets, &config)
                    .await
                {
                    Ok(_) => true,
                    Err(_) => {
                        error!("Failed to save DC offsets");
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

// ============================================================================
// Public API for reading/writing config
// ============================================================================

/// Save motor parameters (sends to storage task)
pub async fn save_motor_params(config: MotorParamsConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::SaveMotorParams(config))
        .await;
}

/// Save Hall calibration (sends to storage task)
pub async fn save_hall_calibration(config: HallCalibrationConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::SaveHallCalibration(config))
        .await;
}

/// Save DC offsets (sends to storage task)
pub async fn save_dc_offsets(config: DcOffsetsConfig) {
    FLASH_CHANNEL
        .send(FlashOperation::SaveDcOffsets(config))
        .await;
}

/// Erase all stored configuration
pub async fn erase_all_config() {
    FLASH_CHANNEL.send(FlashOperation::EraseAll).await;
}

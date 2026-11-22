//! Resumable app-slot updates shared by the bridge transports.

use core::{cell::RefCell, pin::pin};

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use ergot::socket::HeaderMessage;
use esp_hal::{rng::Rng, system};
use oxifoc_core::{
    icd::BridgeOtaEndpoint,
    types::bridge_ota::{CHUNK_SIZE, Digest, OtaError, OtaRequest, OtaResponse, is_bridge_image},
};
use sha2::{Digest as _, Sha256};
use static_cell::StaticCell;

use crate::{flash::Flash, transport::Stack};

mod slots;

use slots::{SECTOR, Slots};

const IDLE_LIMIT: Duration = Duration::from_secs(60);

pub type Service = RefCell<Updater>;

pub fn start(spawner: Spawner, stack: &'static Stack, flash: Flash) {
    static STATE: StaticCell<Service> = StaticCell::new();
    let updater = Updater {
        slots: Slots::read(flash),
        flash,
        boot: None,
        upload: None,
        rebooting: None,
    };
    let service = STATE.init(RefCell::new(updater));
    spawner.spawn(run(stack, service).expect("bridge OTA server is spawned once"));
}

pub struct Updater {
    flash: Flash,
    slots: Result<Slots, OtaError>,
    /// Chosen on the first request, once a radio has seeded the generator.
    boot: Option<u32>,
    upload: Option<Upload>,
    rebooting: Option<u8>,
}

struct Upload {
    length: u32,
    expected: Digest,
    hash: Sha256,
    offset: u32,
    erased: u32,
    last_activity: Instant,
}

impl Updater {
    fn handle(&mut self, request: &OtaRequest) -> Result<OtaResponse, OtaError> {
        let slots = self.slots?;
        let inactive = slots.inactive();
        let now = Instant::now();
        if self.rebooting.is_none()
            && self
                .upload
                .as_ref()
                .is_some_and(|upload| now - upload.last_activity >= IDLE_LIMIT)
        {
            self.upload = None;
        }
        // Status stays available while the activation reply is on its way.
        if let Some(slot) = self.rebooting
            && !matches!(request, OtaRequest::Info)
        {
            return match request {
                OtaRequest::Finish => Ok(OtaResponse::Rebooting { slot }),
                _ => Err(OtaError::Busy),
            };
        }

        match request {
            OtaRequest::Info => Ok(OtaResponse::Info {
                // The requesting BLE/Wi-Fi link has already enabled its radio.
                boot: *self.boot.get_or_insert_with(|| Rng::new().random()),
                booted_slot: slots.booted,
                capacity: inactive.len(),
            }),
            OtaRequest::Begin { length, digest } => {
                if *length < 256 || *length > inactive.len() || !length.is_multiple_of(4) {
                    return Err(OtaError::InvalidLength);
                }
                if let Some(current) = &mut self.upload {
                    if (current.length, current.expected) != (*length, *digest) {
                        return Err(OtaError::Busy);
                    }
                    current.last_activity = now;
                    return Ok(OtaResponse::Progress {
                        offset: current.offset,
                    });
                }
                slots.check_running_selection(&mut self.flash)?;
                self.upload = Some(Upload {
                    length: *length,
                    expected: *digest,
                    hash: Sha256::new(),
                    offset: 0,
                    erased: 0,
                    last_activity: now,
                });
                Ok(OtaResponse::Progress { offset: 0 })
            }
            OtaRequest::Chunk { offset, data } => {
                let current = self.upload.as_mut().ok_or(OtaError::NoUpload)?;
                current.write(&mut self.flash, inactive.offset(), *offset, data)?;
                current.last_activity = now;
                Ok(OtaResponse::Progress {
                    offset: current.offset,
                })
            }
            OtaRequest::Finish => {
                let current = self.upload.as_ref().ok_or(OtaError::NoUpload)?;
                if current.offset != current.length {
                    return Err(OtaError::UnexpectedOffset);
                }
                if <[u8; 32]>::from(current.hash.clone().finalize()) != current.expected {
                    return Err(OtaError::DigestMismatch);
                }
                let slot = slots.select_inactive(&mut self.flash)?;
                self.rebooting = Some(slot);
                Ok(OtaResponse::Rebooting { slot })
            }
            OtaRequest::Abort => {
                self.upload = None;
                Ok(OtaResponse::Aborted)
            }
        }
    }
}

impl Upload {
    fn write(
        &mut self,
        flash: &mut Flash,
        base: u32,
        offset: u32,
        data: &[u8],
    ) -> Result<(), OtaError> {
        if data.is_empty() || !data.len().is_multiple_of(4) || !offset.is_multiple_of(4) {
            return Err(OtaError::InvalidLength);
        }
        let end = offset
            .checked_add(data.len() as u32)
            .ok_or(OtaError::UnexpectedOffset)?;
        if end > self.length || offset > self.offset || (offset < self.offset && end > self.offset)
        {
            return Err(OtaError::UnexpectedOffset);
        }
        if offset == 0 && !is_bridge_image(data) {
            return Err(OtaError::InvalidImage);
        }
        if offset == self.offset {
            while self.erased < end {
                let at = base + self.erased;
                flash.erase(at, at + SECTOR).map_err(|_| OtaError::Flash)?;
                self.erased += SECTOR;
            }
            flash
                .write(base + offset, data)
                .map_err(|_| OtaError::Flash)?;
        }
        let mut readback = [0; CHUNK_SIZE];
        let readback = &mut readback[..data.len()];
        flash
            .read(base + offset, readback)
            .map_err(|_| OtaError::Flash)?;
        if readback != data {
            return Err(OtaError::Flash);
        }
        if offset == self.offset {
            self.hash.update(readback);
            self.offset = end;
        }
        Ok(())
    }
}

#[embassy_executor::task]
async fn run(stack: &'static Stack, service: &'static Service) {
    let server = stack
        .endpoints()
        .bounded_server::<BridgeOtaEndpoint, 1>(None);
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve_full(async |msg: &HeaderMessage<OtaRequest>| {
                service
                    .borrow_mut()
                    .handle(&msg.t)
                    .unwrap_or_else(OtaResponse::Error)
            })
            .await;
        if service.borrow().rebooting.is_some() {
            // Return the activation reply before resetting the bridge.
            Timer::after_millis(750).await;
            system::software_reset();
        }
    }
}

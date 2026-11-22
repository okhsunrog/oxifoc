//! Application-slot updates for the C5 bridge over Ergot.

use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use super::bridge_auth::Nonce;

/// Leave room for the HMAC, nonce, sequence and Ergot header in one BLE frame.
pub const CHUNK_SIZE: usize = 384;

pub type Digest = [u8; 32];

#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
#[allow(
    clippy::large_enum_variant,
    reason = "bounded chunks keep the wire type allocation-free"
)]
pub enum OtaRequest {
    Info,
    Authenticated { upload: UploadRequest, tag: Digest },
}

#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
pub struct UploadRequest {
    pub nonce: Nonce,
    pub action: OtaAction,
}

#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
#[allow(
    clippy::large_enum_variant,
    reason = "bounded chunks keep the wire type allocation-free"
)]
pub enum OtaAction {
    Begin {
        length: u32,
        digest: Digest,
    },
    Chunk {
        offset: u32,
        data: Vec<u8, CHUNK_SIZE>,
    },
    Finish,
    Abort,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub enum OtaError {
    Disabled,
    PartitionLayout,
    Unauthorized,
    Busy,
    InvalidLength,
    InvalidImage,
    NoUpload,
    UnexpectedOffset,
    DigestMismatch,
    Flash,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Schema)]
pub enum OtaResponse {
    Info {
        nonce: Nonce,
        booted_slot: u8,
        capacity: u32,
    },
    Progress {
        offset: u32,
    },
    Rebooting {
        slot: u8,
    },
    Aborted,
    Error(OtaError),
}

/// The ESP-IDF image header and app descriptor identify the bridge image.
pub fn is_bridge_image(first_bytes: &[u8]) -> bool {
    first_bytes.first() == Some(&0xe9)
        && first_bytes.get(12..14) == Some(&23u16.to_le_bytes())
        && first_bytes.get(32..36) == Some(&0xabcd_5432u32.to_le_bytes())
        && first_bytes.get(80..94) == Some(b"oxifoc-bridge\0")
}

#[cfg(feature = "bridge-auth")]
mod authentication {
    use super::{CHUNK_SIZE, Digest, UploadRequest};
    use crate::types::bridge_auth::{self, Key};

    const DOMAIN: &[u8] = b"OxiFOC C5 OTA v1\0";

    pub fn sign(key: &Key, upload: &UploadRequest) -> Option<Digest> {
        let mut bytes = [0; CHUNK_SIZE + 32];
        let data = postcard::to_slice(upload, &mut bytes).ok()?;
        Some(bridge_auth::sign(key, DOMAIN, data))
    }

    pub fn verify(key: &Key, upload: &UploadRequest, tag: &Digest) -> bool {
        let mut bytes = [0; CHUNK_SIZE + 32];
        postcard::to_slice(upload, &mut bytes)
            .is_ok_and(|data| bridge_auth::verify(key, DOMAIN, data, tag))
    }
}

#[cfg(feature = "bridge-auth")]
pub use authentication::{sign, verify};

#[cfg(all(test, feature = "bridge-auth"))]
mod tests {
    use super::*;

    #[test]
    fn authenticated_chunk_fits_ble_and_binds_offset_and_nonce() {
        let key = [0x23; 32];
        let mut upload = UploadRequest {
            nonce: [5; 16],
            action: OtaAction::Chunk {
                offset: 0x1de000,
                data: Vec::from_slice(&[0xff; CHUNK_SIZE]).unwrap(),
            },
        };
        let tag = sign(&key, &upload).unwrap();
        assert!(verify(&key, &upload, &tag));
        assert!(!verify(&[0; 32], &upload, &tag));
        let request = OtaRequest::Authenticated {
            upload: upload.clone(),
            tag,
        };
        let mut wire = [0; 481]; // 505-byte BLE MTU minus maximum Ergot header.
        assert!(postcard::to_slice(&request, &mut wire).is_ok());
        upload.nonce[0] ^= 1;
        assert!(!verify(&key, &upload, &tag));
        upload.nonce[0] ^= 1;
        let OtaAction::Chunk { offset, .. } = &mut upload.action else {
            unreachable!()
        };
        *offset += 4;
        assert!(!verify(&key, &upload, &tag));
    }
}

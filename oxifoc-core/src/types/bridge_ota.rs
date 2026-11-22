//! Application-slot updates for the C5 bridge over Ergot.

use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

/// Leave room for the HMAC, nonce, sequence and Ergot header in one BLE frame.
pub const CHUNK_SIZE: usize = 384;

pub type Nonce = [u8; 16];
pub type Digest = [u8; 32];
pub type OtaKey = [u8; 32];

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

#[cfg(feature = "bridge-ota-auth")]
mod authentication {
    use hmac::{Hmac, KeyInit as _, Mac as _};
    use sha2::Sha256;

    use super::{CHUNK_SIZE, Digest, OtaKey, UploadRequest};

    pub fn parse_key(text: &str) -> Option<OtaKey> {
        let (pairs, rest) = text.as_bytes().as_chunks::<2>();
        if pairs.len() != 32 || !rest.is_empty() {
            return None;
        }
        let mut key = [0; 32];
        for (byte, pair) in key.iter_mut().zip(pairs) {
            let pair = core::str::from_utf8(pair).ok()?;
            *byte = u8::from_str_radix(pair, 16).ok()?;
        }
        Some(key)
    }

    fn mac(key: &OtaKey, upload: &UploadRequest) -> Option<Hmac<Sha256>> {
        let mut bytes = [0; CHUNK_SIZE + 32];
        let data = postcard::to_slice(upload, &mut bytes).ok()?;
        let mut mac = Hmac::<Sha256>::new_from_slice(key).ok()?;
        mac.update(b"OxiFOC C5 OTA v1\0");
        mac.update(data);
        Some(mac)
    }

    pub fn sign(key: &OtaKey, upload: &UploadRequest) -> Option<Digest> {
        Some(mac(key, upload)?.finalize().into_bytes().into())
    }

    pub fn verify(key: &OtaKey, upload: &UploadRequest, tag: &Digest) -> bool {
        mac(key, upload).is_some_and(|mac| mac.verify_slice(tag).is_ok())
    }
}

#[cfg(feature = "bridge-ota-auth")]
pub use authentication::{parse_key, sign, verify};

#[cfg(all(test, feature = "bridge-ota-auth"))]
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

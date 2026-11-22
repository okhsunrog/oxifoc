//! Application-slot updates for the C5 bridge over Ergot.

use heapless::Vec;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

/// A chunk request fits one BLE frame with its Ergot header.
pub const CHUNK_SIZE: usize = 384;

pub type Digest = [u8; 32];

#[derive(Debug, Clone, Serialize, Deserialize, Schema)]
#[allow(
    clippy::large_enum_variant,
    reason = "bounded chunks keep the wire type allocation-free"
)]
pub enum OtaRequest {
    Info,
    /// Start an upload, or resume the one with the same length and digest.
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
    PartitionLayout,
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
        /// Random per boot: a change tells the host the bridge restarted.
        boot: u32,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_chunk_fits_a_ble_frame() {
        let request = OtaRequest::Chunk {
            offset: u32::MAX,
            data: Vec::from_slice(&[0xff; CHUNK_SIZE]).unwrap(),
        };
        let mut wire = [0; 481]; // 505-byte BLE MTU minus maximum Ergot header.
        assert!(postcard::to_slice(&request, &mut wire).is_ok());
    }
}

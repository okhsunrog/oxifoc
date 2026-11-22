//! Sequenced Ergot datagrams negotiated between the C5 and its UDP host.
//!
//! Downstream frames keep their original Ergot bytes. Upstream frames keep
//! the standard UDP format. The host requests repeats of downstream packets.

use crate::icd::UDP_MTU;

const MAGIC: [u8; 3] = *b"OXU";
pub const HEADER_BYTES: usize = 12;
pub const FRAME_MTU: u16 = UDP_MTU - HEADER_BYTES as u16;
pub const HISTORY_PACKETS: usize = 32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Kind {
    Hello = 0x10,
    Ready = 0x11,
    Frame = 0x12,
    Replay = 0x13,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Header {
    pub kind: Kind,
    pub token: u32,
    pub sequence: u32,
}

impl Header {
    pub fn encode(self) -> [u8; HEADER_BYTES] {
        let mut bytes = [0; HEADER_BYTES];
        bytes[..3].copy_from_slice(&MAGIC);
        bytes[3] = self.kind as u8;
        bytes[4..8].copy_from_slice(&self.token.to_le_bytes());
        bytes[8..].copy_from_slice(&self.sequence.to_le_bytes());
        bytes
    }

    pub fn decode(bytes: &[u8]) -> Option<(Self, &[u8])> {
        let (header, payload) = bytes.split_first_chunk::<HEADER_BYTES>()?;
        if header[..3] != MAGIC {
            return None;
        }
        let kind = match header[3] {
            0x10 => Kind::Hello,
            0x11 => Kind::Ready,
            0x12 => Kind::Frame,
            0x13 => Kind::Replay,
            _ => return None,
        };
        if (kind == Kind::Frame) == payload.is_empty() || payload.len() > usize::from(FRAME_MTU) {
            return None;
        }
        Some((
            Self {
                kind,
                token: u32::from_le_bytes(header[4..8].try_into().ok()?),
                sequence: u32::from_le_bytes(header[8..].try_into().ok()?),
            },
            payload,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::{HEADER_BYTES, Header, Kind};

    #[test]
    fn envelope_preserves_the_frame() {
        let header = Header {
            kind: Kind::Frame,
            token: 42,
            sequence: u32::MAX,
        };
        let mut bytes = [0; HEADER_BYTES + 3];
        bytes[..HEADER_BYTES].copy_from_slice(&header.encode());
        bytes[HEADER_BYTES..].copy_from_slice(&[0, 0xff, 0]);
        assert_eq!(Header::decode(&bytes), Some((header, &[0, 0xff, 0][..])));
        assert!(Header::decode(&bytes[..HEADER_BYTES]).is_none());
        bytes[3] = Kind::Replay as u8;
        assert!(Header::decode(&bytes).is_none());
        assert_eq!(
            Header::decode(&bytes[..HEADER_BYTES]).unwrap().0.kind,
            Kind::Replay
        );
    }
}

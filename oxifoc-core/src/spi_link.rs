//! Fixed-size full-duplex SPI slots carrying complete Ergot frames.
//!
//! DMA always transfers the same size, so resets and short CS transactions do
//! not change the next transaction's expected length. Frames are coalesced to
//! amortize DMA wakeups. CRC covers the header, used payload and zero alignment
//! bytes; unused slot capacity is ignored. This is best-effort framing:
//! application requests retain their existing acknowledgements and retry IDs.

pub const PAYLOAD_BYTES: usize = 4096;
pub const HEADER_BYTES: usize = 20;
pub const SLOT_BYTES: usize = HEADER_BYTES + PAYLOAD_BYTES;
pub const MAX_FRAME_BYTES: usize = 2304;
const MAGIC: [u8; 4] = *b"OXSP";
const VERSION: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    BufferSize,
    Signature,
    Version,
    Checksum,
    Length,
    Capacity,
}

/// A DMA buffer borrow ends at `finish`, before DMA takes ownership again.
pub struct Builder<'a> {
    bytes: &'a mut [u8],
    used: usize,
}

impl<'a> Builder<'a> {
    pub fn new(bytes: &'a mut [u8], epoch: u32, seq: u32) -> Result<Self, Error> {
        if bytes.len() != SLOT_BYTES {
            return Err(Error::BufferSize);
        }
        bytes.fill(0);
        bytes[..4].copy_from_slice(&MAGIC);
        bytes[4..8].copy_from_slice(&epoch.to_le_bytes());
        bytes[8..12].copy_from_slice(&seq.to_le_bytes());
        bytes[14] = VERSION;
        Ok(Self { bytes, used: 0 })
    }

    pub fn remaining(&self) -> usize {
        PAYLOAD_BYTES - self.used
    }

    /// Failure leaves the existing slot unchanged; keep the queue grant.
    pub fn append(&mut self, frame: &[u8]) -> Result<(), Error> {
        if frame.is_empty() || frame.len() > MAX_FRAME_BYTES {
            return Err(Error::Length);
        }
        if self.remaining() < frame.len() + 2 {
            return Err(Error::Capacity);
        }
        let at = HEADER_BYTES + self.used;
        self.bytes[at..at + 2].copy_from_slice(&(frame.len() as u16).to_le_bytes());
        self.bytes[at + 2..at + 2 + frame.len()].copy_from_slice(frame);
        self.used += frame.len() + 2;
        Ok(())
    }

    pub fn finish(self, checksum: impl FnOnce(&[u8]) -> u32) {
        self.bytes[12..14].copy_from_slice(&(self.used as u16).to_le_bytes());
        let end = (HEADER_BYTES + self.used + 3) & !3;
        let crc = checksum(&self.bytes[..end]);
        self.bytes[16..20].copy_from_slice(&crc.to_le_bytes());
    }
}

pub struct Received<'a> {
    pub epoch: u32,
    pub seq: u32,
    payload: &'a [u8],
}

impl Received<'_> {
    pub fn frames(&self) -> Frames<'_> {
        Frames { rest: self.payload }
    }
}

/// Validates the entire slot before exposing any frame to the router.
pub fn parse(bytes: &mut [u8], checksum: impl FnOnce(&[u8]) -> u32) -> Result<Received<'_>, Error> {
    if bytes.len() != SLOT_BYTES {
        return Err(Error::BufferSize);
    }
    if bytes[..4] != MAGIC {
        return Err(Error::Signature);
    }
    if bytes[14] != VERSION || bytes[15] != 0 {
        return Err(Error::Version);
    }
    let used = u16::from_le_bytes([bytes[12], bytes[13]]) as usize;
    if used > PAYLOAD_BYTES {
        return Err(Error::Length);
    }
    let expected = u32::from_le_bytes(bytes[16..20].try_into().map_err(|_| Error::Length)?);
    bytes[16..20].fill(0);
    let actual = checksum(&bytes[..(HEADER_BYTES + used + 3) & !3]);
    bytes[16..20].copy_from_slice(&expected.to_le_bytes());
    if actual != expected {
        return Err(Error::Checksum);
    }
    let payload = &bytes[HEADER_BYTES..HEADER_BYTES + used];
    // Walk it now: malformed trailing frames cannot partially reach Ergot.
    let mut rest = payload;
    while !rest.is_empty() {
        if rest.len() < 2 {
            return Err(Error::Length);
        }
        let len = u16::from_le_bytes([rest[0], rest[1]]) as usize;
        if len == 0 || len > MAX_FRAME_BYTES || len + 2 > rest.len() {
            return Err(Error::Length);
        }
        rest = &rest[len + 2..];
    }
    Ok(Received {
        epoch: u32::from_le_bytes(bytes[4..8].try_into().map_err(|_| Error::Length)?),
        seq: u32::from_le_bytes(bytes[8..12].try_into().map_err(|_| Error::Length)?),
        payload,
    })
}

pub struct Frames<'a> {
    rest: &'a [u8],
}
impl<'a> Iterator for Frames<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<Self::Item> {
        if self.rest.is_empty() {
            return None;
        }
        let len = u16::from_le_bytes([self.rest[0], self.rest[1]]) as usize;
        let frame = &self.rest[2..len + 2];
        self.rest = &self.rest[len + 2..];
        Some(frame)
    }
}

/// CRC-32/MPEG-2 reference. Platforms may substitute hardware or ROM code.
pub fn checksum(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for byte in bytes {
        crc ^= u32::from(*byte) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ 0x04c1_1db7
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_reference_and_coalesced_unaligned_frames() {
        assert_eq!(checksum(b"123456789"), 0x0376_e6e7);
        let mut bytes = [0; SLOT_BYTES];
        let mut tx = Builder::new(&mut bytes, 123, 456).unwrap();
        tx.append(&[9; 137]).unwrap();
        tx.append(&[5; 512]).unwrap();
        tx.finish(checksum);
        let rx = parse(&mut bytes, checksum).unwrap();
        assert_eq!((rx.epoch, rx.seq), (123, 456));
        let mut frames = rx.frames();
        assert_eq!(frames.next().unwrap(), &[9; 137]);
        assert_eq!(frames.next().unwrap(), &[5; 512]);
        assert!(frames.next().is_none());
    }

    #[test]
    fn corruption_is_rejected_before_delivering_any_frame() {
        let mut bytes = [0; SLOT_BYTES];
        let mut tx = Builder::new(&mut bytes, 1, 2).unwrap();
        tx.append(&[3; 137]).unwrap();
        tx.finish(checksum);
        bytes[HEADER_BYTES + 5] ^= 1;
        assert!(matches!(parse(&mut bytes, checksum), Err(Error::Checksum)));
        bytes[12..14].copy_from_slice(&u16::MAX.to_le_bytes());
        assert!(matches!(parse(&mut bytes, checksum), Err(Error::Length)));
    }

    #[test]
    fn capacity_failure_preserves_the_previous_frame() {
        let mut bytes = [0; SLOT_BYTES];
        let mut tx = Builder::new(&mut bytes, 1, 2).unwrap();
        tx.append(&[1; MAX_FRAME_BYTES]).unwrap();
        assert_eq!(tx.append(&[2; MAX_FRAME_BYTES]), Err(Error::Capacity));
        tx.finish(checksum);
        assert_eq!(parse(&mut bytes, checksum).unwrap().frames().count(), 1);
    }
}

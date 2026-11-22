//! OpenOCD CMSIS-DAP TCP envelope. The negotiated CMSIS-DAP packet size
//! bounds both request and response payloads, excluding this header.

pub const PORT: u16 = 4441;
pub const HEADER_LEN: usize = 8;
pub const PACKET_SIZE: u16 = 128;
const SIGNATURE: [u8; 4] = *b"DAP\0";
const REQUEST: u8 = 1;
const RESPONSE: u8 = 2;

pub fn request_len(header: [u8; HEADER_LEN]) -> Option<usize> {
    let [d, a, p, zero, lo, hi, kind, reserved] = header;
    let len = u16::from_le_bytes([lo, hi]);
    ([d, a, p, zero] == SIGNATURE
        && kind == REQUEST
        && reserved == 0
        && (1..=PACKET_SIZE).contains(&len))
    .then_some(usize::from(len))
}

pub fn response_header(len: u16) -> [u8; HEADER_LEN] {
    let [d, a, p, zero] = SIGNATURE;
    let [lo, hi] = len.to_le_bytes();
    [d, a, p, zero, lo, hi, RESPONSE, 0]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn openocd_envelope() {
        assert_eq!(PORT, 4441);
        assert_eq!(request_len(*b"DAP\0\x02\x00\x01\x00"), Some(2));
        assert_eq!(request_len(*b"DAP\0\x80\x00\x01\x00"), Some(128));
        assert_eq!(response_header(4), *b"DAP\0\x04\x00\x02\x00");
        assert_eq!(response_header(1024), *b"DAP\0\x00\x04\x02\x00");
    }

    #[test]
    fn rejects_invalid_headers() {
        for header in [
            *b"BAD\0\x01\x00\x01\x00",
            *b"DAP\0\x00\x00\x01\x00",
            *b"DAP\0\x81\x00\x01\x00",
            *b"DAP\0\x01\x01\x01\x00",
            *b"DAP\0\x01\x00\x02\x00",
            *b"DAP\0\x01\x00\x01\x01",
        ] {
            assert_eq!(request_len(header), None);
        }
    }
}

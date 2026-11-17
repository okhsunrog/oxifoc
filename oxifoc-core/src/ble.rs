//! BLE GATT identity of the OxiFOC ergot transport.
//!
//! The service mirrors the Nordic UART Service layout (one write-without-response
//! RX characteristic, one notify TX characteristic) under its own random 128-bit
//! base. Generic NUS clients such as VESC Tool therefore do not list an OxiFOC
//! board, and OxiFOC hosts can filter scans by [`SERVICE_UUID`].
//!
//! Values are big-endian `u128` in the usual textual order. BLE carries them
//! little-endian; use `u128::to_le_bytes` where a stack wants raw bytes.

/// OxiFOC ergot transport service: `0b960001-d748-4b2e-b03a-6ada6467f15d`.
pub const SERVICE_UUID: u128 = 0x0b960001_d748_4b2e_b03a_6ada6467f15d;

/// RX characteristic, central → peripheral, write without response:
/// `0b960002-d748-4b2e-b03a-6ada6467f15d`.
pub const RX_UUID: u128 = 0x0b960002_d748_4b2e_b03a_6ada6467f15d;

/// TX characteristic, peripheral → central, notify:
/// `0b960003-d748-4b2e-b03a-6ada6467f15d`.
pub const TX_UUID: u128 = 0x0b960003_d748_4b2e_b03a_6ada6467f15d;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn characteristics_share_the_service_base() {
        let base = |uuid: u128| uuid & !(0xffff_u128 << 96);
        assert_eq!(base(RX_UUID), base(SERVICE_UUID));
        assert_eq!(base(TX_UUID), base(SERVICE_UUID));
        assert_eq!((SERVICE_UUID >> 96) & 0xffff, 1);
        assert_eq!((RX_UUID >> 96) & 0xffff, 2);
        assert_eq!((TX_UUID >> 96) & 0xffff, 3);
    }
}

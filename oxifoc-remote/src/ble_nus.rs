//! Nordic UART Service (NUS) constants.
//!
//! The remote is a BLE central (GATT client), so it doesn't define
//! a GATT server — only the shared constants for NUS UUIDs and payload size.

/// Maximum payload per GATT characteristic value.
///
/// With packet pool MTU 512 (set via TROUBLE_HOST_DEFAULT_PACKET_POOL_MTU),
/// negotiated ATT_MTU is 508, giving 505 bytes of usable payload.
pub const NUS_MAX_PAYLOAD: usize = 505;

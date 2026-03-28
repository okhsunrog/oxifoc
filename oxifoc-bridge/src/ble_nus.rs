//! Nordic UART Service (NUS) GATT definition.
//!
//! Defines a GATT server with a single NUS service for bidirectional
//! ergot frame transfer over BLE. Each ergot frame maps to one GATT
//! write (central → peripheral) or notification (peripheral → central).

use trouble_host::prelude::*;

/// Maximum payload per GATT characteristic value.
///
/// With packet pool MTU 512 (set via TROUBLE_HOST_DEFAULT_PACKET_POOL_MTU),
/// negotiated ATT_MTU is 508, giving 505 bytes of usable payload.
pub const NUS_MAX_PAYLOAD: usize = 505;

#[gatt_server]
pub struct NusServer {
    pub nus: NusService,
}

#[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
pub struct NusService {
    /// RX characteristic — central writes ergot frames here.
    #[characteristic(uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e", write_without_response)]
    pub rx: heapless::Vec<u8, NUS_MAX_PAYLOAD>,

    /// TX characteristic — peripheral notifies ergot frames to central.
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
    pub tx: heapless::Vec<u8, NUS_MAX_PAYLOAD>,
}

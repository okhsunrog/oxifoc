//! OxiFOC ergot transport GATT definition.
//!
//! A NUS-shaped service under the OxiFOC UUIDs from [`oxifoc_core::ble`]
//! carries ergot frames in both directions. Each ergot frame maps to one GATT
//! write (central → peripheral) or notification (peripheral → central).

// The gatt_server/gatt_service macros re-qualify types in their expansions,
// tripping unused_qualifications on spans we can't reach with item-level
// allows — suppress for the whole (small) module.
#![allow(unused_qualifications)]

use heapless::Vec;
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

#[gatt_service(uuid = oxifoc_core::ble::SERVICE_UUID)]
pub struct NusService {
    /// RX characteristic — central writes ergot frames here.
    #[characteristic(uuid = oxifoc_core::ble::RX_UUID, write_without_response)]
    pub rx: Vec<u8, NUS_MAX_PAYLOAD>,

    /// TX characteristic — peripheral notifies ergot frames to central.
    #[characteristic(uuid = oxifoc_core::ble::TX_UUID, notify)]
    pub tx: Vec<u8, NUS_MAX_PAYLOAD>,
}

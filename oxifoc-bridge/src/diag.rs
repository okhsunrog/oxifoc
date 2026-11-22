//! Bridge diagnostics, served as `req/bridge_diag`: the counters of the links
//! the bridge carries and its own crash record, reachable over BLE even when
//! the controller is silent.

use core::pin::pin;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use oxifoc_core::icd::BridgeDiagEndpoint;
use oxifoc_core::types::diag::{
    BleLinkDiag, BridgeDiag, BridgeDiagRequest, BridgeDiagResponse, SpiLinkDiag, UsbLinkDiag,
    WifiLinkDiag,
};

use crate::{
    crash,
    transport::{BLE_LINKS, Stack, spi, usb},
};

/// Counters of one BLE central slot.
pub struct BleCounters {
    pub connected: AtomicBool,
    pub trusted: AtomicBool,
    pub connects: AtomicU32,
    pub disconnects: AtomicU32,
    pub last_disconnect_reason: AtomicU8,
    pub untrusted_drops: AtomicU32,
    pub notify_errors: AtomicU32,
    pub dropped: AtomicU32,
}

impl BleCounters {
    const fn new() -> Self {
        Self {
            connected: AtomicBool::new(false),
            trusted: AtomicBool::new(false),
            connects: AtomicU32::new(0),
            disconnects: AtomicU32::new(0),
            last_disconnect_reason: AtomicU8::new(0),
            untrusted_drops: AtomicU32::new(0),
            notify_errors: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
        }
    }

    pub fn bump(counter: &AtomicU32) {
        counter.fetch_add(1, Ordering::Relaxed);
    }

    fn snapshot(&self) -> BleLinkDiag {
        BleLinkDiag {
            connected: self.connected.load(Ordering::Relaxed),
            trusted: self.trusted.load(Ordering::Relaxed),
            connects: self.connects.load(Ordering::Relaxed),
            disconnects: self.disconnects.load(Ordering::Relaxed),
            last_disconnect_reason: self.last_disconnect_reason.load(Ordering::Relaxed),
            untrusted_drops: self.untrusted_drops.load(Ordering::Relaxed),
            notify_errors: self.notify_errors.load(Ordering::Relaxed),
            dropped: self.dropped.load(Ordering::Relaxed),
        }
    }
}

pub static BLE: [BleCounters; BLE_LINKS] = [const { BleCounters::new() }; BLE_LINKS];

/// The bridge's raw reset-reason code, recorded at boot.
pub static RESET_REASON: AtomicU8 = AtomicU8::new(0);

fn snapshot() -> BridgeDiag {
    let load = |counter: &AtomicU32| counter.load(Ordering::Relaxed);
    BridgeDiag {
        uptime_ms: embassy_time::Instant::now().as_millis(),
        reset_reason: RESET_REASON.load(Ordering::Relaxed),
        spi: SpiLinkDiag {
            slots: load(&spi::SPI_LINK_SLOTS),
            errors: load(&spi::SPI_LINK_ERRORS),
            length_errors: load(&spi::SPI_RX_LENGTH_ERRORS),
            crc_errors: load(&spi::SPI_RX_CRC_ERRORS),
            header_errors: load(&spi::SPI_RX_HEADER_ERRORS),
            peer_boots: load(&spi::SPI_PEER_BOOTS),
        },
        usb: UsbLinkDiag {
            dropped: load(&usb::USB_DROPPED),
            write_errors: load(&usb::USB_WRITE_ERRORS),
            max_grant: load(&usb::USB_MAX_GRANT),
        },
        ble: core::array::from_fn(|link| BLE[link].snapshot()),
        wifi: wifi(),
    }
}

#[cfg(feature = "wifi")]
fn wifi() -> WifiLinkDiag {
    use crate::transport::tcp;
    WifiLinkDiag {
        connected: tcp::CONNECTED.load(Ordering::Relaxed),
        connects: tcp::CONNECTS.load(Ordering::Relaxed),
        dropped: tcp::DROPPED.load(Ordering::Relaxed),
        max_backlog: tcp::MAX_BACKLOG.load(Ordering::Relaxed),
    }
}

#[cfg(not(feature = "wifi"))]
fn wifi() -> WifiLinkDiag {
    WifiLinkDiag::default()
}

/// Serve `req/bridge_diag`.
pub async fn server(stack: &'static Stack) {
    let server = stack
        .endpoints()
        .bounded_server::<BridgeDiagEndpoint, 2>(Some("bridge_diag"));
    let server = pin!(server);
    let mut handle = server.attach();
    loop {
        let _ = handle
            .serve(|request: &BridgeDiagRequest| {
                let reply = match request {
                    BridgeDiagRequest::Links => BridgeDiagResponse::Links(snapshot()),
                    BridgeDiagRequest::Crash => BridgeDiagResponse::Crash(crash::read()),
                    BridgeDiagRequest::ClearCrash => {
                        crash::clear();
                        BridgeDiagResponse::Crash(None)
                    }
                };
                async move { reply }
            })
            .await;
    }
}

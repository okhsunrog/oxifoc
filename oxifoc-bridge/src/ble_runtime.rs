use crate::{ble_nus::NusServer, transport};
use bt_hci::controller::ExternalController;
use defmt::{error, info, warn};
use embassy_futures::{
    join::join,
    select::{Either3, select3},
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile,
    profiles::router::{RouterFrameProcessor, UPSTREAM_IDENT},
};
use ergot::net_stack::services::{
    SeedLease, bridge_seed_assign, bridge_seed_refresh, release_seed_lease,
};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

fn upstream_discovered(stack: &'static transport::Stack) -> bool {
    stack.manage_profile(|p| matches!(p.interface_state(UPSTREAM_IDENT), Some(InterfaceState::Active { net_id, .. }) if net_id != 0))
}

#[embassy_executor::task]
pub async fn run(
    stack: &'static transport::Stack,
    bt: esp_hal::peripherals::BT<'static>,
    ble_ident: u8,
) {
    // ========== BLE setup ==========
    let bt_transport = BleConnector::new(bt, Default::default()).expect("BLE connector init");
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    let ble_stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .build();
    let mut peripheral = ble_stack.peripheral();
    let runner = ble_stack.runner();

    let nus_server = NusServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Oxifoc Bridge",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .expect("NUS GATT server init");

    join(ble_runner(runner), async {
        loop {
            match advertise(&mut peripheral, &nus_server).await {
                Ok(conn) => {
                    info!("[ble] connection established");

                    // Request 2M PHY for higher throughput
                    if let Err(_e) = conn.raw().set_phy(&ble_stack, PhyKind::Le2M).await {
                        warn!("[ble] failed to set 2M PHY, continuing with 1M");
                    } else {
                        info!("[ble] 2M PHY requested");
                    }

                    // The BLE segment needs a root-leased net before it
                    // can carry routed traffic.
                    let Some(mut lease) = acquire_ble_lease(stack, ble_ident).await else {
                        warn!("[ble] no seed lease (upstream down?), dropping connection");
                        continue;
                    };
                    info!("[ble] seed lease acquired: net_id={}", lease.net_id);

                    nus_connection_task(stack, &nus_server, &conn, ble_ident, &mut lease).await;

                    stack.manage_profile(|router| {
                        let _ = router.set_interface_state(ble_ident, InterfaceState::Inactive);
                    });
                    // Best-effort: hand the net back instead of letting
                    // the lease age out on the root.
                    let _ =
                        with_timeout(Duration::from_secs(1), release_seed_lease(&stack, &lease))
                            .await;
                    info!("[ble] disconnected, returning to advertising");
                }
                Err(_e) => {
                    warn!("[ble] advertise error, retrying");
                    Timer::after_secs(1).await;
                }
            }
        }
    })
    .await;
}

#[allow(
    clippy::large_stack_frames,
    reason = "the ergot request future dominates; this future lives in main's \
    arena-allocated join, not on a call stack"
)]
async fn acquire_ble_lease(stack: &'static transport::Stack, ble_ident: u8) -> Option<SeedLease> {
    for _ in 0..10 {
        if upstream_discovered(stack) {
            match bridge_seed_assign(&stack, UPSTREAM_IDENT, ble_ident).await {
                Ok(lease) => return Some(lease),
                Err(_e) => warn!("[ble] seed assignment attempt failed"),
            }
        }
        Timer::after_secs(1).await;
    }
    None
}

async fn ble_runner<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(_e) = runner.run().await {
            error!("[ble] runner error, restarting");
        }
    }
}

// ========== BLE advertise ==========

#[allow(clippy::large_stack_frames)]
async fn advertise<'v, 's, C: Controller>(
    peripheral: &mut Peripheral<'v, C, DefaultPacketPool>,
    server: &'s NusServer<'v>,
) -> Result<GattConnection<'v, 's, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"Oxifoc Bridge"),
        ],
        &mut adv_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("[ble] advertising...");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

// ========== BLE NUS connection handler ==========

#[allow(clippy::large_stack_frames)]
async fn nus_connection_task<P: PacketPool>(
    stack: &'static transport::Stack,
    server: &NusServer<'_>,
    conn: &GattConnection<'_, '_, P>,
    ble_ident: u8,
    lease: &mut SeedLease,
) {
    let rx_handle = server.nus.rx.handle;
    let tx_char = &server.nus.tx;
    let consumer = transport::BLE_OUTQ.framed_consumer();
    // net_id 0 is the pending placeholder: the processor syncs the
    // seed-assigned net from the (now Active) slot on the first frame.
    let mut processor = RouterFrameProcessor::new(0);
    // Absolute deadline held across loop iterations — recomputing it per
    // iteration would re-arm the timer on every GATT/TX event and starve
    // the refresh under active traffic.
    let mut next_refresh = lease_refresh_deadline(lease);

    loop {
        let gatt_event = conn.next();
        let tx_frame = consumer.wait_read();
        let refresh_at = Timer::at(next_refresh);

        match select3(gatt_event, tx_frame, refresh_at).await {
            Either3::First(event) => match event {
                GattConnectionEvent::Disconnected { .. } => {
                    info!("[ble] disconnected");
                    break;
                }
                GattConnectionEvent::Gatt { event } => {
                    if let GattEvent::Write(ref write_event) = event
                        && write_event.handle() == rx_handle
                    {
                        let changed = write_event.with_data(|offset, data| {
                            offset == 0 && processor.process_frame(data, &stack, ble_ident)
                        });
                        if changed {
                            transport::STATE_NOTIFY.wake_all();
                        }
                    }
                    match event.accept() {
                        Ok(reply) => reply.send().await,
                        Err(_e) => warn!("[ble] error sending gatt reply"),
                    }
                }
                _ => {}
            },

            Either3::Second(grant) => {
                let payload: heapless::Vec<u8, { crate::ble_nus::NUS_MAX_PAYLOAD }> =
                    heapless::Vec::from_slice(&grant).unwrap_or_default();
                if let Err(_e) = tx_char.notify(conn, &payload, false).await {
                    warn!("[ble] notify error, disconnecting");
                    grant.release();
                    break;
                }
                grant.release();
            }

            Either3::Third(()) => {
                next_refresh = maintain_ble_lease(stack, ble_ident, lease).await;
            }
        }
    }
}

/// When the current lease should be refreshed: once the remaining time drops
/// inside the root's refresh window (with a 2 s margin so the request lands
/// inside it even under link jitter).
fn lease_refresh_deadline(lease: &SeedLease) -> Instant {
    let delay = lease
        .expires_seconds
        .saturating_sub(lease.min_refresh_seconds)
        .saturating_add(2);
    // Anchored to the acquire/refresh moment: the lease carries relative
    // seconds, and each successful refresh resets the horizon.
    Instant::now() + Duration::from_secs(u64::from(delay.max(1)))
}

/// Refresh the BLE segment's seed lease; if the root no longer knows it
/// (expired while the link was down, root rebooted), fall back to a fresh
/// assignment, which also re-points the slot's net_id. Returns the next
/// refresh deadline: a full window on success, a short retry on failure.
#[allow(
    clippy::large_stack_frames,
    reason = "the ergot request future dominates; this future lives in main's \
    arena-allocated join, not on a call stack"
)]
async fn maintain_ble_lease(
    stack: &'static transport::Stack,
    ble_ident: u8,
    lease: &mut SeedLease,
) -> Instant {
    match bridge_seed_refresh(&stack, lease).await {
        Ok(refreshed) => {
            *lease = refreshed;
            lease_refresh_deadline(lease)
        }
        Err(_e) => {
            warn!("[ble] lease refresh failed, re-assigning");
            if let Ok(fresh) = bridge_seed_assign(&stack, UPSTREAM_IDENT, ble_ident).await {
                info!("[ble] re-assigned net_id={}", fresh.net_id);
                *lease = fresh;
                lease_refresh_deadline(lease)
            } else {
                Instant::now() + Duration::from_secs(2)
            }
        }
    }
}

use crate::{ble_nus::NusServer, transport};
use bt_hci::controller::ExternalController;
use defmt::{error, info, warn};
use embassy_futures::{
    join::join3,
    select::{Either, select},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::Timer;
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::router::RouterFrameProcessor,
};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
const CONNECTIONS_MAX: usize = transport::BLE_LINKS;
// Signal + ATT per link.
const L2CAP_CHANNELS_MAX: usize = 2 * CONNECTIONS_MAX;

#[embassy_executor::task]
pub async fn run(
    stack: &'static transport::Stack,
    bt: esp_hal::peripherals::BT<'static>,
    ble_idents: [u8; transport::BLE_LINKS],
) {
    // ========== BLE setup ==========
    // Maximum TX power: the board is powered and the link carries the throttle.
    let radio_config = esp_radio::ble::Config::default()
        .with_default_tx_power(esp_radio::ble::TxPower::P20)
        .with_max_connections(CONNECTIONS_MAX as u16);
    let bt_transport = BleConnector::new(bt, radio_config).expect("BLE connector init");
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    let ble_stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .build();
    let peripheral = Mutex::<NoopRawMutex, _>::new(ble_stack.peripheral());
    let runner = ble_stack.runner();

    let nus_server = NusServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Oxifoc Bridge",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .expect("NUS GATT server init");

    // Each link slot advertises while it is free. The peripheral mutex lets
    // one free slot advertise at a time; a connected slot releases it, so the
    // bridge keeps advertising while any slot is free.
    let [ident_a, ident_b] = ble_idents;
    join3(
        ble_runner(runner),
        link_slot(stack, &peripheral, &nus_server, 0, ident_a),
        link_slot(stack, &peripheral, &nus_server, 1, ident_b),
    )
    .await;
}

#[allow(clippy::large_stack_frames)]
async fn link_slot<'v, C: Controller>(
    stack: &'static transport::Stack,
    peripheral: &Mutex<NoopRawMutex, Peripheral<'v, C, DefaultPacketPool>>,
    server: &NusServer<'v>,
    link: usize,
    ident: u8,
) {
    loop {
        let accepted = {
            let mut peripheral = peripheral.lock().await;
            advertise(&mut peripheral, server).await
        };
        match accepted {
            Ok(conn) => {
                // The central chooses the PHY: the remote stays on 1M for
                // sensitivity, host tools may request 2M for telemetry.
                info!("[ble{}] connection established", link);
                nus_connection_task(stack, server, &conn, link, ident).await;
                stack.manage_profile(|router| {
                    let _ = router.set_interface_state(ident, InterfaceState::Inactive);
                });
                // Never send frames queued for the old central to the next one.
                let consumer = transport::BLE_OUTQ[link].framed_consumer();
                while let Ok(frame) = consumer.read() {
                    frame.release();
                }
                info!("[ble{}] disconnected, slot free", link);
            }
            Err(_e) => {
                warn!("[ble{}] advertise error, retrying", link);
                Timer::after_secs(1).await;
            }
        }
    }
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
    // Flags + the 128-bit service UUID take 21 of the 31 advertising bytes, so
    // the name goes in the scan response. Hosts filter scans by the UUID.
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[oxifoc_core::ble::SERVICE_UUID.to_le_bytes()]),
        ],
        &mut adv_data[..],
    )?;
    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(b"Oxifoc Bridge")],
        &mut scan_data[..],
    )?;

    // Fast advertising: the bridge is board-powered and a reconnecting remote
    // should find it within one scan window.
    let params = AdvertisementParameters {
        interval_min: embassy_time::Duration::from_millis(20),
        interval_max: embassy_time::Duration::from_millis(30),
        ..Default::default()
    };
    let advertiser = peripheral
        .advertise(
            &params,
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &scan_data[..scan_len],
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
    link: usize,
    ble_ident: u8,
) {
    let rx_handle = server.nus.rx.handle;
    let tx_char = &server.nus.tx;
    let consumer = transport::BLE_OUTQ[link].framed_consumer();
    let net = stack
        .manage_profile(|p| p.net_id_of(ble_ident))
        .expect("registered BLE segment");
    let mut processor = RouterFrameProcessor::new(net);
    loop {
        let gatt_event = conn.next();
        let tx_frame = consumer.wait_read();

        match select(gatt_event, tx_frame).await {
            Either::First(event) => match event {
                GattConnectionEvent::Disconnected { .. } => {
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

            Either::Second(grant) => {
                let payload: heapless::Vec<u8, { crate::ble_nus::NUS_MAX_PAYLOAD }> =
                    heapless::Vec::from_slice(&grant).unwrap_or_default();
                if let Err(_e) = tx_char.notify(conn, &payload, false).await {
                    warn!("[ble] notify error, disconnecting");
                    grant.release();
                    break;
                }
                grant.release();
            }
        }
    }
}

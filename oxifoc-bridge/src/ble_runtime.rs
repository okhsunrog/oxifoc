use crate::{ble_nus::NusServer, transport};
use bt_hci::controller::ExternalController;
use defmt::{error, info, warn};
use embassy_futures::{
    join::join,
    select::{Either, select},
};
use embassy_time::Timer;
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::router::RouterFrameProcessor,
};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

#[embassy_executor::task]
pub async fn run(
    stack: &'static transport::Stack,
    bt: esp_hal::peripherals::BT<'static>,
    ble_ident: u8,
) {
    // ========== BLE setup ==========
    let bt_transport = BleConnector::new(bt, Default::default()).expect("BLE connector init");
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
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

                    nus_connection_task(stack, &nus_server, &conn, ble_ident).await;
                    stack.manage_profile(|router| {
                        let _ = router.set_interface_state(ble_ident, InterfaceState::Inactive);
                    });
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
) {
    let rx_handle = server.nus.rx.handle;
    let tx_char = &server.nus.tx;
    let consumer = transport::BLE_OUTQ.framed_consumer();
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

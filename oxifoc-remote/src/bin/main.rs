#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;

use bt_hci::controller::ExternalController;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_futures::select::{Either, select};
use embassy_time::Timer;

use ergot::NetStack;
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};

use oxifoc_core::icd::{HardwareInfo, HardwareInfoEndpoint};
use oxifoc_remote::transport;

use defmt::{error, info, warn};

use panic_rtt_target as _;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + ATT + safety margin

/// Address of the bridge peripheral (must match oxifoc-bridge)
const BRIDGE_ADDR_BYTES: [u8; 6] = [0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff];

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Oxifoc Remote initialized!");

    // ========== BLE setup ==========
    let bt_transport =
        BleConnector::new(peripherals.BT, Default::default()).expect("BLE connector init");
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x2b, 0x06, 0xe5, 0xff]);
    let ble_stack = trouble_host::new(ble_controller, &mut resources).set_random_address(address);
    let Host {
        mut central,
        mut runner,
        ..
    } = ble_stack.build();

    let bridge_addr = Address::random(BRIDGE_ADDR_BYTES);

    // ========== Ergot stack setup (DirectEdge target) ==========
    let stack: &'static transport::Stack = {
        static STACK_CELL: static_cell::StaticCell<transport::Stack> =
            static_cell::StaticCell::new();
        STACK_CELL.init(NetStack::new_with_profile(DirectEdge::new_target(
            framed_stream::Sink::new(transport::BLE_OUTQ.framed_producer(), transport::BLE_MTU),
        )))
    };

    info!("Ergot stack initialized (DirectEdge target)");

    // ========== Run ==========
    let _ = join4(
        // BLE runner
        async {
            loop {
                if let Err(_e) = runner.run().await {
                    error!("[ble] runner error, restarting");
                }
            }
        },
        // HardwareInfo endpoint server
        info_server(stack),
        // Ergot device discovery handler
        stack
            .services()
            .device_info_handler::<2>(&ergot::well_known::DeviceInfo {
                name: Some("Oxifoc Remote".try_into().unwrap_or_default()),
                description: Some("ESK8 remote ctrl".try_into().unwrap_or_default()),
                unique_id: 0,
            }),
        // Connect + application loop
        async {
            loop {
                info!("[ble] connecting to bridge...");

                let connect_config = ConnectConfig {
                    connect_params: Default::default(),
                    scan_config: ScanConfig {
                        filter_accept_list: &[(bridge_addr.kind, &bridge_addr.addr)],
                        phys: PhySet::M1M2,
                        ..Default::default()
                    },
                };

                match central.connect(&connect_config).await {
                    Ok(conn) => {
                        info!("[ble] connected to bridge");

                        // Request 2M PHY
                        if let Err(_e) = conn.set_phy(&ble_stack, PhyKind::Le2M).await {
                            warn!("[ble] failed to set 2M PHY");
                        } else {
                            info!("[ble] 2M PHY set");
                        }

                        // Create GATT client
                        let client =
                            match GattClient::<_, DefaultPacketPool, 10>::new(&ble_stack, &conn)
                                .await
                            {
                                Ok(c) => c,
                                Err(_e) => {
                                    error!("[ble] GATT client creation failed");
                                    continue;
                                }
                            };

                        // Run GATT client task + NUS connection
                        let _ = join(client.task(), async {
                            nus_session(stack, &client, &conn).await;
                        })
                        .await;

                        // Disconnected
                        stack.manage_profile(|im| {
                            let _ = im.set_interface_state((), InterfaceState::Inactive);
                        });
                        info!("[ble] disconnected, will reconnect");
                    }
                    Err(_e) => {
                        warn!("[ble] connect failed, retrying");
                    }
                }

                Timer::after_secs(2).await;
            }
        },
    )
    .await;

    unreachable!()
}

/// NUS session: discover service, subscribe, run bidirectional loop.
#[allow(
    clippy::large_stack_frames,
    reason = "async BLE NUS future holds Notification<512> + DefaultPacketPool \
    buffers; it lives in embassy task storage, not the call stack."
)]
async fn nus_session<C: Controller>(
    stack: &'static transport::Stack,
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
    _conn: &Connection<'_, DefaultPacketPool>,
) {
    // NUS UUIDs
    let nus_uuid = Uuid::new_long([
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40,
        0x6e,
    ]);
    let rx_uuid = Uuid::new_long([
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40,
        0x6e,
    ]);
    let tx_uuid = Uuid::new_long([
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40,
        0x6e,
    ]);

    // Discover NUS service
    let services = match client.services_by_uuid(&nus_uuid).await {
        Ok(s) => s,
        Err(_e) => {
            error!("[nus] service discovery failed");
            return;
        }
    };
    let Some(service) = services.first() else {
        error!("[nus] NUS service not found");
        return;
    };

    // Find characteristics
    let rx_char: Characteristic<[u8]> = match client.characteristic_by_uuid(service, &rx_uuid).await
    {
        Ok(c) => c,
        Err(_e) => {
            error!("[nus] RX characteristic not found");
            return;
        }
    };
    let tx_char: Characteristic<[u8]> = match client.characteristic_by_uuid(service, &tx_uuid).await
    {
        Ok(c) => c,
        Err(_e) => {
            error!("[nus] TX characteristic not found");
            return;
        }
    };

    info!("[nus] service discovered, subscribing to notifications");

    // Subscribe to TX notifications (bridge → remote)
    let mut listener = match client.subscribe(&tx_char, false).await {
        Ok(l) => l,
        Err(_e) => {
            error!("[nus] subscribe failed");
            return;
        }
    };

    // Activate ergot interface
    stack.manage_profile(|im| {
        let _ = im.set_interface_state(
            (),
            InterfaceState::Active {
                net_id: 1,
                node_id: EDGE_NODE_ID,
            },
        );
    });
    transport::STATE_NOTIFY.wake_all();

    info!("[nus] active, running bidirectional loop");

    let mut processor = EdgeFrameProcessor::new();
    let consumer = transport::BLE_OUTQ.framed_consumer();

    // Bidirectional loop
    loop {
        let notification = listener.next();
        let tx_frame = consumer.wait_read();

        match select(notification, tx_frame).await {
            // Incoming notification from bridge
            Either::First(notif) => {
                let data = notif.as_ref();
                let changed = processor.process_frame(data, &stack, ());
                if changed {
                    transport::STATE_NOTIFY.wake_all();
                }
            }

            // Outgoing ergot frame → write to bridge RX characteristic
            Either::Second(grant) => {
                if let Err(_e) = client
                    .write_characteristic_without_response(&rx_char, grant.as_ref())
                    .await
                {
                    warn!("[nus] write error, disconnecting");
                    grant.release();
                    break;
                }
                grant.release();
            }
        }
    }
}

// ========== Info server ==========

// HardwareInfo + the pinned server future live in this frame (~1.5 KB);
// same situation as the other allows in this file.
#[allow(clippy::large_stack_frames)]
async fn info_server(stack: &'static transport::Stack) {
    use core::pin::pin;

    let device_info = HardwareInfo {
        hw: "ESP32-C6 Remote".try_into().unwrap_or_default(),
        sw: "oxifoc-remote-0.1.0".try_into().unwrap_or_default(),
        mcu: "ESP32-C6".try_into().unwrap_or_default(),
        uuid: heapless::String::new(),
        foc_freq_hz: 0,
        max_current_a: 0.0,
    };

    let server = stack
        .endpoints()
        .bounded_server::<HardwareInfoEndpoint, 2>(Some("hardware_info"));
    let server = pin!(server);
    let mut hdl = server.attach();

    info!("[info] server running");
    loop {
        let _ = hdl
            .serve(|_req: &()| {
                let info = device_info.clone();
                async move { info }
            })
            .await;
    }
}

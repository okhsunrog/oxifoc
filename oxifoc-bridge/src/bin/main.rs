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
use esp_hal::uart::{Config as UartConfig, Uart};

use bt_hci::controller::ExternalController;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use embassy_executor::Spawner;
use embassy_futures::join::join5;
use embassy_futures::select::{Either, select};
use embassy_time::Timer;

use ergot::NetStack;
use ergot::interface_manager::profiles::router::RouterFrameProcessor;
use ergot::interface_manager::utils::{cobs_stream, framed_stream};
use ergot::interface_manager::{FrameProcessor, InterfaceState, LivenessConfig, Profile};

use oxifoc_bridge::ble_nus::NusServer;
use oxifoc_bridge::transport::{self, BridgeSink};
use oxifoc_core::icd::{HardwareInfo, HardwareInfoEndpoint};

use defmt::{error, info, warn};

use panic_rtt_target as _;

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // ========== UART setup (upstream) ==========
    let uart_config = UartConfig::default().with_baudrate(transport::UART_BAUD);
    let uart = Uart::new(peripherals.UART1, uart_config)
        .expect("UART1 init failed")
        .with_rx(peripherals.GPIO20)
        .with_tx(peripherals.GPIO19)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();

    // ========== BLE setup ==========
    let bt_transport = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    let ble_stack = trouble_host::new(ble_controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = ble_stack.build();

    let nus_server = NusServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Oxifoc Bridge",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    // ========== Ergot stack setup ==========
    let rng = esp_hal::rng::Rng::new();
    let upstream_sink = BridgeSink::Uart(cobs_stream::Sink::new(
        transport::UART_OUTQ.stream_producer(),
        transport::UART_MTU,
    ));
    let router = ergot::interface_manager::profiles::router::Router::new_bridge(rng, upstream_sink);
    let stack: &'static transport::Stack = {
        static STACK_CELL: static_cell::StaticCell<transport::Stack> =
            static_cell::StaticCell::new();
        STACK_CELL.init(NetStack::new_with_profile(router))
    };

    // Get upstream interface ident (upstream is always ident 0 in bridge mode)
    let upstream_ident: u8 = 0;
    let upstream_net_id = stack
        .manage_profile(|router| {
            router
                .interface_state(upstream_ident)
                .and_then(|s| match s {
                    InterfaceState::Active { net_id, .. } => Some(net_id),
                    _ => None,
                })
        })
        .unwrap_or(1);

    // Register BLE downstream interface
    let ble_ident = stack.manage_profile(|router| {
        router
            .register_interface(BridgeSink::Ble(framed_stream::Sink::new(
                transport::BLE_OUTQ.framed_producer(),
                transport::BLE_MTU,
            )))
            .expect("BLE interface registration failed")
    });
    stack.manage_profile(|router| {
        let _ = router.set_interface_state(ble_ident, InterfaceState::Inactive);
    });

    info!(
        "Ergot bridge initialized: upstream ident={} net_id={}, ble ident={}",
        upstream_ident, upstream_net_id, ble_ident
    );

    // ========== UART RX worker ==========
    let mut uart_rx_worker = transport::UartRxWorker::new(
        stack,
        uart_rx,
        RouterFrameProcessor::new(upstream_net_id),
        upstream_ident,
    )
    .with_liveness(LivenessConfig {
        timeout_ms: transport::LIVENESS_TIMEOUT_MS,
    })
    .with_state_notify(&transport::STATE_NOTIFY);

    // Spawn UART TX worker
    spawner.must_spawn(run_uart_tx(uart_tx, stack, upstream_ident));

    // ========== Run all tasks ==========
    static RECV_BUF: static_cell::StaticCell<[u8; 512]> = static_cell::StaticCell::new();
    static SCRATCH_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();

    let _ = join5(
        // BLE runner (HCI background task)
        ble_runner(runner),
        // HardwareInfo endpoint server
        info_server(stack),
        // Ergot device discovery handler
        stack.services().device_info_handler::<2>(&ergot::well_known::DeviceInfo {
            name: Some("Oxifoc Bridge".try_into().unwrap_or_default()),
            description: Some("BLE+UART bridge".try_into().unwrap_or_default()),
            unique_id: 0,
        }),
        // UART RX worker
        async {
            let recv_buf = RECV_BUF.init_with(|| [0u8; 512]);
            let scratch_buf = SCRATCH_BUF.init_with(|| [0u8; 64]);
            let _ = uart_rx_worker
                .run(InterfaceState::Inactive, recv_buf, scratch_buf)
                .await;
            error!("[uart] rx worker exited");
        },
        // BLE advertising + connection loop
        async {
            loop {
                match advertise(&mut peripheral, &nus_server).await {
                    Ok(conn) => {
                        info!("[ble] connection established");

                        // Request 2M PHY for higher throughput
                        if let Err(_e) = conn
                            .raw()
                            .set_phy(&ble_stack, trouble_host::prelude::PhyKind::Le2M)
                            .await
                        {
                            warn!("[ble] failed to set 2M PHY, continuing with 1M");
                        } else {
                            info!("[ble] 2M PHY requested");
                        }

                        let net_id = stack
                            .manage_profile(|router| {
                                router.interface_state(ble_ident).and_then(|s| match s {
                                    InterfaceState::Active { net_id, .. } => Some(net_id),
                                    _ => None,
                                })
                            })
                            .unwrap_or(0);

                        nus_connection_task(stack, &nus_server, &conn, ble_ident, net_id).await;

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
        },
    )
    .await;

    unreachable!()
}

// ========== UART TX worker ==========

#[embassy_executor::task]
async fn run_uart_tx(
    mut tx: esp_hal::uart::UartTx<'static, esp_hal::Async>,
    stack: &'static transport::Stack,
    ident: u8,
) {
    let consumer = transport::UART_OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let len = grant.len();

        let is_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(ident),
                Some(InterfaceState::Active { .. })
            )
        });

        if is_active {
            let mut remaining = &grant[..];
            while !remaining.is_empty() {
                match embassy_time::with_timeout(
                    embassy_time::Duration::from_millis(500),
                    tx.write_async(remaining),
                )
                .await
                {
                    Ok(Ok(n)) => remaining = &remaining[n..],
                    _ => break,
                }
            }
        }
        grant.release(len);
    }
}

// ========== BLE runner ==========

async fn ble_runner<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(_e) = runner.run().await {
            error!("[ble] runner error, restarting");
        }
    }
}

// ========== BLE advertise ==========

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

async fn nus_connection_task<P: PacketPool>(
    stack: &'static transport::Stack,
    server: &NusServer<'_>,
    conn: &GattConnection<'_, '_, P>,
    ble_ident: u8,
    net_id: u16,
) {
    let rx_handle = server.nus.rx.handle;
    let tx_char = &server.nus.tx;
    let consumer = transport::BLE_OUTQ.framed_consumer();
    let mut processor = RouterFrameProcessor::new(net_id);

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
                    if let GattEvent::Write(ref write_event) = event {
                        if write_event.handle() == rx_handle {
                            let data = write_event.data();
                            let changed = processor.process_frame(data, &stack, ble_ident);
                            if changed {
                                transport::STATE_NOTIFY.wake_all();
                            }
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
                let payload: heapless::Vec<u8, { oxifoc_bridge::ble_nus::NUS_MAX_PAYLOAD }> =
                    heapless::Vec::from_slice(&grant).unwrap_or_default();
                if let Err(_e) = tx_char.notify(conn, &payload).await {
                    warn!("[ble] notify error, disconnecting");
                    grant.release();
                    break;
                }
                grant.release();
            }
        }
    }
}

// ========== Info server ==========

async fn info_server(stack: &'static transport::Stack) {
    use core::pin::pin;

    let device_info = HardwareInfo {
        hw: "ESP32-C6 Bridge".try_into().unwrap_or_default(),
        sw: "oxifoc-bridge-0.1.0".try_into().unwrap_or_default(),
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

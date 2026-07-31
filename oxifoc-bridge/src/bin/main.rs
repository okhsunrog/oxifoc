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
use embassy_futures::join::{join, join5};
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use ergot::NetStack;
use ergot::interface_manager::profiles::direct_edge::{
    CENTRAL_NODE_ID, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::profiles::router::{RouterFrameProcessor, UPSTREAM_IDENT};
use ergot::interface_manager::utils::{cobs_stream, framed_stream};
use ergot::interface_manager::{FrameProcessor, InterfaceState, LivenessConfig, Profile};
use ergot::net_stack::services::{
    SeedLease, bridge_seed_assign, bridge_seed_refresh, release_seed_lease,
};
use ergot::well_known::ErgotPingEndpoint;

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
    let bt_transport =
        BleConnector::new(peripherals.BT, Default::default()).expect("BLE connector init");
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
    .expect("NUS GATT server init");

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

    // Register the BLE downstream as PENDING: it gets no self-allocated
    // net_id (which would collide with the root-owned numbering) and stays
    // Down until a seed lease from the root assigns it a routable net at
    // BLE-connect time.
    let ble_ident = stack.manage_profile(|router| {
        router
            .register_interface_pending(BridgeSink::Ble(framed_stream::Sink::new(
                transport::BLE_OUTQ.framed_producer(),
                transport::BLE_MTU,
            )))
            .expect("BLE interface registration failed")
    });

    info!(
        "Ergot bridge initialized: upstream ident={}, ble ident={} (pending)",
        UPSTREAM_IDENT, ble_ident
    );

    // ========== UART RX worker (upstream) ==========
    // The upstream is an edge of the root's segment: EdgeFrameProcessor
    // discovers its net_id from inbound frames (guarded against transit),
    // bound to the reserved UPSTREAM_IDENT.
    let mut uart_rx_worker =
        transport::UartRxWorker::new(stack, uart_rx, EdgeFrameProcessor::new(), UPSTREAM_IDENT)
            .with_liveness(LivenessConfig {
                timeout_ms: transport::LIVENESS_TIMEOUT_MS,
            })
            .with_state_notify(&transport::STATE_NOTIFY);

    // Spawn UART TX worker
    spawner.must_spawn(run_uart_tx(uart_tx, stack));

    // ========== Run all tasks ==========
    // Must hold one max-size incoming UART frame (see transport::UART_MTU).
    static RECV_BUF: static_cell::StaticCell<[u8; transport::UART_MTU as usize]> =
        static_cell::StaticCell::new();
    static SCRATCH_BUF: static_cell::StaticCell<[u8; 64]> = static_cell::StaticCell::new();

    let _ = join(
        join5(
            // BLE runner (HCI background task)
            ble_runner(runner),
            // HardwareInfo endpoint server
            info_server(stack),
            // Ergot device discovery handler
            stack
                .services()
                .device_info_handler::<2>(&ergot::well_known::DeviceInfo {
                    name: Some("Oxifoc Bridge".try_into().unwrap_or_default()),
                    description: Some("BLE+UART bridge".try_into().unwrap_or_default()),
                    unique_id: 0,
                }),
            // UART RX worker. Read errors (framing/overrun — routine on a
            // cable near a motor drive) are recoverable: log and re-enter.
            // The initial Active{net 0} state is link-local addressing, so
            // the bridge can initiate upstream contact before discovery.
            async {
                let recv_buf = RECV_BUF.init_with(|| [0u8; transport::UART_MTU as usize]);
                let scratch_buf = SCRATCH_BUF.init_with(|| [0u8; 64]);
                loop {
                    let res = uart_rx_worker
                        .run(
                            InterfaceState::Active {
                                net_id: 0,
                                node_id: EDGE_NODE_ID,
                            },
                            recv_buf,
                            scratch_buf,
                        )
                        .await;
                    error!("[uart] rx worker error ({:?}), restarting", res.err());
                    Timer::after_millis(100).await;
                }
            },
            // BLE advertising + connection loop
            async {
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

                            nus_connection_task(stack, &nus_server, &conn, ble_ident, &mut lease)
                                .await;

                            stack.manage_profile(|router| {
                                let _ =
                                    router.set_interface_state(ble_ident, InterfaceState::Inactive);
                            });
                            // Best-effort: hand the net back instead of letting
                            // the lease age out on the root.
                            let _ = with_timeout(
                                Duration::from_secs(1),
                                release_seed_lease(&stack, &lease),
                            )
                            .await;
                            info!("[ble] disconnected, returning to advertising");
                        }
                        Err(_e) => {
                            warn!("[ble] advertise error, retrying");
                            Timer::after_secs(1).await;
                        }
                    }
                }
            },
        ),
        // Upstream bootstrap/keepalive
        upstream_link_task(stack),
    )
    .await;

    unreachable!()
}

// ========== Upstream link maintenance ==========

/// Whether the upstream has discovered its real (non-link-local) net_id.
fn upstream_discovered(stack: &'static transport::Stack) -> bool {
    stack.manage_profile(|router| {
        matches!(
            router.interface_state(UPSTREAM_IDENT),
            Some(InterfaceState::Active { net_id, .. }) if net_id != 0
        )
    })
}

/// Bootstrap and keep alive the upstream link.
///
/// The edge-style upstream learns its net_id from the first frame addressed
/// to the bridge, so somebody has to provoke that frame: a link-local ping to
/// the root does it. The steady-state ping doubles as a keepalive that stops
/// the liveness window from marking a quiet-but-healthy line Inactive. After
/// a genuine quiet period the interface IS Inactive and TX is gated off —
/// re-arm link-local addressing first so the ping can leave at all.
async fn upstream_link_task(stack: &'static transport::Stack) {
    let mut seq: u32 = 0;
    loop {
        if !upstream_discovered(stack) {
            stack.manage_profile(|router| {
                if matches!(
                    router.interface_state(UPSTREAM_IDENT),
                    Some(InterfaceState::Inactive)
                ) {
                    let _ = router.set_interface_state(
                        UPSTREAM_IDENT,
                        InterfaceState::Active {
                            net_id: 0,
                            node_id: EDGE_NODE_ID,
                        },
                    );
                }
            });
        }
        seq = seq.wrapping_add(1);
        let _ = with_timeout(
            Duration::from_millis(750),
            stack.endpoints().request::<ErgotPingEndpoint>(
                ergot::Address {
                    network_id: 0,
                    node_id: CENTRAL_NODE_ID,
                    port_id: 0,
                },
                &seq,
                Some("ping"),
            ),
        )
        .await;
        Timer::after_secs(2).await;
    }
}

/// Lease a routable net_id for the BLE segment from the root, waiting for
/// upstream discovery first. Bounded: a central that connects while the
/// upstream is dead gets dropped rather than parked forever.
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

// ========== UART TX worker ==========

#[embassy_executor::task]
async fn run_uart_tx(
    mut tx: esp_hal::uart::UartTx<'static, esp_hal::Async>,
    stack: &'static transport::Stack,
) {
    let consumer = transport::UART_OUTQ.stream_consumer();
    loop {
        let grant = consumer.wait_read().await;
        let len = grant.len();

        let is_active = stack.manage_profile(|im| {
            matches!(
                im.interface_state(UPSTREAM_IDENT),
                Some(InterfaceState::Active { .. })
            )
        });

        if !is_active {
            // Interface down: drop the whole grant (frame-aligned, so the
            // stream stays COBS-consistent).
            grant.release(len);
            continue;
        }

        let mut written = 0usize;
        while written < len {
            match with_timeout(
                Duration::from_millis(500),
                tx.write_async(&grant[written..]),
            )
            .await
            {
                Ok(Ok(n)) => written += n,
                _ => break,
            }
        }
        if written < len {
            // Release only what actually left: dropping unwritten bytes
            // mid-COBS-frame would desync the receiver past the next
            // delimiter. The remainder is retried on the next grant.
            warn!("[uart] tx stalled at {}/{} bytes", written, len);
            grant.release(written);
        } else {
            grant.release(len);
        }
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
                        let data = write_event.data();
                        let changed = processor.process_frame(data, &stack, ble_ident);
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
                let payload: heapless::Vec<u8, { oxifoc_bridge::ble_nus::NUS_MAX_PAYLOAD }> =
                    heapless::Vec::from_slice(&grant).unwrap_or_default();
                if let Err(_e) = tx_char.notify(conn, &payload).await {
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

// ========== Info server ==========

#[allow(clippy::large_stack_frames)]
async fn info_server(stack: &'static transport::Stack) {
    use core::pin::pin;

    let device_info = HardwareInfo {
        proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
        hw: "ESP32-C6 Bridge".try_into().unwrap_or_default(),
        sw: "oxifoc-bridge-0.1.0".try_into().unwrap_or_default(),
        mcu: "ESP32-C6".try_into().unwrap_or_default(),
        uuid: heapless::String::new(),
        foc_freq_hz: 0,
        max_current_a: 0.0,
        // No motor/current sensing on the bridge — report default calibration.
        calib: oxifoc_core::types::BoardCalib::default(),
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

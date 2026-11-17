#![deny(
    clippy::mem_forget,
    reason = "DMA and radio drivers borrow buffers for the duration of a transfer"
)]
#![deny(clippy::large_stack_frames)]

use embassy_nrf::{
    bind_interrupts,
    interrupt::{Priority, typelevel::Interrupt},
    peripherals, rng, saadc, twim,
};
use nrf_mpsl::{self as mpsl, MultiprotocolServiceLayer};
use trouble_host::prelude::*;

use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join4;
use embassy_futures::select::{Either, select, select3};
use embassy_time::{Duration, Instant, Timer, with_deadline};

use ergot::NetStack;
use ergot::interface_manager::profiles::direct_edge::{DirectEdge, EdgeFrameProcessor};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};

use oxifoc_core::icd::{HardwareInfo, HardwareInfoEndpoint};
use oxifoc_remote::transport;

use defmt::{error, info, warn};
use static_cell::StaticCell;

#[path = "analog.rs"]
mod analog;
#[path = "battery.rs"]
mod battery;
#[path = "csprng.rs"]
mod csprng;
#[path = "display.rs"]
mod display;
#[path = "storage.rs"]
mod storage;

use embassy_crypto_rustcrypto as _;

const CONNECTIONS_MAX: usize = 1;
/// The remote bonds only with its bridge.
const BONDS_MAX: usize = 1;
/// Resuming encryption with the stored bond.
const RESUME_TIMEOUT: Duration = Duration::from_secs(3);
/// Pairing waits for the user to type the code on a trusted device.
const PAIRING_TIMEOUT: Duration = Duration::from_secs(70);
const LFCLK_ACCURACY_PPM: u16 = 50;
/// nRF52840 maximum. The link carries the throttle, so range wins over current.
const TX_POWER_DBM: i8 = 8;
/// A lost session reconnects immediately. About a third of attempts against the
/// C5 fail to establish (0x3E) and the next one usually succeeds, so the first
/// failures retry at once and only a longer run of failures backs off.
const IMMEDIATE_RETRIES: u32 = 3;
const RETRY_BACKOFF_MIN: Duration = Duration::from_millis(50);
const RETRY_BACKOFF_MAX: Duration = Duration::from_millis(1000);
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + ATT + safety margin

/// Address of the bridge peripheral (must match oxifoc-bridge)
const BRIDGE_ADDR_BYTES: [u8; 6] = [0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff];

bind_interrupts!(struct Irqs {
    CLOCK_POWER => mpsl::ClockInterruptHandler;
    EGU0_SWI0 => mpsl::LowPrioInterruptHandler;
    RADIO => mpsl::HighPrioInterruptHandler;
    TIMER0 => mpsl::HighPrioInterruptHandler;
    RTC0 => mpsl::HighPrioInterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    SAADC => saadc::InterruptHandler;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

pub type BootTask = fn(Spawner, &'static transport::Stack, u64);

pub fn start(boot_task: Option<BootTask>) -> ! {
    rtt_target::rtt_init_defmt!();
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    EXECUTOR
        .init_with(Executor::new)
        .run(|spawner| spawner.spawn(run(spawner, boot_task).expect("allocate remote task")));
}

#[allow(
    clippy::large_stack_frames,
    reason = "Trouble's host resources and GATT futures live in static Embassy task storage"
)]
#[embassy_executor::task]
async fn run(spawner: Spawner, boot_task: Option<BootTask>) -> ! {
    let mut config = embassy_nrf::config::Config::default();
    config.time_interrupt_priority = Priority::P6;
    let p = embassy_nrf::init(config);
    info!("OxiFOC Remote on nRF52840");

    // MPSL owns priority 0 for its radio, timer and RTC interrupts.
    embassy_nrf::interrupt::typelevel::TWISPI0::set_priority(Priority::P6);
    embassy_nrf::interrupt::typelevel::RNG::set_priority(Priority::P6);

    // Voyager board wiring, including the nice!nano switched 3.3 V output.
    let display_power = embassy_nrf::gpio::Output::new(
        p.P0_13,
        embassy_nrf::gpio::Level::High,
        embassy_nrf::gpio::OutputDrive::Standard,
    );
    static TWIM_TX: StaticCell<[u8; 16]> = StaticCell::new();
    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    let i2c = twim::Twim::new(
        p.TWISPI0,
        Irqs,
        p.P0_17,
        p.P0_20,
        twim_config,
        TWIM_TX.init([0; 16]),
    );
    spawner.spawn(display::run(i2c, display_power).expect("allocate OLED task"));

    if boot_task.is_none() {
        embassy_nrf::interrupt::typelevel::SAADC::set_priority(Priority::P6);
        let throttle = saadc::ChannelConfig::single_ended(p.P0_02);
        let battery = saadc::ChannelConfig::single_ended(saadc::VddhDiv5Input);
        let adc = saadc::Saadc::new(p.SAADC, Irqs, saadc::Config::default(), [throttle, battery]);
        spawner.spawn(analog::sample(adc).expect("allocate ADC task"));
    }

    // ========== BLE setup ==========
    static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
    // One timeslot session for the flash driver (settings storage).
    static MPSL_SESSIONS: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    let mpsl = MPSL.init(
        MultiprotocolServiceLayer::with_timeslots(
            mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31),
            Irqs,
            // The nice!nano has a 32.768 kHz crystal. 50 ppm is a conservative
            // bound for a generic watch crystal; the peer widens its windows by it.
            mpsl::raw::mpsl_clock_lfclk_cfg_t {
                source: mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
                rc_ctiv: 0,
                rc_temp_ctiv: 0,
                accuracy_ppm: LFCLK_ACCURACY_PPM,
                skip_wait_lfclk_started: false,
            },
            MPSL_SESSIONS.init_with(mpsl::SessionMem::new),
        )
        .expect("MPSL init"),
    );
    spawner.spawn(mpsl_task(mpsl).expect("allocate MPSL task"));
    let settings =
        embassy_sync::mutex::Mutex::<embassy_sync::blocking_mutex::raw::NoopRawMutex, _>::new(
            storage::Settings::new(mpsl::Flash::take(mpsl, p.NVMC)),
        );

    static RNG: StaticCell<rng::Rng<'static, embassy_nrf::mode::Async>> = StaticCell::new();
    let rng = RNG.init(rng::Rng::new(p.RNG, Irqs));
    let drive_session = boot_task.map(|_| rng.blocking_next_u64());
    // Seed the pairing CSPRNG before the SoftDevice Controller takes the RNG.
    let mut seed = [0u8; 32];
    rng.blocking_fill_bytes(&mut seed);
    csprng::seed(seed);
    if !csprng::cmac::self_test() {
        error!("[crypto] AES-CMAC self-test failed; pairing will fail");
    }
    static SDC_MEM: StaticCell<nrf_sdc::Mem<4904>> = StaticCell::new();
    // Controller ACL fragments are at most 251 bytes. Trouble reassembles
    // them into its 512-byte L2CAP packets used by the existing NUS transport.
    // No 2M PHY: the throttle needs little throughput, and 1M has the better
    // receiver sensitivity. Without the feature the link stays on 1M.
    let ble_controller = nrf_sdc::Builder::new()
        .expect("SDC init")
        .support_central()
        .support_dle_central()
        // Resolvable private addresses: a fresh one per boot, resolved by the
        // bonded bridge with our IRK (docs/notes/ble-pairing.md).
        .support_le_privacy()
        .default_tx_power(TX_POWER_DBM)
        .expect("SDC TX power")
        .central_count(CONNECTIONS_MAX as u8)
        .expect("SDC connection count")
        .buffer_cfg(251, 251, 3, 3)
        .expect("SDC ACL buffers")
        .build(
            nrf_sdc::Peripherals::new(
                p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
                p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
            ),
            rng,
            mpsl,
            SDC_MEM.init_with(nrf_sdc::Mem::new),
        )
        .expect("enable SDC");
    info!("BLE central ready");
    let mut resources: HostResources<
        DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        1,
        BONDS_MAX,
    > = HostResources::new();

    let identity = own_identity(&settings).await;
    let address = Address::random(identity.address);
    info!("[ble] identity address {:?}", address);
    // The remote has a screen and no keys: it shows the passkey and the user
    // types it on a trusted device, which hands it to the bridge.
    let ble_stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .enable_privacy(IdentityResolvingKey::new(identity.irk).expect("nonzero IRK"))
        .set_io_capabilities(IoCapabilities::DisplayOnly)
        .build();
    if let Some(bond) = settings.lock().await.bridge_bond().await {
        info!("[ble] bonded with the bridge");
        if ble_stack
            .add_bond_information(bridge_bond_info(&bond))
            .is_err()
        {
            warn!("[ble] bond table full");
        }
    }
    let mut central = ble_stack.central();
    let mut runner = ble_stack.runner();

    let bridge_addr = Address::random(BRIDGE_ADDR_BYTES);

    // ========== Ergot stack setup (DirectEdge target) ==========
    let stack: &'static transport::Stack = {
        static STACK_CELL: StaticCell<transport::Stack> = StaticCell::new();
        STACK_CELL.init(NetStack::new_with_profile(DirectEdge::new_target(
            framed_stream::Sink::new(transport::BLE_OUTQ.framed_producer(), transport::BLE_MTU),
        )))
    };

    info!("Ergot stack initialized (DirectEdge target)");
    if let Some((spawn, session)) = boot_task.zip(drive_session) {
        spawn(spawner, stack, session);
    }

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
            let mut backoff = Duration::from_ticks(0);
            let mut failures = 0u32;
            // Handles from the last discovery survive power cycles, so even the
            // first connection after power-on skips characteristic discovery.
            let gatt_cache = core::cell::Cell::new(settings.lock().await.bridge_handles().await);
            loop {
                if backoff.as_ticks() > 0 {
                    Timer::after(backoff).await;
                }
                display::BRIDGE_READY.signal(false);
                info!("[ble] connecting to bridge...");

                let connect_config = ConnectConfig {
                    // A short interval keeps an Ergot round trip inside the drive deadman
                    // and avoided the C5 missing the first connection after a link loss
                    // (docs/notes/ble-link-reliability.md). The 1 s supervision timeout
                    // frees the bridge's single slot quickly after a remote reset.
                    connect_params: RequestedConnParams {
                        min_connection_interval: Duration::from_millis(15),
                        max_connection_interval: Duration::from_millis(20),
                        supervision_timeout: Duration::from_millis(1000),
                        ..Default::default()
                    },
                    scan_config: ScanConfig {
                        filter_accept_list: &[bridge_addr],
                        ..Default::default()
                    },
                };

                match central.connect(&connect_config).await {
                    Ok(conn) => {
                        info!("[ble] connected to bridge");
                        if !secure_link(&ble_stack, &conn, &settings).await {
                            warn!("[ble] link not secured, retrying");
                            failures += 1;
                            backoff = retry_backoff(failures, backoff);
                            continue;
                        }

                        // Create GATT client
                        let client =
                            match GattClient::<_, DefaultPacketPool, 10>::new(&ble_stack, &conn)
                                .await
                            {
                                Ok(c) => c,
                                Err(_e) => {
                                    warn!("[ble] link not established, retrying");
                                    failures += 1;
                                    backoff = retry_backoff(failures, backoff);
                                    continue;
                                }
                            };

                        // Run the GATT client task against the NUS session.
                        // select, not join: on disconnect `client.task()`
                        // returns but `nus_session` blocks forever (the
                        // notification listener never signals disconnect).
                        let active = core::cell::Cell::new(false);
                        let _ = select3(
                            client.task(),
                            nus_session(stack, &client, &gatt_cache, &settings, &active),
                            connection_events(&ble_stack, &conn),
                        )
                        .await;
                        // A session that came up proves the link; retry at once.
                        if active.get() {
                            failures = 0;
                            backoff = Duration::from_ticks(0);
                        } else {
                            failures += 1;
                            backoff = retry_backoff(failures, backoff);
                        }
                        display::BRIDGE_READY.signal(false);

                        // Disconnected
                        stack.manage_profile(|im| {
                            let _ = im.set_interface_state((), InterfaceState::Inactive);
                        });
                        // Never replay frames enqueued for the old connection.
                        let consumer = transport::BLE_OUTQ.framed_consumer();
                        while let Ok(frame) = consumer.read() {
                            frame.release();
                        }
                        info!("[ble] disconnected, will reconnect");
                    }
                    Err(_e) => {
                        warn!("[ble] connect failed, retrying");
                        failures += 1;
                        backoff = retry_backoff(failures, backoff);
                    }
                }
            }
        },
    )
    .await;

    unreachable!()
}

/// Our identity address and IRK, generated on first boot and kept in flash.
async fn own_identity(settings: &Settings<'_>) -> storage::OwnIdentity {
    let mut settings = settings.lock().await;
    if let Some(identity) = settings.own_identity().await {
        return identity;
    }
    let mut address = [0u8; 6];
    csprng::fill(&mut address);
    address[5] |= 0xc0; // static random address
    let mut irk = [0u8; 16];
    csprng::fill(&mut irk);
    let identity = storage::OwnIdentity {
        address,
        irk: u128::from_le_bytes(irk) | 1, // nonzero
    };
    settings.set_own_identity(&identity).await;
    info!("[ble] new identity generated");
    identity
}

fn bridge_bond_info(bond: &storage::BridgeBond) -> BondInformation {
    BondInformation::new(
        Identity {
            addr: Address {
                kind: if bond.address_random {
                    AddrKind::RANDOM
                } else {
                    AddrKind::PUBLIC
                },
                addr: BdAddr::new(bond.address),
            },
            irk: bond.irk.and_then(IdentityResolvingKey::new),
        },
        LongTermKey(bond.ltk),
        SecurityLevel::EncryptedAuthenticated,
        true,
    )
}

/// Encrypt the link with the stored bond, or pair when there is none.
///
/// Pairing starts on its own when the remote has no bond; the bridge accepts
/// it only inside a window opened by a trusted channel. The remote shows the
/// passkey on its screen.
#[allow(
    clippy::large_stack_frames,
    reason = "the connection event future lives in embassy task storage"
)]
async fn secure_link<C: Controller>(
    ble_stack: &Stack<'_, C, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
    settings: &Settings<'_>,
) -> bool {
    let mut bonded = settings.lock().await.bridge_bond().await.is_some();
    let _ = conn.set_bondable(true);
    if conn.request_security().is_err() {
        warn!("[ble] security request refused");
        return false;
    }
    let mut pairing = false;
    let mut deadline = Instant::now()
        + if bonded {
            RESUME_TIMEOUT
        } else {
            PAIRING_TIMEOUT
        };
    let result = loop {
        let Ok(event) = with_deadline(deadline, conn.next()).await else {
            warn!("[ble] security timed out");
            break false;
        };
        match event {
            ConnectionEvent::Encrypted {
                security_level,
                bond: Some(_),
            } if bonded && !pairing && security_level.authenticated() => {
                info!("[ble] encrypted with the stored bond");
                break true;
            }
            ConnectionEvent::PassKeyDisplay(key) => {
                pairing = true;
                info!("[ble] pairing code {=u32:06}", key.value());
                display::PASSKEY.signal(Some(key.value()));
            }
            // Resuming with the stored bond: trouble reports the encryption
            // change as a completed pairing without new bond information.
            ConnectionEvent::PairingComplete {
                security_level,
                bond: None,
            } if bonded && !pairing && security_level.authenticated() => {
                info!("[ble] encrypted with the stored bond");
                break true;
            }
            ConnectionEvent::PairingComplete {
                security_level,
                bond: Some(bond),
            } if security_level.authenticated() => {
                let mut address = [0u8; 6];
                address.copy_from_slice(bond.identity.addr.addr.raw());
                let stored = storage::BridgeBond {
                    address,
                    address_random: bond.identity.addr.kind == AddrKind::RANDOM,
                    irk: bond.identity.irk.map(|irk| irk.0.get()),
                    ltk: bond.ltk.0,
                };
                settings.lock().await.set_bridge_bond(&stored).await;
                info!("[ble] bonded with the bridge");
                break true;
            }
            ConnectionEvent::PairingComplete { .. } => {
                warn!("[ble] pairing without MITM protection or bond, rejecting");
                break false;
            }
            ConnectionEvent::PairingFailed(_e) => {
                warn!("[ble] pairing failed");
                break false;
            }
            ConnectionEvent::BondLost => {
                warn!("[ble] the bridge lost our bond");
                forget_bridge(ble_stack, settings).await;
                // trouble pairs afresh on this link; give the user time.
                bonded = false;
                deadline = Instant::now() + PAIRING_TIMEOUT;
            }
            ConnectionEvent::Disconnected { reason } => {
                warn!("[ble] disconnected during security: {:?}", reason);
                if bonded
                    && (reason == bt_hci::param::Status::PIN_OR_KEY_MISSING
                        || reason == bt_hci::param::Status::AUTHENTICATION_FAILURE)
                {
                    // The bridge no longer knows us: pair again next time.
                    forget_bridge(ble_stack, settings).await;
                }
                break false;
            }
            _ => {}
        }
    };
    display::PASSKEY.signal(None);
    result
}

async fn forget_bridge<C: Controller>(
    ble_stack: &Stack<'_, C, DefaultPacketPool>,
    settings: &Settings<'_>,
) {
    let mut settings = settings.lock().await;
    if let Some(bond) = settings.bridge_bond().await {
        let _ = ble_stack.remove_bond_information(bridge_bond_info(&bond).identity);
        settings.clear_bridge_bond().await;
        info!("[ble] bridge bond removed");
    }
}

fn retry_backoff(failures: u32, current: Duration) -> Duration {
    if failures <= IMMEDIATE_RETRIES {
        Duration::from_ticks(0)
    } else if current.as_ticks() == 0 {
        RETRY_BACKOFF_MIN
    } else {
        (current * 2).min(RETRY_BACKOFF_MAX)
    }
}

/// Log link-layer events and answer parameter requests until the link drops.
async fn connection_events<C: Controller>(
    stack: &Stack<'_, C, DefaultPacketPool>,
    conn: &Connection<'_, DefaultPacketPool>,
) {
    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("[ble] link lost: {:?}", reason);
                return;
            }
            ConnectionEvent::RequestConnectionParams(request) => {
                // Keep the remote's short interval; the bridge has no reason to slow it.
                warn!("[ble] peer requested connection parameters, rejecting");
                if request.reject(stack).await.is_err() {
                    warn!("[ble] parameter request reply failed");
                }
            }
            event => info!("[ble] {:?}", event),
        }
    }
}

/// Attribute handles of the bridge's OxiFOC service, kept across reconnects.
///
/// The bridge's attribute table is fixed per firmware build, so a reconnect
/// skips characteristic and descriptor discovery. The service range is
/// re-checked and the CCCD write is acknowledged; either failing drops the
/// cache and falls back to full discovery.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct GattHandles {
    service_start: u16,
    service_end: u16,
    rx: u16,
    tx: u16,
    tx_cccd: u16,
}

const CCCD_NOTIFY: [u8; 2] = [0x01, 0x00];

type Settings<'d> = embassy_sync::mutex::Mutex<
    embassy_sync::blocking_mutex::raw::NoopRawMutex,
    storage::Settings<'d>,
>;

/// Find the service range: one Find By Type Value exchange (plus the
/// terminating one).
async fn service_range<C: Controller>(
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
) -> Option<(u16, u16)> {
    let uuid = Uuid::from(oxifoc_core::ble::SERVICE_UUID);
    match client.services_by_uuid(&uuid).await {
        Ok(services) => services.first().map(|service| {
            let range = service.handle_range();
            (*range.start(), *range.end())
        }),
        Err(_e) => {
            error!("[nus] service discovery failed");
            None
        }
    }
}

/// Full discovery: service, both characteristics and the TX CCCD.
#[allow(
    clippy::large_stack_frames,
    reason = "discovery futures hold GATT response buffers; it lives in embassy task storage, not the call stack."
)]
async fn discover_handles<C: Controller>(
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
) -> Option<GattHandles> {
    let uuid = Uuid::from(oxifoc_core::ble::SERVICE_UUID);
    let services = match client.services_by_uuid(&uuid).await {
        Ok(s) => s,
        Err(_e) => {
            error!("[nus] service discovery failed");
            return None;
        }
    };
    let Some(service) = services.first() else {
        error!("[nus] OxiFOC service not found");
        return None;
    };
    let rx_uuid = Uuid::from(oxifoc_core::ble::RX_UUID);
    let tx_uuid = Uuid::from(oxifoc_core::ble::TX_UUID);
    let Ok(rx) = client
        .characteristic_by_uuid::<[u8]>(service, &rx_uuid)
        .await
    else {
        error!("[nus] RX characteristic not found");
        return None;
    };
    let Ok(tx) = client
        .characteristic_by_uuid::<[u8]>(service, &tx_uuid)
        .await
    else {
        error!("[nus] TX characteristic not found");
        return None;
    };
    let Some(tx_cccd) = tx.cccd_handle else {
        error!("[nus] TX characteristic has no CCCD");
        return None;
    };
    let range = service.handle_range();
    Some(GattHandles {
        service_start: *range.start(),
        service_end: *range.end(),
        rx: rx.handle,
        tx: tx.handle,
        tx_cccd,
    })
}

/// Resolve the handles, from the cache when it still matches the bridge.
async fn resolve_handles<C: Controller>(
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
    cache: &core::cell::Cell<Option<GattHandles>>,
    settings: &Settings<'_>,
) -> Option<GattHandles> {
    if let Some(cached) = cache.get() {
        let range = service_range(client).await;
        if range == Some((cached.service_start, cached.service_end))
            && client
                .write_handle(cached.tx_cccd, &CCCD_NOTIFY)
                .await
                .is_ok()
        {
            info!("[nus] using cached handles");
            return Some(cached);
        }
        warn!("[nus] cached handles stale, rediscovering");
        cache.set(None);
    }
    let handles = discover_handles(client).await?;
    if client
        .write_handle(handles.tx_cccd, &CCCD_NOTIFY)
        .await
        .is_err()
    {
        error!("[nus] subscribe failed");
        return None;
    }
    info!("[nus] service discovered: {}", handles);
    cache.set(Some(handles));
    settings.lock().await.set_bridge_handles(&handles).await;
    Some(handles)
}

/// NUS session: resolve handles, enable notifications, run bidirectional loop.
#[allow(
    clippy::large_stack_frames,
    reason = "the session future holds Notification<512> and packet buffers; it lives in embassy task storage, not the call stack."
)]
async fn nus_session<C: Controller>(
    stack: &'static transport::Stack,
    client: &GattClient<'_, C, DefaultPacketPool, 10>,
    cache: &core::cell::Cell<Option<GattHandles>>,
    settings: &Settings<'_>,
    active: &core::cell::Cell<bool>,
) {
    // Listen before enabling notifications so none is missed.
    let Ok(mut listener) = client.listen_all() else {
        error!("[nus] notification listener unavailable");
        return;
    };
    let Some(handles) = resolve_handles(client, cache, settings).await else {
        return;
    };

    // Activate the ergot interface with link-local addressing: the real
    // net_id is whatever the bridge's seed lease assigned to this BLE
    // segment, and the edge processor discovers it from the first inbound
    // frame. A guessed constant here would race the lease allocation.
    stack.manage_profile(|im| {
        let _ = im.set_interface_state((), InterfaceState::edge_link_local());
    });
    transport::STATE_NOTIFY.wake_all();

    info!("[nus] active, running bidirectional loop");
    active.set(true);
    display::BRIDGE_READY.signal(true);

    let mut processor = EdgeFrameProcessor::new();
    let consumer = transport::BLE_OUTQ.framed_consumer();

    loop {
        match select(listener.next(), consumer.wait_read()).await {
            // Incoming notification from the bridge's TX characteristic
            Either::First(notif) => {
                if notif.handle() != handles.tx {
                    continue;
                }
                if processor.process_frame(notif.as_ref(), &stack, ()) {
                    transport::STATE_NOTIFY.wake_all();
                }
            }
            // Outgoing ergot frame → write to the bridge's RX characteristic
            Either::Second(grant) => {
                let result = client
                    .write_handle_without_response(handles.rx, grant.as_ref())
                    .await;
                grant.release();
                if result.is_err() {
                    warn!("[nus] write error, disconnecting");
                    break;
                }
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
        bootstrap_magic: oxifoc_core::types::ICD_BOOTSTRAP_MAGIC,
        proto_version: oxifoc_core::types::ICD_PROTO_VERSION,
        capabilities: 0,
        reserved: [0; 8],
        hw: "nRF52840 Remote".try_into().unwrap_or_default(),
        sw: "oxifoc-remote-0.1.0".try_into().unwrap_or_default(),
        mcu: "nRF52840".try_into().unwrap_or_default(),
        uuid: heapless::String::new(),
        foc_freq_hz: 0,
        max_current_a: 0.0,
        // No motor/current sensing on the remote — report default calibration.
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

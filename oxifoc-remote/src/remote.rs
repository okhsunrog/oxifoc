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
use embassy_time::{Duration, Timer};

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
#[path = "display.rs"]
mod display;

const CONNECTIONS_MAX: usize = 1;
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
    let mpsl = MPSL.init(
        MultiprotocolServiceLayer::new(
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
        )
        .expect("MPSL init"),
    );
    spawner.spawn(mpsl_task(mpsl).expect("allocate MPSL task"));

    static RNG: StaticCell<rng::Rng<'static, embassy_nrf::mode::Async>> = StaticCell::new();
    let rng = RNG.init(rng::Rng::new(p.RNG, Irqs));
    let drive_session = boot_task.map(|_| rng.blocking_next_u64());
    static SDC_MEM: StaticCell<nrf_sdc::Mem<2904>> = StaticCell::new();
    // Controller ACL fragments are at most 251 bytes. Trouble reassembles
    // them into its 512-byte L2CAP packets used by the existing NUS transport.
    // No 2M PHY: the throttle needs little throughput, and 1M has the better
    // receiver sensitivity. Without the feature the link stays on 1M.
    let ble_controller = nrf_sdc::Builder::new()
        .expect("SDC init")
        .support_central()
        .support_dle_central()
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
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x2b, 0x06, 0xe5, 0xff]);
    let ble_stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .build();
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
                            nus_session(stack, &client, &conn, &active),
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
    active: &core::cell::Cell<bool>,
) {
    let nus_uuid = Uuid::from(oxifoc_core::ble::SERVICE_UUID);
    let rx_uuid = Uuid::from(oxifoc_core::ble::RX_UUID);
    let tx_uuid = Uuid::from(oxifoc_core::ble::TX_UUID);

    // Discover the OxiFOC transport service
    let services = match client.services_by_uuid(&nus_uuid).await {
        Ok(s) => s,
        Err(_e) => {
            error!("[nus] service discovery failed");
            return;
        }
    };
    let Some(service) = services.first() else {
        error!("[nus] OxiFOC service not found");
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

#![deny(
    clippy::mem_forget,
    reason = "DMA and radio drivers borrow buffers for the duration of a transfer"
)]
#![deny(clippy::large_stack_frames)]

use embassy_nrf::{
    bind_interrupts,
    interrupt::{Priority, typelevel::Interrupt},
    peripherals, rng, twim,
};
use nrf_mpsl::{self as mpsl, MultiprotocolServiceLayer};
use trouble_host::prelude::*;

use embassy_executor::{Executor, Spawner};
use embassy_futures::join::join4;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

use ergot::NetStack;
use ergot::interface_manager::profiles::direct_edge::{DirectEdge, EdgeFrameProcessor};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};

use oxifoc_core::icd::{HardwareInfo, HardwareInfoEndpoint};
use oxifoc_remote::transport;

use defmt::{error, info, warn};
use static_cell::StaticCell;

#[path = "display.rs"]
mod display;

const CONNECTIONS_MAX: usize = 1;
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

    // ========== BLE setup ==========
    static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
    let mpsl = MPSL.init(
        MultiprotocolServiceLayer::new(
            mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31),
            Irqs,
            mpsl::raw::mpsl_clock_lfclk_cfg_t {
                source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
                rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
                rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
                accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
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
    let ble_controller = nrf_sdc::Builder::new()
        .expect("SDC init")
        .support_central()
        .support_le_2m_phy()
        .support_phy_update_central()
        .support_dle_central()
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
            loop {
                display::BRIDGE_READY.signal(false);
                info!("[ble] connecting to bridge...");

                let connect_config = ConnectConfig {
                    connect_params: if boot_task.is_some() {
                        // Two BLE events for an Ergot round trip must fit in the drive deadman.
                        RequestedConnParams {
                            min_connection_interval: Duration::from_millis(15),
                            max_connection_interval: Duration::from_millis(20),
                            ..Default::default()
                        }
                    } else {
                        Default::default()
                    },
                    scan_config: ScanConfig {
                        filter_accept_list: &[bridge_addr],
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

                        // Run the GATT client task against the NUS session.
                        // select, not join: on disconnect `client.task()`
                        // returns but `nus_session` blocks forever (the
                        // notification listener never signals disconnect).
                        let _ = select(client.task(), async {
                            nus_session(stack, &client, &conn).await;
                        })
                        .await;
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

    // Activate the ergot interface with link-local addressing: the real
    // net_id is whatever the bridge's seed lease assigned to this BLE
    // segment, and the edge processor discovers it from the first inbound
    // frame. A guessed constant here would race the lease allocation.
    stack.manage_profile(|im| {
        let _ = im.set_interface_state((), InterfaceState::edge_link_local());
    });
    transport::STATE_NOTIFY.wake_all();

    info!("[nus] active, running bidirectional loop");
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

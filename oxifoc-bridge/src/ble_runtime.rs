//! BLE peripheral: two link slots, the pairing gate and pairing management.
//!
//! Only links encrypted with a stored, authenticated bond reach Ergot. Other
//! links may only pair, and only inside a window opened by a trusted channel.
//! See docs/notes/ble-pairing.md.

use core::future::pending;
use core::pin::pin;
use core::sync::atomic::Ordering;

use crate::{
    ble_nus::NusServer,
    bonds::{BondStore, StoredBond},
    flash::Flash,
    pairing::{Input, NO_BOND, PairingState, Stage},
    transport,
};
use bt_hci::{
    cmd::le::LeReadLocalSupportedFeatures,
    controller::{ControllerCmdSync, ExternalController},
};
use embassy_futures::{
    join::join5,
    select::{Either, Either4, select, select4},
};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Timer};
use ergot::interface_manager::{
    FrameProcessor, InterfaceState, Profile, profiles::router::RouterFrameProcessor,
};
use ergot::socket::HeaderMessage;
use esp_radio::ble::controller::BleConnector;
use log::{debug, error, info, warn};
use oxifoc_core::icd::{PairingEndpoint, PairingEventTopic};
use oxifoc_core::types::pairing::{
    BondRole, MAX_BONDS, PairingError, PairingEvent, PairingFailure, PairingRequest,
    PairingResponse, WindowCloseReason,
};
use trouble_host::prelude::*;

const CONNECTIONS_MAX: usize = transport::BLE_LINKS;
// Signal + ATT per link.
const L2CAP_CHANNELS_MAX: usize = 2 * CONNECTIONS_MAX;
/// An untrusted link that neither encrypts with a bond nor pairs is dropped,
/// so a stranger holds a link slot only briefly. Bonded devices encrypt well
/// within this (see docs/notes/ble-pairing.md for measurements).
const UNTRUSTED_TIMEOUT: Duration = Duration::from_secs(3);
/// Data bytes of one ACL packet, as the controller reports them to the host.
///
/// esp-radio 1.0.0-beta.1 takes HCI packets of at most 259 bytes from the
/// host, and one with the default 255 data bytes is 260 with its header and
/// packet indicator. It drops that packet, then reads the rest of it as new
/// packets: the first notification longer than 251 bytes ended all BLE
/// traffic and left the radio deaf to Wi-Fi as well. 251 is one link-layer
/// payload.
const CONTROLLER_ACL_BYTES: u16 = 251;
/// ACL packets the controller lets the host queue for transmission.
///
/// esp-radio takes an mbuf per packet from pools fixed at 12 blocks of
/// 256 bytes and 24 of 320, and panics when one is empty. Its default of 24
/// packets outgrew them once Wi-Fi kept the radio from sending: the bridge
/// reset during a 20 kHz capture with a central driving. A packet of 229 to
/// 240 bytes can take two small blocks, so four packets stay within the
/// small pool with room for received ones.
const CONTROLLER_ACL_PACKETS: u16 = 4;
/// The connection interval asked of a trusted central that keeps a longer
/// one, the same as the remote sets itself. A Linux host connects at 45 ms:
/// three connection events span the controller's 150 ms command deadman, and
/// a velocity setpoint held over that link stopped with CommTimeout within
/// ten seconds.
const DRIVE_INTERVAL_MAX: Duration = Duration::from_millis(20);
const DRIVE_PARAMS: RequestedConnParams = RequestedConnParams {
    min_connection_interval: Duration::from_millis(15),
    max_connection_interval: DRIVE_INTERVAL_MAX,
    max_latency: 0,
    min_event_length: Duration::from_millis(0),
    max_event_length: Duration::from_millis(0),
    supervision_timeout: Duration::from_millis(2000),
};

type Bonds = Mutex<NoopRawMutex, BondStore>;

static PAIRING: PairingState = PairingState::new();

#[allow(
    clippy::large_futures,
    reason = "the task future lives in static Embassy task storage; the partition \
    table buffer and the link slot futures are awaited in place on purpose"
)]
#[embassy_executor::task]
pub async fn run(
    stack: &'static transport::Stack,
    bt: esp_hal::peripherals::BT<'static>,
    ble_idents: [u8; transport::BLE_LINKS],
    flash: Flash,
) {
    // ========== BLE setup ==========
    // Maximum TX power: the board is powered and the link carries the throttle.
    let radio_config = esp_radio::ble::Config::default()
        .with_default_tx_power(esp_radio::ble::TxPower::P20)
        .with_max_connections(CONNECTIONS_MAX as u16)
        .with_acl_buf_size(CONTROLLER_ACL_BYTES)
        .with_acl_buf_count(CONTROLLER_ACL_PACKETS)
        // One filter accept list and resolving list entry per bond.
        .with_whitelist_size(MAX_BONDS as u8)
        .with_ll_resolv_list_size(MAX_BONDS as u16);
    let bt_transport = BleConnector::new(bt, radio_config).expect("BLE connector init");
    crate::csprng::seed();
    let ble_controller = ExternalController::<_, CONNECTIONS_MAX>::new(bt_transport);
    let mut resources: HostResources<
        DefaultPacketPool,
        CONNECTIONS_MAX,
        L2CAP_CHANNELS_MAX,
        1,
        MAX_BONDS,
    > = HostResources::new();

    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    // The bridge's keyboard and display are a trusted device at the other end
    // of Ergot, so every pairing method with MITM protection is available.
    let ble_stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .set_io_capabilities(IoCapabilities::KeyboardDisplay)
        .build();

    let bonds = Mutex::<NoopRawMutex, _>::new(BondStore::open(flash).await);
    for (_, bond) in bonds.lock().await.iter() {
        if ble_stack.add_bond_information(bond.to_trouble()).is_err() {
            warn!("[bonds] trouble bond table full");
        }
    }

    let peripheral = Mutex::<NoopRawMutex, _>::new(ble_stack.peripheral());
    let runner = ble_stack.runner();

    let nus_server = NusServer::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Oxifoc Bridge",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .expect("NUS GATT server init");

    let nets = ble_idents.map(|ident| {
        stack
            .manage_profile(|p| p.net_id_of(ident))
            .expect("registered BLE segment")
    });
    let ctx = Ctx {
        stack,
        ble_stack: &ble_stack,
        bonds: &bonds,
    };

    // Each link slot advertises while it is free. The peripheral mutex lets
    // one free slot advertise at a time; a connected slot releases it, so the
    // bridge keeps advertising while any slot is free.
    let [ident_a, ident_b] = ble_idents;
    join5(
        ble_runner(runner),
        link_slot(&ctx, &peripheral, &nus_server, 0, ident_a),
        link_slot(&ctx, &peripheral, &nus_server, 1, ident_b),
        pairing_server(&ctx, nets),
        window_timer(stack),
    )
    .await;
}

/// Shared handles for the link slots and the pairing server.
struct Ctx<'a, 's, C: Controller> {
    stack: &'static transport::Stack,
    ble_stack: &'a Stack<'s, C, DefaultPacketPool>,
    bonds: &'a Bonds,
}

fn publish(stack: &'static transport::Stack, event: &PairingEvent) {
    match event {
        PairingEvent::ShowPasskey { attempt, .. } | PairingEvent::CompareNumber { attempt, .. } => {
            info!("[pairing] confirmation requested for attempt {attempt}");
        }
        _ => info!("[pairing] event {event:?}"),
    }
    if stack
        .topics()
        .broadcast::<PairingEventTopic>(event, None)
        .is_err()
    {
        warn!("[pairing] event not sent");
    }
}

fn close_window(stack: &'static transport::Stack, attempt: u32, reason: WindowCloseReason) {
    if let Some(window) = PAIRING.close(attempt) {
        // A bonded candidate stays connected; any other end drops it.
        if reason != WindowCloseReason::Bonded
            && let Some(link) = window.stage.link()
        {
            PAIRING.inputs[link].signal(Input::Cancel);
        }
        publish(stack, &PairingEvent::WindowClosed { attempt, reason });
        info!("[pairing] window {attempt} closed: {reason:?}");
    }
}

#[allow(clippy::large_stack_frames)]
async fn link_slot<'v, C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>>(
    ctx: &Ctx<'_, '_, C>,
    peripheral: &Mutex<NoopRawMutex, Peripheral<'v, C, DefaultPacketPool>>,
    server: &NusServer<'v>,
    link: usize,
    ident: u8,
) {
    loop {
        let accepted = {
            let mut peripheral = peripheral.lock().await;
            loop {
                PAIRING.adv_changed.reset();
                // Outside a pairing window only bonded devices may connect: the
                // controller ignores everyone else, so strangers take no slot.
                let filtered = !PAIRING.is_open();
                if filtered {
                    let addresses: heapless::Vec<Address, MAX_BONDS> = ctx
                        .bonds
                        .lock()
                        .await
                        .iter()
                        .map(|(_, bond)| bond.identity().addr)
                        .collect();
                    set_filter_accept_list(&mut peripheral, &addresses).await;
                }
                match select(
                    advertise(&mut peripheral, server, filtered),
                    PAIRING.adv_changed.wait(),
                )
                .await
                {
                    Either::First(result) => break result,
                    // Dropping the advertiser stops advertising; the short idle
                    // gap also lets trouble update the resolving list.
                    Either::Second(()) => Timer::after_millis(20).await,
                }
            }
        };
        match accepted {
            Ok(conn) => {
                // The central chooses the PHY: the remote stays on 1M for
                // sensitivity, host tools may request 2M for telemetry.
                debug!("[ble{link}] connection established");
                let counters = &crate::diag::BLE[link];
                crate::diag::BleCounters::bump(&counters.connects);
                counters.connected.store(true, Ordering::Relaxed);
                link_session(ctx, server, &conn, link, ident).await;
                crate::diag::BleCounters::bump(&counters.disconnects);
                counters.connected.store(false, Ordering::Relaxed);
                counters.trusted.store(false, Ordering::Relaxed);
                PAIRING.link_bond[link].store(NO_BOND, Ordering::Relaxed);
                stack_inactive(ctx.stack, ident);
                // Never send frames queued for the old central to the next one.
                let consumer = transport::BLE_OUTQ[link].framed_consumer();
                while let Ok(frame) = consumer.read() {
                    frame.release();
                }
                debug!("[ble{link}] disconnected, slot free");
            }
            Err(_e) => {
                warn!("[ble{link}] advertise error, retrying");
                Timer::after_secs(1).await;
            }
        }
    }
}

/// The controller refuses list changes while advertising is still stopping.
async fn set_filter_accept_list<C: Controller>(
    peripheral: &mut Peripheral<'_, C, DefaultPacketPool>,
    addresses: &[Address],
) {
    for _ in 0..10 {
        if peripheral.set_filter_accept_list(addresses).await.is_ok() {
            return;
        }
        Timer::after_millis(10).await;
    }
    warn!("[ble] filter accept list not updated");
}

fn stack_inactive(stack: &'static transport::Stack, ident: u8) {
    stack.manage_profile(|router| {
        let _ = router.set_interface_state(ident, InterfaceState::Inactive);
    });
}

async fn ble_runner<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(error) = runner.run().await {
            error!("[ble] runner error, restarting: {error:?}");
        }
    }
}

// ========== BLE advertise ==========

#[allow(clippy::large_stack_frames)]
async fn advertise<'v, 's, C: Controller>(
    peripheral: &mut Peripheral<'v, C, DefaultPacketPool>,
    server: &'s NusServer<'v>,
    filtered: bool,
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
        interval_min: Duration::from_millis(20),
        interval_max: Duration::from_millis(30),
        filter_policy: if filtered {
            AdvFilterPolicy::FilterConn
        } else {
            AdvFilterPolicy::Unfiltered
        },
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

    debug!("[ble] advertising, filtered: {filtered}");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

/// Ask a trusted central for [`DRIVE_PARAMS`] unless it already runs them.
async fn request_drive_interval<C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>>(
    ctx: &Ctx<'_, '_, C>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    link: usize,
) {
    let interval = conn.raw().params().conn_interval;
    if interval <= DRIVE_INTERVAL_MAX {
        return;
    }
    info!(
        "[ble{}] connection interval {} us, asking for 15-20 ms",
        link,
        interval.as_micros()
    );
    if conn
        .raw()
        .update_connection_params(ctx.ble_stack, &DRIVE_PARAMS)
        .await
        .is_err()
    {
        warn!("[ble{link}] connection parameter request failed");
    }
}

// ========== Link session: gate, pairing and frame pump ==========

#[allow(clippy::large_stack_frames)]
async fn link_session<C: Controller + ControllerCmdSync<LeReadLocalSupportedFeatures>>(
    ctx: &Ctx<'_, '_, C>,
    server: &NusServer<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    link: usize,
    ble_ident: u8,
) {
    let rx_handle = server.nus.rx.handle;
    let tx_char = &server.nus.tx;
    let protected = |handle: u16| {
        handle == rx_handle || handle == tx_char.handle || Some(handle) == tx_char.cccd_handle
    };
    let consumer = transport::BLE_OUTQ[link].framed_consumer();
    let net = ctx
        .stack
        .manage_profile(|p| p.net_id_of(ble_ident))
        .expect("registered BLE segment");
    let mut processor = RouterFrameProcessor::new(net);

    PAIRING.inputs[link].reset();
    PAIRING.kick[link].reset();
    // A bond can only be created while a window is open at connection time:
    // the bonding flag is exchanged when pairing starts.
    let _ = conn.raw().set_bondable(PAIRING.is_open());
    let connected_at = Instant::now();
    let mut trusted: Option<u8> = None;
    let mut attempt: Option<u32> = None;
    // While a window is open a new device may need longer before it starts
    // pairing (service discovery first), so it may stay until the window ends.
    let untrusted_deadline = match PAIRING.window() {
        Some(window) => window.deadline.max(Instant::now() + UNTRUSTED_TIMEOUT),
        None => Instant::now() + UNTRUSTED_TIMEOUT,
    };

    loop {
        let frame = async {
            if trusted.is_some() {
                consumer.wait_read().await
            } else {
                pending().await
            }
        };
        let deadline = async {
            if trusted.is_some() || attempt.is_some() {
                PAIRING.kick[link].wait().await;
            } else {
                match select(PAIRING.kick[link].wait(), Timer::at(untrusted_deadline)).await {
                    Either::First(()) | Either::Second(()) => {}
                }
            }
        };
        match select4(conn.next(), frame, PAIRING.inputs[link].wait(), deadline).await {
            Either4::First(event) => match event {
                GattConnectionEvent::Disconnected { reason } => {
                    crate::diag::BLE[link]
                        .last_disconnect_reason
                        .store(reason.into_inner(), Ordering::Relaxed);
                    break;
                }
                GattConnectionEvent::Encrypted {
                    security_level,
                    bond: Some(bond),
                } => {
                    let id = ctx.bonds.lock().await.find(&bond.identity);
                    if let Some(id) = id
                        && security_level.authenticated()
                    {
                        trusted = Some(id);
                        PAIRING.link_bond[link].store(id, Ordering::Relaxed);
                        crate::diag::BLE[link]
                            .trusted
                            .store(true, Ordering::Relaxed);
                        info!(
                            "[ble{}] trusted, bond {}, {} ms after connect",
                            link,
                            id,
                            connected_at.elapsed().as_millis()
                        );
                        request_drive_interval(ctx, conn, link).await;
                    }
                }
                GattConnectionEvent::ConnectionParamsUpdated { conn_interval, .. } => {
                    info!(
                        "[ble{}] connection interval {} us",
                        link,
                        conn_interval.as_micros()
                    );
                }
                // The method tells the device kind: a remote has only a screen
                // (it shows the code, Passkey Entry with input here); phones
                // and laptops compare numbers or type a code. A candidate of
                // the wrong kind does not take the window.
                GattConnectionEvent::PassKeyDisplay(key) => {
                    // The candidate types the code (it is KeyboardOnly).
                    if let Some(a) = claim(ctx, conn, link, &mut attempt, Some(BondRole::Host)) {
                        PAIRING.set_stage(a, Stage::Pairing { link });
                        publish(
                            ctx.stack,
                            &PairingEvent::ShowPasskey {
                                attempt: a,
                                passkey: key.value(),
                            },
                        );
                    } else {
                        let _ = conn.raw().pass_key_cancel();
                    }
                }
                GattConnectionEvent::PassKeyConfirm(key) => {
                    if let Some(a) = claim(ctx, conn, link, &mut attempt, Some(BondRole::Host)) {
                        PAIRING.set_stage(a, Stage::AwaitComparison { link });
                        publish(
                            ctx.stack,
                            &PairingEvent::CompareNumber {
                                attempt: a,
                                number: key.value(),
                            },
                        );
                    } else {
                        let _ = conn.raw().pass_key_cancel();
                    }
                }
                GattConnectionEvent::PassKeyInput => {
                    if let Some(a) = claim(ctx, conn, link, &mut attempt, Some(BondRole::Remote)) {
                        PAIRING.set_stage(a, Stage::AwaitPasskey { link });
                        publish(ctx.stack, &PairingEvent::EnterPasskey { attempt: a });
                    } else {
                        let _ = conn.raw().pass_key_cancel();
                    }
                }
                GattConnectionEvent::PairingComplete { .. } if trusted.is_some() => {
                    // trouble reports completion again with the encryption change.
                    debug!("[ble{link}] repeated pairing completion ignored");
                }
                GattConnectionEvent::PairingComplete {
                    security_level,
                    bond,
                } => {
                    match pairing_complete(ctx, conn, link, &mut attempt, security_level, bond)
                        .await
                    {
                        Some(id) => {
                            trusted = Some(id);
                            PAIRING.link_bond[link].store(id, Ordering::Relaxed);
                            crate::diag::BLE[link]
                                .trusted
                                .store(true, Ordering::Relaxed);
                            request_drive_interval(ctx, conn, link).await;
                        }
                        None => {
                            conn.raw().disconnect();
                        }
                    }
                }
                GattConnectionEvent::PairingFailed(e) => {
                    warn!("[ble{link}] pairing failed: {e:?}");
                    let reason = match e {
                        Error::Security(
                            PairingFailedReason::ConfirmValueFailed
                            | PairingFailedReason::DHKeyCheckFailed,
                        ) => PairingFailure::WrongCode,
                        Error::Security(PairingFailedReason::PasskeyEntryFailed) => {
                            PairingFailure::Cancelled
                        }
                        _ => PairingFailure::Protocol,
                    };
                    if let Some(a) = attempt.take() {
                        fail(ctx.stack, a, reason);
                    }
                    conn.raw().disconnect();
                }
                GattConnectionEvent::Encrypted {
                    security_level,
                    bond: None,
                } => {
                    info!("[ble{link}] encrypted without a stored bond: {security_level:?}");
                }
                GattConnectionEvent::BondLost => {
                    warn!("[ble{link}] peer lost its bond");
                }
                GattConnectionEvent::Gatt { event } => {
                    let handle = match &event {
                        GattEvent::Read(e) => Some(e.handle()),
                        GattEvent::Write(e) => Some(e.handle()),
                        _ => None,
                    };
                    if trusted.is_none() && handle.is_some_and(protected) {
                        // Makes OS centrals start pairing; ours pair first.
                        match event.reject(AttErrorCode::INSUFFICIENT_AUTHENTICATION) {
                            Ok(reply) => reply.send().await,
                            Err(_e) => warn!("[ble] error sending gatt reply"),
                        }
                        continue;
                    }
                    if let GattEvent::Write(ref write_event) = event
                        && write_event.handle() == rx_handle
                    {
                        let changed = write_event.with_data(|offset, data| {
                            offset == 0 && processor.process_frame(data, &ctx.stack, ble_ident)
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
            Either4::Second(grant) => {
                let payload: heapless::Vec<u8, { crate::ble_nus::NUS_MAX_PAYLOAD }> =
                    heapless::Vec::from_slice(&grant).unwrap_or_default();
                let result = tx_char.notify(conn, &payload, false).await;
                grant.release();
                if result.is_err() {
                    warn!("[ble] notify error, disconnecting");
                    crate::diag::BleCounters::bump(&crate::diag::BLE[link].notify_errors);
                    break;
                }
            }
            Either4::Third(input) => match input {
                Input::Passkey(passkey) => {
                    if conn.raw().pass_key_input(passkey).is_err() {
                        warn!("[ble{link}] passkey input refused");
                    }
                }
                Input::Comparison(true) => {
                    if conn.raw().pass_key_confirm().is_err() {
                        warn!("[ble{link}] comparison confirm refused");
                    }
                }
                Input::Comparison(false) | Input::Cancel => {
                    let _ = conn.raw().pass_key_cancel();
                    if let Some(a) = attempt.take() {
                        let reason = if matches!(input, Input::Cancel) {
                            PairingFailure::Cancelled
                        } else {
                            PairingFailure::Rejected
                        };
                        fail(ctx.stack, a, reason);
                    }
                    conn.raw().disconnect();
                }
            },
            Either4::Fourth(()) => {
                debug!("[ble{link}] dropping link (untrusted or bond removed)");
                if trusted.is_none() {
                    crate::diag::BleCounters::bump(&crate::diag::BLE[link].untrusted_drops);
                }
                conn.raw().disconnect();
                break;
            }
        }
    }

    if let Some(a) = attempt {
        fail(ctx.stack, a, PairingFailure::Disconnected);
    }
}

/// Claim the open window for this link; on the first claim, report it.
fn claim<C: Controller, P: PacketPool>(
    ctx: &Ctx<'_, '_, C>,
    conn: &GattConnection<'_, '_, P>,
    link: usize,
    attempt: &mut Option<u32>,
    role: Option<BondRole>,
) -> Option<u32> {
    let claimed = PAIRING.claim(link, role)?;
    if attempt.replace(claimed).is_none() {
        let mut address = [0; 6];
        address.copy_from_slice(conn.raw().peer_address().addr.raw());
        publish(
            ctx.stack,
            &PairingEvent::CandidateConnected {
                attempt: claimed,
                address,
            },
        );
        info!("[ble{link}] pairing in window {claimed}");
    }
    Some(claimed)
}

fn fail(stack: &'static transport::Stack, attempt: u32, reason: PairingFailure) {
    warn!("[pairing] attempt {attempt} failed: {reason:?}");
    publish(stack, &PairingEvent::Failed { attempt, reason });
    close_window(stack, attempt, WindowCloseReason::Failed);
}

/// Accept the bond only inside the window and only with MITM protection.
async fn pairing_complete<C: Controller, P: PacketPool>(
    ctx: &Ctx<'_, '_, C>,
    conn: &GattConnection<'_, '_, P>,
    link: usize,
    attempt: &mut Option<u32>,
    security_level: SecurityLevel,
    bond: Option<BondInformation>,
) -> Option<u8> {
    // Just Works raises no passkey event, so claim here as well.
    let claimed = attempt.or_else(|| claim(ctx, conn, link, attempt, None));
    // Drop a bond trouble added for this pairing, never one we store.
    let forget = async |bond: &Option<BondInformation>| {
        if let Some(bond) = bond
            && ctx.bonds.lock().await.find(&bond.identity).is_none()
        {
            let _ = ctx.ble_stack.remove_bond_information(bond.identity);
        }
    };
    let Some(a) = claimed else {
        info!("[ble{link}] pairing outside a window, dropping");
        forget(&bond).await;
        return None;
    };
    let failure = if !security_level.authenticated() {
        Some(PairingFailure::NotAuthenticated)
    } else if bond.is_none() {
        Some(PairingFailure::NotBonded)
    } else {
        None
    };
    if let Some(reason) = failure {
        forget(&bond).await;
        attempt.take();
        fail(ctx.stack, a, reason);
        return None;
    }
    let bond = bond?;
    let window = PAIRING.window().filter(|w| w.attempt == a)?;
    let stored = StoredBond::from_trouble(&bond, window.role, window.name, window.added_by);
    let mut bonds = ctx.bonds.lock().await;
    match bonds.insert(stored).await {
        Ok(id) => {
            attempt.take();
            if let Some(entry) = bonds.entry(id) {
                publish(ctx.stack, &PairingEvent::Bonded(entry));
            }
            PAIRING.adv_changed.signal(());
            close_window(ctx.stack, a, WindowCloseReason::Bonded);
            info!("[ble{link}] bonded as {id}");
            Some(id)
        }
        Err(_e) => {
            drop(bonds);
            forget(&Some(bond)).await;
            attempt.take();
            fail(ctx.stack, a, PairingFailure::Storage);
            None
        }
    }
}

// ========== Pairing management endpoint ==========

#[allow(clippy::large_stack_frames)]
async fn pairing_server<C: Controller>(ctx: &Ctx<'_, '_, C>, nets: [u16; transport::BLE_LINKS]) {
    let server = ctx
        .stack
        .endpoints()
        .bounded_server::<PairingEndpoint, 2>(None);
    let server = pin!(server);
    let mut hdl = server.attach();
    info!("[pairing] server running");
    loop {
        let _ = hdl
            .serve_full(async |msg: &HeaderMessage<PairingRequest>| {
                handle_request(ctx, &nets, msg).await
            })
            .await;
    }
}

async fn handle_request<C: Controller>(
    ctx: &Ctx<'_, '_, C>,
    nets: &[u16; transport::BLE_LINKS],
    msg: &HeaderMessage<PairingRequest>,
) -> PairingResponse {
    let result = match &msg.t {
        PairingRequest::OpenWindow {
            role,
            seconds,
            name,
        } => {
            // TODO(ble-pairing invariant 3): also require a stopped controller.
            if ctx.bonds.lock().await.is_full() {
                return PairingResponse::Error(PairingError::TableFull);
            }
            let added_by = PAIRING.bond_on_net(nets, msg.hdr.src.network_id);
            match PAIRING.open(*role, name.clone(), *seconds, added_by) {
                Ok((attempt, expires_ms)) => {
                    info!("[pairing] window {attempt} open for {expires_ms} ms");
                    publish(
                        ctx.stack,
                        &PairingEvent::WindowOpened {
                            attempt,
                            role: *role,
                            expires_ms,
                        },
                    );
                    return PairingResponse::WindowOpened {
                        attempt,
                        expires_ms,
                    };
                }
                Err(e) => Err(e),
            }
        }
        PairingRequest::SubmitPasskey { attempt, passkey } => {
            PAIRING.submit(*attempt, Input::Passkey(*passkey))
        }
        PairingRequest::ConfirmComparison { attempt, accept } => {
            PAIRING.submit(*attempt, Input::Comparison(*accept))
        }
        PairingRequest::Cancel { attempt } => {
            if PAIRING.window().is_some_and(|w| w.attempt == *attempt) {
                close_window(ctx.stack, *attempt, WindowCloseReason::Cancelled);
                Ok(())
            } else {
                Err(PairingError::NoSuchAttempt)
            }
        }
        PairingRequest::ListBonds => {
            return PairingResponse::Bonds(ctx.bonds.lock().await.entries());
        }
        PairingRequest::RemoveBond { id } => {
            let removed = ctx.bonds.lock().await.remove(*id).await;
            removed.map(|bond| {
                let _ = ctx.ble_stack.remove_bond_information(bond.identity());
                for (link, kick) in PAIRING.kick.iter().enumerate() {
                    if PAIRING.link_bond[link].load(Ordering::Relaxed) == *id {
                        kick.signal(());
                    }
                }
                publish(ctx.stack, &PairingEvent::BondRemoved { id: *id });
                PAIRING.adv_changed.signal(());
                info!("[pairing] bond {id} removed");
            })
        }
        PairingRequest::RenameBond { id, name } => {
            ctx.bonds.lock().await.rename(*id, name.clone()).await
        }
    };
    match result {
        Ok(()) => PairingResponse::Ok,
        Err(e) => PairingResponse::Error(e),
    }
}

/// Close the window when it expires.
async fn window_timer(stack: &'static transport::Stack) {
    loop {
        match PAIRING.window() {
            None => PAIRING.changed.wait().await,
            Some(window) => {
                if let Either::First(()) =
                    select(Timer::at(window.deadline), PAIRING.changed.wait()).await
                {
                    close_window(stack, window.attempt, WindowCloseReason::Timeout);
                }
            }
        }
    }
}

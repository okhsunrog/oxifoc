//! BLE framed transport over the OxiFOC GATT service (NUS-shaped, own UUIDs):
//! one ergot frame per GATT write/notification.
//! The shared backend owns BLE; UI signals never carry protocol packets.
use anyhow::{Context, Result, anyhow, ensure};
use blec::{Connection, OnDisconnectHandler, models::WriteType};
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};
use ergot::net_stack::ArcNetStack;
use std::sync::{Arc, LazyLock};
use tokio_util::sync::CancellationToken;
use tracing::{info, warn};
use uuid::Uuid;

type StdQueue = ergot::interface_manager::utils::std::StdQueue;
type WaitQueue = ergot::toolkits::tokio_stream::WaitQueue;
pub(crate) const SERVICE_UUID: Uuid = Uuid::from_u128(oxifoc_core::ble::SERVICE_UUID);
const RX_UUID: Uuid = Uuid::from_u128(oxifoc_core::ble::RX_UUID);
const TX_UUID: Uuid = Uuid::from_u128(oxifoc_core::ble::TX_UUID);
const BLE_MTU: u16 = 505;
const OUT_BUFFER_SIZE: usize = 4096;
const RX_QUEUE_FRAMES: usize = 128;

// blec's global adapter must not belong to the CLI's short-lived scan runtime.
// Initialize it on a process-lifetime thread. Connection workers still belong
// to each HostRuntime and are explicitly joined before it shuts down.
static BLE_RUNTIME: LazyLock<tokio::runtime::Handle> = LazyLock::new(|| {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("BLE runtime");
    let handle = runtime.handle().clone();
    std::thread::Builder::new()
        .name("oxifoc-ble".into())
        .spawn(move || runtime.block_on(std::future::pending::<()>()))
        .expect("BLE thread");
    handle
});

pub(crate) async fn handler() -> Result<&'static blec::Handler> {
    BLE_RUNTIME
        .spawn(async {
            static INIT: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
            let _guard = INIT.lock().await;
            let handler = blec::init().await?;
            // init() creates only the handler; the adapter/event pump is lazy.
            // Force that first access onto the long-lived runtime too. Android
            // may report Unknown until permission is granted; discovery returns
            // the actionable error, and the next call retries initialization.
            let _ = handler.get_adapter_state().await;
            Ok::<_, blec::Error>(handler)
        })
        .await
        .context("BLE initialization task")?
        .context("BLE initialization")
}

pub struct BleNusInterface;
impl ergot::interface_manager::Interface for BleNusInterface {
    type Sink = framed_stream::Sink<StdQueue>;
}
pub type EdgeStack = ArcNetStack<
    ergot::exports::mutex::raw_impls::cs::CriticalSectionRawMutex,
    DirectEdge<BleNusInterface>,
>;
pub fn new_stack() -> (EdgeStack, StdQueue) {
    let queue = ergot::interface_manager::utils::std::new_std_queue(OUT_BUFFER_SIZE);
    let stack = EdgeStack::new_with_profile(DirectEdge::new_target(
        framed_stream::Sink::new_from_handle(queue.clone(), BLE_MTU),
    ));
    (stack, queue)
}

#[derive(Default)]
pub(crate) struct Session {
    address: Option<String>,
    subscribed: bool,
    workers: Vec<tokio::task::JoinHandle<()>>,
}
impl Session {
    pub async fn close(&mut self) {
        for worker in self.workers.drain(..) {
            worker.abort();
            let _ = worker.await;
        }
        let subscribed = std::mem::take(&mut self.subscribed);
        if let Some(address) = self.address.take()
            && let Ok(handler) = handler().await
        {
            // Android can retain the physical link for other GATT clients.
            // Explicitly end our notification session even in that case.
            if subscribed
                && handler.is_device_connected(&address)
                && let Err(error) = handler
                    .unsubscribe_from(&address, TX_UUID, Some(SERVICE_UUID))
                    .await
            {
                warn!("BLE unsubscribe: {error}");
            }
            // Also cancel a pending connect: it has a link entry but has not
            // become is_device_connected() yet. No entry is normal when a
            // cancelled connect was still discovering the address.
            if let Err(error) = handler.disconnect_device(&address).await
                && !matches!(
                    error,
                    blec::Error::DeviceNotConnected(_) | blec::Error::NoDeviceConnected
                )
            {
                warn!("BLE disconnect: {error}");
            }
        }
    }

    pub async fn register(
        &mut self,
        stack: &EdgeStack,
        queue: &StdQueue,
        address: &str,
        state_notify: Arc<WaitQueue>,
    ) -> Result<()> {
        self.close().await;
        ensure!(
            stack.manage_profile(|im| matches!(
                im.interface_state(()),
                Some(InterfaceState::Down) | None
            )),
            "BLE interface not in Down state"
        );
        let handler = handler().await?;
        self.address = Some(address.to_owned());
        let failed = CancellationToken::new();
        let disconnected = failed.clone();
        let connection = handler
            .connect(
                address,
                OnDisconnectHandler::from_sync(move || {
                    disconnected.cancel();
                }),
                false,
            )
            .await
            .context("BLE connection")?;
        let setup = async {
            let mtu = connection.mtu().await.context("ATT MTU")?;
            validate_mtu(mtu)?;
            let services = connection.services().await.context("NUS discovery")?;
            let nus = services
                .iter()
                .find(|s| s.uuid == SERVICE_UUID)
                .ok_or_else(|| anyhow!("OxiFOC BLE service missing"))?;
            ensure!(
                nus.characteristics.iter().any(|c| c.uuid == RX_UUID),
                "OxiFOC BLE RX missing"
            );
            ensure!(
                nus.characteristics.iter().any(|c| c.uuid == TX_UUID),
                "OxiFOC BLE TX missing"
            );
            let (tx, rx) = tokio::sync::mpsc::channel(RX_QUEUE_FRAMES);
            let overflow = failed.clone();
            // Also clean up if setup is cancelled while the CCCD write is in flight.
            self.subscribed = true;
            connection
                .subscribe(TX_UUID, Some(SERVICE_UUID), move |data: Vec<u8>| {
                    // Never stall blec's event loop. A dropped protocol frame requires
                    // a new session; it must not silently corrupt the ergot stream.
                    if data.len() > usize::from(BLE_MTU) || tx.try_send(data).is_err() {
                        overflow.cancel();
                    }
                })
                .await
                .context("NUS notifications")?;
            ensure!(
                !failed.is_cancelled(),
                "BLE disconnected during subscription"
            );
            stack.manage_profile(|im| {
                let _ = im.set_interface_state(
                    (),
                    InterfaceState::Active {
                        net_id: 0,
                        node_id: EDGE_NODE_ID,
                    },
                );
            });
            state_notify.wake_all();
            self.workers.push(tokio::spawn(rx_worker(
                stack.clone(),
                rx,
                failed.clone(),
                state_notify.clone(),
            )));
            self.workers.push(tokio::spawn(tx_worker(
                stack.clone(),
                queue.clone(),
                connection.clone(),
                failed.clone(),
                state_notify.clone(),
            )));
            info!("BLE NUS active: {address}, ATT MTU {mtu}, frame limit {BLE_MTU}");
            Ok(())
        }
        .await;
        if setup.is_err() {
            self.close().await;
        }
        setup
    }
}

fn validate_mtu(mtu: u16) -> Result<()> {
    ensure!(
        mtu.saturating_sub(3) >= BLE_MTU,
        "ATT MTU {mtu} is too small: OxiFOC needs at least {} for {BLE_MTU}-byte frames",
        BLE_MTU + 3
    );
    Ok(())
}

async fn rx_worker(
    stack: EdgeStack,
    mut rx: tokio::sync::mpsc::Receiver<Vec<u8>>,
    failed: CancellationToken,
    notify: Arc<WaitQueue>,
) {
    let mut processor = EdgeFrameProcessor::new();
    let mut have_received = false;
    loop {
        tokio::select! {
            biased;
            _ = failed.cancelled() => break,
            data = rx.recv() => match data {
                Some(data) => {
                    have_received = true;
                    if processor.process_frame(&data, &stack, ()) { notify.wake_all(); }
                }
                None => break,
            },
            // Match the other framed transports: handshake bounds initial
            // silence, then loss of incoming traffic fails the active link.
            _ = tokio::time::sleep(std::time::Duration::from_millis(
                oxifoc_core::icd::LIVENESS_TIMEOUT_MS
            )), if have_received => {
                warn!("BLE liveness timeout");
                break;
            },
        }
    }
    failed.cancel();
    mark_down(&stack, &notify);
}
async fn tx_worker(
    stack: EdgeStack,
    queue: StdQueue,
    connection: Connection,
    failed: CancellationToken,
    notify: Arc<WaitQueue>,
) {
    use ergot::exports::bbqueue::traits::bbqhdl::BbqHandle;
    let consumer: ergot::exports::bbqueue::prod_cons::framed::FramedConsumer<StdQueue> =
        queue.framed_consumer();
    loop {
        tokio::select! {
            biased;
            _ = failed.cancelled() => break,
            frame = consumer.wait_read() => {
                let result = connection.send_data(RX_UUID, Some(SERVICE_UUID), &frame, WriteType::WithoutResponse).await;
                frame.release();
                if let Err(error) = result { warn!("BLE write: {error}"); break; }
            }
        }
    }
    failed.cancel();
    mark_down(&stack, &notify);
}
fn mark_down(stack: &EdgeStack, notify: &WaitQueue) {
    stack.manage_profile(|im| {
        let _ = im.set_interface_state((), InterfaceState::Down);
    });
    notify.wake_all();
}

#[cfg(test)]
mod tests {
    #[tokio::test]
    async fn silent_connected_peer_goes_down_without_a_disconnect_event() {
        use super::*;
        let (stack, _queue) = new_stack();
        stack.manage_profile(|im| {
            im.set_interface_state(
                (),
                InterfaceState::Active {
                    net_id: 0,
                    node_id: EDGE_NODE_ID,
                },
            )
            .unwrap();
        });
        let (tx, rx) = tokio::sync::mpsc::channel(2);
        let failed = CancellationToken::new();
        let notify = Arc::new(WaitQueue::new());
        // Keep the sender open: only the liveness deadline can end this worker.
        tx.send(Vec::new()).await.unwrap();
        tokio::time::timeout(
            std::time::Duration::from_millis(oxifoc_core::icd::LIVENESS_TIMEOUT_MS * 2),
            rx_worker(stack.clone(), rx, failed.clone(), notify),
        )
        .await
        .unwrap();
        assert!(failed.is_cancelled());
        assert!(
            stack.manage_profile(|im| matches!(im.interface_state(()), Some(InterfaceState::Down)))
        );
        drop(tx);
    }

    #[test]
    fn nus_requires_a_complete_frame_in_one_att_packet() {
        for mtu in [0, 3, 23, 247, 507] {
            assert!(super::validate_mtu(mtu).is_err());
        }
        for mtu in [508, 512, 517] {
            assert!(super::validate_mtu(mtu).is_ok());
        }
    }
}

#[cfg(all(test, feature = "ble-mock-tests"))]
mod lifecycle_tests {
    use super::*;
    use blec::mock::{DeviceSpec, FailKind, MockWorld, OpBehaviour, ServiceSpec};
    use btleplug::api::CharPropFlags;
    use std::time::Duration;

    fn spec(last: u8, mtu: u16) -> DeviceSpec {
        DeviceSpec::new([0xfa, 0, 0, 0, 0, last])
            .name("OxiFOC test")
            .mtu(mtu)
            .service(
                ServiceSpec::new(SERVICE_UUID)
                    .characteristic(RX_UUID, CharPropFlags::WRITE_WITHOUT_RESPONSE)
                    .characteristic(TX_UUID, CharPropFlags::NOTIFY),
            )
    }
    fn is_down(stack: &EdgeStack) -> bool {
        stack.manage_profile(|im| matches!(im.interface_state(()), Some(InterfaceState::Down)))
    }

    // One test owns the process-wide mock adapter, just as one application does.
    // Use wall time: the adapter intentionally survives individual Tokio runtimes.
    #[test]
    fn scan_runtime_drop_mtu_failure_reconnect_and_shutdown() {
        let world = MockWorld::global();
        let good = world.add_device(spec(1, 517));
        let small = world.add_device(spec(2, 247));
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        let found = rt
            .block_on(crate::try_scan_ble_devices(Duration::from_millis(250)))
            .unwrap();
        assert_eq!(found.len(), 2);
        drop(rt);
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async {
            let (stack, queue) = new_stack();
            let notify = Arc::new(WaitQueue::new());
            let mut session = Session::default();
            let error = session
                .register(&stack, &queue, &small.address_string(), notify.clone())
                .await
                .unwrap_err();
            assert!(error.to_string().contains("too small"));
            assert!(!small.is_connected());
            assert!(is_down(&stack));
            let pending = world.add_device(spec(3, 517));
            pending.set_connect_behaviour(blec::mock::ConnectBehaviour::Hang);
            assert!(
                tokio::time::timeout(
                    Duration::from_millis(200),
                    session.register(&stack, &queue, &pending.address_string(), notify.clone())
                )
                .await
                .is_err()
            );
            assert!(pending.count_ops(&blec::mock::Op::Connect) > 0);
            session.close().await;
            assert!(pending.count_ops(&blec::mock::Op::Disconnect) > 0);
            pending.complete_connect();
            tokio::time::timeout(Duration::from_secs(2), async {
                while pending.is_connected() {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap();
            good.set_subscribe_behaviour(OpBehaviour::Fail(FailKind::NotSupported));
            assert!(
                session
                    .register(&stack, &queue, &good.address_string(), notify.clone())
                    .await
                    .is_err()
            );
            assert!(!good.is_connected());
            assert!(is_down(&stack));
            good.set_subscribe_behaviour(OpBehaviour::Ok);
            for _ in 0..3 {
                session
                    .register(&stack, &queue, &good.address_string(), notify.clone())
                    .await
                    .unwrap();
                assert!(!is_down(&stack));
                assert_eq!(good.subscriptions().len(), 1);
                good.drop_link();
                tokio::time::timeout(Duration::from_secs(2), async {
                    while !is_down(&stack) {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                })
                .await
                .unwrap();
                assert!(is_down(&stack));
            }
            session
                .register(&stack, &queue, &good.address_string(), notify.clone())
                .await
                .unwrap();
            // An invalid/oversized packet must fail the whole session, not drop
            // a packet and leave the transport apparently healthy.
            assert!(good.notify(SERVICE_UUID, TX_UUID, vec![0; 506]));
            tokio::time::timeout(Duration::from_secs(2), async {
                while !is_down(&stack) {
                    tokio::time::sleep(Duration::from_millis(5)).await;
                }
            })
            .await
            .unwrap();
            session.close().await;
            assert!(!good.is_connected());
            assert!(session.workers.is_empty());
            assert!(good.count_ops(&blec::mock::Op::Unsubscribe(TX_UUID)) > 0);
            assert!(!session.subscribed);
        });
        drop(rt);
    }
}

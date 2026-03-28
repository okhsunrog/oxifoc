//! BLE NUS transport for oxifoc host communication.
//!
//! Uses bluest to connect as a BLE central to an oxifoc bridge device
//! running a Nordic UART Service (NUS) GATT server. Each ergot frame
//! maps to one GATT write (host → bridge) or notification (bridge → host).
//!
//! This is a **framed** transport (like USB/UDP), not a COBS stream.

use anyhow::{Context, Result, anyhow};
use bluest::{Adapter, Characteristic, Device, Uuid};
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::{FrameProcessor, InterfaceState, Profile};
use ergot::net_stack::ArcNetStack;
use futures::StreamExt;
use std::sync::Arc;
use tracing::{debug, error, info, warn};

type StdQueue = ergot::interface_manager::utils::std::StdQueue;
type WaitQueue = ergot::toolkits::tokio_stream::WaitQueue;

// NUS UUIDs
const NUS_SERVICE_UUID: Uuid = Uuid::from_u128(0x6e400001_b5a3_f393_e0a9_e50e24dcca9e);
const NUS_RX_UUID: Uuid = Uuid::from_u128(0x6e400002_b5a3_f393_e0a9_e50e24dcca9e);
const NUS_TX_UUID: Uuid = Uuid::from_u128(0x6e400003_b5a3_f393_e0a9_e50e24dcca9e);

/// BLE NUS MTU: bridge PacketPool MTU (512) - L2CAP (4) - ATT (3) = 505
const BLE_MTU: u16 = 505;
const OUT_BUFFER_SIZE: usize = 4096;

/// BLE NUS interface marker.
pub struct BleNusInterface;

impl ergot::interface_manager::Interface for BleNusInterface {
    type Sink = framed_stream::Sink<StdQueue>;
}

pub type EdgeStack = ArcNetStack<
    ergot::exports::mutex::raw_impls::cs::CriticalSectionRawMutex,
    DirectEdge<BleNusInterface>,
>;

/// Connect to an oxifoc bridge device over BLE NUS.
pub async fn connect(device: &Device, state_notify: Option<Arc<WaitQueue>>) -> Result<EdgeStack> {
    // Connect via adapter
    let adapter = Adapter::default()
        .await
        .context("Failed to get BLE adapter")?;
    info!("Connecting to BLE device...");
    adapter
        .connect_device(device)
        .await
        .context("BLE connection failed")?;
    info!("BLE connected");

    // Discover NUS service
    let services = device
        .discover_services_with_uuid(NUS_SERVICE_UUID)
        .await
        .context("NUS service discovery failed")?;

    let nus_service = services
        .first()
        .ok_or_else(|| anyhow!("NUS service not found on device"))?;

    let chars = nus_service
        .discover_characteristics()
        .await
        .context("Failed to discover NUS characteristics")?;

    let rx_char = chars
        .iter()
        .find(|c| c.uuid() == NUS_RX_UUID)
        .ok_or_else(|| anyhow!("NUS RX characteristic not found"))?
        .clone();

    let tx_char = chars
        .iter()
        .find(|c| c.uuid() == NUS_TX_UUID)
        .ok_or_else(|| anyhow!("NUS TX characteristic not found"))?
        .clone();

    info!("NUS service discovered (RX + TX characteristics)");

    // Create ergot stack
    let queue = ergot::interface_manager::utils::std::new_std_queue(OUT_BUFFER_SIZE);
    let stack = EdgeStack::new_with_profile(DirectEdge::new_target(
        framed_stream::Sink::new_from_handle(queue.clone(), BLE_MTU),
    ));

    stack.manage_profile(|im| {
        let _ = im.set_interface_state(
            (),
            InterfaceState::Active {
                net_id: 1,
                node_id: EDGE_NODE_ID,
            },
        );
    });

    if let Some(ref n) = state_notify {
        n.wake_all();
    }

    // Spawn RX worker — tx_char moved in so notify() borrow is 'static
    let rx_stack = stack.clone();
    let rx_notify = state_notify.clone();
    tokio::spawn(async move {
        let notifications = match tx_char.notify().await {
            Ok(n) => n,
            Err(e) => {
                error!("[ble] failed to subscribe to NUS TX: {:?}", e);
                return;
            }
        };
        ble_rx_worker(rx_stack, notifications, rx_notify).await;
    });

    // Spawn TX worker
    tokio::spawn(async move {
        ble_tx_worker(queue, rx_char).await;
    });

    info!("BLE NUS transport registered");
    Ok(stack)
}

/// RX worker: reads NUS TX notifications and feeds them to ergot.
async fn ble_rx_worker(
    stack: EdgeStack,
    mut notifications: impl futures::Stream<Item = Result<Vec<u8>, bluest::Error>> + Unpin + Send,
    state_notify: Option<Arc<WaitQueue>>,
) {
    let mut processor = EdgeFrameProcessor::new_controller(1);

    loop {
        match notifications.next().await {
            Some(Ok(data)) => {
                debug!("[ble rx] {} bytes", data.len());
                let changed = processor.process_frame(&data, &stack, ());
                if changed {
                    if let Some(ref n) = state_notify {
                        n.wake_all();
                    }
                }
            }
            Some(Err(e)) => {
                error!("[ble rx] notification error: {:?}", e);
                break;
            }
            None => {
                info!("[ble rx] notification stream ended");
                break;
            }
        }
    }

    stack.manage_profile(|im| {
        let _ = im.set_interface_state((), InterfaceState::Down);
    });
    if let Some(ref n) = state_notify {
        n.wake_all();
    }
    warn!("[ble rx] worker exited");
}

/// TX worker: reads framed ergot data from bbqueue and writes to NUS RX char.
async fn ble_tx_worker(queue: StdQueue, rx_char: Characteristic) {
    use ergot::exports::bbqueue::traits::bbqhdl::BbqHandle;
    let consumer: ergot::exports::bbqueue::prod_cons::framed::FramedConsumer<StdQueue> =
        queue.framed_consumer();

    loop {
        let frame = consumer.wait_read().await;
        debug!("[ble tx] {} bytes", frame.len());
        if let Err(e) = rx_char.write_without_response(&frame).await {
            error!("[ble tx] write error: {:?}", e);
            frame.release();
            break;
        }
        frame.release();
    }

    warn!("[ble tx] worker exited");
}

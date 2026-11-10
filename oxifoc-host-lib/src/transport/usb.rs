//! USB transport via nusb bulk endpoints.
//!
//! USB is packet-framed (no COBS needed). Uses ergot's `nusb_v0_1` toolkit
//! with the DirectEdge profile — the host is an edge device connecting to a
//! device-side Router. The stack is created once and survives reconnects;
//! each (re)connection re-discovers the device and registers it onto the
//! stack (only succeeds while the interface is Down, i.e. after a teardown).

use crate::raw_telemetry::{RawIngress, RawTelemetryState};
use anyhow::{Context, Result, anyhow, bail};
use ergot::interface_manager::InterfaceState;
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::utils::std::StdQueue;
use ergot::toolkits::nusb_v0_1::{EdgeStack, find_new_devices, register_edge_interface};
use std::collections::HashSet;
use std::sync::Arc;
use tokio_util::sync::CancellationToken;
use tracing::info;

const MTU: u16 = 512;
const OUT_BUFFER_SIZE: usize = 4096;

pub(crate) struct RawUsbConnection {
    cancel: CancellationToken,
    task: Option<tokio::task::JoinHandle<()>>,
}

impl RawUsbConnection {
    pub(crate) async fn shutdown(mut self) {
        self.cancel.cancel();
        if let Some(task) = self.task.take() {
            let _ = task.await;
        }
    }
}

impl Drop for RawUsbConnection {
    fn drop(&mut self) {
        self.cancel.cancel();
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}

/// USB descriptor identity used to narrow reconnect enumeration. The host
/// additionally pins the firmware HardwareInfo UUID after the handshake, so
/// duplicate board-level USB serial strings cannot switch controllers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UsbIdentity {
    serial_number: Option<String>,
    manufacturer: Option<String>,
    product: Option<String>,
}

impl UsbIdentity {
    fn from_device(device: &ergot::toolkits::nusb_v0_1::NewDevice) -> Self {
        Self {
            serial_number: device.info.usb_serial_number.clone(),
            manufacturer: device.info.usb_manufacturer.clone(),
            product: device.info.usb_product.clone(),
        }
    }

    fn description(&self) -> String {
        format!(
            "product={:?}, manufacturer={:?}, serial={:?}",
            self.product, self.manufacturer, self.serial_number
        )
    }
}

pub fn new_stack() -> (EdgeStack, StdQueue) {
    let queue = ergot::interface_manager::utils::std::new_std_queue(OUT_BUFFER_SIZE);
    let stack = EdgeStack::new_with_profile(DirectEdge::new_target(
        framed_stream::Sink::new_from_handle(queue.clone(), MTU),
    ));
    (stack, queue)
}

pub async fn register(
    stack: &EdgeStack,
    queue: &StdQueue,
    state_notify: Option<Arc<ergot::toolkits::tokio_stream::WaitQueue>>,
    selected_identity: &mut Option<UsbIdentity>,
) -> Result<()> {
    info!("Searching for ergot USB devices...");

    let devices = find_new_devices(&HashSet::new()).await;
    let expected_identity = selected_identity.clone();
    let mut matching = devices.into_iter().filter(|device| {
        expected_identity
            .as_ref()
            .is_none_or(|selected| UsbIdentity::from_device(device) == *selected)
    });
    let device = matching.next().ok_or_else(|| match selected_identity {
        Some(identity) => anyhow!(
            "Previously selected ergot USB device not found ({})",
            identity.description()
        ),
        None => anyhow!("No ergot USB device found"),
    })?;
    if matching.next().is_some() {
        return Err(match selected_identity {
            Some(identity) => anyhow!(
                "Multiple ergot USB devices match the selected identity ({}); refusing an ambiguous connection",
                identity.description()
            ),
            None => anyhow!(
                "Multiple ergot USB devices found; refusing to select a controller arbitrarily"
            ),
        });
    }

    let identity = UsbIdentity::from_device(&device);
    if selected_identity.is_none() {
        info!("Pinned USB device identity: {}", identity.description());
        *selected_identity = Some(identity);
    }

    info!(
        "Found USB device: {:?}",
        device.info.usb_product.as_deref().unwrap_or("unknown")
    );

    register_edge_interface(
        stack,
        device,
        queue,
        EdgeFrameProcessor::new(),
        InterfaceState::Active {
            net_id: 0,
            node_id: EDGE_NODE_ID,
        },
        MTU,
        state_notify,
    )
    .await
    .map_err(|_| anyhow!("USB interface not in Down state"))?;

    info!("USB device registered (DirectEdge)");
    Ok(())
}

/// Claim both functions from one device handle, before handing control queues
/// to Ergot. No second enumeration can silently pair an identical board serial.
pub(crate) async fn register_raw(
    stack: &EdgeStack,
    queue: &StdQueue,
    state_notify: Option<Arc<ergot::toolkits::tokio_stream::WaitQueue>>,
    selected_identity: &mut Option<UsbIdentity>,
    raw: Arc<RawTelemetryState>,
    host_cancel: CancellationToken,
) -> Result<RawUsbConnection> {
    use ergot::interface_manager::interface_impls::nusb_bulk::{DeviceInfo, NewDevice};
    use nusb::transfer::{Direction, EndpointType, RequestBuffer};
    let mut devices = nusb::list_devices()
        .context("enumerate raw USB controller")?
        .filter(ergot::toolkits::nusb_v0_1::coarse_device_filter)
        .filter(|device| {
            selected_identity.as_ref().is_none_or(|identity| {
                identity.serial_number.as_deref() == device.serial_number()
                    && identity.manufacturer.as_deref() == device.manufacturer_string()
                    && identity.product.as_deref() == device.product_string()
            })
        });
    let found = devices
        .next()
        .context("no matching Ergot USB controller for raw telemetry")?;
    if devices.next().is_some() {
        bail!("multiple matching raw USB controllers; refusing ambiguous selection");
    }
    let info = DeviceInfo {
        usb_serial_number: found.serial_number().map(str::to_owned),
        usb_manufacturer: found.manufacturer_string().map(str::to_owned),
        usb_product: found.product_string().map(str::to_owned),
    };
    let device = found.open().context("open raw USB controller")?;
    let control = device
        .claim_interface(0)
        .context("claim Ergot interface0")?;
    let alt = control
        .descriptors()
        .find(|alt| {
            alt.alternate_setting() == 0
                && alt.class() == 0xff
                && alt.subclass() == 0xca
                && alt.protocol() == 0x7d
        })
        .context("interface0 does not have the Ergot descriptor")?;
    let mut incoming = None;
    let mut outgoing = None;
    for endpoint in alt
        .endpoints()
        .filter(|ep| ep.transfer_type() == EndpointType::Bulk)
    {
        match endpoint.direction() {
            Direction::In if incoming.is_none() => incoming = Some(endpoint.address()),
            Direction::Out if outgoing.is_none() => {
                outgoing = Some((endpoint.address(), endpoint.max_packet_size()));
            }
            _ => bail!("ambiguous Ergot control bulk endpoints"),
        }
    }
    let (out, mps) = outgoing.context("missing Ergot bulk OUT")?;
    let incoming = incoming.context("missing Ergot bulk IN")?;
    let data = device
        .claim_interface(1)
        .context("firmware does not expose raw telemetry interface1")?;
    let raw_alt = data
        .descriptors()
        .find(|alt| {
            alt.alternate_setting() == 0
                && alt.class() == 0xff
                && alt.subclass() == 0xca
                && alt.protocol() == 0x7e
        })
        .context("unsupported raw USB interface descriptor")?;
    let endpoints: Vec<_> = raw_alt.endpoints().collect();
    if endpoints.len() != 1
        || endpoints[0].address() != 0x82
        || endpoints[0].transfer_type() != EndpointType::Bulk
        || endpoints[0].direction() != Direction::In
        || endpoints[0].max_packet_size() != 64
    {
        bail!("unsupported raw USB endpoint layout");
    }
    let ergot = NewDevice {
        info,
        biq: control.bulk_in_queue(incoming),
        boq: control.bulk_out_queue(out),
        max_packet_size: Some(mps),
    };
    let identity = UsbIdentity::from_device(&ergot);
    if selected_identity.is_none() {
        *selected_identity = Some(identity);
    }
    let mut reads = data.bulk_in_queue(0x82);
    const READ_BYTES: usize = 4096;
    for _ in 0..32 {
        reads.submit(RequestBuffer::new(READ_BYTES));
    }
    let cancel = host_cancel.child_token();
    let task_cancel = cancel.clone();
    let task = tokio::spawn(async move {
        let mut ingress = RawIngress::new(raw.clone());
        raw.reader_ready();
        let result:Result<()> = async {
            let mut ticker=tokio::time::interval(std::time::Duration::from_millis(100));
            ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tokio::select! {
                    _=task_cancel.cancelled()=>return Ok(()),
                    _=ticker.tick()=>raw.check()?,
                    completion=reads.next_complete()=>{
                        completion.status.map_err(|error|anyhow!("raw USB transfer failed: {error:?}"))?;
                        ingress.push(&completion.data)?;
                        reads.submit(RequestBuffer::reuse(completion.data,READ_BYTES));
                    }
                }
            }
        }.await;
        if let Err(error) = result {
            raw.connection_lost(error);
        }
    });
    let connection = RawUsbConnection {
        cancel,
        task: Some(task),
    };
    if register_edge_interface(
        stack,
        ergot,
        queue,
        EdgeFrameProcessor::new(),
        InterfaceState::Active {
            net_id: 0,
            node_id: EDGE_NODE_ID,
        },
        MTU,
        state_notify,
    )
    .await
    .is_err()
    {
        connection.shutdown().await;
        bail!("USB control interface was not Down");
    }
    Ok(connection)
}

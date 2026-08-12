//! USB transport via nusb bulk endpoints.
//!
//! USB is packet-framed (no COBS needed). Uses ergot's `nusb_v0_1` toolkit
//! with the DirectEdge profile — the host is an edge device connecting to a
//! device-side Router. The stack is created once and survives reconnects;
//! each (re)connection re-discovers the device and registers it onto the
//! stack (only succeeds while the interface is Down, i.e. after a teardown).

use anyhow::{Result, anyhow};
use ergot::interface_manager::InterfaceState;
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::interface_manager::utils::std::StdQueue;
use ergot::toolkits::nusb_v0_1::{EdgeStack, find_new_devices, register_edge_interface};
use std::collections::HashSet;
use std::sync::Arc;
use tracing::info;

const MTU: u16 = 512;
const OUT_BUFFER_SIZE: usize = 4096;

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

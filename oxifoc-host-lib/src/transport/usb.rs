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
) -> Result<()> {
    info!("Searching for ergot USB devices...");

    let devices = find_new_devices(&HashSet::new()).await;
    let device = devices
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("No ergot USB device found"))?;

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

//! USB transport via nusb bulk endpoints.
//!
//! USB is packet-framed (no COBS needed). Uses ergot's `nusb_v0_1` toolkit
//! with the DirectEdge profile — the host is an edge device connecting to a
//! device-side Router.

use anyhow::{Result, anyhow};
use ergot::interface_manager::InterfaceState;
use ergot::interface_manager::profiles::direct_edge::{
    DirectEdge, EDGE_NODE_ID, EdgeFrameProcessor,
};
use ergot::interface_manager::utils::framed_stream;
use ergot::toolkits::nusb_v0_1::{EdgeStack, find_new_devices, register_edge_interface};
use std::collections::HashSet;
use std::sync::Arc;
use tracing::info;

const MTU: u16 = 512;
const OUT_BUFFER_SIZE: usize = 4096;

pub async fn connect(
    state_notify: Option<Arc<ergot::toolkits::tokio_stream::WaitQueue>>,
) -> Result<EdgeStack> {
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

    let queue = ergot::interface_manager::utils::std::new_std_queue(OUT_BUFFER_SIZE);
    let stack = EdgeStack::new_with_profile(DirectEdge::new_target(
        framed_stream::Sink::new_from_handle(queue.clone(), MTU),
    ));

    register_edge_interface(
        &stack,
        device,
        &queue,
        EdgeFrameProcessor::new(),
        InterfaceState::Active {
            net_id: 0,
            node_id: EDGE_NODE_ID,
        },
        MTU,
        state_notify,
    )
    .await
    .map_err(|_| anyhow!("Failed to register USB edge interface"))?;

    info!("USB device registered (DirectEdge)");
    Ok(stack)
}

//! USB transport via nusb bulk endpoints.
//!
//! USB is packet-framed (no COBS needed). Uses ergot's `nusb_v0_1` toolkit
//! with the Router profile (handles single device fine, also supports multi-device).

use anyhow::{Result, anyhow};
use ergot::toolkits::nusb_v0_1::{RouterStack, find_new_devices, register_router_interface};
use std::collections::HashSet;
use tracing::info;

const MTU: u16 = 512;
const OUT_BUFFER_SIZE: usize = 4096;

pub async fn connect() -> Result<RouterStack> {
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

    let stack: RouterStack = RouterStack::new();
    register_router_interface(&stack, device, MTU, OUT_BUFFER_SIZE)
        .await
        .map_err(|_| anyhow!("Failed to register USB interface (out of net IDs)"))?;

    info!("USB device registered");
    Ok(stack)
}

//! Ergot UDP server — binds a UDP socket and runs protocol servers.
//!
//! Uses DirectEdge target profile. The host connects as controller (node 1),
//! we are the target (node 2). Unlike TCP, UDP is connectionless — a single
//! socket handles all communication.

use core::cell::RefCell;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::interface_manager::profiles::direct_edge::tokio_udp::InterfaceKind;
use ergot::toolkits::tokio_udp::{self as udp_kit, EdgeStack};
use heapless::String;
use tokio::net::UdpSocket;
use tracing::info;

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::fast_telemetry_stream;
use oxifoc_core::state::{MotorControlState, TELEMETRY};
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 512;

pub async fn run(
    port: u16,
    host_addr: &str,
    foc_freq_hz: u32,
    max_current_a: f32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let bind_addr = format!("0.0.0.0:{port}");
    let socket = UdpSocket::bind(&bind_addr).await?;
    socket.connect(host_addr).await?;
    info!("UDP bound on {bind_addr}, connected to {host_addr}");

    let queue = udp_kit::new_std_queue(4096);
    let stack: EdgeStack = udp_kit::new_target_stack(&queue, ERGOT_MTU);

    udp_kit::register_edge_interface(&stack, socket, &queue, InterfaceKind::Target)
        .await
        .map_err(|_| anyhow::anyhow!("UDP interface already active"))?;

    // Build device info
    let mut hw: String<32> = String::new();
    let mut sw: String<32> = String::new();
    let mut mcu: String<32> = String::new();
    let mut uuid: String<32> = String::new();
    let _ = hw.push_str("Virtual-BLDC");
    let _ = sw.push_str("oxifoc-virtual-0.1.0");
    let _ = mcu.push_str("x86_64 (virtual)");
    let _ = uuid.push_str("00000000-virtual");
    let device_info = DeviceInfo {
        hw,
        sw,
        mcu,
        uuid,
        foc_freq_hz,
        max_current_a,
    };

    // Spawn fast telemetry streaming
    tokio::spawn({
        let stack = stack.clone();
        async move {
            fast_telemetry_stream(stack, &TELEMETRY, state_mutex).await;
        }
    });

    // Run protocol servers (blocks forever)
    let endpoints = stack.endpoints();
    run_all_servers_with_config(
        endpoints,
        device_info,
        state_mutex,
        fault_registry,
        runtime_config,
        foc_freq_hz,
    )
    .await;

    Ok(())
}

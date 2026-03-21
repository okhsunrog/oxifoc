//! Ergot TCP server — accepts a single host connection and runs protocol servers.
//!
//! Uses DirectEdge target profile (same topology as real embedded devices).
//! The host connects as controller (node 1), we are the target (node 2).

use core::cell::RefCell;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::toolkits::tokio_stream::{self as stream_kit, EdgeStack, register_target_stream};
use heapless::String;
use tokio::net::TcpListener;
use tracing::info;

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::runtime::streaming::{fast_telemetry_stream, slow_telemetry_stream};
use oxifoc_core::state::{MotorControlState, TELEMETRY};
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 512;

pub async fn run(
    port: u16,
    foc_freq_hz: u32,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;

    info!("Listening on 0.0.0.0:{port}");
    loop {
        let (socket, addr) = listener.accept().await?;
        info!("Client connected: {addr}");

        // Create a fresh ergot edge stack for this connection.
        // We are the target (node 2), host is controller (node 1).
        let queue = stream_kit::new_std_queue(4096);
        let stack: EdgeStack = stream_kit::new_target_stack(&queue, ERGOT_MTU);

        let (rx, tx) = socket.into_split();
        register_target_stream(stack.clone(), rx, tx, queue)
            .await
            .map_err(|_| anyhow::anyhow!("Interface already active"))?;

        // Protocol servers for this connection
        tokio::spawn({
            let endpoints = stack.endpoints();
            async move {
                let mut hw: String<32> = String::new();
                let mut sw: String<32> = String::new();
                let _ = hw.push_str("Virtual-BLDC");
                let _ = sw.push_str("oxifoc-virtual-0.1.0");
                let device_info = DeviceInfo { hw, sw };

                run_all_servers_with_config(
                    endpoints,
                    device_info,
                    state_mutex,
                    fault_registry,
                    runtime_config,
                    foc_freq_hz,
                )
                .await;
            }
        });

        // Telemetry streaming tasks for this connection
        tokio::spawn({
            let stack = stack.clone();
            async move {
                fast_telemetry_stream(stack, &TELEMETRY, state_mutex).await;
            }
        });
        tokio::spawn({
            let stack = stack.clone();
            async move {
                slow_telemetry_stream(stack, state_mutex, fault_registry).await;
            }
        });
    }
}

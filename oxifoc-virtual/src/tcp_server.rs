//! Ergot TCP server — accepts host connections and runs protocol servers.

use core::cell::RefCell;

use anyhow::Result;
use critical_section::Mutex as CriticalSectionMutex;
use ergot::toolkits::tokio_tcp::{RouterStack, register_router_interface};
use heapless::String;
use tokio::net::TcpListener;
use tracing::info;

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const MAX_ERGOT_PACKET_SIZE: u16 = 1024;
const TX_BUFFER_SIZE: usize = 4096;

pub async fn run(
    port: u16,
    state_mutex: &'static CriticalSectionMutex<RefCell<MotorControlState>>,
    fault_registry: &'static FaultRegistry<VirtualFault>,
    runtime_config: &'static CriticalSectionMutex<RefCell<RuntimeConfig>>,
) -> Result<()> {
    let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;
    let stack: RouterStack = RouterStack::new();

    // Spawn protocol servers on this stack
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
            )
            .await;
        }
    });

    info!("Listening on 0.0.0.0:{port}");
    loop {
        let (socket, addr) = listener.accept().await?;
        info!("Client connected: {addr}");
        register_router_interface(&stack, socket, MAX_ERGOT_PACKET_SIZE, TX_BUFFER_SIZE)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to register interface: {e:?}"))?;
    }
}

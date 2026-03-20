//! Ergot TCP server — accepts a single host connection and runs protocol servers.
//!
//! Uses DirectEdge target profile (same topology as real embedded devices).
//! The host connects as controller (node 1), we are the target (node 2).

use core::cell::RefCell;

use anyhow::Result;
use cobs_acc::{CobsAccumulator, FeedResult};
use critical_section::Mutex as CriticalSectionMutex;
use ergot::interface_manager::interface_impls::tokio_serial_cobs::TokioSerialInterface;
use ergot::interface_manager::profiles::direct_edge::DirectEdge;
use ergot::interface_manager::profiles::direct_edge::process_frame as ergot_edge_process_frame;
use ergot::interface_manager::utils::cobs_stream::Sink as ErgotSink;
use ergot::interface_manager::utils::std::new_std_queue;
use ergot::net_stack::ArcNetStack;
use heapless::String;
use mutex::raw_impls::cs::CriticalSectionRawMutex;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tracing::{error, info};

use oxifoc_core::foc::fault::FaultRegistry;
use oxifoc_core::icd::DeviceInfo;
use oxifoc_core::runtime::servers::run_all_servers_with_config;
use oxifoc_core::state::MotorControlState;
use oxifoc_core::storage::RuntimeConfig;

use crate::fault::VirtualFault;

const ERGOT_MTU: u16 = 512;

pub async fn run(
    port: u16,
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
        let queue = new_std_queue(4096);

        type EdgeProfile = DirectEdge<TokioSerialInterface>;
        type EdgeStack = ArcNetStack<CriticalSectionRawMutex, EdgeProfile>;

        let stack: EdgeStack = ArcNetStack::new_with_profile(DirectEdge::new_target(
            ErgotSink::new_from_handle(queue.clone(), ERGOT_MTU),
        ));

        let (mut rx, mut tx) = socket.into_split();

        // RX worker: TCP → COBS decode → ergot stack
        tokio::spawn({
            let stack = stack.clone();
            async move {
                let mut buf = vec![0u8; 2048];
                let mut cobs_acc = CobsAccumulator::new_boxslice((ERGOT_MTU as usize) + 64);
                let mut net_id = None;
                loop {
                    match rx.read(&mut buf).await {
                        Ok(0) => {
                            info!("Client disconnected: {addr}");
                            break;
                        }
                        Ok(count) => {
                            let mut window = &mut buf[..count];
                            while !window.is_empty() {
                                window = match cobs_acc.feed_raw(window) {
                                    FeedResult::Consumed => break,
                                    FeedResult::OverFull(rem) | FeedResult::DecodeError(rem) => rem,
                                    FeedResult::Success { data, remaining }
                                    | FeedResult::SuccessInput { data, remaining } => {
                                        ergot_edge_process_frame(&mut net_id, data, &stack, ());
                                        remaining
                                    }
                                };
                            }
                        }
                        Err(e) => {
                            error!("TCP read error from {addr}: {e:?}");
                            break;
                        }
                    }
                }
            }
        });

        // TX worker: ergot stack → COBS → TCP
        tokio::spawn({
            let tx_queue = queue.clone();
            async move {
                let tx_consumer = tx_queue.stream_consumer();
                loop {
                    let frame = tx_consumer.wait_read().await;
                    let len = frame.len();
                    if len == 0 {
                        frame.release(len);
                        continue;
                    }
                    if let Err(e) = tx.write_all(&frame[..len]).await {
                        error!("TCP write error: {e:?}");
                        frame.release(len);
                        break;
                    }
                    frame.release(len);
                }
            }
        });

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
                )
                .await;
            }
        });
    }
}

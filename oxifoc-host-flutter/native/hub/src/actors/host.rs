//! Host actor - bridges Flutter UI to oxifoc-host-lib

use crate::signals::{
    AdcSample, ConnectRtt, ConnectSerial, ConnectionStatus, Disconnect, ListProbes,
    ListSerialPorts, MotorCommand, MotorCommandType, ProbeInfo, ProbesList, SerialPortInfo,
    SerialPortsList,
};
use async_trait::async_trait;
use messages::prelude::{Actor, Address, Context, Notifiable};
use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::{HostCommand, HostConfig, HostRuntime, TransportType};
use rinf::{DartSignal, RustSignal};
use std::sync::atomic::Ordering;
use std::time::Duration;
use tokio::task::JoinSet;

/// The host actor manages device connection and communication
pub struct HostActor {
    runtime: Option<HostRuntime>,
    _owned_tasks: JoinSet<()>,
}

impl Actor for HostActor {}

impl HostActor {
    pub fn new(self_addr: Address<Self>) -> Self {
        let mut _owned_tasks = JoinSet::new();

        // Spawn listeners for Dart signals
        _owned_tasks.spawn(Self::listen_list_serial_ports(self_addr.clone()));
        _owned_tasks.spawn(Self::listen_list_probes(self_addr.clone()));
        _owned_tasks.spawn(Self::listen_connect_serial(self_addr.clone()));
        _owned_tasks.spawn(Self::listen_connect_rtt(self_addr.clone()));
        _owned_tasks.spawn(Self::listen_disconnect(self_addr.clone()));
        _owned_tasks.spawn(Self::listen_motor_command(self_addr.clone()));

        Self {
            runtime: None,
            _owned_tasks,
        }
    }

    async fn listen_list_serial_ports(mut self_addr: Address<Self>) {
        let receiver = ListSerialPorts::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    async fn listen_list_probes(mut self_addr: Address<Self>) {
        let receiver = ListProbes::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    async fn listen_connect_serial(mut self_addr: Address<Self>) {
        let receiver = ConnectSerial::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    async fn listen_connect_rtt(mut self_addr: Address<Self>) {
        let receiver = ConnectRtt::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    async fn listen_disconnect(mut self_addr: Address<Self>) {
        let receiver = Disconnect::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    async fn listen_motor_command(mut self_addr: Address<Self>) {
        let receiver = MotorCommand::get_dart_signal_receiver();
        while let Some(signal_pack) = receiver.recv().await {
            let _ = self_addr.notify(signal_pack.message).await;
        }
    }

    fn start_adc_streaming(&mut self) {
        if let Some(ref runtime) = self.runtime {
            let adc_rx = runtime.adc_rx.clone();
            self._owned_tasks.spawn(async move {
                while let Ok(sample) = adc_rx.recv() {
                    AdcSample::from(sample).send_signal_to_dart();
                }
            });
        }
    }

    fn start_connection_monitor(&mut self) {
        if let Some(ref runtime) = self.runtime {
            let connected = runtime.connected.clone();
            self._owned_tasks.spawn(async move {
                let mut last_state = false;
                loop {
                    let current = connected.load(Ordering::Relaxed);
                    if current != last_state {
                        ConnectionStatus {
                            connected: current,
                            message: if current {
                                Some("Device connected".to_string())
                            } else {
                                Some("Device disconnected".to_string())
                            },
                        }
                        .send_signal_to_dart();
                        last_state = current;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            });
        }
    }
}

#[async_trait]
impl Notifiable<ListSerialPorts> for HostActor {
    async fn notify(&mut self, _msg: ListSerialPorts, _: &Context<Self>) {
        let ports = oxifoc_host_lib::list_serial_ports();
        SerialPortsList {
            ports: ports
                .into_iter()
                .map(|p| SerialPortInfo {
                    path: p.path,
                    product: p.product,
                    manufacturer: p.manufacturer,
                })
                .collect(),
        }
        .send_signal_to_dart();
    }
}

#[async_trait]
impl Notifiable<ListProbes> for HostActor {
    async fn notify(&mut self, _msg: ListProbes, _: &Context<Self>) {
        let probes = oxifoc_host_lib::list_probes();
        ProbesList {
            probes: probes
                .into_iter()
                .map(|p| ProbeInfo {
                    identifier: p.identifier,
                    vid: p.vid,
                    pid: p.pid,
                    serial_number: p.serial_number,
                    probe_type: p.probe_type,
                })
                .collect(),
        }
        .send_signal_to_dart();
    }
}

#[async_trait]
impl Notifiable<ConnectSerial> for HostActor {
    async fn notify(&mut self, msg: ConnectSerial, _: &Context<Self>) {
        // Disconnect existing connection if any
        if let Some(ref runtime) = self.runtime {
            runtime.shutdown();
        }

        let config = HostConfig {
            transport: Some(TransportType::Serial),
            serial_path: Some(msg.port_path),
            serial_baud: Some(msg.baud_rate),
            probe: None,
            chip: None,
            elf: None,
            stream_defmt: Some(true),
            stream_ergot: Some(true),
        };

        ConnectionStatus {
            connected: false,
            message: Some("Connecting...".to_string()),
        }
        .send_signal_to_dart();

        let runtime = oxifoc_host_lib::start_host(config);
        self.runtime = Some(runtime);
        self.start_adc_streaming();
        self.start_connection_monitor();
    }
}

#[async_trait]
impl Notifiable<ConnectRtt> for HostActor {
    async fn notify(&mut self, msg: ConnectRtt, _: &Context<Self>) {
        // Disconnect existing connection if any
        if let Some(ref runtime) = self.runtime {
            runtime.shutdown();
        }

        let config = HostConfig {
            transport: Some(TransportType::Rtt),
            serial_path: None,
            serial_baud: None,
            probe: Some(msg.probe_id),
            chip: Some(msg.chip),
            elf: None,
            stream_defmt: Some(true),
            stream_ergot: Some(true),
        };

        ConnectionStatus {
            connected: false,
            message: Some("Connecting via RTT...".to_string()),
        }
        .send_signal_to_dart();

        let runtime = oxifoc_host_lib::start_host(config);
        self.runtime = Some(runtime);
        self.start_adc_streaming();
        self.start_connection_monitor();
    }
}

#[async_trait]
impl Notifiable<Disconnect> for HostActor {
    async fn notify(&mut self, _msg: Disconnect, _: &Context<Self>) {
        if let Some(ref runtime) = self.runtime {
            runtime.shutdown();
        }
        self.runtime = None;

        ConnectionStatus {
            connected: false,
            message: Some("Disconnected".to_string()),
        }
        .send_signal_to_dart();
    }
}

#[async_trait]
impl Notifiable<MotorCommand> for HostActor {
    async fn notify(&mut self, msg: MotorCommand, _: &Context<Self>) {
        if let Some(ref runtime) = self.runtime {
            let control_mode = match msg.command {
                MotorCommandType::Stop => ControlMode::Stopped,
                MotorCommandType::Start { iq_target } => ControlMode::CurrentControl {
                    iq_target,
                    id_target: 0.0,
                },
            };
            let _ = runtime.cmd_tx.send(HostCommand::Motor(control_mode));
        }
    }
}

use oxifoc_host_lib::{HostConfig, HostRuntime, TransportType, start_host};
use oxifoc_ui::{Capabilities, ConnectionForm, ConnectionStage, Device, Transport};
use std::{
    sync::{Arc, Mutex, atomic::Ordering, mpsc},
    time::{Duration, Instant},
};

#[derive(Clone)]
pub struct ConnectedSession {
    pub runtime: Arc<HostRuntime>,
    pub form: ConnectionForm,
}
impl PartialEq for ConnectedSession {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.runtime, &other.runtime)
    }
}

#[derive(Clone, Default, PartialEq)]
pub struct Snapshot {
    pub devices: Vec<Device>,
    pub scanning: bool,
    pub scanned: bool,
    pub stage: ConnectionStage,
    pub session: Option<ConnectedSession>,
    /// Successful connections in this app session; no addresses are persisted.
    pub recent: Vec<ConnectionForm>,
}
enum Command {
    Scan(Transport),
    Connect(ConnectionForm),
    Disconnect,
    Quit,
}
pub struct Controller {
    sender: mpsc::Sender<Command>,
    state: Arc<Mutex<Snapshot>>,
    worker: Option<std::thread::JoinHandle<()>>,
}
impl Drop for Controller {
    fn drop(&mut self) {
        let _ = self.sender.send(Command::Quit);
        // Final application teardown must give the backend time to release
        // GATT/probe handles before the process exits. Interactive Disconnect
        // still runs entirely on the worker and never joins on the UI thread.
        if let Some(worker) = self.worker.take() {
            let deadline = Instant::now() + Duration::from_millis(3500);
            while !worker.is_finished() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(10));
            }
            if worker.is_finished() {
                let _ = worker.join();
            }
        }
    }
}
impl Controller {
    pub fn new(capabilities: Capabilities) -> Self {
        let (sender, receiver) = mpsc::channel();
        let state = Arc::new(Mutex::new(Snapshot::default()));
        let shared = state.clone();
        let worker = std::thread::spawn(move || run(receiver, shared, capabilities));
        Self {
            sender,
            state,
            worker: Some(worker),
        }
    }
    pub fn snapshot(&self) -> Snapshot {
        self.state.lock().unwrap().clone()
    }
    pub fn connect(&self, form: ConnectionForm) {
        let _ = self.sender.send(Command::Connect(form));
    }
    pub fn disconnect(&self) {
        let _ = self.sender.send(Command::Disconnect);
    }
    pub fn scan(&self, transport: Transport) {
        self.permission_pending();
        let _ = self.sender.send(Command::Scan(transport));
    }
    pub fn permission_pending(&self) {
        let mut s = self.state.lock().unwrap();
        s.scanning = true;
        s.stage = ConnectionStage::Idle;
    }
    pub fn error(&self, message: String) {
        let mut s = self.state.lock().unwrap();
        s.scanning = false;
        s.stage = ConnectionStage::Error(message);
    }
    pub fn clear_discovery(&self) {
        let mut s = self.state.lock().unwrap();
        s.devices.clear();
        s.scanned = false;
        s.stage = ConnectionStage::Idle;
    }
}

fn config(form: &ConnectionForm, capabilities: Capabilities) -> Result<HostConfig, String> {
    form.validate(capabilities)?;
    let mut config = HostConfig {
        stream_defmt: Some(false),
        ..Default::default()
    };
    match form.transport {
        Transport::Ble => {
            config.transport = Some(TransportType::Ble);
            config.ble_address = Some(form.device.clone());
        }
        Transport::Tcp => {
            config.transport = Some(TransportType::Tcp);
            config.tcp_host = Some(form.host.trim().into());
            config.tcp_port = form.port.parse().ok();
        }
        Transport::Udp => {
            config.transport = Some(TransportType::Udp);
            config.udp_host = Some(form.host.trim().into());
            config.udp_port = form.port.parse().ok();
        }
        Transport::Usb => {
            config.transport = Some(TransportType::Usb);
        }
        #[cfg(feature = "desktop")]
        Transport::Serial => {
            config.transport = Some(TransportType::Serial);
            config.serial_path = Some(form.device.clone());
            config.serial_baud = form.baud.parse().ok();
        }
        Transport::Rtt => {
            return Err("The RTT transport was removed from the host library.".into());
        }
        #[cfg(not(feature = "desktop"))]
        Transport::Serial => {
            return Err("This transport is not included in this build.".into());
        }
    }
    Ok(config)
}

fn scan(transport: Transport) -> Result<Vec<Device>, String> {
    match transport {
        Transport::Ble => {
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .map_err(|e| e.to_string())?;
            let devices = runtime.block_on(oxifoc_host_lib::try_scan_ble_devices(Duration::from_secs(4))).map_err(|e| format!("Bluetooth search failed: {e:#}. Check that Bluetooth is enabled and try again."))?;
            Ok(devices
                .into_iter()
                .map(|d| Device {
                    name: d.name.unwrap_or_else(|| "Unnamed controller".into()),
                    id: d.id,
                    detail: d.rssi.map(|r| format!("{r} dBm")).unwrap_or_default(),
                })
                .collect())
        }
        #[cfg(feature = "desktop")]
        Transport::Serial => Ok(oxifoc_host_lib::list_serial_ports()
            .into_iter()
            .map(|d| Device {
                name: d.product.unwrap_or_else(|| "Serial interface".into()),
                id: d.path,
                detail: d.manufacturer.unwrap_or_default(),
            })
            .collect()),
        #[cfg(feature = "desktop")]
        Transport::Rtt => Ok(oxifoc_host_lib::list_probes()
            .into_iter()
            .map(|d| Device {
                name: d.probe_type,
                id: d.identifier,
                detail: d.serial_number.unwrap_or_default(),
            })
            .collect()),
        _ => Ok(Vec::new()),
    }
}

fn close(runtime: &mut Option<Arc<HostRuntime>>) {
    if let Some(runtime) = runtime.take() {
        runtime.shutdown();
    }
}
fn run(receiver: mpsc::Receiver<Command>, state: Arc<Mutex<Snapshot>>, capabilities: Capabilities) {
    let mut runtime = None::<Arc<HostRuntime>>;
    let mut pending = None::<(ConnectionForm, Instant)>;
    loop {
        match receiver.recv_timeout(Duration::from_millis(40)) {
            Ok(Command::Quit) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
            Ok(Command::Disconnect) => {
                pending = None;
                close(&mut runtime);
                let mut s = state.lock().unwrap();
                s.session = None;
                s.stage = ConnectionStage::Idle;
            }
            Ok(Command::Scan(transport)) => {
                let result = if capabilities.allows(transport) {
                    scan(transport)
                } else {
                    Err("Transport is unavailable on this platform.".into())
                };
                let mut s = state.lock().unwrap();
                s.scanning = false;
                s.scanned = true;
                match result {
                    Ok(devices) => s.devices = devices,
                    Err(message) => s.stage = ConnectionStage::Error(message),
                }
            }
            Ok(Command::Connect(form)) => {
                if pending.is_some() || runtime.is_some() {
                    continue;
                }
                match config(&form, capabilities) {
                    Ok(config) => {
                        state.lock().unwrap().stage = ConnectionStage::Connecting;
                        runtime = Some(Arc::new(start_host(config)));
                        pending = Some((form, Instant::now()));
                    }
                    Err(message) => state.lock().unwrap().stage = ConnectionStage::Error(message),
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
        if let Some((form, started)) = &pending {
            if runtime
                .as_ref()
                .is_some_and(|r| r.connected.load(Ordering::Relaxed))
            {
                let mut s = state.lock().unwrap();
                s.session = Some(ConnectedSession {
                    runtime: runtime.as_ref().unwrap().clone(),
                    form: form.clone(),
                });
                s.stage = ConnectionStage::Idle;
                s.recent.retain(|r| r != form);
                s.recent.insert(0, form.clone());
                s.recent.truncate(4);
                pending = None;
            } else if started.elapsed() > Duration::from_secs(20) {
                pending = None;
                close(&mut runtime);
                state.lock().unwrap().stage = ConnectionStage::Error("The controller did not complete the connection within 20 seconds. Check the device, address and interface, then try again.".into());
            }
        }
    }
    close(&mut runtime);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancel_silent_tcp_peer_allows_a_new_attempt() {
        // The port accepts TCP, but never performs the OxiFOC handshake.
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let form = ConnectionForm {
            transport: Transport::Tcp,
            host: "127.0.0.1".into(),
            port: listener.local_addr().unwrap().port().to_string(),
            ..Default::default()
        };
        let controller = Controller::new(Capabilities::MOBILE);
        let wait_for = |stage| {
            let deadline = Instant::now() + Duration::from_secs(4);
            loop {
                let snapshot = controller.snapshot();
                if snapshot.stage == stage {
                    return snapshot;
                }
                assert!(Instant::now() < deadline, "connection state timed out");
                std::thread::sleep(Duration::from_millis(10));
            }
        };
        for _ in 0..2 {
            controller.connect(form.clone());
            assert!(wait_for(ConnectionStage::Connecting).session.is_none());
            controller.disconnect();
            assert!(wait_for(ConnectionStage::Idle).session.is_none());
        }
        assert!(controller.snapshot().recent.is_empty());
    }
    #[test]
    fn mobile_rejects_usb_even_if_requested_outside_ui() {
        assert!(
            config(
                &ConnectionForm {
                    transport: Transport::Usb,
                    ..Default::default()
                },
                Capabilities::MOBILE
            )
            .is_err()
        );
    }
    #[test]
    fn network_fields_select_the_matching_backend() {
        let f = ConnectionForm {
            transport: Transport::Udp,
            host: "::1".into(),
            port: "25267".into(),
            ..Default::default()
        };
        let c = config(&f, Capabilities::MOBILE).unwrap();
        assert_eq!(c.transport, Some(TransportType::Udp));
        assert_eq!(c.udp_port, Some(25267));
        assert_eq!(c.udp_host.as_deref(), Some("::1"));
        assert!(c.tcp_host.is_none());
    }
}

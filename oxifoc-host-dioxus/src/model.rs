use oxifoc_core::types::{ConfigGroupId, ControlMode};
use oxifoc_core::units::amps;
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, TransportType, motor_channel, ops, start_host,
};
use oxifoc_plot::PlotBuffer;
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

pub const CAPACITY: usize = 32768;
#[derive(Clone, Default)]
pub struct Snapshot {
    pub connected: bool,
    pub connection_generation: u64,
    pub samples: u64,
    pub hz: f64,
    pub rpm: f32,
    pub iq: Option<f32>,
    pub motor_temp: Option<f32>,
    pub fast_seen: Option<Instant>,
    pub slow_seen: Option<Instant>,
    pub volts: f32,
    pub temperature: f32,
    pub state: String,
    pub faults: u8,
    pub events: Vec<String>,
    pub pending: bool,
}
pub struct Model {
    pub runtime: Arc<HostRuntime>,
    pub plots: [PlotBuffer; 4],
    pub paused: AtomicBool,
    pub snapshot: Mutex<Snapshot>,
    pub alive: AtomicBool,
}
impl PartialEq for Model {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::eq(self, other)
    }
}
impl Model {
    pub fn connect() -> Arc<Self> {
        let port = std::env::var("OXIFOC_TCP_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(2025);
        let host = std::env::var("OXIFOC_TCP_HOST").unwrap_or_else(|_| "127.0.0.1".into());
        let ble_address = std::env::var("OXIFOC_BLE_ADDRESS").ok();
        let runtime = start_host(HostConfig {
            transport: Some(if ble_address.is_some() {
                TransportType::Ble
            } else {
                TransportType::Tcp
            }),
            ble_address,
            tcp_host: Some(host),
            tcp_port: Some(port),
            fast_hz: Some(1000),
            stream_defmt: Some(false),
            ..Default::default()
        });
        Self::from_runtime(Arc::new(runtime))
    }
    pub fn from_runtime(runtime: Arc<HostRuntime>) -> Arc<Self> {
        let model = Arc::new(Self {
            runtime,
            plots: [
                PlotBuffer::new(5, CAPACITY),
                PlotBuffer::new(1, CAPACITY),
                PlotBuffer::new(1, CAPACITY),
                PlotBuffer::new(1, CAPACITY),
            ],
            paused: AtomicBool::new(false),
            snapshot: Mutex::new(Snapshot::default()),
            alive: AtomicBool::new(true),
        });
        let weak = Arc::downgrade(&model);
        std::thread::spawn(move || {
            let mut ctx = None;
            let mut connected = false;
            let mut clock = Instant::now();
            let mut count = 0u64;
            loop {
                let Some(m) = weak.upgrade() else { break };
                if !m.alive.load(Ordering::Relaxed) {
                    break;
                }
                let now_connected = m.runtime.connected.load(Ordering::Relaxed);
                if now_connected != connected {
                    connected = now_connected;
                    m.event(if connected {
                        "Controller connected"
                    } else {
                        "Connection lost; waiting to reconnect"
                    });
                    if !connected {
                        ctx = None;
                    }
                }
                while let Ok(info) = m.runtime.device_info_rx.try_recv() {
                    m.snapshot.lock().unwrap().connection_generation += 1;
                    ctx = oxifoc_host_lib::build_enrich_ctx(&m.runtime.cmd_tx, Some(&info));
                    for plot in &m.plots {
                        plot.clear();
                    }
                    m.event("Controller identity and calibration received");
                }
                let paused = m.paused.load(Ordering::Relaxed);
                let mut last = None;
                let mut iq = None;
                for sample in m.runtime.fast_rx.try_iter() {
                    count += 1;
                    if let Some(ctx) = &ctx {
                        let rich = sample.enrich(ctx);
                        iq = Some(rich.iq);
                        if !paused {
                            m.plots[0].push_frame(&[rich.ia, rich.ib, rich.ic, rich.id, rich.iq]);
                            m.plots[1].push_frame(&[f32::from(sample.rpm) * 2.0]);
                        }
                    }
                    last = Some(sample);
                }
                let mut s = m.snapshot.lock().unwrap();
                s.connected = connected;
                if let Some(last) = last {
                    s.rpm = f32::from(last.rpm) * 2.0;
                    s.iq = iq;
                    s.fast_seen = Some(Instant::now());
                }
                for sample in m.runtime.slow_rx.try_iter() {
                    s.slow_seen = Some(Instant::now());
                    s.motor_temp = sample.motor_temp_c_x10.map(|v| v as f32 / 10.);
                    s.volts = sample.vbus_mv as f32 / 1000.;
                    s.temperature = f32::from(sample.fet_temp_c_x10) / 10.;
                    s.state = format!("{:?}", sample.motor_state);
                    s.faults = sample.fault_count;
                    if !paused {
                        m.plots[2].push_frame(&[s.volts]);
                        m.plots[3].push_frame(&[s.temperature]);
                    }
                }
                for faults in m.runtime.fault_rx.try_iter() {
                    s.events.push(format!(
                        "Fault status: {} active; revision {}",
                        faults.total, faults.generation
                    ));
                }
                if clock.elapsed().as_secs_f64() >= 1.0 {
                    s.hz = count as f64 / clock.elapsed().as_secs_f64();
                    s.samples += count;
                    count = 0;
                    clock = Instant::now();
                }
                if s.events.len() > 100 {
                    let n = s.events.len() - 100;
                    s.events.drain(..n);
                }
                drop(s);
                drop(m);
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        model
    }
    pub fn event(&self, text: &str) {
        self.snapshot.lock().unwrap().events.push(text.into());
        eprintln!("native: {text}");
    }
    pub fn motor(self: &Arc<Self>, mode: ControlMode) {
        {
            let mut s = self.snapshot.lock().unwrap();
            if !s.connected || (s.pending && !matches!(mode, ControlMode::Stopped)) {
                return;
            }
            s.pending = true;
        }
        let m = self.clone();
        std::thread::spawn(move || {
            let (tx, rx) = motor_channel();
            let result = m
                .runtime
                .cmd_tx
                .send(HostCommand::MotorAck(mode, tx))
                .map_err(|e| e.to_string())
                .and_then(|_| rx.blocking_recv().map_err(|e| e.to_string()))
                .and_then(|r| r.map_err(|e| e.to_string()));
            let label = match mode {
                ControlMode::Stopped => "Stop drive".to_string(),
                ControlMode::CurrentControl { iq_target, .. } => {
                    format!("Apply Iq {:.2} A", iq_target.value)
                }
                _ => format!("{mode:?}"),
            };
            m.event(&match result {
                Ok(status) => format!("{label}: {:?}; state {:?}", status.outcome, status.state),
                Err(e) => format!("{label} failed: {e}"),
            });
            m.snapshot.lock().unwrap().pending = false;
        });
    }
    pub fn configuration_connection(&self) -> Option<u64> {
        let snapshot = self.snapshot.lock().unwrap();
        (snapshot.connected && self.runtime.connected.load(Ordering::Relaxed))
            .then_some(snapshot.connection_generation)
    }
}
impl Drop for Model {
    fn drop(&mut self) {
        self.alive.store(false, Ordering::Relaxed);
        self.runtime.shutdown();
    }
}

/// Explicit integration probe for a separately launched virtual controller.
pub fn verify_virtual() -> anyhow::Result<()> {
    anyhow::ensure!(
        std::env::var("OXIFOC_BLE_ADDRESS").is_err()
            && std::env::var("OXIFOC_TCP_HOST").unwrap_or_else(|_| "127.0.0.1".into())
                == "127.0.0.1",
        "verification requires loopback"
    );
    anyhow::ensure!(
        std::env::var("OXIFOC_TCP_PORT").ok().as_deref() == Some("25265"),
        "launch a dedicated oxifoc-virtual on port 25265"
    );
    let model = Model::connect();
    let result = (|| -> anyhow::Result<()> {
        anyhow::ensure!(
            model.runtime.wait_for_connection(Duration::from_secs(15)),
            "connection timed out"
        );
        let (value, state) =
            ops::config::current_value(&model.runtime.cmd_tx, ConfigGroupId::MotorParams)?;
        anyhow::ensure!(value.is_object(), "motor params absent");
        let (tx, rx) = motor_channel();
        model.runtime.cmd_tx.send(HostCommand::MotorAck(
            ControlMode::CurrentControl {
                iq_target: amps(1.),
                id_target: amps(0.),
            },
            tx,
        ))?;
        let ack = rx.blocking_recv()??;
        std::thread::sleep(Duration::from_secs(3));
        let s = model.snapshot.lock().unwrap().clone();
        anyhow::ensure!(
            s.samples > 100 && model.plots[0].available_samples() > 100,
            "no calibrated telemetry"
        );
        anyhow::ensure!(s.rpm.abs() > 1., "virtual rotor did not move");
        eprintln!(
            "VERIFIED samples={} rpm={} volts={} config_revision={} command={ack:?}",
            s.samples, s.rpm, s.volts, state.revision
        );
        Ok(())
    })();
    let (tx, rx) = motor_channel();
    let stop = model
        .runtime
        .cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx));
    if stop.is_ok() {
        eprintln!("VERIFY STOP {:?}", rx.blocking_recv());
    }
    model.alive.store(false, Ordering::Relaxed);
    model.runtime.shutdown();
    result
}

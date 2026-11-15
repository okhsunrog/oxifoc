use oxifoc_core::types::ControlMode;
use oxifoc_host_lib::{
    HostCommand, HostConfig, HostRuntime, TransportType, motor_channel, start_host,
};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
struct Live {
    samples: Vec<f32>,
    status: String,
}
pub struct Source {
    runtime: Option<HostRuntime>,
    data: Arc<Mutex<Live>>,
    alive: Arc<AtomicBool>,
    clock: Instant,
    sample: u64,
}
impl Source {
    pub fn new(bench: bool) -> Self {
        let data = Arc::new(Mutex::new(Live {
            samples: Vec::new(),
            status: "Waiting for controller".into(),
        }));
        let alive = Arc::new(AtomicBool::new(true));
        let runtime = if bench {
            None
        } else {
            let port = std::env::var("OXIFOC_TCP_PORT")
                .ok()
                .and_then(|p| p.parse().ok())
                .unwrap_or(25266);
            let runtime = start_host(HostConfig {
                transport: Some(TransportType::Tcp),
                tcp_host: Some("127.0.0.1".into()),
                tcp_port: Some(port),
                fast_hz: Some(1000),
                stream_defmt: Some(false),
                ..Default::default()
            });
            let (info, fast, slow, faults, cmd, connected) = (
                runtime.device_info_rx.clone(),
                runtime.fast_rx.clone(),
                runtime.slow_rx.clone(),
                runtime.fault_rx.clone(),
                runtime.cmd_tx.clone(),
                runtime.connected.clone(),
            );
            let (data, alive) = (data.clone(), alive.clone());
            std::thread::spawn(move || {
                let mut ctx = None;
                let mut rpm = 0.;
                let mut volts = 0.;
                let mut state = String::from("Waiting");
                let mut fault_count = 0;
                let mut samples = 0u64;
                while alive.load(Ordering::Relaxed) {
                    if let Ok(info) = info.try_recv() {
                        ctx = oxifoc_host_lib::build_enrich_ctx(&cmd, Some(&info));
                        println!("[desktop-source] calibration={}", ctx.is_some());
                    }
                    let mut batch = Vec::new();
                    for s in fast.try_iter() {
                        if let Some(ctx) = &ctx {
                            let rich = s.enrich(ctx);
                            batch.extend_from_slice(&[rich.ia, rich.ib, rich.ic]);
                            samples += 1;
                            rpm = f32::from(s.rpm) * 2.;
                        }
                    }
                    for s in slow.try_iter() {
                        volts = s.vbus_mv as f32 / 1000.;
                        state = format!("{:?}", s.motor_state);
                        fault_count = s.fault_count;
                    }
                    for fault in faults.try_iter() {
                        println!("[desktop-source] fault={fault:?}");
                    }
                    let mut d = data.lock().unwrap();
                    if d.samples.len() + batch.len() > 65536 * 3 {
                        d.samples.clear();
                    }
                    d.samples.extend(batch);
                    d.status = format!(
                        "{} · {state} · {rpm:.0} RPM · {volts:.1} V · faults {fault_count} · {samples} samples",
                        if connected.load(Ordering::Relaxed) {
                            "Virtual controller"
                        } else {
                            "Disconnected"
                        }
                    );
                    drop(d);
                    std::thread::sleep(Duration::from_millis(4));
                }
            });
            Some(runtime)
        };
        Self {
            runtime,
            data,
            alive,
            clock: Instant::now(),
            sample: 0,
        }
    }
    pub fn next(&mut self) -> serde_json::Value {
        if self.runtime.is_some() {
            let mut d = self.data.lock().unwrap();
            let samples = std::mem::take(&mut d.samples);
            return serde_json::json!({"samples":samples,"status":d.status});
        }
        let target = 65536 + (self.clock.elapsed().as_secs_f64() * 12000.) as u64;
        let count = (target - self.sample).min(3000);
        let mut data = Vec::with_capacity(count as usize * 3);
        for _ in 0..count {
            let t = self.sample as f64 / 12000.;
            let amplitude = (0.6 + 0.65_f32 as f64 * 2.7) * (1. + 0.08 * (t * 0.8).sin());
            for ch in 0..3 {
                let p = t * 38. * std::f64::consts::TAU + ch as f64 * std::f64::consts::TAU / 3.;
                data.push((amplitude * (p.sin() + 0.05 * (p * 9.).sin())) as f32);
            }
            self.sample += 1;
        }
        serde_json::json!({"samples":data,"status":"Benchmark · 12,000 samples/s · 3 channels · ±4 A"})
    }
    pub fn command(&self, command: &str) {
        if let Some(runtime) = &self.runtime {
            let cmd = runtime.cmd_tx.clone();
            let mode = if command == "start" {
                ControlMode::CurrentControl {
                    iq_target: 1.,
                    id_target: 0.,
                }
            } else {
                ControlMode::Stopped
            };
            std::thread::spawn(move || {
                let (tx, rx) = motor_channel();
                if cmd.send(HostCommand::MotorAck(mode, tx)).is_ok() {
                    println!("[desktop-command] {:?}", rx.blocking_recv());
                }
            });
        }
    }
}
impl Drop for Source {
    fn drop(&mut self) {
        self.alive.store(false, Ordering::Relaxed);
        if let Some(r) = &self.runtime {
            r.shutdown();
        }
    }
}

//! End-to-end test: spawn the `oxifoc-virtual` Router and drive it with
//! `oxifoc-host-lib`, exercising the delivery layer against a real (simulated)
//! device — no hardware required, so it runs in CI.
//!
//! Runs over both transports (the device is a `Router` either way):
//! - TCP: COBS over a connected stream,
//! - UDP: datagrams; the device binds an unconnected socket and learns the
//!   host's address from the first datagram (ergot UDP peer learning).
//!
//! Covers:
//! - HardwareInfo handshake (request/response over the Router),
//! - `at_least_once` Motor setpoint (the sim spins up in response),
//! - `effectively_once` Detect (a `Keyed` request the device deduplicates).
//!
//! `start_host` runs its own tokio runtime on a background thread, so these are
//! plain `#[test]`s that drive the runtime via its (sync) channels.

use std::net::{TcpListener, UdpSocket};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use oxifoc_core::types::{ControlMode, DetectRequest, DetectResponse};
use oxifoc_host_lib::{
    HostCommand, HostConfig, ReconnectPolicy, TransportType, detect_channel, start_host,
};

/// Kills the spawned virtual device when the test ends (even on panic).
struct ChildGuard(Child);
impl Drop for ChildGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Grab an ephemeral free TCP port (closed immediately; small TOCTOU window).
fn free_tcp_port() -> u16 {
    TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// Grab an ephemeral free UDP port (closed immediately; small TOCTOU window).
fn free_udp_port() -> u16 {
    UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

/// Poll `cond` until it returns true or the timeout elapses.
fn wait_until(timeout: Duration, mut cond: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if cond() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

/// Spawn the virtual device and drive handshake + Motor + Detect over the given
/// transport, asserting each step. Shared by the TCP and UDP tests below.
fn run_e2e(transport: TransportType) {
    let (transport_arg, port) = match transport {
        TransportType::Tcp => ("tcp", free_tcp_port()),
        TransportType::Udp => ("udp", free_udp_port()),
        other => panic!("unsupported transport in e2e: {other:?}"),
    };

    // Spawn the virtual device as a Router on the chosen port/transport.
    let child = Command::new(env!("CARGO_BIN_EXE_oxifoc-virtual"))
        .args([
            "--transport",
            transport_arg,
            "--port",
            &port.to_string(),
            "--vbus",
            "24",
            "--pole-pairs",
            "7",
            "--max-current",
            "20",
        ])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn oxifoc-virtual");
    let _guard = ChildGuard(child);

    // Connect the host backend. `stream_defmt = false` (no device ELF for
    // virtual); the host retries the connect until the device binds.
    let cfg = HostConfig {
        transport: Some(transport),
        tcp_host: Some("127.0.0.1".to_string()),
        tcp_port: Some(port),
        udp_host: Some("127.0.0.1".to_string()),
        udp_port: Some(port),
        stream_defmt: Some(false),
        stream_ergot: Some(true),
        fast_hz: Some(500),
        reconnect: Some(ReconnectPolicy::Limited(20)),
        ..Default::default()
    };
    let rt = start_host(cfg);

    // 1) Handshake: the host reports connected once HardwareInfo round-trips.
    assert!(
        rt.wait_for_connection(Duration::from_secs(15)),
        "[{transport_arg}] device should connect (HardwareInfo handshake)"
    );

    // 2) Motor at_least_once: command a current setpoint; the sim should spin.
    rt.cmd_tx
        .send(HostCommand::Motor(ControlMode::CurrentControl {
            iq_target: 4.0,
            id_target: 0.0,
        }))
        .expect("send motor command");
    let spun = wait_until(Duration::from_secs(8), || {
        // Drain whatever fast-telemetry samples are buffered; spinning ⇒ erpm != 0.
        let mut moving = false;
        while let Ok(sample) = rt.fast_rx.try_recv() {
            if sample.erpm != 0 {
                moving = true;
            }
        }
        moving
    });
    assert!(
        spun,
        "[{transport_arg}] motor should spin (erpm != 0) after the at_least_once Motor command"
    );

    // 3) Detect effectively_once: routes via Reliable::effectively_once (Keyed
    //    request + device-side dedup) and returns a measured resistance.
    let (tx, mut rx) = detect_channel();
    rt.cmd_tx
        .send(HostCommand::Detect(
            DetectRequest::MeasureResistance {
                max_power_loss_w: 8.0,
            },
            tx,
        ))
        .expect("send detect command");
    let deadline = Instant::now() + Duration::from_secs(30);
    let result = loop {
        if let Ok(res) = rx.try_recv() {
            break res;
        }
        if Instant::now() >= deadline {
            panic!("[{transport_arg}] detect did not respond within 30s");
        }
        std::thread::sleep(Duration::from_millis(100));
    };
    let resp = result.expect("detect should succeed");
    assert!(
        matches!(resp, DetectResponse::Resistance { .. }),
        "[{transport_arg}] expected a Resistance result, got {resp:?}"
    );

    rt.shutdown();
}

#[test]
fn e2e_motor_and_detect_over_tcp() {
    run_e2e(TransportType::Tcp);
}

#[test]
fn e2e_motor_and_detect_over_udp() {
    run_e2e(TransportType::Udp);
}

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

use std::net::{TcpListener, TcpStream, UdpSocket};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use oxifoc_core::storage::FailsafeConfigStored;
use oxifoc_core::types::{
    ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, ConfigValue, ConfigWrite,
    ControlMode, CurrentOffsetMethod, DetectRequest, DetectResponse, Keyed, MotorCommandOutcome,
    ReqId,
};
use oxifoc_host_lib::{
    HostCommand, HostConfig, ReconnectPolicy, TransportType, config_channel, detect_channel,
    motor_channel, start_host,
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
    let hw = {
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut latest = None;
        loop {
            while let Ok(info) = rt.device_info_rx.try_recv() {
                latest = Some(info);
            }
            if let Some(info) = latest {
                break info;
            }
            if Instant::now() >= deadline {
                panic!("[{transport_arg}] host did not receive HardwareInfo");
            }
            std::thread::sleep(Duration::from_millis(25));
        }
    };
    let enrich = oxifoc_host_lib::build_enrich_ctx(&rt.cmd_tx, Some(&hw))
        .expect("virtual device should provide enrichment context");

    // 2) Motor at_least_once: command a current setpoint; the sim should spin.
    rt.cmd_tx
        .send(HostCommand::Motor(ControlMode::CurrentControl {
            iq_target: 4.0,
            id_target: 0.0,
        }))
        .expect("send motor command");
    let spun = wait_until(Duration::from_secs(8), || {
        // Drain whatever fast-telemetry samples are buffered. The raw frame
        // carries mechanical RPM; host enrichment reconstructs eRPM.
        let mut moving = false;
        while let Ok(sample) = rt.fast_rx.try_recv() {
            let rich = sample.enrich(&enrich);
            if sample.rpm != 0
                && rich.erpm.abs() > 1.0
                && (rich.erpm - rich.mech_rpm * 7.0).abs() < 20.0
            {
                moving = true;
            }
        }
        moving
    });
    assert!(
        spun,
        "[{transport_arg}] motor should spin with coherent mechanical RPM and enriched eRPM"
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

    // 4) The optional offset diagnostic shares the same effectively-once
    // endpoint but returns raw ADC-domain calibration values.
    let offsets = oxifoc_host_lib::ops::detect::measure_current_offsets(
        &rt.cmd_tx,
        CurrentOffsetMethod::PerPhase50,
        1000,
        false,
    )
    .expect("offset diagnostic should succeed");
    assert_eq!(offsets.offsets, [2047.5; 3]);

    // 5) EmergencyStop shares the motor endpoint, floats immediately, and
    // latches active drive until an explicit safe neutral command.
    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::EmergencyStop(tx))
        .expect("send emergency stop");
    let emergency = rx
        .blocking_recv()
        .expect("emergency response channel")
        .expect("emergency stop should apply");
    assert_eq!(emergency.outcome, MotorCommandOutcome::Applied);
    assert_eq!(emergency.mode, ControlMode::Stopped);

    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            },
            tx,
        ))
        .expect("send latched drive request");
    assert!(
        rx.blocking_recv()
            .expect("latched response channel")
            .is_err(),
        "active drive must remain rejected until neutral"
    );

    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx))
        .expect("send safe re-arm acknowledgement");
    assert!(
        rx.blocking_recv().expect("safe response channel").is_ok(),
        "safe neutral must release the emergency latch"
    );

    // 6) Config mutation is a revisioned two-phase operation. A retried Apply
    // is deduplicated, a stale writer conflicts, and Persist marks only the
    // exact live revision durable.
    let (tx, rx) = config_channel();
    rt.cmd_tx
        .send(HostCommand::ConfigRead(ConfigGroupId::Failsafe, tx))
        .expect("send initial config read");
    let initial = rx
        .blocking_recv()
        .expect("initial config response channel")
        .expect("initial config read");
    assert_eq!(
        initial,
        ConfigResponse::Snapshot(oxifoc_core::types::ConfigSnapshot {
            group: ConfigGroupId::Failsafe,
            revision: 0,
            persisted: false,
            value: None,
        })
    );

    let apply = Keyed::new(
        ReqId(0xA11E_0001),
        ConfigApply {
            expected_revision: 0,
            write: ConfigWrite::Failsafe(FailsafeConfigStored::default()),
        },
    );
    for attempt in 0..2 {
        let (tx, rx) = config_channel();
        rt.cmd_tx
            .send(HostCommand::ConfigApply(apply.clone(), tx))
            .expect("send config apply");
        assert_eq!(
            rx.blocking_recv()
                .expect("apply response channel")
                .expect("config apply"),
            ConfigResponse::Applied {
                req_id: apply.id,
                revision: 1,
            },
            "apply attempt {attempt} must return the same deduplicated acknowledgement"
        );
    }

    let (tx, rx) = config_channel();
    rt.cmd_tx
        .send(HostCommand::ConfigApply(
            Keyed::new(
                ReqId(0xA11E_0002),
                ConfigApply {
                    expected_revision: 0,
                    write: ConfigWrite::Failsafe(FailsafeConfigStored::default()),
                },
            ),
            tx,
        ))
        .expect("send stale config apply");
    assert_eq!(
        rx.blocking_recv()
            .expect("stale apply response channel")
            .expect("stale apply protocol response"),
        ConfigResponse::Conflict {
            current_revision: 1,
        }
    );

    let persist = Keyed::new(
        ReqId(0xA11E_0003),
        ConfigPersist {
            group: ConfigGroupId::Failsafe,
            expected_revision: 1,
        },
    );
    for attempt in 0..2 {
        let (tx, rx) = config_channel();
        rt.cmd_tx
            .send(HostCommand::ConfigPersist(persist.clone(), tx))
            .expect("send config persist");
        assert_eq!(
            rx.blocking_recv()
                .expect("persist response channel")
                .expect("config persist"),
            ConfigResponse::Persisted {
                req_id: persist.id,
                revision: 1,
            },
            "persist attempt {attempt} must return the same deduplicated acknowledgement"
        );
    }

    let (tx, rx) = config_channel();
    rt.cmd_tx
        .send(HostCommand::ConfigRead(ConfigGroupId::Failsafe, tx))
        .expect("send final config read");
    let ConfigResponse::Snapshot(snapshot) = rx
        .blocking_recv()
        .expect("final config response channel")
        .expect("final config read")
    else {
        panic!("final config response must be a snapshot");
    };
    assert_eq!(snapshot.revision, 1);
    assert!(snapshot.persisted);
    assert!(matches!(snapshot.value, Some(ConfigValue::Failsafe(_))));

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

#[test]
fn tcp_connection_churn_does_not_kill_server() {
    let port = free_tcp_port();
    let child = Command::new(env!("CARGO_BIN_EXE_oxifoc-virtual"))
        .args(["--transport", "tcp", "--port", &port.to_string()])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("spawn oxifoc-virtual");
    let mut guard = ChildGuard(child);

    let deadline = Instant::now() + Duration::from_secs(10);
    let first = loop {
        match TcpStream::connect(("127.0.0.1", port)) {
            Ok(stream) => break stream,
            Err(_) if Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(25));
            }
            Err(e) => panic!("virtual TCP listener did not start: {e}"),
        }
    };

    // Keep every client open. The old server accumulated four live Router
    // interfaces and returned from run() on the fifth registration failure.
    let mut clients = vec![first];
    for _ in 0..8 {
        clients.push(TcpStream::connect(("127.0.0.1", port)).expect("connect churn client"));
        std::thread::sleep(Duration::from_millis(25));
    }
    std::thread::sleep(Duration::from_millis(250));

    assert!(
        guard.0.try_wait().unwrap().is_none(),
        "connection churn must not terminate the virtual device"
    );
}

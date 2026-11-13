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
//! - Current-limit writes affect measured current through the firmware driver.
//! - Link loss latches re-arm; controller reboot restores handshake, streaming
//!   and the boot-local fault generation on the existing host backend.
//!
//! `start_host` runs its own tokio runtime on a background thread, so these are
//! plain `#[test]`s that drive the runtime via its (sync) channels.

use std::net::{TcpListener, TcpStream, UdpSocket};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};

use oxifoc_core::storage::{CurrentLimitsConfig, FailsafeConfigStored};
use oxifoc_core::types::{
    ConfigApply, ConfigGroupId, ConfigPersist, ConfigResponse, ConfigValue, ControlMode,
    CurrentOffsetMethod, DetectRequest, DetectResponse, FaultClear, FaultClearTarget, FaultRequest,
    FaultResponse, Keyed, MotorCommandOutcome, ReqId,
};
use oxifoc_host_lib::{
    HostCommand, HostConfig, ReconnectPolicy, TransportType, config_channel, fault_channel,
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

fn spawn_device(transport_arg: &str, port: u16) -> ChildGuard {
    let log = || {
        std::env::var_os("OXIFOC_E2E_LOG_DIR").map_or_else(Stdio::null, |dir| {
            let path =
                std::path::PathBuf::from(dir).join(format!("device-{transport_arg}-{port}.log"));
            Stdio::from(
                std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(path)
                    .unwrap(),
            )
        })
    };
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
            "--load",
            "0.2",
        ])
        .stdout(log())
        .stderr(log())
        .spawn()
        .expect("spawn oxifoc-virtual");
    ChildGuard(child)
}

/// Spawn the virtual device and drive handshake + Motor + Detect over the given
/// transport, asserting each step. Shared by the TCP and UDP tests below.
fn run_e2e(transport: TransportType) {
    if std::env::var_os("OXIFOC_E2E_LOG_DIR").is_some() {
        oxifoc_host_lib::init_tracing();
    }
    let (transport_arg, port) = match transport {
        TransportType::Tcp => ("tcp", free_tcp_port()),
        TransportType::Udp => ("udp", free_udp_port()),
        other => panic!("unsupported transport in e2e: {other:?}"),
    };

    // Spawn the virtual device as a Router on the chosen port/transport.
    let mut guard = spawn_device(transport_arg, port);

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
    let rt = start_host(cfg.clone());

    // 1) Handshake: the host reports connected once HardwareInfo round-trips.
    assert!(
        rt.wait_for_connection(Duration::from_secs(15)),
        "[{transport_arg}] device should connect (HardwareInfo handshake)"
    );
    // The connect path must have RESOLVED the controller via SocketQuery,
    // not silently fallen back to the link-local peer — the virtual device
    // serves socket_query_handler, so a fallback means the resolution flood
    // or the handler regressed.
    let addr = rt.device_address();
    assert!(
        (addr.network_id, addr.node_id) != (0, 1),
        "[{transport_arg}] controller address should be resolved, got link-local fallback"
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

    // A config ACK must have a physical effect on the common control path.
    // The bare-controller implementation ignored this command and delivered
    // the requested 4 A under load instead of the configured 1 A.
    oxifoc_host_lib::ops::config::apply_write(
        &rt.cmd_tx,
        oxifoc_host_lib::ops::config::read_group(&rt.cmd_tx, ConfigGroupId::CurrentLimits)
            .unwrap()
            .boot_epoch,
        0,
        ConfigValue::CurrentLimits(CurrentLimitsConfig {
            max_iq_a: 1.0,
            bus_in_max_a: 5.0,
            bus_regen_max_a: 0.0,
            ..Default::default()
        }),
    )
    .expect("apply current limit");

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

    let mut iq_sum = 0.0;
    let mut iq_count = 0;
    assert!(
        wait_until(Duration::from_secs(4), || {
            while let Ok(sample) = rt.fast_rx.try_recv() {
                iq_sum += sample.enrich(&enrich).iq.abs();
                iq_count += 1;
            }
            iq_count >= 64
        }),
        "[{transport_arg}] need enough samples to observe the current limit"
    );
    let mean_iq = iq_sum / iq_count as f32;
    assert!(
        (0.2..1.3).contains(&mean_iq),
        "[{transport_arg}] configured 1 A limit must constrain measured current, got {mean_iq} A"
    );

    // Detection owns a stopped motor. Clear the host's periodic drive affirm
    // before handing over, just as the physical-device workflow does.
    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx))
        .unwrap();
    rx.blocking_recv().unwrap().expect("stop before detection");

    // 3) The host opens a measurement session and waits for a terminal result.
    let resp = oxifoc_host_lib::ops::detect::run_step(
        &rt.cmd_tx,
        DetectRequest::MeasureResistance {
            max_power_loss_w: 8.0,
            current_limit_a: None,
        },
    )
    .expect("detect should succeed");
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
    let ConfigResponse::Snapshot(snapshot) = &initial else {
        panic!("expected config snapshot");
    };
    let boot_epoch = snapshot.boot_epoch;
    assert_eq!(
        initial,
        ConfigResponse::Snapshot(oxifoc_core::types::ConfigSnapshot {
            effective: None,
            default: None,
            boot_epoch,
            valid: true,
            group: ConfigGroupId::Failsafe,
            revision: 0,
            persisted: false,
            value: None,
        })
    );

    let apply = Keyed::new(
        ReqId(0xA11E_0001),
        ConfigApply {
            boot_epoch,
            expected_revision: 0,
            write: ConfigValue::Failsafe(FailsafeConfigStored::default()),
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

    for (request, expected) in [
        (
            Keyed::new(
                apply.id,
                ConfigApply {
                    boot_epoch,
                    expected_revision: 0,
                    write: ConfigValue::Failsafe(FailsafeConfigStored {
                        staleness_timeout_ms: 200,
                        ..Default::default()
                    }),
                },
            ),
            ConfigResponse::Invalid,
        ),
        (
            Keyed::new(
                ReqId(0xA11E_0010),
                ConfigApply {
                    boot_epoch: boot_epoch ^ 1,
                    expected_revision: 1,
                    write: apply.inner.write.clone(),
                },
            ),
            ConfigResponse::Conflict {
                current_revision: 1,
            },
        ),
    ] {
        let (tx, rx) = config_channel();
        rt.cmd_tx
            .send(HostCommand::ConfigApply(request, tx))
            .unwrap();
        assert_eq!(rx.blocking_recv().unwrap().unwrap(), expected);
    }

    let (tx, rx) = config_channel();
    rt.cmd_tx
        .send(HostCommand::ConfigApply(
            Keyed::new(
                ReqId(0xA11E_0002),
                ConfigApply {
                    boot_epoch,
                    expected_revision: 0,
                    write: ConfigValue::Failsafe(FailsafeConfigStored::default()),
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
            boot_epoch,
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

    // 7) Fault Clear is generation-guarded and keyed. Retrying a lost ACK
    // cannot clear a later occurrence, while a stale observation conflicts.
    let (tx, rx) = fault_channel();
    rt.cmd_tx
        .send(HostCommand::Fault(FaultRequest::Query, tx))
        .expect("send fault query");
    let FaultResponse::Snapshot(faults) = rx
        .blocking_recv()
        .expect("fault query response channel")
        .expect("fault query")
    else {
        panic!("fault query must return a snapshot");
    };
    let clear = Keyed::new(
        ReqId(0xFA17_0001),
        FaultClear {
            expected_generation: faults.generation,
            target: FaultClearTarget::All,
        },
    );
    for attempt in 0..2 {
        let (tx, rx) = fault_channel();
        rt.cmd_tx
            .send(HostCommand::Fault(FaultRequest::Clear(clear.clone()), tx))
            .expect("send fault clear");
        assert_eq!(
            rx.blocking_recv()
                .expect("fault clear response channel")
                .expect("fault clear"),
            FaultResponse::Cleared {
                req_id: clear.id,
                snapshot: faults.clone(),
            },
            "fault clear attempt {attempt} must be deduplicated"
        );
    }

    let (tx, rx) = fault_channel();
    rt.cmd_tx
        .send(HostCommand::Fault(
            FaultRequest::Clear(Keyed::new(
                ReqId(0xFA17_0002),
                FaultClear {
                    expected_generation: faults.generation.wrapping_add(1),
                    target: FaultClearTarget::All,
                },
            )),
            tx,
        ))
        .expect("send stale fault clear");
    assert_eq!(
        rx.blocking_recv()
            .expect("stale fault response channel")
            .expect("stale fault protocol response"),
        FaultResponse::Conflict(faults)
    );

    // Lose the command/link stream while driving. The firmware runtime must
    // stop and latch; a new host session cannot immediately resume torque.
    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            },
            tx,
        ))
        .unwrap();
    rx.blocking_recv().unwrap().expect("drive before link loss");
    rt.shutdown();
    // TCP EOF is immediate; UDP also exercises the one-second liveness bound.
    std::thread::sleep(Duration::from_millis(1500));
    let replacement = start_host(cfg);
    assert!(replacement.wait_for_connection(Duration::from_secs(15)));
    let (tx, rx) = motor_channel();
    replacement
        .cmd_tx
        .send(HostCommand::MotorAck(
            ControlMode::CurrentControl {
                iq_target: 1.0,
                id_target: 0.0,
            },
            tx,
        ))
        .unwrap();
    assert!(
        rx.blocking_recv().unwrap().is_err(),
        "[{transport_arg}] disconnect must require neutral acknowledgement"
    );
    let (tx, rx) = motor_channel();
    replacement
        .cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx))
        .unwrap();
    rx.blocking_recv()
        .unwrap()
        .expect("neutral must re-arm after disconnect");

    // Keep this backend alive across a controller reboot, exercising the
    // shared reconnect loop, UUID pinning, new fault generation and streaming.
    while replacement.device_info_rx.try_recv().is_ok() {}
    while replacement.fault_rx.try_recv().is_ok() {}
    // Keep a bound, silent UDP port during the outage. This exercises the
    // liveness timeout rather than ICMP errors, whose retry-loop timeout
    // reset in the pinned ergot is tracked separately in docs/TODO.md.
    let silent_udp = if transport_arg == "udp" {
        let socket =
            socket2::Socket::new(socket2::Domain::IPV4, socket2::Type::DGRAM, None).unwrap();
        socket.set_reuse_address(true).unwrap();
        socket
            .bind(
                &format!("127.0.0.1:{port}")
                    .parse::<std::net::SocketAddr>()
                    .unwrap()
                    .into(),
            )
            .unwrap();
        Some(socket)
    } else {
        None
    };
    guard.0.kill().unwrap();
    guard.0.wait().unwrap();
    assert!(
        wait_until(Duration::from_secs(15), || !replacement
            .connected
            .load(std::sync::atomic::Ordering::Relaxed)),
        "[{transport_arg}] backend must observe device loss"
    );
    while replacement.fast_rx.try_recv().is_ok() {}
    drop(silent_udp);
    guard = spawn_device(transport_arg, port);
    let _reconnected_info = replacement
        .device_info_rx
        .recv_timeout(Duration::from_secs(20))
        .expect("reconnect must perform a fresh HardwareInfo handshake");
    assert!(replacement.wait_for_connection(Duration::from_secs(3)));
    replacement
        .fast_rx
        .recv_timeout(Duration::from_secs(3))
        .expect("reconnect must restore telemetry streaming");
    let (tx, rx) = fault_channel();
    replacement
        .cmd_tx
        .send(HostCommand::Fault(FaultRequest::Query, tx))
        .unwrap();
    let FaultResponse::Snapshot(boot_faults) = rx.blocking_recv().unwrap().unwrap() else {
        panic!("fault query must return a snapshot");
    };
    let mut observed = Vec::new();
    assert!(
        wait_until(Duration::from_secs(3), || {
            observed.extend(replacement.fault_rx.try_iter());
            observed.contains(&boot_faults)
        }),
        "[{transport_arg}] reboot must deliver the fresh boot-local faults {boot_faults:?}; observed {observed:?}"
    );
    replacement.shutdown();
    drop(guard);
}

#[test]
fn drive_config_applies_to_the_throttle_command() {
    use oxifoc_core::storage::DriveConfig;
    use oxifoc_host_lib::ops::config::{apply_write, read_group};

    let port = free_tcp_port();
    let _device = spawn_device("tcp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Tcp),
        tcp_host: Some("127.0.0.1".into()),
        tcp_port: Some(port),
        fast_hz: Some(500),
        stream_defmt: Some(false),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    let drive = DriveConfig {
        invert_direction: true,
    };
    let derating = oxifoc_core::motor::derating::DeratingConfig {
        speed: oxifoc_core::motor::derating::SpeedDerating {
            max_forward_erad_s: 350.0,
            max_reverse_erad_s: 120.0,
            ..Default::default()
        },
        ..Default::default()
    };
    let boot_epoch = read_group(&rt.cmd_tx, ConfigGroupId::Drive)
        .unwrap()
        .boot_epoch;
    apply_write(&rt.cmd_tx, boot_epoch, 0, ConfigValue::Derating(derating)).unwrap();
    let group = ConfigGroupId::Drive;
    let revision = apply_write(&rt.cmd_tx, boot_epoch, 0, ConfigValue::Drive(drive)).unwrap();
    let snapshot = read_group(&rt.cmd_tx, group).unwrap();
    assert_eq!(snapshot.revision, revision);
    assert_eq!(snapshot.value, Some(ConfigValue::Drive(drive)));

    let send = |mode| {
        let (tx, rx) = motor_channel();
        rt.cmd_tx.send(HostCommand::MotorAck(mode, tx)).unwrap();
        rx.blocking_recv().unwrap().unwrap()
    };
    let throttle = ControlMode::Throttle { fraction: 0.1 };
    let ack = send(throttle);
    assert_eq!(ack.outcome, MotorCommandOutcome::Applied);
    assert_eq!(ack.mode, throttle);
    assert!(wait_until(Duration::from_secs(2), || {
        rt.fast_rx.try_iter().any(|sample| sample.rpm < 0)
    }));
    assert!(
        apply_write(
            &rt.cmd_tx,
            boot_epoch,
            revision,
            ConfigValue::Drive(DriveConfig::DEFAULT),
        )
        .is_err()
    );
    assert_eq!(
        send(ControlMode::Stopped).outcome,
        MotorCommandOutcome::Applied
    );
    rt.shutdown();
}

#[test]
fn partial_config_updates_use_device_defaults() {
    use oxifoc_core::storage::{CurrentLimitsConfig, HallTuningConfig, VoltageLimitsConfig};
    use oxifoc_host_lib::ops::config::{apply_fields, apply_write, read_group};

    let port = free_tcp_port();
    let _device = spawn_device("tcp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Tcp),
        tcp_host: Some("127.0.0.1".into()),
        tcp_port: Some(port),
        stream_defmt: Some(false),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    let group = ConfigGroupId::VoltageLimits;
    let before = read_group(&rt.cmd_tx, group).unwrap();
    let Some(ConfigValue::VoltageLimits(active)) = before.effective else {
        panic!("device must report its voltage limits");
    };
    assert!(before.value.is_none());
    let (_, revision) = apply_fields(
        &rt.cmd_tx,
        group,
        &[format!("max_vbus_mv={}", active.max_vbus_mv - 1000)],
    )
    .unwrap();
    let after = read_group(&rt.cmd_tx, group).unwrap();
    assert_eq!(after.revision, revision);
    assert_eq!(
        after.value,
        Some(ConfigValue::VoltageLimits(VoltageLimitsConfig {
            max_vbus_mv: active.max_vbus_mv - 1000,
            ..active
        }))
    );

    let group = ConfigGroupId::CurrentLimits;
    apply_fields(&rt.cmd_tx, group, &["bus_in_max_a=5".into()]).unwrap();
    assert_eq!(
        read_group(&rt.cmd_tx, group).unwrap().value,
        Some(ConfigValue::CurrentLimits(CurrentLimitsConfig {
            bus_in_max_a: 5.0,
            ..CurrentLimitsConfig::AUTO
        }))
    );

    let group = ConfigGroupId::HallTuning;
    let tuning = HallTuningConfig {
        interp_min_erpm: 600.0,
        ..Default::default()
    };
    let before = read_group(&rt.cmd_tx, group).unwrap();
    let revision = apply_write(
        &rt.cmd_tx,
        before.boot_epoch,
        before.revision,
        ConfigValue::HallTuning(tuning.clone()),
    )
    .unwrap();
    let after = read_group(&rt.cmd_tx, group).unwrap();
    assert_eq!(after.revision, revision);
    assert_eq!(after.value, Some(ConfigValue::HallTuning(tuning)));
    rt.shutdown();
}

#[test]
fn invalid_loop_config_does_not_advance_the_revision() {
    use oxifoc_core::storage::VelocityLoopConfig;
    use oxifoc_host_lib::ops::config::{apply_write, read_group};

    let port = free_tcp_port();
    let _device = spawn_device("tcp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Tcp),
        tcp_host: Some("127.0.0.1".into()),
        tcp_port: Some(port),
        stream_defmt: Some(false),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    for write in [
        ConfigValue::Velocity(VelocityLoopConfig {
            kp: -0.01,
            ..Default::default()
        }),
        ConfigValue::Failsafe(FailsafeConfigStored {
            policy: 99,
            ..Default::default()
        }),
        ConfigValue::Failsafe(FailsafeConfigStored {
            slew_a_per_s: 0.0,
            ..Default::default()
        }),
    ] {
        let group = write.group();
        let before = read_group(&rt.cmd_tx, group).unwrap();
        let error = apply_write(&rt.cmd_tx, before.boot_epoch, before.revision, write).unwrap_err();
        assert!(error.to_string().contains("fails validation"));
        assert_eq!(read_group(&rt.cmd_tx, group).unwrap(), before);
    }
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

/// Pinned ergot restarts its receive timeout on every ICMP receive error,
/// so repeated host requests keep the dead interface Active indefinitely.
#[test]
#[ignore = "ergot UDP receive errors reset the liveness deadline; see docs/TODO.md"]
fn udp_connection_refused_must_disconnect() {
    let port = free_udp_port();
    let mut guard = spawn_device("udp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Udp),
        udp_host: Some("127.0.0.1".to_string()),
        udp_port: Some(port),
        stream_defmt: Some(false),
        reconnect: Some(ReconnectPolicy::None),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    guard.0.kill().unwrap();
    guard.0.wait().unwrap();
    assert!(wait_until(Duration::from_secs(5), || !rt
        .connected
        .load(std::sync::atomic::Ordering::Relaxed)));
    rt.shutdown();
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

/// Calibration changes the measurement source, not the simulated supply or
/// current calibration. Exercise validation, stop/capture gates and persistence.
#[test]
fn board_calibration_e2e_tcp() {
    use oxifoc_core::storage::BoardCalibrationConfig;
    use oxifoc_core::types::{ConfigSnapshot, TelemetryConfig};
    use oxifoc_host_lib::ops::config::{persist_group, read_group, reset_all};
    let port = free_tcp_port();
    let _device = spawn_device("tcp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Tcp),
        tcp_host: Some("127.0.0.1".into()),
        tcp_port: Some(port),
        fast_hz: Some(0),
        stream_defmt: Some(false),
        reconnect: Some(ReconnectPolicy::Limited(20)),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    let group = ConfigGroupId::BoardCalibration;
    let initial = read_group(&rt.cmd_tx, group).unwrap();
    assert!(initial.valid && !initial.persisted && initial.value.is_none());
    let apply = |gain, revision, id| {
        let (tx, rx) = config_channel();
        rt.cmd_tx
            .send(HostCommand::ConfigApply(
                Keyed::new(
                    ReqId(id),
                    ConfigApply {
                        boot_epoch: initial.boot_epoch,
                        expected_revision: revision,
                        write: ConfigValue::BoardCalibration(BoardCalibrationConfig {
                            vbus_gain: gain,
                        }),
                    },
                ),
                tx,
            ))
            .unwrap();
        rx.blocking_recv().unwrap().unwrap()
    };
    let telemetry = |hz| {
        let (tx, rx) = oxifoc_host_lib::telemetry_channel();
        rt.cmd_tx
            .send(HostCommand::SetTelemetryConfig(
                TelemetryConfig { fast_hz: hz },
                Some(tx),
            ))
            .unwrap();
        rx.blocking_recv().unwrap().unwrap()
    };
    assert_eq!(apply(1.2, 0, 100), ConfigResponse::Invalid);
    let accepted = apply(1.01, 0, 101);
    assert_eq!(
        accepted,
        ConfigResponse::Applied {
            req_id: ReqId(101),
            revision: 1
        }
    );
    assert_eq!(apply(1.01, 0, 101), accepted, "retry must be deduplicated");
    assert_eq!(
        apply(1.02, 0, 102),
        ConfigResponse::Conflict {
            current_revision: 1
        }
    );
    // Applying the same absolute multiplier with a fresh revision is not cumulative.
    assert_eq!(
        apply(1.01, 1, 103),
        ConfigResponse::Applied {
            req_id: ReqId(103),
            revision: 2
        }
    );
    assert!(
        wait_until(Duration::from_secs(3), || {
            rt.slow_rx
                .try_iter()
                .any(|s| (s.vbus_mv as i32 - 24_240).abs() <= 2)
        }),
        "slow telemetry must use the corrected measurement"
    );
    assert!(telemetry(500).actual_fast_hz > 0);
    assert_eq!(
        apply(1.02, 2, 104),
        ConfigResponse::Busy,
        "capture must block a scale change"
    );
    assert!(
        reset_all(&rt.cmd_tx).is_err(),
        "reset must not change a capture's scale"
    );
    telemetry(0);
    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(
            ControlMode::CurrentControl {
                iq_target: 0.1,
                id_target: 0.0,
            },
            tx,
        ))
        .unwrap();
    assert!(rx.blocking_recv().unwrap().is_ok());
    assert_eq!(
        apply(1.02, 2, 105),
        ConfigResponse::Busy,
        "running must block calibration"
    );
    let (tx, rx) = motor_channel();
    rt.cmd_tx
        .send(HostCommand::MotorAck(ControlMode::Stopped, tx))
        .unwrap();
    assert!(rx.blocking_recv().unwrap().is_ok());
    assert!(persist_group(&rt.cmd_tx, group, initial.boot_epoch, 2).unwrap());
    let ConfigSnapshot {
        value,
        persisted,
        revision,
        valid,
        ..
    } = read_group(&rt.cmd_tx, group).unwrap();
    assert!(persisted && valid);
    assert_eq!(revision, 2);
    assert_eq!(
        value,
        Some(ConfigValue::BoardCalibration(BoardCalibrationConfig {
            vbus_gain: 1.01
        }))
    );
    reset_all(&rt.cmd_tx).unwrap();
    assert!(read_group(&rt.cmd_tx, group).unwrap().value.is_none());
    assert!(
        wait_until(Duration::from_secs(3), || {
            rt.slow_rx.try_iter().any(|s| s.vbus_mv == 24_000)
        }),
        "reset must restore the identity scale immediately"
    );
}

#[test]
fn drive_setup_save_replays_exact_request_and_commits_all_groups() {
    use oxifoc_core::drive_setup::DriveCurrent;
    use oxifoc_core::types::{ConfigValue, SaveDriveSetup, commissioning::SourceLimits};
    use oxifoc_host_lib::ops::{config::read_group, detect};

    let port = free_tcp_port();
    let _device = spawn_device("tcp", port);
    let rt = start_host(HostConfig {
        transport: Some(TransportType::Tcp),
        tcp_host: Some("127.0.0.1".into()),
        tcp_port: Some(port),
        fast_hz: Some(0),
        stream_defmt: Some(false),
        reconnect: Some(ReconnectPolicy::Limited(20)),
        ..Default::default()
    });
    assert!(rt.wait_for_connection(Duration::from_secs(15)));
    let mut plan = detect::DetectionPlan::from(
        detect::PRESETS
            .iter()
            .find(|preset| preset.id == "medium-outrunner")
            .unwrap(),
    );
    plan.source = Some(SourceLimits {
        input_a: 5.0,
        regen_a: 0.0,
    });
    plan.calibrate_hall = false;
    let outcome = detect::run_sequence(&rt.cmd_tx, plan, |_, request| {
        detect::run_measurement(&rt.cmd_tx, request)
    })
    .unwrap();
    let request = Keyed::new(
        ReqId(0x5516),
        SaveDriveSetup {
            session: outcome.session.unwrap(),
            setup: outcome.setup(plan, DriveCurrent::Override(10.0)).unwrap(),
        },
    );
    let send = |request| {
        let (tx, rx) = config_channel();
        rt.cmd_tx
            .send(HostCommand::SaveDriveSetup(request, tx))
            .unwrap();
        rx.blocking_recv().unwrap().unwrap()
    };
    let mut tampered = request.clone();
    tampered.inner.setup.source.input_a = 6.0;
    assert_eq!(send(tampered.clone()), ConfigResponse::Invalid);
    let ack = send(request.clone());
    assert_eq!(
        ack,
        ConfigResponse::Persisted {
            req_id: request.id,
            revision: 1
        }
    );
    assert_eq!(send(request), ack);
    assert_eq!(send(tampered), ConfigResponse::Invalid);
    let limits = read_group(&rt.cmd_tx, ConfigGroupId::CurrentLimits).unwrap();
    assert!(limits.persisted && limits.revision == 1);
    let Some(ConfigValue::CurrentLimits(limits)) = limits.value else {
        panic!("missing limits")
    };
    assert_eq!(limits.max_iq_a, 10.0);
    assert_eq!(limits.bus_in_max_a, 5.0);
    assert_eq!(limits.bus_regen_max_a, 0.0);
    let hall = read_group(&rt.cmd_tx, ConfigGroupId::HallCalibration).unwrap();
    assert!(hall.persisted && hall.revision == 1 && hall.value.is_none());
    let motor = read_group(&rt.cmd_tx, ConfigGroupId::MotorParams).unwrap();
    assert!(motor.persisted && motor.revision == 1);
    let Some(ConfigValue::MotorParams(motor)) = motor.value else {
        panic!("missing motor")
    };
    assert_eq!(motor.resistance_ohm, outcome.resistance_ohm);
    assert_eq!(motor.flux_linkage_wb, outcome.flux_linkage_wb);
}

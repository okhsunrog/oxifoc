//! Every endpoint and topic that is measured, with the header it is sent
//! with and representative values built after the code that sends them.
//! An endpoint or topic without representatives is still measured at its
//! schema extremes.

use core::num::NonZeroU32;

use ergot::{
    FrameKind,
    interface_manager::{NodeClaimAssignment, SeedNetAssignment},
    nash::NameHash,
    traits::Endpoint,
    well_known::{
        AddressClaimGranted, AddressClaimRequest, AddressRefreshRequest, ErgotAddressClaimEndpoint,
        ErgotAddressRefreshEndpoint, ErgotDeviceInfoTopic, ErgotPathMtuEndpoint, ErgotPingEndpoint,
        ErgotSeedRouterAssignmentEndpoint, ErgotSeedRouterRefreshEndpoint,
        ErgotSeedRouterReleaseEndpoint, ErgotSocketQueryResponseTopic, ErgotSocketQueryTopic,
        NameRequirement, SeedRouterAssignment, SeedRouterRefreshRequest, SocketQuery,
        SocketQueryResponse,
    },
};
use oxifoc_core::{
    foc::fault::{FaultCategory, FaultInfo},
    icd::{
        BridgeDiagEndpoint, BridgeOtaEndpoint, ConfigEndpoint, DetectEndpoint, DiagEndpoint,
        FaultEndpoint, FaultTopic, HardwareInfoEndpoint, MotorEndpoint, PairingEndpoint,
        PairingEventTopic, RawRecordEndpoint, RawRelayEndpoint, RawTelemetryBlock,
        RawTelemetryBlockTopic, RawTelemetryEndpoint, SlowTelemetryEndpoint, SwdProbeEndpoint,
        VelocityDebugArmEndpoint, VelocityDebugRelayEndpoint,
    },
    raw_record::{
        RECORD_CHUNK_BYTES, RecordChunk, RecordEnd, RecordMode, RecordRequest, RecordResponse,
        RecordState, RecordStatus,
    },
    raw_telemetry::{
        RAW_MAX_BLOCK_BYTES, RawEncoding, RawTelemetryAck, RawTelemetryRequest, RawTelemetryStatus,
        VelocityDebugArm,
    },
    types::{
        ConfigGroupId, ConfigRequest, ControlMode, ControlTiming, FaultRequest, FaultResponse,
        FaultSnapshot, HardwareInfo, MotorCommand, MotorCommandOutcome, MotorRequest, MotorState,
        MotorStatus, SlowTelemetry, TelemetryConfigAck,
        bridge_ota::{CHUNK_SIZE, OtaRequest, OtaResponse},
    },
};

use crate::{
    catalog::{Catalog, Group, Representative},
    header::{
        ANONYMOUS_REQUEST, BUS_ANONYMOUS_REQUEST, BUS_KNOWN_PORT_REQUEST, BUS_RESPONSE,
        CLAIM_REQUEST, KNOWN_PORT, RESPONSE, SOCKET_QUERY, TOPIC_BROADCAST, TOPIC_UNICAST_ANY_PORT,
        TOPIC_UNICAST_KNOWN_PORT, named_request,
    },
    raw_blocks::{RawBlocks, Spread, ble_block_limit},
};

/// A drive session as the host makes it: nanoseconds since the epoch (late
/// 2026) xor a counter (`oxifoc-host-lib/src/lib.rs`, `new_drive_session`).
/// Its varint is as long as a real one.
pub const SESSION: u64 = 1_796_000_000_123_456_789;
/// The previous session an Arm names when it takes over.
const PREVIOUS_SESSION: u64 = SESSION - 7;
const FAST_HZ: u16 = 20_000;

pub fn catalog(raw_blocks: Option<&RawBlocks>) -> Catalog {
    let mut catalog = Catalog::default();
    oxifoc(&mut catalog, raw_blocks);
    ergot_control_plane(&mut catalog);
    catalog
}

fn oxifoc(catalog: &mut Catalog, raw_blocks: Option<&RawBlocks>) {
    let g = Group::Oxifoc;
    let hardware = HardwareInfo {
        hw: heapless_string("VESC 6 MK5 (F405)"),
        sw: heapless_string("oxifoc-0.1.0"),
        mcu: heapless_string("STM32F405RG"),
        uuid: heapless_string("3A0047001851333036383837"),
        foc_freq_hz: u32::from(FAST_HZ),
        capabilities: 0b1001,
        reserved: [0; 8],
        ..HardwareInfo::default()
    };
    catalog.endpoint::<HardwareInfoEndpoint>(
        g,
        named_request("hardware_info"),
        RESPONSE,
        vec![],
        vec![("F405 MK5 strings", hardware)],
    );

    let motor = |seq, command| MotorRequest {
        source_session: SESSION,
        seq,
        command,
    };
    let throttle = ControlMode::Throttle { fraction: 0.25 };
    catalog.endpoint::<MotorEndpoint>(
        g,
        named_request("motor"),
        RESPONSE,
        vec![
            (
                "Affirm, seq 12000",
                motor(
                    12_000,
                    MotorCommand::Affirm {
                        setpoint_seq: 11_990,
                    },
                ),
            ),
            (
                "SetMode Throttle 0.25",
                motor(11_990, MotorCommand::SetMode(throttle)),
            ),
            (
                "SetMode Stopped",
                motor(12_001, MotorCommand::SetMode(ControlMode::Stopped)),
            ),
            (
                "Affirm, seq 5, session 0",
                MotorRequest {
                    source_session: 0,
                    seq: 5,
                    command: MotorCommand::Affirm { setpoint_seq: 4 },
                },
            ),
        ],
        vec![(
            "Applied, Running, Throttle",
            MotorStatus {
                outcome: MotorCommandOutcome::Applied,
                state: MotorState::Running,
                mode: throttle,
                fault_count: 0,
            },
        )],
    );

    let riding = SlowTelemetry {
        vbus_mv: 36_000,
        fet_temp_c_x10: 352,
        motor_temp_c_x10: Some(418),
        board_temp_c_x10: 301,
        motor_state: MotorState::Running,
        control_mode: throttle,
        fault_count: 0,
        seq: 12_345,
        derate_drive_pct: 100,
        derate_brake_pct: 100,
        fault_generation: 3,
        derate_regen_pct: 100,
        ..SlowTelemetry::default()
    };
    catalog.endpoint::<SlowTelemetryEndpoint>(
        g,
        named_request("slow_telem"),
        RESPONSE,
        vec![("()", ())],
        vec![("riding, Throttle", riding)],
    );

    raw_sessions(catalog);
    raw_record(catalog);

    catalog.endpoint::<FaultEndpoint>(
        g,
        named_request("fault"),
        RESPONSE,
        vec![("Query", FaultRequest::Query)],
        vec![
            ("Snapshot, no fault", FaultResponse::Snapshot(faults(0))),
            ("Snapshot, 1 hall fault", FaultResponse::Snapshot(faults(1))),
            ("Snapshot, 3 faults", FaultResponse::Snapshot(faults(3))),
        ],
    );
    catalog.endpoint::<DiagEndpoint>(g, named_request("diag"), RESPONSE, vec![], vec![]);
    catalog.endpoint::<BridgeDiagEndpoint>(g, ANONYMOUS_REQUEST, RESPONSE, vec![], vec![]);
    catalog.endpoint::<DetectEndpoint>(g, named_request("detect"), RESPONSE, vec![], vec![]);
    catalog.endpoint::<PairingEndpoint>(g, ANONYMOUS_REQUEST, RESPONSE, vec![], vec![]);
    catalog.endpoint::<SwdProbeEndpoint>(g, ANONYMOUS_REQUEST, RESPONSE, vec![], vec![]);

    let full_chunk = heapless::Vec::from_slice(&[0xa5; CHUNK_SIZE]).expect("CHUNK_SIZE fits");
    catalog.endpoint::<BridgeOtaEndpoint>(
        g,
        ANONYMOUS_REQUEST,
        RESPONSE,
        vec![
            (
                "Chunk 384 at 200 KB",
                OtaRequest::Chunk {
                    offset: 200_064,
                    data: full_chunk,
                },
            ),
            (
                "Begin 457 KB",
                OtaRequest::Begin {
                    length: 457_488,
                    digest: [0x3c; 32],
                },
            ),
        ],
        vec![("Progress 200 KB", OtaResponse::Progress { offset: 200_448 })],
    );
    catalog.endpoint::<ConfigEndpoint>(
        g,
        named_request("config"),
        RESPONSE,
        vec![(
            "Read MotorParams",
            ConfigRequest::Read(ConfigGroupId::MotorParams),
        )],
        vec![],
    );

    catalog.topic::<RawTelemetryBlockTopic>(
        g,
        TOPIC_UNICAST_ANY_PORT,
        raw_blocks
            .map(raw_block_representatives)
            .unwrap_or_default(),
    );
    catalog.topic::<FaultTopic>(
        g,
        TOPIC_BROADCAST,
        vec![
            ("no fault", faults(0)),
            ("1 hall fault", faults(1)),
            ("3 faults", faults(3)),
        ],
    );
    catalog.topic::<PairingEventTopic>(g, TOPIC_BROADCAST, vec![]);
}

/// The four endpoints that arm a raw telemetry session, directly or through
/// the bridge's relay.
fn raw_sessions(catalog: &mut Catalog) {
    let g = Group::Oxifoc;
    let arm = RawTelemetryRequest::Arm {
        session: SESSION,
        previous_session: PREVIOUS_SESSION,
        encoding: RawEncoding::Packed28,
        fast_hz: FAST_HZ,
        max_block_bytes: RAW_MAX_BLOCK_BYTES as u16,
    };
    let velocity_arm = VelocityDebugArm {
        session: SESSION,
        previous_session: PREVIOUS_SESSION,
        fast_hz: FAST_HZ,
    };
    let armed = RawTelemetryAck {
        session: SESSION,
        encoding: RawEncoding::Packed28,
        status: RawTelemetryStatus::Armed,
        timing: timing(),
    };
    catalog.endpoint::<RawTelemetryEndpoint>(
        g,
        named_request("raw_telemetry"),
        RESPONSE,
        vec![("Arm 20 kHz", arm)],
        vec![("Armed", armed)],
    );
    catalog.endpoint::<VelocityDebugArmEndpoint>(
        g,
        named_request("velocity_debug"),
        RESPONSE,
        vec![("20 kHz", velocity_arm)],
        vec![("Armed", armed)],
    );
    catalog.endpoint::<RawRelayEndpoint>(
        g,
        named_request("raw_relay"),
        RESPONSE,
        vec![("Arm 20 kHz", arm)],
        vec![("Armed", armed)],
    );
    catalog.endpoint::<VelocityDebugRelayEndpoint>(
        g,
        named_request("raw_relay_velocity"),
        RESPONSE,
        vec![("20 kHz", velocity_arm)],
        vec![("Armed", armed)],
    );
}

fn raw_record(catalog: &mut Catalog) {
    let stopped = RecordStatus {
        state: RecordState::Stopped(RecordEnd::Request),
        mode: RecordMode::Ring,
        session: SESSION,
        started_unix_ms: 1_796_000_000_000,
        timing: timing(),
        bytes: 4_000_000,
        capacity: 8_388_608,
        blocks: 6_000,
        missed: 0,
        overwritten: 0,
    };
    let data = |len: usize| {
        RecordResponse::Data(RecordChunk {
            data: heapless::Vec::from_slice(&vec![0x5a; len]).expect("a chunk fits"),
        })
    };
    let read = |max_bytes: usize| RecordRequest::Read {
        offset: 4_000_000,
        max_bytes: max_bytes as u16,
    };
    // The host CLI reads chunks of its link's raw block limit less 8 bytes
    // (`oxifoc-host-cli/src/recording.rs`); on BLE that is 470.
    let ble_chunk = usize::from(ble_block_limit()) - 8;
    catalog.endpoint::<RawRecordEndpoint>(
        Group::Oxifoc,
        ANONYMOUS_REQUEST,
        RESPONSE,
        vec![
            ("Read 2048 at 4 MB (USB/TCP host)", read(RECORD_CHUNK_BYTES)),
            ("Read 470 at 4 MB (BLE host)", read(ble_chunk)),
        ],
        vec![
            ("Data 2048 (USB/TCP host)", data(RECORD_CHUNK_BYTES)),
            ("Data 470 (BLE host)", data(ble_chunk)),
            ("Status, stopped ring 4 MB", RecordResponse::Status(stopped)),
        ],
    );
}

fn raw_block_representatives(
    blocks: &RawBlocks,
) -> Vec<Representative<'static, RawTelemetryBlock>> {
    let mut representatives = Vec::new();
    let mut add = |description, block: &[u8]| {
        representatives.push((
            description,
            RawTelemetryBlock {
                data: heapless::Vec::from_slice(block).expect("a raw block fits"),
            },
        ));
    };
    if let Some(full) = Spread::of(&blocks.delta_full) {
        add("Delta28 64 rec, median", full.median);
        add("Delta28 64 rec, smallest", full.smallest);
        add("Delta28 64 rec, largest", full.largest);
    }
    if let Some(ble) = Spread::of(&blocks.delta_ble) {
        add("Delta28 15 rec (BLE limit), median", ble.median);
        add("Delta28 15 rec, smallest", ble.smallest);
        add("Delta28 15 rec, largest", ble.largest);
    }
    add("END block", &blocks.end);
    representatives
}

fn ergot_control_plane(catalog: &mut Catalog) {
    let g = Group::Ergot;
    let token = [0x9e; 8];
    let node = NodeClaimAssignment {
        node_id: 19,
        net_id: 3,
        expires_seconds: 30,
        max_refresh_seconds: 120,
        min_refresh_seconds: 62,
        refresh_token: token,
    };
    let seed_net = |expires_seconds| SeedNetAssignment {
        net_id: 9,
        expires_seconds,
        max_refresh_seconds: 120,
        min_refresh_seconds: 62,
        refresh_token: token,
    };
    catalog.endpoint::<ErgotAddressClaimEndpoint>(
        g,
        CLAIM_REQUEST,
        BUS_RESPONSE,
        vec![(
            "candidate 19, chip nonce",
            AddressClaimRequest {
                candidate_node_id: 19,
                nonce: 0x7c2a_91d4_e5f0_3b18,
            },
        )],
        vec![(
            "granted 3.19",
            Ok(AddressClaimGranted {
                assignment: node.clone(),
                refresh_port: 4,
            }),
        )],
    );
    catalog.endpoint::<ErgotAddressRefreshEndpoint>(
        g,
        BUS_KNOWN_PORT_REQUEST,
        BUS_RESPONSE,
        vec![(
            "node 19",
            AddressRefreshRequest {
                node_id: 19,
                refresh_token: token,
            },
        )],
        vec![(
            "refreshed 120 s",
            Ok(NodeClaimAssignment {
                expires_seconds: 120,
                ..node
            }),
        )],
    );
    catalog.endpoint::<ErgotSeedRouterAssignmentEndpoint>(
        g,
        BUS_ANONYMOUS_REQUEST,
        BUS_RESPONSE,
        vec![("()", ())],
        vec![(
            "net 9, 30 s",
            Ok(SeedRouterAssignment {
                assignment: seed_net(30),
                refresh_port: 5,
                release_port: 6,
            }),
        )],
    );
    catalog.endpoint::<ErgotSeedRouterRefreshEndpoint>(
        g,
        BUS_KNOWN_PORT_REQUEST,
        BUS_RESPONSE,
        vec![(
            "net 9",
            SeedRouterRefreshRequest {
                refresh_net: 9,
                refresh_token: token,
            },
        )],
        vec![("120 s", Ok(seed_net(120)))],
    );
    catalog.endpoint::<ErgotSeedRouterReleaseEndpoint>(
        g,
        BUS_KNOWN_PORT_REQUEST,
        BUS_RESPONSE,
        vec![],
        vec![],
    );
    catalog.endpoint::<ErgotPingEndpoint>(
        g,
        ANONYMOUS_REQUEST,
        RESPONSE,
        vec![("seq 1000", 1000)],
        vec![("seq 1000", 1000)],
    );
    catalog.endpoint::<ErgotPathMtuEndpoint>(g, ANONYMOUS_REQUEST, RESPONSE, vec![], vec![]);
    catalog.topic::<ErgotSocketQueryTopic>(
        g,
        SOCKET_QUERY,
        vec![(
            "MotorEndpoint key, NameRequirement::Any",
            SocketQuery {
                key: MotorEndpoint::REQ_KEY.to_bytes(),
                nash_req: NameRequirement::Any,
                frame_kind: FrameKind::ENDPOINT_REQ,
                broadcast: false,
            },
        )],
    );
    catalog.topic::<ErgotSocketQueryResponseTopic>(
        g,
        TOPIC_UNICAST_KNOWN_PORT,
        vec![(
            "name hash, port 5",
            SocketQueryResponse {
                name: Some(NameHash::new("motor")),
                port: KNOWN_PORT,
            },
        )],
    );
    catalog.topic::<ErgotDeviceInfoTopic>(g, TOPIC_UNICAST_KNOWN_PORT, vec![]);
}

fn timing() -> TelemetryConfigAck {
    TelemetryConfigAck {
        actual_fast_hz: FAST_HZ,
        decimation_m: NonZeroU32::new(1),
        control_timing: ControlTiming {
            clock_hz: 168_000_000,
            period_ticks: 8_400,
        },
    }
}

fn heapless_string<const N: usize>(s: &str) -> heapless::String<N> {
    s.try_into().expect("the string fits its capacity")
}

/// A fault snapshot with the first `count` of three faults.
fn faults(count: usize) -> FaultSnapshot {
    let all = [
        (
            FaultCategory::HallError,
            "edges stopped at speed - dead cable/sensor",
        ),
        (FaultCategory::OverTemp, ""),
        (FaultCategory::UnderVoltage, ""),
    ];
    let mut faults = heapless::Vec::new();
    for (category, details) in &all[..count] {
        faults
            .push(FaultInfo::new(*category, details))
            .expect("three faults fit");
    }
    FaultSnapshot {
        generation: 3,
        faults,
        total: count as u8,
    }
}

//! One-shot bench firmware for the MK5 fixture. Flash the normal remote image after testing.
#![no_std]
#![no_main]

use core::pin::pin;

use defmt::{info, warn};
use embassy_time::{Duration, Instant, Timer, with_deadline, with_timeout};
use ergot::{
    Address, FrameKind,
    interface_manager::{InterfaceState, Profile},
    traits::Endpoint,
    well_known::{
        ErgotSocketQueryResponseTopic, ErgotSocketQueryTopic, NameRequirement, SocketQuery,
    },
};
use oxifoc_core::{
    icd::{HardwareInfoEndpoint, MotorEndpoint, SlowTelemetryEndpoint},
    types::{
        ControlMode, ICD_BOOTSTRAP_MAGIC, ICD_PROTO_VERSION, MotorCommand, MotorCommandOutcome,
        MotorRequest, MotorState, MotorStatus,
    },
};
use oxifoc_remote::transport::Stack;
use panic_rtt_target as _;

#[path = "../remote.rs"]
mod remote;

const BENCH_UUID: &str = "2B002C001047343031363330";
const DRIVE_TIME: Duration = Duration::from_secs(10);
const AFFIRM_PERIOD: Duration = Duration::from_millis(50);
const ACK_TIMEOUT: Duration = Duration::from_millis(100);
const SETPOINT_SEQ: u32 = 1;
const DRIVE: ControlMode = ControlMode::CurrentControl {
    iq_target: 1.0,
    id_target: 0.0,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    remote::start(Some(|spawner, stack, session| {
        spawner.spawn(run(stack, session).expect("allocate motor-test task"));
    }));
}

#[embassy_executor::task]
async fn run(stack: &'static Stack, session: u64) {
    info!("[motor-test] waiting for bench controller");
    let Ok(Some(addr)) = with_timeout(Duration::from_secs(20), find_motor(stack)).await else {
        warn!("[motor-test] could not identify a single motor controller");
        return;
    };
    let Ok(Ok(hw)) = with_timeout(
        Duration::from_secs(2),
        stack
            .endpoints()
            .request::<HardwareInfoEndpoint>(addr, &(), None),
    )
    .await
    else {
        warn!("[motor-test] hardware info unavailable");
        return;
    };
    if hw.uuid.as_str() != BENCH_UUID
        || hw.bootstrap_magic != ICD_BOOTSTRAP_MAGIC
        || hw.proto_version != ICD_PROTO_VERSION
    {
        warn!(
            "[motor-test] unexpected controller {} or ICD version {}",
            hw.uuid.as_str(),
            hw.proto_version
        );
        return;
    }
    let Ok(Ok(before)) = with_timeout(
        Duration::from_secs(2),
        stack
            .endpoints()
            .request::<SlowTelemetryEndpoint>(addr, &(), None),
    )
    .await
    else {
        warn!("[motor-test] initial telemetry unavailable");
        return;
    };
    if before.motor_state != MotorState::Stopped
        || before.control_mode != ControlMode::Stopped
        || before.fault_count != 0
    {
        warn!("[motor-test] controller is not stopped and fault-free");
        return;
    }

    let mut seq = SETPOINT_SEQ;
    let started = Instant::now();
    let deadline = started + DRIVE_TIME;
    let initial = motor_command(
        stack,
        addr,
        session,
        seq,
        MotorCommand::SetMode(DRIVE),
        started + ACK_TIMEOUT,
    )
    .await;
    if initial.is_some_and(|status| {
        !status.outcome.is_success() && status.outcome != MotorCommandOutcome::TimedOut
    }) {
        warn!("[motor-test] start rejected: {:?}", initial);
        return;
    }

    let mut affirms = 0u32;
    if initial.is_some_and(driving) {
        info!("[motor-test] started: {:?}", DRIVE);
        let mut next = started + AFFIRM_PERIOD;
        loop {
            Timer::at(next.min(deadline)).await;
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            seq += 1;
            let ack = motor_command(
                stack,
                addr,
                session,
                seq,
                MotorCommand::Affirm {
                    setpoint_seq: SETPOINT_SEQ,
                },
                (now + ACK_TIMEOUT).min(deadline),
            )
            .await;
            if !ack.is_some_and(driving) {
                if ack.is_some() || Instant::now() < deadline {
                    warn!("[motor-test] lost drive confirmation: {:?}", ack);
                }
                break;
            }
            affirms += 1;
            next = now + AFFIRM_PERIOD;
        }
    } else {
        warn!("[motor-test] start was not confirmed: {:?}", initial);
    }

    let elapsed = started.elapsed();
    // Stop even when a start acknowledgment was lost. With a lost link the controller's
    // drive deadman expires because this task stops affirming and never starts again.
    let stopped = motor_command(
        stack,
        addr,
        session,
        seq + 1,
        MotorCommand::SetMode(ControlMode::Stopped),
        Instant::now() + ACK_TIMEOUT,
    )
    .await;
    if let Some(status) = stopped {
        info!(
            "[motor-test] stop command after {} ms, {} affirms: {:?}",
            elapsed.as_millis(),
            affirms,
            status.outcome
        );
    } else {
        warn!(
            "[motor-test] stop not acknowledged after {} ms",
            elapsed.as_millis()
        );
    }

    // The command acknowledgment may be produced before the published state changes.
    Timer::after_millis(150).await;
    match with_timeout(
        Duration::from_secs(1),
        stack
            .endpoints()
            .request::<SlowTelemetryEndpoint>(addr, &(), None),
    )
    .await
    {
        Ok(Ok(status))
            if status.motor_state == MotorState::Stopped
                && status.control_mode == ControlMode::Stopped =>
        {
            info!("[motor-test] motor stopped, faults: {}", status.fault_count);
        }
        Ok(Ok(status)) => warn!(
            "[motor-test] unexpected state after stop: {:?}, {:?}, {} faults",
            status.motor_state, status.control_mode, status.fault_count
        ),
        _ => warn!("[motor-test] post-stop telemetry unavailable"),
    }
}

fn driving(status: MotorStatus) -> bool {
    status.outcome.is_success()
        && status.state == MotorState::Running
        && status.mode == DRIVE
        && status.fault_count == 0
}

async fn motor_command(
    stack: &Stack,
    addr: Address,
    session: u64,
    seq: u32,
    command: MotorCommand,
    deadline: Instant,
) -> Option<MotorStatus> {
    let request = MotorRequest {
        source_session: session,
        seq,
        command,
    };
    match with_deadline(
        deadline,
        stack
            .endpoints()
            .request::<MotorEndpoint>(addr, &request, Some("motor")),
    )
    .await
    {
        Ok(Ok(status)) => Some(status),
        Ok(Err(error)) => {
            warn!(
                "[motor-test] motor request failed: {:?}",
                defmt::Debug2Format(&error)
            );
            None
        }
        Err(_) => None,
    }
}

async fn find_motor(stack: &Stack) -> Option<Address> {
    while !stack.manage_profile(|profile| {
        matches!(
            profile.interface_state(()),
            Some(InterfaceState::Active { .. } | InterfaceState::ActiveLocal { .. })
        )
    }) {
        Timer::after_millis(100).await;
    }

    let mut receiver = pin!(
        stack
            .topics()
            .bounded_receiver::<ErgotSocketQueryResponseTopic, 4>(None)
    );
    let mut replies = receiver.as_mut().subscribe_unicast();
    let query = SocketQuery {
        key: MotorEndpoint::REQ_KEY.to_bytes(),
        nash_req: NameRequirement::Any,
        frame_kind: FrameKind::ENDPOINT_REQ,
        broadcast: false,
    };
    let mut found = None;
    while found.is_none() {
        let _ = stack
            .topics()
            .broadcast_with_src_port::<ErgotSocketQueryTopic>(&query, None, replies.port());
        let scan_end = Instant::now() + Duration::from_millis(600);
        while let Ok(reply) = with_deadline(scan_end, replies.recv()).await {
            let addr = Address {
                port_id: 0,
                ..reply.hdr.src
            };
            if found.is_some_and(|other| other != addr) {
                return None;
            }
            found = Some(addr);
        }
    }
    found
}

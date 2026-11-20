//! Simulated bridge pairing service (`--simulate-pairing`), for testing the
//! pairing screens without hardware. Mirrors the bridge's request and event
//! sequence (docs/notes/ble-pairing.md) with in-memory bonds.
//!
//! A few seconds after a window opens, a fake candidate connects. For the
//! Remote role it "shows" a passkey, printed to this terminal; for the Host
//! role it asks for Numeric Comparison. Nothing is persisted.

use std::sync::Mutex;
use std::time::Duration;

use ergot::socket::HeaderMessage;
use oxifoc_core::icd::{PairingEndpoint, PairingEventTopic};
use oxifoc_core::types::pairing::{
    BondEntry, BondRole, MAX_BONDS, MAX_WINDOW_SECS, PairingError, PairingEvent, PairingFailure,
    PairingRequest, PairingResponse, WindowCloseReason,
};
use tracing::{info, warn};

use crate::tcp_server::RouterStack;

/// Delay before the fake candidate connects.
const CANDIDATE_DELAY: Duration = Duration::from_secs(3);

#[derive(Clone, Copy, PartialEq)]
enum Expect {
    Nothing,
    Passkey(u32),
    Comparison,
}

struct Window {
    attempt: u32,
    role: BondRole,
    name: heapless::String<24>,
    added_by: Option<u8>,
    expect: Expect,
}

struct Sim {
    bonds: [Option<BondEntry>; MAX_BONDS],
    window: Option<Window>,
    next_attempt: u32,
}

static SIM: Mutex<Sim> = Mutex::new(Sim {
    bonds: [const { None }; MAX_BONDS],
    window: None,
    next_attempt: 1,
});

fn publish(stack: &RouterStack, event: PairingEvent) {
    info!("[pairing-sim] {event:?}");
    if stack
        .topics()
        .broadcast::<PairingEventTopic>(&event, None)
        .is_err()
    {
        warn!("[pairing-sim] event not sent");
    }
}

/// Close `attempt` if it is still open, publishing the failure first.
fn close(
    stack: &RouterStack,
    attempt: u32,
    reason: WindowCloseReason,
    failure: Option<PairingFailure>,
) {
    let closed = {
        let mut sim = SIM.lock().unwrap();
        sim.window.take_if(|w| w.attempt == attempt).is_some()
    };
    if closed {
        if let Some(reason) = failure {
            publish(stack, PairingEvent::Failed { attempt, reason });
        }
        publish(stack, PairingEvent::WindowClosed { attempt, reason });
    }
}

fn bond(stack: &RouterStack, attempt: u32) {
    let entry = {
        let mut sim = SIM.lock().unwrap();
        let Some(window) = sim.window.as_ref().filter(|w| w.attempt == attempt) else {
            return;
        };
        let Some(id) = sim.bonds.iter().position(Option::is_none) else {
            return;
        };
        let mut address: [u8; 6] = rand::random();
        address[5] |= 0xC0; // random static
        let entry = BondEntry {
            id: id as u8,
            role: window.role,
            name: window.name.clone(),
            address,
            address_random: true,
            added_by: window.added_by,
        };
        sim.bonds[id] = Some(entry.clone());
        entry
    };
    publish(stack, PairingEvent::Bonded(entry));
    close(stack, attempt, WindowCloseReason::Bonded, None);
}

/// The fake candidate's side of one window.
async fn candidate(stack: RouterStack, attempt: u32, seconds: u16) {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(seconds.into());
    let script = async {
        tokio::time::sleep(CANDIDATE_DELAY).await;
        let mut address: [u8; 6] = rand::random();
        address[5] = (address[5] & 0x3F) | 0x40; // resolvable private
        publish(
            &stack,
            PairingEvent::CandidateConnected { attempt, address },
        );
        tokio::time::sleep(Duration::from_millis(400)).await;
        let event = {
            let mut sim = SIM.lock().unwrap();
            let Some(window) = sim.window.as_mut().filter(|w| w.attempt == attempt) else {
                return;
            };
            match window.role {
                BondRole::Remote => {
                    let passkey = rand::random_range(0..1_000_000);
                    window.expect = Expect::Passkey(passkey);
                    eprintln!("\n  simulated remote shows passkey {passkey:06}\n");
                    PairingEvent::EnterPasskey { attempt }
                }
                BondRole::Host => {
                    window.expect = Expect::Comparison;
                    PairingEvent::CompareNumber {
                        attempt,
                        number: rand::random_range(0..1_000_000),
                    }
                }
            }
        };
        publish(&stack, event);
        std::future::pending::<()>().await;
    };
    if tokio::time::timeout_at(deadline, script).await.is_err() {
        let pairing = SIM
            .lock()
            .unwrap()
            .window
            .as_ref()
            .is_some_and(|w| w.attempt == attempt && w.expect != Expect::Nothing);
        close(
            &stack,
            attempt,
            WindowCloseReason::Timeout,
            pairing.then_some(PairingFailure::Cancelled),
        );
    }
}

/// Answer after a short delay, as the real Security Manager does.
fn answer(stack: &RouterStack, attempt: u32, ok: bool, failure: PairingFailure) {
    let stack = stack.clone();
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(300)).await;
        if ok {
            bond(&stack, attempt);
        } else {
            close(&stack, attempt, WindowCloseReason::Failed, Some(failure));
        }
    });
}

fn handle(stack: &RouterStack, request: &PairingRequest) -> Result<PairingResponse, PairingError> {
    let mut sim = SIM.lock().unwrap();
    match request {
        PairingRequest::OpenWindow {
            role,
            seconds,
            name,
        } => {
            if sim.bonds.iter().all(Option::is_some) {
                return Err(PairingError::TableFull);
            }
            if sim.window.is_some() {
                return Err(PairingError::WindowBusy);
            }
            let attempt = sim.next_attempt;
            sim.next_attempt += 1;
            let seconds = (*seconds).clamp(1, MAX_WINDOW_SECS);
            sim.window = Some(Window {
                attempt,
                role: *role,
                name: name.clone(),
                added_by: None,
                expect: Expect::Nothing,
            });
            drop(sim);
            let expires_ms = u32::from(seconds) * 1000;
            publish(
                stack,
                PairingEvent::WindowOpened {
                    attempt,
                    role: *role,
                    expires_ms,
                },
            );
            tokio::spawn(candidate(stack.clone(), attempt, seconds));
            Ok(PairingResponse::WindowOpened {
                attempt,
                expires_ms,
            })
        }
        PairingRequest::SubmitPasskey { attempt, passkey } => {
            let window = sim.window.as_mut().filter(|w| w.attempt == *attempt);
            let Some(Expect::Passkey(expected)) = window.as_ref().map(|w| w.expect) else {
                return Err(PairingError::NoSuchAttempt);
            };
            window.unwrap().expect = Expect::Nothing;
            answer(
                stack,
                *attempt,
                *passkey == expected,
                PairingFailure::WrongCode,
            );
            Ok(PairingResponse::Ok)
        }
        PairingRequest::ConfirmComparison { attempt, accept } => {
            let Some(window) = sim
                .window
                .as_mut()
                .filter(|w| w.attempt == *attempt && w.expect == Expect::Comparison)
            else {
                return Err(PairingError::NoSuchAttempt);
            };
            window.expect = Expect::Nothing;
            answer(stack, *attempt, *accept, PairingFailure::Rejected);
            Ok(PairingResponse::Ok)
        }
        PairingRequest::Cancel { attempt } => {
            if sim.window.as_ref().is_none_or(|w| w.attempt != *attempt) {
                return Err(PairingError::NoSuchAttempt);
            }
            drop(sim);
            close(stack, *attempt, WindowCloseReason::Cancelled, None);
            Ok(PairingResponse::Ok)
        }
        PairingRequest::ListBonds => Ok(PairingResponse::Bonds(
            sim.bonds.iter().flatten().cloned().collect(),
        )),
        PairingRequest::RemoveBond { id } => {
            sim.bonds
                .get_mut(usize::from(*id))
                .and_then(Option::take)
                .ok_or(PairingError::NoSuchBond)?;
            drop(sim);
            publish(stack, PairingEvent::BondRemoved { id: *id });
            Ok(PairingResponse::Ok)
        }
        PairingRequest::RenameBond { id, name } => {
            let bond = sim
                .bonds
                .get_mut(usize::from(*id))
                .and_then(Option::as_mut)
                .ok_or(PairingError::NoSuchBond)?;
            bond.name = name.clone();
            Ok(PairingResponse::Ok)
        }
    }
}

pub async fn serve(stack: RouterStack) {
    info!("[pairing-sim] simulated pairing service running");
    let server = stack
        .endpoints()
        .bounded_server::<PairingEndpoint, 2>(Some("pairing-sim"));
    let server = std::pin::pin!(server);
    let mut hdl = server.attach();
    loop {
        let _ = hdl
            .serve_full(async |msg: &HeaderMessage<PairingRequest>| {
                handle(&stack, &msg.t).unwrap_or_else(PairingResponse::Error)
            })
            .await;
    }
}

//! Framework-independent state of a GUI pairing screen: the bridge's bonds,
//! the open window and the prompt it waits for (docs/notes/ble-pairing.md).
//!
//! A GUI feeds it [`PairingEvent`]s from `HostRuntime::pairing_rx` and the
//! results of [`Op::run`], then renders the fields. Requests block, so run
//! [`Op::run`] off the UI thread.

use std::time::{Duration, Instant};

use oxifoc_core::types::pairing::{
    BOND_NAME_LEN, BondEntry, BondRole, MAX_WINDOW_SECS, PairingEvent, PairingFailure,
    WindowCloseReason,
};

use crate::{CommandSender, ops::pairing as ops};

/// Window length requested by the GUI.
pub const WINDOW_SECS: u16 = MAX_WINDOW_SECS;
/// Grace after the deadline before a window without a close event is dropped.
const CLOSE_GRACE: Duration = Duration::from_secs(5);

/// What the open window waits for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// No candidate yet.
    Waiting,
    /// A candidate connected; the Security Manager is running.
    Negotiating,
    /// Type this passkey on the candidate.
    ShowPasskey(u32),
    /// The candidate (a remote) shows a passkey; enter it here.
    EnterPasskey,
    /// Compare this number with the candidate's screen.
    CompareNumber(u32),
    /// The answer was sent; waiting for the result.
    Checking,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Window {
    pub attempt: u32,
    /// `None` until the bridge reports it (a window opened elsewhere).
    pub role: Option<BondRole>,
    /// Name requested by this client, `None` for a window opened elsewhere.
    pub name: Option<String>,
    pub deadline: Instant,
    pub step: Step,
    pub candidate: Option<[u8; 6]>,
    failure: Option<PairingFailure>,
}

impl Window {
    pub fn remaining(&self, now: Instant) -> Duration {
        self.deadline.saturating_duration_since(now)
    }

    /// True when this client opened the window and can answer its prompts.
    pub fn is_own(&self) -> bool {
        self.name.is_some()
    }
}

/// How the last window ended.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Bonded(BondEntry),
    Closed {
        reason: WindowCloseReason,
        failure: Option<PairingFailure>,
    },
}

impl Outcome {
    pub fn is_success(&self) -> bool {
        matches!(self, Self::Bonded(_))
    }

    pub fn message(&self) -> String {
        match self {
            Self::Bonded(entry) => format!("Paired {} ({})", entry.name, role_label(entry.role)),
            Self::Closed {
                failure: Some(f), ..
            } => failure_message(*f).to_string(),
            Self::Closed { reason, .. } => match reason {
                WindowCloseReason::Bonded => "Paired".into(),
                WindowCloseReason::Failed => "Pairing failed".into(),
                WindowCloseReason::Cancelled => "Pairing window cancelled".into(),
                WindowCloseReason::Timeout => "No device paired before the window closed".into(),
            },
        }
    }
}

pub fn failure_message(failure: PairingFailure) -> &'static str {
    match failure {
        PairingFailure::Rejected => "The number was rejected",
        PairingFailure::WrongCode => "The code did not match",
        PairingFailure::Cancelled => "The window closed while pairing",
        PairingFailure::NotAuthenticated => {
            "The device offered no code check (Just Works); it cannot pair"
        }
        PairingFailure::NotBonded => "The device did not store a bond",
        PairingFailure::Protocol => "The Bluetooth security exchange failed",
        PairingFailure::Disconnected => "The device disconnected while pairing",
        PairingFailure::Storage => "The bridge could not store the bond",
    }
}

pub fn role_label(role: BondRole) -> &'static str {
    match role {
        BondRole::Remote => "Remote",
        BondRole::Host => "Phone or computer",
    }
}

/// Little-endian wire address in the usual most-significant-first form.
pub fn format_address(address: &[u8; 6]) -> String {
    address
        .iter()
        .rev()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(":")
}

/// A six-digit passkey; spaces are ignored.
pub fn parse_passkey(text: &str) -> Result<u32, &'static str> {
    let digits: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    if digits.len() != 6 || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return Err("Enter the six digits shown on the device");
    }
    digits
        .parse()
        .map_err(|_| "Enter the six digits shown on the device")
}

/// Bond name as the bridge stores it.
pub fn validate_name(text: &str) -> Result<String, String> {
    let name = text.trim();
    if name.is_empty() {
        Err("Enter a name".into())
    } else if name.len() > BOND_NAME_LEN {
        Err(format!("Use at most {BOND_NAME_LEN} bytes"))
    } else {
        Ok(name.to_string())
    }
}

/// `123456` as `123 456`, easier to compare.
pub fn format_code(code: u32) -> String {
    format!("{:03} {:03}", code / 1000 % 1000, code % 1000)
}

/// One request to the bridge's pairing endpoint.
#[derive(Clone, Debug, PartialEq)]
pub enum Op {
    List,
    Open { role: BondRole, name: String },
    Passkey { attempt: u32, passkey: u32 },
    Confirm { attempt: u32, accept: bool },
    Cancel { attempt: u32 },
    Remove { id: u8 },
    Rename { id: u8, name: String },
}

/// Successful result of an [`Op`].
#[derive(Clone, Debug, PartialEq)]
pub enum Reply {
    Bonds(Vec<BondEntry>),
    Opened { attempt: u32, expires_ms: u32 },
    Done,
}

impl Op {
    /// Send the request and wait for the answer (up to a few seconds).
    pub fn run(&self, cmd: &CommandSender) -> anyhow::Result<Reply> {
        Ok(match self {
            Self::List => {
                // Opened right after connecting, the link may not be up yet.
                ops::wait_for_bridge(cmd, Duration::from_secs(5))?;
                Reply::Bonds(ops::list(cmd)?)
            }
            Self::Open { role, name } => {
                let (attempt, expires_ms) = ops::open_window(cmd, *role, WINDOW_SECS, name)?;
                Reply::Opened {
                    attempt,
                    expires_ms,
                }
            }
            Self::Passkey { attempt, passkey } => {
                ops::submit_passkey(cmd, *attempt, *passkey)?;
                Reply::Done
            }
            Self::Confirm { attempt, accept } => {
                ops::confirm(cmd, *attempt, *accept)?;
                Reply::Done
            }
            Self::Cancel { attempt } => {
                ops::cancel(cmd, *attempt)?;
                Reply::Done
            }
            Self::Remove { id } => {
                ops::remove(cmd, *id)?;
                Reply::Done
            }
            Self::Rename { id, name } => {
                ops::rename(cmd, *id, name)?;
                Reply::Done
            }
        })
    }

    fn label(&self) -> &'static str {
        match self {
            Self::List => "Reading bonds",
            Self::Open { .. } => "Opening the window",
            Self::Passkey { .. } => "Sending the code",
            Self::Confirm { .. } => "Sending the answer",
            Self::Cancel { .. } => "Cancelling",
            Self::Remove { .. } => "Removing the bond",
            Self::Rename { .. } => "Renaming the bond",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct PairingDesk {
    /// `None` until the first successful list.
    pub bonds: Option<Vec<BondEntry>>,
    pub window: Option<Window>,
    pub outcome: Option<Outcome>,
    /// Label of the request in flight.
    pub busy: Option<&'static str>,
    pub error: Option<String>,
}

impl PairingDesk {
    /// Start `op` unless another request is in flight. Returns false when busy.
    pub fn begin(&mut self, op: &Op) -> bool {
        if self.busy.is_some() {
            return false;
        }
        self.busy = Some(op.label());
        self.error = None;
        if matches!(op, Op::Open { .. }) {
            self.outcome = None;
        }
        true
    }

    pub fn finish(&mut self, op: &Op, result: anyhow::Result<Reply>, now: Instant) {
        self.busy = None;
        let reply = match result {
            Ok(reply) => reply,
            Err(e) => {
                self.error = Some(error_message(op, &e));
                return;
            }
        };
        match (op, reply) {
            (Op::List, Reply::Bonds(bonds)) => self.bonds = Some(bonds),
            (
                Op::Open { role, name },
                Reply::Opened {
                    attempt,
                    expires_ms,
                },
            ) => {
                // The WindowOpened event may arrive before or after this reply.
                let window = self.window_for(attempt, expires_ms, now);
                window.role = Some(*role);
                window.name = Some(name.clone());
            }
            (Op::Passkey { attempt, .. } | Op::Confirm { attempt, .. }, Reply::Done) => {
                if let Some(window) = self.window.as_mut().filter(|w| w.attempt == *attempt) {
                    window.step = Step::Checking;
                }
            }
            (Op::Cancel { attempt }, Reply::Done) => {
                // Do not rely on the event: the topic is best effort.
                self.close(*attempt, WindowCloseReason::Cancelled);
            }
            (Op::Remove { id }, Reply::Done) => self.remove_bond(*id),
            (Op::Rename { id, name }, Reply::Done) => {
                if let Some(bond) = self.bond_mut(*id) {
                    bond.name = name.as_str().try_into().unwrap_or_default();
                }
            }
            (op, reply) => self.error = Some(format!("Unexpected answer to {op:?}: {reply:?}")),
        }
    }

    /// Apply one bridge event.
    pub fn apply(&mut self, event: &PairingEvent, now: Instant) {
        match event {
            PairingEvent::WindowOpened {
                attempt,
                role,
                expires_ms,
            } => {
                self.window_for(*attempt, *expires_ms, now).role = Some(*role);
            }
            PairingEvent::WindowClosed { attempt, reason } => self.close(*attempt, *reason),
            PairingEvent::CandidateConnected { attempt, address } => {
                if let Some(window) = self.window_mut(*attempt) {
                    window.candidate = Some(*address);
                    window.step = Step::Negotiating;
                }
            }
            PairingEvent::ShowPasskey { attempt, passkey } => {
                self.set_step(*attempt, Step::ShowPasskey(*passkey));
            }
            PairingEvent::EnterPasskey { attempt } => self.set_step(*attempt, Step::EnterPasskey),
            PairingEvent::CompareNumber { attempt, number } => {
                self.set_step(*attempt, Step::CompareNumber(*number));
            }
            PairingEvent::Bonded(entry) => {
                if let Some(bonds) = self.bonds.as_mut() {
                    bonds.retain(|b| b.id != entry.id);
                    bonds.push(entry.clone());
                    bonds.sort_by_key(|b| b.id);
                }
                self.outcome = Some(Outcome::Bonded(entry.clone()));
            }
            PairingEvent::BondRemoved { id } => self.remove_bond(*id),
            PairingEvent::Failed { attempt, reason } => {
                if let Some(window) = self.window_mut(*attempt) {
                    window.failure = Some(*reason);
                }
            }
        }
    }

    /// Drop a window whose close event never arrived.
    pub fn tick(&mut self, now: Instant) {
        if let Some(window) = &self.window
            && now >= window.deadline + CLOSE_GRACE
        {
            let attempt = window.attempt;
            self.close(attempt, WindowCloseReason::Timeout);
        }
    }

    /// Forget bridge state, for example after the connection changed.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    fn window_for(&mut self, attempt: u32, expires_ms: u32, now: Instant) -> &mut Window {
        if self.window.as_ref().is_none_or(|w| w.attempt != attempt) {
            self.outcome = None;
            self.window = Some(Window {
                attempt,
                role: None,
                name: None,
                deadline: now + Duration::from_millis(expires_ms.into()),
                step: Step::Waiting,
                candidate: None,
                failure: None,
            });
        }
        self.window.as_mut().expect("window set above")
    }

    fn window_mut(&mut self, attempt: u32) -> Option<&mut Window> {
        self.window.as_mut().filter(|w| w.attempt == attempt)
    }

    fn set_step(&mut self, attempt: u32, step: Step) {
        if let Some(window) = self.window_mut(attempt) {
            window.step = step;
        }
    }

    fn close(&mut self, attempt: u32, reason: WindowCloseReason) {
        let Some(window) = self.window.take_if(|w| w.attempt == attempt) else {
            return;
        };
        // Keep the Bonded outcome, which carries the new bond.
        if !(reason == WindowCloseReason::Bonded
            && matches!(self.outcome, Some(Outcome::Bonded(_))))
        {
            self.outcome = Some(Outcome::Closed {
                reason,
                failure: window.failure,
            });
        }
    }

    fn bond_mut(&mut self, id: u8) -> Option<&mut BondEntry> {
        self.bonds.as_mut()?.iter_mut().find(|b| b.id == id)
    }

    fn remove_bond(&mut self, id: u8) {
        if let Some(bonds) = self.bonds.as_mut() {
            bonds.retain(|b| b.id != id);
        }
    }
}

fn error_message(op: &Op, error: &anyhow::Error) -> String {
    let what = match op {
        Op::List => "Could not read bonds",
        Op::Open { .. } => "Could not open a pairing window",
        Op::Passkey { .. } => "Could not send the code",
        Op::Confirm { .. } => "Could not send the answer",
        Op::Cancel { .. } => "Could not cancel the window",
        Op::Remove { .. } => "Could not remove the bond",
        Op::Rename { .. } => "Could not rename the bond",
    };
    match error.downcast_ref::<ops::Refused>() {
        Some(refused) => format!("{what}: {refused}."),
        None => format!("{what}: {error:#}."),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxifoc_core::types::pairing::PairingError;

    fn entry(id: u8, name: &str) -> BondEntry {
        BondEntry {
            id,
            role: BondRole::Remote,
            name: name.try_into().unwrap(),
            address: [1, 2, 3, 4, 5, 0xC6],
            address_random: true,
            added_by: None,
        }
    }

    fn open_desk(now: Instant) -> (PairingDesk, Op) {
        let mut desk = PairingDesk::default();
        let op = Op::Open {
            role: BondRole::Remote,
            name: "remote".into(),
        };
        assert!(desk.begin(&op));
        desk.finish(
            &op,
            Ok(Reply::Opened {
                attempt: 7,
                expires_ms: 60_000,
            }),
            now,
        );
        (desk, op)
    }

    #[test]
    fn passkey_flow_ends_bonded() {
        let now = Instant::now();
        let (mut desk, _) = open_desk(now);
        desk.bonds = Some(vec![entry(0, "laptop")]);
        desk.apply(
            &PairingEvent::WindowOpened {
                attempt: 7,
                role: BondRole::Remote,
                expires_ms: 60_000,
            },
            now,
        );
        let window = desk.window.as_ref().unwrap();
        assert!(window.is_own(), "event after the reply keeps our name");
        assert_eq!(window.step, Step::Waiting);

        desk.apply(
            &PairingEvent::CandidateConnected {
                attempt: 7,
                address: [0; 6],
            },
            now,
        );
        desk.apply(&PairingEvent::EnterPasskey { attempt: 7 }, now);
        assert_eq!(desk.window.as_ref().unwrap().step, Step::EnterPasskey);

        let op = Op::Passkey {
            attempt: 7,
            passkey: 123_456,
        };
        assert!(desk.begin(&op));
        desk.finish(&op, Ok(Reply::Done), now);
        assert_eq!(desk.window.as_ref().unwrap().step, Step::Checking);

        desk.apply(&PairingEvent::Bonded(entry(1, "remote")), now);
        desk.apply(
            &PairingEvent::WindowClosed {
                attempt: 7,
                reason: WindowCloseReason::Bonded,
            },
            now,
        );
        assert!(desk.window.is_none());
        assert!(desk.outcome.as_ref().unwrap().is_success());
        assert_eq!(desk.bonds.as_ref().unwrap().len(), 2);
    }

    #[test]
    fn event_before_reply_and_failure_reason() {
        let now = Instant::now();
        let mut desk = PairingDesk::default();
        desk.apply(
            &PairingEvent::WindowOpened {
                attempt: 3,
                role: BondRole::Host,
                expires_ms: 60_000,
            },
            now,
        );
        assert!(!desk.window.as_ref().unwrap().is_own());
        let op = Op::Open {
            role: BondRole::Host,
            name: "phone".into(),
        };
        desk.begin(&op);
        desk.finish(
            &op,
            Ok(Reply::Opened {
                attempt: 3,
                expires_ms: 60_000,
            }),
            now,
        );
        assert!(desk.window.as_ref().unwrap().is_own());

        desk.apply(
            &PairingEvent::Failed {
                attempt: 3,
                reason: PairingFailure::WrongCode,
            },
            now,
        );
        desk.apply(
            &PairingEvent::WindowClosed {
                attempt: 3,
                reason: WindowCloseReason::Failed,
            },
            now,
        );
        assert_eq!(
            desk.outcome,
            Some(Outcome::Closed {
                reason: WindowCloseReason::Failed,
                failure: Some(PairingFailure::WrongCode),
            })
        );
    }

    #[test]
    fn busy_refusal_and_lost_close_event() {
        let now = Instant::now();
        let (mut desk, op) = open_desk(now);
        assert!(desk.begin(&Op::List));
        assert!(!desk.begin(&op), "one request at a time");
        desk.finish(
            &Op::List,
            Err(ops::Refused(PairingError::TableFull).into()),
            now,
        );
        assert!(desk.error.as_ref().unwrap().contains("remove a bond first"));

        desk.tick(now + Duration::from_secs(61));
        assert!(desk.window.is_some(), "grace period");
        desk.tick(now + Duration::from_secs(66));
        assert!(desk.window.is_none());
        assert!(matches!(
            desk.outcome,
            Some(Outcome::Closed {
                reason: WindowCloseReason::Timeout,
                ..
            })
        ));
    }

    #[test]
    fn stale_attempt_events_are_ignored() {
        let now = Instant::now();
        let (mut desk, _) = open_desk(now);
        desk.apply(&PairingEvent::EnterPasskey { attempt: 6 }, now);
        desk.apply(
            &PairingEvent::WindowClosed {
                attempt: 6,
                reason: WindowCloseReason::Timeout,
            },
            now,
        );
        assert_eq!(desk.window.as_ref().unwrap().step, Step::Waiting);
    }

    #[test]
    fn bond_list_edits() {
        let now = Instant::now();
        let mut desk = PairingDesk {
            bonds: Some(vec![entry(0, "a"), entry(2, "b")]),
            ..Default::default()
        };
        let rename = Op::Rename {
            id: 2,
            name: "board".into(),
        };
        desk.begin(&rename);
        desk.finish(&rename, Ok(Reply::Done), now);
        desk.apply(&PairingEvent::BondRemoved { id: 0 }, now);
        let bonds = desk.bonds.unwrap();
        assert_eq!(bonds.len(), 1);
        assert_eq!(bonds[0].name.as_str(), "board");
    }

    #[test]
    fn input_validation() {
        assert_eq!(parse_passkey("012 345"), Ok(12_345));
        assert!(parse_passkey("12345").is_err());
        assert!(parse_passkey("12345a").is_err());
        assert!(validate_name("  ").is_err());
        assert!(validate_name(&"x".repeat(25)).is_err());
        assert_eq!(validate_name(" phone "), Ok("phone".into()));
        assert_eq!(format_code(42), "000 042");
        assert_eq!(format_address(&[1, 2, 3, 4, 5, 0xC6]), "C6:05:04:03:02:01");
    }
}

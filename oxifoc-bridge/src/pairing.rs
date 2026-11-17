//! Pairing window state shared by the BLE link slots and the pairing endpoint
//! (docs/notes/ble-pairing.md).

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};
use heapless::String;
use oxifoc_core::types::pairing::{BOND_NAME_LEN, BondRole, MAX_WINDOW_SECS, PairingError};

use crate::transport::BLE_LINKS;

pub const NO_BOND: u8 = u8::MAX;

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Stage {
    /// Waiting for a candidate to start pairing.
    Open,
    /// A link slot is pairing; it waits for no user input.
    Pairing { link: usize },
    /// The candidate shows a passkey; a trusted device must submit it.
    AwaitPasskey { link: usize },
    /// Numeric Comparison; a trusted device must answer.
    AwaitComparison { link: usize },
}

impl Stage {
    pub fn link(self) -> Option<usize> {
        match self {
            Self::Open => None,
            Self::Pairing { link }
            | Self::AwaitPasskey { link }
            | Self::AwaitComparison { link } => Some(link),
        }
    }
}

#[derive(Clone)]
pub struct Window {
    pub attempt: u32,
    pub role: BondRole,
    pub name: String<BOND_NAME_LEN>,
    pub added_by: Option<u8>,
    pub deadline: Instant,
    pub stage: Stage,
}

/// User input for the link slot that is pairing.
#[derive(Clone, Copy, defmt::Format)]
pub enum Input {
    Passkey(u32),
    Comparison(bool),
    Cancel,
}

pub struct PairingState {
    window: Mutex<CriticalSectionRawMutex, RefCell<Option<Window>>>,
    next_attempt: AtomicU32,
    /// Input for each link slot.
    pub inputs: [Signal<CriticalSectionRawMutex, Input>; BLE_LINKS],
    /// The window opened or closed; wakes the timeout task.
    pub changed: Signal<CriticalSectionRawMutex, ()>,
    /// Bond in use on each link slot, [`NO_BOND`] when untrusted or free.
    pub link_bond: [AtomicU8; BLE_LINKS],
    /// Ask a link slot to disconnect (its bond was removed).
    pub kick: [Signal<CriticalSectionRawMutex, ()>; BLE_LINKS],
}

impl Default for PairingState {
    fn default() -> Self {
        Self::new()
    }
}

impl PairingState {
    pub const fn new() -> Self {
        Self {
            window: Mutex::new(RefCell::new(None)),
            next_attempt: AtomicU32::new(1),
            inputs: [const { Signal::new() }; BLE_LINKS],
            changed: Signal::new(),
            link_bond: [const { AtomicU8::new(NO_BOND) }; BLE_LINKS],
            kick: [const { Signal::new() }; BLE_LINKS],
        }
    }

    pub fn window(&self) -> Option<Window> {
        self.window.lock(|w| w.borrow().clone())
    }

    pub fn is_open(&self) -> bool {
        self.window.lock(|w| w.borrow().is_some())
    }

    /// Open a window; returns the attempt id and its length in ms.
    pub fn open(
        &self,
        role: BondRole,
        name: String<BOND_NAME_LEN>,
        seconds: u16,
        added_by: Option<u8>,
    ) -> Result<(u32, u32), PairingError> {
        let seconds = seconds.clamp(1, MAX_WINDOW_SECS);
        let attempt = self.window.lock(|w| {
            let mut w = w.borrow_mut();
            if w.is_some() {
                return Err(PairingError::WindowBusy);
            }
            let attempt = self.next_attempt.fetch_add(1, Ordering::Relaxed);
            *w = Some(Window {
                attempt,
                role,
                name,
                added_by,
                deadline: Instant::now() + Duration::from_secs(seconds.into()),
                stage: Stage::Open,
            });
            Ok(attempt)
        })?;
        self.changed.signal(());
        Ok((attempt, u32::from(seconds) * 1000))
    }

    /// Close the window if it is still `attempt`.
    pub fn close(&self, attempt: u32) -> Option<Window> {
        let closed = self.window.lock(|w| {
            let mut w = w.borrow_mut();
            if w.as_ref().is_some_and(|w| w.attempt == attempt) {
                w.take()
            } else {
                None
            }
        });
        if closed.is_some() {
            self.changed.signal(());
        }
        closed
    }

    /// Let `link` pair inside the open window. Returns the attempt when the
    /// window is open and no other slot holds it.
    pub fn claim(&self, link: usize) -> Option<u32> {
        self.window.lock(|w| {
            let mut w = w.borrow_mut();
            let window = w.as_mut()?;
            match window.stage.link() {
                None => {
                    window.stage = Stage::Pairing { link };
                    Some(window.attempt)
                }
                Some(owner) if owner == link => Some(window.attempt),
                Some(_) => None,
            }
        })
    }

    pub fn set_stage(&self, attempt: u32, stage: Stage) {
        self.window.lock(|w| {
            if let Some(window) = w.borrow_mut().as_mut()
                && window.attempt == attempt
            {
                window.stage = stage;
            }
        });
    }

    /// Route user input to the link slot that waits for it.
    pub fn submit(&self, attempt: u32, input: Input) -> Result<(), PairingError> {
        let link = self.window.lock(|w| {
            let w = w.borrow();
            let window = w.as_ref().filter(|w| w.attempt == attempt);
            match (window.map(|w| w.stage), input) {
                (Some(Stage::AwaitPasskey { link }), Input::Passkey(_))
                | (Some(Stage::AwaitComparison { link }), Input::Comparison(_)) => Ok(link),
                _ => Err(PairingError::NoSuchAttempt),
            }
        })?;
        self.inputs[link].signal(input);
        Ok(())
    }

    /// Bond id of the trusted link whose router net is `net`.
    pub fn bond_on_net(&self, nets: &[u16; BLE_LINKS], net: u16) -> Option<u8> {
        nets.iter()
            .position(|n| *n == net)
            .map(|link| self.link_bond[link].load(Ordering::Relaxed))
            .filter(|id| *id != NO_BOND)
    }
}

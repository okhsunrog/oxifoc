//! BLE pairing management, served by the bridge (`cmd/pairing`, `events/pairing`).
//!
//! Design: docs/notes/ble-pairing.md. Any trusted Ergot channel (USB, or a BLE
//! link encrypted with a stored bond) may send these requests. New devices pair
//! only inside a window that such a request opens.

use heapless::{String, Vec};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

/// Maximum number of stored bonds on the bridge.
pub const MAX_BONDS: usize = 8;
/// Maximum bond name length in bytes.
pub const BOND_NAME_LEN: usize = 24;
/// Bond name as stored and reported.
pub type BondName = String<BOND_NAME_LEN>;
/// Longest pairing window a request may open.
pub const MAX_WINDOW_SECS: u16 = 60;

/// Role recorded with a bond. Drive enforcement by role is not active yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BondRole {
    /// A handheld remote: may become the drive controller.
    Remote,
    /// A phone or laptop.
    Host,
}

/// One stored bond, as reported to clients. Keys are never reported.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BondEntry {
    /// Stable slot id, `0..MAX_BONDS`.
    pub id: u8,
    pub role: BondRole,
    pub name: String<BOND_NAME_LEN>,
    /// Identity address, little endian as on air.
    pub address: [u8; 6],
    /// True for a random static identity, false for a public one.
    pub address_random: bool,
    /// Bond id of the device that opened the pairing window, if it was a bond.
    pub added_by: Option<u8>,
}

/// Pairing management request.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingRequest {
    /// Open a pairing window for one new device with the given role.
    /// `seconds` is clamped to [`MAX_WINDOW_SECS`].
    OpenWindow {
        role: BondRole,
        seconds: u16,
        /// Name stored with the new bond.
        name: String<BOND_NAME_LEN>,
    },
    /// Passkey shown by the candidate (a remote), typed on a trusted device.
    SubmitPasskey {
        attempt: u32,
        passkey: u32,
    },
    /// Numeric Comparison answer from a trusted device.
    ConfirmComparison {
        attempt: u32,
        accept: bool,
    },
    /// Close the window and abort any pairing in progress.
    Cancel {
        attempt: u32,
    },
    ListBonds,
    RemoveBond {
        id: u8,
    },
    RenameBond {
        id: u8,
        name: String<BOND_NAME_LEN>,
    },
}

/// Pairing management response.
#[allow(
    clippy::large_enum_variant,
    reason = "wire type for postcard; no heap to box the bond list in no_std"
)]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingResponse {
    WindowOpened { attempt: u32, expires_ms: u32 },
    Ok,
    Bonds(Vec<BondEntry, MAX_BONDS>),
    Error(PairingError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingError {
    /// All [`MAX_BONDS`] slots are used; remove a bond first.
    TableFull,
    /// Another window is open.
    WindowBusy,
    /// The attempt is not the current one, or it is not waiting for this input.
    NoSuchAttempt,
    NoSuchBond,
    /// The motor controller is not stopped, or its state is unknown.
    NotStopped,
    /// Flash storage failed.
    Storage,
}

/// Why a pairing window closed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WindowCloseReason {
    Bonded,
    Failed,
    Cancelled,
    Timeout,
}

/// Why a pairing attempt failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingFailure {
    /// Rejected on a trusted device (Numeric Comparison answered no).
    Rejected,
    /// The passkey or comparison did not match (wrong code typed).
    WrongCode,
    /// The window closed or was cancelled while pairing was in progress.
    Cancelled,
    /// The candidate offered no MITM-protected method (Just Works).
    NotAuthenticated,
    /// The candidate did not create a bond.
    NotBonded,
    /// The Security Manager reported an error.
    Protocol,
    /// The candidate disconnected.
    Disconnected,
    Storage,
}

/// Pairing progress and bond changes, published to every trusted channel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Schema)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PairingEvent {
    WindowOpened {
        attempt: u32,
        role: BondRole,
        expires_ms: u32,
    },
    WindowClosed {
        attempt: u32,
        reason: WindowCloseReason,
    },
    CandidateConnected {
        attempt: u32,
        address: [u8; 6],
    },
    /// Show this passkey; the user types it on the candidate.
    ShowPasskey {
        attempt: u32,
        passkey: u32,
    },
    /// The candidate shows a passkey; submit it with
    /// [`PairingRequest::SubmitPasskey`].
    EnterPasskey {
        attempt: u32,
    },
    /// Compare with the number on the candidate and answer with
    /// [`PairingRequest::ConfirmComparison`].
    CompareNumber {
        attempt: u32,
        number: u32,
    },
    Bonded(BondEntry),
    BondRemoved {
        id: u8,
    },
    Failed {
        attempt: u32,
        reason: PairingFailure,
    },
}

//! BLE pairing management on the bridge, shared by CLI and GUI
//! (docs/notes/ble-pairing.md).

use anyhow::{Context, Result, bail};
use oxifoc_core::types::pairing::{
    BOND_NAME_LEN, BondEntry, BondName, BondRole, PairingError, PairingRequest, PairingResponse,
};

use crate::{CommandSender, HostCommand};

/// The bridge answered with [`PairingResponse::Error`]. Callers may downcast
/// the `anyhow::Error` to this type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Refused(pub PairingError);

impl std::fmt::Display for Refused {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self.0 {
            PairingError::TableFull => "all bond slots are used; remove a bond first",
            PairingError::WindowBusy => "another pairing window is open",
            PairingError::NoSuchAttempt => "this pairing attempt is no longer active",
            PairingError::NoSuchBond => "no such bond",
            PairingError::NotStopped => "stop the motor before pairing",
            PairingError::Storage => "the bridge could not write its flash",
        })
    }
}

impl std::error::Error for Refused {}

/// No node on the link serves the pairing endpoint (no bridge, or the link
/// is not up yet). Nothing was sent, so the request may be retried.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NoBridge;

impl std::fmt::Display for NoBridge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("no bridge serves the pairing endpoint")
    }
}

impl std::error::Error for NoBridge {}

/// Wait until a bridge answers pairing requests, up to `wait`.
pub fn wait_for_bridge(cmd: &CommandSender, wait: std::time::Duration) -> Result<()> {
    let deadline = std::time::Instant::now() + wait;
    loop {
        match request(cmd, PairingRequest::ListBonds) {
            Ok(_) => return Ok(()),
            Err(e) if e.is::<NoBridge>() && std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(200));
            }
            Err(e) => return Err(e),
        }
    }
}

/// Send one pairing request and return the bridge's answer. Bridge-reported
/// errors are returned as `Err` wrapping [`Refused`].
pub fn request(cmd: &CommandSender, request: PairingRequest) -> Result<PairingResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::Pairing(request, tx))
        .context("send pairing request")?;
    match rx
        .blocking_recv()
        .context("backend dropped the pairing request")??
    {
        PairingResponse::Error(e) => Err(Refused(e).into()),
        response => Ok(response),
    }
}

fn name(value: &str) -> Result<BondName> {
    BondName::try_from(value).map_err(|_| anyhow::anyhow!("name longer than {BOND_NAME_LEN} bytes"))
}

/// Open a window for one new device; returns `(attempt, expires_ms)`.
pub fn open_window(
    cmd: &CommandSender,
    role: BondRole,
    seconds: u16,
    bond_name: &str,
) -> Result<(u32, u32)> {
    match request(
        cmd,
        PairingRequest::OpenWindow {
            role,
            seconds,
            name: name(bond_name)?,
        },
    )? {
        PairingResponse::WindowOpened {
            attempt,
            expires_ms,
        } => Ok((attempt, expires_ms)),
        other => bail!("unexpected answer to OpenWindow: {other:?}"),
    }
}

pub fn list(cmd: &CommandSender) -> Result<Vec<BondEntry>> {
    match request(cmd, PairingRequest::ListBonds)? {
        PairingResponse::Bonds(bonds) => Ok(bonds.into_iter().collect()),
        other => bail!("unexpected answer to ListBonds: {other:?}"),
    }
}

pub fn remove(cmd: &CommandSender, id: u8) -> Result<()> {
    request(cmd, PairingRequest::RemoveBond { id }).map(drop)
}

pub fn rename(cmd: &CommandSender, id: u8, bond_name: &str) -> Result<()> {
    request(
        cmd,
        PairingRequest::RenameBond {
            id,
            name: name(bond_name)?,
        },
    )
    .map(drop)
}

pub fn submit_passkey(cmd: &CommandSender, attempt: u32, passkey: u32) -> Result<()> {
    request(cmd, PairingRequest::SubmitPasskey { attempt, passkey }).map(drop)
}

pub fn confirm(cmd: &CommandSender, attempt: u32, accept: bool) -> Result<()> {
    request(cmd, PairingRequest::ConfirmComparison { attempt, accept }).map(drop)
}

pub fn cancel(cmd: &CommandSender, attempt: u32) -> Result<()> {
    request(cmd, PairingRequest::Cancel { attempt }).map(drop)
}

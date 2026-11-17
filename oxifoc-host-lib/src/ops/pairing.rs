//! BLE pairing management on the bridge, shared by CLI and GUI
//! (docs/notes/ble-pairing.md).

use anyhow::{Context, Result, bail};
use oxifoc_core::types::pairing::{
    BOND_NAME_LEN, BondEntry, BondName, BondRole, PairingRequest, PairingResponse,
};

use crate::{CommandSender, HostCommand};

/// Send one pairing request and return the bridge's answer. Bridge-reported
/// errors are returned as `Err`.
pub fn request(cmd: &CommandSender, request: PairingRequest) -> Result<PairingResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::Pairing(request, tx))
        .context("send pairing request")?;
    match rx
        .blocking_recv()
        .context("backend dropped the pairing request")??
    {
        PairingResponse::Error(e) => bail!("bridge refused: {e:?}"),
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

//! Controller diagnostics reads shared by CLI and GUI.

use anyhow::{Context, Result, bail};
use oxifoc_core::types::diag::{
    BridgeDiag, BridgeDiagRequest, BridgeDiagResponse, CrashRecord, DiagRequest, DiagResponse,
    DiagSummary, IsrWindow,
};

use crate::{CommandSender, HostCommand};

fn request(cmd: &CommandSender, request: DiagRequest) -> Result<DiagResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::Diag(request, tx))
        .context("send diag request")?;
    rx.blocking_recv()
        .context("backend dropped the diag request")?
        .context("diag request failed")
}

fn bridge_request(cmd: &CommandSender, request: BridgeDiagRequest) -> Result<BridgeDiagResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::BridgeDiag(request, tx))
        .context("send bridge diag request")?;
    rx.blocking_recv()
        .context("backend dropped the bridge diag request")?
        .context("bridge diag request failed")
}

/// The bridge's link counters; needs only the link to the bridge.
pub fn bridge(cmd: &CommandSender) -> Result<BridgeDiag> {
    match bridge_request(cmd, BridgeDiagRequest::Links)? {
        BridgeDiagResponse::Links(links) => Ok(links),
        other => bail!("bridge answered {other:?} to a links request"),
    }
}

/// The bridge's last panic record, kept across resets until cleared.
pub fn bridge_crash(cmd: &CommandSender) -> Result<Option<CrashRecord>> {
    match bridge_request(cmd, BridgeDiagRequest::Crash)? {
        BridgeDiagResponse::Crash(record) => Ok(record),
        other => bail!("bridge answered {other:?} to a crash request"),
    }
}

/// Forget the bridge's crash record.
pub fn clear_bridge_crash(cmd: &CommandSender) -> Result<()> {
    match bridge_request(cmd, BridgeDiagRequest::ClearCrash)? {
        BridgeDiagResponse::Crash(None) => Ok(()),
        other => bail!("bridge answered {other:?} to a crash clear"),
    }
}

/// Totals since boot and the reset cause.
pub fn summary(cmd: &CommandSender) -> Result<DiagSummary> {
    match request(cmd, DiagRequest::Summary)? {
        DiagResponse::Summary(summary) => Ok(summary),
        other => bail!("device answered {other:?} to a diag summary"),
    }
}

/// The last panic/HardFault record, kept across resets until cleared.
pub fn crash(cmd: &CommandSender) -> Result<Option<CrashRecord>> {
    match request(cmd, DiagRequest::Crash)? {
        DiagResponse::Crash(record) => Ok(record),
        other => bail!("device answered {other:?} to a crash request"),
    }
}

/// Forget the crash record.
pub fn clear_crash(cmd: &CommandSender) -> Result<()> {
    match request(cmd, DiagRequest::ClearCrash)? {
        DiagResponse::Crash(None) => Ok(()),
        other => bail!("device answered {other:?} to a crash clear"),
    }
}

/// The last completed one-second ISR window, if the device has one.
pub fn isr_window(cmd: &CommandSender) -> Result<Option<IsrWindow>> {
    match request(cmd, DiagRequest::IsrWindow)? {
        DiagResponse::IsrWindow(window) => Ok(Some(window)),
        DiagResponse::Unavailable => Ok(None),
        other => bail!("device answered {other:?} to an ISR window request"),
    }
}

/// Wait until a bridge answers for its diagnostics, up to `wait`. Every
/// bridge serves them, with or without a radio.
pub fn wait_for_bridge(cmd: &CommandSender, wait: std::time::Duration) -> Result<()> {
    let deadline = std::time::Instant::now() + wait;
    loop {
        match bridge_request(cmd, BridgeDiagRequest::Links) {
            Ok(_) => return Ok(()),
            Err(error)
                if error.is::<super::pairing::NoBridge>()
                    && std::time::Instant::now() < deadline =>
            {
                std::thread::sleep(std::time::Duration::from_millis(200));
            }
            Err(error) => return Err(error),
        }
    }
}

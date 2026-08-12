//! Revision-safe fault queries and clears shared by CLI and GUI.

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{
    FaultClear, FaultClearTarget, FaultRequest, FaultResponse, FaultSnapshot, Keyed, ReqId,
};

use crate::{CommandSender, HostCommand, fault_channel};

fn next_fault_id() -> ReqId {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CTR: std::sync::OnceLock<AtomicU64> = std::sync::OnceLock::new();
    let ctr = CTR.get_or_init(|| {
        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        AtomicU64::new(seed | 1)
    });
    ReqId(ctr.fetch_add(1, Ordering::Relaxed))
}

fn request(cmd: &CommandSender, request: FaultRequest) -> Result<FaultResponse> {
    let (tx, rx) = fault_channel();
    cmd.send(HostCommand::Fault(request, tx))
        .context("send fault request")?;
    rx.blocking_recv()
        .context("backend dropped the fault request")?
        .context("fault request failed")
}

/// Read the current complete fault snapshot.
pub fn query(cmd: &CommandSender) -> Result<FaultSnapshot> {
    match request(cmd, FaultRequest::Query)? {
        FaultResponse::Snapshot(snapshot) => Ok(snapshot),
        other => bail!("device answered {other:?} to a fault query"),
    }
}

/// Clear against the exact generation observed immediately before this call.
/// A concurrent add/refinement returns an error and is never silently cleared.
pub fn clear(cmd: &CommandSender, target: FaultClearTarget) -> Result<FaultSnapshot> {
    let observed = query(cmd)?;
    let id = next_fault_id();
    match request(
        cmd,
        FaultRequest::Clear(Keyed::new(
            id,
            FaultClear {
                expected_generation: observed.generation,
                target,
            },
        )),
    )? {
        FaultResponse::Cleared { req_id, snapshot } if req_id == id => Ok(snapshot),
        FaultResponse::Conflict(snapshot) => bail!(
            "fault state changed concurrently (observed generation {}, current {}); nothing was cleared",
            observed.generation,
            snapshot.generation,
        ),
        other => bail!("fault clear rejected: {other:?}"),
    }
}

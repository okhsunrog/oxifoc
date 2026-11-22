//! Control the C5 probe through the USB ergot link.

use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use ergot::{net_stack::NetStackHandle, traits::Endpoint};
use oxifoc_core::{
    icd::SwdProbeEndpoint,
    types::swd_probe::{ProbeRequest, ProbeResponse},
};

use crate::{CommandSender, HostCommand, connection};

#[derive(Debug)]
pub struct NoProbe;

impl std::fmt::Display for NoProbe {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("no bridge serves the wireless SWD endpoint")
    }
}

impl std::error::Error for NoProbe {}

pub fn request(cmd: &CommandSender, request: ProbeRequest) -> Result<ProbeResponse> {
    let (tx, rx) = tokio::sync::oneshot::channel();
    cmd.send(HostCommand::SwdProbe(request, tx))
        .context("send wireless SWD request")?;
    rx.blocking_recv()
        .context("backend dropped the SWD request")?
}

pub(crate) async fn request_on_link<NS>(ns: &NS, request: &ProbeRequest) -> Result<ProbeResponse>
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let addr = connection::resolve_endpoint_addr(
        ns,
        SwdProbeEndpoint::REQ_KEY.to_bytes(),
        Duration::from_millis(500),
    )
    .await
    .ok_or(NoProbe)?;

    // Retry discovery only. Resending Arm after an uncertain outcome could
    // replace a live connection and extend the window.
    tokio::time::timeout(
        Duration::from_secs(3),
        ns.stack()
            .endpoints()
            .request::<SwdProbeEndpoint>(addr, request, None),
    )
    .await
    .context("wireless SWD request timed out")?
    .map_err(|err| anyhow!("wireless SWD request failed: {err:?}"))
}

//! Bridge OTA requests over the host's active Ergot transport.

use std::time::Duration;

use anyhow::{Context, Result, anyhow};
use ergot::{Address, net_stack::NetStackHandle, traits::Endpoint};
use oxifoc_core::{
    icd::BridgeOtaEndpoint,
    types::bridge_ota::{OtaRequest, OtaResponse},
};

use crate::{CommandSender, HostCommand, connection};

/// Keep the resolved address for a transfer so chunks cost one round trip.
pub struct Client<'a> {
    pub cmd: &'a CommandSender,
    pub target: Option<Address>,
}

impl Client<'_> {
    pub fn request(&mut self, request: OtaRequest) -> Result<OtaResponse> {
        let (tx, rx) = tokio::sync::oneshot::channel();
        self.cmd
            .send(HostCommand::BridgeOta(self.target, Box::new(request), tx))
            .context("send bridge OTA request")?;
        let (target, response) = rx.blocking_recv().context("bridge OTA response lost")??;
        self.target = Some(target);
        Ok(response)
    }
}

pub(crate) async fn request_on_link<NS>(
    ns: &NS,
    target: Option<Address>,
    request: &OtaRequest,
) -> Result<(Address, OtaResponse)>
where
    NS: NetStackHandle + Clone + Send + Sync + 'static,
{
    let target = match target {
        Some(addr) => addr,
        None => connection::resolve_endpoint_addr(
            ns,
            BridgeOtaEndpoint::REQ_KEY.to_bytes(),
            Duration::from_millis(500),
        )
        .await
        .context("no bridge serves the OTA endpoint")?,
    };
    let response = tokio::time::timeout(
        Duration::from_secs(3),
        ns.stack()
            .endpoints()
            .request::<BridgeOtaEndpoint>(target, request, None),
    )
    .await
    .context("bridge OTA request timed out")?
    .map_err(|err| anyhow!("bridge OTA request failed: {err:?}"))?;
    Ok((target, response))
}

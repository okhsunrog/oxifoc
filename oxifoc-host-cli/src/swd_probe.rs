//! Arm or disarm the bridge's SWD TCP listener over Ergot.

use std::{
    net::Ipv4Addr,
    thread,
    time::{Duration, Instant},
};

use anyhow::{Result, bail};
use clap::Subcommand;
use oxifoc_core::types::swd_probe::{MAX_ARM_MINUTES, ProbeRequest, ProbeResponse};
use oxifoc_host_lib::{HostRuntime, ops::swd_probe as ops};
use serde_json::json;

use crate::emit;

#[derive(Subcommand)]
pub enum ProbeAction {
    /// Enable Wi-Fi debugging for a limited time. Stop the motor first.
    Arm {
        #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u8).range(1..=i64::from(MAX_ARM_MINUTES)))]
        minutes: u8,
    },
    /// Close the Wi-Fi listener and release the SWD pins
    Disarm,
}

pub fn run(runtime: &HostRuntime, action: ProbeAction, wait: Duration, json: bool) -> Result<()> {
    let request = match action {
        ProbeAction::Arm { minutes } => ProbeRequest::Arm { minutes },
        ProbeAction::Disarm => ProbeRequest::Disarm,
    };
    let deadline = Instant::now() + wait;
    let reply = loop {
        match ops::request(&runtime.cmd_tx, request) {
            Err(err) if err.is::<ops::NoProbe>() && Instant::now() < deadline => {
                thread::sleep(Duration::from_millis(200));
            }
            result => break result?,
        }
    };

    match reply {
        ProbeResponse::Armed { ipv4, minutes } => {
            let selector = format!("0000:0001:{}", Ipv4Addr::from(ipv4));
            emit(
                json,
                json!({"armed": true, "minutes": minutes, "probe": selector}),
                format!("Armed for {minutes} min. Use --probe {selector}."),
            );
        }
        ProbeResponse::Disarmed => emit(json, json!({"armed": false}), "Disarmed.".into()),
        ProbeResponse::InvalidDuration => bail!("arm duration must be 1–{MAX_ARM_MINUTES} minutes"),
        ProbeResponse::WifiUnavailable => {
            bail!("the bridge has no Wi-Fi address. Check its build-time credentials and DHCP")
        }
    }
    Ok(())
}

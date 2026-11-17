//! `pairing` subcommand: BLE pairing windows and bonds on the bridge
//! (docs/notes/ble-pairing.md).

use std::io::{BufRead, Write};
use std::time::{Duration, Instant};

use anyhow::Result;
use clap::{Subcommand, ValueEnum};
use oxifoc_core::types::pairing::{BondRole, PairingEvent};
use oxifoc_host_lib::{HostRuntime, ops::pairing as ops};
use serde_json::json;

use crate::emit;

#[derive(Clone, Copy, ValueEnum)]
pub enum RoleArg {
    Remote,
    Host,
}

impl From<RoleArg> for BondRole {
    fn from(role: RoleArg) -> Self {
        match role {
            RoleArg::Remote => Self::Remote,
            RoleArg::Host => Self::Host,
        }
    }
}

#[derive(Subcommand)]
pub enum PairingAction {
    /// Open a pairing window for one new device and guide the pairing
    Open {
        #[arg(long, value_enum)]
        role: RoleArg,
        /// Name stored with the new bond
        #[arg(long)]
        name: String,
        #[arg(long, default_value_t = 60)]
        seconds: u16,
    },
    /// List stored bonds
    List,
    /// Remove a bond and drop its link
    Remove { id: u8 },
    /// Rename a bond
    Rename { id: u8, name: String },
    /// Cancel a pairing window
    Cancel { attempt: u32 },
    /// Print pairing events
    Events {
        #[arg(
            long,
            default_value_t = 0,
            help = "Stop after N seconds (0 = until Ctrl-C)"
        )]
        seconds: u64,
    },
}

pub fn run(runtime: &HostRuntime, action: PairingAction, json: bool) -> Result<()> {
    let cmd = &runtime.cmd_tx;
    match action {
        PairingAction::Open {
            role,
            name,
            seconds,
        } => {
            // Drop stale events before the window opens.
            while runtime.pairing_rx.try_recv().is_ok() {}
            let (attempt, expires_ms) = ops::open_window(cmd, role.into(), seconds, &name)?;
            emit(
                json,
                json!({"window": attempt, "expires_ms": expires_ms}),
                format!(
                    "Pairing window {attempt} open for {} s. Pair the new device now.",
                    expires_ms / 1000
                ),
            );
            guide(
                runtime,
                attempt,
                Duration::from_millis(u64::from(expires_ms) + 5000),
                json,
            )
        }
        PairingAction::List => {
            let bonds = ops::list(cmd)?;
            let human = if bonds.is_empty() {
                "no bonds".to_string()
            } else {
                bonds
                    .iter()
                    .map(|b| {
                        format!(
                            "{}: {} ({:?}) {} added by {}",
                            b.id,
                            b.name,
                            b.role,
                            format_address(&b.address),
                            b.added_by
                                .map_or("USB".to_string(), |id| format!("bond {id}"))
                        )
                    })
                    .collect::<Vec<_>>()
                    .join("\n")
            };
            emit(json, serde_json::to_value(&bonds)?, human);
            Ok(())
        }
        PairingAction::Remove { id } => {
            ops::remove(cmd, id)?;
            emit(json, json!({"removed": id}), format!("bond {id} removed"));
            Ok(())
        }
        PairingAction::Rename { id, name } => {
            ops::rename(cmd, id, &name)?;
            emit(json, json!({"renamed": id}), format!("bond {id} renamed"));
            Ok(())
        }
        PairingAction::Cancel { attempt } => {
            ops::cancel(cmd, attempt)?;
            emit(
                json,
                json!({"cancelled": attempt}),
                format!("window {attempt} cancelled"),
            );
            Ok(())
        }
        PairingAction::Events { seconds } => {
            let end = (seconds > 0).then(|| Instant::now() + Duration::from_secs(seconds));
            loop {
                let timeout = end.map_or(Duration::from_secs(3600), |end| {
                    end.saturating_duration_since(Instant::now())
                });
                if timeout.is_zero() {
                    return Ok(());
                }
                if let Ok(event) = runtime.pairing_rx.recv_timeout(timeout) {
                    print_event(&event, json);
                }
            }
        }
    }
}

/// Follow one attempt: answer comparisons and passkey prompts on this terminal.
fn guide(runtime: &HostRuntime, attempt: u32, limit: Duration, json: bool) -> Result<()> {
    let end = Instant::now() + limit;
    loop {
        let remaining = end.saturating_duration_since(Instant::now());
        anyhow::ensure!(!remaining.is_zero(), "no result from the bridge");
        let Ok(event) = runtime.pairing_rx.recv_timeout(remaining) else {
            continue;
        };
        print_event(&event, json);
        match event {
            PairingEvent::CompareNumber { attempt: a, number } if a == attempt => {
                let accept = ask(&format!(
                    "Does the new device show {number:06}? Confirm there too. [y/N] "
                ))?
                .eq_ignore_ascii_case("y");
                ops::confirm(&runtime.cmd_tx, attempt, accept)?;
            }
            PairingEvent::EnterPasskey { attempt: a } if a == attempt => {
                let code = ask("Passkey shown on the new device: ")?;
                let passkey = code
                    .parse::<u32>()
                    .map_err(|_| anyhow::anyhow!("not a number"))?;
                ops::submit_passkey(&runtime.cmd_tx, attempt, passkey)?;
            }
            PairingEvent::WindowClosed { attempt: a, reason } if a == attempt => {
                use oxifoc_core::types::pairing::WindowCloseReason;
                return match reason {
                    WindowCloseReason::Bonded => Ok(()),
                    other => anyhow::bail!("pairing window closed: {other:?}"),
                };
            }
            _ => {}
        }
    }
}

fn ask(prompt: &str) -> Result<String> {
    eprint!("{prompt}");
    std::io::stderr().flush()?;
    let mut line = String::new();
    std::io::stdin().lock().read_line(&mut line)?;
    Ok(line.trim().to_string())
}

fn print_event(event: &PairingEvent, json: bool) {
    let human = match event {
        PairingEvent::WindowOpened {
            attempt,
            role,
            expires_ms,
        } => format!("window {attempt} opened for {role:?}, {expires_ms} ms"),
        PairingEvent::WindowClosed { attempt, reason } => {
            format!("window {attempt} closed: {reason:?}")
        }
        PairingEvent::CandidateConnected { attempt, address } => {
            format!("window {attempt}: candidate {}", format_address(address))
        }
        PairingEvent::ShowPasskey { attempt, passkey } => {
            format!("window {attempt}: type {passkey:06} on the new device")
        }
        PairingEvent::EnterPasskey { attempt } => {
            format!("window {attempt}: the new device shows a passkey")
        }
        PairingEvent::CompareNumber { attempt, number } => {
            format!("window {attempt}: compare {number:06}")
        }
        PairingEvent::Bonded(entry) => format!(
            "bonded {}: {} ({:?}) {}",
            entry.id,
            entry.name,
            entry.role,
            format_address(&entry.address)
        ),
        PairingEvent::BondRemoved { id } => format!("bond {id} removed"),
        PairingEvent::Failed { attempt, reason } => {
            format!("window {attempt}: pairing failed: {reason:?}")
        }
    };
    emit(json, serde_json::to_value(event).unwrap_or_default(), human);
}

/// Addresses travel little endian; print them as usual, most significant first.
fn format_address(address: &[u8; 6]) -> String {
    address
        .iter()
        .rev()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(":")
}

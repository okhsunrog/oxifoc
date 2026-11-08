//! Acknowledged, bounded telemetry configuration for synchronous frontends.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use oxifoc_core::types::{TelemetryConfig, TelemetryConfigAck};
use tokio::sync::oneshot::error::TryRecvError;

use crate::{CommandSender, HostCommand, SETPOINT_POLICY, telemetry_channel};

/// Set the stream rate and wait for this request's acknowledgement.
///
/// The deadline also bounds time spent behind other backend commands. A zero
/// request only succeeds when the device confirms that streaming is disabled.
pub fn configure(cmd: &CommandSender, fast_hz: u16) -> Result<TelemetryConfigAck> {
    configure_with_timeout(
        cmd,
        fast_hz,
        Duration::from_millis(SETPOINT_POLICY.deadline_ms + 250),
    )
}

fn configure_with_timeout(
    cmd: &CommandSender,
    fast_hz: u16,
    timeout: Duration,
) -> Result<TelemetryConfigAck> {
    let (tx, mut rx) = telemetry_channel();
    cmd.send(HostCommand::SetTelemetryConfig(
        TelemetryConfig { fast_hz },
        Some(tx),
    ))
    .context("send telemetry config")?;
    let deadline = Instant::now() + timeout;
    loop {
        match rx.try_recv() {
            Ok(result) => {
                let ack = result.context("device did not acknowledge telemetry config")?;
                if fast_hz == 0 && ack.actual_fast_hz != 0 {
                    bail!("device acknowledged an active stream while disabling telemetry");
                }
                return Ok(ack);
            }
            Err(TryRecvError::Closed) => bail!("backend dropped telemetry config request"),
            Err(TryRecvError::Empty) => {}
        }
        let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
            bail!("telemetry config acknowledgement timed out");
        };
        std::thread::sleep(remaining.min(Duration::from_millis(5)));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stop_waits_for_its_acknowledgement() {
        let (cmd, mut requests) = tokio::sync::mpsc::unbounded_channel();
        let peer = std::thread::spawn(move || {
            let HostCommand::SetTelemetryConfig(config, Some(reply)) =
                requests.blocking_recv().unwrap()
            else {
                panic!("expected acknowledged telemetry config")
            };
            assert_eq!(config.fast_hz, 0);
            std::thread::sleep(Duration::from_millis(30));
            reply
                .send(Ok(TelemetryConfigAck {
                    actual_fast_hz: 0,
                    ..Default::default()
                }))
                .unwrap();
        });
        let started = Instant::now();
        assert_eq!(configure(&cmd, 0).unwrap().actual_fast_hz, 0);
        assert!(started.elapsed() >= Duration::from_millis(30));
        peer.join().unwrap();
    }

    #[test]
    fn active_stream_ack_is_not_a_successful_stop() {
        let (cmd, mut requests) = tokio::sync::mpsc::unbounded_channel();
        let peer = std::thread::spawn(move || {
            let HostCommand::SetTelemetryConfig(_, Some(reply)) = requests.blocking_recv().unwrap()
            else {
                panic!("expected acknowledged telemetry config")
            };
            reply
                .send(Ok(TelemetryConfigAck {
                    actual_fast_hz: 5000,
                    ..Default::default()
                }))
                .unwrap();
        });
        assert!(
            configure(&cmd, 0)
                .unwrap_err()
                .to_string()
                .contains("active stream")
        );
        peer.join().unwrap();
    }

    #[test]
    fn queued_request_has_a_bounded_wait() {
        let (cmd, mut requests) = tokio::sync::mpsc::unbounded_channel();
        let started = Instant::now();
        let error = configure_with_timeout(&cmd, 0, Duration::from_millis(20)).unwrap_err();
        assert!(error.to_string().contains("timed out"));
        assert!(started.elapsed() < Duration::from_secs(1));
        assert!(requests.try_recv().is_ok());
    }

    #[test]
    fn closed_backend_is_reported() {
        let (cmd, requests) = tokio::sync::mpsc::unbounded_channel();
        drop(requests);
        assert!(configure(&cmd, 0).is_err());
    }
}

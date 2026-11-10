use crate::transport::{TransportConfig, TransportType};
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::{env, fs, path::PathBuf};

/// Fast samples may bypass the command transport on explicitly selected bench builds.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum FastSource {
    #[default]
    Ergot,
    UsbRaw,
    RttRaw,
    ErgotRaw,
}

impl FastSource {
    pub fn is_raw(self) -> bool {
        self != Self::Ergot
    }
}

#[derive(Debug, Default, Deserialize, Clone)]
pub struct HostConfig {
    /// Transport type: "serial" (default), "rtt", "tcp", "udp", or "usb"
    pub transport: Option<TransportType>,

    // RTT transport options (desktop only)
    #[cfg(feature = "desktop")]
    pub probe: Option<String>, // e.g. "0483:374b:<serial>" or "0483:374b"
    #[cfg(feature = "desktop")]
    pub chip: Option<String>, // e.g. "STM32G431CBTx"

    // Serial transport options (desktop only)
    #[cfg(feature = "desktop")]
    pub serial_path: Option<String>, // e.g. "/dev/ttyACM0"
    #[cfg(feature = "desktop")]
    pub serial_baud: Option<u32>, // e.g. 921600

    // TCP transport options
    pub tcp_host: Option<String>, // e.g. "127.0.0.1"
    pub tcp_port: Option<u16>,    // e.g. 2025

    // UDP transport options
    pub udp_host: Option<String>, // e.g. "127.0.0.1"
    pub udp_port: Option<u16>,    // e.g. 2025

    // BLE transport options
    /// Bluest device handle (set programmatically from scan results, not from config file)
    #[serde(skip)]
    pub ble_device: Option<bluest::Device>,

    // Common options
    pub elf: Option<String>,        // path to device ELF with .defmt
    pub stream_defmt: Option<bool>, // default: true
    pub stream_ergot: Option<bool>, // default: true

    /// Reconnection policy for COBS-stream transports (TCP, Serial, RTT).
    /// None = use default (infinite retries)
    pub reconnect: Option<ReconnectPolicy>,

    /// Desired fast telemetry rate in Hz (default 1000).
    /// Set to 0 to disable fast telemetry streaming.
    pub fast_hz: Option<u16>,
    pub fast_source: Option<FastSource>,
    pub fast_encoding: Option<oxifoc_core::raw_telemetry::RawEncoding>,
}

/// Controls how the host handles transport disconnection/failure.
#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum ReconnectPolicy {
    /// Disconnect immediately on any failure, including loss of an established
    /// connection, with no retry.
    None,
    /// Allow at most N consecutive connection generations before giving up.
    /// A completed protocol handshake resets the failure count.
    Limited(u32),
    /// Retry forever (default behavior)
    Infinite,
}

impl HostConfig {
    pub fn load_default() -> Result<Option<Self>> {
        if let Some(p) = env::var_os("OXIFOC_HOST_CONFIG") {
            return Self::from_path(PathBuf::from(p)).map(Some);
        }
        let cwd = env::current_dir().context("resolve current directory for host config")?;
        let p = cwd.join("oxifoc-host.toml");
        if p.exists() {
            return Self::from_path(p).map(Some);
        }
        Ok(None)
    }

    fn from_path(path: PathBuf) -> Result<Self> {
        let contents = fs::read_to_string(&path)
            .with_context(|| format!("read host config {}", path.display()))?;
        toml::from_str::<Self>(&contents)
            .with_context(|| format!("parse host config {} as TOML", path.display()))
    }

    pub fn reconnect_policy(&self) -> ReconnectPolicy {
        self.reconnect.unwrap_or(ReconnectPolicy::Infinite)
    }

    pub fn stream_defmt(&self) -> bool {
        self.stream_defmt.unwrap_or(true)
    }
    pub fn stream_ergot(&self) -> bool {
        self.stream_ergot.unwrap_or(true)
    }
    pub fn fast_hz(&self) -> u16 {
        self.fast_hz.unwrap_or(0)
    }

    #[cfg(feature = "desktop")]
    pub fn serial_path(&self) -> String {
        self.serial_path
            .clone()
            .unwrap_or_else(|| "/dev/ttyACM0".to_string())
    }

    #[cfg(feature = "desktop")]
    pub fn serial_baud(&self) -> u32 {
        self.serial_baud.unwrap_or(921_600)
    }

    pub fn transport_type(&self) -> TransportType {
        self.transport.clone().unwrap_or_else(|| {
            if self.fast_source().is_raw() {
                TransportType::Usb
            } else {
                TransportType::default()
            }
        })
    }

    pub fn fast_source(&self) -> FastSource {
        self.fast_source.unwrap_or_else(|| {
            if self.transport == Some(TransportType::UsbSerialJtag) {
                FastSource::ErgotRaw
            } else {
                FastSource::default()
            }
        })
    }

    pub fn fast_encoding(&self) -> oxifoc_core::raw_telemetry::RawEncoding {
        self.fast_encoding
            .unwrap_or(oxifoc_core::raw_telemetry::RawEncoding::Packed28)
    }

    pub fn transport_config(&self) -> Result<TransportConfig> {
        match self.transport_type() {
            #[cfg(feature = "desktop")]
            TransportType::Serial => Ok(TransportConfig::Serial {
                path: self.serial_path(),
                baud: self.serial_baud(),
            }),
            #[cfg(feature = "desktop")]
            TransportType::UsbSerialJtag => Ok(TransportConfig::Serial {
                path: match &self.serial_path {
                    Some(path) => path.clone(),
                    None => crate::discovery::usb_serial_jtag_port(
                        &crate::discovery::list_serial_ports(),
                    )?,
                },
                // USB Serial/JTAG ignores UART line coding; this is not its bit rate.
                baud: self.serial_baud(),
            }),
            #[cfg(feature = "desktop")]
            TransportType::Rtt => {
                let chip = self.chip.clone().ok_or_else(|| {
                    anyhow::anyhow!("RTT transport requires 'chip' to be specified in config")
                })?;
                Ok(TransportConfig::Rtt {
                    probe: self.probe.clone(),
                    chip,
                })
            }
            TransportType::Tcp => Ok(TransportConfig::Tcp {
                host: self
                    .tcp_host
                    .clone()
                    .unwrap_or_else(|| "127.0.0.1".to_string()),
                port: self.tcp_port.unwrap_or(2025),
            }),
            TransportType::Udp => Ok(TransportConfig::Udp {
                host: self
                    .udp_host
                    .clone()
                    .unwrap_or_else(|| "127.0.0.1".to_string()),
                port: self.udp_port.unwrap_or(2025),
            }),
            TransportType::Usb => Ok(TransportConfig::Usb),
            TransportType::Ble => {
                let device = self.ble_device.clone().ok_or_else(|| {
                    anyhow::anyhow!("BLE transport requires a device from scan results")
                })?;
                Ok(TransportConfig::Ble { device })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn raw_sources_select_usb_commands_without_changing_legacy_defaults() {
        let default = HostConfig::default();
        assert_eq!(default.fast_source(), FastSource::Ergot);
        let bridge = HostConfig {
            transport: Some(TransportType::UsbSerialJtag),
            ..Default::default()
        };
        assert_eq!(bridge.fast_source(), FastSource::ErgotRaw);
        assert_eq!(
            bridge.fast_encoding(),
            oxifoc_core::raw_telemetry::RawEncoding::Packed28
        );
        assert_eq!(default.transport_type(), TransportType::default());
        for source in ["usb-raw", "rtt-raw", "ergot-raw"] {
            let cfg: HostConfig =
                toml::from_str(&format!("fast_source='{source}'\nfast_encoding='Packed26'"))
                    .unwrap();
            assert!(cfg.fast_source().is_raw());
            assert_eq!(cfg.transport_type(), TransportType::Usb);
            assert_eq!(
                cfg.fast_encoding(),
                oxifoc_core::raw_telemetry::RawEncoding::Packed26
            );
        }
    }

    fn temp_config_path(name: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("system clock before unix epoch")
            .as_nanos();
        env::temp_dir().join(format!("oxifoc-host-{name}-{nonce}.toml"))
    }

    #[test]
    fn explicit_invalid_config_is_an_error() {
        let path = temp_config_path("invalid");
        fs::write(&path, "transport = [not valid toml").expect("write temporary invalid config");

        let err = HostConfig::from_path(path.clone()).expect_err("invalid config must fail");
        assert!(err.to_string().contains("parse host config"));

        fs::remove_file(path).expect("remove temporary config");
    }

    #[test]
    fn missing_explicit_config_is_an_error() {
        let path = temp_config_path("missing");
        let err = HostConfig::from_path(path).expect_err("missing config must fail");
        assert!(err.to_string().contains("read host config"));
    }
}

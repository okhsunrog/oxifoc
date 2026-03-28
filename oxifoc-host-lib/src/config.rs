use crate::transport::{TransportConfig, TransportType};
use serde::Deserialize;
use std::{env, fs, path::PathBuf};

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
}

/// Controls how the host handles transport disconnection/failure.
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ReconnectPolicy {
    /// Disconnect immediately on failure, no retries
    None,
    /// Retry up to N times, then give up
    Limited(u32),
    /// Retry forever (default behavior)
    Infinite,
}

impl HostConfig {
    pub fn load_default() -> Option<Self> {
        if let Ok(p) = env::var("OXIFOC_HOST_CONFIG") {
            return Self::from_path(PathBuf::from(p));
        }
        let cwd = env::current_dir().ok()?;
        let p = cwd.join("oxifoc-host.toml");
        if p.exists() {
            return Self::from_path(p);
        }
        None
    }

    fn from_path(path: PathBuf) -> Option<Self> {
        match fs::read_to_string(&path) {
            Ok(s) => match toml::from_str::<HostConfig>(&s) {
                Ok(cfg) => Some(cfg),
                Err(e) => {
                    eprintln!("Failed to parse config (TOML) {}: {}", path.display(), e);
                    None
                }
            },
            Err(e) => {
                eprintln!("Failed to read config {}: {}", path.display(), e);
                None
            }
        }
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
        self.transport.clone().unwrap_or_default()
    }

    pub fn transport_config(&self) -> anyhow::Result<TransportConfig> {
        match self.transport_type() {
            #[cfg(feature = "desktop")]
            TransportType::Serial => Ok(TransportConfig::Serial {
                path: self.serial_path(),
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

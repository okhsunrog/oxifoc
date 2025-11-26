use crate::transport::{TransportConfig, TransportType};
use serde::Deserialize;
use std::{env, fs, path::PathBuf};

#[derive(Debug, Default, Deserialize, Clone)]
pub struct HostConfig {
    /// Transport type: "serial" (default) or "rtt"
    pub transport: Option<TransportType>,

    // RTT transport options
    pub probe: Option<String>, // e.g. "0483:374b:<serial>" or "0483:374b"
    pub chip: Option<String>,  // e.g. "STM32G431CBTx"

    // Serial transport options
    pub serial_path: Option<String>, // e.g. "/dev/ttyACM0"
    pub serial_baud: Option<u32>,    // e.g. 921600

    // Common options
    pub elf: Option<String>,        // path to device ELF with .defmt
    pub stream_defmt: Option<bool>, // default: true
    pub stream_ergot: Option<bool>, // default: true
}

impl HostConfig {
    pub fn load_default() -> Option<Self> {
        // Priority: OXIFOC_HOST_CONFIG env var, then ./oxifoc-host.toml if exists
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

    pub fn stream_defmt(&self) -> bool {
        self.stream_defmt.unwrap_or(true)
    }
    pub fn stream_ergot(&self) -> bool {
        self.stream_ergot.unwrap_or(true)
    }

    pub fn serial_path(&self) -> String {
        self.serial_path
            .clone()
            .unwrap_or_else(|| "/dev/ttyACM0".to_string())
    }

    pub fn serial_baud(&self) -> u32 {
        self.serial_baud.unwrap_or(921_600)
    }

    /// Get the transport type (defaults to Serial).
    pub fn transport_type(&self) -> TransportType {
        self.transport.clone().unwrap_or_default()
    }

    /// Get the transport configuration based on settings.
    pub fn transport_config(&self) -> anyhow::Result<TransportConfig> {
        match self.transport_type() {
            TransportType::Serial => Ok(TransportConfig::Serial {
                path: self.serial_path(),
                baud: self.serial_baud(),
            }),
            TransportType::Rtt => {
                let chip = self.chip.clone().ok_or_else(|| {
                    anyhow::anyhow!("RTT transport requires 'chip' to be specified in config")
                })?;
                Ok(TransportConfig::Rtt {
                    probe: self.probe.clone(),
                    chip,
                })
            }
        }
    }
}

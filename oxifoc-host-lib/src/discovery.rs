//! Device discovery for oxifoc transports.
//!
//! Provides enumeration of available devices for each transport type:
//! - Serial ports (via tokio_serial, which re-exports mio_serial/serialport)
//! - Debug probes for RTT (via probe-rs)

use probe_rs::probe::list::Lister;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Information about a discovered serial port.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SerialPortInfo {
    /// System path (e.g., "/dev/ttyACM0" or "COM3")
    pub path: String,
    /// USB Vendor ID (if USB device)
    pub vid: Option<u16>,
    /// USB Product ID (if USB device)
    pub pid: Option<u16>,
    /// USB serial number
    pub serial_number: Option<String>,
    /// Manufacturer name
    pub manufacturer: Option<String>,
    /// Product name
    pub product: Option<String>,
}

impl fmt::Display for SerialPortInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(ref product) = self.product {
            write!(f, "{} ({})", self.path, product)
        } else if let (Some(vid), Some(pid)) = (self.vid, self.pid) {
            write!(f, "{} [{:04x}:{:04x}]", self.path, vid, pid)
        } else {
            write!(f, "{}", self.path)
        }
    }
}

/// Information about a discovered debug probe (for RTT transport).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct ProbeInfo {
    /// Identifier string ("VID:PID" or "VID:PID:SERIAL")
    pub identifier: String,
    /// USB Vendor ID
    pub vid: u16,
    /// USB Product ID
    pub pid: u16,
    /// USB serial number (if available)
    pub serial_number: Option<String>,
    /// Probe type (e.g., "STLink", "JLink", "CMSIS-DAP")
    pub probe_type: String,
}

impl fmt::Display for ProbeInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(ref serial) = self.serial_number {
            write!(
                f,
                "{} [{:04x}:{:04x}:{}]",
                self.probe_type, self.vid, self.pid, serial
            )
        } else {
            write!(f, "{} [{:04x}:{:04x}]", self.probe_type, self.vid, self.pid)
        }
    }
}

/// List all available serial ports.
///
/// Returns information about each discovered port including USB metadata
/// when available (VID, PID, serial number, manufacturer, product name).
pub fn list_serial_ports() -> Vec<SerialPortInfo> {
    match tokio_serial::available_ports() {
        Ok(ports) => ports
            .into_iter()
            .map(|port| {
                let (vid, pid, serial_number, manufacturer, product) = match port.port_type {
                    tokio_serial::SerialPortType::UsbPort(usb_info) => (
                        Some(usb_info.vid),
                        Some(usb_info.pid),
                        usb_info.serial_number,
                        usb_info.manufacturer,
                        usb_info.product,
                    ),
                    _ => (None, None, None, None, None),
                };
                SerialPortInfo {
                    path: port.port_name,
                    vid,
                    pid,
                    serial_number,
                    manufacturer,
                    product,
                }
            })
            .collect(),
        Err(e) => {
            tracing::warn!("Failed to enumerate serial ports: {}", e);
            Vec::new()
        }
    }
}

/// List all available debug probes (for RTT transport).
///
/// Uses probe-rs to enumerate connected debug probes such as
/// ST-Link, J-Link, CMSIS-DAP, etc.
pub fn list_probes() -> Vec<ProbeInfo> {
    let lister = Lister::new();
    lister
        .list_all()
        .into_iter()
        .map(|probe| {
            let serial_number = probe.serial_number.clone();
            let identifier = if let Some(ref serial) = serial_number {
                format!(
                    "{:04x}:{:04x}:{}",
                    probe.vendor_id, probe.product_id, serial
                )
            } else {
                format!("{:04x}:{:04x}", probe.vendor_id, probe.product_id)
            };
            let probe_type = format!("{:?}", probe.probe_type());
            ProbeInfo {
                identifier,
                vid: probe.vendor_id,
                pid: probe.product_id,
                serial_number,
                probe_type,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_list_serial_ports() {
        // Just verify it doesn't panic
        let ports = list_serial_ports();
        println!("Found {} serial ports", ports.len());
        for port in &ports {
            println!("  {}", port);
        }
    }

    #[test]
    fn test_list_probes() {
        // Just verify it doesn't panic
        let probes = list_probes();
        println!("Found {} probes", probes.len());
        for probe in &probes {
            println!("  {}", probe);
        }
    }
}

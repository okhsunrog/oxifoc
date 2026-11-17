//! Device discovery for oxifoc transports.
//!
//! Provides enumeration of available devices for each transport type:
//! - Serial ports (via tokio_serial, which re-exports mio_serial/serialport)
//! - Debug probes for RTT (via probe-rs)
//! - BLE devices (via blec)

#[cfg(feature = "desktop")]
use probe_rs::probe::list::Lister;
#[cfg(feature = "desktop")]
use serde::{Deserialize, Serialize};
use std::fmt;
use std::time::Duration;

#[cfg(feature = "desktop")]
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

#[cfg(feature = "desktop")]
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

#[cfg(feature = "desktop")]
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

#[cfg(feature = "desktop")]
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

#[cfg(feature = "desktop")]
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

/// Select only Espressif's built-in USB Serial/JTAG, never an arbitrary ttyACM.
#[cfg(feature = "desktop")]
pub fn usb_serial_jtag_port(ports: &[SerialPortInfo]) -> anyhow::Result<String> {
    let mut matching = ports
        .iter()
        .filter(|p| p.vid == Some(0x303a) && p.pid == Some(0x1001));
    let first = matching.next().ok_or_else(|| {
        anyhow::anyhow!("No Espressif USB Serial/JTAG port found; specify --serial-path")
    })?;
    anyhow::ensure!(
        matching.next().is_none(),
        "Multiple USB Serial/JTAG ports found; specify --serial-path"
    );
    Ok(first.path.clone())
}

#[cfg(feature = "desktop")]
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

/// Information about a discovered BLE device.
#[derive(Debug, Clone)]
pub struct BleDeviceInfo {
    /// Device ID string (MAC address on Linux/Android, opaque on macOS)
    pub id: String,
    /// Local name advertised by the device (if any)
    pub name: Option<String>,
    /// RSSI in dBm (if available)
    pub rssi: Option<i16>,
}

impl fmt::Display for BleDeviceInfo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(ref name) = self.name {
            write!(f, "{name} ({})", self.id)?;
        } else {
            write!(f, "{}", self.id)?;
        }
        if let Some(rssi) = self.rssi {
            write!(f, " [{rssi} dBm]")?;
        }
        Ok(())
    }
}

/// Scan for BLE devices for the given duration.
///
/// Returns addresses independent of the scanner runtime and GUI framework.
pub async fn scan_ble_devices(scan_duration: Duration) -> Vec<BleDeviceInfo> {
    match scan_ble_devices_inner(scan_duration).await {
        Ok(devices) => devices,
        Err(e) => {
            tracing::warn!("BLE scan failed: {}", e);
            Vec::new()
        }
    }
}

/// Blocking convenience over [`scan_ble_devices`] for synchronous callers
/// (the CLI): spins a private current-thread runtime for the scan.
///
/// Must NOT be called from within a tokio runtime (use the async variant
/// there).
pub fn scan_ble_devices_blocking(scan_duration: Duration) -> Vec<BleDeviceInfo> {
    match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt.block_on(scan_ble_devices(scan_duration)),
        Err(e) => {
            tracing::warn!("BLE scan runtime unavailable: {e}");
            Vec::new()
        }
    }
}

/// Initialize the process-wide adapter independently of a GUI or scan runtime.
pub async fn initialize_ble() -> anyhow::Result<()> {
    crate::transport::ble::handler().await.map(|_| ())
}

/// Fallible discovery for interfaces that need to show permission/adapter errors.
pub async fn try_scan_ble_devices(scan_duration: Duration) -> anyhow::Result<Vec<BleDeviceInfo>> {
    scan_ble_devices_inner(scan_duration).await
}

async fn scan_ble_devices_inner(scan_duration: Duration) -> anyhow::Result<Vec<BleDeviceInfo>> {
    // Serialize scan ownership: cancelling one picker must not stop another scan.
    static SCAN: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());
    let _guard = SCAN.lock().await;
    let handler = crate::transport::ble::handler().await?;
    let (tx, mut rx) = tokio::sync::mpsc::channel(8);
    handler
        .discover(
            Some(tx),
            scan_duration.as_millis().max(200) as u64,
            blec::models::ScanFilter::Service(crate::transport::ble::SERVICE_UUID),
            false,
        )
        .await?;
    let mut devices = std::collections::BTreeMap::new();
    while let Some(batch) = rx.recv().await {
        for device in batch {
            devices.insert(
                device.address.clone(),
                BleDeviceInfo {
                    id: device.address,
                    name: (!device.name.is_empty()).then_some(device.name),
                    rssi: device.rssi,
                },
            );
        }
    }
    for device in handler.connected_devices().await {
        if device
            .services
            .contains(&crate::transport::ble::SERVICE_UUID)
        {
            devices
                .entry(device.address.clone())
                .or_insert(BleDeviceInfo {
                    id: device.address,
                    name: Some(device.name),
                    rssi: device.rssi,
                });
        }
    }
    let mut devices: Vec<_> = devices.into_values().collect();
    devices.sort_by_key(|d| std::cmp::Reverse(d.rssi.unwrap_or(i16::MIN)));
    Ok(devices)
}

#[cfg(all(test, feature = "desktop"))]
mod tests {
    use super::*;

    #[test]
    fn usb_serial_jtag_selection_rejects_other_adapters_and_ambiguity() {
        let port = |path: &str, vid, pid| SerialPortInfo {
            path: path.into(),
            vid: Some(vid),
            pid: Some(pid),
            serial_number: None,
            manufacturer: None,
            product: None,
        };
        let stlink = port("stlink", 0x0483, 0x3754);
        let c5 = port("c5", 0x303a, 0x1001);
        assert!(usb_serial_jtag_port(&[]).is_err());
        assert!(usb_serial_jtag_port(std::slice::from_ref(&stlink)).is_err());
        assert_eq!(usb_serial_jtag_port(&[stlink, c5.clone()]).unwrap(), "c5");
        assert!(usb_serial_jtag_port(&[c5, port("second", 0x303a, 0x1001)]).is_err());
    }

    #[test]
    fn test_list_serial_ports() {
        // Just verify it doesn't panic
        let ports = list_serial_ports();
        println!("Found {} serial ports", ports.len());
        for port in &ports {
            println!("  {port}");
        }
    }

    #[test]
    fn test_list_probes() {
        // Just verify it doesn't panic
        let probes = list_probes();
        println!("Found {} probes", probes.len());
        for probe in &probes {
            println!("  {probe}");
        }
    }
}

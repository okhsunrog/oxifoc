use crate::*;
use dioxus::prelude::*;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Transport {
    #[default]
    Ble,
    Tcp,
    Udp,
    Usb,
    Serial,
    Rtt,
}
impl Transport {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ble => "Bluetooth LE",
            Self::Tcp => "TCP",
            Self::Udp => "UDP",
            Self::Usb => "USB",
            Self::Serial => "Serial",
            Self::Rtt => "Debug probe",
        }
    }
    pub fn cable(self) -> bool {
        matches!(self, Self::Usb | Self::Serial | Self::Rtt)
    }
    pub fn network(self) -> bool {
        matches!(self, Self::Tcp | Self::Udp)
    }
}
#[derive(Clone, Copy, PartialEq)]
pub struct Capabilities {
    pub usb: bool,
}
impl Capabilities {
    pub const MOBILE: Self = Self { usb: false };
    pub const DESKTOP: Self = Self { usb: true };
    pub fn allows(self, transport: Transport) -> bool {
        self.usb || !transport.cable()
    }
}

#[derive(Clone, PartialEq)]
pub struct Device {
    pub id: String,
    pub name: String,
    pub detail: String,
}
#[derive(Clone, PartialEq)]
pub struct ConnectionForm {
    pub transport: Transport,
    pub host: String,
    pub port: String,
    pub device: String,
    pub baud: String,
    pub chip: String,
}
impl Default for ConnectionForm {
    fn default() -> Self {
        Self {
            transport: Transport::Ble,
            host: String::new(),
            port: "2025".into(),
            device: String::new(),
            baud: "921600".into(),
            chip: "STM32G474RETx".into(),
        }
    }
}
impl ConnectionForm {
    pub fn validate(&self, capabilities: Capabilities) -> Result<(), String> {
        if !capabilities.allows(self.transport) {
            return Err("This connection is not available on this platform.".into());
        }
        if self.transport.network() {
            if self.host.trim().is_empty()
                || self.host.chars().any(char::is_whitespace)
                || self.host.contains("://")
                || self.host.contains('/')
            {
                return Err("Enter an IP address or hostname, without a URL prefix.".into());
            }
            if self.port.parse::<u16>().ok().filter(|p| *p > 0).is_none() {
                return Err("Port must be between 1 and 65535.".into());
            }
        }
        if matches!(
            self.transport,
            Transport::Ble | Transport::Serial | Transport::Rtt
        ) && self.device.is_empty()
        {
            return Err("Select a device to continue.".into());
        }
        if self.transport == Transport::Serial
            && self.baud.parse::<u32>().ok().filter(|b| *b > 0).is_none()
        {
            return Err("Enter a valid baud rate.".into());
        }
        if self.transport == Transport::Rtt && self.chip.trim().is_empty() {
            return Err("Enter the target chip name.".into());
        }
        Ok(())
    }
    pub fn endpoint(&self) -> String {
        if self.transport.network() {
            format!("{}:{}", self.host.trim(), self.port)
        } else if self.transport == Transport::Usb {
            "USB controller".into()
        } else {
            self.device.clone()
        }
    }
}

#[derive(Clone, PartialEq, Default)]
pub enum ConnectionStage {
    #[default]
    Idle,
    Connecting,
    Error(String),
}

#[component]
pub fn DeviceRow(
    device: Device,
    selected: bool,
    #[props(default)] disabled: bool,
    onclick: EventHandler<String>,
) -> Element {
    let id = device.id.clone();
    let key_id = id.clone();
    rsx! { button { class: if selected { "device-row selected" } else { "device-row" }, r#type: "button",
        disabled, "aria-pressed": selected, onclick: move |_| onclick.call(id.clone()),
        onkeydown: move |e| { if keyboard_activate(&e) && !disabled { onclick.call(key_id.clone()); } },
        div { class: "device-glyph", Icon { kind: IconKind::Bluetooth } }
        div { class: "device-description", strong { "{device.name}" } span { "{device.id}" } }
        div { class: "device-meta", span { "{device.detail}" } if selected { Icon { kind: IconKind::Check } } }
    } }
}

#[component]
pub fn ConnectionPage(
    capabilities: Capabilities,
    mut form: Signal<ConnectionForm>,
    #[props(default)] devices: Vec<Device>,
    #[props(default)] scanning: bool,
    #[props(default)] scanned: bool,
    #[props(default)] stage: ConnectionStage,
    #[props(default)] recent: Vec<ConnectionForm>,
    onscan: EventHandler<Transport>,
    onconnect: EventHandler<ConnectionForm>,
    oncancel: EventHandler<()>,
    #[props(default)] ontransport: EventHandler<Transport>,
) -> Element {
    let light = use_signal(|| false);
    let f = form();
    let busy = stage == ConnectionStage::Connecting;
    let selected = devices
        .iter()
        .find(|d| d.id == f.device)
        .map(|d| d.name.clone());
    let valid = f.validate(capabilities).is_ok();
    let heading = if f.transport.network() {
        "Connect over a network"
    } else if f.transport.cable() {
        "Connect with a cable"
    } else {
        "Find your controller"
    };
    let description = if f.transport.network() {
        "Enter the controller or bridge address on your network."
    } else if f.transport.cable() {
        "Choose the interface exposed by your controller or bridge."
    } else {
        "Turn on your controller or BLE bridge, then search for nearby devices."
    };
    let mut change = move |t: Transport| {
        form.write().transport = t;
        form.write().device.clear();
        ontransport.call(t);
    };
    rsx! {
        style { "{CSS}" }
        div { class: if light() { "ox-app light" } else { "ox-app" },
            header { class: "app-header", Brand {} div { class: "flex items-center gap-4", span { class: "header-caption", "Drive studio" } ThemeButton { light } } }
            main { class: "connect-layout",
                aside { class: "connect-intro",
                    h1 { "Your next " br {} "connection." }
                    p { class: "intro-copy", "A clear view of your drive, from the first signal to the final tune." }
                    div { class: "connection-route", div { class: "route-node", Icon { kind: IconKind::Settings } span { "This device" } }
                        div { class: "route-line" }
                        div { class: "route-node", Icon { kind: if f.transport.network() { IconKind::Network } else if f.transport.cable() { IconKind::Usb } else { IconKind::Bluetooth } } span { "Controller" } }
                    }
                    if !recent.is_empty() {
                        div { class: "recent", h2 { "Recent connections" }
                            for entry in recent { { let e = entry.clone(); let key_entry = entry.clone(); rsx! {
                                button { class: "recent-row", disabled: busy || scanning, onclick: move |_| { form.set(e.clone()); ontransport.call(e.transport); },
                                    onkeydown: move |event| { if keyboard_activate(&event) && !busy && !scanning { form.set(key_entry.clone()); ontransport.call(key_entry.transport); } },
                                    strong { "{entry.transport.label()}" } span { "{entry.endpoint()}" }
                                }
                            } } }
                        }
                    }
                    p { class: "intro-footnote", "Connect first. Inspect, configure and tune when you’re ready." }
                }
                section { class: "connection-panel", "aria-label": "Connect to a controller",
                    div { class: "panel-heading", div { h2 { "Connect a controller" } p { "Choose how you want to connect." } } Status { "Not connected" } }
                    div { class: "transport-tabs", role: "group", "aria-label": "Connection method",
                        for (label, icon, target, active) in [
                            ("Bluetooth", IconKind::Bluetooth, Transport::Ble, f.transport == Transport::Ble),
                            ("Network", IconKind::Network, Transport::Tcp, f.transport.network()),
                            ("USB", IconKind::Usb, Transport::Usb, f.transport.cable()),
                        ].into_iter().filter(|(_, _, t, _)| capabilities.allows(*t)) {
                            button { class: if active { "transport-tab active" } else { "transport-tab" }, disabled: busy || scanning,
                                r#type: "button", "aria-pressed": active, onclick: move |_| change(target),
                                onkeydown: move |e| { if keyboard_activate(&e) && !busy && !scanning { change(target); } },
                                Icon { kind: icon } "{label}"
                            }
                        }
                    }
                    div { class: "connection-body",
                        div { class: "section-heading", h3 { "{heading}" } p { "{description}" } }
                        if f.transport.network() {
                            div { class: "protocol-switch", role: "group", "aria-label": "Network protocol",
                                for t in [Transport::Tcp, Transport::Udp] {
                                    button { r#type: "button", class: if f.transport == t { "protocol active" } else { "protocol" }, disabled: busy,
                                        onkeydown: move |e| { if keyboard_activate(&e) && !busy { change(t); } },
                                        "aria-pressed": f.transport == t, onclick: move |_| change(t), "{t.label()}" }
                                }
                            }
                            div { class: "network-fields",
                                Field { id: "host", label: "Address", value: f.host.clone(), placeholder: "192.168.1.42", disabled: busy,
                                    oninput: move |v| form.write().host = v, hint: "IP address or hostname" }
                                Field { id: "port", label: "Port", value: f.port.clone(), numeric: true, disabled: busy,
                                    oninput: move |v| form.write().port = v }
                            }
                            p { class: "connection-note", "Use the address of the device running OxiFOC or its network bridge." }
                        } else {
                            if f.transport.cable() {
                                div { class: "protocol-switch", role: "group", "aria-label": "USB interface",
                                    for t in [Transport::Usb, Transport::Serial, Transport::Rtt] {
                                        button { r#type: "button", class: if f.transport == t { "protocol active" } else { "protocol" }, disabled: busy || scanning,
                                            onkeydown: move |e| { if keyboard_activate(&e) && !busy && !scanning { change(t); } },
                                            "aria-pressed": f.transport == t, onclick: move |_| change(t), "{t.label()}" }
                                    }
                                }
                            }
                            if f.transport == Transport::Usb {
                                div { class: "empty-state", Icon { kind: IconKind::Usb } h4 { "Plug in your controller" }
                                    p { "Connect one compatible USB controller. We’ll identify it when you connect." } }
                            } else {
                                div { class: "list-toolbar", span { if scanning { "Searching…" } else if scanned { if devices.len() == 1 { "1 device found" } else { "{devices.len()} devices found" } } else { "Available devices" } }
                                    Button { variant: ButtonVariant::Secondary, busy: scanning, disabled: busy,
                                        onclick: move |_| onscan.call(f.transport), if scanned { "Search again" } else { "Search devices" } }
                                }
                                div { class: "device-list", "aria-label": "Available devices",
                                    if devices.is_empty() {
                                        div { class: "empty-state", Icon { kind: if f.transport.cable() { IconKind::Usb } else { IconKind::Bluetooth } }
                                            h4 { if scanning { "Looking for devices" } else if scanned { "No devices found" } else { "Ready when you are" } }
                                            p { if scanning { "Keep your controller nearby." } else if scanned { "Check that your controller is on and available, then search again." } else { "Search to discover controllers you can connect to." } }
                                        }
                                    }
                                    for device in devices { DeviceRow { selected: f.device == device.id, device, disabled: busy || scanning,
                                        onclick: move |id| form.write().device = id } }
                                }
                                if f.transport == Transport::Serial { Field { id: "baud", label: "Baud rate", value: f.baud.clone(), numeric: true, disabled: busy, oninput: move |v| form.write().baud = v } }
                                if f.transport == Transport::Rtt { Field { id: "chip", label: "Target chip", value: f.chip.clone(), disabled: busy, oninput: move |v| form.write().chip = v } }
                                Disclosure { title: "Connection details", p { "{f.transport.label()}" } p { class: "address-text", if f.device.is_empty() { "No device selected" } else { "{f.device}" } } }
                            }
                        }
                        if let ConnectionStage::Error(message) = stage {
                            div { class: "notice danger", role: "alert", strong { "Unable to connect" } p { "{message}" } }
                        }
                        if busy { div { class: "notice", role: "status", strong { "Connecting to controller…" } p { "Waiting for the controller to identify itself." } } }
                    }
                    Separator { decorative: true }
                    footer { class: "connection-footer",
                        div { class: "selection-summary", strong { if valid { if let Some(name) = selected { "{name}" } else { "{f.transport.label()}" } } else { "Choose a connection" } }
                            span { if valid { "{f.endpoint()}" } else { "Select a device or enter its network address." } }
                        }
                        if busy { Button { variant: ButtonVariant::Secondary, onclick: move |_| oncancel.call(()), "Cancel" } }
                        else { Button { disabled: !valid || scanning, onclick: move |_| onconnect.call(form()), "Connect" Icon { kind: IconKind::Arrow } } }
                    }
                }
            }
            footer { class: "app-footer", span { "OxiFOC · Motor control workspace" } span { "Made for a closer look." } }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transport_availability_is_independent_of_screen_width() {
        for t in [Transport::Ble, Transport::Tcp, Transport::Udp] {
            assert!(Capabilities::MOBILE.allows(t));
        }
        for t in [Transport::Usb, Transport::Serial, Transport::Rtt] {
            assert!(!Capabilities::MOBILE.allows(t));
            assert!(Capabilities::DESKTOP.allows(t));
        }
    }
    #[test]
    fn network_input_rejects_urls_and_invalid_ports() {
        let mut f = ConnectionForm {
            transport: Transport::Tcp,
            host: "localhost".into(),
            ..Default::default()
        };
        assert!(f.validate(Capabilities::MOBILE).is_ok());
        for port in ["0", "65536", "-1", "abc", ""] {
            f.port = port.into();
            assert!(f.validate(Capabilities::MOBILE).is_err());
        }
        f.port = "2025".into();
        for host in ["", "http://localhost", "one two", "localhost/path"] {
            f.host = host.into();
            assert!(f.validate(Capabilities::MOBILE).is_err());
        }
        f.host = "::1".into();
        assert!(f.validate(Capabilities::MOBILE).is_ok());
    }
}

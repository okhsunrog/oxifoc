//! Shared connection flow for OxiFOC Dioxus hosts.
mod controller;
pub use controller::{ConnectedSession, Controller, Snapshot};
use dioxus::prelude::*;
use oxifoc_ui::*;
use std::sync::{Arc, atomic::Ordering};
use std::time::Duration;

#[derive(Clone)]
pub struct Flow {
    pub controller: Arc<Controller>,
    pub snapshot: Signal<Snapshot>,
    pub form: Signal<ConnectionForm>,
    pub capabilities: Capabilities,
}
impl PartialEq for Flow {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.controller, &other.controller)
    }
}

pub fn use_connection(capabilities: Capabilities) -> Flow {
    let controller = use_hook(|| Arc::new(Controller::new(capabilities)));
    let form = use_signal(|| {
        let mut f = ConnectionForm::default();
        if let Ok(host) = std::env::var("OXIFOC_TCP_HOST") {
            f.host = host;
            f.transport = Transport::Tcp;
        }
        if let Ok(port) = std::env::var("OXIFOC_TCP_PORT") {
            f.port = port;
            f.transport = Transport::Tcp;
            if f.host.is_empty() {
                f.host = "127.0.0.1".into();
            }
        }
        f
    });
    let mut snapshot = use_signal(Snapshot::default);
    let poller = controller.clone();
    use_future(move || {
        let poller = poller.clone();
        async move {
            loop {
                let next = poller.snapshot();
                if *snapshot.peek() != next {
                    snapshot.set(next);
                }
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    });
    Flow {
        controller,
        snapshot,
        form,
        capabilities,
    }
}

#[component]
pub fn ConnectScreen(flow: Flow) -> Element {
    let s = (flow.snapshot)();
    let scan = flow.controller.clone();
    let connect = flow.controller.clone();
    let cancel = flow.controller.clone();
    let change = flow.controller.clone();
    rsx! { ConnectionPage {
        capabilities: flow.capabilities, form: flow.form,
        devices: s.devices, scanning: s.scanning, scanned: s.scanned,
        stage: s.stage, recent: s.recent,
        onscan: move |transport| {
            let controller = scan.clone();
            spawn(async move {
                #[cfg(target_os = "android")]
                if transport == Transport::Ble {
                    controller.permission_pending();
                    let permission = async {
                        dioxus_blec::prepare_android().await.map_err(|e| e.to_string())?;
                        oxifoc_host_lib::initialize_ble().await.map_err(|e| e.to_string())?;
                        if !dioxus_blec::blec::check_permissions(false).await.map_err(|e| e.to_string())? {
                            return Err("Nearby devices permission was denied. Allow Bluetooth access in Android settings, then search again.".to_string());
                        }
                        Ok::<_, String>(())
                    }.await;
                    if let Err(error) = permission { controller.error(error); return; }
                }
                controller.scan(transport);
            });
        },
        ontransport: move |_| change.clear_discovery(),
        onconnect: move |form| connect.connect(form), oncancel: move |_| cancel.disconnect(),
    } }
}

/// WebView/mobile entry point. The native host supplies its existing plot workspace.
#[component]
pub fn HostApp() -> Element {
    // Android's JNI entry point has no Rust command-line argument setup.
    #[cfg(not(target_os = "android"))]
    if std::env::args().any(|s| s == "--catalogue") {
        return rsx! { Catalogue {} };
    }
    let capabilities = if cfg!(feature = "desktop") && !cfg!(target_os = "android") {
        Capabilities::DESKTOP
    } else {
        Capabilities::MOBILE
    };
    let flow = use_connection(capabilities);
    match (flow.snapshot)().session {
        Some(session) => {
            rsx! { SessionOverview { session, ondisconnect: move |_| flow.controller.disconnect() } }
        }
        None => rsx! { ConnectScreen { flow } },
    }
}

#[component]
pub fn SessionOverview(session: ConnectedSession, ondisconnect: EventHandler<()>) -> Element {
    let light = use_signal(|| false);
    let mut connected = use_signal(|| true);
    let mut samples = use_signal(|| 0u64);
    let mut volts = use_signal(|| "—".to_owned());
    let mut motor = use_signal(|| "Waiting for telemetry".to_owned());
    let mut identity = use_signal(|| "Controller".to_owned());
    let mut faults = use_signal(|| 0u8);
    let runtime = session.runtime.clone();
    use_future(move || {
        let runtime = runtime.clone();
        async move {
            loop {
                connected.set(runtime.connected.load(Ordering::Relaxed));
                *samples.write() += runtime.fast_rx.try_iter().count() as u64;
                for data in runtime.slow_rx.try_iter() {
                    volts.set(format!("{:.1} V", data.vbus_mv as f32 / 1000.));
                    motor.set(format!("{:?}", data.motor_state));
                    faults.set(data.fault_count);
                }
                for info in runtime.device_info_rx.try_iter() {
                    identity.set(info.hw.to_string());
                }
                for _ in runtime.fault_rx.try_iter() {}
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    });
    rsx! {
        style { "{CSS}" }
        div { class: if light() { "ox-app light" } else { "ox-app" },
            header { class: "app-header", Brand {} ThemeButton { light } }
            main { class: "session-layout",
                div { class: "session-heading", div { h1 { "{identity}" } p { "{session.form.transport.label()} · {session.form.endpoint()}" } }
                    Button { variant: ButtonVariant::Secondary, onclick: move |_| ondisconnect.call(()), "Disconnect" } }
                Status { tone: if connected() { Tone::Success } else { Tone::Warning }, if connected() { "Connected" } else { "Reconnecting" } }
                div { class: "session-metrics",
                    div { class: "metric", p { "Bus voltage" } strong { class: "numeric", if connected() { "{volts}" } else { "—" } } }
                    div { class: "metric", p { "Drive state" } strong { if connected() { "{motor}" } else { "Unavailable" } } }
                    div { class: "metric", p { "Samples received" } strong { class: "numeric", "{samples}" } }
                }
                div { class: "notice", p { "Active faults: {faults}" } }
                p { class: "connection-note", "Live controller telemetry" }
            }
        }
    }
}

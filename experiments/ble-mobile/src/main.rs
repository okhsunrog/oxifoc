use dioxus::prelude::*;
use oxifoc_host_lib::{
    BleDeviceInfo, HostConfig, HostRuntime, TransportType, start_host, try_scan_ble_devices,
};
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn main() {
    dioxus::launch(App);
}

#[component]
fn App() -> Element {
    let runtime = use_hook(|| Arc::new(Mutex::new(None::<HostRuntime>)));
    let mut devices = use_signal(Vec::<BleDeviceInfo>::new);
    let mut status = use_signal(|| "Ready to scan for OxiFOC NUS devices".to_owned());
    let mut scanning = use_signal(|| false);
    let mut telemetry = use_signal(|| "Disconnected".to_owned());
    let poll_runtime = runtime.clone();
    use_future(move || {
        let runtime = poll_runtime.clone();
        async move {
            let mut samples = 0u64;
            let mut last = String::new();
            loop {
                {
                    let guard = runtime.lock().unwrap();
                    if let Some(host) = guard.as_ref() {
                        samples += host.fast_rx.try_iter().count() as u64;
                        for value in host.slow_rx.try_iter() {
                            last = format!(
                                "{:?} · {:.1} V · faults {}",
                                value.motor_state,
                                value.vbus_mv as f32 / 1000.,
                                value.fault_count
                            );
                        }
                        for info in host.device_info_rx.try_iter() {
                            tracing::info!("BLE device identity: {info:?}");
                        }
                        for _ in host.fault_rx.try_iter() {}
                        let connected = host.connected.load(std::sync::atomic::Ordering::Relaxed);
                        let text = format!("Connected: {connected}\nSamples: {samples}\n{last}");
                        telemetry.set(text);
                        tracing::info!("BLELAB connected={connected} samples={samples} {last}");
                    } else {
                        samples = 0;
                        last.clear();
                        telemetry.set("Disconnected".into());
                    }
                }
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    });
    let connect_runtime = runtime.clone();
    let disconnect_runtime = runtime.clone();
    rsx! {
        style { "body{{margin:0;background:#101820;color:#edf4f4;font-family:system-ui;padding:48px 24px}} h1{{font-size:32px}} p{{color:#9bafb5;line-height:1.6}} button{{background:#40d9b0;color:#10251f;border:0;border-radius:16px;padding:16px 20px;margin:8px 0;font-size:16px;display:block;width:100%}} pre{{background:#1d2b35;border-radius:18px;padding:20px;white-space:pre-wrap}} .device{{background:#243943;color:#edf4f4;text-align:left}}" }
        p { "OXIFOC / BLE LAB" }
        h1 { "Drive connection" }
        p { "Shared Rust backend · Android Bluetooth · NUS transport" }
        button { disabled: scanning(), onclick: move |_| async move {
            scanning.set(true);
            status.set("Scanning…".into());
            // Referencing the wrapper also packages its Kotlin Gradle module.
            let result = async {
                dioxus_blec::prepare_android().await.map_err(|e| e.to_string())?;
                oxifoc_host_lib::initialize_ble().await.map_err(|e| e.to_string())?;
                let granted = dioxus_blec::blec::check_permissions(false).await.map_err(|e| e.to_string())?;
                if !granted { return Err("Bluetooth permission denied".into()); }
                try_scan_ble_devices(Duration::from_secs(4)).await.map_err(|e| format!("{e:#}"))
            }.await;
            match result {
                Ok(found) => { status.set(format!("Found {} NUS devices", found.len())); devices.set(found); }
                Err(error) => { tracing::error!("BLELAB scan: {error}"); status.set(error); }
            }
            scanning.set(false);
        }, "Scan devices" }
        p { "{status}" }
        for device in devices() {
            {let runtime = connect_runtime.clone(); let address = device.id.clone(); rsx! {
                button { class: "device", key: "{device.id}", onclick: move |_| {
                    let mut guard = runtime.lock().unwrap();
                    if let Some(old) = guard.take() { old.shutdown(); }
                    *guard = Some(start_host(HostConfig { transport: Some(TransportType::Ble), ble_address: Some(address.clone()), fast_hz: Some(100), stream_defmt: Some(false), ..Default::default() }));
                    status.set(format!("Connecting to {address}"));
                }, "{device}" }
            }}
        }
        pre { "{telemetry}" }
        button { onclick: move |_| {
            if let Some(host) = disconnect_runtime.lock().unwrap().take() { host.shutdown(); }
            status.set("Disconnected".into());
        }, "Disconnect" }
        p { "This lab only reads controller telemetry. Motor commands stay in the main application." }
    }
}

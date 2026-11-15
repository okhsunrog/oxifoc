use crate::*;
use dioxus::prelude::*;

/// Deliberately separate from the product navigation; launch with --catalogue.
#[component]
pub fn Catalogue() -> Element {
    let light = use_signal(|| false);
    let mut name = use_signal(|| "192.168.1.42".to_string());
    let mut message = use_signal(|| "Choose an action to inspect its state.".to_string());
    let mut dialog = use_signal(|| false);
    let mut dialog_trigger = use_signal(|| None::<std::rc::Rc<MountedData>>);
    let restore_focus = move || {
        if let Some(node) = dialog_trigger() {
            spawn(async move {
                let _ = node.set_focus(true).await;
            });
        }
    };
    let mut selected = use_signal(|| true);
    rsx! {
        style { "{CSS}" }
        div { class: if light() { "ox-app light" } else { "ox-app" },
            header { class: "app-header", Brand {} ThemeButton { light } }
            main { class: "catalogue",
                h1 { "Interface workshop" }
                p { class: "intro-copy", "OxiFOC components. Reference states, keyboard interaction and renderer compatibility." }
                section { class: "catalogue-section", h2 { "Actions" }
                    div { class: "flex flex-wrap gap-3",
                        Button { onclick: move |_| message.set("Primary action received.".into()), "Connect" }
                        Button { variant: ButtonVariant::Secondary, onclick: move |_| message.set("Search requested.".into()), "Search again" }
                        button { r#type: "button", class: "ox-button button-ghost",
                            onmounted: move |e| dialog_trigger.set(Some(e.data())),
                            onclick: move |_| dialog.set(true),
                            onkeydown: move |e| { if keyboard_activate(&e) { dialog.set(true); } },
                            "Open dialog" }
                        Button { variant: ButtonVariant::Danger, onclick: move |_| message.set("Destructive action preview. No command sent.".into()), "Disconnect" }
                        Button { disabled: true, "Unavailable" }
                        Button { busy: true, "Searching" }
                    }
                    p { role: "status", "{message}" }
                }
                section { class: "catalogue-section", h2 { "Fields & feedback" }
                    div { class: "catalogue-grid",
                        Field { id: "example-address", label: "Address", value: name(), oninput: move |v| name.set(v), hint: "Hostname or IP address" }
                        Field { id: "example-invalid", label: "Port", value: "70000", error: "Port must be between 1 and 65535.", oninput: move |_| {} }
                    }
                    div { class: "flex flex-wrap gap-4", Status { "Not connected" } Status { tone: Tone::Success, "Connected" }
                        Status { tone: Tone::Warning, "Reconnecting" } Status { tone: Tone::Danger, "Connection failed" } }
                }
                section { class: "catalogue-section", h2 { "Device selection" }
                    p { "Reference content for this catalogue only." }
                    DeviceRow { device: Device { id: "AA:BB:CC:DD:EE:FF".into(), name: "Reference controller".into(), detail: "−54 dBm".into() },
                        selected: selected(), onclick: move |_| { let next = !selected(); selected.set(next); } }
                    Disclosure { title: "Advanced connection settings", p { "This disclosure uses Rust state and native button semantics. It does not execute JavaScript." }
                        Field { id: "example-baud", label: "Baud rate", value: "921600", oninput: move |_| {} } }
                }
                section { class: "catalogue-section", h2 { "Scroll & density" }
                    div { class: "scroll-demo", tabindex: "0", "aria-label": "Scrollable reference list",
                        for i in 1..=12 { div { class: "scroll-row", span { "Signal {i}" } span { class: "numeric", "{i}.00 A" } } }
                    }
                }
            }
            if dialog() {
                ConfirmDialog { title: "Leave this session?", description: "This is a component preview. No controller is connected.",
                    confirm: "Leave session", onclose: move |_| { dialog.set(false); message.set("Dialog cancelled.".into()); restore_focus(); },
                    onconfirm: move |_| { dialog.set(false); message.set("Dialog confirmed.".into()); restore_focus(); } }
            }
        }
    }
}

/// Focus stays in the two-action dialog using Dioxus mounted-element handles.
/// No browser document/eval dependency. Return focus is owned by the caller.
#[component]
pub fn ConfirmDialog(
    title: String,
    description: String,
    confirm: String,
    onclose: EventHandler<()>,
    onconfirm: EventHandler<()>,
) -> Element {
    let mut cancel_node = use_signal(|| None::<std::rc::Rc<MountedData>>);
    let mut confirm_node = use_signal(|| None::<std::rc::Rc<MountedData>>);
    let mut focus_confirm = use_signal(|| false);
    rsx! { div { class: "dialog-backdrop", onclick: move |_| onclose.call(()),
        div { class: "ox-dialog", role: "dialog", "aria-modal": "true", "aria-labelledby": "dialog-title", "aria-describedby": "dialog-description",
            onclick: move |e| e.stop_propagation(),
            onkeydown: move |e| {
                if e.key() == Key::Named(NamedKey::Escape) { e.stop_propagation(); onclose.call(()); }
                if e.key() == Key::Named(NamedKey::Tab) {
                    e.prevent_default(); e.stop_propagation();
                    let next = !focus_confirm(); focus_confirm.set(next);
                    let node = if next { confirm_node() } else { cancel_node() };
                    if let Some(node) = node { spawn(async move { let _ = node.set_focus(true).await; }); }
                }
            },
            h2 { id: "dialog-title", "{title}" } p { id: "dialog-description", "{description}" }
            div { class: "dialog-actions",
                button { r#type: "button", class: "ox-button button-secondary",
                    onkeydown: move |e| { if keyboard_activate(&e) { onclose.call(()); } },
                    onmounted: move |e| { let node = e.data(); cancel_node.set(Some(node.clone())); spawn(async move { let _ = node.set_focus(true).await; }); },
                    onfocus: move |_| focus_confirm.set(false), onclick: move |_| onclose.call(()), "Cancel" }
                button { r#type: "button", class: "ox-button button-danger",
                    onkeydown: move |e| { if keyboard_activate(&e) { onconfirm.call(()); } },
                    onmounted: move |e| confirm_node.set(Some(e.data())), onfocus: move |_| focus_confirm.set(true),
                    onclick: move |_| onconfirm.call(()), "{confirm}" }
            }
        }
    } }
}

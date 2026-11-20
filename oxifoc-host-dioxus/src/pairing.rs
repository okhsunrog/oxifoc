//! BLE pairing page: bonds on the bridge and the pairing window
//! (docs/notes/ble-pairing.md). State lives in `pairing_desk`.

use crate::model::Model;
use dioxus::prelude::*;
use oxifoc_core::types::pairing::{BondEntry, BondRole, MAX_BONDS};
use oxifoc_host_lib::pairing_desk::{
    Op, PairingDesk, Step, WINDOW_SECS, Window, format_address, format_code, parse_passkey,
    role_label, validate_name,
};
use oxifoc_ui::{Button, ButtonVariant, Field, Status, Tone};
use std::{sync::Arc, time::Instant};

#[component]
pub(crate) fn Pairing(model: Arc<Model>, desk: PairingDesk) -> Element {
    let m = model.clone();
    use_hook(move || {
        if m.snapshot.lock().unwrap().pairing.bonds.is_none() {
            m.pairing(Op::List);
        }
    });
    let mut removing = use_signal(|| None::<BondEntry>);
    let busy = desk.busy.is_some();
    let count = desk.bonds.as_ref().map_or(0, Vec::len);
    let list_model = model.clone();
    let remove_model = model.clone();

    rsx! {
        section { class: "pairing",
            div { class: "titlebar",
                div {
                    h1 { "Paired devices" }
                    p { "Only devices bonded with the bridge reach the controller over Bluetooth." }
                }
                Button {
                    variant: ButtonVariant::Secondary,
                    disabled: busy,
                    onclick: move |_| list_model.pairing(Op::List),
                    "Refresh"
                }
            }
            if let Some(error) = &desk.error {
                div { class: "notice danger", role: "alert",
                    strong { "Request failed" }
                    p { "{error}" }
                }
            }
            if let Some(window) = &desk.window {
                WindowCard { model: model.clone(), window: window.clone(), busy }
            } else {
                NewDevice { model: model.clone(), desk: desk.clone() }
            }
            section { class: "pairing-card",
                div { class: "card-heading",
                    h2 { "Bonds" }
                    span { class: "numeric", "{count} of {MAX_BONDS}" }
                }
                match &desk.bonds {
                    None => rsx! {
                        p { class: "card-empty",
                            if busy { "Reading bonds from the bridge…" } else { "Bonds not read yet." }
                        }
                    },
                    Some(bonds) if bonds.is_empty() => rsx! {
                        p { class: "card-empty", "No devices are paired. Pair one above." }
                    },
                    Some(bonds) => rsx! {
                        div { class: "bond-list",
                            for bond in bonds.iter().cloned() {
                                BondRow {
                                    key: "{bond.id}",
                                    model: model.clone(),
                                    bond: bond.clone(),
                                    by: bond.added_by.map(|id| bonds.iter().find(|b| b.id == id)
                                        .map_or(format!("bond {id}"), |b| b.name.to_string())),
                                    busy,
                                    onremove: move |bond| removing.set(Some(bond)),
                                }
                            }
                        }
                    },
                }
            }
        }
        if let Some(bond) = removing() {
            div { class: "dialog-backdrop",
                div { class: "ox-dialog", role: "alertdialog", "aria-labelledby": "remove-title",
                    h2 { id: "remove-title", "Remove {bond.name}?" }
                    p { "The device is disconnected and must pair again to reach the controller." }
                    p { class: "dialog-note",
                        "If this computer is connected over Bluetooth with this bond, it loses the connection too."
                    }
                    div { class: "dialog-actions",
                        Button { variant: ButtonVariant::Ghost, onclick: move |_| removing.set(None), "Keep" }
                        Button {
                            variant: ButtonVariant::Danger,
                            disabled: busy,
                            onclick: move |_| {
                                remove_model.pairing(Op::Remove { id: bond.id });
                                removing.set(None);
                            },
                            "Remove bond"
                        }
                    }
                }
            }
        }
    }
}

/// Form that opens a window. Shown when no window is open.
#[component]
fn NewDevice(model: Arc<Model>, desk: PairingDesk) -> Element {
    let mut role = use_signal(|| BondRole::Remote);
    let mut name = use_signal(String::new);
    let mut error = use_signal(String::new);
    let busy = desk.busy.is_some();
    let full = desk.bonds.as_ref().is_some_and(|b| b.len() >= MAX_BONDS);
    let mut open = move || match validate_name(&name()) {
        Ok(valid) => {
            error.set(String::new());
            model.pairing(Op::Open {
                role: role(),
                name: valid,
            });
        }
        Err(e) => error.set(e),
    };
    let how = match role() {
        BondRole::Remote => "Turn the remote on. It shows a six-digit code, which you enter here.",
        BondRole::Host => {
            "Connect from the phone or computer. Both screens then show a number to compare."
        }
    };
    rsx! {
        section { class: "pairing-card",
            div { class: "card-heading", h2 { "Pair a new device" } }
            if let Some(outcome) = &desk.outcome {
                div { class: if outcome.is_success() { "outcome success" } else { "outcome failure" },
                    role: "status",
                    Status {
                        tone: if outcome.is_success() { Tone::Success } else { Tone::Danger },
                        "{outcome.message()}"
                    }
                }
            }
            div { class: "pair-form",
                div { class: "role-switch", role: "radiogroup", "aria-label": "Device type",
                    for option in [BondRole::Remote, BondRole::Host] {
                        button {
                            class: if role() == option { "role-option active" } else { "role-option" },
                            role: "radio", "aria-checked": role() == option,
                            onclick: move |_| role.set(option),
                            onkeydown: move |e| { if oxifoc_ui::keyboard_activate(&e) { role.set(option); } },
                            "{role_label(option)}"
                        }
                    }
                }
                Field {
                    id: "bond-name", label: "Name", value: name(),
                    placeholder: if role() == BondRole::Remote { "Skate remote" } else { "My phone" },
                    hint: "Stored on the bridge with the bond.",
                    error: error(),
                    oninput: move |v| name.set(v),
                }
                p { class: "pair-how", "{how}" }
                div { class: "pair-actions",
                    if full { p { class: "error", "All {MAX_BONDS} slots are used. Remove a bond first." } }
                    Button {
                        busy: desk.busy == Some("Opening the window"),
                        disabled: busy || full,
                        onclick: move |_| open(),
                        "Open pairing window"
                    }
                }
            }
        }
    }
}

/// An open window: countdown and the prompt it waits for.
#[component]
fn WindowCard(model: Arc<Model>, window: Window, busy: bool) -> Element {
    let mut passkey = use_signal(String::new);
    let mut passkey_error = use_signal(String::new);
    let attempt = window.attempt;
    let left = window.remaining(Instant::now()).as_secs();
    let role = window.role.map_or("device", role_label);
    let own = window.is_own();
    let candidate = window.candidate.map(|a| format_address(&a));
    let cancel_model = model.clone();
    let yes_model = model.clone();
    let no_model = model.clone();
    let mut submit = move || match parse_passkey(&passkey()) {
        Ok(code) => {
            passkey_error.set(String::new());
            model.pairing(Op::Passkey {
                attempt,
                passkey: code,
            });
        }
        Err(e) => passkey_error.set(e.into()),
    };
    let mut submit_key = submit.clone();
    let title = match &window.name {
        Some(name) => format!("Pairing {name}"),
        None => "Pairing window open".into(),
    };
    rsx! {
        section { class: "pairing-card window-card",
            div { class: "card-heading",
                div {
                    h2 { "{title}" }
                    p { "{role} • window {attempt}" }
                }
                div { class: "countdown numeric", "aria-live": "off",
                    strong { "{left}" } span { " s left" }
                }
            }
            div { class: "window-progress",
                div { class: "window-progress-fill", style: "width:{(left * 100 / u64::from(WINDOW_SECS)).min(100)}%" }
            }
            div { class: "window-body", "aria-live": "polite",
                if !own {
                    p { "Another client opened this window and answers its prompts." }
                }
                match window.step {
                    Step::Waiting => rsx! {
                        div { class: "step",
                            span { class: "spinner", "aria-hidden": "true" }
                            div {
                                strong { "Waiting for the {role}" }
                                p {
                                    if window.role == Some(BondRole::Remote) {
                                        "Turn the remote on near the board. It connects by itself."
                                    } else {
                                        "Start pairing from the phone or computer now."
                                    }
                                }
                            }
                        }
                    },
                    Step::Negotiating => rsx! {
                        div { class: "step",
                            span { class: "spinner", "aria-hidden": "true" }
                            div {
                                strong { "Device connected" }
                                p { "Exchanging keys with {candidate.clone().unwrap_or_default()}…" }
                            }
                        }
                    },
                    Step::EnterPasskey => rsx! {
                        div { class: "prompt",
                            strong { "Enter the code shown on the remote" }
                            div { class: "passkey-row",
                                input {
                                    id: "passkey", r#type: "text", inputmode: "numeric",
                                    class: if passkey_error().is_empty() { "ox-input passkey" } else { "ox-input passkey invalid" },
                                    value: "{passkey}", placeholder: "000 000", autocomplete: "off",
                                    spellcheck: false, disabled: !own,
                                    "aria-label": "Six-digit code",
                                    oninput: move |e| passkey.set(e.value()),
                                    onkeydown: move |e| { if e.key() == Key::Named(NamedKey::Enter) { submit_key(); } },
                                }
                                Button { disabled: busy || !own, onclick: move |_| submit(), "Pair" }
                            }
                            if !passkey_error().is_empty() { p { class: "error", "{passkey_error}" } }
                        }
                    },
                    Step::CompareNumber(number) => rsx! {
                        div { class: "prompt",
                            strong { "Does the device show this number?" }
                            div { class: "code numeric", "{format_code(number)}" }
                            p { "Confirm on the device too. Reject if the numbers differ." }
                            div { class: "prompt-actions",
                                Button {
                                    variant: ButtonVariant::Secondary, disabled: busy || !own,
                                    onclick: move |_| no_model.pairing(Op::Confirm { attempt, accept: false }),
                                    "Numbers differ"
                                }
                                Button {
                                    disabled: busy || !own,
                                    onclick: move |_| yes_model.pairing(Op::Confirm { attempt, accept: true }),
                                    "Numbers match"
                                }
                            }
                        }
                    },
                    Step::ShowPasskey(code) => rsx! {
                        div { class: "prompt",
                            strong { "Type this code on the device" }
                            div { class: "code numeric", "{format_code(code)}" }
                        }
                    },
                    Step::Checking => rsx! {
                        div { class: "step",
                            span { class: "spinner", "aria-hidden": "true" }
                            div { strong { "Checking" } p { "Waiting for the device to finish pairing…" } }
                        }
                    },
                }
            }
            div { class: "window-footer",
                Button {
                    variant: ButtonVariant::Ghost, disabled: busy,
                    onclick: move |_| cancel_model.pairing(Op::Cancel { attempt }),
                    "Cancel pairing"
                }
            }
        }
    }
}

#[component]
fn BondRow(
    model: Arc<Model>,
    bond: BondEntry,
    by: Option<String>,
    busy: bool,
    onremove: EventHandler<BondEntry>,
) -> Element {
    let mut editing = use_signal(|| None::<String>);
    let mut error = use_signal(String::new);
    let id = bond.id;
    let address = format_address(&bond.address);
    let origin = by.map_or("Added over a wired link".to_string(), |by| {
        format!("Added by {by}")
    });
    let mut save = move || {
        let Some(text) = editing() else { return };
        match validate_name(&text) {
            Ok(name) => {
                model.pairing(Op::Rename { id, name });
                editing.set(None);
                error.set(String::new());
            }
            Err(e) => error.set(e),
        }
    };
    let mut save_key = save.clone();
    let remove_bond = bond.clone();
    rsx! {
        div { class: "bond-row",
            div { class: "bond-glyph", "aria-hidden": "true",
                if bond.role == BondRole::Remote { "◉" } else { "▭" }
            }
            div { class: "bond-main",
                if let Some(text) = editing() {
                    div { class: "rename-row",
                        input {
                            class: if error().is_empty() { "ox-input" } else { "ox-input invalid" },
                            value: "{text}", "aria-label": "New name", spellcheck: false,
                            oninput: move |e| editing.set(Some(e.value())),
                            onkeydown: move |e| match e.key() {
                                Key::Named(NamedKey::Enter) => save_key(),
                                Key::Named(NamedKey::Escape) => editing.set(None),
                                _ => {}
                            },
                        }
                        Button { disabled: busy, onclick: move |_| save(), "Save" }
                        Button { variant: ButtonVariant::Ghost, onclick: move |_| editing.set(None), "Cancel" }
                    }
                    if !error().is_empty() { p { class: "error", "{error}" } }
                } else {
                    div { class: "bond-title",
                        strong { "{bond.name}" }
                        span { class: "role-badge", "{role_label(bond.role)}" }
                    }
                    p { class: "bond-meta",
                        span { class: "numeric", "{address}" }
                        span { "{origin}" }
                    }
                }
            }
            if editing().is_none() {
                div { class: "bond-actions",
                    Button {
                        variant: ButtonVariant::Ghost, disabled: busy,
                        onclick: move |_| editing.set(Some(bond.name.to_string())),
                        "Rename"
                    }
                    Button {
                        variant: ButtonVariant::Secondary, disabled: busy,
                        onclick: move |_| onremove.call(remove_bond.clone()),
                        "Remove"
                    }
                }
            }
        }
    }
}

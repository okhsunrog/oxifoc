use crate::model::Model;
use dioxus::{dioxus_vdom::Runtime, prelude::*};
use oxifoc_host_lib::motor_editor::{Action, FIELDS, FieldValue};
use oxifoc_ui::{Button, ButtonVariant};
use std::sync::Arc;

pub(crate) use oxifoc_host_lib::motor_editor::Editor;

fn request(mut editor: Signal<Editor>, model: &Arc<Model>, running: bool, action: Action) {
    let Some(connection) = model.configuration_connection() else {
        return;
    };
    let Some(operation) = editor.write().start(connection, running, action) else {
        return;
    };
    let model = model.clone();
    // Keep requests in the studio's scope when the user switches tabs.
    Runtime::current().spawn(editor.origin_scope(), async move {
        let worker_model = model.clone();
        let result = tokio::task::spawn_blocking(move || {
            anyhow::ensure!(
                worker_model.configuration_connection() == Some(connection),
                "Connection changed"
            );
            operation.run(&worker_model.runtime.cmd_tx)
        })
        .await
        .map_err(anyhow::Error::from)
        .and_then(|result| result);
        editor
            .write()
            .finish(connection, model.configuration_connection(), result);
    });
}

pub(crate) fn sync_connection(
    mut editor: Signal<Editor>,
    model: &Arc<Model>,
    connection: Option<u64>,
    active: bool,
) {
    if editor.peek().connection() != connection {
        editor.write().sync_connection(connection);
    }
    if active && editor.peek().should_read() {
        request(editor, model, false, Action::Read);
    }
}

#[component]
pub(crate) fn Parameters(
    mut editor: Signal<Editor>,
    model: Arc<Model>,
    connection: Option<u64>,
    running: bool,
) -> Element {
    let view = editor.read().view(connection, running);
    let read_model = model.clone();
    let apply_model = model.clone();

    rsx! {
        section { class: "parameter-editor",
            div { class: "parameter-heading",
                h1 { "Motor parameters" }
                Button {
                    variant: ButtonVariant::Secondary,
                    disabled: !view.can_read,
                    onclick: move |_| request(editor, &read_model, running, Action::Read),
                    if view.changed && view.error.is_some() { "Read & discard edits" } else { "Read controller" }
                }
            }
            div { class: "parameter-panel",
                div { class: "parameter-status", role: "status",
                    strong { "{view.status}" }
                    if view.changed {
                        span { class: "parameter-dirty", "Unapplied edits" }
                    }
                }
                if let Some(error) = view.error {
                    p { class: "parameter-error", role: "alert", "{error}" }
                }
                if let Some(fields) = view.fields {
                    div { class: "parameter-groups",
                        section { class: "parameter-group",
                            h2 { "Electrical model" }
                            for index in 0..5 {
                                ParameterField { key: "{index}", index, editor,
                                    disabled: !view.editable, value: fields[index].clone() }
                            }
                        }
                        section { class: "parameter-group",
                            h2 { "Thermal rating" }
                            for index in 5..FIELDS.len() {
                                ParameterField { key: "{index}", index, editor,
                                    disabled: !view.editable, value: fields[index].clone() }
                            }
                        }
                    }
                    div { class: "parameter-actions",
                        p {
                            if running {
                                "Stop the drive before applying or saving parameters."
                            } else {
                                "Apply updates the running configuration. Save writes it to flash."
                            }
                        }
                        div { class: "parameter-buttons",
                            Button {
                                variant: ButtonVariant::Ghost,
                                disabled: !view.editable || !view.changed,
                                onclick: move |_| editor.write().revert(),
                                "Revert edits"
                            }
                            Button {
                                variant: if view.can_apply { ButtonVariant::Primary } else { ButtonVariant::Secondary },
                                disabled: !view.can_apply,
                                onclick: move |_| request(editor, &apply_model, running, Action::Apply),
                                "Apply to RAM"
                            }
                            Button {
                                variant: if view.can_save { ButtonVariant::Primary } else { ButtonVariant::Secondary },
                                disabled: !view.can_save,
                                onclick: move |_| request(editor, &model, running, Action::Save),
                                "Save to flash"
                            }
                        }
                    }
                }
            }
        }
    }
}

#[component]
fn ParameterField(
    index: usize,
    mut editor: Signal<Editor>,
    disabled: bool,
    value: FieldValue,
) -> Element {
    let field = FIELDS[index];
    let id = format!("motor-parameter-{index}");
    let detail = value.error.unwrap_or(field.hint);
    rsx! {
        div { class: if value.changed {"parameter-row changed"} else {"parameter-row"},
            div {
                label { r#for: "{id}", "{field.label}" }
                if !detail.is_empty() {
                    p { id: "{id}-detail", class: if value.error.is_some() {"error"} else {""}, "{detail}" }
                }
            }
            div { class: "parameter-input",
                input {
                    id: "{id}", r#type: "text", value: "{value.value}", disabled,
                    "aria-invalid": value.error.is_some(),
                    "aria-describedby": (!detail.is_empty()).then(|| format!("{id}-detail")),
                    spellcheck: false,
                    oninput: move |event| editor.write().set_value(index, event.value()),
                }
                span { class: "parameter-unit", "{field.unit}" }
            }
        }
    }
}

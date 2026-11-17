use crate::model::Model;
use dioxus::{dioxus_vdom::Runtime, prelude::*};
use oxifoc_core::{
    storage::MotorParamsConfig,
    types::{ConfigGroupId, ConfigValue},
};
use oxifoc_host_lib::{HostRuntime, ops::config};
use oxifoc_ui::{Button, ButtonVariant};
use std::sync::Arc;

const GROUP: ConfigGroupId = ConfigGroupId::MotorParams;

#[derive(Clone, Copy)]
struct Field {
    label: &'static str,
    unit: &'static str,
    hint: &'static str,
}

const FIELDS: [Field; 7] = [
    Field {
        label: "Phase resistance",
        unit: "Ω",
        hint: "Phase to neutral",
    },
    Field {
        label: "d-axis inductance",
        unit: "H",
        hint: "",
    },
    Field {
        label: "q-axis inductance",
        unit: "H",
        hint: "",
    },
    Field {
        label: "Flux linkage",
        unit: "Wb",
        hint: "Permanent magnets",
    },
    Field {
        label: "Pole pairs",
        unit: "",
        hint: "",
    },
    Field {
        label: "Continuous current ceiling",
        unit: "A",
        hint: "0 derives the ceiling when the loss budget is known",
    },
    Field {
        label: "Copper loss budget",
        unit: "W",
        hint: "0 leaves the budget unspecified",
    },
];

type Draft = [String; FIELDS.len()];

fn draft_from(motor: &MotorParamsConfig) -> Draft {
    [
        motor.resistance_ohm.to_string(),
        motor.inductance_d_h.to_string(),
        motor.inductance_q_h.to_string(),
        motor.flux_linkage_wb.to_string(),
        motor.pole_pairs.to_string(),
        motor.max_current_a.to_string(),
        motor.max_power_loss_w.to_string(),
    ]
}

fn decimal(text: &str, positive: bool) -> Result<f32, &'static str> {
    text.trim()
        .parse::<f32>()
        .ok()
        .filter(|value| {
            value.is_finite()
                && if positive {
                    *value > 0.0
                } else {
                    *value >= 0.0
                }
        })
        .ok_or(if positive {
            "Enter a finite number greater than zero."
        } else {
            "Enter a finite number of zero or more."
        })
}

fn pole_pairs(text: &str) -> Result<u8, &'static str> {
    text.trim()
        .parse::<u8>()
        .ok()
        .filter(|&value| value > 0)
        .ok_or("Enter a whole number from 1 to 255.")
}

fn field_error(index: usize, text: &str) -> Option<&'static str> {
    if index == 4 {
        pole_pairs(text).err()
    } else {
        decimal(text, index < 3).err()
    }
}

fn edited_motor(
    draft: &Draft,
    original: &MotorParamsConfig,
) -> Result<MotorParamsConfig, &'static str> {
    Ok(MotorParamsConfig {
        resistance_ohm: decimal(&draft[0], true)?,
        inductance_d_h: decimal(&draft[1], true)?,
        inductance_q_h: decimal(&draft[2], true)?,
        flux_linkage_wb: decimal(&draft[3], false)?,
        pole_pairs: pole_pairs(&draft[4])?,
        max_current_a: decimal(&draft[5], false)?,
        max_power_loss_w: decimal(&draft[6], false)?,
        ..original.clone()
    })
}

#[derive(Clone, Copy, PartialEq)]
enum Storage {
    Volatile,
    Saved,
    Unavailable,
}

#[derive(Clone)]
struct Readback {
    motor: MotorParamsConfig,
    present: bool,
    revision: u32,
    boot_epoch: u64,
    storage: Storage,
}

enum Operation {
    Read,
    Apply(Readback),
    Save(Readback),
}

impl Operation {
    fn label(&self) -> &'static str {
        match self {
            Self::Read => "Reading motor parameters",
            Self::Apply(_) => "Applying to RAM",
            Self::Save(_) => "Saving to flash",
        }
    }

    fn run(self, runtime: &HostRuntime) -> anyhow::Result<Readback> {
        let cmd = &runtime.cmd_tx;
        match self {
            Self::Read => {
                let snapshot = config::read_group(cmd, GROUP)?;
                anyhow::ensure!(
                    snapshot.valid,
                    "Controller returned an invalid motor configuration"
                );
                let present = snapshot.value.is_some();
                let motor = match snapshot.value.or(snapshot.default) {
                    Some(ConfigValue::MotorParams(motor)) => motor,
                    None => MotorParamsConfig::default(),
                    Some(_) => anyhow::bail!("Controller returned a different configuration group"),
                };
                Ok(Readback {
                    motor,
                    present,
                    revision: snapshot.revision,
                    boot_epoch: snapshot.boot_epoch,
                    storage: if snapshot.persisted {
                        Storage::Saved
                    } else {
                        Storage::Volatile
                    },
                })
            }
            Self::Apply(mut next) => {
                next.revision = config::apply_write(
                    cmd,
                    next.boot_epoch,
                    next.revision,
                    ConfigValue::MotorParams(next.motor.clone()),
                )?;
                next.present = true;
                if next.storage != Storage::Unavailable {
                    next.storage = Storage::Volatile;
                }
                Ok(next)
            }
            Self::Save(mut current) => {
                current.storage =
                    if config::persist_group(cmd, GROUP, current.boot_epoch, current.revision)? {
                        Storage::Saved
                    } else {
                        Storage::Unavailable
                    };
                Ok(current)
            }
        }
    }
}

#[derive(Default)]
pub(crate) struct Editor {
    connection: Option<u64>,
    readback: Option<Readback>,
    draft: Draft,
    pending: Option<&'static str>,
    error: Option<String>,
}

fn request(mut editor: Signal<Editor>, model: &Arc<Model>, operation: Operation) {
    let Some(connection) = model.configuration_connection() else {
        return;
    };
    let label = operation.label();
    {
        let state = editor.peek();
        if state.connection != Some(connection) || state.pending.is_some() {
            return;
        }
    }
    editor.write().pending = Some(label);
    let model = model.clone();
    // Keep requests in the studio's scope when the user switches tabs.
    Runtime::current().spawn(editor.origin_scope(), async move {
        let worker_model = model.clone();
        let result = tokio::task::spawn_blocking(move || {
            anyhow::ensure!(
                worker_model.configuration_connection() == Some(connection),
                "Connection changed"
            );
            operation.run(&worker_model.runtime)
        })
        .await;
        let mut state = editor.write();
        state.pending = None;
        if state.connection != Some(connection)
            || model.configuration_connection() != Some(connection)
        {
            return;
        }
        match result {
            Ok(Ok(readback)) => {
                state.draft = draft_from(&readback.motor);
                state.readback = Some(readback);
                state.error = None;
            }
            Ok(Err(error)) => state.error = Some(format!("{label} failed. {error:#}")),
            Err(error) => state.error = Some(format!("{label} failed. {error}")),
        }
    });
}

pub(crate) fn sync_connection(
    mut editor: Signal<Editor>,
    model: &Arc<Model>,
    connection: Option<u64>,
    active: bool,
) {
    if editor.peek().connection != connection {
        let mut state = editor.write();
        *state = Editor {
            connection,
            pending: state.pending,
            ..Editor::default()
        };
    }
    let state = editor.peek();
    let read = active
        && connection.is_some()
        && state.readback.is_none()
        && state.pending.is_none()
        && state.error.is_none();
    drop(state);
    if read {
        request(editor, model, Operation::Read);
    }
}

#[component]
pub(crate) fn Parameters(
    mut editor: Signal<Editor>,
    model: Arc<Model>,
    connection: Option<u64>,
    running: bool,
) -> Element {
    let state = editor.read();
    let ready = connection.is_some() && state.connection == connection;
    let readback = state.readback.as_ref().filter(|_| ready);
    let loaded_fields = readback
        .map(|readback| draft_from(&readback.motor))
        .unwrap_or_default();
    let changed = readback.is_some() && state.draft != loaded_fields;
    let next = readback.and_then(|readback| {
        edited_motor(&state.draft, &readback.motor)
            .ok()
            .filter(|motor| *motor != readback.motor)
            .map(|motor| Readback {
                motor,
                ..readback.clone()
            })
    });
    let editable = ready && state.pending.is_none() && state.error.is_none();
    let can_apply = editable && !running && next.is_some();
    let can_save = editable
        && !running
        && !changed
        && readback
            .is_some_and(|readback| readback.present && readback.storage == Storage::Volatile);
    let status = if !ready {
        "Waiting for controller".to_string()
    } else if let Some(pending) = state.pending {
        format!("{pending}…")
    } else if state.error.is_some() {
        "Read the controller to continue".into()
    } else if let Some(readback) = readback {
        let description = match (readback.present, readback.storage) {
            (false, _) => "No motor model on controller",
            (_, Storage::Saved) => "Saved to flash",
            (_, Storage::Volatile) => "Active in RAM",
            (_, Storage::Unavailable) => "Active in RAM · Flash unavailable",
        };
        format!("Revision {} · {description}", readback.revision)
    } else {
        "Reading motor parameters…".into()
    };
    let read_model = model.clone();
    let apply_model = model.clone();
    let save = readback.cloned();

    rsx! {
        section { class: "parameter-editor",
            div { class: "parameter-heading",
                h1 { "Motor parameters" }
                Button {
                    variant: ButtonVariant::Secondary,
                    disabled: !ready || state.pending.is_some() || (changed && state.error.is_none()),
                    onclick: move |_| request(editor, &read_model, Operation::Read),
                    if changed && state.error.is_some() { "Read & discard edits" } else { "Read controller" }
                }
            }
            div { class: "parameter-panel",
                div { class: "parameter-status", role: "status",
                    strong { "{status}" }
                    if changed {
                        span { class: "parameter-dirty", "Unapplied edits" }
                    }
                }
                if ready {
                    if let Some(error) = &state.error {
                        p { class: "parameter-error", role: "alert", "{error}" }
                    }
                }
                if readback.is_some() {
                    div { class: "parameter-groups",
                        section { class: "parameter-group",
                            h2 { "Electrical model" }
                            for index in 0..5 {
                                ParameterField {
                                    key: "{index}", index, editor, disabled: !editable,
                                    changed: state.draft[index] != loaded_fields[index],
                                }
                            }
                        }
                        section { class: "parameter-group",
                            h2 { "Thermal rating" }
                            for index in 5..FIELDS.len() {
                                ParameterField {
                                    key: "{index}", index, editor, disabled: !editable,
                                    changed: state.draft[index] != loaded_fields[index],
                                }
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
                                disabled: !editable || !changed,
                                onclick: move |_| editor.write().draft = loaded_fields.clone(),
                                "Revert edits"
                            }
                            Button {
                                variant: if can_apply { ButtonVariant::Primary } else { ButtonVariant::Secondary },
                                disabled: !can_apply,
                                onclick: move |_| {
                                    if let Some(next) = next.clone() {
                                        request(editor, &apply_model, Operation::Apply(next));
                                    }
                                },
                                "Apply to RAM"
                            }
                            Button {
                                variant: if can_save { ButtonVariant::Primary } else { ButtonVariant::Secondary },
                                disabled: !can_save,
                                onclick: move |_| {
                                    if let Some(current) = save.clone() {
                                        request(editor, &model, Operation::Save(current));
                                    }
                                },
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
    changed: bool,
) -> Element {
    let field = FIELDS[index];
    let value = editor.read().draft[index].clone();
    let error = field_error(index, &value);
    let id = format!("motor-parameter-{index}");
    let detail = error.unwrap_or(field.hint);
    rsx! {
        div { class: if changed {"parameter-row changed"} else {"parameter-row"},
            div {
                label { r#for: "{id}", "{field.label}" }
                if !detail.is_empty() {
                    p { id: "{id}-detail", class: if error.is_some() {"error"} else {""}, "{detail}" }
                }
            }
            div { class: "parameter-input",
                input {
                    id: "{id}", r#type: "text", value: "{value}", disabled,
                    "aria-invalid": error.is_some(),
                    "aria-describedby": (!detail.is_empty()).then(|| format!("{id}-detail")),
                    spellcheck: false,
                    oninput: move |event| editor.write().draft[index] = event.value(),
                }
                span { class: "parameter-unit", "{field.unit}" }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edits_preserve_fundamental_inductance_calibration() {
        let original = MotorParamsConfig {
            resistance_ohm: 0.5,
            inductance_d_h: 0.00047,
            inductance_q_h: 0.00049,
            pole_pairs: 7,
            ld_fundamental_h: 0.00081,
            lq_fundamental_h: 0.00121,
            ..Default::default()
        };
        let mut draft = draft_from(&original);
        draft[0] = "6e-1".into();
        let edited = edited_motor(&draft, &original).unwrap();
        assert_eq!(
            edited,
            MotorParamsConfig {
                resistance_ohm: 0.6,
                ..original
            }
        );
    }
}

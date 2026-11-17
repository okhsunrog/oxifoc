//! Framework-independent motor parameter form and revision-aware operations.

use crate::{CommandSender, ops::config};
use oxifoc_core::{
    storage::MotorParamsConfig,
    types::{ConfigGroupId, ConfigValue},
};

const GROUP: ConfigGroupId = ConfigGroupId::MotorParams;

#[derive(Clone, Copy)]
pub struct Field {
    pub label: &'static str,
    pub unit: &'static str,
    pub hint: &'static str,
}

pub const FIELDS: [Field; 7] = [
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

pub type ReadResult = anyhow::Result<Readback>;

#[derive(Clone)]
pub struct Readback {
    motor: MotorParamsConfig,
    present: bool,
    revision: u32,
    boot_epoch: u64,
    storage: Storage,
}

pub enum Operation {
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

    pub fn run(self, cmd: &CommandSender) -> anyhow::Result<Readback> {
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

#[derive(Clone, Copy)]
pub enum Action {
    Read,
    Apply,
    Save,
}

#[derive(Clone, Default, PartialEq)]
pub struct FieldValue {
    pub value: String,
    pub changed: bool,
    pub error: Option<&'static str>,
}

#[derive(PartialEq)]
pub struct View {
    pub fields: Option<[FieldValue; FIELDS.len()]>,
    pub status: String,
    pub error: Option<String>,
    pub changed: bool,
    pub editable: bool,
    pub can_read: bool,
    pub can_apply: bool,
    pub can_save: bool,
}

#[derive(Default)]
pub struct Editor {
    connection: Option<u64>,
    readback: Option<Readback>,
    draft: Draft,
    pending: Option<&'static str>,
    error: Option<String>,
}

impl Editor {
    pub fn connection(&self) -> Option<u64> {
        self.connection
    }

    pub fn sync_connection(&mut self, connection: Option<u64>) {
        if self.connection != connection {
            *self = Self {
                connection,
                pending: self.pending,
                ..Self::default()
            };
        }
    }

    pub fn should_read(&self) -> bool {
        self.connection.is_some()
            && self.readback.is_none()
            && self.pending.is_none()
            && self.error.is_none()
    }

    pub fn set_value(&mut self, index: usize, text: String) {
        if self.editable()
            && let Some(field) = self.draft.get_mut(index)
        {
            *field = text;
        }
    }

    pub fn revert(&mut self) {
        if self.editable()
            && let Some(readback) = &self.readback
        {
            self.draft = draft_from(&readback.motor);
        }
    }

    fn editable(&self) -> bool {
        self.connection.is_some() && self.pending.is_none() && self.error.is_none()
    }

    fn changed(&self) -> bool {
        self.readback
            .as_ref()
            .is_some_and(|value| self.draft != draft_from(&value.motor))
    }

    fn next_motor(&self) -> Option<Readback> {
        let readback = self.readback.as_ref()?;
        let motor = edited_motor(&self.draft, &readback.motor).ok()?;
        (motor != readback.motor).then(|| Readback {
            motor,
            ..readback.clone()
        })
    }

    fn can_read(&self) -> bool {
        self.pending.is_none() && (!self.changed() || self.error.is_some())
    }

    fn can_save(&self) -> bool {
        !self.changed()
            && self
                .readback
                .as_ref()
                .is_some_and(|value| value.present && value.storage == Storage::Volatile)
    }

    pub fn start(&mut self, connection: u64, running: bool, action: Action) -> Option<Operation> {
        if self.connection != Some(connection) || self.pending.is_some() {
            return None;
        }
        let operation = match action {
            Action::Read if self.can_read() => Operation::Read,
            Action::Apply if self.editable() && !running => Operation::Apply(self.next_motor()?),
            Action::Save if self.editable() && !running && self.can_save() => {
                Operation::Save(self.readback.clone()?)
            }
            _ => return None,
        };
        self.pending = Some(operation.label());
        Some(operation)
    }

    pub fn finish(
        &mut self,
        generation: u64,
        connection: Option<u64>,
        result: anyhow::Result<Readback>,
    ) {
        let Some(label) = self.pending.take() else {
            return;
        };
        if self.connection != Some(generation) || connection != Some(generation) {
            return;
        }
        match result {
            Ok(readback) => {
                self.draft = draft_from(&readback.motor);
                self.readback = Some(readback);
                self.error = None;
            }
            Err(error) => self.error = Some(format!("{label} failed. {error:#}")),
        }
    }

    pub fn view(&self, connection: Option<u64>, running: bool) -> View {
        let ready = connection.is_some() && self.connection == connection;
        let readback = self.readback.as_ref().filter(|_| ready);
        let changed = ready && self.changed();
        let editable = ready && self.editable();
        let status = if !ready {
            "Waiting for controller".to_string()
        } else if let Some(pending) = self.pending {
            format!("{pending}…")
        } else if self.error.is_some() {
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
        View {
            fields: readback.map(|readback| {
                let loaded = draft_from(&readback.motor);
                std::array::from_fn(|index| FieldValue {
                    value: self.draft[index].clone(),
                    changed: self.draft[index] != loaded[index],
                    error: field_error(index, &self.draft[index]),
                })
            }),
            status,
            error: self.error.clone().filter(|_| ready),
            changed,
            editable,
            can_read: ready && self.can_read(),
            can_apply: editable && !running && self.next_motor().is_some(),
            can_save: editable && !running && self.can_save(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reconnect_discards_the_previous_read_reply() {
        let mut editor = Editor::default();
        editor.sync_connection(Some(1));
        assert!(editor.start(1, false, Action::Read).is_some());

        editor.sync_connection(Some(2));
        assert!(editor.start(2, false, Action::Read).is_none());
        editor.finish(
            1,
            Some(2),
            Ok(Readback {
                motor: MotorParamsConfig::default(),
                present: false,
                revision: 0,
                boot_epoch: 1,
                storage: Storage::Volatile,
            }),
        );

        assert!(editor.view(Some(2), false).fields.is_none());
        assert!(editor.should_read());
    }

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

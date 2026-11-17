use crate::{App, MotorEditAction, MotorEditorUi, MotorEditorView, MotorField, SharedRuntime};
use oxifoc_host_lib::{
    CommandSender,
    motor_editor::{Action, Editor, FIELDS, ReadResult, View},
};
use slint::{ComponentHandle, Model, ModelRc, Timer, TimerMode, VecModel};
use std::{
    cell::RefCell,
    rc::Rc,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
        mpsc,
    },
    thread,
    time::Duration,
};

type Reply = (u64, ReadResult);

struct Session {
    editor: RefCell<Editor>,
    runtime: SharedRuntime,
    generation: Arc<AtomicU64>,
    responses: mpsc::Receiver<Reply>,
    reply: mpsc::Sender<Reply>,
    fields: Rc<VecModel<MotorField>>,
    last_view: RefCell<Option<View>>,
}

impl Session {
    fn connection(&self) -> Option<(u64, CommandSender)> {
        let slot = self.runtime.lock().unwrap();
        let runtime = slot.as_ref()?;
        runtime.connected.load(Ordering::Relaxed).then(|| {
            (
                self.generation.load(Ordering::Relaxed),
                runtime.cmd_tx.clone(),
            )
        })
    }

    fn start(&self, app: &App, action: Action) {
        let Some((generation, cmd)) = self.connection() else {
            return;
        };
        let Some(operation) =
            self.editor
                .borrow_mut()
                .start(generation, app.get_motor_running(), action)
        else {
            return;
        };
        let reply = self.reply.clone();
        thread::spawn(move || {
            let _ = reply.send((generation, operation.run(&cmd)));
        });
    }

    fn refresh(&self, app: &App) {
        let connection = self.connection().map(|(generation, _)| generation);
        let active =
            app.get_page() == "main" && app.get_active_tab() == 1 && app.get_config_group() == 0;
        let mut editor = self.editor.borrow_mut();
        editor.sync_connection(connection);
        for (generation, result) in self.responses.try_iter() {
            editor.finish(generation, connection, result);
        }
        let read = active && editor.should_read();
        drop(editor);
        if read {
            self.start(app, Action::Read);
        }
        if active {
            self.publish(app, connection);
        }
    }

    fn publish(&self, app: &App, connection: Option<u64>) {
        let view = self
            .editor
            .borrow()
            .view(connection, app.get_motor_running());
        let mut last = self.last_view.borrow_mut();
        if last.as_ref() == Some(&view) {
            return;
        }
        if let Some(fields) = &view.fields {
            let previous = last.as_ref().and_then(|view| view.fields.as_ref());
            for (index, (field, value)) in FIELDS.iter().zip(fields).enumerate() {
                if previous.is_some_and(|previous| previous[index] == *value) {
                    continue;
                }
                self.fields.set_row_data(
                    index,
                    MotorField {
                        label: field.label.into(),
                        unit: field.unit.into(),
                        hint: field.hint.into(),
                        value: value.value.as_str().into(),
                        error: value.error.unwrap_or_default().into(),
                        dirty: value.changed,
                    },
                );
            }
        }
        app.global::<MotorEditorUi>().set_state(MotorEditorView {
            status: view.status.as_str().into(),
            error: view.error.as_deref().unwrap_or_default().into(),
            loaded: view.fields.is_some(),
            dirty: view.changed,
            editable: view.editable,
            can_read: view.can_read,
            can_apply: view.can_apply,
            can_save: view.can_save,
        });
        *last = Some(view);
    }
}

pub(super) fn bind(app: &App, runtime: &SharedRuntime, generation: Arc<AtomicU64>) -> Timer {
    let (reply, responses) = mpsc::channel();
    let fields = Rc::new(VecModel::from(vec![MotorField::default(); FIELDS.len()]));
    let ui = app.global::<MotorEditorUi>();
    ui.set_fields(ModelRc::from(fields.clone()));
    let session = Rc::new(Session {
        editor: RefCell::new(Editor::default()),
        runtime: runtime.clone(),
        generation,
        responses,
        reply,
        fields,
        last_view: RefCell::new(None),
    });
    let weak = app.as_weak();
    let state = session.clone();
    ui.on_act(move |action| {
        let Some(app) = weak.upgrade() else { return };
        match action {
            MotorEditAction::Read => state.start(&app, Action::Read),
            MotorEditAction::Apply => state.start(&app, Action::Apply),
            MotorEditAction::Save => state.start(&app, Action::Save),
            MotorEditAction::Revert => state.editor.borrow_mut().revert(),
        }
        state.refresh(&app);
    });
    let weak = app.as_weak();
    let state = session.clone();
    ui.on_edit(move |index, text| {
        let Some(app) = weak.upgrade() else { return };
        state
            .editor
            .borrow_mut()
            .set_value(index as usize, text.to_string());
        state.refresh(&app);
    });
    let weak = app.as_weak();
    let timer = Timer::default();
    timer.start(TimerMode::Repeated, Duration::from_millis(80), move || {
        if let Some(app) = weak.upgrade() {
            session.refresh(&app);
        }
    });
    timer
}

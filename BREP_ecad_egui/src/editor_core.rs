//! Shared state and history for the standalone part editors.
use brep_ecad_core::History;
use egui::Vec2;
use crate::{Change, HistoryKey};

pub(crate) struct EditorCore<Doc, Selection, Drag> {
    pub history: History<Doc>,
    pub pending_change: Option<Change>,
    pub selected: Option<Selection>,
    pub drag: Option<Drag>,
    pub pan: Vec2,
    pub zoom: f32,
}

impl<Doc: PartialEq, Selection, Drag> EditorCore<Doc, Selection, Drag> {
    pub fn new(zoom: f32) -> Self {
        Self {
            history: History::default(), pending_change: None,
            selected: None, drag: None, pan: Vec2::ZERO, zoom,
        }
    }
    pub fn record(&mut self, before: Doc, after: &Doc, coalesce: Option<&str>) {
        if before != *after {
            self.history.record(before, after);
            self.note_change(coalesce.map(str::to_owned));
        }
    }
    pub fn note_change(&mut self, coalesce: Option<String>) {
        Change::record(&mut self.pending_change, coalesce);
    }
    pub fn take_change(&mut self) -> Option<Change> {
        self.pending_change.take()
    }
    pub fn undo(&mut self, doc: &mut Doc) {
        if self.history.can_undo() {
            self.history.undo(doc);
            self.selected = None;
            self.note_change(None);
        }
    }
    pub fn redo(&mut self, doc: &mut Doc) {
        if self.history.can_redo() {
            self.history.redo(doc);
            self.selected = None;
            self.note_change(None);
        }
    }
    pub fn step_history(&mut self, doc: &mut Doc, key: HistoryKey) {
        if self.drag.is_none() {
            match key {
                HistoryKey::Undo => self.undo(doc),
                HistoryKey::Redo => self.redo(doc),
            }
        }
    }
    pub fn can_step_history(&self, key: HistoryKey) -> bool {
        self.drag.is_none() && match key {
            HistoryKey::Undo => self.history.can_undo(),
            HistoryKey::Redo => self.history.can_redo(),
        }
    }
}

pub(crate) struct WindowSpec {
    pub title: &'static str,
    pub id: &'static str,
    pub default_size: Vec2,
    pub min_size: Vec2,
    pub inspector_width: f32,
    pub apply: &'static str,
    pub hint: &'static str,
    // The symbol canvas measures the remaining height after its Inspector.
    pub measure_canvas_after_inspector: bool,
}

pub(crate) trait WindowEditor {
    fn toolbar(&mut self, ui: &mut egui::Ui);
    fn properties(&mut self, ui: &mut egui::Ui);
    fn canvas(&mut self, ui: &mut egui::Ui, size: Vec2);
    fn validate(&self) -> Result<(), String>;
    fn error(&mut self) -> &mut String;
}

/// Shared window, Inspector/canvas layout, validation and Apply/Cancel handling.
pub(crate) fn editor_window(
    editor: &mut impl WindowEditor,
    ctx: &egui::Context,
    spec: WindowSpec,
) -> Option<bool> {
    let mut open = true;
    let mut action = None;
    egui::Window::new(spec.title)
        .id(egui::Id::new(spec.id))
        .default_size(spec.default_size)
        .min_size(spec.min_size)
        .open(&mut open)
        .collapsible(false)
        .show(ctx, |ui| {
            editor.toolbar(ui);
            ui.separator();
            ui.horizontal_top(|ui| {
                let height = (ui.available_height() - 55.).max(300.);
                ui.allocate_ui_with_layout(
                    Vec2::new(spec.inspector_width, height),
                    egui::Layout::top_down(egui::Align::Min),
                    |ui| editor.properties(ui),
                );
                let height = if spec.measure_canvas_after_inspector {
                    (ui.available_height() - 55.).max(300.)
                } else { height };
                let size = Vec2::new(ui.available_width().max(300.), height);
                editor.canvas(ui, size);
            });
            ui.separator();
            if !editor.error().is_empty() {
                ui.colored_label(ui.visuals().error_fg_color, &*editor.error());
            }
            ui.horizontal(|ui| {
                if ui.button(spec.apply).clicked() {
                    match editor.validate() {
                        Ok(()) => action = Some(true),
                        Err(error) => *editor.error() = error,
                    }
                }
                if ui.button("Cancel").clicked() { action = Some(false); }
                ui.small(spec.hint);
            });
        });
    if !open { action = Some(false); }
    action
}

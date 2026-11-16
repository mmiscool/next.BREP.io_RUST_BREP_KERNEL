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
    /// Scale the view by `factor` holding still the point `anchor` points from
    /// the canvas centre: the pointer's offset for the wheel, zero (the centre)
    /// for the Zoom buttons.
    pub fn zoom_about(&mut self, factor: f32, anchor: Vec2) {
        crate::zoom_at(&mut self.zoom, &mut self.pan, factor, anchor);
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
    pub fn can_step_history(&self, key: HistoryKey) -> bool {
        self.drag.is_none() && match key {
            HistoryKey::Undo => self.history.can_undo(),
            HistoryKey::Redo => self.history.can_redo(),
        }
    }
}




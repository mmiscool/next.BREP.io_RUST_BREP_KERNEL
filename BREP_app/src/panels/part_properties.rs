//! The PART PROPERTIES dialog — the open document's own BOM attributes, on the
//! toolbar.
//!
//! # What it edits
//!
//! `partAttributes`, the top-level document key described in
//! `brep_render::engine_state`'s BOM-attributes block. A document IS a part: a
//! part file's Part Number, Material and Mass live on it and travel with it,
//! which is exactly why an assembly's BOM can read them off a parts-library
//! entry's EMBEDDED document. This dialog is the other door onto the same
//! record — the one for the part you have OPEN, which until now had no editor
//! at all. (An assembly document has the record too, and for the same reason: a
//! rigidly nested assembly is ONE part, one BOM row, in its parent.)
//!
//! OCCURRENCE attributes are deliberately absent. They describe one PLACEMENT,
//! so they belong to the assembly that places the part, not to the part — the
//! BOM panel's occurrence columns are their editor.
//!
//! # Every field, not the starred ones
//!
//! The BOM column configuration decides what the parts LIST is wide enough to
//! show. It says nothing about what a part HAS. So the dialog draws every part
//! field — the configured ones in the user's own order, then the built-ins the
//! configuration leaves out, then anything already stored that neither names
//! (see [`bom_columns::part_fields`]). A field a user hid from the table is
//! still a field of the part, and a dialog that hid it too would leave a stored
//! value with no way to see or clear it.
//!
//! # The editors are the BOM's own
//!
//! Each row draws [`column_tree::value_editor`] for the field's `CellKind` —
//! the very function the BOM table's cells draw. One attribute cannot have two
//! editing behaviours when it has one store: text commits on focus-loss (an
//! attribute write re-runs the history), a choice always offers the blank
//! entry so it can be cleared, and clearing writes `""`, which the engine reads
//! as REMOVE.

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree;
use crate::panels::bom_columns;
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The dialog's state: whether it is open, and this frame's widget rects.
#[derive(Default)]
pub struct PartPropertiesPanel {
    /// Whether the floating window is shown. Toggled by the toolbar's
    /// Properties button and by the window's own `×`; public so the toolbar can
    /// bind it, exactly as the Settings window's flag is.
    pub open: bool,
    /// Per-frame egui widget screen rects (keyed `field:<Field>`), published for
    /// the automation layer. Rebuilt every frame.
    hits: HashMap<String, egui::Rect>,
    configuration: crate::panels::bom_configuration::BomConfiguration,
    plm_target: Option<(std::rc::Rc<crate::plm::client::PlmClient>,String)>,
}

impl PartPropertiesPanel {
    pub fn configure_plm(&mut self,client:Option<std::rc::Rc<crate::plm::client::PlmClient>>,document:Option<&str>){
        self.plm_target=client.zip(document.filter(|key|crate::plm::bom::revision_of_document(key).is_some()).map(str::to_string));
        if self.open {if let Some((client,key))=&self.plm_target{self.configuration.ensure(client,vec![key.clone()]);}}
    }
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the floating window (if open) at ctx level, after the panels, so it
    /// floats over the shell. `title` is the active document's tab title — the
    /// part whose properties these are, named in the window so a user with
    /// several tabs open can never edit the wrong one — and `document` its
    /// process-unique id, which salts the per-field widget ids so an
    /// in-progress edit belongs to the document it was typed into and cannot
    /// reappear over another tab's value.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        title: &str,
        document: u64,
    ) {
        if !self.open {
            return;
        }
        // `egui::Window::open` needs its own `&mut bool`; borrow a copy so the
        // draw closure can still take `&mut self`, then fold the close back in.
        let mut open = true;
        egui::Window::new("Part Properties")
            .open(&mut open)
            .movable(true)
            .resizable(true)
            // A bounded default size + the filling ScrollArea in `body`: without
            // a filling child egui hugs the window to its content and the user
            // cannot drag it larger (the Settings window's rule).
            .default_size([340.0, 420.0])
            // Offset from the Settings window's own default rest position, so
            // opening both does not stack them pixel-for-pixel.
            .default_pos([660.0, 96.0])
            .show(ctx, |ui| self.body(ui, state, title, document));
        self.open = open;

        if crate::automation::registry::enabled() {
            crate::automation::registry::publish(
                "__brepPartPropertiesHit",
                "part properties window widget rects (field:*)",
                &self.hits_json(),
            );
        }
    }

    /// The window body: the part's name, then one labelled row per field.
    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState, title: &str, document: u64) {
        self.hits.clear();
        if let Some((client,key))=&self.plm_target {
            ui.label(egui::RichText::new(title).strong());
            self.configuration.attributes_ui(ui,client,key,&mut self.hits);
            return;
        }
        ui.label(egui::RichText::new(title).strong());
        ui.label(
            egui::RichText::new(
                "Stored on this document and carried into any assembly's BOM.",
            )
            .weak()
            .small(),
        );
        ui.separator();

        // Read the record LIVE each frame rather than caching it: an undo, the
        // automation layer, or a switch to another document tab all change it
        // under the dialog, and a cached copy would show — and then write back —
        // the previous document's values.
        let stored = state.document_part_attributes();
        // Through `effective_text`, exactly as the BOM table reads its columns:
        // the stored setting is EMPTY until the user configures it, and the
        // shipped default is what the table is ordered by. Reading the raw
        // string here would order a never-configured document's fields by the
        // catalogue instead, so the dialog and the table would disagree for
        // every user who never opened the settings.
        let fields = bom_columns::part_fields(
            &bom_columns::effective_text(&state.settings.bom_columns),
            &stored,
        );

        // Commit AFTER the loop: `set_document_part_attribute` re-runs the
        // history, and re-entering the engine mid-draw while `stored` is
        // borrowed from it is exactly the kind of half-applied frame that makes
        // a text buffer fight its own value. A LIST, not one edit: a frame that
        // blurs a text field while a drag value moves carries two, and dropping
        // either is a lost keystroke.
        let mut edits: Vec<(String, Value)> = Vec::new();
        egui::ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Grid::new("part-properties-grid")
                    .num_columns(2)
                    .spacing([8.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        for field in &fields {
                            ui.label(field.label());
                            let size = egui::vec2(
                                ui.available_width().max(80.0),
                                ui.spacing().interact_size.y,
                            );
                            let (committed, rect) = column_tree::value_editor(
                                ui,
                                egui::Id::new(("part-properties", document, &field.field)),
                                &field.kind(),
                                stored.get(&field.field),
                                size,
                            );
                            self.hits.insert(format!("field:{}", field.field), rect);
                            if let Some(value) = committed {
                                edits.push((field.field.clone(), value));
                            }
                            ui.end_row();
                        }
                    });
            });

        for (key, value) in edits {
            // A refusal here is a programming error (an empty key), not
            // something the user can produce from a drawn row, so it has no
            // banner: the field list never yields one.
            let _ = state.set_document_part_attribute(&key, value);
        }
    }

    /// The published widget hit-rects (egui points) for the automation layer.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[HitKeyDoc {
    panel: "partproperties",
    prefix: "field:",
    meaning: "a part-attribute editor in the Part Properties window (field:<Field>)",
    command: Some("part_attribute_set"),
}];


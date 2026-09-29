//! Toolbar — a top `Panel` strip of primary actions above the viewport:
//! Undo / Redo, a Wireframe toggle, and Zoom-to-fit, plus the File-actions
//! SEAM (owned by the concurrent file panel). Standard views live on the
//! ViewCube navigation gizmo, not here.
//!
//! Follows the panel pattern (a small state struct + a `show(&mut self, ui,
//! state, store)` the shell calls once), but unlike the left-column sections it
//! creates its OWN top panel — so the shell just calls `self.toolbar.show(…)`
//! FIRST in `App::ui` (before the left panel + viewport) to reserve the strip.
//!
//! The MODEL is engine-owned: the toolbar only TRIGGERS engine methods
//! (`state.undo()` / `state.redo()`) and drives the wireframe through the
//! existing settings-apply path (`apply_settings_json` → bumps generation +
//! dirty). It owns only the per-frame `hits` map (widget screen rects) the
//! headed verifier reads to drive real clicks, exactly like the history panel.
//! Zoom-to-fit is the one action it SURFACES rather than runs, because what
//! "fit" means depends on what the central tile is drawing — the 3D scene or an
//! OPEN SHEET's paper — and only the shell can see both (`BrepApp::zoom_to_fit`).
//!
//! The strip ends in the WORKBENCH BUTTON ROW: the active workbench's own
//! declared actions plus the SHARED ones every workbench carries, and the ONE
//! place a mode's tools live. A declared button may be conditional on document
//! state and may read as pressed from it (see
//! [`crate::workbench::WorkbenchButton`]), which is how the sheet viewport's
//! tools — Back to 3D and the five dimension constructions — appear here while
//! a sheet is open instead of on a floating toolbar over the paper, and how the
//! sketcher's ten draw tools and Auto-constrain appear here while a sketch is
//! being edited instead of on a strip of their own.
//!
//! All controls use the same wrapping layout as the workbench action strip.
//! Narrowing the window adds rows, preserving every offered button's id, icon,
//! pressed/disabled state, menu and hit rect. No workbench has its own overflow
//! policy or hardcoded width floor. The viewport uses the space below the rows.
//!
//! The File buttons don't touch storage here: a click returns a [`FileAction`]
//! from [`ToolbarPanel::show`] and the shell hands it to the reusable
//! [`crate::panels::file::FileDialog`].
//!
//! Buttons draw their artwork from `assets/glyphs/*.svg`, with the text label in
//! the hover tooltip. The COLOUR icons are single SVGs from the icon catalog
//! (see [`crate::icons`]), painted by [`toolbar_button`] whenever a glyph
//! resolves to colour artwork — they replaced the private-use glyph STACKS this
//! file used to assemble by hand, one layer per colour. Neither depends on an OS
//! font or a bitmap asset. All styling/sizing goes through the shared
//! [`crate::panels::toolbar_button`] helpers (the ONE place toolbar-button style
//! lives), so a change lands globally. Glyph per button:
//!   New        U+E010 (doc)                     colour
//!   Open       U+E011 (folder)                  colour
//!   Save       U+E012 (disk)                    colour
//!   Save As    U+E013 (disk + badge)            colour
//!   Import     U+E014 (tray + down arrow)       colour
//!   Export     U+E015 (tray + up arrow)         colour
//!   Undo       U+E016 (curved arrow)            colour
//!   Redo       U+E017 (curved arrow)            colour
//!   Projctn    U+E018 (camera)                  colour
//!   Submit Bug U+1F41E (bug)                    colour
//!   Wireframe  U+1F578 (single glyph)
//!   Faces      U+E028  (shaded cube)
//!   Edges      U+E029  (edge cube)
//!   Vertices   U+E02A  (corner points)
//!   Fit        U+26F6  (single glyph)
//!   Settings   U+2699  (single glyph)
//!   Help       U+2753  (circled question mark)
//!   Info       U+2139  (circled i)
//!   Properties U+E066  (part tag)
//!
//! The Properties button here is the DOCUMENT's: it opens the open part's own
//! BOM attribute record (see [`crate::panels::part_properties`]), which is a
//! property of the whole document and so has nothing to be selected first.
//! ENTITY inspection is a different thing and is not here: it is opened from the
//! selection-driven CONTEXT bar (see [`crate::panels::context_bar`]), which spawns
//! a pinned per-entity window (see [`crate::panels::info_windows`]).

use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::file::FileAction;
use crate::panels::toolbar_button;
use crate::store::{ModelStore, SETTINGS_KEY};
use crate::workbench;
use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// Toolbar actions returned to the shell for dispatch.
#[derive(Default)]
pub struct ToolbarOutcome {
    /// A File button click (New / Open / Save / …), dispatched to the file dialog.
    pub file: Option<FileAction>,
    /// A workbench toolbar button click, surfaced by its `WorkbenchButton::id`.
    pub workbench_button: Option<&'static str>,
    /// The "Submit Bug" button was clicked this frame — the shell begins the
    /// screenshot-capture + report flow (see [`crate::panels::bug_report`]).
    pub bug_report: bool,
    /// Zoom-to-fit was clicked. It is SURFACED rather than run here because what
    /// "fit" means depends on what the viewport is drawing — the 3D scene, or
    /// the OPEN SHEET's paper — and the sheet's transform lives in the viewport,
    /// not the engine. `BrepApp::zoom_to_fit` owns that branch, and the
    /// `zoom_to_fit` command calls the very same method.
    pub zoom_to_fit: bool,
    /// A row of the PLM review inbox was opened: the review or change order it
    /// names (`panels::plm_review`, S4). The shell hands it to the PLM pane.
    pub plm_review: Vec<crate::panels::plm_review::PlmReviewEvent>,
}

/// The toolbar's own state: the per-frame map of egui widget screen rects,
/// published to JS for the headed verifier to drive real clicks. Rebuilt each
/// frame (there is no DOM — egui is drawn on the canvas).
#[derive(Default)]
pub struct ToolbarPanel {
    hits: HashMap<String, egui::Rect>,
    /// The PLM review inbox badge beside the connection indicator (S4).
    inbox: crate::panels::plm_review::InboxBadge,
}

impl ToolbarPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the toolbar as a top panel of primary actions. Called FIRST in the
    /// shell's `App::ui` so the strip reserves the top before the left panel and
    /// the central viewport. Rebuilds `hits` each frame as it draws. Returns the
    /// [`FileAction`] a clicked File button requests (the shell dispatches it to
    /// the file dialog), or `None`.
    ///
    /// `settings_open` is the shell-owned open flag of the floating Settings window
    /// (see [`crate::panels::settings`]): the gear button reflects it (highlighted
    /// while open) and toggles it on click. `info_open` is the same arrangement
    /// for the Info window (see [`crate::panels::info`]).
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        ecad: Option<&crate::workbench::ecad::Editors>,
        store: &dyn ModelStore,
        settings_open: &mut bool,
        properties_open: &mut bool,
        info_open: &mut bool,
    ) -> ToolbarOutcome {
        self.hits.clear();
        let mut outcome = ToolbarOutcome::default();
        egui::containers::panel::Panel::top("brep-toolbar")
            .resizable(false)
            .show(ui, |ui| {
                ui.add_space(3.0);
                toolbar_button::wrapped(ui, |ui| {
                    // Workbench selector FIRST (top-left). It is a UI FILTER, not a
                    // mode switch — it only trims the feature-creation palette /
                    // offers. Its extra BUTTONS render LAST (after the normal icons).
                    self.workbench_selector(ui, state, store);
                    ui.separator();
                    outcome.file = self.file_actions(ui);
                    ui.separator();
                    self.edit_actions(ui, state);
                    ui.separator();
                    outcome.zoom_to_fit = self.view_actions(ui, state, store);
                    ui.separator();
                    self.properties_action(ui, properties_open);
                    self.settings_action(ui, settings_open);
                    self.help_action(ui);
                    self.info_action(ui, info_open);
                    outcome.bug_report = self.bug_action(ui);
                    outcome.plm_review = self.plm_indicator(ui, store, settings_open);
                    // The active workbench's extra buttons go at the END of the
                    // toolbar, after the standard icons (a leading separator draws
                    // only when the active workbench actually declares buttons).
                    let current = state.settings.workbench.clone();
                    let buttons = workbench::ButtonState { engine: state, ecad };
                    outcome.workbench_button = self.workbench_buttons_row(ui, &current, &buttons);
                });
                ui.add_space(3.0);
            });
        outcome
    }

    /// The WORKBENCH dropdown: iterates the per-file registry
    /// ([`workbench::WORKBENCHES`]) so labels/order are never hardcoded, shows the
    /// current selection (`state.settings.workbench`), and on change writes the id
    /// through the SAME apply+save path the wireframe/projection toggles use here
    /// (`apply_settings_json` + `store.write`), so it persists across a reload.
    /// Publishes the header + per-item hit-rects for the headed verifier.
    fn workbench_selector(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) {
        let current = state.settings.workbench.clone();
        let options: Vec<(&str, &str, &str)> =
            workbench::WORKBENCHES.iter().map(|w| (w.label, w.id, w.glyph)).collect();
        let result =
            toolbar_button::select(ui, "workbench", workbench::resolve(&current).id, &options);
        self.hits.insert("workbench".into(), result.header_rect);
        for (id, rect) in &result.item_rects {
            self.hits.insert(format!("workbench:item:{id}"), *rect);
        }
        if let Some(next) = result.changed {
            let json = serde_json::json!({ "workbench": next }).to_string();
            let _ = state.apply_settings_json(&json);
            // Persist through the same seam wireframe/projection use so the choice
            // survives a reload and the settings blob agrees.
            let _ = store.write(SETTINGS_KEY, &state.settings_json());
        }
    }

    /// Render the ACTIVE workbench's extra toolbar buttons generically — the row
    /// every mode's own actions live in, sheets included.
    ///
    /// The list is [`workbench::offered_buttons`], so a button declared
    /// CONDITIONAL on document state is simply absent while its condition is
    /// false: Drawing's eleven sheet buttons are in the row exactly while a sheet is
    /// open, and the eleven SHARED sketch tools exactly while a sketch is being
    /// edited. The row itself decides nothing about WHICH buttons exist — it
    /// renders what the registry offers for this engine state.
    ///
    fn workbench_buttons_row(
        &mut self,
        ui: &mut egui::Ui,
        current: &str,
        state: &workbench::ButtonState,
    ) -> Option<&'static str> {
        let buttons = workbench::offered_buttons(current, state);
        // A leading separator only when there is at least one button, so an
        // empty-button workbench (e.g. Modeling) leaves no dangling separator at
        // the toolbar's end. Measured AFTER it, because it is drawn.
        if buttons.is_empty() {
            return None;
        }
        ui.separator();
        self.draw_workbench_buttons(ui, &buttons, state)
    }

    /// The popup a menu opener opens: one row per button, with its glyph, its
    /// live label and its pressed state, greyed with the reason when it is
    /// disabled. Each row publishes `workbench:btn:<id>` while the menu is
    /// open; a click surfaces that id and closes the menu. Used by every declared MENU button.
    fn menu_rows(
        &mut self,
        ui: &mut egui::Ui,
        opener: &egui::Response,
        rows: &[&workbench::WorkbenchButton],
        state: &workbench::ButtonState,
    ) -> Option<&'static str> {
        let labels: Vec<String> = rows.iter().map(|b| b.label(state)).collect();
        let pairs: Vec<(&str, &str)> =
            rows.iter().zip(&labels).map(|(b, label)| (b.glyph, label.as_str())).collect();
        let width = toolbar_button::menu_width(ui, &pairs);
        let mut clicked = None;
        egui::Popup::menu(opener).width(width).show(|ui| {
            for (button, label) in rows.iter().zip(&labels) {
                let disabled = button.disabled_reason(state);
                let draw = |ui: &mut egui::Ui| {
                    toolbar_button::menu_row(ui, button.glyph, label, button.is_pressed(state), width)
                };
                let resp = match disabled {
                    Some(why) => ui
                        .add_enabled_ui(false, draw)
                        .inner
                        .on_disabled_hover_text(format!("{label} \u{2014} {why}")),
                    None => draw(ui),
                };
                self.hits.insert(format!("workbench:btn:{}", button.id), resp.rect);
                if resp.clicked() {
                    clicked = Some(button.id);
                    ui.close();
                }
            }
        });
        clicked
    }

    /// Draw an explicit list of workbench buttons via the shared button helper,
    /// publishing each hit-rect and surfacing the clicked button's `id` (the
    /// toolbar RETURN PATH — the shell dispatches on it, exactly like a
    /// [`FileAction`]). Split out from [`Self::workbench_buttons_row`] so the
    /// mechanism can be unit-tested with a synthetic button.
    ///
    /// A button that declares a PRESSED predicate draws as a TOGGLE reflecting
    /// it — the armed sheet-dimension construction is the lit button, the way
    /// wireframe and projection light up above. Its hover text is its live
    /// label ([`workbench::WorkbenchButton::label`]); a DISABLED one draws
    /// greyed with the reason after it, and a click on it does nothing. A MENU
    /// button opens its entries ([`Self::menu_rows`]) instead of surfacing its
    /// own id.
    fn draw_workbench_buttons(
        &mut self,
        ui: &mut egui::Ui,
        buttons: &[&workbench::WorkbenchButton],
        state: &workbench::ButtonState,
    ) -> Option<&'static str> {
        let mut clicked = None;
        for button in buttons {
            let label = button.label(state);
            let draw = |ui: &mut egui::Ui| match button.pressed {
                Some(_) => toolbar_button::toggle(ui, button.is_pressed(state), button.glyph, &label),
                None => toolbar_button::button(ui, button.glyph, &label),
            };
            let resp = match button.disabled_reason(state) {
                Some(why) => ui
                    .add_enabled_ui(false, draw)
                    .inner
                    .on_disabled_hover_text(format!("{label} \u{2014} {why}")),
                None => draw(ui),
            };
            self.hits.insert(format!("workbench:btn:{}", button.id), resp.rect);
            if !button.menu.is_empty() {
                let entries: Vec<&workbench::WorkbenchButton> =
                    button.menu.iter().filter(|entry| entry.offered(state)).collect();
                if let Some(id) = self.menu_rows(ui, &resp, &entries, state) {
                    clicked = Some(id);
                }
            } else if resp.clicked() {
                clicked = Some(button.id);
            }
        }
        clicked
    }

    /// The Help button: opens the generated help site (`brep-docs` writes it to
    /// `web/help/` next to the served page) in a new tab. A circled question
    /// mark (U+2753), drawn in the same line style as the ℹ beside it — this app
    /// ships NO font fallback, so a glyph is only ever a key into the SVG
    /// catalog (`assets/glyphs/icon_2753.svg`) and an uncatalogued character
    /// would paint as tofu on wasm.
    fn help_action(&mut self, ui: &mut egui::Ui) {
        let btn = toolbar_button::button(ui, "\u{2753}", "Help");
        self.hits.insert("help".into(), btn.rect);
        if btn.clicked() {
            // ONE literal for the help URL, shared with the `help_open` command.
            ui.ctx().open_url(egui::OpenUrl::new_tab(crate::automation::HELP_URL));
        }
    }

    /// The Info toggle: opens / closes the floating Info window — the licences
    /// and this session's renderer diagnostics (see [`crate::panels::info`]).
    /// The ℹ glyph (U+2139) this button now carries is the one the single Docs
    /// button used to; Help took the question mark, which is what a user looks
    /// for when they want the manual.
    fn info_action(&mut self, ui: &mut egui::Ui, open: &mut bool) {
        let btn = toolbar_button::toggle(ui, *open, "\u{2139}", "Info (licences and diagnostics)");
        self.hits.insert("info".into(), btn.rect);
        if btn.clicked() {
            *open = !*open;
        }
    }

    /// The Submit Bug button: opens the in-app problem-report flow, which first
    /// grabs a screenshot of the app (UI + 3D model) BEFORE its dialog appears,
    /// then collects a description + optional email and posts the report.
    /// `bug_report` Material Symbol (base glyph U+1F41E) — `toolbar_button`
    /// auto-renders it from the icon catalog: U+1F41E is COLOUR artwork, so the
    /// button paints the SVG rather than the font glyph.
    /// Returns whether it was clicked.
    fn bug_action(&mut self, ui: &mut egui::Ui) -> bool {
        let btn = toolbar_button::button(ui, "\u{1F41E}", "Submit Bug");
        self.hits.insert("bug".into(), btn.rect);
        btn.clicked()
    }

    /// The Part Properties toggle: opens / closes the floating window that edits
    /// the OPEN document's own BOM attributes (Part Number, Material, Mass, …).
    /// A tag glyph (U+E066), reflecting the live open state like the gear beside
    /// it. Document-level, so it needs no selection and is never disabled — every
    /// document is a part, an assembly included (it is one BOM row in its parent).
    fn properties_action(&mut self, ui: &mut egui::Ui, open: &mut bool) {
        let btn = toolbar_button::toggle(ui, *open, "\u{E066}", "Part properties");
        self.hits.insert("properties".into(), btn.rect);
        if btn.clicked() {
            *open = !*open;
        }
    }

    /// The Settings toggle: opens / closes the floating Settings window. A
    /// selectable gear glyph (U+2699, bundled DejaVu font) reflecting the live
    /// open state; the label lives in the tooltip, matching the other buttons.
    fn settings_action(&mut self, ui: &mut egui::Ui, open: &mut bool) {
        // Gear (U+2699) — renders in the bundled DejaVu font (no tofu). A toggle
        // reflecting the live open state.
        let btn = toolbar_button::toggle(ui, *open, "\u{2699}", "Settings");
        self.hits.insert("settings".into(), btn.rect);
        if btn.clicked() {
            *open = !*open;
        }
    }

    /// The PLM CONNECTION INDICATOR (plan S1 part 3): present only while the
    /// store reports a PLM, so a file-only session has none. `PLM: <user>`
    /// when this session is on the server, `PLM: sign in` when it is
    /// configured and not on it (plain text: no glyph a font could lack);
    /// the tooltip says where and as whom, or why not, and a click opens
    /// Settings, whose PLM tab signs in. S4's inbox badge sits beside it
    /// while the session is on the server; the rows it opens are returned.
    fn plm_indicator(
        &mut self,
        ui: &mut egui::Ui,
        store: &dyn ModelStore,
        settings_open: &mut bool,
    ) -> Vec<crate::panels::plm_review::PlmReviewEvent> {
        use crate::plm::connection::Status;
        let Some(session) = store.plm_session() else { return Vec::new() };
        ui.separator();
        let (label, tip) = match &session.status {
            Status::Connected { url, username } => (format!("PLM: {username}"), format!("On the PLM at {url} as {username}")),
            Status::NotConnected { url, reason } => (
                "PLM: sign in".to_string(),
                match reason {
                    Some(why) => format!("Not connected to the PLM at {url}: {why}. Settings → PLM signs in."),
                    None => format!("Not connected to the PLM at {url}. Settings → PLM signs in."),
                },
            ),
        };
        let btn = ui.button(label).on_hover_text(tip);
        self.hits.insert("plm:indicator".into(), btn.rect);
        if btn.clicked() {
            *settings_open = true;
        }
        let Some(client) = store.plm_client() else { return Vec::new() };
        let events = self.inbox.show(ui, &crate::panels::plm_review::PlmReviewClient { client });
        self.hits.extend(self.inbox.hits().iter().map(|(key, rect)| (key.clone(), *rect)));
        events
    }

    /// Ask the review inbox again on the next frame: the user just decided,
    /// submitted or released something (`panels::plm_review`).
    pub fn refresh_inbox(&mut self) {
        self.inbox.refresh();
    }

    /// The review inbox badge, for the state blob and tests.
    pub fn inbox(&self) -> &crate::panels::plm_review::InboxBadge {
        &self.inbox
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// File actions (New / Open / Save / Save As) as glyph buttons. Each returns
    /// the matching [`FileAction`] on click; the shell hands it to the file
    /// dialog (which owns all storage). Glyph → label in the tooltip.
    fn file_actions(&mut self, ui: &mut egui::Ui) -> Option<FileAction> {
        let mut action = None;
        // New — page (U+1F4C4, the previous-app glyph).
        let new = toolbar_button::button(ui, "\u{E010}", "New");
        self.hits.insert("file:new".into(), new.rect);
        if new.clicked() {
            action = Some(FileAction::New);
        }
        // New of a chosen class: a family seed (`.fbrep`) or a template
        // (`.tbrep`) starts here, or from Save As on an existing part.
        let new_menu = toolbar_button::button(ui, "\u{25BE}", "New part, family or template\u{2026}");
        self.hits.insert("file:newmenu".into(), new_menu.rect);
        egui::Popup::menu(&new_menu).show(|ui| {
            for class in crate::document_class::DocumentClass::ALL {
                let label = match class {
                    crate::document_class::DocumentClass::Normal => "New part (.nbrep)",
                    crate::document_class::DocumentClass::Family => "New family (.fbrep)",
                    crate::document_class::DocumentClass::Template => "New template (.tbrep)",
                };
                let row = ui.button(label);
                self.hits.insert(format!("file:newclass:{}", class.slug()), row.rect);
                if row.clicked() {
                    action = Some(FileAction::NewOfClass(class));
                    ui.close();
                }
            }
        });
        // Open — open folder (U+1F5C1). Kept: the previous app had no Open.
        let open = toolbar_button::button(ui, "\u{E011}", "Open");
        self.hits.insert("file:open".into(), open.rect);
        if open.clicked() {
            action = Some(FileAction::Open);
        }
        // Save — floppy disk (U+1F4BE, the previous-app glyph).
        let save = toolbar_button::button(ui, "\u{E012}", "Save");
        self.hits.insert("file:save".into(), save.rect);
        if save.clicked() {
            action = Some(FileAction::Save);
        }
        // Save As has a dedicated custom-font glyph so its plus badge shares
        // the disk's weight, alignment, and square advance.
        let save_as = toolbar_button::button(ui, "\u{E013}", "Save As");
        self.hits.insert("file:saveas".into(), save_as.rect);
        if save_as.clicked() {
            action = Some(FileAction::SaveAs);
        }
        ui.separator();
        // Import — neutral CAD files use their native readers; meshes run through
        // RANSAC reconstruction before being appended as an IMPORT3D feature.
        let import = toolbar_button::button(ui, "\u{E014}", "Import STEP / IGES / STL / OBJ\u{2026}");
        self.hits.insert("file:import".into(), import.rect);
        if import.clicked() {
            action = Some(FileAction::Import);
        }
        // Export — outbox tray (U+1F4E4): write the model OUT as STEP / IGES / STL.
        let export = toolbar_button::button(ui, "\u{E015}", "Export\u{2026} (STEP / IGES / STL)");
        self.hits.insert("file:export".into(), export.rect);
        if export.clicked() {
            action = Some(FileAction::Export);
        }
        action
    }

    /// Undo / Redo — trigger the engine-owned undo history. Buttons enable only
    /// when a step is available so the affordance reflects the real stack. Glyph
    /// only; the label lives in the tooltip.
    ///
    /// While a sketch is open these SAME buttons drive the per-session SKETCH
    /// history instead of the model-level undo — the sketch has no undo/redo of
    /// its own; it shares this toolbar pair (matching the Ctrl+Z / Ctrl+Shift+Z
    /// keyboard router in `app.rs`).
    fn edit_actions(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        let sketch = state.sketch_mode();
        let can_undo = if sketch { state.sketch_can_undo() } else { state.can_undo() };
        let undo = toolbar_button::button_enabled(
            ui,
            can_undo,
            "\u{E016}",
            if sketch { "Undo sketch edit (Ctrl+Z)" } else { "Undo" },
        );
        self.hits.insert("undo".into(), undo.rect);
        if undo.clicked() {
            if sketch {
                state.sketch_undo();
            } else {
                state.undo();
            }
        }
        let can_redo = if sketch { state.sketch_can_redo() } else { state.can_redo() };
        let redo = toolbar_button::button_enabled(
            ui,
            can_redo,
            "\u{E017}",
            if sketch { "Redo sketch edit (Ctrl+Y)" } else { "Redo" },
        );
        self.hits.insert("redo".into(), redo.rect);
        if redo.clicked() {
            if sketch {
                state.sketch_redo();
            } else {
                state.redo();
            }
        }
    }

    /// View actions: the Wireframe toggle (drives `settings.wireframe` through the
    /// settings-apply path + persists it like the settings panel), the Projection
    /// toggle (orthographic ↔ perspective via `set_projection`), Zoom-to-fit, and
    /// quick standard-view buttons.
    fn view_actions(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
    ) -> bool {
        // Wireframe: reflect the LIVE engine value so the toggle is always honest,
        // and flip it via the same apply-path the settings panel uses (bumps
        // settings_generation + dirty so the GPU re-derives styles).
        let wire = state.settings.wireframe;
        // Open cube with dashed hidden edges (U+1F578).
        let wf = toolbar_button::toggle(ui, wire, "\u{1F578}", "Wireframe");
        self.hits.insert("wireframe".into(), wf.rect);
        if wf.clicked() {
            let next = !wire;
            let _ = state.apply_settings_json(&format!("{{\"wireframe\": {next}}}"));
            // Persist the full settings through the same seam the settings panel
            // uses, so the toggle survives a reload and both views agree.
            let _ = store.write(SETTINGS_KEY, &state.settings_json());
        }

        // Projection: reflect the LIVE camera mode — the toggle is highlighted while
        // in perspective. Flip it through the SAME settings-apply path wireframe uses
        // (`apply_settings_json` reads `orthographic` and drives the camera), then
        // persist the full settings — so, like wireframe, the projection is now a
        // real setting that survives a reload and agrees with the settings panel.
        let is_persp =
            matches!(state.camera.projection, brep_render::view::Projection::Perspective { .. });
        // Still-camera icon artwork — the ortho ↔ perspective
        // toggle. The tooltip names the CURRENT mode.
        let proj_tip = if is_persp {
            "Perspective projection"
        } else {
            "Orthographic projection"
        };
        let proj = toolbar_button::toggle(ui, is_persp, "\u{E018}", proj_tip);
        self.hits.insert("projection".into(), proj.rect);
        if proj.clicked() {
            let want_ortho = is_persp; // currently perspective → switch to orthographic
            let _ = state.apply_settings_json(&format!("{{\"orthographic\": {want_ortho}}}"));
            let _ = store.write(SETTINGS_KEY, &state.settings_json());
        }

        // The three DISPLAY-CLASS toggles, beside wireframe and projection
        // because they answer the same question — what does the viewport draw.
        // Each is a real setting (`showFaces` / `showEdges` / `showVertices`),
        // not a zeroed size: turning edges off and on again must give back the
        // edge width the user chose, not 0.
        for (glyph, label, key, on) in [
            ("\u{E028}", "Show faces", "show:faces", state.settings.show_faces),
            ("\u{E029}", "Show edges", "show:edges", state.settings.show_edges),
            ("\u{E02A}", "Show vertices", "show:vertices", state.settings.show_vertices),
        ] {
            let btn = toolbar_button::toggle(ui, on, glyph, label);
            self.hits.insert(key.into(), btn.rect);
            if btn.clicked() {
                // The settings JSON key is the hit key's tail, capitalised —
                // `show:faces` drives `showFaces`, through the same apply+persist
                // path wireframe and projection use.
                let field = match key {
                    "show:faces" => "showFaces",
                    "show:edges" => "showEdges",
                    _ => "showVertices",
                };
                let next = !on;
                let _ = state.apply_settings_json(&format!("{{\"{field}\": {next}}}"));
                let _ = store.write(SETTINGS_KEY, &state.settings_json());
            }
        }

        // Square-with-four-corners (U+26F6) — the previous-app zoom-to-fit glyph.
        // ONE fit button for both things the central tile can draw: the tooltip
        // names which, and the click is surfaced to `BrepApp::zoom_to_fit`,
        // which frames the OPEN SHEET's paper when there is one and the 3D scene
        // otherwise. This is why the sheet needs no Fit button of its own.
        let on_paper = state.sheet_open().is_some();
        let fit = toolbar_button::button(
            ui,
            "\u{26F6}",
            if on_paper { "Zoom to fit (the open sheet's paper)" } else { "Zoom to fit" },
        );
        self.hits.insert("fit".into(), fit.rect);
        fit.clicked()
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "toolbar", prefix: "file:new", meaning: "new document", command: Some("doc_new") },
    HitKeyDoc { panel: "toolbar", prefix: "file:newmenu", meaning: "the New menu: a new part, family or template", command: Some("doc_new_of_class") },
    HitKeyDoc { panel: "toolbar", prefix: "file:newclass:", meaning: "a New menu row: file:newclass:normal, file:newclass:family, file:newclass:template (open while the New menu is)", command: Some("doc_new_of_class") },
    HitKeyDoc { panel: "toolbar", prefix: "file:open", meaning: "open", command: Some("doc_load") },
    HitKeyDoc { panel: "toolbar", prefix: "file:save", meaning: "save", command: Some("doc_json") },
    HitKeyDoc { panel: "toolbar", prefix: "file:saveas", meaning: "save as", command: Some("doc_json") },
    HitKeyDoc { panel: "toolbar", prefix: "file:import", meaning: "import a file", command: Some("doc_import") },
    HitKeyDoc { panel: "toolbar", prefix: "file:export", meaning: "export", command: Some("doc_export") },
    HitKeyDoc { panel: "toolbar", prefix: "undo", meaning: "undo", command: Some("undo") },
    HitKeyDoc { panel: "toolbar", prefix: "redo", meaning: "redo", command: Some("redo") },
    HitKeyDoc { panel: "toolbar", prefix: "fit", meaning: "zoom to fit \u{2014} the whole model, or the OPEN SHEET's whole paper while a sheet is open", command: Some("zoom_to_fit") },
    HitKeyDoc { panel: "toolbar", prefix: "projection", meaning: "toggle perspective/orthographic", command: Some("set_projection") },
    HitKeyDoc { panel: "toolbar", prefix: "wireframe", meaning: "toggle wireframe", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "show:faces", meaning: "toggle the shaded faces", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "show:edges", meaning: "toggle the edges", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "show:vertices", meaning: "toggle the vertex points", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "properties", meaning: "open the part properties window", command: Some("part_properties_window") },
    HitKeyDoc { panel: "toolbar", prefix: "settings", meaning: "open the settings window", command: Some("settings_window") },
    HitKeyDoc { panel: "toolbar", prefix: "plm:", meaning: "the PLM connection indicator, plm:indicator, drawn only while a PLM is configured: opens Settings (whose PLM tab signs in); beside it while signed in, the review inbox badge plm:inbox (`Inbox (N)`, N waiting on you) and, while its list is open, a row plm:inbox:row:<target><revision id> opening that review or change order in the PLM pane", command: Some("settings_window") },
    HitKeyDoc { panel: "toolbar", prefix: "help", meaning: "open the help site", command: Some("help_open") },
    HitKeyDoc { panel: "toolbar", prefix: "info", meaning: "open the info window (licences + diagnostics)", command: Some("info_window") },
    HitKeyDoc { panel: "toolbar", prefix: "bug", meaning: "open the bug report", command: Some("bug_report_open") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench", meaning: "the workbench dropdown", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench:btn:", meaning: "one button of the ACTIVE workbench's action row (workbench:btn:<button id>) \u{2014} published only while that button is offered, so Drawing's `drawing.sheet_close`, the six `drawing.dim.*` constructions, the two `drawing.ord.*` sets, `drawing.section` and `drawing.detail` are here exactly while a sheet is open, and the ten `sketch.tool.*` draw tools and `sketch.autoconstrain` (which EVERY workbench carries) exactly while a sketch is being edited; the armed construction and the armed draw tool draw pressed. A button that is offered but DISABLED (an eCAD action its editor does not allow yet) draws greyed and `workbench_button` refuses it with the reason. A MENU button (PCB's copper-layer picker) opens a menu whose entries publish this key while it is OPEN. All offered buttons wrap onto additional rows when needed and keep their hit keys; explicit menu entries publish only while their menu is open", command: Some("workbench_button") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench:item:", meaning: "pick a workbench from the open dropdown (workbench:item:id)", command: Some("settings_set") },
];

//! Global toolbar shell: File menu and either Ribbon or Classic presentation.
//!
//! Both renderers consume the same offered-command snapshots and button helper.
//! Command targets retain the existing engine, workbench, plugin and creation
//! dispatch paths. This panel owns presentation and automation hit rectangles;
//! the shell owns File dialogs and creation outcomes.

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
    pub recent_document: Option<String>,
    pub creation: super::workbench_toolbar::WorkbenchToolbarOutcome,
    /// A workbench toolbar button click, surfaced by its `WorkbenchButton::id`.
    pub workbench_button: Option<&'static str>,
    pub plugin_action: Option<String>,
    pub plugins: bool,
    pub javascript: bool,
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
    ribbon: super::ribbon::RibbonPanel,
    pub read_only: bool,
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
                let style = state.settings.toolbar_style;
                let mut target = None;
                toolbar_button::wrapped(ui, |ui| {
                    let file = super::file::file_menu(
                        ui,
                        state,
                        store,
                        settings_open,
                        properties_open,
                        &mut self.hits,
                    );
                    outcome.file = file.action;
                    outcome.recent_document = file.recent;
                    outcome.plugins |= file.plugins;
                    outcome.javascript |= file.javascript;
                    if style == brep_render::style::ToolbarStyle::Ribbon {
                        self.ribbon.tabs(ui, &mut self.hits);
                    } else {
                        let commands = self.commands(state, ecad, *info_open);
                        let mut last_group = None;
                        for command in commands.iter().filter(|c| !c.is_creation()) {
                            let segments = command.segments();
                            let group = (segments[0], segments[1]);
                            if last_group != Some(group) {
                                ui.separator();
                                last_group = Some(group);
                            }
                            if let Some(t) = super::ribbon::draw_command(
                                ui,
                                command,
                                true,
                                false,
                                &mut self.hits,
                            ) {
                                target = Some(t);
                            }
                        }
                    }
                    outcome.plm_review = self.plm_indicator(ui, store, settings_open);
                });
                if style == brep_render::style::ToolbarStyle::Ribbon {
                    let commands = self.commands(state, ecad, *info_open);
                    target = self.ribbon.show(ui, &commands, &mut self.hits).or(target);
                }
                if let Some(target) = target {
                    self.dispatch_target(ui.ctx(), target, state, store, info_open, &mut outcome);
                }
                ui.add_space(3.0);
            });
        outcome
    }

    fn commands(
        &self,
        state: &EngineState,
        ecad: Option<&workbench::ecad::Editors>,
        info_open: bool,
    ) -> Vec<workbench::OfferedCommand> {
        let mut commands = workbench::offered_commands(
            &workbench::ButtonState {
                engine: state,
                ecad,
            },
            info_open,
        );
        if self.read_only {
            for c in commands.iter_mut().filter(|c| c.is_creation()) {
                c.disabled_reason = Some("Document is read-only".into());
            }
        }
        commands
    }

    fn dispatch_target(
        &mut self,
        ctx: &egui::Context,
        target: workbench::CommandTarget,
        state: &mut EngineState,
        store: &dyn ModelStore,
        info_open: &mut bool,
        outcome: &mut ToolbarOutcome,
    ) {
        use workbench::CommandTarget;
        let key = match target {
            CommandTarget::Workbench(id) => {
                outcome.workbench_button = Some(id);
                return;
            }
            CommandTarget::Plugin(id) => {
                outcome.plugin_action = Some(id);
                return;
            }
            CommandTarget::Feature(id) => {
                outcome.creation.feature = Some(id);
                return;
            }
            CommandTarget::Constraint(id) => {
                outcome.creation.constraint = Some(id);
                return;
            }
            CommandTarget::Annotation(id) => {
                outcome.creation.annotation = Some(id);
                return;
            }
            CommandTarget::Shell(key) => key,
        };
        match key {
            "file:new" => outcome.file = Some(FileAction::New),
            "file:open" => outcome.file = Some(FileAction::Open),
            "file:save" => outcome.file = Some(FileAction::Save),
            "file:saveas" => outcome.file = Some(FileAction::SaveAs),
            "file:import" => outcome.file = Some(FileAction::Import),
            "file:export" => outcome.file = Some(FileAction::Export),
            "undo" => {
                if state.sketch_mode() {
                    state.sketch_undo();
                } else {
                    state.undo();
                }
            }
            "redo" => {
                if state.sketch_mode() {
                    state.sketch_redo();
                } else {
                    state.redo();
                }
            }
            "fit" => outcome.zoom_to_fit = true,
            "info" => *info_open = !*info_open,
            "bug" => outcome.bug_report = true,
            "workbench:manage:plugins" => outcome.plugins = true,
            "workbench:manage:javascript" => outcome.javascript = true,
            "help" => ctx.open_url(egui::OpenUrl::new_tab(crate::automation::HELP_URL)),
            key => {
                let (field, value) = match key {
                    "wireframe" => ("wireframe", !state.settings.wireframe),
                    "projection" => (
                        "orthographic",
                        matches!(
                            state.camera.projection,
                            brep_render::view::Projection::Perspective { .. }
                        ),
                    ),
                    "show:faces" => ("showFaces", !state.settings.show_faces),
                    "show:edges" => ("showEdges", !state.settings.show_edges),
                    "show:vertices" => ("showVertices", !state.settings.show_vertices),
                    _ => return,
                };
                let _ = state.apply_settings_json(&serde_json::json!({field: value}).to_string());
                let _ = store.write(SETTINGS_KEY, &state.settings_json());
            }
        }
    }

    pub fn home_hits(&self) -> HashMap<String, egui::Rect> {
        self.hits
            .iter()
            .filter(|(key, _)| key.starts_with("wbtb:"))
            .map(|(k, r)| (k.clone(), *r))
            .collect()
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
        let Some(session) = store.plm_session() else {
            return Vec::new();
        };
        ui.separator();
        let (label, tip) = match &session.status {
            Status::Connected { url, username } => (
                format!("PLM: {username}"),
                format!("On the PLM at {url} as {username}"),
            ),
            Status::NotConnected { url, reason } => (
                "PLM: sign in".to_string(),
                match reason {
                    Some(why) => format!(
                        "Not connected to the PLM at {url}: {why}. Settings → PLM signs in."
                    ),
                    None => format!("Not connected to the PLM at {url}. Settings → PLM signs in."),
                },
            ),
        };
        let btn = ui.button(label).on_hover_text(tip);
        self.hits.insert("plm:indicator".into(), btn.rect);
        if btn.clicked() {
            *settings_open = true;
        }
        if let Some(url) = crate::panels::plm_host::web_url(store, "#/reviews") {
            let link = crate::panels::plm_host::web_link(ui, "PLM inbox", url);
            self.hits.insert("plm:inbox".into(), link.rect);
        }
        Vec::new()
    }

    /// Ask the review inbox again on the next frame: the user just decided,
    /// submitted or released something (`panels::plm_review`).
    pub fn refresh_inbox(&mut self) {
        self.inbox.refresh();
    }

    /// The review inbox badge, for tests (never drawn: the toolbar links to the
    /// web inbox instead, and publishes no blob for it).
    pub fn inbox(&self) -> &crate::panels::plm_review::InboxBadge {
        &self.inbox
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "toolbar", prefix: "workbench:manage:plugins", meaning: "open the installed plugin manager: the Plugins row under Settings in the File menu (both styles)", command: Some("plugins_window") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench:manage:javascript", meaning: "open the JavaScript editor: the JavaScript row under Settings in the File menu (both styles)", command: Some("javascript_window") },
    HitKeyDoc { panel: "toolbar", prefix: "plugin:action:", meaning: "open the form for a named plugin action", command: Some("plugin_action") },
    HitKeyDoc { panel: "toolbar", prefix: "file:menu", meaning: "open the shared File menu", command: None },
    HitKeyDoc { panel: "toolbar", prefix: "file:recent", meaning: "recent documents section and document rows, present only when nonempty", command: Some("doc_load") },
    HitKeyDoc { panel: "toolbar", prefix: "ribbon:", meaning: "ribbon tabs (ribbon:tab:<Tab>), group borders (ribbon:group:<Group>) and the overflow menu (ribbon:overflow)", command: None },
    HitKeyDoc { panel: "toolbar", prefix: "wbtb:", meaning: "Home creation commands; same keys as the Classic feature strip", command: Some("feature_add") },
    HitKeyDoc { panel: "toolbar", prefix: "file:new", meaning: "new document", command: Some("doc_new") },
    HitKeyDoc { panel: "toolbar", prefix: "file:newmenu", meaning: "legacy key for the flat New part row in File", command: Some("doc_new_of_class") },
    HitKeyDoc { panel: "toolbar", prefix: "file:newclass:", meaning: "a New menu row: file:newclass:normal, file:newclass:family, file:newclass:template (visible directly in File)", command: Some("doc_new_of_class") },
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
    HitKeyDoc { panel: "toolbar", prefix: "plm:", meaning: "plm:indicator opens connection settings; plm:inbox opens the PLM web inbox in another tab", command: Some("settings_window") },
    HitKeyDoc { panel: "toolbar", prefix: "help", meaning: "open the help site", command: Some("help_open") },
    HitKeyDoc { panel: "toolbar", prefix: "info", meaning: "open the info window (licences + diagnostics)", command: Some("info_window") },
    HitKeyDoc { panel: "toolbar", prefix: "bug", meaning: "open the bug report", command: Some("bug_report_open") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench", meaning: "legacy opener key for the shared File menu", command: Some("settings_set") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench:btn:", meaning: "one button of the ACTIVE workbench's action row (workbench:btn:<button id>) \u{2014} published only while that button is offered, so Drawing's `drawing.sheet_close`, the six `drawing.dim.*` constructions, the two `drawing.ord.*` sets, `drawing.section` and `drawing.detail` are here exactly while a sheet is open, and the ten `sketch.tool.*` draw tools and `sketch.autoconstrain` (which EVERY workbench carries) exactly while a sketch is being edited; the armed construction and the armed draw tool draw pressed. A button that is offered but DISABLED (an eCAD action its editor does not allow yet) draws greyed and `workbench_button` refuses it with the reason. A MENU button (PCB's copper-layer picker) opens a menu whose entries publish this key while it is OPEN. Classic wraps available buttons; Ribbon places them in their declared tabs and groups, compacting Large buttons before whole-group overflow. Menu entries publish only while their menu is open", command: Some("workbench_button") },
    HitKeyDoc { panel: "toolbar", prefix: "workbench:item:", meaning: "pick a radio row from File / Workbench (workbench:item:id)", command: Some("settings_set") },
];

//! In-app scratch scripts. Source is saved only on request and never on a model.
use crate::{
    document::Documents,
    store::{ModelStore, JAVASCRIPT_DRAFT_KEY},
};
use crate::automation::hit_keys::HitKeyDoc;
use eframe::egui;
use std::collections::HashMap;

/// The window's egui id — one name for the panel, the tests and the host.
pub const WINDOW_ID: &str = "brep-javascript-editor";

/// The hit keys this window publishes under `__brepJavascriptHit`
/// (see `automation::hit_keys`); `panel:clip` is the key every scrolling pane
/// publishes for its visible region, so `click_widget` can wheel a row into view.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "javascript", prefix: "panel:clip", meaning: "the JavaScript editor window's OUTER rect — its whole visible region, frame included, so the title and grip handles below count as in view", command: None },
    HitKeyDoc { panel: "javascript", prefix: "title", meaning: "a point on the window's title bar — a drag from it MOVES the window", command: None },
    HitKeyDoc { panel: "javascript", prefix: "grip", meaning: "a point on the window's bottom-right resize corner — a drag from it RESIZES the window in both axes, up to the app surface", command: None },
    HitKeyDoc { panel: "javascript", prefix: "run", meaning: "run the script in the active document", command: None },
    HitKeyDoc { panel: "javascript", prefix: "cancel", meaning: "cancel the running script", command: None },
    HitKeyDoc { panel: "javascript", prefix: "save", meaning: "save the source as the local draft", command: None },
    HitKeyDoc { panel: "javascript", prefix: "open", meaning: "open a .js file into the editor (native; file-interchange stores only)", command: None },
    HitKeyDoc { panel: "javascript", prefix: "export", meaning: "export the source as script.js through the file dialog's export door: the Save-file chooser on desktop, a download in the browser", command: None },
    HitKeyDoc { panel: "javascript", prefix: "help", meaning: "the CAD API and examples collapsing header", command: None },
    HitKeyDoc { panel: "javascript", prefix: "example", meaning: "load the box example into the editor (inside `help`)", command: None },
    HitKeyDoc { panel: "javascript", prefix: "source", meaning: "the source editor's scroll region (the free space of the window: it grows with the window)", command: None },
    HitKeyDoc { panel: "javascript", prefix: "output", meaning: "the output's scroll region, at the bottom of the window", command: None },
];

const EXAMPLE: &str = r#"// Run adds a box to the current document. Undo removes the whole run.
cad.document.addFeature("P.CU", { sizeX: 30, sizeY: 20, sizeZ: 10 });
console.log("Created a box");
ui.notify("Edit its dimensions in the feature history");
return { featuresBefore: cad.document.features.length };
"#;

/// Where Run sends a program when the app has a script runner: the app's
/// own server's code mode (`crate::mcp::EditorHost`), the same realm, limits
/// and generated `brep.<group>.<tool>` functions an agent's `run_script`
/// gets. Without one (the web build, a bare test harness) Run uses the
/// plugin runner's editor action.
pub trait ScriptHost {
    /// Whether the runner is up; the first call starts it.
    fn ready(&self) -> Result<bool, String>;
    /// Submit a program; answers its execution id.
    fn run(&self, source: &str) -> Result<String, String>;
    /// The execution's result once it has ended, `None` while it runs.
    fn result(&self, id: &str) -> Result<Option<serde_json::Value>, String>;
    /// Halt it: the worker is killed wherever it is.
    fn halt(&self, id: &str) -> Result<(), String>;
}

/// A run on the [`ScriptHost`], in its document. Its edits fold into one
/// undo group, kept when the program succeeds and rolled back otherwise, as
/// the plugin runner's transaction is.
enum Script {
    Starting { doc: u64, program: String },
    Running { doc: u64, id: String, token: u64 },
    /// Ended: the group closes once the document's engine is idle, so a
    /// staged feature still landing folds in (or rolls back) with the rest.
    Closing { doc: u64, token: u64, rollback: bool, output: String, since: web_time::Instant, idle: u32 },
}

/// How long a closing run waits for its document to go idle before the
/// group closes anyway (an edit still landing then stands on its own).
const CLOSE_WAIT: std::time::Duration = std::time::Duration::from_secs(30);

/// A closing run's document must read idle this many polls running, and
/// [`CLOSE_SETTLE`] must have passed: a tool call the program issued just
/// before it ended (or was killed) may still be on its way to the app's
/// queue, which drains AFTER this poll in the frame, and its edit belongs in
/// the group.
const CLOSE_IDLE_POLLS: u32 = 5;
const CLOSE_SETTLE: std::time::Duration = std::time::Duration::from_millis(250);

/// An editor script as a program for the [`ScriptHost`]. A module (a line
/// starting `import` or `export`) runs as written. Anything else is a classic
/// editor body: it runs with `cad.document` (the history snapshot with
/// `addFeature(type, params, persistentData)`, `updateFeature(id, params)`
/// (a merge, as `feature_set_params` merges) and `deleteFeature(id)`),
/// `cad.selection`, `cad.brep` (every app tool) and `ui.notify`; the document
/// commands apply in order after it returns, and a returned value prints as
/// `Result:`. The body may `await`.
pub fn program(source: &str) -> String {
    if source.lines().any(|l| l.starts_with("import ") || l.starts_with("export ")) {
        return source.to_string();
    }
    format!(
        r#"import {{ brep }} from "@brep/cad";
export default async function main() {{
  const commands = [];
  const stage = c => {{ if (commands.length >= 256) throw Error("too many document commands"); commands.push(c); }};
  const snapshot = (await brep.document.doc_json({{}})).document;
  const selection = (await brep.scene.selection({{}})).selection;
  const document = Object.freeze({{
    ...snapshot,
    addFeature: (type, params = {{}}, persistentData = {{}}) => stage(() => brep.features.feature_add(Object.keys(persistentData).length ? {{ type, params, persistent_data: persistentData }} : {{ type, params }})),
    updateFeature: (id, params) => stage(() => brep.features.feature_set_params({{ id, patch: params }})),
    deleteFeature: id => stage(() => brep.history.feature_delete({{ id }})),
  }});
  const notify = message => console.log(String(message));
  const cad = Object.freeze({{ document, selection, params: {{}}, notify, brep }});
  const ui = Object.freeze({{ notify }});
  const result = await (async function (cad, ui) {{
{source}
  }})(cad, ui);
  for (const command of commands) await command();
  if (result !== undefined) console.log("Result:", JSON.stringify(result));
  return null;
}}
"#
    )
}

/// What a finished execution shows: the outcome, the console lines, and a
/// module's returned value.
fn script_output(result: &serde_json::Value) -> String {
    let mut lines = vec![match result["state"].as_str() {
        Some("succeeded") => "Completed.".to_string(),
        Some("cancelled") => "Cancelled.".to_string(),
        _ => format!("Failed: {}", result["error"]["message"].as_str().unwrap_or("the script failed")),
    }];
    if let Some(log) = result["logText"].as_str().filter(|l| !l.is_empty()) {
        lines.push(log.to_string());
    }
    if !result["output"].is_null() {
        lines.push(format!("Result: {}", result["output"]));
    }
    lines.join("\n")
}

pub struct JavaScriptEditor {
    pub open: bool,
    source: String,
    output: String,
    pending: Option<u64>, // originating document, retained across tab switches
    /// The app's script runner, when it has one (see [`ScriptHost`]).
    pub host: Option<Box<dyn ScriptHost>>,
    script: Option<Script>,
    importing: bool,
    /// This frame's widget rects (egui points) for the automation host; empty
    /// while the window is closed.
    hits: HashMap<String, egui::Rect>,
    /// The window's outer rect this frame, `None` while it is closed.
    rect: Option<egui::Rect>,
    /// The surface the window may fill (`ctx.content_rect()`), for the host.
    surface: egui::Rect,
    /// "Export .js…" asked: the shell hands these to the file dialog's export
    /// door ([`Self::take_save_requests`]), which on desktop asks where.
    exports: Vec<crate::panels::file::SaveRequest>,
}


impl JavaScriptEditor {
    pub fn running_in(&self, document: u64) -> bool {
        self.pending == Some(document)
    }

    pub fn set_source(&mut self, source: String) {
        self.source = source;
    }

    pub fn editing_source(ctx: &egui::Context) -> bool {
        ctx.memory(|memory| memory.has_focus(egui::Id::new("cad-javascript-source")))
    }

    pub fn new(store: &dyn ModelStore) -> Self {
        Self {
            open: false,
            source: store
                .read(JAVASCRIPT_DRAFT_KEY)
                .unwrap_or_else(|| EXAMPLE.into()),
            output: String::new(),
            pending: None,
            host: None,
            script: None,
            importing: false,
            hits: HashMap::new(),
            exports: Vec::new(),
            rect: None,
            surface: egui::Rect::NOTHING,
        }
    }

    /// Logical state for the automation host: `rect` is the window's outer
    /// rect `[x, y, w, h]` in egui points (null while closed), `surface` the
    /// screen rect it may fill `[w, h]`.
    pub fn state_json(&self) -> String {
        let rect = |r: egui::Rect| serde_json::json!([r.min.x, r.min.y, r.width(), r.height()]);
        serde_json::json!({
            "open": self.open,
            "rect": self.rect.map(rect),
            "surface": [self.surface.width(), self.surface.height()],
            "running": self.pending.is_some(),
            "output": self.output,
        })
        .to_string()
    }

    /// The published widget hit-rects (`[x, y, w, h]`, egui points).
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Halt the running script and roll its edits back once its document
    /// is idle; `output` says why.
    fn halt_script(&mut self, docs: &mut Documents, output: String) {
        match self.script.take() {
            Some(Script::Starting { .. }) => {
                self.pending = None;
                self.output = output;
            }
            Some(Script::Running { doc, id, token }) => {
                if let Some(host) = &self.host {
                    let _ = host.halt(&id);
                }
                // Brake a history run the script started, as the app's
                // cancel button does; a staged feature then rolls back.
                if let Some(d) = docs.iter_mut().find(|d| d.id() == doc) {
                    d.engine.cancel_run();
                }
                self.script = Some(Script::Closing { doc, token, rollback: true, output, since: web_time::Instant::now(), idle: 0 });
            }
            other => self.script = other,
        }
    }

    /// One frame of a run on the [`ScriptHost`].
    fn poll_script(&mut self, ctx: &egui::Context, docs: &mut Documents) {
        ctx.request_repaint_after(std::time::Duration::from_millis(30));
        let doc_id = match &self.script {
            Some(Script::Starting { doc, .. } | Script::Running { doc, .. } | Script::Closing { doc, .. }) => *doc,
            None => return,
        };
        if !docs.iter().any(|d| d.id() == doc_id) {
            if let (Some(Script::Running { id, .. }), Some(host)) = (&self.script, &self.host) {
                let _ = host.halt(id);
            }
            self.script = None;
            self.pending = None;
            self.output = "The script's document was closed.".into();
            return;
        }
        // The app's tools act on the ACTIVE document: a run whose document
        // is no longer active would write another one.
        if docs.active_id() != doc_id && matches!(self.script, Some(Script::Starting { .. } | Script::Running { .. })) {
            return self.halt_script(docs, "Halted: the active document changed while the script ran; its edits were rolled back.".into());
        }
        let Some(host) = &self.host else { return };
        let doc = docs.iter_mut().find(|d| d.id() == doc_id).expect("checked above");
        match self.script.take() {
            Some(Script::Starting { doc: d, program }) => match host.ready() {
                Ok(true) => {
                    let token = doc.engine.history_group_begin();
                    match host.run(&program) {
                        Ok(id) => self.script = Some(Script::Running { doc: d, id, token }),
                        Err(e) => {
                            let _ = doc.engine.history_group_end(token, false);
                            self.pending = None;
                            self.output = e;
                        }
                    }
                }
                Ok(false) => self.script = Some(Script::Starting { doc: d, program }),
                Err(e) => {
                    self.pending = None;
                    self.output = e;
                }
            },
            Some(Script::Running { doc: d, id, token }) => match host.result(&id) {
                Ok(None) => self.script = Some(Script::Running { doc: d, id, token }),
                Ok(Some(result)) => {
                    let rollback = result["state"] != "succeeded";
                    let output = script_output(&result);
                    self.script = Some(Script::Closing { doc: d, token, rollback, output, since: web_time::Instant::now(), idle: 0 });
                }
                Err(e) => self.script = Some(Script::Closing { doc: d, token, rollback: true, output: e, since: web_time::Instant::now(), idle: 0 }),
            },
            Some(Script::Closing { doc: d, token, rollback, output, since, idle }) => {
                doc.engine.pump();
                let quiet = !doc.engine.run_pending() && !doc.engine.stage_feature_pending();
                let idle = if quiet { idle + 1 } else { 0 };
                let settled = idle >= CLOSE_IDLE_POLLS && since.elapsed() >= CLOSE_SETTLE;
                if !settled && since.elapsed() < CLOSE_WAIT {
                    self.script = Some(Script::Closing { doc: d, token, rollback, output, since, idle });
                    return;
                }
                self.pending = None;
                self.output = match doc.engine.history_group_end(token, rollback) {
                    Ok(true) => format!("{output}\nIts edits were rolled back."),
                    Ok(false) => output,
                    Err(e) => format!("{output}\n{e}"),
                };
            }
            None => {}
        }
    }

    /// Record a widget's VISIBLE rect for the host.
    fn hit(&mut self, key: &str, resp: &egui::Response) {
        self.hits.insert(key.to_string(), resp.interact_rect);
    }

    pub fn poll(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        if self.importing {
            if let Some(file) = store.take_import() {
                self.importing = false;
                match String::from_utf8(file.bytes) {
                    Ok(source) => self.source = source,
                    Err(e) => self.output = format!("Cannot open script: {e}"),
                }
            }
        }
        if self.script.is_some() {
            return self.poll_script(ctx, docs);
        }
        let Some(id) = self.pending else {
            return;
        };
        let Some(doc) = docs.iter_mut().find(|doc| doc.id() == id) else {
            self.pending = None;
            self.output = "The script's document was closed.".into();
            return;
        };
        doc.engine.pump();
        let status = doc.engine.plugin_action_status();
        match status["state"].as_str() {
            Some("complete") => {
                let lines: Vec<_> = status["result"]["notifications"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(|v| v.as_str())
                    .collect();
                self.output = format!("Completed.\n{}", lines.join("\n"));
                self.pending = None;
            }
            Some("error") => {
                self.output = status["error"].as_str().unwrap_or("Script failed").into();
                self.pending = None;
            }
            _ => {
                ctx.request_repaint_after(std::time::Duration::from_millis(30));
            }
        }
    }

    /// The files "Export .js…" asked to save, once: the shell hands them to
    /// [`crate::panels::file::FileDialog::request_save_file`].
    pub fn take_save_requests(&mut self) -> Vec<crate::panels::file::SaveRequest> {
        std::mem::take(&mut self.exports)
    }

    pub fn show(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        self.hits.clear();
        self.rect = None;
        self.surface = ctx.content_rect();
        let mut open = self.open;
        let shown = egui::Window::new("JavaScript editor").id(egui::Id::new(WINDOW_ID)).open(&mut open)
            // Under the toolbar / Ribbon like Settings, Plugins and Info: without
            // a position egui put it over the menu bar (ui-polish-2026-10-28 row 5).
            .default_pos(crate::panels::below_chrome(ctx, 16.0, 0.0))
            .default_size(egui::vec2(720.0, 620.0)).min_size(egui::vec2(420.0, 320.0)).resizable(true).show(ctx, |ui| {
            ui.label(format!("Run in: {}", docs.active().title()));
            if let Some(id) = self.pending {
                if let Some(doc) = docs.iter().find(|doc| doc.id() == id) {
                    ui.horizontal(|ui| {
                        ui.spinner();
                        ui.label(format!("Running in: {}", doc.title()));
                    });
                }
            }
            ui.horizontal(|ui| {
                let busy = self.pending.is_some() || docs.engine().run_pending()
                    || docs.engine().plugin_action_status()["state"] == "pending";
                let run = ui.add_enabled(!busy, egui::Button::new("Run script"));
                self.hit("run", &run);
                // `cad.geometry` (kernel solids in an editor body) is the
                // plugin runner's; such a script stays there.
                if run.clicked() && self.host.is_some() && !self.source.contains("cad.geometry") {
                    self.pending = Some(docs.active_id());
                    self.script = Some(Script::Starting { doc: docs.active_id(), program: program(&self.source) });
                    self.output = "Running…".into();
                } else if run.clicked() {
                    match docs.engine_mut().run_javascript(&self.source) {
                        Ok(_) => { self.pending = Some(docs.active_id()); self.output = "Running…".into(); }
                        Err(e) => self.output = e,
                    }
                }
                let cancel = ui.add_enabled(self.pending.is_some(), egui::Button::new("Cancel"));
                self.hit("cancel", &cancel);
                if cancel.clicked() && self.script.is_some() {
                    self.halt_script(docs, "Cancelled.".into());
                } else if cancel.clicked() {
                    if let Some(id) = self.pending {
                        if let Some(doc) = docs.iter_mut().find(|doc| doc.id() == id) {
                            doc.engine.cancel_plugin_action();
                        }
                    }
                }
                let save = ui.button("Save draft");
                self.hit("save", &save);
                if save.clicked() {
                    self.output = match store.write(JAVASCRIPT_DRAFT_KEY, &self.source) {
                        Ok(()) => "Draft saved.".into(), Err(e) => format!("Cannot save draft: {e}"),
                    };
                }
                // Open .js… drives the browser's file input, so only where
                // there is one. Export leaves through the file dialog's export
                // door on every platform (the Save-file chooser on desktop),
                // so it is always offered: gated with Open, desktop had no
                // way to export a script at all.
                if store.supports_file_interchange() {
                    let open = ui.add_enabled(!self.importing, egui::Button::new("Open .js…"));
                    self.hit("open", &open);
                    if open.clicked() {
                        match store.begin_import_filtered(("JavaScript", &["js"])) {
                            Ok(()) => self.importing = true, Err(e) => self.output = e,
                        }
                    }
                }
                let export = ui.button("Export .js…");
                self.hit("export", &export);
                if export.clicked() {
                    self.exports.push(crate::panels::file::SaveRequest::text("Export the script", "script.js", self.source.clone()));
                }
            });
            let help = ui.collapsing("CAD API and examples", |ui| {
                ui.label("Write JavaScript statements. cad.document is a snapshot of the current history; cad.selection contains the current selection.");
                ui.monospace("cad.document.addFeature(type, params)\ncad.document.updateFeature(id, params)\ncad.document.deleteFeature(id)\nconsole.log / info / warn / error(...)\nui.notify(text)\nreturn { any: 'JSON result' };");
                ui.label("Built-in types include P.CU (sizeX, sizeY, sizeZ), P.S (radius), and P.CY (radius, height). Successful changes form one undo step. Each Run executes the script again.");
                if self.host.is_some() {
                    ui.label("Every app tool is a function too: await cad.brep.<group>.<tool>(args), the same functions an agent's run_script gets (typed in the server's brep://cad/sdk.d.ts resource). A program that starts with import { brep } from \"@brep/cad\" and export default async function main() runs as written. A failed or cancelled run is rolled back.");
                }
                let example = ui.button("Load box example");
                if example.clicked() { self.source = EXAMPLE.into(); }
                example
            });
            self.hits.insert("help".into(), help.header_response.interact_rect);
            if let Some(example) = help.body_returned {
                self.hits.insert("example".into(), example.interact_rect);
            }
            // The rest of the window is the editor over the output. Two panels
            // rather than two capped scroll areas: egui sizes a window's axis to
            // its CONTENT (its Resize is not resizable, the content ui merely
            // gets the dragged size as its max rect), so content that does not
            // fill the dragged size snaps the window back to the content's
            // height. Panels fill whatever the window is given, so the window
            // follows the drag — up to the surface, which egui clamps to.
            // The output panel keeps its own height (draggable at its top edge)
            // and the editor takes everything above it.
            egui::containers::panel::Panel::bottom(egui::Id::new("brep-javascript-output"))
                .resizable(true)
                .default_size(160.0)
                .min_size(48.0)
                .frame(egui::Frame::NONE.inner_margin(egui::Margin::symmetric(0, 4)))
                .show_inside(ui, |ui| {
                    ui.label("Output");
                    let output = egui::ScrollArea::both().id_salt("javascript-output").auto_shrink(false).show(ui, |ui| {
                        ui.add(egui::Label::new(egui::RichText::new(&self.output).monospace()).selectable(true));
                    });
                    self.hits.insert("output".into(), output.inner_rect);
                });
            egui::CentralPanel::default().frame(egui::Frame::NONE).show_inside(ui, |ui| {
                // The text box fills the free space, so a click in the empty
                // area under a short script still lands in the editor. By ROWS:
                // egui's `TextEdit::min_size` honours only the width, and the
                // box's height is its rows (or its text, whichever is taller).
                // Floored, so a box that fills the space never scrolls by the
                // odd point.
                let free = ui.available_height();
                let row = ui.fonts_mut(|f| f.row_height(&egui::TextStyle::Monospace.resolve(ui.style())));
                let rows = ((free - 4.0) / row).floor().max(4.0) as usize;
                let source = egui::ScrollArea::both().id_salt("javascript-source").auto_shrink(false).show(ui, |ui| {
                    ui.add(egui::TextEdit::multiline(&mut self.source).code_editor()
                        .id(egui::Id::new("cad-javascript-source"))
                        .desired_rows(rows).desired_width(f32::INFINITY));
                });
                self.hits.insert("source".into(), source.inner_rect);
            });
        });
        // Published only while OPEN: egui keeps returning the window through its
        // fade-out, and a host reading a rect for a window the user just closed
        // would drag on nothing.
        if let (Some(shown), true) = (shown, open) {
            let rect = shown.response.rect;
            self.rect = Some(rect);
            // The window's handles, as points egui's own hit zones answer to: the
            // title bar's centre (a drag moves the window) and a point just inside
            // the bottom-right corner, within the corner grab radius (a drag
            // resizes). `panel:clip` is the outer rect so both read as in view.
            self.hits.insert("panel:clip".into(), rect);
            self.hits.insert("title".into(), egui::Rect::from_center_size(egui::pos2(rect.center().x, rect.min.y + 12.0), egui::Vec2::ZERO));
            self.hits.insert("grip".into(), egui::Rect::from_center_size(rect.max - egui::vec2(3.0, 3.0), egui::Vec2::ZERO));
        } else {
            self.hits.clear();
        }
        self.open = open;
    }
}

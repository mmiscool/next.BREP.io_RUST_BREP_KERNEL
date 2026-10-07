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
    HitKeyDoc { panel: "javascript", prefix: "export", meaning: "export the source as script.js (native; file-interchange stores only)", command: None },
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

pub struct JavaScriptEditor {
    pub open: bool,
    source: String,
    output: String,
    pending: Option<u64>, // originating document, retained across tab switches
    importing: bool,
    /// This frame's widget rects (egui points) for the automation host; empty
    /// while the window is closed.
    hits: HashMap<String, egui::Rect>,
    /// The window's outer rect this frame, `None` while it is closed.
    rect: Option<egui::Rect>,
    /// The surface the window may fill (`ctx.content_rect()`), for the host.
    surface: egui::Rect,
}


impl JavaScriptEditor {
    pub fn running_in(&self, document: u64) -> bool {
        self.pending == Some(document)
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
            importing: false,
            hits: HashMap::new(),
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

    pub fn show(&mut self, ctx: &egui::Context, docs: &mut Documents, store: &dyn ModelStore) {
        self.hits.clear();
        self.rect = None;
        self.surface = ctx.content_rect();
        let mut open = self.open;
        let shown = egui::Window::new("JavaScript editor").id(egui::Id::new(WINDOW_ID)).open(&mut open)
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
                if run.clicked() {
                    match docs.engine_mut().run_javascript(&self.source) {
                        Ok(_) => { self.pending = Some(docs.active_id()); self.output = "Running…".into(); }
                        Err(e) => self.output = e,
                    }
                }
                let cancel = ui.add_enabled(self.pending.is_some(), egui::Button::new("Cancel"));
                self.hit("cancel", &cancel);
                if cancel.clicked() {
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
                if store.supports_file_interchange() {
                    let open = ui.add_enabled(!self.importing, egui::Button::new("Open .js…"));
                    self.hit("open", &open);
                    if open.clicked() {
                        match store.begin_import_filtered(("JavaScript", &["js"])) {
                            Ok(()) => self.importing = true, Err(e) => self.output = e,
                        }
                    }
                    let export = ui.button("Export .js…");
                    self.hit("export", &export);
                    if export.clicked() {
                        if let Err(e) = store.export_file_named("script.js", &self.source) { self.output = e; }
                    }
                }
            });
            let help = ui.collapsing("CAD API and examples", |ui| {
                ui.label("Write JavaScript statements. cad.document is a snapshot of the current history; cad.selection contains the current selection.");
                ui.monospace("cad.document.addFeature(type, params)\ncad.document.updateFeature(id, params)\ncad.document.deleteFeature(id)\nconsole.log / info / warn / error(...)\nui.notify(text)\nreturn { any: 'JSON result' };");
                ui.label("Built-in types include P.CU (sizeX, sizeY, sizeZ), P.S (radius), and P.CY (radius, height). Successful changes form one undo step. Each Run executes the script again.");
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

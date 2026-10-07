//! Explicit installation and host-only acquisition. Document opening never installs code.
use crate::store::{ModelStore, PLUGINS_KEY};
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::{cell::RefCell, rc::Rc, sync::mpsc::Receiver};

const MAX_BUNDLE_BYTES: usize = 16 * 1024 * 1024;

#[derive(Clone, Serialize, Deserialize)]
pub struct InstalledBundle {
    /// Original UTF-8 bytes, preserved for portable export and offline reopening.
    pub bytes: String,
    pub enabled: bool,
    pub id: String,
    pub digest: String,
    pub version: String,
    pub source: Option<String>,
}

pub type SharedPackages = Rc<RefCell<Vec<InstalledBundle>>>;

pub fn restore(engine: &mut EngineState, packages: &[InstalledBundle]) -> Result<u64, String> {
    let entries: Result<Vec<_>, String> = packages
        .iter()
        .map(|p| {
            let bundle: brep_plugins::PackageBundle =
                serde_json::from_str(&p.bytes).map_err(|e| e.to_string())?;
            let pin = bundle.pin()?;
            if pin.id != p.id || pin.version != p.version || pin.digest != p.digest {
                return Err(format!(
                    "Stored plugin {} content does not match its pinned identity",
                    p.id
                ));
            }
            Ok(json!({"bundle": bundle, "enabled": p.enabled}))
        })
        .collect();
    engine.submit_plugin_installations(Value::Array(entries?))
}

/// Parse only the portable container here; runtime validates modules and registrations.
pub fn parse_bundle(bytes: &[u8]) -> Result<(String, Value), String> {
    if bytes.len() > MAX_BUNDLE_BYTES {
        return Err("Plugin bundle exceeds 16 MiB".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|e| format!("Bundle must be UTF-8: {e}"))?;
    let value: Value =
        serde_json::from_str(text).map_err(|e| format!("Invalid plugin bundle JSON: {e}"))?;
    if !value["manifest"].is_object() || !value["modules"].is_object() {
        return Err("Expected a bundle containing manifest and modules objects".into());
    }
    Ok((text.to_owned(), value))
}

/// GitHub blob links resolve their ref through the commits API before downloading.
/// Direct HTTP(S) bundles are pinned by their validated content digest.
#[derive(Debug, PartialEq)]
enum Acquisition {
    Direct(String),
    Github {
        owner: String,
        repo: String,
        revision: String,
        path: String,
    },
}
fn acquisition(url: &str) -> Result<Acquisition, String> {
    let url = url.trim();
    if let Some(rest) = url.strip_prefix("https://github.com/") {
        let parts: Vec<_> = rest.split('/').collect();
        if parts.len() < 5
            || parts[2] != "blob"
            || parts
                .iter()
                .any(|p| p.is_empty() || *p == ".." || p.contains(['?', '#', '\\']))
        {
            return Err("Use a GitHub blob URL to a .brep-plugin.json bundle (refs containing / require an immutable commit URL)".into());
        }
        return Ok(Acquisition::Github {
            owner: parts[0].into(),
            repo: parts[1].into(),
            revision: parts[3].into(),
            path: parts[4..].join("/"),
        });
    }
    if !(url.starts_with("https://") || url.starts_with("http://")) {
        return Err("Enter an HTTP(S) bundle URL or GitHub blob URL".into());
    }
    Ok(Acquisition::Direct(url.into()))
}

struct Download {
    rx: Receiver<Result<Vec<u8>, String>>,
    source: String,
    github: Option<(String, String, String)>,
}

pub struct PluginsPanel {
    pub open: bool,
    pub packages: SharedPackages,
    pub error: Option<String>,
    pub message: Option<String>,
    pub(crate) storage_error: Option<String>,
    url: String,
    #[cfg(not(target_arch = "wasm32"))]
    local_path: String,
    pasted: String,
    download: Option<Download>,
    importing: bool,
    action: Option<ActionForm>,
    panel_params: std::collections::HashMap<String, Value>,
    panel_hits: std::collections::HashMap<String, egui::Rect>,
    ui_package_stamp: String,
    synced: std::collections::HashMap<u64, String>,
    pub(crate) active_document: u64,
    pending_install: Option<(u64, u64, Vec<InstalledBundle>)>,
}
struct ActionForm {
    id: String,
    package_digest: Value,
    label: String,
    schema: Value,
    params: Value,
    error: Option<String>,
    pending: bool,
}

impl PluginsPanel {
    pub fn new(store: &dyn ModelStore) -> Self {
        let (packages, error) = match store.read(PLUGINS_KEY) {
            Some(text) => match serde_json::from_str::<Vec<InstalledBundle>>(&text) {
                Ok(v) => (v, None),
                Err(e) => (
                    vec![],
                    Some(format!("Installed plugins could not be read: {e}")),
                ),
            },
            None => (vec![], None),
        };
        Self {
            open: false,
            packages: Rc::new(RefCell::new(packages)),
            error,
            message: None,
            storage_error: None,
            url: String::new(),
            #[cfg(not(target_arch = "wasm32"))]
            local_path: String::new(),
            pasted: String::new(),
            download: None,
            importing: false,
            action: None,
            panel_params: Default::default(),
            panel_hits: Default::default(),
            ui_package_stamp: String::new(),
            synced: Default::default(),
            active_document: 0,
            pending_install: None,
        }
    }

    pub fn sync_documents(&mut self, docs: &mut crate::document::Documents) {
        let packages = self.packages.borrow();
        let stamp = packages
            .iter()
            .map(|p| format!("{}:{}:{}", p.id, p.digest, p.enabled))
            .collect::<Vec<_>>()
            .join("|");
        if self.ui_package_stamp != stamp { self.panel_params.clear(); self.action = None; self.ui_package_stamp = stamp.clone(); }
        if self.active_document != docs.active_id() { self.panel_params.clear(); self.action = None; }
        self.active_document = docs.active_id();
        self.synced
            .retain(|id, _| docs.iter().any(|d| d.id() == *id));
        for doc in docs.iter_mut() {
            if self
                .pending_install
                .as_ref()
                .is_some_and(|p| p.0 == doc.id())
            {
                continue;
            }
            let status = doc.engine.plugin_installation_status();
            if status["state"] == "error" {
                self.error = status["error"].as_str().map(str::to_owned);
            }
            if status["state"] == "pending" {
                continue;
            }
            if self.synced.get(&doc.id()) != Some(&stamp) {
                let expected: Vec<_> = packages
                    .iter()
                    .map(|p| json!([p.id, p.digest, p.enabled]))
                    .collect();
                let actual: Vec<_> = doc
                    .engine
                    .installed_plugins()
                    .as_array()
                    .into_iter()
                    .flatten()
                    .map(|p| json!([p["id"], p["digest"], p["enabled"]]))
                    .collect();
                if expected == actual {
                    self.synced.insert(doc.id(), stamp.clone());
                    continue;
                }
                match restore(&mut doc.engine, &packages) {
                    Ok(_) => {
                        self.synced.insert(doc.id(), stamp.clone());
                    }
                    Err(e) => {
                        self.error = Some(e);
                        self.synced.insert(doc.id(), stamp.clone());
                    }
                }
            }
        }
    }

    pub fn close_action(&mut self) {
        self.action = None;
    }

    fn persist(&self, store: &dyn ModelStore, next: &[InstalledBundle]) -> Result<(), String> {
        store.write(
            PLUGINS_KEY,
            &serde_json::to_string(next).map_err(|e| e.to_string())?,
        )
    }

    pub fn install(
        &mut self,
        engine: &mut EngineState,
        _store: &dyn ModelStore,
        bytes: &[u8],
        source: Option<String>,
    ) -> Result<u64, String> {
        let (bytes, bundle) = parse_bundle(bytes)?;
        let typed: brep_plugins::PackageBundle =
            serde_json::from_value(bundle).map_err(|e| e.to_string())?;
        let pin = typed.pin()?;
        let mut next = self.packages.borrow().clone();
        next.retain(|p| !(p.id == pin.id && p.digest == pin.digest));
        next.push(InstalledBundle {
            bytes,
            enabled: true,
            id: pin.id,
            digest: pin.digest,
            version: pin.version,
            source,
        });
        self.queue_install(engine, next)
    }

    fn queue_install(
        &mut self,
        engine: &mut EngineState,
        next: Vec<InstalledBundle>,
    ) -> Result<u64, String> {
        if self.pending_install.is_some() {
            return Err("A plugin installation is already pending".into());
        }
        let id = restore(engine, &next)?;
        self.pending_install = Some((self.active_document, id, next));
        self.error = None;
        self.storage_error = None;
        self.message = Some("Validating plugin registrations in the history worker…".into());
        Ok(id)
    }

    pub fn finish_installations(
        &mut self,
        docs: &mut crate::document::Documents,
        store: &dyn ModelStore,
    ) {
        let Some((owner, id, _)) = &self.pending_install else {
            return;
        };
        let Some(doc) = docs.iter_mut().find(|d| d.id() == *owner) else {
            self.pending_install = None;
            self.error = Some("Installation cancelled because its document was closed".into());
            return;
        };
        let status = doc.engine.plugin_installation_status();
        if status["id"].as_u64() != Some(*id) {
            return;
        }
        match status["state"].as_str() {
            Some("complete") => {
                let (_, _, next) = self.pending_install.take().unwrap();
                match self.persist(store, &next) {
                    Ok(()) => {
                        *self.packages.borrow_mut() = next;
                        self.message = Some(
                            "Installed exact package bytes. Existing document pins stay unchanged."
                                .into(),
                        );
                    }
                    Err(e) => {
                        self.storage_error = Some(e.clone());
                        self.error = Some(e);
                        let _ = restore(&mut doc.engine, &self.packages.borrow());
                    }
                }
            }
            Some("error") => {
                self.error = Some(
                    status["error"]
                        .as_str()
                        .unwrap_or("Plugin installation failed")
                        .into(),
                );
                self.pending_install = None;
            }
            _ => {}
        }
    }

    pub fn open_action(&mut self, engine: &EngineState, id: &str) -> Result<(), String> {
        if self.action.as_ref().is_some_and(|a| a.pending) {
            return Err("Wait for or cancel the pending action before opening another".into());
        }
        let catalogue = engine.plugin_catalogue();
        let action = catalogue["actions"]
            .as_array()
            .and_then(|a| a.iter().find(|a| a["id"] == id))
            .ok_or_else(|| format!("Plugin action {id} is unavailable"))?;
        let schema = action["inputParamsSchema"].clone();
        let mut params = json!({});
        if let Some(fields) = schema.as_object() {
            for (key, field) in fields {
                if let Some(default) = field.get("default_value") {
                    params[key] = default.clone();
                }
            }
        }
        self.action = Some(ActionForm {
            id: id.into(),
            package_digest: action["package"]["digest"].clone(),
            label: action["ribbonPath"].as_str().and_then(|p| p.rsplit('/').next()).or(action["label"].as_str()).unwrap_or(id).into(),
            schema,
            params,
            error: None,
            pending: false,
        });
        Ok(())
    }

    pub fn poll(&mut self, ctx: &egui::Context, engine: &mut EngineState, store: &dyn ModelStore) {
        if self.importing {
            if let Some(file) = store.take_import() {
                self.importing = false;
                if let Err(e) = self.install(engine, store, &file.bytes, None) {
                    self.error = Some(e);
                }
            }
        }
        let result = self.download.as_ref().and_then(|d| d.rx.try_recv().ok());
        if let Some(result) = result {
            let download = self.download.take().unwrap();
            match result {
                Err(e) => self.error = Some(e),
                Ok(bytes) => {
                    if let Some((owner, repo, path)) = download.github {
                        let commit = serde_json::from_slice::<Value>(&bytes)
                            .ok()
                            .and_then(|v| v["sha"].as_str().map(str::to_owned));
                        match commit.filter(|sha| {
                            sha.len() == 40 && sha.bytes().all(|c| c.is_ascii_hexdigit())
                        }) {
                            Some(sha) => {
                                let source = format!(
                                    "https://raw.githubusercontent.com/{owner}/{repo}/{sha}/{path}"
                                );
                                self.download = Some(Download {
                                    rx: crate::http::fetch_bytes(ctx, source.clone()),
                                    source,
                                    github: None,
                                });
                            }
                            None => {
                                self.error =
                                    Some("GitHub did not return an immutable commit SHA".into())
                            }
                        }
                    } else if let Err(e) =
                        self.install(engine, store, &bytes, Some(download.source))
                    {
                        self.error = Some(e);
                    }
                }
            }
        }
        if let Some(result) = engine.poll_plugin_action() {
            match result {
                Ok(value) => {
                    self.message = Some(value.to_string());
                    if let Some(form) = &mut self.action {
                        form.pending = false;
                    }
                }
                Err(e) => {
                    self.error = Some(e.clone());
                    if let Some(form) = &mut self.action {
                        form.error = Some(e);
                        form.pending = false;
                    }
                }
            }
        }
        if self.download.is_some()
            || self.pending_install.is_some()
            || engine.plugin_installation_status()["state"] == "pending"
            || self.action.as_ref().is_some_and(|a| a.pending)
        {
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }
    }

    pub fn show(&mut self, ctx: &egui::Context, engine: &mut EngineState, store: &dyn ModelStore) {
        let mut open = self.open;
        egui::Window::new("Plugins").open(&mut open).default_width(560.0).show(ctx, |ui| {
            ui.label("Install trusted JavaScript package bundles. Code runs with the CAD host API; installation is separate from opening a document.");
            if let Some(error) = &self.error { ui.colored_label(egui::Color32::LIGHT_RED, error); }
            if let Some(message) = &self.message { ui.label(message); }
            if engine.plugin_installation_status()["state"] == "pending" { ui.spinner(); ui.label("Validating package registrations…"); }
            for error in engine.plugin_catalogue()["errors"].as_array().into_iter().flatten() { if let Some(error) = error.as_str() { ui.colored_label(egui::Color32::LIGHT_RED, error); } }
            ui.horizontal(|ui| {
                ui.text_edit_singleline(&mut self.url);
                if ui.add_enabled(self.download.is_none(), egui::Button::new("Install URL / GitHub")).clicked() {
                    match acquisition(&self.url) {
                        Ok(Acquisition::Direct(source)) => self.download = Some(Download { rx: crate::http::fetch_bytes(ctx, source.clone()), source, github: None }),
                        Ok(Acquisition::Github { owner, repo, revision, path }) => { let source = format!("https://api.github.com/repos/{owner}/{repo}/commits/{revision}"); self.download = Some(Download { rx: crate::http::fetch_bytes(ctx, source.clone()), source, github: Some((owner, repo, path)) }); },
                        Err(e) => self.error = Some(e),
                    }
                }
            });
            if self.download.is_some() { ui.spinner(); ui.label("Acquiring pinned package bytes…"); }
            if store.supports_file_interchange() && ui.button("Import local bundle…").clicked() {
                match store.begin_import_filtered(("Plugin bundle", &["json"])) { Ok(()) => self.importing = true, Err(e) => self.error = Some(e) }
            }
            #[cfg(not(target_arch = "wasm32"))]
            ui.horizontal(|ui| { ui.label("Local bundle path"); ui.text_edit_singleline(&mut self.local_path); if ui.button("Import").clicked() { let result = std::fs::read(&self.local_path).map_err(|e| e.to_string()).and_then(|bytes| self.install(engine, store, &bytes, None)); if let Err(e) = result { self.error = Some(e); } } });
            ui.collapsing("Paste portable bundle JSON", |ui| { ui.add(egui::TextEdit::multiline(&mut self.pasted).code_editor().desired_rows(6)); if ui.button("Install pasted bundle").clicked() { let bytes = self.pasted.as_bytes().to_vec(); if let Err(e) = self.install(engine, store, &bytes, None) { self.error = Some(e); } } });
            ui.separator();
            let installed = self.packages.borrow().clone();
            for package in &installed {
                ui.group(|ui| {
                    ui.label(format!("{}  {}", package.id, package.version));
                    ui.small(format!("SHA-256 {}", package.digest));
                    if let Some(source) = &package.source { ui.small(source); }
                    ui.horizontal_wrapped(|ui| {
                        let mut enabled = package.enabled;
                        if ui.checkbox(&mut enabled, "Enabled").changed() { self.change(engine, store, package, Some(enabled)); }
                        if ui.button("Use in document").clicked() { if let Err(e) = engine.select_plugin(&package.id, &package.digest) { self.error = Some(e); } }
                        if ui.button("Migrate document").clicked() { if let Err(e) = engine.migrate_plugin(&package.id, &package.digest) { self.error = Some(e); } }
                        if ui.button("Reload stored bytes").clicked() { if let Err(e) = self.queue_install(engine, installed.clone()) { self.error = Some(e); } }
                        if ui.button("Export").clicked() { if let Err(e) = store.export_file_named(&format!("{}-{}.brep-plugin.json", package.id.replace('/', "_"), package.version), &package.bytes) { self.error = Some(e); } }
                        if ui.button("Remove").clicked() { self.change(engine, store, package, None); }
                    });
                });
            }
            ui.small("Updates keep old bundles. Reload revalidates stored bytes; import again to load edited source. Panels appear in their claimed workbench; typed annotations are authored in PMI.");
        });
        self.open = open;
        self.show_action(ctx, engine);
    }

    pub(crate) fn change(
        &mut self,
        engine: &mut EngineState,
        _store: &dyn ModelStore,
        package: &InstalledBundle,
        enabled: Option<bool>,
    ) {
        let result = (|| {
            let mut next = self.packages.borrow().clone();
            if let Some(enabled) = enabled {
                for p in &mut next {
                    if p.id == package.id && p.digest == package.digest {
                        p.enabled = enabled;
                    }
                }
            } else {
                next.retain(|p| !(p.id == package.id && p.digest == package.digest));
            }
            self.queue_install(engine, next)?;
            self.action = None;
            Ok::<_, String>(())
        })();
        if let Err(e) = result {
            self.error = Some(e);
        }
    }

    pub fn clear_panel_hits(&mut self) { self.panel_hits.clear(); }
    pub fn panel_hits_json(&self) -> String { crate::automation::hit_rects::hits_json(&self.panel_hits) }

    /// Draw validated static controls. Only button clicks submit worker actions.
    pub fn show_panel(&mut self, ui: &mut egui::Ui, engine: &mut EngineState, id: &str) {
        let catalogue = engine.plugin_catalogue();
        let Some(panel) = catalogue["panels"].as_array().into_iter().flatten().find(|p| p["id"] == id) else {
            ui.label("Panel owner is unavailable for this document."); return;
        };
        let pending = engine.plugin_action_status()["state"] == "pending";
        egui::ScrollArea::vertical().id_salt(("plugin-panel", id)).show(ui, |ui| {
            for (index, control) in panel["controls"].as_array().into_iter().flatten().enumerate() {
                ui.push_id((id, index), |ui| {
                    match control["type"].as_str().unwrap_or("") {
                        "text" => { ui.label(control["text"].as_str().unwrap_or("")); }
                        "table" => { egui::Grid::new("table").striped(true).show(ui, |ui| {
                            for column in control["columns"].as_array().into_iter().flatten() { ui.strong(column.as_str().unwrap_or("")); } ui.end_row();
                            for row in control["rows"].as_array().into_iter().flatten() {
                                for cell in row.as_array().into_iter().flatten() { ui.label(cell.as_str().unwrap_or("")); } ui.end_row();
                            }
                        }); }
                        "action" | "form" => {
                            let action_id = control["action"].as_str().unwrap_or("");
                            let Some(action) = catalogue["actions"].as_array().into_iter().flatten().find(|a| a["id"] == action_id) else {
                                ui.label("Action owner unavailable"); return;
                            };
                            let key = format!("{}:{}:{}", panel["package"]["digest"], id, control["id"]);
                            let params = self.panel_params.entry(key).or_insert_with(|| {
                                let mut params = schema_defaults(&action["inputParamsSchema"]);
                                if let Some(overrides) = control["params"].as_object() { params.as_object_mut().unwrap().extend(overrides.clone()); } params
                            });
                            ui.add_enabled_ui(!pending && engine.history.locked().is_none(), |ui| {
                                if control["type"] == "form" { schema_fields(ui, engine, &action["inputParamsSchema"], params); }
                                let button = ui.button(control["label"].as_str().unwrap_or(action_id));
                                self.panel_hits.insert(format!("panel:{id}:{}", control["id"].as_str().unwrap_or("")), button.rect);
                                if button.clicked() {
                                    let selection = serde_json::from_str(&engine.selection_json()).unwrap_or(Value::Null);
                                    match engine.plugin_action(action_id, params.clone(), selection) {
                                        Ok(_) => { self.error = None; ui.ctx().request_repaint(); }
                                        Err(e) => self.error = Some(e),
                                    }
                                }
                            });
                        }
                        _ => { ui.label("Unsupported panel control"); }
                    }
                });
            }
            if pending { ui.spinner(); ui.ctx().request_repaint_after(std::time::Duration::from_millis(30)); }
            if let Some(error) = &self.error { ui.colored_label(egui::Color32::LIGHT_RED, error); }
        });
    }

    fn show_action(&mut self, ctx: &egui::Context, engine: &mut EngineState) {
        let Some(form) = &mut self.action else { return };
        if !engine.plugin_catalogue()["actions"].as_array().into_iter().flatten().any(|a| a["id"] == form.id && a["package"]["digest"] == form.package_digest) {
            self.action = None; return;
        }
        let mut open = true;
        egui::Window::new(&form.label)
            .id(egui::Id::new("plugin-action-form"))
            .open(&mut open)
            .show(ctx, |ui| {
                ui.small(&form.id);
                if let Some(error) = &form.error {
                    ui.colored_label(egui::Color32::LIGHT_RED, error);
                }
                ui.add_enabled_ui(!form.pending, |ui| {
                    schema_fields(ui, engine, &form.schema, &mut form.params);
                    if ui.button("Run action").clicked() {
                        match engine.plugin_action(
                            &form.id,
                            form.params.clone(),
                            serde_json::from_str(&engine.selection_json()).unwrap_or(Value::Null),
                        ) {
                            Ok(_) => {
                                form.pending = true;
                                form.error = None;
                            }
                            Err(e) => form.error = Some(e),
                        }
                    }
                });
                if form.pending {
                    ui.spinner();
                    if ui.button("Cancel").clicked() {
                        engine.cancel_plugin_action();
                        form.pending = false;
                    }
                }
            });
        if !open {
            if form.pending {
                engine.cancel_plugin_action();
            }
            self.action = None;
        }
    }
}

pub(crate) fn schema_defaults(schema: &Value) -> Value {
    let mut params = json!({});
    for (key, field) in schema.as_object().into_iter().flatten() {
        if let Some(default) = field.get("default_value") { params[key] = default.clone(); }
    }
    params
}

pub(crate) fn schema_fields(ui: &mut egui::Ui, engine: &EngineState, schema: &Value, params: &mut Value) {
    let entry = json!({"inputParamsSchema": schema});
    let fields = brep_render::features::plugin_form_fields_from_schema(&entry);
    let selection: Value = serde_json::from_str(&engine.selection_json()).unwrap_or(Value::Null);
    for field in &fields {
        ui.label(&field.label);
        let mut actions = crate::form::FieldActions::default();
        crate::form::field_input(ui, field, params, None, &mut actions);
        if actions.clicked.is_some() {
            if let brep_render::style::FieldKind::Reference { filter, multiple } = &field.kind {
                let names: Vec<Value> = filter.iter().flat_map(|kind| {
                    let key = match kind.to_ascii_lowercase().as_str() {
                        "solid" => "solids", "face" => "faces", "edge" => "edges", "datum" => "datums", _ => "",
                    };
                    selection[key].as_array().into_iter().flatten().cloned()
                }).collect();
                params[field.key()] = if *multiple { Value::Array(names) } else { names.into_iter().next().unwrap_or(Value::Null) };
            }
        }
        if matches!(&field.kind, brep_render::style::FieldKind::Reference { .. }) { ui.small("Select uses the current viewport selection."); }
    }
}

pub fn feature_label(engine: &EngineState, ty: &str) -> String {
    if !ty.contains('/') {
        return brep_render::features::feature_plain_name(ty);
    }
    engine
        .feature_schema(ty)
        .and_then(|s| s["longName"].as_str().map(str::to_owned))
        .unwrap_or_else(|| ty.to_owned())
}


pub const HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[crate::automation::hit_keys::HitKeyDoc { panel: "plugins", prefix: "panel:", meaning: "submit the named action for a declarative panel control", command: Some("plugin_action") }];

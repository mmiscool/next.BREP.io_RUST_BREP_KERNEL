//! Trusted package installations and document-scoped, asynchronous transactions.
use super::EngineState;
use crate::history::History;
use brep_plugins::{PackageBundle, PluginPin, Runtime};
use serde_json::{json, Value};
use std::collections::VecDeque;

#[derive(Default)]
pub(super) struct Plugins {
    packages: Vec<(PackageBundle, bool)>,
    runtime: Option<Runtime>,
    revision: u64,
    next_action: u64,
    pending: Option<PendingAction>,
    results: VecDeque<Result<Value, String>>,
    status: Value,
    install_pending: Option<PendingInstall>,
    install_status: Value,
    deferred_run: bool,
}

struct PendingInstall {
    id: u64,
    revision: u64,
    packages: Vec<(PackageBundle, bool)>,
}

fn parse_installations(packages: Value) -> Result<Vec<(PackageBundle, bool)>, String> {
    let rows = packages
        .as_array()
        .ok_or("installed plugins must be an array")?;
    let mut installed = Vec::new();
    let mut seen = std::collections::BTreeSet::new();
    for row in rows {
        let bundle: PackageBundle = serde_json::from_value(row["bundle"].clone())
            .map_err(|e| format!("plugin bundle: {e}"))?;
        let pin = bundle.pin()?;
        if !seen.insert((pin.id.clone(), pin.digest)) {
            return Err(format!("duplicate installed package {}", pin.id));
        }
        let enabled = row["enabled"]
            .as_bool()
            .ok_or("plugin enabled must be boolean")?;
        installed.push((bundle, enabled));
    }
    Ok(installed)
}

struct PendingAction {
    id: u64,
    revision: u64,
    registry_revision: u64,
    selection: String,
    generation: u64,
    pins: Vec<PluginPin>,
    staged: Option<History>,
    notifications: Vec<String>,
    annotation_id: Option<String>,
    allowed_annotation_errors: Vec<AnnotationFailure>,
}

impl Plugins {
    fn bundles(&self) -> Vec<PackageBundle> {
        self.packages
            .iter()
            .filter(|(_, enabled)| *enabled)
            .map(|(p, _)| p.clone())
            .collect()
    }
    fn rebuild(&mut self, packages: Vec<(PackageBundle, bool)>) -> Result<(), String> {
        let enabled = packages
            .iter()
            .filter(|(_, enabled)| *enabled)
            .map(|(p, _)| p.clone())
            .collect();
        let runtime = Runtime::new(enabled)?;
        self.packages = packages;
        self.runtime = Some(runtime);
        self.revision = self.revision.wrapping_add(1);
        Ok(())
    }
}

impl EngineState {
    /// Restore persisted installations in one atomic registry revision.
    pub fn restore_plugins(&mut self, packages: Value) -> Result<(), String> {
        self.plugins.rebuild(parse_installations(packages)?)?;
        self.plugins_changed();
        Ok(())
    }

    /// Submit a complete installation set to the runner for atomic validation.
    /// Does not enable code or change an existing registry before the reply succeeds.
    pub fn submit_plugin_installations(&mut self, installations: Value) -> Result<u64, String> {
        if self.plugins.install_pending.is_some() {
            return Err("plugin installation already pending".into());
        }
        let packages = parse_installations(installations)?;
        self.plugins.next_action += 1;
        let id = self.plugins.next_action;
        let enabled: Vec<_> = packages
            .iter()
            .filter(|(_, enabled)| *enabled)
            .map(|(p, _)| p)
            .collect();
        if !self
            .runner
            .submit_plugin_install(crate::runner::PluginInstallRequest {
                id,
                packages: json!(enabled),
            })
        {
            return Err("runner does not support asynchronous plugin installation".into());
        }
        self.plugins.install_pending = Some(PendingInstall {
            id,
            revision: self.plugins.revision,
            packages,
        });
        self.plugins.install_status = json!({"state":"pending","id":id});
        self.pump();
        Ok(id)
    }

    pub(super) fn defer_plugin_run(&mut self) -> bool {
        if self.plugins.install_pending.is_none() {
            return false;
        }
        self.plugins.deferred_run = true;
        self.run_generation += 1;
        true
    }
    pub(super) fn plugin_work_pending(&self) -> bool {
        self.plugins.install_pending.is_some() || self.plugins.pending.is_some()
    }

    pub(super) fn plugin_installation_pending(&self) -> bool {
        self.plugins.install_pending.is_some()
    }

    pub(super) fn plugin_runner_replaced(&mut self) {
        if self.plugins.pending.take().is_some() {
            self.plugin_result(Err(
                "plugin transaction cancelled because runner changed".into()
            ));
        }
        if let Some(pending) = &self.plugins.install_pending {
            let enabled: Vec<_> = pending
                .packages
                .iter()
                .filter(|(_, enabled)| *enabled)
                .map(|(p, _)| p)
                .collect();
            if !self
                .runner
                .submit_plugin_install(crate::runner::PluginInstallRequest {
                    id: pending.id,
                    packages: json!(enabled),
                })
            {
                self.plugins.install_pending = None;
                self.plugins.install_status =
                    json!({"state":"error","error":"replacement runner cannot validate plugins"});
            }
        }
    }

    pub fn plugin_installation_status(&self) -> Value {
        self.plugins.install_status.clone()
    }

    pub(super) fn pump_plugin_installations(&mut self) {
        while let Some(reply) = self.runner.poll_plugin_install() {
            if !self
                .plugins
                .install_pending
                .as_ref()
                .is_some_and(|p| p.id == reply.id)
            {
                continue;
            }
            let pending = self.plugins.install_pending.take().unwrap();
            let result = if pending.revision != self.plugins.revision {
                Err("plugin installations changed during validation".to_string())
            } else {
                reply.result.and_then(|registry| {
                    let enabled = pending
                        .packages
                        .iter()
                        .filter(|(_, enabled)| *enabled)
                        .map(|(p, _)| p.clone())
                        .collect();
                    Runtime::from_validated(enabled, registry)
                })
            };
            let deferred = std::mem::take(&mut self.plugins.deferred_run);
            match result {
                Ok(runtime) => {
                    self.plugins.runtime = Some(runtime);
                    self.plugins.packages = pending.packages;
                    self.plugins.revision = self.plugins.revision.wrapping_add(1);
                    self.plugins.install_status = json!({"state":"complete","id":pending.id});
                    self.plugins_changed();
                }
                Err(error) => {
                    self.plugins.install_status =
                        json!({"state":"error","id":pending.id,"error":error});
                    if deferred {
                        self.rerun_history();
                    }
                }
            }
        }
    }

    /// Restore explicitly installed/enabled packages, independently of loading a document.
    pub fn set_plugin_packages(&mut self, packages: Vec<PackageBundle>) -> Result<(), String> {
        self.plugins
            .rebuild(packages.into_iter().map(|p| (p, true)).collect())?;
        self.plugins_changed();
        Ok(())
    }

    pub fn install_plugin(&mut self, package: Value) -> Result<Value, String> {
        let package: PackageBundle =
            serde_json::from_value(package).map_err(|e| format!("plugin package: {e}"))?;
        let pin = package.pin()?;
        let mut packages = self.plugins.packages.clone();
        packages.retain(|(p, _)| p.pin().ok().as_ref() != Some(&pin));
        packages.push((package, true));
        self.plugins.rebuild(packages)?;
        self.plugins_changed();
        Ok(json!(pin))
    }

    pub fn installed_plugins(&self) -> Value {
        Value::Array(
            self.plugins
                .packages
                .iter()
                .filter_map(|(p, enabled)| {
                    p.pin().ok().map(|pin| {
                        json!({"id":pin.id,"version":pin.version,"apiVersion":pin.api_version,
                "digest":pin.digest,"name":p.manifest.name,"enabled":enabled})
                    })
                })
                .collect(),
        )
    }

    pub fn export_plugin(&self, id: &str, digest: &str) -> Result<Value, String> {
        let (package, _) = self
            .plugins
            .packages
            .iter()
            .find(|(p, _)| {
                p.pin()
                    .is_ok_and(|pin| pin.id == id && pin.digest == digest)
            })
            .ok_or_else(|| format!("plugin {id} at {digest} is not installed"))?;
        serde_json::to_value(package).map_err(|e| e.to_string())
    }

    pub fn enable_plugin(&mut self, id: &str, digest: &str, enabled: bool) -> Result<(), String> {
        let mut packages = self.plugins.packages.clone();
        let (_, state) = packages
            .iter_mut()
            .find(|(p, _)| {
                p.pin()
                    .is_ok_and(|pin| pin.id == id && pin.digest == digest)
            })
            .ok_or_else(|| format!("plugin {id} at {digest} is not installed"))?;
        *state = enabled;
        self.plugins.rebuild(packages)?;
        self.plugins_changed();
        Ok(())
    }

    pub fn remove_plugin(&mut self, id: &str, digest: &str) -> Result<(), String> {
        let mut packages = self.plugins.packages.clone();
        let before = packages.len();
        packages.retain(|(p, _)| {
            !p.pin()
                .is_ok_and(|pin| pin.id == id && pin.digest == digest)
        });
        if packages.len() == before {
            return Err(format!("plugin {id} at {digest} is not installed"));
        }
        self.plugins.rebuild(packages)?;
        self.plugins_changed();
        Ok(())
    }

    fn plugins_changed(&mut self) {
        self.cancel_plugin_action();
        self.runner.reset();
        self.rerun_history();
    }

    fn saved_plugin_pins(&self) -> Result<Vec<PluginPin>, String> {
        serde_json::from_value(
            self.history
                .document_block("plugins")
                .cloned()
                .unwrap_or_else(|| json!([])),
        )
        .map_err(|e| format!("document plugin pins: {e}"))
    }

    /// Existing document pins always win; an unpinned package uses the last explicit install.
    fn available_plugin_pins(&self) -> Result<Vec<PluginPin>, String> {
        let mut pins = self.saved_plugin_pins()?;
        for (package, enabled) in self.plugins.packages.iter().rev() {
            if !enabled {
                continue;
            }
            let pin = package.pin()?;
            if !pins.iter().any(|p| p.id == pin.id) {
                pins.push(pin);
            }
        }
        Ok(pins)
    }

    pub fn plugin_catalogue(&self) -> Value {
        let mut features = Vec::new();
        let mut actions = Vec::new();
        let mut workbenches = Vec::new();
        let mut panels = Vec::new();
        let mut annotations = Vec::new();
        let mut errors = Vec::new();
        if let Some(runtime) = &self.plugins.runtime {
            match self.available_plugin_pins() {
                Ok(pins) => {
                    for pin in pins {
                        if self.installed_pin(&pin.id, &pin.digest).as_ref() != Ok(&pin) {
                            errors.push(format!(
                                "plugin {} at {} is missing or disabled",
                                pin.id, pin.digest
                            ));
                            continue;
                        }
                        // One unavailable dependency must not hide healthy package controls.
                        match runtime.registry().for_document(&[pin]) {
                            Ok(registry) => {
                                features.extend(registry.features().iter().map(|f| {
                                let mut schema = f.input_params_schema.clone();
                                schema["id"] = json!({"type":"string","default_value":null});
                                json!({"type":f.id,"id":f.id,"label":f.label,"longName":f.label,"ribbonPath":f.ribbon_path,"commandSize":f.command_size,"icon":f.glyph,
                                    "shortName":if f.short_name.is_empty() { f.id.rsplit('/').next().unwrap_or("Plugin") } else { &f.short_name },
                                    "inputParamsSchema":schema,"package":f.package})
                            }));
                                panels.extend(registry.panels().iter().map(|p| json!(p)));
                                annotations.extend(registry.annotations().iter().map(|a| {
                                    let mut schema = a.input_params_schema.clone();
                                    schema["id"] = json!({"type":"string","default_value":null});
                                    json!({"type":a.id,"id":a.id,"label":a.label,"longName":a.label,
                                        "shortName":"ANN","inputParamsSchema":schema,"package":a.package})
                                }));
                                actions.extend(registry.actions().iter().map(|a| json!(a)));
                                workbenches.extend(registry.workbenches().iter().map(|w| json!(w)));
                            }
                            Err(error) => errors.push(error),
                        }
                    }
                }
                Err(error) => errors.push(error),
            }
        } else if let Ok(pins) = self.saved_plugin_pins() {
            errors.extend(pins.iter().map(|p| {
                format!(
                    "plugin {} at {} is not installed or enabled",
                    p.id, p.digest
                )
            }));
        }
        json!({"revision":self.plugins.revision,"documentRevision":self.history.revision(),
            "features":features,"actions":actions,"workbenches":workbenches,"panels":panels,"annotations":annotations,"errors":errors})
    }

    pub fn plugin_workbenches(&self) -> Value {
        self.plugin_catalogue()["workbenches"].clone()
    }

    /// Engine-local schema overlay; never adds package schemas to the process-wide built-ins.
    pub fn feature_catalogue(&self) -> Value {
        let mut catalogue = crate::features::feature_catalogue();
        if let Some(features) = catalogue["features"].as_array_mut() {
            features.extend(
                self.plugin_catalogue()["features"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .cloned(),
            );
        }
        catalogue
    }
    pub fn feature_schema(&self, feature_type: &str) -> Option<Value> {
        crate::features::feature_schema(feature_type).or_else(|| {
            self.plugin_catalogue()["features"]
                .as_array()?
                .iter()
                .find(|f| f["type"] == feature_type)
                .cloned()
        })
    }
    pub fn feature_form_fields(&self, feature_type: &str) -> Vec<crate::style::FormField> {
        if let Some(schema) = crate::features::feature_schema(feature_type) {
            return crate::features::form_fields_from_schema(&schema);
        }
        self.feature_schema(feature_type)
            .as_ref()
            .map(crate::features::plugin_form_fields_from_schema)
            .unwrap_or_default()
    }
    pub fn feature_default_params(&self, feature_type: &str) -> Value {
        let mut params = serde_json::Map::new();
        if let Some(schema) = self.feature_schema(feature_type) {
            if let Some(props) = schema["inputParamsSchema"].as_object() {
                for (name, spec) in props {
                    params.insert(name.clone(), spec["default_value"].clone());
                }
            }
        }
        Value::Object(params)
    }

    /// Selecting a first pin is undoable; replacing an existing pin requires migration.
    pub fn select_plugin(&mut self, id: &str, digest: &str) -> Result<(), String> {
        self.require_plugin_editable()?;
        let pin = self.installed_pin(id, digest)?;
        let mut pins = self.saved_plugin_pins()?;
        if let Some(old) = pins.iter().find(|p| p.id == id) {
            return if old == &pin {
                Ok(())
            } else {
                Err("use migrate_plugin to validate a version change".into())
            };
        }
        pins.push(pin);
        self.history
            .set_document_block("plugins", Some(json!(pins)), None);
        self.rerun_history();
        Ok(())
    }

    fn installed_pin(&self, id: &str, digest: &str) -> Result<PluginPin, String> {
        self.plugins
            .packages
            .iter()
            .filter(|(_, enabled)| *enabled)
            .find_map(|(p, _)| {
                p.pin()
                    .ok()
                    .filter(|pin| pin.id == id && pin.digest == digest)
            })
            .ok_or_else(|| format!("plugin {id} at {digest} is not installed and enabled"))
    }
    fn require_plugin_editable(&self) -> Result<(), String> {
        if self.plugins.install_pending.is_some() {
            return Err("wait for plugin installation validation".into());
        }
        if let Some(reason) = self.history.locked() {
            return Err(format!("the document is read-only: {reason}"));
        }
        if self.plugins.pending.is_some() {
            return Err("a plugin transaction is already pending".into());
        }
        if self.run_pending() {
            return Err("wait for the current history recompute".into());
        }
        Ok(())
    }

    pub fn migrate_plugin(&mut self, id: &str, digest: &str) -> Result<u64, String> {
        self.require_plugin_editable()?;
        let pin = self.installed_pin(id, digest)?;
        let mut pins = self.saved_plugin_pins()?;
        pins.retain(|p| p.id != id);
        pins.push(pin);
        let pending = self.new_plugin_pending(pins.clone());
        let action_id = pending.id;
        let mut staged = self.history.clone();
        staged.set_document_block_no_undo("plugins", Some(json!(pins)));
        self.plugins.pending = Some(pending);
        if let Err(error) = self.submit_plugin_candidate(staged) {
            self.plugins.pending = None;
            return Err(error);
        }
        self.pump();
        Ok(action_id)
    }

    fn new_plugin_pending(&mut self, pins: Vec<PluginPin>) -> PendingAction {
        self.plugins.next_action += 1;
        PendingAction {
            id: self.plugins.next_action,
            revision: self.history.revision(),
            registry_revision: self.plugins.revision,
            selection: self.selection_json(),
            generation: self.run_generation,
            pins,
            staged: None,
            notifications: Vec::new(),
            annotation_id: None,
            allowed_annotation_errors: Vec::new(),
        }
    }

    pub fn submit_plugin_action(&mut self, id: &str, params: Value) -> Result<(), String> {
        let selection = serde_json::from_str(&self.selection_json()).unwrap_or(Value::Null);
        self.plugin_action(id, params, selection).map(|_| ())
    }

    /// Submit a callback; neither callback commands nor recompute touch live history.
    pub fn plugin_action(
        &mut self,
        id: &str,
        params: Value,
        selection: Value,
    ) -> Result<u64, String> {
        self.require_plugin_editable()?;
        let pins = self.available_plugin_pins()?;
        let runtime = self
            .plugins
            .runtime
            .as_ref()
            .ok_or("no plugins installed")?;
        let registry = runtime.registry().for_document(&pins)?;
        if !registry.actions().iter().any(|a| a.id == id) {
            return Err(format!("plugin action {id} is unavailable"));
        }
        let pending = self.new_plugin_pending(pins.clone());
        let action_id = pending.id;
        let input = json!({"params":params,"selection":selection,
            "document":serde_json::from_str::<Value>(&self.history.request_json()).map_err(|e| e.to_string())?});
        self.sync_plugin_runner();
        let request = crate::runner::PluginActionRequest {
            id: action_id,
            pins,
            action: id.into(),
            input,
            script: None,
        };
        if !self.runner.submit_plugin_action(request) {
            return Err("the installed runner does not support plugin actions".into());
        }
        self.plugins.pending = Some(pending);
        self.plugins.status = json!({"state":"pending","id":action_id});
        self.pump();
        Ok(action_id)
    }

    pub fn plugin_action_status(&self) -> Value {
        self.plugins.status.clone()
    }

    /// Run an editor script through the same worker and atomic undo transaction
    /// as named actions, without installing the editor source as a dependency.
    pub fn run_javascript(&mut self, source: &str) -> Result<u64, String> {
        self.require_plugin_editable()?;
        if source.len() > 1024 * 1024 {
            return Err("script exceeds 1 MiB".into());
        }
        let pins = self.available_plugin_pins()?;
        let pending = self.new_plugin_pending(pins.clone());
        let id = pending.id;
        let input = json!({"params":{},
            "selection":serde_json::from_str::<Value>(&self.selection_json()).unwrap_or(Value::Null),
            "document":serde_json::from_str::<Value>(&self.history.request_json()).map_err(|e| e.to_string())?});
        self.sync_plugin_runner();
        let request = crate::runner::PluginActionRequest {
            id, pins, action: String::new(), input, script: Some(source.into()),
        };
        if !self.runner.submit_plugin_action(request) {
            return Err("the installed runner does not support JavaScript execution".into());
        }
        self.plugins.pending = Some(pending);
        self.plugins.status = json!({"state":"pending","id":id});
        self.pump();
        Ok(id)
    }
    pub fn poll_plugin_action(&mut self) -> Option<Result<Value, String>> {
        self.pump();
        self.plugins.results.pop_front()
    }
    pub fn cancel_plugin_action(&mut self) -> bool {
        if self.plugins.pending.take().is_none() {
            return false;
        }
        self.plugin_result(Err("plugin transaction cancelled".into()));
        // Queued old outputs can no longer apply. Restore the live document's baseline.
        self.run_generation += 1;
        self.applied_generation = self.run_generation;
        self.runner.reset();
        self.rerun_history();
        true
    }

    pub(super) fn sync_plugin_runner(&mut self) {
        let plugins = &self.plugins;
        self.runner
            .sync_plugins(plugins.revision, &mut || json!(plugins.bundles()));
    }

    fn plugin_result(&mut self, result: Result<Value, String>) {
        self.plugins.status = match &result {
            Ok(value) => json!({"state":"complete","result":value}),
            Err(error) => json!({"state":"error","error":error}),
        };
        self.plugins.results.push_back(result);
    }

    fn plugin_pending_valid(&self, pending: &PendingAction) -> bool {
        pending.revision == self.history.revision()
            && pending.registry_revision == self.plugins.revision
            && pending.selection == self.selection_json()
            && pending.generation == self.run_generation
            && self.history.locked().is_none()
    }

    pub(super) fn pump_plugin_callbacks(&mut self) {
        if self
            .plugins
            .pending
            .as_ref()
            .is_some_and(|p| !self.plugin_pending_valid(p))
        {
            let staged = self.plugins.pending.take().unwrap().staged.is_some();
            self.plugin_result(Err(
                "plugin transaction invalidated by a document or selection change".into(),
            ));
            if staged {
                self.runner.reset();
                self.rerun_history();
            }
        }
        while let Some(reply) = self.runner.poll_plugin_action() {
            let Some(pending) = self.plugins.pending.as_ref() else {
                continue;
            };
            if pending.id != reply.id {
                continue;
            }
            if !self.plugin_pending_valid(pending) {
                self.plugins.pending = None;
                self.plugin_result(Err(
                    "plugin transaction invalidated by a document or selection change".into(),
                ));
                continue;
            }
            if let Ok(plan) = &reply.result {
                if plan.commands.is_empty() {
                    self.plugins.pending = None;
                    self.plugin_result(Ok(
                        json!({"id":reply.id,"notifications":plan.notifications}),
                    ));
                    continue;
                }
            }
            let staged = reply.result.and_then(|plan| {
                let mut staged = self.history.clone();
                let pending = self.plugins.pending.as_mut().unwrap();
                staged.set_document_block_no_undo("plugins", Some(json!(pending.pins)));
                pending.notifications = plan.notifications;
                // Use the serializable command contract to keep all validation at one door.
                let commands = serde_json::to_value(plan.commands).map_err(|e| e.to_string())?;
                for command in commands.as_array().ok_or("invalid action commands")? {
                    match command["op"].as_str().unwrap_or("") {
                        "addFeature" => {
                            let ty = command["featureType"].as_str().ok_or("addFeature requires featureType")?;
                            let schema = self.feature_schema(ty).ok_or_else(|| format!("unknown feature type {ty}"))?;
                            let mut params = self.feature_default_params(ty);
                            let supplied = command["inputParams"].as_object().ok_or("feature parameters must be an object")?;
                            params.as_object_mut().unwrap().extend(supplied.clone());
                            let id = staged.next_feature_id(schema["shortName"].as_str().unwrap_or(ty));
                            params["id"] = json!(id);
                            staged.push_feature(json!({"type":ty,"inputParams":params,"persistentData":command["persistentData"]}));
                        }
                        "updateFeature" => {
                            let id = command["id"].as_str().ok_or("updateFeature requires id")?;
                            let index = staged.index_of(id).ok_or_else(|| format!("feature {id} not found"))?;
                            let mut params = command["inputParams"].clone();
                            if !params.is_object() { return Err("feature parameters must be an object".into()); }
                            params["id"] = json!(id);
                            staged.set_feature_params(index, params);
                        }
                        "deleteFeature" => {
                            let id = command["id"].as_str().ok_or("deleteFeature requires id")?;
                            let index = staged.index_of(id).ok_or_else(|| format!("feature {id} not found"))?;
                            staged.remove_feature(index);
                        }
                        op => return Err(format!("unsupported plugin document command {op}")),
                    }
                }
                Ok(staged)
            });
            match staged.and_then(|staged| self.submit_plugin_candidate(staged)) {
                Ok(()) => {}
                Err(error) => {
                    self.plugins.pending = None;
                    self.plugin_result(Err(error));
                }
            }
        }
    }

    fn submit_plugin_candidate(&mut self, mut staged: History) -> Result<(), String> {
        staged.set_rollback(staged.len().saturating_sub(1));
        let mut request: brep_kernel::HistoryRequest =
            serde_json::from_value(staged.prefix_request()).map_err(|e| e.to_string())?;
        request.display_lod = self.settings.lod_factor;
        self.run_generation += 1;
        let pending = self
            .plugins
            .pending
            .as_mut()
            .ok_or("plugin transaction disappeared")?;
        pending.generation = self.run_generation;
        pending.staged = Some(staged);
        self.plugins.status = json!({"state":"pending","id":pending.id});
        // Full delta means discarding an unsuccessful candidate cannot poison the display baseline.
        self.runner.reset();
        self.sync_plugin_runner();
        self.runner.sync_parts_library(
            brep_kernel::parts_library_revision(),
            &mut brep_kernel::parts_library_map,
        );
        self.runner.submit_run(request, self.run_generation);
        Ok(())
    }

    /// Returns false when this candidate must be discarded, without applying its scene.
    pub(super) fn accept_plugin_run(&mut self, reply: &crate::runner::RunReply) -> bool {
        let Some(pending) = self.plugins.pending.as_ref() else {
            return true;
        };
        if pending.staged.is_none() || pending.generation != reply.generation {
            return true;
        }
        let valid = self.plugin_pending_valid(pending);
        let mut pending = self.plugins.pending.take().unwrap();
        let report = &reply.output.report;
        let annotation_errors: Vec<_> = annotation_failures(report.pmi.as_ref())
            .into_iter()
            .filter(|failure| !pending.allowed_annotation_errors.contains(failure))
            .map(|failure| format!("{}: {}", failure.id, failure.message))
            .collect();
        let result = if !valid {
            Err("plugin transaction invalidated by a document or selection change".into())
        } else if !report.feature_errors.is_empty()
            || !report.unresolved.is_empty()
            || !report.display_errors.is_empty()
            || !annotation_errors.is_empty()
        {
            Err(format!(
                "plugin transaction recompute failed: {}",
                report
                    .feature_errors
                    .iter()
                    .chain(&report.unresolved)
                    .chain(&report.display_errors)
                    .chain(&annotation_errors)
                    .cloned()
                    .collect::<Vec<_>>()
                    .join("; ")
            ))
        } else {
            fold_annotation_data(pending.staged.as_mut().unwrap(), report.pmi.as_ref());
            self.history
                .commit_staged(pending.staged.as_ref().unwrap())
                .map(|_| json!({"id":pending.id,"notifications":pending.notifications,"annotationId":pending.annotation_id}))
        };
        let accepted = result.is_ok();
        self.plugin_result(result);
        if !accepted {
            self.runner.reset();
            self.rerun_history();
        }
        accepted
    }

    /// Add package pins and features together so undo restores both.
    pub(super) fn append_plugin_features(&mut self, features: &[Value]) -> Result<bool, String> {
        if !features
            .iter()
            .any(|f| f["type"].as_str().is_some_and(|t| t.contains('/')))
        {
            return Ok(false);
        }
        if let Some(reason) = self.history.locked() {
            return Err(format!("the document is read-only: {reason}"));
        }
        let mut pins = self.saved_plugin_pins()?;
        for feature in features {
            let ty = feature["type"].as_str().unwrap_or("");
            if !ty.contains('/') {
                continue;
            }
            let schema = self
                .feature_schema(ty)
                .ok_or_else(|| format!("plugin feature {ty} is unavailable"))?;
            let pin: PluginPin =
                serde_json::from_value(schema["package"].clone()).map_err(|e| e.to_string())?;
            if !pins.iter().any(|p| p.id == pin.id) {
                pins.push(pin);
            }
        }
        let mut staged = self.history.clone();
        staged.set_document_block_no_undo("plugins", Some(json!(pins)));
        staged.push_features(features.to_vec());
        staged.set_rollback(staged.len().saturating_sub(1));
        self.history.commit_staged(&staged)?;
        Ok(true)
    }

    pub(super) fn plugin_export_ready(&self) -> Result<(), String> {
        fn uses_plugins(value: &Value) -> bool {
            match value {
                Value::Object(map) => {
                    map.get("plugins")
                        .and_then(Value::as_array)
                        .is_some_and(|p| !p.is_empty())
                        || map
                            .get("type")
                            .and_then(Value::as_str)
                            .is_some_and(|ty| ty.contains('/'))
                        || map.values().any(uses_plugins)
                }
                Value::Array(values) => values.iter().any(uses_plugins),
                _ => false,
            }
        }
        let document: Value =
            serde_json::from_str(&self.history.request_json()).map_err(|e| e.to_string())?;
        if !uses_plugins(&document) {
            return Ok(());
        }
        if self.run_pending()
            || self.plugins.pending.is_some()
            || self.plugins.install_pending.is_some()
        {
            return Err("plugin geometry is pending; wait for recompute before export".into());
        }
        let annotation_errors = plugin_annotation_errors(self.pmi_report.as_ref());
        if !annotation_errors.is_empty() {
            return Err(format!(
                "unavailable plugin annotations: {}",
                annotation_errors.join("; ")
            ));
        }
        let report: Value = serde_json::from_str(&self.history_report).unwrap_or(Value::Null);
        if report.get("error").is_some()
            || ["featureErrors", "unresolved", "displayErrors"]
                .iter()
                .any(|key| {
                    report[*key]
                        .as_array()
                        .is_some_and(|items| !items.is_empty())
                })
        {
            return Err("plugin document has failed or unavailable geometry; recompute successfully before export".into());
        }
        Ok(())
    }

    pub(super) fn plugin_resident_handles(
        &self,
        request: &brep_kernel::HistoryRequest,
    ) -> Result<Vec<(String, u32)>, String> {
        let result = self.execute_plugin_history(request);
        let mut names = Vec::new();
        let mut handles = std::collections::HashMap::new();
        for feature in result.results {
            if let Some(error) = feature.error {
                return Err(format!("{}: {error}", feature.id));
            }
            for removed in feature.removed {
                handles.remove(&removed);
                names.retain(|n| n != &removed);
            }
            for added in feature.added {
                if handles.insert(added.name.clone(), added.handle).is_none() {
                    names.push(added.name);
                }
            }
        }
        Ok(names
            .into_iter()
            .map(|name| {
                let handle = handles[&name];
                (name, handle)
            })
            .collect())
    }

    pub(super) fn inherit_plugins(&mut self, parent: &Self) {
        self.plugins.packages = parent.plugins.packages.clone();
        self.plugins.runtime = parent.plugins.runtime.clone();
        self.plugins.revision = parent.plugins.revision;
    }

    pub(super) fn with_plugin_provider<R>(&self, f: impl FnOnce() -> R) -> R {
        match &self.plugins.runtime {
            Some(runtime) => runtime.with_provider(f),
            None => f(),
        }
    }

    pub(super) fn execute_plugin_history(
        &self,
        request: &brep_kernel::HistoryRequest,
    ) -> brep_kernel::HistoryResult {
        match &self.plugins.runtime {
            Some(runtime) => runtime.execute_history(request),
            None => brep_kernel::execute_history(request),
        }
    }

    pub(super) fn run_plugin_scene(
        &mut self,
        request: &brep_kernel::HistoryRequest,
    ) -> Result<crate::pipeline::SceneBuildReport, String> {
        match &self.plugins.runtime {
            Some(runtime) => runtime.with_provider(|| {
                crate::pipeline::update_scene_from_history(&mut self.scene, request)
            }),
            None => crate::pipeline::update_scene_from_history(&mut self.scene, request),
        }
    }
}


/// The exact error identity used by repair transactions. Other operations stay strict.
#[derive(Clone, Debug, PartialEq, Eq)]
struct AnnotationFailure {
    view_id: String,
    id: String,
    kind: String,
    message: String,
}

fn annotation_failures(report: Option<&brep_kernel::PmiReport>) -> Vec<AnnotationFailure> {
    report
        .into_iter()
        .flat_map(|r| &r.views)
        .flat_map(|view| {
            view.annotations
                .iter()
                .filter(|a| {
                    a.enabled && a.kind.contains('/') && a.status == brep_kernel::PmiStatus::Error
                })
                .map(|a| AnnotationFailure {
                    view_id: view.id.clone(),
                    id: a.id.clone(),
                    kind: a.kind.clone(),
                    message: a.message.clone(),
                })
        })
        .collect()
}

/// Even tolerated repair errors continue to block exports.
pub(super) fn plugin_annotation_errors(report: Option<&brep_kernel::PmiReport>) -> Vec<String> {
    annotation_failures(report)
        .into_iter()
        .map(|a| format!("{}: {}", a.id, a.message))
        .collect()
}

fn fold_annotation_data(history: &mut History, report: Option<&brep_kernel::PmiReport>) {
    let Some(block) = history.pmi_block() else {
        return;
    };
    let Ok(mut state) = serde_json::from_value::<brep_kernel::PmiState>(block.clone()) else {
        return;
    };
    let mut changed = false;
    for row in report
        .into_iter()
        .flat_map(|r| &r.views)
        .flat_map(|v| &v.annotations)
    {
        if row.status != brep_kernel::PmiStatus::Ok || !row.kind.contains('/') {
            continue;
        }
        if let (Some(data), Some(annotation)) =
            (&row.persistent_data, state.find_annotation_mut(&row.id))
        {
            if annotation.kind == row.kind
                && (annotation.persistent_data != *data
                    || annotation.plugin_replay != row.plugin_replay)
            {
                annotation.persistent_data = data.clone();
                annotation.plugin_replay = row.plugin_replay.clone();
                changed = true;
            }
        }
    }
    if changed {
        history.set_pmi_block_no_undo(Some(json!(state)));
    }
}

impl EngineState {
    pub fn plugin_panels(&self) -> Value {
        self.plugin_catalogue()["panels"].clone()
    }

    pub fn plugin_annotation_catalogue(&self) -> Value {
        self.plugin_catalogue()["annotations"].clone()
    }

    /// Built-ins and document-scoped providers share the normal annotation form shape.
    pub fn annotation_schema(&self, kind: &str) -> Option<Value> {
        brep_kernel::pmi_type(kind)
            .map(|d| (d.schema)())
            .or_else(|| {
                self.plugin_annotation_catalogue()
                    .as_array()?
                    .iter()
                    .find(|a| a["type"] == kind)
                    .cloned()
            })
    }

    pub(super) fn fold_plugin_annotation_data(&mut self) {
        if self.expression_preview.is_none() {
            fold_annotation_data(&mut self.history, self.pmi_report.as_ref());
        }
    }

    pub fn plugin_add_annotation(
        &mut self,
        view_id: &str,
        kind: &str,
        mut params: Value,
    ) -> Result<u64, String> {
        self.require_plugin_editable()?;
        let schema = self
            .plugin_annotation_catalogue()
            .as_array()
            .into_iter()
            .flatten()
            .find(|s| s["type"] == kind)
            .cloned()
            .ok_or_else(|| format!("unavailable plugin annotation {kind}"))?;
        if !params.is_object() {
            return Err("annotation parameters must be an object".into());
        }
        let mut state = self.pmi_state();
        let id = state.next_id("ANN");
        params["id"] = json!(id);
        state
            .find_view_mut(view_id)
            .ok_or_else(|| format!("no PMI view '{view_id}'"))?
            .annotations
            .push(brep_kernel::PmiAnnotation {
                kind: kind.into(),
                enabled: true,
                params,
                persistent_data: Value::Null,
                plugin_replay: None,
                label_world: None,
            });
        let pin: PluginPin =
            serde_json::from_value(schema["package"].clone()).map_err(|e| e.to_string())?;
        let mut pins = self.saved_plugin_pins()?;
        if let Some(old) = pins.iter().find(|p| p.id == pin.id) {
            if old != &pin {
                return Err("annotation package does not match document pin".into());
            }
        } else {
            pins.push(pin);
        }
        self.submit_annotation_state(state, pins, id, false)
    }

    pub fn plugin_update_annotation(&mut self, id: &str, mut params: Value) -> Result<u64, String> {
        self.require_plugin_editable()?;
        if !params.is_object() {
            return Err("annotation parameters must be an object".into());
        }
        let mut state = self.pmi_state();
        let annotation = state
            .find_annotation_mut(id)
            .ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        if !annotation.kind.contains('/') {
            return Err("annotation is not plugin-owned".into());
        }
        params["id"] = json!(id);
        annotation.params = params;
        self.submit_annotation_state(state, self.saved_plugin_pins()?, id.into(), false)
    }

    pub fn plugin_delete_annotation(&mut self, id: &str) -> Result<u64, String> {
        self.require_plugin_editable()?;
        let mut state = self.pmi_state();
        let (v, a) = state
            .locate_annotation(id)
            .ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        if !state.views[v].annotations[a].kind.contains('/') {
            return Err("annotation is not plugin-owned".into());
        }
        state.views[v].annotations.remove(a);
        self.submit_annotation_state(state, self.saved_plugin_pins()?, id.into(), true)
    }

    pub fn plugin_set_annotation_enabled(
        &mut self,
        id: &str,
        enabled: bool,
    ) -> Result<u64, String> {
        self.require_plugin_editable()?;
        let mut state = self.pmi_state();
        let annotation = state
            .find_annotation_mut(id)
            .ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        if !annotation.kind.contains('/') {
            return Err("annotation is not plugin-owned".into());
        }
        annotation.enabled = enabled;
        self.submit_annotation_state(state, self.saved_plugin_pins()?, id.into(), !enabled)
    }

    fn submit_annotation_state(
        &mut self,
        state: brep_kernel::PmiState,
        pins: Vec<PluginPin>,
        annotation_id: String,
        repair: bool,
    ) -> Result<u64, String> {
        let mut pending = self.new_plugin_pending(pins.clone());
        if repair {
            let baseline = self.pmi_state();
            pending.allowed_annotation_errors = annotation_failures(self.pmi_report.as_ref())
                .into_iter()
                .filter(|failure| {
                    let old = baseline
                        .find_view(&failure.view_id)
                        .and_then(|v| v.annotations.iter().find(|a| a.id() == failure.id));
                    let survivor = state
                        .find_view(&failure.view_id)
                        .and_then(|v| v.annotations.iter().find(|a| a.id() == failure.id));
                    old.is_some() && old == survivor
                })
                .collect();
        }
        let id = pending.id;
        pending.annotation_id = Some(annotation_id);
        let mut staged = self.history.clone();
        staged.set_pmi_block_no_undo(Some(json!(state)));
        staged.set_document_block_no_undo("plugins", Some(json!(pins)));
        self.plugins.pending = Some(pending);
        if let Err(error) = self.submit_plugin_candidate(staged) {
            self.plugins.pending = None;
            return Err(error);
        }
        self.pump();
        Ok(id)
    }
}

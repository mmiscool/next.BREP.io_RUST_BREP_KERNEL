//! Trusted, explicitly provisioned packages. Saved pins never carry executable code.
pub use brep_kernel::feature_pipeline::extension::PluginPin;
use brep_kernel::feature_pipeline::{self as kernel, FeatureContext, FeatureResult};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};
mod annotations;
mod invocation;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PackageManifest {
    pub id: String,
    pub version: String,
    pub api_version: u32,
    pub name: String,
    pub entry: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PackageBundle {
    pub manifest: PackageManifest,
    pub modules: BTreeMap<String, String>,
    #[serde(default)]
    pub assets: BTreeMap<String, Vec<u8>>,
}
fn local_path(s: &str) -> bool {
    !s.is_empty()
        && !s.contains('\\')
        && !s.contains(':')
        && s.split('/').all(|p| !p.is_empty() && p != "." && p != "..")
}
impl PackageBundle {
    pub fn pin(&self) -> Result<PluginPin, String> {
        let m = &self.manifest;
        if m.api_version != 1
            || m.id.is_empty()
            || m.id.contains('/')
            || !m
                .id
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || ".-_".contains(c))
            || m.version.is_empty()
            || m.name.is_empty()
        {
            return Err(format!(
                "{}: invalid manifest or unsupported API version",
                m.id
            ));
        }
        if !local_path(&m.entry)
            || !self.modules.contains_key(&m.entry)
            || self.modules.len() > 128
            || self
                .modules
                .keys()
                .chain(self.assets.keys())
                .any(|p| !local_path(p))
        {
            return Err(format!(
                "{}: invalid package-local paths or module count",
                m.id
            ));
        }
        if self.modules.values().map(String::len).sum::<usize>()
            + self.assets.values().map(Vec::len).sum::<usize>()
            > 8 * 1024 * 1024
        {
            return Err("package exceeds 8 MiB".into());
        }
        let digest = format!(
            "{:x}",
            Sha256::digest(serde_json::to_vec(self).map_err(|e| e.to_string())?)
        );
        Ok(PluginPin {
            id: m.id.clone(),
            version: m.version.clone(),
            api_version: m.api_version,
            digest,
        })
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FeatureRegistration {
    pub id: String,
    pub label: String,
    pub glyph: String,
    pub ribbon_path: String,
    pub command_size: String,
    pub short_name: String,
    pub input_params_schema: Value,
    pub package: PluginPin,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ActionRegistration {
    pub id: String,
    pub label: String,
    pub glyph: String,
    pub ribbon_path: String,
    pub command_size: String,
    pub input_params_schema: Value,
    pub package: PluginPin,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WorkbenchRegistration {
    pub id: String,
    pub label: String,
    pub feature_types: Vec<String>,
    pub actions: Vec<String>,
    pub panels: Vec<String>,
    pub package: PluginPin,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AnnotationRegistration {
    pub id: String,
    pub label: String,
    pub input_params_schema: Value,
    pub package: PluginPin,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PanelRegistration {
    pub id: String,
    pub label: String,
    pub controls: Vec<PanelControl>,
    pub package: PluginPin,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "camelCase", deny_unknown_fields)]
pub enum PanelControl {
    Text {
        text: String,
    },
    Table {
        columns: Vec<String>,
        rows: Vec<Vec<String>>,
    },
    Action {
        id: String,
        label: String,
        action: String,
        #[serde(default = "empty_object")]
        params: Value,
    },
    Form {
        id: String,
        label: String,
        action: String,
        #[serde(default = "empty_object")]
        params: Value,
    },
}
fn empty_object() -> Value {
    json!({})
}
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Registry {
    features: Vec<FeatureRegistration>,
    actions: Vec<ActionRegistration>,
    workbenches: Vec<WorkbenchRegistration>,
    #[serde(default)]
    panels: Vec<PanelRegistration>,
    #[serde(default)]
    annotations: Vec<AnnotationRegistration>,
    packages: Vec<PluginPin>,
    revision: String,
}
impl Registry {
    pub fn panels(&self) -> &[PanelRegistration] {
        &self.panels
    }
    pub fn annotations(&self) -> &[AnnotationRegistration] {
        &self.annotations
    }
    pub fn features(&self) -> &[FeatureRegistration] {
        &self.features
    }
    pub fn actions(&self) -> &[ActionRegistration] {
        &self.actions
    }
    pub fn workbenches(&self) -> &[WorkbenchRegistration] {
        &self.workbenches
    }
    pub fn revision(&self) -> &str {
        &self.revision
    }
    pub fn install(packages: &[PackageBundle]) -> Result<Self, String> {
        let mut result = Self::default();
        let mut seen = BTreeSet::new();
        for package in packages {
            let pin = package.pin()?;
            if !seen.insert((pin.id.clone(), pin.digest.clone())) {
                return Err(format!("duplicate package {}", pin.id));
            }
            let r = invocation::invoke(package, None, None, None, None)?.0;
            result.packages.push(pin);
            result.features.extend(r.features);
            result.actions.extend(r.actions);
            result.workbenches.extend(r.workbenches);
            result.panels.extend(r.panels);
            result.annotations.extend(r.annotations);
        }
        result.revision = format!("{:x}", Sha256::digest(serde_json::to_vec(&result).unwrap()));
        Ok(result)
    }
    pub fn for_document(&self, pins: &[PluginPin]) -> Result<Self, String> {
        let mut ids = BTreeSet::new();
        for pin in pins {
            if !ids.insert(&pin.id) {
                return Err(format!("duplicate document pin {}", pin.id));
            }
            if !self.packages.contains(pin) {
                return Err(format!("missing or disabled pinned package {}", pin.id));
            }
        }
        let mut r = self.clone();
        r.packages = pins.to_vec();
        r.features.retain(|x| pins.contains(&x.package));
        r.actions.retain(|x| pins.contains(&x.package));
        r.workbenches.retain(|x| pins.contains(&x.package));
        r.panels.retain(|x| pins.contains(&x.package));
        r.annotations.retain(|x| pins.contains(&x.package));
        r.revision = format!("{:x}", Sha256::digest(serde_json::to_vec(&r).unwrap()));
        Ok(r)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "camelCase")]
pub enum DocumentCommand {
    AddFeature {
        #[serde(rename = "featureType")]
        feature_type: String,
        #[serde(rename = "inputParams")]
        input_params: Value,
        #[serde(rename = "persistentData")]
        persistent_data: Value,
    },
    UpdateFeature {
        id: String,
        #[serde(rename = "inputParams")]
        input_params: Value,
    },
    DeleteFeature {
        id: String,
    },
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ActionPlan {
    pub commands: Vec<DocumentCommand>,
    pub notifications: Vec<String>,
}

/// Execute an explicitly submitted editor script without installing or pinning it.
/// The source is a function body with `cad` (the action context) and `ui.notify`.
/// Only the resulting document commands survive; built-in features replay normally.
pub fn execute_script(source: &str, input: &Value) -> Result<ActionPlan, String> {
    let package = PackageBundle {
        manifest: PackageManifest {
            id: "cad.editor".into(), version: "1".into(), api_version: 1,
            name: "JavaScript editor".into(), entry: "editor.js".into(),
        },
        modules: BTreeMap::from([("editor.js".into(), format!(r#"
export default function install(app) {{
    app.registerAction({{id: 'cad.editor/run', label: 'Run script', run(cad) {{
        const ui = Object.freeze({{notify: cad.notify}});
        const result = (function(cad, ui) {{
{source}
        }})(cad, ui);
        if (result && typeof result.then === 'function') throw Error('Promise results are unsupported');
        if (result !== undefined) console.log('Result:', JSON.stringify(result));
    }} }});
}}
"#))]),
        assets: BTreeMap::new(),
    };
    let (_, value, _) = invocation::invoke(
        &package, Some(("action", "cad.editor/run")), Some(input), None, None,
    )?;
    serde_json::from_value(value).map_err(|e| e.to_string())
}
#[derive(Clone)]
pub struct Runtime {
    packages: Vec<PackageBundle>,
    registry: Registry,
}
impl Runtime {
    pub fn new(packages: Vec<PackageBundle>) -> Result<Self, String> {
        let registry = Registry::install(&packages)?;
        Ok(Self { packages, registry })
    }
    /// Rehydrate a registry validated by a trusted worker. This is not an install API
    /// for untrusted metadata. Hash/owner checks do not execute JavaScript.
    pub fn from_validated(
        packages: Vec<PackageBundle>,
        registry: Registry,
    ) -> Result<Self, String> {
        let pins = packages
            .iter()
            .map(PackageBundle::pin)
            .collect::<Result<Vec<_>, _>>()?;
        if pins != registry.packages {
            return Err("validated registry package pins mismatch".into());
        }
        for (id, pin) in registry
            .features
            .iter()
            .map(|r| (&r.id, &r.package))
            .chain(registry.actions.iter().map(|r| (&r.id, &r.package)))
            .chain(registry.workbenches.iter().map(|r| (&r.id, &r.package)))
            .chain(registry.panels.iter().map(|r| (&r.id, &r.package)))
            .chain(registry.annotations.iter().map(|r| (&r.id, &r.package)))
        {
            if !pins.contains(pin) || !id.starts_with(&format!("{}/", pin.id)) {
                return Err("validated registry ownership mismatch".into());
            }
        }
        Ok(Self { packages, registry })
    }
    pub fn registry(&self) -> &Registry {
        &self.registry
    }
    pub fn with_provider<R>(&self, f: impl FnOnce() -> R) -> R {
        kernel::extension::with_provider(Rc::new(self.clone()), f)
    }
    pub fn execute_history(&self, request: &kernel::HistoryRequest) -> kernel::HistoryResult {
        self.with_provider(|| kernel::execute_history(request))
    }
    fn package(&self, pins: &[PluginPin], id: &str) -> Result<&PackageBundle, String> {
        let owner = id.split('/').next().unwrap_or("");
        let matches: Vec<_> = pins.iter().filter(|p| p.id == owner).collect();
        if matches.len() != 1 {
            return Err(format!("{} requires exactly one document plugin pin", id));
        }
        self.packages
            .iter()
            .find(|p| p.pin().as_ref() == Ok(matches[0]))
            .ok_or_else(|| format!("missing or disabled pinned package for {id}"))
    }
    pub fn execute_action(
        &self,
        pins: &[PluginPin],
        id: &str,
        input: &Value,
    ) -> Result<ActionPlan, String> {
        let p = self.package(pins, id)?;
        let (r, v, _) = invocation::invoke(
            p,
            Some(("action", id)),
            Some(input),
            None,
            Some(&self.registry.for_document(&[p.pin()?])?),
        )?;
        self.verify(p, &r)?;
        serde_json::from_value(v).map_err(|e| e.to_string())
    }
    fn verify(&self, p: &PackageBundle, r: &Registry) -> Result<(), String> {
        let expected = self.registry.for_document(&[p.pin()?])?;
        if expected.features != r.features
            || expected.actions != r.actions
            || expected.workbenches != r.workbenches
            || expected.panels != r.panels
            || expected.annotations != r.annotations
        {
            return Err("registration metadata changed during invocation".into());
        }
        Ok(())
    }
}
impl kernel::extension::ExtensionProvider for Runtime {
    fn execute_annotation(
        &self,
        pins: &[PluginPin],
        annotation: &kernel::pmi::PmiAnnotation,
        context: &kernel::pmi::PmiContext<'_>,
    ) -> Result<kernel::pmi::PluginAnnotationOutput, String> {
        Runtime::execute_annotation(self, pins, annotation, context)
    }
    fn identity(&self, pins: &[PluginPin], kind: &str) -> Result<String, String> {
        let p = self.package(pins, kind)?;
        if !self
            .registry
            .features
            .iter()
            .any(|r| r.id == kind && r.package == p.pin().unwrap())
        {
            return Err(format!("unregistered plugin feature {kind}"));
        }
        Ok(format!("{}:{}:1", p.pin()?.digest, kind))
    }
    fn execute(&self, pins: &[PluginPin], ctx: &FeatureContext<'_>) -> FeatureResult {
        let run = || -> Result<FeatureResult, String> {
            let p = self.package(pins, &ctx.feature_type)?;
            let reg = self
                .registry
                .features
                .iter()
                .find(|r| r.id == ctx.feature_type && r.package == p.pin().unwrap())
                .ok_or("feature registration unavailable")?;
            let mut params = ctx.params.clone();
            validate_params(&reg.input_params_schema, &mut params, Some(ctx))?;
            let input = json!({"params":params,"persistentData":ctx.persistent,"featureId":ctx.id,"units":"mm"});
            let (r, _, result) = invocation::invoke(
                p,
                Some(("feature", &ctx.feature_type)),
                Some(&input),
                Some(ctx),
                Some(&self.registry.for_document(&[p.pin()?])?),
            )?;
            self.verify(p, &r)?;
            // Invocation publishes only after metadata, return and all handles validate.
            result.ok_or_else(|| "missing feature result".into())
        };
        match run() {
            Ok(r) => r,
            Err(e) => ctx.fail(e),
        }
    }
}
fn validate_params(
    schema: &Value,
    params: &mut Value,
    ctx: Option<&FeatureContext<'_>>,
) -> Result<(), String> {
    let obj = params
        .as_object_mut()
        .ok_or("parameters must be an object")?;
    let fields = schema.as_object().ok_or("schema must be object")?;
    for key in obj.keys() {
        if key != "id" && !fields.contains_key(key) {
            return Err(format!("unknown parameter {key}"));
        }
    }
    if obj.get("id").is_some_and(|v| !v.is_string()) {
        return Err("feature id must be string".into());
    }
    for (key, s) in schema.as_object().ok_or("schema must be object")? {
        if !obj.contains_key(key) {
            if let Some(v) = s.get("default_value") {
                obj.insert(key.clone(), v.clone());
            }
        }
        let v = obj
            .get_mut(key)
            .ok_or_else(|| format!("missing parameter {key}"))?;
        match s["type"].as_str().unwrap_or("") {
            "number" => {
                let n = if let Some(n) = v.as_f64() {
                    n
                } else if let (Some(c), Some(text)) = (ctx, v.as_str()) {
                    c.env.eval(text)?
                } else {
                    return Err(format!("{key} must be numeric"));
                };
                if !n.is_finite()
                    || s["min"].as_f64().is_some_and(|m| n < m)
                    || s["max"].as_f64().is_some_and(|m| n > m)
                {
                    return Err(format!("{key} outside finite numeric bounds"));
                }
                *v = json!(n);
            }
            "reference_selection" => {
                fn reference(v: &Value) -> bool {
                    v.is_string()
                        || v.as_object()
                            .is_some_and(|o| o.get("name").is_some_and(Value::is_string))
                }
                if !reference(v) && !v.as_array().is_some_and(|a| a.iter().all(reference)) {
                    return Err(format!("{key} must contain reference names"));
                }
            }
            "string" if !v.is_string() => return Err(format!("{key} must be string")),
            "boolean" if !v.is_boolean() => return Err(format!("{key} must be boolean")),
            _ => {}
        }
    }
    Ok(())
}

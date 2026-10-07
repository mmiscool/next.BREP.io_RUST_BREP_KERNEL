//! Typed, interpreter-independent adapter for plugin annotation presentation.
use super::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// Provider output. Only geometry with a host-owned layout adapter is accepted.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct PluginAnnotationOutput {
    pub text: String,
    pub geometry: PmiGeometry,
    pub default_label: [f64; 3],
    #[serde(default)]
    pub references: Vec<String>,
    #[serde(default)]
    pub value: Option<f64>,
    #[serde(default)]
    pub unit: String,
    #[serde(default)]
    pub persistent_data: Value,
}

pub(super) fn resolve(
    annotation: &PmiAnnotation,
    context: &PmiContext<'_>,
) -> Result<(Resolved, Value, PluginAnnotationReplay), String> {
    let mut input = annotation.clone();
    if let Some(replay) = &annotation.plugin_replay {
        if replay.matches(annotation, context) {
            input.persistent_data = replay.input_data.clone();
        }
    }
    // Providers must still validate exact pins and execute on every replay. A
    // saved checkpoint is input state, not proof that a provider is available.
    input.plugin_replay = None;
    let output = super::super::extension::execute_annotation(&input, context)?;
    output.validate()?;
    if annotation
        .label_world
        .as_ref()
        .is_some_and(|p| p.iter().any(|x| !x.is_finite()))
    {
        return Err("invalid plugin annotation label position".into());
    }
    for name in &output.references {
        super::resolve::resolve_reference(context.scene, name)?;
    }
    let unit = match output.unit.as_str() {
        "" => "",
        "mm" => "mm",
        "deg" => "deg",
        _ => return Err("unsupported plugin annotation unit".into()),
    };
    let replay = PluginAnnotationReplay {
        version: 1,
        fingerprint: fingerprint(annotation, context),
        dependencies: dependency_snapshots(context, output.references.iter().map(String::as_str))?,
        input_data: input.persistent_data,
        output_data: output.persistent_data.clone(),
    };
    Ok((
        Resolved {
            text: output.text,
            value: output.value,
            unit,
            references: output.references,
            geometry: output.geometry,
            default_label: output.default_label,
        },
        output.persistent_data,
        replay,
    ))
}

impl PluginAnnotationOutput {
    /// Shared validation for runtime helpers and replay's Rust adapter.
    pub fn validate(&self) -> Result<(), String> {
        let finite = |p: &[f64; 3]| p.iter().all(|x| x.is_finite());
    if self.text.len() > 16_384 || self.references.len() > 256
        || !finite(&self.default_label)
        || self.value.is_some_and(|x| !x.is_finite())
        || serde_json::to_vec(&self.persistent_data).map_err(|e| e.to_string())?.len() > 1_048_576
    {
        return Err("invalid plugin annotation output or output limit exceeded".into());
    }
    match &self.geometry {
        PmiGeometry::Note { position } if finite(position) => {},
        PmiGeometry::Leader { targets, .. }
            if !targets.is_empty() && targets.len() <= 256 && targets.iter().all(finite) => {},
        PmiGeometry::Linear { a, b, component }
            if finite(a) && finite(b) && component.is_none_or(|c| matches!(c, 'X' | 'Y' | 'Z')) => {},
        _ => return Err("unsupported or invalid plugin annotation geometry (expected note, leader or linear)".into()),
    }
        if let PmiGeometry::Linear { a, b, component: Some(axis) } = &self.geometry {
            let index = match axis { 'X' => 0, 'Y' => 1, 'Z' => 2, _ => unreachable!() };
            if (0..3).any(|i| i != index && (a[i] - b[i]).abs() > 1e-9) {
                return Err("invalid plugin annotation geometry: component dimension endpoints must be projected onto its axis".into());
            }
        }
        if !matches!(self.unit.as_str(), "" | "mm" | "deg") {
            return Err("unsupported plugin annotation unit".into());
        }
        Ok(())
    }
}

/// Serializable transition input needed to reproduce an accepted callback result.
/// No geometry/output presentation is cached: callbacks always run and validate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PluginAnnotationReplay {
    pub version: u32,
    pub fingerprint: Value,
    pub dependencies: BTreeMap<String, Value>,
    pub input_data: Value,
    pub output_data: Value,
}

impl PluginAnnotationReplay {
    fn matches(&self, annotation: &PmiAnnotation, context: &PmiContext<'_>) -> bool {
        self.version == 1
            && self.output_data == annotation.persistent_data
            && self.fingerprint == fingerprint(annotation, context)
            && dependency_snapshots(context, self.dependencies.keys().map(String::as_str))
                .is_ok_and(|current| current == self.dependencies)
    }
}

fn fingerprint(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Value {
    let owner = annotation.kind.split('/').next().unwrap_or("");
    let pins: Vec<_> = context
        .request
        .plugins
        .iter()
        .filter(|p| p.id == owner)
        .collect();
    // Schema evaluation belongs to the provider. Including raw expressions and
    // configurator conservatively invalidates every expression-environment edit.
    // Camera, label placement and unrelated annotations are not callback inputs.
    json!({"type": annotation.kind, "pins": pins, "params": annotation.params,
        "expressions": context.request.expressions, "configurator": context.request.configurator})
}

fn dependency_snapshots<'a>(
    context: &PmiContext<'_>,
    names: impl IntoIterator<Item = &'a str>,
) -> Result<BTreeMap<String, Value>, String> {
    names
        .into_iter()
        .map(|name| {
            let geometry = super::resolve::resolve_reference(context.scene, name)?;
            let value = serde_json::to_value(geometry).map_err(|e| e.to_string())?;
            Ok((name.to_owned(), value))
        })
        .collect()
}

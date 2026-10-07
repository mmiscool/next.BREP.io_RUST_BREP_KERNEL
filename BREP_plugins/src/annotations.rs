//! Synchronous typed PMI callbacks. No resident geometry handles enter JavaScript.
use super::*;
use kernel::pmi::{PluginAnnotationOutput, PmiAnnotation, PmiContext, PmiGeometry};

impl Runtime {
    pub fn execute_annotation(
        &self,
        pins: &[PluginPin],
        annotation: &PmiAnnotation,
        context: &PmiContext<'_>,
    ) -> Result<PluginAnnotationOutput, String> {
        let package = self.package(pins, &annotation.kind)?;
        let pin = package.pin()?;
        let registration = self
            .registry
            .annotations
            .iter()
            .find(|r| r.id == annotation.kind && r.package == pin)
            .ok_or("annotation registration unavailable")?;
        let mut params = annotation.params.clone();
        // Use the document's expression environment, then the same schema validator
        // as actions/features. Reference fields are resolved separately below.
        if let Some(fields) = params.as_object_mut() {
            for (key, schema) in registration.input_params_schema.as_object().unwrap() {
                if schema["type"] == "number" {
                    if let Some(source) = fields.get(key).and_then(Value::as_str) {
                        let value = context.env.eval(source)?;
                        if !value.is_finite() {
                            return Err("annotation parameter must be finite".into());
                        }
                        fields.insert(key.clone(), json!(value));
                    }
                }
            }
        }
        validate_params(&registration.input_params_schema, &mut params, None)?;
        let mut names = BTreeSet::new();
        fn collect(value: &Value, names: &mut BTreeSet<String>) -> Result<(), String> {
            match value {
                Value::String(name) => {
                    if !name.trim().is_empty() {
                        names.insert(name.clone());
                    }
                }
                Value::Object(object) => {
                    let name = object["name"].as_str().ok_or("reference missing name")?;
                    if !name.trim().is_empty() {
                        names.insert(name.to_owned());
                    }
                }
                Value::Array(values) => {
                    for value in values {
                        collect(value, names)?;
                    }
                }
                _ => return Err("invalid annotation reference".into()),
            }
            Ok(())
        }
        for (key, schema) in registration.input_params_schema.as_object().unwrap() {
            if schema["type"] == "reference_selection" {
                collect(&params[key], &mut names)?;
            }
        }
        if names.len() > 256 {
            return Err("annotation dependency limit exceeded".into());
        }
        let mut dependencies = serde_json::Map::new();
        for name in &names {
            if name.is_empty() || name.len() > 1024 {
                return Err("invalid annotation dependency name".into());
            }
            let point = kernel::pmi::resolve::resolve_reference(context.scene, name)?
                .representative_point();
            let point = [point.x, point.y, point.z];
            if !point.iter().all(|v| v.is_finite()) {
                return Err("nonfinite annotation dependency".into());
            }
            dependencies.insert(name.clone(), json!({"point":point}));
        }
        let input = json!({"params":params,"persistentData":annotation.persistent_data,"annotationId":annotation.id(),"units":"mm","dependencies":dependencies});
        if input.to_string().len() > 1024 * 1024 {
            return Err("annotation input exceeds 1 MiB".into());
        }
        let (registry, value, _) = invocation::invoke(
            package,
            Some(("annotation", &annotation.kind)),
            Some(&input),
            None,
            Some(&self.registry.for_document(&[pin])?),
        )?;
        self.verify(package, &registry)?;
        let mut output = validate_output(value)?;
        if output.references.iter().any(|r| !names.contains(r)) {
            return Err("annotation output contains undeclared reference".into());
        }
        // Include every declared dependency, even if the callback omits references.
        output.references = names.into_iter().collect();
        Ok(output)
    }
}

pub(super) fn validate_output(value: Value) -> Result<PluginAnnotationOutput, String> {
    let object = value
        .as_object()
        .ok_or("annotation result must be an object")?;
    if object.keys().any(|k| {
        ![
            "text",
            "geometry",
            "defaultLabel",
            "references",
            "value",
            "unit",
            "persistentData",
        ]
        .contains(&k.as_str())
    }) {
        return Err("unknown annotation result property".into());
    }
    let geometry = object
        .get("geometry")
        .and_then(Value::as_object)
        .ok_or("annotation geometry must be an object")?;
    let allowed: &[&str] = match geometry.get("kind").and_then(Value::as_str) {
        Some("note") => &["kind", "position"],
        Some("leader") => &["kind", "targets", "dot"],
        Some("linear") => &["kind", "a", "b", "component"],
        _ => return Err("unsupported annotation geometry; expected note, leader or linear".into()),
    };
    if geometry.keys().any(|k| !allowed.contains(&k.as_str())) {
        return Err("unknown annotation geometry property".into());
    }
    let output: PluginAnnotationOutput =
        serde_json::from_value(value).map_err(|e| format!("invalid annotation result: {e}"))?;
    output.validate()?;
    let finite = |p: &[f64; 3]| p.iter().all(|v| v.is_finite() && v.abs() <= 1e12);
    if output.text.is_empty()
        || output.text.len() > 16384
        || !matches!(output.unit.as_str(), "" | "mm" | "deg")
        || !finite(&output.default_label)
        || output.value.is_some_and(|v| !v.is_finite())
        || output.references.len() > 256
        || output
            .references
            .iter()
            .any(|s| s.is_empty() || s.len() > 1024)
        || output.persistent_data.to_string().len() > 1024 * 1024
    {
        return Err("invalid annotation text, value, references, persistent data or label".into());
    }
    match &output.geometry {
        PmiGeometry::Note { position } if finite(position) => {}
        PmiGeometry::Leader { targets, .. }
            if !targets.is_empty() && targets.len() <= 256 && targets.iter().all(finite) => {}
        PmiGeometry::Linear { a, b, component }
            if finite(a)
                && finite(b)
                && a != b
                && component.is_none_or(|c| match c {
                    'X' => a[1] == b[1] && a[2] == b[2],
                    'Y' => a[0] == b[0] && a[2] == b[2],
                    'Z' => a[0] == b[0] && a[1] == b[1],
                    _ => false,
                }) => {}
        _ => return Err("invalid annotation geometry".into()),
    }
    Ok(output)
}

pub(super) fn primitive(op: &str, params: &Value) -> Result<Value, String> {
    let mut output =
        json!({"text":params["text"], "defaultLabel":params["defaultLabel"], "persistentData":{}});
    output["geometry"] = match op {
        "annotation.note" => {
            output["defaultLabel"] = params["position"].clone();
            json!({"kind":"note", "position":params["position"]})
        }
        "annotation.leader" => {
            json!({"kind":"leader", "targets":params["targets"], "dot":params.get("dot").cloned().unwrap_or(json!(false))})
        }
        "annotation.linear" => {
            json!({"kind":"linear", "a":params["a"], "b":params["b"], "component":params.get("component").cloned().unwrap_or(Value::Null)})
        }
        _ => return Err("unknown annotation primitive".into()),
    };
    // PMI layout draws a→b directly. Project the endpoint before validation so
    // component measurements and the rendered dimension describe the same span.
    if op == "annotation.linear" {
        if let Some(axis) = match params["component"].as_str() {
            Some("X") => Some(0),
            Some("Y") => Some(1),
            Some("Z") => Some(2),
            _ => None,
        } {
            let a: [f64; 3] = serde_json::from_value(params["a"].clone())
                .map_err(|e| format!("invalid linear endpoint: {e}"))?;
            let original_b: [f64; 3] = serde_json::from_value(params["b"].clone())
                .map_err(|e| format!("invalid linear endpoint: {e}"))?;
            if !a
                .iter()
                .chain(original_b.iter())
                .all(|v| v.is_finite() && v.abs() <= 1e12)
            {
                return Err("invalid linear endpoint".into());
            }
            let mut b = a;
            b[axis] = original_b[axis];
            output["geometry"]["b"] = json!(b);
        }
    }
    let mut validated = validate_output(output)?;
    if let PmiGeometry::Linear { a, b, component } = &validated.geometry {
        let delta = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        validated.value = Some(match component {
            Some('X') => delta[0].abs(),
            Some('Y') => delta[1].abs(),
            Some('Z') => delta[2].abs(),
            _ => delta.iter().map(|v| v * v).sum::<f64>().sqrt(),
        });
        validated.unit = "mm".into();
    }
    serde_json::to_value(validated).map_err(|e| e.to_string())
}

use super::*;
use brep_kernel::{BrepSolid, Vec3};
use brep_script_core::boa_engine::{
    self as boa, Context, JsNativeError, JsResult, JsValue, NativeFunction, Source,
};
use std::cell::RefCell;
fn error(s: impl ToString) -> boa::JsError {
    JsNativeError::typ().with_message(s.to_string()).into()
}
#[derive(Default)]
struct Arena {
    solids: Vec<BrepSolid>,
    references: BTreeMap<String, BrepSolid>,
    operations: usize,
}
impl Arena {
    fn get(&self, v: &Value) -> Result<&BrepSolid, String> {
        self.solids
            .get(v.as_u64().ok_or("invalid geometry token")? as usize)
            .ok_or("invalid geometry token".into())
    }
    fn call(&mut self, op: &str, args: &Value) -> Result<Value, String> {
        self.operations += 1;
        if self.operations > 256 {
            return Err("geometry operation limit exceeded".into());
        }
        let a = &args[0];
        if op.starts_with("annotation.") {
            return super::annotations::primitive(op, a);
        }
        let origin = Vec3::new(0., 0., 0.);
        let axis = Vec3::new(0., 1., 0.);
        let number = |key: &str| -> Result<f64, String> {
            a[key]
                .as_f64()
                .filter(|n| n.is_finite())
                .ok_or_else(|| format!("{key} must be finite number"))
        };
        let s = match op {
            "sphere" => brep_kernel::make_sphere_brep(origin, number("radius")?, axis)?,
            "cube" => brep_kernel::make_box_brep(
                origin,
                number("sizeX")?,
                number("sizeY")?,
                number("sizeZ")?,
            )?,
            "cylinder" => {
                brep_kernel::make_cylinder_brep(origin, axis, number("radius")?, number("height")?)?
            }
            "reference" => self
                .references
                .get(a.as_str().ok_or("reference name must be string")?)
                .ok_or("reference is unresolved or was not declared in parameters")?
                .clone(),
            "transform" => {
                let p = &args[1];
                let vec = |key: &str, default: [f64; 3]| -> Result<[f64; 3], String> {
                    if p[key].is_null() {
                        return Ok(default);
                    }
                    let v = p[key]
                        .as_array()
                        .filter(|a| a.len() == 3)
                        .ok_or("transform vector must contain three numbers")?;
                    let mut out = [0.; 3];
                    for i in 0..3 {
                        out[i] = v[i]
                            .as_f64()
                            .filter(|x| x.is_finite())
                            .ok_or("transform requires finite numbers")?;
                    }
                    Ok(out)
                };
                kernel::extension::transform(
                    self.get(a)?,
                    vec("position", [0.; 3])?,
                    vec("rotationEuler", [0.; 3])?,
                    vec("scale", [1.; 3])?,
                )?
            }
            "boolean" => {
                let op = match a.as_str().unwrap_or("").to_uppercase().as_str() {
                    "UNION" => brep_kernel::BooleanOperation::Union,
                    "SUBTRACT" => brep_kernel::BooleanOperation::Subtract,
                    "INTERSECT" => brep_kernel::BooleanOperation::Intersect,
                    _ => return Err("unsupported boolean operation".into()),
                };
                brep_kernel::boolean_operation(
                    self.get(&args[1])?,
                    self.get(&args[2])?,
                    op,
                    &Default::default(),
                )
                .map_err(|e| e.to_string())?
            }
            "query" => {
                return Ok(json!({"volume":brep_kernel::solid_signed_volume(self.get(a)?)?}))
            }
            _ => return Err("unsupported geometry operation".into()),
        };
        let index = self.solids.len();
        self.solids.push(s);
        Ok(json!(index))
    }
}
fn host_method(cx: &mut Context, name: &str, args: &[JsValue]) -> JsResult<JsValue> {
    let host = cx
        .eval(Source::from_bytes("__pluginHost"))?
        .as_object()
        .ok_or_else(|| error("missing plugin host"))?;
    let f = host
        .get(boa::JsString::from(name), cx)?
        .as_callable()
        .ok_or_else(|| error("missing host method"))?;
    f.call(&host.into(), args, cx)
}
pub(super) fn invoke(
    package: &PackageBundle,
    callback: Option<(&str, &str)>,
    input: Option<&Value>,
    ctx: Option<&FeatureContext<'_>>,
    expected: Option<&Registry>,
) -> Result<(Registry, Value, Option<FeatureResult>), String> {
    let pin = package.pin()?;
    let arena = Rc::new(RefCell::new(Arena::default()));
    if let Some(ctx) = ctx {
        fn collect(v: &Value, out: &mut BTreeSet<String>) {
            match v {
                Value::String(s) => {
                    out.insert(s.clone());
                }
                Value::Array(a) => {
                    for v in a {
                        collect(v, out)
                    }
                }
                Value::Object(o) => {
                    for v in o.values() {
                        collect(v, out)
                    }
                }
                _ => {}
            }
        }
        let mut names = BTreeSet::new();
        collect(ctx.params, &mut names);
        collect(ctx.persistent, &mut names);
        for name in names {
            if ctx.scene.resolve_solid(&name).is_some() {
                arena.borrow_mut().references.insert(
                    name.clone(),
                    kernel::extension::clone_reference(ctx, &name)?,
                );
            }
        }
    }
    let owned = arena.clone();
    let console_logs = Rc::new(RefCell::new(None::<brep_script_core::Logs>));
    let captured_logs = console_logs.clone();
    let output = brep_script_core::modules::with_module(
        &package.modules,
        &package.manifest.entry,
        move |cx, logs| {
            *captured_logs.borrow_mut() = Some(logs.clone());
            // SAFETY: captures only owned Rust BREP values and JSON; no GC-managed values.
            let function = unsafe {
                NativeFunction::from_closure(move |_, args, cx| {
                    let op = args
                        .first()
                        .and_then(JsValue::as_string)
                        .ok_or_else(|| error("missing operation"))?
                        .to_std_string_escaped();
                    let args = args
                        .get(1)
                        .ok_or_else(|| error("missing geometry args"))?
                        .to_json(cx)?
                        .unwrap_or(Value::Null);
                    let value = owned.borrow_mut().call(&op, &args).map_err(error)?;
                    JsValue::from_json(&value, cx)
                })
            };
            cx.register_global_builtin_callable(boa::JsString::from("__geometry"), 2, function)?;
            cx.eval(Source::from_bytes(include_str!("host.js")))?;
            Ok(())
        },
        |module, cx| {
            let mut install = module.get_value(boa::JsString::from("default"), cx)?;
            if !install.is_callable() {
                install = module.get_value(boa::JsString::from("install"), cx)?;
            }
            let install = install
                .as_callable()
                .ok_or_else(|| error("package must export default install or named install"))?;
            let app = cx.eval(Source::from_bytes("__pluginHost.app"))?;
            let result = install.call(&JsValue::undefined(), &[app], cx)?;
            brep_script_core::modules::synchronous(result.clone())?;
            host_method(cx, "sync", &[result])?;
            host_method(cx, "seal", &[])?;
            let metadata = host_method(cx, "metadata", &[])?
                .to_json(cx)?
                .ok_or_else(|| error("invalid registration metadata"))?;
            if metadata.to_string().len() > 1024 * 1024 {
                return Err(error("registration metadata exceeds 1 MiB"));
            }
            let registry = registry(&metadata, &pin).map_err(error)?;
            if let Some(expected) = expected {
                if registry.features != expected.features
                    || registry.actions != expected.actions
                    || registry.workbenches != expected.workbenches
                    || registry.panels != expected.panels
                    || registry.annotations != expected.annotations
                {
                    return Err(error("registration metadata changed during invocation"));
                }
            }
            let value = if let Some((kind, id)) = callback {
                let mut input = input.cloned().unwrap_or(json!({}));
                if kind == "action" {
                    let def = registry
                        .actions
                        .iter()
                        .find(|d| d.id == id)
                        .ok_or_else(|| error("unknown action"))?;
                    if input.get("params").is_some() {
                        validate_params(&def.input_params_schema, &mut input["params"], None)
                            .map_err(error)?;
                    } else {
                        validate_params(&def.input_params_schema, &mut input, None)
                            .map_err(error)?;
                    }
                }
                let arg = JsValue::from_json(&input, cx)?;
                let value = host_method(
                    cx,
                    "invoke",
                    &[
                        boa::JsString::from(kind).into(),
                        boa::JsString::from(id).into(),
                        arg,
                    ],
                )?;
                value.to_json(cx)?.unwrap_or(Value::Null)
            } else {
                Value::Null
            };
            if value.to_string().len() > 1024 * 1024 {
                return Err(error("callback result exceeds 1 MiB"));
            }
            Ok((registry, value))
        },
    )
    .map_err(|e| {
        let logs = if e.logs.is_empty() {
            String::new()
        } else {
            format!("\n{}", e.logs.join("\n"))
        };
        format!("{}: {}{logs}", pin.id, e.message)
    })?;
    if let Some(ctx) = ctx {
        let outputs = output.1["outputs"]
            .as_object()
            .ok_or("feature outputs must be object")?;
        if outputs.is_empty() || outputs.len() > 64 {
            return Err("feature requires 1..64 outputs".into());
        }
        let arena = arena.borrow();
        let mut staged = Vec::new();
        for (key, token) in outputs {
            if key.is_empty()
                || !key
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
            {
                return Err("invalid output key".into());
            }
            staged.push((format!("{}:{key}", ctx.id), arena.get(token)?.clone()));
        }
        let mut result = FeatureResult::empty(&ctx.id, &ctx.feature_type);
        result.persistent_data = Some(output.1["persistentData"].clone());
        for (name, solid) in staged {
            result.added.push(kernel::extension::publish(solid, &name));
        }
        Ok((output.0, Value::Null, Some(result)))
    } else {
        let mut value = output.1;
        if callback.is_some_and(|(kind, _)| kind == "action") {
            if let Some(logs) = console_logs.borrow().as_ref() {
                if let Some(notifications) = value["notifications"].as_array_mut() {
                    notifications.extend(logs.borrow().iter().cloned().map(Value::String));
                }
            }
        }
        Ok((output.0, value, None))
    }
}
fn registry(v: &Value, pin: &PluginPin) -> Result<Registry, String> {
    let mut r = Registry::default();
    r.packages.push(pin.clone());
    let mut ids = BTreeSet::new();
    for (kind, list) in [
        ("feature", &v["features"]),
        ("action", &v["actions"]),
        ("workbench", &v["workbenches"]),
        ("panel", &v["panels"]),
        ("annotation", &v["annotations"]),
    ] {
        let list = list.as_array().ok_or("invalid registration list")?;
        if list.len() > 256 {
            return Err("registration limit exceeded".into());
        }
        for d in list {
            let permitted = match kind {
                "feature" => &[
                    "id",
                    "label",
                    "shortName",
                    "inputParamsSchema",
                    "ribbonPath",
                    "commandSize",
                    "glyph",
                ][..],
                "action" => &[
                    "id",
                    "label",
                    "inputParamsSchema",
                    "ribbonPath",
                    "commandSize",
                    "glyph",
                ][..],
                "annotation" => &["id", "label", "inputParamsSchema"][..],
                "panel" => &["id", "label", "controls"][..],
                _ => &["id", "label", "featureTypes", "actions", "panels"][..],
            };
            for key in d.as_object().ok_or("registration must be object")?.keys() {
                if !permitted.contains(&key.as_str()) {
                    return Err(format!("unsupported {kind} registration property {key}"));
                }
            }
            let id = d["id"]
                .as_str()
                .ok_or("registration missing id")?
                .to_owned();
            if !id.starts_with(&format!("{}/", pin.id))
                || id.len() > 192
                || id.split('/').count() != 2
                || id.ends_with('/')
                || !id
                    .chars()
                    .all(|c| c.is_ascii_alphanumeric() || "./_-".contains(c))
                || !ids.insert(id.clone())
            {
                return Err(format!("invalid or duplicate registration id {id}"));
            }
            let label = d["label"]
                .as_str()
                .filter(|s| !s.is_empty() && s.len() < 256)
                .ok_or("registration requires label")?
                .to_owned();
            let schema = d.get("inputParamsSchema").cloned().unwrap_or(json!({}));
            let object = schema
                .as_object()
                .ok_or("inputParamsSchema must be object")?;
            for (key, s) in object {
                if key.is_empty()
                    || !matches!(
                        s["type"].as_str(),
                        Some("number" | "string" | "boolean" | "reference_selection")
                    )
                {
                    return Err(format!("unsupported schema type for {key}"));
                }
                for b in ["min", "max"] {
                    if let Some(v) = s.get(b) {
                        if !v.as_f64().is_some_and(f64::is_finite) {
                            return Err("schema bounds must be finite".into());
                        }
                    }
                }
                if s["min"]
                    .as_f64()
                    .zip(s["max"].as_f64())
                    .is_some_and(|(a, b)| a > b)
                {
                    return Err("schema min exceeds max".into());
                }
            }
            for (key, s) in object {
                if let Some(default) = s.get("default_value") {
                    validate_params(&json!({key:s}), &mut json!({key:default}), None)?;
                }
            }
            let (ribbon_path, command_size) = if matches!(kind, "feature" | "action") {
                // Features and actions both default to Home's Extensions group:
                // the ribbon has no Tools tab, every workbench group is on Home.
                let path = match d.get("ribbonPath") {
                    Some(v) => v.as_str().ok_or("ribbonPath must be text")?.to_owned(),
                    None => format!("Home/Extensions/{}", label.replace('/', " ")),
                };
                let segments: Vec<_> = path.split('/').collect();
                if segments.len() != 3
                    || segments
                        .iter()
                        .any(|s| s.trim().is_empty() || s.trim() != *s)
                    || !["Home", "View", "Help"].contains(&segments[0])
                    || (kind == "feature" && segments[0] != "Home")
                {
                    return Err(format!("invalid {kind} ribbonPath: {path}"));
                }
                let size = match d.get("commandSize") {
                    Some(v) => v.as_str().ok_or("commandSize must be text")?,
                    None => {
                        if kind == "feature" {
                            "Large"
                        } else {
                            "Compact"
                        }
                    }
                };
                if !["Large", "Compact"].contains(&size) {
                    return Err(format!("invalid commandSize: {size}"));
                }
                (path, size.to_owned())
            } else {
                (String::new(), String::new())
            };
            match kind {
                "feature" => r.features.push(FeatureRegistration {
                    glyph: d["glyph"].as_str().unwrap_or("\u{2699}").to_owned(),
                    ribbon_path,
                    command_size,
                    id,
                    label,
                    short_name: d["shortName"].as_str().unwrap_or("").to_owned(),
                    input_params_schema: schema,
                    package: pin.clone(),
                }),
                "action" => r.actions.push(ActionRegistration {
                    glyph: d["glyph"].as_str().unwrap_or("\u{2699}").to_owned(),
                    ribbon_path,
                    command_size,
                    id,
                    label,
                    input_params_schema: schema,
                    package: pin.clone(),
                }),
                "annotation" => r.annotations.push(AnnotationRegistration {
                    id,
                    label,
                    input_params_schema: schema,
                    package: pin.clone(),
                }),
                "panel" => r.panels.push(PanelRegistration {
                    id,
                    label,
                    controls: serde_json::from_value(
                        d.get("controls").cloned().unwrap_or(json!([])),
                    )
                    .map_err(|e| format!("invalid panel controls: {e}"))?,
                    package: pin.clone(),
                }),
                _ => {
                    let strings = |key: &str| -> Result<Vec<String>, String> {
                        serde_json::from_value(d.get(key).cloned().unwrap_or(json!([])))
                            .map_err(|e| e.to_string())
                    };
                    let panels = strings("panels")?;
                    r.workbenches.push(WorkbenchRegistration {
                        id,
                        label,
                        feature_types: strings("featureTypes")?,
                        actions: strings("actions")?,
                        panels,
                        package: pin.clone(),
                    });
                }
            }
        }
    }
    validate_panels(&r)?;
    for w in &r.workbenches {
        for p in &w.panels {
            if !r.panels.iter().any(|r| &r.id == p) {
                return Err(format!("unknown workbench panel {p}"));
            }
        }
        for a in &w.actions {
            if !r.actions.iter().any(|r| &r.id == a) {
                return Err(format!("unknown workbench action {a}"));
            }
        }
        for f in &w.feature_types {
            if !r.features.iter().any(|r| &r.id == f)
                && !brep_kernel::feature_schema_catalogue()["features"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|r| r["type"].as_str() == Some(f))
            {
                return Err(format!("unknown workbench feature {f}"));
            }
        }
    }
    Ok(r)
}

fn validate_panels(r: &Registry) -> Result<(), String> {
    for panel in &r.panels {
        if panel.controls.len() > 128 {
            return Err("panel control limit exceeded".into());
        }
        let mut ids = BTreeSet::new();
        for control in &panel.controls {
            match control {
                PanelControl::Text { text } => {
                    if text.len() > 16384 {
                        return Err("panel text too long".into());
                    }
                }
                PanelControl::Table { columns, rows } => {
                    if columns.is_empty()
                        || columns.len() > 32
                        || rows.len() > 256
                        || columns.iter().any(|s| s.len() > 256)
                        || rows.iter().any(|row| {
                            row.len() != columns.len() || row.iter().any(|s| s.len() > 4096)
                        })
                    {
                        return Err("invalid panel table dimensions or text".into());
                    }
                }
                PanelControl::Action {
                    id,
                    label,
                    action,
                    params,
                }
                | PanelControl::Form {
                    id,
                    label,
                    action,
                    params,
                } => {
                    if id.is_empty()
                        || id.len() > 128
                        || !id
                            .chars()
                            .all(|c| c.is_ascii_alphanumeric() || "_-".contains(c))
                        || !ids.insert(id)
                        || label.is_empty()
                        || label.len() > 256
                    {
                        return Err("invalid or duplicate panel control id/label".into());
                    }
                    let action = r
                        .actions
                        .iter()
                        .find(|a| &a.id == action)
                        .ok_or("unknown panel action")?;
                    let mut params = params.clone();
                    if matches!(control, PanelControl::Form { .. }) {
                        let fields = params
                            .as_object()
                            .ok_or("panel parameters must be object")?;
                        let schema: serde_json::Map<String, Value> = action
                            .input_params_schema
                            .as_object()
                            .unwrap()
                            .iter()
                            .filter(|(k, _)| fields.contains_key(*k))
                            .map(|(k, v)| (k.clone(), v.clone()))
                            .collect();
                        validate_params(&Value::Object(schema), &mut params, None)?;
                    } else {
                        validate_params(&action.input_params_schema, &mut params, None)?;
                    }
                }
            }
        }
    }
    Ok(())
}

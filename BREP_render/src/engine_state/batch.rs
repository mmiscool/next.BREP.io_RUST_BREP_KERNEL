//! Native batch authoring. Validation works on a cloned recipe; execution uses
//! a native runner barrier so the transaction cannot expose pending results.
use super::*;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

fn alias_ref(text: &str) -> Option<(&str, &str)> {
    let rest = text.strip_prefix('@')?;
    let n = rest.find(['/', ':']).unwrap_or(rest.len());
    Some((&rest[..n], &rest[n..]))
}

/// Validate reference grammar without requiring an output to exist yet.
fn validate_reference(text: &str, aliases: &BTreeMap<String, String>) -> Result<(), String> {
    if let Some((alias, suffix)) = alias_ref(text) {
        if !aliases.contains_key(alias) {
            return Err(format!("missing alias @{alias}"));
        }
        if !suffix.is_empty()
            && !suffix.starts_with(':')
            && suffix
                .strip_prefix('/')
                .and_then(|s| s.split_once('/'))
                .filter(|(kind, _)| matches!(*kind, "output" | "face" | "edge"))
                .and_then(|(_, index)| index.parse::<usize>().ok())
                .is_none()
        {
            return Err(format!("invalid reference {text}"));
        }
    }
    Ok(())
}

fn validate_references(
    value: &Value,
    aliases: &BTreeMap<String, String>,
    path: &str,
) -> Result<(), String> {
    let mut refs = Vec::new();
    strings(value, &mut refs);
    for reference in refs {
        validate_reference(&reference, aliases).map_err(|error| format!("{path}: {error}"))?;
    }
    Ok(())
}

fn strings(value: &Value, out: &mut Vec<String>) {
    match value {
        Value::String(s) => out.push(s.clone()),
        Value::Array(a) => a.iter().for_each(|v| strings(v, out)),
        Value::Object(o) => o.values().for_each(|v| strings(v, out)),
        _ => (),
    }
}

fn resolve(
    value: &mut Value,
    aliases: &BTreeMap<String, String>,
    outputs: &BTreeMap<String, Vec<String>>,
) -> Result<(), String> {
    match value {
        Value::String(s) => {
            if let Some((alias, suffix)) = alias_ref(s) {
                let id = aliases
                    .get(alias)
                    .ok_or_else(|| format!("missing alias @{alias}"))?;
                *s = if let Some((kind, index)) =
                    suffix.strip_prefix('/').and_then(|s| s.split_once('/'))
                {
                    if !matches!(kind, "output" | "face" | "edge") {
                        return Err(format!("invalid output kind in {s}"));
                    }
                    let index: usize = index
                        .parse()
                        .map_err(|_| format!("invalid output index in {s}"))?;
                    let key = if kind == "output" {
                        id.clone()
                    } else {
                        format!("{id}/{kind}")
                    };
                    outputs
                        .get(&key)
                        .and_then(|o| o.get(index))
                        .cloned()
                        .ok_or_else(|| format!("{kind} {index} of @{alias} was not generated"))?
                } else if suffix.is_empty() || suffix.starts_with(':') {
                    format!("{id}{suffix}")
                } else {
                    return Err(format!("invalid alias reference {s}"));
                };
            }
        }
        Value::Array(a) => {
            for v in a {
                resolve(v, aliases, outputs)?;
            }
        }
        Value::Object(o) => {
            for v in o.values_mut() {
                resolve(v, aliases, outputs)?;
            }
        }
        _ => (),
    }
    Ok(())
}

fn merge(base: &mut Value, patch: &Value) {
    if let (Some(b), Some(p)) = (base.as_object_mut(), patch.as_object()) {
        for (k, v) in p {
            if let Some(old) = b.get_mut(k) {
                merge(old, v);
            } else {
                b.insert(k.clone(), v.clone());
            }
        }
    } else {
        *base = patch.clone();
    }
}

fn prepare(
    catalogue: &Value,
    items: &[Value],
    history: &mut History,
    aliases: &mut BTreeMap<String, String>,
) -> Result<Vec<Value>, String> {
    let mut ids: BTreeSet<String> = history
        .features()
        .iter()
        .filter_map(|f| f["inputParams"]["id"].as_str().map(str::to_owned))
        .collect();
    ids.extend(aliases.values().cloned());
    let mut features = Vec::new();
    for (i, item) in items.iter().enumerate() {
        let ty = item["type"]
            .as_str()
            .ok_or_else(|| format!("features[{i}].type is required"))?;
        let schema = catalogue["features"].as_array().and_then(|fs| fs.iter().find(|f| f["type"] == ty || (!f["type"].as_str().unwrap_or("").contains('/') && f["shortName"] == ty))).cloned()
            .ok_or_else(|| format!("features[{i}].type: unknown type {ty}"))?;
        let mut params = json!({});
        if let Some(properties) = schema["inputParamsSchema"].as_object() {
            for (key, spec) in properties { params[key] = spec["default_value"].clone(); }
        }
        params["id"] = Value::Null;
        let patch = item
            .get("inputParams")
            .or_else(|| item.get("params"))
            .cloned()
            .unwrap_or(json!({}));
        let object = patch
            .as_object()
            .ok_or_else(|| format!("features[{i}].params must be an object"))?;
        for key in object.keys() {
            if params.get(key).is_none()
                && !(super::components::is_acomp_feature_type(ty) && key == "occurrenceAttributes")
            {
                return Err(format!("features[{i}].params.{key}: unknown field"));
            }
        }
        merge(&mut params, &patch);
        let id = item["id"]
            .as_str()
            .or_else(|| params["id"].as_str())
            .map(str::to_owned)
            .unwrap_or_else(|| loop {
                let id = history.next_feature_id(schema["shortName"].as_str().unwrap_or(ty));
                if !ids.contains(&id) {
                    break id;
                }
            });
        if id.is_empty() || !ids.insert(id.clone()) {
            return Err(format!("features[{i}].id: empty or duplicate id `{id}`"));
        }
        if matches!(ty, "ACOMP" | "ASSEMBLY COMPONENT") && !brep_kernel::is_component_reference(&id)
        {
            return Err(format!(
                "features[{i}].id: component ids must be ACOMP<digits>"
            ));
        }
        params["id"] = json!(id);
        if super::components::is_acomp_feature_type(ty) {
            if !object.contains_key("isFixed") {
                params.as_object_mut().unwrap().remove("isFixed");
            }
            if let Some(attributes) = item.get("attributes") {
                if !attributes.is_object() {
                    return Err(format!("features[{i}].attributes must be an object"));
                }
                merge(&mut params["occurrenceAttributes"], attributes);
            }
            if let Some(name) = item.get("display_name") {
                params["occurrenceAttributes"]["Name"] = name.clone();
            }
            if let Some(material) = item.get("material") {
                params["occurrenceAttributes"]["Material"] = material.clone();
            }
        }
        if let Some(alias) = item.get("alias") {
            let alias = alias
                .as_str()
                .filter(|s| !s.is_empty() && !s.contains(['@', '/', ':']))
                .ok_or_else(|| format!("features[{i}].alias: invalid alias"))?;
            if aliases.insert(alias.into(), id.clone()).is_some() {
                return Err(format!("features[{i}].alias: duplicate alias {alias}"));
            }
        }
        features.push(json!({"type":schema["type"], "inputParams":params, "persistentData":item.get("persistent_data").or_else(|| item.get("persistentData")).cloned().unwrap_or(json!({}))}));
    }
    Ok(features)
}

/// Stable topological order; dependencies include explicit `depends_on` aliases
/// and alias references anywhere in parameters or persistent data.
fn plan(
    items: &[Value],
    features: &[Value],
    aliases: &BTreeMap<String, String>,
) -> Result<(Vec<usize>, Vec<Vec<usize>>), String> {
    let positions: BTreeMap<&str, usize> = features
        .iter()
        .enumerate()
        .map(|(i, f)| (f["inputParams"]["id"].as_str().unwrap(), i))
        .collect();
    let mut deps = vec![Vec::new(); features.len()];
    for (i, feature) in features.iter().enumerate() {
        let mut refs = Vec::new();
        strings(&feature["inputParams"], &mut refs);
        strings(&feature["persistentData"], &mut refs);
        if let Some(explicit) = items[i].get("depends_on") {
            let a = explicit
                .as_array()
                .ok_or_else(|| format!("features[{i}].depends_on must be an array"))?;
            for r in a {
                refs.push(format!(
                    "@{}",
                    r.as_str().ok_or("depends_on entries must be aliases")?
                ));
            }
        }
        for r in refs {
            validate_reference(&r, aliases).map_err(|error| format!("features[{i}]: {error}"))?;
            if let Some((a, _)) = alias_ref(&r) {
                let id = &aliases[a];
                if let Some(&p) = positions.get(id.as_str()) {
                    deps[i].push(p);
                }
            }
        }
        deps[i].sort_unstable();
        deps[i].dedup();
    }
    let mut order = Vec::new();
    while order.len() < features.len() {
        let Some(i) = (0..features.len())
            .find(|i| !order.contains(i) && deps[*i].iter().all(|d| order.contains(d)))
        else {
            return Err(format!(
                "dependency cycle among feature positions {:?}",
                (0..features.len())
                    .filter(|i| !order.contains(i))
                    .collect::<Vec<_>>()
            ));
        };
        order.push(i);
    }
    Ok((order, deps))
}

fn failed(report: &Value) -> bool {
    report.get("error").is_some()
        || [
            "featureErrors",
            "displayErrors",
            "unresolved",
            "featureFulfilment",
        ]
        .iter()
        .any(|key| match &report[*key] {
            Value::Object(o) => !o.is_empty(),
            Value::Array(a) => !a.is_empty(),
            _ => false,
        })
}

fn refusal_category(refusal: &Value) -> &'static str {
    match refusal["class"].as_str() {
        Some("degenerate_arrangement" | "non_integral_genus" | "invalid_result_topology") => {
            "invalid_topology"
        }
        Some("tangent_node_singularity" | "conservative_empty_overlap") => "contact_related",
        Some("non_convergence") => "numerical_nonconvergence",
        Some("invalid_input") => "invalid_input",
        Some("unsupported_geometry") => "unsupported_geometry",
        _ => "geometry_execution_failure",
    }
}

impl EngineState {
    /// Full diagnostics from the last batch, including a rolled-back attempt.
    pub fn geometry_diagnostics(&self, id: &str) -> Option<Value> {
        self.geometry_diagnostics.get(id).cloned()
    }

    /// Wait on the installed native runner, preserving its resident geometry
    /// and delta baseline. A transaction is one app command, so other commands
    /// cannot observe its intermediate recipes. Kernel operations are not
    /// preemptible inside a feature.
    fn finish_batch_run(&mut self) {
        while self.run_pending() {
            self.pump();
            #[cfg(not(target_arch = "wasm32"))]
            if self.run_pending() {
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }
    }

    /// Full-document rollback includes recipe, library, metadata, counters and
    /// undo/redo. Runtime caches and the generation counter are deliberately
    /// invalidated, not rewound. No file, network or document-tab side effects.
    pub fn author_batch(&mut self, args: &Value) -> Result<Value, String> {
        if cfg!(target_arch = "wasm32") {
            return Err("structured native authoring requires a native MCP host".into());
        }
        if self.run_pending()
            || self.queries_pending()
            || self.mesh_imports_pending()
            || self.topology_pending()
        {
            return Err("wait for pending engine work before authoring a batch".into());
        }
        if let Some(reason) = self.history.locked() {
            return Err(format!("document is locked: {reason}"));
        }
        let items = args["features"]
            .as_array()
            .ok_or("features must be an array")?;
        let rollback = args["rollback_on_error"].as_bool().unwrap_or(false);
        let before = self.history.clone();
        let metadata = self.metadata.clone();
        let library = brep_kernel::parts_library_map();
        let mut staged = before.clone();
        let mut document: Value =
            serde_json::from_str(&staged.request_json()).map_err(|e| e.to_string())?;
        let mut aliases = BTreeMap::new();
        let mut part_aliases = BTreeMap::new();
        let mut part_work = Vec::new();
        let mut constraint_ids = Vec::new();
        if let Some(parts) = args.get("parts") {
            for (i, part) in parts
                .as_array()
                .ok_or("parts must be an array")?
                .iter()
                .enumerate()
            {
                let alias = part["alias"]
                    .as_str()
                    .filter(|s| !s.is_empty() && !s.contains(['@', '/', ':']))
                    .ok_or_else(|| format!("parts[{i}].alias is required"))?;
                if aliases.insert(alias.into(), alias.into()).is_some()
                    || document["partsLibrary"].get(alias).is_some()
                {
                    return Err(format!("parts[{i}]: duplicate part {alias}"));
                }
                let mut doc = part["document"].clone();
                let native = doc["features"]
                    .as_array()
                    .ok_or_else(|| format!("parts[{i}].document.features must be an array"))?
                    .clone();
                let mut allocation_base = doc.clone();
                allocation_base["features"] = json!([]);
                let mut part_history = History::from_request_json(&allocation_base.to_string())?;
                let mut local_aliases = BTreeMap::new();
                let mut scope = EngineState::new();
                scope.inherit_plugins(self);
                scope.history = part_history.clone();
                let mut prepared = prepare(&scope.feature_catalogue(), &native, &mut part_history, &mut local_aliases)
                    .map_err(|e| format!("parts[{i}].document.{e}"))?;
                let _ = plan(&native, &prepared, &local_aliases)
                    .map_err(|e| format!("parts[{i}].document.{e}"))?;
                for (index, f) in prepared.iter_mut().enumerate() {
                    for key in ["alias", "depends_on"] {
                        if let Some(value) = native[index].get(key) {
                            f[key] = value.clone();
                        }
                    }
                }
                part_aliases.insert(alias.to_owned(), local_aliases);
                // Keep client order for result positions; the isolated batch
                // below executes its own dependency plan.
                doc["features"] = json!(prepared);
                doc["featureCounter"] = serde_json::from_str::<Value>(&part_history.request_json())
                    .unwrap_or_default()["featureCounter"]
                    .clone();
                part_work.push((alias.to_owned(), doc.clone()));
                document["partsLibrary"][alias] = json!({"sourceKey":alias,"sourceSignature":"batch", "document":doc,"snapshot":""});
            }
        }
        let mut features = prepare(&self.feature_catalogue(), items, &mut staged, &mut aliases)?;
        let (order, deps) = plan(items, &features, &aliases)?;
        document["featureCounter"] = serde_json::from_str::<Value>(&staged.request_json())
            .unwrap_or_default()["featureCounter"]
            .clone();
        if let Some(expressions) = args.get("expressions") {
            if !expressions.is_string() {
                return Err("expressions must be a script string".into());
            }
            document["expressions"] = expressions.clone();
        }
        // Constraints use the native catalogue and descriptor shape.
        if let Some(constraints) = args.get("constraints") {
            let catalogue = brep_kernel::constraint_schema_catalogue();
            let mut counter = document["assembly"]["idCounter"].as_u64().unwrap_or(0);
            for (i, constraint) in constraints
                .as_array()
                .ok_or("constraints must be an array")?
                .iter()
                .enumerate()
            {
                let ty = constraint["type"]
                    .as_str()
                    .ok_or("constraint type is required")?;
                let entry = catalogue
                    .as_array()
                    .and_then(|a| a.iter().find(|e| e["type"].as_str() == Some(ty)))
                    .ok_or_else(|| format!("constraints[{i}].type: unknown constraint {ty}"))?;
                counter += 1;
                let id = format!("{}{}", entry["shortName"].as_str().unwrap_or("C"), counter);
                let mut params = json!({});
                if let Some(schema) = entry["inputParamsSchema"].as_object() {
                    for (k, v) in schema {
                        params[k] = v["default_value"].clone();
                    }
                }
                merge(
                    &mut params,
                    constraint
                        .get("params")
                        .or_else(|| constraint.get("inputParams"))
                        .unwrap_or(&json!({})),
                );
                for key in params
                    .as_object()
                    .ok_or_else(|| format!("constraints[{i}].params must be an object"))?
                    .keys()
                {
                    if entry["inputParamsSchema"].get(key).is_none() {
                        return Err(format!("constraints[{i}].params.{key}: unknown field"));
                    }
                }
                constraint_ids.push(id.clone());
                params["id"] = json!(id);
                validate_references(&params, &aliases, &format!("constraints[{i}].params"))?;
                if let Some(a) = constraint["alias"].as_str() {
                    if aliases.insert(a.into(), id).is_some() {
                        return Err(format!("constraints[{i}].alias is duplicate"));
                    }
                }
                if !document["assembly"]["constraints"].is_array() {
                    document["assembly"]["constraints"] = json!([]);
                }
                let mut persistent = constraint
                    .get("persistent_data")
                    .or_else(|| constraint.get("persistentData"))
                    .cloned()
                    .unwrap_or(json!({}));
                validate_references(
                    &persistent,
                    &aliases,
                    &format!("constraints[{i}].persistentData"),
                )?;
                let explicit = constraint
                    .get("params")
                    .or_else(|| constraint.get("inputParams"));
                // Interactive creation adopts a measurement on its first solve.
                // A batch's explicitly supplied target already expresses intent.
                for (field, flag) in [
                    ("distance", "initializedDistance"),
                    ("angle", "initializedAngle"),
                ] {
                    if explicit.is_some_and(|p| p.get(field).is_some()) {
                        persistent
                            .as_object_mut()
                            .ok_or_else(|| {
                                format!("constraints[{i}].persistent_data must be an object")
                            })?
                            .entry(flag)
                            .or_insert(json!(true));
                    }
                }
                document["assembly"]["constraints"].as_array_mut().unwrap().push(json!({"type":ty,"inputParams":params,"persistentData":persistent,"enabled":constraint.get("enabled").cloned().unwrap_or(json!(true))}));
            }
            document["assembly"]["idCounter"] = json!(counter);
        }
        if let Some(patch) = args.get("metadata") {
            let map = patch.as_object().ok_or("metadata must be an object")?;
            for key in map.keys() {
                if let Some((a, _)) = alias_ref(key) {
                    if !aliases.contains_key(a) {
                        return Err(format!("metadata.{key}: missing alias @{a}"));
                    }
                }
            }
        }
        // Parse the entire proposed request before touching any live state.
        let mut validation = document.clone();
        validation["features"]
            .as_array_mut()
            .ok_or("document features missing")?
            .extend(features.clone());
        serde_json::from_value::<HistoryRequest>(validation)
            .map_err(|e| format!("batch validation: {e}"))?;
        self.geometry_diagnostics.clear();
        let run = (|| -> Result<Value, String> {
            // Compile each native definition in isolation first. This resolves
            // generated-output aliases inside part histories too, without
            // turning the part into an imported/baked solid.
            let mut part_results = Vec::new();
            for (position, (alias, source)) in part_work.iter().enumerate() {
                let mut base = source.clone();
                let part_features = base["features"].take();
                base["features"] = json!([]);
                base.as_object_mut().unwrap().remove("metadata");
                let mut part = EngineState::new();
                part.inherit_plugins(self);
                part.set_history_json(&base.to_string())?;
                let mut part_args = json!({"features":part_features,"rollback_on_error":true});
                if let Some(metadata) = source.get("metadata") {
                    part_args["metadata"] = metadata.clone();
                }
                let mut evaluated = part.author_batch(&part_args)?;
                for (id, details) in &part.geometry_diagnostics {
                    self.geometry_diagnostics
                        .insert(format!("part:{alias}:{id}"), details.clone());
                }
                for item in evaluated["items"].as_array_mut().into_iter().flatten() {
                    if let Some(id) = item["diagnostic"]["detailId"].as_str().map(str::to_owned) {
                        item["diagnostic"]["detailId"] = json!(format!("part:{alias}:{id}"));
                    }
                }
                part_results.push(json!({"position":position,"alias":alias,"id":alias,"status":if evaluated["success"] == true {"evaluated"} else {"failed"},"committed":false,"items":evaluated["items"],"outputs":evaluated["outputs"]}));
                if evaluated["success"] != true {
                    for (i, (remaining, _)) in part_work.iter().enumerate().skip(position + 1) {
                        part_results.push(json!({"position":i,"alias":remaining,"id":remaining,"status":"not_evaluated","committed":false}));
                    }
                    let pending: Vec<Value> = items.iter().enumerate().map(|(i,item)| json!({"position":i,"alias":item["alias"],"id":features[i]["inputParams"]["id"],"status":"not_evaluated","committed":false,"dependencyFailure":{"part":alias}})).collect();
                    return Ok(
                        json!({"success":false,"requiresRestore":true,"items":pending,"partResults":part_results,"constraintResults":[],"aliases":aliases,"partAliases":part_aliases,"diagnosticsTool":"geometry_diagnostics"}),
                    );
                }
                document["partsLibrary"][alias]["document"] =
                    serde_json::from_str::<Value>(&part.history_request_json())
                        .map_err(|e| e.to_string())?;
            }
            // Delay constraints until every instance exists.
            let constraints = document.get("assembly").cloned();
            document.as_object_mut().unwrap().remove("assembly");
            self.edit_document_json(&document.to_string())?;
            self.finish_batch_run();
            let prepared_library =
                serde_json::from_value(document.get("partsLibrary").cloned().unwrap_or(json!({})))
                    .map_err(|e| format!("partsLibrary: {e}"))?;
            let mut outputs: BTreeMap<String, Vec<String>> = BTreeMap::new();
            let mut results = vec![Value::Null; items.len()];
            let mut failed_items = BTreeSet::new();
            for i in order {
                let id = features[i]["inputParams"]["id"]
                    .as_str()
                    .unwrap()
                    .to_string();
                let blocked: Vec<usize> = deps[i]
                    .iter()
                    .filter(|d| failed_items.contains(*d))
                    .copied()
                    .collect();
                let mut result = json!({"position":i,"alias":items[i]["alias"],"id":id,"committed":false,"dependencies":deps[i]});
                if !blocked.is_empty() {
                    failed_items.insert(i);
                    result["status"] = json!("not_evaluated");
                    result["dependencyFailures"] = json!(blocked);
                    results[i] = result;
                    continue;
                }
                if let Err(error) = resolve(&mut features[i], &aliases, &outputs) {
                    failed_items.insert(i);
                    result["status"] = json!("failed");
                    result["error"] = json!({"category":"reference_resolution","message":error,"path":format!("features[{i}]")});
                    results[i] = result;
                    continue;
                }
                brep_kernel::install_parts_library(&prepared_library);
                self.history.set_parts_library(
                    serde_json::to_value(brep_kernel::parts_library_map())
                        .map_err(|e| e.to_string())?,
                );
                self.add_features(&[features[i].clone()]);
                self.finish_batch_run();
                let report: Value =
                    serde_json::from_str(&self.history_report_json()).map_err(|e| e.to_string())?;
                let names: Vec<String> =
                    serde_json::from_value(report["featureOutputs"][&id].clone())
                        .unwrap_or_default();
                outputs.insert(id.clone(), names.clone());
                let errors: Vec<Value> = report["featureErrors"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter(|e| e.as_str().is_some_and(|e| e.starts_with(&format!("{id}:"))))
                    .cloned()
                    .collect();
                let has_error = !errors.is_empty()
                    || report["featureFulfilment"].get(&id).is_some()
                    || report.get("error").is_some();
                if has_error {
                    failed_items.insert(i);
                }
                result["status"] = json!(if has_error {
                    "failed"
                } else if report["featureTimings"].get(&id).is_some() {
                    "evaluated"
                } else {
                    "not_evaluated"
                });
                if result["status"] == "not_evaluated" {
                    failed_items.insert(i);
                }
                result["generatedSolids"] = json!(names);
                self.geometry_diagnostics.insert(id.clone(), json!({"errors":errors,"refusal":report["featureRefusals"][&id],"unresolved":report["unresolved"],"fulfilment":report["featureFulfilment"][&id],"approximations":report["featureApproximations"][&id],"operation":features[i]["type"],"operands":features[i]["inputParams"],"dependencies":deps[i]}));
                result["error"] = json!(errors
                    .iter()
                    .filter_map(Value::as_str)
                    .map(|s| s.chars().take(240).collect::<String>())
                    .collect::<Vec<_>>());
                let mut references = Vec::new();
                for name in &names {
                    if let Some(solid) = self.scene.solid(name) {
                        references.push(json!({"solid":name,"faces":solid.faces.iter().map(|f| &f.name).collect::<Vec<_>>(),"edges":solid.edges.iter().map(|e| &e.name).collect::<Vec<_>>()}));
                    }
                }
                outputs.insert(
                    format!("{id}/face"),
                    references
                        .iter()
                        .flat_map(|r| r["faces"].as_array().into_iter().flatten())
                        .filter_map(Value::as_str)
                        .map(str::to_owned)
                        .collect(),
                );
                outputs.insert(
                    format!("{id}/edge"),
                    references
                        .iter()
                        .flat_map(|r| r["edges"].as_array().into_iter().flatten())
                        .filter_map(Value::as_str)
                        .map(str::to_owned)
                        .collect(),
                );
                result["generatedReferences"] = json!(references);
                let schema =
                    self.feature_schema(features[i]["type"].as_str().unwrap_or(""))
                        .unwrap_or_default();
                let operands: serde_json::Map<String, Value> = features[i]["inputParams"]
                    .as_object()
                    .into_iter()
                    .flat_map(|o| o.iter())
                    .filter(|(k, _)| {
                        matches!(
                            schema["inputParamsSchema"][*k]["type"].as_str(),
                            Some("reference_selection" | "boolean_operation")
                        ) || k.as_str() == "partName"
                    })
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect();
                result["diagnostic"] = json!({"operation":features[i]["type"],"owningComponent": if super::components::is_acomp_feature_type(features[i]["type"].as_str().unwrap_or("")) { Some(&id) } else { None }, "operands":operands,"affectedTopology":report["featureRefusals"][&id]["faces"],"category":if has_error { Some(refusal_category(&report["featureRefusals"][&id])) } else { None },"kernelClass":report["featureRefusals"][&id]["class"],"detailId":id});
                result["committed"] = json!(true);
                results[i] = result;
            }
            let mut constraint_errors = BTreeMap::new();
            if let Some(mut constraints) = constraints {
                if let Some(entries) = constraints["constraints"].as_array_mut() {
                    entries.retain_mut(|entry| {
                        let id = entry["inputParams"]["id"].as_str().unwrap_or("").to_owned();
                        if let Err(error) = resolve(entry, &aliases, &outputs) {
                            constraint_errors.insert(id, error);
                            false
                        } else {
                            true
                        }
                    });
                }
                let mut final_doc: Value = serde_json::from_str(&self.history.request_json())
                    .map_err(|e| e.to_string())?;
                final_doc["assembly"] = constraints;
                self.edit_document_json(&final_doc.to_string())?;
                self.finish_batch_run();
            }
            if let Some(patch) = args.get("metadata").and_then(Value::as_object) {
                let mut merged = self.metadata.to_json();
                for (key, record) in patch {
                    let mut name = json!(key);
                    resolve(&mut name, &aliases, &outputs)?;
                    merge(&mut merged[name.as_str().unwrap()], record);
                }
                self.metadata.load_json(Some(&merged));
                self.sync_colors_from_metadata();
            }
            let final_report: Value =
                serde_json::from_str(&self.history_report_json()).map_err(|e| e.to_string())?;
            let statuses = self.assembly_statuses_value();
            let constraint_failure = statuses.as_array().is_some_and(|a| {
                a.iter()
                    .any(|s| s["enabled"] != false && s["satisfied"] == false)
            });
            let success = failed_items.is_empty()
                && !failed(&final_report)
                && !constraint_failure
                && constraint_errors.is_empty();
            let constraint_results: Vec<Value> = constraint_ids.iter().enumerate().map(|(i,id)| {
                let row = statuses.as_array().and_then(|a| a.iter().find(|r| r["id"].as_str() == Some(id)));
                let state = if constraint_errors.contains_key(id) || row.is_none() || row.is_some_and(|r| r["enabled"] == false) { "not_evaluated" } else if row.is_some_and(|r| r["satisfied"] == false) { "failed" } else { "evaluated" };
                json!({"position":i,"alias":args["constraints"][i]["alias"],"id":id,"status":state,"committed":!constraint_errors.contains_key(id),"error":constraint_errors.get(id),"diagnostics":row,"dependencyFailures":if constraint_errors.contains_key(id) { failed_items.iter().copied().collect::<Vec<_>>() } else { vec![] }})
            }).collect();
            // Keep explicitly supplied definitions, including ones not yet
            // instantiated. Normal library GC still applies on later edits.
            let mut retained_library = brep_kernel::parts_library_map();
            for (alias, _) in &part_work {
                if let Some(entry) = prepared_library.get(alias) {
                    retained_library
                        .entry(alias.clone())
                        .or_insert_with(|| entry.clone());
                }
            }
            brep_kernel::install_parts_library(&retained_library);
            self.history.set_parts_library(
                serde_json::to_value(&retained_library).map_err(|e| e.to_string())?,
            );
            for result in &mut part_results {
                result["committed"] = json!(true);
            }
            self.geometry_diagnostics
                .insert("__report__".into(), final_report);
            Ok(
                json!({"success":success,"items":results,"partResults":part_results,"constraintResults":constraint_results,"aliases":aliases,"partAliases":part_aliases,"outputs":outputs,"diagnosticsTool":"geometry_diagnostics","constraintStatuses":statuses,"dof":self.assembly_dof_value()}),
            )
        })();
        let restore = run
            .as_ref()
            .map(|r| r["requiresRestore"] == true || (rollback && r["success"] != true))
            .unwrap_or(true);
        if restore {
            self.history = before.clone();
            self.metadata = metadata.clone();
            brep_kernel::install_parts_library(&Default::default());
            brep_kernel::install_parts_library(&library);
            self.rerun_history();
            self.finish_batch_run();
            // The restoration rebuild may heal/GC library entries or fold a
            // solver pose. Preserve the exact pre-transaction authored state.
            self.history = before;
            self.metadata = metadata;
            brep_kernel::install_parts_library(&Default::default());
            brep_kernel::install_parts_library(&library);
        } else {
            // Collapse the staged rebuilds into a single user undo checkpoint.
            let final_document = self.history.request_json();
            self.history = before;
            self.history.adopt_document_checkpointed(&final_document)?;
            self.history.retain_feature_allocations(&staged);
            self.history
                .set_rollback(self.history.len().saturating_sub(1));
        }
        let mut result = run?;
        result.as_object_mut().unwrap().remove("requiresRestore");
        result["rolledBack"] = json!(restore);
        result["transaction"] = json!({"validationAtomic":true,"geometryAtomic":rollback,"idCountersRolledBack":restore,"runtimeGenerationRolledBack":false});
        if restore {
            fn uncommit(value: &mut Value) {
                match value {
                    Value::Object(object) => {
                        if object.contains_key("committed") {
                            object.insert("committed".into(), json!(false));
                        }
                        for value in object.values_mut() {
                            uncommit(value);
                        }
                    }
                    Value::Array(array) => {
                        for value in array {
                            uncommit(value);
                        }
                    }
                    _ => (),
                }
            }
            uncommit(&mut result);
        }
        Ok(result)
    }
}

/// Transform strings are expressions, including strings nested in vec3s.
pub(super) fn contains_expression(value: &Value) -> bool {
    match value {
        Value::String(_) => true,
        Value::Array(a) => a.iter().any(contains_expression),
        Value::Object(o) => o.values().any(contains_expression),
        _ => false,
    }
}






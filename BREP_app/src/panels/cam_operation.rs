//! The OPERATION dialog of the CAM pane: one generic form over an operation
//! of the shared model, driven by its strategy's parameter struct rather
//! than written per strategy — the dialog Fusion 360 draws with its Tool /
//! Geometry / Heights / Passes / Linking tabs, here as the form's groups.
//!
//! The **Passes** group is a WALK of the strategy's serialized parameters:
//! a number is a scalar field, a bool a checkbox, a nested struct a sub-group,
//! and a string an enum chosen from the model's own variant list for that
//! field (the tables below, pinned against the model like the strategy
//! kinds). So every one of the model's strategies — the thirty-odd today and
//! the ones lane C0 adds — has a dialog the day its struct exists. The
//! **Linking** group walks `LinkingParams` the same way. A strategy's
//! containment, whose boundary loops and slope / Z ranges are arrays the
//! walk cannot express, is shown as text ([`containment_as_text`]): the
//! boundary as `x y, x y, x y; …` in machine XY, the ranges as `min, max`,
//! empty for none; the text reads back into the model's arrays and is
//! checked by the model's own rule when the form is applied.
//!
//! The other groups are fixed: **Tool** (the tool, from the document's
//! library; optional feed overrides), **Geometry** (the bodies and faces,
//! picked through the engine's CAM target), **Heights** (an optional override
//! of the setup's), and the operation's own name, enabled flag, tolerance and
//! stock to leave.

use brep_cam::model::{ChainRef, ChainSide, ChainSource, Containment, EnumChoice, LinkingParams, Operation, Point3, Strategy, TargetRef, Tool};
use crate::document::cam::{chain_from_names, pick_names, BOUNDARY_PICK_KEY, CHAIN_PICK_KEY};
use brep_render::style::{FieldKind, FormField};
use serde_json::{json, Value};
use std::collections::HashSet;

/// The model's enum choice for the walked field at `path` (`strategy.…` for
/// the strategy kind `kind`'s own fields, anything else from the operation's
/// list): its variants, or `None` for a free-text string.
fn enum_choice(kind: &str, path: &[String]) -> Option<&'static [&'static str]> {
    let (head, rest) = path.split_first()?;
    let dotted = rest.join(".");
    let choices: &[EnumChoice] = if head == "strategy" { Strategy::enum_choices(kind) } else { Operation::ENUM_CHOICES };
    let full = path.join(".");
    choices.iter().find(|c| if head == "strategy" { c.path == dotted } else { c.path == full }).map(|c| c.variants)
}

/// `stepDown` → `Step down`; `passAngleDeg` → `Pass angle deg`.
fn label_of(key: &str) -> String {
    let mut out = String::new();
    for (i, ch) in key.chars().enumerate() {
        if ch.is_ascii_uppercase() {
            out.push(' ');
            out.push(ch.to_ascii_lowercase());
        } else if i == 0 {
            out.push(ch.to_ascii_uppercase());
        } else {
            out.push(ch);
        }
    }
    out.replace(" deg", " (deg)").replace("2 d", "2D").replace("3 d", "3D")
}

/// Walk a serialized struct into form fields under `group`: numbers,
/// bools, enum strings (from the tables, or — for a field that names a
/// TOOL, `referenceTool` — a choice over `tool_ids`) and nested structs;
/// arrays and nulls are left to the model (not editable here).
fn walk(kind: &str, prefix: &[String], value: &Value, group: &str, tool_ids: &[String], out: &mut Vec<FormField>) {
    let Some(object) = value.as_object() else { return };
    // Scalars first, then the nested structs, so a group reads top-down.
    for (key, v) in object {
        let mut path = prefix.to_vec();
        path.push(key.clone());
        let kind = match v {
            Value::Number(_) => FieldKind::Scalar { step: 0.1 },
            Value::Bool(_) => FieldKind::Bool,
            Value::String(_) if key == "referenceTool" => FieldKind::Enum { variants: tool_ids.to_vec() },
            // A string the model lists as an enum field is a choice; any
            // other string (a rotary axis NAME, a probe result prefix) is free text.
            Value::String(_) => match enum_choice(kind, &path) {
                Some(variants) => FieldKind::Enum { variants: variants.iter().map(|s| s.to_string()).collect() },
                None => FieldKind::Text { read_only: false },
            },
            _ => continue,
        };
        let label = match (prefix.last().map(String::as_str), containment_label(key)) {
            (Some("containment"), Some(label)) if CONTAINMENT_TEXT.contains(&key.as_str()) => label.to_string(),
            _ => label_of(key),
        };
        out.push(FormField { path, label, group: group.to_string(), kind });
    }
    for (key, v) in object {
        if v.is_object() {
            let mut path = prefix.to_vec();
            path.push(key.clone());
            walk(kind, &path, v, &format!("{group} \u{00b7} {}", label_of(key)), tool_ids, out);
        }
    }
}

fn field(key: &str, label: &str, group: &str, kind: FieldKind) -> FormField {
    FormField { path: vec![key.to_string()], label: label.to_string(), group: group.to_string(), kind }
}

fn nested(path: &[&str], label: &str, group: &str, kind: FieldKind) -> FormField {
    FormField { path: path.iter().map(|s| s.to_string()).collect(), label: label.to_string(), group: group.to_string(), kind }
}

fn reference(key: &str, label: &str, group: &str, filter: &[&str]) -> FormField {
    nested(&["targets", key], label, group, FieldKind::Reference { filter: filter.iter().map(|s| s.to_string()).collect(), multiple: true })
}

/// The strategy's parameters without its tag, for the walk and the params;
/// every containment's arrays as text ([`containment_as_text`]).
fn strategy_params(strategy: &Strategy) -> Value {
    let mut value = serde_json::to_value(strategy).unwrap_or(Value::Null);
    if let Some(object) = value.as_object_mut() {
        object.remove("kind");
    }
    containment_as_text(&mut value);
    value
}

/// The containment's array fields, edited as text.
const CONTAINMENT_TEXT: [&str; 3] = ["boundary", "slopeDeg", "zRange"];

/// Every `containment` object under `value` with its boundary loops as
/// `x y, x y, x y; …` and its slope / Z ranges as `min, max` (empty: no
/// boundary, every slope, every level), so the form walks them as text.
pub fn containment_as_text(value: &mut Value) {
    let Some(object) = value.as_object_mut() else { return };
    for (key, v) in object.iter_mut() {
        if key == "containment" && v.is_object() {
            let c = v.as_object_mut().expect("checked");
            let loops = c.get("boundary").and_then(Value::as_array).cloned().unwrap_or_default();
            let text = loops
                .iter()
                .map(|l| l.as_array().map(|pts| pts.iter().map(|p| p.as_array().map(|xy| xy.iter().map(number_text).collect::<Vec<_>>().join(" ")).unwrap_or_default()).collect::<Vec<_>>().join(", ")).unwrap_or_default())
                .collect::<Vec<_>>()
                .join("; ");
            c.insert("boundary".into(), json!(text));
            for range in ["slopeDeg", "zRange"] {
                let text = c.get(range).and_then(Value::as_array).map(|r| r.iter().map(number_text).collect::<Vec<_>>().join(", ")).unwrap_or_default();
                c.insert(range.into(), json!(text));
            }
        } else {
            containment_as_text(v);
        }
    }
}

fn number_text(v: &Value) -> String {
    v.as_f64().map(|n| n.to_string()).unwrap_or_default()
}

/// The inverse of [`containment_as_text`] for every `containment` under
/// `value` (`at` is its dotted path): text reads back into the model's
/// arrays (an array — a script's patch — is taken as it is), and each
/// containment must pass the model's own rule. A refusal names the field.
fn containment_from_text(value: &mut Value, at: &str) -> Result<(), String> {
    let Some(object) = value.as_object_mut() else { return Ok(()) };
    for (key, v) in object.iter_mut() {
        let path = format!("{at}.{key}");
        if key == "containment" && v.is_object() {
            let c = v.as_object_mut().expect("checked");
            // A lone number typed into the text (made a number by the form's
            // numeric pass) is read as the text it was.
            if let Some(v) = c.get("boundary").cloned().filter(|v| v.is_string() || v.is_number()) {
                let text = v.as_str().map(str::to_string).unwrap_or_else(|| number_text(&v));
                c.insert("boundary".into(), boundary_from_text(&text).map_err(|e| format!("{path}.boundary: {e}"))?);
            }
            for range in ["slopeDeg", "zRange"] {
                if let Some(v) = c.get(range).cloned().filter(|v| v.is_string() || v.is_number()) {
                    let text = v.as_str().map(str::to_string).unwrap_or_else(|| number_text(&v));
                    c.insert(range.into(), range_from_text(&text).map_err(|e| format!("{path}.{range}: {e}"))?);
                }
            }
            let parsed: Containment = serde_json::from_value(v.clone()).map_err(|e| format!("{path}: {e}"))?;
            parsed.validate().map_err(|e| format!("{path}: {e}"))?;
        } else {
            containment_from_text(v, &path)?;
        }
    }
    Ok(())
}

/// `x y, x y, x y; …` → `[[[x, y], …], …]`; empty → no loops.
fn boundary_from_text(text: &str) -> Result<Value, String> {
    let mut loops = Vec::new();
    for l in text.split(';').map(str::trim).filter(|l| !l.is_empty()) {
        let mut points = Vec::new();
        for p in l.split(',').map(str::trim).filter(|p| !p.is_empty()) {
            let xy: Vec<f64> = p.split_whitespace().map(|t| t.parse::<f64>()).collect::<Result<_, _>>().map_err(|_| format!("`{p}` is not a point `x y`"))?;
            if xy.len() != 2 {
                return Err(format!("`{p}` is not a point `x y`"));
            }
            points.push(json!([xy[0], xy[1]]));
        }
        loops.push(Value::Array(points));
    }
    Ok(Value::Array(loops))
}

/// `min, max` → `[min, max]`; empty → none.
fn range_from_text(text: &str) -> Result<Value, String> {
    if text.trim().is_empty() {
        return Ok(Value::Null);
    }
    let values: Vec<f64> = text
        .split(|c: char| c == ',' || c.is_whitespace())
        .filter(|t| !t.is_empty())
        .map(|t| t.parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|_| format!("`{text}` is not `min, max`"))?;
    match values.as_slice() {
        [lo, hi] => Ok(json!([lo, hi])),
        _ => Err(format!("`{text}` is not `min, max`")),
    }
}

/// The label of a containment text field, saying its format.
fn containment_label(key: &str) -> Option<&'static str> {
    match key {
        "boundary" => Some("Machining boundary (x y, x y, …; next loop — empty: the bodies' outline)"),
        "slopeDeg" => Some("Slope range (deg from horizontal: min, max — empty: every slope)"),
        "zRange" => Some("Z range (machine Z: min, max — empty: every level)"),
        _ => None,
    }
}

/// The form's fields for `operation`, with `tools` as the Tool choice.
pub fn operation_form_fields(operation: &Operation, tools: &[Tool]) -> Vec<FormField> {
    let mut fields = vec![
        field("name", "Name", "Operation", FieldKind::Text { read_only: false }),
        field("enabled", "Enabled", "Operation", FieldKind::Bool),
        // The strategy is chosen here, from the model's kinds; choosing another
        // starts it at that kind's defaults (apply_operation_form).
        field("strategyKind", "Strategy", "Operation", FieldKind::Enum { variants: crate::document::cam::strategy_kinds().iter().map(|k| k.to_string()).collect() }),
        field("tool", "Tool", "Tool", FieldKind::Enum { variants: tools.iter().map(|t| t.id.as_str().to_string()).collect() }),
        field("liveToolMode", "Live tooling", "Tool", FieldKind::Enum { variants: vec!["auto".into(), "live".into(), "off".into()] }),
        field("feedsOverride", "Override the tool's feeds", "Tool", FieldKind::Bool),
        nested(&["feeds", "spindleRpm"], "Spindle speed (rpm)", "Tool", FieldKind::Scalar { step: 10.0 }),
        nested(&["feeds", "cuttingFeed"], "Cutting feed", "Tool", FieldKind::Scalar { step: 10.0 }),
        nested(&["feeds", "plungeFeed"], "Plunge feed", "Tool", FieldKind::Scalar { step: 10.0 }),
        reference("targetSolids", "Bodies to machine", "Geometry", &["SOLID"]),
        reference("driveFaces", "Faces to machine (contact)", "Geometry", &["FACE"]),
        reference("avoidFaces", "Faces to avoid", "Geometry", &["FACE"]),
        reference("protectedSolids", "Protected bodies", "Geometry", &["SOLID"]),
        reference("chainEdges", "Add a chain: edges or sketch curves", "Geometry", &["EDGE"]),
        nested(&["targets", "chainProfile"], "Add a chain: sketch profile", "Geometry", FieldKind::Reference { filter: vec!["FACE".to_string()], multiple: false }),
        field("heightsOverride", "Override the setup's heights", "Heights", FieldKind::Bool),
        nested(&["heights", "clearance"], "Clearance height", "Heights", FieldKind::Scalar { step: 0.5 }),
        nested(&["heights", "retract"], "Retract height", "Heights", FieldKind::Scalar { step: 0.5 }),
        nested(&["heights", "feed"], "Feed height", "Heights", FieldKind::Scalar { step: 0.5 }),
        field("tolerance", "Machining tolerance", "Passes", FieldKind::Scalar { step: 0.001 }),
        nested(&["stockToLeave", "radial"], "Stock to leave (radial)", "Passes", FieldKind::Scalar { step: 0.1 }),
        nested(&["stockToLeave", "axial"], "Stock to leave (axial)", "Passes", FieldKind::Scalar { step: 0.1 }),
    ];
    let tool_ids: Vec<String> = tools.iter().map(|t| t.id.as_str().to_string()).collect();
    fields.extend(chain_fields(operation));
    let kind = operation.strategy.kind_name();
    let strategy = strategy_params(&operation.strategy);
    walk(kind, &["strategy".to_string()], &strategy, "Passes", &tool_ids, &mut fields);
    if strategy.get("containment").is_some_and(Value::is_object) {
        // Its boundary can be picked as a sketch profile, sampled into the
        // text field's loops (machine XY) when the pick commits.
        fields.push(nested(
            &["targets", "boundaryProfile"],
            "Machining boundary from a sketch profile",
            "Passes \u{00b7} Containment",
            FieldKind::Reference { filter: vec!["FACE".to_string()], multiple: false },
        ));
    }
    walk(kind, &["linking".to_string()], &serde_json::to_value(&operation.linking).unwrap_or(Value::Null), "Linking", &tool_ids, &mut fields);
    fields
}

/// The field keys the overrides in `params` hide.
pub fn hidden_fields(params: &Value) -> HashSet<String> {
    let mut hidden = HashSet::new();
    if !params.get("feedsOverride").and_then(Value::as_bool).unwrap_or(false) {
        for key in ["spindleRpm", "cuttingFeed", "plungeFeed"] {
            hidden.insert(key.to_string());
        }
    }
    if !params.get("heightsOverride").and_then(Value::as_bool).unwrap_or(false) {
        for key in ["clearance", "retract", "feed"] {
            hidden.insert(key.to_string());
        }
    }
    hidden
}

fn names(refs: &[TargetRef]) -> Value {
    json!(refs.iter().map(|r| r.face.clone().unwrap_or_else(|| r.solid.clone())).collect::<Vec<_>>())
}

/// The model's choices for a chain field (`side`, `source.kind`), by its
/// `targets.chains[].…` path in `Operation::ENUM_CHOICES`.
fn chain_choice(field: &str) -> Vec<String> {
    let path = format!("targets.chains[].{field}");
    Operation::ENUM_CHOICES.iter().find(|c| c.path == path).map(|c| c.variants.iter().map(|v| v.to_string()).collect()).unwrap_or_default()
}

/// One group of fields per chain, as a list item `targets.chains.<i>.…`:
/// what it is (the source kind, read only — it is what was picked), its
/// curves in words, closed / reversed, the side and offset, an optional
/// start point, a profile's regions, and a Remove button (`removeChain<i>`).
fn chain_fields(operation: &Operation) -> Vec<FormField> {
    let mut out = Vec::new();
    for (i, chain) in operation.targets.chains.iter().enumerate() {
        let group = format!("Geometry \u{00b7} Chain {}", i + 1);
        let at = |key: &[&str]| -> Vec<String> { ["targets", "chains", &i.to_string()].iter().chain(key.iter()).map(|s| s.to_string()).collect() };
        let f = |key: &[&str], label: &str, kind: FieldKind| FormField { path: at(key), label: label.to_string(), group: group.clone(), kind };
        out.push(f(&["source", "kind"], "Chain of", FieldKind::Text { read_only: true }));
        out.push(f(&["summary"], "Curves", FieldKind::Text { read_only: true }));
        out.push(f(&["closed"], "Closed", FieldKind::Bool));
        out.push(f(&["reversed"], "Reversed", FieldKind::Bool));
        out.push(f(&["side"], "Tool side", FieldKind::Enum { variants: chain_choice("side") }));
        out.push(f(&["offset"], "Offset from the chain", FieldKind::Scalar { step: 0.1 }));
        if matches!(chain.source, ChainSource::SketchProfile { .. }) {
            out.push(f(&["regions"], "Regions (indices, empty = all)", FieldKind::Text { read_only: false }));
        }
        out.push(f(&["startSet"], "Start at a point", FieldKind::Bool));
        out.push(f(&["start"], "Start point", FieldKind::Vec3 { step: 0.5 }));
        out.push(FormField {
            path: vec!["targets".to_string(), format!("removeChain{i}")],
            label: "Remove chain".to_string(),
            group: group.clone(),
            kind: FieldKind::Button { label: "Remove chain".to_string() },
        });
    }
    out
}

/// A chain's curves in words, for the read-only `summary` field.
fn chain_summary(chain: &ChainRef) -> String {
    match &chain.source {
        ChainSource::Edges { edges } => {
            let names: Vec<String> = edges.iter().map(|e| if e.solid.is_empty() { e.edge.clone() } else { format!("{} of {}", e.edge, e.solid) }).collect();
            format!("{} edge{}: {}", edges.len(), if edges.len() == 1 { "" } else { "s" }, names.join(", "))
        }
        ChainSource::SketchCurves { sketch, curves } => format!("sketch {sketch}, curve{} {}", if curves.len() == 1 { "" } else { "s" }, curves.join(", ")),
        ChainSource::SketchProfile { sketch, regions } if regions.is_empty() => format!("sketch {sketch}, its whole profile"),
        ChainSource::SketchProfile { sketch, regions } => format!("sketch {sketch}, region{} {}", if regions.len() == 1 { "" } else { "s" }, regions.iter().map(u32::to_string).collect::<Vec<_>>().join(", ")),
    }
}

/// The chains as the form's list: an object keyed by index (form paths are
/// object keys), each the editable fields of one chain.
fn chain_params(chains: &[ChainRef]) -> Value {
    let mut map = serde_json::Map::new();
    for (i, chain) in chains.iter().enumerate() {
        let start = chain.start.unwrap_or_default();
        let mut entry = json!({
            "source": { "kind": serde_json::to_value(&chain.source).ok().and_then(|v| v.get("kind").cloned()).unwrap_or(Value::Null) },
            "summary": chain_summary(chain),
            "closed": chain.closed,
            "reversed": chain.reversed,
            "side": serde_json::to_value(chain.side).unwrap_or(Value::Null),
            "offset": chain.offset,
            "startSet": chain.start.is_some(),
            "start": [start.x, start.y, start.z],
        });
        if let ChainSource::SketchProfile { regions, .. } = &chain.source {
            entry["regions"] = json!(regions.iter().map(u32::to_string).collect::<Vec<_>>().join(", "));
        }
        map.insert(i.to_string(), entry);
    }
    Value::Object(map)
}

/// Apply the form's chain list (`targets.chains`, keyed by index) and any
/// committed pick (`chainEdges`, `chainProfile`, `chainPick`) to `chains`.
/// Edits name their chain (`targets.chains.<i>.side: …`); `remove: true`
/// drops a chain; nothing is applied when any of it is refused.
fn apply_chains(targets: &Value, chains: &mut Vec<ChainRef>) -> Result<(), String> {
    let mut removed: Vec<usize> = Vec::new();
    if let Some(list) = targets.get("chains").and_then(Value::as_object) {
        for (key, entry) in list {
            let i: usize = key.parse().map_err(|_| format!("targets.chains.{key}: not a chain index"))?;
            let chain = chains.get_mut(i).ok_or_else(|| format!("targets.chains.{i}: there is no chain {i}"))?;
            let at = |field: &str| format!("targets.chains.{i}.{field}");
            if let Some(v) = entry.get("closed") {
                chain.closed = v.as_bool().ok_or_else(|| format!("{}: not true or false", at("closed")))?;
            }
            if let Some(v) = entry.get("reversed") {
                chain.reversed = v.as_bool().ok_or_else(|| format!("{}: not true or false", at("reversed")))?;
            }
            if let Some(v) = entry.get("side") {
                chain.side = serde_json::from_value::<ChainSide>(v.clone()).map_err(|e| format!("{}: {e}", at("side")))?;
            }
            if let Some(v) = number_at(entry, &["offset"]).map_err(|e| format!("{}: {e}", at("offset")))? {
                chain.offset = v;
            }
            let start_set = entry.get("startSet").and_then(Value::as_bool).unwrap_or(chain.start.is_some());
            chain.start = if start_set {
                let raw = entry.get("start").cloned().unwrap_or_else(|| json!(chain.start.map(|p| [p.x, p.y, p.z])));
                let xyz: Vec<f64> = raw
                    .as_array()
                    .map(|l| l.iter().filter_map(|c| c.as_f64().or_else(|| c.as_str().and_then(|s| s.trim().parse().ok()))).collect())
                    .unwrap_or_default();
                if xyz.len() != 3 {
                    return Err(format!("{}: three numbers [x, y, z]", at("start")));
                }
                Some(Point3::new(xyz[0], xyz[1], xyz[2]))
            } else {
                None
            };
            if let Some(v) = entry.get("regions") {
                if let ChainSource::SketchProfile { regions, .. } = &mut chain.source {
                    *regions = match v {
                        Value::Number(n) => vec![n.as_u64().ok_or_else(|| format!("{}: {n} is not a region index", at("regions")))? as u32],
                        Value::String(s) => s
                            .split(|c: char| c == ',' || c.is_whitespace())
                            .filter(|t| !t.is_empty())
                            .map(|t| t.parse::<u32>().map_err(|_| format!("{}: `{t}` is not a region index", at("regions"))))
                            .collect::<Result<_, _>>()?,
                        Value::Array(l) => l.iter().map(|x| x.as_u64().map(|n| n as u32).ok_or_else(|| format!("{}: {x} is not a region index", at("regions")))).collect::<Result<_, _>>()?,
                        _ => return Err(format!("{}: region indices like `0, 2`", at("regions"))),
                    };
                }
            }
            if entry.get("remove").and_then(Value::as_bool).unwrap_or(false) {
                removed.push(i);
            }
        }
    }
    removed.sort_unstable();
    for i in removed.into_iter().rev() {
        chains.remove(i);
    }
    for key in ["chainEdges", "chainProfile", CHAIN_PICK_KEY] {
        if let Some(chain) = targets.get(key).and_then(|v| chain_from_names(&pick_names(v))) {
            chains.push(chain);
        }
    }
    // The model's own rules, with a body not yet filled from the scene read
    // as present (the pane fills it; the generator names one that stays empty).
    for (i, chain) in chains.iter().enumerate() {
        let mut probe = chain.clone();
        if let ChainSource::Edges { edges } = &mut probe.source {
            for e in edges.iter_mut().filter(|e| e.solid.is_empty()) {
                e.solid = "?".to_string();
            }
        }
        probe.validate().map_err(|e| format!("targets.chains.{i}: {e}"))?;
    }
    Ok(())
}

/// The form's params for `operation`.
pub fn operation_form_params(operation: &Operation) -> Value {
    let feeds = operation.feeds.clone().unwrap_or_default();
    let heights = operation.heights.clone().unwrap_or_default();
    json!({
        "name": operation.name,
        "enabled": operation.enabled,
        "strategyKind": operation.strategy.kind_name(),
        "tool": operation.tool.as_str(),
        "liveToolMode": match operation.live_tool { None => "auto", Some(true) => "live", Some(false) => "off" },
        "feedsOverride": operation.feeds.is_some(),
        "feeds": { "spindleRpm": feeds.spindle_rpm, "cuttingFeed": feeds.cutting_feed, "plungeFeed": feeds.plunge_feed },
        "targets": {
            "targetSolids": operation.targets.target_solids,
            "driveFaces": names(&operation.targets.drive_faces),
            "avoidFaces": names(&operation.targets.avoid_faces),
            "protectedSolids": operation.targets.protected_solids,
            "chainEdges": [],
            "chainProfile": "",
            "boundaryProfile": "",
            "chains": chain_params(&operation.targets.chains),
        },
        "heightsOverride": operation.heights.is_some(),
        "heights": { "clearance": heights.clearance, "retract": heights.retract, "feed": heights.feed },
        "tolerance": operation.tolerance,
        "stockToLeave": serde_json::to_value(&operation.stock_to_leave).unwrap_or(Value::Null),
        "strategy": strategy_params(&operation.strategy),
        "linking": serde_json::to_value(&operation.linking).unwrap_or(Value::Null),
    })
}

/// Numeric strings the scalar field writes become numbers, recursively, so
/// the model's structs deserialize; anything else is left as it is.
fn numbers_in(value: &mut Value) {
    match value {
        Value::Object(map) => map.values_mut().for_each(numbers_in),
        Value::Array(list) => list.iter_mut().for_each(numbers_in),
        Value::String(s) => {
            if let Ok(n) = s.trim().parse::<f64>() {
                if let Some(n) = serde_json::Number::from_f64(n) {
                    *value = Value::Number(n);
                }
            }
        }
        _ => {}
    }
}

/// Deep-merge `patch` into `base` (objects merge, everything else replaces).
fn merge(base: &mut Value, patch: &Value) {
    match (base, patch) {
        (Value::Object(b), Value::Object(p)) => {
            for (k, v) in p {
                match b.get_mut(k) {
                    Some(slot) if slot.is_object() && v.is_object() => merge(slot, v),
                    _ => {
                        b.insert(k.clone(), v.clone());
                    }
                }
            }
        }
        (b, p) => *b = p.clone(),
    }
}

fn face_refs(list: &Value, current: &[TargetRef]) -> Vec<TargetRef> {
    list.as_array()
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_str)
                .map(|name| {
                    // A name the operation already resolved keeps its solid.
                    current
                        .iter()
                        .find(|r| r.face.as_deref() == Some(name))
                        .cloned()
                        .unwrap_or_else(|| TargetRef { solid: String::new(), face: Some(name.to_string()) })
                })
                .collect()
        })
        .unwrap_or_default()
}

fn number_at(value: &Value, path: &[&str]) -> Result<Option<f64>, String> {
    let mut cur = value;
    for seg in path {
        match cur.get(seg) {
            Some(v) => cur = v,
            None => return Ok(None),
        }
    }
    match cur {
        Value::Null => Ok(None),
        Value::Number(n) => Ok(n.as_f64()),
        Value::String(s) if s.trim().is_empty() => Ok(None),
        Value::String(s) => s.trim().parse::<f64>().map(Some).map_err(|_| format!("{}: `{s}` is not a number", path.join("."))),
        other => Err(format!("{}: {other} is not a number", path.join("."))),
    }
}

/// Apply the form's params (or a PATCH of them, deep-merged: absent keys
/// unchanged) to `operation`. Nothing is applied unless the strategy, the
/// linking, the heights, the feeds and the stock to leave read back as the
/// model's types and pass their own value checks; a refusal names the field
/// or the group. The strategy's GEOMETRY rule (a morph needs its two
/// boundaries, a trace its chain) is not a refusal here — a default
/// operation has none yet and every edit before the pick would be refused
/// for it — it is [`strategy_problem`], shown in the dialog and reported by
/// `cam_state`, and the generator's error until it is met.
pub fn apply_operation_form(params: &Value, operation: &mut Operation, tools: &[Tool]) -> Result<(), String> {
    let mut merged = operation_form_params(operation);
    merge(&mut merged, params);
    numbers_in(&mut merged);
    let mut next = operation.clone();
    next.live_tool = if let Some(value) = params.get("liveTool") {
        serde_json::from_value(value.clone()).map_err(|e| format!("liveTool: {e}"))?
    } else {
        match merged.get("liveToolMode").and_then(Value::as_str).ok_or("liveToolMode: expected auto, live or off")? {
            "auto" => None, "live" => Some(true), "off" => Some(false),
            _ => return Err("liveToolMode: expected auto, live or off".into()),
        }
    };
    if let Some(name) = merged.get("name").and_then(Value::as_str) {
        if !name.trim().is_empty() {
            next.name = name.to_string();
        }
    }
    if let Some(enabled) = merged.get("enabled").and_then(Value::as_bool) {
        next.enabled = enabled;
    }
    if let Some(tool) = merged.get("tool").and_then(Value::as_str) {
        if !tools.iter().any(|t| t.id.as_str() == tool) {
            return Err(format!("tool: no tool `{tool}` in the document"));
        }
        next.tool = tool.into();
    }
    next.feeds = if merged.get("feedsOverride").and_then(Value::as_bool).unwrap_or(false) {
        let mut feeds = next.feeds.clone().unwrap_or_default();
        if let Some(v) = number_at(&merged, &["feeds", "spindleRpm"])? {
            feeds.spindle_rpm = v;
        }
        if let Some(v) = number_at(&merged, &["feeds", "cuttingFeed"])? {
            feeds.cutting_feed = v;
        }
        if let Some(v) = number_at(&merged, &["feeds", "plungeFeed"])? {
            feeds.plunge_feed = v;
        }
        feeds.validate().map_err(|e| format!("feeds: {e}"))?;
        Some(feeds)
    } else {
        None
    };
    next.heights = if merged.get("heightsOverride").and_then(Value::as_bool).unwrap_or(false) {
        let mut heights = next.heights.clone().unwrap_or_default();
        if let Some(v) = number_at(&merged, &["heights", "clearance"])? {
            heights.clearance = v;
        }
        if let Some(v) = number_at(&merged, &["heights", "retract"])? {
            heights.retract = v;
        }
        if let Some(v) = number_at(&merged, &["heights", "feed"])? {
            heights.feed = v;
        }
        heights.validate().map_err(|e| format!("heights: {e}"))?;
        Some(heights)
    } else {
        None
    };
    if let Some(targets) = merged.get("targets") {
        let strings = |key: &str| -> Vec<String> {
            targets.get(key).and_then(Value::as_array).map(|l| l.iter().filter_map(Value::as_str).map(str::to_string).collect()).unwrap_or_default()
        };
        if targets.get("targetSolids").is_some() {
            next.targets.target_solids = strings("targetSolids");
        }
        if targets.get("protectedSolids").is_some() {
            next.targets.protected_solids = strings("protectedSolids");
        }
        if let Some(list) = targets.get("driveFaces") {
            next.targets.drive_faces = face_refs(list, &operation.targets.drive_faces);
        }
        if let Some(list) = targets.get("avoidFaces") {
            next.targets.avoid_faces = face_refs(list, &operation.targets.avoid_faces);
        }
        apply_chains(targets, &mut next.targets.chains)?;
    }
    if let Some(v) = number_at(&merged, &["tolerance"])? {
        if !v.is_finite() || v <= 0.0 {
            return Err(format!("tolerance: {v} is not a positive tolerance"));
        }
        next.tolerance = v;
    }
    if let Some(stl) = merged.get("stockToLeave") {
        next.stock_to_leave = serde_json::from_value(stl.clone()).map_err(|e| format!("stockToLeave: {e}"))?;
        next.stock_to_leave.validate().map_err(|e| format!("stockToLeave: {e}"))?;
    }
    if let Some(linking) = merged.get("linking") {
        next.linking = serde_json::from_value::<LinkingParams>(linking.clone()).map_err(|e| format!("linking: {e}"))?;
    }
    let kind = merged.get("strategyKind").and_then(Value::as_str).unwrap_or(operation.strategy.kind_name());
    if kind != operation.strategy.kind_name() {
        // A different strategy: its own parameters at the model's defaults;
        // the old kind's parameters do not carry over.
        next.strategy = crate::document::cam::strategy_of_kind(kind).map_err(|e| format!("strategyKind: {e}"))?;
        // An operation still carrying its automatic name (`<kind> <n>`, from
        // New operation) is renamed with its new kind; a name the user gave
        // is kept.
        let old = format!("{} ", operation.strategy.kind_name());
        if let Some(n) = next.name.strip_prefix(&old) {
            if !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()) && merged.get("name").and_then(Value::as_str) == Some(operation.name.as_str()) {
                next.name = format!("{} {n}", next.strategy.kind_name());
            }
        }
    } else if let Some(strategy) = merged.get("strategy") {
        let mut tagged = strategy.clone();
        containment_from_text(&mut tagged, "strategy")?;
        if let Some(object) = tagged.as_object_mut() {
            object.insert("kind".into(), json!(operation.strategy.kind_name()));
        }
        next.strategy = serde_json::from_value::<Strategy>(tagged).map_err(|e| format!("strategy: {e}"))?;
    }
    *operation = next;
    Ok(())
}

/// What the strategy's own rule says is still wrong with the operation (a
/// boundary not yet picked, a non-positive step), or `None` when it would
/// generate.
pub fn strategy_problem(operation: &Operation) -> Option<String> {
    operation.strategy.validate().err()
}

/// Where a reference field's pick commits: the block path from the root,
/// the entity kinds and the label. `None` for a key that is not one.
pub fn pick_path(operation_id: &str, key: &str) -> Option<(Vec<String>, Vec<String>, bool, String)> {
    let path = |k: &str| vec!["operations".to_string(), format!("#{operation_id}"), "targets".to_string(), k.to_string()];
    let kinds = |k: &[&str]| k.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    Some(match key {
        "targetSolids" => (path(key), kinds(&["SOLID"]), true, "Bodies to machine".into()),
        "driveFaces" => (path(key), kinds(&["FACE"]), true, "Faces to machine".into()),
        "avoidFaces" => (path(key), kinds(&["FACE"]), true, "Faces to avoid".into()),
        "protectedSolids" => (path(key), kinds(&["SOLID"]), true, "Protected bodies".into()),
        // Both chain fields commit to the staging key the load folds into a new chain.
        "chainEdges" => (path(CHAIN_PICK_KEY), kinds(&["EDGE"]), true, "A chain: edges or sketch curves".into()),
        "chainProfile" => (path(CHAIN_PICK_KEY), kinds(&["FACE"]), false, "A chain: sketch profile".into()),
        // Staged like a chain pick; the pane samples it into the boundary.
        "boundaryProfile" => (path(BOUNDARY_PICK_KEY), kinds(&["FACE"]), false, "Machining boundary: sketch profile".into()),
        _ => return None,
    })
}

/// Fill the owning solid of every face reference that has none, through
/// `owner` (a face name → its solid's name, from the scene). Returns whether
/// anything changed; a face no solid owns is left for the generator to name.
pub fn resolve_face_owners(operation: &mut Operation, owner: &dyn Fn(&str) -> Option<String>) -> bool {
    let mut changed = false;
    for list in [&mut operation.targets.drive_faces, &mut operation.targets.avoid_faces] {
        for r in list.iter_mut() {
            if r.solid.is_empty() {
                if let Some(solid) = r.face.as_deref().and_then(owner) {
                    r.solid = solid;
                    changed = true;
                }
            }
        }
    }
    changed
}

/// Fill the owning body of every edge of `operation`'s edge chains that has
/// none, through `owner` (an edge name → its solid's name, from the scene).
pub fn resolve_edge_owners(operation: &mut Operation, owner: &dyn Fn(&str) -> Option<String>) -> bool {
    let mut changed = false;
    for chain in operation.targets.chains.iter_mut() {
        if let ChainSource::Edges { edges } = &mut chain.source {
            for e in edges.iter_mut().filter(|e| e.solid.is_empty()) {
                if let Some(solid) = owner(&e.edge) {
                    e.solid = solid;
                    changed = true;
                }
            }
        }
    }
    changed
}

/// Every strategy's parameter walk, for the tests and the record: the kind
/// and how many editable fields its struct gives the dialog.
pub fn walk_counts(kinds: &[&str]) -> Vec<(String, usize)> {
    kinds
        .iter()
        .filter_map(|kind| {
            let strategy: Strategy = serde_json::from_value(json!({ "kind": kind })).ok()?;
            let mut out = Vec::new();
            walk(kind, &["strategy".to_string()], &strategy_params(&strategy), "Passes", &[], &mut out);
            Some((kind.to_string(), out.len()))
        })
        .collect()
}


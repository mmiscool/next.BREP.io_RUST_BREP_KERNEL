//! Reading a wizard [`Level`] straight from an assembly document's JSON.
//!
//! An assembly document lists its placements as `ACOMP` features
//! (`inputParams.id`, `partName`, `isFixed`, `bom.Find_Number`), embeds each
//! placed part in its `partsLibrary` (with `sourceSignature` and the part's own
//! document, whose `partAttributes` say `Fastener`, `Torque` and
//! `Serial_Tracked`), keeps its mates in `assembly.constraints` (each naming
//! entities by `elements`), and its exploded views as PMI views with `explode`
//! rows. A placed part whose own document has placements is a sub-assembly,
//! read the same way one level down.

use crate::model::{FASTENER_ATTRIBUTE, SERIAL_TRACKED_ATTRIBUTE, TORQUE_ATTRIBUTE};
use crate::wizard::{Component, Level};
use serde_json::Value;

/// How deep sub-assemblies are read.
pub const MAX_DEPTH: usize = 16;

/// The level of `document`, its placements' paths under `prefix`
/// (`"ACOMP1"` when the document is placed as `ACOMP1`, `""` at the top), the
/// level itself at `path` (`""` for the target assembly).
pub fn level_from_document(document: &Value, prefix: &str, path: &str, name: &str) -> Level {
    read_level(document, prefix, path, name, 0)
}

fn read_level(document: &Value, prefix: &str, path: &str, name: &str, depth: usize) -> Level {
    let library = &document["partsLibrary"];
    let mut components = Vec::new();
    let mut ids = Vec::new();
    let mut first = true;
    for feature in document["features"].as_array().into_iter().flatten() {
        let kind = feature["type"].as_str().unwrap_or_default();
        if !matches!(kind, "ACOMP" | "ASSEMBLY COMPONENT") || feature["suppressed"].as_bool() == Some(true) {
            continue;
        }
        let params = &feature["inputParams"];
        let Some(id) = params["id"].as_str() else { continue };
        let part = params["partName"].as_str().unwrap_or_default().to_string();
        // The kernel's rule: an explicit isFixed wins, else the first
        // placement is grounded.
        let grounded = params["isFixed"].as_bool().unwrap_or(first);
        first = false;
        let entry = &library[part.as_str()];
        let part_document = &entry["document"];
        let attributes = &part_document["partAttributes"];
        let component_path = join(prefix, id);
        let children = (depth + 1 < MAX_DEPTH && has_placements(part_document))
            .then(|| Box::new(read_level(part_document, &component_path, &component_path, &part, depth + 1)));
        components.push(Component {
            path: component_path,
            part_id: entry["sourceKey"].as_str().and_then(plm_part_id),
            part,
            signature: entry["sourceSignature"].as_str().unwrap_or_default().to_string(),
            find_number: text(&params["bom"]["Find_Number"]),
            grounded,
            fastener: truthy(&attributes[FASTENER_ATTRIBUTE]),
            serial_tracked: truthy(&attributes[SERIAL_TRACKED_ATTRIBUTE]),
            torque: Some(text(&attributes[TORQUE_ATTRIBUTE])).filter(|t| !t.is_empty()),
            children,
        });
        ids.push(id.to_string());
    }
    let owner = |element: &str| -> Option<String> {
        let head = element.split(':').next()?;
        ids.iter().any(|id| id == head).then(|| join(prefix, head))
    };
    let mut mates = Vec::new();
    for constraint in document["assembly"]["constraints"].as_array().into_iter().flatten() {
        if constraint["enabled"].as_bool() == Some(false) {
            continue;
        }
        let owners: Vec<String> = constraint["inputParams"]["elements"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
            .filter_map(owner)
            .collect();
        for pair in owners.windows(2) {
            if pair[0] != pair[1] {
                mates.push((pair[0].clone(), pair[1].clone()));
            }
        }
    }
    let mut exploded_order: Vec<String> = Vec::new();
    for view in document["pmi"]["views"].as_array().into_iter().flatten() {
        let rows: Vec<&Value> = view["annotations"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|a| a["type"].as_str() == Some("explode") && a["enabled"].as_bool() != Some(false))
            .collect();
        if rows.is_empty() {
            continue;
        }
        for row in rows {
            for target in references(&row["inputParams"]["targets"]) {
                if let Some(owner) = owner(&target) {
                    if !exploded_order.contains(&owner) {
                        exploded_order.push(owner);
                    }
                }
            }
        }
        // The first exploded view seeds the order.
        break;
    }
    Level { path: path.to_string(), name: name.to_string(), components, mates, exploded_order }
}

/// The PLM part id a library entry's source key names: `part/<id>/rev/<rev>`
/// (a PLM session's key) or the explorer's path form of it
/// (`/models/part/<id>/rev/<rev>.nbrep`); `None` for a file's key.
pub fn plm_part_id(key: &str) -> Option<String> {
    let rest = key.find("part/").map(|at| &key[at + "part/".len()..])?;
    let (id, _) = rest.split_once("/rev/")?;
    (!id.is_empty() && !id.contains('/')).then(|| id.to_string())
}

/// Whether a document places parts (is an assembly).
pub fn has_placements(document: &Value) -> bool {
    document["features"]
        .as_array()
        .is_some_and(|features| features.iter().any(|f| matches!(f["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT"))))
}

fn join(prefix: &str, id: &str) -> String {
    if prefix.is_empty() {
        id.to_string()
    } else {
        format!("{prefix}:{id}")
    }
}

/// A reference field: one name or an array of names.
fn references(value: &Value) -> Vec<String> {
    match value {
        Value::String(name) if !name.trim().is_empty() => vec![name.trim().to_string()],
        Value::Array(items) => items.iter().filter_map(Value::as_str).map(str::to_string).collect(),
        _ => Vec::new(),
    }
}

fn text(value: &Value) -> String {
    match value {
        Value::String(s) => s.trim().to_string(),
        Value::Number(n) => n.to_string(),
        _ => String::new(),
    }
}

/// An attribute's yes: `true`, or text `true` / `yes` / `1` / `y` (any case).
fn truthy(value: &Value) -> bool {
    match value {
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64().is_some_and(|v| v != 0.0),
        Value::String(s) => matches!(s.trim().to_ascii_lowercase().as_str(), "true" | "yes" | "1" | "y"),
        _ => false,
    }
}


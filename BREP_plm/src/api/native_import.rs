//! The final write of the native-model import wizard. Allocation and checkout
//! use the normal APIs; document and BOM land together with a reviewed hash.
use super::{require_author, Shared};
use crate::Error;
use axum::{
    body::Body,
    extract::{Path, State},
    http::HeaderMap,
    Json,
};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Import {
    expected_hash: String,
    document: Value,
}

#[derive(Deserialize)]
pub struct PlannedNumber {
    part_type: String,
    number: String,
}

/// Validate typed numbers with the SAME Rust pattern implementation used by
/// allocation. Script numbers are deliberately assigned only when committing.
pub async fn validate(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(parts): Json<Vec<PlannedNumber>>,
) -> Result<Json<Value>, Error> {
    require_author(&db, &headers)?;
    db.read(|state| {
        let mut numbers = std::collections::BTreeSet::new();
        for part in parts {
            let kind = state
                .part_type(&part.part_type)
                .ok_or_else(|| Error::bad_request("unknown part type"))?;
            let typed = part.number.trim();
            match &kind.mode {
                crate::model::NumberMode::Counter => {
                    if !typed.is_empty() {
                        return Err(Error::bad_request("counter types assign their own numbers"));
                    }
                }
                crate::model::NumberMode::Script { .. } => continue,
                mode => {
                    crate::db::check_number(typed)?;
                    if let crate::model::NumberMode::Pattern { regex } = mode {
                        if !crate::db::full_match(regex)?.is_match(typed) {
                            return Err(Error::bad_request(format!(
                                "{typed} does not match {} ({regex})",
                                kind.name
                            )));
                        }
                    }
                    if state.part_with_number(typed).is_some()
                        || !numbers.insert(typed.to_ascii_lowercase())
                    {
                        return Err(Error::conflict(format!(
                            "part number {typed} is already used"
                        )));
                    }
                }
            }
        }
        Ok(Json(json!({"ok": true})))
    })
}

/// Refresh the engine's sorted-key signatures after nested references change.
/// Reject unresolved components before any document is written.
fn prepare(document: &mut Value, depth: usize) -> Result<Value, Error> {
    if depth > 64 {
        return Err(Error::bad_request("assembly nesting exceeds 64 levels"));
    }
    if !document.get("features").is_some_and(Value::is_array) {
        return Err(Error::bad_request(
            "a native model must contain a features array",
        ));
    }
    if let Some(library) = document.get_mut("partsLibrary") {
        let library = library
            .as_object_mut()
            .ok_or_else(|| Error::bad_request("partsLibrary must be an object"))?;
        for entry in library.values_mut() {
            if let Some(nested) = entry.get_mut("document") {
                prepare(nested, depth + 1)?;
                let signature = format!("{:016x}", crate::seed::stable_json_hash(nested));
                entry["sourceSignature"] = json!(signature);
            }
        }
    }
    let mut uses: Vec<Value> = Vec::new();
    for feature in document["features"].as_array().unwrap() {
        if !matches!(
            feature["type"].as_str(),
            Some("ACOMP" | "ASSEMBLY COMPONENT")
        ) {
            continue;
        }
        let params = &feature["inputParams"];
        let name = params["partName"].as_str().unwrap_or_default();
        let key = document["partsLibrary"][name]["sourceKey"]
            .as_str()
            .unwrap_or_default();
        let (part, revision) = crate::identity::split_key(key).ok_or_else(|| Error::bad_request(format!("component {name} has no PLM revision")))?;
        let reference = params["bom"]["Reference_Designator"]
            .as_str()
            .filter(|s| !s.trim().is_empty())
            .or(params["id"].as_str())
            .unwrap_or(name);
        if let Some(line) = uses
            .iter_mut()
            .find(|line| line["part"] == part && line["revision"] == revision)
        {
            line["quantity"] = json!(line["quantity"].as_u64().unwrap() + 1);
            line["reference"] = json!(format!(
                "{}, {reference}",
                line["reference"].as_str().unwrap()
            ));
        } else {
            uses.push(json!({"part": part, "revision": revision, "quantity": 1, "unit": "each", "find_number": "", "reference": reference, "notes": ""}));
        }
    }
    Ok(json!({"uses": uses}))
}

pub async fn write(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((part, revision)): Path<(String, String)>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let limit = db.security().config.document_limit();
    let text = super::store::read_bounded(&headers, body, limit, || {
        "the import exceeds the document size limit".into()
    })
    .await?;
    let mut request: Import =
        serde_json::from_str(&text).map_err(|e| Error::bad_request(e.to_string()))?;
    let uses = prepare(&mut request.document, 0)?;
    let text = serde_json::to_string(&request.document).map_err(Error::internal)?;
    let seq = db.write_import_document(
        &user,
        &part,
        &revision,
        &request.expected_hash,
        &text,
        &uses,
    )?;
    Ok(Json(
        json!({"ok": true, "seq": seq, "content_hash": db.read(|state|state.part(&part).and_then(|p|p.revision(&revision)).map(|r|r.content_hash.clone()).unwrap_or_default())}),
    ))
}

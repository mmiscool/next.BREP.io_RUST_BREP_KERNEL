//! A document with no PLM item of its own gets one (owner, 2026-10-21: a part
//! is never kept only inside an assembly). The assembly's Save on a PLM makes
//! the items, then writes each document through the store like any other
//! revision ([`crate::document::owned_save`]).

use std::rc::Rc;

use super::client::PlmClient;
use crate::panels::plm_parts::CreatedPart;

/// The part type a promoted document is filed under: its own kind, the rule
/// the PLM import wizard uses — `assembly` when it places components, else
/// `component`.
pub fn part_type_of(document: &serde_json::Value) -> &'static str {
    let assembly = document["features"].as_array().is_some_and(|f| f.iter().any(|f| f["type"] == "ACOMP"));
    if assembly { "assembly" } else { "component" }
}

/// Make each requested `(tag, name, part type)` part on the PLM and check its
/// first revision out to this client, so the caller can write its document
/// through the store. Answers each revision's store key, in request order. A
/// server without the requested part type gets a `component`. Each part is
/// linked into the user's workspace folder `folder` (its id; empty: the top).
pub async fn make_items(client: Rc<PlmClient>, requests: Vec<(String, String, String)>, folder: String) -> Result<Vec<String>, String> {
    let body = |value: serde_json::Value| Some(serde_json::to_vec(&value).unwrap_or_default());
    let mut keys = Vec::new();
    for (_, name, part_type) in requests {
        let create = |part_type: &str| client.call("POST", "/api/parts", body(serde_json::json!({ "part_type": part_type, "name": name, "document_class": "normal", "workspace_folder": folder })));
        let response = match create(&part_type).await {
            Ok(response) => response,
            Err(_) if part_type != "component" => create("component").await.map_err(|e| format!("'{name}': {e}"))?,
            Err(error) => return Err(format!("'{name}': {error}")),
        };
        let part: CreatedPart = serde_json::from_slice(&response.body).map_err(|e| format!("'{name}': the new part's answer: {e}"))?;
        let revision = part.revisions.first().ok_or_else(|| format!("'{name}': the new part has no revision"))?;
        let path = format!("{}/checkout", crate::plm::identity::revision_path(&part.id, &revision.id));
        client.call("POST", &path, body(serde_json::json!({ "client_id": "brep-app" }))).await.map_err(|e| format!("'{name}': {e}"))?;
        keys.push(crate::plm::identity::document_key(&part.id, &revision.id));
    }
    Ok(keys)
}

/// The one folder of the user's own workspace that links `part` (its id), as
/// `(id, path)`; the top is `("", "")`. Where the items an assembly's Save
/// makes are filed when it was opened some other way than from a workspace
/// folder (by number, from a search). `None` when no folder links it, when
/// several do (which one is not ours to guess), or when the lookup fails.
pub async fn linking_folder(client: &PlmClient, part: &str) -> Option<(String, String)> {
    let path = format!("/api/workspace/links?part={}", crate::plm::identity::segment(part));
    let answer = client.call("GET", &path, None).await.ok()?;
    let answer: serde_json::Value = serde_json::from_slice(&answer.body).ok()?;
    match answer["folders"].as_array()?.as_slice() {
        [one] => Some((one["id"].as_str()?.to_string(), one["path"].as_str()?.to_string())),
        _ => None,
    }
}


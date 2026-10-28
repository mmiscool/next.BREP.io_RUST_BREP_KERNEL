//! Derived geometry metadata. Feature names alone never establish geometry.
use std::collections::BTreeMap;

/// Take every embedded part's cached evaluation (`snapshot`, and the nested
/// rows taken with it) as this build's own, at any depth.
///
/// The kernel serves a snapshot only to the build that made it, and rebuilds
/// any other from the part's history. A CAD app built from another kernel
/// stamps every snapshot it saves with ITS build, so without this a server one
/// build apart rebuilt every embedded part of every document on every write:
/// slower with each part (a debug server lost a Save As outright), and a part
/// only its snapshot can show answered differently. Whether a saved model has
/// geometry is the client's evaluation to report; a different build may only
/// cost the cache, never change the answer.
fn trust_cached_parts(document: &mut serde_json::Value, stamp: &str) {
    let Some(library) = document.get_mut("partsLibrary").and_then(serde_json::Value::as_object_mut) else { return };
    for entry in library.values_mut() {
        if entry.get("snapshot").and_then(serde_json::Value::as_str).is_some_and(|s| !s.is_empty()) {
            entry["snapshotProducer"] = serde_json::Value::String(stamp.into());
            if let Some(rows) = entry.get_mut("nestedRows").and_then(serde_json::Value::as_object_mut) {
                rows.insert("producer".into(), serde_json::Value::String(stamp.into()));
            }
        }
        if let Some(inner) = entry.get_mut("document") {
            trust_cached_parts(inner, stamp);
        }
    }
}

/// The request [`contains_geometry`] runs for `body`: the whole history, with
/// every embedded part's cached evaluation taken as this build's own
/// (`trust_cached_parts`). `None` when the body is not a model.
pub fn geometry_request(body: &str) -> Option<brep_kernel::HistoryRequest> {
    let mut document = serde_json::from_str::<serde_json::Value>(body).ok()?;
    if let Some(stamp) = brep_kernel::kernel_source_stamp() {
        trust_cached_parts(&mut document, stamp);
    }
    let mut request = serde_json::from_value::<brep_kernel::HistoryRequest>(document).ok()?;
    request.stop_at_id = None;
    request.stop_before_id = None;
    Some(request)
}

/// Evaluate a saved model in an isolated kernel thread. Inspect the final
/// scene, so consumed bodies, failed features and empty imports are handled.
/// Pure native imports can be inspected without executing a feature history.
pub fn contains_geometry(body: &str) -> bool {
    let Some(request) = geometry_request(body) else { return false };
    if request.features.is_empty() { return false; }
    if request.features.iter().all(|f| f.feature_type == "IMPORT3D" && f.input_params["nativeBrep"].is_string()) {
        return request.features.iter().any(|f| {
            brep_kernel::restore_solids(f.input_params["nativeBrep"].as_str().unwrap())
                .is_ok_and(|snapshot| snapshot.solids.iter().any(|s| s.solid.shells.iter().any(|shell| !shell.faces.is_empty())))
        });
    }
    std::thread::spawn(move || {
        let result = brep_kernel::execute_history(&request);
        let mut scene = BTreeMap::new();
        for feature in result.results {
            for name in feature.removed { scene.remove(&name); }
            for solid in feature.added { scene.insert(solid.name, solid.handle); }
        }
        let geometry = scene.values().any(|handle| {
            brep_kernel::registered_solid_clone(*handle)
                .is_ok_and(|solid| solid.shells.iter().any(|shell| !shell.faces.is_empty()))
        });
        brep_kernel::clear_history_cache();
        geometry
    }).join().unwrap_or(false)
}

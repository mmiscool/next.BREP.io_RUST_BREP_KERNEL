//! Derived geometry metadata. Feature names alone never establish geometry.
use std::collections::BTreeMap;

/// Evaluate a saved model in an isolated kernel thread. Inspect the final
/// scene, so consumed bodies, failed features and empty imports are handled.
/// Pure native imports can be inspected without executing a feature history.
pub fn contains_geometry(body: &str) -> bool {
    let Ok(mut request) = serde_json::from_str::<brep_kernel::HistoryRequest>(body) else { return false };
    request.stop_at_id = None;
    request.stop_before_id = None;
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

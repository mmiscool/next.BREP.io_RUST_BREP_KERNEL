//! Scene metadata — the kernel-owned name-keyed key/value store.
//!
//! The port of the retired `MetadataManager`: arbitrary metadata records against
//! scene object NAMES (material assignments, layers, weld callouts, simulation
//! attributes …) with single-parent inheritance via an `inheritsFrom` key —
//! resolution merges the parent chain, child values winning. The caller side is
//! now a thin proxy over these wasm exports; the store itself lives HERE and
//! serializes with the part through [`scene_metadata_dump_json`] /
//! [`scene_metadata_load_json`].
//!
//! TOPOLOGY metadata (per-face / per-edge: cap roles, sourceFeatureId, thread
//! info, bend data) uses the same store keyed by the face/edge NAME — the
//! kernel's persistent identity. Because names propagate through booleans on
//! the records themselves, name-keyed metadata follows topology with zero
//! propagation machinery. The pipeline stamps it (`stamp_face_metadata`);
//! the caller's display layer reads it back per solid via
//! `topo_metadata_for_names_json`.

use std::cell::RefCell;
use std::collections::HashMap;
use wasm_bindgen::prelude::*;

pub(crate) type Record_ = serde_json::Map<String, serde_json::Value>;

thread_local! {
    static SCENE_METADATA: RefCell<HashMap<String, Record_>> = RefCell::new(HashMap::new());
}

/// Merge the inheritance chain for `name` (child wins; cycle-guarded).
fn resolve(store: &HashMap<String, Record_>, name: &str) -> Record_ {
    let mut chain: Vec<&Record_> = Vec::new();
    let mut seen: Vec<String> = Vec::new();
    let mut current = name.to_string();
    while let Some(record) = store.get(&current) {
        if seen.contains(&current) {
            break; // inheritance cycle — stop
        }
        seen.push(current.clone());
        chain.push(record);
        match record.get("inheritsFrom").and_then(|v| v.as_str()) {
            Some(parent) if !parent.is_empty() => current = parent.to_string(),
            _ => break,
        }
    }
    let mut merged = Record_::new();
    for record in chain.iter().rev() {
        for (key, value) in record.iter() {
            merged.insert(key.clone(), value.clone());
        }
    }
    merged
}

/// Set (replace) the metadata record for a scene object name.
#[wasm_bindgen]
pub fn scene_metadata_set_json(name: &str, record_json: &str) -> Result<(), JsValue> {
    let record: Record_ = serde_json::from_str(record_json)
        .map_err(|error| JsValue::from_str(&format!("scene_metadata_set: {error}")))?;
    SCENE_METADATA.with(|store| {
        let mut store = store.borrow_mut();
        if record.is_empty() {
            store.remove(name);
        } else {
            store.insert(name.to_string(), record);
        }
    });
    Ok(())
}

/// The RESOLVED metadata for a name (inheritance chain merged, child wins).
#[wasm_bindgen]
pub fn scene_metadata_get_json(name: &str) -> String {
    SCENE_METADATA.with(|store| {
        serde_json::Value::Object(resolve(&store.borrow(), name)).to_string()
    })
}

/// The OWN metadata record only (no inheritance).
#[wasm_bindgen]
pub fn scene_metadata_get_own_json(name: &str) -> String {
    SCENE_METADATA.with(|store| {
        store
            .borrow()
            .get(name)
            .map(|record| serde_json::Value::Object(record.clone()).to_string())
            .unwrap_or_else(|| "{}".to_string())
    })
}

/// Remove a whole record.
#[wasm_bindgen]
pub fn scene_metadata_remove(name: &str) {
    SCENE_METADATA.with(|store| {
        store.borrow_mut().remove(name);
    });
}

/// The whole store (for part serialization).
#[wasm_bindgen]
pub fn scene_metadata_dump_json() -> String {
    SCENE_METADATA.with(|store| {
        serde_json::to_string(&*store.borrow()).unwrap_or_else(|_| "{}".to_string())
    })
}

/// Replace the whole store (part load / reset — pass `{}` to clear).
#[wasm_bindgen]
pub fn scene_metadata_load_json(all_json: &str) -> Result<(), JsValue> {
    let all: HashMap<String, Record_> = serde_json::from_str(all_json)
        .map_err(|error| JsValue::from_str(&format!("scene_metadata_load: {error}")))?;
    SCENE_METADATA.with(|store| {
        *store.borrow_mut() = all;
    });
    Ok(())
}

/// Replace a record WHOLESALE (empty removes it) — the ACOMP stamping entry.
/// A restored component's records are authoritative for its namespaced
/// entities each run; merging instead would leave residue when a refreshed
/// part's record DROPPED keys.
pub(crate) fn set_record(name: &str, record: serde_json::Map<String, serde_json::Value>) {
    SCENE_METADATA.with(|store| {
        let mut store = store.borrow_mut();
        if record.is_empty() {
            store.remove(name);
        } else {
            store.insert(name.to_string(), record);
        }
    });
}

/// Take the WHOLE store out, leaving it empty — the isolated sub-part run
/// bracket (`parts_library::rebuild_snapshot`): the sub-part's features stamp
/// UN-namespaced records that must never merge into (or read) the parent
/// document's records. Pair with [`restore_store`] on every exit path.
pub(crate) fn take_store() -> HashMap<String, Record_> {
    SCENE_METADATA.with(|store| std::mem::take(&mut *store.borrow_mut()))
}

/// Put a taken store back, discarding whatever the bracketed run stamped.
pub(crate) fn restore_store(saved: HashMap<String, Record_>) {
    SCENE_METADATA.with(|store| {
        *store.borrow_mut() = saved;
    });
}

/// The [`take_store`]/[`restore_store`] bracket as an RAII guard, for
/// OUT-OF-CRATE producers that build a payload against a LIVE document.
///
/// The hazard it exists for: [`crate::snapshot_solids`] captures whatever record
/// the thread's store holds for each name it is sealing. That is exactly right
/// for a snapshot of the CURRENT scene (a face's material rides along), and
/// exactly WRONG for a payload built out of freshly-named solids that were never
/// in this scene — the STEP-assembly import lane's per-part
/// [`crate::native_import_payload`] call being the first such producer. There,
/// any collision between the names being stamped and names already in the
/// document's store would silently seal the CURRENT document's metadata into the
/// new part. Holding this guard across the encode makes the payload a pure
/// function of its solids, which is what its determinism claim assumes.
///
/// Scope it to the ENCODE only. Anything that runs a history — the display
/// rebuild, a `pump` — stamps records the document must keep, and dropping the
/// guard afterwards would throw exactly those away.
///
/// Restores on drop, panics included; nesting is harmless (the inner guard takes
/// an already-empty store and restores it).
#[must_use = "the store is only isolated while the guard is alive"]
pub struct IsolatedSceneMetadata {
    saved: HashMap<String, Record_>,
}

impl IsolatedSceneMetadata {
    /// Empty the thread's scene-metadata store until the guard drops.
    pub fn begin() -> Self {
        Self {
            saved: take_store(),
        }
    }
}

impl Drop for IsolatedSceneMetadata {
    fn drop(&mut self) {
        restore_store(std::mem::take(&mut self.saved));
    }
}

/// Merge keys into a record WITHOUT replacing it (existing keys win when
/// `overwrite` is false) — the pipeline-side stamping entry.
pub fn merge_record(name: &str, patch: &serde_json::Map<String, serde_json::Value>, overwrite: bool) {
    if patch.is_empty() {
        return;
    }
    SCENE_METADATA.with(|store| {
        let mut store = store.borrow_mut();
        let record = store.entry(name.to_string()).or_default();
        for (key, value) in patch {
            if overwrite || !record.contains_key(key) {
                record.insert(key.clone(), value.clone());
            }
        }
    });
}

/// Every record carrying an imported COLOUR, as `{name: "#RRGGBB"}`.
///
/// The narrow read the display layer needs. It exists because this store is
/// THREAD-LOCAL to whichever thread ran the history: `BREP_render` executes a
/// run on a background thread / worker, so anything the main thread wants must
/// be read runner-side and shipped in the `RunOutput` (the same reason the
/// assembly pose write-backs are read there). See `io/appearance.rs` for the
/// record convention and `BREP_render/src/pipeline.rs` for the consumer.
///
/// Deliberately colour-only rather than a whole-store dump: every face carries
/// `sourceFeatureId` and friends, and pushing all of that across the seam into
/// the caller's own metadata store is a different decision from "an imported
/// colour should be visible".
#[wasm_bindgen]
pub fn scene_metadata_colors_json() -> String {
    SCENE_METADATA.with(|store| {
        let mut out = serde_json::Map::new();
        for (name, record) in store.borrow().iter() {
            if let Some(color) = record
                .get(crate::COLOR_METADATA_KEY)
                .and_then(|value| value.as_str())
            {
                out.insert(name.clone(), serde_json::Value::String(color.to_string()));
            }
        }
        serde_json::Value::Object(out).to_string()
    })
}

/// The OWN metadata record for a name (no inheritance) — the snapshot capture
/// read (`io/snapshot.rs` embeds the record verbatim so a restored part carries
/// its stamped topology metadata). `None` when the name has no record.
pub(crate) fn own_record(name: &str) -> Option<serde_json::Map<String, serde_json::Value>> {
    SCENE_METADATA.with(|store| store.borrow().get(name).cloned())
}

/// Batch-read RESOLVED records for a list of names (the per-solid display
/// read: one call per materialization, not one per face).
#[wasm_bindgen]
pub fn topo_metadata_for_names_json(names_json: &str) -> Result<String, JsValue> {
    let names: Vec<String> = serde_json::from_str(names_json)
        .map_err(|error| JsValue::from_str(&format!("topo_metadata_for_names: {error}")))?;
    SCENE_METADATA.with(|store| {
        let store = store.borrow();
        let mut out = serde_json::Map::new();
        for name in names {
            let resolved = resolve(&store, &name);
            if !resolved.is_empty() {
                out.insert(name, serde_json::Value::Object(resolved));
            }
        }
        Ok(serde_json::Value::Object(out).to_string())
    })
}


//! The Properties-panel DATA layer (UI comes later): a name-keyed metadata
//! store, per-entity measurements, and object provenance — all engine-native,
//! all crossing the R3 boundary as plain JSON/scalars.
//!
//! # Metadata store ([`MetadataStore`])
//!
//! `object name → { attribute → value }`, keyed by the KERNEL OBJECT NAME (a
//! solid / face / edge name), **not** a feature id. This loose coupling is the
//! whole point: a record survives feature edits, rollback and re-tessellation as
//! long as the object's name persists (edge/face names ARE propagated through
//! booleans, splits and welds by the kernel). Values are strings; the well-known
//! `density` attribute (mass units per mm³) drives a solid's weight. The store is
//! persisted WITH the model — [`crate::engine_state::EngineState::history_request_json`]
//! folds it in as a top-level `metadata` field and
//! [`crate::engine_state::EngineState::set_history_json`] lifts it back out, so it
//! round-trips through save/open.
//!
//! # Measurements + provenance
//!
//! The [`EngineState`] methods below resolve an object NAME to its kind (solid /
//! face / edge, via the scene) and return the right measurements from the
//! kernel's exact integrators (volume, surface area, arc length), plus the
//! feature that produced the object (provenance, via the history's per-feature
//! output solids). Units are millimetres (the kernel length convention).

use crate::engine_state::EngineState;
use brep_kernel::COLOR_METADATA_KEY;
use crate::runner::{MeasureKind, MeasureQuery};
use serde_json::Value;
use std::collections::BTreeMap;

/// The default density (mass per unit volume) when an object carries no
/// `density` metadata: unit density, so `weight == volume`.
pub const DEFAULT_DENSITY: f64 = 1.0;

/// The name-keyed metadata store: `object name → { attribute → value }`, string
/// values, deterministic iteration (a `BTreeMap` so the persisted JSON is
/// stable). Empty records are never retained (removing the last attribute drops
/// the whole record).
#[derive(Debug, Clone, Default)]
pub struct MetadataStore {
    entries: BTreeMap<String, BTreeMap<String, String>>,
}

impl MetadataStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the store holds no records at all.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Set (or overwrite) one attribute of `name`'s record. An empty object name
    /// or key is ignored (no phantom records).
    pub fn set_attribute(&mut self, name: &str, key: &str, value: &str) {
        if name.is_empty() || key.is_empty() {
            return;
        }
        self.entries
            .entry(name.to_string())
            .or_default()
            .insert(key.to_string(), value.to_string());
    }

    /// Remove one attribute of `name`'s record, dropping the record if it becomes
    /// empty. Returns whether the attribute existed.
    pub fn remove_attribute(&mut self, name: &str, key: &str) -> bool {
        let Some(record) = self.entries.get_mut(name) else {
            return false;
        };
        let existed = record.remove(key).is_some();
        if record.is_empty() {
            self.entries.remove(name);
        }
        existed
    }

    /// One attribute's value (`None` if the object or key is unknown).
    pub fn attribute(&self, name: &str, key: &str) -> Option<&str> {
        self.entries.get(name)?.get(key).map(String::as_str)
    }

    /// One object's whole record (empty map if the object has no metadata).
    pub fn record(&self, name: &str) -> BTreeMap<String, String> {
        self.entries.get(name).cloned().unwrap_or_default()
    }

    /// The whole store (all records), read-only.
    pub fn all(&self) -> &BTreeMap<String, BTreeMap<String, String>> {
        &self.entries
    }

    /// The resolved density (mass per mm³) for `name`: its `density` attribute
    /// parsed as a positive finite number, else [`DEFAULT_DENSITY`].
    pub fn density(&self, name: &str) -> f64 {
        self.attribute(name, "density")
            .and_then(|value| value.trim().parse::<f64>().ok())
            .filter(|density| density.is_finite() && *density > 0.0)
            .unwrap_or(DEFAULT_DENSITY)
    }

    /// Drop every record (a part load replaces the store wholesale).
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// One object's record as a JSON object `{ key: value, ... }` (`{}` when the
    /// object has no metadata).
    pub fn record_json(&self, name: &str) -> String {
        Value::Object(
            self.record(name)
                .into_iter()
                .map(|(key, value)| (key, Value::String(value)))
                .collect(),
        )
        .to_string()
    }

    /// The whole store as a JSON value `{ name: { key: value, ... }, ... }` — the
    /// persisted shape and the whole-store getter.
    pub fn to_json(&self) -> Value {
        Value::Object(
            self.entries
                .iter()
                .map(|(name, record)| {
                    let object = record
                        .iter()
                        .map(|(key, value)| (key.clone(), Value::String(value.clone())))
                        .collect();
                    (name.clone(), Value::Object(object))
                })
                .collect(),
        )
    }

    /// The whole store as a JSON string (the whole-store getter's string form).
    pub fn to_json_string(&self) -> String {
        self.to_json().to_string()
    }

    /// Replace the whole store from a persisted `metadata` value (or `None`, which
    /// clears it — loading a part with no metadata). Non-string leaf values are
    /// coerced to their JSON text so a legacy document never fails the load.
    pub fn load_json(&mut self, value: Option<&Value>) {
        self.entries.clear();
        let Some(Value::Object(objects)) = value else {
            return;
        };
        for (name, record) in objects {
            let Value::Object(attributes) = record else {
                continue;
            };
            let map: BTreeMap<String, String> = attributes
                .iter()
                .map(|(key, value)| (key.clone(), value_to_string(value)))
                .collect();
            if !map.is_empty() {
                self.entries.insert(name.clone(), map);
            }
        }
    }
}

/// Coerce a persisted metadata leaf to a string value (strings verbatim, `null`
/// to empty, everything else to its JSON text).
fn value_to_string(value: &Value) -> String {
    match value {
        Value::String(text) => text.clone(),
        Value::Null => String::new(),
        other => other.to_string(),
    }
}

/// The kind an object NAME resolves to in the current scene.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ObjectKind {
    Solid,
    Face,
    Edge,
}

// --- EngineState: metadata store API (a SEPARATE impl block, appended, so
//     concurrent edits to the primary block don't conflict) -------------------
impl EngineState {
    /// One object's metadata record as JSON `{ key: value, ... }` (`{}` if none).
    pub fn object_metadata_json(&self, name: &str) -> String {
        self.metadata.record_json(name)
    }

    /// Set (or overwrite) one metadata attribute of an object by NAME. String
    /// value; the well-known `density` key drives the object's weight and the
    /// well-known `color` key drives its shaded colour.
    pub fn set_metadata_attribute(&mut self, name: &str, key: &str, value: &str) {
        self.metadata.set_attribute(name, key, value);
        // A metadata edit (notably `density`) feeds the object-info output but does
        // NOT trigger a history rerun, so drop this object's cached info so a
        // re-selection re-measures with the new attribute.
        self.info_cache.remove(name);
        // `color` is the one attribute the VIEWPORT reads, and there is no rerun
        // to carry it through — push it to the display right here so recolouring
        // a body is immediate.
        if key == COLOR_METADATA_KEY {
            self.sync_colors_from_metadata();
        }
    }

    /// Remove one metadata attribute of an object. Returns whether it existed.
    pub fn remove_metadata_attribute(&mut self, name: &str, key: &str) -> bool {
        let existed = self.metadata.remove_attribute(name, key);
        // Same rerun-less invalidation as `set_metadata_attribute`.
        self.info_cache.remove(name);
        if existed && key == COLOR_METADATA_KEY {
            self.sync_colors_from_metadata();
        }
        existed
    }

    /// Re-derive the display scene's colours from the metadata store — the ONE
    /// call that connects the durable `color` attribute to the renderer.
    ///
    /// Every colour the viewport shows comes from here: a STEP import's stamped
    /// body/face colours, a colour typed or picked in the Info window, a colour
    /// restored from a saved document. The store is the authority; the display's
    /// `color_override` fields are a derived cache of it, rebuilt after every
    /// history apply ([`EngineState::finish_apply`]), on every document load, on
    /// a metadata edit, and whenever the display setting flips.
    ///
    /// [`crate::style::RenderSettings::override_model_colors`] is honoured HERE
    /// rather than in the renderer, and it is deliberately a READ of the store,
    /// never a write: ticking the box resolves every colour to `None` so the
    /// viewport falls back to `faceColorMode`, while the stored attributes stay
    /// exactly as they were. Unticking restores them from the same records.
    ///
    /// Returns whether anything changed, and marks the engine dirty only then —
    /// the no-op case must stay free, since this runs after every single run.
    pub fn sync_colors_from_metadata(&mut self) -> bool {
        // Disjoint field borrows: the closure reads `self.metadata` while
        // `self.scene` is borrowed mutably.
        let ignore = self.settings.override_model_colors;
        let metadata = &self.metadata;
        let changed = self.scene.apply_metadata_colors(|name| {
            if ignore {
                return None;
            }
            metadata
                .attribute(name, COLOR_METADATA_KEY)
                .and_then(crate::style::parse_css_hex)
                // A derived BOARD body has a colour of its own and no producing
                // feature to have coloured it, so without this fallback the
                // pass would resolve it to `None` and the board would draw in
                // the name-hashed default — green substrate and copper traces
                // are the whole point of drawing it. A stored attribute still
                // wins, so a user may recolour the board like anything else.
                .or_else(|| crate::engine_state::board_default_color(name))
        });
        if changed {
            self.dirty = true;
        }
        changed
    }

    /// The whole metadata store as JSON `{ name: { key: value } }`.
    pub fn metadata_json(&self) -> String {
        self.metadata.to_json_string()
    }
}

// --- EngineState: measurements + provenance (SEPARATE impl block) ------------
impl EngineState {
    /// Resolve an object NAME to `(kind, owning solid name)` via the display
    /// scene: a solid resolves to itself; a face/edge resolves to its owning
    /// solid. `None` for an empty or unknown name (vertices carry no kernel name).
    fn resolve_object(&self, name: &str) -> Option<(ObjectKind, String)> {
        if name.is_empty() {
            return None;
        }
        if self.scene.solid(name).is_some() {
            return Some((ObjectKind::Solid, name.to_string()));
        }
        for solid in self.scene.solids() {
            if solid.faces.iter().any(|face| face.name == name) {
                return Some((ObjectKind::Face, solid.name.clone()));
            }
        }
        for solid in self.scene.solids() {
            if solid.edges.iter().any(|edge| edge.name == name) {
                return Some((ObjectKind::Edge, solid.name.clone()));
            }
        }
        None
    }

    /// The feature that an object ORIGINATES from, as `(feature id, feature type)`.
    /// For a FACE/EDGE this is the feature that first gave it its name (its true
    /// origin, from the eager `entity_origin` first-writer map) — NOT the owning
    /// solid's last producer — so "Edit owning feature" rolls back to where the
    /// entity was born. For a SOLID it's the solid's producer (last writer, the
    /// eager `provenance` map). `None` if the name is unknown or has no known
    /// producer. Reads only the eager maps the last run shipped, so it never re-runs
    /// the history — the freeze side-door `context_bar` hit every selected frame is
    /// now O(1).
    pub fn creating_feature(&self, name: &str) -> Option<(String, String)> {
        let Some((kind, owner)) = self.resolve_object(name) else {
            // A datum / construction PLANE (or one plane of a datum) is not a
            // resident solid/face/edge — it's a named frame. Its producer is the
            // D/P feature that emitted the frame (same fallback object_info_json
            // uses), so "Edit owning feature" resolves for a lone plane pick too.
            return self.datum_feature_for_name(name);
        };
        let id = match kind {
            // A face/edge resolves to its ORIGIN. Fall back to the owning solid's
            // producer when the name is somehow absent from `entity_origin` (keeps
            // the "Edit owning feature" button from vanishing).
            ObjectKind::Face | ObjectKind::Edge => self
                .entity_origin
                .get(name)
                .cloned()
                .or_else(|| self.provenance.get(&owner).cloned())?,
            // A solid keeps its existing last-writer producer semantics.
            ObjectKind::Solid => self.provenance.get(&owner)?.clone(),
        };
        Some((id.clone(), self.feature_type_of(&id)))
    }

    /// Whether the object `name` (a solid, or a face/edge owned by a solid) sits
    /// on a SHEET-METAL body — its owning display solid carries the sheet-metal
    /// marker the pipeline stamped from the resident handle's `SheetTree`. Reads
    /// only the display scene, so it is O(1) and thread-safe (no `SheetTree`
    /// thread-local touched on the UI thread — [`crate::scene::SolidDisplay::
    /// is_sheet_metal`]). The gate for the sheet-metal edit features (SM Flange /
    /// Fillet / Chamfer). `false` for an unknown name or a synthesized sketch
    /// sheet (no resident handle, no tree).
    pub fn is_sheet_metal_object(&self, name: &str) -> bool {
        self.resolve_object(name)
            .and_then(|(_, owner)| self.scene.solid(&owner))
            .is_some_and(|solid| solid.is_sheet_metal)
    }

    /// A feature's type token by id (empty string if the id is not in the history).
    fn feature_type_of(&self, id: &str) -> String {
        self.history
            .index_of(id)
            .and_then(|index| self.history.feature_type(index))
            .unwrap_or_default()
    }

    /// The `{ id, type }` provenance JSON for an object by NAME (or `null` if it has
    /// no known producer). Delegates to [`creating_feature`](Self::creating_feature)
    /// so a face/edge reports its ORIGIN (first-writer) and a solid its producer
    /// (last-writer) — one resolver, so the Info tab and the context bar agree.
    fn creating_feature_value(&self, name: &str) -> Value {
        match self.creating_feature(name) {
            Some((id, ty)) => serde_json::json!({ "id": id, "type": ty }),
            None => Value::Null,
        }
    }

    /// The full Properties-panel info for an object by NAME: its resolved kind,
    /// the right measurements, and provenance. Units are millimetres.
    ///
    /// - **Solid** — `{ ok, name, kind:"solid", volume, surfaceArea,
    ///   edgeLengthTotal, density, weight, creatingFeature }`, where
    ///   `weight = density · volume` and `density` comes from the object's
    ///   metadata (default [`DEFAULT_DENSITY`]).
    /// - **Face** — `{ ok, name, kind:"face", solid, surfaceType, area,
    ///   edgeLengthTotal, creatingFeature }` (`edgeLengthTotal` = its boundary
    ///   edges; `surfaceType` = the carrier-surface classification, e.g.
    ///   `"Plane"`/`"Cylinder"`/`"Cone"`/`"Sphere"`/`"Torus"`/`"NURBS"`).
    /// - **Edge** — `{ ok, name, kind:"edge", solid, length, creatingFeature }`.
    ///
    /// `{ ok:false, name, message }` for an empty/unknown name, a non-resident
    /// solid, or a kernel measurement failure.
    ///
    /// The real solid/face/edge MEASUREMENT is routed to the [`HistoryRunner`] (so
    /// the warm-registry runner answers it, never the potentially-cold main side)
    /// and CACHED here keyed by name — fired once per selection, served from the
    /// cache every subsequent frame. For the synchronous
    /// [`InlineRunner`](crate::runner::InlineRunner) the submit → `pump_queries`
    /// resolves same-call, so this returns the merged JSON immediately and stays
    /// byte-identical to the pre-seam in-process result; a background
    /// [`ThreadRunner`](crate::runner::ThreadRunner) returns a `pending` placeholder
    /// for the frame(s) until its reply lands (drained by `pump_queries`). The cache
    /// is invalidated on any geometry change (`apply_run_output`) or metadata edit
    /// (`set_metadata_attribute`).
    ///
    /// [`HistoryRunner`]: crate::runner::HistoryRunner
    pub fn object_info_json(&mut self, name: &str) -> String {
        let Some((kind, owner)) = self.resolve_object(name) else {
            // A construction datum/plane carries no resident geometry (no volume /
            // area / length), so it never resolves as a solid/face/edge. Return a
            // graceful minimal record — name + kind + creating feature — rather than
            // erroring, so the Properties Info tab renders for a selected datum and
            // its name flows into the (name-keyed) Metadata tab.
            if let Some((feature_id, feature_type)) = self.datum_feature_for_name(name) {
                let kind_label = if feature_type == "P" { "plane" } else { "datum" };
                return serde_json::json!({
                    "ok": true,
                    "name": name,
                    "kind": kind_label,
                    "creatingFeature": { "id": feature_id, "type": feature_type },
                })
                .to_string();
            }
            return serde_json::json!({
                "ok": false, "name": name, "message": "unknown object",
            })
            .to_string();
        };
        // A committed-sketch SHEET is a scene solid with NO kernel handle, so the
        // handle-based measurement path can't serve it. Measure straight off its
        // synthesized display (planar-mesh triangle areas / edge polylines) and
        // report `kind:"sketch"` — no volume. Handle-less ⇒ synchronous, no query.
        if let Some(solid) = self.scene.solid(&owner) {
            if solid.is_sketch {
                return self.sketch_info_json(name, kind, solid);
            }
        }

        // Real geometry: serve from the info cache, else fire a measurement query at
        // the runner (deduped: never submit a second query for a name already in
        // flight), pump, and return the resolved JSON — or a pending placeholder.
        if let Some(cached) = self.info_cache.get(name) {
            return cached.clone();
        }
        if !self.pending_query.values().any(|pending| pending == name) {
            self.next_query_id += 1;
            let id = self.next_query_id;
            let measure_kind = match kind {
                ObjectKind::Solid => MeasureKind::Solid,
                ObjectKind::Face => MeasureKind::Face,
                ObjectKind::Edge => MeasureKind::Edge,
            };
            let density = self.metadata.density(name);
            self.pending_query.insert(id, name.to_string());
            self.runner.submit_query(MeasureQuery {
                id,
                kind: measure_kind,
                owner,
                entity: name.to_string(),
                density,
            });
        }
        self.pump_queries();
        match self.info_cache.get(name) {
            Some(resolved) => resolved.clone(),
            None => serde_json::json!({ "ok": false, "name": name, "pending": true }).to_string(),
        }
    }

    /// Drain every completed measurement reply from the runner and fold it into the
    /// name-keyed info cache — the query counterpart of [`EngineState::pump`]. Called
    /// once per frame from `pump` AND synchronously from
    /// [`Self::object_info_json`] (so the Inline runner resolves same-call). Each
    /// reply is MERGED with the main-injected `name` + `creatingFeature` (from the
    /// eager provenance) into the final object-info JSON.
    pub fn pump_queries(&mut self) {
        while let Some(reply) = self.runner.poll_query() {
            let Some(name) = self.pending_query.remove(&reply.id) else {
                // A reply whose request was invalidated (a rerun cleared the pending
                // set) — drop it; the re-selection re-queries against fresh geometry.
                continue;
            };
            let fragment: Value = serde_json::from_str(&reply.result).unwrap_or(Value::Null);
            let merged = self.merge_info(&name, &fragment);
            self.info_cache.insert(name, merged);
        }
    }

    /// Merge a runner measurement FRAGMENT with the main-side `name` +
    /// `creatingFeature` into the final object-info JSON, preserving the exact field
    /// ORDER the pre-seam `object_info_json` emitted (so the output is
    /// byte-identical): `ok`, then `name`, then the fragment's remaining fields in
    /// order (`kind`, the measurements — or `message` on error), then
    /// `creatingFeature` (only when `ok`, since an error record carries none).
    fn merge_info(&self, name: &str, fragment: &Value) -> String {
        let object = fragment.as_object();
        let ok = object
            .and_then(|map| map.get("ok"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        let mut merged = serde_json::Map::new();
        merged.insert("ok".to_string(), Value::Bool(ok));
        merged.insert("name".to_string(), Value::String(name.to_string()));
        if let Some(map) = object {
            for (key, value) in map {
                if key == "ok" {
                    continue;
                }
                merged.insert(key.clone(), value.clone());
            }
        }
        if ok {
            merged.insert(
                "creatingFeature".to_string(),
                // Resolve by ENTITY NAME (not `owner`): a face/edge reports its
                // ORIGIN, a solid its producer — the same correction the context bar
                // gets, so the Info tab's `creatingFeature` agrees.
                self.creating_feature_value(name),
            );
        }
        Value::Object(merged).to_string()
    }

    /// Properties info for a committed-sketch SHEET object (the sheet solid, its
    /// planar face, or a boundary edge), measured off the synthesized display — a
    /// sketch carries no resident kernel geometry. `kind:"sketch"` for the whole
    /// sheet (area + total edge length, NO volume); `"face"` for its planar face;
    /// `"edge"` for a boundary edge. Provenance is the sketch feature itself.
    fn sketch_info_json(
        &self,
        name: &str,
        kind: ObjectKind,
        solid: &crate::scene::SolidDisplay,
    ) -> String {
        let creating = serde_json::json!({
            "id": solid.name,
            "type": self.feature_type_of(&solid.name),
        });
        let edge_total: f64 = solid.edges.iter().map(|e| polyline_length(&e.polyline)).sum();
        match kind {
            ObjectKind::Solid => serde_json::json!({
                "ok": true,
                "name": name,
                "kind": "sketch",
                "area": sheet_mesh_area(solid),
                "edgeLengthTotal": edge_total,
                "creatingFeature": creating,
            })
            .to_string(),
            ObjectKind::Face => serde_json::json!({
                "ok": true,
                "name": name,
                "kind": "face",
                "solid": solid.name,
                // A committed sketch's sheet face is planar by construction.
                "surfaceType": "Plane",
                "area": sheet_mesh_area(solid),
                "edgeLengthTotal": edge_total,
                "creatingFeature": creating,
            })
            .to_string(),
            ObjectKind::Edge => {
                let length = solid
                    .edges
                    .iter()
                    .find(|e| e.name == name)
                    .map(|e| polyline_length(&e.polyline))
                    .unwrap_or(0.0);
                serde_json::json!({
                    "ok": true,
                    "name": name,
                    "kind": "edge",
                    "solid": solid.name,
                    "length": length,
                    "creatingFeature": creating,
                })
                .to_string()
            }
        }
    }
}

/// Total surface area (mm²) of a synthesized sheet's planar display mesh — the
/// sum of its triangle areas.
fn sheet_mesh_area(solid: &crate::scene::SolidDisplay) -> f64 {
    let p = &solid.mesh.positions;
    solid
        .mesh
        .indices
        .chunks_exact(3)
        .map(|t| {
            let a = p[t[0] as usize];
            let b = p[t[1] as usize];
            let c = p[t[2] as usize];
            let ab = [
                (b[0] - a[0]) as f64,
                (b[1] - a[1]) as f64,
                (b[2] - a[2]) as f64,
            ];
            let ac = [
                (c[0] - a[0]) as f64,
                (c[1] - a[1]) as f64,
                (c[2] - a[2]) as f64,
            ];
            let cross = [
                ab[1] * ac[2] - ab[2] * ac[1],
                ab[2] * ac[0] - ab[0] * ac[2],
                ab[0] * ac[1] - ab[1] * ac[0],
            ];
            (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt() * 0.5
        })
        .sum()
}

/// Arc length (mm) of a sampled edge polyline — the sum of its segment lengths.
fn polyline_length(polyline: &[[f32; 3]]) -> f64 {
    polyline
        .windows(2)
        .map(|w| {
            let d = [
                (w[1][0] - w[0][0]) as f64,
                (w[1][1] - w[0][1]) as f64,
                (w[1][2] - w[0][2]) as f64,
            ];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        })
        .sum()
}


//! Feature-history execution with exact name resolution and stable output names.
//!
//! [`execute_history`] updates its live [`SceneMap`] after every feature. Reference
//! parameters resolve by exact name; misses produce structured `unresolved`
//! entries in [`FeatureResult`]. Output names must preserve saved-document naming
//! conventions because later features store those names as references.
//!
//! Feature descriptors retain `{type, inputParams, persistentData, timestamp}`.
//! Numeric parameters accept literals or expressions evaluated against [`Env`].
//!
//! Registry borrows stay local to each operation and never span feature execution.
//! Features call Rust operations directly, preserving names on resident solids.
//! Removed scene solids and consumed intermediates must release their handles.

pub mod extension;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use wasm_bindgen::prelude::*;

use crate::{KernelRefusal, NurbsCurve, Vec3};

// The scene component concept (assemblies): component records + per-instance
// entity-name namespacing at the component boundary. See component.rs.
pub(crate) mod component;
pub use component::ComponentRecord;
// The per-document parts library (assemblies): unique part payloads the ACOMP
// instance feature references by name; kernel-resident, request-seeded,
// GC'd at the end of every history run. See parts_library.rs.
pub(crate) mod parts_library;
pub use parts_library::{
    add_part_to_library, add_part_to_library_impl, install_parts_library, missing_library_parts,
    parts_library_json, parts_library_map, parts_library_revision, refresh_library_entry,
    refresh_library_entry_impl, same_build,
    PartsLibraryEntry, PartsLibraryMap, stable_json_hash,
};
// Assembly constraint state + lifecycle: the `assembly` history block, the
// ten constraint schemas, the mate-mapping + solve tail, and the exported
// constraint-mutation ABI. See assembly.rs.
pub mod assembly;
/// Wire-harness routing: the document's `wireHarness` block, the sided port
/// graph, per-connection routes and the bundle solids — solved at the tail of
/// every run like `assembly` (see `wire_harness.rs`).
pub mod wire_harness;
/// A part's symbol pins ARE its harness ports, bound by label (eCAD plan
/// decision 8): the pin/port report, following an edit from one side to the
/// other, and resolving a placed part's pin to its port id (see `part_pins.rs`).
pub mod part_pins;
pub mod pmi;
pub mod ports;
pub(crate) use features::import3d::imported_solid_names;
pub use assembly::{AssemblyState, ConstraintEntry};
mod expression;
#[path = "features/mod.rs"]
pub(crate) mod features;
pub use features::common::{first_reference_name, reference_names};
pub(crate) use features::transform::bbox_center as transform_bbox_center;
// Transform Face's default pivot over a replayed prefix: the value the app
// stores into `pivot` when a face selection is committed, computed by the same
// resolution and centre the feature falls back to for a null pivot.
pub use features::transform_face::face_transform_pivot;
// The native IMPORT3D payload encoder: finished solids → an `io/snapshot`
// payload carrying IMPORT3D's own names, ready to become an
// `inputParams.nativeBrep` part document. Lives WITH the feature that reads it
// back (one naming convention, one implementation); exported for the import
// lanes that build part documents.
pub use features::import3d::{native_import_payload, native_import_payload_with_appearance};
pub(crate) mod scene_metadata;
// The scene-metadata isolation bracket, for out-of-crate producers that encode a
// payload against a LIVE document (the STEP-assembly import lane).
pub use scene_metadata::IsolatedSceneMetadata;
// The colour-only read of the name-keyed store: THREAD-LOCAL, so a caller whose
// history runs off-thread must read it runner-side (`io/appearance.rs`).
pub use scene_metadata::scene_metadata_colors_json;
mod schema;
// The kernel-owned feature-schema catalogue, reachable by NATIVE Rust consumers
// (the engine-native egui UI in brep-app, via a brep-render re-export) — the same
// definitions the wasm `feature_schemas_json` export serves the caller's feature registry.
pub use schema::feature_schema_catalogue;
pub use schema::feature_hidden_params;
pub mod context_offer;
// Selection-context applicability: the per-feature show/no-show predicates the
// app's context bar runs against the current selection (schema.rs's sibling —
// definitions live with each feature, this only aggregates).
pub use context_offer::{feature_context_applicable, SelectionProbe};
#[path = "sheet_metal/mod.rs"]
mod sheet_metal;
// Flat-pattern (unfold) → 2D vector export, for the native engine's export lane.
// Keyed by resident handle so `SheetTree` stays private to the pipeline.
pub use sheet_metal::{flat_pattern_dxf, flat_pattern_svg, is_sheet_metal_handle};

pub use expression::Env;
// The sketch loop-id write-back (the editor calls it on commit) — the
// persistence half of the per-loop face-naming scheme. See
// `features/sketch/loop_ids.rs`.
pub use features::sketch::assign_sketch_loop_ids;

// ===========================================================================
// Descriptor — the serialized `{ type, inputParams, persistentData, timestamp }`
// ===========================================================================

/// One feature as serialized by `PartHistory.toSerializable`. Permissive: unknown
/// fields are ignored, so an entire saved part file's `features[]` deserializes
/// as-is. `input_params` stays a `serde_json::Value` (an object) so features read
/// their own params with their own type knowledge (which params are numeric).
// `persistent_data` / `timestamp` are contract pass-throughs (round-tripped for
// features that need them, e.g. hole/pattern persistent state); not read yet.
#[allow(dead_code)]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureDescriptor {
    #[serde(rename = "type", default)]
    pub feature_type: String,
    #[serde(rename = "inputParams", default)]
    pub input_params: serde_json::Value,
    #[serde(rename = "persistentData", default)]
    pub persistent_data: serde_json::Value,
    /// Opaque pass-through (any shape) so one odd saved file cannot fail the parse.
    #[serde(default)]
    pub timestamp: Option<serde_json::Value>,
}

/// The `execute_history_json` request: the expression source, the configurator
/// state, and the ordered feature list. Permissive — a whole saved part file
/// parses as a request (extra top-level fields ignored).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HistoryRequest {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub plugins: Vec<extension::PluginPin>,
    #[serde(default, deserialize_with = "de_string_lenient")]
    pub expressions: String,
    #[serde(default)]
    pub configurator: serde_json::Value,
    #[serde(default)]
    pub features: Vec<FeatureDescriptor>,
    /// One-shot execution directive: rebuild these features even if their inputs
    /// match the cache (for example, when a dimension drag is finalized). Their
    /// consumers follow the usual dirty wavefront. Not part of the saved model.
    #[serde(default, rename = "forceRebuild", skip_serializing_if = "Vec::is_empty")]
    pub force_rebuild: Vec<String>,
    /// Stop AFTER executing the feature with this id (the editor's "stop at the
    /// expanded feature"). The features past the stop stay in the request so the
    /// incremental cache RETAINS their entries — expanding/collapsing a panel
    /// must not thrash the cache (marshal-side truncation would free them).
    #[serde(default, rename = "stopAtId")]
    pub stop_at_id: Option<String>,
    /// Stop BEFORE executing the feature with this id.
    #[serde(default, rename = "stopBeforeId")]
    pub stop_before_id: Option<String>,
    /// The assembly constraint block: `{ constraints, idCounter }`. Absent on
    /// every non-assembly document (and omitted on re-serialize, so
    /// componentless files persist byte-identically). Solved at the tail of
    /// every run; see `assembly.rs`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub assembly: Option<assembly::AssemblyState>,
    /// The wire-harness block: `{ connections, idCounter, buildBundles }`.
    /// Absent on every document without harness connections (and omitted on
    /// re-serialize). Routed at the tail of every run; see `wire_harness.rs`.
    #[serde(default, rename = "wireHarness", skip_serializing_if = "Option::is_none")]
    pub wire_harness: Option<wire_harness::WireHarnessState>,
    /// The PMI block: `{ views: [{ id, name, camera, display, annotations }],
    /// idCounter }`. Absent on every document without PMI (and omitted on
    /// re-serialize). Resolved at the tail of every run; see `pmi.rs`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pmi: Option<pmi::PmiState>,
    /// Render level-of-detail factor for DISPLAY tessellation (the app's "LOD
    /// factor" setting). `execute_history` ignores it — it rides on the request
    /// only because the request is the serialized run boundary the display runner
    /// receives, and the runner scales its per-solid chord tolerance by it
    /// (higher = coarser mesh). Defaults to `1.0` (the "Normal" preset) so a saved
    /// file / seed request with no `displayLod` tessellates exactly as before.
    #[serde(default = "default_display_lod", rename = "displayLod")]
    pub display_lod: f64,
    /// The parts library block: unique part payloads the ACOMP instance
    /// features reference by name. Ingested into the kernel-resident library
    /// at the start of every run (this is how a LOADED document seeds it);
    /// SAVE serializes `parts_library_json()` back out — never this loaded
    /// block (see parts_library.rs).
    #[serde(default, rename = "partsLibrary")]
    pub parts_library: parts_library::PartsLibraryMap,
    /// The declared-ports block: the part's connection points as DATA, a
    /// sibling of `features` (see `ports.rs`). Absent on every document that
    /// declares none (and omitted on re-serialize). Resolved at the tail of
    /// every run, against the finished scene.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ports: Vec<ports::PortDeclaration>,
    /// The EXPLICIT encapsulation mark: `true` hides this document's children
    /// behind its declared ports, `false` keeps them reachable. Absent means
    /// derive it — see [`ports::document_is_boundary`].
    #[serde(default, rename = "portBoundary", skip_serializing_if = "Option::is_none")]
    pub port_boundary: Option<bool>,
    /// Whether the document carries a `pcb` block, which makes it a board and
    /// therefore a port boundary by construction. Presence ONLY: the block
    /// itself is eCAD's and the kernel neither reads nor rewrites it, so this
    /// never serializes back out.
    #[serde(
        default,
        rename = "pcb",
        deserialize_with = "de_present",
        skip_serializing
    )]
    pub pcb_present: bool,
}

/// Deserialize any value as "it was there" - see [`HistoryRequest::pcb_present`].
fn de_present<'de, D>(deserializer: D) -> Result<bool, D::Error>
where
    D: serde::Deserializer<'de>,
{
    serde::Deserialize::deserialize(deserializer).map(|_: serde::de::IgnoredAny| true)
}

/// The "Normal" render LOD — the [`HistoryRequest::display_lod`] serde default.
fn default_display_lod() -> f64 {
    1.0
}

/// Accept `null`/missing as `""` for a string field (saved files sometimes store
/// `expressions: null`).
fn de_string_lenient<'de, D>(deserializer: D) -> Result<String, D::Error>
where
    D: serde::Deserializer<'de>,
{
    let value = Option::<String>::deserialize(deserializer)?;
    Ok(value.unwrap_or_default())
}

// ===========================================================================
// Result — the per-feature output contract the caller consumes
// ===========================================================================

/// A solid produced by a feature: its resident handle, its solid name, and its
/// named faces/edges (`(topology_id, name)`). the caller tessellates/pulls names via the
/// handle; the scene-map registers the names for downstream reference resolution.
#[derive(Debug, Clone, Default, Serialize)]
pub struct AddedSolid {
    pub handle: u32,
    pub name: String,
    pub face_names: Vec<(u64, String)>,
    /// Edge names propagated/authored on the solid. Present so edge-selecting
    /// features (fillet/chamfer) can resolve "the edge of the extrude above"
    /// against the scene-map — the pinned example in contract rule 1.
    pub edge_names: Vec<(u64, String)>,
    /// CONTAINER groups: `(container_name, member face NAMES)`. A profile-swept
    /// feature registers the un-keyed name of each of its per-loop face families
    /// here (`{cap_base}_START` -> every per-loop START cap), so a reference
    /// stored before per-loop naming still resolves. See [`SceneMap::face_groups`].
    ///
    /// Members are NAMES, not `(handle, id)` refs, and are resolved at lookup
    /// time. Name fidelity carries a face through a later boolean or dressup, so
    /// a group registered by the extrude keeps naming the right faces after the
    /// solid is consumed and re-registered by something downstream — which a
    /// captured handle would not survive.
    #[serde(default)]
    pub face_groups: Vec<(String, Vec<String>)>,
    /// The same, for the derived edge names that embed those face names.
    #[serde(default)]
    pub edge_groups: Vec<(String, Vec<String>)>,
}

/// The output of one feature execution.
/// A named world point a feature published (a SKETCH publishes every solved
/// point as `{sketchId}:P{pid}`; a spline its anchors, a helix its ends).
/// `construction` carries the sketch point's own flag: a construction point (the
/// sketcher's ◐ toggle, or a materialized external reference) still resolves by
/// name — a helix may start at one — but it is NOT model geometry, so a placement
/// consumer (the hole) skips it exactly as the profile skips construction lines.
/// Serialized because the render-side build report carries it across the worker
/// boundary.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ScenePoint {
    pub position: Vec3,
    pub construction: bool,
}

impl ScenePoint {
    /// A model point (the default for every non-sketch publisher).
    pub fn model(position: Vec3) -> Self {
        Self { position, construction: false }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct FeatureResult {
    #[serde(default, rename = "persistentData", skip_serializing_if = "Option::is_none")]
    pub persistent_data: Option<serde_json::Value>,
    pub id: String,
    pub feature_type: String,
    pub added: Vec<AddedSolid>,
    /// Names of prior solids this feature consumed/superseded (removed from the
    /// scene-map; their handles are freed by the loop).
    pub removed: Vec<String>,
    /// Set on a hard failure. `execute_history` records it and HALTS the
    /// remaining features (mirrors the abort-on-error loop). Never a panic.
    pub error: Option<String>,
    /// The TYPED refusal behind `error`, when the producer that refused minted
    /// one (`KernelRefusal`: a closed class, the stage, the same text). `error`
    /// is always that refusal's message verbatim, so a consumer that only
    /// reads text sees nothing new; one that dispatches on the class reads it
    /// here. `None` on success and on a refusal from a producer that still
    /// returns text.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub refusal: Option<KernelRefusal>,
    /// What a SUCCESSFUL feature did with what it was asked, when that is not
    /// "everything, exactly": the references it applied and the ones it
    /// rejected, each with why. `None` on a refusal and on a feature that did
    /// everything it was asked. A feature that proceeds on the subset of its
    /// selection that resolved says so HERE, typed, instead of only in
    /// `unresolved` (which stays populated for the rename hint). This is a
    /// success stating what it did, not a refusal carrying a partial result —
    /// the payload the 2026-09-11 survey rejected.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fulfilment: Option<Fulfilment>,
    /// Which named motion STEP of a feature refused: Transform Face records
    /// `"rotation"` or `"translation"` (`features::transform_face::REFUSED_STEP_*`)
    /// for a refusal from either motion, alone or in rotate-then-translate.
    /// `None` for every other result, including a Transform Face refused before
    /// any motion ran. The refusal's class is the step's own; this says which
    /// step it was, which no class can. Additive: absent when `None`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub refused_step: Option<String>,
    /// `reference_selection` names this feature could not resolve against the
    /// scene-map. NOT an error/halt — the caller runs a snapshot-repair pass and
    /// re-dispatches (migration-plan Stage 5 gate).
    pub unresolved: Vec<String>,
    /// What the kernel REPAIRED while building this feature, in the words of the
    /// repair that made it. A repair changes the answer the lane returns — the
    /// self-crossing fixer drops material and re-trims its neighbours — so it is
    /// never silent: the feature that ran it says so here, and the case gate
    /// records it per case. Empty on every feature that repaired nothing, which
    /// is nearly all of them.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
    /// MEASURED approximations a SUCCESSFUL result carries
    /// ([`crate::Approximation`]): the result stands, and each entry says by
    /// how much and why it is not exact — today, an imported body whose shell
    /// closes over the pcurve fit bar (`import.shell_closure`), with the bound
    /// that puts on its volume. Distinct from `notes` (a REPAIR changed the
    /// answer), from `fulfilment` (what was done of what was asked) and from
    /// `refusal` (the result does not stand). The case runner records them per
    /// feature and will not read a volume oracle as `correct` inside the bound.
    /// Empty on nearly every feature.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub approximations: Vec<crate::Approximation>,
    /// True when this result was REPLAYED from the incremental history cache (the
    /// feature and everything it references are unchanged since the last run).
    /// The handles are the same resident solids — the caller can skip re-tessellation.
    pub reused: bool,
    /// Sketch profiles this feature produced (a SKETCH feature emits one under
    /// its name; solid features emit none). INTERNAL side-channel: `#[serde(skip)]`
    /// so it never crosses the wasm boundary to the caller — [`SceneMap::apply`] ingests it
    /// into `scene.profiles` for downstream profile-consumers (extrude/revolve/…).
    #[serde(skip)]
    pub profiles: Vec<(String, SketchProfile)>,
    /// Named plane frames this feature produced (DATUM emits three, PLANE one).
    /// Same INTERNAL `#[serde(skip)]` side-channel — ingested into `scene.frames`
    /// so a later SKETCH can resolve its plane by name, fully headless.
    #[serde(skip)]
    pub frames: Vec<(String, Frame)>,
    /// Named axis lines this feature produced (a SKETCH emits one per line
    /// geometry). Same INTERNAL `#[serde(skip)]` side-channel — ingested into
    /// `scene.axes` so a later revolve/sweep can resolve its axis by name.
    #[serde(skip)]
    pub axes: Vec<(String, Axis)>,
    /// Named path chains this feature produced (a SKETCH publishes its ordered
    /// open/closed geometry chain). Same INTERNAL `#[serde(skip)]` side-channel —
    /// ingested into `scene.paths` for sweep/path_sweep/rib trajectory resolution.
    #[serde(skip)]
    pub paths: Vec<(String, Vec<NurbsCurve>)>,
    /// The SOURCE NAME of each curve in a published path chain, parallel to the
    /// matching [`Self::paths`] entry (`{sketchId}:G{gid}` for a sketch segment,
    /// `None` for a curve with no name behind it). Same INTERNAL `#[serde(skip)]`
    /// side-channel — ingested into `scene.path_segment_names` so a multi-segment
    /// consumer (the SWEEP) can name the geometry each PATH SEGMENT built after
    /// that segment, instead of after its position in the chain.
    #[serde(skip)]
    pub path_segment_names: Vec<(String, Vec<Option<String>>)>,
    /// The SCREW AXIS of each curve in a published path chain, parallel to the
    /// matching [`Self::paths`] entry: `Some` for a curve its builder KNOWS is a
    /// helix about that axis (the HELIX feature), `None` for everything else.
    /// Same INTERNAL `#[serde(skip)]` side-channel — ingested into
    /// `scene.path_segment_axes` so the SWEEP's `pathAlign` can carry a profile by
    /// the helix's own screw motion rather than by a rotation-minimizing frame,
    /// which rolls away from it by the helix's integrated torsion.
    #[serde(skip)]
    pub path_segment_axes: Vec<(String, Vec<Option<Axis>>)>,
    /// Named world points this feature produced (a SKETCH publishes every point
    /// as `{sketchId}:P{pid}`, construction flag included — see [`ScenePoint`]).
    /// Same side-channel — ingested into `scene.points` for hole-center /
    /// placement resolution and drawn by the committed-sketch display.
    #[serde(skip)]
    pub points: Vec<(String, ScenePoint)>,
    /// Harness ports this feature produced (a PORT feature emits one under its
    /// id). Same INTERNAL `#[serde(skip)]` side-channel — ingested into
    /// `scene.ports` for spline attachments and the wire-harness tail.
    #[serde(skip)]
    pub ports: Vec<(String, PortRecord)>,
    /// Component records this feature produced (an ACOMP feature emits one per
    /// instance; its members ride `added` like any solids). Same INTERNAL
    /// `#[serde(skip)]` side-channel — ingested into `scene.components` so the
    /// component surface (`component.rs`) resolves against the live scene.
    #[serde(skip)]
    pub components: Vec<ComponentRecord>,
}

impl FeatureResult {
    /// An empty successful result (no solids, no error) — the shape a non-solid
    /// pass-through feature returns.
    pub fn empty(id: impl Into<String>, feature_type: impl Into<String>) -> Self {
        Self {
            persistent_data: None,
            id: id.into(),
            feature_type: feature_type.into(),
            added: Vec::new(),
            removed: Vec::new(),
            error: None,
            refusal: None,
            fulfilment: None,
            refused_step: None,
            unresolved: Vec::new(),
            notes: Vec::new(),
            approximations: Vec::new(),
            reused: false,
            profiles: Vec::new(),
            frames: Vec::new(),
            axes: Vec::new(),
            paths: Vec::new(),
            path_segment_names: Vec::new(),
            path_segment_axes: Vec::new(),
            points: Vec::new(),
            ports: Vec::new(),
            components: Vec::new(),
        }
    }

    /// A hard failure. Halts the history loop.
    pub fn error(
        id: impl Into<String>,
        feature_type: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::refuse(id, feature_type, message.into())
    }

    /// The feature proceeded on the references that resolved and skipped the
    /// ones in `unresolved`: record that as a typed partial fulfilment.
    /// `requested` is the selection as the document names it; `applied` is
    /// what remains after the misses. Nothing is recorded when every reference
    /// resolved. Call it at the point the feature DECIDES to proceed, not at
    /// resolution: a feature that no-ops or refuses on a miss has no partial.
    pub fn note_partial_resolution(&mut self, requested: &[String]) {
        if self.unresolved.is_empty() {
            return;
        }
        let applied: Vec<String> = requested
            .iter()
            .filter(|name| !self.unresolved.contains(name))
            .cloned()
            .collect();
        let rejected = self
            .unresolved
            .iter()
            .map(|name| Rejected {
                name: name.clone(),
                kind: RejectedKind::Unresolved,
                reason: "resolves to nothing on this model".into(),
            })
            .collect();
        self.fulfilment = Some(Fulfilment {
            requested: requested.to_vec(),
            applied,
            rejected,
        });
    }

    /// A hard failure from either kind of producer: `error` carries the text
    /// (the refusal's own message, verbatim, when it is typed) and `refusal`
    /// the class and stage when there is one.
    pub fn refuse(
        id: impl Into<String>,
        feature_type: impl Into<String>,
        refusal: impl Into<FeatureRefusal>,
    ) -> Self {
        let mut result = Self::empty(id, feature_type);
        match refusal.into() {
            FeatureRefusal::Text(message) => result.error = Some(message),
            FeatureRefusal::Typed(refusal) => {
                result.error = Some(refusal.message.clone());
                result.refusal = Some(refusal);
            }
        }
        result
    }

    /// A non-solid construction feature (datum/plane/sketch/spline/port/helix):
    /// a descriptor pass-through with no resident handle. The loop continues.
    pub fn pass_through(id: impl Into<String>, feature_type: impl Into<String>) -> Self {
        Self::empty(id, feature_type)
    }

    /// A solid-producing (or no-kernel) feature not yet ported to Rust. Halts the
    /// loop with a clear message — replaced when a fan-out agent implements the
    /// feature's `features/<feat>.rs`.
    pub fn not_yet_migrated(id: impl Into<String>, feature_type: impl Into<String>) -> Self {
        let feature_type = feature_type.into();
        let message = format!("feature type '{feature_type}' is not yet migrated to the Rust pipeline");
        Self::error(id, feature_type, message)
    }
}

/// The whole-history output.
#[derive(Debug, Clone, Serialize)]
pub struct HistoryResult {
    pub results: Vec<FeatureResult>,
    /// Per-feature wall-clock execution time `(feature id, milliseconds)` in run
    /// order — the lightweight timing surface the engine-native history tree's
    /// "N ms" readout consumes. A REUSED (cache-replayed) feature records `0.0`
    /// (its geometry was not re-executed this run). INTERNAL side-channel:
    /// `#[serde(skip)]` so it never changes the caller-facing `execute_history_json`
    /// shape; brep-render reads it natively via the re-exported struct.
    #[serde(skip)]
    pub timings: Vec<(String, f64)>,
    /// The wire-harness tail's routing report (`None` when the request carries
    /// no `wireHarness` block). INTERNAL side-channel like `timings`: never on
    /// the JSON ABI; brep-render ships it to the harness panel.
    #[serde(skip)]
    pub wire_harness: Option<wire_harness::WireHarnessReport>,
    /// The PMI tail's resolution of every view's annotations (`None` when the
    /// request carries no `pmi` block). INTERNAL side-channel like `timings`.
    #[serde(skip)]
    pub pmi: Option<pmi::PmiReport>,
    /// The ports tail's resolution of the declared-ports block (`None` when the
    /// document declares none). INTERNAL side-channel like `timings`: the tail
    /// is not a feature, so this is where its unresolved references and refused
    /// names surface.
    #[serde(skip)]
    pub ports: Option<ports::PortsReport>,
}

// ===========================================================================
// SceneMap — the live name -> handle/topology index maintained across the loop
// ===========================================================================

// `handle` + `face_id`/`edge_id` are the contract the caller (and edge-selecting
// fan-out features) read to resolve a selection to a resident topology entity.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct FaceRef {
    pub handle: u32,
    pub face_id: u64,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub struct EdgeRef {
    pub handle: u32,
    pub edge_id: u64,
}

/// A solved sketch's extracted profile, placed in 3D. Produced by the SKETCH
/// pipeline feature and consumed BY NAME by the profile-consumers (extrude,
/// revolve, loft, sweep, rib, sheet-metal tab/contour-flange) — the exact-BREP
/// replacement for the old "marshal the resolved curves into `inputParams`".
///
/// The closed loops are grouped into disjoint REGIONS by even-odd containment
/// depth: a depth-EVEN loop is a region's OUTER boundary, a depth-ODD loop is a
/// hole of the region whose outer directly contains it (so an island inside a
/// hole is its own region). Each region is `[outer, holes…]`; consumers build
/// one solid per region and UNION them — the established multi-region behavior.
/// The plane frame (`origin` + orthonormal `x_axis`/`y_axis`/`z_axis`) is resolved
/// LIVE in Rust from the sketch's plane reference (DATUM/PLANE feature frame or a
/// resident face), falling back to the persisted `basis`; the loop curves are
/// emitted in WORLD space via that frame, so `z_axis` is the natural extrude
/// direction / a revolve-plane hint.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SketchProfile {
    pub origin: Vec3,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
    pub z_axis: Vec3,
    /// Disjoint regions, each `[outer, holes…]` (containment-classified).
    pub regions: Vec<Vec<ProfileLoop>>,
}

/// One closed boundary of a [`SketchProfile`]: head-to-tail world-space curves plus
/// the per-curve source edge name (`{sketchId}:G{gid}`, parallel to `curves`). The
/// profile-consumers stamp sidewall names from `edge_names` (extrude → `{name}_E`).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ProfileLoop {
    pub curves: Vec<NurbsCurve>,
    pub edge_names: Vec<Option<String>>,
    /// The loop's STABLE identity within its source sketch (see the sketch
    /// feature's `loop_ids`), persisted per edge in `persistentData.sketch` and
    /// resilient to adding/removing edges. The profile-swept features embed it in
    /// their per-loop cap and hole-wall names, so those names no longer move when
    /// a DIFFERENT loop is added or removed. `None` for a loop with no numeric
    /// geometry id behind it (a FACE profile, or a hand-built test profile) —
    /// [`ProfileLoop::key`] then falls back to the loop's own edge names.
    #[serde(default)]
    pub loop_id: Option<u64>,
}

impl ProfileLoop {
    /// This loop's naming KEY — the token the profile-swept features embed in the
    /// faces they build from it (`{cap_base}:{key}_START`, `{id}:HOLE:{key}`).
    ///
    /// `L{loop_id}` for a sketch loop. `None` when the loop has no identity to key
    /// on, and the caller keeps its previous un-keyed spelling: a FACE profile
    /// (always a single region — nothing to disambiguate, and its faces keep the
    /// names every saved model already stores) or a hand-built profile with no
    /// numeric geometry ids behind it.
    pub fn key(&self) -> Option<String> {
        self.loop_id.map(|id| format!("L{id}"))
    }
}

/// Annotate an UNRESOLVED reference with the name that replaced it, when the miss
/// looks like the per-loop naming change.
///
/// Per-loop cap/hole naming keyed names that used to be positional
/// (`{cap_base}_START` -> `{cap_base}:L4_START`, `{id}:HOLE:1` -> `{id}:HOLE:L9`).
/// Live models mostly do not need this — the CONTAINER groups keep the old
/// spelling resolvable — but a reference the containers cannot cover (a positional
/// `X[1]` disambiguation from before the change, or a `{id}:HOLE:{n}` written
/// against a since-edited sketch) still misses, and a bare "unresolved" says
/// nothing about why. This turns the miss into the fix:
///
/// ```text
/// E4:S3:PROFILE_START[1] (renamed: E4:S3:PROFILE:L9_START)
/// ```
///
/// The candidates come from the live scene, so the hint is what the model
/// ACTUALLY built, not a guess.
pub fn annotate_rename(scene: &SceneMap, name: &str) -> String {
    let candidates = rename_candidates(scene, name);
    if candidates.is_empty() {
        return name.to_string();
    }
    format!("{name} (renamed: {})", candidates.join(", "))
}

/// The keyed names that plausibly replaced `name`. A stale reference differs from
/// its replacement only by the inserted loop key, so a candidate is any live face
/// name that matches once the key is accounted for — `{stem}:{key}_START` for a
/// cap, `{id}:HOLE:{key}` for a hole wall. A trailing positional `[n]` (the old
/// duplicate-name disambiguation, the very thing this change removes) is stripped
/// before matching.
fn rename_candidates(scene: &SceneMap, name: &str) -> Vec<String> {
    let stale = name.rsplit_once('[').map_or(name, |(head, tail)| {
        if tail.ends_with(']') && tail[..tail.len() - 1].chars().all(|c| c.is_ascii_digit()) {
            head
        } else {
            name
        }
    });
    // Cap: `{stem}_START` / `{stem}_END` -> `{stem}:{key}_START` / `_END`.
    let cap = ["_START", "_END"]
        .into_iter()
        .find_map(|suffix| stale.strip_suffix(suffix).map(|stem| (stem.to_string(), suffix)));
    // Hole wall: `{id}:HOLE:{anything}` -> `{id}:HOLE:{key}`.
    let hole = stale
        .rsplit_once(":HOLE:")
        .map(|(owner, _)| format!("{owner}:HOLE:"));

    let mut candidates: Vec<String> = scene
        .faces
        .keys()
        .filter(|live| {
            if let Some((stem, suffix)) = &cap {
                if let Some(rest) = live.strip_prefix(stem.as_str()) {
                    // Exactly `:{key}` was inserted before the suffix.
                    return rest.starts_with(':')
                        && rest.ends_with(suffix)
                        && rest.len() > suffix.len() + 1
                        && !rest[1..rest.len() - suffix.len()].contains(':');
                }
            }
            if let Some(prefix) = &hole {
                return live.starts_with(prefix.as_str()) && live.as_str() != stale;
            }
            false
        })
        .cloned()
        .collect();
    candidates.sort();
    candidates
}

/// An orthonormal placement frame — a named plane (DATUM/PLANE feature) or a
/// resolved sketch plane. `z_axis` is the plane normal.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Frame {
    pub origin: Vec3,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
    pub z_axis: Vec3,
}

impl Frame {
    /// Derive an orthonormal frame from an origin + plane normal via the worldUp
    /// convention, the
    /// SINGLE source of truth for how a sketch/plane reference becomes in-plane
    /// axes: `refUp = |n·(0,1,0)| > 0.9 ? (1,0,0) : (0,1,0)`;
    /// `x = norm(refUp × n)`; `y = norm(n × x)`; `z = n`. The referenced object's
    /// own x/y are intentionally ignored — only its (origin, normal) matter.
    pub fn from_origin_normal(origin: Vec3, normal: Vec3) -> Result<Self, String> {
        let z_axis = normal.normalized()?;
        let world_up = Vec3::new(0.0, 1.0, 0.0);
        let ref_up = if z_axis.dot(world_up).abs() > 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            world_up
        };
        let x_axis = ref_up.cross(z_axis).normalized()?;
        let y_axis = z_axis.cross(x_axis).normalized()?;
        Ok(Frame {
            origin,
            x_axis,
            y_axis,
            z_axis,
        })
    }

    /// Derive an orthonormal frame whose **x_axis** is `direction` — the frame a
    /// connection point's placement is an offset in (its seat, `ports::seat_of`).
    ///
    /// A connection point's direction is its rotation applied to **+X** (the
    /// convention every feature transform reads), so the frame that seats one
    /// must carry the direction on X, not on Z. Rather than invent a second
    /// worldUp rule this is the CYCLIC RELABEL of [`Self::from_origin_normal`]'s
    /// triad — x = z', y = x', z = y' — which is still right-handed and keeps
    /// ONE convention for how a direction becomes a basis. A sketch on the
    /// point's published frame and the point's own offset axes therefore agree
    /// by construction: they are the same three vectors, named round.
    pub fn from_origin_direction(origin: Vec3, direction: Vec3) -> Result<Self, String> {
        let normal = Self::from_origin_normal(origin, direction)?;
        Ok(Frame {
            origin,
            x_axis: normal.z_axis,
            y_axis: normal.x_axis,
            z_axis: normal.y_axis,
        })
    }

    /// Map a plane-local `(u, v)` into world space: `origin + u·x + v·y`.
    pub fn to_3d(&self, u: f64, v: f64) -> Vec3 {
        self.origin
            .add(self.x_axis.scale(u))
            .add(self.y_axis.scale(v))
    }
}

/// A named world-space line — a revolve/sweep axis. Published by SKETCH features
/// (every sketch line geometry, construction included) and resolvable from a
/// resident solid edge, so an `axis`/path reference resolves fully headless.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Axis {
    pub point: Vec3,
    pub direction: Vec3,
}

/// What a connection point is to the harness router: a cable END, or a
/// pass-through that joins spline paths into a network. Not a user param any
/// more - it is decided by WHICH producer made the record (a declared port
/// point is a termination, a WAYPOINT feature is a waypoint).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PortKind {
    Termination,
    Waypoint,
}

/// One resolved connection point, keyed in [`SceneMap::ports`] by its ADDRESS
/// (`J1.VCC`, `ACOMP3:J1.VCC` once placed - see `ports.rs`). Published by the
/// ports tail, by the WAYPOINT feature (under its own feature id) and by an
/// ACOMP re-publishing a placed part's points.
///
/// `direction` is the unit vector a wire leaves along on side **A**; side **B**
/// is its negation. `extension` is the straight run a wire keeps from `point`
/// before it may bend (a spline anchor attached here takes it as its
/// forward/backward distance) and `display_length` the length of the drawn
/// line.
///
/// `port_name` / `point_name` / `purpose` stay PART-LOCAL through every
/// namespacing: a placed part's point is still point `VCC` of port `J1`
/// whatever occurrence chain its address carries. That is what a pin binds to.
/// A WAYPOINT carries an empty `port_name` and `purpose`, and its own name as
/// `point_name`.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PortRecord {
    pub point: Vec3,
    pub direction: Vec3,
    pub kind: PortKind,
    pub extension: f64,
    pub display_length: f64,
    #[serde(default, rename = "portName")]
    pub port_name: String,
    #[serde(default, rename = "pointName")]
    pub point_name: String,
    #[serde(default)]
    pub purpose: String,
}

impl PortRecord {
    /// The PART-LOCAL address of this point (`J1.VCC`), or just the name for a
    /// WAYPOINT, which belongs to no port. The occurrence chain is the map key's
    /// business, not the record's.
    pub fn local_address(&self) -> String {
        if self.port_name.is_empty() {
            self.point_name.clone()
        } else {
            ports::address(&self.port_name, &self.point_name)
        }
    }

    /// The direction a wire leaves this port along on `side` (`A` = the port
    /// direction, `B` = its negation).
    pub fn side_direction(&self, side: wire_harness::PortSide) -> Vec3 {
        match side {
            wire_harness::PortSide::A => self.direction,
            wire_harness::PortSide::B => self.direction.scale(-1.0),
        }
    }
}

/// The live scene-map. Exact-match resolution only (contract rule 1).
#[derive(Debug, Clone, Default)]
pub struct SceneMap {
    pub solids: HashMap<String, u32>,
    pub faces: HashMap<String, FaceRef>,
    pub edges: HashMap<String, EdgeRef>,
    /// CONTAINER name -> the faces it stands for. A profile-swept feature names
    /// each of its per-loop caps individually (`{cap_base}:L{loopId}_START`) and
    /// registers the un-keyed `{cap_base}_START` here as the name of the whole
    /// sketch container — every cap the feature made. So the name a model saved
    /// before per-loop naming still resolves, and still means the right thing:
    /// the one cap on a single-loop sketch, all of them on a multi-loop one.
    pub face_groups: HashMap<String, Vec<String>>,
    /// CONTAINER name -> the edges it stands for. The derived edge-name
    /// convention embeds face names (`{faceA}|{faceB}[n]`), so keying the caps
    /// renamed every cap-adjacent edge too; this holds the container-name
    /// spelling of each such edge for the same reason [`Self::face_groups`] does.
    pub edge_groups: HashMap<String, Vec<String>>,
    /// Sketch name -> its extracted profile. Populated by SKETCH features; read by
    /// profile-consumers via [`SceneMap::resolve_profile`].
    pub profiles: HashMap<String, SketchProfile>,
    /// Plane name -> its frame. Populated by DATUM/PLANE features; read by SKETCH
    /// (and PLANE datum-refs) via [`SceneMap::resolve_frame`] for headless plane
    /// resolution (fully headless).
    pub frames: HashMap<String, Frame>,
    /// Axis name -> its world line. Populated by SKETCH line geometries; read by
    /// revolve/sweep via [`SceneMap::resolve_axis`].
    pub axes: HashMap<String, Axis>,
    /// Path name -> its ordered world curve chain (open or closed). Populated by
    /// SKETCH features; read by sweep/path_sweep/rib via [`SceneMap::resolve_path`].
    pub paths: HashMap<String, Vec<NurbsCurve>>,
    /// Path name -> the SOURCE NAME of each curve in its chain, parallel to the
    /// [`Self::paths`] entry of the same name. Populated by SKETCH features; read
    /// by the SWEEP via [`SceneMap::resolve_path_segment_names`] so the faces one
    /// path SEGMENT builds carry that segment's name rather than its position.
    pub path_segment_names: HashMap<String, Vec<Option<String>>>,
    /// Path name -> the SCREW AXIS of each curve in its chain (`Some` for a
    /// helix), parallel to the [`Self::paths`] entry of the same name. Populated
    /// by HELIX features; read by the SWEEP via
    /// [`SceneMap::resolve_path_segment_axes`].
    pub path_segment_axes: HashMap<String, Vec<Option<Axis>>>,
    /// Point name -> its world position + construction flag. Populated by SKETCH
    /// features (`{sketchId}:P{pid}`), splines and helices; read by hole placement
    /// via [`SceneMap::model_points_with_prefix`] and by name via
    /// [`SceneMap::resolve_point`].
    pub points: HashMap<String, ScenePoint>,
    /// Connection-point ADDRESS -> its record. Populated by the ports tail
    /// (the document's declared points), by WAYPOINT features and by ACOMP
    /// (a placed part's points, namespaced and posed); read by SPLINE
    /// attachments ([`SceneMap::resolve_port`]) and by the wire-harness tail,
    /// which builds its sided routing graph from it.
    pub ports: HashMap<String, PortRecord>,
    /// Owning feature id -> component record (assemblies). Populated by ACOMP
    /// features via the `components` side-channel; the whole component API
    /// surface (`resolve_component` / `owning_component` / the fence predicate)
    /// lives in component.rs. Empty in every componentless scene — nothing in
    /// the ordinary modeling path reads or writes it.
    pub components: component::ComponentMap,
    /// Naming-contract guard (accumulated across [`SceneMap::apply`] calls): one
    /// message per detected cross-solid name COLLISION — an added solid's face /
    /// edge name that already maps to a DIFFERENT still-resident solid. Two live
    /// solids must never share a face/edge name (the SceneMap is a global
    /// name → one-solid map). `execute_history` surfaces any collision introduced
    /// by a freshly-run feature as that feature's error, so the offending
    /// operation trips its own tests + shows an error in the app.
    pub name_collisions: Vec<String>,
}

impl SceneMap {
    /// Resolve a solid name to its resident handle (exact match).
    pub fn resolve_solid(&self, name: &str) -> Option<u32> {
        self.solids.get(name).copied()
    }

    /// Resolve a face name to its `(handle, face_id)` (exact match).
    pub fn resolve_face(&self, name: &str) -> Option<FaceRef> {
        if let Some(face) = self.faces.get(name) {
            return Some(*face);
        }
        // A CONTAINER name holding exactly ONE face resolves to it. Not a scoring
        // heuristic (contract rule 1 stands): the container is a name the feature
        // registered deliberately, and a single member leaves nothing to choose
        // between. This is what keeps every reference in a model saved before
        // per-loop cap naming resolving unchanged — a single-loop sketch has one
        // cap, so its container has one member. A container with SEVERAL members
        // is ambiguous for a single-face consumer and stays unresolved; the
        // multi-face consumers take it through [`Self::resolve_face_group`].
        match self.resolve_face_group(name).as_slice() {
            [only] => Some(*only),
            _ => None,
        }
    }

    /// Every face a name stands for: the one exact match, else the members of the
    /// container by that name (possibly several). The multi-face resolution the
    /// selection-list consumers use — [`Self::resolve_face`] is its single-face
    /// sibling.
    pub fn resolve_face_group(&self, name: &str) -> Vec<FaceRef> {
        if let Some(face) = self.faces.get(name) {
            return vec![*face];
        }
        // Members are resolved NOW, so a member a later operation consumed simply
        // drops out instead of leaving a stale handle behind.
        self.face_groups
            .get(name)
            .map(|members| {
                members
                    .iter()
                    .filter_map(|member| self.faces.get(member).copied())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Every edge a name stands for — [`Self::resolve_face_group`] for edges.
    pub fn resolve_edge_group(&self, name: &str) -> Vec<EdgeRef> {
        if let Some(edge) = self.edges.get(name) {
            return vec![*edge];
        }
        self.edge_groups
            .get(name)
            .map(|members| {
                members
                    .iter()
                    .filter_map(|member| self.edges.get(member).copied())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Resolve an edge name to its `(handle, edge_id)` (exact match). Contract
    /// surface for edge-selecting fan-out features (fillet/chamfer); no
    /// implemented feature consumes it yet.
    #[allow(dead_code)]
    pub fn resolve_edge(&self, name: &str) -> Option<EdgeRef> {
        if let Some(edge) = self.edges.get(name) {
            return Some(*edge);
        }
        // The [`Self::resolve_face`] rule for edges: derived edge names embed face
        // names, so keying the caps renamed every cap-adjacent edge too, and a
        // container holding exactly one edge resolves to it. This is what keeps a
        // stored projected-edge / axis / path reference working.
        match self.resolve_edge_group(name).as_slice() {
            [only] => Some(*only),
            _ => None,
        }
    }

    /// Resolve a profile reference name to the sketch profile it names. Tries the
    /// exact name first, then the name with a trailing `:PROFILE` stripped — a
    /// sketch is selectable both by its group name (`{id}`) and by its profile
    /// face name (`{id}:PROFILE`); both must resolve to
    /// the one profile stored under `{id}`.
    pub fn resolve_profile(&self, name: &str) -> Option<&SketchProfile> {
        if let Some(profile) = self.profiles.get(name) {
            return Some(profile);
        }
        name.strip_suffix(":PROFILE")
            .and_then(|base| self.profiles.get(base))
    }

    /// Resolve a named plane frame (exact match), for sketch/plane resolution.
    pub fn resolve_frame(&self, name: &str) -> Option<Frame> {
        self.frames.get(name).copied()
    }

    /// Resolve a named axis line (exact match), for revolve/sweep.
    pub fn resolve_axis(&self, name: &str) -> Option<Axis> {
        self.axes.get(name).copied()
    }

    /// Resolve a named path chain (exact match; also strips a trailing `:PROFILE`),
    /// for sweep/path_sweep/rib trajectories.
    pub fn resolve_path(&self, name: &str) -> Option<&Vec<NurbsCurve>> {
        if let Some(path) = self.paths.get(name) {
            return Some(path);
        }
        name.strip_suffix(":PROFILE")
            .and_then(|base| self.paths.get(base))
    }

    /// The per-segment SOURCE NAMES of a named path chain, resolved exactly as
    /// [`SceneMap::resolve_path`] resolves the chain itself (so the two always
    /// answer for the same entry). `None` when the publisher named no segments —
    /// the caller then falls back to naming by the reference itself.
    pub fn resolve_path_segment_names(&self, name: &str) -> Option<&Vec<Option<String>>> {
        if let Some(names) = self.path_segment_names.get(name) {
            return Some(names);
        }
        name.strip_suffix(":PROFILE")
            .and_then(|base| self.path_segment_names.get(base))
    }

    /// The per-segment SCREW AXES of a named path chain, resolved exactly as
    /// [`SceneMap::resolve_path`] resolves the chain itself. `None` when the
    /// publisher recorded none — every segment is then an ordinary curve.
    pub fn resolve_path_segment_axes(&self, name: &str) -> Option<&Vec<Option<Axis>>> {
        if let Some(axes) = self.path_segment_axes.get(name) {
            return Some(axes);
        }
        name.strip_suffix(":PROFILE")
            .and_then(|base| self.path_segment_axes.get(base))
    }

    /// Resolve a named world point (exact match), for hole/placement centers.
    /// Contract surface consumed by the spline/hole placement tests; kept as the
    /// documented point-resolution API even where no shipping feature reads it
    /// yet (mirrors `resolve_edge`).
    #[allow(dead_code)]
    pub fn resolve_point(&self, name: &str) -> Option<Vec3> {
        self.points.get(name).map(|point| point.position)
    }

    /// Resolve a connection point by its address (exact match).
    pub fn resolve_port(&self, name: &str) -> Option<&PortRecord> {
        self.ports.get(name)
    }

    /// Every MODEL (non-construction) point whose name starts with `prefix`, as
    /// `(name, position)` pairs in map order (the caller sorts). The scene-map
    /// indexes points by exact name only, so a consumer that wants "all the
    /// points of sketch S" prefix-scans `S:P` — safe because the prefix ends in
    /// `:P`, which another sketch id cannot spell into (`S1:P` never prefixes
    /// `S10:P…`). Construction points (the sketcher's ◐ toggle, materialized
    /// external references) are skipped: they constrain, they never model.
    pub fn model_points_with_prefix(&self, prefix: &str) -> Vec<(String, Vec3)> {
        self.points
            .iter()
            .filter(|(name, point)| name.starts_with(prefix) && !point.construction)
            .map(|(name, point)| (name.clone(), point.position))
            .collect()
    }

    /// Apply a feature's effect: removals first (a boolean result reuses a removed
    /// target's name), then additions. Removal only unmaps NAMES — it never frees
    /// the handle. Handles are owned by the incremental history cache (each by the
    /// entry of the feature that produced it) and die exactly when that entry is
    /// invalidated; freeing here would kill a clean upstream feature's cached
    /// solid the moment a downstream boolean consumed it.
    fn apply(&mut self, result: &FeatureResult) {
        for name in &result.removed {
            // A removed COMPONENT id drops the record and unmaps every member
            // solid (names only — handle ownership stays with the cache, same
            // as plain solid removal). Feature ids never collide with solid
            // names, so this is a no-op for every ordinary removal.
            if let Some(record) = self.components.remove(name) {
                for member in &record.solids {
                    if let Some(handle) = self.solids.remove(member) {
                        self.faces.retain(|_, face| face.handle != handle);
                        self.edges.retain(|_, edge| edge.handle != handle);
                    }
                }
                // ...and the ports the part carried, with their entities.
                for port in &record.ports {
                    self.ports.remove(port);
                    self.axes.remove(port);
                    self.frames.remove(port);
                    self.points.remove(&format!("{port}:Base"));
                    self.paths.remove(&format!("{port}:PortLine"));
                }
            }
            if let Some(handle) = self.solids.remove(name) {
                // Purge the removed solid's face/edge entries (names only).
                self.faces.retain(|_, face| face.handle != handle);
                self.edges.retain(|_, edge| edge.handle != handle);
            }
            // A consumed sketch's PUBLISHED GEOMETRY (profile / axis / path / point)
            // is deliberately NOT purged here. Unlike a solid, one sketch can drive
            // several features — a later feature legitimately resolves the sketch's
            // axis or path even after another feature consumed its profile (see
            // `revolve::tests::extruded_face_then_revolve_face_profile`, which
            // revolves about a consumed sketch's `Sk:G20` axis). Removing a consumed
            // sketch from the SCENE DISPLAY is handled render-side (the committed-
            // sketch list honors `removed`); the kernel keeps its geometry resolvable.
            // A removed name not in the map is a silent no-op (established semantics).
        }
        // Ingest any sketch profiles this feature produced (SKETCH features only).
        for (name, profile) in &result.profiles {
            self.profiles.insert(name.clone(), profile.clone());
        }
        // Ingest any named plane frames (DATUM/PLANE features).
        for (name, frame) in &result.frames {
            self.frames.insert(name.clone(), *frame);
        }
        // Ingest any named axis lines (SKETCH line geometries).
        for (name, axis) in &result.axes {
            self.axes.insert(name.clone(), *axis);
        }
        // Ingest any named path chains (SKETCH geometry chains).
        for (name, names) in &result.path_segment_names {
            self.path_segment_names.insert(name.clone(), names.clone());
        }
        for (name, axes) in &result.path_segment_axes {
            self.path_segment_axes.insert(name.clone(), axes.clone());
        }
        for (name, path) in &result.paths {
            self.paths.insert(name.clone(), path.clone());
        }
        // Ingest any named world points (SKETCH points).
        for (name, point) in &result.points {
            self.points.insert(name.clone(), *point);
        }
        // Ingest any harness ports (PORT features).
        for (name, port) in &result.ports {
            self.ports.insert(name.clone(), port.clone());
        }
        // Ingest any component records (ACOMP features).
        for record in &result.components {
            self.components.insert(record.id.clone(), record.clone());
        }
        // Insert the added solids' names FIRST so the resident-handle set is
        // complete (a name colliding with another solid ADDED in the same result
        // is still a collision).
        for added in &result.added {
            self.solids.insert(added.name.clone(), added.handle);
        }
        let resident: std::collections::HashSet<u32> = self.solids.values().copied().collect();
        for added in &result.added {
            for (face_id, name) in &added.face_names {
                self.check_name_collision(name, added.handle, &added.name, &resident, true);
                self.faces.insert(
                    name.clone(),
                    FaceRef {
                        handle: added.handle,
                        face_id: *face_id,
                    },
                );
            }
            for (edge_id, name) in &added.edge_names {
                self.check_name_collision(name, added.handle, &added.name, &resident, false);
                self.edges.insert(
                    name.clone(),
                    EdgeRef {
                        handle: added.handle,
                        edge_id: *edge_id,
                    },
                );
            }
            // Container groups. NOT collision-checked: a container is a name for
            // faces this same solid owns, never a claim on another solid's name,
            // and a real face by that name always wins (both resolvers try the
            // exact maps first). Stored as member NAMES and resolved on lookup,
            // so a group survives the solid being consumed and re-registered by a
            // later feature (name fidelity carries the members across).
            for (container, members) in &added.face_groups {
                self.face_groups.insert(container.clone(), members.clone());
            }
            for (container, members) in &added.edge_groups {
                self.edge_groups.insert(container.clone(), members.clone());
            }
        }
    }

    /// Record a naming-contract collision if `name` (a face when `is_face`, else
    /// an edge) already maps to a DIFFERENT still-RESIDENT solid — an added
    /// solid's name stealing another live solid's. A `prev` handle no longer in
    /// `resident` is a stale leftover (rollback / cache eviction), not a
    /// violation, so it is ignored. The message names the offender + the resident
    /// owner and points at the fix.
    fn check_name_collision(
        &mut self,
        name: &str,
        handle: u32,
        solid_name: &str,
        resident: &std::collections::HashSet<u32>,
        is_face: bool,
    ) {
        let prev = if is_face {
            self.faces.get(name).map(|face| face.handle)
        } else {
            self.edges.get(name).map(|edge| edge.handle)
        };
        let Some(prev_handle) = prev else { return };
        if prev_handle == handle || !resident.contains(&prev_handle) {
            return;
        }
        let owner = self
            .solids
            .iter()
            .find(|(_, resident_handle)| **resident_handle == prev_handle)
            .map(|(owner_name, _)| owner_name.as_str())
            .unwrap_or("another resident solid");
        let kind = if is_face { "face" } else { "edge" };
        // One line PER colliding name; the `namespace_copy_names` hint is appended
        // ONCE per feature error where these lines are joined (execute_history).
        self.name_collisions.push(format!(
            "naming-contract violation: solid `{solid_name}` {kind} name `{name}` \
             collides with resident solid `{owner}` (two live solids must not share a \
             face/edge name)."
        ));
    }
}

/// Join a feature's per-name collision lines into ONE error string, appending the
/// `namespace_copy_names` fix hint exactly ONCE (a copy-feature can collide on
/// dozens of names — repeating the hint per name buried the signal). Shared so the
/// surfacing in `execute_history` and its test agree on the wording.
pub(crate) fn name_collision_error(lines: &[String]) -> String {
    let mut message = lines.join("; ");
    message.push_str(
        " — an operation that COPIES geometry (mirror / pattern / transform-copy / \
         any new one) must call `common::namespace_copy_names(&mut solid, suffix)` \
         so the copy's faces and edges get UNIQUE names.",
    );
    message
}

// ===========================================================================
// FeatureContext — everything a feature's `execute(ctx)` needs
// ===========================================================================

/// The read-only context handed to each feature's `execute`. Features resolve
/// their `reference_selection` params against `scene` and read numeric params via
/// [`FeatureContext::number`] (which evaluates expression strings against `env`).
pub struct FeatureContext<'a> {
    pub id: String,
    pub feature_type: String,
    pub params: &'a serde_json::Value,
    /// The feature's `persistentData` (round-tripped from the saved part file).
    /// The SKETCH feature reads its `sketch` + `basis` here (the scene-computed
    /// plane frame stays caller-side per directive §1 and is marshaled in via this).
    pub persistent: &'a serde_json::Value,
    pub env: &'a Env,
    pub scene: &'a SceneMap,
}

impl<'a> FeatureContext<'a> {
    /// The raw param value (an object field of `inputParams`), if present.
    pub fn param(&self, key: &str) -> Option<&serde_json::Value> {
        self.params.get(key)
    }

    /// A numeric param: a JSON number passes through; a JSON string is evaluated
    /// as an expression against the shared env; anything else is an error. This is
    /// how a feature declares which of its params are numeric (the engine has no
    /// per-feature schema).
    pub fn number(&self, key: &str) -> Result<f64, String> {
        match self.param(key) {
            Some(serde_json::Value::Number(number)) => number
                .as_f64()
                .ok_or_else(|| format!("param `{key}` is not a finite number")),
            Some(serde_json::Value::String(source)) => self
                .env
                .eval(source)
                .map_err(|error| format!("param `{key}`: {error}")),
            Some(other) => Err(format!(
                "param `{key}` must be a number or expression string, found {other}"
            )),
            None => Err(format!("missing required param `{key}`")),
        }
    }

    /// A string param (read verbatim — never expression-evaluated; used for ids
    /// and `reference_selection` names). Contract surface for fan-out features;
    /// the two implemented features read the id via `ctx.id` directly.
    #[allow(dead_code)]
    pub fn string(&self, key: &str) -> Option<String> {
        match self.param(key) {
            Some(serde_json::Value::String(text)) => Some(text.clone()),
            _ => None,
        }
    }

    /// A convenience [`FeatureResult::refuse`] carrying this feature's id +
    /// type: text from a stringly producer, or a typed [`KernelRefusal`].
    pub fn fail(&self, refusal: impl Into<FeatureRefusal>) -> FeatureResult {
        FeatureResult::refuse(self.id.clone(), self.feature_type.clone(), refusal)
    }
}

/// What a SUCCESSFUL feature did with what it was asked, when that is not
/// "everything, exactly" (plan: typed-refusal-reporting, item 3). Requested and
/// applied are reference names as the document wrote them; every rejected one
/// says which kind of rejection and why. A partial fulfilment is a different
/// answer from the one asked for, so it is reported on the feature that made it
/// and displayed by the app beside the feature; it is never a refusal class
/// (a partial selection is a separate outcome) and never carries geometry.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Fulfilment {
    pub requested: Vec<String>,
    pub applied: Vec<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub rejected: Vec<Rejected>,
}

impl Fulfilment {
    /// Something asked for was not done.
    pub fn is_partial(&self) -> bool {
        !self.rejected.is_empty()
    }

    /// One line for a report or a tree leaf: "2 of 3 selected references
    /// applied; `ghost` resolves to nothing on this model".
    pub fn summary(&self) -> String {
        let misses: Vec<String> = self
            .rejected
            .iter()
            .map(|rejected| format!("`{}` {}", rejected.name, rejected.reason))
            .collect();
        format!(
            "{} of {} selected reference{} applied; {}",
            self.applied.len(),
            self.requested.len(),
            if self.requested.len() == 1 { "" } else { "s" },
            misses.join("; ")
        )
    }
}

/// One reference a successful feature did not act on.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rejected {
    pub name: String,
    pub kind: RejectedKind,
    pub reason: String,
}

/// Why a reference was rejected. Closed; a new kind is a producer that exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RejectedKind {
    /// The name resolves to no entity of the scene.
    Unresolved,
}

/// What a feature's `build` hands back when it refuses. A feature that drives
/// a typed producer propagates the [`KernelRefusal`] whole (`?` converts it),
/// so its class reaches [`FeatureResult::refusal`]; one that drives a stringly
/// producer propagates the text. There is no `From<FeatureRefusal> for
/// String`: the text of a typed refusal is read through
/// [`FeatureRefusal::message`], never by dropping the class.
#[derive(Clone, Debug, PartialEq)]
pub enum FeatureRefusal {
    Text(String),
    Typed(KernelRefusal),
}

impl FeatureRefusal {
    /// The human text either way, verbatim.
    pub fn message(&self) -> &str {
        match self {
            Self::Text(message) => message,
            Self::Typed(refusal) => &refusal.message,
        }
    }
}

impl From<String> for FeatureRefusal {
    fn from(message: String) -> Self {
        Self::Text(message)
    }
}

impl From<&str> for FeatureRefusal {
    fn from(message: &str) -> Self {
        Self::Text(message.to_string())
    }
}

impl From<KernelRefusal> for FeatureRefusal {
    fn from(refusal: KernelRefusal) -> Self {
        Self::Typed(refusal)
    }
}

impl std::fmt::Display for FeatureRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.message())
    }
}

// ===========================================================================
// Dispatch — a match on `feature_type` to the owning `features/<feat>.rs`
// ===========================================================================

/// Extract the feature id from `inputParams` (`id`, falling back to the
/// non-enumerable `featureID`). The solid + all face/edge names are prefixed
/// with this id, so name fidelity depends on it.
pub(crate) fn extract_id(params: &serde_json::Value) -> String {
    for key in ["id", "featureID"] {
        if let Some(serde_json::Value::String(text)) = params.get(key) {
            if !text.is_empty() {
                return text.clone();
            }
        }
    }
    String::new()
}

/// Resolve one feature descriptor to a [`FeatureResult`] against the current
/// scene. Each stub's arm calls its own `features::<feat>::execute` so a fan-out
/// agent implementing a feature touches ONLY that file — never this dispatch.
pub fn execute_feature(
    descriptor: &FeatureDescriptor,
    env: &Env,
    scene: &SceneMap,
) -> FeatureResult {
    let id = extract_id(&descriptor.input_params);
    let feature_type = descriptor.feature_type.clone();
    // The feature fence: component geometry is valid input for sketch
    // attach/project and assembly constraints ONLY. A modeling feature naming
    // it as an operand fails cleanly HERE — the one altitude every feature
    // passes through — instead of per-feature checks. Free for componentless
    // scenes (the walk is skipped entirely).
    if let Err(message) = component::enforce_reference_fence(&feature_type, descriptor, scene) {
        return FeatureResult::error(id, feature_type, message);
    }
    let ctx = FeatureContext {
        id: id.clone(),
        feature_type: feature_type.clone(),
        params: &descriptor.input_params,
        persistent: &descriptor.persistent_data,
        env,
        scene,
    };
    // Alias resolution mirrors the caller's feature registry, which matches short name,
    // LONG name, and class name (all uppercased) — saved files and tests use
    // long-name type strings like "CHAMFER"/"PLANE".
    match feature_type.as_str() {
        // --- Fully implemented (the exemplary pattern) ---
        "P.CU" | "CUBE" => features::cube::execute(&ctx),
        "P.S" | "SPHERE" => features::sphere::execute(&ctx),
        // --- Solid-producing primitives ---
        "P.CY" | "CYLINDER" => features::cylinder::execute(&ctx),
        "P.CO" | "CONE" => features::cone::execute(&ctx),
        "P.T" | "TORUS" => features::torus::execute(&ctx),
        "P.PY" | "PYRAMID" => features::pyramid::execute(&ctx),
        // --- Add-material ---
        "E" | "EXTRUDE" => features::extrude::execute(&ctx),
        "R" | "REVOLVE" => features::revolve::execute(&ctx),
        "LOFT" => features::loft::execute(&ctx),
        "SW" | "SWEEP" => features::sweep::execute(&ctx),
        "SWP" | "PATH SWEEP" | "PATHSWEEP" => features::path_sweep::execute(&ctx),
        "RIB" => features::rib::execute(&ctx),
        "TU" | "TUBE" => features::tube::execute(&ctx),
        // --- Edit / boolean / transform ---
        "B" | "BOOLEAN" => features::boolean::execute(&ctx),
        "M" | "MIRROR" => features::mirror::execute(&ctx),
        "SPL" | "SPLIT" => features::split::execute(&ctx),
        "XFORM" | "TRANSFORM" => features::transform::execute(&ctx),
        "PATTERN" => features::pattern::execute(&ctx),
        // --- Dressups ---
        "F" | "FILLET" => features::fillet::execute(&ctx),
        "CH" | "CHAMFER" => features::chamfer::execute(&ctx),
        "O.S" | "OFFSET SHELL" | "OFFSETSHELL" => features::offset_shell::execute(&ctx),
        "O.F" | "OFFSET FACE" | "OFFSETFACE" => features::offset_face::execute(&ctx),
        "PF" | "PUSHFACE" | "PUSH FACE" => features::push_face::execute(&ctx),
        "TF" | "TRANSFORMFACE" | "TRANSFORM FACE" => features::transform_face::execute(&ctx),
        "DF" | "DELETE FACE" | "DELETEFACE" => features::delete_face::execute(&ctx),
        "RFS" | "REFIT FACES" | "REFITFACES" => features::refit_faces::execute(&ctx),
        "THK" | "THICKEN" => features::thicken::execute(&ctx),
        "H" | "HOLE" => features::hole::execute(&ctx),
        // --- No-kernel-op / long tail ---
        "IMPORT3D" => features::import3d::execute(&ctx),
        "SM.TAB" => features::sheet_metal_tab::execute(&ctx),
        "SM.CF" => features::sheet_metal_contour_flange::execute(&ctx),
        "SM.F" => features::sheet_metal_flange::execute(&ctx),
        "SM.HEM" => features::sheet_metal_hem::execute(&ctx),
        "SM.FILLET" => features::sheet_metal_corner::execute_fillet(&ctx),
        "SM.CHAMFER" => features::sheet_metal_corner::execute_chamfer(&ctx),
        "SM.CUTOUT" => features::sheet_metal_cutout::execute(&ctx),
        "SM.UNFOLD" => features::sheet_metal_unfold::execute(&ctx),
        // --- Assemblies: one placed parts-library instance. Keep these
        //     literals in sync with `parts_library::is_acomp_type` (the
        //     fingerprint-mix + GC hooks key on the same strings). ---
        "ACOMP" | "ASSEMBLY COMPONENT" => features::assembly_component::execute(&ctx),
        // --- Construction geometry that registers named plane frames (headless
        //     sketch-plane resolution) ---
        "D" | "DATUM" | "DATIUM" => features::datum::execute(&ctx),
        "P" | "PLANE" => features::plane::execute(&ctx),
        // --- Sketch: solves + extracts a profile the add-material features consume ---
        "S" | "SKETCH" => features::sketch::execute(&ctx),
        // --- Non-solid construction geometry: editor-drawn pass-throughs ---
        "SP" | "SPLINE" => features::spline::execute(&ctx),
        "WP" | "WAYPOINT" => features::waypoint::execute(&ctx),
        "HX" | "HELIX" => features::helix::execute(&ctx),
        // --- Scoped extensions cannot replace built-ins. ---
        _ if feature_type.contains('/') => extension::execute(&ctx),
        // --- Genuinely unknown type string ---
        _ => FeatureResult::error(id, feature_type.clone(), format!("unknown feature type '{feature_type}'")),
    }
}

// ===========================================================================
// The incremental history cache — dependency-driven dirty tracking
// ===========================================================================
//
// A feature is DIRTY when it itself changed (its serialized descriptor hash — or
// the expression env — differs) or when a NAME it references was (re)produced,
// removed, or re-resolved differently since its cached run. Everything else is
// CLEAN: its cached result (resident handles, profiles, frames, axes, paths)
// replays into the scene untouched and is flagged `reused` so the caller skips display
// rebuild. Dependency edges come from the exact-match name contract itself
// (contract rule 1): any string in a feature's params matching a name produced
// earlier in the history IS a reference.
//
// OWNERSHIP: cache entries own the handles their feature produced. A handle (and
// its sheet-metal tree) is freed exactly when its producing entry is invalidated
// (feature dirty/deleted, or `clear_history_cache`). Nothing else frees scene
// handles — not `SceneMap::apply`, and not the caller (display Solids are non-owning
// views post-flip). Consumed intermediates therefore stay resident until an edit
// invalidates their producer: the accepted memory cost of incremental replay.

/// One cached feature execution.
struct CachedFeature {
    /// A component's resident members were re-posed after feature execution.
    /// Its cached pre-solve component record must not replay against those
    /// mutated handles, even when authored expressions evaluate identically.
    posed_externally: bool,
    /// Hash of `type` + `inputParams` + `persistentData` + the env fingerprint.
    fingerprint: u64,
    /// The successful result (errored results are NEVER cached — an errored
    /// feature is unconditionally dirty next run).
    result: FeatureResult,
    /// The referenced names that RESOLVED against the names available at its
    /// position in the history. Compared for equality each run: a reference that
    /// starts/stops resolving (producer added/deleted/reordered) flips dirty.
    consumed: std::collections::BTreeSet<String>,
    /// Every name this feature produced or removed — the invalidation blast
    /// radius handed to `changed` when the entry dies.
    touched: HashMap<String, ()>,
    /// Content hash of this feature's EFFECTIVE inputs: its own `fingerprint`
    /// PLUS, for every consumed name, the producing feature's `output_version`
    /// (a Merkle hash over the feature DAG). This is what makes reuse correct
    /// ACROSS runs — including after a stop-point edit re-caches an upstream
    /// feature so it looks "clean" on the next full run. The per-run `changed`
    /// wavefront (below) only sees dirtiness WITHIN one run; this sees it across
    /// runs by comparing the input content directly. See `feature_output_version`.
    output_version: u64,
}

thread_local! {
    /// Feature id -> its cached execution, persisted across `execute_history`
    /// calls (same thread = same wasm instance).
    static HISTORY_CACHE: std::cell::RefCell<HashMap<String, CachedFeature>> =
        std::cell::RefCell::new(HashMap::new());
}

/// In-place component motion invalidates the cached producer, but must not
/// free its handles until the next rebuild (the current scene still uses them).
fn mark_component_cache_posed(id: &str) {
    HISTORY_CACHE.with(|cache| {
        if let Some(entry) = cache.borrow_mut().get_mut(id) {
            entry.posed_externally = true;
        }
    });
}

/// Free a dead entry's owned resources: each produced handle and its
/// sheet-metal tree. The SINGLE place cached handles die.
fn free_entry(entry: &CachedFeature) {
    for added in &entry.result.added {
        sheet_metal::remove_tree(added.handle);
        crate::free_registered_solid(added.handle);
    }
}

/// Drop the whole cache, freeing every owned handle + tree. Called by the caller on a
/// part/document switch (and by every Rust test for hermeticity).
pub fn clear_history_cache() {
    HISTORY_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        for entry in cache.values() {
            free_entry(entry);
        }
        cache.clear();
    });
    sheet_metal::clear_trees();
    // The wire-harness tail's bundle solids are owned outside the per-feature
    // entries; they die with the cache too.
    wire_harness::clear_cache();
    // Document-switch semantics: the parts library is per-document state and
    // resets with the cache (the next run's request block re-seeds it).
    parts_library::clear_all();
}

/// wasm boundary for [`clear_history_cache`].
#[wasm_bindgen]
pub fn clear_history_cache_json() {
    clear_history_cache();
}

/// Stable-enough fingerprint of one descriptor, with its expression-bearing
/// string leaves EVALUATED against `env`. `DefaultHasher` is stable within a
/// process, which is exactly the cache's lifetime.
///
/// This is what makes expression-edit invalidation FINE-GRAINED: an edit to the
/// expression sheet (or the configurator) only moves the fingerprint of a feature
/// whose parameter VALUES actually change, instead of the old coarse
/// `env_fingerprint` that hashed the whole sheet into every feature and thus
/// dirtied everything. The env's variable + configurator values reach a feature
/// only through the string params it evaluates, so hashing those evaluated values
/// captures exactly the feature's dependency on the sheet.
fn descriptor_fingerprint(descriptor: &FeatureDescriptor, env: &Env) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    descriptor.feature_type.hash(&mut hasher);
    extension::identity(&descriptor.feature_type).hash(&mut hasher);
    // Both param sources: `persistent_data` (e.g. sketch coordinates) can hold
    // expression strings too, so it must be evaluated or an expression-driven
    // sketch coord would go stale on a sheet edit.
    hash_evaluated(&descriptor.input_params, env, &mut hasher);
    hash_evaluated(&descriptor.persistent_data, env, &mut hasher);
    hasher.finish()
}

/// Feed a descriptor value into the hasher with every string leaf contributing
/// BOTH its raw text AND, if it parses as an expression, its evaluated value.
///
/// - Raw-ALWAYS keeps verbatim-read strings (ids, `reference_selection` names,
///   enum codes) honest: swapping one for a *different* string that happens to
///   evaluate to the same number still moves the hash.
/// - The evaluated value is what lets an UNRELATED sheet edit leave a
///   numeric-expression field untouched (its raw text `"w"` is unchanged and its
///   value only moves when `w` moves).
/// - A non-expression string simply fails to `eval` and contributes only its raw
///   text. A reference name that collides with a variable evaluates too → at
///   worst a harmless FALSE-dirty when that variable is edited (never a stale
///   reuse; the reference's real dependency rides the consumed-set /
///   `output_version`).
///
/// Object keys are visited in sorted order so the hash is representation-stable.
fn hash_evaluated(value: &serde_json::Value, env: &Env, hasher: &mut impl std::hash::Hasher) {
    use std::hash::Hash;
    match value {
        serde_json::Value::String(text) => {
            0u8.hash(hasher);
            text.hash(hasher);
            // Only attempt the (parsing) eval on strings short enough to be a real
            // expression — `persistent_data` can be large, and the raw text above
            // already covers everything for correctness.
            if text.len() <= 64 {
                if let Ok(number) = env.eval(text) {
                    1u8.hash(hasher);
                    number.to_bits().hash(hasher);
                }
            }
        }
        serde_json::Value::Array(items) => {
            2u8.hash(hasher);
            for item in items {
                hash_evaluated(item, env, hasher);
            }
        }
        serde_json::Value::Object(map) => {
            3u8.hash(hasher);
            let mut keys: Vec<&String> = map.keys().collect();
            keys.sort();
            for key in keys {
                key.hash(hasher);
                hash_evaluated(&map[key], env, hasher);
            }
        }
        other => {
            // Number / bool / null — canonical serialized form.
            4u8.hash(hasher);
            other.to_string().hash(hasher);
        }
    }
}

/// Every suffix a stored reference can carry that is an ALIAS for the SKETCH it
/// names rather than a produced name of its own. The kernel run publishes a
/// sketch's profile under the bare sketch id, and each of these spellings
/// resolves back to it:
///
/// - `:PROFILE` — the profile's face-name spelling ([`SceneMap::resolve_profile`]
///   and [`SceneMap::resolve_path`] both strip it).
/// - `:FACE` — the RENDER-side display sheet a committed sketch is drawn as; its
///   one planar face is PICKED as `{sketch}:FACE` (`abi::display`), a name the
///   kernel run NEVER materializes, so `available` can only ever hold the base.
///   `features::common::normalize_profile_alias` strips it at execute time, and
///   real saved models store exactly this spelling for a sketch pick — leaving it
///   un-canonicalised here is what kept a sketch edit from re-running its extrude.
///
/// Field-blind on purpose: a false-dirty from an unrelated string that happens to
/// end this way is harmless, a missed edge is stale geometry.
const SKETCH_REFERENCE_ALIASES: [&str; 2] = [":PROFILE", ":FACE"];

/// Collect every string in the descriptor (inputParams + persistentData,
/// recursively) that EXACT-matches a name available earlier in the history
/// (directly, or with a selection ALIAS suffix — [`SKETCH_REFERENCE_ALIASES`] —
/// stripped) — the feature's resolvable reference set.
fn scan_consumed(
    descriptor: &FeatureDescriptor,
    available: &HashMap<String, ()>,
) -> std::collections::BTreeSet<String> {
    fn walk(
        value: &serde_json::Value,
        available: &HashMap<String, ()>,
        out: &mut std::collections::BTreeSet<String>,
    ) {
        match value {
            serde_json::Value::String(text) => {
                let trimmed = text.trim();
                if trimmed.is_empty() {
                    return;
                }
                // Record the CANONICAL produced name, not the alias spelling:
                // a `{sketch}:PROFILE` / `{sketch}:FACE` reference resolves
                // against the profile published under `{sketch}`, and the dirty
                // check intersects this set with produced/changed names —
                // recording the alias verbatim made a sketch edit invisible to
                // its consumers (changed = {"S1"}, consumed = {"S1:PROFILE"},
                // no overlap). EXACT match wins first, so a real produced name
                // that happens to end in an alias suffix is never rewritten.
                if available.contains_key(trimmed) {
                    out.insert(trimmed.to_string());
                } else if let Some(base) = SKETCH_REFERENCE_ALIASES
                    .iter()
                    .find_map(|alias| trimmed.strip_suffix(alias))
                    .filter(|base| available.contains_key(*base))
                {
                    out.insert(base.to_string());
                }
            }
            serde_json::Value::Array(items) => {
                for item in items {
                    walk(item, available, out);
                }
            }
            serde_json::Value::Object(map) => {
                for item in map.values() {
                    walk(item, available, out);
                }
            }
            _ => {}
        }
    }
    let mut out = std::collections::BTreeSet::new();
    walk(&descriptor.input_params, available, &mut out);
    walk(&descriptor.persistent_data, available, &mut out);
    out
}

/// The EFFECTIVE input hash of one feature: its own descriptor `fingerprint`
/// combined with the `output_version` of every name it consumes. Because each
/// producer's `output_version` is itself computed this way, this is a Merkle hash
/// over the whole upstream DAG: it changes iff the feature's own descriptor OR any
/// transitive input changed. A feature is reusable iff its cached `output_version`
/// still equals this — which is exactly "nothing changed for any of its inputs",
/// and is correct across runs (a stop-point edit that re-cached an upstream feature
/// bumps that feature's version, so this consumer's version no longer matches).
///
/// `consumed` is a `BTreeSet`, so the iteration order is deterministic. A consumed
/// name should always have a version (it came from an already-walked producer); a
/// missing one hashes as `0` rather than panicking.
fn feature_output_version(
    fingerprint: u64,
    consumed: &std::collections::BTreeSet<String>,
    name_version: &HashMap<String, u64>,
) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    fingerprint.hash(&mut hasher);
    for name in consumed {
        name.hash(&mut hasher);
        name_version.get(name).copied().unwrap_or(0).hash(&mut hasher);
    }
    hasher.finish()
}

/// Every name a result adds to the scene: solid/face/edge names, the CONTAINER
/// group aliases, plus the profile/frame/axis/path names from the side-channels.
///
/// This must mirror [`SceneMap::apply`] exactly — every map a name can be
/// RESOLVED from has to be versioned here, or a consumer referencing that name
/// records nothing in its consumed set, its `output_version` never moves, and it
/// replays stale geometry across a real upstream edit.
fn produced_names(result: &FeatureResult) -> Vec<String> {
    let mut names = Vec::new();
    for added in &result.added {
        names.push(added.name.clone());
        names.extend(added.face_names.iter().map(|(_, name)| name.clone()));
        names.extend(added.edge_names.iter().map(|(_, name)| name.clone()));
        // The un-keyed CONTAINER spellings (`{cap_base}_START` and the edge
        // names derived from it) are resolvable scene names — a model saved
        // before per-loop cap naming references them, and `resolve_face_group` /
        // `resolve_edge_group` answer. They are the name a dressup feature
        // actually stores, so they must version like any other produced name.
        names.extend(added.face_groups.iter().map(|(name, _)| name.clone()));
        names.extend(added.edge_groups.iter().map(|(name, _)| name.clone()));
    }
    names.extend(result.profiles.iter().map(|(name, _)| name.clone()));
    names.extend(result.frames.iter().map(|(name, _)| name.clone()));
    names.extend(result.axes.iter().map(|(name, _)| name.clone()));
    names.extend(result.paths.iter().map(|(name, _)| name.clone()));
    names.extend(result.points.iter().map(|(name, _)| name.clone()));
    names.extend(result.ports.iter().map(|(name, _)| name.clone()));
    // A component's id is a produced name: a reference to the component (a
    // COMPONENT-type selection in a descriptor) versions against its producer.
    names.extend(result.components.iter().map(|record| record.id.clone()));
    names
}

// ===========================================================================
// The history loop
// ===========================================================================

/// Run a whole feature history incrementally: build the expression env once,
/// then walk the ordered features. A CLEAN feature (see the cache section above)
/// replays its cached result; a DIRTY one re-executes (its old outputs are freed
/// first). On a feature error, record it and HALT (abort-on-error);
/// errored results are never cached. Never panics.
pub fn execute_history(request: &HistoryRequest) -> HistoryResult {
    execute_history_observed(request, &mut |_| true)
}

/// A feature about to EXECUTE inside [`execute_history_observed`] — a dirty
/// one, not a cached replay (replays are instant and are not reported).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HistoryProgress<'a> {
    /// Zero-based position of the feature in the request.
    pub index: usize,
    /// The request's feature count.
    pub total: usize,
    pub id: &'a str,
    pub feature_type: &'a str,
}

/// [`execute_history`] with a progress observer. `observe` is called
/// immediately BEFORE each feature the run executes (cached replays are not
/// reported); returning `false` stops the run at that boundary — the feature
/// is not executed, nothing after it runs, and the result holds what was
/// built so far. The boundary is the only cooperative stop the kernel offers:
/// nothing inside a feature (a boolean, a blend, a sew) checks anything.
///
/// The cache bookkeeping is untouched by an early stop: the entries of the
/// features that did run are as valid as after a full run, and the entries of
/// the ones that did not were retained as parse-only exactly as under a
/// `stop_before_id`, so the next full run replays the former and executes the
/// latter.
pub fn execute_history_observed(
    request: &HistoryRequest,
    observe: &mut dyn FnMut(HistoryProgress<'_>) -> bool,
) -> HistoryResult {
    let _plugin_scope = extension::enter(&request.plugins);
    let env = Env::build(&request.expressions, &request.configurator)
        .unwrap_or_else(Env::poisoned);

    // Seed the kernel-resident parts library from the request block (document
    // load; resident-wins rules keep a stale block from undoing kernel-side
    // heals/refreshes — see parts_library.rs).
    parts_library::ingest(&request.parts_library);

    // Entries whose feature was DELETED from the history die now. Their
    // consumers go dirty via the consumed-set equality check (the vanished names
    // drop out of the available set), so no changed-seeding is needed here.
    let request_ids: HashMap<String, ()> = request
        .features
        .iter()
        .map(|descriptor| (extract_id(&descriptor.input_params), ()))
        .collect();
    HISTORY_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        cache.retain(|id, entry| {
            let keep = request_ids.contains_key(id);
            if !keep {
                free_entry(entry);
            }
            keep
        });
    });

    let mut scene = SceneMap::default();
    let mut results = Vec::with_capacity(request.features.len());
    // Per-feature wall-clock timing, in run order (`web_time::Instant` — plain
    // `std::time::Instant` traps 'unreachable' on wasm32).
    let mut timings: Vec<(String, f64)> = Vec::with_capacity(request.features.len());
    // Names whose content changed this run (produced/removed by any re-executed
    // or invalidated feature) — the dirty wavefront.
    let mut changed: HashMap<String, ()> = HashMap::new();
    // Names available for reference so far this run (from clean AND dirty
    // features alike).
    let mut available: HashMap<String, ()> = HashMap::new();
    // Each available name -> the `output_version` of the feature that produced it
    // this run (walk order: a boolean that reuses its target name overwrites the
    // producer's version, so consumers depend on the boolean's version). Feeds
    // `feature_output_version` so cross-run reuse is decided by input CONTENT.
    let mut name_version: HashMap<String, u64> = HashMap::new();
    let mut seen_ids: HashMap<String, ()> = HashMap::new();

    // The declared connection points whose placement nothing the walk builds
    // can move, published BEFORE it so a SPLINE in the same document can
    // attach to one and a sketch can sit on its frame (`ports.rs` states the
    // rule and why this is the one deviation from tail-only resolution). Each
    // name carries the CONTENT VERSION of its declaration, so a consumer is
    // invalidated when the point moves.
    for (name, version) in ports::seed_history_run(request, &mut scene, &env) {
        available.insert(name.clone(), ());
        name_version.insert(name, version);
    }

    let total = request.features.len();
    for (index, descriptor) in request.features.iter().enumerate() {
        let id = extract_id(&descriptor.input_params);
        // Editor stop-point: BEFORE this feature — the rest of the request is
        // parse-only (their cache entries were already retained above).
        if request.stop_before_id.as_deref() == Some(id.as_str()) {
            break;
        }
        // An empty or duplicated id cannot key a cache entry: execute plainly,
        // touching no cached state (a duplicate must not free the entry its
        // first occurrence just wrote).
        let cacheable = !id.is_empty() && seen_ids.insert(id.clone(), ()).is_none();
        let fingerprint = descriptor_fingerprint(descriptor, &env);
        // Assemblies: an ACOMP instance's effective input includes its
        // parts-library entry's CONTENT (key/signature/document — not the
        // snapshot, so a self-heal does not dirty it), and a DIRTY entry
        // (refresh_library_entry) forces re-execution outright. A no-op for
        // every other feature type.
        let (fingerprint, library_dirty) =
            parts_library::mix_descriptor_fingerprint(descriptor, fingerprint);
        let consumed = scan_consumed(descriptor, &available);
        // Merkle input hash over this feature + its consumed producers' versions.
        // Computed for EVERY feature (cacheable or not) so its produced names can
        // publish a version for downstream consumers.
        let output_version = feature_output_version(fingerprint, &consumed, &name_version);

        let clean = cacheable
            && !library_dirty
            && !request.force_rebuild.iter().any(|forced| forced == &id)
            && HISTORY_CACHE.with(|cache| {
                cache.borrow().get(&id).is_some_and(|entry| {
                    !entry.posed_externally
                        && entry.fingerprint == fingerprint
                        && entry.consumed == consumed
                        && consumed.iter().all(|name| !changed.contains_key(name))
                        // The cross-run clause: reuse only if the effective input
                        // content is byte-identical to what this entry was built
                        // from. Subsumes the three clauses above (they are hashed
                        // in); kept additively so the diff stays minimal and the
                        // handle-ownership / blast-radius paths are untouched.
                        && entry.output_version == output_version
                })
            });

        if clean {
            let mut result =
                HISTORY_CACHE.with(|cache| cache.borrow().get(&id).unwrap().result.clone());
            result.reused = true;
            scene.apply(&result);
            for name in produced_names(&result) {
                available.insert(name.clone(), ());
                // A clean feature's current version equals its cached one; publish
                // it so downstream consumers hash the SAME value they cached against.
                name_version.insert(name, output_version);
            }
            results.push(result);
            timings.push((id.clone(), 0.0)); // replayed, not re-executed
            // Editor stop-point: AFTER this feature.
            if request.stop_at_id.as_deref() == Some(id.as_str()) {
                break;
            }
            continue;
        }

        // Dirty: the old entry (if any) dies — its blast radius joins `changed`.
        if cacheable {
            if let Some(old) = HISTORY_CACHE.with(|cache| cache.borrow_mut().remove(&id)) {
                free_entry(&old);
                changed.extend(old.touched.iter().map(|(name, _)| (name.clone(), ())));
            }
        }

        // The observer's boundary: the feature is dirty and about to execute.
        if !observe(HistoryProgress {
            index,
            total,
            id: &id,
            feature_type: &descriptor.feature_type,
        }) {
            break;
        }

        // Anything left in the ledger by a DIRECT api call on this thread
        // (`move_faces_json`, an MCP edit) belongs to that call, not to the
        // feature about to run: drop it rather than attribute it wrongly.
        let _ = crate::take_crossing_repairs();
        let _ = crate::take_blend_notes();
        let _ = crate::take_blend_approximations(&[]);
        // The feature is ONE blend operation however many blend calls it
        // makes: none of them drops another's reports before the consume below.
        let blend_operation = crate::blend::BlendOperation::enter();
        let feat_start = web_time::Instant::now();
        let mut result = execute_feature(descriptor, &env, &scene);
        timings.push((id.clone(), feat_start.elapsed().as_secs_f64() * 1000.0));
        // A result the soundness acceptance REPAIRED rather than returned as
        // built says so on the feature that built it. Drained here, so a repair
        // made by any lane reaches the caller's report by one route instead of
        // each lane growing a channel of its own.
        result.notes.extend(
            crate::take_crossing_repairs()
                .into_iter()
                .map(|repair| repair.to_string()),
        );
        // What a blend that shipped says about its own wall (a station ladder
        // that ran out with its rails still off their carriers), by the same
        // route.
        result.notes.extend(crate::take_blend_notes());
        // A fillet wall that shipped short of its construction request says so
        // TYPED, on the body that carries it: ONE snapshot of the ledger read
        // against EVERY body this feature returned. A report whose wall no
        // returned body carries (a lane that refused, a body a later step
        // discarded) is dropped; a refused feature returns no body, so it
        // carries none. Attached before the result is cached, so a replay
        // carries the same typed reports.
        if crate::blend::blend_approximations_pending() {
            let solids: Vec<(String, crate::BrepSolid)> = result
                .added
                .iter()
                .filter_map(|added| crate::registered_solid_clone(added.handle).ok().map(|solid| (added.name.clone(), solid)))
                .collect();
            let bodies: Vec<(&str, &crate::BrepSolid)> = solids.iter().map(|(name, solid)| (name.as_str(), solid)).collect();
            result.approximations.extend(crate::take_blend_approximations(&bodies));
        }
        drop(blend_operation);
        // `halt` reflects the feature's OWN error only — a naming collision flags
        // the feature but does NOT truncate the history (the model still renders).
        let halt = result.error.is_some();
        let collisions_before = scene.name_collisions.len();
        scene.apply(&result);
        // Naming-contract guard: if applying this feature's output left two RESIDENT
        // solids sharing a face/edge name, surface it as the feature's error so the
        // operation trips its own tests + shows an error in the app (the hint points
        // at `common::namespace_copy_names`). Fresh path only — a cached replay
        // carries its collision error, if any, already.
        if result.error.is_none() && scene.name_collisions.len() > collisions_before {
            result.error = Some(name_collision_error(
                &scene.name_collisions[collisions_before..],
            ));
        }

        // Kernel-owned topology metadata: every face/edge this feature produced
        // records its producer (the `_seedSourceFeatureMetadata` step, moved here).
        // Non-overwriting — faces propagated through a boolean keep their
        // original sourceFeatureId, matching the established seed semantics.
        if !id.is_empty() {
            let mut seed = serde_json::Map::new();
            seed.insert("sourceFeatureId".into(), serde_json::Value::String(id.clone()));
            for added in &result.added {
                for (_, face_name) in &added.face_names {
                    scene_metadata::merge_record(face_name, &seed, false);
                }
                for (_, edge_name) in &added.edge_names {
                    scene_metadata::merge_record(edge_name, &seed, false);
                }
            }
        }

        // Enrich every miss with the name that replaced it, when the miss looks
        // like the per-loop naming change (see `annotate_rename`).
        for name in &mut result.unresolved {
            *name = annotate_rename(&scene, name);
        }
        // The same hint on the typed report, in the same pass, so the two
        // lists never say different things about one miss.
        if let Some(fulfilment) = &mut result.fulfilment {
            for rejected in &mut fulfilment.rejected {
                rejected.name = annotate_rename(&scene, &rejected.name);
            }
        }

        let produced = produced_names(&result);
        for name in &produced {
            available.insert(name.clone(), ());
            changed.insert(name.clone(), ());
            // Publish this run's version for the name. Runs for cacheable AND
            // non-cacheable (empty/duplicate id) features alike — a non-cacheable
            // feature has no cache entry but its outputs still version their
            // downstream consumers correctly.
            name_version.insert(name.clone(), output_version);
        }
        for name in &result.removed {
            changed.insert(name.clone(), ());
        }

        if cacheable && !halt {
            let mut touched: HashMap<String, ()> =
                produced.into_iter().map(|name| (name, ())).collect();
            touched.extend(result.removed.iter().map(|name| (name.clone(), ())));
            HISTORY_CACHE.with(|cache| {
                cache.borrow_mut().insert(
                    id.clone(),
                    CachedFeature {
                        posed_externally: false,
                        fingerprint,
                        result: result.clone(),
                        consumed,
                        touched,
                        output_version,
                    },
                );
            });
        }

        results.push(result);
        if halt {
            break;
        }
        // Editor stop-point: AFTER this feature.
        if request.stop_at_id.as_deref() == Some(id.as_str()) {
            break;
        }
    }
    // Orphan GC: a parts-library entry whose last ACOMP instance is gone from
    // the request is dropped now (orphaned payloads never accumulate, the
    // request carries the FULL feature list even under a stop point, so
    // truncation cannot GC live entries).
    parts_library::gc_after_rebuild(request);
    // Assembly tail: solve the request's constraints against the just-built
    // scene and install the post-solve session the exported assembly ABI
    // serves. Unconditional, so a document without an `assembly` block
    // resets any stale session.
    assembly::finish_history_run(request, &mut scene, &env);
    // Wire-harness tail: route the block's connections over the ports and
    // spline segments the run published, and build the bundle solids. Its
    // solids ride an extra result so the display pipeline shows them through
    // the standard solid path. Runs AFTER the assembly solve so the bundles
    // are built against the posed scene.
    // Ports tail: resolve the declared-ports block against the FINISHED scene
    // (a port point may reference any geometry in the part, which is why this
    // runs here and not as a history feature) and publish every point. Before
    // the harness, which routes over what this published.
    let ports = ports::finish_history_run(request, &mut scene, &env);
    if let Some(result) = ports.result {
        results.push(result);
    }
    let harness = wire_harness::finish_history_run(request, &mut scene, &env);
    if let Some(result) = harness.result {
        results.push(result);
    }
    // PMI tail: resolve every view's annotations against the finished scene
    // (bundles and posed components included). Pure reads — never touches
    // the cache.
    let pmi = pmi::finish_history_run(request, &scene, &env);
    HistoryResult {
        results,
        timings,
        wire_harness: harness.report,
        pmi,
        ports: ports.report,
    }
}

// ===========================================================================
// wasm boundary
// ===========================================================================

/// Deserialize a history request, run it, and serialize the per-feature results.
/// The handles in the result are `u32`s the caller tessellates / pulls names
/// from via the existing `*_handle` wasm exports.
#[wasm_bindgen]
pub fn execute_history_json(request_json: &str) -> Result<String, JsValue> {
    let request: HistoryRequest = serde_json::from_str(request_json)
        .map_err(|error| JsValue::from_str(&format!("execute_history_json: bad request: {error}")))?;
    let result = execute_history(&request);
    serde_json::to_string(&result).map_err(|error| JsValue::from_str(&error.to_string()))
}



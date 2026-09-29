//! History → scene: run a whole feature history through the kernel's native
//! `execute_history` (same process, same thread — the solid registry is
//! thread-local) and populate a [`RenderScene`] from the resident handles via
//! the kernel's native display payload accessor. No JSON, no typed-array
//! boundary — the R1 promise.

use crate::scene::{solid_display_from_payload, RenderScene, SolidDisplay};
use brep_kernel::{display_payload_handle_native, execute_history, HistoryRequest};
use std::collections::HashMap;

/// Non-fatal diagnostics from a scene build (mirrors the previous app's run-history
/// reporting: a failed feature halts the remaining features but the solids
/// built so far still display — seeing what a failing history DID build is the
/// point of the artifact).
#[derive(Debug, Default, Clone, serde::Serialize, serde::Deserialize)]
pub struct SceneBuildReport {
    /// Per-feature hard errors, as `"<feature id>: <message>"`.
    pub feature_errors: Vec<String>,
    /// Unresolved reference-selection names, as `"<feature id>: <name>"`.
    pub unresolved: Vec<String>,
    /// What the kernel REPAIRED while building a feature, as
    /// `"<feature id>: <what>"` — today, a self-crossing the soundness
    /// acceptance split and re-trimmed rather than refused. Not an error: the
    /// feature SUCCEEDED, with a result the kernel changed on its way out, and
    /// a change the user cannot see is the one thing a repair must never be.
    /// `#[serde(default)]` keeps reports serialized before this field crossing
    /// the worker boundary.
    #[serde(default)]
    pub feature_notes: Vec<String>,
    /// What a SUCCESSFUL feature did with what it was asked, when not everything
    /// (`brep_kernel::FeatureResult::fulfilment`): `(feature id, fulfilment)` in
    /// run order, only for the features that report one. A partial is a
    /// different answer from the one asked for, so the history tree shows it on
    /// the feature's row and the run report carries it typed. `#[serde(default)]`
    /// for reports serialized before this field.
    #[serde(default)]
    pub feature_fulfilment: Vec<(String, brep_kernel::Fulfilment)>,
    /// The kernel's typed refusal for each feature that failed with one
    /// (`brep_kernel::FeatureResult::refusal`): `(feature id, refusal, refused
    /// step)` in run order — the class, stage and text `feature_errors` flattens
    /// to a line, and the motion step that refused when the feature records one
    /// (`FeatureResult::refused_step`). Features whose refusal is text only do
    /// not appear. Additive: `#[serde(default)]` for reports serialized before
    /// it, and omitted when empty.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub feature_refusals: Vec<(String, brep_kernel::KernelRefusal, Option<String>)>,
    /// Solids whose display payload (tessellation) failed, as
    /// `"<solid name>: <message>"` — the solid is skipped, not fatal.
    pub display_errors: Vec<String>,
    /// Per-feature wall-clock execution time `(feature id, milliseconds)` in run
    /// order, carried through from the kernel's [`brep_kernel::HistoryResult`]
    /// timings — the history tree's "N ms" readout.
    pub feature_timings: Vec<(String, f64)>,
    /// Per-feature output solid names `(feature id, [solid names])` in run order —
    /// the history tree's read-only "Outputs" node.
    pub feature_outputs: Vec<(String, Vec<String>)>,
    /// Named plane FRAMES this run resolved `(frame name, frame)`, in run order —
    /// every DATUM registers three (`{id}:XY|XZ|YZ`), every PLANE one (`{id}`), and
    /// a SKETCH its own plane (`{id}`). Surfaced here so the engine can DISPLAY the
    /// construction datum/plane frames (filtered to the D/P producing features) as
    /// first-class scene citizens — the resolved frames ride
    /// [`brep_kernel::FeatureResult::frames`] straight through, no kernel change.
    pub frames: Vec<(String, brep_kernel::Frame)>,
    /// Solved sketch PROFILES this run produced `(sketch id, profile)`, in run
    /// order — every SKETCH feature publishes one under its own id. Surfaced here
    /// (exactly like [`Self::frames`]) so the engine can display each committed
    /// sketch as a SHEET SOLID (planar face + named boundary edges + corner
    /// vertices) via `sketch_display_payload`, no kernel-contract change.
    pub profiles: Vec<(String, brep_kernel::SketchProfile)>,
    /// Named axis LINES this run produced `(axis name, line)`, in run order — a
    /// SKETCH publishes one per line geometry (construction included). Surfaced
    /// here (exactly like [`Self::frames`]/[`Self::profiles`]) so the engine can
    /// resolve a revolve/sweep `axis` reference to a world line at annotation-build
    /// time, fully headless. `#[serde(default)]` keeps older serialized reports
    /// (pre-`axes`) deserializing cleanly across the worker boundary.
    #[serde(default)]
    pub axes: Vec<(String, brep_kernel::Axis)>,
    /// Named PATH chains this run produced `(path name, curves)`, in run order — a
    /// SKETCH publishes its whole ordered chain under `{id}` and EACH model segment
    /// under `{id}:G{gid}`. Surfaced here (exactly like [`Self::profiles`]) so the
    /// engine can draw a committed sketch's OPEN geometry: an open chain closes no
    /// region, so it publishes no profile, and before this the sheet builder had
    /// nothing to draw it from — an open sketch was invisible in 3D. `#[serde(default)]`
    /// keeps older serialized reports (pre-`paths`) deserializing cleanly across the
    /// worker boundary.
    #[serde(default)]
    pub paths: Vec<(String, Vec<brep_kernel::NurbsCurve>)>,
    /// Named world POINTS this run produced `(point name, point)`, in run order —
    /// a SKETCH publishes every solved point under `{id}:P{pid}` with its
    /// construction flag. Surfaced here (exactly like [`Self::paths`]) so the
    /// engine can draw a committed sketch's STANDALONE points: a points-only
    /// sketch (a hole-placement sketch) has no segment and no profile, so before
    /// this the sheet builder had nothing to draw it from — it was invisible and
    /// unpickable in 3D. `#[serde(default)]` keeps older serialized reports
    /// (pre-`points`) deserializing cleanly across the worker boundary.
    #[serde(default)]
    pub points: Vec<(String, brep_kernel::ScenePoint)>,
    /// The wire-harness tail's routing report (endpoints, segments, one route
    /// per connection, bundles). Read RUNNER-SIDE like everything else here —
    /// the tail ran on the runner's thread — and shipped so the harness panel
    /// reads it off the applied run. `#[serde(default)]` keeps older serialized
    /// reports deserializing across the worker boundary.
    #[serde(default)]
    pub wire_harness: Option<brep_kernel::WireHarnessReport>,
    /// The PMI tail's resolution of every view's annotations (`None` when the
    /// document carries no `pmi` block).
    #[serde(default)]
    pub pmi: Option<brep_kernel::PmiReport>,
    /// The PORTS tail's resolution of the declared-ports block (`None` when the
    /// document declares none). Shipped for the same reason the two above are:
    /// the tail is not a feature, so its refused names and unresolved
    /// references reach no feature row — the Qualify panel reads them here.
    #[serde(default)]
    pub ports: Option<brep_kernel::PortsReport>,
}

/// Execute a serialized `HistoryRequest` (the `execute_history_json` request
/// shape — a saved part file parses as one) and build the display scene from
/// the final resident solids.
pub fn scene_from_history_json(
    request_json: &str,
) -> Result<(RenderScene, SceneBuildReport), String> {
    let request: HistoryRequest = serde_json::from_str(request_json)
        .map_err(|error| format!("history request parse: {error}"))?;
    scene_from_history(&request)
}

/// Typed-request variant of [`scene_from_history_json`].
pub fn scene_from_history(
    request: &HistoryRequest,
) -> Result<(RenderScene, SceneBuildReport), String> {
    let mut scene = RenderScene::new();
    let report = update_scene_from_history(&mut scene, request)?;
    Ok((scene, report))
}

/// The fold of a history run into an ordered scene layout: each entry is the
/// solid's final name, its resident handle, and whether the feature that
/// produced it REPLAYED from the incremental cache (R10 — a reused solid is the
/// same resident geometry, so its display can be kept verbatim and its GPU
/// buffers reused).
struct SceneLayout {
    order: Vec<String>,
    handles: std::collections::HashMap<String, u32>,
    reused: std::collections::HashSet<String>,
    /// `name -> creating-feature id` of the FINAL resident SOLIDS (last writer
    /// wins; a `removed` name drops its entry) — the eager provenance the runner
    /// ships so the main thread never has to re-run the history to answer
    /// "what feature produced this SOLID?". Matches the resident semantics of the
    /// old `resident_handles_and_creators`.
    creators: std::collections::HashMap<String, String>,
    /// `face/edge NAME -> ORIGINATING feature id` — the FIRST feature (timeline
    /// order) to emit each face/edge name, i.e. the entity's TRUE origin (the
    /// feature that gave it its name). Unlike `creators` this is FIRST-writer-wins
    /// and is NEVER pruned on `removed`: a boolean removes its target solid and
    /// re-adds the same solid name carrying mostly the same face names, so pruning
    /// then re-adding would reset those origins to the boolean feature — the same
    /// last-writer bug one level down. Accepted edge case: a fully-deleted solid
    /// whose name later recurs on unrelated geometry keeps its old origin — but
    /// under this app's DETERMINISTIC naming a recurring name is the same
    /// conceptual entity, and the whole map is rebuilt every run, so it can never
    /// point at a feature that was deleted from the history.
    entity_origin: std::collections::HashMap<String, String>,
}

/// Whether `request` is an ASSEMBLY document: any ACOMP-typed feature, or a
/// present `assembly` constraint block. Mirrors
/// `EngineState::history_has_assembly` — a componentless document ships no
/// [`AssemblySync`] and the main side clears its projection, exactly as the
/// main-side sync used to.
fn request_has_assembly(request: &HistoryRequest) -> bool {
    request.assembly.is_some()
        || request.features.iter().any(|feature| {
            feature.feature_type.eq_ignore_ascii_case("ACOMP")
                || feature.feature_type.eq_ignore_ascii_case("ASSEMBLY COMPONENT")
        })
}

fn fold_history(result: &brep_kernel::HistoryResult, report: &mut SceneBuildReport) -> SceneLayout {
    // Removals first (a boolean result reuses a removed target's name), then
    // additions, insertion-ordered — mirrors SceneMap::apply.
    let mut order: Vec<String> = Vec::new();
    let mut handles: std::collections::HashMap<String, u32> = std::collections::HashMap::new();
    let mut reused: std::collections::HashSet<String> = std::collections::HashSet::new();
    let mut creators: std::collections::HashMap<String, String> = std::collections::HashMap::new();
    let mut entity_origin: std::collections::HashMap<String, String> =
        std::collections::HashMap::new();
    for (index, feature) in result.results.iter().enumerate() {
        // A feature whose `inputParams` carry no `id` still has to be nameable in
        // the report, or its errors read as `": <message>"` and point at nothing.
        // Its POSITION is the only handle it has, and the one `feature_delete`
        // takes to remove it again.
        let label = if feature.id.is_empty() { format!("feature[{index}]") } else { feature.id.clone() };
        for removed in &feature.removed {
            if handles.remove(removed).is_some() {
                order.retain(|name| name != removed);
            }
            reused.remove(removed);
            creators.remove(removed);
            // NOTE: `entity_origin` is deliberately NOT pruned here — see its doc on
            // `SceneLayout`. First-writer-with-no-pruning is the whole point.
        }
        for added in &feature.added {
            if handles.insert(added.name.clone(), added.handle).is_none() {
                order.push(added.name.clone());
            }
            creators.insert(added.name.clone(), feature.id.clone());
            // The FIRST feature to emit a given face/edge name IS its origin (this
            // loop is in timeline order). `entry(..).or_insert_with` keeps that
            // first writer — using `insert` here would be last-writer and silently
            // reproduce the exact bug this map exists to fix.
            for (_, name) in &added.face_names {
                entity_origin
                    .entry(name.clone())
                    .or_insert_with(|| feature.id.clone());
            }
            for (_, name) in &added.edge_names {
                entity_origin
                    .entry(name.clone())
                    .or_insert_with(|| feature.id.clone());
            }
            // A solid displays as reused only when its whole producing feature
            // replayed unchanged; a re-run feature re-tessellates.
            if feature.reused {
                reused.insert(added.name.clone());
            } else {
                reused.remove(&added.name);
            }
        }
        if let Some(error) = &feature.error {
            report.feature_errors.push(format!("{label}: {error}"));
        }
        for name in &feature.unresolved {
            report.unresolved.push(format!("{label}: {name}"));
        }
        for note in &feature.notes {
            report.feature_notes.push(format!("{label}: {note}"));
        }
        if let Some(fulfilment) = &feature.fulfilment {
            report
                .feature_fulfilment
                .push((feature.id.clone(), fulfilment.clone()));
        }
        if let Some(refusal) = &feature.refusal {
            report
                .feature_refusals
                .push((feature.id.clone(), refusal.clone(), feature.refused_step.clone()));
        }
        // The feature's output solid name(s) — the history tree's Outputs node.
        report.feature_outputs.push((
            feature.id.clone(),
            feature.added.iter().map(|a| a.name.clone()).collect(),
        ));
        // The named plane frames this feature resolved (DATUM three / PLANE one /
        // SKETCH its own). Carried straight through so the engine can display the
        // construction datum/plane frames (it filters to the D/P producers).
        for (name, frame) in &feature.frames {
            report.frames.push((name.clone(), *frame));
        }
        // The solved sketch profile (SKETCH features publish one under `{id}`) —
        // carried straight through so the engine can synthesize its sheet solid.
        for (name, profile) in &feature.profiles {
            report.profiles.push((name.clone(), profile.clone()));
        }
        // The named axis lines this feature published (a SKETCH emits one per line
        // geometry) — carried through so the engine can resolve a revolve `axis`
        // reference to a world line for the angle gizmo.
        for (name, axis) in &feature.axes {
            report.axes.push((name.clone(), *axis));
        }
        // The named path chains this feature published (a SKETCH emits its whole
        // chain under `{id}` and every model segment under `{id}:G{gid}`) — carried
        // through so the engine can draw the segments no closed profile covers.
        for (name, curves) in &feature.paths {
            report.paths.push((name.clone(), curves.clone()));
        }
        // The named world points this feature published (a SKETCH emits every
        // solved point under `{id}:P{pid}`) — carried through so the engine can
        // draw a sketch's standalone points, which no segment covers.
        for (name, point) in &feature.points {
            report.points.push((name.clone(), *point));
        }
    }
    // Per-feature timing rides the kernel result straight through.
    report.feature_timings = result.timings.clone();
    // The wire-harness routing report rides through the same way; its bundle
    // solids arrived above as the appended `WireHarness` result's `added`.
    report.wire_harness = result.wire_harness.clone();
    report.pmi = result.pmi.clone();
    report.ports = result.ports.clone();
    SceneLayout { order, handles, reused, creators, entity_origin }
}

/// Executes history without accessing the render scene and emits display deltas.
/// Tracks resident handles so unchanged solids retain their meshes and GPU
/// buffers when the main thread applies the resulting [`RunOutput`].
pub struct SceneRunner {
    /// Last emitted handle per solid name. Handles are never recycled, so a
    /// matching handle identifies a cache replay whose display can be reused.
    last_sent: HashMap<String, u32>,
    /// Baseline tessellation LOD. A change clears `last_sent` so even unchanged
    /// handles receive meshes built with the new chord tolerance.
    last_lod: f64,
    /// Revision installed via `Command::SetPartsLibrary`. Runs with a different
    /// revision are refused; `reset` clears this alongside the kernel store.
    pub(crate) parts_library_revision: Option<u64>,
    /// Names in the last library install. A missing installed part triggers a
    /// library reload; a name never installed proceeds to the feature's normal
    /// unresolved-reference error, avoiding an endless reload loop.
    pub(crate) parts_library_names: std::collections::BTreeSet<String>,
}

/// One placed component as the run left it — the engine's own projection of the
/// kernel's `ComponentRecord`, which is a `#[serde(skip)]` side-channel and
/// carries no serde derives of its own. Shipped in the reply so the main thread
/// has the post-solve component set without re-running the history.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ComponentSnapshot {
    /// The owning ACOMP feature id — also the namespace prefix segment.
    pub id: String,
    /// The display / parts-library part name.
    pub part_name: String,
    /// The rigid instance pose (part-local snapshot space -> assembly space)
    /// as the kernel's row-major 4x4; [`Self::affine`] wraps it back up.
    pub transform: [f64; 16],
    /// Grounded flag (the feature's `isFixed`).
    pub fixed: bool,
    /// Opaque source metadata (`sourceKey` / `sourceSignature` / …).
    pub source: serde_json::Value,
    /// Member solid scene names, already namespaced (`{id}:{part solid name}`).
    pub solids: Vec<String>,
    /// The part's harness PORT ids, namespaced (`{id}:{part port id}`).
    pub ports: Vec<String>,
}

impl ComponentSnapshot {
    /// The pose as the kernel's transform type (for `transform_brep` and the
    /// structured STEP writer).
    pub fn affine(&self) -> brep_kernel::AffineTransform {
        brep_kernel::AffineTransform { elements: self.transform }
    }
}

/// Bake an ACOMP `transform` param (`{translate, rotateEulerDeg}`, degrees,
/// intrinsic `XYZ` — `R = Rx·Ry·Rz`) into the kernel's row-major 4x4. The
/// inverse of [`brep_kernel::transform_to_pose_params`], which is the encoder
/// the solver's write-back uses; a component's record pose and its
/// `inputParams.transform` are the same rigid placement in the two spellings,
/// and `component_snapshot_pose_matches_the_kernel_bake` pins this against the
/// kernel's own bake of the same params.
///
/// Needed because the per-feature `components` side-channel is captured when the
/// ACOMP EXECUTES, which is before the constraint tail solves; the solved pose
/// arrives separately as the write-back. Missing/garbage values fall back to the
/// identity placement, exactly like the feature's own reader.
fn pose_params_to_transform(pose: &serde_json::Value) -> [f64; 16] {
    let vec3 = |key: &str| -> [f64; 3] {
        let value = &pose[key];
        std::array::from_fn(|axis| value[axis].as_f64().unwrap_or(0.0))
    };
    let translate = vec3("translate");
    let [rx, ry, rz] = vec3("rotateEulerDeg").map(f64::to_radians);
    let (sx, cx) = rx.sin_cos();
    let (sy, cy) = ry.sin_cos();
    let (sz, cz) = rz.sin_cos();
    let x = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]];
    let y = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]];
    let z = [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]];
    let mul = |a: [[f64; 3]; 3], b: [[f64; 3]; 3]| -> [[f64; 3]; 3] {
        std::array::from_fn(|i| {
            std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum())
        })
    };
    let r = mul(x, mul(y, z));
    [
        r[0][0], r[0][1], r[0][2], translate[0],
        r[1][0], r[1][1], r[1][2], translate[1],
        r[2][0], r[2][1], r[2][2], translate[2],
        0.0, 0.0, 0.0, 1.0,
    ]
}

impl From<&brep_kernel::ComponentRecord> for ComponentSnapshot {
    fn from(record: &brep_kernel::ComponentRecord) -> Self {
        Self {
            id: record.id.clone(),
            part_name: record.part_name.clone(),
            transform: record.transform.elements,
            fixed: record.fixed,
            source: record.source.clone(),
            solids: record.solids.clone(),
            ports: record.ports.clone(),
        }
    }
}

/// Everything the main thread needs from the run's ASSEMBLY tail, read on the
/// runner right after the run and shipped in the reply.
///
/// The kernel's assembly session — the post-solve constraint state, its
/// per-constraint statuses, the DOF summary and the overlay geometry — is
/// thread-local to whoever ran `execute_history`. Before this the main thread
/// obtained them by executing the whole history a SECOND time
/// (`sync_assembly`), which on a document with a wire harness swept every
/// bundle twice per edit. It reads them from here instead; the only main-side
/// replay left is the one a constraint MUTATION needs, because that solve runs
/// against resident geometry on the thread that asks (see
/// `EngineState::ensure_assembly_session`).
///
/// `None` for a componentless document: nothing here is read and the reads cost
/// nothing.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct AssemblySync {
    /// The scene's component set in deterministic id order — the same fold
    /// (`removed` unmaps, `components` inserts) the scene build honours.
    pub components: Vec<ComponentSnapshot>,
    /// The post-solve `assembly` block (`{constraints, idCounter}`), which the
    /// applier folds back into the document — the pose-authority write-back's
    /// state half (the pose/`isFixed` half rides
    /// [`RunOutput::assembly_poses`]/[`RunOutput::assembly_fixed`]).
    pub state: serde_json::Value,
    /// Per-constraint status rows for the constraints panel.
    pub statuses: serde_json::Value,
    /// The solve's DOF / diagnostics summary.
    pub dof: serde_json::Value,
    /// Per-constraint overlay rows (world anchors/directions/status/value).
    pub overlay: serde_json::Value,
    /// The runner's parts-library store, shipped ONLY when the run itself moved
    /// it — the ACOMP self-heal and the orphan GC are library mutations and the
    /// main side has no other way to see them. Omitted (`None`) otherwise: it
    /// carries every embedded part payload, which for an imported STEP assembly
    /// is megabytes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parts_library: Option<brep_kernel::PartsLibraryMap>,
}

/// The delta a [`SceneRunner::run`] produces: the displayed KERNEL solids IN
/// ORDER plus the run's [`SceneBuildReport`]. The applier walks `snapshot` in
/// order, reusing or replacing each entry (see the field docs).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RunOutput {
    /// The displayed KERNEL solids IN ORDER. `display: Some` = freshly tessellated
    /// (new, or the handle changed since last emit); `None` = UNCHANGED — the
    /// applier keeps its existing [`SolidDisplay`] for this name, whose
    /// `source_handle` equals `handle`.
    pub snapshot: Vec<(String /* name */, u32 /* handle */, Option<SolidDisplay>)>,
    pub report: SceneBuildReport,
    /// Eager PROVENANCE: `name -> creating-feature id` for the run's FINAL resident
    /// SOLIDS (same last-writer-wins fold as the display snapshot). Shipped WITH the
    /// run so the main thread can answer SOLID provenance without a cold
    /// `execute_history` — critical once the run lives on a background thread and the
    /// main-side registry is cold.
    pub provenance: Vec<(String, String)>,
    /// Eager ENTITY ORIGIN: `face/edge NAME -> ORIGINATING feature id` (FIRST writer
    /// in timeline order, never pruned — see `SceneLayout::entity_origin`). Shipped
    /// beside `provenance` so `creating_feature` can answer "which feature gave this
    /// face/edge its name?" (the "Edit owning feature" context action + the Info
    /// tab's `creatingFeature`) with no cold re-run. Crosses the background-worker
    /// seam, so the serde round-trip test asserts it survives.
    pub entity_origin: Vec<(String, String)>,
    /// Assembly-solver POSE write-backs from this run's constraint tail:
    /// `(component feature id, {translate, rotateEulerDeg})` — read RUNNER-SIDE
    /// (the kernel assembly session is thread-local to the thread/worker that ran
    /// `execute_history`, so only the runner can see it) and shipped here so the
    /// engine folds them into the owning ACOMP features' `inputParams` (the
    /// pose-authority contract). Empty on a no-motion solve — a satisfied
    /// assembly must not churn feature fingerprints. `#[serde(default)]` keeps
    /// pre-assembly serialized replies deserializing.
    #[serde(default)]
    pub assembly_poses: Vec<(String, serde_json::Value)>,
    /// `isFixed` write-backs riding the same fold (the Fixed-constraint lane
    /// grounds a component); shape mirrors [`Self::assembly_poses`].
    #[serde(default)]
    pub assembly_fixed: Vec<(String, bool)>,
    /// Names of solids the solve re-posed IN PLACE this run (the display seam:
    /// their producing feature may have replayed `reused`, yet their geometry
    /// moved). The runner already defeated the handle-unchanged reuse fast path
    /// for these (their snapshot entries arrive `Some`, freshly tessellated);
    /// shipped for observability/tests. Empty on zero-mate/no-motion runs.
    #[serde(default)]
    pub moved_solids: Vec<String>,
    /// IMPORTED COLOURS: `entity name -> "#RRGGBB"` for every solid/face this run
    /// left a `color` scene-metadata record on (STEP presentation entities, read
    /// by `brep_kernel::io/appearance.rs` and stamped by IMPORT3D).
    ///
    /// Read RUNNER-SIDE for the same reason as `assembly_poses`: the kernel's
    /// scene-metadata store is thread-local to whoever ran `execute_history`, so
    /// a main-thread read under a background runner sees nothing. The applier
    /// folds these into the engine's own [`crate::metadata::MetadataStore`]
    /// WITHOUT overwriting, so the Info window shows an imported colour and a
    /// user's edit of it still wins.
    ///
    /// Filtered to the names this run actually produced, so a colour left in the
    /// (never-cleared) kernel store by a previous document cannot bleed into
    /// this one.
    #[serde(default)]
    pub imported_colors: Vec<(String, String)>,
    /// Every output name some feature of this run CONSUMED — the union of the
    /// per-feature `removed` lists, sorted. The committed-sketch refresh hides a
    /// sketch a downstream extrude/revolve/sheet-metal consumer absorbed, and it
    /// used to learn that by executing the history again on the UI thread; it
    /// reads this instead. NOT pruned the way the display fold prunes: a name
    /// removed and later re-added is still a name this run consumed.
    #[serde(default)]
    pub consumed: Vec<String>,
    /// The run's assembly tail, for a document that has one (see
    /// [`AssemblySync`]).
    #[serde(default)]
    pub assembly: Option<AssemblySync>,
}

impl SceneRunner {
    pub fn new() -> Self {
        Self {
            last_sent: HashMap::new(),
            last_lod: 1.0,
            parts_library_revision: None,
            parts_library_names: std::collections::BTreeSet::new(),
        }
    }

    /// Reset the delta baseline (call on a wholesale document switch so the next
    /// run is a full rebuild — no stale reuse across unrelated models).
    pub fn reset(&mut self) {
        self.last_sent.clear();
        // A reset always accompanies the `clear_history_cache` that empties
        // this side's parts library, so forget what was installed — the next
        // run's stamp will not match and the library is re-sent.
        self.parts_library_revision = None;
        self.parts_library_names.clear();
    }

    /// Execute `request` and fold it into an ordered delta snapshot (does NOT
    /// touch any scene). For each displayed kernel solid in order: if the handle
    /// is unchanged since the last emit, emit `(name, handle, None)` (the applier
    /// keeps its existing display); otherwise (re-)tessellate the resident handle
    /// and emit `(name, handle, Some(display))`. A display-payload error is
    /// recorded in `report.display_errors` and the entry is SKIPPED — and NOT
    /// recorded in the new baseline, so a later successful run re-tessellates it.
    ///
    ///
    /// Colours are NOT applied here. A display arrives colourless and the main
    /// side paints it from the metadata store
    /// ([`EngineState::sync_colors_from_metadata`](crate::engine_state::EngineState::sync_colors_from_metadata)),
    /// which is the only colour authority — a runner may be a background thread
    /// or worker with no view of that store.
    pub fn run(&mut self, request: &HistoryRequest) -> RunOutput {
        self.run_observed(request, &mut |_| true)
    }

    /// [`Self::run`] with a progress observer: `observe` is called before every
    /// feature the kernel actually executes (see
    /// [`brep_kernel::execute_history_observed`]); returning `false` stops the
    /// run at that boundary, and the partial result is folded like any other.
    pub fn run_observed(
        &mut self,
        request: &HistoryRequest,
        observe: &mut dyn FnMut(brep_kernel::HistoryProgress<'_>) -> bool,
    ) -> RunOutput {
        // The parts-library revision ACROSS the run: a run heals a dirty entry
        // and GCs orphans, and those are the only library mutations the main
        // side cannot see for itself. Compared below to decide whether the
        // (potentially megabyte) store has to ride the reply.
        let library_revision_before = brep_kernel::parts_library_revision();
        let result = {
            let mut trace = crate::run_trace::span("runner");
            let result = brep_kernel::execute_history_observed(request, observe);
            if let Some(trace) = trace.as_mut() {
                trace.result(&result);
            }
            result
        };
        let mut report = SceneBuildReport::default();
        let layout = fold_history(&result, &mut report);

        // --- assembly pose-authority read (RUNNER-SIDE) ---- The kernel assembly
        // session (constraint solve tail) is THREAD-LOCAL to whoever ran
        // `execute_history` — i.e. this thread/worker — so the pose and isFixed
        // write-backs must be read HERE and shipped in the RunOutput; a
        // main-thread read under a background runner would see a cold session.
        let pose_updates: serde_json::Value =
            serde_json::from_str(&brep_kernel::assembly_pose_updates_json())
                .unwrap_or(serde_json::Value::Null);
        let assembly_poses: Vec<(String, serde_json::Value)> = pose_updates["poses"]
            .as_object()
            .map(|map| map.iter().map(|(id, pose)| (id.clone(), pose.clone())).collect())
            .unwrap_or_default();
        let assembly_fixed: Vec<(String, bool)> = pose_updates["isFixed"]
            .as_object()
            .map(|map| {
                map.iter()
                    .filter_map(|(id, flag)| flag.as_bool().map(|b| (id.clone(), b)))
                    .collect()
            })
            .unwrap_or_default();
        // The display seam: solids the solve re-posed IN PLACE keep their resident
        // handle, so the handle-unchanged reuse fast path below would wrongly skip
        // re-tessellating them even though their geometry moved. Drop them from
        // the baseline so they re-emit fresh displays. The `movedSolids` key is
        // ABSENT on zero-mate reports — tolerated (empty).
        let dof: serde_json::Value = serde_json::from_str(&brep_kernel::assembly_dof_json())
            .unwrap_or(serde_json::Value::Null);
        let moved_solids: Vec<String> = dof["movedSolids"]
            .as_array()
            .map(|items| {
                items
                    .iter()
                    .filter_map(|item| item["name"].as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default();
        for name in &moved_solids {
            self.last_sent.remove(name);
        }

        // --- the assembly session + consumed names (RUNNER-SIDE) -------------
        // Same thread-local reason as the poses above, and the whole point of
        // `AssemblySync`: read here, applied there, never re-executed.
        let assembly = request_has_assembly(request).then(|| {
            let mut components: std::collections::BTreeMap<String, ComponentSnapshot> =
                std::collections::BTreeMap::new();
            for feature in &result.results {
                for removed in &feature.removed {
                    components.remove(removed);
                }
                for record in &feature.components {
                    components.insert(record.id.clone(), ComponentSnapshot::from(record));
                }
            }
            // A record's pose is captured when its ACOMP EXECUTES, which is
            // before the constraint tail solves — so a solved component's record
            // still carries its authored placement. Re-pose from the same solver
            // write-back the document adopts, or the projection (the structure
            // tree, the BOM, the structured STEP writer, the component vertex
            // ref) would lag the solve by a run.
            for (id, pose) in &assembly_poses {
                if let Some(component) = components.get_mut(id) {
                    component.transform = pose_params_to_transform(pose);
                }
            }
            let parse = |json: String| {
                serde_json::from_str::<serde_json::Value>(&json).unwrap_or(serde_json::Value::Null)
            };
            AssemblySync {
                components: components.into_values().collect(),
                state: parse(brep_kernel::assembly_state_json()),
                statuses: parse(brep_kernel::assembly_statuses_json()),
                dof: dof.clone(),
                overlay: parse(brep_kernel::assembly_overlay_json()),
                parts_library: (brep_kernel::parts_library_revision() != library_revision_before)
                    .then(brep_kernel::parts_library_map),
            }
        });
        // The consumed-name union the committed-sketch refresh needs. Sorted, so
        // the reply is byte-stable for a given run (the worker seam serializes it).
        let consumed: Vec<String> = result
            .results
            .iter()
            .flat_map(|feature| feature.removed.iter().cloned())
            .collect::<std::collections::BTreeSet<String>>()
            .into_iter()
            .collect();

        // Display LOD: a finite, positive factor (garbage from a hand-edited saved
        // file → the "Normal" 1.0, since chord = extent·1.5e-3·lod and a 0/NaN lod
        // would zero the tolerance → runaway refinement). A change since the last
        // run means every resident mesh must re-tessellate at the new chord even
        // though its handle is unchanged, so drop the reuse baseline.
        let lod = if request.display_lod.is_finite() && request.display_lod > 0.0 {
            request.display_lod
        } else {
            1.0
        };
        if lod != self.last_lod {
            self.last_sent.clear();
            self.last_lod = lod;
        }

        let mut snapshot: Vec<(String, u32, Option<SolidDisplay>)> =
            Vec::with_capacity(layout.order.len());
        let mut next_sent: HashMap<String, u32> = HashMap::with_capacity(layout.order.len());

        for name in &layout.order {
            let handle = layout.handles[name];
            if self.last_sent.get(name) == Some(&handle) {
                // Handle unchanged since last emit ⇒ same resident geometry ⇒ the
                // applier keeps its existing display. This is EXACTLY the old
                // `reused && source_handle == handle` fast path (monotonic handles):
                // the canary below verifies the implication holds.
                debug_assert!(
                    layout.reused.contains(name),
                    "handle unchanged must imply reused (monotonic handles): {name}"
                );
                snapshot.push((name.clone(), handle, None));
                next_sent.insert(name.clone(), handle);
                continue;
            }
            match display_payload_handle_native(handle, lod) {
                Ok(payload) => {
                    let mut solid = solid_display_from_payload(name, payload);
                    solid.source_handle = handle;
                    // Runner thread → the SheetTree thread-local is warm here; stamp
                    // the sheet-metal marker so the UI thread reads it off the scene
                    // (never calling the thread-local from a cold UI thread).
                    solid.is_sheet_metal = brep_kernel::is_sheet_metal_handle(handle);
                    snapshot.push((name.clone(), handle, Some(solid)));
                    next_sent.insert(name.clone(), handle);
                }
                // A tessellation failure skips the solid (non-fatal) and does NOT
                // poison the baseline — dropping it lets a later successful run
                // re-tessellate from scratch.
                Err(error) => report.display_errors.push(format!("{name}: {error}")),
            }
        }

        self.last_sent = next_sent;
        // Eager provenance for the run's final resident solids (the applier stores
        // this map main-side so per-frame provenance queries never re-run history).
        let provenance: Vec<(String, String)> = layout
            .creators
            .iter()
            .map(|(name, id)| (name.clone(), id.clone()))
            .collect();
        // Eager entity origin (face/edge NAME -> originating feature id) rides
        // alongside, so face/edge provenance survives the off-thread seam too.
        let entity_origin: Vec<(String, String)> = layout
            .entity_origin
            .iter()
            .map(|(name, id)| (name.clone(), id.clone()))
            .collect();
        // Imported colours, read HERE for the thread-local reason above and
        // filtered to this run's own entities: the solids it displays plus the
        // faces/edges it named. The kernel store is never cleared between
        // documents, so an unfiltered ship could hand the applier a colour that
        // belongs to a model the user closed.
        let imported_colors: Vec<(String, String)> = serde_json::from_str::<serde_json::Value>(
            &brep_kernel::scene_metadata_colors_json(),
        )
        .ok()
        .and_then(|value| value.as_object().cloned())
        .map(|map| {
            map.into_iter()
                .filter(|(name, _)| {
                    layout.handles.contains_key(name) || layout.entity_origin.contains_key(name)
                })
                .filter_map(|(name, hex)| hex.as_str().map(|hex| (name, hex.to_string())))
                .collect()
        })
        .unwrap_or_default();

        RunOutput {
            snapshot,
            report,
            provenance,
            entity_origin,
            assembly_poses,
            assembly_fixed,
            moved_solids,
            imported_colors,
            consumed,
            assembly,
        }
    }

    /// The resident handle last EMITTED for `name` (the delta baseline), or `None`
    /// if the runner has not emitted a solid of that name. The measurement-query
    /// path resolves an object's owning-solid handle through this so the query runs
    /// against the SAME resident geometry the last run displayed (on the runner's
    /// own thread, whose registry is warm from that run).
    pub fn handle_of(&self, name: &str) -> Option<u32> {
        self.last_sent.get(name).copied()
    }
}

impl Default for SceneRunner {
    fn default() -> Self {
        Self::new()
    }
}

/// Execute the history and reconcile `scene` in place (R10 incremental update):
/// reused solids already present keep their [`SolidDisplay`] verbatim (stable
/// `revision` ⇒ the renderer reuses their GPU buffers); everything else is
/// (re-)tessellated from the resident handle; departed solids are dropped.
///
/// Implemented on the M1 seam: a throwaway [`SceneRunner`] primed from the
/// CURRENT scene (`name → source_handle` of what is displayed) reproduces the
/// pre-seam reuse in a single synchronous call, and its [`RunOutput`] delta is
/// applied back to `scene` by MOVING existing displays out (via
/// [`RenderScene::drain`]) so a reused solid's mesh is never cloned.
pub fn update_scene_from_history(
    scene: &mut RenderScene,
    request: &HistoryRequest,
) -> Result<SceneBuildReport, String> {
    let mut runner = SceneRunner::new();
    runner.last_sent = scene
        .solids()
        .iter()
        .map(|solid| (solid.name.clone(), solid.source_handle))
        .collect();
    let output = runner.run(request);

    // Move the current displays out, then reinsert in snapshot ORDER: a fresh
    // entry replaces, an UNCHANGED entry reuses the moved-out display (whose
    // `source_handle` equals the run's handle — guaranteed by the reuse
    // invariant). Leftovers (departed names) are dropped.
    let mut kept: HashMap<String, SolidDisplay> = scene
        .drain()
        .into_iter()
        .map(|solid| (solid.name.clone(), solid))
        .collect();
    for (name, _handle, maybe) in output.snapshot {
        match maybe {
            Some(display) => scene.insert_solid(display),
            None => scene.insert_solid(
                kept.remove(&name).expect("keep target present"),
            ),
        }
    }
    Ok(output.report)
}

/// Execute `request` and return the FINAL resident solids as `(name, handle)` in
/// display order — the export lane (STEP/STL) needs the current solids' resident
/// handles, which the scene build does not itself retain. Run right after a scene
/// build (warm incremental cache) this replays the cached features, so the
/// handles it returns are the very ones the displayed scene was built from; run
/// cold it re-executes and registers fresh (still-valid) handles. Either way the
/// handles are live in the thread-local registry when this returns, ready to
/// hand to `brep_kernel::export_step_handles`.
pub fn resident_solid_handles(request: &HistoryRequest) -> Vec<(String, u32)> {
    let _trace = crate::run_trace::span("export_resident_handles");
    let result = execute_history(request);
    let mut report = SceneBuildReport::default();
    let layout = fold_history(&result, &mut report);
    layout
        .order
        .iter()
        .map(|name| (name.clone(), layout.handles[name]))
        .collect()
}


//! GENERATION from the CAM pane: the jobs the engine's runner takes
//! ([`brep_render::runner::CamJob`]), what each operation's result means
//! against the document as it stands now, and the toolpath preview the
//! viewport draws from the results.
//!
//! A job is one operation with everything the generator reads, by value:
//! the operation, its setup, the document's tools, and the resident bodies
//! by `(name, handle)` out of the scene. The INPUTS HASH travels with it:
//! the operation's, the setup's and the tool's serialized form and the
//! bodies' content revisions, so a result is STALE the moment any of them
//! moves, without comparing geometry. Toolpaths are derived and stay in the
//! engine's result cache; the document never holds one.

use crate::document::cam::CamDocument;
use brep_cam::cutter::{Cutter, ToolPart, ToolProfile, DEFAULT_CHORD};
use brep_cam::model::{MotionKind, Point3, RigidTransform, Tool};
use brep_render::engine_state::EngineState;
use brep_render::runner::CamJob;
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::hash::{Hash, Hasher};

/// Work units per progress / cancel check inside a job.
pub const BUDGET_CHUNK: u64 = 2048;

/// What the pane knows about one operation's generation.
#[derive(Clone, Debug, PartialEq)]
pub enum OpStatus {
    /// No result and not queued.
    Idle,
    Queued,
    Running,
    /// A result computed from the inputs as they stand.
    Ok { motions: usize, cutting_length: f64 },
    /// A result whose inputs have moved since.
    Stale { motions: usize },
    Error(String),
}

impl OpStatus {
    /// The one-character mark the tree row leads with.
    pub fn mark(&self) -> &'static str {
        match self {
            OpStatus::Idle => "\u{25cb}",
            OpStatus::Queued => "\u{25d4}",
            OpStatus::Running => "\u{25d0}",
            OpStatus::Ok { .. } => "\u{25cf}",
            OpStatus::Stale { .. } => "\u{25d1}",
            OpStatus::Error(_) => "\u{2716}",
        }
    }

    pub fn word(&self) -> &'static str {
        match self {
            OpStatus::Idle => "idle",
            OpStatus::Queued => "queued",
            OpStatus::Running => "running",
            OpStatus::Ok { .. } => "ok",
            OpStatus::Stale { .. } => "stale",
            OpStatus::Error(_) => "error",
        }
    }
}

/// The finest chord a job's part mesh may be tessellated at, in model units:
/// below it a request is refused rather than tessellated (a tolerance that
/// small is a typing error or a part the mesh path cannot carry).
pub const MIN_CHORD: f64 = 1e-4;

/// The chordal tolerance the job's part mesh is tessellated at: a quarter of
/// the operation's own machining tolerance, so the mesh the generator's
/// gouge and collision bounds run against is finer than the tolerance it
/// certifies (a 0.05 mm operation gets 0.0125 mm, whatever the part's size).
/// Scallop alone gets half: it certifies the drive mesh's own sag against
/// half the tolerance and refuses a coarser mesh by name, so its certificate
/// covers that chord. No strategy is ever looser than half.
/// Below [`MIN_CHORD`] it is refused by name — never silently coarsened.
pub fn job_chord(op_id: &str, op: &brep_cam::model::Operation) -> Result<f64, String> {
    let operation_tolerance = op.tolerance;
    let share = if matches!(op.strategy, brep_cam::model::Strategy::Scallop(_)) { 0.5 } else { 0.25 };
    let chord = operation_tolerance * share;
    if !chord.is_finite() || chord < MIN_CHORD {
        return Err(format!(
            "{op_id}: a machining tolerance of {operation_tolerance} needs the part mesh at a chord of {chord}, finer than {MIN_CHORD}; raise the operation's tolerance"
        ));
    }
    Ok(chord)
}

/// The hash of everything a job's result depends on.
pub fn inputs_hash(state: &EngineState, doc: &CamDocument, op_id: &str) -> Option<u64> {
    let op = doc.operation(op_id)?;
    let setup = doc.setup(op.setup.as_str())?;
    let tool = doc.tool(op.tool.as_str());
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    serde_json::to_string(op).ok()?.hash(&mut hasher);
    serde_json::to_string(&doc.autoprogram_inputs.get(op_id)).ok()?.hash(&mut hasher);
    serde_json::to_string(setup).ok()?.hash(&mut hasher);
    serde_json::to_string(&tool).ok()?.hash(&mut hasher);
    // The setup's machine as resolved (a custom machine's limits can change
    // without the setup's id changing).
    serde_json::to_string(&doc.setup_machine(op.setup.as_str())).ok()?.hash(&mut hasher);
    if let brep_cam::model::Strategy::MultiAxisFinish(_) = &op.strategy {
        crate::panels::cam_multiaxis::hash_source(state, doc, op_id, &mut hasher);
    }
    if let brep_cam::model::Strategy::RestFinish(p) = &op.strategy {
        serde_json::to_string(&doc.tool(p.reference_tool.as_str())).ok()?.hash(&mut hasher);
    }
    // A chain's sketch geometry is part of its input: editing the sketch makes
    // the result stale.
    match state.cam_chain_geometry(&op.targets.chains) {
        Ok((curves, profiles)) => {
            serde_json::to_string(&curves).ok()?.hash(&mut hasher);
            serde_json::to_string(&profiles).ok()?.hash(&mut hasher);
        }
        Err(why) => why.hash(&mut hasher),
    }
    for name in body_names(doc, op_id) {
        name.hash(&mut hasher);
        if let Some(solid) = state.scene.solid(&name) {
            solid.revision.hash(&mut hasher);
        } else {
            0u64.hash(&mut hasher);
        }
    }
    // The stock this operation starts from (lane C7): every earlier operation
    // of its setup and of the setups it continues from, as defined and as last
    // generated (their stored result hashes, so this never recurses).
    let (previous, _) = setup_chain(doc, setup);
    let upstream = previous.iter().map(|s| (s.id.as_str(), None)).chain(std::iter::once((setup.id.as_str(), Some(op_id))));
    for (setup_id, before) in upstream {
        for earlier in doc.operations_of(setup_id).take_while(|o| Some(o.id.as_str()) != before) {
            serde_json::to_string(earlier).ok()?.hash(&mut hasher);
            serde_json::to_string(&doc.tool(earlier.tool.as_str())).ok()?.hash(&mut hasher);
            match state.cam.results.get(earlier.id.as_str()) {
                Some(result) => (result.inputs_hash, result.error.is_some(), result.toolpath.is_some()).hash(&mut hasher),
                None => 0u8.hash(&mut hasher),
            }
        }
    }
    Some(hasher.finish())
}

/// The setups `setup` continues from (`Stock::FromRest`), oldest first, and
/// whether the chain is broken (a missing setup or a cycle). The runner names
/// a broken chain on the operation; nothing is guessed here.
fn setup_chain<'a>(doc: &'a CamDocument, setup: &'a brep_cam::model::Setup) -> (Vec<&'a brep_cam::model::Setup>, bool) {
    let mut chain: Vec<&brep_cam::model::Setup> = Vec::new();
    let mut current = setup;
    while let brep_cam::model::Stock::FromRest { setup: previous } = &current.stock {
        match doc.setup(previous.as_str()) {
            Some(p) if p.id != setup.id && !chain.iter().any(|c| c.id == p.id) => {
                chain.push(p);
                current = p;
            }
            _ => {
                chain.reverse();
                return (chain, true);
            }
        }
    }
    chain.reverse();
    (chain, false)
}

/// The setups `setup` continues from (`Stock::FromRest`), oldest first, each
/// with its operations and current toolpaths: what generation and the
/// simulator replay into the stock `setup` starts from.
pub(crate) fn setup_history(state: &EngineState, doc: &CamDocument, setup: &brep_cam::model::Setup) -> Vec<brep_cam::SetupHistory> {
    setup_chain(doc, setup)
        .0
        .into_iter()
        .map(|p| brep_cam::SetupHistory {
            setup: p.clone(),
            machine: doc.setup_machine(p.id.as_str()),
            operations: planned_operations(state, doc, p.id.as_str(), None),
        })
        .collect()
}

/// `setup_id`'s operations in program order up to (not including) `before`
/// (all of them when `None`), each with its toolpath when its result is
/// current; a stale, failed or missing result travels as `None` and the
/// runner names it.
fn planned_operations(state: &EngineState, doc: &CamDocument, setup_id: &str, before: Option<&str>) -> Vec<(brep_cam::model::Operation, Option<brep_cam::model::Toolpath>)> {
    doc.operations_of(setup_id)
        .take_while(|o| Some(o.id.as_str()) != before)
        .map(|o| {
            let toolpath = state
                .cam
                .results
                .get(o.id.as_str())
                .filter(|r| r.error.is_none() && inputs_hash(state, doc, o.id.as_str()) == Some(r.inputs_hash))
                .and_then(|r| r.toolpath.clone());
            (o.clone(), toolpath)
        })
        .collect()
}

/// The bodies a job carries: the setup's targets and protected bodies and
/// the operation's own, de-duplicated and sorted.
fn body_names(doc: &CamDocument, op_id: &str) -> Vec<String> {
    let mut names = BTreeSet::new();
    if let Some(binding) = doc.autoprogram_inputs.get(op_id) { names.extend(binding.body_fingerprints.keys().cloned()); }
    if let Some(op) = doc.operation(op_id) {
        names.extend(op.targets.target_solids.iter().cloned());
        names.extend(op.targets.protected_solids.iter().cloned());
        // An edge chain's bodies ride along so the runner can read the edges.
        for chain in &op.targets.chains {
            if let brep_cam::model::ChainSource::Edges { edges } = &chain.source {
                names.extend(edges.iter().map(|e| e.solid.clone()));
            }
        }
        if let Some(setup) = doc.setup(op.setup.as_str()) {
            names.extend(setup.targets.iter().map(|t| t.solid.clone()));
            names.extend(setup.protected.iter().map(|t| t.solid.clone()));
        }
    }
    names.into_iter().filter(|n| !n.is_empty()).collect()
}

/// The job for operation `op_id`, or why there is none (a missing setup or
/// tool, a body that is not on screen, no target).
pub fn job_for(state: &EngineState, doc: &CamDocument, op_id: &str) -> Result<(CamJob, u64), String> {
    let op = doc.operation(op_id).ok_or_else(|| format!("no operation `{op_id}`"))?;
    if !op.enabled {
        return Err(format!("{op_id} is disabled"));
    }
    let setup = doc.setup(op.setup.as_str()).ok_or_else(|| format!("{op_id}: its setup `{}` is gone", op.setup))?;
    if doc.tool(op.tool.as_str()).is_none() {
        return Err(format!("{op_id}: its tool `{}` is gone", op.tool));
    }
    if op.targets.target_solids.is_empty() && op.targets.chains.is_empty() {
        return Err(format!("{op_id}: no body to machine (pick its targets or a chain)"));
    }
    if let Some(edge) = op.targets.chains.iter().find_map(|c| match &c.source {
        brep_cam::model::ChainSource::Edges { edges } => edges.iter().find(|e| e.solid.is_empty()),
        _ => None,
    }) {
        return Err(format!("{op_id}: chain edge `{}` belongs to no body on screen", edge.edge));
    }
    if matches!(op.strategy, brep_cam::Strategy::TurnFace(_) | brep_cam::Strategy::TurnProfile(_)) {
        let tool = doc.tool(op.tool.as_str()).unwrap();
        if tool.turning_geometry.is_none() { return Err(format!("tool {} has no insert geometry", tool.id)); }
        if setup.turning_stick_out.is_none() { return Err("setup has no stick-out".into()); }
    }
    let (chain_curves, chain_profiles) = state.cam_chain_geometry(&op.targets.chains).map_err(|e| format!("{op_id}: {e}"))?;
    let mut solids = Vec::new();
    for name in body_names(doc, op_id) {
        let solid = state.scene.solid(&name).ok_or_else(|| format!("{op_id}: body `{name}` is not on screen"))?;
        solids.push((name, solid.source_handle));
    }
    let chord_tolerance = job_chord(op_id, op)?;
    let hash = inputs_hash(state, doc, op_id).ok_or_else(|| format!("{op_id}: its inputs cannot be hashed"))?;
    let machine = match &setup.machine {
        Some(id) => Some(doc.machine(id.as_str()).ok_or_else(|| format!("{op_id}: its setup's machine `{id}` is not defined"))?),
        None => None,
    };
    // Lane C7: what the runner replays into the stock this operation starts
    // from — the setup's earlier operations and the setups it continues from.
    let earlier = planned_operations(state, doc, setup.id.as_str(), Some(op_id));
    let previous_setups = setup_history(state, doc, setup);
    // A multi-axis finish carries its designated three-axis source (lane C17).
    let (source, source_operation) = crate::panels::cam_multiaxis::job_source(state, doc, op)?;
    Ok((
        CamJob {
            id: 0,
            operation: op.clone(),
            setup: setup.clone(),
            tools: doc.tools.clone(),
            solids,
            chord_tolerance,
            budget_chunk: BUDGET_CHUNK,
            chain_curves,
            chain_profiles,
            machine,
            autoprogram: doc.autoprogram_inputs.get(op_id).cloned(),
            earlier,
            previous_setups,
            source,
            source_operation,
        },
        hash,
    ))
}

/// The job for `op_id` built from the document as it stands NOW: the engine
/// calls it as each queued job is submitted, so an operation carries the
/// results of the operations generated before it in the same queue (lane C7
/// pre-operation stock) and any edit made while it waited.
pub fn job_at_dispatch(state: &EngineState, op_id: &str) -> Result<(CamJob, u64), String> {
    let doc = crate::panels::cam::cam_document(state)?;
    job_for(state, &doc, op_id)
}

/// The operation's status against the document as it stands.
pub fn status_of(state: &EngineState, doc: &CamDocument, op_id: &str) -> OpStatus {
    if state.cam.in_flight() == Some(op_id) {
        return OpStatus::Running;
    }
    if state.cam.queued().iter().any(|q| q == op_id) {
        return OpStatus::Queued;
    }
    let Some(result) = state.cam.results.get(op_id) else { return OpStatus::Idle };
    if let Some(error) = &result.error {
        return OpStatus::Error(error.clone());
    }
    // A path whose own report says it is not usable (the generator kept it to
    // show where it failed) is an error, not ok: assembly, the player and the
    // post all refuse it.
    if let Some(report) = result.toolpath.as_ref().map(|t| &t.report).filter(|r| r.status == brep_cam::model::OperationStatus::Error) {
        let why: Vec<String> = report.errors.iter().map(|e| format!("{}: {}", e.code, e.message)).collect();
        return OpStatus::Error(if why.is_empty() { "the generated path is not usable".to_string() } else { why.join("; ") });
    }
    let motions = result.toolpath.as_ref().map(|t| t.motions.len()).unwrap_or(0);
    let fresh = inputs_hash(state, doc, op_id) == Some(result.inputs_hash);
    if fresh {
        OpStatus::Ok { motions, cutting_length: result.toolpath.as_ref().map(|t| t.cutting_length()).unwrap_or(0.0) }
    } else {
        OpStatus::Stale { motions }
    }
}

/// Preview colours by motion kind (lines only; the model is never occluded).
const RAPID: [f32; 3] = [0.93, 0.33, 0.30];
const PLUNGE: [f32; 3] = [0.96, 0.62, 0.20];
const CUT: [f32; 3] = [0.30, 0.70, 0.95];
const LINK: [f32; 3] = [0.62, 0.65, 0.70];
const LEAD: [f32; 3] = [0.45, 0.85, 0.55];

fn dim(c: [f32; 3]) -> [f32; 3] {
    [c[0] * 0.45, c[1] * 0.45, c[2] * 0.45]
}

/// The arc's chord points: `steps` straight segments between its ends.
fn arc_points(from: Point3, to: Point3, center: Point3, cw: bool, steps: usize) -> Vec<Point3> {
    let a0 = (from.y - center.y).atan2(from.x - center.x);
    let a1 = (to.y - center.y).atan2(to.x - center.x);
    let mut sweep = a1 - a0;
    if cw && sweep > 0.0 {
        sweep -= std::f64::consts::TAU;
    }
    if !cw && sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    if sweep.abs() < 1e-9 && (from.x - to.x).abs() + (from.y - to.y).abs() > 1e-9 {
        return vec![to];
    }
    let r0 = ((from.x - center.x).powi(2) + (from.y - center.y).powi(2)).sqrt();
    let r1 = ((to.x - center.x).powi(2) + (to.y - center.y).powi(2)).sqrt();
    (1..=steps)
        .map(|i| {
            let t = i as f64 / steps as f64;
            let a = a0 + sweep * t;
            let r = r0 + (r1 - r0) * t;
            Point3::new(center.x + r * a.cos(), center.y + r * a.sin(), from.z + (to.z - from.z) * t)
        })
        .collect()
}

/// The world polyline of a toolpath with the motion kind of each segment,
/// arcs chorded; a segment per point that moves the tool.
pub fn segments(toolpath: &brep_cam::model::Toolpath) -> Vec<(Point3, Point3, &'static str)> {
    let mut out = Vec::new();
    let mut at: Option<Point3> = None;
    for motion in &toolpath.motions {
        let (kind, points): (&'static str, Vec<Point3>) = match &motion.kind {
            MotionKind::Rapid { to } | MotionKind::Retract { to } => ("rapid", vec![*to]),
            MotionKind::Plunge { to } => ("plunge", vec![*to]),
            MotionKind::Cut { to } => ("cut", vec![*to]),
            MotionKind::LeadIn { to } | MotionKind::LeadOut { to } => ("lead", vec![*to]),
            MotionKind::Link { to } => ("link", vec![*to]),
            MotionKind::ArcCw { to, center, .. } => ("cut", at.map(|from| arc_points(from, *to, *center, true, 12)).unwrap_or_else(|| vec![*to])),
            MotionKind::ArcCcw { to, center, .. } => ("cut", at.map(|from| arc_points(from, *to, *center, false, 12)).unwrap_or_else(|| vec![*to])),
            MotionKind::Cycle { at: hole, .. } => ("plunge", vec![*hole]),
            _ => continue,
        };
        for p in points {
            if let Some(from) = at {
                out.push((from, p, kind));
            }
            at = Some(p);
        }
    }
    out
}

fn colour(kind: &str) -> [f32; 3] {
    match kind {
        "rapid" => RAPID,
        "plunge" => PLUNGE,
        "lead" => LEAD,
        "link" => LINK,
        _ => CUT,
    }
}

/// The tool's wireframe for the preview: the cutter's profile (and its
/// holder's) as circles at every profile height with four generatrices, the
/// cutting part, the shank and the holder each in their own colour.
pub fn tool_profile(tool: &Tool) -> Option<ToolProfile> {
    let cutter = Cutter::from_tool(tool).ok()?;
    Some(ToolProfile::new(&cutter, tool.holder.as_ref(), DEFAULT_CHORD))
}

/// The overlay group of one result: every segment in its kind's colour, the
/// selected operation at full strength and the others dimmed; the segments
/// are the toolpath's machine coordinates taken back to the model through
/// `to_model` (the setup's machine-to-model transform). `scrub`, a fraction
/// of the segment count, marks the tool position at that point and, with
/// `tool`, draws the tool's wireframe standing on it along the machine's
/// +Z. `None` toolpath = an empty group (removes the name).
pub fn toolpath_group(
    op_id: &str,
    toolpath: Option<&brep_cam::model::Toolpath>,
    selected: bool,
    scrub: Option<f64>,
    to_model: &RigidTransform,
    tool: Option<&ToolProfile>,
) -> Value {
    let name = format!("cam-toolpath:{op_id}");
    let Some(toolpath) = toolpath else { return json!({ "name": name }) };
    let segments = segments(toolpath);
    let mut positions: Vec<f64> = Vec::with_capacity(segments.len() * 6);
    let mut colors: Vec<f32> = Vec::with_capacity(segments.len() * 6);
    for (a, b, kind) in &segments {
        let (a, b) = (to_model.apply_point(*a), to_model.apply_point(*b));
        positions.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
        let c = if selected { colour(kind) } else { dim(colour(kind)) };
        colors.extend_from_slice(&c);
        colors.extend_from_slice(&c);
    }
    let mut points: Vec<f64> = Vec::new();
    let mut point_colors: Vec<f32> = Vec::new();
    if let (Some(fraction), true) = (scrub, selected && !segments.is_empty()) {
        let index = ((fraction.clamp(0.0, 1.0) * segments.len() as f64).floor() as usize).min(segments.len() - 1);
        let (_, tip, _) = segments[index];
        let p = to_model.apply_point(tip);
        points.extend_from_slice(&[p.x, p.y, p.z]);
        point_colors.extend_from_slice(&[1.0, 0.95, 0.2]);
        if let Some(profile) = tool {
            tool_wireframe(profile, tip, to_model, &mut positions, &mut colors);
        }
    }
    let mut group = json!({ "name": name, "renderOrder": 7, "lines": { "positions": positions, "colors": colors } });
    if !points.is_empty() {
        group["points"] = json!({ "positions": points, "colors": point_colors, "size": 10.0 });
    }
    group
}

/// The tool's wireframe standing with its tip at `tip` (machine
/// coordinates), appended to `positions` / `colors` in model coordinates.
fn tool_wireframe(profile: &ToolProfile, tip: Point3, to_model: &RigidTransform, positions: &mut Vec<f64>, colors: &mut Vec<f32>) {
    const SIDES: usize = 24;
    const CUTTING: [f32; 3] = [0.95, 0.90, 0.35];
    const SHANK: [f32; 3] = [0.75, 0.75, 0.78];
    const HOLDER: [f32; 3] = [0.55, 0.58, 0.62];
    let at = |radius: f64, height: f64, angle: f64| -> Point3 {
        to_model.apply_point(Point3::new(tip.x + radius * angle.cos(), tip.y + radius * angle.sin(), tip.z + height))
    };
    let part_colour = |part: ToolPart| match part {
        ToolPart::Holder => HOLDER,
        ToolPart::Shank => SHANK,
        _ => CUTTING,
    };
    let mut line = |a: Point3, b: Point3, c: [f32; 3]| {
        positions.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
        colors.extend_from_slice(&c);
        colors.extend_from_slice(&c);
    };
    for point in &profile.points {
        if point.radius <= 0.0 {
            continue;
        }
        let c = part_colour(point.part);
        for i in 0..SIDES {
            let a0 = i as f64 / SIDES as f64 * std::f64::consts::TAU;
            let a1 = (i + 1) as f64 / SIDES as f64 * std::f64::consts::TAU;
            line(at(point.radius, point.height, a0), at(point.radius, point.height, a1), c);
        }
    }
    for w in profile.points.windows(2) {
        let c = part_colour(w[1].part);
        for k in 0..4 {
            let angle = k as f64 * std::f64::consts::FRAC_PI_2;
            line(at(w[0].radius, w[0].height, angle), at(w[1].radius, w[1].height, angle), c);
        }
    }
}

/// The scrub position's point (machine coordinates) and the segment index it falls on.
pub fn scrub_point(toolpath: &brep_cam::model::Toolpath, fraction: f64) -> Option<(Point3, usize)> {
    let segments = segments(toolpath);
    if segments.is_empty() {
        return None;
    }
    let index = ((fraction.clamp(0.0, 1.0) * segments.len() as f64).floor() as usize).min(segments.len() - 1);
    Some((segments[index].1, index))
}

/// The scrub position of operation `op_id` in MODEL coordinates (through its
/// setup's machine-to-model transform) and the segment index.
pub fn scrub_world(state: &EngineState, doc: &CamDocument, op_id: &str, fraction: f64) -> Option<(Point3, usize)> {
    let result = state.cam.results.get(op_id)?;
    let toolpath = result.toolpath.as_ref()?;
    let (p, index) = scrub_point(toolpath, fraction)?;
    let to_model = doc.operation(op_id).and_then(|op| doc.setup(op.setup.as_str())).map(|s| s.machine_to_model()).unwrap_or_default();
    Some((to_model.apply_point(p), index))
}

/// A result's motions as JSON (the `cam_toolpath` read).
pub fn toolpath_json(state: &EngineState, op_id: &str) -> Option<Value> {
    let result = state.cam.results.get(op_id)?;
    let toolpath = result.toolpath.as_ref()?;
    let motions = serde_json::to_value(&toolpath.motions).unwrap_or(Value::Null);
    Some(json!({
        "operation": toolpath.operation,
        "tool": toolpath.tool,
        "motions": motions,
        "report": toolpath.report,
        "cuttingLength": toolpath.cutting_length(),
        "segments": segments(toolpath).len(),
        "cutBounds": cut_bounds(&motions),
    }))
}

/// The box around every `cut` motion's end point (`{min, max}`, machine
/// coordinates), or null without one: where the operation removes material,
/// for a reader to check against a boundary.
fn cut_bounds(motions: &Value) -> Value {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for m in motions.as_array().into_iter().flatten().filter(|m| m.get("kind").and_then(Value::as_str) == Some("cut")) {
        let Some(to) = m.get("to").and_then(Value::as_array) else { continue };
        let p: Vec<f64> = to.iter().filter_map(Value::as_f64).collect();
        if p.len() != 3 {
            continue;
        }
        any = true;
        for axis in 0..3 {
            min[axis] = min[axis].min(p[axis]);
            max[axis] = max[axis].max(p[axis]);
        }
    }
    if any { json!({ "min": min, "max": max }) } else { Value::Null }
}


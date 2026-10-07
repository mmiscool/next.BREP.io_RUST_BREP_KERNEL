use super::*;
use crate::camera::Aabb;

// ===========================================================================
// Interference check — pairwise NON-DESTRUCTIVE kernel INTERSECT booleans
// over the component instances, reporting every overlapping pair + its
// intersection volume. Zero interference is a PASS.
//
// Runs MAIN-SIDE like every other assembly op (the thread-local rule): the
// component solids' kernel handles come from [`EngineState::
// resident_solid_handles`] — a warm-cache replay of the rolled-to history on
// THIS thread — and `brep_kernel::boolean_handle` / `mass_properties_handle_
// native` read the SAME thread-local registry. The operand handles belong to
// the incremental cache + scene; ONLY the boolean RESULT handle is freed.
//
// Scale sanity: N components = N·(N−1)/2 pairs, but a full boolean only runs
// for pairs whose (inflated, mesh-derived) bboxes overlap — [`plan_pairs`]
// prefilters, so disjoint pairs are PROVEN clear for the cost of a box test.
// Bbox-overlapping pairs beyond [`MAX_BOOLEAN_PAIRS`] are SKIPPED with an
// explicit per-pair note (never silently). Hidden components participate —
// interference is a physical question — with the pair flagged so the window
// can note it.
// ===========================================================================

/// Budget of component pairs allowed to run REAL booleans per check run. Full
/// booleans cost ~10–100 ms each; the bbox prefilter keeps normal assemblies
/// far below this. Pairs beyond the budget are reported as skipped.
const MAX_BOOLEAN_PAIRS: usize = 64;

/// Interference counts only above this intersection volume (mm³): exact face
/// contact (mated components) integrates to ~0 and must read as clear.
const VOLUME_EPSILON: f64 = 1e-6;

/// One INTERFERING component pair: the two owning ACOMP feature ids, their
/// summed intersection volume (mm³, over all member-solid cross pairs), and
/// whether either participant is currently hidden (it still participates —
/// the window notes it).
#[derive(Debug, Clone, PartialEq)]
pub struct InterferencePair {
    pub a: String,
    pub b: String,
    pub volume: f64,
    pub a_hidden: bool,
    pub b_hidden: bool,
}

/// The result of one interference run — everything the results window shows.
/// `pairs` empty + `skipped`/`unverified` empty = the green all-clear.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct InterferenceReport {
    /// Number of component instances that participated.
    pub component_count: usize,
    /// Every pair considered: N·(N−1)/2.
    pub pair_total: usize,
    /// Pairs that ran the boolean lane (bbox-overlapping, within budget).
    /// The rest were PROVEN clear by the bbox prefilter (or noted below).
    pub booleans_run: usize,
    /// The interfering pairs, largest intersection volume first.
    pub pairs: Vec<InterferencePair>,
    /// Anything NOT fully checked, one human-readable line each: pairs beyond
    /// the boolean budget, components with no resident geometry. NEVER silent.
    pub skipped: Vec<String>,
    /// Pairs whose boolean REFUSED (the kernel is conservative on grazing /
    /// tangent contact): not a pass, not an interference — shown under their
    /// own heading so a mated assembly never reads as a wall of errors.
    pub unverified: Vec<String>,
}

// --- the test seam: count trips through the boolean lane --------------------


/// Tally one boolean-lane invocation (test-observable; free in release).
fn note_boolean_call() {
}



// --- the boolean lane -------------------------------------------------------

/// Intersection volume (mm³) of two RESIDENT solids, non-destructively:
/// `boolean_handle_native` reads both operands by reference (pipeline-default
/// options), the result registers under a NEW handle whose exact volume is
/// integrated, and that intermediate is freed. A legitimately-disjoint
/// intersect yields an EMPTY solid → volume 0. The OPERAND handles are owned
/// by the incremental cache + scene — never free them here.
fn intersect_volume(a: u32, b: u32) -> Result<f64, String> {
    note_boolean_call();
    let result = brep_kernel::boolean_handle_native(
        a,
        b,
        brep_kernel::BooleanOperation::Intersect,
        &brep_kernel::BooleanOptions::default(),
    )?;
    let volume = brep_kernel::mass_properties_handle_native(result, 1.0).map(|p| p.volume);
    // Free ONLY the result — the intermediate this check minted.
    brep_kernel::free_solid(result);
    volume
}

// --- the pure pair planner (prefilter + budget) ------------------------------

/// One component as the planner sees it: id, the union bbox of its member
/// solids (EMPTY = no resident geometry), and the hidden flag.
pub(crate) struct PlanComponent {
    pub id: String,
    pub bbox: Aabb,
    pub hidden: bool,
}

/// What [`plan_pairs`] decides: which index pairs go to the boolean lane, and
/// the explicit notes for everything that will NOT be boolean-checked.
pub(crate) struct PairPlan {
    pub boolean_pairs: Vec<(usize, usize)>,
    pub skipped: Vec<String>,
    pub pair_total: usize,
}

/// Grow a mesh-derived bbox conservatively before the overlap test: the scene
/// bbox is over TESSELLATED positions, which under-approximate curved BREP by
/// up to the chord deviation — a false positive costs one boolean that comes
/// back empty; a false negative would silently miss real interference.
fn inflated(bbox: &Aabb) -> Aabb {
    if bbox.is_empty() {
        return *bbox;
    }
    let size = bbox.size();
    let diagonal = (size[0] * size[0] + size[1] * size[1] + size[2] * size[2]).sqrt();
    let margin = (diagonal * 0.01).max(1e-6);
    let mut out = *bbox;
    for axis in 0..3 {
        out.min[axis] -= margin;
        out.max[axis] += margin;
    }
    out
}

/// Axis-aligned overlap (empty boxes never overlap).
fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    (0..3).all(|axis| a.min[axis] <= b.max[axis] && b.min[axis] <= a.max[axis])
}

/// The PURE planning pass: which pairs must pay for a boolean. Bbox-disjoint
/// pairs are PROVEN clear (checked, not skipped); pairs past `cap` and
/// geometry-less components get explicit notes. Deterministic id order.
pub(crate) fn plan_pairs(components: &[PlanComponent], cap: usize) -> PairPlan {
    let mut plan = PairPlan {
        boolean_pairs: Vec::new(),
        skipped: Vec::new(),
        pair_total: components.len().saturating_sub(1) * components.len() / 2,
    };
    for component in components {
        if component.bbox.is_empty() {
            plan.skipped
                .push(format!("{} — no resident geometry, not checked", component.id));
        }
    }
    let boxes: Vec<Aabb> = components.iter().map(|c| inflated(&c.bbox)).collect();
    for i in 0..components.len() {
        for j in (i + 1)..components.len() {
            if !overlaps(&boxes[i], &boxes[j]) {
                continue; // proven clear by the prefilter — no boolean needed
            }
            if plan.boolean_pairs.len() >= cap {
                plan.skipped.push(format!(
                    "{} × {} — skipped (boolean budget of {cap} pairs reached)",
                    components[i].id, components[j].id
                ));
                continue;
            }
            plan.boolean_pairs.push((i, j));
        }
    }
    plan
}

// --- the engine surface ------------------------------------------------------

impl EngineState {
    /// Run the interference check over every component instance (hidden ones
    /// included — interference is a physical question). Non-destructive: the
    /// component solids are only READ; each pairwise INTERSECT result is
    /// measured and freed. Returns the full report for the results window.
    pub fn interference_check(&mut self) -> InterferenceReport {

        // Gather the participants: members + union bbox + hidden, in the
        // deterministic history order component_ids() gives.
        let ids = self.component_ids();
        let mut members: Vec<Vec<String>> = Vec::with_capacity(ids.len());
        let mut plan_input: Vec<PlanComponent> = Vec::with_capacity(ids.len());
        for id in &ids {
            let info = self.component_info(id);
            let solids = info.map(|info| info.members).unwrap_or_default();
            let mut bbox = Aabb::empty();
            let mut hidden = false;
            for name in &solids {
                if let Some(solid) = self.scene.solid(name) {
                    bbox.union(&solid.bbox);
                    hidden |= !solid.visible;
                }
            }
            plan_input.push(PlanComponent {
                id: id.clone(),
                bbox,
                hidden,
            });
            members.push(solids);
        }

        let plan = plan_pairs(&plan_input, MAX_BOOLEAN_PAIRS);
        let mut report = InterferenceReport {
            component_count: ids.len(),
            pair_total: plan.pair_total,
            booleans_run: 0,
            pairs: Vec::new(),
            skipped: plan.skipped,
            unverified: Vec::new(),
        };
        if plan.boolean_pairs.is_empty() {
            return report;
        }

        // The warm MAIN-SIDE handle map (cache-hit replay on this thread) —
        // the same lane the Info windows' mass properties ride.
        let handles = self.resident_solid_handles();
        for (i, j) in plan.boolean_pairs {
            report.booleans_run += 1;
            let mut volume = 0.0;
            let mut refusal: Option<String> = None;
            for solid_a in &members[i] {
                for solid_b in &members[j] {
                    let (Some(&ha), Some(&hb)) = (handles.get(solid_a), handles.get(solid_b))
                    else {
                        refusal.get_or_insert_with(|| format!("missing resident operand: {solid_a} or {solid_b}"));
                        continue;
                    };
                    // Member-level prefilter: within an overlapping component
                    // pair, only member solids whose own boxes overlap pay.
                    let (Some(a), Some(b)) =
                        (self.scene.solid(solid_a), self.scene.solid(solid_b))
                    else {
                        refusal.get_or_insert_with(|| format!("missing display operand: {solid_a} or {solid_b}"));
                        continue;
                    };
                    if !overlaps(&inflated(&a.bbox), &inflated(&b.bbox)) {
                        continue;
                    }
                    match intersect_volume(ha, hb) {
                        Ok(v) => volume += v,
                        Err(error) => {
                            // A conservative kernel refusal (grazing/tangent
                            // contact): the pair is UNVERIFIED, never a
                            // silent pass and never fake interference.
                            refusal.get_or_insert(error);
                        }
                    }
                }
            }
            let (a, b) = (&plan_input[i], &plan_input[j]);
            if let Some(error) = refusal {
                report
                    .unverified
                    .push(format!("{} × {} — boolean refused: {error}", a.id, b.id));
            }
            if volume > VOLUME_EPSILON {
                report.pairs.push(InterferencePair {
                    a: a.id.clone(),
                    b: b.id.clone(),
                    volume,
                    a_hidden: a.hidden,
                    b_hidden: b.hidden,
                });
            }
        }
        // Largest interference first (stable → id order breaks ties).
        report
            .pairs
            .sort_by(|x, y| y.volume.total_cmp(&x.volume));
        report
    }
}


/// A positive-weight rational surface lies inside its Euclidean control hull.
/// Unlike tessellation bounds this cannot miss a curved extremum. Refuse the
/// shortcut when any weight is nonpositive or geometry is unavailable.
fn conservative_bounds(solid: &brep_kernel::BrepSolid) -> Option<Aabb> {
    let mut bounds = Aabb::empty();
    let mut count = 0;
    for face in solid.shells.iter().flat_map(|s| &s.faces) {
        if face.surface.control_points.is_empty() { return None; }
        for cp in face.surface.control_points.iter().flatten() {
            if cp.w <= 0.0 { return None; }
            let p = [cp.x / cp.w, cp.y / cp.w, cp.z / cp.w];
            if !p.iter().all(|x| x.is_finite()) { return None; }
            for axis in 0..3 { bounds.min[axis] = bounds.min[axis].min(p[axis] - 1e-6); bounds.max[axis] = bounds.max[axis].max(p[axis] + 1e-6); }
            count += 1;
        }
    }
    (count > 0).then_some(bounds)
}

/// Positive evidence only: a topological vertex on the other solid's boundary
/// within 1e-6 mm. No witness is not evidence of clearance.
fn contact_witness(a: &brep_kernel::BrepSolid, b: &brep_kernel::BrepSolid) -> Result<bool, String> {
    for (vertices, target) in [(&a.vertices, b), (&b.vertices, a)] {
        for v in vertices {
            if brep_kernel::classify_point(v.point, target, 1e-6)?.class == brep_kernel::PointClass::On { return Ok(true); }
        }
    }
    Ok(false)
}

/// Continuations are server-owned, scoped to one engine and one applied model.
/// Keep only a bounded number; expired tokens fail explicitly.
#[derive(Clone)]
pub(super) struct InterferenceCursor {
    revision: u64,
    document: String,
    scope: Vec<String>,
    next: usize,
    unresolved: usize,
    interfering: usize,
}

impl EngineState {
    /// A pair budget bounds progress even for assemblies whose boxes are disjoint.
    /// Zero-volume intersections do not establish contact: without an exact
    /// distance witness they remain unresolved (never an all-clear).
    pub fn interference_page(&mut self, scope: Option<Vec<String>>, budget: usize, token: Option<String>) -> Result<serde_json::Value, String> {
        use serde_json::json;
        if self.run_pending() { return Err("model rebuild is pending; wait before checking interference".into()); }
        if !(1..=4096).contains(&budget) { return Err("budget must be in 1..=4096 component pairs".into()); }
        let revision = self.applied_generation();
        let document = self.history.request_json();
        let mut cursor = if let Some(token) = &token {
            let c = self.interference_sessions.get(token).ok_or("unknown or expired continuation token")?.clone();
            if c.revision != revision || c.document != document { return Err("stale continuation: model changed; restart without a token".into()); }
            if scope.as_ref().is_some_and(|s| s != &c.scope) { return Err("continuation scope differs from requested components".into()); }
            c
        } else {
            let known = self.component_ids();
            let ids = scope.unwrap_or_else(|| known.clone());
            let mut seen = std::collections::HashSet::new();
            for id in &ids {
                if !known.contains(id) { return Err(format!("unknown component `{id}`")); }
                if !seen.insert(id) { return Err(format!("duplicate component `{id}`")); }
            }
            InterferenceCursor { revision, document, scope: ids, next: 0, unresolved: 0, interfering: 0 }
        };
        let total = cursor.scope.len() * cursor.scope.len().saturating_sub(1) / 2;
        let start = cursor.next;
        let end = start.saturating_add(budget).min(total);
        let handles = self.resident_solid_handles();
        let mut outcomes = Vec::new();
        let mut booleans_run = 0usize;
        let mut boolean_operations = 0usize;
        let mut pair_index = 0;
        for i in 0..cursor.scope.len() {
            for j in i+1..cursor.scope.len() {
                let index = pair_index;
                pair_index += 1;
                if index < start || index >= end { continue; }
                let a = &cursor.scope[i];
                let b = &cursor.scope[j];
                let ma = self.component_info(a).map(|c| c.members).unwrap_or_default();
                let mb = self.component_info(b).map(|c| c.members).unwrap_or_default();
                let mut volume = 0.0;
                let mut errors = Vec::new();
                let mut ambiguous = false;
                let mut touching = false;
                let mut ran_boolean = false;
                if ma.is_empty() || mb.is_empty() { errors.push(json!({"operation":"resolve_operands", "category":"missing_geometry", "operands":[a,b]})); }
                for sa in &ma {
                    for sb in &mb {
                        let (Some(ha), Some(hb), Some(da), Some(db)) = (handles.get(sa), handles.get(sb), self.scene.solid(sa), self.scene.solid(sb)) else {
                            errors.push(json!({"operation":"resolve_operands", "category":"missing_geometry", "operands":[sa,sb]}));
                            continue;
                        };
                        let _ = (da, db); // Display presence is coverage, not geometric evidence.
                        let exact_a = brep_kernel::registered_solid_clone(*ha);
                        let exact_b = brep_kernel::registered_solid_clone(*hb);
                        if let (Ok(a), Ok(b)) = (&exact_a, &exact_b) {
                            if let (Some(ba), Some(bb)) = (conservative_bounds(a), conservative_bounds(b)) {
                                if !overlaps(&ba, &bb) { continue; }
                            }
                        }
                        ran_boolean = true;
                        boolean_operations += 1;
                        match intersect_volume(*ha, *hb) {
                            Ok(v) if v.is_finite() && v >= 0.0 => {
                                volume += v;
                                if v <= VOLUME_EPSILON {
                                    match (&exact_a, &exact_b) {
                                        (Ok(a), Ok(b)) => match contact_witness(a, b) {
                                            Ok(true) => touching = true,
                                            Ok(false) => ambiguous = true,
                                            Err(e) => errors.push(json!({"operation":"contact_verification", "category":"kernel_refusal", "operands":[sa,sb], "detail":e})),
                                        },
                                        _ => ambiguous = true,
                                    }
                                }
                            },
                            Ok(_) => errors.push(json!({"operation":"intersection_volume", "category":"numerical", "operands":[sa,sb]})),
                            Err(e) => errors.push(json!({"operation":"intersect", "category":"kernel_refusal", "operands":[sa,sb], "detail":e})),
                        }
                    }
                }
                for (n, diagnostic) in errors.iter_mut().enumerate() {
                    if let Some(detail) = diagnostic.get("detail").and_then(serde_json::Value::as_str).map(str::to_owned) {
                        let id = format!("interference:{revision}:{a}:{b}:{n}");
                        self.geometry_diagnostics.insert(id.clone(), diagnostic.clone());
                        diagnostic.as_object_mut().unwrap().remove("detail");
                        diagnostic["message"] = json!(detail.chars().take(240).collect::<String>());
                        diagnostic["detailId"] = json!(id);
                    }
                }
                if ran_boolean { booleans_run += 1; }
                let status = if !errors.is_empty() { "unverified" }
                    else if volume > VOLUME_EPSILON { "verified_interference" }
                    else if ambiguous { "unverified" } else if touching { "verified_touching" } else { "verified_clearance" };
                if status == "unverified" { cursor.unresolved += 1; }
                if volume > VOLUME_EPSILON { cursor.interfering += 1; }
                outcomes.push(json!({"index":index, "a":a, "b":b, "status":status, "volume":volume,
                    "aHidden":ma.iter().any(|n| self.scene.solid(n).is_some_and(|s| !s.visible)),
                    "bHidden":mb.iter().any(|n| self.scene.solid(n).is_some_and(|s| !s.visible)),
                    "diagnostics":errors, "reason":if ambiguous && errors.is_empty() { Some("zero_volume_does_not_distinguish_contact_from_clearance") } else { None }}));
            }
        }
        cursor.next = end;
        let complete = end == total;
        let next_token = if complete { None } else {
            self.interference_sequence += 1;
            Some(format!("interference-{revision}-{}", self.interference_sequence))
        };
        if let Some(old) = token { self.interference_sessions.remove(&old); }
        if let Some(next) = &next_token {
            if self.interference_sessions.len() >= 16 {
                let oldest = self.interference_sessions.keys().min_by_key(|key| key.rsplit('-').next().and_then(|s| s.parse::<u64>().ok()).unwrap_or(0)).cloned();
                if let Some(oldest) = oldest { self.interference_sessions.remove(&oldest); }
            }
            self.interference_sessions.insert(next.clone(), cursor.clone());
        }
        let mut interfering: Vec<&serde_json::Value> = outcomes.iter().filter(|p| p["volume"].as_f64().is_some_and(|v| v > VOLUME_EPSILON)).collect();
        interfering.sort_by(|a,b| b["volume"].as_f64().unwrap().total_cmp(&a["volume"].as_f64().unwrap()));
        let skipped: Vec<String> = if complete { vec![] } else { vec![format!("{} component pairs not checked: pair budget exhausted", total-end)] };
        let unverified: Vec<String> = outcomes.iter().filter(|p| p["status"] == "unverified").map(|p| format!("{} × {}: unverified (see outcomes)",p["a"].as_str().unwrap_or(""),p["b"].as_str().unwrap_or(""))).collect();
        Ok(json!({"modelRevision":revision, "scope":cursor.scope, "componentCount":cursor.scope.len(),
            "budget":budget, "pairTotal":total, "checkedPairs":end, "pageStart":start, "pairs":interfering, "outcomes":outcomes, "skipped":skipped, "unverified":unverified, "booleansRun":booleans_run, "booleanOperations":boolean_operations,
            "notChecked": {"count":total-end, "fromPairIndex":end, "status":"not_checked_budget_exhausted", "reason":if complete { serde_json::Value::Null } else { json!("budget_exhausted") }},
            "unresolvedCount":cursor.unresolved, "interferenceCount":cursor.interfering,
            "complete":complete, "allClear":complete && cursor.unresolved == 0 && cursor.interfering == 0,
            "continuationToken":next_token, "volumeToleranceMm3":VOLUME_EPSILON,
            "contactToleranceMm":1e-6, "contactVerification":"boundary vertex witness within tolerance after successful intersection; unwitnessed zero-volume pairs remain unverified"}))
    }
}


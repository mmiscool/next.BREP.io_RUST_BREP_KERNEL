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
                        continue; // not resident (rolled back mid-frame)
                    };
                    // Member-level prefilter: within an overlapping component
                    // pair, only member solids whose own boxes overlap pay.
                    let (Some(a), Some(b)) =
                        (self.scene.solid(solid_a), self.scene.solid(solid_b))
                    else {
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


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
// Scale: N components = N·(N−1)/2 pairs, but a full boolean only runs for
// pairs whose (inflated, mesh-derived) bboxes overlap — [`plan_pairs`]
// prefilters, so disjoint pairs are PROVEN clear for the cost of a box test.
// There is no pair budget: every box-overlapping pair is checked. Hidden
// components participate — interference is a physical question — with the
// pair flagged so the window can note it.
//
// Clearance (near miss): with a threshold > 0, pairs whose boxes come within
// it get a minimum distance over the DISPLAY tessellation (triangle–triangle,
// so approximate by the chord deviation) and are reported when the gap is at
// or under the threshold. The interference volumes and the near-miss gaps are
// drawn in the viewport overlay group [`INTERFERENCE_OVERLAY_GROUP`].
// ===========================================================================

/// Interference counts only above this intersection volume (mm³): exact face
/// contact (mated components) integrates to ~0 and must read as clear.
const VOLUME_EPSILON: f64 = 1e-6;

/// The viewport overlay group carrying the interference volumes (red
/// triangles) and the near-miss gaps (amber segment + end points).
pub const INTERFERENCE_OVERLAY_GROUP: &str = "assembly-interference";

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

/// One NEAR-MISS component pair: no interference, but the gap between them
/// is at or under the clearance threshold. `distance` (mm) is measured over
/// the display tessellation; `witness` is the closest point on `a` then `b`.
#[derive(Debug, Clone, PartialEq)]
pub struct ClearancePair {
    pub a: String,
    pub b: String,
    pub distance: f64,
    pub witness: [[f64; 3]; 2],
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
    /// Pairs that ran the boolean lane (bbox-overlapping). The rest were
    /// PROVEN clear by the bbox prefilter (or noted below).
    pub booleans_run: usize,
    /// The interfering pairs, largest intersection volume first.
    pub pairs: Vec<InterferencePair>,
    /// The clearance threshold this run used (mm); 0 = no near-miss check.
    pub clearance: f64,
    /// The near-miss pairs (gap ≤ `clearance`), smallest gap first.
    pub near_misses: Vec<ClearancePair>,
    /// Anything NOT fully checked, one human-readable line each (components
    /// with no resident geometry). NEVER silent.
    pub skipped: Vec<String>,
    /// Pairs whose boolean REFUSED (the kernel is conservative on grazing /
    /// tangent contact): not a pass, not an interference — shown under their
    /// own heading so a mated assembly never reads as a wall of errors.
    pub unverified: Vec<String>,
    /// The model generation the run measured ([`EngineState::
    /// applied_generation`]); a later rebuild makes the report stale.
    pub generation: u64,
    /// The interference volumes as a world-space triangle soup (3 points per
    /// triangle) — what the overlay draws.
    pub volume_triangles: Vec<[f32; 3]>,
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
/// intersect yields an EMPTY solid → volume 0. With `mesh_lod`, an
/// interfering result is tessellated (world-space triangle soup) before it is
/// freed. The OPERAND handles are owned by the incremental cache + scene —
/// never free them here.
fn intersect_volume(a: u32, b: u32, mesh_lod: Option<f64>) -> Result<(f64, Vec<[f32; 3]>), String> {
    note_boolean_call();
    let result = brep_kernel::boolean_handle_native(
        a,
        b,
        brep_kernel::BooleanOperation::Intersect,
        &brep_kernel::BooleanOptions::default(),
    )?;
    let volume = brep_kernel::mass_properties_handle_native(result, 1.0).map(|p| p.volume);
    let mut triangles = Vec::new();
    if let (Ok(v), Some(lod)) = (&volume, mesh_lod) {
        if *v > VOLUME_EPSILON {
            // A display-only extra: a tessellation failure leaves the volume
            // reported, just not drawn.
            if let Ok(payload) = brep_kernel::display_payload_handle_native(result, lod) {
                let mesh = crate::scene::solid_display_from_payload("interference", payload).mesh;
                triangles.extend(mesh.indices.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()));
            }
        }
    }
    // Free ONLY the result — the intermediate this check minted.
    brep_kernel::free_solid(result);
    volume.map(|v| (v, triangles))
}

// --- the pure pair planner (prefilter) ---------------------------------------

/// One component as the planner sees it: id, the union bbox of its member
/// solids (EMPTY = no resident geometry), and the hidden flag.
pub(crate) struct PlanComponent {
    pub id: String,
    pub bbox: Aabb,
    pub hidden: bool,
}

/// What [`plan_pairs`] decides: which index pairs need a closer look, and the
/// explicit notes for everything that will NOT be checked.
pub(crate) struct PairPlan {
    pub candidate_pairs: Vec<(usize, usize)>,
    pub skipped: Vec<String>,
    pub pair_total: usize,
}

/// Grow a mesh-derived bbox conservatively before the overlap test: the scene
/// bbox is over TESSELLATED positions, which under-approximate curved BREP by
/// up to the chord deviation — a false positive costs one boolean that comes
/// back empty; a false negative would silently miss real interference. `extra`
/// (the clearance threshold) widens it further for the near-miss check.
fn inflated(bbox: &Aabb, extra: f64) -> Aabb {
    if bbox.is_empty() {
        return *bbox;
    }
    let size = bbox.size();
    let diagonal = (size[0] * size[0] + size[1] * size[1] + size[2] * size[2]).sqrt();
    let margin = (diagonal * 0.01).max(1e-6) + extra.max(0.0);
    let mut out = *bbox;
    for axis in 0..3 {
        out.min[axis] -= margin;
        out.max[axis] += margin;
    }
    out
}

/// Axis-aligned overlap (empty boxes never overlap).
pub(crate) fn overlaps(a: &Aabb, b: &Aabb) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    (0..3).all(|axis| a.min[axis] <= b.max[axis] && b.min[axis] <= a.max[axis])
}

/// The PURE planning pass: which pairs need a boolean or a distance. Pairs
/// whose boxes (inflated, plus `clearance`) are disjoint are PROVEN clear
/// (checked, not skipped); geometry-less components get explicit notes.
/// Deterministic id order. Sweep-and-prune on x keeps large assemblies cheap.
pub(crate) fn plan_pairs(components: &[PlanComponent], clearance: f64) -> PairPlan {
    let mut plan = PairPlan {
        candidate_pairs: Vec::new(),
        skipped: Vec::new(),
        pair_total: components.len().saturating_sub(1) * components.len() / 2,
    };
    for component in components {
        if component.bbox.is_empty() {
            plan.skipped
                .push(format!("{} — no resident geometry, not checked", component.id));
        }
    }
    let boxes: Vec<Aabb> = components.iter().map(|c| inflated(&c.bbox, clearance)).collect();
    let mut order: Vec<usize> = (0..components.len()).filter(|&i| !boxes[i].is_empty()).collect();
    order.sort_by(|&x, &y| boxes[x].min[0].total_cmp(&boxes[y].min[0]));
    for (k, &i) in order.iter().enumerate() {
        for &j in &order[k + 1..] {
            if boxes[j].min[0] > boxes[i].max[0] {
                break; // sorted on min x: nothing later can overlap `i`
            }
            if overlaps(&boxes[i], &boxes[j]) {
                plan.candidate_pairs.push((i.min(j), i.max(j)));
            }
        }
    }
    plan.candidate_pairs.sort_unstable();
    plan
}

// --- the near-miss distance (display tessellation) ---------------------------

type Triangle = [[f64; 3]; 3];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add_scaled(a: [f64; 3], d: [f64; 3], t: f64) -> [f64; 3] {
    [a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dist2(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = sub(a, b);
    dot(d, d)
}

/// Closest point on triangle `t` to `p` (Ericson, Real-Time Collision
/// Detection §5.1.5).
fn closest_on_triangle(p: [f64; 3], t: &Triangle) -> [f64; 3] {
    let [a, b, c] = *t;
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        return add_scaled(a, ab, d1 / (d1 - d3));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        return add_scaled(a, ac, d2 / (d2 - d6));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        return add_scaled(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }
    let denom = 1.0 / (va + vb + vc);
    add_scaled(add_scaled(a, ab, vb * denom), ac, vc * denom)
}

/// Closest points between segments `p1q1` and `p2q2` (Ericson §5.1.9).
fn closest_segment_segment(p1: [f64; 3], q1: [f64; 3], p2: [f64; 3], q2: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let (d1, d2, r) = (sub(q1, p1), sub(q2, p2), sub(p1, p2));
    let (a, e, f) = (dot(d1, d1), dot(d2, d2), dot(d2, r));
    const EPS: f64 = 1e-18;
    let (s, t);
    if a <= EPS && e <= EPS {
        return (p1, p2);
    }
    if a <= EPS {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= EPS {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let s0 = if denom > EPS { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
            let t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            } else {
                t = t0;
                s = s0;
            }
        }
    }
    (add_scaled(p1, d1, s), add_scaled(p2, d2, t))
}

/// Where segment `p→q` pierces triangle `t`, if it does (Möller–Trumbore,
/// parameter clamped to the segment). Coplanar contact returns `None`; the
/// vertex/edge terms of [`triangle_distance`] cover it.
fn segment_hits_triangle(p: [f64; 3], q: [f64; 3], t: &Triangle) -> Option<[f64; 3]> {
    let dir = sub(q, p);
    let (e1, e2) = (sub(t[1], t[0]), sub(t[2], t[0]));
    let h = cross(dir, e2);
    let det = dot(e1, h);
    if det.abs() < 1e-18 {
        return None;
    }
    let inv = 1.0 / det;
    let s = sub(p, t[0]);
    let u = inv * dot(s, h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qv = cross(s, e1);
    let v = inv * dot(dir, qv);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let w = inv * dot(e2, qv);
    (0.0..=1.0).contains(&w).then(|| add_scaled(p, dir, w))
}

/// Exact distance between two triangles with its witness points (on `a`,
/// on `b`): zero where an edge of one pierces the other, else the least of
/// the six vertex–triangle and nine edge–edge distances.
pub(crate) fn triangle_distance(a: &Triangle, b: &Triangle) -> (f64, [f64; 3], [f64; 3]) {
    for (s, t) in [(a, b), (b, a)] {
        for k in 0..3 {
            if let Some(hit) = segment_hits_triangle(s[k], s[(k + 1) % 3], t) {
                return (0.0, hit, hit);
            }
        }
    }
    let mut best = (f64::INFINITY, a[0], b[0]);
    let mut consider = |p: [f64; 3], q: [f64; 3]| {
        let d = dist2(p, q);
        if d < best.0 {
            best = (d, p, q);
        }
    };
    for k in 0..3 {
        consider(a[k], closest_on_triangle(a[k], b));
        consider(closest_on_triangle(b[k], a), b[k]);
        for m in 0..3 {
            let (p, q) = closest_segment_segment(a[k], a[(k + 1) % 3], b[m], b[(m + 1) % 3]);
            consider(p, q);
        }
    }
    (best.0.sqrt(), best.1, best.2)
}

fn triangle_box(t: &Triangle) -> Aabb {
    let mut bbox = Aabb::empty();
    for p in t {
        bbox.expand(*p);
    }
    bbox
}

/// Lower bound on the distance between two boxes (0 when they overlap).
fn box_gap(a: &Aabb, b: &Aabb) -> f64 {
    let mut sum = 0.0;
    for axis in 0..3 {
        let gap = (a.min[axis] - b.max[axis]).max(b.min[axis] - a.max[axis]).max(0.0);
        sum += gap * gap;
    }
    sum.sqrt()
}

/// The display triangles of `solid` whose boxes touch `window`.
fn triangles_near(solid: &crate::scene::SolidDisplay, window: &Aabb) -> Vec<(Triangle, Aabb)> {
    let point = |i: u32| solid.mesh.positions.get(i as usize).map(|p| p.map(f64::from));
    solid
        .mesh
        .indices
        .chunks_exact(3)
        .filter_map(|tri| Some([point(tri[0])?, point(tri[1])?, point(tri[2])?]))
        .map(|t| (t, triangle_box(&t)))
        .filter(|(_, bbox)| overlaps(bbox, window))
        .collect()
}

/// Minimum distance between two display solids, considering only geometry
/// within `reach` of the other solid's box. `None` = nothing within reach.
pub(crate) fn display_min_distance(
    a: &crate::scene::SolidDisplay,
    b: &crate::scene::SolidDisplay,
    reach: f64,
) -> Option<(f64, [f64; 3], [f64; 3])> {
    let near_a = triangles_near(a, &inflated(&b.bbox, reach));
    let near_b = triangles_near(b, &inflated(&a.bbox, reach));
    let mut best: Option<(f64, [f64; 3], [f64; 3])> = None;
    for (ta, box_a) in &near_a {
        for (tb, box_b) in &near_b {
            let bound = best.map_or(f64::INFINITY, |b| b.0);
            if box_gap(box_a, box_b) >= bound {
                continue;
            }
            let found = triangle_distance(ta, tb);
            if found.0 < bound {
                best = Some(found);
                if found.0 == 0.0 {
                    return best;
                }
            }
        }
    }
    best
}

/// `p` through the row-major 4×4 `m` (a rigid pose).
fn transform_point(m: &[f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    ]
}

/// `bbox` posed by `m`: the box of its eight corners.
pub(crate) fn posed_bbox(bbox: &Aabb, m: &[f64; 16]) -> Aabb {
    let mut out = Aabb::empty();
    if bbox.is_empty() {
        return out;
    }
    for corner in 0..8 {
        let p = [
            if corner & 1 == 0 { bbox.min[0] } else { bbox.max[0] },
            if corner & 2 == 0 { bbox.min[1] } else { bbox.max[1] },
            if corner & 4 == 0 { bbox.min[2] } else { bbox.max[2] },
        ];
        out.expand(transform_point(m, p));
    }
    out
}

/// The display triangles of `solid`, posed by `m`, whose boxes touch `window`.
fn posed_triangles_near(solid: &crate::scene::SolidDisplay, m: &[f64; 16], window: &Aabb) -> Vec<(Triangle, Aabb)> {
    let point = |i: u32| solid.mesh.positions.get(i as usize).map(|p| transform_point(m, p.map(f64::from)));
    solid
        .mesh
        .indices
        .chunks_exact(3)
        .filter_map(|tri| Some([point(tri[0])?, point(tri[1])?, point(tri[2])?]))
        .map(|t| (t, triangle_box(&t)))
        .filter(|(_, bbox)| overlaps(bbox, window))
        .collect()
}

/// Whether display `a` posed by `ma` penetrates display `b` posed by `mb`: an
/// edge of one pierces a face of the other. Face-to-face contact is not a
/// penetration (coplanar edges return no hit), and a part wholly inside
/// another is not seen — the motion-study check is a frame-by-frame screen,
/// not the boolean interference check.
pub(crate) fn displays_pierce(a: &crate::scene::SolidDisplay, ma: &[f64; 16], b: &crate::scene::SolidDisplay, mb: &[f64; 16]) -> bool {
    let (box_a, box_b) = (posed_bbox(&a.bbox, ma), posed_bbox(&b.bbox, mb));
    if !overlaps(&box_a, &box_b) {
        return false;
    }
    let near_a = posed_triangles_near(a, ma, &box_b);
    let near_b = posed_triangles_near(b, mb, &box_a);
    for (ta, bx_a) in &near_a {
        for (tb, bx_b) in &near_b {
            if !overlaps(bx_a, bx_b) {
                continue;
            }
            for (s, t) in [(ta, tb), (tb, ta)] {
                for k in 0..3 {
                    if segment_hits_triangle(s[k], s[(k + 1) % 3], t).is_some() {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// The part occurrence a member solid belongs to: its leading run of
/// component-id segments (`ACOMP2:ACOMP1:ACOMP1` of
/// `ACOMP2:ACOMP1:ACOMP1:Part`), never shorter than the top component `top`.
/// A solid's own name may hold `::` (a union), so it is not the last `:`.
fn part_occurrence(solid: &str, top: &str) -> String {
    let rest = solid.strip_prefix(top).and_then(|r| r.strip_prefix(':')).unwrap_or("");
    let mut leaf = top.to_string();
    let segments: Vec<&str> = rest.split(':').collect();
    // The last segment is the solid's own name; the ones before it that are
    // component ids are the nested occurrences.
    for segment in segments.iter().take(segments.len().saturating_sub(1)) {
        let id = segment.strip_prefix("ACOMP").is_some_and(|d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()));
        if !id {
            break;
        }
        leaf.push(':');
        leaf.push_str(segment);
    }
    leaf
}

// --- the engine surface ------------------------------------------------------

impl EngineState {
    /// Every PART occurrence, at any depth, with its member solids, in
    /// history order (Pattern Component instances after the placed ones): a
    /// sub-assembly's parts are checked against each other and against
    /// everything else. (Only top-level components were, so two parts inside
    /// one sub-assembly were never checked: on l500, 10 module pairs.) A part
    /// placed at the top is its own occurrence, as before.
    /// Also the motion-study clash check's grouping (`simulation_play`).
    pub(crate) fn interference_participants(&self) -> Vec<(String, Vec<String>)> {
        let mut tops = self.component_ids();
        tops.extend(self.pattern_instance_ids());
        let mut out: Vec<(String, Vec<String>)> = Vec::new();
        for top in &tops {
            let members = self.component_members(top);
            if members.is_empty() {
                // No geometry (not built yet, rolled back): still a
                // participant, so it reads as unchecked, never as clear.
                out.push((top.clone(), Vec::new()));
            }
            for name in members {
                let leaf = part_occurrence(&name, top);
                match out.iter_mut().find(|(id, _)| *id == leaf) {
                    Some((_, solids)) => solids.push(name),
                    None => out.push((leaf, vec![name])),
                }
            }
        }
        out
    }

    /// [`Self::interference_check_with`] without a near-miss check.
    pub fn interference_check(&mut self) -> InterferenceReport {
        self.interference_check_with(0.0)
    }

    /// Run the interference check over every component instance (hidden ones
    /// included — interference is a physical question), plus the near-miss
    /// check for gaps up to `clearance` mm (0 = off). Non-destructive: the
    /// component solids are only READ; each pairwise INTERSECT result is
    /// measured, tessellated for the overlay, and freed. Draws the overlay
    /// and returns the full report for the results window.
    pub fn interference_check_with(&mut self, clearance: f64) -> InterferenceReport {
        let clearance = if clearance.is_finite() { clearance.max(0.0) } else { 0.0 };

        // Gather the participants: members + union bbox + hidden, in the
        // deterministic history order component_ids() gives.
        // Pattern Component instances are components without a feature of
        // their own: they take part after the placed ones.
        let (ids, grouped): (Vec<String>, Vec<Vec<String>>) = self.interference_participants().into_iter().unzip();
        let mut members: Vec<Vec<String>> = Vec::with_capacity(ids.len());
        let mut plan_input: Vec<PlanComponent> = Vec::with_capacity(ids.len());
        for (id, solids) in ids.iter().zip(grouped) {
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

        let plan = plan_pairs(&plan_input, clearance);
        let mut report = InterferenceReport {
            component_count: ids.len(),
            pair_total: plan.pair_total,
            clearance,
            skipped: plan.skipped,
            generation: self.applied_generation(),
            ..Default::default()
        };
        let lod = if self.settings.lod_factor.is_finite() && self.settings.lod_factor > 0.0 {
            self.settings.lod_factor
        } else {
            1.0
        };

        // The warm MAIN-SIDE handle map (cache-hit replay on this thread) —
        // the same lane the Info windows' mass properties ride.
        let handles = if plan.candidate_pairs.is_empty() {
            Default::default()
        } else {
            self.resident_solid_handles()
        };
        for (i, j) in plan.candidate_pairs {
            let (a, b) = (&plan_input[i], &plan_input[j]);
            let mut volume = 0.0;
            let mut refusal: Option<String> = None;
            let mut ran_boolean = false;
            let mut gap: Option<(f64, [f64; 3], [f64; 3])> = None;
            for solid_a in &members[i] {
                for solid_b in &members[j] {
                    let (Some(da), Some(db)) =
                        (self.scene.solid(solid_a), self.scene.solid(solid_b))
                    else {
                        refusal.get_or_insert_with(|| format!("missing display operand: {solid_a} or {solid_b}"));
                        continue;
                    };
                    // Member-level prefilter: within a candidate component
                    // pair, only member solids whose own boxes overlap pay
                    // for a boolean; those within the clearance pay for a
                    // distance.
                    if overlaps(&inflated(&da.bbox, 0.0), &inflated(&db.bbox, 0.0)) {
                        let (Some(&ha), Some(&hb)) = (handles.get(solid_a), handles.get(solid_b))
                        else {
                            refusal.get_or_insert_with(|| format!("missing resident operand: {solid_a} or {solid_b}"));
                            continue;
                        };
                        ran_boolean = true;
                        match intersect_volume(ha, hb, Some(lod)) {
                            Ok((v, triangles)) => {
                                volume += v;
                                report.volume_triangles.extend(triangles);
                            }
                            Err(error) => {
                                // A conservative kernel refusal (grazing/tangent
                                // contact): the pair is UNVERIFIED, never a
                                // silent pass and never fake interference.
                                refusal.get_or_insert(error);
                            }
                        }
                    }
                    if clearance > 0.0 && overlaps(&inflated(&da.bbox, clearance), &db.bbox) {
                        if let Some(found) = display_min_distance(da, db, clearance) {
                            if gap.is_none_or(|g| found.0 < g.0) {
                                gap = Some(found);
                            }
                        }
                    }
                }
            }
            if ran_boolean {
                report.booleans_run += 1;
            }
            if let Some(error) = &refusal {
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
            } else if let Some((distance, pa, pb)) = gap.filter(|g| g.0 <= clearance) {
                report.near_misses.push(ClearancePair {
                    a: a.id.clone(),
                    b: b.id.clone(),
                    distance,
                    witness: [pa, pb],
                    a_hidden: a.hidden,
                    b_hidden: b.hidden,
                });
            }
        }
        // Largest interference first, smallest gap first (stable → id order
        // breaks ties).
        report
            .pairs
            .sort_by(|x, y| y.volume.total_cmp(&x.volume));
        report
            .near_misses
            .sort_by(|x, y| x.distance.total_cmp(&y.distance));
        self.show_interference_overlay(&report);
        report
    }

    /// Draw `report` in the viewport: interference volumes as red triangles,
    /// near-miss gaps as an amber segment with its two end points. An empty
    /// report removes the group.
    pub fn show_interference_overlay(&mut self, report: &InterferenceReport) {
        let rgb = |status: &str| crate::assembly_status::status_color_rgb(status).map(|c| f32::from(c) / 255.0);
        let (red, amber) = (rgb("error"), rgb("unsupported-selection"));
        let tri_positions: Vec<f32> = report.volume_triangles.iter().flatten().copied().collect();
        let tri_colors: Vec<f32> = report.volume_triangles.iter().flat_map(|_| red).collect();
        let ends: Vec<f32> = report
            .near_misses
            .iter()
            .flat_map(|pair| pair.witness)
            .flatten()
            .map(|v| v as f32)
            .collect();
        let end_colors: Vec<f32> = (0..ends.len() / 3).flat_map(|_| amber).collect();
        let _ = self.set_overlay_json(
            &serde_json::json!({
                "groups": [{
                    "name": INTERFERENCE_OVERLAY_GROUP,
                    "renderOrder": 10001,
                    "tris": { "positions": tri_positions, "colors": tri_colors },
                    "lines": { "positions": ends, "colors": end_colors },
                    "points": { "positions": ends, "colors": end_colors },
                }]
            })
            .to_string(),
        );
    }

    /// Remove the interference overlay (window closed, report stale).
    pub fn clear_interference_overlay(&mut self) {
        let _ = self.set_overlay_json(
            &serde_json::json!({ "groups": [{ "name": INTERFERENCE_OVERLAY_GROUP }] }).to_string(),
        );
    }

    /// Frame the camera on the union of the given components' member solids
    /// (the interference window's row click). Unknown ids are ignored.
    pub fn zoom_to_components(&mut self, component_ids: &[String]) {
        let mut bbox = Aabb::empty();
        for id in component_ids {
            for name in self.component_info(id).map(|info| info.members).unwrap_or_default() {
                if let Some(solid) = self.scene.solid(&name) {
                    bbox.union(&solid.bbox);
                }
            }
        }
        if !bbox.is_empty() {
            self.camera.zoom_to_fit(&bbox, 1.5);
            self.dirty = true;
        }
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
    pub fn interference_page(&mut self, scope: Option<Vec<String>>, budget: usize, token: Option<String>, clearance: Option<f64>) -> Result<serde_json::Value, String> {
        use serde_json::json;
        if self.run_pending() { return Err("model rebuild is pending; wait before checking interference".into()); }
        if !(1..=4096).contains(&budget) { return Err("budget must be in 1..=4096 component pairs".into()); }
        let clearance = clearance.unwrap_or(0.0);
        if !clearance.is_finite() || clearance < 0.0 { return Err("clearanceMm must be a finite distance >= 0".into()); }
        let revision = self.applied_generation();
        let document = self.history.request_json();
        let mut cursor = if let Some(token) = &token {
            let c = self.interference_sessions.get(token).ok_or("unknown or expired continuation token")?.clone();
            if c.revision != revision || c.document != document { return Err("stale continuation: model changed; restart without a token".into()); }
            if scope.as_ref().is_some_and(|s| s != &c.scope) { return Err("continuation scope differs from requested components".into()); }
            c
        } else {
            let known: Vec<String> = self.interference_participants().into_iter().map(|(id, _)| id).collect();
            let mut tops = self.component_ids();
            tops.extend(self.pattern_instance_ids());
            // A component named in the scope stands for its part occurrences
            // (a sub-assembly for every part inside it).
            let ids: Vec<String> = match scope {
                None => known.clone(),
                Some(requested) => {
                    let mut seen = std::collections::HashSet::new();
                    for id in &requested {
                        if !known.contains(id) && !tops.contains(id) { return Err(format!("unknown component `{id}`")); }
                        if !seen.insert(id.clone()) { return Err(format!("duplicate component `{id}`")); }
                    }
                    let mut expanded: Vec<String> = Vec::new();
                    for id in requested {
                        let prefix = format!("{id}:");
                        for leaf in known.iter().filter(|leaf| **leaf == id || leaf.starts_with(&prefix)) {
                            if !expanded.contains(leaf) { expanded.push(leaf.clone()); }
                        }
                    }
                    expanded
                }
            };
            InterferenceCursor { revision, document, scope: ids, next: 0, unresolved: 0, interfering: 0 }
        };
        let total = cursor.scope.len() * cursor.scope.len().saturating_sub(1) / 2;
        let start = cursor.next;
        let end = start.saturating_add(budget).min(total);
        let handles = self.resident_solid_handles();
        let participants: std::collections::HashMap<String, Vec<String>> = self.interference_participants().into_iter().collect();
        // Each solid's conservative bounds, once (they were computed from a
        // clone of both exact solids for every solid pair: on l500, 10 module
        // pairs cloned their 100×100 solid pairs, ~3 s).
        let mut bounds: std::collections::HashMap<String, Option<Aabb>> = std::collections::HashMap::new();
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
                let ma = participants.get(a).cloned().unwrap_or_default();
                let mb = participants.get(b).cloned().unwrap_or_default();
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
                        let mut bound = |name: &String, handle: u32| -> Option<Aabb> {
                            *bounds.entry(name.clone()).or_insert_with(|| {
                                brep_kernel::registered_solid_clone(handle).ok().and_then(|solid| conservative_bounds(&solid))
                            })
                        };
                        if let (Some(ba), Some(bb)) = (bound(sa, *ha), bound(sb, *hb)) {
                            if !overlaps(&ba, &bb) { continue; }
                        }
                        let exact_a = brep_kernel::registered_solid_clone(*ha);
                        let exact_b = brep_kernel::registered_solid_clone(*hb);
                        ran_boolean = true;
                        boolean_operations += 1;
                        match intersect_volume(*ha, *hb, None).map(|(v, _)| v) {
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
                // Near miss: the gap over the display tessellation (approximate
                // by the chord deviation), only for non-interfering pairs.
                let mut min_distance: Option<f64> = None;
                if clearance > 0.0 && volume <= VOLUME_EPSILON {
                    for sa in &ma {
                        for sb in &mb {
                            let (Some(da), Some(db)) = (self.scene.solid(sa), self.scene.solid(sb)) else { continue };
                            if !overlaps(&inflated(&da.bbox, clearance), &db.bbox) { continue; }
                            if let Some((d, _, _)) = display_min_distance(da, db, clearance) {
                                min_distance = Some(min_distance.map_or(d, |m: f64| m.min(d)));
                            }
                        }
                    }
                }
                let status = if !errors.is_empty() { "unverified" }
                    else if volume > VOLUME_EPSILON { "verified_interference" }
                    else if ambiguous { "unverified" } else if touching { "verified_touching" } else { "verified_clearance" };
                if status == "unverified" { cursor.unresolved += 1; }
                if volume > VOLUME_EPSILON { cursor.interfering += 1; }
                outcomes.push(json!({"index":index, "a":a, "b":b, "status":status, "volume":volume,
                    "aHidden":ma.iter().any(|n| self.scene.solid(n).is_some_and(|s| !s.visible)),
                    "bHidden":mb.iter().any(|n| self.scene.solid(n).is_some_and(|s| !s.visible)),
                    "minDistance":min_distance, "nearMiss":min_distance.is_some_and(|d| d <= clearance),
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
        let mut near_misses: Vec<&serde_json::Value> = outcomes.iter().filter(|p| p["nearMiss"] == true).collect();
        near_misses.sort_by(|a,b| a["minDistance"].as_f64().unwrap().total_cmp(&b["minDistance"].as_f64().unwrap()));
        let skipped: Vec<String> = if complete { vec![] } else { vec![format!("{} component pairs not checked: pair budget exhausted", total-end)] };
        let unverified: Vec<String> = outcomes.iter().filter(|p| p["status"] == "unverified").map(|p| format!("{} × {}: unverified (see outcomes)",p["a"].as_str().unwrap_or(""),p["b"].as_str().unwrap_or(""))).collect();
        Ok(json!({"modelRevision":revision, "scope":cursor.scope, "componentCount":cursor.scope.len(),
            "budget":budget, "pairTotal":total, "checkedPairs":end, "pageStart":start, "pairs":interfering, "clearanceMm":clearance, "nearMisses":near_misses, "outcomes":outcomes, "skipped":skipped, "unverified":unverified, "booleansRun":booleans_run, "booleanOperations":boolean_operations,
            "notChecked": {"count":total-end, "fromPairIndex":end, "status":"not_checked_budget_exhausted", "reason":if complete { serde_json::Value::Null } else { json!("budget_exhausted") }},
            "unresolvedCount":cursor.unresolved, "interferenceCount":cursor.interfering,
            "complete":complete, "allClear":complete && cursor.unresolved == 0 && cursor.interfering == 0,
            "continuationToken":next_token, "volumeToleranceMm3":VOLUME_EPSILON,
            "contactToleranceMm":1e-6, "contactVerification":"boundary vertex witness within tolerance after successful intersection; unwitnessed zero-volume pairs remain unverified"}))
    }
}


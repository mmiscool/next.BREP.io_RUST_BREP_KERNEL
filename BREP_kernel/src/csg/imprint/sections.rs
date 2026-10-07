use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use super::*;

/// A marched intersection branch that runs along existing boundary edges of
/// BOTH faces cannot bound any new fragment — the boundary is already there.
/// Worse, the fitted polyline wiggles within tolerance of the true curve and
/// its crossings spray micro edge-splits over the very edges it follows
/// (glue unions march the touching pair in one operand order and produced
/// thousands of noise splits). Skip such branches entirely.
pub(super) fn branch_follows_shared_boundary(
    points: &[Vec3],
    first: TaggedFace<'_>,
    second: TaggedFace<'_>,
    edges: &HashMap<(u8, u64), &EdgeRecord>,
    tolerance: f64,
    scale: f64,
) -> Result<bool, KernelRefusal> {
    let Some(last) = points.last() else {
        return Ok(false);
    };
    let weld_limit = assembler_weld(tolerance);
    let step = (points.len() / 16).max(1);
    for face in [first, second] {
        let face_edges = face_edges(face, edges)?;
        for point in points.iter().step_by(step).chain(std::iter::once(last)) {
            // A tangential contact curve is only determinable to the
            // square-root of the tolerance times the local size (the
            // intersection is ill-conditioned there), so the marched points
            // legitimately wander that far from the exact shared boundary.
            // Widen the proximity band accordingly where the two surface
            // normals are (anti-)parallel.
            let first_projection = project_point_to_surface(&first.face.surface, *point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
            let second_projection = project_point_to_surface(&second.face.surface, *point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
            // At a surface pole — e.g. the apex a partial revolve leaves where a
            // profile edge touches the axis — the normal is undefined (the patch
            // collapses to a point, so the derivative cross product is zero).
            // The normals here only decide whether to WIDEN the proximity band
            // for an ill-conditioned tangential contact; a pole is exactly such an
            // ill-conditioned spot. Treat a missing normal as tangential (widen)
            // instead of propagating the zero-length error and aborting the whole
            // boolean.
            let first_normal = first
                .face
                .surface
                .normal(first_projection.u, first_projection.v)
                .ok();
            let second_normal = second
                .face
                .surface
                .normal(second_projection.u, second_projection.v)
                .ok();
            let tangential = match (first_normal, second_normal) {
                (Some(a), Some(b)) => a.cross(b).length() <= 1e-2,
                _ => true,
            };
            let limit = if tangential {
                (tolerance.max(1e-7) * merge_scale(scale))
                    .sqrt()
                    .max(weld_limit)
            } else {
                weld_limit
            };
            let mut near = false;
            let mut best = f64::INFINITY;
            for edge in &face_edges {
                if edge.degenerate {
                    continue;
                }
                let projection = project_point_to_curve(&edge.curve, *point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                let low = edge.t0.min(edge.t1);
                let high = edge.t0.max(edge.t1);
                let on = edge.curve.evaluate(projection.u.clamp(low, high)).or_refuse(KernelStage::Intersect, "evaluate")?;
                let distance = on.sub(*point).length();
                if distance < best {
                    best = distance;
                }
                if distance <= limit {
                    near = true;
                    break;
                }
            }
            if !near {
                if std::env::var("BREP_DEBUG_BRANCH").is_ok() {
                    eprintln!(
                        "branch reject: face {} best={:.3e} limit={:.3e} point=({:.6},{:.6},{:.6})",
                        face.face.id, best, limit, point.x, point.y, point.z
                    );
                }
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Keep only the runs of a marched branch that lie inside (or on the
/// boundary of) BOTH faces' trimmed regions. The marcher works on the
/// untrimmed carriers and happily chases intersections far outside the
/// actual faces (an extruded wall's carrier spans the full tool travel);
/// those runaway sections then spray noise splits over real edges during
/// imprint processing. Points are tested on a subsample grid with one-step
/// margin so genuine trim crossings keep a little slack for the exact
/// piece-level clipping that follows.
/// SEED-REFINED CLIP (multi-interval section rescue). `clip_branch_to_trims`
/// classifies only the branch's own SAMPLE points, so a trim interval SHORTER
/// than the march step is invisible: a section line crossing an annular face
/// twice (solid 00000327's serrated rings vs a box wall) had its ~1.9e-3 exit
/// interval fall entirely between two ~3.5e-3-apart samples — the clip saw no
/// inside point, dropped the interval, the strip never split, and the whole
/// outline chain stranded one-use (non-integral genus, `did not converge`
/// churn downstream). The boundary-pierce SEEDS already computed for the
/// marcher are the EXACT crossing points of the section with the faces' trim
/// boundaries — the interval endpoints. Inserting every seed that lies ON the
/// branch polyline into it (ordered along its segment) makes each interval
/// carry its own endpoints, so the point classification cannot miss it.
/// Purely additive: points are only ADDED, every previously-kept run is still
/// kept. The insertion gate (`max(tolerance*10, 0.1*segment)`) admits the
/// zero-sag analytic cases and the small sag of curved marches while
/// rejecting seeds of OTHER nearby branches. Escape hatch
/// `BREP_SEED_CLIP_REFINE=0`.
pub(super) fn insert_seed_points_into_branch(points: &[Vec3], seeds: &[Vec3], tolerance: f64) -> Vec<Vec3> {
    if points.len() < 2
        || seeds.is_empty()
        || std::env::var("BREP_SEED_CLIP_REFINE").as_deref() == Ok("0")
    {
        return points.to_vec();
    }
    // Per segment, the seeds that project onto it, ordered by fraction.
    let mut insertions: Vec<Vec<(f64, Vec3)>> = vec![Vec::new(); points.len() - 1];
    for seed in seeds {
        let mut best: Option<(usize, f64, f64)> = None; // (segment, distance, fraction)
        for (index, pair) in points.windows(2).enumerate() {
            let direction = pair[1].sub(pair[0]);
            let length_squared = direction.dot(direction);
            if length_squared <= 0.0 {
                continue;
            }
            let fraction = (seed.sub(pair[0]).dot(direction) / length_squared).clamp(0.0, 1.0);
            let distance = seed.sub(pair[0].add(direction.scale(fraction))).length();
            if best.is_none_or(|(_, d, _)| distance < d) {
                best = Some((index, distance, fraction));
            }
        }
        let Some((index, distance, fraction)) = best else {
            continue;
        };
        let segment_length = points[index + 1].sub(points[index]).length();
        if distance > (tolerance * 10.0).max(segment_length * 0.1) {
            continue; // not on this branch
        }
        // An interior insertion only — a seed at an existing sample adds nothing.
        let near_end = fraction * segment_length <= tolerance
            || (1.0 - fraction) * segment_length <= tolerance;
        if near_end {
            continue;
        }
        // One pierce point, one station. Operands already split at the
        // section's ends (the idempotence corpus's split siblings) carry the
        // pierce as the shared vertex of two half-edges, and both halves report
        // the hit, so the same point arrives here twice. Inserted twice it is a
        // zero-length chord beside the clip end, which the sliver-end evening
        // then re-solves in place for every round it has instead of merging
        // the sliver into its neighbour: `28_march_swap_rescue_t217` intersect
        // kept a fit 5.9e-5 off the cylinder on the split operands where the
        // raw operands' evening reached 4.9e-9 (2026-10-03).
        if insertions[index].iter().any(|(_, queued)| queued.sub(*seed).length() <= tolerance) {
            continue;
        }
        insertions[index].push((fraction, *seed));
    }
    if insertions.iter().all(Vec::is_empty) {
        return points.to_vec();
    }
    let mut refined = Vec::with_capacity(points.len() + seeds.len());
    for (index, point) in points.iter().enumerate() {
        refined.push(*point);
        if index < insertions.len() {
            let segment = &mut insertions[index];
            segment.sort_by(|a, b| a.0.total_cmp(&b.0));
            refined.extend(segment.iter().map(|(_, seed)| *seed));
        }
    }
    refined
}

pub(super) fn clip_branch_to_trims(
    points: &[Vec3],
    first: TaggedFace<'_>,
    second: TaggedFace<'_>,
) -> Result<Vec<Vec<Vec3>>, KernelRefusal> {
    let count = points.len();
    if count < 2 {
        return Ok(Vec::new());
    }
    let debug_clip = std::env::var("BREP_DEBUG_CLIP").is_ok_and(|value| {
        value == "all"
            || value == first.face.id.to_string()
            || value == second.face.id.to_string()
    });
    let inside_both = |point: Vec3| -> Result<bool, KernelRefusal> {
        for face in [first, second] {
            // The face's CHART, not its carrier: where the trim is drawn past a
            // closed direction's domain, the carrier's own projection folds the
            // point back into the domain and the trim's even-odd test then
            // answers Outside for every point of the overhang the trim covers
            // (helmet `Face_15`: a section point at u = −0.030 read at 0.970).
            // On the chart the parameter is the one the loops are drawn in.
            let projection = project_point_to_surface(face.chart(), point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
            let status = parameter_point_in_face(
                face.face,
                Vec2 {
                    x: projection.u,
                    y: projection.v,
                },
                1e-6,
            ).or_refuse(KernelStage::Intersect, "csg.imprint.sections")?;
            if debug_clip {
                eprintln!(
                    "clip {}x{}: p=({:.5},{:.5},{:.5}) on face {} uv=({:.5},{:.5}) dist={:.2e} {:?}",
                    first.face.id,
                    second.face.id,
                    point.x,
                    point.y,
                    point.z,
                    face.face.id,
                    projection.u,
                    projection.v,
                    projection.distance,
                    status
                );
            }
            if status == PolygonClass::Outside {
                return Ok(false);
            }
        }
        Ok(true)
    };
    let step = (count / 256).max(1);
    let mut keep = vec![false; count];
    let mut index = 0;
    loop {
        let sample = index.min(count - 1);
        if inside_both(points[sample])? {
            let low = sample.saturating_sub(step);
            let high = (sample + step).min(count - 1);
            for flag in &mut keep[low..=high] {
                *flag = true;
            }
        }
        if sample == count - 1 {
            break;
        }
        index += step;
    }
    let mut runs = Vec::new();
    let mut current: Vec<Vec3> = Vec::new();
    for (point, kept) in points.iter().zip(&keep) {
        if *kept {
            current.push(*point);
        } else if !current.is_empty() {
            runs.push(std::mem::take(&mut current));
        }
    }
    if !current.is_empty() {
        runs.push(current);
    }
    Ok(runs)
}

/// True when at least three of five interior samples of `curve` lie on the
/// trimmed span of `edge` — a (near-)coincident overlap rather than a
/// transversal crossing.
pub(super) fn curve_overlaps_edge(curve: &NurbsCurve, edge: &EdgeRecord, limit: f64) -> Result<bool, KernelRefusal> {
    let [c0, c1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let low = edge.t0.min(edge.t1);
    let high = edge.t0.max(edge.t1);
    let mut on = 0;
    for fraction in [0.1, 0.3, 0.5, 0.7, 0.9] {
        let point = curve.evaluate(c0 + (c1 - c0) * fraction).or_refuse(KernelStage::Intersect, "evaluate")?;
        let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
        let nearest = edge.curve.evaluate(projection.u.clamp(low, high)).or_refuse(KernelStage::Intersect, "evaluate")?;
        if nearest.sub(point).length() <= limit {
            on += 1;
        }
    }
    Ok(on >= 3)
}

/// Worst distance from any sample of `a` to the curve `b` (a one-sided Hausdorff
/// distance, sampled). Used bidirectionally so the B2 span-coincidence test
/// rejects partial overlaps and same-endpoint-but-different-path curves.
pub(super) fn max_curve_deviation(a: &NurbsCurve, b: &NurbsCurve) -> Result<f64, KernelRefusal> {
    let [d0, d1] = a.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let mut worst = 0.0f64;
    for k in 0..=32 {
        let t = d0 + (d1 - d0) * (k as f64 / 32.0);
        let point = a.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
        worst = worst.max(project_point_to_curve(b, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?.distance);
    }
    Ok(worst)
}

/// The sub-curve of `curve` over the parameter window `[lo, hi]`, by exact
/// knot-insertion splitting (mirrors `boolean::curve_subrange` but by absolute
/// parameter rather than fraction). `lo`/`hi` are clamped away from the domain
/// ends by the split guard, so a window flush with an end keeps that end.
fn curve_param_subrange(curve: &NurbsCurve, lo: f64, hi: f64) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let epsilon = ((d1 - d0).abs() * 1e-9).max(2e-9);
    let mut result = curve.clone();
    if lo > d0 + epsilon && lo < d1 - epsilon {
        result = result.split(lo).or_refuse(KernelStage::Intersect, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Intersect, "domain")?;
    if hi < domain[1] - epsilon && hi > domain[0] + epsilon {
        result = result.split(hi).or_refuse(KernelStage::Intersect, "split")?.0;
    }
    Ok(result)
}

/// B2 (OCCT `BOPDS_CommonBlock` / `IsExistingPaveBlock`): a SSI section piece
/// whose supporting curve coincides ALONG ITS WHOLE SPAN with a SUB-ARC of an
/// EXISTING boundary edge of one of its support faces is not a new 1-cell — it
/// IS that boundary. `apply_edge_splits` will mint that sub-arc as its own edge
/// (splitting the parent at the section's endpoints), which the OWNING face
/// then carries as a boundary. So instead of the section drifting a graze-gap
/// away from that sub-arc (the marched/analytic chord vs the vendor arc — the
/// one-use pair), we REPLACE the section's geometry with the sub-arc itself and
/// drop the now-redundant cut from the owning face. The section on the KEPT
/// (other) face then welds to the split sub-arc by identical geometry — one
/// shared edge, no duplicate — exactly OCCT's "reuse the existing pave block".
///
/// The decision is IDENTITY-grounded, never a bare proximity grab (per the B1
/// falsification): BOTH section endpoints must project ONTO the boundary edge's
/// curve within the weld radius (at parameters inside its live window), and the
/// section must lie within a size-relative span band of the sub-arc in BOTH
/// directions across its whole span. It must ALSO deviate from the sub-arc by
/// MORE than the weld radius somewhere along the span — a section already within
/// the weld radius is a coincident / cosurface boundary-exchange copy the
/// assembler welds as-is, and reusing it would strand a coincident face's split.
/// Any geometry step that fails (sub-arc extraction, pcurve rebuild on the kept
/// face) aborts that one reuse and keeps the historical behaviour. Escape hatch
/// `BREP_SHARED_SECTION_EDGE=0` restores the mint-a-duplicate path wholesale
/// (the clean A/B lever).
pub(super) fn reuse_boundary_section_edges(
    result: &mut ImprintResultRecord,
    face_edge_lists: &HashMap<FaceKey, Vec<&EdgeRecord>>,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    if std::env::var("BREP_SHARED_SECTION_EDGE").as_deref() == Ok("0") {
        return Ok(());
    }
    let debug = std::env::var("BREP_DEBUG_SHARED_SECTION").is_ok();
    // Endpoint gate: the section's endpoints must lie ON the boundary edge's
    // curve (not merely near its end vertices) — the coincident edge is a SUB-
    // ARC that `apply_edge_splits` later mints at exactly these junction points.
    // Measured on fixture 09: matching endpoints project at ~1e-9; the nearest
    // non-matching edge is ≥3.5e-3 — 2.5 orders of separation.
    let endpoint_gate = assembler_weld(tolerance);
    let scale = solid_scale(solid_a).max(solid_scale(solid_b));
    let span_band = (SHARED_SECTION_BAND_FRACTION * scale).max(endpoint_gate);
    let piece_point: HashMap<u64, Vec3> =
        result.vertices.iter().map(|v| (v.id, v.point)).collect();
    let face_surface = |key: &FaceKey| -> Option<&NurbsSurface> {
        let solid = if key.operand == 0 { solid_a } else { solid_b };
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == key.face_id)
            .map(|face| &face.surface)
    };

    struct Reuse {
        piece_index: usize,
        owning: FaceKey,
        edge_id: u64,
        arc: NurbsCurve,
        aligned: bool,
        /// Every support face whose OWN boundary already carries this section
        /// (the owning face included). See the two-sided note at the apply
        /// loop: a face in this list must not be cut.
        redundant: Vec<FaceKey>,
    }
    // Decide first (immutable borrow of result.pieces), apply after.
    let mut decisions: Vec<Reuse> = Vec::new();
    for (piece_index, piece) in result.pieces.iter().enumerate() {
        if piece.shared_edge.is_some() {
            continue;
        }
        let (Some(&ps), Some(&pe)) = (
            piece_point.get(&piece.start_vertex_id),
            piece_point.get(&piece.end_vertex_id),
        ) else {
            continue;
        };
        if ps.sub(pe).length() <= endpoint_gate {
            // CLOSED-RING REUSE (problemInbox equator-tangent): a closed
            // section ring coincident with an existing closed boundary edge
            // (the inscribed sphere's equator IS the cylinder's cap-edge
            // circle) can never be unified by the assembler's endpoint weld —
            // closed rings have no distinct endpoints to weld — so the two
            // records stay separate and strand one-use. Unlike the open-arc
            // path below, EXACT coincidence must be accepted here (no lower
            // floor): reuse is the ONLY identity mechanism available to
            // rings. Escape hatch: BREP_SHARED_SECTION_RING=0.
            if std::env::var("BREP_SHARED_SECTION_RING").as_deref() != Ok("0") {
                let mut chosen: Option<Reuse> = None;
                let mut best_score = f64::INFINITY;
                for face_key in &piece.support_faces {
                    let Some(edges) = face_edge_lists.get(face_key) else {
                        continue;
                    };
                    for edge in edges {
                        if edge.degenerate {
                            continue;
                        }
                        let (Ok(es), Ok(ee)) =
                            (edge.curve.evaluate(edge.t0), edge.curve.evaluate(edge.t1))
                        else {
                            continue;
                        };
                        if es.sub(ee).length() > endpoint_gate {
                            continue; // open edge cannot host a ring
                        }
                        let Ok(ring) = edge_subcurve(edge) else {
                            continue;
                        };
                        let (Ok(dev_a), Ok(dev_b)) = (
                            max_curve_deviation(&piece.curve, &ring),
                            max_curve_deviation(&ring, &piece.curve),
                        ) else {
                            continue;
                        };
                        let deviation = dev_a.max(dev_b);
                        // Rings ride a boundary edge only on EXACT identity
                        // (weld-scale): a near-coincident-but-distinct ring
                        // (t149: dev 1.1e-2) is real separate topology, and
                        // absorbing it corrupts the arrangement. The open-arc
                        // path's generous span_band does not apply here.
                        if deviation > endpoint_gate || deviation >= best_score {
                            continue;
                        }
                        // Orientation: compare tangents at the piece start's
                        // image on the ring.
                        let Ok(projection) = project_point_to_curve(&ring, ps) else {
                            continue;
                        };
                        let (Ok((_, piece_tangent)), Ok((_, ring_tangent))) =
                            (piece.curve.deriv1(piece.t0), ring.deriv1(projection.u))
                        else {
                            continue;
                        };
                        best_score = deviation;
                        chosen = Some(Reuse {
                            piece_index,
                            owning: *face_key,
                            edge_id: edge.id,
                            arc: ring.clone(),
                            aligned: piece_tangent.dot(ring_tangent) >= 0.0,
                            redundant: vec![*face_key],
                        });
                    }
                }
                if let Some(reuse) = chosen {
                    if debug {
                        eprintln!(
                            "shared-section RING piece {} -> edge {}:{} dev={best_score:.3e}",
                            piece.id, reuse.owning.operand, reuse.edge_id
                        );
                    }
                    decisions.push(reuse);
                }
            }
            continue;
        }
        let mut chosen: Option<Reuse> = None;
        let mut best_score = f64::INFINITY;
        // TWO-SIDED COMMON BLOCK. The section is the intersection of its two
        // support faces, so when it retraces an existing boundary edge it can
        // retrace one on EACH of them — two copies of the same 1-cell, one per
        // operand. That is the 2026-09-14 herringbone document's z = 0 seam: a
        // right tooth band's flank meets the left band's flank exactly along
        // the profile edge both faces already end on. Dropping the redundant
        // cut from the owning face alone left the OTHER face cut by a curve
        // sitting 4.2e-4 off its own boundary edge, and the arrangement's lens
        // between the two produced a region no chain could complete
        // ("chain N fragmented into an incomplete run"). Collect every support
        // face whose boundary carries the section so the apply loop can decline
        // to cut all of them.
        let mut redundant: Vec<FaceKey> = Vec::new();
        for face_key in &piece.support_faces {
            let Some(edges) = face_edge_lists.get(face_key) else {
                continue;
            };
            for edge in edges {
                if edge.degenerate {
                    continue;
                }
                // Both section endpoints must lie ON the edge's curve, at
                // parameters inside the edge's live window (closed rings
                // included — the parent is often a ring the section grazes).
                let (Ok(proj_s), Ok(proj_e)) = (
                    project_point_to_curve(&edge.curve, ps),
                    project_point_to_curve(&edge.curve, pe),
                ) else {
                    continue;
                };
                if proj_s.distance > endpoint_gate || proj_e.distance > endpoint_gate {
                    continue;
                }
                let lo_t = edge.t0.min(edge.t1);
                let hi_t = edge.t0.max(edge.t1);
                let margin = (hi_t - lo_t).abs() * 1e-6 + 1e-9;
                if proj_s.u < lo_t - margin
                    || proj_s.u > hi_t + margin
                    || proj_e.u < lo_t - margin
                    || proj_e.u > hi_t + margin
                {
                    continue;
                }
                let aligned = proj_s.u <= proj_e.u;
                let lo = proj_s.u.min(proj_e.u);
                let hi = proj_s.u.max(proj_e.u);
                if hi - lo <= margin {
                    continue; // sub-arc collapses to a point
                }
                // The sub-arc of THIS edge between the two junctions — the exact
                // 1-cell apply_edge_splits will mint. Orient it ps→pe and snap
                // its ends to the section's canonical vertices.
                let Ok(mut arc) = curve_param_subrange(&edge.curve, lo, hi) else {
                    continue;
                };
                if !aligned {
                    let Ok(reversed) = arc.reversed() else {
                        continue;
                    };
                    arc = reversed;
                }
                let Ok(arc) = snap_curve_ends(&arc, ps, pe) else {
                    continue;
                };
                // Bidirectional whole-span coincidence between the section and
                // the SUB-ARC (rejects partial overlap / different-path curves).
                let sec_to_arc = max_curve_deviation(&piece.curve, &arc)?;
                let arc_to_sec = max_curve_deviation(&arc, &piece.curve)?;
                if debug {
                    eprintln!(
                        "shared-section? piece {} sup=[{}:{},{}:{}] vs edge {}:{} \
                         proj=({:.2e},{:.2e}) sec->arc={:.3e} arc->sec={:.3e} band={:.3e}",
                        piece.id,
                        piece.support_faces[0].operand,
                        piece.support_faces[0].face_id,
                        piece.support_faces[1].operand,
                        piece.support_faces[1].face_id,
                        face_key.operand,
                        edge.id,
                        proj_s.distance,
                        proj_e.distance,
                        sec_to_arc,
                        arc_to_sec,
                        span_band,
                    );
                }
                // Lower floor is the crux of the identity discrimination: only
                // reuse when the section deviates from the sub-arc by MORE than
                // the weld radius — a genuine GRAZE the assembler's geometric
                // weld cannot fuse (fixture 09: 8.2e-4/1.67e-3). A section
                // ALREADY within the weld radius of a boundary edge is a
                // coincident / cosurface boundary-exchange copy (a section that
                // IS a boundary edge, deviation ~1e-15..1e-6); the assembler
                // welds those as-is, and dropping their cut would strand the
                // coincident face's split (revolve_pole_union one-use edges).
                let span_dev = sec_to_arc.max(arc_to_sec);
                if span_dev > endpoint_gate && span_dev <= span_band {
                    if !redundant.contains(face_key) {
                        redundant.push(*face_key);
                    }
                    if span_dev < best_score {
                        best_score = span_dev;
                        chosen = Some(Reuse {
                            piece_index,
                            owning: *face_key,
                            edge_id: edge.id,
                            arc,
                            aligned,
                            redundant: Vec::new(),
                        });
                    }
                }
            }
        }
        if let Some(mut reuse) = chosen {
            // Escape hatch BREP_SHARED_SECTION_BOTH=0: drop the cut from the
            // owning face only, as before the two-sided case was handled.
            reuse.redundant = if std::env::var("BREP_SHARED_SECTION_BOTH").as_deref() == Ok("0") {
                vec![reuse.owning]
            } else {
                redundant
            };
            decisions.push(reuse);
        }
    }

    // Apply: swap the section's geometry for the reused sub-arc, recompute the
    // pcurves on the faces that are still cut, drop the redundant cut from
    // every face whose own boundary already carries the section.
    // Any failure in this path aborts this one reuse and keeps today's behaviour.
    for reuse in decisions {
        let piece = &result.pieces[reuse.piece_index];
        let mut new_pcurves = Vec::with_capacity(piece.pcurves.len());
        let mut ok = true;
        for facepc in &piece.pcurves {
            let key = FaceKey {
                operand: facepc.operand,
                face_id: facepc.face_id,
            };
            if reuse.redundant.contains(&key) {
                continue; // dropped — this face already carries the edge
            }
            let Some(surface) = face_surface(&key) else {
                ok = false;
                break;
            };
            // Reusing an imported boundary must not introduce that boundary's
            // residual onto a different support carrier. Its owning face may
            // inherit an import error; a newly cut face may not borrow it.
            if std::env::var("BREP_SHARED_SECTION_CARRIER_FLOOR").as_deref() != Ok("0") {
                let [lo, hi] = reuse.arc.domain().or_refuse(KernelStage::Intersect, "domain")?;
                let mut off = 0.0f64;
                // Include each knot span so a short span is not skipped by a
                // whole-domain grid. This is a sampled residual check, like the
                // existing bidirectional section/arc comparison above.
                for knots in reuse.arc.knots.windows(2) {
                    let a = knots[0].max(lo);
                    let b = knots[1].min(hi);
                    if b <= a {
                        continue;
                    }
                    for station in 0..=8 {
                        let point = reuse.arc.evaluate(a + (b - a) * station as f64 / 8.0)
                            .or_refuse(KernelStage::Intersect, "evaluate")?;
                        off = off.max(project_point_to_surface(surface, point)
                            .or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
                    }
                }
                if debug {
                    eprintln!("shared-section CARRIER piece {} face {}:{} off={off:.3e}", piece.id, key.operand, key.face_id);
                }
                if off > tolerance.max(1e-7) {
                    ok = false;
                    break;
                }
            }
            match build_pcurve_on_surface(surface, &reuse.arc) {
                Ok(pcurve) => new_pcurves.push(FacePcurve {
                    operand: key.operand,
                    face_id: key.face_id,
                    pcurve,
                }),
                Err(_) => {
                    ok = false;
                    break;
                }
            }
        }
        let Ok([nt0, nt1]) = reuse.arc.domain() else {
            continue;
        };
        // An empty kept set is legitimate ONLY in the two-sided case, where
        // every support face already carries the section as its own boundary
        // and none of them is cut. Anywhere else it means a pcurve rebuild
        // produced nothing, which is a failure.
        let two_sided = !piece.pcurves.is_empty()
            && piece.pcurves.iter().all(|facepc| {
                reuse.redundant.contains(&FaceKey {
                    operand: facepc.operand,
                    face_id: facepc.face_id,
                })
            });
        if !ok || (new_pcurves.is_empty() && !two_sided) {
            if debug {
                eprintln!(
                    "shared-section SKIP piece {} (pcurve rebuild on kept face failed)",
                    result.pieces[reuse.piece_index].id
                );
            }
            continue;
        }
        let piece = &mut result.pieces[reuse.piece_index];
        let piece_id = piece.id;
        piece.curve = reuse.arc;
        piece.t0 = nt0;
        piece.t1 = nt1;
        piece.pcurves = new_pcurves;
        piece.shared_edge = Some((reuse.owning.operand, reuse.edge_id, reuse.aligned));
        for key in &reuse.redundant {
            if let Some(entry) = result
                .by_face
                .iter_mut()
                .find(|f| f.operand == key.operand && f.face_id == key.face_id)
            {
                entry.piece_ids.retain(|id| *id != piece_id);
            }
        }
        if debug {
            eprintln!(
                "shared-section REUSE piece {} -> boundary edge {}:{} sub-arc \
                 aligned={} (dropped redundant cut from {})",
                piece_id,
                reuse.owning.operand,
                reuse.edge_id,
                reuse.aligned,
                reuse
                    .redundant
                    .iter()
                    .map(|key| format!("face {}:{}", key.operand, key.face_id))
                    .collect::<Vec<_>>()
                    .join(", "),
            );
        }
    }
    Ok(())
}

/// Canonical unordered key for a support pair.
fn support_pair_key(a: FaceKey, b: FaceKey) -> (FaceKey, FaceKey) {
    if (a.operand, a.face_id) <= (b.operand, b.face_id) {
        (a, b)
    } else {
        (b, a)
    }
}

/// The worst distance from the chord `o`→`d` to `surface`, over stations along
/// the whole span. The bridge's own gate reads ONE station (the midpoint); a
/// chord that replaces a section has to run on the carrier everywhere, and on a
/// carrier whose curvature is not symmetric about the midpoint the worst
/// station is not the middle one.
pub(super) fn chord_off_carrier(
    surface: &NurbsSurface,
    o: Vec3,
    d: Vec3,
) -> Result<f64, KernelRefusal> {
    const STATIONS: usize = 32;
    let mut worst = 0.0f64;
    for station in 0..=STATIONS {
        let fraction = station as f64 / STATIONS as f64;
        let point = o.add(d.sub(o).scale(fraction));
        worst = worst.max(
            project_point_to_surface(surface, point)
                .or_refuse(KernelStage::Intersect, "project_point_to_surface")?
                .distance,
        );
    }
    Ok(worst)
}

/// A face's trim window as the march reads it: the pcurve control hull against
/// the carrier's domain, and the overhang the window's clamp discards on a
/// closed direction (census only).
fn describe_trim_window(face: &FaceRecord) -> String {
    let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return "?".into();
    };
    let Ok((closed_u, closed_v)) = face.surface.closed_directions() else {
        return "?".into();
    };
    let mut low = [f64::INFINITY; 2];
    let mut high = [f64::NEG_INFINITY; 2];
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        for control in &coedge.pcurve.control_points {
            let Ok(point) = control.point() else {
                return "?".into();
            };
            low[0] = low[0].min(point.x);
            high[0] = high[0].max(point.x);
            low[1] = low[1].min(point.y);
            high[1] = high[1].max(point.y);
        }
    }
    let mut parts = Vec::new();
    for (axis, closed, domain) in [(0usize, closed_u, [u0, u1]), (1, closed_v, [v0, v1])] {
        let below = (domain[0] - low[axis]).max(0.0);
        let above = (high[axis] - domain[1]).max(0.0);
        if !closed || (below <= 0.0 && above <= 0.0) {
            continue;
        }
        parts.push(format!(
            "{}overhang[{:.3e},{:.3e}]{}",
            if axis == 0 { "u" } else { "v" },
            below,
            above,
            if high[axis] - low[axis] < domain[1] - domain[0] {
                "discarded"
            } else {
                "full"
            }
        ));
    }
    if parts.is_empty() {
        "in-domain".into()
    } else {
        parts.join(",")
    }
}

/// TRUNCATED-SECTION EXTENSION (near-tangent SSI under-capture rescue).
///
/// Where a face F grazes another face G near-tangentially, the SSI marcher can
/// terminate F∩G's section curve at an INTERIOR turning point instead of the
/// crossing vertex where F's trim boundary actually meets G — the section piece
/// is left ~a-percent short of the corner. The stranded stub cannot separate
/// F's domain (a cut that reaches the boundary at only one end leaves the face
/// whole), and on G's side the cross-section loop gaps exactly there, so both
/// operands' fragments strand their neighbours' shared section edges as one-use
/// (non-integral assembly genus). The gap sits in the `[arr_tol, junction_
/// radius]` band the per-face bridges cannot close, and because F and G index
/// the SAME piece, healing it once at the imprint fixes both.
///
/// Add ONE bridging section piece from the dangling endpoint D to the orphaned
/// crossing vertex O, supported on the same pair {F, G}. Gate: D is a valence-1
/// piece endpoint interior to F (not at any crossing), O is a valence-1 vertex
/// ON F's trim boundary that carries NO {F,G} section endpoint, both share G,
/// within a span tied to the stub's own length, and the chord's midpoint sits
/// on F. A closed section loop has no such orphaned crossing, so the gate never
/// trips a face the marcher already reconstructed. Escape hatch
/// `BREP_EXTEND_TRUNCATED_SECTIONS=0`.
pub(super) fn extend_truncated_sections(
    result: &mut ImprintResultRecord,
    face_edge_lists: &HashMap<FaceKey, Vec<&EdgeRecord>>,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    if std::env::var("BREP_EXTEND_TRUNCATED_SECTIONS").as_deref() == Ok("0") {
        return Ok(0);
    }
    let debug = std::env::var("BREP_DEBUG_EXTEND_SECTIONS").is_ok();

    let vpoint: HashMap<u64, Vec3> = result.vertices.iter().map(|v| (v.id, v.point)).collect();
    // Section-endpoint valence and each unordered support pair's endpoint set.
    let mut valence: HashMap<u64, usize> = HashMap::default();
    let mut pair_endpoints: HashMap<(FaceKey, FaceKey), HashSet<u64>> = HashMap::default();
    for piece in &result.pieces {
        for vid in [piece.start_vertex_id, piece.end_vertex_id] {
            *valence.entry(vid).or_insert(0) += 1;
        }
        let entry = pair_endpoints
            .entry(support_pair_key(piece.support_faces[0], piece.support_faces[1]))
            .or_default();
        entry.insert(piece.start_vertex_id);
        entry.insert(piece.end_vertex_id);
    }

    let surface_of = |key: FaceKey| -> Option<&NurbsSurface> {
        let solid = if key.operand == 0 { solid_a } else { solid_b };
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == key.face_id)
            .map(|face| chart_of(charts, key, face))
    };
    // "On F's trim boundary" uses the same clearance polish_endpoint snaps
    // section termini to, so a genuine crossing (~tolerance*100 off) reads On
    // while a marcher truncation (a measurable stub short) reads Off.
    let on_boundary = |pt: Vec3, fkey: FaceKey| -> Result<bool, KernelRefusal> {
        let tol = (tolerance * 100.0).max(1e-6) * (1.0 + pt.length());
        if let Some(edges) = face_edge_lists.get(&fkey) {
            for edge in edges {
                if edge.degenerate {
                    continue;
                }
                if project_point_to_curve(&edge.curve, pt).or_refuse(KernelStage::Intersect, "project_point_to_curve")?.distance <= tol {
                    return Ok(true);
                }
            }
        }
        Ok(false)
    };

    let face_of = |key: FaceKey| -> Option<&FaceRecord> {
        let solid = if key.operand == 0 { solid_a } else { solid_b };
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == key.face_id)
    };

    struct BridgeSpec {
        f: FaceKey,
        g: FaceKey,
        o: u64,
        d: u64,
        d_param: Vec3,
    }
    let mut bridges: Vec<BridgeSpec> = Vec::new();
    let mut claimed: HashSet<(u64, u64)> = HashSet::default();

    for piece in &result.pieces {
        let support = piece.support_faces;
        for &(f, g) in &[(support[0], support[1]), (support[1], support[0])] {
            let fg = support_pair_key(f, g);
            for (d, d_is_start) in [(piece.start_vertex_id, true), (piece.end_vertex_id, false)] {
                if valence.get(&d).copied().unwrap_or(0) != 1 {
                    continue;
                }
                let d_pt = vpoint[&d];
                // A dangling stub is interior to BOTH support faces — a genuine
                // terminus sits on a trim boundary.
                if on_boundary(d_pt, f)? || on_boundary(d_pt, g)? {
                    if debug {
                        eprintln!(
                            "extend: dangler v{d} F={}:{} G={}:{} REJECT on-boundary(f={},g={})",
                            f.operand,
                            f.face_id,
                            g.operand,
                            g.face_id,
                            on_boundary(d_pt, f)?,
                            on_boundary(d_pt, g)?
                        );
                    }
                    continue;
                }
                let plen = piece.curve.evaluate(piece.t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(piece.curve.evaluate(piece.t1).or_refuse(KernelStage::Intersect, "evaluate")?).length();
                let span_bound = (plen * 5.0).max(tolerance * 100.0);
                // Nearest orphaned crossing vertex on F's boundary that shares G.
                let mut best: Option<(f64, u64)> = None;
                for other in &result.pieces {
                    if other.support_faces[0] != g && other.support_faces[1] != g {
                        continue;
                    }
                    for o in [other.start_vertex_id, other.end_vertex_id] {
                        if o == d {
                            continue;
                        }
                        if pair_endpoints.get(&fg).is_some_and(|set| set.contains(&o)) {
                            continue; // already a {F,G} terminus — not orphaned
                        }
                        let o_pt = vpoint[&o];
                        let gap = o_pt.sub(d_pt).length();
                        if gap <= tolerance * 10.0 || gap > span_bound {
                            continue;
                        }
                        if !on_boundary(o_pt, f)? {
                            continue;
                        }
                        if best.is_none_or(|(g0, _)| gap < g0) {
                            best = Some((gap, o));
                        }
                    }
                }
                let Some((gap, o)) = best else {
                    if debug {
                        eprintln!(
                            "extend: dangler v{d} F={}:{} G={}:{} REJECT no-orphan (span_bound={span_bound:.3e})",
                            f.operand, f.face_id, g.operand, g.face_id
                        );
                    }
                    continue;
                };
                if claimed.contains(&(o, d)) || claimed.contains(&(d, o)) {
                    continue;
                }
                // THE BRIDGE IS A SECTION, AND A SECTION LIES ON BOTH CARRIERS.
                // The gate this replaces read ONE station — the chord's
                // midpoint, against F only, inside a band of max(100 tol, 0.25
                // gap)·(1 + |mid|), about 1.5e-3 on the helmet: it admitted a
                // 6.05e-3 chord standing 7.874e-4 off Face_15, 430 times the
                // section trim floor, and the fit reported the chord's residual
                // against its own polyline rather than the carrier's, so no
                // floor downstream caught it. A bridge is now measured along
                // its WHOLE span against BOTH carriers and accepted only inside
                // the fit tolerance a marched section is held to. Past that the
                // chord is not the section, and the imprint refuses by name
                // rather than minting a wrong one: the stranded stub's own
                // refusal (an open assembly) would name the topology and not
                // the geometry that caused it.
                let (Some(fsurf), Some(gsurf)) = (surface_of(f), surface_of(g)) else {
                    continue;
                };
                let o_pt = vpoint[&o];
                let off_f = chord_off_carrier(fsurf, o_pt, d_pt)?;
                let off_g = chord_off_carrier(gsurf, o_pt, d_pt)?;
                // The fit tolerance, as `build_imprints` passes it to the
                // section fit: `options.tolerance.max(1e-7)`.
                let fit_tolerance = tolerance.max(1e-7);
                if off_f.max(off_g) > fit_tolerance {
                    return Err(KernelRefusal::new(
                        RefusalClass::NonConvergence {
                            what: "truncated section bridge".into(),
                        },
                        KernelStage::Intersect,
                        format!(
                            "boolean: the section of faces {}:{} and {}:{} is truncated at \
                             ({:.6}, {:.6}, {:.6}), {gap:.3e} short of the crossing at \
                             ({:.6}, {:.6}, {:.6}), and the chord that would bridge it stands \
                             {:.3e} off face {}'s carrier and {:.3e} off face {}'s, against the \
                             fit tolerance {fit_tolerance:.1e} every marched section is held to. \
                             A chord that is not on both carriers is a wrong section, not a \
                             loose one.",
                            f.operand,
                            f.face_id,
                            g.operand,
                            g.face_id,
                            d_pt.x,
                            d_pt.y,
                            d_pt.z,
                            o_pt.x,
                            o_pt.y,
                            o_pt.z,
                            off_f,
                            f.face_id,
                            off_g,
                            g.face_id,
                        ),
                    ));
                }
                // D's parameter on F, read from the stub's own F pcurve.
                let f_pcurve = pcurve_for_piece(piece, f)?;
                let [pd0, pd1] = f_pcurve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                let d_param = f_pcurve.evaluate(if d_is_start { pd0 } else { pd1 }).or_refuse(KernelStage::Intersect, "evaluate")?;
                claimed.insert((o, d));
                if debug {
                    // CENSUS: the chord that was accepted, against the bars it
                    // cleared and the two faces' march windows — the population
                    // the overhang slice measured before it had a gate.
                    eprintln!(
                        "extend: bridge F={}:{} G={}:{} d={d} o={o} gap={gap:.3e} off_f={off_f:.3e} off_g={off_g:.3e} fit={fit_tolerance:.1e} f_window={} g_window={}",
                        f.operand,
                        f.face_id,
                        g.operand,
                        g.face_id,
                        face_of(f).map(describe_trim_window).unwrap_or_default(),
                        face_of(g).map(describe_trim_window).unwrap_or_default(),
                    );
                }
                bridges.push(BridgeSpec { f, g, o, d, d_param });
            }
        }
    }

    if bridges.is_empty() {
        return Ok(0);
    }
    let mut next_id = result
        .vertices
        .iter()
        .map(|v| v.id)
        .chain(result.pieces.iter().map(|p| p.id))
        .max()
        .unwrap_or(0)
        + 1;
    let mut added = 0usize;
    for spec in bridges {
        let o_pt = vpoint[&spec.o];
        let d_pt = vpoint[&spec.d];
        let (Some(fsurf), Some(gsurf)) = (surface_of(spec.f), surface_of(spec.g)) else {
            continue;
        };
        let line = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Vec4::from_point(o_pt, 1.0), Vec4::from_point(d_pt, 1.0)],
        ).or_refuse(KernelStage::Intersect, "csg.imprint.sections")?;
        // G pcurve: exact for an affine carrier (the common cutting plane),
        // sampled-and-fit otherwise.
        let g_pcurve = build_pcurve_on_surface(gsurf, &line).or_refuse(KernelStage::Intersect, "build_pcurve_on_surface")?;
        // F pcurve: a param-space chord whose O end is projected SEEDED from the
        // D end, so a seam-straddling F cannot flip O onto the wrong branch.
        let o_proj = project_point_to_surface_seeded(fsurf, o_pt, spec.d_param.x, spec.d_param.y).or_refuse(KernelStage::Intersect, "project_point_to_surface_seeded")?;
        // On a seam-closed F, the projection can return O on the far seam copy
        // (u+period) while D sits on the near one; a param-space chord between
        // them would cross the seam and the arrangement then splits D onto the
        // opposite side, disconnecting the bridge from the stub it extends.
        // Unwrap O onto the seam representative NEAREST D so the chord stays on
        // one sheet and its ends coincide with the stub and boundary chains.
        let (closed_u, closed_v) = fsurf.closed_directions().or_refuse(KernelStage::Intersect, "closed_directions")?;
        let [fu0, fu1] = fsurf.domain_u().or_refuse(KernelStage::Intersect, "domain_u")?;
        let [fv0, fv1] = fsurf.domain_v().or_refuse(KernelStage::Intersect, "domain_v")?;
        let unwrap = |value: f64, target: f64, lo: f64, hi: f64, closed: bool| {
            if !closed {
                return value;
            }
            let period = hi - lo;
            if period <= 0.0 {
                return value;
            }
            let mut result = value;
            while result - target > period * 0.5 {
                result -= period;
            }
            while target - result > period * 0.5 {
                result += period;
            }
            result
        };
        let o_u = unwrap(o_proj.u, spec.d_param.x, fu0, fu1, closed_u);
        let o_v = unwrap(o_proj.v, spec.d_param.y, fv0, fv1, closed_v);
        if std::env::var("BREP_DEBUG_EXTEND_SECTIONS").is_ok() {
            eprintln!(
                "extend apply: F={}:{} d_param=({:.4},{:.4}) o_param=({:.4},{:.4},dist={:.2e})",
                spec.f.operand, spec.f.face_id, spec.d_param.x, spec.d_param.y,
                o_u, o_v, o_proj.distance
            );
        }
        let f_pcurve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                Vec4::from_point(Vec3::new(o_u, o_v, 0.0), 1.0),
                Vec4::from_point(Vec3::new(spec.d_param.x, spec.d_param.y, 0.0), 1.0),
            ],
        ).or_refuse(KernelStage::Intersect, "csg.imprint.sections")?;
        let id = next_id;
        next_id += 1;
        // operand-0 face first, matching the freshly-minted-piece convention.
        let (support_faces, pcurves) = if spec.f.operand <= spec.g.operand {
            (
                [spec.f, spec.g],
                vec![
                    FacePcurve { operand: spec.f.operand, face_id: spec.f.face_id, pcurve: f_pcurve },
                    FacePcurve { operand: spec.g.operand, face_id: spec.g.face_id, pcurve: g_pcurve },
                ],
            )
        } else {
            (
                [spec.g, spec.f],
                vec![
                    FacePcurve { operand: spec.g.operand, face_id: spec.g.face_id, pcurve: g_pcurve },
                    FacePcurve { operand: spec.f.operand, face_id: spec.f.face_id, pcurve: f_pcurve },
                ],
            )
        };
        result.pieces.push(ImprintPieceRecord {
            id,
            curve: line,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: spec.o,
            end_vertex_id: spec.d,
            pcurves,
            support_faces,
            shared_edge: None,
        });
        for key in [spec.f, spec.g] {
            match result.by_face.iter_mut().find(|record| {
                record.operand == key.operand && record.face_id == key.face_id
            }) {
                Some(record) => record.piece_ids.push(id),
                None => result.by_face.push(FaceImprints {
                    operand: key.operand,
                    face_id: key.face_id,
                    piece_ids: vec![id],
                }),
            }
        }
        added += 1;
    }
    Ok(added)
}

fn pcurve_for_piece(piece: &ImprintPieceRecord, key: FaceKey) -> Result<&NurbsCurve, KernelRefusal> {
    piece
        .pcurves
        .iter()
        .find(|pcurve| pcurve.operand == key.operand && pcurve.face_id == key.face_id)
        .map(|pcurve| &pcurve.pcurve)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.sections", "extend_truncated_sections: piece lacks face pcurve"))
}

/// Dissolve a marched section's own PARAMETERIZATION ORIGIN.
///
/// `intersect_surfaces` returns a closed rim as one branch whose first and last
/// samples are the same point, and that point is wherever the trace happened to
/// start — the first unclaimed grid start that refined, or a pierce seed when
/// none did. `process_curve` then cuts the curve at each crossing it finds and
/// pushes the remainder, so a CYCLE with k crossings arrives as k+1 arcs: the
/// origin stands as a break that no operand edge passes through. A hole drilled
/// through a torus, a general revolution or a fitted wall therefore reached
/// `delete_face_and_heal` as a five-coedge strip and was gated out before any
/// heal ran, and the flat pattern saw several rims where there is one.
///
/// The decision cannot be made inside `process_curve`, and that is the whole
/// reason this is a post-pass. The same physical section is minted by SEVERAL
/// pairs — a cosurface boundary copy and the analytic ring of the same circle,
/// a rim carried by two halves of a split skin — and each call sees only its
/// own `split_faces`, so one call can close a cycle while another still cuts
/// it. A closed record and an open-arc record can never endpoint-weld, and the
/// result is the one-use / non-integral-genus signature the OVERLAP-JUNCTION
/// EXCHANGE already exists to prevent (measured: the rotated equator-tangency
/// subtract goes from clean to `V=3 E=4 F=2 S=1 one_use=2`). Deciding it HERE,
/// once, over the finished piece set makes every pair carrying a vertex agree
/// about it.
///
/// The rule is a statement about the VERTEX, not about any one curve: a section
/// vertex that lies on NO operand edge of either solid is not a junction of
/// anything — nothing meets there — so where exactly two section pieces of the
/// same support pair end on it, they are two arcs of one curve and it is the
/// cut the marcher's parameterization left behind. Joining them is exact
/// homogeneous concatenation (`concatenate_exact_curve_pieces`, which verifies
/// the join against both originals by sampling and declines rather than
/// approximate), and their trims are rebuilt from the joined curve on the same
/// faces. A vertex that IS on an operand edge stays, whatever it is: a seam
/// crossing, a pole, a face boundary, a ridden ring's pave. Escape hatch
/// `BREP_SECTION_ORIGIN_DISSOLVE=0`.
pub(super) fn dissolve_section_origin_vertices(
    result: &mut ImprintResultRecord,
    marched_pieces: &HashSet<u64>,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    if std::env::var("BREP_SECTION_ORIGIN_DISSOLVE").as_deref() == Ok("0") {
        return Ok(0);
    }
    // An imprint that admitted a TANGENT NODE is left exactly as the march cut
    // it. Not because its origin vertices are anything else — on the equal-radius
    // torus pair they sit 0.75 and 3.15 from the nearest node, plain
    // parameterization cuts — but because dissolving them changes that lane's
    // verdict and the verdict is not this pass's to make. Measured with the pass
    // applied there: both torus∪torus and torus−torus stop refusing and build,
    // at 299.473659929 and 121.820780771 against an independent lens integral of
    // 299.4736604 and 121.8207812; the union's mesh is closed with one component,
    // and the difference's mesh carries a four-use edge at three of the four
    // nodes, which is the zero-thickness pinch the boolean refuses everywhere
    // else as unrepresentable. The assembly is the authority on whether a node
    // was imprintable (`ImprintResultRecord::tangent_nodes`), so the change of
    // verdict belongs to the node lane with these numbers in hand.
    if !result.tangent_nodes.is_empty() {
        return Ok(0);
    }
    let debug = std::env::var("BREP_DEBUG_ORIGIN_DISSOLVE").is_ok();
    // The same band `process_curve` calls the meaningful noise floor for
    // "this point is on that edge": anything finer cannot survive as distinct
    // topology anyway.
    let band = tolerance.max(COINCIDENCE_DISTANCE_FLOOR).max(assembler_weld(tolerance));
    let vertex_point: HashMap<u64, Vec3> =
        result.vertices.iter().map(|v| (v.id, v.point)).collect();

    // Which section vertices sit on an operand edge? Asked only of a vertex that
    // is otherwise a candidate, memoized, and prefiltered by each live edge's
    // control-hull box (a positive-weight rational curve lies in its hull), so
    // the pass costs nothing on an imprint with no marched cycle in it.
    let mut edge_boxes: Vec<(&EdgeRecord, Vec3, Vec3)> = Vec::new();
    for solid in [solid_a, solid_b] {
        for edge in &solid.edges {
            if edge.degenerate {
                continue;
            }
            let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            let mut hull_ok = true;
            for control in &edge.curve.control_points {
                if control.w <= 0.0 {
                    hull_ok = false;
                    break;
                }
                let point = Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w);
                low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
                high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
            }
            if !hull_ok {
                // No hull guarantee: never prefilter this edge out.
                low = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                high = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            }
            edge_boxes.push((edge, low, high));
        }
    }
    let lies_on_operand_edge = |point: Vec3| -> Result<bool, KernelRefusal> {
        for (edge, low, high) in &edge_boxes {
            if point.x < low.x - band
                || point.y < low.y - band
                || point.z < low.z - band
                || point.x > high.x + band
                || point.y > high.y + band
                || point.z > high.z + band
            {
                continue;
            }
            let projection = project_point_to_curve(&edge.curve, point)
                .or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
            let nearest = edge
                .curve
                .evaluate(projection.u.clamp(edge.t0.min(edge.t1), edge.t0.max(edge.t1)))
                .or_refuse(KernelStage::Intersect, "evaluate")?;
            if nearest.sub(point).length() <= band {
                return Ok(true);
            }
        }
        Ok(false)
    };
    let mut on_operand_edge: HashMap<u64, bool> = HashMap::default();

    let mut dissolved = 0usize;
    loop {
        // Per support pair, the pieces ending on each vertex. A vertex is only
        // a parameterization cut when EVERY pair that carries it carries
        // exactly two pieces there — otherwise it is a junction of something.
        let mut incident: HashMap<(FaceKey, FaceKey, u64), Vec<(usize, bool)>> = HashMap::default();
        for (index, piece) in result.pieces.iter().enumerate() {
            if piece.start_vertex_id == piece.end_vertex_id {
                continue; // already a closed ring
            }
            let key = support_pair_key(piece.support_faces[0], piece.support_faces[1]);
            incident
                .entry((key.0, key.1, piece.start_vertex_id))
                .or_default()
                .push((index, true));
            incident
                .entry((key.0, key.1, piece.end_vertex_id))
                .or_default()
                .push((index, false));
        }
        let mut pairs_at: HashMap<u64, usize> = HashMap::default();
        for &(_, _, vertex) in incident.keys() {
            *pairs_at.entry(vertex).or_default() += 1;
        }
        // Deterministic order: the lowest vertex id, then the lowest piece id.
        let mut candidates: Vec<(u64, FaceKey, FaceKey, usize, usize)> = Vec::new();
        for (&(first, second, vertex), slots) in &incident {
            if pairs_at.get(&vertex).copied().unwrap_or(0) != 1 {
                continue; // a meeting of two pairs' sections is a junction
            }
            if slots.len() != 2 {
                continue;
            }
            // Oriented head-to-tail: one piece ENDS here and the other STARTS
            // here. Anything else would need a reversal, which would have to
            // reverse the trims too; a cycle the marcher cut never produces it.
            let (a, b) = match (slots[0], slots[1]) {
                ((left, false), (right, true)) => (left, right),
                ((left, true), (right, false)) => (right, left),
                _ => continue,
            };
            if a == b {
                continue;
            }
            // MARCHED arcs only. An exact analytic ring (a plane through a
            // sphere) is cut at its frame origin too, but closing it here moved
            // AnotherOffsetShellProblem's z = 1.5 carve-plane corners AWAY from
            // their closed form — area error 1.18e-4 -> 3.24e-4 — where the
            // coalescer's exact band closes it downstream as it always has.
            if !marched_pieces.contains(&result.pieces[a].id)
                || !marched_pieces.contains(&result.pieces[b].id)
            {
                continue;
            }
            candidates.push((vertex, first, second, a, b));
        }
        // Last and dearest: is anything of either operand passing through it?
        // A support face's carrier seam the two arcs cross counts as something:
        // the split there (`process_curve`'s carrier-seam pass) is what keeps
        // each arc's trim on its analytic carrier's period, and the joined trim
        // would cross the branch cut. Crossing means the arcs' middles stand on
        // opposite sides of the seam's plane, each clear of it by the band — a
        // section riding in that plane crosses nothing.
        let mut kept = Vec::with_capacity(candidates.len());
        for candidate in candidates {
            let vertex = candidate.0;
            let point = vertex_point.get(&vertex).copied().ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Intersect,
                    "imprint.sections",
                    "dissolve_section_origin_vertices: piece names an unknown vertex",
                )
            })?;
            let mut on_carrier_seam = false;
            for key in [candidate.1, candidate.2] {
                let solid = if key.operand == 0 { solid_a } else { solid_b };
                let Some(face) = solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .find(|face| face.id == key.face_id)
                else {
                    continue;
                };
                for seam in CarrierSeam::of(&face.surface)? {
                    if seam.offset(point).abs() > band || !seam.on_seam_half(point) {
                        continue;
                    }
                    let middle_offset = |index: usize| -> Result<f64, KernelRefusal> {
                        let curve = &result.pieces[index].curve;
                        let [t0, t1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                        Ok(seam.offset(
                            curve
                                .evaluate(0.5 * (t0 + t1))
                                .or_refuse(KernelStage::Intersect, "evaluate")?,
                        ))
                    };
                    let (before, after) = (middle_offset(candidate.3)?, middle_offset(candidate.4)?);
                    if before.abs() > band && after.abs() > band && (before < 0.0) != (after < 0.0) {
                        on_carrier_seam = true;
                    }
                }
            }
            if on_carrier_seam {
                continue;
            }
            let on_edge = match on_operand_edge.get(&vertex) {
                Some(known) => *known,
                None => {
                    let point = vertex_point.get(&vertex).copied().ok_or_else(|| {
                        KernelRefusal::internal(
                            KernelStage::Intersect,
                            "imprint.sections",
                            "dissolve_section_origin_vertices: piece names an unknown vertex",
                        )
                    })?;
                    let known = lies_on_operand_edge(point)?;
                    on_operand_edge.insert(vertex, known);
                    known
                }
            };
            if !on_edge {
                kept.push(candidate);
            }
        }
        let mut candidates = kept;
        candidates.sort_by_key(|(vertex, _, _, a, b)| {
            (*vertex, result.pieces[*a].id, result.pieces[*b].id)
        });
        let Some(&(vertex, _, _, a, b)) = candidates.first() else {
            break;
        };
        // One join per sweep: the indices above are invalidated by the removal.
        let joined = join_section_pieces(result, a, b, solid_a, solid_b, charts, tolerance)?;
        if !joined {
            // Record the refusal so a declined concatenation is visible, and
            // stop rather than spinning on the same pair.
            if debug {
                eprintln!(
                    "origin-dissolve: vertex {vertex} declined by concatenate_exact_curve_pieces"
                );
            }
            on_operand_edge.insert(vertex, true);
            continue;
        }
        if debug {
            let point = vertex_point.get(&vertex).copied().unwrap_or_default();
            let node = result
                .tangent_nodes
                .iter()
                .map(|node| node.sub(point).length())
                .fold(f64::INFINITY, f64::min);
            eprintln!(
                "origin-dissolve: vertex {vertex} at ({:.6},{:.6},{:.6}) dissolved \
                 (nearest tangent node {node:.3e})",
                point.x, point.y, point.z
            );
        }
        dissolved += 1;
    }
    if dissolved > 0 {
        // A dissolved vertex is referenced by nothing; leaving it would mint a
        // stray topological vertex downstream.
        let mut referenced: HashSet<u64> = HashSet::default();
        for piece in &result.pieces {
            referenced.insert(piece.start_vertex_id);
            referenced.insert(piece.end_vertex_id);
        }
        result.vertices.retain(|v| referenced.contains(&v.id));
    }
    Ok(dissolved)
}

/// Join section pieces `a` (which ends at the shared vertex) and `b` (which
/// starts there) into one, keeping `a`'s id. Returns false — changing nothing —
/// when the exact concatenation declines or the two pieces do not carry the
/// same trims.
fn join_section_pieces(
    result: &mut ImprintResultRecord,
    a: usize,
    b: usize,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    if result.pieces[a].shared_edge.is_some() || result.pieces[b].shared_edge.is_some() {
        return Ok(false); // riding an existing boundary edge: not ours to join
    }
    let mut a_faces: Vec<(u8, u64)> = result.pieces[a]
        .pcurves
        .iter()
        .map(|p| (p.operand, p.face_id))
        .collect();
    let mut b_faces: Vec<(u8, u64)> = result.pieces[b]
        .pcurves
        .iter()
        .map(|p| (p.operand, p.face_id))
        .collect();
    a_faces.sort_unstable();
    b_faces.sort_unstable();
    if a_faces != b_faces {
        return Ok(false); // different trims: not two arcs of one rim
    }
    let [a0, a1] = result.pieces[a]
        .curve
        .domain()
        .or_refuse(KernelStage::Intersect, "domain")?;
    let [b0, b1] = result.pieces[b]
        .curve
        .domain()
        .or_refuse(KernelStage::Intersect, "domain")?;
    let (a_span, b_span) = (a1 - a0, b1 - b0);
    if a_span <= 0.0 || b_span <= 0.0 {
        return Ok(false);
    }
    let split = a_span / (a_span + b_span);
    let weld = assembler_weld(tolerance) * merge_scale(solid_scale(solid_a).max(solid_scale(solid_b)));
    let Some(curve) = crate::concatenate_exact_curve_pieces(
        &result.pieces[a].curve,
        &result.pieces[b].curve,
        split,
        weld,
    )
    .map_err(|refusal| KernelRefusal { stage: KernelStage::Intersect, ..refusal })?
    else {
        return Ok(false);
    };
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    // The trims are JOINED the way the curve is, not rebuilt from it. A rebuild
    // re-projects the whole loop, and a re-projected full period is where a trim
    // fit is worst: the coalescer's closing band was set on exactly that loss
    // (the crossing-pipe tee, 2.8e-7 from its closed form as two arcs, 1.7e-6
    // once its closure refit the trims). Each arc's pcurve is a linear image of
    // its own piece's domain, so the same split carries both into one, and the
    // joined trim is the two original trims to the bit. A rebuild stays the
    // fallback where the join declines.
    let mut pcurves = Vec::with_capacity(a_faces.len());
    for (operand, face_id) in &a_faces {
        let solid = if *operand == 0 { solid_a } else { solid_b };
        let Some(face) = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == *face_id)
        else {
            return Ok(false);
        };
        let find = |index: usize| {
            result.pieces[index]
                .pcurves
                .iter()
                .find(|p| p.operand == *operand && p.face_id == *face_id)
                .map(|p| p.pcurve.clone())
        };
        let chart = chart_of(charts, FaceKey { operand: *operand, face_id: *face_id }, face);
        let joined = match (find(a), find(b)) {
            (Some(first), Some(second)) => {
                join_trims(chart, &first, &second, split, &curve)?
            }
            _ => None,
        };
        let pcurve = match joined {
            Some(pcurve) => pcurve,
            None => match {
                if std::env::var("BREP_DEBUG_ORIGIN_DISSOLVE").is_ok() {
                    eprintln!(
                        "origin-dissolve: trim join declined on op{operand} face {face_id}; rebuilding"
                    );
                }
                build_pcurve_on_surface_marched(chart, &curve)
            } {
                Ok(pcurve) => pcurve,
                Err(_) => return Ok(false),
            },
        };
        // A merged rim may end up spanning a FULL PERIOD of a closed direction.
        // On a RULED REVOLUTION that is the ordinary state of every bore wall in
        // this kernel — its seam is a straight generator its own loop already
        // carries — and it is merged. On a DOUBLY-CURVED periodic carrier it is
        // not, measured on the torus collar (a torus welded through a box face,
        // whose collar runs once around the tube): merged, its trim reads back
        // at 3.7e-6 and the volume is unchanged, but the blender's closed-chain
        // lane then places the blend's support start where no neighbour edge
        // passes ("neighbour edge 15 does not pass through the blend's support
        // start (off by 0.1087)", `blend::tests::fold`, the corner-torus-collar
        // inbox suite), and the saved collar document's `…|P.T2_Side_1[1]`
        // reference names the arc that no longer exists. Both are consumers
        // reading the collar as the two arcs they were written against — the
        // seam-marker half of "one rim, one edge", owned by `blending/` and the
        // naming pass — so the arcs stay here until they read one closed edge.
        if spans_a_period_of_a_doubly_curved_carrier(&face.surface, &pcurve)? {
            return Ok(false);
        }
        pcurves.push(FacePcurve {
            operand: *operand,
            face_id: *face_id,
            pcurve,
        });
    }
    let absorbed = result.pieces[b].id;
    let keeper = result.pieces[a].id;
    let end_vertex = result.pieces[b].end_vertex_id;
    let piece = &mut result.pieces[a];
    piece.curve = curve;
    piece.t0 = t0;
    piece.t1 = t1;
    piece.end_vertex_id = end_vertex;
    piece.pcurves = pcurves;
    result.pieces.remove(b);
    for record in &mut result.by_face {
        if let Some(position) = record.piece_ids.iter().position(|&id| id == absorbed) {
            if record.piece_ids.contains(&keeper) {
                record.piece_ids.remove(position);
            } else {
                record.piece_ids[position] = keeper;
            }
        }
    }
    Ok(true)
}

/// Whether `pcurve` runs a full period of a CLOSED direction of `surface`, on a
/// carrier that is not a ruled revolution. See the call site: a full-period
/// trim on a cylinder is ordinary, on a torus or a sphere it is a rim with no
/// start on the seam.
fn spans_a_period_of_a_doubly_curved_carrier(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
) -> Result<bool, KernelRefusal> {
    if matches!(
        surface.analytic(),
        Some(crate::AnalyticSurface::RuledRevolution { .. })
    ) {
        return Ok(false);
    }
    let (closed_u, closed_v) = surface
        .closed_directions()
        .or_refuse(KernelStage::Intersect, "closed_directions")?;
    if !closed_u && !closed_v {
        return Ok(false);
    }
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Intersect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Intersect, "domain_v")?;
    let [t0, t1] = pcurve
        .domain()
        .or_refuse(KernelStage::Intersect, "domain")?;
    let (mut u_low, mut u_high) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_low, mut v_high) = (f64::INFINITY, f64::NEG_INFINITY);
    for step in 0..=64 {
        let uv = pcurve
            .evaluate(t0 + (t1 - t0) * step as f64 / 64.0)
            .or_refuse(KernelStage::Intersect, "evaluate")?;
        u_low = u_low.min(uv.x);
        u_high = u_high.max(uv.x);
        v_low = v_low.min(uv.y);
        v_high = v_high.max(uv.y);
    }
    // "A full period" with the same slack `restricted_carrier` uses to decide a
    // trim hull leaves a direction FULL.
    let spans = |low: f64, high: f64, start: f64, end: f64| {
        let period = end - start;
        period > 0.0 && (high - low) >= period * (1.0 - 1e-3)
    };
    Ok((closed_u && spans(u_low, u_high, u0, u1)) || (closed_v && spans(v_low, v_high, v0, v1)))
}

/// Join two arcs' trims on one face into the trim of their joined curve, or
/// `None` where that cannot be done exactly.
///
/// `second` is first moved by whole periods of the carrier's closed directions
/// so it starts on the branch `first` ends on (two arcs fitted separately may
/// sit a period apart), then the two are concatenated exactly at `split` — the
/// same split that joined the 3D arcs. The result is accepted only if it still
/// lies on the joined curve: at nine stations, `surface(trim(s))` must be as
/// close to `curve(s)` as the two original trims were to their own arcs, which
/// is what fails if a trim was ever not a linear image of its piece.
fn join_trims(
    surface: &NurbsSurface,
    first: &NurbsCurve,
    second: &NurbsCurve,
    split: f64,
    curve: &NurbsCurve,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    let (closed_u, closed_v) = surface
        .closed_directions()
        .or_refuse(KernelStage::Intersect, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Intersect, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Intersect, "domain_v")?;
    let [f0, f1] = first.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let [s0, _] = second.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let end = first.evaluate(f1).or_refuse(KernelStage::Intersect, "evaluate")?;
    let start = second.evaluate(s0).or_refuse(KernelStage::Intersect, "evaluate")?;
    let shift = |gap: f64, closed: bool, period: f64| {
        if closed && period > 0.0 {
            (gap / period).round() * period
        } else {
            0.0
        }
    };
    let du = shift(end.x - start.x, closed_u, u1 - u0);
    let dv = shift(end.y - start.y, closed_v, v1 - v0);
    let mut moved = second.clone();
    for control in &mut moved.control_points {
        control.x += du * control.w;
        control.y += dv * control.w;
    }
    let span = (u1 - u0).abs().max((v1 - v0).abs()).max(1.0);
    let Some(joined) = crate::concatenate_exact_curve_pieces(first, &moved, split, 1e-9 * span)
        .map_err(|refusal| KernelRefusal { stage: KernelStage::Intersect, ..refusal })?
    else {
        return Ok(None);
    };
    // The 3D/2D correspondence, measured on the result against the originals.
    let [c0, c1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let [j0, j1] = joined.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let off = |trim: &NurbsCurve, s: f64, at: f64| -> Result<f64, KernelRefusal> {
        let [d0, d1] = trim.domain().or_refuse(KernelStage::Intersect, "domain")?;
        let uv = trim
            .evaluate(d0 + (d1 - d0) * s)
            .or_refuse(KernelStage::Intersect, "evaluate")?;
        let on_surface = surface
            .evaluate(uv.x, uv.y)
            .or_refuse(KernelStage::Intersect, "evaluate")?;
        let on_curve = curve.evaluate(at).or_refuse(KernelStage::Intersect, "evaluate")?;
        Ok(on_surface.sub(on_curve).length())
    };
    let mut original = 0.0f64;
    let mut after = 0.0f64;
    for station in 1..10 {
        let fraction = station as f64 / 10.0;
        let at = c0 + (c1 - c0) * fraction;
        after = after.max(off(&joined, (j0 + (j1 - j0) * fraction - j0) / (j1 - j0), at)?);
        original = original.max(if fraction <= split {
            off(first, fraction / split, at)?
        } else {
            off(&moved, (fraction - split) / (1.0 - split), at)?
        });
    }
    if after > original * 1.5 + 1e-9 * span {
        return Ok(None);
    }
    Ok(Some(joined))
}

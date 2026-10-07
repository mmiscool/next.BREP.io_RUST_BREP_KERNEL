use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use super::builder::weld_refused_by_identity;

fn curve_subrange(
    curve: &crate::NurbsCurve,
    start_fraction: f64,
    end_fraction: f64,
) -> Result<crate::NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let first = start + (end - start) * start_fraction;
    let second = start + (end - start) * end_fraction;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if first > start + epsilon && first < end - epsilon {
        result = result.split(first).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Sew, "domain")?;
    // After the first split the domain START moved to `first`; a collapsed
    // subrange (second <= first) must not split again or NurbsCurve::split
    // errors with a parameter exactly at the new domain start.
    if second < domain[1] - epsilon && second > domain[0] + epsilon {
        result = result.split(second).or_refuse(KernelStage::Sew, "split")?.0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && second < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in curve_subrange");
    }
    Ok(result)
}

fn edge_fraction_at_point(
    edge: &EdgeRecord,
    point: Vec3,
    tolerance: f64,
) -> Result<Option<f64>, KernelRefusal> {
    let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
    if projection.distance > tolerance {
        return Ok(None);
    }
    let span = edge.t1 - edge.t0;
    if span <= 0.0 {
        return Ok(None);
    }
    let fraction = (projection.u - edge.t0) / span;
    Ok(((-1e-7..=1.0 + 1e-7).contains(&fraction)).then_some(fraction.clamp(0.0, 1.0)))
}

pub(super) fn edge_lies_on(piece: &EdgeRecord, carrier: &EdgeRecord, tolerance: f64) -> Result<bool, KernelRefusal> {
    for fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let point = piece
            .curve
            .evaluate(piece.t0 + (piece.t1 - piece.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        if edge_fraction_at_point(carrier, point, tolerance)?.is_none() {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Like [`edge_lies_on`] but samples only the INTERIOR (0.25/0.5/0.75), skipping
/// the 0.0/1.0 endpoints. For a pair whose endpoint VERTICES already coincide,
/// a curve endpoint that drifts a whisker off its own vertex projects just past
/// the carrier's [0,1] span and trips `edge_fraction_at_point`'s range guard —
/// even though the spans coincide. The interior samples avoid that boundary case
/// while still rejecting a chord paired with a bulged arc (its interior sagitta
/// exceeds tolerance).
pub(crate) fn edge_interior_lies_on(
    piece: &EdgeRecord,
    carrier: &EdgeRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    for fraction in [0.25, 0.5, 0.75] {
        let point = piece
            .curve
            .evaluate(piece.t0 + (piece.t1 - piece.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        if edge_fraction_at_point(carrier, point, tolerance)?.is_none() {
            return Ok(false);
        }
    }
    Ok(true)
}

fn pcurve_fraction_at_edge_fraction(
    face: &FaceRecord,
    coedge: &CoedgeRecord,
    edge: &EdgeRecord,
    edge_fraction: f64,
) -> Result<f64, KernelRefusal> {
    // Closed edges have the same spatial point at both parameter ends.
    // Preserve the named traversal endpoint instead of projecting it onto an
    // arbitrary preimage of that point on the closed pcurve.
    if edge_fraction == 0.0 || edge_fraction == 1.0 {
        return Ok(if coedge.forward { edge_fraction } else { 1.0 - edge_fraction });
    }
    let point = edge
        .curve
        .evaluate(edge.t0 + (edge.t1 - edge.t0) * edge_fraction).or_refuse(KernelStage::Sew, "evaluate")?;
    let surface_projection = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Sew, "project_point_to_surface")?;
    let pcurve_projection = project_point_to_curve(
        &coedge.pcurve,
        Vec3::new(surface_projection.u, surface_projection.v, 0.0),
    ).or_refuse(KernelStage::Sew, "csg.boolean.assemble.edge_conform")?;
    let [start, end] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    Ok(((pcurve_projection.u - start) / (end - start)).clamp(0.0, 1.0))
}

/// Diagnostic: (#one-use edges, #coedges, geometry-hash of the one-use set).
/// The hash quantizes each one-use edge's endpoint midpoint + length so a
/// REPEATING hash across passes reveals an oscillation, a GROWING one_use count
/// reveals runaway subdivision, and a shrinking one reveals slow progress.
fn one_use_census_signature(
    assembler: &Assembler,
    faces: &[FaceRecord],
) -> (usize, usize, u64) {
    let mut use_counts = HashMap::<u64, usize>::default();
    let mut coedge_count = 0usize;
    for coedge in faces
        .iter()
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
        coedge_count += 1;
    }
    let vertex_map = assembler
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let mut hashes = Vec::new();
    let mut one_use = 0usize;
    for edge in &assembler.edges {
        if use_counts.get(&edge.id) != Some(&1) || edge.degenerate {
            continue;
        }
        one_use += 1;
        let (Some(s), Some(e)) = (
            vertex_map.get(&edge.start_vertex_id),
            vertex_map.get(&edge.end_vertex_id),
        ) else {
            continue;
        };
        let q = |v: f64| (v * 1e6).round() as i64;
        let mid = s.add(*e).scale(0.5);
        let len = e.sub(*s).length();
        let mut h = 1469598103934665603u64;
        for k in [q(mid.x), q(mid.y), q(mid.z), q(len)] {
            h ^= k as u64;
            h = h.wrapping_mul(1099511628211);
        }
        hashes.push(h);
    }
    hashes.sort_unstable();
    let mut acc = 1469598103934665603u64;
    for h in hashes {
        acc ^= h;
        acc = acc.wrapping_mul(1099511628211);
    }
    if std::env::var("BREP_DEBUG_CONFORM_DUMP").is_ok() {
        let mut rows: Vec<String> = Vec::new();
        for edge in &assembler.edges {
            if use_counts.get(&edge.id) != Some(&1) || edge.degenerate {
                continue;
            }
            let (Some(s), Some(e)) = (
                vertex_map.get(&edge.start_vertex_id),
                vertex_map.get(&edge.end_vertex_id),
            ) else {
                continue;
            };
            let len = e.sub(*s).length();
            let key = assembler.edge_boundary_key.get(&edge.id).copied();
            rows.push(format!(
                "  edge {:>4} len={:.3e} key={:?} s=({:.5},{:.5},{:.5}) e=({:.5},{:.5},{:.5})",
                edge.id, len, key, s.x, s.y, s.z, e.x, e.y, e.z
            ));
        }
        rows.sort();
        for r in rows {
            eprintln!("{r}");
        }
    }
    (one_use, coedge_count, acc)
}

/// Partition long one-use edges wherever shorter one-use edges cover the same
/// exact curve. This lets sewing share one edge identity even when adjacent
/// faces reached the same boundary through different arrangement splits.
pub(super) fn conform_overlapping_one_use_edges(
    assembler: &mut Assembler,
    faces: &mut [FaceRecord],
) -> Result<(), KernelRefusal> {
    // A conformance split on one face can CREATE the shorter one-use pieces
    // its coincident partner needs as split guides (the candidate set is
    // computed at pass entry), so iterate to a fixpoint: gluing two solids
    // along a coplanar face conformed one side on pass 1 and the other on
    // pass 2 before this loop existed. A cap that is still making changes
    // when it runs out means an unconformed (or oscillating) edge set —
    // fail loudly rather than hand the genus check silently corrupt input.
    const MAXIMUM_PASSES: usize = 8;
    let debug_pass = std::env::var("BREP_DEBUG_CONFORM_PASS").is_ok();
    for pass in 0..MAXIMUM_PASSES {
        if debug_pass {
            let sig = one_use_census_signature(assembler, faces);
            eprintln!(
                "conform PASS {pass}: one_use={} coedges={} sig={:016x}",
                sig.0, sig.1, sig.2
            );
        }
        if !conform_overlapping_one_use_edges_pass(assembler, faces)? {
            return Ok(());
        }
    }
    if debug_pass {
        let sig = one_use_census_signature(assembler, faces);
        eprintln!(
            "conform PASS final: one_use={} coedges={} sig={:016x}",
            sig.0, sig.1, sig.2
        );
    }
    if conform_overlapping_one_use_edges_pass(assembler, faces)? {
        return Err(KernelRefusal::non_convergence(
            KernelStage::Sew,
            "edge_conform",
            format!(
            "one-use edge conformance did not converge after {MAXIMUM_PASSES} passes"
        )));
    }
    Ok(())
}

fn conform_overlapping_one_use_edges_pass(
    assembler: &mut Assembler,
    faces: &mut [FaceRecord],
) -> Result<bool, KernelRefusal> {
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in faces
        .iter()
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    let edge_map = assembler
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect::<HashMap<_, _>>();
    let vertex_map = assembler
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    // The assembler provisionally marks unwelded closed loops degenerate.
    // Such a loop can still be the carrier of real one-use arc pieces on an
    // adjacent face. Only point-collapsed geometry must be excluded here.
    let mut conformable = std::collections::HashSet::new();
    for edge in edge_map.values() {
        if use_counts.get(&edge.id) != Some(&1) { continue; }
        let mut real_span = !edge.degenerate;
        if !real_span {
            let start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?;
            for fraction in [0.25, 0.5, 0.75] {
                let point = edge.curve.evaluate(edge.t0 + fraction * (edge.t1-edge.t0))
                    .or_refuse(KernelStage::Sew, "evaluate")?;
                real_span |= point.sub(start).length() > assembler_weld(assembler.tolerance);
            }
        }
        if real_span { conformable.insert(edge.id); }
    }
    let candidates = edge_map
        .values()
        .filter(|edge| conformable.contains(&edge.id))
        .cloned()
        .collect::<Vec<_>>();
    let geometric_tolerance = assembler.tolerance.max(1e-3);
    // A split landing within the assembler's WELD radius of an endpoint mints a
    // piece whose chordal length is ≤ that same radius — which `Assembler::edge`
    // stamps degenerate on arrival, so the piece can never weld and the rest is
    // re-fit unchanged: pure churn. When a keyless (imprint/derived) guide vertex
    // sits just outside the weld band of a short boundary edge's ENDPOINT (a
    // near-miss vertex the assembler declined to fuse), its projection lands a
    // few 1e-8 from that endpoint and the pass peels a stillborn sliver there
    // every iteration, its fraction drifting microscopically between re-fits so
    // the fractional 1e-7 guard never trips — `did not converge after 8 passes`.
    // Reject such endpoint-hugging splits: the guide is effectively AT the
    // endpoint (the weld-tolerance / degeneracy threshold agree by construction),
    // so no useful split is lost and the loop reaches a fixpoint. Additive — it
    // only suppresses provably-degenerate slivers; a genuine interior split (its
    // point ≫ weld tol from both endpoints) is untouched. Escape hatch for A/B.
    let endpoint_sliver_guard = std::env::var("BREP_CONFORM_ENDPOINT_GUARD").as_deref() != Ok("0");
    let endpoint_weld = assembler_weld(assembler.tolerance);
    // A conformance split at `other`'s endpoints only ELIMINATES a one-use edge
    // if the resulting sub-piece of `edge` can WELD to a piece of `other` — that
    // is the entire purpose of the split. `Assembler::edge` welds only at the
    // assembler's weld radius (`assembler_weld`), yet candidate guides are
    // gathered by `edge_lies_on` at the far COARSER `geometric_tolerance`
    // (max(model,1e-3)). Two DISTINCT keyless arrangement edges from a
    // near-tangent grazing contact run inside that coarse radius but sit ~1e-4
    // apart — far beyond weld scale — so each split they seed on the other mints
    // a fresh keyless sliver that can NEVER weld, and the one-use set GROWS every
    // pass (measured on t62: 9 → 30 over 8 passes, strictly diverging → `did not
    // converge`). The keyed identity guard above stops this only for edges of the
    // SAME operand with DIFFERENT ids; keyless section/imprint arrangement edges
    // carry no boundary key, so it never fires on them. Require the guide to
    // coincide with `edge` at WELD scale before it may seed a split. A genuine
    // same-boundary sub-piece is weld-coincident by construction (model tol ≪
    // weld), so no useful split is lost; only provably-unweldable graze slivers
    // are refused. Interior samples (0.25/0.5/0.75) dodge the endpoint
    // range-guard false-miss in `edge_lies_on`. Escape hatch for A/B.
    let weldable_guide_gate =
        std::env::var("BREP_CONFORM_WELDABLE_GUIDE").as_deref() != Ok("0");
    let weld_scale = assembler_weld(assembler.tolerance);
    let debug_conform = std::env::var("BREP_DEBUG_CONFORM").is_ok();
    if std::env::var("BREP_DEBUG_CONFORM_DUMP").is_ok() {
        eprintln!(
            "conform tolerances: geometric={geometric_tolerance:.3e} weld_scale={weld_scale:.3e} endpoint_weld={endpoint_weld:.3e} candidates={}",
            candidates.len()
        );
    }
    let mut plans = HashMap::<u64, Vec<CoedgeRecord>>::default();
    for face in faces.iter() {
        for coedge in face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            let Some(edge) = edge_map.get(&coedge.edge_id) else {
                continue;
            };
            if !conformable.contains(&edge.id) {
                continue;
            }
            let mut fractions = vec![0.0, 1.0];
            for other in &candidates {
                // A conformance split at `other`'s endpoint only helps if the
                // resulting piece can later WELD to `other` (share one edge
                // identity). When identity-weld is refused — `other` is a
                // DIFFERENT original edge of the SAME operand — no piece of
                // `edge` can ever weld to it, so splitting here only mints
                // fresh one-use slivers. Two distinct near-coincident original
                // edges (a thin imported wedge sharing one endpoint, ~2e-4
                // apart: inside the coarse `edge_lies_on` radius but a distinct
                // boundary) would otherwise feed each other split guides every
                // pass, each shaving a new sub-tolerance sliver at a drifting
                // reprojected fraction — the loop never reaches a fixpoint
                // (`did not converge after 8 passes`). Any partner that CAN
                // weld (same original edge, or a keyless arrangement edge)
                // contributes its own guard-passing fractions, so no useful
                // split is lost.
                if weld_refused_by_identity(
                    assembler.edge_boundary_key.get(&edge.id).copied(),
                    assembler.edge_boundary_key.get(&other.id).copied(),
                ) {
                    continue;
                }
                let lies_on = edge_lies_on(other, edge, geometric_tolerance)?;
                if debug_conform && other.id != edge.id {
                    eprintln!(
                        "conform: edge {} vs other {} lies_on={}",
                        edge.id, other.id, lies_on
                    );
                }
                if other.id == edge.id || !lies_on {
                    continue;
                }
                // Weldable-guide gate (see the block comment above): a guide that
                // only coincides at the coarse candidate radius but not at weld
                // scale seeds unweldable graze slivers → divergence. Refuse it.
                if weldable_guide_gate && !edge_interior_lies_on(other, edge, weld_scale)? {
                    if debug_conform {
                        eprintln!(
                            "conform: edge {} REFUSED guide {} (coarse lies_on but not weld-scale {:.3e})",
                            edge.id, other.id, weld_scale
                        );
                    }
                    continue;
                }
                for vertex_id in [other.start_vertex_id, other.end_vertex_id] {
                    if let Some(fraction) =
                        edge_fraction_at_point(edge, vertex_map[&vertex_id], geometric_tolerance)?
                    {
                        // Endpoint-sliver guard, in ABSOLUTE geometry (the
                        // fraction's scale varies with edge length): reject a
                        // split whose point lands within the weld radius of this
                        // edge's own start/end vertex. Such a split only mints a
                        // piece `Assembler::edge` stamps degenerate on arrival —
                        // it can never weld, the rest is re-fit unchanged, and
                        // the re-projected fraction drifts a hair each pass so
                        // the fractional 1e-7 guard below never trips (the
                        // `did not converge` churn).
                        if endpoint_sliver_guard {
                            let split_pt = edge
                                .curve
                                .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                                .unwrap_or(vertex_map[&vertex_id]);
                            let dstart =
                                split_pt.sub(vertex_map[&edge.start_vertex_id]).length();
                            let dend = split_pt.sub(vertex_map[&edge.end_vertex_id]).length();
                            if dstart <= endpoint_weld || dend <= endpoint_weld {
                                continue;
                            }
                        }
                        if fraction > 1e-7
                            && fraction < 1.0 - 1e-7
                            && !fractions
                                .iter()
                                .any(|value| (*value - fraction).abs() <= 1e-6)
                        {
                            fractions.push(fraction);
                        }
                    }
                }
            }
            fractions.sort_by(f64::total_cmp);
            if fractions.len() <= 2 {
                continue;
            }
            let traversal_fractions = if coedge.forward {
                fractions
            } else {
                fractions
                    .into_iter()
                    .rev()
                    .map(|fraction| 1.0 - fraction)
                    .collect()
            };
            let mut replacements = Vec::new();
            let mut plan_valid = true;
            for pair in traversal_fractions.windows(2) {
                let traversal_a = pair[0];
                let traversal_b = pair[1];
                if traversal_b - traversal_a <= 1e-7 {
                    continue;
                }
                let edge_a = if coedge.forward {
                    traversal_a
                } else {
                    1.0 - traversal_a
                };
                let edge_b = if coedge.forward {
                    traversal_b
                } else {
                    1.0 - traversal_b
                };
                let low = edge_a.min(edge_b);
                let high = edge_a.max(edge_b);
                let t0 = edge.t0 + (edge.t1 - edge.t0) * low;
                let t1 = edge.t0 + (edge.t1 - edge.t0) * high;
                let start = edge.curve.evaluate(t0).or_refuse(KernelStage::Sew, "evaluate")?;
                let end = edge.curve.evaluate(t1).or_refuse(KernelStage::Sew, "evaluate")?;
                let forward = edge_a < edge_b;
                // A conformance split inherits its parent assembler edge's
                // original-boundary identity so its pieces re-weld only to
                // pieces of the SAME original edge, never to a coincident twin.
                let parent_key = assembler.edge_boundary_key.get(&edge.id).copied();
                let parent_constructed = assembler.imprint_edges.contains(&edge.id);
                let (edge_id, reversed) = assembler.edge(
                    SourceEdge {
                        curve: edge.curve.clone(),
                        t0,
                        t1,
                        start,
                        end,
                        degenerate: false,
                        name: edge.name.clone(),
                        constructed: parent_constructed,
                        boundary_key: parent_key,
                    },
                    forward,
                )?;
                let pcurve_start = pcurve_fraction_at_edge_fraction(face, coedge, edge, edge_a)?;
                let pcurve_end = pcurve_fraction_at_edge_fraction(face, coedge, edge, edge_b)?;
                // A piece that spans real edge length but collapses in
                // pcurve space means the pcurve mapping is unreliable here;
                // emitting a degenerate piece would open the loop, so keep
                // the original coedge untouched (the assembly then reports
                // a clean error instead of building an invalid solid).
                if (pcurve_end - pcurve_start).abs() <= 1e-9 {
                    plan_valid = false;
                    break;
                }
                let mut pcurve = curve_subrange(
                    &coedge.pcurve,
                    pcurve_start.min(pcurve_end),
                    pcurve_start.max(pcurve_end),
                )?;
                if pcurve_start > pcurve_end {
                    pcurve = pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                }
                replacements.push(CoedgeRecord {
                    id: assembler.next_coedge_id,
                    edge_id,
                    forward: if reversed { !forward } else { forward },
                    pcurve,
                });
                assembler.next_coedge_id += 1;
            }
            if plan_valid && replacements.len() > 1 {
                if debug_conform {
                    let start = vertex_map[&edge.start_vertex_id];
                    let end = vertex_map[&edge.end_vertex_id];
                    eprintln!(
                        "conform PLAN: coedge {} edge {} len={:.3e} pieces={} s=({:.6},{:.6},{:.6}) e=({:.6},{:.6},{:.6})",
                        coedge.id,
                        edge.id,
                        end.sub(start).length(),
                        replacements.len(),
                        start.x, start.y, start.z,
                        end.x, end.y, end.z
                    );
                }
                plans.insert(coedge.id, replacements);
            }
        }
    }
    let mut applied = false;
    for face in faces {
        for loop_record in &mut face.loops {
            let mut conformed = Vec::new();
            for coedge in loop_record.coedges.drain(..) {
                if let Some(replacements) = plans.remove(&coedge.id) {
                    conformed.extend(replacements);
                    applied = true;
                } else {
                    conformed.push(coedge);
                }
            }
            loop_record.coedges = conformed;
        }
    }
    Ok(applied)
}


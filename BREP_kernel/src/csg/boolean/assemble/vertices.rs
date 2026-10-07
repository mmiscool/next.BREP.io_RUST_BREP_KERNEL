//! Vertex-onto-carriers settling: move a vertex that sits off one of its own
//! faces' carriers to the point that lies on all of them, and re-snap the
//! ends of its edges there.
//!
//! Where the assembler welds fragment endpoints inside its weld band it keeps
//! one fragment's endpoint as the vertex. The other fragments' trims were
//! built from the exact sections and still meet at the true corner; the kept
//! vertex can sit off the third carrier by the weld's slack, and every edge
//! end snapped onto it inherits that error (`BadBoolean`: vertex 14 2.16e-5
//! off the offset ball, edges 99 and 33 off it by the same, the
//! offset/subtract fixture: corner (9, 1, 5.489065) 7.3e-4 off the cone where
//! the cone's symmetry and the trims put it at z = 5.490732). Rebuilding the
//! trims from those edges adopts the wrong corner and moved an oracle row by
//! +1.9e-4, so the trims are the witness this pass keeps and the vertex is
//! what moves.
//!
//! LOCAL: only a vertex whose worst carrier residual exceeds the bar is
//! touched. BOUNDED: the solved point must lie on every incident carrier
//! within the bar; the move must not exceed the validator's edge/trim
//! contract (`pcurve_consistency`) nor eight times the residual it closes
//! (a carrier met at less than ~7° is left alone); a vertex farther off a
//! carrier than the contract is not this pass's case and is reported. The
//! solve is Gauss-Newton on the signed distances to the carriers' tangent
//! planes with a minimum-norm step, so a vertex on two carriers whose third
//! is nearly tangent to their intersection is not dragged along it.
//! `polish_triple_junction_vertices` solves three carriers too, but it never
//! runs on the open path and accepts a solve only when it reduces the
//! curve-endpoint gap — which is zero here, because the edges were snapped to
//! the wrong vertex; that is the case this pass exists for.
//!
//! **Which side is the witness is measured, not assumed (2026-10-03).** The
//! premise above — the trims were built for the true corner and the vertex is
//! what moves — holds for the family this pass was built on (`BadBoolean`,
//! the offset/subtract corner). It does not hold where the imprint ended a
//! CONSTRUCTED section at a vendor edge's crossing and this pass then settled
//! the vertex onto the carriers' triple point: on `anotherBooleanFail` the
//! eight sphere ∩ torus sections' trims kept ending 3.514e-4 / 1.556e-4 from
//! the vertices the pass had moved, and the sphere face read 208.693367
//! against its input-only area 208.693474. So after every vertex has settled,
//! [`refresh_section_trims`] reads each re-snapped constructed edge's trims
//! against the paired carriers and the edge against all of them, and rebuilds
//! a trim from its edge only where the edge is the better witness, loop by
//! loop and under the whole shell's closure reading. Vendor coedges are never
//! candidates; nothing here moves a vertex, a vendor curve or a bar.

use super::builder::snap_edge_curve_endpoints;
use crate::{BrepSolid, KernelRefusal, KernelStage, KernelTolerances, OrRefuse, Vec3};
use rustc_hash::FxHashMap as HashMap;

/// One accepted vertex move with everything needed to put it back.
pub(in crate::boolean) struct VertexMove {
    pub vertex_id: u64,
    pub previous: Vec3,
    /// The residual the vertex had to its worst carrier before the move.
    pub residual_before: f64,
    /// The worst residual to any incident carrier after the move.
    pub residual_after: f64,
    /// Incident edges as they were: `(edge id, curve, t0, t1)`.
    pub edges: Vec<(u64, crate::NurbsCurve, f64, f64)>,
    /// Constructed-section trims refreshed from their re-snapped edge, as
    /// they were: `(face id, loop id, coedge id, pcurve)`.
    pub pcurves: Vec<(u64, u64, u64, crate::NurbsCurve)>,
}

/// Put back every vertex and edge in `moves`.
pub(in crate::boolean) fn restore_vertices(solid: &mut BrepSolid, moves: Vec<VertexMove>) {
    for item in moves {
        if let Some(vertex) = solid
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == item.vertex_id)
        {
            vertex.point = item.previous;
        }
        for (edge_id, curve, t0, t1) in item.edges {
            if let Some(edge) = solid.edges.iter_mut().find(|edge| edge.id == edge_id) {
                edge.curve = curve;
                edge.t0 = t0;
                edge.t1 = t1;
            }
        }
        for (face_id, loop_id, coedge_id, pcurve) in item.pcurves {
            for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
                if face.id != face_id {
                    continue;
                }
                for loop_record in face.loops.iter_mut().filter(|lp| lp.id == loop_id) {
                    for coedge in loop_record.coedges.iter_mut().filter(|c| c.id == coedge_id) {
                        coedge.pcurve = pcurve.clone();
                    }
                }
            }
        }
    }
}

/// How a constructed section's trim read against its re-snapped edge, and
/// what happened to it. One line per coedge under `BREP_DEBUG_JOINTS`.
#[derive(Clone, Copy, Debug, PartialEq)]
enum TrimRefresh {
    /// The trim's image already meets the edge at both ends within the bar.
    Meets,
    /// The trim's image is at least as close to the paired carriers as the
    /// edge is to its carriers: the trims are the witness (BadBoolean), the
    /// trim stands.
    TrimIsWitness,
    /// Rebuilt from the edge: the edge sits on its carriers where the trim
    /// image leaves the paired carrier, and the rebuilt trim tracks the edge
    /// more closely than the old one did.
    Rebuilt,
    /// The rebuild failed or fitted the edge worse than the old trim: the old
    /// trim stands.
    Declined,
}

/// Image of a coedge's pcurve on its face at `fraction` of the pcurve's domain
/// (the pcurve runs in the coedge's traversal direction).
fn trim_image(face: &crate::FaceRecord, coedge: &crate::topology::CoedgeRecord, fraction: f64) -> Result<Vec3, KernelRefusal> {
    let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let uv = coedge.pcurve.evaluate(p0 + fraction * (p1 - p0)).or_refuse(KernelStage::Sew, "evaluate")?;
    face.surface.evaluate_extended(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate_extended")
}

/// The edge's point at `fraction` of the coedge's traversal.
fn edge_point(edge: &crate::topology::EdgeRecord, forward: bool, fraction: f64) -> Result<Vec3, KernelRefusal> {
    let t = if forward { edge.t0 + fraction * (edge.t1 - edge.t0) } else { edge.t1 - fraction * (edge.t1 - edge.t0) };
    edge.curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")
}

/// Sampling fractions for the trim-vs-edge readings: the ends and 64 spans.
fn refresh_fractions() -> impl Iterator<Item = f64> {
    (0..=64).map(|k| k as f64 / 64.0)
}

/// Worst distance of the images `points` to any of `surfaces`.
fn worst_off(points: &[Vec3], surfaces: &[&crate::NurbsSurface]) -> Result<f64, KernelRefusal> {
    let mut worst = 0.0f64;
    for point in points {
        for surface in surfaces {
            worst = worst.max(crate::project_point_to_surface(surface, *point).or_refuse(KernelStage::Sew, "project_point_to_surface")?.distance);
        }
    }
    Ok(worst)
}

/// The constructed section's trim follows its edge where the EDGE is the
/// witness. After [`settle_vertices_onto_carriers`] has moved a vertex onto its
/// carriers and re-snapped the incident edges, a trim built before the move
/// still ends where the edge used to end. On `anotherBooleanFail` the eight
/// sphere ∩ torus sections ended at the vendor junction edge's sphere
/// crossing, 2.213e-4 inside the tube; settling put every vertex on the
/// triple point (moves of 3.514e-4 / 1.556e-4) and the re-snapped edges read
/// on both carriers to 1.9e-7 end to end, while both pcurves of each edge kept
/// ending 3.514e-4 from the vertex and the sphere face read 208.693367 against
/// the input-only lens area 208.693474 (2026-10-03).
///
/// Which side is the witness is measured, never assumed: the trim's image is
/// read against the PAIRED carriers (the other faces' surfaces along the same
/// edge) and the edge against all of them, at 65 matched fractions. Where the
/// trim image leaves the paired carrier by more than the edge leaves its
/// carriers, the edge is the witness and the trim is rebuilt from it
/// (`build_pcurve_on_surface_range` on the coedge's traversal, period branch
/// preserved); where the trims sit closer (BadBoolean: vertex 2.16e-5 off the
/// ball, trims exact) nothing changes. A rebuilt trim stands only when it
/// tracks the edge more closely at its worst sample than the old one did.
/// Only edges created from an imprint piece (`constructed`) are candidates: a
/// vendor edge's trims are the file's and are never refreshed here.
fn refresh_section_trims(
    solid: &BrepSolid,
    edge_id: u64,
    bar: f64,
    debug: bool,
    candidates: &mut Vec<TrimCandidate>,
    counts: &mut [usize; 4],
) -> Result<(), KernelRefusal> {
    let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id).cloned() else {
        return Ok(());
    };
    if edge.degenerate {
        return Ok(());
    }
    // Every (face, loop, coedge) using the edge, and each face's carrier.
    let mut uses: Vec<(usize, usize, usize)> = Vec::new();
    let faces: Vec<&crate::FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    for (face_index, face) in faces.iter().enumerate() {
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                if coedge.edge_id == edge_id {
                    uses.push((face_index, loop_index, coedge_index));
                }
            }
        }
    }
    if uses.len() < 2 {
        // A one-use edge has no paired carrier to read the trim against.
        return Ok(());
    }
    let carriers: Vec<&crate::NurbsSurface> = uses.iter().map(|&(f, _, _)| &faces[f].surface).collect();
    let mut decisions: Vec<(usize, usize, usize, TrimRefresh, Option<crate::NurbsCurve>, f64, f64, f64, f64)> = Vec::new();
    for &(face_index, loop_index, coedge_index) in &uses {
        let face = faces[face_index];
        let coedge = &face.loops[loop_index].coedges[coedge_index];
        let mut images = Vec::new();
        let mut points = Vec::new();
        let mut gap = 0.0f64;
        for fraction in refresh_fractions() {
            let image = trim_image(face, coedge, fraction)?;
            let point = edge_point(&edge, coedge.forward, fraction)?;
            gap = gap.max(image.sub(point).length());
            images.push(image);
            points.push(point);
        }
        let end_gap = images[0].sub(points[0]).length().max(images[images.len() - 1].sub(points[points.len() - 1]).length());
        if end_gap <= bar {
            decisions.push((face_index, loop_index, coedge_index, TrimRefresh::Meets, None, end_gap, gap, 0.0, 0.0));
            continue;
        }
        let paired: Vec<&crate::NurbsSurface> = uses.iter().filter(|&&(f, _, _)| f != face_index).map(|&(f, _, _)| &faces[f].surface).collect();
        let trim_off = worst_off(&images, &paired)?;
        let edge_off = worst_off(&points, &carriers)?;
        // The END witness. The whole-trim reading above compares maxima, and
        // on a section that stands off a curved carrier by its own standoff
        // that maximum is the INTERIOR standoff, which hides the ends: on
        // fixture 25 (a4c84871, BREP_DEBUG_JOINTS) settling moved three
        // vertices 2.0e-7..5.5e-7 onto their carriers, every section trim on
        // the curved faces read TrimIsWitness (trim off its paired carrier
        // 3.3e-7..5.8e-6 against an edge 1.65e-6..1.27e-5 off its carriers),
        // and each kept ending 3.25e-7..5.25e-7 from its settled vertex: the
        // every-use control's tracking maxima sat exactly at q 0 or 1, where
        // the edge stands 1e-10..1e-14 off. So the ends are read on their own:
        // when the edge's ENDS (the settled vertices) lie on every carrier
        // within the bar and the trim's ends stand farther off the paired
        // carriers than they do, the vertex is the witness at that end and
        // the trim is rebuilt even though its interior reads as the witness.
        // A vertex off its carriers (BadBoolean: 2.16e-5 off the ball, trims
        // exact) is never an end witness.
        let end_images = [images[0], images[images.len() - 1]];
        let end_points = [points[0], points[points.len() - 1]];
        let trim_end_off = worst_off(&end_images, &paired)?;
        let vertex_off = worst_off(&end_points, &carriers)?;
        let end_witness = is_end_witness(trim_off, edge_off, vertex_off, trim_end_off, bar);
        if trim_off <= edge_off && !end_witness {
            decisions.push((face_index, loop_index, coedge_index, TrimRefresh::TrimIsWitness, None, end_gap, gap, trim_off, edge_off));
            continue;
        }
        // An END-witness rebuild is judged by the common reader below, so it
        // takes the range lane's fit as it comes: `build_pcurve_on_surface_range`
        // refuses any fit off its bar (`accept_fit`), and on an off-carrier
        // section that bar is the interior standoff, so every end-witness
        // rebuild of fixture 25 read Declined before the judge saw it (aa9117:
        // "0 rebuilt, 8 declined"). Every other rebuild keeps its builder.
        // The fit's own report is RECORDED once on the open pcurve scope
        // (`pcurve::record_fit`, the semantics the curve-only builders use),
        // with its actual exit and residual, and printed in full under debug
        // before the common reader judges it. A report with a non-finite
        // residual or standoff is declined here, before anything is mutated.
        // `build_pcurve_on_surface_range` (the other branch) records nothing,
        // so nothing is counted twice.
        let built = if end_witness {
            match crate::pcurve::fit_pcurve_on_surface_range(&face.surface, &edge.curve, edge.t0, edge.t1, coedge.forward, bar) {
                Ok(fit) => {
                    crate::pcurve::record_fit(&fit.report);
                    if debug {
                        eprintln!("vertices: edge {edge_id} trim on face {}: end-witness rebuild fit {:?}", face.id, fit.report);
                    }
                    if fit.report.residual.is_finite() && fit.report.off_surface.is_finite() {
                        Ok(fit.curve)
                    } else {
                        Err(format!("non-finite fit report {:?}", fit.report))
                    }
                }
                Err(error) => Err(error),
            }
        } else {
            crate::pcurve::build_pcurve_on_surface_range(&face.surface, &edge.curve, edge.t0, edge.t1, coedge.forward, bar)
        };
        if debug && end_witness {
            if let Err(error) = &built {
                eprintln!("vertices: edge {edge_id} trim on face {}: end-witness rebuild failed: {error}", face.id);
            }
        }
        let rebuilt = built
            .ok()
            .map(|pcurve| align_period_branch(&face.surface, pcurve, &coedge.pcurve))
            .transpose()?;
        let mut outcome = (TrimRefresh::Declined, None);
        if let Some(pcurve) = rebuilt {
            let candidate = crate::topology::CoedgeRecord { id: coedge.id, edge_id: coedge.edge_id, forward: coedge.forward, pcurve };
            let mut new_gap = 0.0f64;
            for (k, fraction) in refresh_fractions().enumerate() {
                new_gap = new_gap.max(trim_image(face, &candidate, fraction)?.sub(points[k]).length());
            }
            // An END-witness rebuild is judged by the common tracking reader
            // (`pcurve::common_station_read`: one shared admitted foot per
            // probe; both pcurves' own knot spans and endpoints): strictly
            // better tracking, and a deviation no worse than the old trim's or
            // within the edge's own standoff plus the bar. The whole-trim gap
            // cannot judge it: both trims deviate by the interior standoff.
            // Every other rebuild keeps its original gate.
            let accept = if end_witness {
                end_witness_rebuild_tracks(face, &edge, coedge.forward, &coedge.pcurve, &candidate.pcurve, edge_off, bar, debug)
            } else {
                new_gap < gap
            };
            if accept {
                outcome = (TrimRefresh::Rebuilt, Some(candidate.pcurve));
            }
        }
        decisions.push((face_index, loop_index, coedge_index, outcome.0, outcome.1, end_gap, gap, trim_off, edge_off));
    }
    let face_ids: Vec<u64> = faces.iter().map(|face| face.id).collect();
    drop(faces);
    for (face_index, loop_index, coedge_index, outcome, pcurve, end_gap, gap, trim_off, edge_off) in decisions {
        if debug {
            eprintln!(
                "vertices: edge {edge_id} trim on face {} {outcome:?}: end gap {end_gap:.3e}, worst image-edge gap {gap:.3e}, trim off paired carrier {trim_off:.3e}, edge off carriers {edge_off:.3e}",
                face_ids[face_index]
            );
        }
        match pcurve {
            Some(pcurve) => candidates.push(TrimCandidate { face_index, loop_index, coedge_index, pcurve }),
            None => counts[outcome as usize] += 1,
        }
    }
    Ok(())
}

/// The END-witness rule of [`refresh_section_trims`]: a trim the whole-trim
/// reading keeps (`trim_off <= edge_off`) is still rebuilt when the edge's ends
/// (settled vertices) lie on every carrier within `bar` (`vertex_off`) and the
/// trim's end images stand farther off the paired carriers (`trim_end_off`).
/// A vertex off its carriers is never the witness.
fn is_end_witness(trim_off: f64, edge_off: f64, vertex_off: f64, trim_end_off: f64, bar: f64) -> bool {
    trim_off <= edge_off && vertex_off <= bar && trim_end_off > vertex_off
}

/// Whether an END-witness rebuild `new` of `old` (both on `face`, both over
/// the coedge fraction domain [0, 1]) tracks the edge better at the common
/// probes: strictly lower tracking, deviation no worse than the old trim's or
/// within `edge_off + bar`. Any unreadable read, or a pcurve not on [0, 1],
/// declines.
#[allow(clippy::too_many_arguments)]
fn end_witness_rebuild_tracks(face: &crate::FaceRecord, edge: &crate::topology::EdgeRecord, forward: bool, old: &crate::NurbsCurve, new: &crate::NurbsCurve, edge_off: f64, bar: f64, debug: bool) -> bool {
    let unit = |c: &crate::NurbsCurve| matches!(c.domain(), Ok([a, b]) if a == 0.0 && b == 1.0);
    if !(unit(old) && unit(new)) {
        return false;
    }
    let Ok(probes) = crate::pcurve::native_probe_fractions(&[old, new]) else { return false };
    let evaluate = |fraction: f64| -> Result<Vec3, String> {
        let t = if forward { edge.t0 + fraction * (edge.t1 - edge.t0) } else { edge.t1 - fraction * (edge.t1 - edge.t0) };
        edge.curve.evaluate(t)
    };
    let read = crate::pcurve::common_station_read(&face.surface, &evaluate, old, new, &probes);
    if debug {
        eprintln!("vertices: edge {} trim on face {}: end-witness common read {read:?} (edge off {edge_off:.3e}, bar {bar:.1e})", edge.id, face.id);
    }
    match read {
        Ok(((old_tracking, old_deviation), (new_tracking, new_deviation))) => {
            new_tracking < old_tracking && new_deviation <= old_deviation.max(edge_off + bar)
        }
        Err(_) => false,
    }
}

/// A rebuilt trim waiting for its loop's closure reading.
struct TrimCandidate {
    face_index: usize,
    loop_index: usize,
    coedge_index: usize,
    pcurve: crate::NurbsCurve,
}

/// Worst gap between consecutive trim images around `loop_record` on `face`,
/// with `overrides` (coedge index → pcurve) standing in for the loop's own.
pub(super) fn loop_joint_gap(face: &crate::FaceRecord, loop_index: usize, overrides: &HashMap<usize, &crate::NurbsCurve>) -> Result<f64, KernelRefusal> {
    let loop_record = &face.loops[loop_index];
    let n = loop_record.coedges.len();
    let mut worst = 0.0f64;
    for i in 0..n {
        let here = overrides.get(&i).copied().unwrap_or(&loop_record.coedges[i].pcurve);
        let next = overrides.get(&((i + 1) % n)).copied().unwrap_or(&loop_record.coedges[(i + 1) % n].pcurve);
        let [_, p1] = here.domain().or_refuse(KernelStage::Sew, "domain")?;
        let [q0, _] = next.domain().or_refuse(KernelStage::Sew, "domain")?;
        let a = here.evaluate(p1).or_refuse(KernelStage::Sew, "evaluate")?;
        let b = next.evaluate(q0).or_refuse(KernelStage::Sew, "evaluate")?;
        let ia = face.surface.evaluate_extended(a.x, a.y).or_refuse(KernelStage::Sew, "evaluate_extended")?;
        let ib = face.surface.evaluate_extended(b.x, b.y).or_refuse(KernelStage::Sew, "evaluate_extended")?;
        worst = worst.max(ia.sub(ib).length());
    }
    Ok(worst)
}

/// Apply the rebuilt trims loop by loop: a loop's candidates stand together
/// only when the loop still closes at least as well as it did (worst joint gap
/// not above `max(before, bar)`). Where a rebuilt section trim would end on
/// the vertex while its loop neighbour — a vendor edge's trim the file wrote —
/// still ends where the vertex used to be, the rebuild would open the loop by
/// that distance (2.730e-4 on `anotherBooleanFail`'s cut torus faces); the
/// old trim stands and the case is counted as declined.
fn apply_trim_candidates(
    solid: &mut BrepSolid,
    candidates: Vec<TrimCandidate>,
    bar: f64,
    debug: bool,
    replaced: &mut Vec<(u64, u64, u64, crate::NurbsCurve)>,
    counts: &mut [usize; 4],
) -> Result<(), KernelRefusal> {
    let mut by_loop: HashMap<(usize, usize), Vec<&TrimCandidate>> = HashMap::default();
    for candidate in &candidates {
        by_loop.entry((candidate.face_index, candidate.loop_index)).or_default().push(candidate);
    }
    let mut keys: Vec<(usize, usize)> = by_loop.keys().copied().collect();
    keys.sort_unstable();
    let faces: Vec<&crate::FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut accepted: Vec<&TrimCandidate> = Vec::new();
    for key in keys {
        let group = &by_loop[&key];
        let face = faces[key.0];
        let before = loop_joint_gap(face, key.1, &HashMap::default())?;
        let overrides: HashMap<usize, &crate::NurbsCurve> = group.iter().map(|c| (c.coedge_index, &c.pcurve)).collect();
        let after = loop_joint_gap(face, key.1, &overrides)?;
        if after <= before.max(bar) {
            accepted.extend(group.iter().copied());
            counts[TrimRefresh::Rebuilt as usize] += group.len();
        } else {
            counts[TrimRefresh::Declined as usize] += group.len();
            if debug {
                eprintln!(
                    "vertices: face {} loop {}: {} rebuilt trim(s) declined, the loop would open from {before:.3e} to {after:.3e}",
                    face.id, face.loops[key.1].id, group.len()
                );
            }
        }
    }
    let accepted: Vec<(usize, usize, usize, crate::NurbsCurve)> = accepted.into_iter().map(|c| (c.face_index, c.loop_index, c.coedge_index, c.pcurve.clone())).collect();
    drop(faces);
    for (face_index, loop_index, coedge_index, pcurve) in accepted {
        let face = solid.shells.iter_mut().flat_map(|shell| &mut shell.faces).nth(face_index).expect("face index");
        let face_id = face.id;
        let loop_id = face.loops[loop_index].id;
        let coedge = &mut face.loops[loop_index].coedges[coedge_index];
        replaced.push((face_id, loop_id, coedge.id, coedge.pcurve.clone()));
        coedge.pcurve = pcurve;
    }
    Ok(())
}

/// Shift `rebuilt` by whole periods of each closed direction so its mean
/// control point lands nearest `previous`'s: a rebuild on a closed carrier
/// projects onto the canonical branch and would otherwise tear a seam-adjacent
/// trim off its loop (the same alignment `imprint/junctions.rs` applies).
pub(super) fn align_period_branch(surface: &crate::NurbsSurface, mut rebuilt: crate::NurbsCurve, previous: &crate::NurbsCurve) -> Result<crate::NurbsCurve, KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Sew, "closed_directions")?;
    if !closed_u && !closed_v {
        return Ok(rebuilt);
    }
    let mean = |pcurve: &crate::NurbsCurve| -> Option<(f64, f64)> {
        let (mut su, mut sv, mut n) = (0.0, 0.0, 0.0);
        for point in &pcurve.control_points {
            if point.w.abs() > 1e-300 {
                su += point.x / point.w;
                sv += point.y / point.w;
                n += 1.0;
            }
        }
        (n > 0.0).then(|| (su / n, sv / n))
    };
    let (Some((old_u, old_v)), Some((new_u, new_v))) = (mean(previous), mean(&rebuilt)) else {
        return Ok(rebuilt);
    };
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let shift_u = if closed_u && u1 > u0 { ((old_u - new_u) / (u1 - u0)).round() } else { 0.0 };
    let shift_v = if closed_v && v1 > v0 { ((old_v - new_v) / (v1 - v0)).round() } else { 0.0 };
    if shift_u != 0.0 || shift_v != 0.0 {
        for point in &mut rebuilt.control_points {
            point.x += shift_u * (u1 - u0) * point.w;
            point.y += shift_v * (v1 - v0) * point.w;
        }
    }
    Ok(rebuilt)
}

/// Worst distance from `point` to the carriers of the faces in `faces`.
fn worst_residual(surfaces: &[&crate::NurbsSurface], point: Vec3) -> Result<f64, KernelRefusal> {
    let mut worst = 0.0f64;
    for surface in surfaces {
        let projection = crate::project_point_to_surface(surface, point)
            .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
        worst = worst.max(projection.distance);
    }
    Ok(worst)
}

/// The point on all `surfaces` nearest `start`, or `None` when the solve
/// does not converge inside the bar.
fn settle(
    surfaces: &[&crate::NurbsSurface],
    start: Vec3,
    bar: f64,
    step_limit: f64,
) -> Result<Option<Vec3>, KernelRefusal> {
    let mut x = start;
    for _ in 0..40 {
        let mut a = [[0.0f64; 3]; 3];
        let mut b = [0.0f64; 3];
        let mut worst = 0.0f64;
        for surface in surfaces {
            let projection = crate::project_point_to_surface(surface, x)
                .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
            let Ok(normal) = surface.normal(projection.u, projection.v) else {
                return Ok(None);
            };
            let length = normal.length();
            if !(length > 1e-300) {
                return Ok(None);
            }
            let n = normal.scale(1.0 / length);
            let residual = n.dot(x.sub(projection.point));
            worst = worst.max(residual.abs());
            let nv = [n.x, n.y, n.z];
            for row in 0..3 {
                for column in 0..3 {
                    a[row][column] += nv[row] * nv[column];
                }
                b[row] -= residual * nv[row];
            }
        }
        if worst <= bar * 0.1 {
            return Ok(Some(x));
        }
        // Minimum-norm step: a rank-deficient normal set moves only across
        // the carriers, never along their common tangent.
        let trace = a[0][0] + a[1][1] + a[2][2];
        let damping = (trace * 1e-10).max(1e-300);
        for index in 0..3 {
            a[index][index] += damping;
        }
        let Ok(delta) = crate::fit::solve_small(a, b, 3) else {
            return Ok(None);
        };
        let step = Vec3::new(delta[0], delta[1], delta[2]);
        let step_length = step.length();
        if !step_length.is_finite() {
            return Ok(None);
        }
        let step = if step_length > step_limit {
            step.scale(step_limit / step_length)
        } else {
            step
        };
        x = x.add(step);
        if x.sub(start).length() > step_limit {
            return Ok(None);
        }
        if step_length < 1e-15 {
            break;
        }
    }
    Ok((worst_residual(surfaces, x)? <= bar).then_some(x))
}

/// When a dependent normal set misses a represented patch boundary,
/// solve on an isocurve explicitly supported by an incident positive-weight
/// trim. A carrier normal alone cannot constrain motion along its boundary.
/// Every candidate still meets all incident carriers and the same entry move
/// budget. Unsupported boundaries are left unchanged.
fn settle_supported_boundary(
    solid: &BrepSolid,
    faces: &[&crate::FaceRecord],
    adjacent: &[usize],
    vertex_id: u64,
    start: Vec3,
    bar: f64,
    limit: f64,
) -> Option<Vec3> {
    let surfaces: Vec<_> = adjacent.iter().map(|&i| &faces[i].surface).collect();
    let mut boundaries = Vec::new();
    for &index in adjacent {
        let face = faces[index];
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            let edge = solid.edges.iter().find(|e| e.id == coedge.edge_id)?;
            if edge.start_vertex_id != vertex_id && edge.end_vertex_id != vertex_id {
                continue;
            }
            let controls = &coedge.pcurve.control_points;
            if controls.is_empty() || controls.iter().any(|p| !(p.w > 0.0)) {
                continue;
            }
            let first = controls[0];
            let (u, v) = (first.x / first.w, first.y / first.w);
            let [u0, u1] = face.surface.domain_u().ok()?;
            let [v0, v1] = face.surface.domain_v().ok()?;
            if controls
                .iter()
                .all(|p| p.x / p.w == u && p.y / p.w >= v0 && p.y / p.w <= v1)
                && (u == u0 || u == u1)
            {
                if let Ok(c) = face.surface.iso_curve_u(u) {
                    boundaries.push(c);
                }
            }
            if controls
                .iter()
                .all(|p| p.y / p.w == v && p.x / p.w >= u0 && p.x / p.w <= u1)
                && (v == v0 || v == v1)
            {
                if let Ok(c) = face.surface.iso_curve_v(v) {
                    boundaries.push(c);
                }
            }
        }
    }
    let mut best: Option<Vec3> = None;
    for curve in boundaries {
        let Ok(foot) = crate::project_point_to_curve(&curve, start) else {
            continue;
        };
        let Ok([lo, hi]) = curve.domain() else {
            continue;
        };
        let mut t = foot.u;
        for _ in 0..24 {
            let Ok(d) = curve.derivatives(t, 1) else {
                break;
            };
            let point = d[0];
            if point.sub(start).length() > limit {
                break;
            }
            let Ok(residual) = worst_residual(&surfaces, point) else {
                break;
            };
            if residual <= bar * 0.1 {
                // Check complete induced edge motions, not just the vertex.
                let mut safe = true;
                for edge in solid
                    .edges
                    .iter()
                    .filter(|e| e.start_vertex_id == vertex_id || e.end_vertex_id == vertex_id)
                {
                    let endpoint = |id| {
                        if id == vertex_id {
                            Some(point)
                        } else {
                            solid.vertices.iter().find(|v| v.id == id).map(|v| v.point)
                        }
                    };
                    let (Some(a), Some(b)) =
                        (endpoint(edge.start_vertex_id), endpoint(edge.end_vertex_id))
                    else {
                        safe = false;
                        break;
                    };
                    let Ok(candidate) =
                        snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1, a, b)
                    else {
                        safe = false;
                        break;
                    };
                    if !super::clearance::motion_clear(edge, &candidate, faces, &solid.edges) {
                        safe = false;
                        break;
                    }
                }
                if safe
                    && best.map_or(true, |old| {
                        point.sub(start).length() < old.sub(start).length()
                    })
                {
                    best = Some(point);
                }
                break;
            }
            let mut numerator = 0.0;
            let mut denominator = 0.0;
            for surface in &surfaces {
                let Ok(projection) = crate::project_point_to_surface(surface, point) else {
                    denominator = 0.0;
                    break;
                };
                let Ok(n) = surface
                    .normal(projection.u, projection.v)
                    .and_then(|n| n.normalized())
                else {
                    denominator = 0.0;
                    break;
                };
                let rate = n.dot(d[1]);
                numerator += rate * n.dot(point.sub(projection.point));
                denominator += rate * rate;
            }
            if !(denominator > 1e-20) {
                break;
            }
            let step = numerator / denominator;
            let mut next = None;
            for fraction in [1.0, 0.5, 0.25, 0.125] {
                let trial = (t - fraction * step).clamp(lo, hi);
                let Ok(point) = curve.evaluate(trial) else {
                    continue;
                };
                if point.sub(start).length() <= limit
                    && worst_residual(&surfaces, point)
                        .ok()
                        .is_some_and(|r| r < residual)
                {
                    next = Some(trial);
                    break;
                }
            }
            let Some(trial) = next else {
                break;
            };
            t = trial;
        }
    }
    best
}

/// Settle every vertex of `solid` that sits off one of its carriers by more
/// than the joint bar. Returns the accepted moves with their previous state.
pub(in crate::boolean) fn settle_vertices_onto_carriers(
    solid: &mut BrepSolid,
    tolerance: f64,
    constructed: &rustc_hash::FxHashSet<u64>,
) -> Result<Vec<VertexMove>, KernelRefusal> {
    let policy = KernelTolerances::for_solid(solid, 1e-7);
    let bar = policy.model.min(tolerance.max(1e-12));
    let carrier_band = policy.pcurve_consistency;
    let debug =
        std::env::var("BREP_DEBUG_BOOL").is_ok() || std::env::var("BREP_DEBUG_JOINTS").is_ok();
    if std::env::var("BREP_SETTLE_VERTICES").as_deref() == Ok("0") {
        // Diagnostic A/B hatch, not a policy (the record's before column).
        return Ok(Vec::new());
    }

    // Incident faces per vertex, through the coedges' edges.
    let edge_vertices: HashMap<u64, (u64, u64)> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
        .collect();
    let faces: Vec<&crate::FaceRecord> =
        solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut vertex_faces: HashMap<u64, Vec<usize>> = HashMap::default();
    for (face_index, face) in faces.iter().enumerate() {
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                let Some(&(start, end)) = edge_vertices.get(&coedge.edge_id) else {
                    continue;
                };
                for vertex_id in [start, end] {
                    let entry = vertex_faces.entry(vertex_id).or_default();
                    if !entry.contains(&face_index) {
                        entry.push(face_index);
                    }
                }
            }
        }
    }

    let mut moves = Vec::<(u64, Vec3, Vec3, f64, f64)>::new();
    let mut declined = Vec::<String>::new();
    for vertex in &solid.vertices {
        let Some(adjacent) = vertex_faces.get(&vertex.id) else {
            continue;
        };
        if adjacent.len() < 2 {
            continue;
        }
        let surfaces: Vec<&crate::NurbsSurface> = adjacent
            .iter()
            .map(|&index| &faces[index].surface)
            .collect();
        let before = worst_residual(&surfaces, vertex.point)?;
        if before <= bar {
            continue;
        }
        if before > carrier_band {
            declined.push(format!(
                "vertex {} stands {before:.3e} off a carrier, over the {carrier_band:.3e} contract",
                vertex.id
            ));
            continue;
        }
        let limit = carrier_band.min(before * 8.0);
        let regular = settle(&surfaces, vertex.point, bar, limit)?.filter(|point| {
            worst_residual(&surfaces, *point)
                .ok()
                .is_some_and(|r| r <= bar)
        });
        let settled = regular.or_else(|| {
            settle_supported_boundary(solid, &faces, adjacent, vertex.id, vertex.point, bar, limit)
        });
        let Some(settled) = settled else {
            declined.push(format!(
                "vertex {} ({before:.3e} off): no point on all {} carriers within {limit:.3e}",
                vertex.id,
                surfaces.len()
            ));
            continue;
        };
        let after = worst_residual(&surfaces, settled)?;
        let moved = settled.sub(vertex.point).length();
        if after > bar || moved > limit || moved <= 1e-12 {
            declined.push(format!("vertex {} ({before:.3e} off): solved point {after:.3e} off, moved {moved:.3e} (limit {limit:.3e})", vertex.id));
            continue;
        }
        moves.push((vertex.id, vertex.point, settled, before, after));
    }

    let mut accepted = Vec::<VertexMove>::new();
    let mut resnapped_constructed: Vec<u64> = Vec::new();
    for (vertex_id, previous, settled, residual_before, residual_after) in moves {
        if let Some(vertex) = solid
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == vertex_id)
        {
            vertex.point = settled;
        }
        let points: HashMap<u64, Vec3> = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect();
        let mut edges = Vec::new();
        for edge in &mut solid.edges {
            if edge.start_vertex_id != vertex_id && edge.end_vertex_id != vertex_id {
                continue;
            }
            edges.push((edge.id, edge.curve.clone(), edge.t0, edge.t1));
            let start = points[&edge.start_vertex_id];
            let end = points[&edge.end_vertex_id];
            let snapped =
                snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1, start, end)?;
            [edge.t0, edge.t1] = snapped.domain().or_refuse(KernelStage::Sew, "domain")?;
            edge.curve = snapped;
        }
        for (edge_id, _, _, _) in &edges {
            if constructed.contains(edge_id) && !resnapped_constructed.contains(edge_id) {
                resnapped_constructed.push(*edge_id);
            }
        }
        if debug {
            eprintln!(
                "vertices: settled vertex {vertex_id} ({:.6}, {:.6}, {:.6}) -> ({:.6}, {:.6}, {:.6}): off carrier {residual_before:.3e} -> {residual_after:.3e}, moved {:.3e}, {} edges re-snapped",
                previous.x, previous.y, previous.z, settled.x, settled.y, settled.z, settled.sub(previous).length(), edges.len()
            );
        }
        accepted.push(VertexMove {
            vertex_id,
            previous,
            residual_before,
            residual_after,
            edges,
            pcurves: Vec::new(),
        });
    }
    // Second phase, once EVERY vertex has settled: a section edge's two ends
    // may both have moved, and the edge-vs-carrier reading below must see the
    // edge as it finally stands, not with one end still off its carriers.
    let mut pcurves = Vec::new();
    let mut counts = [0usize; 4];
    let mut candidates = Vec::new();
    // `BREP_SECTION_TRIM_REFRESH=0`: diagnostic A/B hatch, not a policy (the
    // record's before column, the same binary).
    let refresh = std::env::var("BREP_SECTION_TRIM_REFRESH").as_deref() != Ok("0");
    for edge_id in resnapped_constructed.iter().filter(|_| refresh) {
        refresh_section_trims(solid, *edge_id, bar, debug, &mut candidates, &mut counts)?;
    }
    if !candidates.is_empty() {
        // The whole-shell reading decides. A trim rebuilt on one face while
        // its mate on the paired face is declined (that loop also holds vendor
        // coedges, §4 of the record) turns a pair of trims that imaged to the
        // same wrong curve — cancelling in the shell's trim vector area — into
        // a net residual. On `anotherBooleanFail` the sphere trims' 2.2e-4
        // bend outweighed that and the residual fell 1.73e-6 → 1.19e-6; on
        // `28_box_corner_vertex_anchor_t427` ten one-sided rebuilds raised it
        // 2.296e-4 → 2.404e-4. So the refresh stands only when no closed
        // shell's trim residual reads worse than before; otherwise every
        // rebuilt trim goes back and the set counts as declined.
        let before = crate::shell_vector_areas(solid);
        apply_trim_candidates(solid, candidates, bar, debug, &mut pcurves, &mut counts)?;
        if !pcurves.is_empty() {
            let after = crate::shell_vector_areas(solid);
            let readable = before.unreadable.is_empty() && after.unreadable.is_empty() && before.shells.len() == after.shells.len();
            // Fail closed: the same shells (ids and closedness, in order) on
            // both reads, and each residual finite BEFORE and after; an unread
            // or non-finite reading is never clean (`ra > NaN` is false, so a
            // NaN before would otherwise pass).
            let worse = !readable
                || before.shells.iter().zip(&after.shells).any(|(b, a)| {
                    let (rb, ra) = (b.trim_residual.length(), a.trim_residual.length());
                    b.shell != a.shell || b.closed != a.closed || !rb.is_finite() || !ra.is_finite() || ra > rb
                });
            if worse {
                if debug {
                    for (b, a) in before.shells.iter().zip(&after.shells) {
                        eprintln!(
                            "vertices: shell {} trim residual {:.3e} -> {:.3e} (bar {:.3e}): the {} rebuilt trim(s) go back",
                            b.shell, b.trim_residual.length(), a.trim_residual.length(), a.bar, pcurves.len()
                        );
                    }
                    if !readable {
                        eprintln!("vertices: closure scan unreadable before/after ({} / {}), the rebuilt trims go back", before.unreadable.len(), after.unreadable.len());
                    }
                }
                let reverted = pcurves.len();
                for (face_id, loop_id, coedge_id, pcurve) in pcurves.drain(..) {
                    for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces).filter(|face| face.id == face_id) {
                        for loop_record in face.loops.iter_mut().filter(|lp| lp.id == loop_id) {
                            for coedge in loop_record.coedges.iter_mut().filter(|c| c.id == coedge_id) {
                                coedge.pcurve = pcurve.clone();
                            }
                        }
                    }
                }
                counts[TrimRefresh::Rebuilt as usize] -= reverted;
                counts[TrimRefresh::Declined as usize] += reverted;
            } else if debug {
                for (b, a) in before.shells.iter().zip(&after.shells) {
                    eprintln!("vertices: shell {} trim residual {:.3e} -> {:.3e} (bar {:.3e}): the rebuilt trims stand", b.shell, b.trim_residual.length(), a.trim_residual.length(), a.bar);
                }
            }
        }
    }
    if debug && counts.iter().any(|count| *count > 0) {
        eprintln!(
            "vertices: constructed-section trims on {} re-snapped edges: {} meet, {} trim-as-witness, {} rebuilt, {} declined",
            resnapped_constructed.len(),
            counts[TrimRefresh::Meets as usize], counts[TrimRefresh::TrimIsWitness as usize], counts[TrimRefresh::Rebuilt as usize], counts[TrimRefresh::Declined as usize]
        );
    }
    if let Some(last) = accepted.last_mut() {
        // Restored together with the vertices: `restore_vertices` puts every
        // move's pcurves back, so they ride on the last accepted move.
        last.pcurves = pcurves;
    }
    if debug {
        eprintln!(
            "vertices: {} settled, {} declined (bar {bar:.1e})",
            accepted.len(),
            declined.len()
        );
        for line in &declined {
            eprintln!("vertices:   declined {line}");
        }
    }
    Ok(accepted)
}


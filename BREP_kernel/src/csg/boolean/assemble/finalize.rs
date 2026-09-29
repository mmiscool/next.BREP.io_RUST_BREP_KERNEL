use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use super::*;
use super::builder::snap_edge_curve_endpoints;
use super::edge_conform::edge_lies_on;
use super::polish::{collapse_short_one_use_edges, sew_coincident_one_use_edges};

/// Recover a shared 3D edge from two agreeing face p-curves when arrangement
/// welding selected an obsolete carrier curve at a pole transition.
fn repair_edges_from_consistent_pcurves(
    solid: &mut BrepSolid,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    // Assembly/search tolerances may be deliberately broad. Repairs are
    // governed by the committed pcurve contract so a search radius cannot
    // authorize geometry that the final validator must reject.
    let pcurve_contract = KernelTolerances::for_solid(solid, 1e-7).pcurve_consistency;
    let repair_tolerance = tolerance.min(pcurve_contract * 0.75);
    #[derive(Clone)]
    struct Use {
        face: usize,
        loop_index: usize,
        coedge: usize,
    }
    let faces = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let edges = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let points = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let mut uses = HashMap::<u64, Vec<Use>>::default();
    for (face_index, face) in faces.iter().enumerate() {
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                uses.entry(coedge.edge_id).or_default().push(Use {
                    face: face_index,
                    loop_index,
                    coedge: coedge_index,
                });
            }
        }
    }
    let traversal_start = |coedge: &CoedgeRecord| {
        let edge = edges[&coedge.edge_id];
        if coedge.forward {
            edge.start_vertex_id
        } else {
            edge.end_vertex_id
        }
    };
    let traversal_end = |coedge: &CoedgeRecord| {
        let edge = edges[&coedge.edge_id];
        if coedge.forward {
            edge.end_vertex_id
        } else {
            edge.start_vertex_id
        }
    };
    let mut repairs = Vec::new();
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        let Some(edge_uses) = uses.get(&edge.id).filter(|uses| uses.len() == 2) else {
            continue;
        };
        let first = &edge_uses[0];
        let first_face = faces[first.face];
        let first_loop = &first_face.loops[first.loop_index];
        let first_coedge = &first_loop.coedges[first.coedge];
        let previous = &first_loop.coedges
            [(first.coedge + first_loop.coedges.len() - 1) % first_loop.coedges.len()];
        let next = &first_loop.coedges[(first.coedge + 1) % first_loop.coedges.len()];
        let intended_traversal_start = traversal_end(previous);
        let intended_traversal_end = traversal_start(next);
        let [p0, p1] = first_coedge.pcurve.domain().or_refuse(KernelStage::Validate, "domain")?;
        // Repairs are uncommon and must satisfy the adaptive validator between
        // interpolation nodes. Use a dense carrier trace for every degree;
        // sparse samples can leave localized SSI-fit drift on tight offsets.
        let sample_count = 256;
        let mut traversal_points = Vec::with_capacity(sample_count + 1);
        let mut parameters = Vec::with_capacity(sample_count + 1);
        for index in 0..=sample_count {
            let fraction = index as f64 / sample_count as f64;
            let uv = first_coedge.pcurve.evaluate(p0 + (p1 - p0) * fraction).or_refuse(KernelStage::Validate, "evaluate")?;
            traversal_points.push(first_face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Validate, "evaluate")?);
            parameters.push(fraction);
        }
        let worst_current = adaptive_coedge_error(
            &first_face.surface,
            &first_coedge.pcurve,
            &edge.curve,
            edge,
            first_coedge.forward,
            repair_tolerance,
        ).or_refuse(KernelStage::Validate, "csg.boolean.assemble.finalize")?;
        if worst_current <= repair_tolerance {
            continue;
        }
        let repair_debug =
            std::env::var("BREP_OS_DEBUG").is_ok_and(|value| !value.is_empty() && value != "0");
        let second = &edge_uses[1];
        let second_face = faces[second.face];
        let second_coedge = &second_face.loops[second.loop_index].coedges[second.coedge];
        let [q0, q1] = second_coedge.pcurve.domain().or_refuse(KernelStage::Validate, "domain")?;
        let mut worst_pair = 0.0f64;
        let pcurves_agree = (0..=16).all(|index| {
            let fraction = index as f64 / 16.0;
            let uv = second_coedge.pcurve.evaluate(q0 + (q1 - q0) * fraction);
            let canonical_fraction = if second_coedge.forward {
                fraction
            } else {
                1.0 - fraction
            };
            let first_fraction = if first_coedge.forward {
                canonical_fraction
            } else {
                1.0 - canonical_fraction
            };
            uv.and_then(|uv| second_face.surface.evaluate(uv.x, uv.y))
                .is_ok_and(|point| {
                    first_coedge
                        .pcurve
                        .evaluate(p0 + (p1 - p0) * first_fraction)
                        .and_then(|first_uv| first_face.surface.evaluate(first_uv.x, first_uv.y))
                        .is_ok_and(|first_point| {
                            let deviation = point.sub(first_point).length();
                            worst_pair = worst_pair.max(deviation);
                            deviation <= repair_tolerance
                        })
                })
        });
        if repair_debug {
            eprintln!(
                "repair candidate edge {} worst_current={worst_current:.6} worst_pair={worst_pair:.6} agree={pcurves_agree} tol={repair_tolerance:.6}",
                edge.id,
            );
        }
        // A shared edge is one source of truth for both incident carriers.
        // Reconstruct it from the midpoint of the two agreeing pcurve traces,
        // rather than privileging whichever face happened to be visited first.
        let mut canonical_points = traversal_points.clone();
        if pcurves_agree {
            let mut averaged = Vec::with_capacity(traversal_points.len());
            let mut dense_pair_agrees = true;
            for (index, first_point) in traversal_points.iter().copied().enumerate() {
                let first_fraction = index as f64 / sample_count as f64;
                let canonical_fraction = if first_coedge.forward {
                    first_fraction
                } else {
                    1.0 - first_fraction
                };
                let second_fraction = if second_coedge.forward {
                    canonical_fraction
                } else {
                    1.0 - canonical_fraction
                };
                let second_uv = second_coedge
                    .pcurve
                    .evaluate(q0 + (q1 - q0) * second_fraction).or_refuse(KernelStage::Validate, "evaluate")?;
                let second_point = second_face.surface.evaluate(second_uv.x, second_uv.y).or_refuse(KernelStage::Validate, "evaluate")?;
                if first_point.sub(second_point).length() > repair_tolerance {
                    dense_pair_agrees = false;
                    break;
                }
                averaged.push(first_point.add(second_point).scale(0.5));
            }
            if dense_pair_agrees {
                canonical_points = averaged;
            }
        }
        let (start_vertex_id, end_vertex_id) = if first_coedge.forward {
            (intended_traversal_start, intended_traversal_end)
        } else {
            canonical_points.reverse();
            (intended_traversal_end, intended_traversal_start)
        };
        canonical_points[0] = points[&start_vertex_id];
        let last = canonical_points.len() - 1;
        canonical_points[last] = points[&end_vertex_id];
        let curve = interpolate_curve(&canonical_points, edge.curve.degree.min(3), &parameters).or_refuse(KernelStage::Validate, "interpolate_curve")?;
        let [candidate_t0, candidate_t1] = curve.domain().or_refuse(KernelStage::Validate, "domain")?;
        let candidate_edge = EdgeRecord {
            id: edge.id,
            curve: curve.clone(),
            t0: candidate_t0,
            t1: candidate_t1,
            start_vertex_id,
            end_vertex_id,
            degenerate: false,
            name: edge.name.clone(),
        };
        let mut first_pcurve = first_coedge.pcurve.clone();
        let mut second_pcurve = second_coedge.pcurve.clone();
        let mut first_error = adaptive_coedge_error(
            &first_face.surface,
            &first_pcurve,
            &curve,
            &candidate_edge,
            first_coedge.forward,
            pcurve_contract,
        ).or_refuse(KernelStage::Validate, "csg.boolean.assemble.finalize")?;
        let mut second_error = adaptive_coedge_error(
            &second_face.surface,
            &second_pcurve,
            &curve,
            &candidate_edge,
            second_coedge.forward,
            pcurve_contract,
        ).or_refuse(KernelStage::Validate, "csg.boolean.assemble.finalize")?;
        let mut rebuilt_pcurves = false;
        if first_error.max(second_error) > pcurve_contract {
            // A rebuilt trim off its bar is this candidate declining, not the
            // boolean refusing: the builder now refuses a fit it cannot bring
            // within its ask plus the curve's own standoff (2026-09-26), where
            // it used to return the fit and let the contract check below skip
            // the candidate. Same outcome, stated by the builder.
            let rebuilt = crate::pcurve::build_pcurve_on_surface_range_classed(
                &first_face.surface,
                &curve,
                candidate_t0,
                candidate_t1,
                first_coedge.forward,
                repair_tolerance,
                KernelStage::Validate,
                "csg.boolean.assemble.finalize",
            )
            .and_then(|first| {
                crate::pcurve::build_pcurve_on_surface_range_classed(
                    &second_face.surface,
                    &curve,
                    candidate_t0,
                    candidate_t1,
                    second_coedge.forward,
                    repair_tolerance,
                    KernelStage::Validate,
                    "csg.boolean.assemble.finalize",
                )
                .map(|second| (first, second))
            });
            match rebuilt {
                Ok((first, second)) => {
                    first_pcurve = first;
                    second_pcurve = second;
                }
                // Dispatched on the CLASS the builder mints, not its text.
                Err(refusal) if crate::pcurve::is_pcurve_off_bar(&refusal) => continue,
                Err(refusal) => return Err(refusal),
            }
            first_error = adaptive_coedge_error(
                &first_face.surface,
                &first_pcurve,
                &curve,
                &candidate_edge,
                first_coedge.forward,
                pcurve_contract,
            ).or_refuse(KernelStage::Validate, "csg.boolean.assemble.finalize")?;
            second_error = adaptive_coedge_error(
                &second_face.surface,
                &second_pcurve,
                &curve,
                &candidate_edge,
                second_coedge.forward,
                pcurve_contract,
            ).or_refuse(KernelStage::Validate, "csg.boolean.assemble.finalize")?;
            rebuilt_pcurves = true;
        }
        if first_error.max(second_error) > pcurve_contract
            || first_error.max(second_error) >= worst_current
        {
            continue;
        }
        if repair_debug {
            let [r0, r1] = curve.domain().or_refuse(KernelStage::Validate, "domain")?;
            let mut residual = 0.0f64;
            for (sample_index, target) in canonical_points.iter().enumerate() {
                let fraction = sample_index as f64 / (canonical_points.len() - 1) as f64;
                if let Ok(point) = curve.evaluate(r0 + (r1 - r0) * fraction) {
                    residual = residual.max(point.sub(*target).length());
                }
            }
            eprintln!("repair edge {} applied, residual={residual:.6}", edge.id);
        }
        let pcurve_repairs = if rebuilt_pcurves {
            vec![
                (first.clone(), first_pcurve),
                (second.clone(), second_pcurve),
            ]
        } else {
            Vec::<(Use, NurbsCurve)>::new()
        };
        repairs.push((
            edge.id,
            curve,
            start_vertex_id,
            end_vertex_id,
            pcurve_repairs,
        ));
    }
    drop(faces);
    drop(edges);
    for (edge_id, curve, start_vertex_id, end_vertex_id, pcurve_repairs) in repairs {
        let edge = solid
            .edges
            .iter_mut()
            .find(|edge| edge.id == edge_id)
            .unwrap();
        let domain = curve.domain().or_refuse(KernelStage::Validate, "domain")?;
        edge.curve = curve;
        edge.t0 = domain[0];
        edge.t1 = domain[1];
        edge.start_vertex_id = start_vertex_id;
        edge.end_vertex_id = end_vertex_id;
        let mut mutable_faces = solid
            .shells
            .iter_mut()
            .flat_map(|shell| shell.faces.iter_mut())
            .collect::<Vec<_>>();
        for (location, pcurve) in pcurve_repairs {
            mutable_faces[location.face].loops[location.loop_index].coedges[location.coedge]
                .pcurve = pcurve;
        }
    }
    Ok(())
}

/// Commit assembler endpoint welds into edge geometry. Candidate matching is
/// intentionally broader than model accuracy; once a candidate is accepted
/// as one topological vertex, the 3D edge must share that vertex exactly.
///
/// Existing pcurves retain their established periodic branch. The correction
/// radius is far below their consistency contract, and rebuilding can choose
/// the opposite seam branch on a closed carrier.
///
/// Also reused to heal §6.9 fillet/chamfer surgery output, whose re-trimmed
/// original edges can drift from the freshly-built blend corner vertices by
/// the amplified solver/intersection noise of the input (see
/// `fillet::heal_edge_vertex_gaps`).
pub(crate) fn commit_nearby_edge_endpoints(
    solid: &mut BrepSolid,
    search_tolerance: f64,
) -> Result<(), KernelRefusal> {
    let points = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let strict_endpoint_tolerance = 1e-5;
    let commit_radius = commit_weld(search_tolerance);
    for edge in &mut solid.edges {
        if edge.degenerate {
            continue;
        }
        let Some(start) = points.get(&edge.start_vertex_id).copied() else {
            continue;
        };
        let Some(end) = points.get(&edge.end_vertex_id).copied() else {
            continue;
        };
        let curve_start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Validate, "evaluate")?;
        let curve_end = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Validate, "evaluate")?;
        let start_gap = curve_start.sub(start).length();
        let end_gap = curve_end.sub(end).length();
        if start_gap <= strict_endpoint_tolerance && end_gap <= strict_endpoint_tolerance {
            continue;
        }
        if start_gap > commit_radius || end_gap > commit_radius {
            continue;
        }
        let curve = snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1, start, end)?;
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Validate, "domain")?;
        edge.curve = curve;
        edge.t0 = t0;
        edge.t1 = t1;
    }
    Ok(())
}

/// The exact heal sequence shared by `finalize_assembled_solid` (the offset
/// pipeline) and the boolean assembly repair fallback (roadmap T4.4): commit
/// nearby endpoint welds into edge geometry, collapse tolerance-length one-use
/// slivers, sew coincident one-use pairs, and recover shared edges from
/// agreeing pcurves. No genus/validation/volume gating — that stays with each
/// caller, so this helper's behavior is byte-identical to the inlined sequence
/// it replaces in `finalize_assembled_solid`.
pub(crate) fn apply_assembly_heal_chain(
    solid: &mut BrepSolid,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    commit_nearby_edge_endpoints(solid, tolerance)?;
    collapse_short_one_use_edges(solid, tolerance * 4.0)?;
    sew_coincident_one_use_edges(solid, tolerance)?;
    repair_edges_from_consistent_pcurves(solid, tolerance)?;
    Ok(())
}

pub(crate) fn finalize_assembled_solid(
    mut solid: BrepSolid,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    apply_assembly_heal_chain(&mut solid, tolerance)?;
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    let face_count: i64 = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len() as i64)
        .sum();
    let hole_count = solid.bounding_hole_count();
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let non_degenerate_vertex_ids = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    let vertex_count = solid
        .vertices
        .iter()
        .filter(|vertex| non_degenerate_vertex_ids.contains(&vertex.id))
        .count() as i64;
    let euler = vertex_count - edge_count + face_count - hole_count;
    let numerator = solid.shells.len() as i64 * 2 - euler;
    // A numerator that is negative or odd supports no genus at all, on ANY
    // body. Until 2026-09-12 a body whose only single-use edges were
    // degenerate (pole edges) was exempted here and, when its numerator came
    // out negative or odd, had `genus = 0` FORCED below instead. Instrumented
    // over the 18-shell `shell_euler_probe` sweep, the 49-case gate and the
    // 129 `offset` lib tests, that exemption was reached 23 times and the
    // forced arm fired 0 times — every pole-carrying body the project can
    // build has an even, non-negative numerator — so both were removed rather
    // than trusted. A body that did reach here with such a numerator now
    // takes this refusal, which the case gate classifies, instead of
    // recording a 0 that `validate()` would later report as a GENUS MISMATCH
    // against the forced number — a refusal that misdescribed its own cause.
    if numerator < 0 || numerator % 2 != 0 {
        let edge_by_id = solid
            .edges
            .iter()
            .map(|edge| (edge.id, edge))
            .collect::<HashMap<_, _>>();
        let point_by_id = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        let open = use_counts
            .iter()
            .filter(|(_, count)| **count == 1)
            .filter_map(|(edge_id, _)| {
                let edge = edge_by_id.get(edge_id)?;
                Some((
                    *edge_id,
                    point_by_id[&edge.start_vertex_id],
                    point_by_id[&edge.end_vertex_id],
                ))
            })
            .collect::<Vec<_>>();
        let open_faces = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| {
                face.loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .filter(|coedge| use_counts.get(&coedge.edge_id) == Some(&1))
                    .map(|coedge| (coedge.edge_id, face.id))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let open_ids = use_counts
            .iter()
            .filter(|(_, count)| **count == 1)
            .map(|(edge_id, _)| *edge_id)
            .collect::<Vec<_>>();
        let overused = use_counts
            .iter()
            .filter(|(_, count)| **count > 2)
            .map(|(edge_id, count)| (*edge_id, *count))
            .collect::<Vec<_>>();
        let mut open_matches = Vec::new();
        for (index, first_id) in open_ids.iter().enumerate() {
            for second_id in open_ids.iter().skip(index + 1) {
                if edge_lies_on(edge_by_id[first_id], edge_by_id[second_id], 2e-3).unwrap_or(false)
                    || edge_lies_on(edge_by_id[second_id], edge_by_id[first_id], 2e-3)
                        .unwrap_or(false)
                {
                    open_matches.push((*first_id, *second_id));
                }
            }
        }
        return Err(KernelRefusal::new(
            RefusalClass::NonIntegralGenus {
                shells: solid.shells.len() as u32,
                euler: vertex_count as i64 - edge_count as i64 + face_count as i64 - hole_count as i64,
            },
            KernelStage::Validate,
            format!(
            "boolean assembly: completion produced non-integral genus \
             (V={} E={} F={} H={} S={} one_use={} {open:?} \
              faces={open_faces:?} matches={open_matches:?} overused={overused:?})",
            vertex_count,
            edge_count,
            face_count,
            hole_count,
            solid.shells.len(),
            use_counts.values().filter(|count| **count == 1).count(),
        )));
    }
    // Record the genus the count produced, on every body. This used to be
    // forced to 0 for a body whose only single-use edges are degenerate,
    // which is not a safe default but a statement the counts contradict: that
    // is true of any body carrying an ordinary pole edge, so a shelled solid
    // with a spherical cavity wall was recorded genus 0 while its own
    // `V - E + F - H` said 3. `validate()` then disagreed with itself, and the
    // disagreement was invisible because its Euler check was skipped on
    // exactly these bodies. Measured on
    // `sphere_carved_opening_shell_is_watertight`: chi = -4, numerator 6, so
    // genus 3 -- and an independent WATERTIGHT MESH count of the same shell
    // reads chi = -4 at three chord tolerances, closed, no boundary or
    // non-manifold edges. The handles are real; only the recorded number was
    // wrong. A shell opened on an ANNULAR face joins its outer and inner
    // skins along two rim curves rather than one, so a non-zero genus there
    // is the expected answer, not a symptom.
    solid.genus = numerator / 2;
    let repair_debug =
        std::env::var("BREP_OS_DEBUG").is_ok_and(|value| !value.is_empty() && value != "0");
    if repair_debug {
        let pre_issues = solid.validate();
        eprintln!(
            "pre-merge validation issues: {:?}",
            pre_issues
                .iter()
                .map(|issue| &issue.message)
                .collect::<Vec<_>>(),
        );
    }
    let solid = merge_curve_continuation_edges(&solid, tolerance).or_refuse(KernelStage::Validate, "merge_curve_continuation_edges")?;
    let issues = solid.validate();
    if !issues.is_empty() {
        let edge_by_id = solid
            .edges
            .iter()
            .map(|edge| (edge.id, edge))
            .collect::<HashMap<_, _>>();
        let point_by_id = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        let mut counts = HashMap::<u64, usize>::default();
        for coedge in solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *counts.entry(coedge.edge_id).or_default() += 1;
        }
        let open = counts
            .iter()
            .filter(|(_, count)| **count == 1)
            .filter_map(|(edge_id, _)| {
                let edge = edge_by_id.get(edge_id)?;
                Some((
                    *edge_id,
                    point_by_id[&edge.start_vertex_id],
                    point_by_id[&edge.end_vertex_id],
                ))
            })
            .collect::<Vec<_>>();
        return Err(KernelRefusal::new(
            RefusalClass::DegenerateArrangement {
                open_edges: open.len() as u32,
                issues: issues.len() as u32,
            },
            KernelStage::Validate,
            format!(
            "boolean assembly: completed shell has invalid topology: {issues:?}; open={open:?}"
        )));
    }
    let _caller = crate::mass_caller("finalize.closed_tail");
    let volume = solid_signed_volume(&solid).or_refuse(KernelStage::Validate, "solid_signed_volume")?;
    if volume <= tolerance.powi(3) {
        return Err(KernelRefusal::new(
            RefusalClass::NonPositiveVolume,
            KernelStage::Validate,
            "boolean assembly produced non-positive volume",
        ));
    }
    Ok(solid)
}

/// Exact repair fallback for a CLOSED boolean assembly (roadmap T4.4).
///
/// When `assemble_fragments_impl` is about to refuse a closed result because
/// one-use edges (open boundaries left by independently-fitted intersection
/// endpoints drifting beyond the assembler weld) push the Euler/genus count off
/// an integer, this offers the SAME heal chain the offset pipeline already runs
/// via `finalize_assembled_solid` — commit endpoints, collapse short one-use
/// slivers, sew coincident one-use pairs, repair pcurves — then re-checks the
/// result and continues the normal post-assembly path (curve-continuation
/// merge + triple-junction polish + validate + volume gate).
///
/// The healed solid is accepted ONLY when it is genuinely a valid closed
/// manifold: NO one-use edges remain, the recomputed genus is integral, and
/// `validate()` is empty (plus the normal post path succeeds with positive
/// volume). Otherwise it returns `None` so the caller falls through to its
/// original, unmodified diagnostic error — the repair never masks a real
/// intersector failure and never returns anything worse than the error it
/// replaces. Runs ONLY on the path that already errors, so every currently
/// succeeding boolean is byte-for-byte untouched.
pub(in crate::boolean) fn repair_open_assembly_via_heal_chain(
    provisional: BrepSolid,
    tolerance: f64,
) -> Option<BrepSolid> {
    let mut solid = provisional;
    // Shared heal sequence (identical to finalize_assembled_solid).
    apply_assembly_heal_chain(&mut solid, tolerance).ok()?;

    // A closed manifold has every edge used by exactly two coedges. Decline
    // the moment any one-use edge survives the heal (no masking of open shells).
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    if use_counts.values().any(|count| *count == 1) {
        return None;
    }

    // Recompute the Euler/genus exactly as the normal assembly path does
    // (all vertices; non-degenerate edges) so the accepted solid's genus is
    // computed identically to a normally-assembled one.
    let face_count: i64 = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len() as i64)
        .sum();
    let hole_count: i64 = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| face.loops.len().saturating_sub(1) as i64)
        .sum();
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let euler = solid.vertices.len() as i64 - edge_count + face_count - hole_count;
    let numerator = solid.shells.len() as i64 * 2 - euler;
    if numerator < 0 || numerator % 2 != 0 {
        return None;
    }
    if !solid.validate().is_empty() {
        return None;
    }
    solid.genus = numerator / 2;

    // Continue the normal post-assembly path with the healed solid. Any
    // failure here means the repair declines (fall through to the caller's
    // original error) rather than surfacing a different/worse failure.
    let mut solid = merge_curve_continuation_edges(&solid, tolerance).ok()?;
    polish_triple_junction_vertices(&mut solid, tolerance).ok()?;
    if !solid.validate().is_empty() {
        return None;
    }
    let _caller = crate::mass_caller("finalize.heal_fallback");
    let volume = solid_signed_volume(&solid).ok()?;
    if volume <= tolerance.powi(3) {
        return None;
    }
    Some(solid)
}


use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use super::types::{Chain, ChainSource};
use super::sampling::{point_segment_distance, subcurve_by_fraction};
pub(super) fn edge_for<'a>(solid: &'a BrepSolid, edge_id: u64) -> Result<&'a EdgeRecord, KernelRefusal> {
    solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!("fragment_face: missing edge {edge_id}")))
}

pub(super) fn vertex_point(solid: &BrepSolid, vertex_id: u64) -> Result<Vec3, KernelRefusal> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == vertex_id)
        .map(|vertex| vertex.point)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!("fragment_face: missing vertex {vertex_id}")))
}

pub(super) fn pcurve_for<'a>(piece: &'a ImprintPieceRecord, face: FaceKey) -> Result<&'a NurbsCurve, KernelRefusal> {
    piece
        .pcurves
        .iter()
        .find(|pcurve| pcurve.operand == face.operand && pcurve.face_id == face.face_id)
        .map(|pcurve| &pcurve.pcurve)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: imprint lacks face pcurve"))
}

pub(super) fn point_in_polygon(point: Vec2, polygon: &[Vec2]) -> bool {
    let mut inside = false;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if (a.y > point.y) != (b.y > point.y) {
            let crossing = a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if crossing > point.x {
                inside = !inside;
            }
        }
    }
    inside
}

/// Interior points of the region, best-clearance first.  The first entry
/// is the same point the single-point selection historically produced;
/// later entries are additional well-separated interior points used as
/// extra classification votes in boolean fragment selection.
pub(super) fn interior_points(
    outer: &[crate::CycleUse],
    holes: &[Vec<crate::CycleUse>],
    area: f64,
    count: usize,
) -> Vec<Vec2> {
    let polygon = |cycle: &[crate::CycleUse]| {
        cycle
            .iter()
            .map(|usage| {
                if usage.forward {
                    usage.piece.a
                } else {
                    usage.piece.b
                }
            })
            .collect::<Vec<_>>()
    };
    let outer_polygon = polygon(outer);
    let hole_polygons = holes.iter().map(|hole| polygon(hole)).collect::<Vec<_>>();
    let scale = area.abs().sqrt();
    let mut seen: Vec<(Vec2, f64)> = Vec::new();
    let mut best = None;
    let mut best_clearance = -1.0;
    for factor in [0.25, 0.1, 0.05, 0.01, 0.002, 4e-4] {
        let epsilon = scale * factor;
        for usage in outer {
            let (a, b) = if usage.forward {
                (usage.piece.a, usage.piece.b)
            } else {
                (usage.piece.b, usage.piece.a)
            };
            let direction = b.sub(a);
            let length = direction.length();
            if length <= 1e-12 {
                continue;
            }
            let candidate = a.add(b).scale(0.5).add(
                Vec2 {
                    x: -direction.y / length,
                    y: direction.x / length,
                }
                .scale(epsilon),
            );
            if !point_in_polygon(candidate, &outer_polygon)
                || hole_polygons
                    .iter()
                    .any(|hole| point_in_polygon(candidate, hole))
            {
                continue;
            }
            let mut clearance = f64::INFINITY;
            for cycle in std::iter::once(outer).chain(holes.iter().map(Vec::as_slice)) {
                for edge in cycle {
                    clearance = clearance.min(point_segment_distance(
                        candidate,
                        edge.piece.a,
                        edge.piece.b,
                    ));
                }
            }
            seen.push((candidate, clearance));
            if clearance > best_clearance {
                best = Some(candidate);
                best_clearance = clearance;
            }
        }
        if best.is_some() && best_clearance >= epsilon * 0.5 {
            break;
        }
    }
    let Some(primary) = best else {
        return Vec::new();
    };
    let mut picked = vec![primary];
    let separation = (scale * 0.05).max(1e-12);
    seen.sort_by(|left, right| {
        right
            .1
            .partial_cmp(&left.1)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for (candidate, _) in seen {
        if picked.len() >= count {
            break;
        }
        if picked
            .iter()
            .all(|existing| existing.sub(candidate).length() >= separation)
        {
            picked.push(candidate);
        }
    }
    picked
}

fn chain_index(usage: &crate::CycleUse) -> Result<usize, KernelRefusal> {
    usage
        .piece
        .parent
        .tag
        .get("chain")
        .and_then(serde_json::Value::as_u64)
        .map(|value| value as usize)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: arrangement lost chain tag"))
}

// Returns `Ok(None)` when the cycle is a spurious zero-area "antenna" region
// (a self-touching fold that encloses no material — see the seam-grazing note at
// the incomplete-run site); the caller drops that region instead of aborting.
pub(super) fn build_loop(
    cycle: &[crate::CycleUse],
    reverse: bool,
    chains: &[Chain],
    tolerance: f64,
    surface: &NurbsSurface,
) -> Result<Option<FragmentLoop>, KernelRefusal> {
    let mut uses = cycle.to_vec();
    if reverse {
        uses.reverse();
        for usage in &mut uses {
            usage.forward = !usage.forward;
        }
    }
    let mut start = 0;
    for index in 0..uses.len() {
        if chain_index(&uses[(index + uses.len() - 1) % uses.len()])? != chain_index(&uses[index])?
        {
            start = index;
            break;
        }
    }
    uses.rotate_left(start);
    let mut coedges = Vec::new();
    let mut index = 0;
    while index < uses.len() {
        let chain_id = chain_index(&uses[index])?;
        let mut run_end_index = index + 1;
        while run_end_index < uses.len() && chain_index(&uses[run_end_index])? == chain_id {
            run_end_index += 1;
        }
        let chain = chains
            .get(chain_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", "fragment_face: unknown arrangement chain"))?;
        let run = &uses[index..run_end_index];
        let run_start = if run[0].forward {
            run[0].piece.a
        } else {
            run[0].piece.b
        };
        let last = &run[run.len() - 1];
        let run_end = if last.forward {
            last.piece.b
        } else {
            last.piece.a
        };
        let epsilon = tolerance * 20.0;
        let chain_closed = chain.start.sub(chain.end).length() <= epsilon;
        let complete = if chain_closed {
            run_start.sub(run_end).length() <= epsilon
        } else {
            (run_start.sub(chain.start).length() <= epsilon
                && run_end.sub(chain.end).length() <= epsilon)
                || (run_start.sub(chain.end).length() <= epsilon
                    && run_end.sub(chain.start).length() <= epsilon)
        };
        if !complete || run.len() < 1usize.max(chain.count / 2) {
            if !chain_closed {
                let (carrier_pcurve, carrier_curve, lift) = match &chain.source {
                    ChainSource::Boundary { coedge, curve } => (&coedge.pcurve, curve, Vec2 { x: 0.0, y: 0.0 }),
                    ChainSource::Cut { pcurve, curve, lift, .. } => (pcurve, curve, *lift),
                };
                // Back onto the pcurve's own period: the arrangement may have
                // lifted the chain a whole period into the face's band.
                let run_start = run_start.sub(lift);
                let run_end = run_end.sub(lift);
                let start_projection = project_point_to_curve(
                    carrier_pcurve,
                    Vec3::new(run_start.x, run_start.y, 0.0),
                ).or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                let end_projection =
                    project_point_to_curve(carrier_pcurve, Vec3::new(run_end.x, run_end.y, 0.0)).or_refuse(KernelStage::Fragment, "project_point_to_curve")?;
                let [p0, p1] = carrier_pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                let parameter_span = p1 - p0;
                let start_fraction = (start_projection.u - p0) / parameter_span;
                let end_fraction = (end_projection.u - p0) / parameter_span;
                let chord_tolerance = |point: Vec2| {
                    chain
                        .segments
                        .iter()
                        .min_by(|(a0, a1), (b0, b1)| {
                            point_segment_distance(point, *a0, *a1)
                                .total_cmp(&point_segment_distance(point, *b0, *b1))
                        })
                        .map(|(a, b)| b.sub(*a).length())
                        .unwrap_or(0.0)
                };
                let start_tolerance = (epsilon * 20.0).max(chord_tolerance(run_start) * 1.01);
                let end_tolerance = (epsilon * 20.0).max(chord_tolerance(run_end) * 1.01);
                if std::env::var("BREP_DEBUG_FRAG").is_ok() {
                    eprintln!(
                        "partial run: chain {chain_id} count={} run={} complete={complete} span={parameter_span:.6e} \
                         start(d={:.6e} tol={:.6e} f={:.9}) end(d={:.6e} tol={:.6e} f={:.9})",
                        chain.count,
                        run.len(),
                        start_projection.distance,
                        start_tolerance,
                        start_fraction,
                        end_projection.distance,
                        end_tolerance,
                        end_fraction,
                    );
                }
                if parameter_span > 1e-12
                    && start_projection.distance <= start_tolerance
                    && end_projection.distance <= end_tolerance
                    && (end_fraction - start_fraction).abs() > 1e-12
                {
                    let low = start_fraction.min(end_fraction).clamp(0.0, 1.0);
                    let high = start_fraction.max(end_fraction).clamp(0.0, 1.0);
                    let mut edge_curve = subcurve_by_fraction(carrier_curve, low, high)?;
                    let mut pcurve = subcurve_by_fraction(carrier_pcurve, low, high)?;
                    let forward = end_fraction > start_fraction;
                    let [partial_p0, partial_p1] = pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                    let endpoint_uv_start = pcurve.evaluate(partial_p0).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let endpoint_uv_end = pcurve.evaluate(partial_p1).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let desired_low = if forward { run_start } else { run_end };
                    let desired_high = if forward { run_end } else { run_start };
                    let endpoint_gap = surface
                        .evaluate(endpoint_uv_start.x, endpoint_uv_start.y).or_refuse(KernelStage::Fragment, "evaluate")?
                        .sub(surface.evaluate(desired_low.x, desired_low.y).or_refuse(KernelStage::Fragment, "evaluate")?)
                        .length()
                        .max(
                            surface
                                .evaluate(endpoint_uv_end.x, endpoint_uv_end.y).or_refuse(KernelStage::Fragment, "evaluate")?
                                .sub(surface.evaluate(desired_high.x, desired_high.y).or_refuse(KernelStage::Fragment, "evaluate")?)
                                .length(),
                        );
                    if endpoint_gap > 2e-3 {
                        let parameters = (0..=32)
                            .map(|sample| sample as f64 / 32.0)
                            .collect::<Vec<_>>();
                        let mut uv_points = parameters
                            .iter()
                            .map(|fraction| {
                                pcurve.evaluate(partial_p0 + (partial_p1 - partial_p0) * fraction)
                            })
                            .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                        uv_points[0] = Vec3::new(desired_low.x, desired_low.y, 0.0);
                        *uv_points.last_mut().unwrap() =
                            Vec3::new(desired_high.x, desired_high.y, 0.0);
                        pcurve = interpolate_curve(&uv_points, 1, &parameters).or_refuse(KernelStage::Fragment, "interpolate_curve")?;
                        let edge_points = uv_points
                            .iter()
                            .map(|uv| surface.evaluate(uv.x, uv.y))
                            .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Fragment, "csg.fragment.loops")?;
                        edge_curve = interpolate_curve(&edge_points, 1, &parameters).or_refuse(KernelStage::Fragment, "interpolate_curve")?;
                    }
                    let [t0, t1] = edge_curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                    let start = edge_curve.evaluate(t0).or_refuse(KernelStage::Fragment, "evaluate")?;
                    let end = edge_curve.evaluate(t1).or_refuse(KernelStage::Fragment, "evaluate")?;
                    coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::Derived {
                            curve: edge_curve,
                            t0,
                            t1,
                            start,
                            end,
                        },
                        forward,
                        pcurve: if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")? },
                    });
                    index = run_end_index;
                    continue;
                }
            }
            // A run that departs from and returns to the SAME point along an
            // OPEN chain is a spurious zero-area "antenna" — a self-touching
            // fold, not a real boundary. This is exactly how a periodic-band
            // face's seam-grazing SSI cut manifests: near the u-seam the cut's
            // in-domain-clamped pcurve folds back on itself, and `arrange_segments`
            // carves off a degenerate micro-loop region bounded solely by that
            // fold (observed param area ~1e-9). That region encloses no material
            // and has no counterpart on the mating (non-periodic) face, so signal
            // the caller to DROP the whole region rather than aborting the entire
            // face's fragmentation. The real, full-area region that also borders
            // this chain still traverses it as one complete run and is unaffected.
            if !chain_closed && run_start.sub(run_end).length() <= epsilon {
                return Ok(None);
            }
            if std::env::var("BREP_DEBUG_FRAG").is_ok() {
                eprintln!(
                    "incomplete run: chain {chain_id} count={} closed={} start=({:.9},{:.9}) end=({:.9},{:.9}); run len={} start=({:.9},{:.9}) end=({:.9},{:.9}) epsilon={:.3e}",
                    chain.count, chain_closed, chain.start.x, chain.start.y, chain.end.x, chain.end.y,
                    run.len(), run_start.x, run_start.y, run_end.x, run_end.y, epsilon
                );
                for usage in run {
                    eprintln!("   use fwd={} a=({:.9},{:.9}) b=({:.9},{:.9})", usage.forward, usage.piece.a.x, usage.piece.a.y, usage.piece.b.x, usage.piece.b.y);
                }
            }
            return Err(KernelRefusal::internal(KernelStage::Fragment, "fragment.loops", format!(
                "fragment_face: chain {chain_id} fragmented into an incomplete run"
            )));
        }
        let chain_forward = run[0].forward;
        match &chain.source {
            ChainSource::Boundary { coedge: source, .. } => coedges.push(FragmentCoedge {
                source: FragmentEdgeSource::Boundary {
                    operand: 0, // replaced by caller after construction
                    edge_id: source.edge_id,
                },
                forward: if chain_forward {
                    source.forward
                } else {
                    !source.forward
                },
                pcurve: if chain_forward {
                    source.pcurve.clone()
                } else {
                    source.pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                },
            }),
            ChainSource::Cut {
                piece_id,
                pcurve,
                shared_ring,
                ..
            } => {
                // CLOSED-RING SHARED SECTION (equator-tangent): a closed
                // piece marked shared_edge coincides with an existing
                // boundary RING, which the assembler's endpoint weld can
                // never unify (no distinct endpoints). Reference the owning
                // boundary edge DIRECTLY so both operands' fragments share
                // one edge record by construction. Open shared arcs keep the
                // landed endpoint-weld path unchanged.
                match shared_ring {
                    Some((operand, edge_id, aligned)) => {
                        if std::env::var("BREP_DEBUG_SHARED_SECTION").is_ok() {
                            eprintln!("fragment: SharedBoundary emit {}:{} aligned={} fwd={}", operand, edge_id, aligned, chain_forward);
                        }
                        coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::SharedBoundary { operand: *operand, edge_id: *edge_id },
                        forward: chain_forward == *aligned,
                        pcurve: if chain_forward {
                            pcurve.clone()
                        } else {
                            pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                        },
                    })
                    }
                    None => coedges.push(FragmentCoedge {
                        source: FragmentEdgeSource::Imprint {
                            piece_id: *piece_id,
                        },
                        forward: chain_forward,
                        pcurve: if chain_forward {
                            pcurve.clone()
                        } else {
                            pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
                        },
                    }),
                }
            }
        }
        index = run_end_index;
    }
    Ok(Some(FragmentLoop { coedges }))
}


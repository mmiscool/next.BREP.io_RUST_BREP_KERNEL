use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// One rim loop's witness that it wraps the full closed-u period: the cyclic
/// coedge pair where the traversal ARRIVES at one u-domain extreme and
/// DEPARTS from the other, sharing a model vertex that sits exactly on the
/// surface seam meridian.
struct BandWrapPair {
    /// Index of the departing coedge; rotating the loop to START here makes
    /// it END with the arriving coedge, i.e. at the seam.
    depart_index: usize,
    /// Whether the arrival lands on the u1 (max) domain edge.
    arrive_at_max: bool,
    /// Rim level (v) at the seam vertex.
    v: f64,
    /// The shared model vertex at the seam foot.
    vertex_id: u64,
}

fn band_wrap_pair(
    loop_record: &LoopRecord,
    edges: &HashMap<u64, &EdgeRecord>,
    u_domain: [f64; 2],
    u_eps: f64,
    v_eps: f64,
) -> Result<Option<BandWrapPair>, KernelRefusal> {
    let count = loop_record.coedges.len();
    if count == 0 {
        return Ok(None);
    }
    let mut found: Option<BandWrapPair> = None;
    for index in 0..count {
        let arrive = &loop_record.coedges[index];
        let depart = &loop_record.coedges[(index + 1) % count];
        let [_, a1] = arrive.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let [d0, _] = depart.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let arrive_end = arrive.pcurve.evaluate(a1).or_refuse(KernelStage::Sew, "evaluate")?;
        let depart_start = depart.pcurve.evaluate(d0).or_refuse(KernelStage::Sew, "evaluate")?;
        let arrive_at_max = (arrive_end.x - u_domain[1]).abs() <= u_eps;
        let arrive_at_min = (arrive_end.x - u_domain[0]).abs() <= u_eps;
        let depart_at_max = (depart_start.x - u_domain[1]).abs() <= u_eps;
        let depart_at_min = (depart_start.x - u_domain[0]).abs() <= u_eps;
        if !((arrive_at_max && depart_at_min) || (arrive_at_min && depart_at_max)) {
            continue;
        }
        // The wrap must be clean: same rim level on both sides, one shared
        // model vertex, and only ONE wrap in the loop.
        if (arrive_end.y - depart_start.y).abs() > v_eps || found.is_some() {
            return Ok(None);
        }
        let arrive_edge = edges
            .get(&arrive.edge_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "boolean.rim", "band seam: loop references unknown edge"))?;
        let depart_edge = edges
            .get(&depart.edge_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "boolean.rim", "band seam: loop references unknown edge"))?;
        let arrive_vertex = if arrive.forward {
            arrive_edge.end_vertex_id
        } else {
            arrive_edge.start_vertex_id
        };
        let depart_vertex = if depart.forward {
            depart_edge.start_vertex_id
        } else {
            depart_edge.end_vertex_id
        };
        if arrive_vertex != depart_vertex {
            return Ok(None);
        }
        found = Some(BandWrapPair {
            depart_index: (index + 1) % count,
            arrive_at_max,
            v: 0.5 * (arrive_end.y + depart_start.y),
            vertex_id: arrive_vertex,
        });
    }
    Ok(found)
}

/// Operand normalization at boolean entry: give a FULL-PERIOD closed-u "band"
/// face (exactly two full-wrap rim loops, no seam edge) the explicit seam
/// edge that natively built periodic faces carry (`make_cylinder_brep`).
///
/// The imprint / flat-arrangement pipeline is designed around seam-CARRYING
/// periodic faces: the seam edge is what splits a seam-crossing intersection
/// curve (it is intersected like any other face edge) and what closes
/// flat-domain cycles along the u=u0/u1 columns. A seamless band (STEP
/// import: e.g. a torus bore ringing a slot) breaks both — a cut that wraps
/// the seam has its analytic pcurve clamped at the domain edge
/// (`build_pcurve_on_surface` keeps analytic carriers in-domain by design),
/// the arrangement carves corrupted micro-regions against the fold, and the
/// face's true outside-the-cutter fragment never forms, surfacing as
/// `edge used 1 times (expected 2)` at assembly validation.
///
/// Every gate below fails SOFT: a face that doesn't match the exact band
/// pattern is left untouched (status quo). Inserting the seam is
/// Euler-neutral (E+1, holes-1), so the solid's derived genus is unchanged.
pub(super) fn insert_periodic_band_seam_edges(
    solid: &mut BrepSolid,
    known_band_faces: &HashSet<u64>,
) -> Result<usize, KernelRefusal> {
    let edges_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertex_points: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut next_edge_id = solid.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;
    let mut next_coedge_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;

    // Immutable planning pass (edges_by_id borrows `solid`), then application.
    struct SeamPlan {
        shell_index: usize,
        face_index: usize,
        edge: EdgeRecord,
        merged_loop: LoopRecord,
    }
    let debug_face: Option<u64> = std::env::var("BREP_DEBUG_BAND_FACE")
        .ok()
        .and_then(|v| v.parse().ok());
    macro_rules! band_dbg {
        ($face:expr, $($arg:tt)*) => {
            if debug_face == Some($face.id) {
                eprintln!("band_seams[{}]: {}", $face.id, format!($($arg)*));
            }
        };
    }
    let mut plans: Vec<SeamPlan> = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        'faces: for (face_index, original) in shell.faces.iter().enumerate() {
            if original.loops.len() != 2 {
                continue;
            }
            let (closed_along_u, closed_along_v) = match original.surface.closed_directions() {
                Ok(value) => value,
                Err(_) => continue,
            };
            // A band whose rims run along v (a pipe elbow imported as a torus
            // patch: rims at two u levels, closed in v, no seam edge) is the
            // same band in the transposed chart. It is planned there and the
            // plan is read back in the face's own chart: the seam is the
            // carrier's v = v0 iso-curve, the seam coedges' pcurves are rows,
            // and the rim coedges are the face's own. Without it a section
            // crossing the carrier's v seam is split there with no edge for
            // the face's arrangement to close against
            // (`boolean_fuzz_corpus/06_fragmentation-incomplete_t12`).
            // Escape hatch `BREP_BAND_SEAM_V=0`.
            let mut transposed_chart: Option<FaceRecord> = None;
            'charts: for transposed in [false, true] {
                let (face, closed_u, closed_v) = if transposed {
                    if !closed_along_v || std::env::var("BREP_BAND_SEAM_V").as_deref() == Ok("0") {
                        continue 'faces;
                    }
                    let Ok(chart) = transposed_band_chart(original) else {
                        continue 'faces;
                    };
                    (&*transposed_chart.insert(chart), closed_along_v, closed_along_u)
                } else {
                    (original, closed_along_u, closed_along_v)
                };
                if !closed_u {
                    band_dbg!(face, "skip: not closed_u");
                    continue 'charts;
                }
                let Ok([u0, u1]) = face.surface.domain_u() else {
                    continue 'charts;
                };
                let Ok([v0, v1]) = face.surface.domain_v() else {
                    continue 'charts;
                };
                let u_eps = (u1 - u0).abs() * 1e-6;
                let v_eps = (v1 - v0).abs() * 1e-6;
                // No degenerate edges and no existing seam-like column coedge
                // (a pcurve running along u=u0 or u=u1 with real v-extent).
                for coedge in face
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                {
                    let Some(edge) = edges_by_id.get(&coedge.edge_id) else {
                        continue 'charts;
                    };
                    if edge.degenerate {
                        band_dbg!(face, "skip: degenerate edge {}", edge.id);
                        continue 'charts;
                    }
                    let Ok([d0, d1]) = coedge.pcurve.domain() else {
                        continue 'charts;
                    };
                    let (Ok(start), Ok(middle), Ok(end)) = (
                        coedge.pcurve.evaluate(d0),
                        coedge.pcurve.evaluate((d0 + d1) * 0.5),
                        coedge.pcurve.evaluate(d1),
                    ) else {
                        continue 'charts;
                    };
                    for extreme in [u0, u1] {
                        if (start.x - extreme).abs() <= u_eps
                            && (middle.x - extreme).abs() <= u_eps
                            && (end.x - extreme).abs() <= u_eps
                            && (end.y - start.y).abs() > v_eps
                        {
                            band_dbg!(face, "skip: existing seam-like column coedge (edge {})", coedge.edge_id);
                            continue 'charts;
                        }
                    }
                }
                let Ok(Some(pair_a)) =
                    band_wrap_pair(&face.loops[0], &edges_by_id, [u0, u1], u_eps, v_eps)
                else {
                    band_dbg!(face, "skip: band_wrap_pair loop0 -> None");
                    continue 'charts;
                };
                let Ok(Some(pair_b)) =
                    band_wrap_pair(&face.loops[1], &edges_by_id, [u0, u1], u_eps, v_eps)
                else {
                    band_dbg!(face, "skip: band_wrap_pair loop1 -> None");
                    continue 'charts;
                };
                // The two rims must arrive at OPPOSITE domain extremes (one rim
                // traversed +u, the other -u) and sit at distinct levels/vertices.
                if pair_a.arrive_at_max == pair_b.arrive_at_max
                    || pair_a.vertex_id == pair_b.vertex_id
                    || (pair_a.v - pair_b.v).abs() <= v_eps * 10.0
                {
                    band_dbg!(face, "skip: rim pair mismatch (same_extreme={} same_vertex={} dv={:.3e})",
                        pair_a.arrive_at_max == pair_b.arrive_at_max,
                        pair_a.vertex_id == pair_b.vertex_id,
                        (pair_a.v - pair_b.v).abs());
                    continue 'charts;
                }
                // The seam column between the rims must run through face INTERIOR
                // on both sides of the seam (a true full-period band).
                let v_mid = 0.5 * (pair_a.v + pair_b.v);
                let inset = (u1 - u0) * 1e-3;
                // Containment is asked of the face in its own chart, whose
                // lanes are the ones the arrangement reads.
                let interior_at = |u: f64, v: f64| -> bool {
                    let point = if transposed { Vec2 { x: v, y: u } } else { Vec2 { x: u, y: v } };
                    matches!(
                        parameter_point_in_face(original, point, 1e-9),
                        Ok(PolygonClass::Inside)
                    )
                };
                // COVERING-PLANE STRIP: `normalize_covering_rim_strips` selected
                // this face as a two-rim strip on an OPEN-v surface (disjoint rim
                // v-hulls), where the material can only be the BETWEEN strip — a
                // complement is unrepresentable on open v — and folded the rims
                // in-domain. But each rim now winds a full period on its own, so
                // `interior_at`'s `wrapped_horizon` classifier bails (it only
                // closes a NET-zero loop, which is exactly the MERGED loop this
                // pass is about to build). Trust the plain path for those known
                // bands.
                let plain = known_band_faces.contains(&face.id)
                    || (interior_at(u0 + inset, v_mid) && interior_at(u1 - inset, v_mid));
                // WRAPPED-BAND VARIANT: when the surface is ALSO closed in v and
                // one rim sits exactly ON the v-seam, the material can be the
                // wrapped COMPLEMENT of the inter-rim window (trial 35's torus
                // face: rims at v=0.25/1.0, material v∈[0,0.25]) — the plain
                // column midpoint then reads Outside and the band was skipped,
                // leaving the flat arrangement unable to close the seam-wrapping
                // fragment (one-use cascade). The material column arc is then
                // fully in-domain on the OTHER side of the seam rim ([v0, va] for
                // a rim at v1, [vb, v1] for a rim at v0), so the same seam-edge
                // construction applies with the on-seam rim's foot wrapped to the
                // opposite domain edge. Escape hatch: BREP_BAND_SEAM_WRAP=0.
                let (mut foot_a, mut foot_b) = (pair_a.v, pair_b.v);
                let mut wrapped = false;
                if !plain
                    && closed_v
                    && std::env::var("BREP_BAND_SEAM_WRAP").as_deref() != Ok("0")
                {
                    let (va, vb) = (pair_a.v.min(pair_b.v), pair_a.v.max(pair_b.v));
                    let rim_at_top = (vb - v1).abs() <= v_eps * 10.0;
                    let rim_at_bottom = (va - v0).abs() <= v_eps * 10.0;
                    if rim_at_top ^ rim_at_bottom {
                        let wrapped_mid = if rim_at_top {
                            0.5 * (v0 + va)
                        } else {
                            0.5 * (vb + v1)
                        };
                        if interior_at(u0 + inset, wrapped_mid) && interior_at(u1 - inset, wrapped_mid)
                        {
                            let wrap_foot = |v: f64| -> f64 {
                                if rim_at_top && (v - vb).abs() <= v_eps * 10.0 {
                                    v0
                                } else if rim_at_bottom && (v - va).abs() <= v_eps * 10.0 {
                                    v1
                                } else {
                                    v
                                }
                            };
                            foot_a = wrap_foot(pair_a.v);
                            foot_b = wrap_foot(pair_b.v);
                            wrapped = true;
                        }
                    }
                }
                if !plain && !wrapped {
                    band_dbg!(face, "skip: interior probe failed (plain=false wrapped=false, closed_v={closed_v}, v_a={:.6} v_b={:.6})", pair_a.v, pair_b.v);
                    continue 'charts;
                }
                // 3D consistency: both seam feet are model vertices sitting on
                // the seam meridian.
                let (Some(&point_a), Some(&point_b)) = (
                    vertex_points.get(&pair_a.vertex_id),
                    vertex_points.get(&pair_b.vertex_id),
                ) else {
                    continue 'charts;
                };
                let on_meridian = |v: f64, point: Vec3| -> bool {
                    face.surface
                        .evaluate(u0, v)
                        .map(|at| at.sub(point).length() <= 1e-6 * (1.0 + point.length()))
                        .unwrap_or(false)
                };
                if !on_meridian(pair_a.v, point_a) || !on_meridian(pair_b.v, point_b) {
                    band_dbg!(face, "skip: wrap vertex not on seam meridian");
                    continue 'charts;
                }
                // The chart's u = u0 iso-curve; in a transposed chart it is the
                // face's own v = v0 row, read off the face's surface.
                let iso = if transposed {
                    original.surface.iso_curve_v(u0)
                } else {
                    face.surface.iso_curve_u(u0)
                };
                let Ok(iso) = iso else {
                    continue 'charts;
                };
                let seam_matches = |v: f64, point: Vec3| -> bool {
                    iso.evaluate(v)
                        .map(|at| at.sub(point).length() <= 1e-6 * (1.0 + point.length()))
                        .unwrap_or(false)
                };
                if !seam_matches(pair_a.v, point_a) || !seam_matches(pair_b.v, point_b) {
                    band_dbg!(face, "skip: iso seam curve does not match wrap vertices");
                    continue 'charts;
                }
                // Seam edge oriented bottom(v)->top(v). The feet are the pair
                // levels in the material column's chart (identical to the pair
                // levels in the plain case; the on-seam rim wrapped to the
                // opposite domain edge in the wrapped case).
                let (v_bottom, bottom_vertex, v_top, top_vertex) = if foot_a < foot_b {
                    (foot_a, pair_a.vertex_id, foot_b, pair_b.vertex_id)
                } else {
                    (foot_b, pair_b.vertex_id, foot_a, pair_a.vertex_id)
                };
                let edge = EdgeRecord {
                    id: next_edge_id,
                    curve: iso,
                    t0: v_bottom,
                    t1: v_top,
                    start_vertex_id: bottom_vertex,
                    end_vertex_id: top_vertex,
                    degenerate: false,
                    name: None,
                };
                next_edge_id += 1;
                let rotate = |loop_record: &LoopRecord, start: usize| -> Vec<CoedgeRecord> {
                    let mut coedges = loop_record.coedges.clone();
                    coedges.rotate_left(start);
                    coedges
                };
                // Loop A rotated to END at its seam arrival, then the seam column
                // at A's arrival extreme (traversed vA->vB), then loop B rotated
                // to depart from that same extreme, then the seam column at the
                // opposite extreme (traversed vB->vA), closing at A's departure.
                let column_a = if pair_a.arrive_at_max { u1 } else { u0 };
                let column_b = if pair_a.arrive_at_max { u0 } else { u1 };
                let chart_point = |u: f64, v: f64| -> Vec3 {
                    if transposed {
                        Vec3::new(v, u, 0.0)
                    } else {
                        Vec3::new(u, v, 0.0)
                    }
                };
                let (Ok(pcurve_up), Ok(pcurve_down)) = (
                    crate::make_line(chart_point(column_a, foot_a), chart_point(column_a, foot_b)),
                    crate::make_line(chart_point(column_b, foot_b), chart_point(column_b, foot_a)),
                ) else {
                    continue 'charts;
                };
                // The rim coedges in the face's own chart (the transposed
                // chart keeps their order).
                let mut coedges = rotate(&original.loops[0], pair_a.depart_index);
                coedges.push(CoedgeRecord {
                    id: next_coedge_id,
                    edge_id: edge.id,
                    forward: foot_a < foot_b,
                    pcurve: pcurve_up,
                });
                next_coedge_id += 1;
                coedges.extend(rotate(&original.loops[1], pair_b.depart_index));
                coedges.push(CoedgeRecord {
                    id: next_coedge_id,
                    edge_id: edge.id,
                    forward: foot_b < foot_a,
                    pcurve: pcurve_down,
                });
                next_coedge_id += 1;
                if std::env::var("BREP_DEBUG_BOOL").is_ok() {
                    eprintln!(
                        "band_seams: face {} gets seam (wrapped={wrapped} transposed={transposed} feet=[{foot_a:.6},{foot_b:.6}])",
                        face.id
                    );
                }
                plans.push(SeamPlan {
                    shell_index,
                    face_index,
                    edge,
                    merged_loop: LoopRecord {
                        id: face.loops[0].id,
                        coedges,
                    },
                });
                continue 'faces;
            }
        }
    }
    drop(edges_by_id);
    let inserted = plans.len();
    for plan in plans {
        solid.edges.push(plan.edge);
        let face = &mut solid.shells[plan.shell_index].faces[plan.face_index];
        face.loops = vec![plan.merged_loop];
    }
    Ok(inserted)
}

/// A copy of `face` in the transposed chart: the control net and every
/// coedge pcurve with u and v exchanged. Geometry and loop order are the
/// face's own; only the band-seam planner reads it.
fn transposed_band_chart(face: &FaceRecord) -> Result<FaceRecord, KernelRefusal> {
    let surface = &face.surface;
    let rows_v = surface.control_points.first().map(|row| row.len()).unwrap_or(0);
    let mut control_points = vec![Vec::with_capacity(surface.control_points.len()); rows_v];
    for row in &surface.control_points {
        for (column, point) in row.iter().enumerate() {
            control_points[column].push(*point);
        }
    }
    let mut chart = face.clone();
    chart.surface = crate::NurbsSurface::new(
        surface.degree_v,
        surface.degree_u,
        surface.knots_v.clone(),
        surface.knots_u.clone(),
        control_points,
    )
    .or_refuse(KernelStage::Sew, "csg.boolean.rim")?;
    for coedge in chart.loops.iter_mut().flat_map(|loop_record| &mut loop_record.coedges) {
        for control in &mut coedge.pcurve.control_points {
            std::mem::swap(&mut control.x, &mut control.y);
        }
    }
    Ok(chart)
}

/// Traversal-aligned subcurve of a p-curve over a fraction of its domain
/// (mirrors `edge_split::subcurve_by_fraction`; kept local so the covering-rim
/// normalization is self-contained for clean A/B attribution).
fn covering_rim_subcurve(
    curve: &NurbsCurve,
    start_fraction: f64,
    end_fraction: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let first = start + (end - start) * start_fraction;
    let second = start + (end - start) * end_fraction;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if first > start + epsilon && first < end - epsilon {
        result = result.split(first).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Sew, "domain")?;
    if second < domain[1] - epsilon && second > domain[0] + epsilon {
        result = result.split(second).or_refuse(KernelStage::Sew, "split")?.0;
    }
    Ok(result)
}

/// Split a set of operand edges at requested 3D-curve parameters, minting a
/// fresh vertex at each interior split and rebuilding EVERY incident coedge
/// (on every face) into traversal-aligned sub-p-curves — the operand-entry
/// equivalent of `apply_edge_splits`, but driven by a direct `edge_id ->
/// parameters` map rather than an imprint record, and minting vertices fresh
/// (no proximity reuse: the split point is a NEW junction on the seam
/// meridian). Coedge p-curves are fraction-consistent reparameterizations of
/// their edge, the same invariant `apply_edge_splits` relies on.
fn split_operand_edges(
    solid: &mut BrepSolid,
    splits: &HashMap<u64, Vec<f64>>,
) -> Result<(), KernelRefusal> {
    let mut next_vertex_id = solid.vertices.iter().map(|v| v.id).max().unwrap_or(0) + 1;
    let mut next_edge_id = solid.edges.iter().map(|e| e.id).max().unwrap_or(0) + 1;
    let mut next_coedge_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut pieces: HashMap<u64, Vec<EdgeRecord>> = HashMap::default();
    let mut new_vertices: Vec<VertexRecord> = Vec::new();
    let mut source_spans: HashMap<u64, (f64, f64)> = HashMap::default();
    for edge in &solid.edges {
        let Some(parameters) = splits.get(&edge.id) else {
            continue;
        };
        let ptol = (edge.t1 - edge.t0).abs() * 1e-10;
        let mut partition: Vec<f64> = parameters
            .iter()
            .copied()
            .filter(|parameter| *parameter > edge.t0 + ptol && *parameter < edge.t1 - ptol)
            .collect();
        partition.sort_by(f64::total_cmp);
        partition.dedup_by(|a, b| (*a - *b).abs() <= ptol);
        partition.insert(0, edge.t0);
        partition.push(edge.t1);
        if partition.len() <= 2 {
            continue;
        }
        let mut vertex_ids = vec![edge.start_vertex_id];
        for parameter in &partition[1..partition.len() - 1] {
            let point = edge.curve.evaluate(*parameter).or_refuse(KernelStage::Sew, "evaluate")?;
            let id = next_vertex_id;
            next_vertex_id += 1;
            new_vertices.push(VertexRecord { id, point });
            vertex_ids.push(id);
        }
        vertex_ids.push(edge.end_vertex_id);
        let mut edge_pieces = Vec::new();
        for index in 0..partition.len() - 1 {
            let name = edge.name.as_ref().map(|name| {
                if index == 0 {
                    name.clone()
                } else {
                    format!("{name}_{index}")
                }
            });
            edge_pieces.push(EdgeRecord {
                id: next_edge_id,
                curve: edge.curve.clone(),
                t0: partition[index],
                t1: partition[index + 1],
                start_vertex_id: vertex_ids[index],
                end_vertex_id: vertex_ids[index + 1],
                degenerate: false,
                name,
            });
            next_edge_id += 1;
        }
        source_spans.insert(edge.id, (edge.t0, edge.t1));
        pieces.insert(edge.id, edge_pieces);
    }
    if pieces.is_empty() {
        return Ok(());
    }
    solid.vertices.extend(new_vertices);
    solid.edges.retain(|edge| !pieces.contains_key(&edge.id));
    for edge_pieces in pieces.values() {
        solid.edges.extend(edge_pieces.iter().cloned());
    }
    for face in solid
        .shells
        .iter_mut()
        .flat_map(|shell| shell.faces.iter_mut())
    {
        for loop_record in &mut face.loops {
            let mut rebuilt = Vec::new();
            for coedge in &loop_record.coedges {
                let Some(edge_pieces) = pieces.get(&coedge.edge_id) else {
                    rebuilt.push(coedge.clone());
                    continue;
                };
                let (ot0, ot1) = source_spans[&coedge.edge_id];
                let span = ot1 - ot0;
                let indices: Vec<usize> = if coedge.forward {
                    (0..edge_pieces.len()).collect()
                } else {
                    (0..edge_pieces.len()).rev().collect()
                };
                for index in indices {
                    let piece = &edge_pieces[index];
                    let (start_fraction, end_fraction) = if coedge.forward {
                        ((piece.t0 - ot0) / span, (piece.t1 - ot0) / span)
                    } else {
                        ((ot1 - piece.t1) / span, (ot1 - piece.t0) / span)
                    };
                    rebuilt.push(CoedgeRecord {
                        id: next_coedge_id,
                        edge_id: piece.id,
                        forward: coedge.forward,
                        pcurve: covering_rim_subcurve(&coedge.pcurve, start_fraction, end_fraction)?,
                    });
                    next_coedge_id += 1;
                }
            }
            loop_record.coedges = rebuilt;
        }
    }
    Ok(())
}

/// One planned rim seam crossing: split edge `edge_id` at 3D parameter
/// `t_split` (the point where the covering-plane rim crosses the seam
/// meridian).
struct RimCrossing {
    edge_id: u64,
    t_split: f64,
}

/// Analyze a single rim loop of a candidate covering-plane strip: unwrap it
/// into the covering plane, confirm it is a clean single full wrap, and — if
/// the wrap crosses the seam MID-COEDGE (no model vertex on the meridian) —
/// return the crossing to imprint. Returns `Ok(None)` (soft-fail) for anything
/// that isn't this exact structure, or whose wrap already sits at a coedge
/// junction (the native seam-vertex case `band_wrap_pair` already handles).
fn plan_rim_seam_crossing(
    face: &FaceRecord,
    loop_record: &LoopRecord,
    edges_by_id: &HashMap<u64, &EdgeRecord>,
    u0: f64,
    u1: f64,
    u_eps: f64,
) -> Result<Option<(RimCrossing, f64, f64)>, KernelRefusal> {
    // Returns (crossing, rim vmin, rim vmax).
    let period = u1 - u0;
    // Per-coedge sampled p-curve, with net folded travel for the wrap check.
    let sample_count = 24usize;
    struct Sampled {
        edge_id: u64,
        forward: bool,
        raw: Vec<Vec2>,
    }
    let mut coedges: Vec<Sampled> = Vec::with_capacity(loop_record.coedges.len());
    let (mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut net_folded = 0.0f64;
    let mut prev_x: Option<f64> = None;
    for coedge in &loop_record.coedges {
        let [d0, d1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let mut raw = Vec::with_capacity(sample_count + 1);
        for index in 0..=sample_count {
            let parameter = d0 + (d1 - d0) * index as f64 / sample_count as f64;
            let evaluated = coedge.pcurve.evaluate(parameter).or_refuse(KernelStage::Sew, "evaluate")?;
            raw.push(Vec2 {
                x: evaluated.x,
                y: evaluated.y,
            });
            vmin = vmin.min(evaluated.y);
            vmax = vmax.max(evaluated.y);
            if let Some(px) = prev_x {
                let du = evaluated.x - px;
                net_folded += du - period * (du / period).round();
            }
            prev_x = Some(evaluated.x);
        }
        coedges.push(Sampled {
            edge_id: coedge.edge_id,
            forward: coedge.forward,
            raw,
        });
    }
    // Loop-closure fold contribution (last sample -> first sample).
    if let (Some(first), Some(last)) = (
        coedges.first().and_then(|c| c.raw.first()),
        coedges.last().and_then(|c| c.raw.last()),
    ) {
        let du = first.x - last.x;
        net_folded += du - period * (du / period).round();
    }
    // A clean single full wrap: net folded u-travel is exactly ±one period.
    if (net_folded.abs() - period).abs() > 2e-2 * period {
        return Ok(None);
    }
    // Exactly ONE coedge must exit the domain (contain the seam crossing);
    // zero => the wrap is at a coedge junction (native case, leave it); more
    // than one => wavy / multi-crossing, decline.
    let mut exit: Option<(usize, f64)> = None; // (coedge index, seam u)
    for (index, sampled) in coedges.iter().enumerate() {
        let below = sampled.raw.iter().any(|p| p.x < u0 - u_eps);
        let above = sampled.raw.iter().any(|p| p.x > u1 + u_eps);
        if below && above {
            return Ok(None); // spans more than a period, not a clean crossing
        }
        if below || above {
            let seam = if above { u1 } else { u0 };
            if exit.is_some() {
                return Ok(None);
            }
            exit = Some((index, seam));
        }
    }
    let Some((exit_index, seam)) = exit else {
        return Ok(None);
    };
    let coedge = &loop_record.coedges[exit_index];
    let [d0, d1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // The exiting coedge must cross `seam` exactly once (single sign change).
    let mut sign_changes = 0usize;
    let sampled = &coedges[exit_index];
    for pair in sampled.raw.windows(2) {
        if (pair[0].x - seam) * (pair[1].x - seam) < 0.0 {
            sign_changes += 1;
        }
    }
    if sign_changes != 1 {
        return Ok(None);
    }
    // Bisect the p-curve parameter where x == seam.
    let x_at = |s: f64| -> Result<f64, KernelRefusal> { Ok(coedge.pcurve.evaluate(s).or_refuse(KernelStage::Sew, "evaluate")?.x - seam) };
    let (mut lo, mut hi) = (d0, d1);
    let mut flo = x_at(lo)?;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let fmid = x_at(mid)?;
        if fmid == 0.0 {
            lo = mid;
            hi = mid;
            break;
        }
        if (flo < 0.0) != (fmid < 0.0) {
            hi = mid;
        } else {
            lo = mid;
            flo = fmid;
        }
    }
    let s_star = 0.5 * (lo + hi);
    let v_cross = coedge.pcurve.evaluate(s_star).or_refuse(KernelStage::Sew, "evaluate")?.y;
    let fraction = (s_star - d0) / (d1 - d0);
    let edge = edges_by_id
        .get(&sampled.edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "boolean.rim", "covering rim: unknown edge"))?;
    if edge.degenerate {
        return Ok(None);
    }
    let t_split = if sampled.forward {
        edge.t0 + fraction * (edge.t1 - edge.t0)
    } else {
        edge.t1 - fraction * (edge.t1 - edge.t0)
    };
    // Reject a crossing that lands on (or within weld distance of) an edge
    // end — that is the native seam-vertex case, not a mid-edge wrap.
    let span = (edge.t1 - edge.t0).abs();
    let end_tol = span * 1e-4;
    if t_split <= edge.t0 + end_tol || t_split >= edge.t1 - end_tol {
        return Ok(None);
    }
    // Validate the fraction-consistency + on-surface assumption: the edge's 3D
    // point at `t_split` must equal the surface point at (seam, v_cross).
    let point_edge = edge.curve.evaluate(t_split).or_refuse(KernelStage::Sew, "evaluate")?;
    let point_surface = face.surface.evaluate(seam, v_cross).or_refuse(KernelStage::Sew, "evaluate")?;
    if point_edge.sub(point_surface).length() > 1e-6 * (1.0 + point_edge.length()) {
        return Ok(None);
    }
    Ok(Some((
        RimCrossing {
            edge_id: sampled.edge_id,
            t_split,
        },
        vmin,
        vmax,
    )))
}

/// COVERING-PLANE RIM NORMALIZATION (case 08:10, face 1513). A singly-periodic
/// strip (closed_u, open_v, exactly two full-wrap rim loops) whose rims are
/// stored as COVERING-PLANE p-curves that run OUT OF the u-domain — each rim
/// crosses the seam meridian MID-COEDGE, with no model vertex on the seam.
/// `insert_periodic_band_seam_edges` can only fire on a band whose rim wrap
/// sits at a shared model vertex ON the seam meridian (`band_wrap_pair`), so
/// the covering-plane strip is skipped and its cut chains never close across
/// the seam — the residual one-use cascade at the strip's rim junctions.
///
/// This pass gives the strip that vertex: for EACH rim it locates the single
/// seam-meridian crossing on the rim edge, SPLITS the edge there (minting a
/// fresh vertex on the meridian and rebuilding every incident coedge — on this
/// face AND its neighbour — by traversal fraction), then FOLDS the now
/// out-of-domain p-curve piece on THIS face back into the domain so the wrap
/// sits at the domain extreme. `insert_periodic_band_seam_edges` then
/// recognizes the wrap and installs the seam column.
///
/// Every gate fails SOFT (the face is left untouched). The caller runs this and
/// the seam insertion under ONE validate-gated backup and additionally reverts
/// unless every face this pass touched ends with a single merged loop — a face
/// folded in-domain but NOT re-seamed would lose its covering-plane containment
/// classifier (`covering_rim_strip_point_in_face` keys on the out-of-domain
/// representation), so the pass is atomic with the insertion it enables.
///
/// Returns the ids of the faces normalized (for the caller's atomicity check).
/// Escape hatch: BREP_COVERING_RIM_NORMALIZE=0.
fn normalize_covering_rim_strips(solid: &mut BrepSolid) -> Result<Vec<u64>, KernelRefusal> {
    if std::env::var("BREP_COVERING_RIM_NORMALIZE").as_deref() == Ok("0") {
        return Ok(Vec::new());
    }
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let edges_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut splits: HashMap<u64, Vec<f64>> = HashMap::default();
    let mut normalized_faces: Vec<u64> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.loops.len() != 2 {
                continue;
            }
            if !matches!(face.surface.closed_directions(), Ok((true, false))) {
                continue;
            }
            let (Ok([u0, u1]), Ok([v0, v1])) =
                (face.surface.domain_u(), face.surface.domain_v())
            else {
                continue;
            };
            let period = u1 - u0;
            if !(period > 0.0) {
                continue;
            }
            let u_eps = period * 1e-6;
            let v_span = (v1 - v0).abs().max(1e-30);
            let (Ok(Some((cross0, lo0, hi0))), Ok(Some((cross1, lo1, hi1)))) = (
                plan_rim_seam_crossing(face, &face.loops[0], &edges_by_id, u0, u1, u_eps),
                plan_rim_seam_crossing(face, &face.loops[1], &edges_by_id, u0, u1, u_eps),
            ) else {
                continue;
            };
            // Distinct rim edges, and disjoint v-hulls (a genuine strip).
            if cross0.edge_id == cross1.edge_id {
                continue;
            }
            let disjoint = hi0 <= lo1 + 1e-6 * v_span || hi1 <= lo0 + 1e-6 * v_span;
            if !disjoint {
                continue;
            }
            for cross in [&cross0, &cross1] {
                splits.entry(cross.edge_id).or_default().push(cross.t_split);
            }
            normalized_faces.push(face.id);
            if debug {
                eprintln!(
                    "covering_rim: face {} plans rim splits on edges {} @ {:.6}, {} @ {:.6}",
                    face.id, cross0.edge_id, cross0.t_split, cross1.edge_id, cross1.t_split
                );
            }
        }
    }
    drop(edges_by_id);
    if normalized_faces.is_empty() {
        return Ok(Vec::new());
    }
    split_operand_edges(solid, &splits)?;
    // Fold the out-of-domain p-curve pieces on the normalized faces back into
    // the domain, so the wrap now lands on a domain extreme (u0/u1).
    let normalized_set: HashSet<u64> = normalized_faces.iter().copied().collect();
    for face in solid
        .shells
        .iter_mut()
        .flat_map(|shell| shell.faces.iter_mut())
    {
        if !normalized_set.contains(&face.id) {
            continue;
        }
        let Ok([u0, u1]) = face.surface.domain_u() else {
            continue;
        };
        let period = u1 - u0;
        let fold_eps = period * 1e-6;
        for loop_record in &mut face.loops {
            for coedge in &mut loop_record.coedges {
                let [d0, d1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let mut lo = f64::INFINITY;
                let mut hi = f64::NEG_INFINITY;
                for index in 0..=16 {
                    let parameter = d0 + (d1 - d0) * index as f64 / 16.0;
                    let x = coedge.pcurve.evaluate(parameter).or_refuse(KernelStage::Sew, "evaluate")?.x;
                    lo = lo.min(x);
                    hi = hi.max(x);
                }
                let shift = if lo >= u1 - fold_eps {
                    -period
                } else if hi <= u0 + fold_eps {
                    period
                } else {
                    0.0
                };
                if shift != 0.0 {
                    let mut control_points = coedge.pcurve.control_points.clone();
                    for cp in &mut control_points {
                        cp.x += shift * cp.w;
                    }
                    coedge.pcurve = NurbsCurve::new(
                        coedge.pcurve.degree,
                        coedge.pcurve.knots.clone(),
                        control_points,
                    ).or_refuse(KernelStage::Sew, "csg.boolean.rim")?;
                }
            }
        }
    }
    Ok(normalized_faces)
}

/// Insert band seams (see `insert_periodic_band_seam_edges`), reverting the
/// whole solid if the normalized topology validates WORSE than the input
/// (soft-fail: the boolean then proceeds exactly as before).
pub(super) fn normalize_operand_band_seams(solid: &mut BrepSolid) -> Result<(), KernelRefusal> {
    // Cheap pre-scan: candidates are 2-loop faces on carriers closed in u or v.
    let has_candidate = solid.shells.iter().flat_map(|shell| &shell.faces).any(|face| {
        face.loops.len() == 2
            && matches!(face.surface.closed_directions(), Ok((true, _) | (_, true)))
    });
    if !has_candidate {
        return Ok(());
    }
    let backup = solid.clone();
    let baseline_issues = backup.validate().len();
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();

    // ATTEMPT 1 — COVERING-PLANE RIM NORMALIZATION + seam insertion. The
    // normalization splits mid-edge rim seam crossings so
    // `insert_periodic_band_seam_edges` can fire on covering-plane strips (case
    // 08:10 face 1513). It must be ATOMIC with the insertion: once a strip is
    // folded in-domain its `covering_rim_strip_point_in_face` classifier stops
    // firing, so a fold that is NOT followed by a merged seam would REGRESS the
    // strip's containment. If the covering-rim work does not fully land (a
    // folded strip left un-seamed, or validation regressed), we discard the
    // whole attempt and fall back to the ORIGINAL plain-band behavior below —
    // so a covering-rim miss can never cost the plain band seams.
    let normalized_faces = normalize_covering_rim_strips(solid).unwrap_or_else(|error| {
        if debug {
            eprintln!("covering_rim: ERROR {error}");
        }
        *solid = backup.clone();
        Vec::new()
    });
    let known_band_faces: HashSet<u64> = normalized_faces.iter().copied().collect();
    let attempt = insert_periodic_band_seam_edges(solid, &known_band_faces);
    let all_seamed = |solid: &BrepSolid| -> bool {
        normalized_faces.iter().all(|&id| {
            solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .find(|face| face.id == id)
                .map(|face| face.loops.len() == 1)
                .unwrap_or(false)
        })
    };
    if let Ok(count) = attempt {
        let issues = solid.validate().len();
        if issues <= baseline_issues && all_seamed(solid) {
            if debug && (count > 0 || !normalized_faces.is_empty()) {
                eprintln!(
                    "band_seams: inserted {count} seam edge(s); covering-rim normalized {} face(s)",
                    normalized_faces.len()
                );
            }
            return Ok(());
        }
    }

    // ATTEMPT 2 — fall back to the ORIGINAL behavior: plain seam insertion with
    // NO covering-rim normalization, validate-gated exactly as before. Reached
    // only when attempt 1 did not fully land (covering-rim strips are rare, so
    // this is the identical status-quo path for every ordinary operand).
    *solid = backup.clone();
    match insert_periodic_band_seam_edges(solid, &HashSet::default()) {
        Ok(0) => {}
        Ok(count) => {
            let issues = solid.validate().len();
            if issues > baseline_issues {
                if debug {
                    eprintln!(
                        "band_seams: REVERTED {count} insertion(s) (validation {baseline_issues} -> {issues})"
                    );
                }
                *solid = backup;
            } else if debug {
                eprintln!("band_seams: inserted {count} seam edge(s)");
            }
        }
        Err(error) => {
            if debug {
                eprintln!("band_seams: ERROR {error}");
            }
            *solid = backup;
        }
    }
    Ok(())
}


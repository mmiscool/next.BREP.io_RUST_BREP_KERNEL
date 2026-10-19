//! The OPEN mixed corner (lane L, 2026-10-14): a SELECTED convex rim A
//! (wall A ∩ cap), the SELECTED concave vertical Q (wall A ∩ wall B) and the
//! UNSELECTED convex rim B (wall B ∩ cap) at one vertex, orthogonal planar
//! walls, one radius — the keyway floor corners of the 2026-10-13 sprocket
//! report `20261013T020933Z-e80a36db` (`37|43` + `37|38` over `38|43`, and
//! the mirror `39|43` + `38|39`).
//!
//! What the cutter composition leaves there: Q's pad was cut first and ends
//! flush on the cap, so the cap's loop carries the pad's bottom arc (radius
//! r about Q's axis), into which rim A now continues G1 — and then into the
//! unselected rim B.  Rim A's cutter, extended over the edge's original
//! extent, ran through the pad: it sliced the pad's fill flat (a unit `:CAP`
//! square in wall A's plane), stood its flush bulkhead at the original
//! corner, and its wall's rail on wall A continues over the square.  The
//! §6.9.7 reading of that corner is the rolling ball leaving wall A for the
//! pad's rim: its centre keeps r above the cap and 2r from Q's axis, a
//! circle, so the closure is the EXACT TORUS SECTOR (major 2r, minor r) from
//! the wall's tangency section (where Q's rail on wall A meets rim A's rail)
//! to the catalogue's C5 stop — the kernel's one-ended runout doctrine (row
//! 3.7, `blend/runout.rs`): the stripe ends where its kept contact, on the
//! cap, reaches the unselected edge's line extended (wall B's plane), at
//! sin θ = r/2r = 30° for one radius, and is closed by a planar CLIFF in that
//! section's own plane, area (1 − π/4)·r².  The one-ended runout composition
//! builds exactly this body when the whole selection is its class (one
//! convex spine among concave edges); this closure gives the cutter
//! composition the same body when the selection carries other convex edges
//! too, so the corner reads the same whichever lane took the group (the
//! Pappus bite is (π/6)(11/6 − π/2) r³, `one_ended_runout_corner.py`).
//!
//! Pure topology surgery on the composed body, no booleans:
//!   * rim A's wall is trimmed back to its section at the tangency plane
//!     (the A junction arc V_A→W_A, r about the wall's axis);
//!   * the pad is trimmed back over the sector: its inner equator V_A→P (r
//!     above the cap) and the generator P→F down to its bottom arc, which is
//!     split at F;
//!   * the cap is trimmed back to the torus' outer contact arc W_A→C (2r
//!     about Q's axis) and the cliff's foot C→F;
//!   * the scaffold square, the misplaced bulkhead and every face that used
//!     only the dropped edges are deleted; the torus sector and the cliff
//!     are sewn on.
//! Refuses by name wherever the body is not that shape (another order, a
//! non-planar or oblique wall, a vertex missing), and the composition then
//! ships what it had.
use super::*;
use super::super::rollover_surgery::split_edge_at_vertices;

const CTX: &str = "round_mixed_concave_open_corner";

/// `BREP_BLEND_OPEN_MIXED_CORNER=1` lets the cutter composition close an open
/// mixed corner with this closure, `=0` keeps the composition's own body.
/// Promoted 2026-10-14 (lane L's record: case manifest OK with it on, the
/// fillet suites green); `=0` is the escape hatch. It also follows the
/// one-ended runout's switch (`BREP_BLEND_ONE_ENDED_RUNOUT`), whose body it
/// builds, so that escape hatch keeps meaning "the class as before".
const OPEN_MIXED_CORNER_DEFAULT: bool = true;

thread_local! {
    static OPEN_MIXED_CORNER_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

pub(crate) fn open_mixed_corner_on() -> bool {
    let own = match OPEN_MIXED_CORNER_FOR_TEST.with(|forced| forced.get()) {
        Some(forced) => forced,
        None => match std::env::var("BREP_BLEND_OPEN_MIXED_CORNER").as_deref() {
            Ok("0") | Ok("off") => false,
            Ok("1") | Ok("on") => true,
            _ => OPEN_MIXED_CORNER_DEFAULT,
        },
    };
    own && crate::blend::runout::one_ended_runout_on()
}


fn unsupported(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{CTX}: {message}"))
}

fn internal(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::internal(KernelStage::Sew, what, format!("{CTX}: {message}"))
}

fn is_planar(face: &FaceRecord) -> bool {
    matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. }))
}

/// Outward normal of a planar face at `at`.
fn planar_outward(face: &FaceRecord, at: Vec3) -> Result<Vec3, KernelRefusal> {
    if !is_planar(face) {
        return Err(unsupported("planar_walls", format!("face {} is not planar", face.id)));
    }
    let projection = crate::project_point_to_surface(&face.surface, at).or_refuse(KernelStage::Classify, "project")?;
    let normal = face.surface.normal(projection.u, projection.v).or_refuse(KernelStage::Classify, "normal")?;
    Ok(if face.same_sense { normal } else { normal.scale(-1.0) })
}

/// The faces of `solid` using `edge_id`, each with its coedge's sense.
fn faces_with_sense(solid: &BrepSolid, edge_id: u64) -> Vec<(&FaceRecord, bool)> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter_map(|face| {
            face.loops
                .iter()
                .flat_map(|lp| &lp.coedges)
                .find(|co| co.edge_id == edge_id)
                .map(|co| (face, co.forward))
        })
        .collect()
}

fn edge_of(solid: &BrepSolid, id: u64) -> Result<&EdgeRecord, KernelRefusal> {
    solid.edges.iter().find(|e| e.id == id).ok_or_else(|| internal("edge", format!("edge {id} missing")))
}

fn unit_tangent(edge: &EdgeRecord, t: f64) -> Result<Vec3, KernelRefusal> {
    let d = edge.curve.derivatives(t, 1).or_refuse(KernelStage::Classify, "derivatives")?;
    d[1].normalized().or_refuse(KernelStage::Classify, "normalized")
}

/// Convexity of a planar×planar edge: `(n_A × n_B)·t > 0` with A the face
/// whose coedge runs forward along the edge (the reading `report_fillet_probe`
/// prints: positive on every convex edge of the census, negative on the
/// concave ones).
fn edge_is_convex(solid: &BrepSolid, edge: &EdgeRecord, at: Vec3) -> Result<(bool, [(u64, bool); 2]), KernelRefusal> {
    let mates = faces_with_sense(solid, edge.id);
    if mates.len() != 2 {
        return Err(unsupported("manifold_edge", format!("edge {} has {} mates", edge.id, mates.len())));
    }
    let (a, b) = if mates[0].1 { (mates[0], mates[1]) } else { (mates[1], mates[0]) };
    let na = planar_outward(a.0, at)?;
    let nb = planar_outward(b.0, at)?;
    let t = unit_tangent(edge, 0.5 * (edge.t0 + edge.t1))?;
    let s = na.cross(nb).dot(t);
    if s.abs() < 1e-6 {
        return Err(unsupported("tangent_mates", format!("edge {} has tangent mates", edge.id)));
    }
    Ok((s > 0.0, [(mates[0].0.id, mates[0].1), (mates[1].0.id, mates[1].1)]))
}

/// Direction of `edge` leaving the vertex at `point` (one of its ends).
fn direction_away_from(edge: &EdgeRecord, point: Vec3, tol: f64) -> Result<Vec3, KernelRefusal> {
    let start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?;
    let end = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Classify, "evaluate")?;
    if start.sub(point).length() <= tol {
        unit_tangent(edge, edge.t0)
    } else if end.sub(point).length() <= tol {
        Ok(unit_tangent(edge, edge.t1)?.scale(-1.0))
    } else {
        Err(unsupported("edge_end", format!("edge {} does not end at {point:?}", edge.id)))
    }
}

fn near(a: Vec3, b: Vec3, tol: f64) -> bool {
    a.sub(b).length() <= tol
}

/// A point on `face`'s carrier, read through its first coedge's pcurve.
fn sample_point(face: &FaceRecord) -> Result<Vec3, KernelRefusal> {
    let co = face
        .loops
        .iter()
        .flat_map(|lp| &lp.coedges)
        .next()
        .ok_or_else(|| internal("empty_face", format!("face {} has no coedge", face.id)))?;
    let [q0, q1] = co.pcurve.domain().or_refuse(KernelStage::Classify, "domain")?;
    let uv = co.pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Classify, "evaluate")?;
    face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Classify, "evaluate")
}

fn distance_to_axis(point: Vec3, axis_point: Vec3, axis_dir: Vec3) -> f64 {
    let d = point.sub(axis_point);
    d.sub(axis_dir.scale(d.dot(axis_dir))).length()
}

/// The pcurve of a fresh edge on `face`'s carrier: an affine image on a
/// plane, a straight parameter line between the run's uv ends on another
/// analytic carrier (every fresh edge here is an iso-line of a cylinder), a
/// projection fit otherwise.
fn fresh_pcurve(face: &FaceRecord, curve: &NurbsCurve, uv_ends: Option<(Vec3, Vec3)>, radius: f64) -> Result<NurbsCurve, KernelRefusal> {
    if is_planar(face) {
        return crate::pcurve::build_pcurve_on_surface(&face.surface, curve).or_refuse(KernelStage::Fragment, "build_pcurve");
    }
    if let (Some(_), Some((uv_a, uv_b))) = (face.surface.analytic(), uv_ends) {
        return crate::sweep_topology::parameter_line(uv_a.x, uv_a.y, uv_b.x, uv_b.y).or_refuse(KernelStage::Fragment, "parameter_line");
    }
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    crate::pcurve::build_pcurve_on_surface_range(&face.surface, curve, t0, t1, true, 1e-9 * (1.0 + radius))
        .or_refuse(KernelStage::Fragment, "build_pcurve")
}


/// The run of `face`'s single loop between the vertices `from` and `to`
/// whose interior vertices all satisfy `inside`: the coedges before it (from
/// the loop's start), its edge ids, the coedges after it, the vertices it
/// starts and ends at IN TRAVERSAL ORDER, and the uv the carrier reads there
/// (the predecessor's pcurve end, the successor's pcurve start).
struct CornerRun {
    prefix: Vec<CoedgeRecord>,
    run_edges: Vec<u64>,
    suffix: Vec<CoedgeRecord>,
    start_vertex: u64,
    end_vertex: u64,
    uv_start: Vec3,
    uv_end: Vec3,
}

fn corner_run(solid: &BrepSolid, face: &FaceRecord, from: u64, to: u64, inside: &dyn Fn(Vec3) -> bool) -> Result<CornerRun, KernelRefusal> {
    if face.loops.len() != 1 {
        return Err(unsupported("multi_loop", format!("face {} has {} loops", face.id, face.loops.len())));
    }
    let point_of = |id: u64| -> Result<Vec3, KernelRefusal> {
        solid.vertices.iter().find(|v| v.id == id).map(|v| v.point).ok_or_else(|| internal("vertex", format!("vertex {id} missing")))
    };
    let ends = |co: &CoedgeRecord| -> Result<(u64, u64), KernelRefusal> {
        let edge = edge_of(solid, co.edge_id)?;
        Ok(if co.forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) })
    };
    let coedges = &face.loops[0].coedges;
    let n = coedges.len();
    let mut walk: Vec<(u64, u64)> = Vec::with_capacity(n);
    for co in coedges {
        walk.push(ends(co)?);
    }
    // The two arcs of the cycle between the anchors; keep the one whose
    // interior vertices are all inside.
    let mut candidates: Vec<(usize, usize)> = Vec::new(); // (first index, count)
    for (start_vertex, end_vertex) in [(from, to), (to, from)] {
        let Some(first) = (0..n).find(|&i| walk[i].0 == start_vertex) else { continue };
        let mut count = 0usize;
        let mut ok = true;
        loop {
            let idx = (first + count) % n;
            count += 1;
            if walk[idx].1 == end_vertex {
                break;
            }
            if !inside(point_of(walk[idx].1)?) {
                ok = false;
                break;
            }
            if count >= n {
                ok = false;
                break;
            }
        }
        if ok {
            candidates.push((first, count));
        }
    }
    let (first, count) = match candidates.as_slice() {
        [one] => *one,
        [a, b] => if a.1 <= b.1 { *a } else { *b },
        _ => return Err(unsupported("corner_run", format!("face {} has no corner run between vertices {from} and {to}", face.id))),
    };
    if count >= n - 1 {
        return Err(unsupported("corner_run_span", format!("face {}'s corner run leaves fewer than two coedges", face.id)));
    }
    let run_edges: Vec<u64> = (0..count).map(|k| coedges[(first + k) % n].edge_id).collect();
    let before_index = (first + n - 1) % n;
    let after_index = (first + count) % n;
    // Rotate the loop so it starts at the successor: suffix is then empty and
    // the prefix runs successor … predecessor.
    let mut prefix = Vec::with_capacity(n - count);
    let mut k = after_index;
    while k != first {
        prefix.push(coedges[k].clone());
        k = (k + 1) % n;
    }
    let before = &coedges[before_index];
    let after = &coedges[after_index];
    let [_, b1] = before.pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let [a0, _] = after.pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    let uv_start = before.pcurve.evaluate(b1).or_refuse(KernelStage::Fragment, "evaluate")?;
    let uv_end = after.pcurve.evaluate(a0).or_refuse(KernelStage::Fragment, "evaluate")?;
    Ok(CornerRun {
        prefix,
        run_edges,
        suffix: Vec::new(),
        start_vertex: walk[first].0,
        end_vertex: walk[(first + count - 1) % n].1,
        uv_start,
        uv_end,
    })
}

/// A fresh edge from `start` to `end` on `curve` (its whole domain).
fn fresh_edge(curve: NurbsCurve, start: u64, end: u64, next_id: &mut dyn FnMut() -> u64) -> Result<EdgeRecord, KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
    Ok(EdgeRecord { id: next_id(), curve, t0, t1, start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None })
}

fn arc_between(centre: Vec3, from: Vec3, to: Vec3, radius: f64) -> Result<NurbsCurve, KernelRefusal> {
    let x_axis = from.sub(centre).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let rb = to.sub(centre);
    let y_axis = rb.sub(x_axis.scale(rb.dot(x_axis))).normalized().or_refuse(KernelStage::Fragment, "normalized")?;
    let sweep = x_axis.dot(rb.normalized().or_refuse(KernelStage::Fragment, "normalized")?).clamp(-1.0, 1.0).acos();
    crate::make_arc(centre, x_axis, y_axis, radius, 0.0, sweep).or_refuse(KernelStage::Fragment, "make_arc")
}

/// Where `point` lies on `edge`: the nearest parameter and its distance.
fn on_edge(edge: &EdgeRecord, point: Vec3) -> Result<(f64, f64), KernelRefusal> {
    let projection = crate::project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Classify, "project_point_to_curve")?;
    Ok((projection.u, projection.distance))
}

/// The edges two faces of `solid` share.
fn common_edges(solid: &BrepSolid, a_id: u64, b_id: u64) -> Vec<u64> {
    let uses = |fid: u64| -> Vec<u64> {
        solid
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .filter(|f| f.id == fid)
            .flat_map(|f| f.loops.iter().flat_map(|lp| lp.coedges.iter().map(|co| co.edge_id)))
            .collect()
    };
    let b = uses(b_id);
    uses(a_id).into_iter().filter(|e| b.contains(e)).collect()
}

/// The vertex of `result` at `point`: an existing one, or a fresh one
/// splitting the edge the two faces share there.
fn ensure_vertex(
    result: &mut BrepSolid,
    faces: (u64, u64),
    point: Vec3,
    tol: f64,
    what: &'static str,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<u64, KernelRefusal> {
    if let Some(v) = result.vertices.iter().find(|v| near(v.point, point, tol)) {
        return Ok(v.id);
    }
    for edge_id in common_edges(result, faces.0, faces.1) {
        let edge = edge_of(result, edge_id)?.clone();
        let (t, distance) = on_edge(&edge, point)?;
        if distance > tol {
            continue;
        }
        let fraction = (t - edge.t0) / (edge.t1 - edge.t0);
        if !(fraction > 1e-9 && fraction < 1.0 - 1e-9) {
            continue;
        }
        let vid = next_id();
        result.vertices.push(VertexRecord { id: vid, point });
        split_edge_at_vertices(result, edge_id, &[(vid, point)], next_id)?;
        return Ok(vid);
    }
    Err(unsupported(what, format!("no rail through {point:?} between faces {} and {}", faces.0, faces.1)))
}

/// The uv of `point` on `face`'s carrier.
fn uv_on(face: &FaceRecord, point: Vec3, tol: f64) -> Result<Vec3, KernelRefusal> {
    let projection = crate::project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Fragment, "project")?;
    if projection.distance > tol {
        return Err(unsupported("off_carrier", format!("{point:?} is {:.3e} off face {}'s carrier", projection.distance, face.id)));
    }
    Ok(Vec3::new(projection.u, projection.v, 0.0))
}

/// A chain of fresh edges replacing a run: each coedge forward along the
/// chain, its pcurve on the carrier (an affine image on a plane, a straight
/// parameter line through the chain's uv stations on another analytic
/// carrier, a fit otherwise).
fn chain_coedges(face: &FaceRecord, chain: &[&EdgeRecord], uv_stations: &[Vec3], radius: f64, next_id: &mut dyn FnMut() -> u64) -> Result<Vec<CoedgeRecord>, KernelRefusal> {
    let mut out = Vec::with_capacity(chain.len());
    for (k, edge) in chain.iter().enumerate() {
        let uv_ends = if is_planar(face) { None } else { Some((uv_stations[k], uv_stations[k + 1])) };
        out.push(CoedgeRecord { id: next_id(), edge_id: edge.id, forward: true, pcurve: fresh_pcurve(face, &edge.curve, uv_ends, radius)? });
    }
    Ok(out)
}

/// Close the open mixed corner of `solid` (the composed body) at `corner`,
/// the two SELECTED edges (ids on `source`, the input body) being the convex
/// rim A and the concave vertical Q in either order. `name` names the torus
/// sector; the cliff takes `{name}_cliff` as the one-ended runout's does.
pub(crate) fn round_mixed_concave_open_corner(
    solid: &BrepSolid,
    source: &BrepSolid,
    corner: Vec3,
    selected_edges: [u64; 2],
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::FxHashSet as HashSet;
    let tol = 1e-6 * (1.0 + radius);

    // ---- 1. Classify on the input: one convex rim, one concave vertical,
    //         three orthogonal planar faces, an unselected convex rim B.
    let e0 = edge_of(source, selected_edges[0])?;
    let e1 = edge_of(source, selected_edges[1])?;
    let (convex0, mates0) = edge_is_convex(source, e0, corner)?;
    let (convex1, mates1) = edge_is_convex(source, e1, corner)?;
    let (rim_a, q, mates_a, mates_q) = match (convex0, convex1) {
        (true, false) => (e0, e1, mates0, mates1),
        (false, true) => (e1, e0, mates1, mates0),
        _ => return Err(unsupported("convexity_mix", "the two selected edges are not one convex and one concave".to_string())),
    };
    let face_of = |id: u64| -> Result<&FaceRecord, KernelRefusal> {
        source.shells.iter().flat_map(|s| &s.faces).find(|f| f.id == id).ok_or_else(|| internal("face", format!("face {id} missing")))
    };
    let wall_a_id = mates_a
        .iter()
        .map(|(id, _)| *id)
        .find(|id| mates_q.iter().any(|(other, _)| other == id))
        .ok_or_else(|| unsupported("common_wall", "the rim and the vertical share no face".to_string()))?;
    let cap_id = mates_a.iter().map(|(id, _)| *id).find(|id| *id != wall_a_id).unwrap();
    let wall_b_id = mates_q.iter().map(|(id, _)| *id).find(|id| *id != wall_a_id).unwrap();
    let (wall_a, cap, wall_b) = (face_of(wall_a_id)?, face_of(cap_id)?, face_of(wall_b_id)?);
    let (na, nb, nc) = (planar_outward(wall_a, corner)?, planar_outward(wall_b, corner)?, planar_outward(cap, corner)?);
    if na.dot(nb).abs() > 1e-6 || na.dot(nc).abs() > 1e-6 || nb.dot(nc).abs() > 1e-6 {
        return Err(unsupported("orthogonal", "the three faces at the corner are not orthogonal".to_string()));
    }
    let rim_b = source
        .edges
        .iter()
        .find(|edge| {
            edge.id != rim_a.id
                && edge.id != q.id
                && !edge.degenerate
                && [edge.t0, edge.t1].iter().any(|t| edge.curve.evaluate(*t).map(|p| near(p, corner, tol)).unwrap_or(false))
                && {
                    let mates = faces_with_sense(source, edge.id);
                    mates.len() == 2 && mates.iter().any(|(f, _)| f.id == wall_b_id) && mates.iter().any(|(f, _)| f.id == cap_id)
                }
        })
        .ok_or_else(|| unsupported("rim_b", "no third edge between wall B and the cap at the corner".to_string()))?;
    if !edge_is_convex(source, rim_b, corner)?.0 {
        return Err(unsupported("rim_b_convex", "the unselected third edge is not convex".to_string()));
    }
    let t_a = direction_away_from(rim_a, corner, tol)?;
    let t_b = direction_away_from(rim_b, corner, tol)?;
    if t_a.dot(nb).abs() < 1.0 - 1e-6 || t_b.dot(na).abs() < 1.0 - 1e-6 {
        return Err(unsupported("rim_directions", "the rims do not run along the opposite walls' normals".to_string()));
    }

    // ---- 2. The corner's closed-form points.  The ball's centre circle:
    //         q_v + 2r(cos θ·d_a + sin θ·d_b), d_a = −n_a toward the A side,
    //         d_b = −n_b toward wall B; C5's stop where the cap contact
    //         reaches wall B's plane: (centre − corner)·n_b = r − 2r sin θ = 0.
    let r = radius;
    let (d_a, d_b) = (na.scale(-1.0), nb.scale(-1.0));
    let q_v = corner.add(na.scale(r)).add(nb.scale(r)).sub(nc.scale(r));
    let q_w = corner.add(na.scale(r)).add(nb.scale(r));
    let v_a = corner.add(nb.scale(r)).sub(nc.scale(r));
    let w_a = corner.add(nb.scale(r)).sub(na.scale(r));
    let c_a = corner.add(nb.scale(r)).sub(na.scale(r)).sub(nc.scale(r));
    let theta = (0.5f64).asin();
    let radial = d_a.scale(theta.cos()).add(d_b.scale(theta.sin()));
    let s_stop = q_v.add(radial.scale(2.0 * r));
    let p_stop = q_v.add(radial.scale(r));
    let c_stop = s_stop.add(nc.scale(r));
    let f_stop = p_stop.add(nc.scale(r));
    let cliff_normal = d_a.scale(-theta.sin()).add(d_b.scale(theta.cos()));
    let axis_dir = nc.scale(-1.0);

    // ---- 3. The composed body: the tangency vertex, the pad, the wall,
    //         the cap face around the corner.
    let v_a_id = solid
        .vertices
        .iter()
        .find(|v| near(v.point, v_a, tol))
        .map(|v| v.id)
        .ok_or_else(|| unsupported("tangency_vertex", format!("no vertex at the wall/pad tangency {v_a:?} (fillet the concave edge before the rim)")))?;
    let edges_at_v_a: Vec<u64> = solid.edges.iter().filter(|e| e.start_vertex_id == v_a_id || e.end_vertex_id == v_a_id).map(|e| e.id).collect();
    let mut pad_id = None;
    let mut fillet_a_id = None;
    for face in solid.shells.iter().flat_map(|s| &s.faces) {
        if is_planar(face) || !face.loops.iter().flat_map(|lp| &lp.coedges).any(|co| edges_at_v_a.contains(&co.edge_id)) {
            continue;
        }
        let sample = sample_point(face)?;
        if (distance_to_axis(sample, q_v, axis_dir) - r).abs() <= tol {
            pad_id = Some(face.id);
        } else if (distance_to_axis(sample, c_a, t_a) - r).abs() <= tol {
            fillet_a_id = Some(face.id);
        }
    }
    let pad_id = pad_id.ok_or_else(|| unsupported("pad", "no concave pad at the tangency vertex".to_string()))?;
    let fillet_a_id = fillet_a_id.ok_or_else(|| unsupported("wall", "no rim wall at the tangency vertex".to_string()))?;
    let on_plane = |face: &FaceRecord, normal: Vec3, through: Vec3| -> bool {
        match face.surface.analytic() {
            Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
                let Ok(n) = u_dir.cross(*v_dir).normalized() else { return false };
                n.dot(normal).abs() > 1.0 - 1e-6 && normal.dot(through.sub(*origin)).abs() < tol
            }
            _ => false,
        }
    };
    let cap_face_id = solid
        .shells
        .iter()
        .flat_map(|s| &s.faces)
        .find(|f| on_plane(f, nc, corner) && !common_edges(solid, f.id, fillet_a_id).is_empty() && !common_edges(solid, f.id, pad_id).is_empty())
        .map(|f| f.id)
        .ok_or_else(|| {
            if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                for f in solid.shells.iter().flat_map(|s| &s.faces).filter(|f| on_plane(f, nc, corner)) {
                    eprintln!(
                        "{CTX}: cap-plane face {} {:?} area {:.6} borders wall {} pad {}",
                        f.id,
                        f.name,
                        crate::face_area(f).unwrap_or(f64::NAN),
                        !common_edges(solid, f.id, fillet_a_id).is_empty(),
                        !common_edges(solid, f.id, pad_id).is_empty()
                    );
                }
                for f in solid.shells.iter().flat_map(|s| &s.faces).filter(|f| f.id == pad_id || f.id == fillet_a_id) {
                    let walk: Vec<String> = f.loops[0].coedges.iter().map(|co| {
                        let e = solid.edges.iter().find(|e| e.id == co.edge_id).unwrap();
                        let (a, b) = (e.curve.evaluate(e.t0).unwrap(), e.curve.evaluate(e.t1).unwrap());
                        format!("{}{}[({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})]", if co.forward { "+" } else { "-" }, e.id, a.x, a.y, a.z, b.x, b.y, b.z)
                    }).collect();
                    eprintln!("{CTX}: face {} {:?}: {}", f.id, f.name, walk.join(" "));
                }
            }
            unsupported("cap_face", "no cap face borders both the rim wall and the pad".to_string())
        })?;

    let mut result = solid.clone();
    let mut next_id = crate::blend::edge::fresh_id_source(&result);
    let w_a_id = ensure_vertex(&mut result, (fillet_a_id, cap_face_id), w_a, tol, "cap_rail", &mut next_id)?;
    let f_id = ensure_vertex(&mut result, (pad_id, cap_face_id), f_stop, tol, "pad_foot", &mut next_id)?;
    let p_id = next_id();
    result.vertices.push(VertexRecord { id: p_id, point: p_stop });
    let c_id = next_id();
    result.vertices.push(VertexRecord { id: c_id, point: c_stop });

    // ---- 4. Rebuild the three carriers' loops.
    let face_by_id = |result: &BrepSolid, id: u64| -> Result<FaceRecord, KernelRefusal> {
        result.shells.iter().flat_map(|s| &s.faces).find(|f| f.id == id).cloned().ok_or_else(|| internal("face", format!("face {id} missing")))
    };
    let mut dropped: HashSet<u64> = HashSet::default();
    let mut fresh_edges: Vec<EdgeRecord> = Vec::new();
    let mut new_loops: Vec<(u64, Vec<CoedgeRecord>)> = Vec::new();
    let point_of = |result: &BrepSolid, id: u64| -> Vec3 { result.vertices.iter().find(|v| v.id == id).map(|v| v.point).unwrap() };
    // A chain of fresh edges from `start` to `end` through `vias`, in the
    // run's traversal order: `curves[k]` joins station k to station k+1.
    let mut rebuild = |result: &BrepSolid, face_id: u64, run: &CornerRun, stations: &[u64], curves: Vec<NurbsCurve>, next_id: &mut dyn FnMut() -> u64| -> Result<Vec<EdgeRecord>, KernelRefusal> {
        let face = face_by_id(result, face_id)?;
        let mut chain = Vec::with_capacity(curves.len());
        for (k, curve) in curves.into_iter().enumerate() {
            chain.push(fresh_edge(curve, stations[k], stations[k + 1], next_id)?);
        }
        let mut uv_stations = vec![run.uv_start];
        for station in &stations[1..stations.len() - 1] {
            uv_stations.push(uv_on(&face, point_of(result, *station), tol)?);
        }
        uv_stations.push(run.uv_end);
        let refs: Vec<&EdgeRecord> = chain.iter().collect();
        let mut coedges = run.prefix.clone();
        coedges.extend(chain_coedges(&face, &refs, &uv_stations, r, next_id)?);
        coedges.extend(run.suffix.clone());
        new_loops.push((face_id, coedges));
        Ok(chain)
    };

    // 4a. The wall: its run between W_A and V_A on the corner side of the
    //     tangency plane; one fresh arc, r about the wall's axis.
    let wall = face_by_id(&result, fillet_a_id)?;
    let wall_run = corner_run(&result, &wall, w_a_id, v_a_id, &|p: Vec3| p.sub(v_a).dot(t_a) <= tol)?;
    let (ws, we) = (point_of(&result, wall_run.start_vertex), point_of(&result, wall_run.end_vertex));
    let arc_a = rebuild(&result, fillet_a_id, &wall_run, &[wall_run.start_vertex, wall_run.end_vertex], vec![arc_between(c_a, ws, we, r)?], &mut next_id)?.remove(0);
    dropped.extend(wall_run.run_edges.iter().copied());
    fresh_edges.push(arc_a.clone());

    // 4b. The pad: its run between V_A and F at or below the equator; the
    //     inner equator V_A→P (r about Q's axis) and the generator P→F.
    let pad = face_by_id(&result, pad_id)?;
    let pad_run = corner_run(&result, &pad, v_a_id, f_id, &|p: Vec3| p.sub(q_v).dot(axis_dir) <= tol)?;
    let pad_stations: Vec<u64> = if pad_run.start_vertex == v_a_id { vec![v_a_id, p_id, f_id] } else { vec![f_id, p_id, v_a_id] };
    let pad_curves = if pad_run.start_vertex == v_a_id {
        vec![arc_between(q_v, v_a, p_stop, r)?, crate::make_line(p_stop, f_stop).or_refuse(KernelStage::Fragment, "make_line")?]
    } else {
        vec![crate::make_line(f_stop, p_stop).or_refuse(KernelStage::Fragment, "make_line")?, arc_between(q_v, p_stop, v_a, r)?]
    };
    let pad_chain = rebuild(&result, pad_id, &pad_run, &pad_stations, pad_curves, &mut next_id)?;
    let (v_arc, generator) = if pad_run.start_vertex == v_a_id { (pad_chain[0].clone(), pad_chain[1].clone()) } else { (pad_chain[1].clone(), pad_chain[0].clone()) };
    dropped.extend(pad_run.run_edges.iter().copied());
    fresh_edges.extend(pad_chain.iter().cloned());

    // 4c. The cap: its run between F and W_A inside the corner box; the
    //     cliff's foot F→C and the torus' contact arc C→W_A (2r about Q's
    //     axis).
    let cap_face = face_by_id(&result, cap_face_id)?;
    let in_box = |p: Vec3| p.sub(corner).dot(t_a) <= r + tol && p.sub(corner).dot(t_b) <= r + tol && p.sub(corner).dot(nc).abs() <= tol;
    let cap_run = corner_run(&result, &cap_face, f_id, w_a_id, &in_box)?;
    let cap_stations: Vec<u64> = if cap_run.start_vertex == f_id { vec![f_id, c_id, w_a_id] } else { vec![w_a_id, c_id, f_id] };
    let cap_curves = if cap_run.start_vertex == f_id {
        vec![crate::make_line(f_stop, c_stop).or_refuse(KernelStage::Fragment, "make_line")?, arc_between(q_w, c_stop, w_a, 2.0 * r)?]
    } else {
        vec![arc_between(q_w, w_a, c_stop, 2.0 * r)?, crate::make_line(c_stop, f_stop).or_refuse(KernelStage::Fragment, "make_line")?]
    };
    let cap_chain = rebuild(&result, cap_face_id, &cap_run, &cap_stations, cap_curves, &mut next_id)?;
    let (foot_edge, w_arc) = if cap_run.start_vertex == f_id { (cap_chain[0].clone(), cap_chain[1].clone()) } else { (cap_chain[1].clone(), cap_chain[0].clone()) };
    dropped.extend(cap_run.run_edges.iter().copied());
    fresh_edges.extend(cap_chain.iter().cloned());

    // ---- 5. The cliff in the final section's plane: the section arc P→C
    //         (r about the stop centre), the foot edge and the generator,
    //         each opposite its neighbour (the cap's and the pad's uses are
    //         forward); the arc takes whichever sense chains.
    let arc_b = fresh_edge(arc_between(s_stop, p_stop, c_stop, r)?, p_id, c_id, &mut next_id)?;
    fresh_edges.push(arc_b.clone());
    let reach = 4.0 * r;
    let plane_u = radial;
    let plane_v = axis_dir;
    let plane_origin = f_stop.sub(plane_u.scale(reach)).sub(plane_v.scale(reach));
    let plane = crate::make_plane(plane_origin, plane_u, plane_v, 2.0 * reach, 2.0 * reach).or_refuse(KernelStage::Fragment, "make_plane")?;
    let mapped = |edge: &EdgeRecord| -> Result<NurbsCurve, KernelRefusal> {
        let mut control = Vec::with_capacity(edge.curve.control_points.len());
        for cp in &edge.curve.control_points {
            let point = cp.point().or_refuse(KernelStage::Fragment, "point")?;
            let projection = crate::project_point_to_surface(&plane, point).or_refuse(KernelStage::Fragment, "project")?;
            control.push(Vec4 { x: projection.u * cp.w, y: projection.v * cp.w, z: 0.0, w: cp.w });
        }
        NurbsCurve::new(edge.curve.degree, edge.curve.knots.clone(), control).or_refuse(KernelStage::Fragment, "new")
    };
    let sides: [(EdgeRecord, Option<bool>); 3] = [(generator.clone(), Some(false)), (foot_edge.clone(), Some(false)), (arc_b.clone(), None)];
    let walk = |edge: &EdgeRecord, forward: bool| -> (u64, u64) {
        if forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        }
    };
    let mut chosen: Vec<(usize, bool)> = vec![(0, false)];
    let mut cursor = walk(&sides[0].0, false).1;
    for _ in 0..2 {
        let mut found = None;
        for s in 0..3 {
            if chosen.iter().any(|(k, _)| *k == s) {
                continue;
            }
            for forward in [true, false] {
                if sides[s].1.is_some_and(|fixed| fixed != forward) {
                    continue;
                }
                if walk(&sides[s].0, forward).0 == cursor {
                    found = Some((s, forward));
                }
            }
        }
        let (s, forward) = found.ok_or_else(|| internal("cliff_chain", "the cliff's boundary does not chain".to_string()))?;
        cursor = walk(&sides[s].0, forward).1;
        chosen.push((s, forward));
    }
    if cursor != walk(&sides[0].0, false).0 {
        return Err(internal("cliff_loop", "the cliff's boundary is not closed".to_string()));
    }
    let arc_b_forward_on_cliff = chosen.iter().find(|(s, _)| *s == 2).map(|(_, f)| *f).unwrap();
    let mut cliff_coedges = Vec::with_capacity(3);
    for (s, forward) in chosen {
        let edge = &sides[s].0;
        let pcurve = mapped(edge)?;
        let pcurve = if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")? };
        cliff_coedges.push(CoedgeRecord { id: next_id(), edge_id: edge.id, forward, pcurve });
    }
    let plane_normal = plane.normal(0.5, 0.5).or_refuse(KernelStage::Fragment, "normal")?;
    // Outward: toward the stripe's continuation (the sharp rim beyond the
    // stop), the cliff keeping it sharp.
    let cliff_loop_id = next_id();
    let cliff = FaceRecord {
        id: next_id(),
        surface: plane,
        same_sense: plane_normal.dot(cliff_normal) > 0.0,
        loops: vec![LoopRecord { id: cliff_loop_id, coedges: cliff_coedges }],
        name: name.map(|n| format!("{n}_cliff")),
    };

    // ---- 6. The torus sector: generatrix the A junction arc, revolved about
    //         Q's axis to the stop's section; the four sides shared with the
    //         wall (arc A), the cap (W arc), the cliff (arc B) and the pad
    //         (V arc), each with the sense opposite its neighbour.
    let patch = build_mixed_sector_patch(
        CTX,
        &arc_a,
        &arc_b,
        &v_arc,
        &w_arc,
        true,
        arc_b_forward_on_cliff,
        (v_a_id, v_a),
        (p_id, p_stop),
        (w_a_id, w_a),
        (c_id, c_stop),
        q_v,
        axis_dir,
        c_a,
        s_stop,
        r,
        name,
        &mut next_id,
    )?;

    // ---- 7. Splice: loops, fresh edges, the two new faces; delete every
    //         other face that used a dropped edge (the scaffold square, the
    //         misplaced bulkhead), then the orphaned edges and vertices.
    let rebuilt: HashSet<u64> = [fillet_a_id, pad_id, cap_face_id].into_iter().collect();
    let mut junk: HashSet<u64> = HashSet::default();
    for shell in &result.shells {
        for face in &shell.faces {
            if rebuilt.contains(&face.id) {
                continue;
            }
            if face.loops.iter().flat_map(|lp| &lp.coedges).any(|co| dropped.contains(&co.edge_id)) {
                if !is_planar(face) {
                    return Err(unsupported("junk_face", format!("a non-planar face {} uses a dropped corner edge", face.id)));
                }
                junk.insert(face.id);
            }
        }
    }
    result.edges.extend(fresh_edges);
    let shell_index = result
        .shells
        .iter()
        .position(|s| s.faces.iter().any(|f| f.id == pad_id))
        .ok_or_else(|| internal("shell", "the pad's shell vanished".to_string()))?;
    for (si, shell) in result.shells.iter_mut().enumerate() {
        shell.faces.retain(|f| !junk.contains(&f.id));
        for face in shell.faces.iter_mut() {
            if let Some((_, coedges)) = new_loops.iter().find(|(id, _)| *id == face.id) {
                face.loops[0].coedges = coedges.clone();
            }
        }
        if si == shell_index {
            shell.faces.push(patch.clone());
            shell.faces.push(cliff.clone());
        }
    }
    let mut edge_uses: rustc_hash::FxHashMap<u64, usize> = rustc_hash::FxHashMap::default();
    for shell in &result.shells {
        for face in &shell.faces {
            for lp in &face.loops {
                for co in &lp.coedges {
                    *edge_uses.entry(co.edge_id).or_default() += 1;
                }
            }
        }
    }
    result.edges.retain(|e| edge_uses.get(&e.id).copied().unwrap_or(0) > 0);
    let referenced: HashSet<u64> = result.edges.iter().flat_map(|e| [e.start_vertex_id, e.end_vertex_id]).collect();
    result.vertices.retain(|v| referenced.contains(&v.id));

    // ---- 8. Validate; a flaw is the closure's own refusal.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(internal("validate", format!("{issues:?}")));
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!("{CTX}: closed the open mixed corner at {corner:?} ({} junk faces dropped)", junk.len());
    }
    Ok(result)
}

//! A set-back corner whose blends are curvature-continuous
//! (`fillet_edges_setback_g2`): three straight convex edges between planes meet
//! at one vertex; each is rounded by the order-2 section of
//! `fillet_edge_g2` (contact of order 2 with both faces) as far as the plane
//! `setback` from the corner along it, and the corner is closed by one patch
//! that meets the three blends and the three faces with their tangent planes
//! and their curvatures.
//!
//! Built in three steps.
//!
//! 1. The corner is cut back: the parallelepiped on the corner spanned by the
//!    three edges, `setback` along each, is taken off, so that each edge ends
//!    on a planar end face at a vertex with one edge along each of its faces,
//!    and the strip of each is built directly (`g2_prism`): the section
//!    extruded along the edge, exact, ending on the carried section.
//! 2. The three end faces go. On each face the two strips' contact lines end
//!    at their cuts, and the face is closed between them by the planar quartic
//!    whose controls are the two cuts, the midpoints toward the lines'
//!    crossing and the crossing itself: tangent to both contact lines and as
//!    straight as they are at its ends (its second derivative vanishes there).
//!    The corner's hole has six sides, three sections and three face curves,
//!    and six right-angled corners.
//! 3. The hole is filled by the side-ribbon transfinite patch over a regular
//!    hexagon (Gregory; Várady): `P = Σ w_k R_k(s_k, d_k)`, `d_k` the distance
//!    to side `k` (1 at the centre), `s_k = d_{k-1} / (d_{k-1} + d_{k+1})` and
//!    `w_k ∝ 1 / d_k³`, so that on side `k` every other weight vanishes with its
//!    first two derivatives and the patch agrees with the ribbon `R_k` to
//!    second order. A strip's ribbon is the strip itself continued toward the
//!    corner (`C(s) + λ(s) φ(d) e`, on its extrusion); a face's is a map into
//!    its plane (`Q(s) + φ(d) M(s) + φ(d)² N(s)`), with `M` and `N` solved at
//!    both its ends so that its second-order jet there is the neighbouring
//!    strip's: the section is flat where it meets the face, so that jet lies in
//!    the plane. `φ(d) = d / (1 + (d/δ)³)^⅓` (δ = `REACH`) keeps each ribbon's jet at its side
//!    and bounds how far it reaches across the patch. The patch is fitted as
//!    three bicubic patches over the hexagon's three rhombi, which share the
//!    centre and the seams to alternate corners; its sides are the exact
//!    sections and face curves, the fitted patch held to them within
//!    `BOUNDARY_BAR` of the corner's size, and its tangent planes and
//!    curvatures checked against the strips' and faces' along every side.
//!    Anything outside this construction is refused by name.
use super::*;
use crate::topology::{CoedgeRecord, LoopRecord, VertexRecord};
use crate::{NurbsSurface, Vec4};

const ENTRY: &str = "fillet_edges_setback_g2";

/// How far each ribbon reaches across the patch (in units of the hexagon's
/// inradius): `φ` saturates at it.
const REACH: f64 = 0.6;

/// Samples each way over a rhombus for the fit, clustered toward both ends.
const GRID: usize = 81;

/// The largest distance between the fitted patch and the exact sides, and
/// between the fit and the transfinite patch inside, as a fraction of the
/// corner's size.
const BOUNDARY_BAR: f64 = 1e-6;

/// The largest angle (radians) between the patch's normal and its
/// neighbour's along a side, or between two patches across a seam.
const ANGLE_BAR: f64 = 1e-4;

/// The largest difference in normal curvature across a side or a seam, as a
/// fraction of the strips' largest curvature.
const CURVATURE_BAR: f64 = 2e-2;

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Refine, what, format!("{ENTRY}: {message}"))
}

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{ENTRY}: {message}"))
}

fn is_line(edge: &EdgeRecord) -> bool {
    edge.curve.degree == 1 && edge.curve.control_points.len() == 2
}

fn ends(edge: &EdgeRecord) -> Result<(Vec3, Vec3), KernelRefusal> {
    Ok((edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?, edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?))
}

fn next_id_after(body: &BrepSolid) -> u64 {
    body.vertices
        .iter()
        .map(|v| v.id)
        .chain(body.edges.iter().map(|x| x.id))
        .chain(body.shells.iter().flat_map(|s| s.faces.iter()).flat_map(|f| std::iter::once(f.id).chain(f.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))))))
        .max()
        .unwrap_or(0)
        + 1
}

/// A straight parameter line from `a` to `b` over the parameter range `[t0, t1]`.
fn uv_line(a: [f64; 2], b: [f64; 2], t0: f64, t1: f64) -> Result<NurbsCurve, KernelRefusal> {
    NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![Vec4 { x: a[0], y: a[1], z: 0.0, w: 1.0 }, Vec4 { x: b[0], y: b[1], z: 0.0, w: 1.0 }]).or_refuse(KernelStage::Refine, "uv_line")
}

/// `fillet_edges_g2` of the three edges the points lie on, meeting at one
/// convex vertex between three planes, whose strips stop `setback` from the
/// vertex (measured along each edge) and whose corner is closed by one
/// curvature-continuous patch: tangent to the strips and the faces and with
/// their curvatures along every side (see the module notes). `offset` is the
/// blends' tangent offset and `center` their centre radius (where not
/// positive, the thumbweight that takes off what the ball's round does).
pub fn fillet_edges_setback_g2(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    offset: f64,
    setback: f64,
    center: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let _operation = crate::blend::BlendOperation::enter();
    if !(offset.is_finite() && offset > 0.0) || !center.is_finite() {
        return Err(input("g2_size", "the offset must be positive and the centre radius finite".into()));
    }
    if !(setback.is_finite() && setback > offset) {
        return Err(input("setback_size", format!("the setback {setback} must exceed the offset {offset}")));
    }
    if edge_points.len() != 3 {
        return Err(KernelRefusal::unsupported(KernelStage::Collect, "setback_selection", format!("{ENTRY}: three edges meeting at one corner are set back")));
    }
    if solid.shells.len() != 1 {
        return Err(refuse("setback_shells", "the body has more than one shell".into()));
    }
    let mut ids = [0u64; 3];
    for (k, point) in edge_points.iter().enumerate() {
        ids[k] = super::edges::resolve_edge_by_point(solid, *point)?;
    }
    if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
        return Err(input("setback_selection", "two of the points lie on one edge".into()));
    }
    let edge_of = |id: u64| solid.edges.iter().find(|e| e.id == id);
    let mut counts: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
    for &id in &ids {
        let edge = edge_of(id).ok_or_else(|| input("setback_edge", "an edge is missing".into()))?;
        for v in [edge.start_vertex_id, edge.end_vertex_id] {
            *counts.entry(v).or_insert(0) += 1;
        }
    }
    let corner_vertex = counts.iter().find(|(_, &c)| c == 3).map(|(&v, _)| v).ok_or_else(|| KernelRefusal::unsupported(KernelStage::Collect, "setback_selection", format!("{ENTRY}: the three edges do not meet at one vertex")))?;
    let corner = solid.vertices.iter().find(|v| v.id == corner_vertex).map(|v| v.point).ok_or_else(|| input("setback_vertex", "the corner vertex is missing".into()))?;
    let at_corner = solid.edges.iter().filter(|e| e.start_vertex_id == corner_vertex || e.end_vertex_id == corner_vertex).count();
    if at_corner != 3 {
        return Err(refuse("setback_g2_corner", format!("the corner has {at_corner} edges, not three")));
    }
    let mut t = [Vec3::new(0.0, 0.0, 0.0); 3];
    let mut length = [0.0f64; 3];
    for (k, &id) in ids.iter().enumerate() {
        let edge = edge_of(id).expect("edge");
        if !is_line(edge) {
            return Err(refuse("setback_g2_edge", "a selected edge is not straight".into()));
        }
        let (a, b) = ends(edge)?;
        let far = if edge.start_vertex_id == corner_vertex { b } else { a };
        length[k] = far.sub(corner).length();
        t[k] = far.sub(corner).normalized().or_refuse(KernelStage::Collect, "normalized")?;
        if !(length[k] > setback * (1.0 + 1e-6)) {
            return Err(input("setback_reach", format!("the setback {setback} reaches the far end of an edge {} long", length[k])));
        }
    }
    let det = t[0].dot(t[1].cross(t[2]));
    if det.abs() < 1e-6 {
        return Err(refuse("setback_g2_corner", "the three edges are nearly coplanar".into()));
    }
    let shortest = length.iter().copied().fold(f64::INFINITY, f64::min);
    let tolerance = 1e-9 * (1.0 + corner.length());
    let inside = corner.add(t[0].add(t[1]).add(t[2]).scale(1e-3 * shortest));
    if crate::classify_point(inside, solid, tolerance).or_refuse(KernelStage::Collect, "classify_point")?.class != crate::PointClass::In {
        return Err(refuse("setback_concave", "the corner is not convex: only a convex corner is set back".into()));
    }

    // 1. Cut the corner back and build the three strips.
    let (a, b, c) = if det > 0.0 { (t[0], t[1], t[2]) } else { (t[0], t[2], t[1]) };
    let margin = setback;
    let cube = crate::make_box_brep(Vec3::new(-margin, -margin, -margin), setback + margin, setback + margin, setback + margin).map_err(|e| refuse("setback_g2_notch", e))?;
    let place = crate::AffineTransform { elements: [a.x, b.x, c.x, corner.x, a.y, b.y, c.y, corner.y, a.z, b.z, c.z, corner.z, 0.0, 0.0, 0.0, 1.0] };
    let cutter = crate::transform_brep(&cube, place, false).map_err(|e| refuse("setback_g2_notch", e))?;
    let mut body = boolean_operation(solid, &cutter, BooleanOperation::Subtract, &BooleanOptions::default())
        .map_err(|r| refuse("setback_g2_notch", format!("the corner could not be cut back: {}", r.message)))?;
    if body.shells.len() != 1 {
        return Err(refuse("setback_g2_notch", "cutting the corner back left more than one shell".into()));
    }
    // The cutter reaches `setback` past each face as well: it must have taken
    // the corner's parallelepiped and nothing else (no rib or wall nearby).
    let taken = crate::solid_mass_properties(solid).or_refuse(KernelStage::Refine, "mass_properties")?.volume - crate::solid_mass_properties(&body).or_refuse(KernelStage::Refine, "mass_properties")?.volume;
    let notch = setback * setback * setback * det.abs();
    if (taken - notch).abs() > 1e-6 * notch {
        return Err(refuse("setback_g2_notch", format!("cutting the corner back takes {taken:.6e}, not the corner's {notch:.6e}: other material lies within the setback of the corner")));
    }
    for k in 0..3 {
        let probe = corner.add(t[k].scale(0.5 * (setback + length[k])));
        let id = super::edges::resolve_edge_by_point(&body, probe)?;
        let cross = analyze_edge(&body, id, offset).map_err(|r| refuse("setback_g2_strip", format!("edge {k}'s section could not be read ({})", r.message)))?;
        if !matches!(cross.path, EdgePath::Straight { .. }) {
            return Err(refuse("setback_g2_strip", "a strip is built only on a straight edge between planes".into()));
        }
        let profile = super::tool::g2_cross_section(&cross, offset, center)?;
        let built = super::g2_prism::blend_prism_edge(&body, id, &cross, &profile[1], super::edges::per_edge_name(edge_names, name, k))
            .ok_or_else(|| refuse("setback_g2_strip", format!("edge {k}'s strip could not be built on the cut-back corner")))?;
        check_blend_interference(&body, &built, &[id], ENTRY)?;
        body = built;
    }

    // 2. The end faces go; each face is closed between its two cuts.
    let faces = body.shells[0].faces.clone();
    let mut end_face = [usize::MAX; 3];
    for (fi, face) in faces.iter().enumerate() {
        let Some(n) = super::g2_prism::planar_normal(face) else { continue };
        let Some(on) = face.surface.control_points.first().and_then(|row| row.first()).and_then(|c| c.point().ok()) else { continue };
        for k in 0..3 {
            let plane = t[(k + 1) % 3].cross(t[(k + 2) % 3]).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let at = corner.add(t[k].scale(setback));
            if n.cross(plane).length() <= 1e-9 && plane.dot(on.sub(at)).abs() <= 1e-9 * (1.0 + setback + at.length()) {
                if end_face[k] != usize::MAX {
                    return Err(refuse("setback_g2_notch", "two faces lie on one end plane".into()));
                }
                end_face[k] = fi;
            }
        }
    }
    if end_face.contains(&usize::MAX) {
        return Err(refuse("setback_g2_notch", "an end face of the cut-back corner is missing".into()));
    }
    let holders = |edge_id: u64| -> Vec<usize> {
        let mut out = Vec::new();
        for (fi, f) in faces.iter().enumerate() {
            if f.loops.iter().any(|l| l.coedges.iter().any(|c| c.edge_id == edge_id)) {
                out.push(fi);
            }
        }
        out
    };
    let mut sections = [(0u64, 0usize); 3]; // (edge, strip face)
    let mut segments: Vec<(u64, usize)> = Vec::new(); // (edge, face)
    let mut inner: Vec<u64> = Vec::new();
    for k in 0..3 {
        let face = &faces[end_face[k]];
        if face.loops.len() != 1 {
            return Err(refuse("setback_g2_notch", "an end face has a hole".into()));
        }
        let mut found = 0;
        for c in &face.loops[0].coedges {
            let others: Vec<usize> = holders(c.edge_id).into_iter().filter(|&f| f != end_face[k]).collect();
            if others.len() != 1 {
                return Err(refuse("setback_g2_notch", "an end face's edge is not shared with one other face".into()));
            }
            let other = others[0];
            if end_face.contains(&other) {
                if !inner.contains(&c.edge_id) {
                    inner.push(c.edge_id);
                }
            } else if super::g2_prism::planar_normal(&faces[other]).is_some() {
                segments.push((c.edge_id, other));
            } else {
                sections[k] = (c.edge_id, other);
                found += 1;
            }
        }
        if found != 1 {
            return Err(refuse("setback_g2_notch", format!("an end face meets {found} strips, not one")));
        }
    }
    if segments.len() != 6 || inner.len() != 3 {
        return Err(refuse("setback_g2_notch", format!("the cut-back corner has {} edges on its faces and {} between its end faces, not 6 and 3", segments.len(), inner.len())));
    }
    let mut result = body.clone();
    let mut next = next_id_after(&result);
    let mut id = || {
        next += 1;
        next
    };
    let edge_now = |result: &BrepSolid, x: u64| result.edges.iter().find(|e| e.id == x).cloned();
    let walked = |edge: &EdgeRecord, forward: bool| if forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) };
    let point_of = |result: &BrepSolid, v: u64| result.vertices.iter().find(|x| x.id == v).map(|x| x.point);
    // (face index, curve edge, its plane's normal)
    let mut face_curves: Vec<(usize, u64, Vec3)> = Vec::new();
    let segment_ids: Vec<u64> = segments.iter().map(|s| s.0).collect();
    let mut closed_faces: Vec<usize> = segments.iter().map(|s| s.1).collect();
    closed_faces.sort_unstable();
    closed_faces.dedup();
    if closed_faces.len() != 3 {
        return Err(refuse("setback_g2_faces", format!("{} faces meet the cut-back corner, not three", closed_faces.len())));
    }
    for &fi in &closed_faces {
        let face = &faces[fi];
        let normal = super::g2_prism::planar_normal(face).expect("planar");
        let li = face.loops.iter().position(|l| l.coedges.iter().any(|c| segment_ids.contains(&c.edge_id))).expect("loop");
        let ring = &face.loops[li].coedges;
        let n = ring.len();
        let at = (0..n)
            .find(|&ci| segment_ids.contains(&ring[ci].edge_id) && segment_ids.contains(&ring[(ci + 1) % n].edge_id))
            .ok_or_else(|| refuse("setback_g2_faces", "a face's two cut edges are not consecutive".into()))?;
        let (first, second) = (&ring[at], &ring[(at + 1) % n]);
        let (before, after) = (&ring[(at + n - 1) % n], &ring[(at + 2) % n]);
        let (rail1, rail2) = (edge_now(&result, before.edge_id).expect("rail"), edge_now(&result, after.edge_id).expect("rail"));
        if !is_line(&rail1) || !is_line(&rail2) {
            return Err(refuse("setback_g2_faces", "a strip's contact line is not straight".into()));
        }
        let (far1, cut1) = walked(&rail1, before.forward);
        let (cut2, far2) = walked(&rail2, after.forward);
        let (v_first, _) = walked(&edge_now(&result, first.edge_id).expect("segment"), first.forward);
        let (_, v_second) = walked(&edge_now(&result, second.edge_id).expect("segment"), second.forward);
        if v_first != cut1 || v_second != cut2 {
            return Err(refuse("setback_g2_faces", "a face's cut edges do not run between its contact lines".into()));
        }
        let (r1, r2) = (point_of(&result, cut1).expect("cut"), point_of(&result, cut2).expect("cut"));
        let d1 = r1.sub(point_of(&result, far1).expect("far")).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let d2 = r2.sub(point_of(&result, far2).expect("far")).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        // The contact lines' crossing: r1 + λ d1 = r2 + μ d2.
        let gap = r2.sub(r1);
        let (a11, a12, a22) = (d1.dot(d1), -d1.dot(d2), d2.dot(d2));
        let (b1, b2) = (d1.dot(gap), -d2.dot(gap));
        let det2 = a11 * a22 - a12 * a12;
        if det2.abs() < 1e-12 {
            return Err(refuse("setback_g2_faces", "a face's two contact lines are parallel".into()));
        }
        let lambda = (b1 * a22 - a12 * b2) / det2;
        let mu = (a11 * b2 - a12 * b1) / det2;
        let crossing = r1.add(d1.scale(lambda));
        if !(lambda > 0.0 && mu > 0.0) || crossing.sub(r2.add(d2.scale(mu))).length() > 1e-7 * (1.0 + setback) {
            return Err(refuse("setback_g2_faces", "a face's contact lines do not cross toward the corner".into()));
        }
        let controls = [r1, r1.add(crossing).scale(0.5), crossing, r2.add(crossing).scale(0.5), r2];
        let quartic = NurbsCurve::new(4, vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0], controls.iter().map(|&p| Vec4::from_point(p, 1.0)).collect()).or_refuse(KernelStage::Refine, "face_curve")?;
        let pcurve = super::g2_prism::mapped_on_plane(face, &quartic).ok_or_else(|| refuse("setback_g2_faces", "a face curve could not be put on its plane".into()))?;
        let e = id();
        result.edges.push(EdgeRecord { id: e, curve: quartic, t0: 0.0, t1: 1.0, start_vertex_id: cut1, end_vertex_id: cut2, degenerate: false, name: None });
        let coedge = CoedgeRecord { id: id(), edge_id: e, forward: true, pcurve };
        let mut coedges = Vec::with_capacity(n - 1);
        for (ci, c) in ring.iter().enumerate() {
            if ci == at {
                coedges.push(coedge.clone());
            } else if ci != (at + 1) % n {
                coedges.push(c.clone());
            }
        }
        result.shells[0].faces[fi].loops[li].coedges = coedges;
        face_curves.push((fi, e, normal));
    }
    let gone_faces: Vec<u64> = end_face.iter().map(|&fi| faces[fi].id).collect();
    result.shells[0].faces.retain(|f| !gone_faces.contains(&f.id));
    result.edges.retain(|e| !segment_ids.contains(&e.id) && !inner.contains(&e.id));
    let used: Vec<u64> = result.edges.iter().flat_map(|e| [e.start_vertex_id, e.end_vertex_id]).collect();
    result.vertices.retain(|v| used.contains(&v.id));

    // 3. The hole's sides, walked against their neighbours, chained.
    let mut pieces: Vec<Side> = Vec::with_capacity(6);
    for k in 0..3 {
        let (edge_id, strip) = sections[k];
        let strip_face = &faces[strip];
        let c = strip_face.loops.iter().flat_map(|l| l.coedges.iter()).find(|c| c.edge_id == edge_id).expect("section use");
        let edge = edge_now(&result, edge_id).expect("section");
        pieces.push(Side::new(edge, !c.forward, Kind::Strip { toward: t[k].scale(-1.0) })?);
    }
    for &(_, e, normal) in &face_curves {
        let edge = edge_now(&result, e).expect("face curve");
        pieces.push(Side::new(edge, false, Kind::Face { normal })?);
    }
    let mut chain = vec![pieces.remove(0)];
    while !pieces.is_empty() {
        let to = chain.last().expect("side").to;
        let at = pieces.iter().position(|p| p.from == to).ok_or_else(|| refuse("setback_g2_hole", "the corner's six sides do not close".into()))?;
        chain.push(pieces.remove(at));
    }
    if chain[5].to != chain[0].from || (0..6).any(|k| matches!(chain[k].kind, Kind::Strip { .. }) != (k % 2 == 0)) {
        return Err(refuse("setback_g2_hole", "the corner's sides do not alternate between strips and faces round a closed loop".into()));
    }
    let sides: [Side; 6] = chain.try_into().map_err(|_| refuse("setback_g2_hole", "the corner does not have six sides".into()))?;
    let patch = CornerPatch::new(sides)?;
    let fitted = patch.fit(corner, &mut id, name)?;
    let curvature_scale = patch.curvature_scale()?;
    fitted.check(&patch, &result, curvature_scale)?;
    fitted.contained(corner, t, setback)?;
    result.vertices.push(fitted.centre);
    result.edges.extend(fitted.seams.iter().cloned());
    result.shells[0].faces.extend(fitted.faces.iter().cloned());
    result.mass_properties_cache = Default::default();
    super::edges::refuse_invalid_result(&result, ENTRY)?;
    Ok(result)
}

#[derive(Clone, Copy, Debug)]
enum Kind {
    /// A strip's section; `toward`: the unit direction along its edge toward the corner.
    Strip { toward: Vec3 },
    /// A face curve; `normal`: its plane's unit normal.
    Face { normal: Vec3 },
}

/// One side of the hole, as the patch walks it: `B(s) = curve(τ(s))`, `s` in
/// `[0, 1]` from `from` to `to`.
#[derive(Clone, Debug)]
struct Side {
    edge: EdgeRecord,
    forward: bool,
    from: u64,
    to: u64,
    kind: Kind,
}

impl Side {
    fn new(edge: EdgeRecord, forward: bool, kind: Kind) -> Result<Side, KernelRefusal> {
        let (from, to) = if forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) };
        Ok(Side { edge, forward, from, to, kind })
    }

    fn tau(&self, s: f64) -> f64 {
        let (t0, t1) = (self.edge.t0, self.edge.t1);
        if self.forward { t0 + s * (t1 - t0) } else { t1 - s * (t1 - t0) }
    }

    fn tau_rate(&self) -> f64 {
        if self.forward { self.edge.t1 - self.edge.t0 } else { self.edge.t0 - self.edge.t1 }
    }

    /// `B`, `B'` and `B''` (with respect to `s`).
    fn jet(&self, s: f64) -> Result<[Vec3; 3], KernelRefusal> {
        let d = self.edge.curve.derivatives(self.tau(s), 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let r = self.tau_rate();
        Ok([d[0], d[1].scale(r), d[2].scale(r * r)])
    }

    fn at(&self, s: f64) -> Result<Vec3, KernelRefusal> {
        self.edge.curve.evaluate(self.tau(s.clamp(0.0, 1.0))).or_refuse(KernelStage::Refine, "evaluate")
    }
}

/// `φ(d)`: `d` to second order at 0, saturating at [`REACH`].
fn phi(d: f64) -> f64 {
    let q = d / REACH;
    d / (1.0 + q * q * q.abs()).cbrt()
}

/// The regular hexagon the patch is built over (circumradius 1, side 1,
/// inradius √3/2), counter-clockwise.
struct Hexagon {
    corners: [[f64; 2]; 6],
    normals: [[f64; 2]; 6],
}

const INRADIUS: f64 = 0.866_025_403_784_438_6;

impl Hexagon {
    fn new() -> Hexagon {
        let mut corners = [[0.0; 2]; 6];
        for (k, c) in corners.iter_mut().enumerate() {
            let a = std::f64::consts::FRAC_PI_2 + k as f64 * std::f64::consts::FRAC_PI_3;
            *c = [a.cos(), a.sin()];
        }
        let mut normals = [[0.0; 2]; 6];
        for k in 0..6 {
            let (p, q) = (corners[k], corners[(k + 1) % 6]);
            let m = [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])];
            let l = m[0].hypot(m[1]);
            normals[k] = [-m[0] / l, -m[1] / l];
        }
        Hexagon { corners, normals }
    }

    /// `d_k(p)`: 0 on side `k`, 1 at the centre.
    fn d(&self, k: usize, p: [f64; 2]) -> f64 {
        let (v, n) = (self.corners[k], self.normals[k]);
        ((p[0] - v[0]) * n[0] + (p[1] - v[1]) * n[1]) / INRADIUS
    }

    fn d_grad(&self, k: usize) -> [f64; 2] {
        [self.normals[k][0] / INRADIUS, self.normals[k][1] / INRADIUS]
    }

    /// `s_k(p)`, its gradient and its Hessian.
    fn s_jet(&self, k: usize, p: [f64; 2]) -> (f64, [f64; 2], [[f64; 2]; 2]) {
        let (before, after) = ((k + 5) % 6, (k + 1) % 6);
        let (a, b) = (self.d(before, p), self.d(after, p));
        let (ga, gb) = (self.d_grad(before), self.d_grad(after));
        let dd = a + b;
        let gd = [ga[0] + gb[0], ga[1] + gb[1]];
        let s = a / dd;
        let g = [ga[0] / dd - a * gd[0] / (dd * dd), ga[1] / dd - a * gd[1] / (dd * dd)];
        let mut h = [[0.0; 2]; 2];
        for i in 0..2 {
            for j in 0..2 {
                h[i][j] = -(ga[i] * gd[j] + ga[j] * gd[i]) / (dd * dd) + 2.0 * a * gd[i] * gd[j] / (dd * dd * dd);
            }
        }
        (s, g, h)
    }

    fn s(&self, k: usize, p: [f64; 2]) -> f64 {
        let (a, b) = (self.d((k + 5) % 6, p), self.d((k + 1) % 6, p));
        (a / (a + b)).clamp(0.0, 1.0)
    }
}

fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn quad2(h: [[f64; 2]; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * (h[0][0] * b[0] + h[0][1] * b[1]) + a[1] * (h[1][0] * b[0] + h[1][1] * b[1])
}

fn in_plane(v: Vec3, n: Vec3) -> Vec3 {
    v.sub(n.scale(v.dot(n)))
}

/// The transfinite patch over the hexagon.
struct CornerPatch {
    hex: Hexagon,
    sides: [Side; 6],
    /// Per strip side: `λ` at `s = 0` and `s = 1`.
    lambda: [[f64; 2]; 6],
    /// Per face side: `M(0), M'(0), M(1), M'(1)` and `N(0), N(1)`.
    m: [[Vec3; 4]; 6],
    n: [[Vec3; 2]; 6],
}

impl CornerPatch {
    fn new(sides: [Side; 6]) -> Result<CornerPatch, KernelRefusal> {
        let hex = Hexagon::new();
        let zero = Vec3::new(0.0, 0.0, 0.0);
        let mut patch = CornerPatch { hex, sides, lambda: [[0.0; 2]; 6], m: [[zero; 4]; 6], n: [[zero; 2]; 6] };
        // λ at each end of a strip: the neighbouring face curve's speed along
        // the shared side's direction, so the two agree along it at the corner.
        for f in (0..6).step_by(2) {
            let Kind::Strip { toward } = patch.sides[f].kind else { unreachable!() };
            for (slot, (l, s_l, corner)) in [((f + 5) % 6, 1.0, f), ((f + 1) % 6, 0.0, (f + 1) % 6)].into_iter().enumerate() {
                let v = patch.hex.corners[corner];
                let x = Self::along(&patch.hex, l);
                let (_, gs, _) = patch.hex.s_jet(l, v);
                let [_, b1, _] = patch.sides[l].jet(s_l)?;
                let gd = patch.hex.d_grad(f);
                let rate = dot2(gd, x);
                if rate.abs() < 1e-12 {
                    return Err(refuse("setback_g2_patch", "a strip side's distance does not change along its neighbour".into()));
                }
                let speed = b1.dot(toward) * dot2(gs, x) / rate;
                if !(speed > 0.0) {
                    return Err(refuse("setback_g2_patch", "a face curve does not leave its cut toward the corner".into()));
                }
                patch.lambda[f][slot] = speed;
            }
        }
        // Each face ribbon's M, M' and N at both ends, from the strip's jet.
        for l in (1..6).step_by(2) {
            let Kind::Face { normal } = patch.sides[l].kind else { unreachable!() };
            for (end, (f, corner, s_l)) in [((l + 5) % 6, l, 0.0), ((l + 1) % 6, (l + 1) % 6, 1.0)].into_iter().enumerate() {
                let v = patch.hex.corners[corner];
                let x = Self::along(&patch.hex, l);
                let y = patch.hex.normals[l];
                let kappa = dot2(patch.hex.d_grad(l), y);
                let (_, gs, hs) = patch.hex.s_jet(l, v);
                let [_, b1, b2] = patch.sides[l].jet(s_l)?;
                let target = |u: [f64; 2], w: Option<[f64; 2]>| patch.strip_jet(f, v, u, w);
                let (ty, txy, tyy) = (target(y, None)?, target(x, Some(y))?, target(y, Some(y))?);
                let (sx, sy) = (dot2(gs, x), dot2(gs, y));
                let m = ty.sub(b1.scale(sy)).scale(1.0 / kappa);
                let m1 = txy.sub(b2.scale(sx * sy)).sub(b1.scale(quad2(hs, x, y))).scale(1.0 / (kappa * sx));
                let nn = tyy.sub(b2.scale(sy * sy)).sub(b1.scale(quad2(hs, y, y))).sub(m1.scale(2.0 * kappa * sy)).scale(1.0 / (2.0 * kappa * kappa));
                patch.m[l][2 * end] = in_plane(m, normal);
                patch.m[l][2 * end + 1] = in_plane(m1, normal);
                patch.n[l][end] = in_plane(nn, normal);
            }
        }
        Ok(patch)
    }

    /// The unit direction of side `k` in the domain, from its start.
    fn along(hex: &Hexagon, k: usize) -> [f64; 2] {
        let (p, q) = (hex.corners[k], hex.corners[(k + 1) % 6]);
        let l = (q[0] - p[0]).hypot(q[1] - p[1]);
        [(q[0] - p[0]) / l, (q[1] - p[1]) / l]
    }

    /// The first (`w` = None) or second derivative of strip `f`'s ribbon,
    /// composed with `s_f` and `d_f`, at the domain corner `v` (where `d_f` = 0).
    fn strip_jet(&self, f: usize, v: [f64; 2], u: [f64; 2], w: Option<[f64; 2]>) -> Result<Vec3, KernelRefusal> {
        let Kind::Strip { toward } = self.sides[f].kind else { unreachable!() };
        let (s, gs, hs) = self.hex.s_jet(f, v);
        let gd = self.hex.d_grad(f);
        let [_, b1, b2] = self.sides[f].jet(s.clamp(0.0, 1.0))?;
        let [l0, l1] = self.lambda[f];
        let lambda = l0 + (l1 - l0) * s;
        let dl = l1 - l0;
        Ok(match w {
            None => b1.scale(dot2(gs, u)).add(toward.scale(lambda * dot2(gd, u))),
            Some(w) => b2
                .scale(dot2(gs, u) * dot2(gs, w))
                .add(b1.scale(quad2(hs, u, w)))
                .add(toward.scale(dl * (dot2(gs, u) * dot2(gd, w) + dot2(gs, w) * dot2(gd, u)))),
        })
    }

    /// Ribbon `k` at `(s, d)`.
    fn ribbon(&self, k: usize, s: f64, d: f64) -> Result<Vec3, KernelRefusal> {
        let s = s.clamp(0.0, 1.0);
        let base = self.sides[k].at(s)?;
        if d == 0.0 {
            return Ok(base);
        }
        let p = phi(d);
        Ok(match self.sides[k].kind {
            Kind::Strip { toward } => {
                let [l0, l1] = self.lambda[k];
                base.add(toward.scale((l0 + (l1 - l0) * s) * p))
            }
            Kind::Face { .. } => {
                let [m0, m0d, m1, m1d] = self.m[k];
                let (h00, h10, h01, h11) = (2.0 * s * s * s - 3.0 * s * s + 1.0, s * s * s - 2.0 * s * s + s, -2.0 * s * s * s + 3.0 * s * s, s * s * s - s * s);
                let m = m0.scale(h00).add(m0d.scale(h10)).add(m1.scale(h01)).add(m1d.scale(h11));
                let n = self.n[k][0].scale(1.0 - s).add(self.n[k][1].scale(s));
                base.add(m.scale(p)).add(n.scale(p * p))
            }
        })
    }

    /// The patch at the domain point `p`.
    fn eval(&self, p: [f64; 2]) -> Result<Vec3, KernelRefusal> {
        let d: [f64; 6] = std::array::from_fn(|k| self.hex.d(k, p).max(0.0));
        let (low, least) = d.iter().copied().enumerate().fold((0, f64::INFINITY), |best, (k, x)| if x < best.1 { (k, x) } else { best });
        if least <= 1e-13 {
            return self.ribbon(low, self.hex.s(low, p), 0.0);
        }
        let inv: [f64; 6] = std::array::from_fn(|k| 1.0 / (d[k] * d[k] * d[k]));
        let total: f64 = inv.iter().sum();
        let mut out = Vec3::new(0.0, 0.0, 0.0);
        for k in 0..6 {
            out = out.add(self.ribbon(k, self.hex.s(k, p), d[k])?.scale(inv[k] / total));
        }
        Ok(out)
    }

    /// Rhombus `m`'s domain point at `(u, v)`: `(0,0)` = corner `2m+1`, `(1,0)` =
    /// corner `2m+2`, `(1,1)` = the centre, `(0,1)` = corner `2m`.
    fn rhombus(&self, m: usize, u: f64, v: f64) -> [f64; 2] {
        let o = self.hex.corners[(2 * m + 1) % 6];
        let a = self.hex.corners[(2 * m + 2) % 6];
        let b = self.hex.corners[(2 * m) % 6];
        [o[0] + u * (a[0] - o[0]) + v * (b[0] - o[0]), o[1] + u * (a[1] - o[1]) + v * (b[1] - o[1])]
    }

    /// The largest curvature of the strips along their sections, the scale the
    /// curvature bar is read against.
    fn curvature_scale(&self) -> Result<f64, KernelRefusal> {
        let mut most = 0.0f64;
        for k in (0..6).step_by(2) {
            for i in 0..=16 {
                let [_, b1, b2] = self.sides[k].jet(i as f64 / 16.0)?;
                let speed = b1.length();
                if speed > 0.0 {
                    most = most.max(b1.cross(b2).length() / (speed * speed * speed));
                }
            }
        }
        Ok(most)
    }

    fn fit(&self, corner: Vec3, id: &mut dyn FnMut() -> u64, name: Option<&str>) -> Result<Fitted, KernelRefusal> {
        // Clustered toward both ends (Chebyshev-Lobatto): the patch bends fastest at the hexagon's corners.
        let n = GRID;
        let params: Vec<f64> = (0..n)
            .map(|i| 0.5 - 0.5 * (std::f64::consts::PI * i as f64 / (n - 1) as f64).cos())
            .map(|t| if t < 1e-15 { 0.0 } else if t > 1.0 - 1e-15 { 1.0 } else { t })
            .collect();
        // The seam from corner 2m to the centre, shared by rhombus m (its top
        // row) and rhombus m-1 (its right column), sampled once.
        let mut seam_samples: Vec<Vec<Vec3>> = Vec::with_capacity(3);
        let centre_point = self.eval([0.0, 0.0])?;
        for m in 0..3 {
            let v = self.hex.corners[2 * m];
            let mut row = Vec::with_capacity(GRID);
            for (i, &t) in params.iter().enumerate() {
                row.push(if i + 1 == GRID { centre_point } else { self.eval([v[0] * (1.0 - t), v[1] * (1.0 - t)])? });
            }
            seam_samples.push(row);
        }
        let mut surfaces = Vec::with_capacity(3);
        for m in 0..3 {
            let g = GRID;
            let mut samples = vec![vec![Vec3::new(0.0, 0.0, 0.0); g]; g]; // [u][v]
            for i in 0..GRID {
                for j in 0..GRID {
                    samples[i][j] = if j + 1 == g {
                        seam_samples[m][i]
                    } else if i + 1 == g {
                        seam_samples[(m + 1) % 3][j]
                    } else {
                        self.eval(self.rhombus(m, params[i], params[j]))?
                    };
                }
            }
            surfaces.push(interpolate_grid(&samples, &params)?);
        }
        // Stamp each shared seam with one set of controls: rhombus m's right
        // column is rhombus m+1's top row.
        for m in 0..3 {
            let next = (m + 1) % 3;
            let column: Vec<Vec4> = surfaces[m].control_points[GRID - 1].clone();
            for (i, c) in column.into_iter().enumerate() {
                let len = surfaces[next].control_points[i].len();
                surfaces[next].control_points[i][len - 1] = c;
            }
        }
        // Vertices and seams.
        let centre = VertexRecord { id: id(), point: centre_point };
        let mut seams = Vec::with_capacity(3);
        let mut seam_ids = [0u64; 3];
        for m in 0..3 {
            // Seam at corner 2m: rhombus (m+2)%3's right column, corner 2m -> centre.
            let owner = (m + 2) % 3;
            let curve = surfaces[owner].iso_curve_u(1.0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
            let [a, b] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let e = id();
            seams.push(EdgeRecord { id: e, curve, t0: a, t1: b, start_vertex_id: self.sides[2 * m].from, end_vertex_id: centre.id, degenerate: false, name: None });
            seam_ids[m] = e;
        }
        let mut faces = Vec::with_capacity(3);
        for m in 0..3 {
            let (bottom, left) = (&self.sides[2 * m + 1], &self.sides[2 * m]);
            let side_pcurve = |side: &Side, a: [f64; 2], b: [f64; 2]| -> Result<NurbsCurve, KernelRefusal> {
                // Walked from a to b as s runs 0 -> 1; over the edge's range.
                let (t0, t1) = (side.edge.t0, side.edge.t1);
                if side.forward { uv_line(a, b, t0, t1) } else { uv_line(b, a, t0, t1)?.reversed().or_refuse(KernelStage::Refine, "reversed") }
            };
            let right_seam = seam_ids[(m + 1) % 3];
            let top_seam = seam_ids[m];
            let coedges = vec![
                CoedgeRecord { id: id(), edge_id: bottom.edge.id, forward: bottom.forward, pcurve: side_pcurve(bottom, [0.0, 0.0], [1.0, 0.0])? },
                CoedgeRecord { id: id(), edge_id: right_seam, forward: true, pcurve: uv_line([1.0, 0.0], [1.0, 1.0], 0.0, 1.0)? },
                CoedgeRecord { id: id(), edge_id: top_seam, forward: false, pcurve: uv_line([0.0, 1.0], [1.0, 1.0], 0.0, 1.0)?.reversed().or_refuse(KernelStage::Refine, "reversed")? },
                CoedgeRecord { id: id(), edge_id: left.edge.id, forward: left.forward, pcurve: side_pcurve(left, [0.0, 1.0], [0.0, 0.0])? },
            ];
            let surface = surfaces[m].clone();
            let middle = surface.evaluate(0.5, 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
            let normal = surface.normal(0.5, 0.5).or_refuse(KernelStage::Refine, "normal")?;
            if normal.dot(corner.sub(middle)) <= 0.0 {
                return Err(KernelRefusal::internal(KernelStage::Refine, "setback_g2_sense", format!("{ENTRY}: a corner patch faces into the body")));
            }
            let loop_id = id();
            faces.push(FaceRecord { id: id(), surface, same_sense: true, loops: vec![LoopRecord { id: loop_id, coedges }], name: name.map(|value| format!("{value}[{m}]")) });
        }
        Ok(Fitted { centre, seams, faces, surfaces })
    }
}

/// The tensor-product cubic interpolant of `grid[u][v]` at `params` both ways.
fn interpolate_grid(grid: &[Vec<Vec3>], params: &[f64]) -> Result<NurbsSurface, KernelRefusal> {
    let n = params.len();
    // Along v for each u, then along u for each v-control.
    let mut columns: Vec<NurbsCurve> = Vec::with_capacity(n);
    for row in grid {
        columns.push(crate::fit::interpolate_curve(row, 3, params).or_refuse(KernelStage::Refine, "patch_fit")?);
    }
    let knots_v = columns[0].knots.clone();
    let count_v = columns[0].control_points.len();
    if count_v != n || columns.iter().any(|c| c.knots != knots_v || c.control_points.len() != n) {
        return Err(refuse("setback_g2_fit", "the corner patch's rows were not interpolated alike".into()));
    }
    let mut knots_u = Vec::new();
    let mut by_v: Vec<Vec<Vec4>> = Vec::with_capacity(count_v);
    for j in 0..count_v {
        let points: Vec<Vec3> = columns.iter().map(|c| c.control_points[j].point()).collect::<Result<_, _>>().or_refuse(KernelStage::Refine, "point")?;
        let across = crate::fit::interpolate_curve(&points, 3, params).or_refuse(KernelStage::Refine, "patch_fit")?;
        if across.control_points.len() != n {
            return Err(refuse("setback_g2_fit", "the corner patch's columns were not interpolated alike".into()));
        }
        knots_u = across.knots.clone();
        by_v.push(across.control_points.clone());
    }
    let net: Vec<Vec<Vec4>> = (0..n).map(|i| (0..count_v).map(|j| by_v[j][i]).collect()).collect();
    NurbsSurface::new(3, 3, knots_u, knots_v, net).or_refuse(KernelStage::Refine, "patch_surface")
}

struct Fitted {
    centre: VertexRecord,
    seams: Vec<EdgeRecord>,
    faces: Vec<FaceRecord>,
    surfaces: Vec<NurbsSurface>,
}

/// A surface's unit normal and its normal curvature along `direction` at `(u, v)`.
fn normal_and_bend(surface: &NurbsSurface, u: f64, v: f64, direction: Vec3) -> Result<(Vec3, f64), KernelRefusal> {
    let d = surface.derivatives(u, v, 2).or_refuse(KernelStage::Refine, "derivatives")?;
    let (su, sv) = (d[1][0], d[0][1]);
    let normal = su.cross(sv).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    // The direction in (u, v): solve su a + sv b = direction (least squares).
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let (p, q) = (su.dot(direction), sv.dot(direction));
    let det = e * g - f * f;
    let (a, b) = ((p * g - f * q) / det, (e * q - f * p) / det);
    let tangent = su.scale(a).add(sv.scale(b));
    let second = d[2][0].scale(a * a).add(d[1][1].scale(2.0 * a * b)).add(d[0][2].scale(b * b));
    let bend = second.dot(normal) / tangent.dot(tangent);
    Ok((normal, bend))
}

impl Fitted {
    /// Every point of the patch within the corner's parallelepiped: at depths
    /// 0 to `setback` along the three edges (within a millionth of it).
    fn contained(&self, corner: Vec3, t: [Vec3; 3], setback: f64) -> Result<(), KernelRefusal> {
        let det = t[0].dot(t[1].cross(t[2]));
        let slack = 1e-6 * setback;
        let samples = 32;
        for surface in &self.surfaces {
            for i in 0..=samples {
                for j in 0..=samples {
                    let p = surface.evaluate(i as f64 / samples as f64, j as f64 / samples as f64).or_refuse(KernelStage::Refine, "evaluate")?.sub(corner);
                    // p = Σ a_k t_k (Cramer).
                    let a = [p.dot(t[1].cross(t[2])) / det, t[0].dot(p.cross(t[2])) / det, t[0].dot(t[1].cross(p)) / det];
                    if a.iter().any(|&x| x < -slack || x > setback + slack) {
                        return Err(refuse("setback_g2_reach", format!("the corner patch leaves the corner's cut-back region (depths {a:?} against 0..{setback}): this construction does not close this corner")));
                    }
                }
            }
        }
        Ok(())
    }

    /// The fit against the exact sides and the transfinite patch, and the
    /// joins across the sides and the seams.
    fn check(&self, patch: &CornerPatch, result: &BrepSolid, curvature_scale: f64) -> Result<(), KernelRefusal> {
        let mut points = Vec::new();
        for side in &patch.sides {
            for i in 0..=4 {
                points.push(side.at(i as f64 / 4.0)?);
            }
        }
        let size = crate::model_scale(points);
        let mut deviation = 0.0f64;
        let mut angle = 0.0f64;
        let mut bend_gap = 0.0f64;
        let samples = 24;
        for m in 0..3 {
            let surface = &self.surfaces[m];
            // Inside: between the fit's samples.
            for i in 0..2 * (GRID - 1) {
                for j in [GRID / 3, GRID / 2, 2 * GRID / 3] {
                    let (u, v) = ((i as f64 + 0.5) / (2 * (GRID - 1)) as f64, (j as f64 + 0.5) / (GRID - 1) as f64);
                    let exact = patch.eval(patch.rhombus(m, u, v))?;
                    deviation = deviation.max(surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(exact).length());
                    let exact = patch.eval(patch.rhombus(m, v, u))?;
                    deviation = deviation.max(surface.evaluate(v, u).or_refuse(KernelStage::Refine, "evaluate")?.sub(exact).length());
                }
            }
            // Along its two sides: position, normal and the curvature across.
            for (k, on) in [(2 * m + 1, true), (2 * m, false)] {
                let side = &patch.sides[k];
                let neighbour = result
                    .shells[0]
                    .faces
                    .iter()
                    .find(|f| !self.faces.iter().any(|g| g.id == f.id) && f.loops.iter().any(|l| l.coedges.iter().any(|c| c.edge_id == side.edge.id)))
                    .ok_or_else(|| refuse("setback_g2_patch", "a side's neighbour is missing".into()))?;
                for i in 1..samples {
                    let s = i as f64 / samples as f64;
                    let (u, v) = if on { (s, 0.0) } else { (0.0, 1.0 - s) };
                    let exact = side.at(s)?;
                    let here = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
                    deviation = deviation.max(here.sub(exact).length());
                    let [_, b1, _] = side.jet(s)?;
                    let foot = crate::project_point_to_surface(&neighbour.surface, exact).or_refuse(KernelStage::Refine, "project")?;
                    let dn = neighbour.surface.derivatives(foot.u, foot.v, 1).or_refuse(KernelStage::Refine, "derivatives")?;
                    let theirs = dn[1][0].cross(dn[0][1]).normalized().or_refuse(KernelStage::Refine, "normalized")?;
                    let (ours, _) = normal_and_bend(surface, u, v, b1)?;
                    let across = ours.cross(b1).normalized().or_refuse(KernelStage::Refine, "normalized")?;
                    let (ours, bend_ours) = normal_and_bend(surface, u, v, across)?;
                    let sign = if ours.dot(theirs) < 0.0 { -1.0 } else { 1.0 };
                    angle = angle.max(ours.cross(theirs).length().atan2(ours.dot(theirs).abs()));
                    let bend_theirs = match side.kind {
                        Kind::Face { .. } => 0.0,
                        Kind::Strip { .. } => {
                            let (_, b) = normal_and_bend(&neighbour.surface, foot.u, foot.v, across)?;
                            sign * b
                        }
                    };
                    bend_gap = bend_gap.max((bend_ours - bend_theirs).abs());
                }
            }
            // Across its right seam, against the next rhombus's top row.
            let next = &self.surfaces[(m + 1) % 3];
            for i in 1..samples {
                let s = i as f64 / samples as f64;
                let (a, _) = normal_and_bend(surface, 1.0, s, Vec3::new(1.0, 0.0, 0.0))?;
                let (b, _) = normal_and_bend(next, s, 1.0, Vec3::new(1.0, 0.0, 0.0))?;
                angle = angle.max(a.cross(b).length().atan2(a.dot(b)));
                let along = surface.derivatives(1.0, s, 1).or_refuse(KernelStage::Refine, "derivatives")?[0][1];
                let across = a.cross(along).normalized().or_refuse(KernelStage::Refine, "normalized")?;
                let (_, ka) = normal_and_bend(surface, 1.0, s, across)?;
                let (_, kb) = normal_and_bend(next, s, 1.0, across)?;
                bend_gap = bend_gap.max((ka - kb).abs());
            }
        }
        if std::env::var("BREP_DEBUG_SETBACK_G2").is_ok() {
            eprintln!("setback g2 corner: fit deviation {deviation:.3e} (size {size:.3e}), worst normal angle {angle:.3e}, worst curvature gap {bend_gap:.3e} (scale {curvature_scale:.3e})");
        }
        if !(deviation <= BOUNDARY_BAR * size) {
            return Err(refuse("setback_g2_fit", format!("the corner patch's fit strays {deviation:.3e} from its sides and its construction (bar {:.3e})", BOUNDARY_BAR * size)));
        }
        if !(angle <= ANGLE_BAR) {
            return Err(refuse("setback_g2_tangent", format!("the corner patch meets its neighbours {angle:.3e} rad out of their tangent planes (bar {ANGLE_BAR:.1e})")));
        }
        if !(bend_gap <= CURVATURE_BAR * curvature_scale) {
            return Err(refuse("setback_g2_curvature", format!("the corner patch's curvature jumps by {bend_gap:.3e} across a side or seam (bar {:.3e})", CURVATURE_BAR * curvature_scale)));
        }
        let regular = self.surfaces.iter().all(|s| crate::certify_regular(s).is_ok());
        if !regular {
            return Err(refuse("setback_g2_fold", "the corner patch folds".into()));
        }
        Ok(())
    }
}

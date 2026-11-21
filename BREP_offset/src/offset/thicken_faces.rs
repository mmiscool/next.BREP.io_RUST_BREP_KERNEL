//! THICKEN a sheet of several faces into one solid.
//!
//! Each face is thickened by itself ([`crate::thicken_trimmed_sheet`]: the
//! face, its offset and the walls ruled between their edges), and the slabs are
//! united. Where two faces meet along a crease the slabs either overlap (the
//! crease closes toward the side thickened, and the union trims each moved face
//! where it meets the other) or leave a gap (the crease opens toward that
//! side). A gap is filled as the caller asks:
//!
//! - [`OffsetCorners::Sharp`]: the moved faces are extended until they meet,
//!   a prism on the kite between the crease, its two moved images and the line
//!   the two moved planes share; where three such creases meet in a corner,
//!   the block out to the point the three moved planes share.
//! - [`OffsetCorners::Rounded`]: the gap is closed by the cylinder of the
//!   thickness about the crease, between the two moved faces; where three such
//!   creases meet in a corner, by the piece of the ball about the corner
//!   between the three.
//!
//! A whole patch of a round cylinder thickened toward its axis by its radius
//! or more is the wedge between it and its axis, with the like wedge opposite
//! for what is left over.
//!
//! A whole patch of a ring (an arc turned about an axis: a bend between a
//! floor and a wall of revolution) thickened toward its tube's centre circle by
//! the tube's radius or more is likewise the arc's sector turned about that
//! axis, with the like sector opposite for what is left over.
//!
//! Faces that meet smoothly along a crease (a bend and the flat it leaves)
//! need no fill; such a crease may be a closed circle. Where a crease closes toward the side thickened at a right
//! angle or wider, the slabs end on each other or overlap; tighter than a
//! right angle their square ends are cut off behind the other face. Fills are built where the crease is a straight line along
//! which both faces keep their normals; any other open crease is refused by
//! name, as is a corner where the open creases are not exactly three.
use crate::topology::FaceRecord;
use crate::{
    boolean_operation, boolean_operation_nary, extrude_profile_brep, AnalyticSurface, make_arc, make_line, make_sphere_brep_framed, project_point_to_surface,
    solid_from_polygons, thicken_trimmed_sheet, BooleanOperation, BooleanOptions, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OffsetCorners,
    OrRefuse, Vec3,
};

/// The prefix of every refusal of this operation.
pub const THICKEN_FACES_REFUSAL: &str = "thickenFaces";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{THICKEN_FACES_REFUSAL}: {message}"))
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{THICKEN_FACES_REFUSAL}: {message}"))
}

/// The sheet a chain of curves sweeps as it slides by `distance` along
/// `direction`: one face per curve (the curve times the slide), each a sheet
/// of its own, in the chain's order. Every face's normal is its curve's
/// direction crossed with the slide's, so the faces of a chain run head to
/// tail have their normals on one side of the sheet, as
/// [`thicken_sheet_faces`] needs them; run the chain the other way for the
/// other side.
pub fn extrude_curves_to_sheet(curves: &[NurbsCurve], direction: Vec3, distance: f64) -> Result<Vec<BrepSolid>, KernelRefusal> {
    if curves.is_empty() {
        return Err(input("curves", "at least one curve is required".into()));
    }
    if !distance.is_finite() || distance.abs() <= 1e-12 {
        return Err(input("distance", "distance must be a nonzero finite value".into()));
    }
    let slide = direction.normalized().map_err(|_| input("direction", "direction must be non-zero".into()))?.scale(distance);
    curves
        .iter()
        .map(|curve| {
            let net = curve
                .control_points
                .iter()
                .map(|pole| vec![*pole, crate::Vec4 { x: pole.x + slide.x * pole.w, y: pole.y + slide.y * pole.w, z: pole.z + slide.z * pole.w, w: pole.w }])
                .collect();
            let surface = crate::NurbsSurface::new(curve.degree, 1, curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], net).or_refuse(KernelStage::Collect, "thickenFaces.swept_face")?;
            crate::face_from_surface(&surface)
        })
        .collect()
}

/// One use of an edge by a face: where it runs, and the face's normal there.
struct Side {
    face: usize,
    start: Vec3,
    middle: Vec3,
    end: Vec3,
    /// The face's normal at the start, the middle and the end.
    normals: [Vec3; 3],
    /// The way this use runs at its middle (a closed edge has no chord to say).
    tangent: Vec3,
}

fn face_normal(face: &FaceRecord, point: Vec3) -> Result<Vec3, KernelRefusal> {
    let surface = &face.surface;
    let foot = project_point_to_surface(surface, point).or_refuse(KernelStage::Classify, "thickenFaces.project")?;
    let normal = match surface.normal(foot.u, foot.v) {
        Ok(normal) => normal,
        // At a pole (a side of the patch collapsed to a point) the surface has no
        // normal of its own: read it a little way in, where it has one.
        Err(_) => {
            let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Classify, "thickenFaces.domain")?;
            let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Classify, "thickenFaces.domain")?;
            let inward = |at: f64, low: f64, high: f64| at + (0.5 * (low + high) - at) * 1e-6;
            surface.normal(inward(foot.u, u0, u1), inward(foot.v, v0, v1)).or_refuse(KernelStage::Classify, "thickenFaces.normal")?
        }
    };
    Ok(if face.same_sense { normal } else { normal.scale(-1.0) })
}

/// The sheet `faces` thickened by `thickness` along the faces' normals
/// (negative: against them), its open creases closed as `corners` says.
///
/// `faces` are sheets of one face or more ([`crate::face_from_surface`],
/// [`crate::face_from_surface_trimmed`]) that meet along common edges, their
/// normals all on one side of the sheet. The result is one solid.
///
/// Refuses faces whose normals disagree across a crease, a crease that is not
/// a straight line with constant normals where it opens toward the thickened
/// side or closes tighter than a right angle, a corner of other than three
/// open creases, and whatever the single-face thicken or the union refuses.
pub fn thicken_sheet_faces(faces: &[BrepSolid], thickness: f64, corners: OffsetCorners) -> Result<BrepSolid, KernelRefusal> {
    if !thickness.is_finite() || thickness.abs() <= 1e-12 {
        return Err(input("thickness", "thickness must be a nonzero finite value".into()));
    }
    let (t, sign) = (thickness.abs(), thickness.signum());
    let mut records: Vec<(&BrepSolid, &FaceRecord)> = Vec::new();
    for body in faces {
        for face in body.shells.iter().flat_map(|shell| &shell.faces) {
            records.push((body, face));
        }
    }
    if records.is_empty() {
        return Err(input("faces", "at least one face is required".into()));
    }
    let size = records
        .iter()
        .flat_map(|(body, _)| &body.vertices)
        .map(|vertex| vertex.point.sub(records[0].0.vertices[0].point).length())
        .fold(t, f64::max);
    let tolerance = 1e-7 * size;
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;

    // every use of an edge by a face, as that face's loop runs along it
    let mut sides = Vec::new();
    for (index, (body, face)) in records.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
            let edge = body.edges.iter().find(|edge| edge.id == coedge.edge_id).ok_or_else(|| input("faces", "a face refers to a missing edge".into()))?;
            if edge.degenerate {
                continue;
            }
            let at = |fraction: f64| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Classify, "thickenFaces.edge");
            let (mut start, middle, mut end) = (at(0.0)?, at(0.5)?, at(1.0)?);
            if !coedge.forward {
                std::mem::swap(&mut start, &mut end);
            }
            let normals = [face_normal(face, start)?, face_normal(face, middle)?, face_normal(face, end)?];
            let along = edge.curve.derivatives(0.5 * (edge.t0 + edge.t1), 1).or_refuse(KernelStage::Classify, "thickenFaces.edge")?;
            let tangent = along.get(1).copied().unwrap_or(end.sub(start)).scale(if coedge.forward { 1.0 } else { -1.0 });
            sides.push(Side { face: index, start, middle, end, normals, tangent });
        }
    }

    // the slabs, face by face
    let mut parts = Vec::new();
    // `BREP_THICKEN_GLUE`: each part's container (a point set whose hull holds
    // it; `None` reads a polyhedral part's own vertices), and whether the
    // parts can be glued at all
    let mut glueable = thicken_glue_on();
    for (_, face) in &records {
        let loops: Vec<Vec<NurbsCurve>> = face.loops.iter().map(|record| record.coedges.iter().map(|coedge| coedge.pcurve.clone()).collect()).collect();
        let along = if face.same_sense { thickness } else { -thickness };
        let wedges = match past_its_axis(face, along, tolerance)? {
            Some(wedges) => Some(wedges),
            None => past_its_ring(face, along, tolerance)?,
        };
        match wedges {
            Some(wedges) => {
                glueable = false;
                parts.extend(wedges)
            }
            None => parts.extend(thicken_trimmed_sheet(&face.surface, &loops, along, false)?),
        }
    }
    let mut containers: Vec<Option<Vec<Vec3>>> = vec![None; parts.len()];

    // the creases: two faces' uses of one edge, run opposite ways
    struct Open {
        faces: [usize; 2],
        ends: [Vec3; 2],
        moved: [Vec3; 2],
    }
    let mut open = Vec::new();
    let mut cuts: Vec<BrepSolid> = Vec::new();
    for (i, a) in sides.iter().enumerate() {
        for b in sides.iter().skip(i + 1) {
            if a.face == b.face || !near(a.middle, b.middle) {
                continue;
            }
            // a closed crease starts where it ends: only the way it runs says how the two uses face
            let closed = near(a.start, a.end);
            let opposite = if closed { a.tangent.dot(b.tangent) < 0.0 } else { near(a.start, b.end) && near(a.end, b.start) };
            if !opposite {
                if closed || (near(a.start, b.start) && near(a.end, b.end)) {
                    return Err(input("normals", format!("faces {} and {} have their normals on opposite sides of the sheet", a.face, b.face)));
                }
                continue;
            }
            let run = a.end.sub(a.start);
            let tangent = if closed { a.tangent } else { run }.normalized().map_err(|_| refuse("crease", "a crease has no length".into()))?;
            // toward face a's inside, across the crease, and how far face b's moved image leans that way
            let inward = a.normals[1].cross(tangent);
            let lean = b.normals[1].scale(sign).dot(inward);
            let moved = [a.normals[1].scale(sign), b.normals[1].scale(sign)];
            let straight = a.middle.sub(a.start).cross(tangent).length() <= tolerance;
            let steady = |side: &Side| side.normals.iter().all(|normal| normal.sub(side.normals[1]).length() <= 1e-9);
            if lean.abs() <= 1e-9 {
                // the faces meet smoothly: the slabs' ends are one wall
                glueable = false;
                continue;
            }
            if lean > 0.0 {
                glueable = false;
                // the crease closes toward the thickened side. At a right angle, or wider, the
                // slabs end on each other or overlap, and the union is the answer. Tighter than a
                // right angle their square ends stand out behind the other face, and are cut off.
                let cosine = moved[0].dot(moved[1]);
                if cosine >= -1e-9 {
                    continue;
                }
                if !straight || !steady(a) || !steady(b) {
                    return Err(refuse("crease_closing", format!("the crease between faces {} and {} closes toward the thickened side tighter than a right angle and is not a straight line with constant normals", a.face, b.face)));
                }
                let inward_b = b.normals[1].cross(tangent.scale(-1.0));
                for (own, other, into) in [(moved[0], moved[1], inward), (moved[1], moved[0], inward_b)] {
                    let top = a.start.add(own.scale(t));
                    let reach = top.add(into.scale(t * -cosine / into.dot(other)));
                    let line = |from: Vec3, to: Vec3| make_line(from, to).or_refuse(KernelStage::Collect, "thickenFaces.cut_line");
                    cuts.push(extrude_profile_brep(&[line(a.start, top)?, line(top, reach)?, line(reach, a.start)?], run, run.length())?);
                }
                continue;
            }
            // b runs the other way: its start is a's end
            if !straight || !steady(a) || !steady(b) {
                return Err(refuse("crease_curved", format!("the crease between faces {} and {} opens toward the thickened side and is not a straight line with constant normals", a.face, b.face)));
            }
            parts.push(crease_fill(a.start, run, moved, t, corners)?);
            containers.push(crease_container(a.start, run, moved, t));
            open.push(Open { faces: [a.face, b.face], ends: [a.start, a.end], moved });
        }
    }

    // the corners: points where open creases end together
    let mut points: Vec<Vec3> = Vec::new();
    for crease in &open {
        for end in crease.ends {
            if !points.iter().any(|point| near(*point, end)) {
                points.push(end);
            }
        }
    }
    for point in points {
        let here: Vec<&Open> = open.iter().filter(|crease| crease.ends.iter().any(|end| near(*end, point))).collect();
        if here.len() == 1 {
            continue;
        }
        // three creases between three faces, each pair once
        let mut around: Vec<(usize, Vec3)> = Vec::new();
        for crease in &here {
            for (face, moved) in crease.faces.iter().zip(crease.moved) {
                if !around.iter().any(|(known, _)| known == face) {
                    around.push((*face, moved));
                }
            }
        }
        if here.len() != 3 || around.len() != 3 {
            return Err(refuse("corner", format!("{} open creases between {} faces meet in one corner; exactly three between three faces are built", here.len(), around.len())));
        }
        parts.push(corner_fill(point, [around[0].1, around[1].1, around[2].1], t, corners)?);
        containers.push(corner_container(point, [around[0].1, around[1].1, around[2].1], t));
    }

    if glueable && cuts.is_empty() && parts.len() > 1 {
        if let Some(glued) = glue_parts(&parts, &containers, tolerance)? {
            return Ok(glued);
        }
    }
    let united = if parts.len() == 1 { parts.remove(0) } else { boolean_operation_nary(&parts, BooleanOperation::Union)? };
    if cuts.is_empty() {
        return Ok(united);
    }
    let mut operands = vec![united];
    operands.extend(cuts);
    boolean_operation_nary(&operands, BooleanOperation::Subtract)
}

/// `BREP_THICKEN_GLUE` (default on since the tier P promotion of
/// 2026-11-20; `=0` restores the union): the parts of a sheet thickened across
/// open creases only (slabs, crease fills, corner fills; no closing or smooth
/// crease, no cut, no wedge past an axis) touch only along whole faces, a
/// slab's end against a fill's flat side, a fill's end against a corner's.
/// Their union is then their faces with each touching pair dropped, sewn and
/// its coplanar neighbours merged, which [`glue_parts`] builds without the
/// union's imprint. thicken/open-box-outward-rounded: seventeen parts, about
/// 0.38 s of its 0.57 s in the union's imprint (host).
fn thicken_glue_on() -> bool {
    switch!("BREP_THICKEN_GLUE", true)
}


/// A crease fill's container: the sharp fill's section (the start, the two
/// moved edges' points and the point at the thickness from both faces) at both
/// ends of the crease. The rounded section's arc lies in it (the arc's
/// tangents at its ends meet at that point). `None` when the faces turn so far
/// that the point runs off.
fn crease_container(start: Vec3, run: Vec3, moved: [Vec3; 2], t: f64) -> Option<Vec<Vec3>> {
    let turn = 1.0 + moved[0].dot(moved[1]);
    if turn <= 1e-6 {
        return None;
    }
    let section = [start, start.add(moved[0].scale(t)), start.add(moved[0].add(moved[1]).scale(t / turn)), start.add(moved[1].scale(t))];
    Some(section.iter().flat_map(|point| [*point, point.add(run)]).collect())
}

/// A corner fill's container: the sharp corner (the corner, the three moved
/// faces' points, the points at the thickness from two faces and from all
/// three), which is the cone of the moved directions cut by the three planes
/// at the thickness; the rounded fill (the ball of radius `t` in that cone)
/// lies in it. `None` when the directions do not span space.
fn corner_container(corner: Vec3, moved: [Vec3; 3], t: f64) -> Option<Vec<Vec3>> {
    let [a, b, c] = moved;
    let volume = a.dot(b.cross(c));
    if volume.abs() <= 1e-9 {
        return None;
    }
    let at = |direction: Vec3, length: f64| corner.add(direction.scale(length));
    let mut points = vec![corner, at(b.cross(c).add(c.cross(a)).add(a.cross(b)).scale(1.0 / volume), t)];
    for (i, j) in [(0, 1), (1, 2), (2, 0)] {
        let turn = 1.0 + moved[i].dot(moved[j]);
        if turn <= 1e-6 {
            return None;
        }
        points.push(at(moved[i], t));
        points.push(at(moved[i].add(moved[j]), t / turn));
    }
    Some(points)
}

/// [`thicken_glue_on`]: the union of `parts` read as their faces with each
/// touching pair dropped, sewn ([`crate::solid_from_sheets`]), its
/// coplanar neighbours merged and the edges that continue each other joined,
/// as the boolean finishes. `None` (the caller takes the union) unless:
/// every touching is a whole planar face of one part against a whole planar
/// face of another, facing it, with the same corners and edge midpoints;
/// every part touches another; no other pair of the parts' planar faces lies
/// coplanar facing across an overlap; and every two parts are apart, their
/// containers (`containers`, or a polyhedral part's own vertices) on the two
/// sides of one of their planar faces' planes or of a box gap. The sewing
/// refuses any edge left open, so a partial touching declines too.
fn glue_parts(parts: &[BrepSolid], containers: &[Option<Vec<Vec3>>], tolerance: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    struct Flat {
        part: usize,
        face: u64,
        normal: Vec3,
        offset: f64,
        corners: Vec<Vec3>,
        middles: Vec<Vec3>,
    }
    let stage = KernelStage::Classify;
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;
    let mut flats: Vec<Flat> = Vec::new();
    let mut hulls: Vec<Vec<Vec3>> = Vec::with_capacity(parts.len());
    for (index, part) in parts.iter().enumerate() {
        let edges: std::collections::HashMap<u64, &crate::topology::EdgeRecord> = part.edges.iter().map(|edge| (edge.id, edge)).collect();
        let straight = |edge: &crate::topology::EdgeRecord| -> Result<bool, KernelRefusal> {
            let (from, to) = (edge.curve.evaluate(edge.t0).or_refuse(stage, "thickenFaces.glue")?, edge.curve.evaluate(edge.t1).or_refuse(stage, "thickenFaces.glue")?);
            let chord = to.sub(from);
            let length = chord.length();
            if length <= tolerance {
                return Ok(false);
            }
            for control in &edge.curve.control_points {
                let point = control.point().or_refuse(stage, "thickenFaces.glue")?;
                if point.sub(from).cross(chord).length() > tolerance * length {
                    return Ok(false);
                }
            }
            Ok(true)
        };
        let mut polyhedral = true;
        for edge in &part.edges {
            if !edge.degenerate && !straight(edge)? {
                polyhedral = false;
            }
        }
        for face in part.shells.iter().flat_map(|shell| &shell.faces) {
            if !matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
                polyhedral = false;
                continue;
            }
            if face.loops.len() != 1 {
                continue;
            }
            let normal = crate::face_merge::outward_normal(face)?.normalized().or_refuse(stage, "thickenFaces.glue")?;
            let (mut corners, mut middles) = (Vec::new(), Vec::new());
            for coedge in &face.loops[0].coedges {
                let edge = edges.get(&coedge.edge_id).ok_or_else(|| input("faces", "a part's face refers to a missing edge".into()))?;
                corners.push(edge.curve.evaluate(if coedge.forward { edge.t0 } else { edge.t1 }).or_refuse(stage, "thickenFaces.glue")?);
                middles.push(edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).or_refuse(stage, "thickenFaces.glue")?);
            }
            let Some(first) = corners.first().copied() else { continue };
            flats.push(Flat { part: index, face: face.id, normal, offset: normal.dot(first), corners, middles });
        }
        hulls.push(match &containers[index] {
            Some(points) => points.clone(),
            None if polyhedral => part.vertices.iter().map(|vertex| vertex.point).collect(),
            None => return Ok(None),
        });
    }
    let same_set = |x: &[Vec3], y: &[Vec3]| x.len() == y.len() && x.iter().all(|p| y.iter().any(|q| near(*p, *q))) && y.iter().all(|q| x.iter().any(|p| near(*p, *q)));
    let bounds = |points: &[Vec3]| {
        points.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(low, high), point| {
            let p = [point.x, point.y, point.z];
            (std::array::from_fn(|k| low[k].min(p[k])), std::array::from_fn(|k| high[k].max(p[k])))
        })
    };
    // touching faces, each once; a coplanar pair facing across an overlap that is not one declines
    let mut partner: Vec<Option<usize>> = vec![None; flats.len()];
    for i in 0..flats.len() {
        for j in i + 1..flats.len() {
            let (f, g) = (&flats[i], &flats[j]);
            if f.part == g.part || f.normal.dot(g.normal) > -1.0 + 1e-9 || (f.offset + g.offset).abs() > tolerance {
                continue;
            }
            if same_set(&f.corners, &g.corners) && same_set(&f.middles, &g.middles) {
                if partner[i].is_some() || partner[j].is_some() {
                    return Ok(None);
                }
                partner[i] = Some(j);
                partner[j] = Some(i);
                continue;
            }
            let (low_f, high_f) = bounds(&[f.corners.as_slice(), f.middles.as_slice()].concat());
            let (low_g, high_g) = bounds(&[g.corners.as_slice(), g.middles.as_slice()].concat());
            let shared = (0..3).filter(|&k| high_f[k].min(high_g[k]) - low_f[k].max(low_g[k]) > tolerance).count();
            let apart = (0..3).any(|k| high_f[k].min(high_g[k]) - low_f[k].max(low_g[k]) < -tolerance);
            if !apart && shared >= 2 {
                return Ok(None);
            }
        }
    }
    if (0..parts.len()).any(|index| !flats.iter().zip(&partner).any(|(flat, other)| flat.part == index && other.is_some())) {
        return Ok(None);
    }
    // every two parts apart
    for i in 0..parts.len() {
        for j in i + 1..parts.len() {
            let ((low_i, high_i), (low_j, high_j)) = (bounds(&hulls[i]), bounds(&hulls[j]));
            if (0..3).any(|k| low_i[k] > high_j[k] + tolerance || low_j[k] > high_i[k] + tolerance) {
                continue;
            }
            let separated = flats.iter().filter(|flat| flat.part == i || flat.part == j).any(|flat| {
                let (own, other) = if flat.part == i { (&hulls[i], &hulls[j]) } else { (&hulls[j], &hulls[i]) };
                own.iter().all(|point| flat.normal.dot(*point) <= flat.offset + tolerance)
                    && other.iter().all(|point| flat.normal.dot(*point) >= flat.offset - tolerance)
            });
            if !separated {
                return Ok(None);
            }
        }
    }
    let mut bodies = Vec::with_capacity(parts.len());
    for (index, part) in parts.iter().enumerate() {
        let dropped: std::collections::HashSet<u64> =
            flats.iter().zip(&partner).filter(|(flat, other)| flat.part == index && other.is_some()).map(|(flat, _)| flat.face).collect();
        let mut body = part.clone();
        for shell in &mut body.shells {
            shell.faces.retain(|face| !dropped.contains(&face.id));
        }
        body.shells.retain(|shell| !shell.faces.is_empty());
        let used: std::collections::HashSet<u64> =
            body.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops).flat_map(|record| &record.coedges).map(|coedge| coedge.edge_id).collect();
        body.edges.retain(|edge| used.contains(&edge.id));
        let ends: std::collections::HashSet<u64> = body.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
        body.vertices.retain(|vertex| ends.contains(&vertex.id));
        body.mass_properties_cache = Default::default();
        bodies.push(body);
    }
    let Ok(sewn) = crate::solid_from_sheets(&bodies, tolerance) else { return Ok(None) };
    let Ok(merged) = crate::merge_same_surface_faces(&sewn, tolerance) else { return Ok(None) };
    match crate::merge_curve_continuation_edges(&merged, tolerance) {
        Ok(merged) if merged.validate().is_empty() => {
            Ok(Some(merged))
        }
        _ => Ok(None),
    }
}

/// A whole patch of a round cylinder (an arc along u, slid straight along v)
/// thickened toward its axis by its radius or more: every point `p + s·n`
/// then runs from the patch to the axis and, past it, out the other side. So
/// the body is the wedge between the patch and its axis and, opposite it, the
/// like wedge of what the thickness has left over the radius. `None` for any
/// other face or a thickness short of the axis, which the single-face thicken
/// takes.
fn past_its_axis(face: &FaceRecord, along: f64, tolerance: f64) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let surface = &face.surface;
    let stage = KernelStage::Classify;
    if surface.degree_v != 1 || surface.control_points.first().map(Vec::len) != Some(2) || !whole_domain(face)? {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(stage, "thickenFaces.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(stage, "thickenFaces.domain")?;
    let point = |u: f64, v: f64| surface.evaluate(u, v).or_refuse(stage, "thickenFaces.point");
    let normal = |u: f64, v: f64| surface.normal(u, v).or_refuse(stage, "thickenFaces.normal");
    let slide = point(u0, v1)?.sub(point(u0, v0)?);
    // the centre the normals at the two ends of the arc pass through, toward the side thickened
    let toward = along.signum();
    let (p0, n0, p1, n1) = (point(u0, v0)?, normal(u0, v0)?.scale(toward), point(u1, v0)?, normal(u1, v0)?.scale(toward));
    let chord = p1.sub(p0);
    let spread = n0.sub(n1);
    if spread.length() <= 1e-9 {
        return Ok(None);
    }
    // p0 + R·n0 = p1 + R·n1
    let radius = chord.dot(spread) / spread.dot(spread);
    if !(radius > 0.0) || along.abs() < radius - tolerance {
        return Ok(None);
    }
    let centre = p0.add(n0.scale(radius));
    const SAMPLES: usize = 16;
    for i in 0..=SAMPLES {
        let u = u0 + (u1 - u0) * i as f64 / SAMPLES as f64;
        for (v, shift) in [(v0, Vec3::new(0.0, 0.0, 0.0)), (v1, slide)] {
            let focus = point(u, v)?.add(normal(u, v)?.scale(toward * radius));
            if focus.sub(centre.add(shift)).length() > tolerance {
                return Ok(None);
            }
        }
    }
    if slide.cross(n0).length() < (1.0 - 1e-9) * slide.length() {
        return Ok(None);
    }
    // the wedge on the arc scaled about the centre by `factor`, slid along the axis
    let arc = surface.iso_curve_v(v0).or_refuse(stage, "thickenFaces.arc")?;
    let wedge = |factor: f64| -> Result<BrepSolid, KernelRefusal> {
        let scaled = |q: Vec3| centre.add(q.sub(centre).scale(factor));
        let poles = arc
            .control_points
            .iter()
            .map(|pole| crate::Vec4::from_point(scaled(Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)), pole.w))
            .collect();
        let rim = NurbsCurve::new(arc.degree, arc.knots.clone(), poles).or_refuse(stage, "thickenFaces.wedge_arc")?;
        let section = vec![
            make_line(centre, scaled(p0)).or_refuse(stage, "thickenFaces.wedge_side")?,
            rim,
            make_line(scaled(p1), centre).or_refuse(stage, "thickenFaces.wedge_side")?,
        ];
        extrude_profile_brep(&section, slide, slide.length())
    };
    let mut wedges = vec![wedge(1.0)?];
    let over = along.abs() - radius;
    if over > tolerance {
        wedges.push(wedge(-over / radius)?);
    }
    Ok(Some(wedges))
}

/// Is the face its surface's whole domain (its one loop's corners the domain's), regular at its corners?
fn whole_domain(face: &FaceRecord) -> Result<bool, KernelRefusal> {
    let surface = &face.surface;
    let stage = KernelStage::Classify;
    if face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(false);
    }
    let [u0, u1] = surface.domain_u().or_refuse(stage, "thickenFaces.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(stage, "thickenFaces.domain")?;
    let corners = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
    let slack = 1e-9 * ((u1 - u0).abs() + (v1 - v0).abs());
    for coedge in &face.loops[0].coedges {
        let [t0, t1] = coedge.pcurve.domain().or_refuse(stage, "thickenFaces.pcurve")?;
        for t in [t0, t1] {
            let at = coedge.pcurve.evaluate(t).or_refuse(stage, "thickenFaces.pcurve")?;
            if !corners.iter().any(|corner| (at.x - corner[0]).abs() + (at.y - corner[1]).abs() <= slack) {
                return Ok(false);
            }
        }
        // and runs along one side of the domain between them, not across the face
        const SAMPLES: usize = 16;
        let mut on_side = [true; 4];
        for index in 1..SAMPLES {
            let at = coedge.pcurve.evaluate(t0 + (t1 - t0) * index as f64 / SAMPLES as f64).or_refuse(stage, "thickenFaces.pcurve")?;
            for (side, off) in [(at.x - u0).abs(), (at.x - u1).abs(), (at.y - v0).abs(), (at.y - v1).abs()].into_iter().enumerate() {
                on_side[side] &= off <= slack;
            }
        }
        if !on_side.iter().any(|on| *on) {
            return Ok(false);
        }
    }
    // a patch with a pole (a side collapsed to a point, where it has no normal) is no arc slid or turned
    Ok(corners.iter().all(|corner| surface.normal(corner[0], corner[1]).is_ok()))
}

/// Places along one parameter of a surface that no span of it can hide
/// between: twice the degree and one more in every knot span, ends included.
/// A span is a rational piece of that degree, so the square of a distance
/// along it, cleared of its weight, is a polynomial of twice the degree: one
/// that vanishes at these vanishes along the span.
fn span_places(knots: &[f64], degree: usize, low: f64, high: f64) -> Vec<f64> {
    let mut breaks = vec![low];
    breaks.extend(knots.iter().copied().filter(|knot| *knot > low && *knot < high));
    breaks.push(high);
    breaks.dedup();
    let per_span = 2 * degree + 1;
    let mut places = vec![low];
    for span in breaks.windows(2) {
        for index in 1..=per_span {
            places.push(span[0] + (span[1] - span[0]) * index as f64 / per_span as f64);
        }
    }
    *places.last_mut().expect("places") = high;
    places
}

/// A whole patch of a ring (a circular arc along one parameter, turned about
/// an axis along the other: a quarter of a torus round a cup's floor)
/// thickened toward its tube's centre circle by the tube's radius or more.
/// Every point `p + s·n` then runs from the patch to that circle and, past it,
/// out the other side, each in its own half-plane through the axis. So the
/// body is the arc's sector (the arc and its centre) turned about the axis
/// and, opposite it, the like sector of what the thickness has left over the
/// radius. `None` for any other face, a thickness short of the centre circle,
/// or a leftover that would reach the axis, which the single-face thicken
/// takes.
fn past_its_ring(face: &FaceRecord, along: f64, tolerance: f64) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    if !whole_domain(face)? {
        return Ok(None);
    }
    for arc_along_u in [false, true] {
        if let Some(sectors) = ring_sectors(face, along, tolerance, arc_along_u)? {
            return Ok(Some(sectors));
        }
    }
    Ok(None)
}

fn ring_sectors(face: &FaceRecord, along: f64, tolerance: f64, arc_along_u: bool) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let surface = &face.surface;
    let stage = KernelStage::Classify;
    let [u0, u1] = surface.domain_u().or_refuse(stage, "thickenFaces.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(stage, "thickenFaces.domain")?;
    // (a, b): along the arc, round the axis
    let ([a0, a1], [b0, b1]) = if arc_along_u { ([u0, u1], [v0, v1]) } else { ([v0, v1], [u0, u1]) };
    let at = |a: f64, b: f64| if arc_along_u { (a, b) } else { (b, a) };
    let point = |a: f64, b: f64| surface.evaluate(at(a, b).0, at(a, b).1).or_refuse(stage, "thickenFaces.point");
    let toward = along.signum();
    let normal = |a: f64, b: f64| surface.normal(at(a, b).0, at(a, b).1).map(|normal| normal.scale(toward)).or_refuse(stage, "thickenFaces.normal");
    // the centre the normals at the two ends of the first arc pass through
    let (p0, n0, p1, n1) = (point(a0, b0)?, normal(a0, b0)?, point(a1, b0)?, normal(a1, b0)?);
    let spread = n0.sub(n1);
    if spread.length() <= 1e-9 {
        return Ok(None);
    }
    let radius = p1.sub(p0).dot(spread) / spread.dot(spread);
    if !(radius > 0.0) || along.abs() < radius - tolerance {
        return Ok(None);
    }
    // read in every span of the surface both ways, so nothing between the readings is taken on trust
    let (arc_places, turn_places) = if arc_along_u {
        (span_places(&surface.knots_u, surface.degree_u, a0, a1), span_places(&surface.knots_v, surface.degree_v, b0, b1))
    } else {
        (span_places(&surface.knots_v, surface.degree_v, a0, a1), span_places(&surface.knots_u, surface.degree_u, b0, b1))
    };
    let (arcs, turns) = (arc_places.len() - 1, turn_places.len() - 1);
    let arc_at = |i: usize| arc_places[i];
    let turn_at = |j: usize| turn_places[j];
    // each arc has one centre, the tube's centre line at that turn
    let mut centres = Vec::with_capacity(turns + 1);
    for j in 0..=turns {
        let centre = point(a0, turn_at(j))?.add(normal(a0, turn_at(j))?.scale(radius));
        for i in 1..=arcs {
            if point(arc_at(i), turn_at(j))?.add(normal(arc_at(i), turn_at(j))?.scale(radius)).sub(centre).length() > tolerance {
                return Ok(None);
            }
        }
        centres.push(centre);
    }
    // the centres lie on a circle: its axis from three of them
    let (f0, f1, f2) = (centres[0], centres[turns / 3], centres[2 * turns / 3]);
    let (e1, e2) = (f1.sub(f0), f2.sub(f0));
    let across = e1.cross(e2);
    if across.length() <= 1e-9 * e1.length() * e2.length() || across.length() <= tolerance * tolerance {
        return Ok(None);
    }
    let axis = across.normalized().or_refuse(stage, "thickenFaces.ring_axis")?;
    // circumcentre of the three
    let middle = f0.add(
        e2.scale(e1.dot(e1)).sub(e1.scale(e2.dot(e2))).cross(across).scale(1.0 / (2.0 * across.dot(across))),
    );
    let ring = f0.sub(middle).length();
    let over = along.abs() - radius;
    if !(ring > over + tolerance) {
        return Ok(None);
    }
    // every arc is the first turned about that axis, by a growing angle
    let mut angle = 0.0;
    for j in 1..=turns {
        let (before, here) = (centres[j - 1].sub(middle), centres[j].sub(middle));
        if (here.length() - ring).abs() > tolerance || here.dot(axis).abs() > tolerance {
            return Ok(None);
        }
        let step = axis.dot(before.cross(here)).atan2(before.dot(here));
        if !(step > 0.0) {
            return Ok(None);
        }
        angle += step;
        for i in 0..=arcs {
            let first = point(arc_at(i), b0)?.sub(middle);
            let turned = middle.add(first.rotate_about(axis, angle));
            if point(arc_at(i), turn_at(j))?.sub(turned).length() > tolerance {
                return Ok(None);
            }
        }
    }
    // the first arc lies in a half-plane through the axis, off the axis itself
    let square = axis.cross(f0.sub(middle));
    for i in 0..=arcs {
        let from_axis = point(arc_at(i), b0)?.sub(middle);
        if from_axis.dot(square).abs() > tolerance * ring.max(1.0) || from_axis.sub(axis.scale(from_axis.dot(axis))).length() <= tolerance {
            return Ok(None);
        }
    }
    let arc = if arc_along_u { surface.iso_curve_v(b0) } else { surface.iso_curve_u(b0) }.or_refuse(stage, "thickenFaces.arc")?;
    let turn = angle.min(std::f64::consts::TAU);
    // the sector on the arc scaled about its centre by `factor`, turned about the axis
    let sector = |factor: f64| -> Result<BrepSolid, KernelRefusal> {
        let scaled = |q: Vec3| f0.add(q.sub(f0).scale(factor));
        let poles = arc
            .control_points
            .iter()
            .map(|pole| crate::Vec4::from_point(scaled(Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)), pole.w))
            .collect();
        let rim = NurbsCurve::new(arc.degree, arc.knots.clone(), poles).or_refuse(stage, "thickenFaces.sector_arc")?;
        let section = vec![
            make_line(f0, scaled(p0)).or_refuse(stage, "thickenFaces.sector_side")?,
            rim,
            make_line(scaled(p1), f0).or_refuse(stage, "thickenFaces.sector_side")?,
        ];
        crate::revolve_profile_brep(&section, middle, axis, turn).or_refuse(KernelStage::Fragment, "thickenFaces.sector")
    };
    let mut sectors = vec![sector(1.0)?];
    if over > tolerance {
        sectors.push(sector(-over / radius)?);
    }
    Ok(Some(sectors))
}

/// The fill of an open straight crease from `start` along `run`, between the
/// two faces' moved images `moved` (unit vectors square to the crease).
fn crease_fill(start: Vec3, run: Vec3, moved: [Vec3; 2], t: f64, corners: OffsetCorners) -> Result<BrepSolid, KernelRefusal> {
    let collect = KernelStage::Collect;
    let (a, b) = (start.add(moved[0].scale(t)), start.add(moved[1].scale(t)));
    let line = |from: Vec3, to: Vec3| make_line(from, to).or_refuse(collect, "thickenFaces.fill_line");
    let section = match corners {
        OffsetCorners::Sharp => {
            // the point at the thickness from both faces, in the section's plane
            let apex = start.add(moved[0].add(moved[1]).scale(t / (1.0 + moved[0].dot(moved[1]))));
            vec![line(start, a)?, line(a, apex)?, line(apex, b)?, line(b, start)?]
        }
        OffsetCorners::Rounded => {
            let across = moved[1].sub(moved[0].scale(moved[1].dot(moved[0]))).normalized().or_refuse(collect, "thickenFaces.fill_frame")?;
            let angle = moved[0].dot(moved[1]).clamp(-1.0, 1.0).acos();
            vec![line(start, a)?, make_arc(start, moved[0], across, t, 0.0, angle).or_refuse(collect, "thickenFaces.fill_arc")?, line(b, start)?]
        }
    };
    extrude_profile_brep(&section, run, run.length())
}

/// The fill of a corner where three open creases meet, between the three
/// faces' moved images `moved` (unit vectors).
fn corner_fill(corner: Vec3, moved: [Vec3; 3], t: f64, corners: OffsetCorners) -> Result<BrepSolid, KernelRefusal> {
    let at = |direction: Vec3, length: f64| corner.add(direction.scale(length));
    match corners {
        OffsetCorners::Sharp => {
            // the point at the thickness from all three faces: moved[i] · x = 1
            let [a, b, c] = moved;
            let volume = a.dot(b.cross(c));
            if volume.abs() <= 1e-9 {
                return Err(refuse("corner", "the three faces at a corner do not span space".into()));
            }
            let far = b.cross(c).add(c.cross(a)).add(a.cross(b)).scale(1.0 / volume);
            let pair = |i: usize, j: usize| at(moved[i].add(moved[j]), t / (1.0 + moved[i].dot(moved[j])));
            let tip = |i: usize| at(moved[i], t);
            let far = at(far, t);
            let mut faces = Vec::new();
            for (i, j) in [(0, 1), (1, 2), (2, 0)] {
                faces.push(vec![corner, tip(i), pair(i, j), tip(j)]);
            }
            for (i, j, k) in [(0, 1, 2), (1, 2, 0), (2, 0, 1)] {
                faces.push(vec![tip(i), pair(i, j), far, pair(i, k)]);
            }
            solid_from_polygons(&faces)
        }
        OffsetCorners::Rounded => {
            // the ball about the corner, kept between the three directions: its poles and its
            // seam turned away from the piece kept
            let middle = moved[0].add(moved[1]).add(moved[2]).normalized().or_refuse(KernelStage::Collect, "thickenFaces.corner_frame")?;
            let pole = middle.cross(moved[0]).normalized().or_refuse(KernelStage::Collect, "thickenFaces.corner_frame")?;
            let ball = make_sphere_brep_framed(corner, t, pole, Some(middle.scale(-1.0))).or_refuse(KernelStage::Collect, "thickenFaces.corner_ball")?;
            let reach = 4.0 * t;
            let tips = [at(moved[0], reach), at(moved[1], reach), at(moved[2], reach)];
            let base = tips[1].sub(tips[0]).cross(tips[2].sub(tips[0]));
            if (tips[0].sub(corner).dot(base) / base.length()).abs() <= t {
                return Err(refuse("corner", "the three faces at a corner are too nearly flat for a ball between them".into()));
            }
            let wedge = solid_from_polygons(&[
                vec![corner, tips[0], tips[1]],
                vec![corner, tips[1], tips[2]],
                vec![corner, tips[2], tips[0]],
                vec![tips[0], tips[1], tips[2]],
            ])?;
            boolean_operation(&ball, &wedge, BooleanOperation::Intersect, &BooleanOptions::default())
        }
    }
}


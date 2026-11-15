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
//! Faces that meet smoothly along a crease (a bend and the flat it leaves)
//! need no fill. Where a crease closes toward the side thickened at a right
//! angle or wider, the slabs end on each other or overlap; tighter than a
//! right angle their square ends are cut off behind the other face. Fills are built where the crease is a straight line along
//! which both faces keep their normals; any other open crease is refused by
//! name, as is a corner where the open creases are not exactly three.
use crate::topology::FaceRecord;
use crate::{
    boolean_operation, boolean_operation_nary, extrude_profile_brep, make_arc, make_line, make_sphere_brep_framed, project_point_to_surface,
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
}

fn face_normal(face: &FaceRecord, point: Vec3) -> Result<Vec3, KernelRefusal> {
    let foot = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Classify, "thickenFaces.project")?;
    let normal = face.surface.normal(foot.u, foot.v).or_refuse(KernelStage::Classify, "thickenFaces.normal")?;
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
            sides.push(Side { face: index, start, middle, end, normals });
        }
    }

    // the slabs, face by face
    let mut parts = Vec::new();
    for (_, face) in &records {
        let loops: Vec<Vec<NurbsCurve>> = face.loops.iter().map(|record| record.coedges.iter().map(|coedge| coedge.pcurve.clone()).collect()).collect();
        let along = if face.same_sense { thickness } else { -thickness };
        match past_its_axis(face, along, tolerance)? {
            Some(wedges) => parts.extend(wedges),
            None => parts.extend(thicken_trimmed_sheet(&face.surface, &loops, along, false)?),
        }
    }

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
            if !(near(a.start, b.end) && near(a.end, b.start)) {
                if near(a.start, b.start) && near(a.end, b.end) {
                    return Err(input("normals", format!("faces {} and {} have their normals on opposite sides of the sheet", a.face, b.face)));
                }
                continue;
            }
            let run = a.end.sub(a.start);
            let tangent = run.normalized().map_err(|_| refuse("crease", "a crease has no length".into()))?;
            // toward face a's inside, across the crease, and how far face b's moved image leans that way
            let inward = a.normals[1].cross(tangent);
            let lean = b.normals[1].scale(sign).dot(inward);
            let moved = [a.normals[1].scale(sign), b.normals[1].scale(sign)];
            let straight = a.middle.sub(a.start).cross(tangent).length() <= tolerance;
            let steady = |side: &Side| side.normals.iter().all(|normal| normal.sub(side.normals[1]).length() <= 1e-9);
            if lean.abs() <= 1e-9 {
                // the faces meet smoothly: the slabs' ends are one wall
                continue;
            }
            if lean > 0.0 {
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
    }

    let united = if parts.len() == 1 { parts.remove(0) } else { boolean_operation_nary(&parts, BooleanOperation::Union)? };
    if cuts.is_empty() {
        return Ok(united);
    }
    let mut operands = vec![united];
    operands.extend(cuts);
    boolean_operation_nary(&operands, BooleanOperation::Subtract)
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
    if surface.degree_v != 1 || surface.control_points.first().map(Vec::len) != Some(2) || face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(stage, "thickenFaces.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(stage, "thickenFaces.domain")?;
    // the face is its surface's whole domain when its loop's corners are the domain's
    let corners = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
    for coedge in &face.loops[0].coedges {
        let [t0, t1] = coedge.pcurve.domain().or_refuse(stage, "thickenFaces.pcurve")?;
        for t in [t0, t1] {
            let at = coedge.pcurve.evaluate(t).or_refuse(stage, "thickenFaces.pcurve")?;
            if !corners.iter().any(|corner| (at.x - corner[0]).abs() + (at.y - corner[1]).abs() <= 1e-9 * ((u1 - u0).abs() + (v1 - v0).abs())) {
                return Ok(None);
            }
        }
    }
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


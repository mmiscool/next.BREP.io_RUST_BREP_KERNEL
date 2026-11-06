use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::offset_retrim::{rebuild_loop_pcurves, retrim_planar_face};
use crate::{RevolutionFrame, Vec4};
use super::face_move::{carrier_ruled, retrim_ruled_face};
use super::face_rotate::rotation_about;

// ---------------------------------------------------------------------------
// DRAFT FACES: tilt chosen faces of a solid about a neutral plane, the way a
// mould designer drafts the walls of a part so it can be pulled out.
//
// The contract (what a draft IS, so every branch below can be checked against
// it): given a PULL direction `d`, a NEUTRAL plane (through `neutral_point`,
// normal `d`) and an angle `α`, every selected face is replaced by its drafted
// carrier, and every other face keeps its carrier and is re-trimmed to meet the
// drafted ones.
//
// * A PLANE parallel to `d` turns about its NEUTRAL LINE (where it crosses the
//   neutral plane) until it leans `α` from `d`. Positive `α` leans it INTO the
//   material as it goes along `d`: its outward normal `m` becomes
//   `m·cos α + d·sin α`. So positive drafts shrink a boss and widen a pocket
//   or a hole as they rise, which is what a part pulled along `d` needs.
// * A CYLINDER whose axis is parallel to `d` becomes the CONE about the same
//   axis through the cylinder's circle on the neutral plane, its every ruling
//   leaning `α` from `d` the same way: radius `r₀ − σ·h·tan α` at height `h`
//   above the neutral plane, `σ = +1` where the material is inside the
//   cylinder (a boss), `−1` where it is outside (a hole).
//
// Both drafted carriers contain the face's NEUTRAL CURVE, and both are ruled
// along `d` tilted by `α`, so two drafted faces meet in a straight line through
// the point where their old common edge crossed the neutral plane (a plane
// pair: the line of their planes; a tangent plane/cone pair, or a cone's seam:
// the shared ruling). That one fact places every corner the draft moves.
//
// The topology is NOT changed: the same faces, edges and vertices come out,
// with new geometry. A draft that would change it — one that consumes a face,
// as a steep draft consumes the top of a narrow box — is taken by the convex
// route below where the body is a convex polyhedron (the result is then the
// intersection of the faces' half-spaces, the drafted ones replaced), and
// refused by name everywhere else. A face that crosses the neutral plane and
// must be drafted AWAY from it on both sides is a PARTING-LINE draft:
// [`draft_faces_parted`] splits the body there and drafts each side.
//
// What is refused, by name, before any geometry is built: a selected face that
// is neither a plane parallel to the pull nor a cylinder about it, a draft of
// 90° or more, a vertex whose carriers this file has no closed-form solve for,
// and a curved edge that is not a section of a cylinder or cone by a plane.
// Nothing here returns a body it has not validated.
// ---------------------------------------------------------------------------

const OP: &str = "draft_faces";

/// The refusal slug a draft that turns an edge end over end, or collapses
/// it, is minted with: the topology the draft would need has fewer faces or
/// edges than the input. The convex route takes these; everything else refuses.
const CONSUMES_FACE: &str = "draft_consumes_face";

/// A face's carrier AFTER the draft, in closed form.
#[derive(Clone)]
enum Carrier {
    /// Unit `normal`; the sign is the surface's own, not the outward one.
    Plane { origin: Vec3, normal: Vec3 },
    /// A cylinder or cone: radius `rho0 + (rho1 − rho0)·h/height` at axial
    /// height `h = (p − frame.origin)·frame.axis` — the parameters
    /// `plane_ruled_section_arc` reads.
    Ruled { frame: RevolutionFrame, rho0: f64, rho1: f64, height: f64 },
}

impl Carrier {
    fn residual(&self, point: Vec3) -> f64 {
        match self {
            Carrier::Plane { origin, normal } => point.sub(*origin).dot(*normal),
            Carrier::Ruled { frame, rho0, rho1, height } => {
                let w = point.sub(frame.origin);
                let h = w.dot(frame.axis);
                let radial = w.sub(frame.axis.scale(h)).length();
                radial - (rho0 + (rho1 - rho0) * h / height)
            }
        }
    }
}

/// How a selected face is drafted.
#[derive(Clone)]
enum Draft {
    /// The face's old plane frame, its OUTWARD normal `outward` (horizontal),
    /// and the rotation that carries the old frame to the drafted one.
    Plane { old: Plane, outward: Vec3, rotation: AffineTransform },
    /// A cylinder about an axis through `axis_point` (on the neutral plane)
    /// parallel to the pull; `sign` is σ (+1 a boss, −1 a hole).
    Cylinder { axis_point: Vec3, radius: f64, sign: f64 },
}

/// The surface normal of `face` turned to point OUT of the material.
fn outward_sign(face: &FaceRecord) -> f64 {
    if face.same_sense {
        1.0
    } else {
        -1.0
    }
}

/// A unit vector perpendicular to `axis` (the frame's `x_axis`).
fn frame_about(origin: Vec3, axis: Vec3) -> Result<RevolutionFrame, KernelRefusal> {
    let x_axis = axis.perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?;
    let y_axis = axis.cross(x_axis).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    Ok(RevolutionFrame { origin, axis, x_axis, y_axis })
}

/// A cylinder whose axis is parallel to `pull`, recognised from the surface
/// itself: every sampled normal square to `pull`, and every sampled point the
/// same distance from one line along `pull`. Returns the axis point (in the
/// plane through the world origin square to `pull`) and the radius. The tag
/// alone is not enough: an extruded arc carries no revolution tag.
fn vertical_cylinder(surface: &NurbsSurface, pull: Vec3, tolerance: f64) -> Option<(Vec3, f64)> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let ex = pull.perpendicular().ok()?;
    let ey = pull.cross(ex).normalized().ok()?;
    let mut samples: Vec<[f64; 2]> = Vec::with_capacity(25);
    for i in 0..5 {
        for j in 0..5 {
            let u = u0 + (u1 - u0) * (0.05 + 0.9 * i as f64 / 4.0);
            let v = v0 + (v1 - v0) * (0.05 + 0.9 * j as f64 / 4.0);
            let normal = surface.normal(u, v).ok()?;
            if normal.dot(pull).abs() > 1e-7 {
                return None;
            }
            let point = surface.evaluate(u, v).ok()?;
            samples.push([point.dot(ex), point.dot(ey)]);
        }
    }
    let distance = |a: [f64; 2], b: [f64; 2]| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let a = samples[0];
    let b = *samples.iter().max_by(|p, q| distance(a, **p).total_cmp(&distance(a, **q)))?;
    let area = |p: [f64; 2]| ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])).abs();
    let c = *samples.iter().max_by(|p, q| area(**p).total_cmp(&area(**q)))?;
    if area(c) <= 1e-9 * distance(a, b).powi(2).max(1e-30) {
        return None; // the samples are collinear: a plane, not a cylinder
    }
    let (bx, by) = (b[0] - a[0], b[1] - a[1]);
    let (cx, cy) = (c[0] - a[0], c[1] - a[1]);
    let det = 2.0 * (bx * cy - by * cx);
    let ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / det;
    let uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / det;
    let center = [a[0] + ux, a[1] + uy];
    let radius = (ux * ux + uy * uy).sqrt();
    if samples.iter().any(|p| (distance(*p, center) - radius).abs() > tolerance) {
        return None;
    }
    Some((ex.scale(center[0]).add(ey.scale(center[1])), radius))
}

/// The closed-form carrier of a face as it stands: its plane, or its cylinder
/// or cone. `None` for every other carrier.
fn carrier_of(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    pull: Vec3,
    plane_tolerance: f64,
) -> Option<Carrier> {
    let &(shell, index) = face_lookup.get(&face_id)?;
    let surface = &solid.shells[shell].faces[index].surface;
    if let Ok(plane) = plane_of_surface(surface, plane_tolerance, OP) {
        return Some(Carrier::Plane { origin: plane.origin, normal: plane.normal });
    }
    if let Some((frame, rho0, rho1, height)) = carrier_ruled(solid, face_lookup, face_id) {
        return Some(Carrier::Ruled { frame, rho0, rho1, height });
    }
    let (axis_point, radius) = vertical_cylinder(surface, pull, plane_tolerance)?;
    let frame = frame_about(axis_point, pull).ok()?;
    Some(Carrier::Ruled { frame, rho0: radius, rho1: radius, height: 1.0 })
}

/// Unwrapped signed azimuth swept by `curve` over `[t0, t1]` about `frame`.
fn swept_azimuth(frame: &RevolutionFrame, curve: &NurbsCurve, t0: f64, t1: f64) -> Result<f64, KernelRefusal> {
    let azimuth = |point: Vec3| {
        let w = point.sub(frame.origin);
        w.dot(frame.y_axis).atan2(w.dot(frame.x_axis))
    };
    let steps = 64;
    let mut previous = azimuth(curve.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?);
    let mut total = 0.0;
    for step in 1..=steps {
        let t = t0 + (t1 - t0) * step as f64 / steps as f64;
        let next = azimuth(curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?);
        let mut delta = next - previous;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        total += delta;
        previous = next;
    }
    Ok(total)
}

/// Where the line `point + t·direction` meets `carrier`, nearest `near`.
fn line_meets(carrier: &Carrier, point: Vec3, direction: Vec3, near: Vec3) -> Option<Vec3> {
    match carrier {
        Carrier::Plane { origin, normal } => {
            let denominator = normal.dot(direction);
            if denominator.abs() <= 1e-12 {
                return None;
            }
            let t = normal.dot(origin.sub(point)) / denominator;
            Some(point.add(direction.scale(t)))
        }
        Carrier::Ruled { frame, rho0, rho1, height } => {
            let k = (rho1 - rho0) / height;
            let w0 = point.sub(frame.origin);
            let h0 = w0.dot(frame.axis);
            let q0 = w0.sub(frame.axis.scale(h0));
            let h1 = direction.dot(frame.axis);
            let q1 = direction.sub(frame.axis.scale(h1));
            let r0 = rho0 + k * h0;
            let a = q1.dot(q1) - k * k * h1 * h1;
            let b = 2.0 * (q0.dot(q1) - r0 * k * h1);
            let c = q0.dot(q0) - r0 * r0;
            let mut roots = Vec::new();
            if a.abs() <= 1e-14 * (b.abs() + c.abs()).max(1e-300) {
                if b.abs() > 0.0 {
                    roots.push(-c / b);
                }
            } else {
                let disc = b * b - 4.0 * a * c;
                if disc < 0.0 {
                    return None;
                }
                let root = disc.sqrt();
                roots.push((-b + root) / (2.0 * a));
                roots.push((-b - root) / (2.0 * a));
            }
            roots
                .into_iter()
                .filter(|t| r0 + k * h1 * t >= 0.0)
                .map(|t| point.add(direction.scale(t)))
                .min_by(|p, q| p.sub(near).length().total_cmp(&q.sub(near).length()))
        }
    }
}

/// The common point of three or more planes (least squares, rank 3 required).
fn planes_meet(planes: &[(Vec3, Vec3)]) -> Option<Vec3> {
    let mut m = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    for (origin, normal) in planes {
        let n = [normal.x, normal.y, normal.z];
        let b = normal.dot(*origin);
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += n[r] * n[c];
            }
            rhs[r] += n[r] * b;
        }
    }
    let det = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(&m);
    let trace = m[0][0] + m[1][1] + m[2][2];
    if d.abs() <= 1e-10 * trace.powi(3) {
        return None;
    }
    let mut solution = [0.0f64; 3];
    for column in 0..3 {
        let mut replaced = m;
        for row in 0..3 {
            replaced[row][column] = rhs[row];
        }
        solution[column] = det(&replaced) / d;
    }
    Some(Vec3::new(solution[0], solution[1], solution[2]))
}

/// Draft `face_ids` of `solid` by `angle_radians` for the pull direction
/// `pull_direction`, about the neutral plane through `neutral_point` square to
/// the pull. See the contract at the top of this file: a positive angle leans
/// each face into the material as it goes along the pull (a boss narrows, a
/// pocket or a hole widens); a negative one leans it out.
///
/// Each selected face must be a plane parallel to the pull or a cylinder whose
/// axis is parallel to it. The other faces keep their carriers and are
/// re-trimmed to the drafted ones; the body's topology is unchanged, except on
/// a convex polyhedron, where a draft that consumes a face is built as the
/// intersection of the faces' half-spaces. Anything else is refused by name.
pub fn draft_faces(
    solid: &BrepSolid,
    face_ids: &[u64],
    pull_direction: Vec3,
    neutral_point: Vec3,
    angle_radians: f64,
) -> Result<BrepSolid, KernelRefusal> {
    match draft_faces_in_place(solid, face_ids, pull_direction, neutral_point, angle_radians) {
        Err(refusal)
            if matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what } if what == CONSUMES_FACE) =>
        {
            draft_convex_polyhedron(solid, face_ids, pull_direction, neutral_point, angle_radians)
                .map_err(|convex| {
                    KernelRefusal::unsupported(KernelStage::Classify, CONSUMES_FACE, format!(
                        "{OP}: the draft changes the body's topology ({refusal}), and the convex \
                         route that builds such a draft cannot take this body ({convex}) — refusing"
                    ))
                })
        }
        other => other,
    }
}

fn check_inputs(
    face_ids: &[u64],
    pull_direction: Vec3,
    neutral_point: Vec3,
    angle_radians: f64,
) -> Result<Vec3, KernelRefusal> {
    let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
    if !finite(pull_direction) {
        return Err(KernelRefusal::input(KernelStage::Collect, "pull_direction", format!("{OP}: the pull direction must be finite")));
    }
    if !finite(neutral_point) {
        return Err(KernelRefusal::input(KernelStage::Collect, "neutral_point", format!("{OP}: the neutral point must be finite")));
    }
    if !angle_radians.is_finite() || angle_radians.abs() >= 89.0f64.to_radians() {
        return Err(KernelRefusal::input(KernelStage::Collect, "angle", format!(
            "{OP}: the draft angle must be finite and less than 89° either way (got {angle_radians})"
        )));
    }
    if face_ids.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "selection", format!("{OP}: no faces selected")));
    }
    pull_direction.normalized().map_err(|_| {
        KernelRefusal::input(KernelStage::Collect, "pull_direction", format!("{OP}: the pull direction has zero length"))
    })
}

fn draft_faces_in_place(
    solid: &BrepSolid,
    face_ids: &[u64],
    pull_direction: Vec3,
    neutral_point: Vec3,
    angle_radians: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let pull = check_inputs(face_ids, pull_direction, neutral_point, angle_radians)?;
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let accept = 10.0 * plane_tolerance;
    let (sin, cos) = angle_radians.sin_cos();
    let tan = angle_radians.tan();
    let height_of = |point: Vec3| point.sub(neutral_point).dot(pull);

    let mut face_lookup: HashMap<u64, (usize, usize)> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            face_lookup.entry(face.id).or_insert((shell_index, face_index));
        }
    }
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    for &face_id in &selected {
        if !face_lookup.contains_key(&face_id) {
            return Err(KernelRefusal::input(KernelStage::Collect, "face_id", format!("{OP}: no face with id {face_id}")));
        }
    }
    if angle_radians == 0.0 {
        return Ok(solid.clone());
    }

    // --- Classify every selected face, and build its drafted carrier --------
    let mut drafts: HashMap<u64, Draft> = HashMap::default();
    let mut carriers: HashMap<u64, Carrier> = HashMap::default();
    for &face_id in &selected {
        let (shell, index) = face_lookup[&face_id];
        let face = &solid.shells[shell].faces[index];
        let sign = outward_sign(face);
        if let Ok(old) = plane_of_surface(&face.surface, plane_tolerance, OP) {
            let outward = old.normal.scale(sign);
            if outward.dot(pull).abs() > 1e-9 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "draft_plane_not_parallel", format!(
                    "{OP}: face {face_id} is a plane that is not parallel to the pull direction \
                     (its normal makes {:.6}° with the pull); a draft turns a face that runs along \
                     the pull — refusing",
                    outward.dot(pull).clamp(-1.0, 1.0).acos().to_degrees()
                )));
            }
            // Turning about k = m × d by +α carries m toward d: m' = m·cos α + d·sin α.
            let hinge = outward.cross(pull).normalized().or_refuse(KernelStage::Classify, "hinge")?;
            let on_hinge = old.origin.add(pull.scale(-height_of(old.origin)));
            let rotation = rotation_about(on_hinge, hinge, angle_radians).or_refuse(KernelStage::Classify, "rotation")?;
            let turned = super::face_rotate::rotate_direction(&rotation, old.normal);
            carriers.insert(face_id, Carrier::Plane { origin: on_hinge, normal: turned });
            drafts.insert(face_id, Draft::Plane { old, outward, rotation });
            continue;
        }
        let Some((axis_point, radius)) = vertical_cylinder(&face.surface, pull, plane_tolerance) else {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "draft_carrier_kind", format!(
                "{OP}: face {face_id} is neither a plane parallel to the pull direction nor a \
                 cylinder whose axis is the pull direction; a draft turns those into a tilted plane \
                 and a cone, and has no drafted carrier for this one — refusing"
            )));
        };
        let axis_point = axis_point.add(pull.scale(height_of(axis_point) * -1.0));
        // σ: +1 where the outward normal points away from the axis (a boss).
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
        let (um, vm) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        let at = face.surface.evaluate(um, vm).or_refuse(KernelStage::Classify, "evaluate")?;
        let normal = face.surface.normal(um, vm).or_refuse(KernelStage::Classify, "normal")?.scale(sign);
        let radial = at.sub(axis_point);
        let radial = radial.sub(pull.scale(radial.dot(pull)));
        let draft_sign = if normal.dot(radial) > 0.0 { 1.0 } else { -1.0 };
        let frame = frame_about(axis_point, pull)?;
        // Any height names the same cone; this one keeps both radii positive.
        let height = 0.5 * radius / tan.abs();
        carriers.insert(face_id, Carrier::Ruled {
            frame,
            rho0: radius,
            rho1: radius - draft_sign * tan * height,
            height,
        });
        drafts.insert(face_id, Draft::Cylinder { axis_point, radius, sign: draft_sign });
    }
    // The generator of a drafted face through `point`: the pull, leaned α
    // into the material.
    let generator = |face_id: u64, point: Vec3| -> Option<Vec3> {
        let outward = match drafts.get(&face_id)? {
            Draft::Plane { outward, .. } => *outward,
            Draft::Cylinder { axis_point, sign, .. } => {
                let radial = point.sub(*axis_point);
                radial.sub(pull.scale(radial.dot(pull))).normalized().ok()?.scale(*sign)
            }
        };
        pull.scale(cos).sub(outward.scale(sin)).normalized().ok()
    };

    // --- Topology maps --------------------------------------------------------
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge.entry(coedge.edge_id).or_default().push(face.id);
                }
            }
        }
    }
    let mut edges_at: HashMap<u64, Vec<usize>> = HashMap::default();
    for (index, edge) in solid.edges.iter().enumerate() {
        let uses = faces_of_edge.get(&edge.id).map(Vec::len).unwrap_or(0);
        let expected = if edge.degenerate { 1 } else { 2 };
        if uses != expected {
            return Err(KernelRefusal::input(KernelStage::Collect, "non_manifold", format!(
                "{OP}: edge {} is used {uses} times (non-manifold input)", edge.id
            )));
        }
        edges_at.entry(edge.start_vertex_id).or_default().push(index);
        if edge.end_vertex_id != edge.start_vertex_id {
            edges_at.entry(edge.end_vertex_id).or_default().push(index);
        }
    }
    let touches_draft = |edge: &EdgeRecord| faces_of_edge[&edge.id].iter().any(|f| selected.contains(f));

    // The carriers of every face that meets a drafted one: fixed faces keep
    // their own. One whose carrier has no closed form refuses here, by name.
    let mut neighbours: HashSet<u64> = HashSet::default();
    for edge in &solid.edges {
        if touches_draft(edge) {
            neighbours.extend(faces_of_edge[&edge.id].iter().copied());
        }
    }
    for vertex_edges in edges_at.values() {
        if vertex_edges.iter().any(|&e| touches_draft(&solid.edges[e])) {
            for &e in vertex_edges {
                neighbours.extend(faces_of_edge[&solid.edges[e].id].iter().copied());
            }
        }
    }
    let mut sorted_neighbours: Vec<u64> = neighbours.iter().copied().collect();
    sorted_neighbours.sort_unstable();
    for face_id in sorted_neighbours {
        if carriers.contains_key(&face_id) {
            continue;
        }
        let Some(carrier) = carrier_of(solid, &face_lookup, face_id, pull, plane_tolerance) else {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "draft_neighbour_kind", format!(
                "{OP}: face {face_id}, which meets a drafted face, is not a plane, a cylinder or \
                 a cone; the draft re-trims its neighbours by closed-form sections only — refusing"
            )));
        };
        carriers.insert(face_id, carrier);
    }

    // --- Re-solve every vertex the draft touches ----------------------------
    let mut new_vertex: HashMap<u64, Vec3> = HashMap::default();
    for vertex in &solid.vertices {
        let Some(vertex_edges) = edges_at.get(&vertex.id) else { continue };
        let mut at: Vec<u64> = vertex_edges
            .iter()
            .flat_map(|&e| faces_of_edge[&solid.edges[e].id].iter().copied())
            .collect();
        at.sort_unstable();
        at.dedup();
        if !at.iter().any(|f| selected.contains(f)) {
            continue;
        }
        let worst = |point: Vec3| at.iter().map(|f| carriers[f].residual(point).abs()).fold(0.0, f64::max);
        if worst(vertex.point) <= tolerance {
            new_vertex.insert(vertex.id, vertex.point); // on the neutral plane: it stays
            continue;
        }
        // Lines the corner must lie on: an edge both of whose faces keep their
        // carriers keeps its own line, and an edge both of whose faces are
        // drafted becomes the line through its neutral point.
        let mut lines: Vec<(Vec3, Vec3)> = Vec::new();
        for &e in vertex_edges {
            let edge = &solid.edges[e];
            if edge.degenerate {
                continue;
            }
            let Some((a, b)) = edge.curve.straight_segment(tolerance) else { continue };
            let uses = &faces_of_edge[&edge.id];
            let drafted = uses.iter().filter(|f| selected.contains(*f)).count();
            let direction = b.sub(a);
            if drafted == 0 {
                lines.push((a, direction));
            } else if drafted == uses.len() {
                let along = direction.dot(pull);
                if along.abs() <= 1e-9 * direction.length() {
                    continue; // a horizontal edge between drafted faces fixes nothing here
                }
                let crossing = a.add(direction.scale(-height_of(a) / along));
                let mut line_direction = None;
                if let (Some(Carrier::Plane { normal: na, .. }), Some(Carrier::Plane { normal: nb, .. })) =
                    (carriers.get(&uses[0]), carriers.get(&uses[1]))
                {
                    let cross = na.cross(*nb);
                    if cross.length() > 1e-9 {
                        line_direction = cross.normalized().ok();
                    }
                }
                let line_direction = match line_direction.or_else(|| generator(uses[0], crossing)) {
                    Some(direction) => direction,
                    None => continue,
                };
                let probe = crossing.add(line_direction.scale(scale));
                if uses.iter().all(|f| {
                    carriers[f].residual(crossing).abs() <= accept && carriers[f].residual(probe).abs() <= accept
                }) {
                    lines.push((crossing, line_direction));
                }
            }
        }
        let mut best: Option<(f64, Vec3)> = None;
        for &(point, direction) in &lines {
            for face_id in &at {
                let carrier = &carriers[face_id];
                let probe = point.add(direction.scale(scale));
                if carrier.residual(point).abs() <= tolerance && carrier.residual(probe).abs() <= tolerance {
                    continue; // the line lies on this carrier: it fixes nothing
                }
                if let Some(candidate) = line_meets(carrier, point, direction, vertex.point) {
                    let error = worst(candidate);
                    if best.map_or(true, |(e, _)| error < e) {
                        best = Some((error, candidate));
                    }
                }
            }
        }
        if best.map_or(true, |(e, _)| e > accept) {
            let planes: Option<Vec<(Vec3, Vec3)>> = at
                .iter()
                .map(|f| match &carriers[f] {
                    Carrier::Plane { origin, normal } => Some((*origin, *normal)),
                    _ => None,
                })
                .collect();
            if let Some(candidate) = planes.as_deref().filter(|p| p.len() >= 3).and_then(planes_meet) {
                let error = worst(candidate);
                if best.map_or(true, |(e, _)| error < e) {
                    best = Some((error, candidate));
                }
            }
        }
        match best {
            Some((error, point)) if error <= accept => {
                new_vertex.insert(vertex.id, point);
            }
            _ => {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "draft_corner", format!(
                    "{OP}: the corner at vertex {} where faces {at:?} meet has no closed-form \
                     solve on the drafted carriers (a corner must lie on an edge line both of \
                     whose faces are drafted or both fixed, or on three planes) — refusing",
                    vertex.id
                )));
            }
        }
    }

    // --- Rebuild every edge the draft touches -------------------------------
    let point_of = |id: u64| -> Vec3 {
        new_vertex
            .get(&id)
            .copied()
            .unwrap_or_else(|| solid.vertices.iter().find(|v| v.id == id).map(|v| v.point).unwrap_or_default())
    };
    let old_point = |id: u64| -> Vec3 { solid.vertices.iter().find(|v| v.id == id).map(|v| v.point).unwrap_or_default() };
    let mut result = solid.clone();
    let mut changed_edges: HashSet<u64> = HashSet::default();
    for edge in &mut result.edges {
        if edge.degenerate {
            continue;
        }
        let uses = faces_of_edge[&edge.id].clone();
        let moved_ends = [edge.start_vertex_id, edge.end_vertex_id]
            .iter()
            .any(|id| point_of(*id).sub(old_point(*id)).length() > tolerance);
        let drafted_here = uses.iter().any(|f| selected.contains(f));
        if !drafted_here && !moved_ends {
            continue;
        }
        let (start, end) = (point_of(edge.start_vertex_id), point_of(edge.end_vertex_id));
        let mid_t = (edge.t0 + edge.t1) * 0.5;
        let old_mid = edge.curve.evaluate(mid_t).or_refuse(KernelStage::Refine, "evaluate")?;
        let on_new = |point: Vec3| uses.iter().all(|f| carriers.get(f).map_or(false, |c| c.residual(point).abs() <= accept));
        if !moved_ends && on_new(old_mid) {
            // Its faces' new carriers still contain it (an edge on the neutral
            // plane, which every drafted carrier keeps): it stays as it is.
            let quarter = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * 0.25).or_refuse(KernelStage::Refine, "evaluate")?;
            if on_new(quarter) {
                continue;
            }
        }
        changed_edges.insert(edge.id);
        if let Some((a, b)) = edge.curve.straight_segment(tolerance) {
            let old_direction = b.sub(a);
            let new_direction = end.sub(start);
            if new_direction.length() <= tolerance || new_direction.dot(old_direction) <= 0.0 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, CONSUMES_FACE, format!(
                    "{OP}: the draft turns edge {} end over end (or collapses it) between faces \
                     {uses:?}: it consumes a face, a change of topology the in-place draft does \
                     not make — refusing",
                    edge.id
                )));
            }
            let curve = make_line(start, end).or_refuse(KernelStage::Sew, "make_line")?;
            let mid = curve.evaluate(0.5).or_refuse(KernelStage::Sew, "evaluate")?;
            if !on_new(mid) {
                return Err(KernelRefusal::internal(KernelStage::Validate, "draft_edge_off_carrier", format!(
                    "{OP}: the rebuilt straight edge {} leaves the carriers of faces {uses:?} — refusing",
                    edge.id
                )));
            }
            edge.curve = curve;
            edge.t0 = 0.0;
            edge.t1 = 1.0;
            continue;
        }
        // A curved edge: the section of a cylinder or cone by a plane.
        let pair = match (carriers.get(&uses[0]), carriers.get(&uses[1])) {
            (Some(Carrier::Plane { origin, normal }), Some(Carrier::Ruled { frame, rho0, rho1, height }))
            | (Some(Carrier::Ruled { frame, rho0, rho1, height }), Some(Carrier::Plane { origin, normal })) => {
                Some((*origin, *normal, frame.clone(), *rho0, *rho1, *height))
            }
            _ => None,
        };
        let Some((origin, normal, frame, rho0, rho1, height)) = pair else {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "draft_curved_edge", format!(
                "{OP}: curved edge {} between faces {uses:?} is not the section of a cylinder or \
                 cone by a plane, the only curved edge the draft re-intersects — refusing",
                edge.id
            )));
        };
        let sweep = swept_azimuth(&frame, &edge.curve, edge.t0, edge.t1)?;
        let azimuth = |point: Vec3| {
            let w = point.sub(frame.origin);
            let a = w.dot(frame.y_axis).atan2(w.dot(frame.x_axis));
            if a < 0.0 { a + std::f64::consts::TAU } else { a }
        };
        let closed = edge.start_vertex_id == edge.end_vertex_id;
        let sweep = if closed { std::f64::consts::TAU.copysign(sweep) } else { sweep };
        let (from, magnitude) = if sweep > 0.0 { (azimuth(start), sweep) } else { (azimuth(end), -sweep) };
        // The sweep between the NEW ends: the old one corrected by how far each
        // end turned about the axis (a section keeps its corners' azimuths
        // only when they ride rulings of this carrier).
        let magnitude = if closed {
            magnitude
        } else {
            let (a0, a1) = if sweep > 0.0 { (azimuth(start), azimuth(end)) } else { (azimuth(end), azimuth(start)) };
            let mut span = a1 - a0;
            while span <= 0.0 {
                span += std::f64::consts::TAU;
            }
            if (span - magnitude).abs() > std::f64::consts::PI {
                magnitude
            } else {
                span
            }
        };
        let mut curve = plane_ruled_section_arc(origin, normal, &frame, rho0, rho1, height, from, magnitude, tolerance)
            .map_err(|error| {
                KernelRefusal::unsupported(KernelStage::Refine, "draft_section", format!(
                    "{OP}: edge {} between faces {uses:?}: {error}", edge.id
                ))
            })?;
        if sweep < 0.0 {
            curve = curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
        }
        let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let (c0, c1) = (
            curve.evaluate(d0).or_refuse(KernelStage::Sew, "evaluate")?,
            curve.evaluate(d1).or_refuse(KernelStage::Sew, "evaluate")?,
        );
        if c0.sub(start).length() > accept || c1.sub(end).length() > accept {
            return Err(KernelRefusal::internal(KernelStage::Validate, "draft_section_ends", format!(
                "{OP}: the re-intersected section of edge {} misses its corners by {:.3e} and \
                 {:.3e} — refusing",
                edge.id,
                c0.sub(start).length(),
                c1.sub(end).length()
            )));
        }
        edge.curve = curve;
        edge.t0 = d0;
        edge.t1 = d1;
    }
    for vertex in &mut result.vertices {
        if let Some(point) = new_vertex.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    let final_edges: HashMap<u64, EdgeRecord> = result.edges.iter().map(|edge| (edge.id, edge.clone())).collect();

    // --- Re-trim every face the draft touches -------------------------------
    let mut ruled_retrims = Vec::new();
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            let touched = face
                .loops
                .iter()
                .flat_map(|l| l.coedges.iter())
                .any(|c| changed_edges.contains(&c.edge_id));
            match drafts.get(&face.id) {
                Some(Draft::Plane { old, rotation, .. }) => {
                    let turned = Plane {
                        origin: rotation.point(old.origin),
                        u_dir: super::face_rotate::rotate_direction(rotation, old.u_dir),
                        v_dir: super::face_rotate::rotate_direction(rotation, old.v_dir),
                        normal: super::face_rotate::rotate_direction(rotation, old.normal),
                    };
                    retrim_planar_face(face, &turned, &final_edges, scale, OP)?;
                }
                Some(Draft::Cylinder { axis_point, radius, sign }) => {
                    let surface = drafted_cylinder(&face.surface, pull, *axis_point, *radius, *sign, tan, face.id)?;
                    let carrier = &carriers[&face.id];
                    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
                    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
                    for i in 0..=4 {
                        for j in 0..=4 {
                            let p = surface
                                .evaluate(u0 + (u1 - u0) * i as f64 / 4.0, v0 + (v1 - v0) * j as f64 / 4.0)
                                .or_refuse(KernelStage::Refine, "evaluate")?;
                            if carrier.residual(p).abs() > accept {
                                return Err(KernelRefusal::internal(KernelStage::Validate, "draft_cone_net", format!(
                                    "{OP}: the drafted net of face {} leaves its cone by {:.3e} — refusing",
                                    face.id,
                                    carrier.residual(p).abs()
                                )));
                            }
                        }
                    }
                    rebuild_loop_pcurves(face, &final_edges, &surface, tolerance, OP)?;
                    face.surface = surface;
                }
                None if touched => match &carriers.get(&face.id) {
                    Some(Carrier::Plane { .. }) => {
                        let plane = plane_of_surface(&face.surface, plane_tolerance, OP)?;
                        retrim_planar_face(face, &plane, &final_edges, scale, OP)?;
                    }
                    Some(Carrier::Ruled { .. }) => ruled_retrims.push(face.id),
                    None => {
                        return Err(KernelRefusal::internal(KernelStage::Sew, "draft_unclassified", format!(
                            "{OP}: face {} lost an edge to the draft but has no carrier — refusing",
                            face.id
                        )));
                    }
                },
                None => {}
            }
        }
    }
    for face_id in ruled_retrims {
        retrim_ruled_face(&mut result, face_id, &final_edges, tolerance, OP)?;
    }
    result.mass_properties_cache = Default::default();
    accept_result(solid, &result)?;
    Ok(result)
}

/// The cylinder's own net, made the drafted cone: every control point at
/// height `h` above the neutral plane is scaled about the axis by
/// `(r₀ − σ·h·tan α)/r₀`. Exact for a net whose rows along the circle each lie
/// at one height (an extruded arc, a revolved line): each row is a circle
/// scaled about its own centre, and the straight blend between rows is then the
/// cone's ruling. Verified on the result by the caller.
fn drafted_cylinder(
    surface: &NurbsSurface,
    pull: Vec3,
    axis_point: Vec3,
    radius: f64,
    sign: f64,
    tan: f64,
    face_id: u64,
) -> Result<NurbsSurface, KernelRefusal> {
    let mut rows = surface.control_points.clone();
    for row in &mut rows {
        for control in row.iter_mut() {
            let point = control.point().or_refuse(KernelStage::Refine, "point")?;
            let w = point.sub(axis_point);
            let h = w.dot(pull);
            let q = w.sub(pull.scale(h));
            let r = radius - sign * h * tan;
            if r <= 0.0 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "draft_cone_apex", format!(
                    "{OP}: the cone drafted from face {face_id} reaches its apex within the face's \
                     carrier (radius {r:.6e} at height {h:.6e}) — refusing"
                )));
            }
            let moved = axis_point.add(pull.scale(h)).add(q.scale(r / radius));
            *control = Vec4::from_point(moved, control.w);
        }
    }
    NurbsSurface::new(
        surface.degree_u,
        surface.degree_v,
        surface.knots_u.clone(),
        surface.knots_v.clone(),
        rows,
    )
    .or_refuse(KernelStage::Refine, "drafted_cone")
}

/// The same acceptance floor `rotate_faces` holds a re-trimmed body to.
fn accept_result(input: &BrepSolid, result: &BrepSolid) -> Result<(), KernelRefusal> {
    let report = result.validate_detailed(&crate::KernelTolerances::for_solid(result, 1e-7));
    if !report.issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "validate", format!(
            "{OP}: the drafted solid failed validation: {:?}", report.issues
        )));
    }
    let crossings = crate::loop_self_crossings(result);
    if !crossings.unreadable.is_empty() || crossings.loops == 0 {
        return Err(KernelRefusal::internal(KernelStage::Validate, "crossing_scan_unreadable", format!(
            "{OP}: the loop self-crossing scan could not read the drafted body ({:?}) — refusing",
            crossings.unreadable
        )));
    }
    if crossings.is_flagged() {
        return Err(KernelRefusal::unsupported(KernelStage::Validate, CONSUMES_FACE, format!(
            "{OP}: the draft makes a face's own loop cross itself ({}) — refusing",
            crossings.summary().unwrap_or_default()
        )));
    }
    if let (Ok(before), Ok(after)) =
        (crate::signed_volume_against(input, 0.0), crate::signed_volume_against(result, 0.0))
    {
        if before * after <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, CONSUMES_FACE, format!(
                "{OP}: the draft inverts the solid (signed volume changed sign) — refusing"
            )));
        }
    }
    if !report.wire_warnings.is_empty() {
        let inherited = input.validate_detailed(&crate::KernelTolerances::for_solid(input, 1e-7)).wire_warnings;
        if report.wire_warnings.len() > inherited.len() {
            return Err(KernelRefusal::internal(KernelStage::Validate, "chart_winding", format!(
                "{OP}: the draft leaves a face whose (u,v) chart is left-handed against its own \
                 normal ({}) — refusing",
                report.wire_warnings.iter().map(|w| w.message.as_str()).collect::<Vec<_>>().join("; ")
            )));
        }
    }
    Ok(())
}

/// THE CONVEX ROUTE. A convex polyhedron is the intersection of its faces'
/// half-spaces, so its draft is the intersection of the same half-spaces with
/// the drafted faces' planes turned — whatever faces that leaves, which is
/// what lets a steep draft consume a face (a box's top drafted away into a hip
/// roof). Taken only for a body all of whose faces are planes and every vertex
/// of which lies inside every face plane (that is convexity, for a closed
/// polyhedral body), and only when every selected face is a plane.
fn draft_convex_polyhedron(
    solid: &BrepSolid,
    face_ids: &[u64],
    pull_direction: Vec3,
    neutral_point: Vec3,
    angle_radians: f64,
) -> Result<BrepSolid, String> {
    let pull = pull_direction.normalized()?;
    let scale = solid_model_scale(solid);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let (sin, cos) = angle_radians.sin_cos();
    let mut half_spaces: Vec<(Vec3, Vec3)> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            let plane = plane_of_surface(&face.surface, plane_tolerance, OP)
                .map_err(|_| format!("face {} is not planar", face.id))?;
            let outward = plane.normal.scale(outward_sign(face));
            if selected.contains(&face.id) {
                let height = plane.origin.sub(neutral_point).dot(pull);
                let on_hinge = plane.origin.sub(pull.scale(height));
                half_spaces.push((on_hinge, outward.scale(cos).add(pull.scale(sin)).normalized()?));
            } else {
                half_spaces.push((plane.origin, outward));
            }
        }
    }
    if solid.shells.len() != 1 {
        return Err("the body has more than one shell".into());
    }
    for shell in &solid.shells {
        for face in &shell.faces {
            let plane = plane_of_surface(&face.surface, plane_tolerance, OP).map_err(|e| e.to_string())?;
            let outward = plane.normal.scale(outward_sign(face));
            if solid.vertices.iter().any(|v| v.point.sub(plane.origin).dot(outward) > plane_tolerance * 10.0) {
                return Err(format!("the body is not convex (a vertex lies outside face {})", face.id));
            }
        }
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for v in &solid.vertices {
        low = Vec3::new(low.x.min(v.point.x), low.y.min(v.point.y), low.z.min(v.point.z));
        high = Vec3::new(high.x.max(v.point.x), high.y.max(v.point.y), high.z.max(v.point.z));
    }
    let extent = high.sub(low).length();
    let margin = extent * (1.0 + angle_radians.tan().abs()) + 1.0;
    let corner = low.sub(Vec3::new(margin, margin, margin));
    let size = high.sub(low).add(Vec3::new(2.0 * margin, 2.0 * margin, 2.0 * margin));
    let mut body = crate::make_box_brep(corner, size.x, size.y, size.z)?;
    for (origin, normal) in &half_spaces {
        match crate::split_solid_by_plane(&body, *origin, *normal) {
            Ok((inside, _)) => body = inside,
            Err(error) if error.contains("does not intersect") => {
                if body.vertices.iter().all(|v| v.point.sub(*origin).dot(*normal) >= -plane_tolerance) {
                    return Err("the drafted half-spaces leave nothing".into());
                }
            }
            Err(error) => return Err(error),
        }
    }
    // The enclosing box must not be part of the answer: an unbounded draft
    // would leave one of its faces standing.
    let inner_low = corner.add(Vec3::new(plane_tolerance, plane_tolerance, plane_tolerance).scale(1e3));
    let inner_high = corner.add(size).sub(Vec3::new(plane_tolerance, plane_tolerance, plane_tolerance).scale(1e3));
    if body.vertices.iter().any(|v| {
        v.point.x <= inner_low.x || v.point.y <= inner_low.y || v.point.z <= inner_low.z
            || v.point.x >= inner_high.x || v.point.y >= inner_high.y || v.point.z >= inner_high.z
    }) {
        return Err("the drafted half-spaces do not bound a solid".into());
    }
    accept_result(solid, &body).map_err(|e| e.to_string())?;
    Ok(body)
}

/// A PARTING-LINE draft: the body is split by the neutral plane through
/// `parting_point` square to `pull_direction`, the pieces of the selected faces
/// on the `+pull` side are drafted for `+pull` and those on the other side for
/// `−pull` — each leaning `angle_radians` away from the parting line, as the
/// two halves of a two-piece mould pull apart — and the two sides are united
/// again along the parting plane. The selected faces must each be a plane
/// parallel to the pull or a cylinder about it, as for [`draft_faces`].
pub fn draft_faces_parted(
    solid: &BrepSolid,
    face_ids: &[u64],
    pull_direction: Vec3,
    parting_point: Vec3,
    angle_radians: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let pull = check_inputs(face_ids, pull_direction, parting_point, angle_radians)?;
    let scale = solid_model_scale(solid);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let mut face_lookup: HashMap<u64, (usize, usize)> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            face_lookup.entry(face.id).or_insert((shell_index, face_index));
        }
    }
    let mut wanted = Vec::new();
    for &face_id in face_ids {
        if !face_lookup.contains_key(&face_id) {
            return Err(KernelRefusal::input(KernelStage::Collect, "face_id", format!("{OP}: no face with id {face_id}")));
        }
        let carrier = carrier_of(solid, &face_lookup, face_id, pull, plane_tolerance).ok_or_else(|| {
            KernelRefusal::unsupported(KernelStage::Classify, "draft_carrier_kind", format!(
                "{OP}: face {face_id} is neither a plane parallel to the pull direction nor a \
                 cylinder whose axis is the pull direction — refusing"
            ))
        })?;
        wanted.push(carrier);
    }
    let (below, above) = crate::split_solid_by_plane(solid, parting_point, pull).map_err(|error| {
        KernelRefusal::unsupported(KernelStage::Classify, "draft_parting_plane", format!(
            "{OP}: the parting plane does not divide the body ({error}) — refusing"
        ))
    })?;
    let same = |a: &Carrier, b: &Carrier| -> bool {
        match (a, b) {
            (Carrier::Plane { origin: oa, normal: na }, Carrier::Plane { origin: ob, normal: nb }) => {
                na.cross(*nb).length() <= 1e-9 && ob.sub(*oa).dot(*na).abs() <= plane_tolerance
            }
            (
                Carrier::Ruled { frame: fa, rho0: ra0, rho1: ra1, height: ha },
                Carrier::Ruled { frame: fb, rho0: rb0, rho1: rb1, height: hb },
            ) => {
                let offset = fb.origin.sub(fa.origin);
                fa.axis.cross(fb.axis).length() <= 1e-9
                    && offset.sub(fa.axis.scale(offset.dot(fa.axis))).length() <= plane_tolerance
                    && ((ra1 - ra0) / ha).abs() <= 1e-9
                    && ((rb1 - rb0) / hb).abs() <= 1e-9
                    && (ra0 - rb0).abs() <= plane_tolerance
            }
            _ => false,
        }
    };
    let mut halves = Vec::new();
    for (piece, side_pull) in [(above, pull), (below, pull.scale(-1.0))] {
        let mut lookup: HashMap<u64, (usize, usize)> = HashMap::default();
        for (shell_index, shell) in piece.shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                lookup.entry(face.id).or_insert((shell_index, face_index));
            }
        }
        let mut ids: Vec<u64> = lookup
            .keys()
            .copied()
            .filter(|&id| {
                carrier_of(&piece, &lookup, id, pull, plane_tolerance)
                    .map_or(false, |c| wanted.iter().any(|w| same(w, &c)))
            })
            .collect();
        ids.sort_unstable();
        if ids.is_empty() {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "draft_parting_side", format!(
                "{OP}: no selected face reaches one side of the parting plane — refusing"
            )));
        }
        halves.push(draft_faces(&piece, &ids, side_pull, parting_point, angle_radians)?);
    }
    crate::boolean_operation(
        &halves[0],
        &halves[1],
        crate::BooleanOperation::Union,
        &crate::BooleanOptions::default(),
    )
}

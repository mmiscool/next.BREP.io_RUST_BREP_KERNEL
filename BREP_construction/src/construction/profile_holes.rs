//! Planar profiles with holes (an outer loop and inner loops), extruded or revolved exactly: every wall is
//! the profile curve itself carried by the motion (an extrusion or a surface of revolution), every cap a
//! plane holding the outer loop round the holes. Built directly, not as a boolean per hole.

use crate::gears::ShellBuilder;
use crate::{BrepSolid, KernelRefusal, KernelStage, NurbsCurve, Vec3, Vec4};

fn refuse(code: &str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, code, message)
}

/// A profile's plane: origin, in-plane axes x and y, normal n = x × y.
#[derive(Clone, Copy)]
struct Plane {
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    n: Vec3,
}

impl Plane {
    fn coords(&self, p: Vec3) -> [f64; 2] {
        let d = p.sub(self.origin);
        [d.dot(self.x), d.dot(self.y)]
    }

    fn flipped(self) -> Plane {
        let n = self.n.scale(-1.0);
        Plane { origin: self.origin, x: self.x, y: n.cross(self.x), n }
    }
}

/// Points round a loop (each curve's start and interior samples, not its end).
fn loop_samples(curves: &[NurbsCurve]) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut points = Vec::with_capacity(32 * curves.len());
    for curve in curves {
        let [t0, t1] = curve.domain().map_err(|e| refuse("profile_curve", e))?;
        for i in 0..32 {
            points.push(curve.evaluate(t0 + (t1 - t0) * i as f64 / 32.0).map_err(|e| refuse("profile_curve", e))?);
        }
    }
    Ok(points)
}

fn signed_area(points: &[[f64; 2]]) -> f64 {
    (0..points.len()).map(|i| {
        let (p, q) = (points[i], points[(i + 1) % points.len()]);
        p[0] * q[1] - q[0] * p[1]
    }).sum::<f64>() / 2.0
}

fn inside(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let mut crossings = 0;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        if (a[1] > point[1]) != (b[1] > point[1]) {
            let x = a[0] + (point[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if x > point[0] {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}

/// Check the loops (each closed, end to start, within `tolerance`), find their common plane (Newell's normal of
/// the outer loop) and refuse a profile out of it or a hole not inside the outer loop. Returns the plane and
/// each loop's signed area about its normal (outer first).
fn profile_plane(outer: &[NurbsCurve], holes: &[Vec<NurbsCurve>]) -> Result<(Plane, Vec<f64>), KernelRefusal> {
    if outer.is_empty() || holes.iter().any(|h| h.is_empty()) {
        return Err(refuse("profile_empty", "A profile loop needs at least one curve.".into()));
    }
    let loops: Vec<&[NurbsCurve]> = std::iter::once(outer).chain(holes.iter().map(|h| h.as_slice())).collect();
    let samples: Vec<Vec<Vec3>> = loops.iter().map(|l| loop_samples(l)).collect::<Result<_, _>>()?;
    let scale = samples.iter().flatten().map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs())).fold(1.0_f64, f64::max);
    let tolerance = 1e-7 * scale;
    for (index, curves) in loops.iter().enumerate() {
        for (i, curve) in curves.iter().enumerate() {
            let next = &curves[(i + 1) % curves.len()];
            let [_, t1] = curve.domain().map_err(|e| refuse("profile_curve", e))?;
            let [s0, _] = next.domain().map_err(|e| refuse("profile_curve", e))?;
            let (end, start) = (curve.evaluate(t1).map_err(|e| refuse("profile_curve", e))?, next.evaluate(s0).map_err(|e| refuse("profile_curve", e))?);
            if end.sub(start).length() > tolerance.max(1e-9) * 10.0 {
                return Err(refuse("profile_open", format!("Profile loop {index} is not closed: curve {i} ends {:.3e} from the next curve's start.", end.sub(start).length())));
            }
        }
    }
    let outer_points = &samples[0];
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..outer_points.len() {
        let (p, q) = (outer_points[i], outer_points[(i + 1) % outer_points.len()]);
        normal = normal.add(Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y)));
    }
    let n = normal.normalized().map_err(|_| refuse("profile_degenerate", "The outer loop encloses no area.".into()))?;
    let origin = outer_points[0];
    let x = n.perpendicular().map_err(|e| refuse("profile_degenerate", e))?;
    let plane = Plane { origin, x, y: n.cross(x), n };
    for points in &samples {
        for p in points {
            let off = p.sub(origin).dot(n).abs();
            if off > 1e-6 * scale {
                return Err(refuse("profile_not_planar", format!("The profile is not planar: a point lies {off:.3e} off its plane.")));
            }
        }
    }
    let flat: Vec<Vec<[f64; 2]>> = samples.iter().map(|points| points.iter().map(|p| plane.coords(*p)).collect()).collect();
    let areas: Vec<f64> = flat.iter().map(|points| signed_area(points)).collect();
    for (index, points) in flat.iter().enumerate().skip(1) {
        if !points.iter().all(|p| inside(*p, &flat[0])) {
            return Err(refuse("hole_outside", format!("Hole {} is not inside the outer loop.", index - 1)));
        }
    }
    Ok((plane, areas))
}

fn translated(curve: &NurbsCurve, by: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    NurbsCurve::new(
        curve.degree,
        curve.knots.clone(),
        curve.control_points.iter().map(|p| Vec4 { x: p.x + p.w * by.x, y: p.y + p.w * by.y, z: p.z + p.w * by.z, w: p.w }).collect(),
    )
    .map_err(|e| refuse("profile_curve", e))
}

fn finish(shell: ShellBuilder, genus: usize, what: &str) -> Result<BrepSolid, KernelRefusal> {
    let mut shell = shell;
    let id = shell.id();
    let shell_id = shell.id();
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id,
        vertices: shell.vertices,
        edges: shell.edges,
        shells: vec![crate::sweep_topology::ShellRecord { id: shell_id, faces: shell.faces }],
        genus: genus as _,
        kind: crate::BodyKind::Solid,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "validate", format!("{what} is not valid: {issues:?}")));
    }
    Ok(solid)
}

/// Extrude a planar profile with holes along `direction` by `distance`: an outer loop and any number of
/// inner loops (holes, inside it and apart from each other), each a closed chain of curves (end to start) in
/// either winding. Walls are the curves extruded (exact), the two caps planes with the outer loop round the
/// holes, every hole a through-hole. The direction may be oblique to the profile but not in its plane. Faces
/// are named `W<i>` (outer walls), `H<h>_<i>` (hole walls), `B` (the profile's cap) and `T` (the far cap).
pub fn extrude_profile_with_holes(outer: &[NurbsCurve], holes: &[Vec<NurbsCurve>], direction: Vec3, distance: f64) -> Result<BrepSolid, KernelRefusal> {
    use crate::sweep_topology::{curve_to_plane_parameters, make_plane, parameter_line};
    let travel = direction.normalized().map_err(|_| refuse("direction", "The extrusion direction must be nonzero.".into()))?.scale(distance);
    let (mut plane, mut areas) = profile_plane(outer, holes)?;
    let along = travel.dot(plane.n);
    if !(distance.is_finite() && along.abs() > 1e-9 * travel.length().max(1e-300)) {
        return Err(refuse("direction", "The extrusion must leave the profile's plane (a nonzero distance, not along the plane).".into()));
    }
    if along < 0.0 {
        plane = plane.flipped();
        areas.iter_mut().for_each(|a| *a = -*a);
    }
    let line = |a: [f64; 2], b: [f64; 2]| parameter_line(a[0], a[1], b[0], b[1]).map_err(|e| refuse("profile_pcurve", e));
    let point_of = |p: Vec4| Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w);
    let mut shell = ShellBuilder { vertices: Vec::new(), edges: Vec::new(), faces: Vec::new(), next: 0 };
    let loops: Vec<&[NurbsCurve]> = std::iter::once(outer).chain(holes.iter().map(|h| h.as_slice())).collect();
    let mut bottoms: Vec<Vec<u64>> = Vec::with_capacity(loops.len());
    let mut tops: Vec<Vec<u64>> = Vec::with_capacity(loops.len());
    for (index, curves) in loops.iter().enumerate() {
        let count = curves.len();
        let (mut b, mut t, mut axial) = (Vec::with_capacity(count), Vec::with_capacity(count), Vec::with_capacity(count));
        for curve in curves.iter() {
            let p = point_of(curve.control_points[0]);
            let (vb, vt) = (shell.vertex(p), shell.vertex(p.add(travel)));
            axial.push(shell.edge(crate::make_line(p, p.add(travel)).map_err(|e| refuse("profile_edge", e))?, vb, vt)?);
            b.push(vb);
            t.push(vt);
        }
        let (mut bottom_e, mut top_e) = (Vec::with_capacity(count), Vec::with_capacity(count));
        for (i, curve) in curves.iter().enumerate() {
            let next = (i + 1) % count;
            bottom_e.push(shell.edge(curve.clone(), b[i], b[next])?);
            top_e.push(shell.edge(translated(curve, travel)?, t[i], t[next])?);
        }
        // A wall's normal (tangent × travel) points out of a loop running counter-clockwise about the travel;
        // the solid's outside is out of the outer loop and into a hole.
        let counter_clockwise = areas[index] > 0.0;
        let same_sense = counter_clockwise == (index == 0);
        for (i, curve) in curves.iter().enumerate() {
            let next = (i + 1) % count;
            let [u0, u1] = curve.domain().map_err(|e| refuse("profile_curve", e))?;
            let surface = crate::make_extrusion(curve, travel).map_err(|e| refuse("profile_surface", e))?;
            let loops = vec![vec![
                (bottom_e[i], true, line([u0, 0.0], [u1, 0.0])?),
                (axial[next], true, line([u1, 0.0], [u1, 1.0])?),
                (top_e[i], false, line([u0, 1.0], [u1, 1.0])?),
                (axial[i], false, line([u0, 0.0], [u0, 1.0])?),
            ]];
            let name = if index == 0 { format!("W{i}") } else { format!("H{}_{i}", index - 1) };
            shell.face(surface, same_sense, loops, name)?;
        }
        bottoms.push(bottom_e);
        tops.push(top_e);
    }
    // The caps: planes over the profile's extent, x × y = n (the travel side).
    let flat: Vec<[f64; 2]> = loop_samples(outer)?.iter().map(|p| plane.coords(*p)).collect();
    let (lo, hi) = flat.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]));
    let pad = 0.1 * (hi[0] - lo[0]).max(hi[1] - lo[1]) + 1e-6;
    let corner = plane.origin.add(plane.x.scale(lo[0] - pad)).add(plane.y.scale(lo[1] - pad));
    let (size_x, size_y) = (hi[0] - lo[0] + 2.0 * pad, hi[1] - lo[1] + 2.0 * pad);
    for (shift, edges, name, same_sense) in [(Vec3::new(0.0, 0.0, 0.0), &bottoms, "B", false), (travel, &tops, "T", true)] {
        let origin = corner.add(shift);
        let mut face_loops = Vec::with_capacity(edges.len());
        for loop_edges in edges.iter() {
            let mut coedges = Vec::with_capacity(loop_edges.len());
            for &edge in loop_edges {
                let pcurve = curve_to_plane_parameters(shell.curve(edge), origin, plane.x, plane.y).map_err(|e| refuse("profile_pcurve", e))?;
                coedges.push((edge, true, pcurve));
            }
            face_loops.push(coedges);
        }
        let surface = make_plane(origin, plane.x, plane.y, size_x, size_y).map_err(|e| refuse("profile_plane", e))?;
        shell.face(surface, same_sense, face_loops, name.into())?;
    }
    finish(shell, holes.len(), "extruded profile")
}

/// Revolve a planar profile with holes about the axis through `axis_point` along `axis_direction` by `angle`
/// (radians, up to a full turn): the outer loop revolved (as [`crate::revolve_profile_brep`] revolves one loop,
/// touching the axis if it must), less each hole revolved. A full turn makes each hole a closed void inside the
/// solid; a part turn opens the holes through the end caps. Holes must lie off the axis and inside the outer
/// loop. Every face is the profile curve revolved or a cap plane: exact.
pub fn revolve_profile_with_holes(outer: &[NurbsCurve], holes: &[Vec<NurbsCurve>], axis_point: Vec3, axis_direction: Vec3, angle: f64) -> Result<BrepSolid, KernelRefusal> {
    let (plane, _) = profile_plane(outer, holes)?;
    let axis = axis_direction.normalized().map_err(|_| refuse("axis", "The revolve axis must be nonzero.".into()))?;
    if axis.dot(plane.n).abs() > 1e-9 || axis_point.sub(plane.origin).dot(plane.n).abs() > 1e-6 * (1.0 + axis_point.length()) {
        return Err(refuse("axis", "The revolve axis must lie in the profile's plane.".into()));
    }
    let revolve = |curves: &[NurbsCurve]| crate::revolve_profile_brep(curves, axis_point, axis, angle).map_err(|e| refuse("revolve", e));
    let mut solid = revolve(outer)?;
    if holes.is_empty() {
        return Ok(solid);
    }
    for (index, hole) in holes.iter().enumerate() {
        // Off the axis: the hole revolves to a body of its own (a ring, or a part ring).
        let near = loop_samples(hole)?.iter().map(|p| { let d = p.sub(axis_point); d.sub(axis.scale(d.dot(axis))).length() }).fold(f64::INFINITY, f64::min);
        if !(near > 1e-9 * (1.0 + axis_point.length())) {
            return Err(refuse("hole_on_axis", format!("Hole {index} touches the revolve axis.")));
        }
        let cutter = revolve(hole)?;
        solid = crate::boolean_operation_nary(&[solid, cutter], crate::BooleanOperation::Subtract)?;
    }
    Ok(solid)
}


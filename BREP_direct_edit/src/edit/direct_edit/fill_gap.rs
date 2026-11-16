//! Deleting faces whose gap closes by FILLING: the last lane, after every
//! other has refused (`BREP_DELETE_FILL_GAP`, default on; `=0` leaves the
//! refusal).
//!
//! The heals before this one re-solve edges: a strip between two walls, a
//! patch capped where its rim lies whole in a face, a band, a corner blend, a
//! ring. Three plain selections fit none of them, and each refused:
//!
//! * an L-block's STEP, riser and tread together: the faces left beside them
//!   (the far side and the far end) have never shared an edge, so no pair of
//!   neighbours re-intersects along anything the selection touched;
//! * a half-round NOTCH through a block's side: one face, whose two straight
//!   edges lie in the same plane, so the "two walls" are one wall;
//! * an open SLOT, two flats and a half cylinder: the same, three faces deep.
//!
//! What "the faces round them run on until they close the gap" means for all
//! three is the same sentence: the material missing between the selection and
//! the planes of the faces that border it comes back. So that is what is
//! built. The gap's region is the box that holds the selected faces, cut to
//! the inner side of every bordering face's plane; the body united with it is
//! the answer IF, and only if, it is checked to be one:
//!
//! * every face that borders the selection is planar, and none of their planes
//!   cuts the box (this first slice builds only the box itself, each side that
//!   no bordering plane holds pushed a little way on into the body);
//! * after the union every sampled point of every selected face is strictly
//!   inside the body: the selection is gone, not merely covered at an edge;
//! * no face of the box is left showing except where it lies in a bordering
//!   face's plane: the fill has added no surface the body did not already have.
//!
//! A selection that removes material when healed (a boss, a concave round)
//! fails the second test, and one whose gap opens onto anything but its
//! bordering planes fails the third; both keep the refusal they came with.
//!
//! A bordering face may also be a CYLINDER, a CONE or a SPHERE: a flat milled
//! along a shaft, whose one face has the shaft's cylinder on both long sides,
//! or an open keyway, whose walls and floor have it all round. Its "inner
//! side" is the solid its whole surface bounds — the round bar the cylinder is
//! the side of, or everything outside it for a bore — so the fill is the box
//! cut to that too, and the box first reaches out to the surface over the gap
//! (each witness's foot on it). The same two checks then decide: the selection
//! is gone, and the fill shows nowhere but on a bordering face's surface.
use super::*;
use crate::classification::PolygonClass;
use crate::analytic_surface::AnalyticSurface;
use crate::{
    boolean_operation, classify_point, make_box_brep, make_cone_brep, make_cylinder_brep, make_sphere_brep, parameter_point_in_face,
    BooleanOperation, BooleanOptions, PointClass, RevolutionFrame, Vec2,
};

/// A curved face bordering the selection: the closed surface it lies on, and
/// whether the material is inside it (a bar's side) or outside it (a bore).
struct CurvedBorder {
    frame: RevolutionFrame,
    /// `Some((rho0, rho1, height))` for a cylinder or cone, `None` for a sphere.
    ruled: Option<(f64, f64, f64)>,
    radius: f64,
    material_inside: bool,
}

impl CurvedBorder {
    /// The point's signed distance outside the surface along its radial
    /// direction (negative inside), and that surface's foot of the point.
    fn reach(&self, point: Vec3) -> Option<(f64, Vec3)> {
        let d = point.sub(self.frame.origin);
        let axial = d.dot(self.frame.axis);
        match self.ruled {
            Some((rho0, rho1, height)) => {
                let rho = rho0 + (rho1 - rho0) * axial / height;
                let radial = d.sub(self.frame.axis.scale(axial));
                let distance = radial.length();
                let toward = radial.normalized().ok()?;
                let foot = self.frame.origin.add(self.frame.axis.scale(axial)).add(toward.scale(rho));
                Some((distance - rho, foot))
            }
            None => {
                let toward = d.normalized().ok()?;
                Some((d.length() - self.radius, self.frame.origin.add(toward.scale(self.radius))))
            }
        }
    }

    /// Whether `point` is on the material's side of the surface or on it.
    fn holds(&self, point: Vec3, tolerance: f64) -> bool {
        match self.reach(point) {
            Some((outside, _)) if self.material_inside => outside <= tolerance,
            Some((outside, _)) => outside >= -tolerance,
            None => self.material_inside,
        }
    }

    /// The solid the whole surface bounds, long enough along its axis to
    /// span every point of `corners` with `margin` to spare.
    fn solid(&self, corners: &[Vec3], margin: f64) -> Option<BrepSolid> {
        match self.ruled {
            None => make_sphere_brep(self.frame.origin, self.radius, self.frame.axis).ok(),
            Some((rho0, rho1, height)) => {
                let axial = |p: &Vec3| p.sub(self.frame.origin).dot(self.frame.axis);
                let low = corners.iter().map(axial).fold(f64::INFINITY, f64::min) - margin;
                let high = corners.iter().map(axial).fold(f64::NEG_INFINITY, f64::max) + margin;
                let rho = |t: f64| rho0 + (rho1 - rho0) * t / height;
                // The slack never carries a cone past its apex: an end the
                // margin would take there stops short of it instead, as long
                // as the box itself (`corners`) stays clear of the apex.
                let (mut low, mut high) = (low, high);
                if rho1 != rho0 {
                    let apex = -rho0 * height / (rho1 - rho0);
                    let short = 1e-3 * margin.max(1e-9);
                    let (inner_low, inner_high) = (low + margin, high - margin);
                    if apex > inner_low - short && apex < inner_high + short {
                        return None;
                    }
                    // Halfway from the box to the apex: a tool ending in a
                    // needle point is no tool a boolean takes reliably.
                    if apex >= inner_high {
                        high = high.min(inner_high + 0.5 * (apex - inner_high));
                    }
                    if apex <= inner_low {
                        low = low.max(inner_low - 0.5 * (inner_low - apex));
                    }
                }
                let (r_low, r_high) = (rho(low), rho(high));
                if !(r_low > 0.0 && r_high > 0.0) {
                    return None;
                }
                let base = self.frame.origin.add(self.frame.axis.scale(low));
                if (r_low - r_high).abs() <= 1e-12 * r_low {
                    make_cylinder_brep(base, self.frame.axis, r_low, high - low).ok()
                } else {
                    make_cone_brep(base, self.frame.axis, r_low, r_high, high - low).ok()
                }
            }
        }
    }
}

/// The last point, within `tolerance`, of the ray from `start` along
/// `direction` before it leaves the region `held` reports, within `span`;
/// `None` if it never does.
fn exit_along(held: &dyn Fn(Vec3) -> bool, start: Vec3, direction: Vec3, span: f64, tolerance: f64) -> Option<Vec3> {
    const STEPS: usize = 512;
    let at = |t: f64| start.add(direction.scale(t));
    let mut inside = 0.0;
    for step in 1..=STEPS {
        let t = span * step as f64 / STEPS as f64;
        if held(at(t)) {
            inside = t;
            continue;
        }
        let (mut a, mut b) = (inside, t);
        while b - a > tolerance {
            let middle = 0.5 * (a + b);
            if held(at(middle)) {
                a = middle;
            } else {
                b = middle;
            }
        }
        // The last point still on the material's side: a box carried past a
        // bordering plane by even the tolerance would leave a sliver.
        return Some(at(a));
    }
    None
}

fn fill_gap_enabled() -> bool {
    switch!("BREP_DELETE_FILL_GAP", true)
}

/// `solid` with `face_ids` deleted by filling the gap they bound, or `None`
/// where that is not provably the heal (see the module docs).
fn declined(step: u32) -> Option<BrepSolid> {
    if crate::debug_flag!("BREP_DEBUG_FILL_GAP") {
        eprintln!("delete fill gap: declined at step {step}");
    }
    None
}

pub fn heal_by_filling_the_gap(solid: &BrepSolid, face_ids: &[u64]) -> Option<BrepSolid> {
    if !fill_gap_enabled() || face_ids.is_empty() {
        return declined(1);
    }
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let mut selected: HashSet<u64> = face_ids.iter().copied().collect();
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let edges_of = |face: &FaceRecord| -> Vec<u64> { face.loops.iter().flat_map(|l| &l.coedges).map(|c| c.edge_id).collect() };
    // A wall the kernel keeps as two faces split at a seam (a half cylinder cut
    // from a block comes as two quarter turns) is one face to whoever picked it:
    // a curved face that borders the selection on the SAME surface as a selected
    // face is taken with it.
    let on_surface_of = |face: &FaceRecord, other: &FaceRecord| -> bool {
        let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else { return false };
        [(0.25, 0.25), (0.5, 0.5), (0.75, 0.25), (0.25, 0.75), (0.75, 0.75)].iter().all(|(a, b)| {
            face.surface
                .evaluate(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b)
                .ok()
                .and_then(|point| crate::project_point_to_surface(&other.surface, point).ok())
                .is_some_and(|foot| foot.distance <= tolerance)
        })
    };
    loop {
        let rim: HashSet<u64> = faces.iter().filter(|f| selected.contains(&f.id)).flat_map(|f| edges_of(f)).collect();
        let more: Vec<u64> = faces
            .iter()
            .filter(|f| !selected.contains(&f.id) && f.surface.analytic().is_some_and(|a| !matches!(a, crate::analytic_surface::AnalyticSurface::Plane { .. })))
            .filter(|f| edges_of(f).iter().any(|edge| rim.contains(edge)))
            .filter(|f| faces.iter().any(|g| selected.contains(&g.id) && on_surface_of(f, g)))
            .map(|f| f.id)
            .collect();
        if more.is_empty() {
            break;
        }
        selected.extend(more);
    }
    let rim: HashSet<u64> = faces.iter().filter(|f| selected.contains(&f.id)).flat_map(|f| edges_of(f)).collect();
    if rim.is_empty() {
        return declined(2);
    }

    // The planes of the faces that border the selection, as (point, outward normal).
    let mut planes: Vec<(Vec3, Vec3)> = Vec::new();
    let mut curved: Vec<CurvedBorder> = Vec::new();
    for face in faces.iter().filter(|f| !selected.contains(&f.id)) {
        if !edges_of(face).iter().any(|edge| rim.contains(edge)) {
            continue;
        }
        let (frame, ruled, radius) = match face.surface.analytic() {
            Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
                let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
                if !face.same_sense {
                    normal = normal.scale(-1.0);
                }
                planes.push((*origin, normal));
                continue;
            }
            Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) if *height > 0.0 => {
                (frame.clone(), Some((*rho0, *rho1, *height)), 0.0)
            }
            Some(AnalyticSurface::Sphere { frame, radius }) => (frame.clone(), None, *radius),
            _ => return declined(3),
        };
        // Which side the material is on: the face's outward normal at the
        // middle of an edge it shares with the selection (a point of the face
        // however it is trimmed), against the direction away from the
        // surface's axis (or centre).
        let mut side = None;
        if let Some(edge) = solid
            .edges
            .iter()
            .find(|edge| !edge.degenerate && rim.contains(&edge.id) && edges_of(face).contains(&edge.id))
        {
            let middle = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).ok()?;
            let foot = crate::project_point_to_surface(&face.surface, middle).ok()?;
            let mut normal = face.surface.normal(foot.u, foot.v).ok()?;
            if !face.same_sense {
                normal = normal.scale(-1.0);
            }
            let d = foot.point.sub(frame.origin);
            let away = if ruled.is_some() { d.sub(frame.axis.scale(d.dot(frame.axis))) } else { d };
            side = Some(normal.dot(away) > 0.0);
        }
        let Some(material_inside) = side else {
            return declined(3);
        };
        curved.push(CurvedBorder { frame, ruled, radius, material_inside });
    }
    if planes.is_empty() && curved.is_empty() {
        return declined(4);
    }

    // The box that holds the selection: its edges at 33 stations each, and the
    // points of each face's own region on a 9 × 9 grid of its surface (a curved
    // face bulges past its edges). The same grid points are the witnesses that
    // the face is gone.
    let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    let take = |low: &mut [f64; 3], high: &mut [f64; 3], p: Vec3| {
        for (axis, value) in [p.x, p.y, p.z].into_iter().enumerate() {
            low[axis] = low[axis].min(value);
            high[axis] = high[axis].max(value);
        }
    };
    for edge in solid.edges.iter().filter(|edge| rim.contains(&edge.id) && !edge.degenerate) {
        for k in 0..=32 {
            take(&mut low, &mut high, edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 32.0).ok()?);
        }
    }
    let mut witnesses: Vec<Vec3> = Vec::new();
    // Each witness's outward normal: the direction the missing material lies in.
    let mut outward: Vec<Vec3> = Vec::new();
    for face in faces.iter().filter(|f| selected.contains(&f.id)) {
        let ([u0, u1], [v0, v1]) = (face.surface.domain_u().ok()?, face.surface.domain_v().ok()?);
        let before = witnesses.len();
        for i in 1..=9 {
            for j in 1..=9 {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / 10.0, v0 + (v1 - v0) * j as f64 / 10.0);
                if matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9).ok()?, PolygonClass::Inside) {
                    witnesses.push(face.surface.evaluate(u, v).ok()?);
                    let mut normal = face.surface.normal(u, v).ok()?;
                    if !face.same_sense {
                        normal = normal.scale(-1.0);
                    }
                    outward.push(normal);
                }
            }
        }
        if witnesses.len() == before {
            return declined(5);
        }
    }
    let span = 4.0 * scale.max(tolerance);
    for (&point, &normal) in witnesses.iter().zip(&outward) {
        take(&mut low, &mut high, point);
        // Out along the normal, the gap runs until a curved border's surface
        // closes it (a flat through a shaft's axis: up to the far side).
        // The march stops at the first border of either kind, so a ray
        // never carries the box past a bordering plane.
        if !curved.is_empty() {
            let held = |p: Vec3| {
                curved.iter().all(|border| border.holds(p, 0.0))
                    && planes.iter().all(|(origin, n)| p.sub(*origin).dot(*n) <= 0.0)
            };
            if let Some(end) = exit_along(&held, point, normal, span, tolerance) {
                take(&mut low, &mut high, end);
            }
        }
        // Over the gap the box reaches out to each curved border's surface.
        // (A witness on the axis, or at a sphere's centre, has no one foot;
        // the witnesses round it reach the surface for it.)
        for border in &curved {
            if let Some((_, foot)) = border.reach(point) {
                take(&mut low, &mut high, foot);
            }
        }
    }
    if (0..3).any(|axis| !(high[axis] - low[axis] > 1e-6 * scale)) {
        return declined(6);
    }
    // This slice builds only the box: no bordering plane may cut into it.
    let outside = |p: Vec3, plane: &(Vec3, Vec3)| p.sub(plane.0).dot(plane.1);
    let cut_by_a_plane = |low: [f64; 3], high: [f64; 3]| {
        (0..8).any(|k| {
            let p = Vec3::new(
                if k & 1 == 0 { low[0] } else { high[0] },
                if k & 2 == 0 { low[1] } else { high[1] },
                if k & 4 == 0 { low[2] } else { high[2] },
            );
            planes.iter().any(|plane| outside(p, plane) > tolerance)
        })
    };
    if cut_by_a_plane(low, high) {
        return declined(7);
    }
    // Each side of the box that no bordering plane holds is pushed a little way
    // on, into the body: a curved face touches its own box along a line (a half
    // cylinder's crown), and a fill that only touches the body there has not
    // joined it. The last test below still requires every such side to lie
    // strictly inside the body.
    let reach = 0.05 * (0..3).map(|axis| high[axis] - low[axis]).fold(0.0f64, f64::max);
    for axis in 0..3 {
        let mut wider = low;
        wider[axis] -= reach;
        if !cut_by_a_plane(wider, high) {
            low = wider;
        }
        let mut wider = high;
        wider[axis] += reach;
        if !cut_by_a_plane(low, wider) {
            high = wider;
        }
    }
    let size = [high[0] - low[0], high[1] - low[1], high[2] - low[2]];

    let mut fill = make_box_brep(Vec3::new(low[0], low[1], low[2]), size[0], size[1], size[2]).ok()?;
    let corners: Vec<Vec3> = (0..8)
        .map(|k| {
            Vec3::new(
                if k & 1 == 0 { low[0] } else { high[0] },
                if k & 2 == 0 { low[1] } else { high[1] },
                if k & 4 == 0 { low[2] } else { high[2] },
            )
        })
        .collect();
    // The tool only has to span the box: a little slack, so that a cone is
    // not carried past its apex beyond the region the fill can reach.
    let margin = 0.05 * size.iter().fold(0.0f64, |a, b| a.max(*b));
    for border in &curved {
        let tool = border.solid(&corners, margin)?;
        let operation = if border.material_inside { BooleanOperation::Intersect } else { BooleanOperation::Subtract };
        fill = boolean_operation(&fill, &tool, operation, &BooleanOptions::default()).ok()?;
        if fill.shells.iter().all(|shell| shell.faces.is_empty()) {
            return declined(11);
        }
    }
    let mut united = boolean_operation(solid, &fill, BooleanOperation::Union, &BooleanOptions::default()).ok()?;
    // The fill's side on a curved border is that border's surface run on: the
    // union can leave it a face of its own beside the one it continues, so
    // the faces on each border's surface are merged into one (where the merge
    // declines, they stand as they are, exact).
    for border in &curved {
        let group: Vec<u64> = united
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| {
                face.surface.domain_u().ok().zip(face.surface.domain_v().ok()).is_some_and(|([u0, u1], [v0, v1])| {
                        [(0.3, 0.3), (0.5, 0.5), (0.7, 0.6)].iter().all(|(a, b)| {
                            face.surface
                                .evaluate(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b)
                                .ok()
                                .and_then(|point| border.reach(point))
                                .is_some_and(|(outside, _)| outside.abs() <= tolerance)
                        })
                    })
            })
            .map(|face| face.id)
            .collect();
        if group.len() > 1 {
            if let Ok((merged, _)) = crate::merge_faces_within(&united, tolerance, &group) {
                united = merged;
            }
        }
    }
    if !united.validate().is_empty() {
        return declined(8);
    }
    let strictly_inside = |p: Vec3| classify_point(p, &united, tolerance).is_ok_and(|c| c.class == PointClass::In);
    // The selection is gone.
    if !witnesses.iter().all(|&p| strictly_inside(p)) {
        return declined(9);
    }
    // The fill shows nowhere but in a bordering plane.
    for axis in 0..3 {
        let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
        for end in [low[axis], high[axis]] {
            for i in 1..=5 {
                for j in 1..=5 {
                    let mut q = [0.0; 3];
                    q[axis] = end;
                    q[a] = low[a] + size[a] * i as f64 / 6.0;
                    q[b] = low[b] + size[b] * j as f64 / 6.0;
                    let q = Vec3::new(q[0], q[1], q[2]);
                    if planes.iter().any(|plane| outside(q, plane).abs() <= tolerance) {
                        continue;
                    }
                    // A point a curved border cuts off is not in the fill, and
                    // one on a border's surface is that face run on.
                    if curved.iter().any(|border| {
                        !border.holds(q, tolerance) || border.reach(q).is_some_and(|(o, _)| o.abs() <= tolerance)
                    }) {
                        continue;
                    }
                    if !strictly_inside(q) {
                        return declined(10);
                    }
                }
            }
        }
    }
    // The fill covered nothing else: every face that neither was selected nor
    // borders the selection is still on the united body's surface. A box that
    // reaches out to a curved border can hold another feature of the body (a
    // groove beside a keyway's end) that the box-face samples above miss, and
    // filling it is no part of this delete. Each face is sampled on a grid over
    // its OWN trim (the box of its loops' pcurves, not its carrier's whole
    // domain, so a narrow fragment of a long carrier is sampled across its
    // width) and along every one of its edges; a face that yields no sample,
    // or a sample the classifier cannot place, declines the fill.
    let on_surface = |point: Vec3| matches!(classify_point(point, &united, tolerance), Ok(c) if c.class == PointClass::On);
    for face in faces.iter().filter(|f| !selected.contains(&f.id) && !edges_of(f).iter().any(|edge| rim.contains(edge))) {
        let (mut u_low, mut u_high, mut v_low, mut v_high) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            for control in &coedge.pcurve.control_points {
                if control.w.abs() > 0.0 {
                    let (u, v) = (control.x / control.w, control.y / control.w);
                    u_low = u_low.min(u);
                    u_high = u_high.max(u);
                    v_low = v_low.min(v);
                    v_high = v_high.max(v);
                }
            }
        }
        if !(u_low < u_high && v_low < v_high) {
            return declined(12);
        }
        let mut sampled = 0usize;
        for i in 1..=9 {
            for j in 1..=9 {
                let (u, v) = (u_low + (u_high - u_low) * i as f64 / 10.0, v_low + (v_high - v_low) * j as f64 / 10.0);
                if !matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9).ok()?, PolygonClass::Inside) {
                    continue;
                }
                sampled += 1;
                if !on_surface(face.surface.evaluate(u, v).ok()?) {
                    return declined(12);
                }
            }
        }
        for edge in solid.edges.iter().filter(|edge| !edge.degenerate && edges_of(face).contains(&edge.id)) {
            for k in 1..=8 {
                sampled += 1;
                if !on_surface(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 9.0).ok()?) {
                    return declined(12);
                }
            }
        }
        if sampled == 0 {
            return declined(12);
        }
    }
    // And exactly: the material the fill adds (the fill less the body) may
    // meet the body only where the selection was, and may show only on the
    // borders' surfaces. Every face of it is placed by a point inside its own
    // trim; a face lying anywhere else (on a groove beside the gap, however
    // narrow, wherever the samples above fell) declines the fill.
    let added = boolean_operation(&fill, solid, BooleanOperation::Subtract, &BooleanOptions::default()).ok()?;
    let on = |surface: &crate::NurbsSurface, point: Vec3| {
        crate::project_point_to_surface(surface, point).is_ok_and(|foot| foot.distance <= tolerance)
    };
    for face in added.shells.iter().flat_map(|shell| &shell.faces) {
        let (mut u_low, mut u_high, mut v_low, mut v_high) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            for control in &coedge.pcurve.control_points {
                if control.w.abs() > 0.0 {
                    let (u, v) = (control.x / control.w, control.y / control.w);
                    u_low = u_low.min(u);
                    u_high = u_high.max(u);
                    v_low = v_low.min(v);
                    v_high = v_high.max(v);
                }
            }
        }
        let mut inside = None;
        'grid: for i in 1..=15 {
            for j in 1..=15 {
                let (u, v) = (u_low + (u_high - u_low) * i as f64 / 16.0, v_low + (v_high - v_low) * j as f64 / 16.0);
                if matches!(parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9), Ok(PolygonClass::Inside)) {
                    inside = face.surface.evaluate(u, v).ok();
                    break 'grid;
                }
            }
        }
        let Some(point) = inside else {
            return declined(13);
        };
        let allowed = faces.iter().any(|f| selected.contains(&f.id) && on(&f.surface, point))
            || planes.iter().any(|plane| outside(point, plane).abs() <= tolerance)
            || curved.iter().any(|border| border.reach(point).is_some_and(|(o, _)| o.abs() <= tolerance));
        if !allowed {
            return declined(13);
        }
    }
    Some(united)
}

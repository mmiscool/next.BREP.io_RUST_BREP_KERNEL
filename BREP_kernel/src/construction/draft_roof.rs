//! A drafted extrusion carried past the taper that closes its section: the
//! roof.
//!
//! [`crate::extrude_profile_brep_draft`] builds the drafted walls up to a far
//! section that is still the profile's offset, and refuses by name once the
//! taper has closed part of it. For a profile of straight sides the solid past
//! that taper is still plain: every wall is a plane leaning in over its side,
//! and the solid is what lies under all of them between the profile's plane and
//! the far one. Where the profile is convex that is the common part of the
//! half-spaces under the walls. Where it has a notch (a run of sides that turn
//! the other way, between two corners of its hull), what lies OUTSIDE the walls
//! of the notch's sides, all of them at once, is cut out of that: the notch
//! grown as its walls lean. Both are convex, so the solid is the convex body
//! of the hull's walls less one convex body for each notch, cut by the kernel's
//! boolean; every face is a plane and nothing is fitted.
use crate::{BooleanOperation, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, RefusalClass, Vec3};

/// The prefix of every refusal of this operation.
pub const DRAFT_ROOF_REFUSAL: &str = "draftRoof";

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{DRAFT_ROOF_REFUSAL}: {message}"))
}

/// The solid of `profile` extruded by `distance` along `direction` with its
/// walls drafted inward by `draft_angle_rad`, whether or not the taper closes
/// the section before the far end.
///
/// Where [`crate::extrude_profile_brep_draft`] builds, this is its solid,
/// unchanged. Where it refuses because the taper closes the section
/// (`draft_section_closes`), the solid under the leaning walls is built: a
/// rectangle drafted past half its width ends in a ridge, an L in a ridge over
/// each arm. Its faces are, in order: the walls, those of each side of the
/// profile together and in the profile's order (sides that run on in one line
/// have one wall, told as the first of them); the face in the profile's plane;
/// and the face in the far plane, if the walls leave one. The extrusion runs
/// square to the profile's plane.
///
/// Refuses, by name: a profile with a side that is not straight
/// (`roof_side`), a taper that is refused as closing but does not lean inward
/// (`roof_taper`), a notch that has a notch of its own (`roof_nested_notch`;
/// its sides must all turn the same way), and a result with a face that joins
/// no wall standing on its own side (`roof_wall`: a wall would have cut
/// material that is not under it).
pub fn extrude_profile_brep_draft_roof(profile: &[NurbsCurve], direction: Vec3, distance: f64, draft_angle_rad: f64) -> Result<BrepSolid, KernelRefusal> {
    match crate::extrude_profile_brep_draft(profile, direction, distance, draft_angle_rad, None) {
        Err(error) if matches!(&error.class, RefusalClass::InvalidInput { what } if what == "draft_section_closes") => {}
        other => return other,
    }
    let stage = KernelStage::Collect;
    // the corners, each side straight
    let mut corners: Vec<Vec3> = Vec::with_capacity(profile.len());
    for (index, side) in profile.iter().enumerate() {
        let [t0, t1] = side.domain().or_refuse(stage, "draftRoof.domain")?;
        let (from, to) = (side.evaluate(t0).or_refuse(stage, "draftRoof.end")?, side.evaluate(t1).or_refuse(stage, "draftRoof.end")?);
        let chord = to.sub(from);
        for k in 1..8 {
            let point = side.evaluate(t0 + (t1 - t0) * k as f64 / 8.0).or_refuse(stage, "draftRoof.point")?;
            if point.sub(from).cross(chord).length() > 1e-9 * chord.length().max(1e-300) * chord.length() {
                return Err(refuse("roof_side", format!("side {index} of the profile is not straight; the roof is built over straight sides only")));
            }
        }
        corners.push(from);
    }
    // The extrusion runs square to the profile's own plane, on the side `direction` and the
    // distance's sign give (the drafted extrusion takes a direction within a few degrees of that
    // normal; the profile is not moved onto another plane).
    let normal = crate::polygon::newell_normal(&corners).normalized().or_refuse(stage, "draftRoof.plane")?;
    let toward = direction.dot(normal) * distance;
    if toward == 0.0 {
        return Err(refuse("roof_taper", "the extrusion's direction lies in the profile's plane".into()));
    }
    let up = normal.scale(toward.signum());
    // the section's inset for each unit of height, by the drafted extrusion's own law
    // (distance · tan(draft) over the whole height): it closes only where that is positive
    let slope = distance.signum() * draft_angle_rad.tan();
    if !(slope > 0.0 && slope.is_finite()) {
        return Err(refuse("roof_taper", "the section is refused as closing, but its taper does not lean inward".into()));
    }
    let origin = corners[0];
    let x = corners[1].sub(origin);
    let x = x.sub(up.scale(x.dot(up))).normalized().or_refuse(stage, "draftRoof.frame")?;
    let y = up.cross(x);
    let all: Vec<[f64; 2]> = corners.iter().map(|corner| [corner.sub(origin).dot(x), corner.sub(origin).dot(y)]).collect();
    // Sides that run on in one line are one wall: only the corners where the outline turns are
    // kept, and the wall is told as the first of its sides.
    let reach = all.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f64::max);
    let kept: Vec<usize> = (0..all.len())
        .filter(|&k| {
            let (a, b, c) = (all[(k + all.len() - 1) % all.len()], all[k], all[(k + 1) % all.len()]);
            let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
            (u[0] * v[1] - u[1] * v[0]).abs() > 1e-12 * reach * reach || u[0] * v[0] + u[1] * v[1] < 0.0
        })
        .collect();
    if kept.len() < 3 {
        return Err(refuse("roof_side", "the profile has fewer than three corners".into()));
    }
    let sides = profile.len();
    let count = kept.len();
    let mut flat: Vec<[f64; 2]> = kept.iter().map(|&k| all[k]).collect();
    // Everything below is built in the profile's own frame (its first corner at the origin, its
    // plane z = 0, the extrusion along +z) and carried to its place at the end, so that no
    // tolerance depends on where the profile lies. The walls have closed by the height at which
    // the inset reaches the profile's extent, so the far plane is never taken above twice that:
    // a far end beyond the roof's top changes nothing.
    let placement = crate::AffineTransform {
        elements: [x.x, y.x, up.x, origin.x, x.y, y.y, up.y, origin.y, x.z, y.z, up.z, origin.z, 0.0, 0.0, 0.0, 1.0],
    };
    let (x, y, up, origin) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 0.0));
    let extent = flat.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f64::max);
    let height = distance.abs().min(2.0 * extent / slope);
    let size = extent.max(height);
    // counter-clockwise seen from the far side; `source[i]`: the profile's side that side i is
    let area: f64 = (0..count).map(|i| flat[i][0] * flat[(i + 1) % count][1] - flat[(i + 1) % count][0] * flat[i][1]).sum();
    let mut source: Vec<usize> = kept.clone();
    if area < 0.0 {
        flat.reverse();
        // corner i is now corner count − 1 − i; side i runs from it to the next: the side count − 2 − i
        source = (0..count).map(|i| kept[(2 * count - 2 - i) % count]).collect();
    }
    let point = |p: [f64; 2], z: f64| origin.add(x.scale(p[0])).add(y.scale(p[1])).add(up.scale(z));
    // the wall over side i, as n · p ≤ d
    let wall = |i: usize| -> (Vec3, f64) {
        let (a, b) = (flat[i], flat[(i + 1) % count]);
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let out = x.scale((b[1] - a[1]) / length).add(y.scale(-(b[0] - a[0]) / length));
        let normal = out.add(up.scale(slope)).scale(1.0 / (1.0 + slope * slope).sqrt());
        (normal, normal.dot(point(a, 0.0)))
    };
    // the hull, keeping corners that lie on its sides
    let turn = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
    let mut order: Vec<usize> = (0..count).collect();
    order.sort_by(|&p, &q| flat[p][0].total_cmp(&flat[q][0]).then(flat[p][1].total_cmp(&flat[q][1])));
    let mut hull: Vec<usize> = Vec::new();
    for pass in 0..2 {
        let start = hull.len();
        let list: Vec<usize> = if pass == 0 { order.clone() } else { order.iter().rev().copied().collect() };
        for index in list {
            while hull.len() >= start + 2 && turn(flat[hull[hull.len() - 2]], flat[hull[hull.len() - 1]], flat[index]) < -1e-12 * size * size {
                hull.pop();
            }
            hull.push(index);
        }
        hull.pop();
    }
    // corners on a hull side that the hull's walk picked out of the profile's order are dropped
    let on_hull: Vec<usize> = { let mut kept = hull.clone(); kept.sort_unstable(); kept.dedup(); kept };
    let far = 4.0 * size + 4.0 * (slope.abs() * height);
    let mut body_planes: Vec<(Vec3, f64)> = vec![(up.scale(-1.0), -up.dot(origin)), (up, up.dot(origin) + height)];
    let mut notches: Vec<Vec<(Vec3, f64)>> = Vec::new();
    for (k, &from) in on_hull.iter().enumerate() {
        let to = on_hull[(k + 1) % on_hull.len()];
        if (from + 1) % count == to {
            body_planes.push(wall(from));
            continue;
        }
        // a notch: the sides from `from` round to `to`; the hull's own side across its mouth is upright
        let (a, b) = (flat[from], flat[to]);
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let out = x.scale((b[1] - a[1]) / length).add(y.scale(-(b[0] - a[0]) / length));
        body_planes.push((out, out.dot(point(a, 0.0))));
        let mut planes: Vec<(Vec3, f64)> = Vec::new();
        let mut i = from;
        while i != to {
            let next = (i + 1) % count;
            if next != to && turn(flat[i], flat[next], flat[(next + 1) % count]) > -1e-12 * size * size {
                return Err(refuse("roof_nested_notch", format!("the notch between corners {from} and {to} of the profile has a corner that turns back in (a notch within a notch); the roof is built where every notch's sides turn one way")));
            }
            let (normal, d) = wall(i);
            planes.push((normal.scale(-1.0), -d));
            i = next;
        }
        // closed off far outside the solid, so that the notch's body is bounded
        let centre = point([0.0, 0.0], 0.5 * height);
        for axis in [x, y, up] {
            planes.push((axis, axis.dot(centre) + far));
            planes.push((axis.scale(-1.0), -axis.dot(centre) + far));
        }
        notches.push(planes);
    }
    let solid_of = |planes: &[(Vec3, f64)]| -> Result<BrepSolid, KernelRefusal> {
        let (points, faces) = crate::solid_sweep_caps::polyhedron_of_planes(planes)?;
        let polygons: Vec<Vec<Vec3>> = faces.iter().map(|face| face.iter().map(|&c| points[c]).collect()).collect();
        crate::solid_from_polygons(&polygons)
    };
    let mut operands = vec![solid_of(&body_planes)?];
    for planes in &notches {
        operands.push(solid_of(planes)?);
    }
    let roof = if operands.len() == 1 { operands.pop().expect("one") } else { crate::boolean_operation_nary(&operands, BooleanOperation::Subtract)? };

    // the faces by the plane they lie in
    // by the footprint's size: a tall roof over a small footprint has walls no farther apart for its height
    let tolerance = 1e-7 * extent + 1e-12 * size;
    let on_plane = |face: &crate::FaceRecord, plane: &(Vec3, f64)| -> bool {
        let Ok(mut vertices) = face_points(&roof, face) else { return false };
        vertices.truncate(64);
        !vertices.is_empty() && vertices.iter().all(|p| (plane.0.dot(*p) - plane.1).abs() <= tolerance)
    };
    let faces = &roof.shells.first().ok_or_else(|| refuse("roof_empty", "the walls leave no solid".into()))?.faces;
    let mut taken = vec![false; faces.len()];
    let mut ordered: Vec<usize> = Vec::with_capacity(faces.len());
    let mut by_source: Vec<Vec<usize>> = vec![Vec::new(); sides];
    for i in 0..count {
        let plane = wall(i);
        let mine: Vec<usize> = (0..faces.len()).filter(|&f| !taken[f] && on_plane(&faces[f], &plane)).collect();
        // one piece, standing on its own side
        let foot = point([0.5 * (flat[i][0] + flat[(i + 1) % count][0]), 0.5 * (flat[i][1] + flat[(i + 1) % count][1])], 0.0);
        let stands = |f: usize| face_edges(&roof, &faces[f]).iter().any(|edge| crate::project_point_to_curve(&edge.curve, foot).map(|hit| hit.distance <= tolerance).unwrap_or(false));
        let Some(first) = mine.iter().copied().find(|&f| stands(f)) else {
            return Err(refuse("roof_wall", format!("the wall over side {} of the profile does not stand on its side", source[i])));
        };
        let mut reached = vec![first];
        let mut grew = true;
        while grew {
            grew = false;
            for &f in &mine {
                if reached.contains(&f) {
                    continue;
                }
                let edges: Vec<u64> = faces[f].loops.iter().flat_map(|l| l.coedges.iter().map(|c| c.edge_id)).collect();
                if reached.iter().any(|&g| faces[g].loops.iter().flat_map(|l| &l.coedges).any(|c| edges.contains(&c.edge_id))) {
                    reached.push(f);
                    grew = true;
                }
            }
        }
        // faces in this plane that do not join it are another side's (two sides in one line share
        // their wall's plane), or nobody's: what is left over at the end is refused
        reached.sort_unstable();
        for &f in &reached {
            taken[f] = true;
        }
        by_source[source[i]] = reached;
    }
    for mine in by_source {
        ordered.extend(mine);
    }
    for plane in &body_planes[..2] {
        let caps: Vec<usize> = (0..faces.len()).filter(|&f| !taken[f] && on_plane(&faces[f], plane)).collect();
        for &f in &caps {
            taken[f] = true;
        }
        ordered.extend(caps);
    }
    if taken.iter().any(|t| !t) {
        return Err(refuse("roof_wall", "the solid has a face that joins no wall standing on its own side: a wall would cut material that is not under it".into()));
    }
    let mut roof = roof.clone();
    let shell = &mut roof.shells[0];
    let all = std::mem::take(&mut shell.faces);
    let mut slots: Vec<Option<crate::FaceRecord>> = all.into_iter().map(Some).collect();
    shell.faces = ordered.into_iter().map(|f| slots[f].take().expect("each face once")).collect();
    crate::transform_brep(&roof, placement, false).or_refuse(KernelStage::Fragment, "draftRoof.place")
}

fn face_edges<'a>(solid: &'a BrepSolid, face: &crate::FaceRecord) -> Vec<&'a crate::topology::EdgeRecord> {
    let ids: Vec<u64> = face.loops.iter().flat_map(|l| l.coedges.iter().map(|c| c.edge_id)).collect();
    solid.edges.iter().filter(|edge| ids.contains(&edge.id)).collect()
}

/// The ends of a face's edges.
fn face_points(solid: &BrepSolid, face: &crate::FaceRecord) -> Result<Vec<Vec3>, String> {
    let mut points = Vec::new();
    for edge in face_edges(solid, face) {
        points.push(edge.curve.evaluate(edge.t0)?);
        points.push(edge.curve.evaluate(edge.t1)?);
        points.push(edge.curve.evaluate(0.5 * (edge.t0 + edge.t1))?);
    }
    Ok(points)
}


//! Fillet beads along a planar spline seam.
//!
//! A swept spline bead stands off its joint faces by the sweep's tolerance,
//! so it could never pass the exact checks. This bead is built so that both
//! contacts are exact: it is the loft on three rails,
//!
//! - the root, the seam `C` itself,
//! - the wall toe, `C` moved `l1` along the wall's rise `d` (a translate, so it
//!   lies exactly on a wall that is `C` extruded along `d`),
//! - the base toe, a spline through `C` moved `l2` across the base plane at
//!   every station (its poles in the plane, so it lies exactly on the base),
//!
//! with the end triangles as sections. The loft's walls are Coons patches
//! with straight cross-sections: the wall contact is the extrusion itself and
//! the base contact a patch of the base plane. Only the base leg's length
//! carries the toe spline's fit, read back and bounded.
use super::fit::{self, NoFit};
use crate::feature_pipeline::SceneMap;
use crate::{NurbsCurve, PointClass, PolygonClass, Vec3};

/// A fitted spline bead: the body and the parts it joins (wall, base).
pub(super) struct Bead {
    pub body: crate::BrepSolid,
    pub joints: [String; 2],
}

struct Face {
    solid: String,
    handle: u32,
    face_id: u64,
}

fn cartesian(curve: &NurbsCurve) -> Vec<Vec3> {
    curve.control_points.iter().map(|p| {
        let w = if p.w.abs() > 1e-300 { p.w } else { 1.0 };
        Vec3::new(p.x / w, p.y / w, p.z / w)
    }).collect()
}

fn on_face(face: &Face, point: Vec3, tolerance: f64) -> Result<bool, String> {
    crate::with_registered_solid_str(face.handle, |body| {
        let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face.face_id)
            .ok_or("a joint face is no longer resident")?;
        Ok(matches!(fit::face_class(record, point, tolerance)?, Some(PolygonClass::Inside | PolygonClass::Boundary)))
    })
}

fn outside(scene: &SceneMap, solid: &str, point: Vec3) -> Result<bool, String> {
    let handle = scene.solids.get(solid).copied().ok_or_else(|| format!("part '{solid}' is not resident"))?;
    crate::with_registered_solid_str(handle, |body| Ok(crate::classify_point(point, body, 1e-9)?.class == PointClass::Out))
}

/// The curve moved by `shift(t)`, fitted span by span between the curve's
/// distinct knots (the move is only once differentiable there, through the
/// tangent): `count` intervals per span, a cubic interpolant at the curve's
/// own parameters on each, joined end to end (position-continuous at the
/// knots, where each piece's end interpolates the same point).
fn moved(curve: &NurbsCurve, count: usize, shift: &dyn Fn(f64) -> Result<Vec3, String>) -> Result<NurbsCurve, String> {
    let ends = span_ends(curve)?;
    let mut joined: Option<NurbsCurve> = None;
    for span in ends.windows(2) {
        let (a, b) = (span[0], span[1]);
        let mut points = Vec::with_capacity(count + 1);
        let mut parameters = Vec::with_capacity(count + 1);
        for k in 0..=count {
            let t = a + (b - a) * k as f64 / count as f64;
            points.push(curve.evaluate(t)?.add(shift(t)?));
            parameters.push(t);
        }
        let piece = crate::interpolate_curve(&points, 3, &parameters)?;
        joined = Some(match joined {
            None => piece,
            Some(head) => {
                // Knots: the head's without its last, the piece's without its
                // first `degree + 1`; poles: the piece's first is the head's last.
                let degree = head.degree;
                let mut knots = head.knots[..head.knots.len() - 1].to_vec();
                knots.extend_from_slice(&piece.knots[degree + 1..]);
                let mut poles = head.control_points.clone();
                poles.extend_from_slice(&piece.control_points[1..]);
                NurbsCurve::new(degree, knots, poles)?
            }
        });
    }
    joined.ok_or_else(|| "the spline has no span".to_string())
}

/// The curve's domain cut at its distinct interior knots.
fn span_ends(curve: &NurbsCurve) -> Result<Vec<f64>, String> {
    let [t0, t1] = curve.domain()?;
    let mut breaks: Vec<f64> = curve.knots.iter().copied().filter(|k| *k > t0 && *k < t1).collect();
    breaks.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (t1 - t0));
    let mut ends = vec![t0];
    ends.extend(breaks);
    ends.push(t1);
    Ok(ends)
}

/// The curve translated by `by` (exact: every pole moved).
fn translated(curve: &NurbsCurve, by: Vec3) -> NurbsCurve {
    let mut moved = curve.clone();
    for pole in &mut moved.control_points {
        pole.x += by.x * pole.w;
        pole.y += by.y * pole.w;
        pole.z += by.z * pole.w;
    }
    moved
}

/// The bead along the spline seam `curve`, legs `[on the wall, on the base]`.
/// `picks` are the user's joint faces (none, or the two). `exclude` is the
/// weld's own id.
pub(super) fn bead(scene: &SceneMap, id: &str, curve: &NurbsCurve, picks: &[String], legs: [f64; 2], exclude: &str) -> Result<Result<Bead, NoFit>, String> {
    let poles = cartesian(curve);
    let [t0, t1] = curve.domain()?;
    let size = poles.iter().map(|p| p.sub(poles[0]).length()).fold(legs[0].max(legs[1]), f64::max).max(1.0);
    let tolerance = 1e-9 * size;
    // The seam's plane, from its poles.
    let origin = poles[0];
    let far = poles.iter().copied().max_by(|a, b| a.sub(origin).length().total_cmp(&b.sub(origin).length())).unwrap_or(origin);
    let normal = poles.iter().map(|p| far.sub(origin).cross(p.sub(origin))).max_by(|a, b| a.length().total_cmp(&b.length()))
        .and_then(|n| n.normalized().ok());
    let Some(normal) = normal else { return Ok(Err("the spline seam is straight; pick it as a line".into())) };
    if poles.iter().any(|p| p.sub(origin).dot(normal).abs() > tolerance) {
        return Ok(Err("the spline seam does not lie in one plane".into()));
    }
    let stations: Vec<f64> = (0..=8).map(|k| t0 + (t1 - t0) * k as f64 / 8.0).collect();
    let on = 1e-7 * size;
    // The joint faces: a base whose plane is the seam's, and a wall the seam
    // rises up square to it (the seam extruded along the base's normal).
    let mut bases: Vec<(Face, Vec3)> = Vec::new();
    let mut walls: Vec<(Face, Vec3)> = Vec::new();
    let mut names: Vec<(&String, &u32)> = scene.solids.iter().filter(|(name, _)| !fit::owned_by(name, exclude)).collect();
    names.sort();
    let picked: Vec<(u32, u64)> = picks.iter().flat_map(|name| scene.resolve_face_group(name)).map(|face| (face.handle, face.face_id)).collect();
    for (solid, &handle) in names {
        let ids: Vec<u64> = crate::with_registered_solid_str(handle, |body| {
            Ok(body.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect())
        })?;
        for face_id in ids {
            if !picked.is_empty() && !picked.contains(&(handle, face_id)) { continue; }
            let face = Face { solid: solid.clone(), handle, face_id };
            let mut carries = true;
            for t in &stations {
                carries = carries && on_face(&face, curve.evaluate(*t)?, on)?;
            }
            if !carries { continue; }
            // The base: flat in the seam's plane, its outward normal one way along it.
            let flat = crate::with_registered_solid_str(handle, |body| {
                let record = body.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face_id).ok_or("a face vanished")?;
                let level = record.surface.control_points.iter().flatten().all(|p| {
                    let w = if p.w.abs() > 1e-300 { p.w } else { 1.0 };
                    Vec3::new(p.x / w, p.y / w, p.z / w).sub(origin).dot(normal).abs() <= tolerance
                });
                if !level { return Ok(None); }
                let (_, outward) = crate::fillet::face_plane_normal(record, origin).map_err(|refusal| refusal.to_string())?;
                Ok(Some(outward))
            })?;
            if let Some(outward) = flat {
                bases.push((face, outward));
                continue;
            }
            // A wall: the seam lifted along the normal (either way) stays on it.
            for rise in [normal, normal.scale(-1.0)] {
                let mut up = true;
                for t in &stations {
                    for h in [0.5, 1.0] {
                        up = up && on_face(&face, curve.evaluate(*t)?.add(rise.scale(legs[0] * h)), on)?;
                    }
                }
                if up {
                    walls.push((Face { solid: face.solid.clone(), handle, face_id }, rise));
                    break;
                }
            }
        }
    }
    // The wall rises off the base's outward side.
    let pairs: Vec<(&(Face, Vec3), &(Face, Vec3))> = bases.iter().flat_map(|base| walls.iter().map(move |wall| (base, wall)))
        .filter(|((_, out), (_, rise))| out.dot(*rise) > 0.5).collect();
    let ((base, _), (wall, rise)) = match pairs.as_slice() {
        [only] => *only,
        [] => return Ok(Err("no flat face carries the spline seam with a wall rising square from it along the whole seam (the wall must be the seam extruded square to the base)".into())),
        _ => return Ok(Err("several corners meet at the spline seam; select the two joint faces".into())),
    };
    let rise = *rise;
    // Across the base, away from the wall: the side where a point just off
    // the seam, a little up, is outside both parts.
    let middle = (t0 + t1) * 0.5;
    let tangent_at = |t: f64| -> Result<Vec3, String> { curve.deriv1(t)?.1.normalized() };
    let across_at = |t: f64| -> Result<Vec3, String> { rise.cross(tangent_at(t)?).normalized() };
    let probe = (legs[0].min(legs[1]) * 0.05).max(on * 100.0);
    let mut side = 0.0;
    for sign in [1.0, -1.0] {
        let point = curve.evaluate(middle)?.add(across_at(middle)?.scale(probe * sign)).add(rise.scale(probe));
        if outside(scene, &wall.solid, point)? && outside(scene, &base.solid, point)? {
            side = sign;
            break;
        }
    }
    if side == 0.0 {
        return Ok(Err("the wall and the base leave no free corner along the spline seam".into()));
    }
    // The three rails; the base toe fitted finer until its leg holds to 1e-7.
    let root = curve.clone();
    let toe_wall = translated(curve, rise.scale(legs[0]));
    let shift = |t: f64| -> Result<Vec3, String> { Ok(across_at(t)?.scale(legs[1] * side)) };
    let mut count = 16;
    let toe_base = loop {
        let toe = moved(curve, count, &shift)?;
        // Read between the fitted stations of every span (the toe runs on
        // the seam's own parameter).
        let mut worst: f64 = 0.0;
        for span in span_ends(curve)?.windows(2) {
            for k in 0..(4 * count) {
                let t = span[0] + (span[1] - span[0]) * (k as f64 + 0.5) / (4 * count) as f64;
                let expected = curve.evaluate(t)?.add(shift(t)?);
                worst = worst.max(toe.evaluate(t)?.sub(expected).length());
            }
        }
        if worst <= 1e-7 * legs[1].max(1.0) { break toe; }
        if count >= 512 {
            return Ok(Err(format!("the base toe along the spline does not fit within {:e} of the leg (the seam bends too sharply for the leg)", 1e-7 * legs[1])));
        }
        count *= 2;
    };
    let triangle = |a: Vec3, b: Vec3, c: Vec3| -> Result<Vec<NurbsCurve>, String> {
        Ok(vec![crate::make_line(a, b)?, crate::make_line(b, c)?, crate::make_line(c, a)?])
    };
    let [s0, s1] = toe_base.domain()?;
    let first = triangle(root.evaluate(t0)?, toe_wall.evaluate(t0)?, toe_base.evaluate(s0)?)?;
    let last = triangle(root.evaluate(t1)?, toe_wall.evaluate(t1)?, toe_base.evaluate(s1)?)?;
    let mut body = match crate::loft_profile_brep_on_guides(&first, &last, &[root.clone(), toe_wall.clone(), toe_base.clone()]) {
        Ok(body) => body,
        Err(refusal) => return Ok(Err(format!("the spline bead could not be lofted: {refusal}"))),
    };
    for (i, face) in body.shells.iter_mut().flat_map(|shell| &mut shell.faces).enumerate() {
        face.name = Some(format!("{id}:BeadFace{}", i + 1));
    }
    // Exact checks: clear of every body, and each contact backed by a slab
    // just inside its part, lofted on the contact's rails and the same rails
    // moved into the part (deep enough for the loft's patches to stand).
    if let Err(reason) = fit::clear(scene, &body, exclude, 1e-6_f64.max(size * 1e-12))? {
        return Ok(Err(reason));
    }
    let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| -> Result<Vec<NurbsCurve>, String> {
        Ok(vec![crate::make_line(a, b)?, crate::make_line(b, c)?, crate::make_line(c, d)?, crate::make_line(d, a)?])
    };
    // Both contacts backed by slabs `depth` deep: the root and each toe, and
    // the same moved into the part.
    let backed_at = |depth: f64| -> Result<Result<(), NoFit>, String> {
        let down = rise.scale(-depth);
        let into = moved(curve, count, &|t| Ok(across_at(t)?.scale(-depth * side)))?;
        let rails = [
            ("base", base, [root.clone(), toe_base.clone(), translated(&toe_base, down), translated(&root, down)]),
            ("wall", wall, [root.clone(), toe_wall.clone(), translated(&into, rise.scale(legs[0])), into.clone()]),
        ];
        for (what, face, [a, b, c, d]) in rails {
            let first = quad(a.evaluate(t0)?, b.evaluate(t0)?, c.evaluate(t0)?, d.evaluate(t0)?)?;
            let last = quad(a.evaluate(t1)?, b.evaluate(t1)?, c.evaluate(t1)?, d.evaluate(t1)?)?;
            let slab = match crate::loft_profile_brep_on_guides(&first, &last, &[a, b, c, d]) {
                Ok(slab) => slab,
                Err(refusal) => return Ok(Err(format!("the {what} support slab could not be lofted: {refusal}"))),
            };
            let part = crate::registered_solid_clone(face.handle)?;
            if let Err(reason) = fit::backed(&slab, &part)? {
                return Ok(Err(format!("the {what} leg is not backed by {} along the whole seam (read {depth:e} deep): {reason}", face.solid)));
            }
        }
        Ok(Ok(()))
    };
    // Each slab is one loft along the whole seam (a loft on trimmed pieces
    // of the rails makes the slab's face coincide with the wall only in part,
    // which the backing Boolean cannot read). It is read 1e-3 of the seam's
    // size deep first (the depth whose coincident-face Booleans hold); where
    // the part does not back that (a plate thinner than it), thinner, halving
    // down to the loft's own degeneracy band (twice the Coons band).
    let mut points = Vec::new();
    for t in &stations {
        points.push(curve.evaluate(*t)?);
        points.push(curve.evaluate(*t)?.add(rise.scale(legs[0])));
    }
    let extent = points.iter().map(|p| p.sub(points[0]).length()).fold(0.0, f64::max) + legs[1];
    let floor = (2.0 * crate::coons::corner_band(extent)).max(100.0 * crate::tolerance::WELD_FLOOR);
    let mut depth = (size * 1e-3).max(floor);
    let mut last_reason = String::new();
    loop {
        match backed_at(depth)? {
            Ok(()) => break,
            Err(reason) => last_reason = reason,
        }
        if depth <= floor {
            return Ok(Err(last_reason));
        }
        depth = (depth * 0.5).max(floor);
    }
    Ok(Ok(Bead { body, joints: [wall.solid.clone(), base.solid.clone()] }))
}

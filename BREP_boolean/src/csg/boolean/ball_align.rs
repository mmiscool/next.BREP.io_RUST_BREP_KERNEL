//! Ball alignment (`BREP_BALL_ALIGN`): a whole ball operand turned so its
//! poles lie on the line the other operand cuts it about.
//!
//! A sphere is the same sphere whatever line its poles are on, but its chart
//! is not. A circle about the polar axis is a latitude, a straight line in
//! (u, v); a circle about any other line has no exact image there and its trim
//! is fitted and refined station by station. Two balls of radius 5 united read
//! 377 ms with their poles on z and 3.8 ms with them on the line through the
//! centres, and the second reads the closed-form volume to 1e-9 where the
//! first is 2.6e-5 off.
//!
//! A ball is a solid of one shell holding one sphere face whose loop is its
//! own seam and poles (`make_sphere_brep`). The line is read from the other
//! operand's faces that reach the ball's box: another whole ball's centre,
//! planes that all share one normal, or a cylinder or cone whose axis runs
//! through the ball's centre (planes at right angles to it allowed beside it).
//! Anything else, or a ball whose poles are on the line already, is left as it
//! is. The turned ball keeps the solid's and the face's ids, name and sense;
//! only its seam and its poles move.

use super::*;
use std::borrow::Cow;
use crate::classification::classify_point;

/// On by default (`BREP_BALL_ALIGN=0` or `off` leaves every ball as built).
/// A ball whose seam and poles a caller hid in what the boolean removes keeps
/// its frame ([`seam_hidden`]), and tangent or near-pole cuts decline.
pub(super) fn ball_align_enabled() -> bool {
    switch!("BREP_BALL_ALIGN", true)
}

/// The centre and radius of `solid` when it is a whole ball.
fn whole_ball(solid: &BrepSolid) -> Option<(Vec3, f64, Vec3)> {
    let [shell] = solid.shells.as_slice() else { return None };
    let [face] = shell.faces.as_slice() else { return None };
    let Some(crate::AnalyticSurface::Sphere { frame, radius }) = face.surface.analytic() else { return None };
    let [lp] = face.loops.as_slice() else { return None };
    if lp.coedges.len() > 4 || solid.edges.len() > 3 || solid.vertices.len() > 2 || !face.same_sense {
        return None;
    }
    // Its loop is the seam there and back and the two poles: nothing cut.
    let seam_or_pole = solid.edges.iter().all(|edge| {
        edge.degenerate || edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).is_ok_and(|point| {
            let offset = point.sub(frame.origin);
            (offset.length() - radius).abs() <= 1e-9 * radius.max(1.0) && offset.dot(frame.y_axis).abs() <= 1e-9 * radius.max(1.0)
        })
    });
    seam_or_pole.then_some((frame.origin, *radius, frame.axis))
}

/// The line through `centre` the faces of `other` reaching the ball cut it
/// about, as a unit direction; `None` when they do not share one.
fn cutting_line(centre: Vec3, radius: f64, other: &BrepSolid, tolerance: f64) -> Option<Vec3> {
    let near = 1e-9 * radius.max(1.0);
    // A cut must cross the ball clear of tangency and clear of the poles it
    // would sit by: its circle's radius between CLEAR and 1 - CLEAR of the
    // ball's (a zero-clearance arm, as wide as its joint ball, touches it
    // along the equator; two balls barely meeting cut a speck by a pole).
    const CLEAR: f64 = 0.05;
    let clear = |circle: f64| circle > CLEAR * radius && circle < (1.0 - CLEAR) * radius;
    let sphere_cut = |other_centre: Vec3, other_radius: f64| -> Option<Vec3> {
        let offset = other_centre.sub(centre);
        let distance = offset.length();
        if distance <= near || distance >= radius + other_radius || distance <= (radius - other_radius).abs() {
            return None;
        }
        let along = (distance * distance + radius * radius - other_radius * other_radius) / (2.0 * distance);
        let circle = (radius * radius - along * along).max(0.0).sqrt();
        clear(circle).then(|| offset.normalized().ok()).flatten()
    };
    if let Some((other_centre, other_radius, _)) = whole_ball(other) {
        return sphere_cut(other_centre, other_radius);
    }
    let ball = crate::spatial::Aabb {
        minimum: centre.sub(Vec3::new(radius, radius, radius)),
        maximum: centre.add(Vec3::new(radius, radius, radius)),
    };
    let mut line: Option<Vec3> = None;
    let mut planes: Vec<Vec3> = Vec::new();
    let parallel = |a: Vec3, b: Vec3| a.cross(b).length() <= 1e-12;
    for face in other.shells.iter().flat_map(|shell| &shell.faces) {
        let Ok(bounds) = crate::Aabb::from_surface_controls(&face.surface) else { return None };
        if !bounds.intersects(ball, tolerance) {
            continue;
        }
        match face.surface.analytic()? {
            crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
                let normal = u_dir.cross(*v_dir).normalized().ok()?;
                let height = origin.sub(centre).dot(normal);
                if height.abs() < radius && !clear((radius * radius - height * height).sqrt()) {
                    return None;
                }
                planes.push(normal);
            }
            crate::AnalyticSurface::Sphere { frame, radius: other_radius } => {
                // Another sphere cuts the ball about the line through both centres.
                let through = sphere_cut(frame.origin, *other_radius)?;
                match line {
                    Some(seen) if !parallel(seen, through) => return None,
                    _ => line = Some(through),
                }
            }
            crate::AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. } => {
                let offset = centre.sub(frame.origin);
                if offset.sub(frame.axis.scale(offset.dot(frame.axis))).length() > near {
                    return None;
                }
                // A cylinder's rims are circles of its radius; a cone's are
                // read where its wall meets the ball, so only its end radii
                // are held clear here.
                if !(clear(rho0.abs()) || rho0.abs() >= radius) || !(clear(rho1.abs()) || rho1.abs() >= radius) {
                    return None;
                }
                if (rho0 - rho1).abs() <= near && !clear(rho0.abs()) {
                    return None;
                }
                match line {
                    Some(seen) if !parallel(seen, frame.axis) => return None,
                    _ => line = Some(frame.axis),
                }
            }
            _ => return None,
        }
    }
    match line {
        Some(axis) => planes.iter().all(|&normal| parallel(normal, axis)).then_some(axis),
        None => {
            let first = *planes.first()?;
            planes.iter().all(|&normal| parallel(normal, first)).then_some(first)
        }
    }
}

/// `line` or its reverse, so that the turned ball's north pole (its centre
/// plus `radius` along the axis, the v = 1 end of its chart) lies in what the
/// boolean removes: inside `other` when `removed_inside`, outside it when not.
/// A south pole in the kept cap reads the closed form: the golf ball (a ball
/// less 62 dimple balls) read 3.9e-6 off with every dimple's north pole in
/// its kept cap and 1.0e-6 with its south pole there (8.5e-7 unturned). Kept
/// as given when neither end classifies.
fn oriented(centre: Vec3, radius: f64, line: Vec3, other: &BrepSolid, removed_inside: bool, tolerance: f64) -> Vec3 {
    let wanted = if removed_inside { PointClass::In } else { PointClass::Out };
    let class = |direction: Vec3| classify_point(centre.add(direction.scale(radius)), other, tolerance).ok().map(|read| read.class);
    if class(line) == Some(wanted) {
        line
    } else if class(line.scale(-1.0)) == Some(wanted) {
        line.scale(-1.0)
    } else {
        line
    }
}

/// Whether the ball's seam and both poles already lie in what the boolean
/// removes (inside `other` when `removed_inside`, outside when not): a frame a
/// caller chose so the kept face borders nothing of its own (a tube joint's
/// ball hides them in the arms' holes), which turning would undo.
fn seam_hidden(solid: &BrepSolid, other: &BrepSolid, removed_inside: bool, tolerance: f64) -> bool {
    let removed = |point: Vec3| match classify_point(point, other, tolerance).map(|read| read.class) {
        Ok(PointClass::In) => removed_inside,
        Ok(PointClass::Out) => !removed_inside,
        Ok(_) => true,
        Err(_) => false,
    };
    solid.edges.iter().all(|edge| {
        (0..=8).all(|step| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0).is_ok_and(removed))
    })
}

/// `solid` turned onto `line` when it is a whole ball whose poles are off it.
fn aligned(solid: &BrepSolid, other: &BrepSolid, removed_inside: bool, tolerance: f64) -> Option<BrepSolid> {
    let (centre, radius, axis) = whole_ball(solid)?;
    let line = cutting_line(centre, radius, other, tolerance)?;
    if axis.cross(line).length() <= 1e-12 {
        return None;
    }
    if seam_hidden(solid, other, removed_inside, tolerance) {
        return None;
    }
    let line = oriented(centre, radius, line, other, removed_inside, tolerance);
    turned_onto(solid, centre, radius, line)
}

/// The whole ball `solid` built again with its poles on `line`, keeping its ids, name and sense.
fn turned_onto(solid: &BrepSolid, centre: Vec3, radius: f64, line: Vec3) -> Option<BrepSolid> {
    // The seam is taken off the line UNSIGNED, so two balls turned onto one
    // line (north poles opposite) put their seams on the same side and a
    // section circle between them is split once, not twice.
    let canonical = if line.x.abs() > 1e-12 { line.scale(line.x.signum()) } else if line.y.abs() > 1e-12 { line.scale(line.y.signum()) } else { line.scale(line.z.signum()) };
    let seam = canonical.perpendicular().ok()?;
    let mut turned = crate::make_sphere_brep_framed(centre, radius, line, Some(seam)).ok()?;
    let (old, new) = (&solid.shells[0].faces[0], &mut turned.shells[0].faces[0]);
    if new.same_sense != old.same_sense {
        return None;
    }
    new.id = old.id;
    new.name = old.name.clone();
    turned.shells[0].id = solid.shells[0].id;
    turned.id = solid.id;
    turned.kind = solid.kind;
    Some(turned)
}

/// Both operands, each whole ball turned onto the line the other cuts it
/// about ([`ball_align_enabled`]); an operand nothing applies to is borrowed.
pub(super) fn align_balls<'a>(
    first: &'a BrepSolid,
    second: &'a BrepSolid,
    operation: BooleanOperation,
    tolerance: f64,
) -> (Cow<'a, BrepSolid>, Cow<'a, BrepSolid>) {
    if !ball_align_enabled() {
        return (Cow::Borrowed(first), Cow::Borrowed(second));
    }
    // What each operand loses: a union and a difference's left operand lose
    // what lies inside the other; an intersection and a difference's tool
    // lose what lies outside it.
    let (first_inside, second_inside) = match operation {
        BooleanOperation::Union => (true, true),
        BooleanOperation::Subtract => (true, false),
        BooleanOperation::Intersect => (false, false),
    };
    let a = aligned(first, second, first_inside, tolerance).map_or(Cow::Borrowed(first), Cow::Owned);
    let b = aligned(second, first, second_inside, tolerance).map_or(Cow::Borrowed(second), Cow::Owned);
    (a, b)
}

/// Each whole ball of `operands` turned onto the one line every other operand
/// reaching it cuts it about ([`ball_align_enabled`]); `None` when nothing
/// applies.
pub(super) fn align_balls_nary(operands: &[BrepSolid], operation: BooleanOperation, tolerance: f64) -> Option<Vec<BrepSolid>> {
    if !ball_align_enabled() || operands.len() < 2 {
        return None;
    }
    let mut changed = false;
    let mut out = operands.to_vec();
    for (index, operand) in operands.iter().enumerate() {
        let Some((centre, radius, axis)) = whole_ball(operand) else { continue };
        let ball = crate::spatial::Aabb {
            minimum: centre.sub(Vec3::new(radius, radius, radius)),
            maximum: centre.add(Vec3::new(radius, radius, radius)),
        };
        let mut line: Option<Vec3> = None;
        let mut cutter: Option<usize> = None;
        let mut single = true;
        for (other_index, other) in operands.iter().enumerate() {
            if other_index == index {
                continue;
            }
            let reaches = other.shells.iter().flat_map(|shell| &shell.faces).any(|face| {
                crate::Aabb::from_surface_controls(&face.surface).is_ok_and(|bounds| bounds.intersects(ball, tolerance))
            });
            if !reaches {
                continue;
            }
            match cutting_line(centre, radius, other, tolerance) {
                Some(through) if line.is_none_or(|seen| seen.cross(through).length() <= 1e-12) => {
                    line = Some(through);
                    cutter.get_or_insert(other_index);
                }
                _ => {
                    single = false;
                    break;
                }
            }
        }
        let (true, Some(line)) = (single, line) else { continue };
        if axis.cross(line).length() <= 1e-12 {
            continue;
        }
        // As the binary pairs read it: the left operand of a difference
        // against its tools, every tool against the left operand.
        let (reference, removed_inside) = match operation {
            BooleanOperation::Union => (cutter, true),
            BooleanOperation::Subtract if index == 0 => (cutter, true),
            BooleanOperation::Subtract => (Some(0), false),
            BooleanOperation::Intersect => (cutter, false),
        };
        if reference.is_some_and(|reference| seam_hidden(operand, &operands[reference], removed_inside, tolerance)) {
            continue;
        }
        let line = match reference {
            Some(reference) => oriented(centre, radius, line, &operands[reference], removed_inside, tolerance),
            None => line,
        };
        if let Some(turned) = turned_onto(operand, centre, radius, line) {
            out[index] = turned;
            changed = true;
        }
    }
    changed.then_some(out)
}

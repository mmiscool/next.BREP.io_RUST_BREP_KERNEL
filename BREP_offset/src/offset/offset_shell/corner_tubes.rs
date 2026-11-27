//! `BREP_OFFSET_CORNER_TUBES` (default on; `=0` restores): the rounded
//! GROWTH by `d` of a body with a concave rounded corner of radius `r < d`, a
//! cylindrical face between two planes it is tangent to, square to the planes
//! at its ends.
//!
//! The body `S` is the sharp-cornered body `S0` (each such corner's fill, the
//! wedge between the arc and its tangent planes' meeting line, taken away)
//! with the fills put back, so `S ⊕ B = (S0 ⊕ B) ∪ (fill ⊕ B)`, and every
//! point within `d` of a fill but not of `S0` is within `d` of the fill's arc
//! face (in the arc's own angular span, where that face's nearest point is
//! radial) or straight over or under the fill. That is the wedge `K` of
//! revolution about the arc's axis, over the arc's span, of the half-section
//! `{0 ≤ ρ ≤ ρ_V : dist((ρ, z), [r, ρ_V] × [z0, z1]) ≤ d}` (ρ_V the meeting
//! line's distance from the axis), cut flat at an end where the face meets a
//! floor rather than an open cap: below a floor `S0` holds everything. So
//! `S ⊕ B = (S0 ⊕ B) ∪ ⋃ K`, exactly; over the corner it is the inner side of
//! the tube of radius `d` about the corner's arc where `ρ < r`.
//!
//! Cause (brep-bench offset/pocketed-block-grown-past-its-corners, a pocket
//! with corners of radius 2 grown by 3, read volume +1.821 and area −0.945;
//! l-block-rounded-inside-grown-past-its-round likewise): the corners' faces
//! vanish in the sharp body, whose concave corner the fillet then rounds
//! about the vertical edge left there, as a ball rolling inside the sharp
//! body does, where the parallel body follows the tube about the source's
//! arc. A union that does not close (the wedges touch the grown body's flat
//! at the arcs' ends) declines, and the body is built as before.
use super::*;
use crate::{extrude_profile_brep, make_arc, make_line, revolve_profile_brep, solid_mass_properties, BooleanOperation, BooleanOptions, KernelRefusal, KernelStage, NurbsCurve, OrRefuse};

/// The switch, default on (`=0` restores).
fn corner_tubes_on() -> bool {
    switch!("BREP_OFFSET_CORNER_TUBES", true)
}


/// One concave rounded corner: the arc's axis through `origin` (at the face's
/// low end) along `axis`, the face from height 0 to `height`, the radial unit
/// `start` at its first tangent line turning by `span` about `axis` (right
/// hand) to the second, radius `radius`; `open` tells whether each end (low,
/// high) is a cap the body ends at, rather than a floor it stands on.
#[derive(Clone, Copy, Debug)]
struct Corner {
    /// The tangent planes' meeting line's distance from the axis.
    reach: f64,
    origin: Vec3,
    axis: Vec3,
    start: Vec3,
    span: f64,
    radius: f64,
    height: f64,
    open: [bool; 2],
}

impl Corner {
    /// The unit radial at angle `phi` from `start` about `axis`.
    fn radial(&self, phi: f64) -> Vec3 {
        let y = self.axis.cross(self.start);
        self.start.scale(phi.cos()).add(y.scale(phi.sin()))
    }
    /// The fill's volume: the kite less the sector, times the height.
    fn fill_volume(&self) -> f64 {
        self.radius * self.radius * ((0.5 * self.span).tan() - 0.5 * self.span) * self.height
    }
}

/// The growth, or `None` where the body has no such corner or a step declines.
pub(super) fn corner_tubes_growth(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !corner_tubes_on() || !(distance > 0.0) {
        return None;
    }
    match corner_tubes_impl(source, distance) {
        Ok(Some(solid)) => {
            let face_images = solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
                .collect();
            Some(OffsetShellResultRecord { solid, face_images })
        }
        Ok(None) => None,
        Err(error) => {
            os_debug!("corner tubes declined: {error}");
            None
        }
    }
}

fn corner_tubes_impl(source: &BrepSolid, distance: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-7 * scale;
    let corners = read_corners(source, distance, tolerance)?;
    if corners.is_empty() {
        return Ok(None);
    }
    os_debug!("corner tubes: {} corner(s)", corners.len());
    // Each wedge's flat reaches the tangent planes' meeting line, so the body
    // must hold the points that far out across the span, low to high: past
    // the fill they are behind the tangent walls.
    for corner in &corners {
        for k in 0..=8 {
            let phi = k as f64 / 8.0;
            for &up in &[0.001, 0.25, 0.5, 0.75, 0.999] {
                let point = corner.origin.add(corner.radial(phi * corner.span).scale(corner.reach)).add(corner.axis.scale(up * corner.height));
                if classify_point(point, source, tolerance).or_refuse(KernelStage::Select, "classify_point")?.class != PointClass::In {
                    os_debug!("corner tubes: the body does not hold its corner's reach at {point:?}");
                    return Ok(None);
                }
            }
        }
    }
    // S0: each corner's kite taken away (past an open end by the radius).
    let options = BooleanOptions::default();
    let mut sharp = source.clone();
    for corner in &corners {
        let low = if corner.open[0] { -corner.radius } else { 0.0 };
        let high = corner.height + if corner.open[1] { corner.radius } else { 0.0 };
        let at = |rho: f64, phi: f64| corner.origin.add(corner.radial(phi).scale(rho)).add(corner.axis.scale(low));
        let kite = [at(0.0, 0.0), at(corner.radius, 0.0), at(corner.reach, 0.5 * corner.span), at(corner.radius, corner.span)];
        let outline = (0..4).map(|i| make_line(kite[i], kite[(i + 1) % 4])).collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Sew, "make_line")?;
        let prism = extrude_profile_brep(&outline, corner.axis, high - low)?;
        sharp = crate::boolean_operation(&sharp, &prism, BooleanOperation::Subtract, &options)?;
    }
    let volume = |solid: &BrepSolid| solid_mass_properties(solid).map(|m| m.volume).or_refuse(KernelStage::Classify, "mass");
    let expected = volume(source)? - corners.iter().map(Corner::fill_volume).sum::<f64>();
    let got = volume(&sharp)?;
    if (got - expected).abs() > 1e-9 * expected.abs().max(1.0) || !sharp.validate().is_empty() {
        os_debug!("corner tubes: the sharp body holds {got}, not {expected}");
        return Ok(None);
    }
    let grown = super::solid::offset_solid_unread_for_pieces(&sharp, distance)?;
    // Each wedge laid with its open ends low, then high (see `laid_wedges`).
    for open_low in [true, false] {
        let mut parts = vec![grown.clone()];
        for corner in &corners {
            for wedge in laid_wedges(corner, open_low) {
                parts.push(tube_wedge(&wedge, distance)?);
            }
        }
        if let Some(union) = union_of(&parts, &options) {
            return Ok(Some(union));
        }
    }
    Ok(None)
}

/// The union of `parts`, n-ary first, else one by one; `None` where neither
/// validates.
fn union_of(parts: &[BrepSolid], options: &BooleanOptions) -> Option<BrepSolid> {
    match crate::boolean_operation_nary(parts, BooleanOperation::Union) {
        Ok(union) if union.validate().is_empty() => return Some(union),
        Ok(_) => os_debug!("corner tubes: the n-ary union does not validate"),
        Err(error) => os_debug!("corner tubes: the n-ary union failed: {error}"),
    }
    let mut body = parts[0].clone();
    for (index, wedge) in parts[1..].iter().enumerate() {
        match crate::boolean_operation(&body, wedge, BooleanOperation::Union, options) {
            Ok(next) => body = next,
            Err(error) => {
                os_debug!("corner tubes: wedge {index}: {error}");
                return None;
            }
        }
    }
    body.validate().is_empty().then_some(body)
}

/// `corner`'s wedge as pieces whose only open end is their own low one
/// (`open_low`) or high one: a tube touches the grown body's plane at the
/// arc's ends, a tangent contact the imprint builds for one of the profile's
/// two arcs and not the other, depending on the body. A corner open at the
/// other end is turned over; one open at both ends is two pieces, each closed
/// a quarter of the height into the other's (inside the grown sharp body,
/// which holds the whole wedge there).
fn laid_wedges(corner: &Corner, open_low: bool) -> Vec<Corner> {
    let open = if open_low { [true, false] } else { [false, true] };
    let flipped = |c: &Corner, from: f64, height: f64| Corner {
        origin: c.origin.add(c.axis.scale(from)),
        axis: c.axis.scale(-1.0),
        start: c.radial(c.span),
        height,
        open,
        ..*c
    };
    let quarter = 0.25 * corner.height;
    match (corner.open, open_low) {
        ([true, false], true) | ([false, true], false) => vec![*corner],
        ([false, true], true) | ([true, false], false) => vec![flipped(corner, corner.height, corner.height)],
        ([true, true], true) => vec![
            Corner { height: corner.height - quarter, open, ..*corner },
            flipped(corner, corner.height, corner.height - quarter),
        ],
        ([true, true], false) => vec![
            Corner { origin: corner.origin.add(corner.axis.scale(quarter)), height: corner.height - quarter, open, ..*corner },
            flipped(corner, corner.height - quarter, corner.height - quarter),
        ],
        _ => vec![*corner],
    }
}

/// The wedge `K` of one corner (module doc).
fn tube_wedge(corner: &Corner, d: f64) -> Result<BrepSolid, KernelRefusal> {
    let (r, top, reach) = (corner.radius, corner.height, corner.reach);
    let h = (d * d - r * r).sqrt();
    let at = |rho: f64, z: f64| corner.origin.add(corner.start.scale(rho)).add(corner.axis.scale(z));
    let line = |a: (f64, f64), b: (f64, f64)| make_line(at(a.0, a.1), at(b.0, b.1)).or_refuse(KernelStage::Sew, "make_line");
    // An arc of radius d about (r, z) from angle a0 to a1 (in the plane of
    // `start` and `axis`), run from a0 to a1 whichever way they are ordered.
    let arc = |z: f64, a0: f64, a1: f64| -> Result<NurbsCurve, KernelRefusal> {
        let curve = if a0 < a1 {
            make_arc(at(r, z), corner.start, corner.axis, d, a0, a1)
        } else {
            make_arc(at(r, z), corner.start, corner.axis, d, a1, a0).and_then(|arc| arc.reversed())
        };
        curve.or_refuse(KernelStage::Sew, "make_arc")
    };
    let side = h.atan2(-r);
    let mut curves = Vec::new();
    // Up the axis, over the top, down the outside, under the bottom.
    let (bottom_axis, top_axis) = (if corner.open[0] { -h } else { 0.0 }, if corner.open[1] { top + h } else { top });
    curves.push(line((0.0, bottom_axis), (0.0, top_axis))?);
    if corner.open[1] {
        curves.push(arc(top, side, std::f64::consts::FRAC_PI_2)?);
        curves.push(line((r, top + d), (reach, top + d))?);
        curves.push(line((reach, top + d), (reach, if corner.open[0] { -d } else { 0.0 }))?);
    } else {
        curves.push(line((0.0, top), (reach, top))?);
        curves.push(line((reach, top), (reach, if corner.open[0] { -d } else { 0.0 }))?);
    }
    if corner.open[0] {
        curves.push(line((reach, -d), (r, -d))?);
        curves.push(arc(0.0, -std::f64::consts::FRAC_PI_2, -side)?);
    } else {
        curves.push(line((reach, 0.0), (0.0, 0.0))?);
    }
    revolve_profile_brep(&curves, corner.origin, corner.axis, corner.span).or_refuse(KernelStage::Sew, "revolve")
}

/// Every concave cylindrical face of radius under `distance` that is a
/// rounded corner (module doc).
fn read_corners(source: &BrepSolid, distance: f64, tolerance: f64) -> Result<Vec<Corner>, KernelRefusal> {
    let faces = source.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    let mut uses = HashMap::<u64, Vec<&FaceRecord>>::default();
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            uses.entry(coedge.edge_id).or_default().push(face);
        }
    }
    let edge = |id: u64| source.edges.iter().find(|edge| edge.id == id);
    let other = |id: u64, face: &FaceRecord| uses.get(&id).and_then(|pair| (pair.len() == 2).then(|| pair.iter().find(|f| f.id != face.id).copied()).flatten());
    let mut corners = Vec::new();
    'faces: for face in &faces {
        if face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
            continue;
        }
        let ids = face.loops[0].coedges.iter().map(|coedge| coedge.edge_id).collect::<Vec<_>>();
        let mut arcs = Vec::new();
        let mut lines = Vec::new();
        for &id in &ids {
            let Some(record) = edge(id) else { continue 'faces };
            if let Some(segment) = record.curve.straight_segment(tolerance) {
                lines.push((id, segment));
            } else if let Some(circle) = fit_arc(record, tolerance)? {
                arcs.push((id, circle, record));
            } else {
                continue 'faces;
            }
        }
        if arcs.len() != 2 || lines.len() != 2 {
            continue;
        }
        let (c0, c1) = (arcs[0].1, arcs[1].1);
        let r = c0.radius;
        if (c1.radius - r).abs() > tolerance || !(r < distance - tolerance) || c0.axis.cross(c1.axis).length() > 1e-9 {
            continue;
        }
        // The axis from the low arc to the high one.
        let rise = c1.center.sub(c0.center);
        let Ok(axis) = rise.normalized() else { continue };
        if rise.sub(axis.scale(rise.dot(axis))).length() > tolerance || axis.cross(c0.axis).length() > 1e-9 {
            continue;
        }
        let height = rise.length();
        let foot = |p: Vec3| p.sub(c0.center).sub(axis.scale(p.sub(c0.center).dot(axis)));
        // Both lines run along the axis at the radius; their radials are the span's ends.
        let mut radials = Vec::new();
        for (_, (a, b)) in &lines {
            let along = b.sub(*a);
            if along.cross(axis).length() > 1e-9 * along.length() || (foot(*a).length() - r).abs() > tolerance || (foot(*b).length() - r).abs() > tolerance {
                continue 'faces;
            }
            let Ok(unit) = foot(*a).normalized() else { continue 'faces };
            radials.push(unit);
        }
        // Order the radials so the arc turns from the first to the second about +axis,
        // through the arc's middle.
        let (e0, e1) = (radials[0], radials[1]);
        let middle_point = arcs[0].2.curve.evaluate(0.5 * (arcs[0].2.t0 + arcs[0].2.t1)).or_refuse(KernelStage::Classify, "evaluate")?;
        let Ok(mid) = foot(middle_point).normalized() else { continue };
        let span = e0.dot(e1).clamp(-1.0, 1.0).acos();
        if !(span > 1e-6 && span < std::f64::consts::PI - 1e-6) || mid.dot(e0.add(e1)) <= 0.0 {
            continue;
        }
        let start = if e0.cross(e1).dot(axis) > 0.0 { e0 } else { e1 };
        // The face's surface is the cylinder: a grid of its points at the radius.
        let (Ok([su0, su1]), Ok([sv0, sv1])) = (face.surface.domain_u(), face.surface.domain_v()) else { continue };
        for i in 0..=6 {
            for j in 0..=6 {
                let Ok(p) = face.surface.evaluate(su0 + (su1 - su0) * i as f64 / 6.0, sv0 + (sv1 - sv0) * j as f64 / 6.0) else { continue 'faces };
                if (foot(p).length() - r).abs() > tolerance {
                    continue 'faces;
                }
            }
        }
        // The face is concave: its outward normal at the arc's middle points at the axis.
        let surface_point = c0.center.add(mid.scale(r)).add(axis.scale(0.5 * height));
        let Ok(foot_uv) = project_point_to_surface(&face.surface, surface_point) else { continue };
        if foot_uv.distance > tolerance {
            continue;
        }
        let Ok(normal) = super::face_normal(face, foot_uv.u, foot_uv.v) else { continue };
        if normal.dot(mid) > -1.0 + 1e-9 {
            continue;
        }
        // Each line's other face is a plane tangent to the cylinder along it.
        for (id, (a, _)) in &lines {
            let Some(neighbour) = other(*id, face) else { continue 'faces };
            let Some((_, plane_normal)) = super::rim_welds::planar_surface_frame(&neighbour.surface, tolerance)? else { continue 'faces };
            let Ok(unit) = foot(*a).normalized() else { continue 'faces };
            if plane_normal.dot(unit).abs() < 1.0 - 1e-9 {
                continue 'faces;
            }
        }
        // Each arc's other face is a plane square to the axis: a cap the body
        // ends at (its outward normal pointing away from the face) or a floor.
        let mut open = [false; 2];
        for (k, (id, circle, _)) in arcs.iter().enumerate() {
            let Some(neighbour) = other(*id, face) else { continue 'faces };
            if super::rim_welds::planar_surface_frame(&neighbour.surface, tolerance)?.is_none() {
                continue 'faces;
            }
            let Ok(foot_cap) = project_point_to_surface(&neighbour.surface, circle.center.add(mid.scale(r))) else { continue 'faces };
            let Ok(cap_normal) = super::face_normal(neighbour, foot_cap.u, foot_cap.v) else { continue 'faces };
            if cap_normal.cross(axis).length() > 1e-9 {
                continue 'faces;
            }
            let away = if k == 0 { axis.scale(-1.0) } else { axis };
            open[k] = cap_normal.dot(away) > 0.0;
        }
        corners.push(Corner { reach: r / (0.5 * span).cos(), origin: c0.center, axis, start, span, radius: r, height, open });
    }
    Ok(corners)
}

/// An arc edge's circle: centre, unit normal, radius (the circle through its
/// ends and middle, every sample of it within `tolerance`).
#[derive(Clone, Copy, Debug)]
struct Arc3 {
    center: Vec3,
    axis: Vec3,
    radius: f64,
}

fn fit_arc(edge: &EdgeRecord, tolerance: f64) -> Result<Option<Arc3>, KernelRefusal> {
    let at = |fraction: f64| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Classify, "evaluate");
    let (a, m, b) = (at(0.0)?, at(0.5)?, at(1.0)?);
    let (u, w) = (m.sub(a), b.sub(a));
    let normal = u.cross(w);
    let Ok(axis) = normal.normalized() else { return Ok(None) };
    let n2 = normal.dot(normal);
    let center = a.add(w.cross(normal).scale(u.dot(u) / (2.0 * n2)).add(normal.cross(u).scale(w.dot(w) / (2.0 * n2))));
    let radius = a.sub(center).length();
    if !(radius > tolerance) {
        return Ok(None);
    }
    for k in 0..=32 {
        let radial = at(k as f64 / 32.0)?.sub(center);
        if radial.dot(axis).abs() > tolerance || (radial.length() - radius).abs() > tolerance {
            return Ok(None);
        }
    }
    Ok(Some(Arc3 { center, axis, radius }))
}

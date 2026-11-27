//! `BREP_OFFSET_CONVEX_EROSION` (default on; `=0` restores): the SHRINK
//! by `d` of a convex body bounded by planes and spheres (material inside), as
//! the intersection of its faces' regions each shrunk: erosion distributes over
//! intersection, `(∩ Rᵢ) ⊖ B = ∩ (Rᵢ ⊖ B)`, a ball of radius `R` shrinks to
//! the concentric ball of radius `R − d` and a half-space moves `d` in. Its
//! convex edges stay sharp, as a shrink leaves them.
//!
//! The body is read as that intersection only when every edge is a convex
//! join, every face is a plane or a sphere it lies inside, and the
//! intersection of the unshrunk regions, built the same way, has the body's
//! volume. It runs only where the pipeline refused.
//!
//! Cause (brep-bench offset/barrel-shrunk, a ball of radius 5 between the
//! planes z = ±3 shrunk by 1): the pipeline's completion closed the moved
//! sphere and planes with a non-integral genus, and the revolved curved eroder
//! reads the sphere face's whole meridian, not the arc between its rims.
use super::*;
use crate::{boolean_operation, make_sphere_brep, solid_mass_properties, split_solid_by_plane, AnalyticSurface, BooleanOperation, BooleanOptions, KernelRefusal, KernelStage, OrRefuse};

/// The switch, default on (`=0` restores).
fn convex_erosion_on() -> bool {
    switch!("BREP_OFFSET_CONVEX_EROSION", true)
}


/// One face's region: a ball, or the half-space behind a plane (outward `normal`).
#[derive(Clone, Copy, Debug)]
enum Region {
    Ball { center: Vec3, radius: f64 },
    Half { point: Vec3, normal: Vec3 },
}

/// The shrink of `source` by `distance` (< 0), or `None` where the lane is
/// off or declines.
pub(super) fn convex_erosion(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !convex_erosion_on() || !(distance < 0.0) || !distance.is_finite() {
        return None;
    }
    match convex_erosion_impl(source, -distance) {
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
            os_debug!("convex erosion declined: {error}");
            None
        }
    }
}

fn convex_erosion_impl(source: &BrepSolid, amount: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    let Some(regions) = read_regions(source, tolerance)? else { return Ok(None) };
    let volume = |solid: &BrepSolid| solid_mass_properties(solid).map(|m| m.volume).or_refuse(KernelStage::Classify, "mass");
    let Some(whole) = intersection(&regions, 0.0, scale)? else { return Ok(None) };
    let (got, want) = (volume(&whole)?, volume(source)?);
    if (got - want).abs() > 1e-9 * want.abs().max(1.0) {
        os_debug!("convex erosion: the regions' intersection holds {got}, the body {want}");
        return Ok(None);
    }
    let Some(shrunk) = intersection(&regions, amount, scale)? else { return Ok(None) };
    Ok(shrunk.validate().is_empty().then_some(shrunk))
}

/// The faces' regions, or `None` unless every face is a plane or a sphere the
/// body lies inside, at least one a sphere, and every edge a convex join.
fn read_regions(source: &BrepSolid, tolerance: f64) -> Result<Option<Vec<Region>>, KernelRefusal> {
    let faces = source.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    if source.shells.len() != 1 {
        return Ok(None);
    }
    let mut uses = HashMap::<u64, Vec<(&FaceRecord, bool)>>::default();
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            uses.entry(coedge.edge_id).or_default().push((face, coedge.forward));
        }
    }
    for edge in &source.edges {
        if edge.degenerate {
            continue;
        }
        let Some(pair) = uses.get(&edge.id) else { continue };
        if pair.len() != 2 {
            return Ok(None);
        }
        // A seam (one face on both sides) is no join.
        if pair[0].0.id == pair[1].0.id {
            continue;
        }
        if super::solid::edge_is_convex_join(edge, pair[0], pair[1].0, 1.0 - 1e-9)? != Some(true) {
            return Ok(None);
        }
    }
    let mut regions: Vec<Region> = Vec::new();
    for face in &faces {
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
        let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let point = face.surface.evaluate(u, v).or_refuse(KernelStage::Classify, "evaluate")?;
        let Ok(normal) = super::face_normal(face, u, v) else { return Ok(None) };
        let region = match face.surface.analytic() {
            Some(AnalyticSurface::Sphere { frame, radius }) => {
                if !(normal.dot(point.sub(frame.origin)) > 0.0) {
                    return Ok(None);
                }
                Region::Ball { center: frame.origin, radius: *radius }
            }
            Some(AnalyticSurface::Plane { .. }) => Region::Half { point, normal },
            _ => return Ok(None),
        };
        // One region per carrier.
        let same = regions.iter().any(|other| match (other, &region) {
            (Region::Ball { center: a, radius: r }, Region::Ball { center: b, radius: s }) => a.sub(*b).length() <= tolerance && (r - s).abs() <= tolerance,
            (Region::Half { point: a, normal: n }, Region::Half { point: b, normal: m }) => n.dot(*m) >= 1.0 - 1e-12 && b.sub(*a).dot(*n).abs() <= tolerance,
            _ => false,
        });
        if !same {
            regions.push(region);
        }
    }
    if !regions.iter().any(|region| matches!(region, Region::Ball { .. })) {
        return Ok(None);
    }
    Ok(Some(regions))
}

/// The intersection of `regions`, each shrunk by `amount`: the first ball,
/// cut by every half-space and met with every other ball. `None` where a
/// region shrinks to nothing or leaves nothing.
fn intersection(regions: &[Region], amount: f64, scale: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let balls = regions.iter().filter_map(|region| match region {
        Region::Ball { center, radius } => Some((*center, radius - amount)),
        Region::Half { .. } => None,
    });
    let mut body: Option<BrepSolid> = None;
    for (center, radius) in balls {
        if !(radius > 1e-9 * scale) {
            return Ok(None);
        }
        let ball = make_sphere_brep(center, radius, Vec3::new(0.0, 0.0, 1.0)).or_refuse(KernelStage::Sew, "make_sphere_brep")?;
        body = Some(match body {
            None => ball,
            Some(body) => boolean_operation(&body, &ball, BooleanOperation::Intersect, &BooleanOptions::default())?,
        });
    }
    let Some(mut body) = body else { return Ok(None) };
    // Every region lies inside the first ball, so the ball bounds the body.
    let Some(&(ball_centre, ball_radius)) = regions.iter().find_map(|region| match region {
        Region::Ball { center, radius } => Some((*center, radius - amount)),
        Region::Half { .. } => None,
    }).as_ref() else { return Ok(None) };
    let band = 1e-9 * scale;
    for region in regions {
        let Region::Half { point, normal } = region else { continue };
        let moved = point.sub(normal.scale(amount));
        match split_solid_by_plane(&body, moved, *normal) {
            Ok((below, _)) => body = below,
            Err(error) => {
                // Only a plane clear of the bounding ball is read: wholly in
                // front of the body (kept whole) or wholly past it (nothing
                // left). A tangent plane or a failed cut declines.
                let offset = ball_centre.sub(moved).dot(*normal);
                if offset + ball_radius < -band {
                    continue;
                }
                if offset - ball_radius > band {
                    return Ok(None);
                }
                os_debug!("convex erosion: a plane's cut declined: {error}");
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "convex_erosion_cut", format!("convex erosion: {error}")));
            }
        }
    }
    Ok(Some(body))
}

/// `BREP_OFFSET_HORN_TORUS` (default on; `=0` restores): a solid torus
/// (tube radius `r` about a circle of radius `R`) grown by `d = R − r` is
/// the horn torus, the disk of radius `R` about the middle circle's point
/// revolved, closing on the axis at the centre: every point within `d` of
/// the torus is within `r + d` of its middle circle. Built as that disk's two
/// half circles from the axis point revolved, as a horn torus is.
///
/// Cause (brep-bench offset/torus-grown-to-its-hole): the general route's fold
/// carve refuses ("the fold locus between the two crossings is a single
/// point"): the grown tube meets itself at one point on the axis.
fn horn_torus_on() -> bool {
    switch!("BREP_OFFSET_HORN_TORUS", true)
}


/// The horn torus `source` grows to by `distance`, or `None` (module doc of
/// [`horn_torus_on`]).
pub(super) fn horn_torus_growth(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !horn_torus_on() || !(distance > 0.0) {
        return None;
    }
    let faces = source.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    if source.shells.len() != 1 || faces.len() != 1 {
        return None;
    }
    let Some(AnalyticSurface::Torus { frame, major_radius, minor_radius }) = faces[0].surface.analytic() else { return None };
    let tolerance = 1e-9 * crate::solid_scale(source).max(1.0);
    if (minor_radius + distance - major_radius).abs() > tolerance {
        return None;
    }
    // The material is the tube's inside: the face's outward normal points away from the middle circle.
    let [u0, u1] = faces[0].surface.domain_u().ok()?;
    let [v0, v1] = faces[0].surface.domain_v().ok()?;
    let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let point = faces[0].surface.evaluate(u, v).ok()?;
    let normal = super::face_normal(faces[0], u, v).ok()?;
    let along = point.sub(frame.origin).dot(frame.axis);
    let radial = point.sub(frame.origin).sub(frame.axis.scale(along));
    let middle = frame.origin.add(radial.normalized().ok()?.scale(*major_radius));
    if !(normal.dot(point.sub(middle)) > 0.0) {
        return None;
    }
    let radius = *major_radius;
    let centre = frame.origin.add(frame.x_axis.scale(radius));
    let halves = [
        crate::make_arc(centre, frame.x_axis, frame.axis, radius, -std::f64::consts::PI, 0.0).ok()?,
        crate::make_arc(centre, frame.x_axis, frame.axis, radius, 0.0, std::f64::consts::PI).ok()?,
    ];
    let solid = crate::revolve_profile_brep(&halves, frame.origin, frame.axis, std::f64::consts::TAU).ok()?;
    if !solid.validate().is_empty() {
        os_debug!("horn torus: the revolved disk does not validate");
        return None;
    }
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: faces[0].id })
        .collect();
    Some(OffsetShellResultRecord { solid, face_images })
}

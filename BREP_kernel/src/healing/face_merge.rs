use crate::analytic_surface::circumcenter;
use crate::mass_properties::{face_area, face_volume_contribution, parameter_space_area};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord};
use crate::{
    build_pcurve_on_surface, make_line, make_plane, make_revolution, KnotVector, NurbsCurve,
    NurbsSurface, Vec3, Vec4,
};
use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

fn controls_match(first: Vec4, second: Vec4, tolerance: f64) -> bool {
    (first.x - second.x).abs() <= tolerance
        && (first.y - second.y).abs() <= tolerance
        && (first.z - second.z).abs() <= tolerance
        && (first.w - second.w).abs() <= tolerance
}

/// Whether two faces ride the SAME carrier patch — identical degrees, knots
/// and control points inside `tolerance`. `pub(crate)` because the direct-edit
/// cap lane asks the same question for the same reason this module does: when
/// two faces share one carrier their coedges' pcurves are interchangeable, so
/// loops can move between them with nothing refit.
pub(crate) fn same_surface(first: &NurbsSurface, second: &NurbsSurface, tolerance: f64) -> bool {
    first.degree_u == second.degree_u
        && first.degree_v == second.degree_v
        && first.knots_u.len() == second.knots_u.len()
        && first.knots_v.len() == second.knots_v.len()
        && first.control_points.len() == second.control_points.len()
        && first
            .control_points
            .iter()
            .zip(&second.control_points)
            .all(|(a, b)| {
                a.len() == b.len()
                    && a.iter()
                        .zip(b)
                        .all(|(a, b)| controls_match(*a, *b, tolerance))
            })
        && first
            .knots_u
            .iter()
            .zip(&second.knots_u)
            .all(|(a, b)| (a - b).abs() <= tolerance)
        && first
            .knots_v
            .iter()
            .zip(&second.knots_v)
            .all(|(a, b)| (a - b).abs() <= tolerance)
}

fn surface_domains(surface: &NurbsSurface) -> Result<([f64; 2], [f64; 2]), KernelRefusal> {
    Ok((
        KnotVector::new(surface.knots_u.clone(), surface.degree_u)
            .or_refuse(KernelStage::Sew, "new")?
            .domain(),
        KnotVector::new(surface.knots_v.clone(), surface.degree_v)
            .or_refuse(KernelStage::Sew, "new")?
            .domain(),
    ))
}

fn outward_normal(face: &FaceRecord) -> Result<Vec3, KernelRefusal> {
    let (u, v) = surface_domains(&face.surface)?;
    let normal = face
        .surface
        .normal((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0)
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
    Ok(if face.same_sense {
        normal
    } else {
        normal.scale(-1.0)
    })
}

fn planar_support(
    face: &FaceRecord,
    tolerance: f64,
) -> Result<Option<(Vec3, Vec3)>, KernelRefusal> {
    let (u, v) = surface_domains(&face.surface)?;
    let midpoint_u = (u[0] + u[1]) / 2.0;
    let midpoint_v = (v[0] + v[1]) / 2.0;
    let origin = face
        .surface
        .evaluate(midpoint_u, midpoint_v)
        .or_refuse(KernelStage::Sew, "evaluate")?;
    let normal = face
        .surface
        .normal(midpoint_u, midpoint_v)
        .or_refuse(KernelStage::Sew, "normal")?;
    let controls = face
        .surface
        .control_points
        .iter()
        .flatten()
        .map(|control| control.point())
        .collect::<Result<Vec<_>, _>>()
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
    let scale = controls
        .iter()
        .map(|point| point.sub(origin).length())
        .fold(1.0f64, f64::max);
    let plane_tolerance = (tolerance * 100.0).max(1e-7) * scale;
    if controls
        .iter()
        .all(|point| point.sub(origin).dot(normal).abs() <= plane_tolerance)
    {
        Ok(Some((origin, normal)))
    } else {
        Ok(None)
    }
}

fn coplanar_with_same_outward_normal(
    first: &FaceRecord,
    second: &FaceRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let Some((first_origin, first_geometric_normal)) = planar_support(first, tolerance)? else {
        return Ok(false);
    };
    let Some((_, second_geometric_normal)) = planar_support(second, tolerance)? else {
        return Ok(false);
    };
    if first_geometric_normal.dot(second_geometric_normal).abs() < 1.0 - 1e-7 {
        return Ok(false);
    }
    let first_normal = outward_normal(first)?;
    let second_normal = outward_normal(second)?;
    if first_normal.dot(second_normal) < 1.0 - 1e-7 {
        return Ok(false);
    }
    let (second_u, second_v) = surface_domains(&second.surface)?;
    // The merged face is emitted carrying `first`'s plane, so every point of
    // `second` has to lie ON that plane, not merely near it.  The band is the
    // caller's coplanarity tolerance, floored where a caller asks for less
    // than its own fitting noise and CAPPED at the assembler weld: past that
    // the assembler holds a point and this plane apart, so the merged face is
    // one nothing exact downstream can consume — the STEP importer's phase
    // 1.75 ear-clips precisely such a face back into planar triangles.
    //
    // The cap replaces a `* (1.0 + first_origin.length())` factor that
    // charged the band by how far the plane's origin sits from the WORLD
    // origin — where the part happens to have been modelled, not a property
    // of its geometry.  On a 190-unit part 50 units out it opened a 1.9e-7
    // request to a 9.7e-4 band, wide enough to merge adjacent facets of a
    // tessellated curve into one "plane" whose own corners sat 5.4e-4 off it.
    let plane_tolerance = (tolerance * 100.0)
        .max(1e-6)
        .min(crate::tolerance::assembler_weld(tolerance));
    for u in second_u {
        for v in second_v {
            if second
                .surface
                .evaluate(u, v)
                .or_refuse(KernelStage::Sew, "evaluate")?
                .sub(first_origin)
                .dot(first_normal)
                .abs()
                > plane_tolerance
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Circular-arc support of a curve, detected by sampling: circumcenter of
/// three spread samples, validated against all samples for constant radius
/// and planarity. Returns (center, unit plane normal, radius).
fn arc_circle_support(
    curve: &NurbsCurve,
    tolerance: f64,
) -> Result<Option<(Vec3, Vec3, f64)>, KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let samples = (0..=8)
        .map(|index| curve.evaluate(t0 + (t1 - t0) * index as f64 / 8.0))
        .collect::<Result<Vec<_>, _>>()
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
    // A full-circle carrier evaluates identical endpoints, so the sample
    // triple must avoid (start, end); fall back across spreads until one is
    // non-degenerate.
    let Some(center) = circumcenter(samples[0], samples[2], samples[5])
        .or_else(|| circumcenter(samples[0], samples[4], samples[8]))
        .or_else(|| circumcenter(samples[1], samples[4], samples[7]))
    else {
        return Ok(None);
    };
    let radius = samples[0].sub(center).length();
    if radius <= tolerance {
        return Ok(None);
    }
    let normal = match samples[2]
        .sub(samples[0])
        .cross(samples[5].sub(samples[0]))
        .normalized()
        .or_else(|_| {
            samples[4]
                .sub(samples[0])
                .cross(samples[8].sub(samples[0]))
                .normalized()
        }) {
        Ok(normal) => normal,
        Err(_) => return Ok(None),
    };
    let band = tolerance.max(1e-9 * radius);
    for sample in &samples {
        let delta = sample.sub(center);
        if (delta.length() - radius).abs() > band || delta.dot(normal).abs() > band {
            if std::env::var("BREP_DEBUG_MERGE").is_ok() {
                eprintln!(
                    "    arc reject: radius={radius} dr={} dplane={} band={band}",
                    (delta.length() - radius).abs(),
                    delta.dot(normal).abs()
                );
            }
            return Ok(None);
        }
    }
    Ok(Some((center, normal, radius)))
}

/// Merge lanes a caller opts into beyond what the solid-wide merge does.
///
/// The solid-wide merge runs after every boolean and keeps every face it does
/// not have to join: the default (all off) is exactly its behaviour. A caller
/// joining faces the USER selected together (Push Face on the halves of an
/// imported bore) asks for more: those faces are one region of one carrier by
/// the user's own reading, and a face the merge leaves split is a face the
/// edit cannot move.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct MergeLanes<'a> {
    /// Admit a CONE (a ruled revolution whose two boundary circles differ in
    /// radius) wherever the cylinder lanes run. Only the full-turn lane builds
    /// its carrier from the radius at each end; the partial-turn lane stays
    /// cylinder-only.
    pub cones: bool,
    /// Let the full-turn lane seam a ring closed by two half-turn patches on
    /// a SHARED axial edge (the halves of a bore written as two faces). The
    /// solid-wide merge keeps such pairs apart (see `merge_cylinder_full_turn`).
    pub shared_seam_ring: bool,
    /// Among equally ranked seam candidates, prefer an axial edge ending at one
    /// of these vertices: the seam a coaxial neighbour of the same selection
    /// already kept, so the rim the two faces share meets both seams at ONE
    /// vertex and closes into one circle.
    pub seam_vertices: Option<&'a HashSet<u64>>,
}

struct CylinderSupport {
    origin: Vec3,
    axis: Vec3,
    /// The radius at `origin`.
    radius: f64,
    /// dρ per unit of axial travel along `axis`: zero for a cylinder, the
    /// cone's slope for a cone (admitted only under `MergeLanes::cones`).
    slope: f64,
    /// +1 when the face's outward normal points radially away from the
    /// axis, -1 when it points inward.
    outward_radial_sign: f64,
}

impl CylinderSupport {
    /// The carrier's radius `axial` along `axis` from `origin`.
    fn radius_at(&self, axial: f64) -> f64 {
        self.radius + self.slope * axial
    }
}

/// Detect a ruled cylinder patch: quadratic circular arc in one parameter
/// direction, linear in the other, both boundary arcs sharing an axis and
/// radius. Covers extruded arcs, revolved axis-parallel lines, and
/// primitive cylinder walls regardless of parameterization. With
/// `allow_cone`, two coaxial boundary arcs of DIFFERENT radii are a cone
/// patch, admitted when the patch's midpoint lies on the cone they span.
fn ruled_cylinder_support(
    face: &FaceRecord,
    tolerance: f64,
    allow_cone: bool,
) -> Result<Option<CylinderSupport>, KernelRefusal> {
    let surface = &face.surface;
    let debug = std::env::var("BREP_DEBUG_MERGE").is_ok();
    let (arc_first, columns) = match (surface.degree_u, surface.degree_v) {
        (2, 1) => (true, surface.control_points[0].len()),
        (1, 2) => (false, surface.control_points.len()),
        _ => {
            if debug {
                eprintln!(
                    "  support face {}: degrees ({},{}) not ruled-arc",
                    face.id, surface.degree_u, surface.degree_v
                );
            }
            return Ok(None);
        }
    };
    // Boolean splits can insert linear-direction knots, so a ruled carrier
    // may hold more than two columns; the outermost columns still bound the
    // patch and interior straightness is enforced by the final area check.
    let last = columns - 1;
    let iso_curve = |side: usize| -> Result<NurbsCurve, KernelRefusal> {
        let controls = if arc_first {
            surface
                .control_points
                .iter()
                .map(|row| row[side])
                .collect::<Vec<_>>()
        } else {
            surface.control_points[side].clone()
        };
        let knots = if arc_first {
            surface.knots_u.clone()
        } else {
            surface.knots_v.clone()
        };
        NurbsCurve::new(2, knots, controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")
    };
    let scale = surface
        .control_points
        .iter()
        .flatten()
        .map(|point| point.point().map(|p| p.length()).unwrap_or(0.0))
        .fold(1.0f64, f64::max);
    let circle_tolerance = (tolerance * 100.0).max(1e-7 * scale);
    let Some((center0, normal0, radius0)) = arc_circle_support(&iso_curve(0)?, circle_tolerance)?
    else {
        if debug {
            eprintln!("  support face {}: first iso not circular", face.id);
        }
        return Ok(None);
    };
    let Some((center1, normal1, radius1)) =
        arc_circle_support(&iso_curve(last)?, circle_tolerance)?
    else {
        if debug {
            eprintln!("  support face {}: last iso not circular", face.id);
        }
        return Ok(None);
    };
    let is_cone = (radius0 - radius1).abs() > circle_tolerance;
    if is_cone && !allow_cone {
        if debug {
            eprintln!(
                "  support face {}: radii differ {} vs {}",
                face.id, radius0, radius1
            );
        }
        return Ok(None);
    }
    let axial = center1.sub(center0);
    if axial.length() <= circle_tolerance {
        if debug {
            eprintln!("  support face {}: zero axial extent", face.id);
        }
        return Ok(None);
    }
    let axis = axial
        .normalized()
        .or_refuse(KernelStage::Sew, "normalized")?;
    if axis.dot(normal0).abs() < 1.0 - 1e-7 || axis.dot(normal1).abs() < 1.0 - 1e-7 {
        if debug {
            eprintln!(
                "  support face {}: arc planes not perpendicular to axis",
                face.id
            );
        }
        return Ok(None);
    }
    let outward = outward_normal(face)?;
    let (u, v) = surface_domains(surface)?;
    let midpoint = surface
        .evaluate((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0)
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
    let foot = center0.add(axis.scale(midpoint.sub(center0).dot(axis)));
    let radial = match midpoint.sub(foot).normalized() {
        Ok(radial) => radial,
        Err(_) => return Ok(None),
    };
    let slope = if is_cone {
        (radius1 - radius0) / axial.length()
    } else {
        0.0
    };
    if is_cone {
        // A twisted ruling between the two circles is a hyperboloid, not a
        // cone: the patch's own midpoint must sit on the cone they span.
        let axial_mid = midpoint.sub(center0).dot(axis);
        let off = (midpoint.sub(foot).length() - (radius0 + slope * axial_mid)).abs();
        if off > circle_tolerance {
            if debug {
                eprintln!("  support face {}: midpoint {off:.3e} off the spanned cone", face.id);
            }
            return Ok(None);
        }
    }
    let alignment = outward.dot(radial);
    // A cone's normal leans off the radial by its half-angle.
    if alignment.abs() < 0.9 / (1.0 + slope * slope).sqrt() {
        if debug {
            eprintln!(
                "  support face {}: outward not radial ({alignment})",
                face.id
            );
        }
        return Ok(None);
    }
    Ok(Some(CylinderSupport {
        origin: center0,
        axis,
        radius: radius0,
        slope,
        outward_radial_sign: alignment.signum(),
    }))
}

fn cosurface_cylinder_pair(
    first: &FaceRecord,
    second: &FaceRecord,
    tolerance: f64,
    lanes: MergeLanes<'_>,
) -> Result<Option<CylinderSupport>, KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_MERGE").is_ok();
    let Some(support_a) = ruled_cylinder_support(first, tolerance, lanes.cones)? else {
        if debug {
            eprintln!(
                "merge {}x{}: first not a ruled cylinder",
                first.id, second.id
            );
        }
        return Ok(None);
    };
    let Some(support_b) = ruled_cylinder_support(second, tolerance, lanes.cones)? else {
        if debug {
            eprintln!(
                "merge {}x{}: second not a ruled cylinder",
                first.id, second.id
            );
        }
        return Ok(None);
    };
    let scale = 1.0f64
        .max(support_a.origin.length())
        .max(support_b.origin.length())
        .max(support_a.radius);
    let band = (tolerance * 100.0).max(1e-7 * scale);
    // Both radii read at A's origin and one radius-length further along A's
    // axis: equal for two cylinders of one radius, and for two patches of
    // one cone (whose slopes agree once B's axis is turned to A's).
    let radius_mismatch = {
        let station = support_a.radius.max(1.0);
        let b_axial = |axial_on_a: f64| {
            support_a
                .origin
                .add(support_a.axis.scale(axial_on_a))
                .sub(support_b.origin)
                .dot(support_b.axis)
        };
        (support_a.radius - support_b.radius_at(b_axial(0.0)))
            .abs()
            .max((support_a.radius_at(station) - support_b.radius_at(b_axial(station))).abs())
    };
    if support_a.axis.dot(support_b.axis).abs() < 1.0 - 1e-7
        || radius_mismatch > band
        || support_a.outward_radial_sign != support_b.outward_radial_sign
    {
        if debug {
            eprintln!(
                "merge {}x{}: axis/radius/sign mismatch dot={} dr={} signs=({},{})",
                first.id,
                second.id,
                support_a.axis.dot(support_b.axis),
                radius_mismatch,
                support_a.outward_radial_sign,
                support_b.outward_radial_sign
            );
        }
        return Ok(None);
    }
    let offset = support_b.origin.sub(support_a.origin);
    let perpendicular = offset.sub(support_a.axis.scale(offset.dot(support_a.axis)));
    if perpendicular.length() > band {
        if debug {
            eprintln!(
                "merge {}x{}: axes offset {}",
                first.id,
                second.id,
                perpendicular.length()
            );
        }
        return Ok(None);
    }
    if debug {
        eprintln!(
            "merge {}x{}: MATCH r={} axis=({:.4},{:.4},{:.4})",
            first.id,
            second.id,
            support_a.radius,
            support_a.axis.x,
            support_a.axis.y,
            support_a.axis.z
        );
    }
    Ok(Some(support_a))
}

struct ExtrusionSupport {
    /// Profile iso curve at the sweep start, in its actual 3D position.
    base: NurbsCurve,
    /// Unit sweep direction (from column 0 toward the last column).
    direction: Vec3,
    /// Axial interval the patch occupies along `direction`.
    interval: [f64; 2],
}

/// Detect a linear sweep of an arbitrary profile curve: one parameter
/// direction is degree 1 with every profile control translated by the SAME
/// vector (weights unchanged), and the profile lies in a plane perpendicular
/// to the sweep. This is exact at the control-point level, so it also merges
/// swept faces whose profile is only approximately circular.
fn ruled_extrusion_support(
    face: &FaceRecord,
    tolerance: f64,
) -> Result<Option<ExtrusionSupport>, KernelRefusal> {
    let surface = &face.surface;
    // Planar faces are the coplanar path's business; a plane recognized as
    // an "extrusion of a line" here could merge across a knife edge.
    if planar_support(face, tolerance)?.is_some() {
        return Ok(None);
    }
    let profile_first = if surface.degree_v == 1 && surface.degree_u >= 1 {
        true
    } else if surface.degree_u == 1 && surface.degree_v >= 1 {
        false
    } else {
        return Ok(None);
    };
    let (profile_count, linear_count) = if profile_first {
        (
            surface.control_points.len(),
            surface.control_points[0].len(),
        )
    } else {
        (
            surface.control_points[0].len(),
            surface.control_points.len(),
        )
    };
    if linear_count < 2 {
        return Ok(None);
    }
    let control = |profile_index: usize, linear_index: usize| -> Vec4 {
        if profile_first {
            surface.control_points[profile_index][linear_index]
        } else {
            surface.control_points[linear_index][profile_index]
        }
    };
    let scale = surface
        .control_points
        .iter()
        .flatten()
        .map(|point| point.point().map(|p| p.length()).unwrap_or(0.0))
        .fold(1.0f64, f64::max);
    let band = (tolerance * 100.0).max(1e-7 * scale);
    let mut translation: Option<Vec3> = None;
    for profile_index in 0..profile_count {
        let first = control(profile_index, 0);
        let last = control(profile_index, linear_count - 1);
        if (first.w - last.w).abs() > 1e-9 * first.w.abs().max(1.0) {
            return Ok(None);
        }
        let step = last
            .point()
            .or_refuse(KernelStage::Sew, "point")?
            .sub(first.point().or_refuse(KernelStage::Sew, "point")?);
        if let Some(reference) = translation {
            if step.sub(reference).length() > band {
                return Ok(None);
            }
        } else {
            translation = Some(step);
        }
        let start = first.point().or_refuse(KernelStage::Sew, "point")?;
        for linear_index in 1..linear_count - 1 {
            let middle = control(profile_index, linear_index);
            if (middle.w - first.w).abs() > 1e-9 * first.w.abs().max(1.0) {
                return Ok(None);
            }
            let offset = middle
                .point()
                .or_refuse(KernelStage::Sew, "point")?
                .sub(start);
            let along = offset.dot(step) / step.dot(step).max(1e-30);
            if offset.sub(step.scale(along)).length() > band
                || !(-1e-9..=1.0 + 1e-9).contains(&along)
            {
                return Ok(None);
            }
        }
    }
    let translation = translation.ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Sew,
            "face_merge",
            "extrusion support: empty net",
        )
    })?;
    if translation.length() <= band {
        return Ok(None);
    }
    let direction = translation
        .normalized()
        .or_refuse(KernelStage::Sew, "normalized")?;
    let base_controls = (0..profile_count).map(|index| control(index, 0)).collect();
    let base_knots = if profile_first {
        surface.knots_u.clone()
    } else {
        surface.knots_v.clone()
    };
    let base_degree = if profile_first {
        surface.degree_u
    } else {
        surface.degree_v
    };
    let base = NurbsCurve::new(base_degree, base_knots, base_controls)
        .or_refuse(KernelStage::Sew, "new")?;
    let [t0, t1] = base.domain().or_refuse(KernelStage::Sew, "domain")?;
    let mut axial = [f64::INFINITY, f64::NEG_INFINITY];
    for index in 0..=8 {
        let z = base
            .evaluate(t0 + (t1 - t0) * index as f64 / 8.0)
            .or_refuse(KernelStage::Sew, "evaluate")?
            .dot(direction);
        axial[0] = axial[0].min(z);
        axial[1] = axial[1].max(z);
    }
    if axial[1] - axial[0] > band {
        // Slanted profiles sweep skewed slabs whose axial union is not an
        // interval; keep the merge to perpendicular profiles.
        return Ok(None);
    }
    let start = (axial[0] + axial[1]) / 2.0;
    Ok(Some(ExtrusionSupport {
        base,
        direction,
        interval: [start, start + translation.length()],
    }))
}

fn perpendicular_profile(curve: &NurbsCurve, direction: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point().or_refuse(KernelStage::Sew, "point")?;
            Ok(Vec4::from_point(
                point.sub(direction.scale(point.dot(direction))),
                control.w,
            ))
        })
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls)
        .or_refuse(KernelStage::Sew, "NurbsCurve::new")
}

fn curves_coincide(
    first: &NurbsCurve,
    second: &NurbsCurve,
    band: f64,
) -> Result<bool, KernelRefusal> {
    for (from, to) in [(first, second), (second, first)] {
        let [t0, t1] = from.domain().or_refuse(KernelStage::Sew, "domain")?;
        for index in 0..=8 {
            let point = from
                .evaluate(t0 + (t1 - t0) * index as f64 / 8.0)
                .or_refuse(KernelStage::Sew, "evaluate")?;
            let projection = match crate::project_point_to_curve(to, point) {
                Ok(projection) => projection,
                Err(_) => return Ok(false),
            };
            if projection.distance > band {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Matching linear sweeps: parallel directions, identical perpendicular
/// profile, adjacent (or touching) axial intervals.
fn coextrusion_pair(
    older: &FaceRecord,
    newer: &FaceRecord,
    tolerance: f64,
) -> Result<Option<(ExtrusionSupport, [f64; 2])>, KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_MERGE").is_ok();
    let Some(support_a) = ruled_extrusion_support(older, tolerance)? else {
        return Ok(None);
    };
    let Some(support_b) = ruled_extrusion_support(newer, tolerance)? else {
        return Ok(None);
    };
    let alignment = support_a.direction.dot(support_b.direction);
    if alignment.abs() < 1.0 - 1e-9 {
        if debug {
            eprintln!(
                "coextrusion {}x{}: directions differ (dot {alignment})",
                older.id, newer.id
            );
        }
        return Ok(None);
    }
    let scale = 1.0f64
        .max(support_a.interval[1].abs())
        .max(support_b.interval[1].abs());
    let band = (tolerance * 100.0).max(1e-7 * scale);
    let profile_a = perpendicular_profile(&support_a.base, support_a.direction)?;
    let profile_b = perpendicular_profile(&support_b.base, support_a.direction)?;
    if !curves_coincide(&profile_a, &profile_b, band)? {
        if debug {
            eprintln!("coextrusion {}x{}: profiles differ", older.id, newer.id);
        }
        return Ok(None);
    }
    // Express B's interval along A's direction regardless of B's own sweep
    // orientation: anti-parallel sweeps negate the axial coordinate.
    let interval_b = if alignment < 0.0 {
        [-support_b.interval[1], -support_b.interval[0]]
    } else {
        support_b.interval
    };
    if interval_b[0] > support_a.interval[1] + band || support_a.interval[0] > interval_b[1] + band
    {
        if debug {
            eprintln!(
                "coextrusion {}x{}: axial gap {:?} vs {:?}",
                older.id, newer.id, support_a.interval, interval_b
            );
        }
        return Ok(None);
    }
    let union = [
        support_a.interval[0].min(interval_b[0]),
        support_a.interval[1].max(interval_b[1]),
    ];
    Ok(Some((support_a, union)))
}

fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if t0 > start + epsilon && t0 < end - epsilon {
        result = result.split(t0).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Sew, "domain")?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        result = result.split(t1).or_refuse(KernelStage::Sew, "split")?.0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in face_merge trimmed");
    }
    Ok(result)
}

/// The axial extent of `curve` about `support`'s axis, read off its control
/// polygon: the axial coordinate is an affine functional of position and
/// the weights are positive, so the curve's own extent lies inside its
/// control points'. Nine samples per edge missed a sphere-cut rim's peak by
/// 0.14 (BadBoolean, master's gate of 2026-10-13); the carrier built that
/// short had the projector fold the rim onto its top boundary, and the body
/// read 0.14 low in volume with the validator none the wiser.
fn axial_hull(curve: &NurbsCurve, support: &CylinderSupport) -> [f64; 2] {
    let origin_axial = support.origin.dot(support.axis);
    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    for point in &curve.control_points {
        let axial = Vec3::new(point.x / point.w, point.y / point.w, point.z / point.w)
            .dot(support.axis)
            - origin_axial;
        range[0] = range[0].min(axial);
        range[1] = range[1].max(axial);
    }
    range
}

/// An affine plane chart, proven from its complete bilinear control net.
/// Rational or warped charts need the general projection path instead.
struct AffinePlaneChart {
    origin: Vec3,
    du: Vec3,
    dv: Vec3,
    u0: f64,
    v0: f64,
}

fn affine_plane_chart(surface: &NurbsSurface) -> Option<AffinePlaneChart> {
    if surface.degree_u != 1 || surface.degree_v != 1
        || surface.control_points.len() != 2
        || surface.control_points.iter().any(|row| row.len() != 2)
        || surface.knots_u.len() != 4 || surface.knots_v.len() != 4
    { return None; }
    let ku = &surface.knots_u;
    let kv = &surface.knots_v;
    if ku[0] != ku[1] || ku[2] != ku[3] || kv[0] != kv[1] || kv[2] != kv[3]
        || ku[2] <= ku[1] || kv[2] <= kv[1]
    { return None; }
    let weight = surface.control_points[0][0].w;
    if !weight.is_finite() || weight.abs() <= 1e-300
        || surface.control_points.iter().flatten().any(|point| point.w != weight)
    { return None; }
    let p00 = surface.control_points[0][0].point().ok()?;
    let p10 = surface.control_points[1][0].point().ok()?;
    let p01 = surface.control_points[0][1].point().ok()?;
    let p11 = surface.control_points[1][1].point().ok()?;
    let u = p10.sub(p00);
    let v = p01.sub(p00);
    let scale = u.length().max(v.length()).max(1.0);
    if p11.sub(p10).sub(v).length() > 1e-12 * scale { return None; }
    let du = u.scale(1.0 / (ku[2] - ku[1]));
    let dv = v.scale(1.0 / (kv[2] - kv[1]));
    let determinant = du.dot(du) * dv.dot(dv) - du.dot(dv).powi(2);
    if !determinant.is_finite() || determinant <= 1e-12 * du.dot(du) * dv.dot(dv) {
        return None;
    }
    Some(AffinePlaneChart { origin: p00, du, dv, u0: ku[1], v0: kv[1] })
}

/// Transfer the existing trimmed locus, rather than replacing it with the
/// shared edge's potentially displaced fitted curve. Affine reparameterization
/// preserves every rational weight, knot and degree, including controls outside
/// the surface's finite patch.
fn transfer_affine_planar_pcurve(
    source: &NurbsSurface,
    target: &NurbsSurface,
    curve: &NurbsCurve,
    tolerance: f64,
) -> Option<NurbsCurve> {
    let source = affine_plane_chart(source)?;
    let target = affine_plane_chart(target)?;
    let uu = target.du.dot(target.du);
    let uv = target.du.dot(target.dv);
    let vv = target.dv.dot(target.dv);
    let determinant = uu * vv - uv * uv;
    let mut transferred = curve.clone();
    for control in &mut transferred.control_points {
        if !control.w.is_finite() || control.w.abs() <= 1e-300 { return None; }
        let point = source.origin
            .add(source.du.scale(control.x / control.w - source.u0))
            .add(source.dv.scale(control.y / control.w - source.v0));
        let relative = point.sub(target.origin);
        let a = relative.dot(target.du);
        let b = relative.dot(target.dv);
        let u = (a * vv - b * uv) / determinant;
        let v = (b * uu - a * uv) / determinant;
        let reconstructed = target.origin.add(target.du.scale(u)).add(target.dv.scale(v));
        if !u.is_finite() || !v.is_finite()
            || reconstructed.sub(point).length() > tolerance.max(1e-12)
        { return None; }
        control.x = (target.u0 + u) * control.w;
        control.y = (target.v0 + v) * control.w;
        control.z = 0.0;
    }
    Some(transferred)
}

fn transfer_planar_loops(
    target: &NurbsSurface,
    loops: &[LoopRecord],
    older: &FaceRecord,
    newer: &FaceRecord,
    tolerance: f64,
) -> Option<Vec<LoopRecord>> {
    let mut transferred = loops.to_vec();
    for loop_record in &mut transferred {
        for coedge in &mut loop_record.coedges {
            let source = [older, newer].into_iter().find(|face| {
                face.loops.iter().flat_map(|lp| &lp.coedges)
                    .any(|original| original.id == coedge.id && original.edge_id == coedge.edge_id)
            })?;
            coedge.pcurve = transfer_affine_planar_pcurve(
                &source.surface, target, &coedge.pcurve, tolerance,
            )?;
        }
    }
    Some(transferred)
}

fn reproject_loops(
    surface: &NurbsSurface,
    loops: Vec<LoopRecord>,
    edges: &HashMap<u64, &EdgeRecord>,
) -> Result<Vec<LoopRecord>, KernelRefusal> {
    let mut projected_loops = Vec::new();
    for (loop_index, loop_record) in loops.into_iter().enumerate() {
        let mut projected = Vec::new();
        for mut coedge in loop_record.coedges {
            let edge = edges[&coedge.edge_id];
            let mut curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
            if !coedge.forward {
                curve = curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
            }
            coedge.pcurve = build_pcurve_on_surface(surface, &curve)
                .or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
            projected.push(coedge);
        }
        projected_loops.push(LoopRecord {
            id: loop_index as u64 + 1,
            coedges: projected,
        });
    }
    Ok(projected_loops)
}

fn traversal_vertices(edge: &EdgeRecord, coedge: &CoedgeRecord) -> (u64, u64) {
    if coedge.forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    }
}

/// The parameter space every leftover coedge's pcurve lives in, when the merged
/// faces share ONE carrier. Only then can a merge read a loop's parameter-space
/// continuity; a reprojecting lane walks before its pcurves share a space.
struct SharedParameterSpace<'a> {
    surface: &'a NurbsSurface,
    /// The radius that made two fitted endpoints one vertex.
    weld: f64,
}

impl SharedParameterSpace<'_> {
    fn coincide(&self, first: Vec3, second: Vec3) -> bool {
        let band = crate::classification::surface_uv_band(self.surface, first.x, first.y, self.weld);
        (first.x - second.x).abs() <= band && (first.y - second.y).abs() <= band
    }
}

fn shared_trim_certificate_enabled() -> bool {
    std::env::var("BREP_FACE_MERGE_TRIM_CERT").as_deref() != Ok("0")
}

/// Prove closure of the complete boundary isocurves. Clamping makes these
/// the first/last control rows; their common positive rational basis bounds
/// the entire difference by the largest paired control-point displacement.
fn closed_control_boundary(surface: &NurbsSurface, along_u: bool, weld: f64) -> bool {
    let (degree, knots, lo, hi) = if along_u {
        let Ok([lo,hi]) = surface.domain_u() else { return false; };
        (surface.degree_u, &surface.knots_u, lo, hi)
    } else {
        let Ok([lo,hi]) = surface.domain_v() else { return false; };
        (surface.degree_v, &surface.knots_v, lo, hi)
    };
    if knots.len() < 2*(degree+1)
        || !knots[..=degree].iter().all(|k| *k == lo)
        || !knots[knots.len()-degree-1..].iter().all(|k| *k == hi)
    { return false; }
    let agree = |a: Vec4, b: Vec4| {
        if a.w <= 0.0 || !a.w.is_finite() || a.w != b.w { return false; }
        let (Ok(a), Ok(b)) = (a.point(), b.point()) else { return false; };
        a.sub(b).length() <= weld
    };
    if along_u {
        let (Some(first), Some(last)) = (surface.control_points.first(), surface.control_points.last()) else { return false; };
        first.len() == last.len() && first.iter().zip(last).all(|(a,b)| agree(*a,*b))
    } else {
        surface.control_points.iter().all(|row| match (row.first(), row.last()) {
            (Some(a), Some(b)) => agree(*a,*b),
            _ => false,
        })
    }
}

/// Linear pcurves on affine boundary isocurves compose to polylines.
/// Matched positive-weight polygon controls therefore bound the COMPLETE
/// paired images by their largest 3D displacement, even when the other UV
/// coordinate differs by roundoff. No UV band or station sample is used.
fn affine_boundary_trim_pair(
    surface: &NurbsSurface,
    along_u: bool,
    first_at_low: bool,
    first: &NurbsCurve,
    reverse: &NurbsCurve,
    weld: f64,
) -> bool {
    let (degree, knots, bounds) = if along_u {
        (surface.degree_v, &surface.knots_v, surface.domain_v())
    } else {
        (surface.degree_u, &surface.knots_u, surface.domain_u())
    };
    let Ok([lo,hi]) = bounds else { return false; };
    if degree != 1 || knots.as_slice() != [lo,lo,hi,hi] || hi <= lo { return false; }
    let ends = if along_u {
        let (Some(a),Some(b)) = (surface.control_points.first(),surface.control_points.last()) else { return false; };
        if a.len()!=2 || b.len()!=2 { return false; }
        [a[0],a[1],b[0],b[1]]
    } else {
        if surface.control_points.len()!=2 { return false; }
        let (a,b) = (&surface.control_points[0],&surface.control_points[1]);
        let (Some(a0),Some(a1),Some(b0),Some(b1)) = (a.first(),a.last(),b.first(),b.last()) else { return false; };
        [*a0,*b0,*a1,*b1]
    };
    if ends[0].w<=0.0 || ends[0].w!=ends[1].w || ends[2].w<=0.0 || ends[2].w!=ends[3].w {
        return false;
    }
    let (Ok(a0),Ok(a1),Ok(b0),Ok(b1)) = (ends[0].point(),ends[1].point(),ends[2].point(),ends[3].point()) else { return false; };
    let (a0,a1,b0,b1) = if first_at_low { (a0,a1,b0,b1) } else { (b0,b1,a0,a1) };
    let image = |p: &Vec4, a: Vec3, b: Vec3| {
        let t = if along_u { p.y/p.w } else { p.x/p.w };
        a.add(b.sub(a).scale((t-lo)/(hi-lo)))
    };
    first.control_points.iter().zip(&reverse.control_points).all(|(a,b)| {
        let distance = image(a,a0,a1).sub(image(b,b0,b1)).length();
        distance.is_finite() && distance <= weld
    })
}

/// Prove that a retained shared pair is a periodic seam over its whole
/// trim. Positive common rational bases match the other parameter, or
/// affine boundary images bound matched linear polygons. Every periodic
/// coordinate control lies on opposite closed domain sides.
/// This qualifies retention after the existing coincidence check fails. It
/// does not change cancellation policy or widen a UV band. Work is linear
/// in the existing control arrays, with no sampling or refinement search.
fn periodic_shared_trim(
    space: &SharedParameterSpace,
    first: &CoedgeRecord,
    second: &CoedgeRecord,
) -> bool {
    if first.forward == second.forward { return false; }
    let Ok(reverse) = second.pcurve.reversed() else { return false; };
    let curve = &first.pcurve;
    if curve.degree != reverse.degree || curve.control_points.len() != reverse.control_points.len()
        || curve.control_points.is_empty()
    { return false; }
    let polygon = |c: &NurbsCurve| {
        c.degree == 1 && c.knots.len() == c.control_points.len()+2
            && c.knots[0] == c.knots[1]
            && c.knots[c.knots.len()-1] == c.knots[c.knots.len()-2]
            && c.knots[1..c.knots.len()-1].windows(2).all(|k| k[1] > k[0])
    };
    let polygon = polygon(curve) && polygon(&reverse);
    if (!polygon && curve.knots != reverse.knots)
        || !curve.control_points.iter().zip(&reverse.control_points).all(|(a,b)| {
            [a,b].iter().all(|p| p.w > 0.0 && p.w.is_finite()
                && (p.x/p.w).is_finite() && (p.y/p.w).is_finite())
                && (polygon || a.w == b.w)
        })
    { return false; }
    let coordinate = |p: &Vec4, u: bool| if u { p.x/p.w } else { p.y/p.w };
    let same_coordinate = |u: bool| curve.control_points.iter().zip(&reverse.control_points)
        .all(|(a,b)| if polygon { coordinate(a,u) == coordinate(b,u) }
            else if u { a.x == b.x } else { a.y == b.y });
    for u in [true,false] {
        let bounds = if u { space.surface.domain_u() } else { space.surface.domain_v() };
        let Ok([lo,hi]) = bounds else { continue; };
        let other_bounds = if u { space.surface.domain_v() } else { space.surface.domain_u() };
        let Ok([other_lo,other_hi]) = other_bounds else { continue; };
        // The closed-boundary control bound applies on its active domain.
        // Positive pcurve hulls must stay inside it; extrapolated traces
        // need a different proof and keep their existing face partition.
        if !curve.control_points.iter().chain(&reverse.control_points)
            .all(|p| coordinate(p,!u) >= other_lo && coordinate(p,!u) <= other_hi)
        { continue; }
        let opposite = [(lo,hi),(hi,lo)].iter().any(|(a,b)|
            curve.control_points.iter().all(|p| coordinate(p,u) == *a)
                && reverse.control_points.iter().all(|p| coordinate(p,u) == *b));
        if opposite && closed_control_boundary(space.surface, u, space.weld)
            && (same_coordinate(!u) || (polygon && affine_boundary_trim_pair(
                space.surface, u, coordinate(&curve.control_points[0],u)==lo,
                curve, &reverse, space.weld)))
        {
            return true;
        }
    }
    false
}

/// A coedge's pcurve at its start, middle and end. The pcurve runs in the
/// coedge's own direction, so its domain start is where the traversal starts.
fn pcurve_stations(coedge: &CoedgeRecord) -> Option<[Vec3; 3]> {
    let [t0, t1] = coedge.pcurve.domain().ok()?;
    Some([
        coedge.pcurve.evaluate(t0).ok()?,
        coedge.pcurve.evaluate((t0 + t1) / 2.0).ok()?,
        coedge.pcurve.evaluate(t1).ok()?,
    ])
}

/// SLIT DISSOLUTION. An edge whose two leftover uses run along the SAME
/// parameter-space segment, one each way, separates nothing: its ring on the
/// other side was the shared edge the merge just cancelled (a flanged hole's
/// annulus, slit to the ring it bridged to). Kept, the walk can only return
/// along it and close a loop of zero area. Two uses a PERIOD apart are a seam
/// the region wraps across, and stay.
fn dissolve_slits(
    remaining: &mut Vec<CoedgeRecord>,
    edges: &HashMap<u64, &EdgeRecord>,
    space: &SharedParameterSpace,
) {
    let mut uses = HashMap::<u64, Vec<usize>>::default();
    for (index, coedge) in remaining.iter().enumerate() {
        uses.entry(coedge.edge_id).or_default().push(index);
    }
    let slits = uses
        .iter()
        .filter(|(edge_id, indices)| {
            if indices.len() != 2 || edges[*edge_id].degenerate {
                return false;
            }
            let (first, second) = (&remaining[indices[0]], &remaining[indices[1]]);
            if first.forward == second.forward {
                return false;
            }
            let (Some(first), Some(second)) = (pcurve_stations(first), pcurve_stations(second)) else {
                return false;
            };
            space.coincide(first[0], second[2])
                && space.coincide(first[1], second[1])
                && space.coincide(first[2], second[0])
        })
        .map(|(edge_id, _)| *edge_id)
        .collect::<HashSet<_>>();
    remaining.retain(|coedge| !slits.contains(&coedge.edge_id));
}

/// Stitch a merged face's leftover coedges into loops, or `None` when the
/// stitching is not determined.
///
/// A loop can pass one VERTEX twice — a bridge's foot on the ring it joins —
/// so reaching the start vertex does not by itself close the loop, and a vertex
/// with several unused coedges leaving it does not name the next one. There the
/// walk follows the loop's PARAMETER-SPACE continuity when the coedges share a
/// carrier (`space`): the successor starts where the current pcurve ends, and a
/// seam's two images of one vertex a period apart are two different places.
/// Without a shared space, or with no single successor, the merge is refused
/// and the faces stay as they were assembled.
fn walk_cycles(
    mut remaining: Vec<CoedgeRecord>,
    edges: &HashMap<u64, &EdgeRecord>,
    space: Option<&SharedParameterSpace>,
) -> Option<Vec<LoopRecord>> {
    let maximum_cycle_length = remaining.len();
    let mut loops = Vec::new();
    while !remaining.is_empty() {
        let first = remaining.remove(0);
        let (start, mut end) = traversal_vertices(edges[&first.edge_id], &first);
        let mut cycle = vec![first];
        loop {
            let leaving = remaining
                .iter()
                .enumerate()
                .filter_map(|(index, coedge)| {
                    (traversal_vertices(edges[&coedge.edge_id], coedge).0 == end).then_some(index)
                })
                .collect::<Vec<_>>();
            let closes = end == start;
            let successor = match (leaving.as_slice(), closes) {
                ([], true) => None,
                ([only], false) => Some(*only),
                ([], false) => return None,
                _ => {
                    let space = space?;
                    let here = pcurve_stations(cycle.last()?)?[2];
                    let closing = closes && space.coincide(here, pcurve_stations(&cycle[0])?[0]);
                    let mut continuing = Vec::new();
                    for index in leaving {
                        if space.coincide(here, pcurve_stations(&remaining[index])?[0]) {
                            continuing.push(index);
                        }
                    }
                    match (continuing.as_slice(), closing) {
                        ([], true) => None,
                        ([only], false) => Some(*only),
                        _ => return None,
                    }
                }
            };
            let Some(index) = successor else {
                break;
            };
            let next = remaining.remove(index);
            end = traversal_vertices(edges[&next.edge_id], &next).1;
            cycle.push(next);
            if cycle.len() > maximum_cycle_length {
                return None;
            }
        }
        // A cycle that traverses every one of its edges both ways encloses
        // nothing, however its parameter-space image reads.
        let mut counts = HashMap::<u64, usize>::default();
        for coedge in &cycle {
            *counts.entry(coedge.edge_id).or_default() += 1;
        }
        if counts.values().all(|count| *count == 2) {
            return None;
        }
        loops.push(LoopRecord {
            id: loops.len() as u64 + 1,
            coedges: cycle,
        });
    }
    Some(loops)
}

/// Whether two faces ride ONE carrier — the same geometric surface, oriented
/// the same way out of the material — by any of the four readings this module
/// merges on: bit-identical patches, coplanar planes with the same outward
/// normal, cosurface cylinder patches, and coextruded profiles.
///
/// `pub(crate)` because the direct-edit rejoin asks the same question for the
/// same reason: a survivor face on the far side of a deleted band can only
/// take the near side's loops when the two are one carrier. It asks GEOMETRIC
/// cosurface rather than [`same_surface`] alone, because a mirrored twin is
/// the same surface reparametrized — reflected control points, flipped
/// `same_sense` — and bit-identity reads that as two.
pub(crate) fn faces_are_cosurface(
    first: &FaceRecord,
    second: &FaceRecord,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    if first.same_sense == second.same_sense
        && same_surface(&first.surface, &second.surface, tolerance)
    {
        return Ok(true);
    }
    if coplanar_with_same_outward_normal(first, second, tolerance)? {
        return Ok(true);
    }
    if cosurface_cylinder_pair(first, second, tolerance, MergeLanes::default())?.is_some() {
        return Ok(true);
    }
    Ok(coextrusion_pair(first, second, tolerance)?.is_some())
}

/// Degree-one clamped trims follow their control polygon regardless of knot
/// spacing. For higher degrees, equal positive rational bases make the
/// difference of the two affine images a convex combination of control-point
/// differences. Bound the whole
/// shared trim, rather than relying on cancellation of its area integral.
fn opposed_planar_trims_agree(
    older: &FaceRecord,
    first: &CoedgeRecord,
    newer: &FaceRecord,
    second: &CoedgeRecord,
    tolerance: f64,
) -> bool {
    let Ok(reverse) = second.pcurve.reversed() else {
        return false;
    };
    let curve = &first.pcurve;
    if curve.degree != reverse.degree || curve.control_points.len() != reverse.control_points.len()
    {
        return false;
    }
    let linear_polygon = |c: &NurbsCurve| {
        c.degree == 1
            && c.knots.len() == c.control_points.len() + 2
            && c.knots[0] == c.knots[1]
            && c.knots[c.knots.len() - 1] == c.knots[c.knots.len() - 2]
            && c.knots[1..c.knots.len() - 1]
                .windows(2)
                .all(|k| k[1] > k[0])
    };
    let polygon = linear_polygon(curve) && linear_polygon(&reverse);
    if !polygon && curve.knots != reverse.knots {
        return false;
    }
    curve
        .control_points
        .iter()
        .zip(&reverse.control_points)
        .all(|(a, b)| {
            if !a.w.is_finite()
                || !b.w.is_finite()
                || a.w <= 0.0
                || b.w <= 0.0
                || (!polygon && a.w != b.w)
            {
                return false;
            }
            let (Ok(a), Ok(b)) = (a.point(), b.point()) else {
                return false;
            };
            let (Ok(a), Ok(b)) = (
                older.surface.evaluate_extended(a.x, a.y),
                newer.surface.evaluate_extended(b.x, b.y),
            ) else {
                return false;
            };
            a.sub(b).length() <= tolerance
        })
}

/// Like [`decline!`], but leaves a labelled lane instead of the whole pair
/// merge, so a later lane can read the same pair again.
macro_rules! fall_through {
    ($older:expr, $newer:expr, $label:lifetime) => {{
        if std::env::var("BREP_DEBUG_MERGE").is_ok() {
            eprintln!(
                "merge {}x{}: lane left at face_merge.rs:{}",
                $older.id,
                $newer.id,
                line!()
            );
        }
        break $label;
    }};
}

/// Trace a declined pair merge under `BREP_DEBUG_MERGE` with the source line
/// that declined it, then return `Ok(None)`. A pair whose carriers MATCH can
/// still be declined a dozen lines later (shared-edge census, cycle walk,
/// area check); without the line the trace reads "MATCH" and then nothing,
/// which is how the E13 cosurface report (2026-10-13) hid its cause.
macro_rules! decline {
    ($older:expr, $newer:expr) => {{
        if std::env::var("BREP_DEBUG_MERGE").is_ok() {
            eprintln!(
                "merge {}x{}: declined at face_merge.rs:{}",
                $older.id,
                $newer.id,
                line!()
            );
        }
        return Ok(None);
    }};
}

fn try_merge_pair(
    older: &FaceRecord,
    newer: &FaceRecord,
    edges: &HashMap<u64, &EdgeRecord>,
    tolerance: f64,
    same_carrier_area_tolerance_ratio: f64,
) -> Result<Option<FaceRecord>, KernelRefusal> {
    try_merge_pair_lanes(
        older,
        newer,
        edges,
        tolerance,
        same_carrier_area_tolerance_ratio,
        MergeLanes::default(),
    )
}

/// [`try_merge_pair`] with the opt-in [`MergeLanes`].
fn try_merge_pair_lanes(
    older: &FaceRecord,
    newer: &FaceRecord,
    edges: &HashMap<u64, &EdgeRecord>,
    tolerance: f64,
    same_carrier_area_tolerance_ratio: f64,
    lanes: MergeLanes<'_>,
) -> Result<Option<FaceRecord>, KernelRefusal> {
    let same_carrier = older.same_sense == newer.same_sense
        && same_surface(&older.surface, &newer.surface, tolerance);
    // Oppositely oriented planar patches can form a folded, zero-thickness
    // overlap along a multi-edge seam. Cancelling that seam preserves the
    // signed region, not the sum of the two unsigned face areas.
    let cancel_planar_fold = older.same_sense != newer.same_sense
        && older.loops.len() == 1
        && newer.loops.len() == 1
        && same_surface(&older.surface, &newer.surface, tolerance)
        && older
            .surface
            .is_affine()
            .or_refuse(KernelStage::Sew, "is_affine")?
        && newer
            .surface
            .is_affine()
            .or_refuse(KernelStage::Sew, "is_affine")?;
    let coplanar_planar =
        !same_carrier && coplanar_with_same_outward_normal(older, newer, tolerance)?;
    let cosurface_cylinder = if !same_carrier && !coplanar_planar {
        cosurface_cylinder_pair(older, newer, tolerance, lanes)?
    } else {
        None
    };
    let cosurface_extrusion = if !same_carrier && !coplanar_planar && cosurface_cylinder.is_none() {
        coextrusion_pair(older, newer, tolerance)?
    } else {
        None
    };
    let older_coedges = older
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .collect::<Vec<_>>();
    let newer_coedges = newer
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .collect::<Vec<_>>();
    let space = (same_carrier || cancel_planar_fold).then(|| SharedParameterSpace {
        surface: &older.surface,
        weld: crate::tolerance::assembler_weld(tolerance),
    });
    let mut shared_older = HashSet::default();
    let mut shared_newer = HashSet::default();
    let mut shared_edges = HashSet::default();
    // Shared edges the merged face keeps, both ways: on ONE carrier, a pair
    // whose two images lie a period apart is the carrier's seam (the
    // dissolve_slits doctrine), and the union wraps across it. Cancelling it
    // tore the seam out of a shelled cylinder's wall against its own strip
    // (the E13 report, 2026-10-13) and the area check declined the pair. A
    // pair that touches ONLY across the seam still declines below: its union
    // is two regions a period apart in this carrier's chart, one face record
    // holding two discs (the slab-corner mirror union read V−E+F = 1 so).
    for first in &older_coedges {
        if edges[&first.edge_id].degenerate {
            continue;
        }
        for second in &newer_coedges {
            if first.edge_id != second.edge_id {
                continue;
            }
            if first.forward == second.forward {
                decline!(older, newer);
            }
            if cancel_planar_fold
                && !opposed_planar_trims_agree(older, first, newer, second, tolerance)
            {
                decline!(older, newer);
            }
            if same_carrier && !cancel_planar_fold {
                if let (Some(space), Some(here), Some(there)) =
                    (&space, pcurve_stations(first), pcurve_stations(second))
                {
                    let retraced = space.coincide(here[0], there[2])
                        && space.coincide(here[1], there[1])
                        && space.coincide(here[2], there[0]);
                    if !retraced {
                        // A noncoincident pair may be retained only as a
                        // proved periodic seam. An interior partition stays
                        // between faces; retaining it here creates a slit.
                        if shared_trim_certificate_enabled()
                            && !periodic_shared_trim(space, first, second)
                        { decline!(older, newer); }
                        continue;
                    }
                } else if shared_trim_certificate_enabled() {
                    decline!(older, newer);
                }
            }
            shared_older.insert(first.id);
            shared_newer.insert(second.id);
            shared_edges.insert(first.edge_id);
        }
    }
    if shared_older.is_empty() || shared_older.len() != shared_newer.len() {
        decline!(older, newer);
    }
    if cancel_planar_fold && shared_older.len() < 2 {
        decline!(older, newer);
    }
    if !same_carrier
        && !cancel_planar_fold
        && !coplanar_planar
        && cosurface_cylinder.is_none()
        && cosurface_extrusion.is_none()
    {
        decline!(older, newer);
    }
    let mut remaining = older_coedges
        .into_iter()
        .filter(|coedge| {
            !shared_older.contains(&coedge.id)
                && !(coplanar_planar && edges[&coedge.edge_id].degenerate)
        })
        .chain(newer_coedges.into_iter().filter(|coedge| {
            !shared_newer.contains(&coedge.id)
                && !(coplanar_planar && edges[&coedge.edge_id].degenerate)
        }))
        .cloned()
        .collect::<Vec<_>>();
    if cancel_planar_fold {
        // The opposite fold shares this affine chart too. A retraced slit
        // contributes no signed boundary, but three coincident samples do
        // not prove that: opposite bows can match there and enclose slivers.
        // Use the whole-curve control bound before discarding either use.
        let mut uses = HashMap::<u64, Vec<&CoedgeRecord>>::default();
        for coedge in &remaining {
            uses.entry(coedge.edge_id).or_default().push(coedge);
        }
        let mut slits = HashSet::default();
        for (id, uses) in uses {
            if uses.len() == 2
                && !edges[&id].degenerate
                && uses[0].forward != uses[1].forward
                && opposed_planar_trims_agree(older, uses[0], older, uses[1], tolerance)
            {
                slits.insert(id);
            }
        }
        remaining.retain(|coedge| !slits.contains(&coedge.edge_id));
    } else if let Some(space) = &space {
        dissolve_slits(&mut remaining, edges, space);
    }
    let Some(mut loops) = walk_cycles(remaining, edges, space.as_ref()) else {
        // Cancelling the shared edges of a cosurface cylinder pair whose
        // union wraps the full turn can strand one face's seam strip at a
        // fork the 3D walk cannot resolve (the E13 report, 2026-10-13); the
        // full-turn lane walks in the merged carrier's own parameter space.
        if let Some(support) = &cosurface_cylinder {
            if let Some(merged) =
                merge_cylinder_full_turn(older, newer, edges, tolerance, support, &shared_edges, lanes)?
            {
                return Ok(Some(merged));
            }
        }
        decline!(older, newer);
    };
    if loops.iter().any(|loop_record| {
        loop_record
            .coedges
            .iter()
            .all(|coedge| edges[&coedge.edge_id].degenerate)
    }) {
        decline!(older, newer);
    }
    if loops.is_empty() {
        decline!(older, newer);
    }
    loops.sort_by(|first, second| {
        let area = |loop_record: &LoopRecord| {
            parameter_space_area(&FaceRecord {
                id: older.id,
                surface: older.surface.clone(),
                same_sense: older.same_sense,
                loops: vec![loop_record.clone()],
                name: None,
            })
            .map(f64::abs)
        };
        match (area(first), area(second)) {
            (Ok(a), Ok(b)) => b.total_cmp(&a),
            _ => std::cmp::Ordering::Equal,
        }
    });
    if same_carrier || cancel_planar_fold {
        if cancel_planar_fold && loops.len() != 1 {
            decline!(older, newer);
        }
        let expected = parameter_space_area(older)
            .or_refuse(KernelStage::Sew, "parameter_space_area")?
            + parameter_space_area(newer).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        if cancel_planar_fold && expected.abs() <= tolerance * tolerance {
            decline!(older, newer);
        }
        let merged = FaceRecord {
            id: older.id,
            surface: older.surface.clone(),
            same_sense: if cancel_planar_fold {
                expected > 0.0
            } else {
                older.same_sense
            },
            loops,
            name: older.name.clone(),
        };

        let actual =
            parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        let area_ratio = if cancel_planar_fold {
            same_carrier_area_tolerance_ratio.min(1e-7)
        } else {
            same_carrier_area_tolerance_ratio
        };
        let area_tolerance = 1e-9f64.max(expected.abs() * area_ratio);
        if (actual - expected).abs() > area_tolerance
            || (actual.abs() > 1e-12 && actual.is_sign_positive() != merged.same_sense)
        {
            decline!(older, newer);
        }
        return Ok(Some(merged));
    }

    if let Some(support) = &cosurface_cylinder {
        // The partial-turn lane first: a union that leaves an angular gap
        // takes a carrier seamed in that gap and never crosses the seam.
        // Whatever it declines (a sampled gap that is not one, a region that
        // wraps the full turn) the full-turn lane reads again on a closed
        // carrier seamed along an existing axial edge.
        'partial_turn: {
            // The partial-turn carrier is built at one radius: a cone goes
            // straight to the full-turn lane, which reads it per end.
            if support.slope != 0.0 {
                fall_through!(older, newer, 'partial_turn);
            }
            // Build one revolution carrier covering both patches: seam at the
            // largest angular gap so the merged region never crosses it, then
            // reproject every loop onto the shared cylinder.
            let x_reference = {
                let seed = loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .find_map(|coedge| {
                        let edge = edges[&coedge.edge_id];
                        edge.curve.evaluate(edge.t0).ok()
                    })
                    .ok_or_else(|| {
                        KernelRefusal::internal(
                            KernelStage::Sew,
                            "face_merge",
                            "cylinder merge: no boundary sample",
                        )
                    })?;
                let delta = seed.sub(support.origin);
                let radial = delta.sub(support.axis.scale(delta.dot(support.axis)));
                match radial.normalized() {
                    Ok(radial) => radial,
                    Err(_) => fall_through!(older, newer, 'partial_turn),
                }
            };
            let y_reference = support.axis.cross(x_reference);
            let mut angles = Vec::new();
            let mut axial_range = [f64::INFINITY, f64::NEG_INFINITY];
            for coedge in loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                let edge = edges[&coedge.edge_id];
                let curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
                let [c0, c1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let hull = axial_hull(&curve, support);
                axial_range[0] = axial_range[0].min(hull[0]);
                axial_range[1] = axial_range[1].max(hull[1]);
                for index in 0..=8 {
                    let point = curve
                        .evaluate(c0 + (c1 - c0) * index as f64 / 8.0)
                        .or_refuse(KernelStage::Sew, "evaluate")?;
                    let delta = point.sub(support.origin);
                    let axial = delta.dot(support.axis);
                    let radial = delta.sub(support.axis.scale(axial));
                    if radial.length() <= support.radius * 1e-6 {
                        continue;
                    }
                    let mut angle = radial.dot(y_reference).atan2(radial.dot(x_reference));
                    if angle < 0.0 {
                        angle += std::f64::consts::TAU;
                    }
                    angles.push(angle);
                }
            }
            if angles.len() < 3 || axial_range[1] - axial_range[0] <= tolerance {
                fall_through!(older, newer, 'partial_turn);
            }
            angles.sort_by(f64::total_cmp);
            let mut gap_start = angles.len() - 1;
            let mut largest_gap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
            for index in 0..angles.len() - 1 {
                let gap = angles[index + 1] - angles[index];
                if gap > largest_gap {
                    largest_gap = gap;
                    gap_start = index;
                }
            }
            let span = std::f64::consts::TAU - largest_gap;
            if span <= tolerance || span >= std::f64::consts::TAU - 1e-6 {
                fall_through!(older, newer, 'partial_turn);
            }
            let start_angle = angles[(gap_start + 1) % angles.len()];
            let start_direction = x_reference
                .scale(start_angle.cos())
                .add(y_reference.scale(start_angle.sin()));
            let generatrix = make_line(
                support
                    .origin
                    .add(support.axis.scale(axial_range[0]))
                    .add(start_direction.scale(support.radius)),
                support
                    .origin
                    .add(support.axis.scale(axial_range[1]))
                    .add(start_direction.scale(support.radius)),
            )
            .or_refuse(KernelStage::Sew, "healing.face_merge")?;
            let surface = make_revolution(support.origin, support.axis, &generatrix, span)
                .or_refuse(KernelStage::Sew, "make_revolution")?;
            let (u_domain, v_domain) = surface_domains(&surface)?;
            let sample = surface
                .evaluate(
                    (u_domain[0] + u_domain[1]) / 2.0,
                    (v_domain[0] + v_domain[1]) / 2.0,
                )
                .or_refuse(KernelStage::Sew, "healing.face_merge")?;
            let geometric_normal = surface
                .normal(
                    (u_domain[0] + u_domain[1]) / 2.0,
                    (v_domain[0] + v_domain[1]) / 2.0,
                )
                .or_refuse(KernelStage::Sew, "healing.face_merge")?;
            let delta = sample.sub(support.origin);
            let radial = delta.sub(support.axis.scale(delta.dot(support.axis)));
            let radial = match radial.normalized() {
                Ok(radial) => radial,
                Err(_) => fall_through!(older, newer, 'partial_turn),
            };
            let same_sense = geometric_normal.dot(radial).signum() == support.outward_radial_sign;
            let mut projected_loops = reproject_loops(&surface, loops, edges)?;
            projected_loops.sort_by(|first, second| {
                let area = |loop_record: &LoopRecord| {
                    parameter_space_area(&FaceRecord {
                        id: older.id,
                        surface: surface.clone(),
                        same_sense,
                        loops: vec![loop_record.clone()],
                        name: None,
                    })
                    .map(f64::abs)
                };
                match (area(first), area(second)) {
                    (Ok(a), Ok(b)) => b.total_cmp(&a),
                    _ => std::cmp::Ordering::Equal,
                }
            });
            let merged = FaceRecord {
                id: older.id,
                surface,
                same_sense,
                loops: projected_loops,
                name: older.name.clone(),
            };
            let area =
                parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
            if area.abs() <= tolerance * tolerance || area.is_sign_positive() != merged.same_sense {
                fall_through!(older, newer, 'partial_turn);
            }
            // The reprojected trims must reproduce the pieces' combined 3D
            // area; a projection or seam mistake shows up here immediately.
            let expected = face_area(older).or_refuse(KernelStage::Sew, "face_area")?
                + face_area(newer).or_refuse(KernelStage::Sew, "face_area")?;
            let actual = face_area(&merged).or_refuse(KernelStage::Sew, "face_area")?;
            if (actual - expected).abs() > expected.abs().max(1e-9) * 5e-3 {
                fall_through!(older, newer, 'partial_turn);
            }
            return Ok(Some(merged));
        }
        if let Some(merged) =
            merge_cylinder_full_turn(older, newer, edges, tolerance, support, &shared_edges, lanes)?
        {
            return Ok(Some(merged));
        }
        decline!(older, newer);
    }

    if let Some((support, union)) = cosurface_extrusion {
        // Shared linear-sweep carrier: the older profile translated to the
        // union's start plane, extruded across the union interval.
        let shift = support.direction.scale(union[0] - support.interval[0]);
        let step = support.direction.scale(union[1] - union[0]);
        let rows = support
            .base
            .control_points
            .iter()
            .map(|control| -> Result<Vec<Vec4>, KernelRefusal> {
                let point = control
                    .point()
                    .or_refuse(KernelStage::Sew, "point")?
                    .add(shift);
                Ok(vec![
                    Vec4::from_point(point, control.w),
                    Vec4::from_point(point.add(step), control.w),
                ])
            })
            .collect::<Result<Vec<_>, _>>()?;
        let surface = NurbsSurface::new(
            support.base.degree,
            1,
            support.base.knots.clone(),
            vec![0.0, 0.0, 1.0, 1.0],
            rows,
        )
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
        let (older_u, older_v) = surface_domains(&older.surface)?;
        let older_mid = older
            .surface
            .evaluate(
                (older_u[0] + older_u[1]) / 2.0,
                (older_v[0] + older_v[1]) / 2.0,
            )
            .or_refuse(KernelStage::Sew, "healing.face_merge")?;
        let projection = crate::project_point_to_surface(&surface, older_mid)
            .or_refuse(KernelStage::Sew, "project_point_to_surface")?;
        let carrier_normal = surface
            .normal(projection.u, projection.v)
            .or_refuse(KernelStage::Sew, "normal")?;
        let same_sense = carrier_normal.dot(outward_normal(older)?) > 0.0;
        let mut projected_loops = reproject_loops(&surface, loops, edges)?;
        projected_loops.sort_by(|first, second| {
            let area = |loop_record: &LoopRecord| {
                parameter_space_area(&FaceRecord {
                    id: older.id,
                    surface: surface.clone(),
                    same_sense,
                    loops: vec![loop_record.clone()],
                    name: None,
                })
                .map(f64::abs)
            };
            match (area(first), area(second)) {
                (Ok(a), Ok(b)) => b.total_cmp(&a),
                _ => std::cmp::Ordering::Equal,
            }
        });
        let merged = FaceRecord {
            id: older.id,
            surface,
            same_sense,
            loops: projected_loops,
            name: older.name.clone(),
        };
        let area =
            parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        if area.abs() <= tolerance * tolerance || area.is_sign_positive() != merged.same_sense {
            decline!(older, newer);
        }
        let expected = face_area(older).or_refuse(KernelStage::Sew, "face_area")?
            + face_area(newer).or_refuse(KernelStage::Sew, "face_area")?;
        let actual = face_area(&merged).or_refuse(KernelStage::Sew, "face_area")?;
        if (actual - expected).abs() > expected.abs().max(1e-9) * 5e-3 {
            if std::env::var("BREP_DEBUG_MERGE").is_ok() {
                eprintln!(
                    "coextrusion {}x{}: area mismatch {actual} vs {expected}",
                    older.id, newer.id
                );
            }
            decline!(older, newer);
        }
        return Ok(Some(merged));
    }

    let (u_domain, v_domain) = surface_domains(&older.surface)?;
    let origin = older
        .surface
        .evaluate(u_domain[0], v_domain[0])
        .or_refuse(KernelStage::Sew, "evaluate")?;
    let derivatives = older
        .surface
        .derivatives(
            (u_domain[0] + u_domain[1]) / 2.0,
            (v_domain[0] + v_domain[1]) / 2.0,
            1,
        )
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
    let u_axis = derivatives[1][0]
        .normalized()
        .or_refuse(KernelStage::Sew, "normalized")?;
    let geometric_normal = derivatives[1][0]
        .cross(derivatives[0][1])
        .normalized()
        .or_refuse(KernelStage::Sew, "normalized")?;
    let v_axis = geometric_normal
        .cross(u_axis)
        .normalized()
        .or_refuse(KernelStage::Sew, "normalized")?;
    let mut bounds_u = [f64::INFINITY, f64::NEG_INFINITY];
    let mut bounds_v = [f64::INFINITY, f64::NEG_INFINITY];
    for coedge in loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let edge = edges[&coedge.edge_id];
        let curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
        for control in &curve.control_points {
            let relative = control
                .point()
                .or_refuse(KernelStage::Sew, "point")?
                .sub(origin);
            let u = relative.dot(u_axis);
            let v = relative.dot(v_axis);
            bounds_u[0] = bounds_u[0].min(u);
            bounds_u[1] = bounds_u[1].max(u);
            bounds_v[0] = bounds_v[0].min(v);
            bounds_v[1] = bounds_v[1].max(v);
        }
    }
    let extent_u = bounds_u[1] - bounds_u[0];
    let extent_v = bounds_v[1] - bounds_v[0];
    if extent_u <= tolerance || extent_v <= tolerance {
        decline!(older, newer);
    }
    let surface_origin = origin
        .add(u_axis.scale(bounds_u[0]))
        .add(v_axis.scale(bounds_v[0]));
    let surface = make_plane(surface_origin, u_axis, v_axis, extent_u, extent_v)
        .or_refuse(KernelStage::Sew, "make_plane")?;
    let mut projected_loops = match transfer_planar_loops(&surface, &loops, older, newer, tolerance)
    {
        Some(transferred) => transferred,
        None => reproject_loops(&surface, loops, edges)?,
    };
    projected_loops.sort_by(|first, second| {
        let area = |loop_record: &LoopRecord| {
            parameter_space_area(&FaceRecord {
                id: older.id,
                surface: surface.clone(),
                same_sense: older.same_sense,
                loops: vec![loop_record.clone()],
                name: None,
            })
            .map(f64::abs)
        };
        match (area(first), area(second)) {
            (Ok(a), Ok(b)) => b.total_cmp(&a),
            _ => std::cmp::Ordering::Equal,
        }
    });
    let merged = FaceRecord {
        id: older.id,
        surface,
        same_sense: older.same_sense,
        loops: projected_loops,
        name: older.name.clone(),
    };
    let area = parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
    if area.abs() <= tolerance * tolerance || area.is_sign_positive() != merged.same_sense {
        decline!(older, newer);
    }
    Ok(Some(merged))
}

/// Angle of `point` about `support`'s axis in the (`x_reference`,
/// `y_reference`) frame, in `[0, TAU)`; `None` on the axis.
fn cylinder_angle(
    support: &CylinderSupport,
    x_reference: Vec3,
    y_reference: Vec3,
    point: Vec3,
) -> Option<f64> {
    let delta = point.sub(support.origin);
    let radial = delta.sub(support.axis.scale(delta.dot(support.axis)));
    if radial.length() <= support.radius * 1e-6 {
        return None;
    }
    let mut angle = radial.dot(y_reference).atan2(radial.dot(x_reference));
    if angle < 0.0 {
        angle += std::f64::consts::TAU;
    }
    // An atan2 a few ulp below zero rounds to TAU itself after the shift; the
    // range is half-open, so that reading is the seam angle 0.
    if angle >= std::f64::consts::TAU {
        angle = 0.0;
    }
    Some(angle)
}

/// Signed angular distance from `from` to `to`, in `(-PI, PI]`.
fn angle_difference(from: f64, to: f64) -> f64 {
    let turn = (to - from).rem_euclid(std::f64::consts::TAU);
    if turn > std::f64::consts::PI {
        turn - std::f64::consts::TAU
    } else {
        turn
    }
}

/// Merge two cosurface cylinder patches whose union wraps the FULL turn.
///
/// The partial-turn lane seams its carrier in the union's largest angular
/// gap; a union with no gap (the shelled cylinder's full outer wall against
/// the half-walls of a coaxial extrude, the E13 report of 2026-10-13) has no
/// such seam, and cancelling every shared edge there strands the full face's
/// seam strip at a fork the 3D walk cannot resolve. This lane builds a closed
/// revolution carrier seamed along an existing AXIAL edge that both faces
/// already traverse twice — one face's own seam pair, or a shared pair — keeps
/// the shared edges on that line as seam pairs instead of cancelling them,
/// places every seam-line trim on the side of the seam its traversal puts the
/// material on, and walks the leftover uses in the carrier's own parameter
/// space. Only a seam one of the faces ALREADY has is a candidate: a face
/// that wraps the turn by itself carries its own seam pair, and the merged
/// face keeps that seam. Two half-turn patches closing a ring between them
/// have no such seam; they were two faces before any boolean touched them
/// (a sketch circle's two arcs, a tube piece's two halves, a plate hole's
/// two bore segments whose names a flange picks) and they stay two faces
/// here — master's battery of 2026-10-13 read four regressions from merging
/// them. Each candidate is tried in turn, and the merged face must close
/// every loop in parameter space, keep every trim on its edge within the
/// validator's own pcurve acceptance, and reproduce the pieces'
/// parameter-space sign and their combined 3D area, or the candidate is
/// dropped; with no candidate left the pair stays as assembled.
///
/// What this lane does NOT do: two full rings seamed at different angles
/// (a shelled cylinder's bore wall and a coaxial extrude's hole wall whose
/// profile circle happened to start opposite) have no axial edge spanning
/// both, and merging them means re-seaming one through its far rim and the
/// cap on it — a solid-wide edit the face merge does not make. Such a pair
/// declines here with its loops open in parameter space and keeps its
/// circular joint.
fn merge_cylinder_full_turn(
    older: &FaceRecord,
    newer: &FaceRecord,
    edges: &HashMap<u64, &EdgeRecord>,
    tolerance: f64,
    support: &CylinderSupport,
    shared_edges: &HashSet<u64>,
    lanes: MergeLanes<'_>,
) -> Result<Option<FaceRecord>, KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_MERGE").is_ok();
    // `BREP_FULL_TURN_MERGE=0` hatches the lane off, for attribution only.
    if std::env::var("BREP_FULL_TURN_MERGE").as_deref() == Ok("0") {
        return Ok(None);
    }
    // Every non-degenerate use of either face, tagged with which face it came
    // from (0 older, 1 newer) so an edge used twice by ONE face reads as its
    // own seam.
    let uses = [older, newer]
        .iter()
        .enumerate()
        .flat_map(|(side, face)| {
            face.loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter(|coedge| !edges[&coedge.edge_id].degenerate)
                .map(move |coedge| (side, coedge.clone()))
        })
        .collect::<Vec<_>>();
    let Some(x_reference) = uses.iter().find_map(|(_, coedge)| {
        let edge = edges[&coedge.edge_id];
        let seed = edge.curve.evaluate(edge.t0).ok()?;
        let delta = seed.sub(support.origin);
        let radial = delta.sub(support.axis.scale(delta.dot(support.axis)));
        radial.normalized().ok()
    }) else {
        return Ok(None);
    };
    let y_reference = support.axis.cross(x_reference);
    let scale = 1.0f64.max(support.origin.length()).max(support.radius);
    let band = (tolerance * 100.0).max(1e-7 * scale);
    // The smallest radius the patches reach bounds the angle `band` subtends
    // (a cylinder's one radius; a cone's radius at its narrow end, read once
    // the axial range is known below and checked positive there).
    let mut angular_band = band / support.radius;

    // Each use's stations: axial coordinate and angle at nine samples, plus
    // the station of its start and end in traversal order.
    struct Station {
        axial: f64,
        angle: Option<f64>,
    }
    let station = |point: Vec3| Station {
        axial: point.sub(support.origin).dot(support.axis),
        angle: cylinder_angle(support, x_reference, y_reference, point),
    };
    let mut axial_range = [f64::INFINITY, f64::NEG_INFINITY];
    let mut samples = Vec::with_capacity(uses.len());
    let mut bbox_lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut bbox_hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    // The boundary length of both faces together: the most area a set of
    // trims admitted `band` off their edges can move, per unit of `band`.
    let mut perimeter = 0.0f64;
    for (_, coedge) in &uses {
        let edge = edges[&coedge.edge_id];
        let curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
        let [c0, c1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let hull = axial_hull(&curve, support);
        axial_range[0] = axial_range[0].min(hull[0]);
        axial_range[1] = axial_range[1].max(hull[1]);
        perimeter += crate::mass_properties::edge_arc_length(edge).unwrap_or(0.0);
        let mut stations = Vec::with_capacity(9);
        for index in 0..=8 {
            let point = curve
                .evaluate(c0 + (c1 - c0) * index as f64 / 8.0)
                .or_refuse(KernelStage::Sew, "evaluate")?;
            bbox_lo = Vec3::new(bbox_lo.x.min(point.x), bbox_lo.y.min(point.y), bbox_lo.z.min(point.z));
            bbox_hi = Vec3::new(bbox_hi.x.max(point.x), bbox_hi.y.max(point.y), bbox_hi.z.max(point.z));
            stations.push(station(point));
        }
        samples.push(stations);
    }
    if axial_range[1] - axial_range[0] <= tolerance {
        return Ok(None);
    }
    if support.slope != 0.0 {
        let narrow = support
            .radius_at(axial_range[0])
            .min(support.radius_at(axial_range[1]));
        if narrow <= band {
            return Ok(None);
        }
        angular_band = band / narrow;
    }
    // An edge is an axial line at angle `theta` when every sample reads that
    // angle: a straight generatrix of the shared cylinder.
    let axial_angle = |stations: &[Station]| -> Option<f64> {
        let first = stations.first()?.angle?;
        stations
            .iter()
            .all(|station| {
                station
                    .angle
                    .is_some_and(|angle| angle_difference(first, angle).abs() <= angular_band)
            })
            .then_some(first)
    };
    let mut use_count = HashMap::<u64, usize>::default();
    for (_, coedge) in &uses {
        *use_count.entry(coedge.edge_id).or_default() += 1;
    }
    // Candidate seam angles, ranked: a face's own seam pair (both uses from
    // one face) before a shared pair; a line used once bounds the region on
    // one side only and cannot seam a wrap by itself.
    let mut candidates: Vec<(u8, f64)> = Vec::new();
    // Candidate angles whose edge ends at a vertex `lanes.seam_vertices` names.
    let mut preferred: Vec<f64> = Vec::new();
    for (index, (side, coedge)) in uses.iter().enumerate() {
        if use_count.get(&coedge.edge_id).copied() != Some(2) {
            continue;
        }
        let Some(theta) = axial_angle(&samples[index]) else {
            continue;
        };
        let own_seam = uses
            .iter()
            .any(|(other_side, other)| {
                other.edge_id == coedge.edge_id
                    && other.id != coedge.id
                    && other_side == side
                    && other.forward != coedge.forward
            });
        let rank = if own_seam { 0 } else { 1 };
        if let Some(vertices) = lanes.seam_vertices {
            let edge = edges[&coedge.edge_id];
            if vertices.contains(&edge.start_vertex_id) || vertices.contains(&edge.end_vertex_id) {
                preferred.push(theta);
            }
        }
        if !candidates
            .iter()
            .any(|(_, known)| angle_difference(*known, theta).abs() <= angular_band)
        {
            candidates.push((rank, theta));
        } else if let Some(known) = candidates
            .iter_mut()
            .find(|(_, known)| angle_difference(*known, theta).abs() <= angular_band)
        {
            known.0 = known.0.min(rank);
        }
    }
    let is_preferred = |theta: f64| {
        preferred
            .iter()
            .any(|known| angle_difference(*known, theta).abs() <= angular_band)
    };
    // `false` sorts first: a preferred seam leads its rank. With no
    // preference every key is `true` and the order is the plain one.
    candidates.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then(is_preferred(b.1).cmp(&is_preferred(a.1)))
            .then(a.1.total_cmp(&b.1))
    });
    if debug {
        eprintln!(
            "merge {}x{}: full-turn lane, {} seam candidate(s) {:?}",
            older.id,
            newer.id,
            candidates.len(),
            candidates
        );
    }
    // A shared pair alone (rank 1) is two half patches closing a ring between
    // them: neither face wraps, and the merged face would carry a seam no
    // input face had. Those pairs stay as assembled (see the doc comment).
    // A caller that opted into `shared_seam_ring` is joining faces the user
    // named as one region; there the shared pair seams the ring.
    if !lanes.shared_seam_ring {
        candidates.retain(|(rank, _)| *rank == 0);
    }
    if candidates.is_empty() {
        if debug {
            eprintln!(
                "merge {}x{}: full-turn lane: no face wraps the turn by itself, the halves stay apart",
                older.id, newer.id
            );
        }
        return Ok(None);
    }
    let weld = crate::tolerance::assembler_weld(tolerance);
    // The validator's own edge/pcurve acceptance on this pair's extent (its
    // size-coupled ceiling floors at `pcurve_consistency`); the pair's
    // diagonal is at most the solid's, so nothing admitted here is refused
    // by the closing validation on that count.
    let pcurve_limit = crate::KernelTolerances::for_scale(scale, 1e-7)
        .pcurve_acceptance(bbox_hi.sub(bbox_lo).length());
    let expected_area = face_area(older).or_refuse(KernelStage::Sew, "face_area")?
        + face_area(newer).or_refuse(KernelStage::Sew, "face_area")?;
    let expected_term = face_volume_contribution(older)
        .or_refuse(KernelStage::Sew, "face_volume_contribution")?
        + face_volume_contribution(newer).or_refuse(KernelStage::Sew, "face_volume_contribution")?;

    'candidate: for (_, theta) in candidates {
        let on_seam = |index: usize| {
            axial_angle(&samples[index])
                .is_some_and(|angle| angle_difference(theta, angle).abs() <= angular_band)
        };
        // Closed carrier seamed at `theta`, spanning both patches axially.
        let start_direction = x_reference
            .scale(theta.cos())
            .add(y_reference.scale(theta.sin()));
        let generatrix = make_line(
            support
                .origin
                .add(support.axis.scale(axial_range[0]))
                .add(start_direction.scale(support.radius_at(axial_range[0]))),
            support
                .origin
                .add(support.axis.scale(axial_range[1]))
                .add(start_direction.scale(support.radius_at(axial_range[1]))),
        )
        .or_refuse(KernelStage::Sew, "healing.face_merge")?;
        let surface = make_revolution(
            support.origin,
            support.axis,
            &generatrix,
            std::f64::consts::TAU,
        )
        .or_refuse(KernelStage::Sew, "make_revolution")?;
        let (u_domain, v_domain) = surface_domains(&surface)?;
        let (u_mid, v_mid) = (
            (u_domain[0] + u_domain[1]) / 2.0,
            (v_domain[0] + v_domain[1]) / 2.0,
        );
        let sample = surface
            .evaluate(u_mid, v_mid)
            .or_refuse(KernelStage::Sew, "healing.face_merge")?;
        let geometric_normal = surface
            .normal(u_mid, v_mid)
            .or_refuse(KernelStage::Sew, "healing.face_merge")?;
        let delta = sample.sub(support.origin);
        let radial = delta.sub(support.axis.scale(delta.dot(support.axis)));
        let Ok(radial) = radial.normalized() else {
            continue;
        };
        let same_sense = geometric_normal.dot(radial).signum() == support.outward_radial_sign;
        let v_of = |axial: f64| {
            v_domain[0]
                + (axial - axial_range[0]) / (axial_range[1] - axial_range[0])
                    * (v_domain[1] - v_domain[0])
        };

        // Leftover uses: a shared edge off the seam line is cancelled; one on
        // it stays, both ways, as the merged face's seam.
        let mut remaining = Vec::new();
        for (index, (_, coedge)) in uses.iter().enumerate() {
            let seam_line = on_seam(index);
            if shared_edges.contains(&coedge.edge_id) && !seam_line {
                continue;
            }
            let edge = edges[&coedge.edge_id];
            let mut coedge = coedge.clone();
            let mut curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
            if !coedge.forward {
                curve = curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
            }
            if seam_line {
                // The material lies to the LEFT of a same-sense loop's
                // traversal in uv (parameter-space area positive) and to the
                // right of an opposite-sense one. A seam-line trim running
                // +v with material on its left is the domain's far side
                // (u = u1); running -v, its near side (u = u0).
                let [c0, c1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let start = curve.evaluate(c0).or_refuse(KernelStage::Sew, "evaluate")?;
                let end = curve.evaluate(c1).or_refuse(KernelStage::Sew, "evaluate")?;
                let rising = end.sub(start).dot(support.axis) > 0.0;
                let far_side = rising == same_sense;
                let u = if far_side { u_domain[1] } else { u_domain[0] };
                // The trim carries the edge's OWN parameterisation: the
                // axial coordinate is an affine functional of position, so
                // mapping each control point (dehomogenised) and keeping the
                // degree, knots and weights reproduces v(t) exactly. A
                // two-point line assumed the edge ran its axis at constant
                // speed, which a spline tube's straight Hermite piece does
                // not (max deviation 0.5 at the validator, 2026-10-13).
                let origin_axial = support.origin.dot(support.axis);
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|point| {
                        let w = point.w;
                        let axial = Vec3::new(point.x / w, point.y / w, point.z / w)
                            .dot(support.axis)
                            - origin_axial;
                        Vec4 { x: u * w, y: v_of(axial) * w, z: 0.0, w }
                    })
                    .collect();
                coedge.pcurve = NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                    .or_refuse(KernelStage::Sew, "NurbsCurve::new")?;
            } else {
                // The kernel's fit, with its report read: a station standing
                // off the carrier (`off_surface`) is an edge this carrier
                // does not hold — the projector folds such a station onto
                // the carrier's boundary and the face becomes the wrong
                // region on a body `validate()` passes — and a residual over
                // the band is a trim that misses its edge between stations.
                let fit = match crate::fit_pcurve_on_surface(&surface, &curve) {
                    Ok(fit) => fit,
                    Err(_) => continue 'candidate,
                };
                if fit.report.off_surface > band || fit.report.residual > band {
                    if debug {
                        eprintln!(
                            "merge {}x{}: full-turn seam {theta:.4}: edge {} stands {:.3e} off the carrier, fit residual {:.3e} (band {band:.3e})",
                            older.id, newer.id, coedge.edge_id, fit.report.off_surface, fit.report.residual
                        );
                    }
                    continue 'candidate;
                }
                coedge.pcurve = fit.curve;
            }
            remaining.push(coedge);
        }
        let space = SharedParameterSpace { surface: &surface, weld };
        dissolve_slits(&mut remaining, edges, &space);
        if debug {
            for coedge in &remaining {
                let stations = pcurve_stations(coedge)
                    .map(|[a, m, b]| format!("({:.4},{:.4})->({:.4},{:.4})->({:.4},{:.4})", a.x, a.y, m.x, m.y, b.x, b.y))
                    .unwrap_or_default();
                eprintln!(
                    "  seam {theta:.4} use coedge {} edge {} fwd={} shared={} same_sense={same_sense} uv {stations}",
                    coedge.id,
                    coedge.edge_id,
                    coedge.forward,
                    shared_edges.contains(&coedge.edge_id)
                );
            }
        }
        let Some(mut loops) = walk_cycles(remaining, edges, Some(&space)) else {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: walk undetermined",
                    older.id, newer.id
                );
            }
            continue;
        };
        if loops.is_empty() {
            continue;
        }
        // Every loop must CLOSE in the carrier's parameter space: the walk
        // closes a cycle on vertex identity when nothing else leaves its
        // end, which on a seamed carrier accepts a full circle running one
        // period from its own vertex, or a ring whose own seam pair (off the
        // chosen seam) just dissolved as a slit. The area oracles below
        // integrate per pcurve and cannot see the grouping.
        let closed = loops.iter().all(|loop_record| {
            let count = loop_record.coedges.len();
            (0..count).all(|index| {
                let next = &loop_record.coedges[(index + 1) % count];
                match (
                    pcurve_stations(&loop_record.coedges[index]),
                    pcurve_stations(next),
                ) {
                    (Some(here), Some(there)) => space.coincide(here[2], there[0]),
                    _ => false,
                }
            })
        });
        if !closed {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: a loop is open in parameter space",
                    older.id, newer.id
                );
            }
            continue;
        }
        if debug {
            for loop_record in &loops {
                eprintln!(
                    "  seam {theta:.4} loop {:?}",
                    loop_record.coedges.iter().map(|c| (c.edge_id, c.forward)).collect::<Vec<_>>()
                );
            }
        }
        loops.sort_by(|first, second| {
            let area = |loop_record: &LoopRecord| {
                parameter_space_area(&FaceRecord {
                    id: older.id,
                    surface: surface.clone(),
                    same_sense,
                    loops: vec![loop_record.clone()],
                    name: None,
                })
                .map(f64::abs)
            };
            match (area(first), area(second)) {
                (Ok(a), Ok(b)) => b.total_cmp(&a),
                _ => std::cmp::Ordering::Equal,
            }
        });
        let merged = FaceRecord {
            id: older.id,
            surface,
            same_sense,
            loops,
            name: older.name.clone(),
        };
        // Every trim must lie on its edge by the validator's own reading;
        // a candidate that fails here falls through and the pair stays,
        // instead of the refusal surfacing from the closing validation as a
        // feature failure (the spline tube, 2026-10-13).
        let mut worst_trim = 0.0f64;
        for coedge in merged.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let edge = edges[&coedge.edge_id];
            let error = crate::topology::coedge_error_floored(
                &merged.surface,
                &coedge.pcurve,
                &edge.curve,
                edge,
                coedge.forward,
                pcurve_limit,
            )
            .unwrap_or(f64::INFINITY);
            worst_trim = worst_trim.max(error);
        }
        if worst_trim > pcurve_limit {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: a trim is {worst_trim:.6} off its edge (limit {pcurve_limit:.6})",
                    older.id, newer.id
                );
            }
            continue;
        }
        let area =
            parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        if area.abs() <= tolerance * tolerance || area.is_sign_positive() != merged.same_sense {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: parameter area {area:.3e} vs same_sense {same_sense}",
                    older.id, newer.id
                );
            }
            continue;
        }
        // The merged face must reproduce the pieces' combined 3D area and
        // their combined volume term about the origin. The bar is what the
        // admitted trims can move: `band` off their edges along the whole
        // boundary (and that much times the farthest point for the volume
        // term), plus the quadrature's drift. A 5e-3 share of the area let
        // a 0.24 sliver through (BadBoolean, 2026-10-13); the volume term
        // sees a region of the right area in the wrong place, which the
        // area cannot.
        let actual = face_area(&merged).or_refuse(KernelStage::Sew, "face_area")?;
        let area_bar = band * perimeter + 1e-9 * expected_area.abs();
        if (actual - expected_area).abs() > area_bar {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: area {actual:.9} vs expected {expected_area:.9} (bar {area_bar:.3e})",
                    older.id, newer.id
                );
            }
            continue;
        }
        let actual_term =
            face_volume_contribution(&merged).or_refuse(KernelStage::Sew, "face_volume_contribution")?;
        let reach = bbox_lo.length().max(bbox_hi.length());
        let term_bar = band * perimeter * reach / 3.0 + 1e-9 * expected_term.abs().max(expected_area.abs());
        if (actual_term - expected_term).abs() > term_bar {
            if debug {
                eprintln!(
                    "merge {}x{}: full-turn seam {theta:.4}: volume term {actual_term:.9} vs expected {expected_term:.9} (bar {term_bar:.3e})",
                    older.id, newer.id
                );
            }
            continue;
        }
        if debug {
            eprintln!(
                "merge {}x{}: full-turn MERGE at seam {theta:.4}, {} loop(s); area {actual:.9} vs {expected_area:.9} (bar {area_bar:.3e}), volume term {actual_term:.9} vs {expected_term:.9} (bar {term_bar:.3e})",
                older.id,
                newer.id,
                merged.loops.len()
            );
        }
        return Ok(Some(merged));
    }
    Ok(None)
}


fn try_merge_connected_group(
    faces: &[FaceRecord],
    edges: &HashMap<u64, &EdgeRecord>,
    tolerance: f64,
    same_carrier_area_tolerance_ratio: f64,
    keep_unmerged: &[String],
) -> Result<Option<(Vec<usize>, FaceRecord)>, KernelRefusal> {
    for seed in 0..faces.len() {
        // A pinned face never seeds or joins a merge group.
        if face_is_pinned(&faces[seed], keep_unmerged) {
            continue;
        }
        let mut component = [seed].into_iter().collect::<HashSet<_>>();
        loop {
            let mut changed = false;
            for candidate in 0..faces.len() {
                if component.contains(&candidate)
                    || face_is_pinned(&faces[candidate], keep_unmerged)
                    || faces[candidate].same_sense != faces[seed].same_sense
                    || !same_surface(&faces[candidate].surface, &faces[seed].surface, 1e-7)
                {
                    continue;
                }
                let candidate_uses = faces[candidate]
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges);
                let adjacent = candidate_uses.into_iter().any(|candidate_use| {
                    !edges[&candidate_use.edge_id].degenerate
                        && component.iter().any(|member| {
                            faces[*member]
                                .loops
                                .iter()
                                .flat_map(|loop_record| &loop_record.coedges)
                                .any(|member_use| {
                                    member_use.edge_id == candidate_use.edge_id
                                        && member_use.forward != candidate_use.forward
                                })
                        })
                });
                if adjacent {
                    component.insert(candidate);
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        if component.len() < 2 {
            continue;
        }

        let mut uses_by_edge = HashMap::<u64, Vec<CoedgeRecord>>::default();
        for index in &component {
            for coedge in faces[*index]
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
            {
                uses_by_edge
                    .entry(coedge.edge_id)
                    .or_default()
                    .push(coedge.clone());
            }
        }
        if uses_by_edge.values().any(|uses| uses.len() > 2) {
            continue;
        }
        let internal = uses_by_edge
            .iter()
            .filter_map(|(edge_id, uses)| {
                (uses.len() == 2
                    && !edges[edge_id].degenerate
                    && uses[0].forward != uses[1].forward)
                    .then_some(*edge_id)
            })
            .collect::<HashSet<_>>();
        let remaining = uses_by_edge
            .into_iter()
            .filter(|(edge_id, _)| !internal.contains(edge_id))
            .flat_map(|(_, uses)| uses)
            .collect::<Vec<_>>();
        let space = SharedParameterSpace {
            surface: &faces[seed].surface,
            weld: crate::tolerance::assembler_weld(tolerance),
        };
        let Some(mut loops) = walk_cycles(remaining, edges, Some(&space)) else {
            continue;
        };
        if loops.is_empty() {
            continue;
        }
        loops.sort_by(|first, second| {
            let area = |loop_record: &LoopRecord| {
                parameter_space_area(&FaceRecord {
                    id: faces[seed].id,
                    surface: faces[seed].surface.clone(),
                    same_sense: faces[seed].same_sense,
                    loops: vec![loop_record.clone()],
                    name: None,
                })
                .map(f64::abs)
            };
            match (area(first), area(second)) {
                (Ok(a), Ok(b)) => b.total_cmp(&a),
                _ => std::cmp::Ordering::Equal,
            }
        });
        let merged = FaceRecord {
            id: faces[seed].id,
            surface: faces[seed].surface.clone(),
            same_sense: faces[seed].same_sense,
            loops,
            name: faces[seed].name.clone(),
        };
        let expected = component
            .iter()
            .map(|index| parameter_space_area(&faces[*index]))
            .collect::<Result<Vec<_>, _>>()
            .or_refuse(KernelStage::Sew, "healing.face_merge")?
            .into_iter()
            .sum::<f64>();
        let actual =
            parameter_space_area(&merged).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        let area_tolerance = 1e-9f64.max(expected.abs() * same_carrier_area_tolerance_ratio);
        if (actual - expected).abs() > area_tolerance
            || (actual.abs() > 1e-12 && actual.is_sign_positive() != merged.same_sense)
        {
            continue;
        }
        let mut indices = component.into_iter().collect::<Vec<_>>();
        indices.sort_unstable();
        return Ok(Some((indices, merged)));
    }
    Ok(None)
}

/// A face is PINNED out of the merge when its `name` contains any substring in
/// `keep_unmerged`: it never coalesces with a neighbour (in any merge lane) and
/// is emitted unchanged. An empty `keep_unmerged` pins nothing — the historical
/// behavior.
fn face_is_pinned(face: &FaceRecord, keep_unmerged: &[String]) -> bool {
    match &face.name {
        Some(name) => keep_unmerged
            .iter()
            .any(|needle| name.contains(needle.as_str())),
        None => false,
    }
}

/// Pairwise merge driver for one shell.
///
/// `try_merge_pair` returns `None` for any two faces that share no
/// non-degenerate edge, so the only candidates are edge-adjacent pairs; they
/// are kept in a set ordered by the faces' original positions and evaluated
/// smallest first. Faces stay in their original slots (a merged face keeps
/// the lower slot, the absorbed face's slot is emptied), so that order never
/// shifts. A rejected pair is dropped for good: `try_merge_pair` is a pure
/// function of the two face records and the constant edge map, and a pair is
/// re-queued only when one of its faces changes. Hence each accepted merge is
/// the lexicographically first accepting pair over all faces — the same pair
/// the former full rescan found — and the result is the same face set, in
/// the same order, reached without re-examining every pair per merge.
struct PairMerger<'a> {
    slots: Vec<Option<FaceRecord>>,
    pinned: Vec<bool>,
    edges: &'a HashMap<u64, &'a EdgeRecord>,
    /// Non-degenerate edge id → slots of the faces using it.
    uses: HashMap<u64, Vec<usize>>,
    /// Edge-adjacent pairs `(lower slot, higher slot)` not yet rejected.
    candidates: std::collections::BTreeSet<(usize, usize)>,
    /// Ids of the faces this merger built (the surviving id of each pair).
    merged_ids: Vec<u64>,
}

impl<'a> PairMerger<'a> {
    fn new(
        faces: Vec<FaceRecord>,
        edges: &'a HashMap<u64, &'a EdgeRecord>,
        keep_unmerged: &[String],
    ) -> Self {
        let pinned = faces
            .iter()
            .map(|face| face_is_pinned(face, keep_unmerged))
            .collect::<Vec<_>>();
        let mut merger = Self {
            slots: faces.into_iter().map(Some).collect(),
            pinned,
            edges,
            uses: HashMap::default(),
            candidates: std::collections::BTreeSet::new(),
            merged_ids: Vec::new(),
        };
        for slot in 0..merger.slots.len() {
            merger.index_slot(slot);
        }
        merger
    }

    fn slot_edge_ids(&self, slot: usize) -> Vec<u64> {
        let Some(face) = &self.slots[slot] else {
            return Vec::new();
        };
        let mut ids = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.edge_id)
            .filter(|edge_id| !self.edges[edge_id].degenerate)
            .collect::<Vec<_>>();
        ids.sort_unstable();
        ids.dedup();
        ids
    }

    /// Register `slot`'s edges and queue it against every face already
    /// using one of them.
    fn index_slot(&mut self, slot: usize) {
        for edge_id in self.slot_edge_ids(slot) {
            let users = self.uses.entry(edge_id).or_default();
            for &other in users.iter() {
                if other != slot && !self.pinned[other] && !self.pinned[slot] {
                    self.candidates.insert((other.min(slot), other.max(slot)));
                }
            }
            users.push(slot);
        }
    }

    fn unindex_slot(&mut self, slot: usize) {
        for edge_id in self.slot_edge_ids(slot) {
            if let Some(users) = self.uses.get_mut(&edge_id) {
                users.retain(|user| *user != slot);
            }
        }
        self.candidates
            .retain(|(lower, higher)| *lower != slot && *higher != slot);
    }

    /// Perform up to `budget` merges; returns how many were made.
    fn run(
        &mut self,
        budget: usize,
        tolerance: f64,
        same_carrier_area_tolerance_ratio: f64,
    ) -> Result<usize, KernelRefusal> {
        let mut merged_count = 0;
        while merged_count < budget {
            let Some(&(lower, higher)) = self.candidates.iter().next() else {
                break;
            };
            self.candidates.remove(&(lower, higher));
            let (Some(older), Some(newer)) = (&self.slots[lower], &self.slots[higher]) else {
                continue;
            };
            let Some(merged) = try_merge_pair(
                older,
                newer,
                self.edges,
                tolerance,
                same_carrier_area_tolerance_ratio,
            )?
            else {
                continue;
            };
            self.unindex_slot(higher);
            self.unindex_slot(lower);
            self.slots[higher] = None;
            self.merged_ids.push(merged.id);
            self.slots[lower] = Some(merged);
            self.index_slot(lower);
            merged_count += 1;
        }
        Ok(merged_count)
    }

    fn into_faces(self) -> Vec<FaceRecord> {
        self.slots.into_iter().flatten().collect()
    }
}

/// Merge adjacent fragments that retain the exact same carrier surface.
///
/// Every shared edge is cancelled in opposite directions and the remaining
/// coedges are stitched into outer/hole cycles without refitting geometry.
fn merge_same_surface_faces_impl(
    solid: &BrepSolid,
    tolerance: f64,
    validate: bool,
    same_carrier_area_tolerance_ratio: f64,
    keep_unmerged: &[String],
    validation_scope: Option<&HashSet<u64>>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut merged_ids: Vec<u64> = Vec::new();
    // `result.edges` is not touched until the retain below, so the id map is
    // built once; every candidate pair sees the same edge records.
    let edges = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    for shell in &mut result.shells {
        loop {
            if !validate {
                if let Some((indices, merged)) = try_merge_connected_group(
                    &shell.faces,
                    &edges,
                    tolerance,
                    same_carrier_area_tolerance_ratio,
                    keep_unmerged,
                )? {
                    let first = indices[0];
                    shell.faces[first] = merged;
                    for index in indices.into_iter().skip(1).rev() {
                        shell.faces.remove(index);
                    }
                    continue;
                }
            }
            // The validating lane has no group phase, so it drains every pair
            // merge in one pass; the open lane merges one pair, then offers
            // the changed shell to the group phase again, exactly as before.
            let budget = if validate { usize::MAX } else { 1 };
            let mut merger = PairMerger::new(
                std::mem::take(&mut shell.faces),
                &edges,
                keep_unmerged,
            );
            let merged = merger.run(budget, tolerance, same_carrier_area_tolerance_ratio);
            merged_ids.extend(merger.merged_ids.iter().copied());
            shell.faces = merger.into_faces();
            if merged? == 0 || validate {
                break;
            }
        }
    }
    // The validation reads the geometry of the caller's scope and of every
    // face this merge built; a face outside both left this pass as it came.
    let validation_scope = validation_scope.map(|scope| {
        let mut scope = scope.clone();
        scope.extend(merged_ids);
        scope
    });
    finish_merge(solid, result, validate, validation_scope.as_ref())
}

/// Merge each face in `absorbed` into an edge-adjacent face outside `absorbed`
/// that rides the same carrier — the pair [`merge_same_surface_faces`] would
/// merge, at the same `tolerance` — and nothing else.
///
/// The absorbing face keeps its id, its name and its slot in the shell, and
/// its carrier is the frame the merged face is built on; a face in `absorbed`
/// that shares a carrier with no such neighbour is left as it was. Two faces
/// outside `absorbed` are never merged with each other, however cosurface: the
/// caller is naming whose material a face is, not normalizing the solid, so a
/// coplanar pair the document kept apart stays apart. The result passes the
/// validating merge's Euler and `validate()` checks or the call refuses.
pub(crate) fn absorb_faces_into_carriers(
    solid: &BrepSolid,
    tolerance: f64,
    absorbed: &HashSet<u64>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let edges = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let edge_ids = |face: &FaceRecord| -> HashSet<u64> {
        face.loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.edge_id)
            .filter(|edge_id| !edges[edge_id].degenerate)
            .collect()
    };
    for shell in &mut result.shells {
        'merge: loop {
            for inner in 0..shell.faces.len() {
                if !absorbed.contains(&shell.faces[inner].id) {
                    continue;
                }
                let inner_edges = edge_ids(&shell.faces[inner]);
                for outer in 0..shell.faces.len() {
                    if outer == inner
                        || absorbed.contains(&shell.faces[outer].id)
                        || edge_ids(&shell.faces[outer]).is_disjoint(&inner_edges)
                    {
                        continue;
                    }
                    let Some(merged) =
                        try_merge_pair(&shell.faces[outer], &shell.faces[inner], &edges, tolerance, 1e-7)?
                    else {
                        continue;
                    };
                    shell.faces[outer] = merged;
                    shell.faces.remove(inner);
                    continue 'merge;
                }
            }
            break;
        }
    }
    finish_merge(solid, result, true, None)
}

/// Merge edge-adjacent faces of `group` that ride the same carrier — the
/// pairs [`merge_same_surface_faces`] would merge, at the same `tolerance`,
/// plus two lanes only a user's selection opts into: cone patches, and two
/// half-turn patches closing a ring on a shared axial edge (see
/// `MergeLanes`) — and nothing else: a face outside `group` is never touched,
/// however cosurface with a neighbour.
///
/// Cancelling a shared edge leaves its end vertices between two pieces of one
/// rim (the two half circles a split bore's halves each bounded); those
/// pieces are joined into one edge where they continue each other exactly
/// (`merge_curve_continuation_edges_at`, at those vertices only), so the merged
/// face carries the closed rims a face built whole carries.
///
/// The surviving face of each merged pair is the one in the lower shell slot;
/// it keeps its id and name. Returns the merged solid and, for every face id
/// of `group` (sorted, deduplicated), the pair `(face, carrier)` naming the
/// face that now carries it (itself when it did not merge). The result passes
/// the validating merge's Euler and `validate()` checks, scoped to the merged
/// faces, or the call refuses.
///
/// Push Face uses it to treat the halves of a split carrier the user selected
/// together (an imported bore written as two half cylinders) as the one face
/// the push moves: a half pushed alone can never meet its unmoved sibling.
pub fn merge_faces_within(
    solid: &BrepSolid,
    tolerance: f64,
    group: &[u64],
) -> Result<(BrepSolid, Vec<(u64, u64)>), KernelRefusal> {
    let group: HashSet<u64> = group.iter().copied().collect();
    let mut carried_by: Vec<(u64, u64)> = group.iter().map(|id| (*id, *id)).collect();
    carried_by.sort_unstable();
    let mut result = solid.clone();
    let edges = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let edge_ids = |face: &FaceRecord| -> HashSet<u64> {
        face.loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.edge_id)
            .filter(|edge_id| !edges[edge_id].degenerate)
            .collect()
    };
    let mut merged_ids: HashSet<u64> = HashSet::default();
    let mut cancelled_edges: HashSet<u64> = HashSet::default();
    // End vertices of the shared edges merged faces kept as their seams.
    let mut seam_vertices: HashSet<u64> = HashSet::default();
    for shell in &mut result.shells {
        'merge: loop {
            // Only the group's own slots are read: a 1 200-face import pays
            // for its four selected faces, not for every pair of the shell.
            let slots: Vec<usize> = (0..shell.faces.len())
                .filter(|slot| group.contains(&shell.faces[*slot].id))
                .collect();
            for (index, &lower) in slots.iter().enumerate() {
                let lower_edges = edge_ids(&shell.faces[lower]);
                for &higher in &slots[index + 1..] {
                    let higher_edges = edge_ids(&shell.faces[higher]);
                    if higher_edges.is_disjoint(&lower_edges) {
                        continue;
                    }
                    let lanes = MergeLanes {
                        cones: true,
                        shared_seam_ring: true,
                        seam_vertices: Some(&seam_vertices),
                    };
                    let Some(mut merged) = try_merge_pair_lanes(
                        &shell.faces[lower],
                        &shell.faces[higher],
                        &edges,
                        tolerance,
                        1e-7,
                        lanes,
                    )?
                    else {
                        continue;
                    };
                    let survivor = shell.faces[lower].id;
                    let absorbed = shell.faces[higher].id;
                    merged.id = survivor;
                    merged.name = shell.faces[lower].name.clone();
                    let kept = edge_ids(&merged);
                    for edge_id in lower_edges.intersection(&higher_edges) {
                        if kept.contains(edge_id) {
                            let edge = edges[edge_id];
                            seam_vertices.insert(edge.start_vertex_id);
                            seam_vertices.insert(edge.end_vertex_id);
                        } else {
                            cancelled_edges.insert(*edge_id);
                        }
                    }
                    for (_, carrier) in carried_by.iter_mut() {
                        if *carrier == absorbed {
                            *carrier = survivor;
                        }
                    }
                    merged_ids.insert(survivor);
                    shell.faces[lower] = merged;
                    shell.faces.remove(higher);
                    continue 'merge;
                }
            }
            break;
        }
    }
    if merged_ids.is_empty() {
        return Ok((result, carried_by));
    }
    let joints: HashSet<u64> = cancelled_edges
        .iter()
        .flat_map(|edge_id| {
            let edge = edges[edge_id];
            [edge.start_vertex_id, edge.end_vertex_id]
        })
        .collect();
    let result = finish_merge(solid, result, true, Some(&merged_ids))?;
    if joints.is_empty() {
        return Ok((result, carried_by));
    }
    let result = crate::coalesce::merge_curve_continuation_edges_at(&result, tolerance, &joints)?;
    Ok((result, carried_by))
}

/// The tail every merge shares: drop the edges and vertices no face uses any
/// more and, on the validating lane, recount the genus and validate.
fn finish_merge(
    solid: &BrepSolid,
    mut result: BrepSolid,
    validate: bool,
    validation_scope: Option<&HashSet<u64>>,
) -> Result<BrepSolid, KernelRefusal> {
    let used_edges = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect::<HashSet<_>>();
    result.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices = result
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    result
        .vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));
    if !validate {
        return Ok(result);
    }
    if std::env::var("BREP_DEBUG_MERGE").is_ok() {
        let entry_faces: i64 = solid.shells.iter().map(|s| s.faces.len() as i64).sum();
        let entry_holes: i64 = solid
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .map(|f| f.loops.len().saturating_sub(1) as i64)
            .sum();
        let entry_edges = solid.edges.iter().filter(|e| !e.degenerate).count() as i64;
        let entry_used: std::collections::HashSet<u64> = solid
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .flat_map(|f| &f.loops)
            .flat_map(|l| &l.coedges)
            .map(|c| c.edge_id)
            .collect();
        eprintln!(
            "merge entry census: V={} E={entry_edges} (coedge-referenced {}) F={entry_faces} H={entry_holes} S={} stored_genus={}",
            solid.vertices.len(),
            entry_used.len(),
            solid.shells.len(),
            solid.genus
        );
    }
    let face_count = result
        .shells
        .iter()
        .map(|shell| shell.faces.len() as i64)
        .sum::<i64>();
    let hole_count = result.bounding_hole_count();
    let edge_count = result.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    // Count only LIVE vertices (referenced by a non-degenerate edge): the
    // merge removes edges and faces, orphaning vertices that then must not
    // participate in the Euler census — mirroring refresh_derived_genus
    // (boolean.rs). Counting stale vertices inflated V and rejected valid
    // merges as "non-integral genus" (the helmet report's final layer).
    let live_vertices: std::collections::HashSet<u64> = result
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let vertex_count = result
        .vertices
        .iter()
        .filter(|vertex| live_vertices.contains(&vertex.id))
        .count() as i64;
    let euler = vertex_count - edge_count + face_count - hole_count;
    let numerator = result.shells.len() as i64 * 2 - euler;
    // Mirror validate()'s convention (topology.rs): the reduced V−E+F−H
    // formula does not model parameter-space POLE COLLAPSES reliably, so on a
    // complex carrying degenerate edges the parity check must not reject —
    // the helmet result holds 17 legitimate degenerate (pole) edges and its
    // odd numerator here is a formula artifact, not a defect; validate()
    // below performs the real integrity checks either way. With no
    // degenerate edges the strict behavior is unchanged.
    let has_degenerate = result.edges.iter().any(|edge| edge.degenerate);
    if numerator < 0 || numerator % 2 != 0 {
        if std::env::var("BREP_DEBUG_MERGE").is_ok() {
            eprintln!(
                "merge euler reject: V={vertex_count} E={edge_count} F={face_count} H={hole_count} S={} euler={euler} numerator={numerator} degenerate={has_degenerate}",
                result.shells.len()
            );
        }
        if !has_degenerate
            || std::env::var("BREP_MERGE_DEGENERATE_EULER_STRICT").as_deref() == Ok("1")
        {
            return Err(KernelRefusal::new(
                RefusalClass::NonIntegralGenus {
                    shells: result.shells.len() as u32,
                    euler: euler as i64,
                },
                KernelStage::Validate,
                "merge_same_surface_faces produced non-integral genus",
            ));
        }
    } else {
        result.genus = numerator / 2;
    }
    let issues = match validation_scope {
        Some(scope) => result.validate_scoped(scope),
        None => result.validate(),
    };
    if !issues.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::DegenerateArrangement {
                open_edges: 0,
                issues: issues.len() as u32,
            },
            KernelStage::Validate,
            format!("merge_same_surface_faces produced invalid topology: {issues:?}"),
        ));
    }
    Ok(result)
}

pub fn merge_same_surface_faces(
    solid: &BrepSolid,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    merge_same_surface_faces_impl(solid, tolerance, true, 1e-7, &[], None)
}

/// [`merge_same_surface_faces_excluding`] whose closing validation reads
/// the geometry of the faces in `scope` and of the faces the merge built;
/// every pair still merges exactly as the plain call merges it (an operand's
/// own coplanar neighbours included: the 2026-10-10 census found a mesh
/// import whose five such pairs the boolean fuses). A boolean passes the
/// faces its operation touched and their neighbours
/// (`BooleanOptions::untouched_passthrough`); the rest were validated by the
/// assembler and left this pass as they came.
pub fn merge_same_surface_faces_scoped(
    solid: &BrepSolid,
    tolerance: f64,
    keep_unmerged: &[String],
    scope: &HashSet<u64>,
) -> Result<BrepSolid, KernelRefusal> {
    merge_same_surface_faces_impl(solid, tolerance, true, 1e-7, keep_unmerged, Some(scope))
}

/// Like [`merge_same_surface_faces`], but any face whose `name` CONTAINS one of
/// `keep_unmerged` is pinned: it never merges with a coplanar/cosurface
/// neighbour and is emitted unchanged, while every other face merges exactly as
/// [`merge_same_surface_faces`] would. An empty `keep_unmerged` is identical to
/// the plain call.
pub fn merge_same_surface_faces_excluding(
    solid: &BrepSolid,
    tolerance: f64,
    keep_unmerged: &[String],
) -> Result<BrepSolid, KernelRefusal> {
    merge_same_surface_faces_impl(solid, tolerance, true, 1e-7, keep_unmerged, None)
}

pub(crate) fn merge_same_surface_faces_open(
    solid: &BrepSolid,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    merge_same_surface_faces_impl(solid, tolerance, false, 5e-3, &[], None)
}


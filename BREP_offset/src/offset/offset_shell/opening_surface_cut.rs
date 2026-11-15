//! OUTWARD sharp shells through openings on a sphere or a cylinder.
//!
//! An outward sharp shell keeps every retained face's offset, extended until it
//! meets the opening's OWN surface, and nothing past it (the convention of the
//! box opened at its top and grown outward: the side walls stop on the top's
//! plane). When the opening lies on a sphere or a cylinder, that surface bounds
//! a solid `K` of its own (the ball; the cylinder clipped past the grown body),
//! and the shell is
//!
//! ```text
//!     (grown ∩ K) − source      for an opening bulging out of the body (a dome)
//!     (grown − K) − source      for one curving into it (a spherical dimple)
//! ```
//!
//! with `grown` the whole source offset outward with sharp joins. The direct
//! pipeline builds an outward wall only on a planar or ruled opening, and its
//! ruled wall needs the trim to be the whole face; this lane answers the
//! openings it refuses.
//!
//! `K` is global and the pipeline's wall is local: they agree only where no
//! retained face comes within the wall of the opening's surface away from the
//! opening itself. That is asked before the shell is returned: every sample of
//! a retained face closer to the opening's surface than the wall must lie
//! within a few walls of the opening's rim. The result must also be valid and
//! weigh exactly `V(grown ∩ K) − V(source)`.
use super::pipeline::{offset_body, Body};
use super::solid::provenance_by_surface;
use super::{OffsetFaceRole, OffsetShellFaceImageRecord, OffsetShellResultRecord};
use crate::analytic_surface::recognize;
use crate::analytic_surface::AnalyticSurface;
use crate::classification::PolygonClass;
use crate::topology::{BrepSolid, FaceRecord};
use crate::{
    boolean_operation, make_cylinder_brep, make_sphere_brep, parameter_point_in_face, BooleanOperation, BooleanOptions,
    KernelRefusal, KernelStage, OrRefuse, Vec2, Vec3,
};

/// `BREP_OFFSET_OPENING_SURFACE_CUT` (default on; `=0` restores the refusal):
/// an outward shell the direct pipeline refuses at a curved opening on a
/// sphere or a cylinder is built as the grown body cut by the opening's own
/// surface.
pub(super) fn opening_surface_cut_on() -> bool {
    crate::switch!("BREP_OFFSET_OPENING_SURFACE_CUT", true)
}

/// The direct refusals this lane answers: the curved-opening and ruled-wall
/// refusals of an outward shell.
pub(super) fn answers(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what }
        if what == "offset_shell_curved_opening" || what == "offset_shell_ruled_wall")
}

/// The closed surface an opening face lies on.
#[derive(Clone, Copy, Debug)]
enum OpeningSurface {
    Sphere { centre: Vec3, radius: f64 },
    Cylinder { origin: Vec3, axis: Vec3, radius: f64 },
}

impl OpeningSurface {
    fn of(face: &FaceRecord) -> Option<Self> {
        let analytic = recognize(&face.surface)?;
        if let Some((centre, radius)) = analytic.sphere_geometry() {
            return Some(OpeningSurface::Sphere { centre, radius });
        }
        match analytic {
            AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }
                if (rho0 - rho1).abs() <= 1e-12 * rho0.abs().max(1.0) && rho0 > 0.0 =>
            {
                Some(OpeningSurface::Cylinder { origin: frame.origin, axis: frame.axis.normalized_or(frame.axis), radius: rho0 })
            }
            _ => None,
        }
    }

    /// The outward radial direction of the surface at `point`.
    fn radial(&self, point: Vec3) -> Vec3 {
        match *self {
            OpeningSurface::Sphere { centre, .. } => point.sub(centre).normalized_or(Vec3::new(0.0, 0.0, 0.0)),
            OpeningSurface::Cylinder { origin, axis, .. } => {
                let offset = point.sub(origin);
                offset.sub(axis.scale(offset.dot(axis))).normalized_or(Vec3::new(0.0, 0.0, 0.0))
            }
        }
    }

    /// How far `point` lies inside the surface (negative outside).
    fn depth(&self, point: Vec3) -> f64 {
        match *self {
            OpeningSurface::Sphere { centre, radius } => radius - point.sub(centre).length(),
            OpeningSurface::Cylinder { origin, axis, radius } => {
                let offset = point.sub(origin);
                radius - offset.sub(axis.scale(offset.dot(axis))).length()
            }
        }
    }

    /// Whether `other` is the same sphere or the same cylinder, within `band`.
    fn same_as(&self, other: &OpeningSurface, band: f64) -> bool {
        match (*self, *other) {
            (OpeningSurface::Sphere { centre, radius }, OpeningSurface::Sphere { centre: c, radius: r }) => {
                centre.sub(c).length() <= band && (radius - r).abs() <= band
            }
            (OpeningSurface::Cylinder { origin, axis, radius }, OpeningSurface::Cylinder { origin: o, axis: a, radius: r }) => {
                let offset = o.sub(origin);
                axis.cross(a).length() <= 1e-9
                    && offset.sub(axis.scale(offset.dot(axis))).length() <= band
                    && (radius - r).abs() <= band
            }
            _ => false,
        }
    }

    /// The solid the surface bounds, reaching past every point of `extent`.
    fn solid(&self, extent: &[Vec3]) -> Result<BrepSolid, KernelRefusal> {
        match *self {
            OpeningSurface::Sphere { centre, radius } => {
                make_sphere_brep(centre, radius, Vec3::new(0.0, 0.0, 1.0)).or_refuse(KernelStage::Sew, "make_sphere_brep")
            }
            OpeningSurface::Cylinder { origin, axis, radius } => {
                let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
                for point in extent {
                    let along = point.sub(origin).dot(axis);
                    low = low.min(along);
                    high = high.max(along);
                }
                if !(low.is_finite() && high.is_finite()) {
                    return Err(decline("offset_shell: the grown body has no extent to bound the opening's cylinder".into()));
                }
                let margin = 0.25 * (high - low).max(radius);
                let base = origin.add(axis.scale(low - margin));
                make_cylinder_brep(base, axis, radius, high - low + 2.0 * margin)
                    .or_refuse(KernelStage::Sew, "make_cylinder_brep")
            }
        }
    }
}

fn decline(message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_opening_surface_cut", message)
}

fn volume(solid: &BrepSolid) -> Result<f64, KernelRefusal> {
    crate::solid_mass_properties(solid)
        .map(|properties| properties.volume)
        .or_refuse(KernelStage::Validate, "solid_mass_properties")
}

/// Points of `face`: an `n × n` grid of its domain inside its trim, its seed
/// (a point inside it, so a face the grid misses is still read), and 64
/// points along each of its edges.
fn face_samples(source: &BrepSolid, face: &FaceRecord, n: usize) -> Result<Vec<Vec3>, KernelRefusal> {
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let mut points = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let u = u0 + (u1 - u0) * (i as f64 + 0.5) / n as f64;
            let v = v0 + (v1 - v0) * (j as f64 + 0.5) / n as f64;
            if parameter_point_in_face(face, Vec2 { x: u, y: v }, 1e-9).or_refuse(KernelStage::Refine, "parameter_point_in_face")?
                == PolygonClass::Inside
            {
                points.push(face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?);
            }
        }
    }
    for seed in super::source_seeds(source, face.id)? {
        points.push(face.surface.evaluate(seed.x, seed.y).or_refuse(KernelStage::Refine, "evaluate")?);
    }
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = source.edges.iter().find(|edge| edge.id == coedge.edge_id) else { continue };
        if edge.degenerate {
            continue;
        }
        for k in 0..=64 {
            points.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * k as f64 / 64.0).or_refuse(KernelStage::Refine, "evaluate")?);
        }
    }
    Ok(points)
}

/// The outward shell of `source` (outward: `distance < 0`) open at
/// `opening_face_ids`, each on a sphere or a cylinder, as the sharp grown body
/// cut by the openings' surfaces less the source.
pub(super) fn build(source: &BrepSolid, opening_face_ids: &[u64], distance: f64) -> Result<OffsetShellResultRecord, KernelRefusal> {
    if !(distance < 0.0) || opening_face_ids.is_empty() {
        return Err(decline("offset_shell: the opening-surface cut builds outward shells only".into()));
    }
    let wall = distance.abs();
    let faces: Vec<&FaceRecord> = source.shells.iter().flat_map(|shell| &shell.faces).collect();
    let band = 1e-7f64.max(crate::solid_scale(source) * 1e-9);
    // Each opening's closed surface and the side of it the source lies on.
    let mut cuts = Vec::new();
    for id in opening_face_ids {
        let face = faces
            .iter()
            .find(|face| face.id == *id)
            .ok_or_else(|| decline(format!("offset_shell: opening face {id} is not a face of the source")))?;
        let surface = OpeningSurface::of(face).ok_or_else(|| {
            decline(format!("offset_shell: opening face {id} lies on neither a sphere nor a cylinder"))
        })?;
        let seeds = super::source_seeds(source, face.id)?;
        let seed = seeds[0];
        let point = face.surface.evaluate(seed.x, seed.y).or_refuse(KernelStage::Refine, "evaluate")?;
        let outward = super::face_normal(face, seed.x, seed.y)?;
        let convex = outward.dot(surface.radial(point)) > 0.0;
        cuts.push((*id, surface, convex));
    }
    // K is global, the wall local: no retained face may come within the wall
    // of an opening's surface (on the source's side of it) away from the
    // opening's rim.
    let mut rim = Vec::new();
    for face in faces.iter().filter(|face| opening_face_ids.contains(&face.id)) {
        for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
            let Some(edge) = source.edges.iter().find(|edge| edge.id == coedge.edge_id) else { continue };
            for k in 0..=64 {
                let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 64.0;
                rim.push(edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?);
            }
        }
    }
    // A retained face ON an opening's surface, or meeting an opening
    // tangentially, has its offset outside K all along it: the cut would strip
    // that face of its wall (a dome split into two faces, one opened, came back
    // as the slab under the base, valid and of the cut's own volume).
    for face in faces.iter().filter(|face| !opening_face_ids.contains(&face.id)) {
        if let Some(own) = OpeningSurface::of(face) {
            if let Some((id, _, _)) = cuts.iter().find(|(_, surface, _)| surface.same_as(&own, band)) {
                return Err(decline(format!(
                    "offset_shell: retained source face {} lies on opening face {id}'s own surface, whose cut would strip it of its wall",
                    face.id
                )));
            }
        }
        for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
            let Some((mate, mate_use)) = super::junction_mate(&faces, face, coedge.edge_id) else { continue };
            if opening_face_ids.contains(&mate.id) && super::uses_are_tangent(face, coedge, mate, mate_use)? {
                return Err(decline(format!(
                    "offset_shell: retained source face {} meets opening face {} tangentially, so the cut by the opening's surface would strip its wall",
                    face.id, mate.id
                )));
            }
        }
    }
    for face in faces.iter().filter(|face| !opening_face_ids.contains(&face.id)) {
        for point in face_samples(source, face, 24)? {
            for (id, surface, convex) in &cuts {
                let depth = if *convex { surface.depth(point) } else { -surface.depth(point) };
                if depth < -band {
                    return Err(decline(format!(
                        "offset_shell: source face {} lies past the surface of opening face {id}",
                        face.id
                    )));
                }
                if depth >= wall {
                    continue;
                }
                let near = rim.iter().map(|q| q.sub(point).length()).fold(f64::INFINITY, f64::min);
                // On the surface itself away from the rim: the same stripping.
                if depth < 0.01 * wall && near > 0.05 * wall + band {
                    return Err(decline(format!(
                        "offset_shell: source face {} lies on the surface of opening face {id} {near:.6} from the opening's rim",
                        face.id
                    )));
                }
                if near > 4.0 * wall {
                    return Err(decline(format!(
                        "offset_shell: source face {} comes within the wall of opening face {id}'s surface \
                         {near:.6} from the opening's rim, where the cut by that surface is not the shell's",
                        face.id
                    )));
                }
            }
        }
    }
    let body = offset_body(source, &[], distance, Body::Whole, "offset_shell_opening_cut")?;
    // Every control point of the grown body's faces: a NURBS surface lies in
    // its control net's hull, so the cylinder cutter reaches past all of it.
    let extent: Vec<Vec3> = body
        .solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| face.surface.control_points.iter().flatten())
        .filter_map(|pole| pole.point().ok())
        .collect();
    let mut cut = body.solid.clone();
    let mut opening_solids = Vec::new();
    for (id, surface, convex) in &cuts {
        let k = surface.solid(&extent)?;
        let operation = if *convex { BooleanOperation::Intersect } else { BooleanOperation::Subtract };
        cut = boolean_operation(&cut, &k, operation, &BooleanOptions::default()).map_err(|error| {
            error.with_message(|message| {
                format!("offset_shell: the grown body could not be cut by opening face {id}'s surface: {message}")
            })
        })?;
        opening_solids.push((*id, k));
    }
    let solid = boolean_operation(&cut, source, BooleanOperation::Subtract, &BooleanOptions::default()).map_err(|error| {
        error.with_message(|message| format!("offset_shell: the opening-surface cut's difference could not be built: {message}"))
    })?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(decline(format!("offset_shell: the opening-surface cut is not a valid solid ({:?})", issues.first())));
    }
    let (expected, measured) = (volume(&cut)? - volume(source)?, volume(&solid)?);
    if !(expected > 0.0) || (measured - expected).abs() > 1e-9 * expected.max(volume(source)?) {
        return Err(decline(format!(
            "offset_shell: the opening-surface cut weighs {measured} against {expected}, so the source is not inside the cut body"
        )));
    }
    let mut operands = body
        .solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .zip(&body.face_images)
        .map(|(face, image)| (face, image.clone()))
        .collect::<Vec<_>>();
    for (id, k) in &opening_solids {
        for face in k.shells.iter().flat_map(|shell| &shell.faces) {
            operands.push((face, OffsetShellFaceImageRecord { role: OffsetFaceRole::Wall, source_face_id: *id }));
        }
    }
    operands.extend(source.shells.iter().flat_map(|shell| &shell.faces).map(|face| {
        (face, OffsetShellFaceImageRecord { role: OffsetFaceRole::Source, source_face_id: face.id })
    }));
    let face_images = provenance_by_surface(&solid, &operands, None)?;
    Ok(OffsetShellResultRecord { solid, face_images })
}

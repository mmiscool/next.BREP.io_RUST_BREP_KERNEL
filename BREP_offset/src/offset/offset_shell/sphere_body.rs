//! Lane O row 36 (`BREP_OFFSET_SPHERE_BODY`, default on since batch 17; `0`/`off` restores the pipeline):
//! the offset BODY of a ball, built directly.
//!
//! A sphere's normal is radial, so the offset of every point of a sphere of
//! radius `R` about `c` is that point scaled about `c` by `k = (R ± d)/R`. A
//! body bounded by one shell whose every face lies on that one sphere is
//! therefore offset, faces, edges and vertices together, by the similarity
//! `x ↦ c + k·(x − c)`. A NURBS map is affine in its control points, so
//! scaling the controls of every carrier and edge curve, and every vertex,
//! scales each image exactly; knots, weights, parameters and pcurves are
//! untouched, so every coedge reads its edge on its carrier as before.
//!
//! The pipeline answers the same body through carriers, an imprint and an
//! assembly. Anything else (a face on another surface or another sphere, two
//! shells, an offset through the centre) declines to the pipeline, and so does
//! a scaled body that does not validate: the lane never answers with a
//! refusal the pipeline would not give.
use super::*;

/// The switch, default on (batch 17): `0`/`off` sends the whole-body offset of a ball
/// to the pipeline instead of the lane.
fn sphere_body_on() -> bool {
    switch!("BREP_OFFSET_SPHERE_BODY", true)
}


/// The lane's answer, or `None` when it declines (or the switch is off).
/// `distance` carries the pipeline's sign: positive moves opposite each face's
/// outward normal.
pub(super) fn sphere_body(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !sphere_body_on() {
        return None;
    }
    match build(source, distance) {
        Ok(result) => {
            os_debug!("sphere body lane built (thread {})", std::thread::current().name().unwrap_or("main"));
            Some(result)
        }
        Err(reason) => {
            os_debug!("sphere body lane declined: {reason}");
            None
        }
    }
}

fn build(source: &BrepSolid, distance: f64) -> Result<OffsetShellResultRecord, String> {
    if !(distance.is_finite() && distance != 0.0) {
        return Err("not a finite non-zero distance".into());
    }
    if source.kind != crate::BodyKind::Solid || source.shells.len() != 1 || source.shells[0].faces.is_empty() {
        return Err("not one shell of a solid".into());
    }
    let faces = &source.shells[0].faces;
    let Some((centre, radius)) = faces[0].surface.analytic().and_then(|shape| shape.sphere_geometry()) else {
        return Err(format!("face {} is not a recognized sphere", faces[0].id));
    };
    let bar = 1e-12 * (centre.length() + radius).max(1.0);
    for face in &faces[1..] {
        match face.surface.analytic().and_then(|shape| shape.sphere_geometry()) {
            Some((other, other_radius)) if other.sub(centre).length() <= bar && (other_radius - radius).abs() <= bar => {}
            _ => return Err(format!("face {} is not on the first face's sphere", face.id)),
        }
    }
    // The move, read off the shared evaluator on every face (each may carry
    // its own sense): radial, exactly |d| long, the same new radius.
    let mut factor = None::<f64>;
    for face in faces {
        let [u0, u1] = face.surface.domain_u()?;
        let [v0, v1] = face.surface.domain_v()?;
        let sample = crate::OffsetEvaluator::new("offset_shell", &face.surface, crate::OffsetNormal::FaceStable { same_sense: face.same_sense })
            .at((u0 + u1) * 0.5, (v0 + v1) * 0.5, -distance)
            .map_err(|error| format!("face {}: {error}", face.id))?;
        let (from, to) = (sample.source.sub(centre), sample.point.sub(centre));
        let new_radius = to.length();
        if (from.length() - radius).abs() > 1e2 * bar
            || to.dot(from) <= 0.0
            || ((new_radius - radius).abs() - distance.abs()).abs() > 1e2 * bar
            || !(new_radius > 1e-6 * radius)
        {
            return Err(format!("face {}: the offset is not the concentric sphere (radius {new_radius})", face.id));
        }
        let k = new_radius / radius;
        if factor.is_some_and(|previous| (previous - k).abs() > 1e-12) {
            return Err("the faces move to different spheres".into());
        }
        factor = Some(k);
    }
    let k = factor.ok_or("no face")?;
    let scale = |point: Vec3| centre.add(point.sub(centre).scale(k));
    let scale_controls = |controls: &mut [crate::Vec4]| -> Result<(), String> {
        for control in controls {
            *control = crate::Vec4::from_point(scale(control.point()?), control.w);
        }
        Ok(())
    };
    let mut solid = source.clone();
    for vertex in &mut solid.vertices {
        vertex.point = scale(vertex.point);
    }
    for edge in &mut solid.edges {
        let mut controls = edge.curve.control_points.clone();
        scale_controls(&mut controls)?;
        edge.curve = crate::NurbsCurve::new(edge.curve.degree, edge.curve.knots.clone(), controls)?;
    }
    let mut face_images = Vec::new();
    for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let mut rows = face.surface.control_points.clone();
        for row in &mut rows {
            scale_controls(row)?;
        }
        let surface = &face.surface;
        face.surface = crate::NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), surface.knots_v.clone(), rows)?;
        face_images.push(OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: face.id });
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the scaled body does not validate: {issues:?}"));
    }
    Ok(OffsetShellResultRecord { solid, face_images })
}

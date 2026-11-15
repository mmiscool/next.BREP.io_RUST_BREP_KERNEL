//! The offset BODY of a whole torus grown past its hole
//! (`BREP_OFFSET_TORUS_APPLE`, default on; `=0` restores the refusal).
//!
//! The points within `d` of a torus of major radius `R` and tube radius `r`
//! are, in every meridian half-plane, the disc of radius `s = r + d` about
//! the tube centre at distance `R` from the axis, together with the same disc
//! about the opposite tube centre (half a turn away) where it reaches across.
//! When `s > R` that far disc's part on this side, `(ρ + R)² + z² ≤ s²` with
//! `ρ ≥ 0`, lies inside the near disc (`(ρ − R)² ≤ (ρ + R)²`), so the body is
//! the near disc cut by the axis, revolved: an APPLE, one face, its two
//! dimples meeting the axis at `z = ±√(s² − R²)` (brep-bench
//! offset/torus-grown-past-its-hole). The pipeline refused it: the tube's
//! offset folds in a band across the axis.
//!
//! The answer is built by revolving the profile (the outer arc of the disc
//! from the axis round to the axis, closed by the axis segment), and checked
//! against Pappus' volume before it is returned. Anything else declines to
//! the pipeline unchanged.
use super::*;

fn enabled() -> bool {
    switch!("BREP_OFFSET_TORUS_APPLE", true)
}


/// The apple, or `None`. `distance` carries the pipeline's sign (negative
/// grows).
pub(super) fn torus_apple(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !enabled() || distance >= 0.0 {
        return None;
    }
    match build(source, -distance) {
        Ok(result) => result,
        Err(reason) => {
            os_debug!("torus apple lane declined: {reason}");
            None
        }
    }
}

fn build(source: &BrepSolid, grow: f64) -> Result<Option<OffsetShellResultRecord>, String> {
    let faces = source.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    if source.shells.len() != 1 || faces.len() != 1 {
        return Ok(None);
    }
    let face = faces[0];
    let Some(crate::AnalyticSurface::Torus { frame, major_radius, minor_radius }) = face.surface.analytic() else {
        return Ok(None);
    };
    // The whole torus: no edge with length bounds the face.
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let Some(edge) = source.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
            return Ok(None);
        };
        // A seam is used twice by this one face; anything else is a boundary.
        let uses = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .filter(|other| other.edge_id == edge.id)
            .count();
        if !edge.degenerate && uses != 2 {
            return Ok(None);
        }
    }
    let (big, s) = (*major_radius, *minor_radius + grow);
    if !(s > big * (1.0 + 1e-9)) || !(big > 0.0) {
        return Ok(None);
    }
    let c = (s * s - big * big).sqrt();
    let phi = (big / s).acos();
    let centre = frame.origin.add(frame.x_axis.scale(big));
    let arc = crate::make_arc(centre, frame.x_axis, frame.axis, s, -(std::f64::consts::PI - phi), std::f64::consts::PI - phi)?;
    let top = frame.origin.add(frame.axis.scale(c));
    let bottom = frame.origin.sub(frame.axis.scale(c));
    let axis_segment = crate::make_line(top, bottom)?;
    let solid = crate::revolve_profile_brep(&[arc, axis_segment], frame.origin, frame.axis, std::f64::consts::TAU)?;
    let pi = std::f64::consts::PI;
    let expected = 2.0 * pi * (pi * big * s * s + 2.0 * c.powi(3) / 3.0 - big * (s * s * phi - big * c));
    let volume = crate::solid_mass_properties(&solid)?.volume;
    if !((volume - expected).abs() <= 1e-9 * expected) || !solid.validate().is_empty() {
        return Err(format!("the revolved apple reads volume {volume} against {expected}"));
    }
    os_debug!("torus apple lane built: s = {s}, dimples at ±{c}");
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: face.id })
        .collect();
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}

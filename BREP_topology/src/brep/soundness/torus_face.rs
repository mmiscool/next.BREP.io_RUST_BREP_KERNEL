//! Single exact torus face certificate (`BREP_SOUNDNESS_TORUS_FACE`, default
//! on since the 2026-11-19 promotion; `=0` scans): a body (or the subset an acceptance hands the scan) of ONE face whose
//! carrier is, to `DELTA`, the torus `make_torus_surface` rebuilds from the
//! carrier's own recognised centre, axis and radii, with the tube clear of the
//! axis, and every trim control inside the carrier's domain, holds no crossing
//! (one face has no other face to cross) and no fold (the torus chart is
//! injective on its one-turn domain, up to the seams the full scan identifies
//! as well), so the scan returns its empty report without meshing. A closed
//! hollow's offset acceptance tessellated such a face to 7436 triangles, a
//! quarter of a second cold. Anything else declines to the full scan.
use super::*;

/// Carrier recognition slack, relative to the torus's size (as the analytic
/// body certificate holds its planes and cylinders).
const DELTA: f64 = 1e-12;

pub(super) fn enabled() -> bool {
    crate::switch!("BREP_SOUNDNESS_TORUS_FACE", true)
}

pub(super) fn certifies(solid: &BrepSolid, _options: SelfIntersectionOptions) -> bool {
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let [face] = faces[..] else { return false };
    let Some((u, v)) = exact_torus_domain(&face.surface) else { return false };
    // Every trim control inside the domain: the face is the image of a
    // region of the injective chart.
    let span = (u[1] - u[0]).abs().max(v[1] - v[0]).max(1.0);
    let band = 1e-12 * span;
    face.loops.iter().flat_map(|loop_record| &loop_record.coedges).all(|coedge| {
        coedge.pcurve.control_points.iter().all(|control| {
            let w = control.w;
            if !(w > 0.0 && w.is_finite()) {
                return false;
            }
            let (x, y) = (control.x / w, control.y / w);
            x >= u[0] - band && x <= u[1] + band && y >= v[0] - band && y <= v[1] + band
        })
    })
}

/// The carrier's (u, v) domain when it is, to `DELTA`, the canonical torus of
/// its own recognised centre, axis and radii with the tube clear of the axis.
fn exact_torus_domain(s: &NurbsSurface) -> Option<([f64; 2], [f64; 2])> {
    let Some(crate::AnalyticSurface::Torus { frame, major_radius, minor_radius }) = s.analytic() else {
        return None;
    };
    let (big, small) = (*major_radius, *minor_radius);
    if !(small > 0.0 && big.is_finite() && small < big * (1.0 - 1e-6)) {
        return None;
    }
    let rebuilt = crate::make_torus_surface(frame.origin, frame.axis, big, small).ok()?;
    let same_bits = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    if !same_bits(&s.knots_u, &rebuilt.knots_u)
        || !same_bits(&s.knots_v, &rebuilt.knots_v)
        || s.degree_u != rebuilt.degree_u
        || s.degree_v != rebuilt.degree_v
        || s.control_points.len() != rebuilt.control_points.len()
        || s.control_points.iter().zip(&rebuilt.control_points).any(|(a, b)| a.len() != b.len())
    {
        return None;
    }
    // Relative to the torus's OWN size, never its distance from the origin
    // (cross review: at 5e9 from the origin an origin-scaled slack let a
    // control cross the axis), and far below the tube's clearance from the
    // axis, so no allowed deviation can carry the tube across it.
    let size = big + small;
    if !(DELTA * size <= 1e-3 * (big - small)) {
        return None;
    }
    let close = s.control_points.iter().flatten().zip(rebuilt.control_points.iter().flatten()).all(|(a, b)| {
        (a.x - b.x).abs() <= DELTA * size
            && (a.y - b.y).abs() <= DELTA * size
            && (a.z - b.z).abs() <= DELTA * size
            && (a.w - b.w).abs() <= DELTA
            && a.w > 0.0
    });
    if !close {
        return None;
    }
    Some(([*s.knots_u.first()?, *s.knots_u.last()?], [*s.knots_v.first()?, *s.knots_v.last()?]))
}

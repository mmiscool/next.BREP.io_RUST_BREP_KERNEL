//! Conservative separating support certificates; no adjacency inference.

use crate::{BrepSolid, FaceRecord, NurbsCurve, NurbsSurface, Vec3, Vec4};
use super::{CertifiedFacePair, CertificateScope};

type Bounds = [[f64; 2]; 3];

pub(super) fn enabled() -> bool {
    matches!(std::env::var("BREP_SOUNDNESS_PAIR_CERTIFICATES").as_deref(), Ok("1" | "on"))
}

fn include(bounds: &mut Bounds, p: Vec3) -> Option<()> {
    for (interval, coordinate) in bounds.iter_mut().zip([p.x, p.y, p.z]) {
        if !coordinate.is_finite() { return None; }
        interval[0] = interval[0].min(coordinate);
        interval[1] = interval[1].max(coordinate);
    }
    Some(())
}

fn controls(bounds: &mut Bounds, points: &[Vec4]) -> Option<()> {
    for p in points {
        // Positive rational bases form a convex combination of these points.
        // Decline negative, zero, extreme or non-finite weights.
        if !(1e-8..=1e8).contains(&p.w) { return None; }
        include(bounds, p.point().ok()?)?;
    }
    Some(())
}

fn bounds(solid: &BrepSolid, face: &FaceRecord) -> Option<Bounds> {
    let s = &face.surface;
    // Public raw arrays may have changed since ensure_valid cached success.
    let fresh = NurbsSurface::new(s.degree_u, s.degree_v, s.knots_u.clone(), s.knots_v.clone(), s.control_points.clone()).ok()?;
    let mut b = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
    for row in &s.control_points { controls(&mut b, row)?; }
    // Physical edge samples and endpoint pins are also tessellator inputs.
    // The WHOLE positive edge hull is conservative for its trimmed interval.
    for co in face.loops.iter().flat_map(|l| &l.coedges) {
        let e = solid.edges.iter().find(|e| e.id == co.edge_id)?;
        NurbsCurve::new(e.curve.degree, e.curve.knots.clone(), e.curve.control_points.clone()).ok()?;
        controls(&mut b, &e.curve.control_points)?;
        for id in [e.start_vertex_id, e.end_vertex_id] {
            include(&mut b, solid.vertices.iter().find(|v| v.id == id)?.point)?;
        }
    }
    // Contact may reach surface confirmation; a stale projector descriptor
    // must not turn a one-sided material bound into a different full-scan
    // verdict. Compare the COMPLETE fresh descriptor and seam state.
    if format!("{:?}", s.analytic()) != format!("{:?}", fresh.analytic())
        || s.closed_directions().ok()? != fresh.closed_directions().ok()? {
        return None;
    }
    b.iter().flatten().all(|v| v.is_finite()).then_some(b)
}

fn separation(a: Bounds, b: Bounds) -> Option<(&'static str, usize)> {
    for axis in 0..3 {
        let [lo, hi] = a[axis];
        let [other_lo, other_hi] = b[axis];
        // Strict separation has no triangle-box hit at all. Leave a derived
        // floating-point guard, not an enlarged geometry tolerance.
        let scale = lo.abs().max(hi.abs()).max(other_lo.abs()).max(other_hi.abs()).max(1.);
        let guard = 4096. * f64::EPSILON * scale;
        if other_lo - hi > guard || lo - other_hi > guard {
            return Some(("positive_control_hulls", axis));
        }
    }
    for axis in 0..3 {
        let [lo, hi] = a[axis];
        let [other_lo, other_hi] = b[axis];
        // EXACT constant coordinate: no tolerant recognition, fitted plane,
        // normal approximation, adjacency or cached descriptor grants this.
        // Contact is allowed; transversality requires material on BOTH sides.
        if (lo == hi && (other_lo >= hi || other_hi <= lo))
            || (other_lo == other_hi && (lo >= other_hi || hi <= other_lo)) {
            return Some(("axis_plane_support", axis));
        }
    }
    None
}

pub(super) struct Plan {
    mask: Vec<Vec<bool>>,
    pub proofs: Vec<CertifiedFacePair>,
    pub scope: CertificateScope,
}

impl Plan {
    pub(super) fn new(solid: &BrepSolid, faces: &[&FaceRecord], corners: &[[Vec3; 3]], face_ids: &[u32]) -> Self {
        let mut supports: Vec<_> = faces.iter().map(|face| bounds(solid, face)).collect();
        // This row intentionally retains the original whole-body mesh/weld.
        // Include ALL actual mesh nodes in the support. An extended evaluator,
        // stale recognition or physical boundary departure can only weaken a
        // certificate, never cause an unbounded node to be silently omitted.
        for (points, &face) in corners.iter().zip(face_ids) {
            if let Some(b) = &mut supports[face as usize] {
                for &point in points { if include(b, point).is_none() { supports[face as usize] = None; break; } }
            }
        }
        let mut mask = vec![vec![false; faces.len()]; faces.len()];
        let mut proofs = Vec::new();
        for first in 0..faces.len() {
            for second in first + 1..faces.len() {
                let (Some(a), Some(b)) = (supports[first], supports[second]) else { continue; };
                let Some((rule, axis)) = separation(a, b) else { continue; };
                mask[first][second] = true;
                mask[second][first] = true;
                proofs.push(CertifiedFacePair { faces: [faces[first].id, faces[second].id], rule, axis, bounds: [a[axis], b[axis]] });
            }
        }
        let unresolved: Vec<_> = (0..faces.len()).map(|a| (0..faces.len()).any(|b| a != b && !mask[a][b])).collect();
        let scope = CertificateScope {
            examined_face_pairs: faces.len().saturating_mul(faces.len().saturating_sub(1)) / 2 - proofs.len(),
            certified_face_pairs: proofs.len(),
            omitted_faces: 0,
            skipped_candidate_pairs: 0,
            retained_fold_faces: faces.len(),
            faces_in_uncertified_pairs: unresolved.iter().filter(|&&v| v).count(),
            triangles_in_uncertified_pairs: face_ids.iter().filter(|&&id| unresolved[id as usize]).count(),
            retained_triangles: face_ids.len(),
        };
        Self { mask, proofs, scope }
    }

    pub(super) fn skips(&self, a: u32, b: u32) -> bool { self.mask[a as usize][b as usize] }
}



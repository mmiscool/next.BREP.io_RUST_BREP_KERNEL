//! A stripe between two miters on one shared face whose corner stops run
//! past each other has no rail: the radius exceeds the face between its two
//! corners, and no ball of that radius fits between the two neighbouring
//! stripes' side faces there.  Where the neighbours' rails on the shared
//! face cross — their rail/rail point `M`, the last ball that fits — both
//! neighbours end, and the stripe becomes the HOOD over the channel's end:
//! its two corners merge into one patch bounded by the stripe's one
//! surviving rail (on its side face) and one curve on each neighbour from
//! that rail's end on the sharp edge up to `M` (the neighbour's rib at the
//! merged corner).  The hood's surface is the stripe's own section with its
//! circle STRETCHED into the quarter ellipse tangent to the side face along
//! the rail and to the shared face along the line through `M`: semi-axes
//! `a` (the distance of `M` from the side face) and `r` (the radius).  At
//! the crossing threshold `a = r` and the hood is the stripe's cylinder, so
//! the conversion continues the full-width channel exactly; the ribs on the
//! neighbours are the level set of the ellipse on each neighbour's blend
//! surface, a crease fading from the sharp edge's dihedral at the rail's end
//! to tangency at `M`, as a miter seam is.
//!
//! Only the exact-extrusion stripe is converted (a straight edge between
//! two planes, whose rows are two sections joined by lines); everything
//! else is named by the network.

use super::*;
use crate::blend::miter::MarchedCurve;

/// The hood's frame: the shared face's and the side face's outward normals,
/// a point on the stripe's edge (the corner line both planes meet in), the
/// ellipse's horizontal semi-axis `a` (from `M`) and its vertical one `r`.
pub(in crate::blend) struct HoodFrame {
    pub(in crate::blend) n_top: Vec3,
    pub(in crate::blend) n_side: Vec3,
    pub(in crate::blend) origin: Vec3,
    pub(in crate::blend) a: f64,
    pub(in crate::blend) r: f64,
}

impl HoodFrame {
    /// Signed distance of `point` from the side face's plane along its
    /// outward normal (negative inside the material).
    pub(in crate::blend) fn s(&self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.n_side)
    }

    /// Signed distance of `point` from the shared face's plane along its
    /// outward normal (negative inside the material).
    pub(in crate::blend) fn t(&self, point: Vec3) -> f64 {
        point.sub(self.origin).dot(self.n_top)
    }

    /// The hood's level: zero on the elliptic cylinder, positive toward the
    /// corner line (the material the hood removes), negative inside.
    pub(in crate::blend) fn level(&self, point: Vec3) -> f64 {
        let s = (self.s(point) + self.a) / self.a;
        let t = (self.t(point) + self.r) / self.r;
        s * s + t * t - 1.0
    }
}

/// The hood's rows: the stripe's exact-extrusion rows with the shared-face
/// contact of each section moved from `r` to `a` inside the side face, so
/// the section (a rational quadratic through the corner-line apex, weight
/// unchanged — an affine image of the quarter circle) is the quarter
/// ellipse.  `top_is_first` says which of the two support rows lies on the
/// shared face.  The rows' parameterisation, pcurves and side row are the
/// stripe's own.
pub(in crate::blend) fn hood_rows(rows: &FittedRows, top_is_first: bool, frame: &HoodFrame) -> Result<FittedRows, KernelRefusal> {
    let control = &rows.surface.control_points;
    if !rows.exact_extrusion || control.len() != 2 || control.iter().any(|row| row.len() != 3) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "crossed_rails", format!(
            "blend network: the crossed stripe's rows are not the exact extrusion of one section \
             ({} rows of {} controls); the hood is built on that stripe only",
            control.len(),
            control.first().map_or(0, Vec::len)
        )));
    }
    let top_column = if top_is_first { 0 } else { 2 };
    let shift = frame.n_side.scale(-(frame.a - frame.r));
    let mut moved = control.clone();
    let mut top_points = Vec::with_capacity(2);
    for row in &mut moved {
        let point = &mut row[top_column];
        if (point.w - 1.0).abs() > 1e-12 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "crossed_rails_weight", "blend network: a section's contact control point is weighted"));
        }
        let shifted = Vec3::new(point.x, point.y, point.z).add(shift);
        *point = Vec4::from_point(shifted, 1.0);
        top_points.push(shifted);
    }
    let surface = NurbsSurface::new(
        rows.surface.degree_u,
        rows.surface.degree_v,
        rows.surface.knots_u.clone(),
        rows.surface.knots_v.clone(),
        moved,
    )
    .or_refuse(KernelStage::Refine, "new")?;
    let top_line = crate::make_line(top_points[0], top_points[1]).or_refuse(KernelStage::Refine, "make_line")?;
    let (cr, cs) = if top_is_first {
        (top_line, rows.cs.clone())
    } else {
        (rows.cr.clone(), top_line)
    };
    Ok(FittedRows {
        surface,
        cr,
        cs,
        cr_pcurve: rows.cr_pcurve.clone(),
        cs_pcurve: rows.cs_pcurve.clone(),
        u_domain: rows.u_domain,
        center: None,
        // Not a circular section any more: the surgery must not transpose
        // it for the analytic recogniser.
        exact_extrusion: false,
        vertex_stations: rows.vertex_stations,
        revolution_axis: None,
    })
}

/// Samples along a rib of the merged corner.
pub(in crate::blend) const RIB_SAMPLES: usize = 48;

/// The rib on a neighbouring stripe's blend surface: the hood's level set on
/// it, from `M` (station `u_top` at the shared-face side `z_top`) down to the
/// rail's end `q` on the sharp edge (station `u_side` at the side-face side
/// `z_side`), with its images on the blend (solved) and on the hood
/// (projected).  Along a section line of the blend the level is a parabola
/// in the distance from the side face, lowest where that distance is `a`
/// (the hood's top line) and positive at the side face itself, so it has
/// TWO roots about the top line; the rib is the one on the side face's side.
/// Each sample scans the section line from the side-face station toward the
/// top-line station for the first sign change and bisects there.
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn march_rib(
    blend: &NurbsSurface,
    hood: &NurbsSurface,
    frame: &HoodFrame,
    m: Vec3,
    u_top: f64,
    z_top: f64,
    q: Vec3,
    u_side: f64,
    z_side: f64,
    edge_id: u64,
) -> Result<MarchedCurve, KernelRefusal> {
    let [u_low, u_high] = {
        let knots = &blend.knots_u;
        [knots[0], knots[knots.len() - 1]]
    };
    let pad = 0.05 * (u_top - u_side).abs().max(1e-6);
    // The scan runs from the side-face station past the top-line station
    // by one pad, within the rows.
    let u_far = if u_top < u_side { (u_top - pad).max(u_low) } else { (u_top + pad).min(u_high) };
    let level_at = |u: f64, z: f64| -> Result<f64, KernelRefusal> {
        Ok(frame.level(blend.evaluate(u, z).or_refuse(KernelStage::Refine, "evaluate")?))
    };
    const SCAN: usize = 96;
    let on_hood = |point: Vec3| -> Result<[f64; 2], KernelRefusal> {
        let foot = crate::project_point_to_surface(hood, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        Ok([foot.u, foot.v])
    };
    let mut marched = MarchedCurve { points: Vec::with_capacity(RIB_SAMPLES + 1), uv_first: Vec::new(), uv_second: Vec::new() };
    for sample in 0..=RIB_SAMPLES {
        let fraction = sample as f64 / RIB_SAMPLES as f64;
        let z = z_top + (z_side - z_top) * fraction;
        let (point, u) = if sample == 0 {
            (m, u_top)
        } else if sample == RIB_SAMPLES {
            (q, u_side)
        } else {
            // First sign change from the side face toward the top line.
            let mut bracket: Option<(f64, f64, f64)> = None;
            let mut previous = (u_side, level_at(u_side, z)?);
            for step in 1..=SCAN {
                let u = u_side + (u_far - u_side) * step as f64 / SCAN as f64;
                let value = level_at(u, z)?;
                if previous.1 * value <= 0.0 {
                    bracket = Some((previous.0, u, previous.1));
                    break;
                }
                previous = (u, value);
            }
            let Some((mut a, mut b, mut fa)) = bracket else {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "crossed_rails", format!(
                    "blend network: the merged corner's rib on edge {edge_id} is not bracketed at \
                     z = {z:.4} (the level does not change sign between stations {u_side:.6} and \
                     {u_far:.6}); the hood does not cut that stripe's section"
                )));
            };
            for _ in 0..80 {
                let mid = 0.5 * (a + b);
                let fm = level_at(mid, z)?;
                if fa * fm <= 0.0 {
                    b = mid;
                } else {
                    a = mid;
                    fa = fm;
                }
                if (b - a).abs() <= 1e-13 * (1.0 + u_high.abs()) {
                    break;
                }
            }
            let u = 0.5 * (a + b);
            (blend.evaluate(u, z).or_refuse(KernelStage::Refine, "evaluate")?, u)
        };
        marched.points.push(point);
        marched.uv_first.push([u, z]);
        marched.uv_second.push(on_hood(point)?);
    }
    Ok(marched)
}

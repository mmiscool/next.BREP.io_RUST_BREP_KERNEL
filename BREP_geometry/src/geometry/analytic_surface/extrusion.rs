use super::*;

/// An affine translation of a rational profile, in the original chart:
/// `S = profile(t) + direction * (s - sweep_domain[0])`.
/// `profile_is_u` selects `(t,s) = (u,v)` or `(v,u)`. Direction is the
/// displacement per unit parameter, not a normalized vector.
#[derive(Clone, Debug)]
pub struct ExtrusionStructure {
    pub profile: NurbsCurve,
    pub direction: Vec3,
    pub sweep_domain: [f64; 2],
    pub profile_is_u: bool,
}

/// Prove the affine sweep from all homogeneous controls, in either chart
/// direction. Constant weights along each ruling make its denominator
/// independent of the sweep. Its Cartesian controls must be the linear
/// function's Greville coefficients. This also admits knot insertion and
/// degree elevation without mistaking a warped ruled patch for an extrusion.
pub fn extrusion_structure(surface: &NurbsSurface) -> Option<ExtrusionStructure> {
    along(surface, true).or_else(|| along(surface, false))
}

fn along(surface: &NurbsSurface, profile_is_u: bool) -> Option<ExtrusionStructure> {
    let (degree, knots, profile_degree, profile_knots, domain) = if profile_is_u {
        (surface.degree_v, &surface.knots_v, surface.degree_u, &surface.knots_u, surface.domain_v().ok()?)
    } else {
        (surface.degree_u, &surface.knots_u, surface.degree_v, &surface.knots_v, surface.domain_u().ok()?)
    };
    let rows = &surface.control_points;
    let width = rows.first()?.len();
    if degree == 0 || width == 0 || rows.iter().any(|r| r.len() != width) { return None; }
    let (profiles, sweeps) = if profile_is_u { (rows.len(), width) } else { (width, rows.len()) };
    if sweeps < 2 || knots.len() != sweeps + degree + 1 { return None; }
    let control = |i: usize, j: usize| if profile_is_u { rows[i][j] } else { rows[j][i] };
    let greville: Vec<_> = (0..sweeps).map(|j| knots[j+1..j+degree+1].iter().map(|t| (t-domain[0]) / degree as f64).sum::<f64>()).collect();
    let extent = greville[sweeps-1] - greville[0];
    if !extent.is_finite() || extent <= 0.0 { return None; }
    let start = control(0, 0).point().ok()?;
    let direction = control(0, sweeps-1).point().ok()?.sub(start).scale(1.0 / extent);
    let length = direction.length() * (domain[1]-domain[0]);
    if !length.is_finite() || length <= 0.0 { return None; }
    let mut profile = Vec::with_capacity(profiles);
    for i in 0..profiles {
        let first = control(i, 0);
        if !first.w.is_finite() || first.w <= 0.0 { return None; }
        let anchor = first.point().ok()?;
        for j in 0..sweeps {
            let p = control(i,j);
            // A weight mismatch changes the parameter law even if the same
            // geometric straight line is traced. Leave that chart general.
            if !p.w.is_finite() || (p.w-first.w).abs() > 32.0*f64::EPSILON*first.w { return None; }
            let point = p.point().ok()?;
            let expected = anchor.add(direction.scale(greville[j]-greville[0]));
            let magnitude = point.length().max(expected.length());
            let tolerance = 1e-11*length + 32.0*f64::EPSILON*magnitude;
            if !point.length().is_finite() || point.sub(expected).length() > tolerance { return None; }
        }
        let base = anchor.sub(direction.scale(greville[0]));
        profile.push(crate::Vec4 { x: base.x*first.w, y: base.y*first.w, z: base.z*first.w, w: first.w });
    }
    Some(ExtrusionStructure {
        profile: NurbsCurve::new(profile_degree, profile_knots.clone(), profile).ok()?,
        direction, sweep_domain: domain, profile_is_u,
    })
}


#[path = "extrusion/projection.rs"]
mod projection;
pub(crate) use projection::ReducedProfile;
#[path = "extrusion/section.rs"]
mod section;
pub use section::plane_extrusion_section;

#[path = "extrusion/pair.rs"]
mod pair;
pub use pair::{parallel_extrusion_sections, certified_parallel_extrusion_sections};

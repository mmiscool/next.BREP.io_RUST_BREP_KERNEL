//! Recognize a rectangular strip spanning one closed surface direction.
//! Its two meridian boundaries are chart seams, not material boundaries.
use super::*;

pub(super) fn seams(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
) -> Result<Option<[usize; 2]>, KernelRefusal> {
    let (closed_u, closed_v) = surface
        .closed_directions()
        .or_refuse(KernelStage::Collect, "closed_directions")?;
    // A doubly periodic carrier still uses the existing closed-sheet refusal.
    // This lane handles one pair of chart seams, with an open meridian.
    if !closed_u || closed_v || loops.len() != 1 || loops[0].len() != 4 {
        return Ok(None);
    }
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Collect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Collect, "domain_v")?;
    let eps_u = 1e-10 * (u1 - u0);
    let eps_v = 1e-10 * (v1 - v0);
    let mut points = Vec::new();
    let mut ends = Vec::new();
    for curve in &loops[0] {
        if curve.degree != 1
            || curve.control_points.len() != 2
            || (curve.control_points[0].w - curve.control_points[1].w).abs() > 1e-12
        {
            return Ok(None);
        }
        ends.push(
            curve.control_points[1]
                .point()
                .or_refuse(KernelStage::Collect, "point")?,
        );
        points.push(
            curve.control_points[0]
                .point()
                .or_refuse(KernelStage::Collect, "point")?,
        );
    }
    let low = points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let high = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
    if low < v0 - eps_v || high > v1 + eps_v || high - low <= eps_v {
        return Ok(None);
    }
    let corners = [
        Vec3::new(u0, low, 0.0),
        Vec3::new(u1, low, 0.0),
        Vec3::new(u1, high, 0.0),
        Vec3::new(u0, high, 0.0),
    ];
    let matches = |p: Vec3, corner: Vec3| {
        (p.x - corner.x).abs() <= eps_u
            && (p.y - corner.y).abs() <= eps_v
            && p.z.abs() <= eps_u.max(eps_v)
    };
    let Some(shift) = (0..4).find(|shift| {
        (0..4).all(|i| {
            matches(points[i], corners[(i + shift) % 4])
                && matches(ends[i], corners[(i + shift + 1) % 4])
        })
    }) else {
        return Ok(None);
    };
    Ok(Some([(1 + 4 - shift) % 4, (3 + 4 - shift) % 4]))
}

pub(super) fn build(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    bottom_distance: f64,
    top_distance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if seams(surface, loops)?.is_none() {
        return Ok(None);
    }
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Collect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Collect, "domain_v")?;
    validate_trim_loops(
        loops,
        1e-7 * (u1 - u0).max(v1 - v0),
        1e-10 * (u1 - u0) * (v1 - v0),
    )?;
    // Keep the fold lane's named refusals; shared seams cannot repair a
    // surface offset through its centre of curvature.
    ensure_offsets_regular(surface, loops, &[bottom_distance, top_distance])?;
    thicken_one_region(surface, loops, bottom_distance, top_distance).map(Some)
}

use super::*;
pub use super::recognition::{full_circle_knots, nearly_equal_points, surface_scale};

/// Frame, generatrix, and sweep of any surface built by `make_revolution`
/// (full or partial), validated by exact reconstruction. Unlike
/// `AnalyticSurface` this does not require the generatrix to be a line or
/// classic quadric profile.
pub struct RevolutionStructure {
    pub frame: RevolutionFrame,
    pub generatrix: NurbsCurve,
    pub sweep: f64,
}

pub fn circumcenter(a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let normal = ab.cross(ac);
    let normal_squared = normal.dot(normal);
    if !normal_squared.is_finite() || normal_squared <= 1e-28 * ab.dot(ab) * ac.dot(ac) {
        return None;
    }
    let offset = normal
        .cross(ab)
        .scale(ac.dot(ac))
        .add(ac.cross(normal).scale(ab.dot(ab)))
        .scale(1.0 / (2.0 * normal_squared));
    Some(a.add(offset))
}

pub fn revolution_structure(surface: &NurbsSurface) -> Option<RevolutionStructure> {
    if surface.degree_u != 2 {
        return None;
    }
    let rows = &surface.control_points;
    if rows.len() < 3 || rows.len() % 2 == 0 {
        return None;
    }
    let spans = (rows.len() - 1) / 2;
    if spans > 4 {
        return None;
    }
    let expected = full_circle_knots(spans);
    if surface.knots_u.len() != expected.len()
        || surface
            .knots_u
            .iter()
            .zip(&expected)
            .any(|(a, b)| (a - b).abs() > 1e-12)
    {
        return None;
    }
    let scale = surface_scale(surface);
    // The mid-row weight ratio encodes the per-span arc angle.
    let mut sweep = None;
    for column in 0..rows[0].len() {
        let corner = rows[0][column].w;
        let middle = rows[1][column].w;
        if corner.abs() <= 1e-12 {
            continue;
        }
        let ratio = middle / corner;
        if !(ratio > 0.0 && ratio <= 1.0 + 1e-12) {
            return None;
        }
        let arc_angle = 2.0 * ratio.min(1.0).acos();
        if arc_angle <= 1e-9 {
            return None;
        }
        sweep = Some(arc_angle * spans as f64);
        break;
    }
    let sweep = sweep?;
    if sweep > std::f64::consts::TAU + 1e-9 {
        return None;
    }
    // Frame from three points on a radiused angular arc (0.4/0.8 avoid the
    // closed-circle wraparound where evaluate(1.0) == evaluate(0.0)).
    let mut frame = None;
    for column in 0..rows[0].len() {
        let arc_points: Vec<crate::Vec4> = rows.iter().map(|row| row[column]).collect();
        let Ok(arc) = NurbsCurve::new(2, surface.knots_u.clone(), arc_points) else {
            return None;
        };
        let p0 = arc.evaluate(0.0).ok()?;
        let p1 = arc.evaluate(0.4).ok()?;
        let p2 = arc.evaluate(0.8).ok()?;
        let Some(center) = circumcenter(p0, p1, p2) else {
            continue;
        };
        let radial = p0.sub(center);
        let radius = radial.length();
        if radius <= RECOGNITION_TOLERANCE * scale.max(1.0) {
            continue;
        }
        let axis = radial.cross(p1.sub(center)).normalized().ok()?;
        let x_axis = radial.scale(1.0 / radius);
        let y_axis = axis.cross(x_axis);
        frame = Some(RevolutionFrame {
            origin: center,
            axis,
            x_axis,
            y_axis,
        });
        break;
    }
    let frame = frame?;
    let generatrix =
        NurbsCurve::new(surface.degree_v, surface.knots_v.clone(), rows[0].clone()).ok()?;
    let rebuilt = make_revolution(frame.origin, frame.axis, &generatrix, sweep).ok()?;
    if !nearly_equal_points(surface, &rebuilt, scale) {
        return None;
    }
    Some(RevolutionStructure {
        frame,
        generatrix,
        sweep,
    })
}

pub(super) fn rotate_curve_about_axis(
    curve: &NurbsCurve,
    origin: Vec3,
    axis: Vec3,
    angle: f64,
) -> Option<NurbsCurve> {
    let (sin, cos) = angle.sin_cos();
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point().ok()?;
            let d = point.sub(origin);
            let axial = axis.scale(d.dot(axis));
            let radial = d.sub(axial);
            let ortho = axis.cross(radial);
            let rotated = origin
                .add(axial)
                .add(radial.scale(cos))
                .add(ortho.scale(sin));
            Some(crate::Vec4::from_point(rotated, control.w))
        })
        .collect::<Option<Vec<_>>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).ok()
}

/// Exact intersection of two surfaces of revolution about a COMMON axis:
/// by rotational symmetry the intersection is the revolution of the 2D
/// profile intersections, i.e. circular arcs over the angular overlap.
/// One exact curve×surface hit-test on a generatrix placed mid-overlap
/// finds the (radius, height) pairs; empty hits prove the surfaces
/// disjoint (skipping the marcher, which is ill-conditioned exactly here
/// when the profiles touch tangentially).
pub(super) fn intersect_coaxial_revolutions(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let a = revolution_structure(first)?;
    let b = revolution_structure(second)?;
    let scale = surface_scale(first).max(surface_scale(second)).max(1.0);
    let alignment = a.frame.axis.dot(b.frame.axis);
    if alignment.abs() < 1.0 - 1e-9 {
        return None;
    }
    let offset = b.frame.origin.sub(a.frame.origin);
    let perpendicular = offset.sub(a.frame.axis.scale(offset.dot(a.frame.axis)));
    if perpendicular.length() > 1e-9 * scale {
        return None;
    }
    // Angular interval of `second` expressed in `first`'s frame, its start
    // brought into [0, τ) first. `atan2` lands in (-π, π], so an anti-aligned
    // full-sweep `second` read as (start − τ, start) began as low as −3π and
    // the ±τ shifts below reached only three quarters of it: the hosted
    // 2026-11-02 "boolean fails" reports' revolve caps (axis down) against a
    // bore cylinder (axis up) imprinted a 270° arc and the bore never
    // pierced the cap.
    let b_start = b
        .frame
        .x_axis
        .dot(a.frame.y_axis)
        .atan2(b.frame.x_axis.dot(a.frame.x_axis));
    let tau = std::f64::consts::TAU;
    let (b_lo, b_hi) = if alignment > 0.0 {
        (b_start, b_start + b.sweep)
    } else {
        (b_start - b.sweep, b_start)
    };
    let b_wrapped = b_lo.rem_euclid(tau);
    let (b_lo, b_hi) = (b_wrapped, b_hi + (b_wrapped - b_lo));
    let mut intervals = Vec::new();
    for shift in [-tau, 0.0, tau] {
        let lo = (b_lo + shift).max(0.0);
        let hi = (b_hi + shift).min(a.sweep);
        if hi - lo > 1e-9 {
            intervals.push((lo, hi));
        }
    }
    // The pieces of one overlap that the wrap cut apart are one arc again,
    // so a full turn comes back as the one closed circle it is.
    intervals.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (lo, hi) in intervals {
        if let Some(last) = merged.last_mut() {
            if lo <= last.1 + 1e-9 {
                last.1 = last.1.max(hi);
                continue;
            }
        }
        merged.push((lo, hi));
    }
    let intervals = merged;
    if intervals.is_empty() {
        return Some(Vec::new());
    }
    let middle = (intervals[0].0 + intervals[0].1) / 2.0;
    let generatrix = rotate_curve_about_axis(&a.generatrix, a.frame.origin, a.frame.axis, middle)?;
    let hits = crate::intersect_curve_surface(&generatrix, second, tolerance).ok()?;
    if hits.len() > 8 {
        // A hit spray means the profiles overlap (coincident surfaces);
        // leave that to the cosurface machinery.
        return None;
    }
    let radius_floor = (tolerance * 10.0).max(1e-9 * scale);
    let mut seen: Vec<(f64, f64)> = Vec::new();
    let mut curves = Vec::new();
    for hit in hits {
        // TANGENT-CONTACT SKIP: a meridian hit whose curve tangent lies IN
        // the other surface (hit.tangential — computed exactly by the
        // curve×surface intersector, no projector involved) sweeps a CONTACT
        // circle where the coaxial surfaces touch without crossing. Minting
        // it duplicates the true section ring (the cap-plane circle) and
        // strands one-use edges at assembly. A transversal meridian crossing
        // keeps tangential=false and is unaffected. Escape hatch:
        // BREP_COAXIAL_TANGENT_SKIP=0.
        if hit.tangential
            && std::env::var("BREP_COAXIAL_TANGENT_SKIP").as_deref() != Ok("0")
        {
            continue;
        }
        let (_, radius, axial) = a.frame.cylindrical(hit.point);
        if radius <= radius_floor {
            continue;
        }
        if seen
            .iter()
            .any(|(r, z)| (r - radius).abs() + (z - axial).abs() <= radius_floor)
        {
            continue;
        }
        seen.push((radius, axial));
        for (lo, hi) in &intervals {
            let center = a.frame.origin.add(a.frame.axis.scale(axial));
            curves.push(
                make_arc(center, a.frame.x_axis, a.frame.y_axis, radius, *lo, *hi).ok()?,
            );
        }
    }
    Some(curves)
}

/// COAXIAL ISO SECTION (`BREP_COAXIAL_ISO_SECTION`, default on). A cylinder
/// (a revolution whose generatrix is a straight segment parallel to its axis)
/// against a surface whose distance from that axis does not change along one
/// of its parameters, as a wall carried along a screw or turned about that
/// axis has: the radius is then a function r(t) of the other parameter alone,
/// and the section is the iso-curve of the surface at each root of r(t) = R,
/// exactly, where the marcher fits one through stations to its own tolerance.
///
/// No control-net bound proves the invariance (a carried wall follows its
/// screw only to its carry's error), so the lane stands on dense samples and
/// answers only when every one agrees, else `None` and the pair goes the way
/// it went before:
/// - the radius at every sample stays within `tolerance` of its value at the
///   middle of the invariant parameter;
/// - r − R changes sign between the same samples across the other parameter in
///   every sampled column (no section begins or ends inside the surface), and
///   no sample comes within ten tolerances of R (no touching root);
/// - every root is clear of the domain's ends (an edge the face owns);
/// - every iso-curve lies on the cylinder within `tolerance` at sixteen points
///   a knot span.
/// - the cylinder is a whole turn, and its generatrix's control heights are
///   monotone (so the patch's height range is that of its ends).
/// Each curve is clipped to that height range on its control points
/// (`clip_to_cylinder_height`).
///
/// The samples are 4 to 16 a knot span. Along the middle column the roots
/// are proven, not sampled: between two samples of one sign, and on each
/// side of a found root up to a gap whose control points span `tolerance`,
/// the column's convex hull is shown to stay on its side
/// (`column_keeps_side`). What stays sampled is the invariance itself.
pub(super) fn intersect_cylinder_radius_invariant(
    cylinder: &NurbsSurface,
    wall: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let structure = revolution_structure(cylinder)?;
    let frame = &structure.frame;
    let scale = surface_scale(cylinder).max(surface_scale(wall)).max(1.0);
    let generatrix: Vec<Vec3> = structure.generatrix.control_points.iter().map(|c| c.point().ok()).collect::<Option<_>>()?;
    let radial = |p: Vec3| {
        let d = p.sub(frame.origin);
        d.sub(frame.axis.scale(d.dot(frame.axis)))
    };
    let first_radial = radial(generatrix[0]);
    let radius = first_radial.length();
    if radius <= 1e-9 * scale || generatrix.iter().any(|p| radial(*p).sub(first_radial).length() > 1e-12 * scale) {
        return None;
    }
    // The patch's height range is the generatrix's image, read at its ends:
    // with positive weights and monotone control heights the height along it
    // is monotone (variation diminishing), else the image is not its ends.
    let heights: Vec<f64> = generatrix.iter().map(|p| p.sub(frame.origin).dot(frame.axis)).collect();
    let rising = heights.windows(2).all(|h| h[1] >= h[0]);
    let falling = heights.windows(2).all(|h| h[1] <= h[0]);
    if !(rising || falling) || structure.generatrix.control_points.iter().any(|c| c.w <= 0.0) {
        return None;
    }
    let [g0, g1] = structure.generatrix.domain().ok()?;
    let height_at = |t: f64| -> Option<f64> { Some(structure.generatrix.evaluate(t).ok()?.sub(frame.origin).dot(frame.axis)) };
    let (h0, h1) = (height_at(g0)?, height_at(g1)?);
    let (low, high) = (h0.min(h1), h0.max(h1));
    if high - low <= 1e-9 * scale {
        return None;
    }
    if structure.sweep < std::f64::consts::TAU - 1e-9 {
        return None; // a part of a turn: its azimuth is not clipped
    }
    let r_at = |u: f64, v: f64| -> Option<f64> { Some(radial(wall.evaluate(u, v).ok()?).length()) };
    let samples = |knots: &[f64], degree: usize| -> Vec<f64> {
        let (lo, hi) = (knots[degree], knots[knots.len() - degree - 1]);
        let mut breaks: Vec<f64> = knots[degree..knots.len() - degree].to_vec();
        breaks.dedup();
        let per = (64 / breaks.len().max(2)).clamp(4, 16);
        // Off the knots and off round fractions of a span by an irrational
        // step, so that no sample lands on a symmetric root by construction.
        let mut out = vec![lo];
        for pair in breaks.windows(2) {
            for k in 0..per {
                out.push(pair[0] + (pair[1] - pair[0]) * (k as f64 + 0.381_966_011_250_105) / per as f64);
            }
        }
        out.push(hi);
        out
    };
    let us = samples(&wall.knots_u, wall.degree_u);
    let vs = samples(&wall.knots_v, wall.degree_v);
    if us.len() * vs.len() > 40_000 {
        return None;
    }
    // r at every sample: grid[i][j] = r(us[i], vs[j]).
    let grid: Vec<Vec<f64>> = us.iter().map(|&u| vs.iter().map(|&v| r_at(u, v)).collect::<Option<Vec<_>>>()).collect::<Option<_>>()?;
    for invariant_v in [true, false] {
        // `across` runs over the parameter r depends on, `along` over the invariant one.
        let (across, along) = if invariant_v { (&us, &vs) } else { (&vs, &us) };
        let value = |i: usize, j: usize| if invariant_v { grid[i][j] } else { grid[j][i] };
        let middle = along.len() / 2;
        let invariant = (0..across.len()).all(|i| (0..along.len()).all(|j| (value(i, j) - value(i, middle)).abs() <= tolerance));
        if !invariant {
            continue;
        }
        if (0..across.len()).any(|i| (0..along.len()).any(|j| (value(i, j) - radius).abs() <= 10.0 * tolerance)) {
            return None;
        }
        let signs = |j: usize| -> Vec<bool> { (0..across.len()).map(|i| value(i, j) > radius).collect() };
        let pattern = signs(middle);
        if (0..along.len()).any(|j| signs(j) != pattern) {
            return None;
        }
        let fixed = along[middle];
        let g = |t: f64| -> Option<f64> { Some(if invariant_v { r_at(t, fixed)? } else { r_at(fixed, t)? } - radius) };
        let (domain_lo, domain_hi) = (across[0], across[across.len() - 1]);
        // The middle column as a curve: between two samples of one sign its
        // convex hull must prove that side (`column_keeps_side`), so a dip to
        // R between samples, however narrow, declines the pair.
        // It is worked on over a domain RESCALED_LENGTH long (`to`), where a
        // split's absolute knot tolerance is a relative 1e-13.
        let (column, c0, c1) = rescaled(&if invariant_v { wall.iso_curve_v(fixed).ok()? } else { wall.iso_curve_u(fixed).ok()? })?;
        let to = |t: f64| (t - c0) / (c1 - c0) * RESCALED_LENGTH;
        for i in 0..across.len() - 1 {
            if pattern[i] == pattern[i + 1] {
                column_keeps_side(&sub_curve(&column, to(across[i]), to(across[i + 1]))?, &radial, radius, pattern[i], 10.0 * tolerance)?;
            }
        }
        let mut curves = Vec::new();
        for i in 0..across.len() - 1 {
            if pattern[i] == pattern[i + 1] {
                continue;
            }
            let (a0, b0) = (across[i], across[i + 1]);
            let (mut a, mut b) = (a0, b0);
            let positive_at_a = pattern[i];
            for _ in 0..200 {
                let mid = 0.5 * (a + b);
                if mid <= a || mid >= b {
                    break;
                }
                if (g(mid)? > 0.0) == positive_at_a {
                    a = mid;
                } else {
                    b = mid;
                }
            }
            let root = 0.5 * (a + b);
            if (root - domain_lo).abs() <= 1e-9 * (domain_hi - domain_lo) || (domain_hi - root).abs() <= 1e-9 * (domain_hi - domain_lo) {
                return None;
            }
            // One root in the interval: each side of it keeps its sample's
            // side up to a gap about the root whose control points span no
            // more than `tolerance` (any further root there is that close).
            let (root_s, a0, b0) = (to(root), to(a0), to(b0));
            let mut delta = 1e-6 * (b0 - a0);
            loop {
                if delta < 4.0 * crate::KNOT_IDENTITY_TOL || root_s - delta <= a0 + 2.0 * crate::KNOT_IDENTITY_TOL || root_s + delta >= b0 - 2.0 * crate::KNOT_IDENTITY_TOL {
                    return None;
                }
                let gap = sub_curve(&column, root_s - delta, root_s + delta)?;
                let points: Vec<Vec3> = gap.control_points.iter().map(|c| c.point().ok()).collect::<Option<_>>()?;
                if points.iter().all(|p| p.sub(points[0]).length() <= 0.5 * tolerance) {
                    break;
                }
                delta *= 0.1;
            }
            column_keeps_side(&sub_curve(&column, a0, root_s - delta)?, &radial, radius, positive_at_a, 0.0)?;
            column_keeps_side(&sub_curve(&column, root_s + delta, b0)?, &radial, radius, !positive_at_a, 0.0)?;
            let curve = if invariant_v { wall.iso_curve_u(root).ok()? } else { wall.iso_curve_v(root).ok()? };
            // On the cylinder at sixteen points a span.
            let mut breaks: Vec<f64> = curve.knots[curve.degree..curve.knots.len() - curve.degree].to_vec();
            breaks.dedup();
            for pair in breaks.windows(2) {
                for k in 0..=16 {
                    let point = curve.evaluate(pair[0] + (pair[1] - pair[0]) * k as f64 / 16.0).ok()?;
                    if (radial(point).length() - radius).abs() > tolerance {
                        return None;
                    }
                }
            }
            curves.extend(clip_to_cylinder_height(&curve, frame, low, high, tolerance)?);
        }
        return Some(curves);
    }
    None
}

/// The domain length the iso section halves its curves over.
const RESCALED_LENGTH: f64 = 1e4;

/// `curve` reparametrised affinely onto `[0, RESCALED_LENGTH]`, with its
/// own domain's ends.
fn rescaled(curve: &NurbsCurve) -> Option<(NurbsCurve, f64, f64)> {
    let [t0, t1] = curve.domain().ok()?;
    if !(t1 > t0) {
        return None;
    }
    let knots = curve.knots.iter().map(|k| (k - t0) / (t1 - t0) * RESCALED_LENGTH).collect();
    Some((NurbsCurve::new(curve.degree, knots, curve.control_points.clone()).ok()?, t0, t1))
}

/// `curve` from `a` to `b`; an end within a knot's identity tolerance of
/// the curve's own end is that end.
fn sub_curve(curve: &NurbsCurve, a: f64, b: f64) -> Option<NurbsCurve> {
    let [start, _] = curve.domain().ok()?;
    let mut piece = curve.clone();
    if a > start + crate::KNOT_IDENTITY_TOL {
        piece = piece.split(a).ok()?.1;
    }
    let [_, end] = piece.domain().ok()?;
    if b < end - crate::KNOT_IDENTITY_TOL {
        piece = piece.split(b).ok()?.0;
    }
    Some(piece)
}

/// `Some` when every point of `curve` is proven to lie outside (`outside`)
/// or inside the radius `radius` about the axis `radial` measures from, on
/// its control points (positive weights keep it in their convex hull):
/// inside when every control point's distance is below `radius` (distance
/// from the axis is convex), outside when every control point lies beyond
/// `radius` along the radial direction of the piece's middle. A piece that
/// proves neither is halved; `None` when a middle is on the wrong side or
/// within `clearance` of the radius, a weight is not positive, a split
/// refuses or the pieces pass a budget.
fn column_keeps_side(curve: &NurbsCurve, radial: &dyn Fn(Vec3) -> Vec3, radius: f64, outside: bool, clearance: f64) -> Option<()> {
    let mut stack = vec![curve.clone()];
    let mut budget = 4096usize;
    while let Some(piece) = stack.pop() {
        budget = budget.checked_sub(1)?;
        if piece.control_points.iter().any(|c| c.w <= 0.0) {
            return None;
        }
        let q: Vec<Vec3> = piece.control_points.iter().map(|c| c.point().ok().map(|p| radial(p))).collect::<Option<_>>()?;
        let [a, b] = piece.domain().ok()?;
        let mid = 0.5 * (a + b);
        let m = radial(piece.evaluate(mid).ok()?);
        let r = m.length();
        let proven = if outside {
            if r - radius <= clearance {
                return None;
            }
            let n = m.scale(1.0 / r);
            q.iter().all(|p| p.dot(n) > radius)
        } else {
            if radius - r <= clearance {
                return None;
            }
            q.iter().all(|p| p.length() < radius)
        };
        if proven {
            continue;
        }
        let (left, right) = piece.split(mid).ok()?;
        stack.push(left);
        stack.push(right);
    }
    Some(())
}

/// The parts of `curve` (on the cylinder about `frame`) from height `low` to
/// `high`, found on its control points: a piece whose points all lie in the
/// band is in it, one whose points all lie on one side is out (a rational
/// curve with positive weights stays in their convex hull), and one that
/// straddles a side is halved until it is a parameter's rounding long. `None`
/// for a weight that is not positive.
pub(super) fn clip_to_cylinder_height(curve: &NurbsCurve, frame: &RevolutionFrame, low: f64, high: f64, tolerance: f64) -> Option<Vec<NurbsCurve>> {
    if curve.control_points.iter().any(|c| c.w <= 0.0) {
        return None;
    }
    let (work, t0, t1) = rescaled(curve)?;
    let back = |s: f64| t0 + s / RESCALED_LENGTH * (t1 - t0);
    let height = |c: &NurbsCurve| -> Option<(f64, f64)> {
        c.control_points.iter().try_fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            let h = p.point().ok()?.sub(frame.origin).dot(frame.axis);
            Some((lo.min(h), hi.max(h)))
        })
    };
    let (low, high) = (low - tolerance, high + tolerance);
    let mut inside: Vec<(f64, f64)> = Vec::new();
    let mut stack = vec![(work, 0.0, RESCALED_LENGTH)];
    while let Some((piece, a, b)) = stack.pop() {
        let (lo, hi) = height(&piece)?;
        if hi < low || lo > high {
            continue;
        }
        if lo >= low && hi <= high {
            inside.push((back(a), back(b)));
            continue;
        }
        let mid = 0.5 * (a + b);
        // Halved until the split refuses (a knot's identity tolerance long).
        let halves = if b - a > 1e-13 * RESCALED_LENGTH && mid > a && mid < b { piece.split(mid).ok() } else { None };
        let Some((left, right)) = halves else {
            // A piece a rounding long at a side of the band: in when its
            // middle is, if its heights span no more than `tolerance`; the
            // split's absolute tolerance can leave a long piece, which declines.
            if hi - lo > tolerance {
                return None;
            }
            let h = piece.evaluate(mid).ok()?.sub(frame.origin).dot(frame.axis);
            if h >= low && h <= high {
                inside.push((back(a), back(b)));
            }
            continue;
        };
        stack.push((right, mid, b));
        stack.push((left, a, mid));
        if stack.len() > 10_000 {
            return None;
        }
    }
    inside.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut merged: Vec<(f64, f64)> = Vec::new();
    for (a, b) in inside {
        match merged.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    let mut out = Vec::new();
    for (a, b) in merged {
        if b - a <= 1e-12 * (t1 - t0) {
            continue;
        }
        // An end within a knot's identity tolerance of the domain's is the domain's.
        let mut piece = curve.clone();
        if let Ok((_, right)) = piece.split(a) {
            piece = right;
        }
        if let Ok((left, _)) = piece.split(b) {
            piece = left;
        }
        // The ends a refused or snapped split left must still be in the band.
        let [p0, p1] = piece.domain().ok()?;
        for t in [p0, p1] {
            let h = piece.evaluate(t).ok()?.sub(frame.origin).dot(frame.axis);
            if h < low - tolerance || h > high + tolerance {
                return None;
            }
        }
        out.push(piece);
    }
    Some(out)
}

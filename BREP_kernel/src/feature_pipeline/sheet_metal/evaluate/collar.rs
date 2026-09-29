use super::*;
use super::frame::Frame;

/// A hole-loop segment's flat-local 2D endpoints (start, end of its domain).
pub(super) fn segment_endpoints_2d(curve: &crate::NurbsCurve) -> Result<([f64; 2], [f64; 2]), String> {
    let domain = curve.domain()?;
    let a = curve.evaluate(domain[0])?;
    let b = curve.evaluate(domain[1])?;
    Ok(([a.x, a.y], [b.x, b.y]))
}

/// Recognize a hole's outer loop as one full circle: every sampled point
/// equidistant (relative 1e-7) from the circumcenter of three spread samples.
/// Returns the flat-local `(center, radius)`. Exact-arc loops (the sketch/tab
/// bake) pass; polygons and mixed loops return `None`.
pub(crate) fn circle_of_loop(loop_curves: &[crate::NurbsCurve]) -> Option<([f64; 2], f64)> {
    let mut samples: Vec<[f64; 2]> = Vec::new();
    for curve in loop_curves {
        let domain = curve.domain().ok()?;
        for step in 0..8 {
            let t = domain[0] + (domain[1] - domain[0]) * (step as f64 / 8.0);
            let point = curve.evaluate(t).ok()?;
            samples.push([point.x, point.y]);
        }
    }
    if samples.len() < 6 {
        return None;
    }
    let (p1, p2, p3) = (
        samples[0],
        samples[samples.len() / 3],
        samples[2 * samples.len() / 3],
    );
    let d = 2.0 * (p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]));
    if d.abs() < 1e-12 {
        return None;
    }
    let sq = |p: [f64; 2]| p[0] * p[0] + p[1] * p[1];
    let cx = (sq(p1) * (p2[1] - p3[1]) + sq(p2) * (p3[1] - p1[1]) + sq(p3) * (p1[1] - p2[1])) / d;
    let cy = (sq(p1) * (p3[0] - p2[0]) + sq(p2) * (p1[0] - p3[0]) + sq(p3) * (p2[0] - p1[0])) / d;
    let radius = ((p1[0] - cx).powi(2) + (p1[1] - cy).powi(2)).sqrt();
    if radius < 1e-9 {
        return None;
    }
    let tolerance = radius.max(1.0) * 1e-7;
    for point in &samples {
        let r = ((point[0] - cx).powi(2) + (point[1] - cy).powi(2)).sqrt();
        if (r - radius).abs() > tolerance {
            return None;
        }
    }
    Some(([cx, cy], radius))
}

/// The circular-hole collar: ONE full-turn revolve about the hole axis whose
/// cross-section is the bend's annular wedge (torus-sector faces) glued to the
/// straight wall leg (cylinder at 90°, cone otherwise), in the canonical
/// `(x = radial ρ, y = axial)` half-plane, folding toward −y:
///
/// * bend-arc center `C = (ρ_e, −Rm)` where `ρ_e` = the (possibly inset-shifted)
///   hole rim radius stored in the loop itself;
/// * the radial ray `d(φ) = (−sinφ, cosφ)` runs from the bore rectangle
///   (`φ = 0` ⇒ the segment `ρ = ρ_e, y ∈ [−t/2, t/2]`) around to the wall at
///   `φ = θ`; the wall leg extends along the tangent `t̂(θ) = (−cosθ, −sinθ)`,
///   so with the default `material_inside` inset (`ρ_e = hole + R + t`) the
///   finished collar bore lands back exactly on the original hole radius;
/// * the profile carries a thin OVERLAP RING extending `t/4` INTO the plate
///   material: an exactly-coincident cylinder↔cylinder union (collar rim on
///   the plate bore) mis-classifies, while the ring makes it transversal and
///   is volume-neutral (entirely subsumed by the plate, its top/bottom in the
///   plate's own face planes — the easy coplanar case).
///
/// `fold ≈ 0` returns `None`: a drawn collar is not developable into a bend
/// allowance strip, so the flat pattern shows only the opening.
pub(super) fn build_collar(
    hole_bend: &HoleBend,
    loop_curves: &[crate::NurbsCurve],
    leg: f64,
    frame: Frame,
    thickness: f64,
    fold: f64,
) -> Result<Option<BrepSolid>, String> {
    let theta = hole_bend.angle_deg.to_radians() * fold;
    if theta.abs() < 1e-9 {
        return Ok(None);
    }
    let (center, rho_e) = circle_of_loop(loop_curves).ok_or_else(|| {
        format!(
            "sheet-metal: collar `{}` sits on a non-circular hole loop",
            hole_bend.id
        )
    })?;
    let side = if theta >= 0.0 { 1.0 } else { -1.0 };
    let mag = theta.abs();
    let half_t = thickness * 0.5;
    let r_mid = (hole_bend.inside_radius + half_t).max(1e-6);
    let r_in = (r_mid - half_t).max(1e-6);
    let r_out = r_mid + half_t;
    if !(rho_e - r_out > 1e-6) {
        return Err(format!(
            "sheet-metal: collar `{}` needs bend radius + thickness ({r_out:.4}) smaller than the hole rim radius ({rho_e:.4})",
            hole_bend.id
        ));
    }

    let c = Vec3::new(rho_e, -r_mid, 0.0);
    let dir = |phi: f64| [-phi.sin(), phi.cos()];
    let (dm, tm) = (dir(mag), [-mag.cos(), -mag.sin()]);
    let at = |r: f64, d: [f64; 2]| Vec3::new(c.x + d[0] * r, c.y + d[1] * r, 0.0);
    let rim_bottom = Vec3::new(rho_e, -half_t, 0.0); // = C + R_in·d(0)
    let rim_top = Vec3::new(rho_e, half_t, 0.0); // = C + R_out·d(0)
    let bend_out_end = at(r_out, dm);
    let bend_in_end = at(r_in, dm);
    let wall_out_tip = Vec3::new(bend_out_end.x + tm[0] * leg, bend_out_end.y + tm[1] * leg, 0.0);
    let wall_in_tip = Vec3::new(bend_in_end.x + tm[0] * leg, bend_in_end.y + tm[1] * leg, 0.0);
    let ov = half_t * 0.5;
    let ring_bottom = Vec3::new(rho_e + ov, -half_t, 0.0);
    let ring_top = Vec3::new(rho_e + ov, half_t, 0.0);
    // make_arc sweeps increasing angle with point = C + x̂·r·cosα + ŷ·r·sinα;
    // x̂ = +y, ŷ = −x reproduces C + r·d(α) exactly.
    let arc_x = Vec3::new(0.0, 1.0, 0.0);
    let arc_y = Vec3::new(-1.0, 0.0, 0.0);
    let profile = vec![
        make_line(rim_bottom, ring_bottom)?, // ring bottom (in the plate B plane)
        make_line(ring_bottom, ring_top)?,   // buried rim (consumed by the union)
        make_line(ring_top, rim_top)?,       // ring top (in the plate A plane)
        make_arc(c, arc_x, arc_y, r_out, 0.0, mag)?, // outer torus face
        make_line(bend_out_end, wall_out_tip)?, // wall, opening side
        make_line(wall_out_tip, wall_in_tip)?, // free rim annulus / cone cap
        make_line(wall_in_tip, bend_in_end)?, // wall, material side
        make_arc(c, arc_x, arc_y, r_in, 0.0, mag)?.reversed()?, // inner torus face
    ];
    let side_names = vec![
        None,
        Some(format!("{}:rim", hole_bend.id)),
        None,
        Some(format!("{}:A", hole_bend.id)),
        Some(format!("{}:wall:A", hole_bend.id)),
        Some(format!("{}:tip", hole_bend.id)),
        Some(format!("{}:wall:B", hole_bend.id)),
        Some(format!("{}:B", hole_bend.id)),
    ];
    let collar = revolve_profile_brep_named(
        &profile,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        std::f64::consts::TAU,
        &side_names,
        &[None, None],
    )?;
    // Place: canonical ŷ (the profile's axial direction) → ±flat-normal. The
    // mirrored (fold-up) side flips the third column too, keeping the affine a
    // proper rotation.
    let axis_frame = Frame {
        o: frame.point(center[0], center[1], 0.0),
        u: frame.u,
        v: frame.w.scale(side),
        w: frame.v.scale(-side),
    };
    transform_brep(&collar, axis_frame.affine()?, false).map(Some)
}

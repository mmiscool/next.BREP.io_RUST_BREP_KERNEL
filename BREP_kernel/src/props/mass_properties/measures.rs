use super::*;
use super::mass_profile;

pub fn face_area(face: &FaceRecord) -> Result<f64, String> {
    if is_affine(&face.surface)? {
        return Ok(closed_parameter_space_area(face)?.abs() * planar_metric(face)?);
    }
    if let Some(values) = biperiodic_band_integral(face, &[Integrand::Area])? {
        return Ok(values[0]);
    }
    if is_untrimmed(face)? {
        integrate_untrimmed(face, Integrand::Area)
    } else {
        integrate_trimmed(face, Integrand::Area)
    }
}

/// Exact 3D arc length of a NURBS curve over the parameter range `[t0, t1]` —
/// ∫|C'(t)| dt evaluated with a composite 8-point Gauss–Legendre rule paneled at
/// the curve's interior knots (each knot span further halved for headroom). The
/// speed integrand `|C'(t)|` is polynomial-exact for the line/circle-arc curves
/// the kernel emits and converges tightly for general NURBS. Returns `0.0` for a
/// zero-width (or inverted) range.
pub fn curve_arc_length(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<f64, String> {
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    if !(hi > lo) || !hi.is_finite() || !lo.is_finite() {
        return Ok(0.0);
    }
    // Break the range at the curve's interior knots so a panel never straddles a
    // C0 kink; then halve each span for extra Gauss headroom.
    let mut breaks: Vec<f64> = vec![lo, hi];
    for &knot in &curve.knots {
        if knot > lo + 1e-12 && knot < hi - 1e-12 {
            breaks.push(knot);
        }
    }
    breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    breaks.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);

    let mut length = 0.0;
    for window in breaks.windows(2) {
        let (a, b) = (window[0], window[1]);
        if !(b > a) {
            continue;
        }
        // Two sub-panels per knot span, each read at the order its own
        // weights ask for: `|C'|` carries the weight in the denominator, so a
        // re-weighted copy of the same curve is the same length only if the
        // rule outruns the weight's nearest root (`rule`). A unit-weight span
        // keeps the two 8-point panels, bit for bit.
        for sub in 0..2 {
            let sa = a + (b - a) * sub as f64 / 2.0;
            let sb = a + (b - a) * (sub + 1) as f64 / 2.0;
            for panel in rule::curve_panels(curve, sa, sb)? {
                let half = 0.5 * (panel.hi - panel.lo);
                let middle = 0.5 * (panel.lo + panel.hi);
                let (nodes, weights) = panel.nodes();
                for index in 0..nodes.len() {
                    let t = middle + half * nodes[index];
                    let speed = curve.derivatives(t, 1)?[1].length();
                    length += weights[index] * half * speed;
                }
            }
        }
    }
    Ok(length)
}

/// Exact 3D arc length of one edge over its active parameter range `[t0, t1]`
/// (the same range the display sampler walks). Degenerate edges measure `0`.
pub fn edge_arc_length(edge: &EdgeRecord) -> Result<f64, String> {
    if edge.degenerate {
        return Ok(0.0);
    }
    curve_arc_length(&edge.curve, edge.t0, edge.t1)
}

/// Total 3D arc length of every non-degenerate edge of a solid (each shared edge
/// counted ONCE — the solid's edge table is already edge-unique). This is the
/// solid's "total edge length" measurement.
pub fn solid_edge_length_total(solid: &BrepSolid) -> Result<f64, String> {
    let mut total = 0.0;
    for edge in &solid.edges {
        total += edge_arc_length(edge)?;
    }
    Ok(total)
}

/// Total 3D arc length of a face's boundary edges: the unique edges referenced by
/// the face's loop coedges, resolved against the owning `solid`'s edge table and
/// summed once each. An edge referenced twice by the same face (e.g. a seam) is
/// still counted once.
pub fn face_boundary_length(solid: &BrepSolid, face: &FaceRecord) -> Result<f64, String> {
    let mut edge_ids: Vec<u64> = Vec::new();
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            if !edge_ids.contains(&coedge.edge_id) {
                edge_ids.push(coedge.edge_id);
            }
        }
    }
    let mut total = 0.0;
    for id in edge_ids {
        if let Some(edge) = solid.edges.iter().find(|edge| edge.id == id) {
            total += edge_arc_length(edge)?;
        }
    }
    Ok(total)
}

pub fn face_volume_contribution(face: &FaceRecord) -> Result<f64, String> {
    if is_affine(&face.surface)? {
        let signed_area = closed_parameter_space_area(face)? * planar_metric(face)?;
        let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
        let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
        let u = ku.domain()[0];
        let v = kv.domain()[0];
        let point = face.surface.evaluate(u, v)?;
        let normal = face.surface.normal(u, v)?;
        return Ok(point.dot(normal) * signed_area / 3.0);
    }
    if let Some(values) = biperiodic_band_integral(face, &[Integrand::Volume])? {
        return Ok(values[0] / 3.0);
    }
    let integral = if is_untrimmed(face)? {
        integrate_untrimmed(face, Integrand::Volume)?
    } else {
        integrate_trimmed(face, Integrand::Volume)?
    };
    Ok(integral / 3.0)
}

/// Area AND volume flux of an AFFINE face from ONE parameter-space area and
/// ONE planar metric. [`face_area`] and [`face_volume_contribution_about`]
/// each compute both quantities from scratch, so asking a plane for its area
/// and its volume walked every trim pcurve twice; the two are pure functions
/// of the face, so sharing them is bit-for-bit the same pair of numbers.
pub(super) fn affine_area_and_volume_about(
    face: &FaceRecord,
    reference: Vec3,
) -> Result<(f64, f64), String> {
    let parameter_area = closed_parameter_space_area(face)?;
    let metric = planar_metric(face)?;
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let u = ku.domain()[0];
    let v = kv.domain()[0];
    let point = face.surface.evaluate(u, v)?;
    let normal = face.surface.normal(u, v)?;
    let signed_area = parameter_area * metric;
    Ok((
        parameter_area.abs() * metric,
        point.sub(reference).dot(normal) * signed_area / 3.0,
    ))
}

pub(super) fn face_volume_contribution_about(face: &FaceRecord, reference: Vec3) -> Result<f64, String> {
    if is_affine(&face.surface)? {
        let signed_area = closed_parameter_space_area(face)? * planar_metric(face)?;
        let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
        let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
        let u = ku.domain()[0];
        let v = kv.domain()[0];
        let point = face.surface.evaluate(u, v)?;
        let normal = face.surface.normal(u, v)?;
        return Ok(point.sub(reference).dot(normal) * signed_area / 3.0);
    }
    let kind = Integrand::VolumeAbout(reference);
    let gate_started = super::profile::profile_started();
    if let Some(values) = biperiodic_band_integral(face, &[kind])? {
        mass_profile(|p| {
            p.route_biperiodic += 1;
            p.gate_ms += super::profile::elapsed_ms(gate_started);
        });
        return Ok(values[0] / 3.0);
    }
    let integral = if is_untrimmed(face)? {
        mass_profile(|p| {
            p.route_untrimmed += 1;
            p.gate_ms += super::profile::elapsed_ms(gate_started);
        });
        let started = super::profile::profile_started();
        let value = integrate_untrimmed(face, kind)?;
        mass_profile(|p| p.untrimmed_ms += super::profile::elapsed_ms(started));
        value
    } else {
        mass_profile(|p| {
            p.trimmed_faces += 1;
            p.gate_ms += super::profile::elapsed_ms(gate_started);
        });
        integrate_trimmed(face, kind)?
    };
    Ok(integral / 3.0)
}

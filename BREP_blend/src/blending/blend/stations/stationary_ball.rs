//! `BREP_BLEND_STATIONARY_BALL` (default off): a ball that cannot roll.
//!
//! A blend of radius r along an edge where one mate is a concave round of
//! the same radius r (a cylinder of radius r met from inside, a fillet run
//! along a transition already rounded at r): the round's offset at the blend
//! radius collapses to its axis, so the ball's centre lies on that line AND at
//! r from the other mate, which is one point C. The ball stands still while
//! the edge is traversed. It touches the round along the great circle of the
//! sphere square to the axis and the other mate at one point W, and the blend
//! is the spherical triangle between them. The march cannot solve it: a
//! collapsed offset has no normal, so its centre wanders (on the benchy's deck
//! transition an out-and-back rail 0.358 long whose ends miss by 1.6e-7).
//!
//! The stations here are exact: the centre C at every station, the round's
//! contact C + r·(the round's radial direction behind the edge point), the
//! other contact W (a rail with no length), and each section the sphere's
//! great-circle arc between them.
use super::*;

/// The switch, default off.
pub(in crate::blend) fn enabled() -> bool {
    switch!("BREP_BLEND_STATIONARY_BALL", false)
}

fn unit_normal(surface: &NurbsSurface, point: Vec3) -> Option<(Vec3, Vec3)> {
    let foot = crate::project_point_to_surface(surface, point).ok()?;
    let n = raw_normal(surface, foot.u, foot.v).ok()?.normalized().ok()?;
    Some((foot.point, n))
}


/// Every station's centre the same point, bit for bit: a still ball's stripe
/// (only [`construct`] builds one; a marched centre never repeats exactly).
pub(in crate::blend) fn is_still(stations: &[Station]) -> bool {
    let bits = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
    stations.len() >= 2 && stations.iter().all(|station| bits(station.center) == bits(stations[0].center))
}

/// The row window a still ball's stripe is read over when its vertex window
/// is empty: a still ball is the same ball at both its ends, so both end
/// sections can land on one edge parameter. Its stations are exact over the
/// whole edge, so they are read there (the overshoot runs off the round's
/// carrier). None for any other stripe.
pub(in crate::blend) fn reading_window(stations: &[Station], station_ts: &[f64], parameters: &[f64], edge: &EdgeRecord) -> Option<[f64; 2]> {
    if !enabled() || !is_still(stations) || station_ts.len() != parameters.len() {
        return None;
    }
    let first = station_ts.iter().position(|&t| t >= edge.t0)?;
    let last = station_ts.iter().rposition(|&t| t <= edge.t1)?;
    (first < last).then(|| crate::blend::edge::shipped_window(parameters, parameters[first], parameters[last])).flatten()
}

/// The stationary ball's stations, their vertex indices and edge parameters;
/// None where the edge is not that case (then the blend marches).
pub(in crate::blend) fn construct(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius: f64,
    overshoot: f64,
    ends: [f64; 2],
    station_count: usize,
) -> Option<(Vec<Station>, [usize; 2], Vec<f64>)> {
    if !enabled() || !(radius.is_finite() && radius > 0.0) || first.rho.abs() != radius || second.rho.abs() != radius {
        return None;
    }
    let bar = 1e-9 * (1.0 + radius);
    let span = edge.t1 - edge.t0;
    let at = |t: f64| edge.curve.evaluate(t).ok();
    // Which mate's offset collapses to a line: the offset of the mate's
    // surface (read over its whole domain, so a round whose edge is a planar
    // arc, every edge point's offset one point, still shows its axis) lies on
    // one line, along which it moves; and the edge's own offsets lie on it.
    let collapsed = |mate: &BlendMate| -> Option<(Vec3, Vec3)> {
        let surface = &mate.face.surface;
        let [u0, u1] = surface.domain_u().ok()?;
        let [v0, v1] = surface.domain_v().ok()?;
        let mut offsets = Vec::with_capacity(25);
        for i in 0..=4 {
            for j in 0..=4 {
                let (u, v) = (u0 + (u1 - u0) * i as f64 / 4.0, v0 + (v1 - v0) * j as f64 / 4.0);
                let p = surface.evaluate(u, v).ok()?;
                let n = raw_normal(surface, u, v).ok()?.normalized().ok()?;
                offsets.push(p.add(n.scale(mate.rho)));
            }
        }
        for k in 0..=8 {
            let p = at(edge.t0 + span * k as f64 / 8.0)?;
            let (foot, n) = unit_normal(surface, p)?;
            offsets.push(foot.add(n.scale(mate.rho)));
        }
        let a = offsets[0];
        let far = offsets.iter().copied().max_by(|x, y| x.sub(a).length().total_cmp(&y.sub(a).length()))?;
        if !(far.sub(a).length() > 1e3 * bar) {
            return None;
        }
        let direction = far.sub(a).normalized().ok()?;
        let off_line = |q: Vec3| { let d = q.sub(a); d.sub(direction.scale(d.dot(direction))).length() };
        offsets.iter().all(|&q| off_line(q) <= bar).then_some((a, direction))
    };
    let (round, other, (origin, axis)) = if let Some(line) = collapsed(first) {
        (first, second, line)
    } else if let Some(line) = collapsed(second) {
        (second, first, line)
    } else {
        return None;
    };
    // C on the axis at r from the other mate: the other mate's offset foot
    // of C is C itself. Secant on the axis parameter s of the residual
    // (C(s) - (foot + rho n)) · axis.
    let residual = |s: f64| -> Option<f64> {
        let c = origin.add(axis.scale(s));
        let (foot, n) = unit_normal(&other.face.surface, c)?;
        Some(c.sub(foot.add(n.scale(other.rho))).dot(axis))
    };
    // Seeded at the edge (the axis point behind its middle), so the root is
    // the ball at this edge, not another tangent ball further along the axis.
    let middle = at(edge.t0 + 0.5 * span)?.sub(origin).dot(axis);
    let (mut s0, mut s1) = (middle, middle + radius * 0.1);
    let (mut f0, mut f1) = (residual(s0)?, residual(s1)?);
    for _ in 0..60 {
        if f1.abs() <= 1e-14 * (1.0 + radius) || (f1 - f0).abs() <= 1e-300 {
            break;
        }
        let s2 = s1 - f1 * (s1 - s0) / (f1 - f0);
        (s0, f0) = (s1, f1);
        s1 = s2;
        f1 = residual(s1)?;
    }
    let centre = origin.add(axis.scale(s1));
    let (wall_foot, wall_normal) = unit_normal(&other.face.surface, centre)?;
    if !(centre.sub(wall_foot.add(wall_normal.scale(other.rho))).length() <= bar) {
        return None;
    }
    // The ball stands at this edge: every edge point within 2r of its
    // centre (a fillet's edge lies in the corner the ball sits in).
    for k in 0..=8 {
        if !(at(edge.t0 + span * k as f64 / 8.0)?.sub(centre).length() <= 2.0 * radius) {
            return None;
        }
    }
    let w = wall_foot;
    let w_uv = { let f = crate::project_point_to_surface(&other.face.surface, w).ok()?; [f.u, f.v] };
    // As many stations as the network's rung asks for, so its refinement
    // ladder refines this stripe as it does a marched one.
    let (station_ts, vertex_indices) = crate::blend::edge::open_station_parameters(edge, overshoot, ends, station_count.max(16), [false, false]);
    if station_ts.len() < 3 || station_ts.windows(2).any(|p| !(p[1] > p[0])) {
        return None;
    }
    // The round's radial direction behind an edge point (from the axis to the
    // point, square to the axis).
    let radial_at = |t: f64| -> Option<Vec3> {
        let d = at(t)?.sub(origin);
        d.sub(axis.scale(d.dot(axis))).normalized().ok()
    };
    // The contact's angle about the axis, read from the edge point itself (the
    // edge parameter is not an angle: a rational arc runs unevenly), unwrapped
    // along the edge (a round of more than a half turn is not its
    // complementary arc). Past the ends, for the overshoot where the edge curve
    // is clamped, the angle runs on at the edge's mean rate.
    const SAMPLES: usize = 256;
    let r0 = radial_at(edge.t0)?;
    let side = axis.cross(r0);
    let raw_angle = |t: f64| -> Option<f64> { let r = radial_at(t)?; Some(r.dot(side).atan2(r.dot(r0))) };
    let mut unwrapped = Vec::with_capacity(SAMPLES + 1);
    unwrapped.push(0.0f64);
    for k in 1..=SAMPLES {
        let angle = raw_angle(edge.t0 + span * k as f64 / SAMPLES as f64)?;
        let previous = unwrapped[k - 1];
        unwrapped.push(angle + std::f64::consts::TAU * ((previous - angle) / std::f64::consts::TAU).round());
    }
    let turn = unwrapped[SAMPLES];
    if !(turn.abs() > 1e-9) || unwrapped.windows(2).any(|p| !((p[1] - p[0]) * turn > 0.0)) {
        return None;
    }
    let step = span / SAMPLES as f64;
    let rate = turn / span;
    let angle_at = |t: f64| -> Option<f64> {
        if t <= edge.t0 {
            return Some(rate * (t - edge.t0));
        }
        if t >= edge.t1 {
            return Some(turn + rate * (t - edge.t1));
        }
        let near = unwrapped[(((t - edge.t0) / step).round() as usize).min(SAMPLES)];
        let angle = raw_angle(t)?;
        Some(angle + std::f64::consts::TAU * ((near - angle) / std::f64::consts::TAU).round())
    };
    let contact_bar = 1e-6 * (1.0 + radius);
    let mut stations = Vec::with_capacity(station_ts.len());
    for &t in &station_ts {
        let angle = angle_at(t)?;
        let radial = r0.scale(angle.cos()).add(side.scale(angle.sin()));
        let p_round = centre.add(radial.scale(radius));
        // The contact is on the round itself, and the round's offset there is
        // the centre (a carrier that is the axis's round only at the sampled
        // points, a bump between them, falls back to the march).
        let round_uv = {
            let f = crate::project_point_to_surface(&round.face.surface, p_round).ok()?;
            let n = raw_normal(&round.face.surface, f.u, f.v).ok()?.normalized().ok()?;
            let inside = t >= edge.t0 && t <= edge.t1;
            if inside && !(f.point.sub(p_round).length() <= contact_bar && f.point.add(n.scale(round.rho)).sub(centre).length() <= contact_bar) {
                return None;
            }
            [f.u, f.v]
        };
        let (d1, d2) = (p_round.sub(centre).scale(1.0 / radius), w.sub(centre).scale(1.0 / radius));
        let cosine = d1.dot(d2);
        if !(cosine > -1.0 + 1e-9) {
            return None;
        }
        let weight = ((1.0 + cosine) * 0.5).sqrt();
        let apex = centre.add(d1.add(d2).scale(radius / (1.0 + cosine)));
        let (p1, uv1, p2, uv2) = if std::ptr::eq(round, first) { (p_round, round_uv, w, w_uv) } else { (w, w_uv, p_round, round_uv) };
        stations.push(Station { uv1, uv2, p1, p2, center: centre, weight, apex });
    }
    // Between the stations too: the round's offset is the centre at three
    // points inside every knot span of the round that each interval's contact
    // crosses (a carrier that bends away between stations would meet the
    // sphere in a crease, not tangentially; within one polynomial span it
    // bends smoothly).
    let on_round = |t: f64| -> Option<bool> {
        let angle = angle_at(t)?;
        let p = centre.add(r0.scale(angle.cos()).add(side.scale(angle.sin())).scale(radius));
        let f = crate::project_point_to_surface(&round.face.surface, p).ok()?;
        let n = raw_normal(&round.face.surface, f.u, f.v).ok()?.normalized().ok()?;
        Some(f.point.sub(p).length() <= contact_bar && f.point.add(n.scale(round.rho)).sub(centre).length() <= contact_bar)
    };
    let round_surface = &round.face.surface;
    let crossed = |knots: &[f64], a: f64, b: f64| -> usize {
        let (low, high) = (a.min(b), a.max(b));
        let mut distinct: Vec<f64> = knots.iter().copied().filter(|&k| k > low && k < high).collect();
        distinct.dedup();
        distinct.len()
    };
    let round_uv_of = |station: &Station| if std::ptr::eq(round, first) { station.uv1 } else { station.uv2 };
    for (pair, ends) in station_ts.windows(2).zip(stations.windows(2)) {
        let (a, b) = (round_uv_of(&ends[0]), round_uv_of(&ends[1]));
        let spans = 1 + crossed(&round_surface.knots_u, a[0], b[0]) + crossed(&round_surface.knots_v, a[1], b[1]);
        let probes = 4 * spans;
        for k in 1..probes {
            let t = pair[0] + (pair[1] - pair[0]) * k as f64 / probes as f64;
            if t > edge.t0 && t < edge.t1 && !on_round(t)? {
                return None;
            }
        }
    }
    crate::blend::record_count("count.network.stationary_ball", 1);
    Some((stations, vertex_indices, station_ts))
}

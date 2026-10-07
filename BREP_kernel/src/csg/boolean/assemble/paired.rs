//! Whole-loop construction for supported ruled analytic carriers. Plane
//! sections are algebraic paired graphs; coaxial revolution sections are
//! represented isocurves. Inherited interiors are never station-refitted.
use super::{face_at_mut, image, image_distance, uv_of, ReplacedTrim};
use crate::{AnalyticSurface, BrepSolid, FaceRecord, NurbsCurve, NurbsSurface, Vec3, Vec4};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

fn restrict(c: NurbsCurve, a: f64, b: f64) -> Option<NurbsCurve> {
    super::super::image::restricted_curve(&c, a, b)
}

fn parameter_line(a: Vec3, b: Vec3, range: [f64; 2]) -> Option<NurbsCurve> {
    NurbsCurve::new(
        1,
        vec![range[0], range[0], range[1], range[1]],
        vec![Vec4::from_point(a, 1.0), Vec4::from_point(b, 1.0)],
    )
    .ok()
}

/// Exact coaxial ruled-revolution crossing. The radius and axial station are
/// linear in the generatrix parameter; equality solves a single linear root.
/// Opposite axes are handled by signed axial coordinates in the first frame.
fn coaxial_v(first: &NurbsSurface, second: &NurbsSurface) -> Option<f64> {
    let (
        Some(AnalyticSurface::RuledRevolution {
            frame: a,
            rho0: r0,
            rho1: r1,
            height: h,
        }),
        Some(AnalyticSurface::RuledRevolution {
            frame: b,
            rho0: s0,
            rho1: s1,
            height: k,
        }),
    ) = (first.analytic(), second.analytic())
    else {
        return None;
    };
    let dot = a.axis.dot(b.axis);
    let shift = b.origin.sub(a.origin);
    let scale =
        1.0 + shift.length() + h.abs() + k.abs() + r0.abs() + r1.abs() + s0.abs() + s1.abs();
    if (dot.abs() - 1.0).abs() > 4096.0 * f64::EPSILON
        || shift.cross(a.axis).length() > 4096.0 * f64::EPSILON * scale
        || *k == 0.0
    {
        return None;
    }
    let z = shift.dot(a.axis);
    let slope = (s1 - s0) / (k * dot);
    let denominator = (r1 - r0) - slope * h;
    if denominator.abs() <= 4096.0 * f64::EPSILON * scale {
        return None;
    }
    let fraction = (s0 - slope * z - r0) / denominator;
    let other = (h * fraction - z) / (k * dot);
    if !(0.0..=1.0).contains(&fraction) || !(0.0..=1.0).contains(&other) {
        return None;
    }
    let [v0, v1] = first.domain_v().ok()?;
    Some(v0 + (v1 - v0) * fraction)
}

/// Bounded Bernstein root isolation for the plane-parallel ruling arm.
/// The final iso curve is checked against the actual partner image; neither
/// averaged inverse coordinates nor an approximate plane classification
/// authorizes a generator.
fn ruling_roots(surface: &NurbsSurface, plane: &NurbsSurface) -> Option<Vec<f64>> {
    let [v0, _] = surface.domain_v().ok()?;
    curve_plane_roots(surface.iso_curve_v(v0).ok()?, plane)
}
fn curve_plane_roots(base: NurbsCurve, plane: &NurbsSurface) -> Option<Vec<f64>> {
    let [pu, _] = plane.domain_u().ok()?;
    let [pv, _] = plane.domain_v().ok()?;
    let d = plane.derivatives(pu, pv, 1).ok()?;
    let normal = d[1][0].cross(d[0][1]).normalized().ok()?;
    let offset = normal.dot(d[0][0]);
    let value = |h: &Vec4| normal.x * h.x + normal.y * h.y + normal.z * h.z - offset * h.w;
    let mut work: Vec<_> = super::super::image::bezier_spans(&base)?
        .into_iter()
        .map(|c| {
            let [a, b] = c.domain().expect("Bezier domain");
            (a, b, c.control_points, 24usize)
        })
        .collect();
    let mut roots = Vec::new();
    let mut budget = 512;
    while let Some((a, b, c, depth)) = work.pop() {
        if budget == 0 {
            return None;
        }
        budget -= 1;
        let f: Vec<_> = c.iter().map(value).collect();
        let scale = c
            .iter()
            .map(|h| h.x.abs() + h.y.abs() + h.z.abs() + offset.abs() * h.w.abs())
            .fold(1.0_f64, f64::max);
        let rounding = 64.0 * f64::EPSILON * scale;
        if f[0].abs() <= rounding {
            roots.push(a);
        }
        if f[f.len() - 1].abs() <= rounding {
            roots.push(b);
        }
        if f.iter().all(|x| *x > 0.0) || f.iter().all(|x| *x < 0.0) {
            continue;
        }
        let increasing = f.windows(2).all(|p| p[1] >= p[0]);
        let decreasing = f.windows(2).all(|p| p[1] <= p[0]);
        if increasing || decreasing {
            if f[0] * f[f.len() - 1] < 0.0 {
                let (mut lo, mut hi) = (a, b);
                let sign = f[0].is_sign_positive();
                for _ in 0..64 {
                    let mid = (lo + hi) * 0.5;
                    let h = base.evaluate(mid).ok()?;
                    let z = h.dot(normal) - offset;
                    if z.is_sign_positive() == sign {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                roots.push((lo + hi) * 0.5);
            }
            continue;
        }
        if depth == 0 {
            return None;
        }
        let mid = (a + b) * 0.5;
        let left = super::super::image::restricted_controls(&base, a, mid)?;
        let right = super::super::image::restricted_controls(&base, mid, b)?;
        work.push((a, mid, left, depth - 1));
        work.push((mid, b, right, depth - 1));
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup();
    (roots.len() <= 32).then_some(roots)
}

fn simple_boundary(
    surface: &NurbsSurface,
    other: &NurbsSurface,
    old: &NurbsCurve,
    start: Vec3,
    end: Vec3,
    bar: f64,
) -> Option<(NurbsCurve, NurbsCurve)> {
    let [a, b] = old.domain().ok()?;
    let ua = uv_of(surface, start, old.evaluate(a).ok()?).ok()?;
    let ub = uv_of(surface, end, old.evaluate(b).ok()?).ok()?;
    if other.is_affine().ok()? && (ua.x - ub.x).abs() > 1e-12 {
        if let Some(pair) = crate::imprint::plane_section_graph_range_bounded(
            other,
            surface,
            [ua.x.min(ub.x), ua.x.max(ub.x)],
            bar * 0.1,
            bar * 0.1,
        ) {
            if super::super::edges::plane_supports_curve(other, &pair.curve, bar * 0.1) {
                return if ua.x < ub.x {
                    Some((pair.pcurve, pair.curve))
                } else {
                    Some((pair.pcurve.reversed().ok()?, pair.curve.reversed().ok()?))
                };
            }
        }
    }
    // A plane can cut an interior represented parallel (e.g. a revolution
    // axial section). Isolate the actual homogeneous plane-form roots on a
    // transverse isocurve; endpoint inverses only rank branches. A complete
    // image check is required because a root on one meridian alone does not
    // imply that the entire parallel lies in this plane.
    if other.is_affine().ok()? && (ua.x - ub.x).abs() > 1e-12 {
        let [u0, _] = surface.domain_u().ok()?;
        let hint = (ua.y + ub.y) * 0.5;
        let mut roots = surface
            .iso_curve_u(u0)
            .ok()
            .and_then(|base| curve_plane_roots(base, other))
            .unwrap_or_default();
        roots.sort_by(|a, b| (a - hint).abs().total_cmp(&(b - hint).abs()));
        for v in roots {
            let Some(curve) =
                restrict(surface.iso_curve_v(v).ok()?, ua.x.min(ub.x), ua.x.max(ub.x))
            else {
                continue;
            };
            let [a, b] = curve.domain().ok()?;
            let (ca, cb) = if ua.x < ub.x {
                (curve.evaluate(a).ok()?, curve.evaluate(b).ok()?)
            } else {
                (curve.evaluate(b).ok()?, curve.evaluate(a).ok()?)
            };
            if ca.sub(start).length() > bar * 0.5
                || cb.sub(end).length() > bar * 0.5
                || !super::super::edges::plane_supports_curve(other, &curve, bar * 0.1)
            {
                continue;
            }
            let pc = parameter_line(Vec3::new(a, v, 0.0), Vec3::new(b, v, 0.0), [a, b])?;
            return if ua.x < ub.x {
                Some((pc, curve))
            } else {
                Some((pc.reversed().ok()?, curve.reversed().ok()?))
            };
        }
    }
    // Two carriers may share an algebraic boundary even when the neighboring
    // surface is not analytically recognized. Certify complete rational iso
    // curves, including their weights and parameterization, before using it.
    let [v0, v1] = surface.domain_v().ok()?;
    for v in [v0, v1] {
        let own = restrict(surface.iso_curve_v(v).ok()?, ua.x.min(ub.x), ua.x.max(ub.x));
        let Some(curve) = own else {
            continue;
        };
        let [a, b] = curve.domain().ok()?;
        let (ca, cb) = if ua.x < ub.x {
            (curve.evaluate(a).ok()?, curve.evaluate(b).ok()?)
        } else {
            (curve.evaluate(b).ok()?, curve.evaluate(a).ok()?)
        };
        if ca.sub(start).length() > bar * 0.5 || cb.sub(end).length() > bar * 0.5 {
            continue;
        }
        let [ou0, ou1] = other.domain_u().ok()?;
        let [ov0, ov1] = other.domain_v().ok()?;
        let neighbors = [
            other.iso_curve_u(ou0),
            other.iso_curve_u(ou1),
            other.iso_curve_v(ov0),
            other.iso_curve_v(ov1),
        ];
        if other.is_affine().ok()?
            && super::super::edges::plane_supports_curve(other, &curve, bar * 0.1)
        {
            let pc = parameter_line(Vec3::new(a, v, 0.0), Vec3::new(b, v, 0.0), [a, b])?;
            return if ua.x < ub.x {
                Some((pc, curve))
            } else {
                Some((pc.reversed().ok()?, curve.reversed().ok()?))
            };
        }
        if neighbors
            .into_iter()
            .filter_map(Result::ok)
            .any(|neighbor| {
                restrict(neighbor, ua.x.min(ub.x), ua.x.max(ub.x))
                    .and_then(|n| super::super::image::difference_bound(&curve, &n))
                    .is_some_and(|d| d <= bar * 0.1)
            })
        {
            let pc = parameter_line(
                Vec3::new(ua.x.min(ub.x), v, 0.0),
                Vec3::new(ua.x.max(ub.x), v, 0.0),
                curve.domain().ok()?,
            )?;
            return if ua.x < ub.x {
                Some((pc, curve))
            } else {
                Some((pc.reversed().ok()?, curve.reversed().ok()?))
            };
        }
    }
    let mut isolines = surface.knots_u.clone();
    isolines.sort_by(f64::total_cmp);
    isolines.dedup();
    if isolines.len() > 32 {
        return None;
    }
    for u in isolines {
        let Some(curve) = restrict(surface.iso_curve_u(u).ok()?, ua.y.min(ub.y), ua.y.max(ub.y))
        else {
            continue;
        };
        let [a, b] = curve.domain().ok()?;
        let (ca, cb) = if ua.y < ub.y {
            (curve.evaluate(a).ok()?, curve.evaluate(b).ok()?)
        } else {
            (curve.evaluate(b).ok()?, curve.evaluate(a).ok()?)
        };
        if ca.sub(start).length() > bar * 0.5 || cb.sub(end).length() > bar * 0.5 {
            continue;
        }
        let mut other_u = other.knots_u.clone();
        other_u.sort_by(f64::total_cmp);
        other_u.dedup();
        let mut other_v = other.knots_v.clone();
        other_v.sort_by(f64::total_cmp);
        other_v.dedup();
        if other_u.len() + other_v.len() > 64 {
            return None;
        }
        let neighbors = other_u
            .into_iter()
            .filter_map(|u| other.iso_curve_u(u).ok())
            .chain(
                other_v
                    .into_iter()
                    .filter_map(|v| other.iso_curve_v(v).ok()),
            );
        if neighbors
            .filter_map(|n| restrict(n, ua.y.min(ub.y), ua.y.max(ub.y)))
            .any(|n| {
                super::super::image::difference_bound(&curve, &n).is_some_and(|d| d <= bar * 0.1)
            })
        {
            let pc = parameter_line(
                Vec3::new(u, ua.y.min(ub.y), 0.0),
                Vec3::new(u, ua.y.max(ub.y), 0.0),
                curve.domain().ok()?,
            )?;
            return if ua.y < ub.y {
                Some((pc, curve))
            } else {
                Some((pc.reversed().ok()?, curve.reversed().ok()?))
            };
        }
    }
    let fixed = if other.is_affine().ok()? {
        None
    } else {
        coaxial_v(surface, other)
    };
    if let Some(v) = fixed {
        let curve = restrict(surface.iso_curve_v(v).ok()?, ua.x.min(ub.x), ua.x.max(ub.x))?;
        let pc = parameter_line(
            Vec3::new(ua.x.min(ub.x), v, 0.0),
            Vec3::new(ua.x.max(ub.x), v, 0.0),
            curve.domain().ok()?,
        )?;
        return if ua.x < ub.x {
            Some((pc, curve))
        } else {
            Some((pc.reversed().ok()?, curve.reversed().ok()?))
        };
    }
    // A plane meridian can have constant u. Check the entire rational image
    // against the affine plane form; positive homogeneous weights bound it.
    if other.is_affine().ok()? {
        let hint = (ua.x + ub.x) * 0.5;
        let mut roots = ruling_roots(surface, other)?;
        roots.sort_by(|a, b| (a - hint).abs().total_cmp(&(b - hint).abs()));
        let u = *roots.first()?;
        let curve = restrict(surface.iso_curve_u(u).ok()?, ua.y.min(ub.y), ua.y.max(ub.y))?;
        let [pu, _] = other.domain_u().ok()?;
        let [pv, _] = other.domain_v().ok()?;
        let d = other.derivatives(pu, pv, 1).ok()?;
        let normal = d[1][0].cross(d[0][1]).normalized().ok()?;
        if curve.control_points.iter().any(|h| {
            h.w <= 0.0
                || h.point()
                    .ok()
                    .is_none_or(|p| p.sub(d[0][0]).dot(normal).abs() > bar * 0.1)
        }) {
            return None;
        }
        if !super::super::edges::plane_supports_curve(other, &curve, bar * 0.1) {
            return None;
        }
        let pc = parameter_line(
            Vec3::new(u, ua.y.min(ub.y), 0.0),
            Vec3::new(u, ua.y.max(ub.y), 0.0),
            curve.domain().ok()?,
        )?;
        return if ua.y < ub.y {
            Some((pc, curve))
        } else {
            Some((pc.reversed().ok()?, curve.reversed().ok()?))
        };
    }
    None
}

struct Boundary {
    own: NurbsCurve,
    curve: NurbsCurve,
    partner: NurbsCurve,
}

fn remap(mut pc: NurbsCurve, range: [f64; 2]) -> Option<NurbsCurve> {
    let [a, b] = pc.domain().ok()?;
    for k in &mut pc.knots {
        *k = range[0] + (*k - a) / (b - a) * (range[1] - range[0]);
    }
    let p = pc.degree;
    pc.knots[..=p].fill(range[0]);
    let n = pc.knots.len();
    pc.knots[n - p - 1..].fill(range[1]);
    Some(pc)
}

/// Fit only the newly constructed carrier intersection, never inherited
/// off-carrier stations. Affine images receive a complete rational bound;
/// analytic images retain the analytic fitter's independent span witnesses.
fn companion(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    old: &NurbsCurve,
    bar: f64,
) -> Option<NurbsCurve> {
    let range = curve.domain().ok()?;
    let mut isolines = Vec::new();
    for (along_u, knots) in [(true, &surface.knots_u), (false, &surface.knots_v)] {
        let mut values = knots.clone();
        values.sort_by(f64::total_cmp);
        values.dedup();
        if values.len() > 64 {
            return None;
        }
        for value in values {
            let iso = if along_u {
                surface.iso_curve_u(value)
            } else {
                surface.iso_curve_v(value)
            };
            if let Ok(iso) = iso {
                isolines.push((along_u, value, iso));
            }
        }
    }
    for (along_u, value, iso) in isolines {
        let Some(iso) = restrict(iso, range[0], range[1]) else {
            continue;
        };
        for reversed in [false, true] {
            let candidate = if reversed {
                iso.reversed().ok()?
            } else {
                iso.clone()
            };
            if super::super::image::difference_bound(curve, &candidate)
                .is_some_and(|d| d <= bar * 0.1)
            {
                let (a, b) = if reversed {
                    (range[1], range[0])
                } else {
                    (range[0], range[1])
                };
                return parameter_line(
                    if along_u {
                        Vec3::new(value, a, 0.0)
                    } else {
                        Vec3::new(a, value, 0.0)
                    },
                    if along_u {
                        Vec3::new(value, b, 0.0)
                    } else {
                        Vec3::new(b, value, 0.0)
                    },
                    range,
                );
            }
        }
    }
    let mut pc = if surface.is_affine().unwrap_or(false) {
        super::super::image::affine_inverse(surface, curve, bar * 0.1)?
    } else {
        let stations = |t: f64| curve.evaluate(range[0] + t * (range[1] - range[0]));
        remap(
            crate::pcurve::analytic_station_pcurve(surface, &stations, bar * 0.01).ok()?,
            range,
        )?
    };
    let closed = surface.closed_directions().ok()?;
    let d = old.domain().ok()?;
    let guide = old.evaluate((d[0] + d[1]) * 0.5).ok()?;
    let center = pc.evaluate((range[0] + range[1]) * 0.5).ok()?;
    let u = surface.domain_u().ok()?;
    let v = surface.domain_v().ok()?;
    let dx = if closed.0 {
        ((guide.x - center.x) / (u[1] - u[0])).round() * (u[1] - u[0])
    } else {
        0.0
    };
    let dy = if closed.1 {
        ((guide.y - center.y) / (v[1] - v[0])).round() * (v[1] - v[0])
    } else {
        0.0
    };
    for h in &mut pc.control_points {
        h.x += dx * h.w;
        h.y += dy * h.w;
    }
    Some(pc)
}

fn boundary(
    surface: &NurbsSurface,
    other: &NurbsSurface,
    old: &NurbsCurve,
    other_old: &NurbsCurve,
    start: Vec3,
    end: Vec3,
    bar: f64,
    allowed: f64,
) -> Option<Boundary> {
    if let Some((own, curve)) = simple_boundary(surface, other, old, start, end, bar) {
        let partner = companion(other, &curve, other_old, bar).or_else(|| {
            if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                eprintln!("paired: exact section companion unsupported");
            }
            None
        })?;
        return Some(Boundary {
            own,
            curve,
            partner,
        });
    }
    let [a, b] = old.domain().ok()?;
    let ua = uv_of(surface, start, old.evaluate(a).ok()?).ok()?;
    let ub = uv_of(surface, end, old.evaluate(b).ok()?).ok()?;
    let pair = crate::imprint::construct_ruled_torus(
        surface,
        other,
        [ua.x, ub.x],
        old,
        allowed,
        (bar * 0.01).min(1e-8),
    )
    .map_err(|e| {
        if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
            eprintln!("paired: torus helper {e}");
        }
        e
    })
    .ok()?;
    if !pair.maximum_image_residual.is_finite()
        || pair.maximum_image_residual > bar * 0.1
        || !pair.maximum_displacement.is_finite()
        || pair.maximum_displacement > allowed
        || pair.segments == 0
    {
        return None;
    }
    Some(Boundary {
        own: pair.own_pcurve,
        curve: pair.boundary,
        partner: pair.partner_pcurve,
    })
}

fn material_not_worse(old: &FaceRecord, new: &FaceRecord, bar: f64) -> bool {
    let Some((area, volume, ae, ve)) = material_reference(new) else {
        return false;
    };
    let (Ok(oa), Ok(na), Ok(ov), Ok(nv)) = (
        crate::face_area(old),
        crate::face_area(new),
        crate::face_volume_contribution(old),
        crate::face_volume_contribution(new),
    ) else {
        return false;
    };
    let aa = 1e-11 * area.abs().max(na.abs()) + bar * bar + ae;
    let va = 1e-11 * volume.abs().max(nv.abs()) + bar * bar + ve;
    na.is_finite()
        && nv.is_finite()
        && (na - area).abs() <= aa
        && (nv - volume).abs() <= va
        && (na - area).abs() <= (oa - area).abs() + aa
        && (nv - volume).abs() <= (ov - volume).abs() + va
}

/// Improve integration partitioning by exact knot insertion only. Geometry
/// is certified unchanged with a complete rational difference bound; no
/// interior station moves and no material tolerance is enlarged.
/// Boehm insertion changes representation, never the trim locus. The full
/// positive rational difference is checked, including all native knot spans.
fn exact_midpoint_partition(original: &NurbsCurve, allowance: f64) -> Option<NurbsCurve> {
    let [a, b] = original.domain().ok()?;
    let mut breaks = vec![a, b];
    breaks.extend(original.knots.iter().copied().filter(|t| *t > a && *t < b));
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    if breaks.len() > 2048 {
        return None;
    }
    let mut curve = original.clone();
    for span in breaks.windows(2) {
        let mid = span[0] + (span[1] - span[0]) * 0.5;
        if mid <= span[0] || mid >= span[1] {
            return None;
        }
        curve = curve.insert_knot(mid, curve.degree).ok()?;
    }
    (super::super::image::difference_bound(original, &curve)? <= allowance).then_some(curve)
}

fn refined_material_candidate(old: &FaceRecord, new: &FaceRecord, bar: f64) -> Option<FaceRecord> {
    let mut face = new.clone();
    for _ in 0..3 {
        for c in face.loops.iter_mut().flat_map(|l| &mut l.coedges) {
            let original = c.pcurve.clone();
            let [a, b] = original.domain().ok()?;
            let mut breaks = vec![a, b];
            breaks.extend(original.knots.iter().copied().filter(|t| *t > a && *t < b));
            breaks.sort_by(f64::total_cmp);
            breaks.dedup();
            if breaks.len() > 2048 {
                return None;
            }
            for pair in breaks.windows(2) {
                let mid = pair[0] + (pair[1] - pair[0]) * 0.5;
                if mid <= pair[0] || mid >= pair[1] {
                    return None;
                }
                c.pcurve = c.pcurve.insert_knot(mid, c.pcurve.degree).ok()?;
            }
            if super::super::image::difference_bound(&original, &c.pcurve)? > bar * 0.01 {
                return None;
            }
        }
        if material_not_worse(old, &face, bar) {
            return Some(face);
        }
    }
    None
}

/// Independent Green primitive for a general native tensor carrier. Split
/// the inner integral at every native V knot and extension boundary; decline
/// exhausted quadrature rather than using the last approximation.
/// The mass-property domain includes straight UV links between successive
/// trim endpoints. Include those links in the independent Green integral too;
/// this does not modify topology or authorize a physical joint mismatch.
fn material_pcurves(face: &FaceRecord) -> Option<Vec<NurbsCurve>> {
    let mut curves = Vec::new();
    for l in &face.loops {
        for (i, c) in l.coedges.iter().enumerate() {
            curves.push(c.pcurve.clone());
            let next = &l.coedges[(i + 1) % l.coedges.len()].pcurve;
            let end = c.pcurve.evaluate(c.pcurve.domain().ok()?[1]).ok()?;
            let start = next.evaluate(next.domain().ok()?[0]).ok()?;
            if end.x != start.x || end.y != start.y {
                curves.push(parameter_line(end, start, [0.0, 1.0])?);
            }
        }
    }
    Some(curves)
}

/// Closed meridian primitives for the helper's fully proved circular
/// profile. Open-V excursions integrate the ACTUAL native tangent line;
/// extending the ideal circle would give the wrong material at edge70.
/// Positive quartic identity bounds the radial reconstruction error AND its
/// derivative. Differentiating Q/W² accounts for rational denominator drift.
fn circle_reconstruction_error(
    curve: &NurbsCurve,
    center: Vec3,
    radius: f64,
) -> Option<(f64, f64, f64)> {
    let mut errors = (0.0_f64, 0.0_f64, 0.0_f64);
    for span in super::super::image::bezier_spans(curve)? {
        if span.degree != 2 || span.control_points.len() != 3 {
            return None;
        }
        let [a, b] = span.domain().ok()?;
        let maxw = span
            .control_points
            .iter()
            .map(|h| h.w)
            .fold(0.0_f64, f64::max);
        let h: Vec<_> = span
            .control_points
            .iter()
            .map(|p| {
                let w = p.w / maxw;
                Some((p.point().ok()?.sub(center).scale(w), w))
            })
            .collect::<Option<_>>()?;
        let w = h.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        if w <= 0.0 || !w.is_finite() || radius <= 0.0 {
            return None;
        }
        let mut q = [0.0_f64; 5];
        let bin = [1.0, 2.0, 1.0];
        let bin4 = [1.0, 4.0, 6.0, 4.0, 1.0];
        for k in 0..5 {
            for i in 0..3 {
                for j in 0..3 {
                    if i + j == k {
                        q[k] += bin[i] * bin[j] / bin4[k]
                            * (h[i].0.dot(h[j].0) - radius * radius * h[i].1 * h[j].1);
                    }
                }
            }
        }
        let maximum = q.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        let radial = maximum / (w * w * radius);
        let lower = radius - radial;
        if lower <= 0.0 {
            return None;
        }
        let dw = h
            .windows(2)
            .map(|p| 2.0 * (p[1].1 - p[0].1).abs() / (b - a))
            .fold(0.0_f64, f64::max);
        let dn = h
            .windows(2)
            .map(|p| p[1].0.sub(p[0].0).length() * 2.0 / (b - a))
            .fold(0.0_f64, f64::max);
        let n = h.iter().map(|p| p.0.length()).fold(0.0_f64, f64::max);
        let speed = (dn + dw * n / w) / w;
        let dq = q
            .windows(2)
            .map(|v| 4.0 * (v[1] - v[0]).abs() / (b - a))
            .fold(0.0_f64, f64::max);
        let radial_derivative = (dq / (w * w) + 2.0 * maximum * dw / (w * w * w)) / (2.0 * lower);
        let derivative = radial_derivative + radial * speed / lower;
        errors.0 = errors.0.max(radial);
        errors.1 = errors.1.max(derivative);
        errors.2 = errors.2.max(speed);
    }
    [errors.0, errors.1, errors.2]
        .iter()
        .all(|v| v.is_finite())
        .then_some(errors)
}

/// Complete absolute horizontal boundary variation. Monotone rational spans
/// use their endpoint difference; turning spans use the positive denominator
/// and the whole derivative-numerator Bernstein hull. This bounds the Green
/// error even for repeated loops, rather than assuming winding multiplicity 1.
fn u_total_variation(curves: &[NurbsCurve]) -> Option<f64> {
    let mut total = 0.0;
    let mut work = 8191usize;
    for curve in curves {
        for span in super::super::image::bezier_spans(curve)? {
            work = work.checked_sub(1)?;
            let scale = span
                .control_points
                .iter()
                .map(|h| h.w)
                .fold(0.0_f64, f64::max);
            let n: Vec<_> = span.control_points.iter().map(|h| h.x / scale).collect();
            let w: Vec<_> = span.control_points.iter().map(|h| h.w / scale).collect();
            let minimum = w.iter().copied().fold(f64::INFINITY, f64::min);
            if minimum <= 0.0 || !minimum.is_finite() {
                return None;
            }
            let degree = span.degree as f64;
            let dn: Vec<_> = n.windows(2).map(|a| degree * (a[1] - a[0])).collect();
            let dw: Vec<_> = w.windows(2).map(|a| degree * (a[1] - a[0])).collect();
            let numerator: Vec<_> = super::super::image::product(&dn, &w)
                .into_iter()
                .zip(super::super::image::product(&n, &dw))
                .map(|(a, b)| a - b)
                .collect();
            let low = numerator.iter().copied().fold(f64::INFINITY, f64::min);
            let high = numerator.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let arithmetic = 128.0
                * f64::EPSILON
                * degree
                * (1.0 + n.iter().map(|v| v.abs()).fold(0.0_f64, f64::max));
            let variation = if low >= arithmetic || high <= -arithmetic {
                (n[n.len() - 1] / w[w.len() - 1] - n[0] / w[0]).abs()
            } else {
                low.abs().max(high.abs()) / (minimum * minimum)
            };
            total += variation + arithmetic / (minimum * minimum);
        }
    }
    total.is_finite().then_some(total)
}

/// Complete homogeneous tensor comparison with the separable revolution.
/// Recognition supplies a frame, not permission to discard remaining rows.
/// Positive denominator hulls bound position and both first/mixed partials.
fn revolution_net_error(
    sf: &NurbsSurface,
    angular: &NurbsCurve,
    profile: &NurbsCurve,
    frame: &crate::RevolutionFrame,
) -> Option<([f64; 4], [f64; 4])> {
    if sf.degree_u != angular.degree
        || sf.degree_v != profile.degree
        || sf.knots_u != angular.knots
        || sf.knots_v != profile.knots
        || sf.control_points.len() != angular.control_points.len()
        || sf
            .control_points
            .iter()
            .any(|r| r.len() != profile.control_points.len())
    {
        return None;
    }
    let mut model = sf.control_points.clone();
    for (i, a) in angular.control_points.iter().enumerate() {
        let radial = Vec3::new(a.x, a.y, a.z).sub(frame.origin.scale(a.w));
        for (j, b) in profile.control_points.iter().enumerate() {
            let centered = Vec3::new(b.x, b.y, b.z).sub(frame.origin.scale(b.w));
            let n = radial
                .scale(centered.dot(frame.x_axis))
                .add(frame.axis.scale(centered.dot(frame.axis) * a.w));
            model[i][j] = Vec4 {
                x: n.x,
                y: n.y,
                z: n.z,
                w: a.w * b.w,
            };
        }
    }
    let scale = model[0][0].w / sf.control_points[0][0].w;
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }
    let actual: Vec<Vec<_>> = sf
        .control_points
        .iter()
        .map(|r| {
            r.iter()
                .map(|h| {
                    let n = Vec3::new(h.x, h.y, h.z)
                        .sub(frame.origin.scale(h.w))
                        .scale(scale);
                    Vec4 {
                        x: n.x,
                        y: n.y,
                        z: n.z,
                        w: h.w * scale,
                    }
                })
                .collect()
        })
        .collect();
    let diff: Vec<Vec<_>> = actual
        .iter()
        .zip(&model)
        .map(|(a, b)| {
            a.iter()
                .zip(b)
                .map(|(a, b)| Vec4 {
                    x: a.x - b.x,
                    y: a.y - b.y,
                    z: a.z - b.z,
                    w: a.w - b.w,
                })
                .collect()
        })
        .collect();
    let jets = |net: &Vec<Vec<Vec4>>| -> Option<[(f64, f64); 4]> {
        let du = |net: &Vec<Vec<Vec4>>| -> Option<Vec<Vec<Vec4>>> {
            (0..net.len() - 1)
                .map(|i| {
                    let span = sf.knots_u[i + sf.degree_u + 1] - sf.knots_u[i + 1];
                    if span <= 0.0 {
                        return None;
                    }
                    let k = sf.degree_u as f64 / span;
                    Some(
                        net[i]
                            .iter()
                            .zip(&net[i + 1])
                            .map(|(a, b)| Vec4 {
                                x: k * (b.x - a.x),
                                y: k * (b.y - a.y),
                                z: k * (b.z - a.z),
                                w: k * (b.w - a.w),
                            })
                            .collect(),
                    )
                })
                .collect()
        };
        let dv = |net: &Vec<Vec<Vec4>>| -> Option<Vec<Vec<Vec4>>> {
            net.iter()
                .map(|r| {
                    (0..r.len() - 1)
                        .map(|j| {
                            let span = sf.knots_v[j + sf.degree_v + 1] - sf.knots_v[j + 1];
                            if span <= 0.0 {
                                return None;
                            }
                            let k = sf.degree_v as f64 / span;
                            let (a, b) = (r[j], r[j + 1]);
                            Some(Vec4 {
                                x: k * (b.x - a.x),
                                y: k * (b.y - a.y),
                                z: k * (b.z - a.z),
                                w: k * (b.w - a.w),
                            })
                        })
                        .collect()
                })
                .collect()
        };
        let u = du(net)?;
        let v = dv(net)?;
        let uv = dv(&u)?;
        let maximum = |n: &Vec<Vec<Vec4>>| -> Option<(f64, f64)> {
            n.iter()
                .flatten()
                .try_fold((0.0_f64, 0.0_f64), |(a, b), h| {
                    let d = Vec3::new(h.x, h.y, h.z).length();
                    (d.is_finite() && h.w.is_finite()).then_some((a.max(d), b.max(h.w.abs())))
                })
        };
        Some([maximum(net)?, maximum(&u)?, maximum(&v)?, maximum(&uv)?])
    };
    let minimum = |net: &Vec<Vec<Vec4>>| {
        net.iter()
            .flatten()
            .map(|h| h.w)
            .fold(f64::INFINITY, f64::min)
    };
    let (wa, wb) = (minimum(&actual), minimum(&model));
    if wa <= 0.0 || wb <= 0.0 {
        return None;
    }
    let (a, b, d) = (jets(&actual)?, jets(&model)?, jets(&diff)?);
    let mut m = [0.0; 4];
    m[0] = b[0].0 / wb;
    m[1] = (b[1].0 + b[1].1 * m[0]) / wb;
    m[2] = (b[2].0 + b[2].1 * m[0]) / wb;
    m[3] = (b[3].0 + b[1].1 * m[2] + b[2].1 * m[1] + b[3].1 * m[0]) / wb;
    let mut e = [0.0; 4];
    e[0] = (d[0].0 + d[0].1 * m[0]) / wa;
    e[1] = (d[1].0 + d[0].1 * m[1] + d[1].1 * m[0] + a[1].1 * e[0]) / wa;
    e[2] = (d[2].0 + d[0].1 * m[2] + d[2].1 * m[0] + a[2].1 * e[0]) / wa;
    e[3] = (d[3].0
        + d[0].1 * m[3]
        + d[1].1 * m[2]
        + d[2].1 * m[1]
        + d[3].1 * m[0]
        + a[1].1 * e[2]
        + a[2].1 * e[1]
        + a[3].1 * e[0])
        / wa;
    m[0] += frame.origin.length();
    e.iter().chain(&m).all(|v| v.is_finite()).then_some((e, m))
}

fn circular_material_reference(face: &FaceRecord) -> Option<(f64, f64, f64, f64)> {
    let profile = crate::imprint::circular_revolution_profile(&face.surface, 1e-11).ok()?;
    let frame = profile.frame;
    let (mut net_error, model_jets) =
        revolution_net_error(&face.surface, &profile.angular_curve, profile.curve, frame)?;
    let coords = |point: Vec3| {
        let d = point.sub(frame.origin);
        (d.dot(frame.x_axis), d.dot(frame.axis))
    };
    let (rho0, z0) = coords(profile.curve.evaluate(profile.domain[0]).ok()?);
    let (rho1, z1) = coords(profile.curve.evaluate(profile.domain[1]).ok()?);
    let phase = |rho: f64, z: f64| (z - profile.axial).atan2(rho - profile.major);
    let phi0 = phase(rho0, z0);
    let delta = |phi: f64| {
        (phi - phi0 + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    };
    let direction = delta(phase(rho1, z1)).signum();
    if direction == 0.0 {
        return None;
    }
    let mut angular = profile.angular_curve.clone();
    for h in &mut angular.control_points {
        h.x -= frame.origin.x * h.w;
        h.y -= frame.origin.y * h.w;
        h.z -= frame.origin.z * h.w;
    }
    let angular_domain = angular.domain().ok()?;
    let center = frame
        .origin
        .add(frame.x_axis.scale(profile.major))
        .add(frame.axis.scale(profile.axial));
    let (profile_error, profile_derivative, profile_speed) =
        circle_reconstruction_error(profile.curve, center, profile.minor)?;
    let (angular_error, angular_derivative, angular_speed) =
        circle_reconstruction_error(&profile.angular_curve, frame.origin, 1.0)?;
    let radius_bound = model_jets[0] + frame.origin.length();
    net_error[0] += profile_error + angular_error * radius_bound;
    net_error[1] += profile_error * angular_speed + angular_derivative * radius_bound;
    net_error[2] += profile_derivative + angular_error * profile_speed;
    net_error[3] += profile_derivative * angular_speed + angular_derivative * profile_speed;
    let (ud, vd) = (face.surface.domain_u().ok()?, face.surface.domain_v().ok()?);
    let mut bounds = [
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for h in face
        .loops
        .iter()
        .flat_map(|l| &l.coedges)
        .flat_map(|c| &c.pcurve.control_points)
    {
        let p = h.point().ok()?;
        if h.w <= 0.0 {
            return None;
        }
        bounds[0] = bounds[0].min(p.x);
        bounds[1] = bounds[1].max(p.x);
        bounds[2] = bounds[2].min(p.y);
        bounds[3] = bounds[3].max(p.y);
    }
    let outside_u = (ud[0] - bounds[0]).max(bounds[1] - ud[1]).max(0.0);
    let outside_v = (vd[0] - bounds[2]).max(bounds[3] - vd[1]).max(0.0);
    let [e, eu, ev, euv] = net_error;
    let [m, mu, mv, muv] = model_jets;
    let e = e + outside_u * eu + outside_v * ev + outside_u * outside_v * euv;
    let eu = eu + outside_v * euv;
    let ev = ev + outside_u * euv;
    let m = m + outside_u * mu + outside_v * mv + outside_u * outside_v * muv;
    let mu = mu + outside_v * muv;
    let mv = mv + outside_u * muv;
    let normal_error = eu * mv + ev * mu + eu * ev;
    let material_curves = material_pcurves(face)?;
    let height = (bounds[2] - profile.domain[0])
        .abs()
        .max((bounds[3] - profile.domain[0]).abs());
    let green_measure = u_total_variation(&material_curves)? * height;
    let geometric_error = [
        green_measure * normal_error,
        green_measure * (e * mu * mv + (m + e) * normal_error) / 3.0,
    ];
    if !geometric_error.iter().all(|v| v.is_finite()) {
        return None;
    }

    let primitive = |t: f64, pc: &NurbsCurve| -> Option<[f64; 2]> {
        let uv = pc.derivatives(t, 1).ok()?;
        if uv[1].x == 0.0 {
            return Some([0.0; 2]);
        }
        let v = uv[0].y;
        let anchor = v.clamp(profile.domain[0], profile.domain[1]);
        let (rho, z) = coords(profile.curve.evaluate_extended(v).ok()?);
        let (ra, za) = coords(profile.curve.evaluate(anchor).ok()?);
        if rho <= 0.0 || ra <= 0.0 {
            return None;
        }
        let phi = phase(ra, za);
        let angle = delta(phi);
        let (s0, sa) = (phi0.sin(), phi.sin());
        let (r, major) = (profile.minor, profile.major);
        let ds = sa - s0;
        let d2 = (2.0 * phi).sin() - (2.0 * phi0).sin();
        let mut area = r * direction * (major * angle + r * ds);
        let mut i1 = major * r * ds + r * r * (0.5 * angle + 0.25 * d2);
        let mut i2 = major * major * r * ds
            + major * r * r * (angle + 0.5 * d2)
            + r * r * r * (ds - (sa * sa * sa - s0 * s0 * s0) / 3.0);
        let (dr, dz) = (rho - ra, z - za);
        if v != anchor {
            area += (v - anchor).signum() * (ra + 0.5 * dr) * dr.hypot(dz);
            i1 += dz * (ra + 0.5 * dr);
            i2 += dz * (ra * ra + ra * dr + dr * dr / 3.0);
        }
        // Outside open U the native angular tangent has the same oriented
        // normal as its endpoint, including mixed U/V corner continuation.
        let u = uv[0].x.clamp(angular_domain[0], angular_domain[1]);
        let a = angular.derivatives(u, 1).ok()?;
        let speed = frame.axis.dot(a[0].cross(a[1]));
        let flux = speed
            * (frame.origin.dot(a[0]) * i1 + 1.5 * i2
                - 0.5 * frame.origin.dot(frame.axis) * (rho * rho - rho0 * rho0)
                - 0.5 * (z * rho * rho - z0 * rho0 * rho0))
            / 3.0;
        let result = [-uv[1].x * speed.abs() * area, -uv[1].x * flux];
        result.iter().all(|v| v.is_finite()).then_some(result)
    };
    let mut total = [0.0; 2];
    let mut errors = [0.0; 2];
    let mut magnitude = [0.0; 2];
    let mut budget = 32768;
    for pc in &material_curves {
        let [a, b] = pc.domain().ok()?;
        let mut breaks = vec![a, b];
        breaks.extend(pc.knots.iter().copied().filter(|t| *t > a && *t < b));
        // Partition at the represented angular knots and open-chart limits.
        // Exact profile/UV roots select integration partitions only; they
        // never move an interior point or authorize carrier correspondence.
        let mut angular_knots = angular.knots.clone();
        angular_knots.sort_by(f64::total_cmp);
        angular_knots.dedup();
        let (umin, umax) = pc.control_points.iter().try_fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(a, b), h| {
                if h.w <= 0.0 || !h.x.is_finite() || !h.w.is_finite() {
                    return None;
                }
                let u = h.x / h.w;
                Some((a.min(u), b.max(u)))
            },
        )?;
        for u in angular_knots {
            if u > umin && u < umax {
                let plane = crate::make_plane(
                    Vec3::new(u, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    1.0,
                    1.0,
                )
                .ok()?;
                breaks.extend(
                    curve_plane_roots(pc.clone(), &plane)?
                        .into_iter()
                        .filter(|t| *t > a && *t < b),
                );
            }
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        if breaks.len() > 4096 {
            return None;
        }
        for span in breaks.windows(2) {
            let (value, error) = integrate(
                &|t| primitive(t, pc),
                span[0],
                span[1],
                1e-12,
                20,
                &mut budget,
            )?;
            for i in 0..2 {
                total[i] += value[i];
                errors[i] += error[i];
                magnitude[i] += value[i].abs();
            }
        }
    }
    // Explicit arithmetic reserve, independent of the kernel's answer.
    // Full native tensor jets and circular radial/derivative uncertainty are
    // propagated over the actual material box, including open-chart corners.
    let roundoff = 256.0 * f64::EPSILON;
    for i in 0..2 {
        errors[i] += roundoff * (1.0 + magnitude[i]) + geometric_error[i];
    }
    if geometric_error[0] > 1e-11 * total[0].abs() || geometric_error[1] > 1e-11 * total[1].abs() {
        return None;
    }
    if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
        eprintln!("paired: circular reference profile deviation {:.3e}, area {:.12}, flux {:.12}, estimated errors {:.3e}/{:.3e}",profile.deviation,total[0].abs(),total[1]*total[0].signum()*if face.same_sense {1.0}else{-1.0},errors[0],errors[1]);
    }
    Some((
        total[0].abs(),
        total[1] * total[0].signum() * if face.same_sense { 1.0 } else { -1.0 },
        errors[0],
        errors[1],
    ))
}

fn general_material_reference(face: &FaceRecord) -> Option<(f64, f64, f64, f64)> {
    let sf = &face.surface;
    let [v0, v1] = sf.domain_v().ok()?;
    let inner_error = std::cell::Cell::new([0.0_f64; 2]);
    let primitive = |t: f64, pc: &NurbsCurve| -> Option<[f64; 2]> {
        let uv = pc.derivatives(t, 1).ok()?;
        let end = uv[0].y;
        if end == v0 {
            return Some([0.0; 2]);
        }
        let mut breaks = vec![v0, end];
        breaks.extend(
            sf.knots_v
                .iter()
                .copied()
                .filter(|v| *v > v0.min(end) && *v < v0.max(end)),
        );
        if v1 > v0.min(end) && v1 < v0.max(end) {
            breaks.push(v1);
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        let mut total = [0.0; 2];
        let mut errors = [0.0; 2];
        let mut budget = 2048;
        for span in breaks.windows(2) {
            let (z, e) = integrate(
                &|v| {
                    let d = sf.derivatives_extended(uv[0].x, v, 1).ok()?;
                    let n = d[1][0].cross(d[0][1]);
                    Some([n.length(), d[0][0].dot(n) / 3.0])
                },
                span[0],
                span[1],
                1e-13,
                16,
                &mut budget,
            )?;
            for i in 0..2 {
                total[i] += z[i];
                errors[i] += e[i] * uv[1].x.abs();
            }
        }
        let mut e = inner_error.get();
        for i in 0..2 {
            e[i] = e[i].max(errors[i]);
        }
        inner_error.set(e);
        let sign = if end >= v0 { -1.0 } else { 1.0 };
        Some([sign * total[0] * uv[1].x, sign * total[1] * uv[1].x])
    };
    let mut total = [0.0; 2];
    let mut errors = [0.0; 2];
    let mut measure = 0.0;
    let mut budget = 32768;
    for pc in &material_pcurves(face)? {
        let [a, b] = pc.domain().ok()?;
        measure += b - a;
        let mut breaks = vec![a, b];
        breaks.extend(pc.knots.iter().copied().filter(|t| *t > a && *t < b));
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        for span in breaks.windows(2) {
            let (z, e) = integrate(
                &|t| primitive(t, pc),
                span[0],
                span[1],
                1e-12,
                20,
                &mut budget,
            )?;
            for i in 0..2 {
                total[i] += z[i];
                errors[i] += e[i];
            }
        }
    }
    let ie = inner_error.get();
    Some((
        total[0].abs(),
        total[1]
            * if sf.is_affine().ok()? {
                1.0
            } else {
                total[0].signum() * if face.same_sense { 1.0 } else { -1.0 }
            },
        errors[0] + ie[0] * measure,
        errors[1] + ie[1] * measure,
    ))
}
/// Ruled area and world-origin divergence primitives, separately evaluated
/// from the kernel's face integrators. Equal ruling weights make v physically
/// affine; numerical quadrature reports estimated error, not interval proof.
fn material_reference(face: &FaceRecord) -> Option<(f64, f64, f64, f64)> {
    let sf = &face.surface;
    if sf.degree_v != 1
        || sf
            .control_points
            .iter()
            .any(|r| r.len() != 2 || r[0].w <= 0.0 || r[0].w != r[1].w)
    {
        return circular_material_reference(face).or_else(|| general_material_reference(face));
    }
    let [v0, v1] = sf.domain_v().ok()?;
    let inner_error = std::cell::Cell::new(0.0_f64);
    // S=C(u)+lambda E(u). Its oriented area vector is A+lambda B,
    // A=C' cross E and B=E' cross E. The volume primitive is polynomial;
    // the area primitive integrates the norm, independently of face_area.
    let primitive = |t: f64, pc: &NurbsCurve| -> Option<[f64; 2]> {
        let uv = pc.derivatives(t, 1).ok()?;
        let a = sf.derivatives_extended(uv[0].x, v0, 1).ok()?;
        let b = sf.derivatives_extended(uv[0].x, v1, 1).ok()?;
        let ruling = b[0][0].sub(a[0][0]);
        let av = a[1][0].cross(ruling);
        let bv = b[1][0].sub(a[1][0]).cross(ruling);
        let lambda = (uv[0].y - v0) / (v1 - v0);
        if !lambda.is_finite() {
            return None;
        }
        let integral = if lambda == 0.0 {
            0.0
        } else {
            let mut budget = 256;
            let (value, error) = integrate(
                &|x| Some([av.add(bv.scale(x)).length(), 0.0]),
                0.0,
                lambda,
                1e-13 * (1.0 + av.length() + bv.length()),
                12,
                &mut budget,
            )?;
            inner_error.set(inner_error.get().max(error[0] * uv[1].x.abs()));
            value[0]
        };
        let volume = a[0][0].dot(av) * lambda + 0.5 * a[0][0].dot(bv) * lambda * lambda;
        Some([-integral * uv[1].x, -volume * uv[1].x / 3.0])
    };
    let mut total = [0.0; 2];
    let mut errors = [0.0; 2];
    let mut budget = 32768usize;
    let mut parameter_measure = 0.0;
    for pc in &material_pcurves(face)? {
        let [a, b] = pc.domain().ok()?;
        parameter_measure += b - a;
        let mut breaks = vec![a, b];
        breaks.extend(pc.knots.iter().copied().filter(|&t| t > a && t < b));
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        for span in breaks.windows(2) {
            let f = |t| primitive(t, pc);
            let (value, error) = integrate(&f, span[0], span[1], 1e-12, 20, &mut budget)?;
            for i in 0..2 {
                total[i] += value[i];
                errors[i] += error[i];
            }
        }
    }
    Some((
        total[0].abs(),
        total[1]
            * if sf.is_affine().ok()? {
                1.0
            } else {
                total[0].signum() * if face.same_sense { 1.0 } else { -1.0 }
            },
        errors[0] + inner_error.get() * parameter_measure,
        errors[1],
    ))
}

fn integrate(
    f: &impl Fn(f64) -> Option<[f64; 2]>,
    a: f64,
    b: f64,
    tolerance: f64,
    depth: usize,
    budget: &mut usize,
) -> Option<([f64; 2], [f64; 2])> {
    if *budget < 12 {
        return None;
    }
    *budget -= 12;
    let m = (a + b) * 0.5;
    let half = (b - a) * 0.5;
    let quadrature = |pairs: &[(f64, f64)]| -> Option<[f64; 2]> {
        let mut result = [0.0; 2];
        for &(node, weight) in pairs {
            let left = f(m - half * node)?;
            let right = f(m + half * node)?;
            for i in 0..2 {
                result[i] += half * weight * (left[i] + right[i]);
            }
        }
        result.iter().all(|v| v.is_finite()).then_some(result)
    };
    let coarse = quadrature(&[
        (0.8611363115940526, 0.3478548451374538),
        (0.3399810435848563, 0.6521451548625461),
    ])?;
    let fine = quadrature(&[
        (0.9602898564975363, 0.1012285362903763),
        (0.7966664774136267, 0.2223810344533745),
        (0.5255324099163290, 0.3137066458778873),
        (0.1834346424956498, 0.3626837833783619),
    ])?;
    let error = std::array::from_fn::<_, 2, _>(|i| (fine[i] - coarse[i]).abs());
    if error.iter().all(|&e| e <= tolerance) {
        return Some((fine, error));
    }
    if depth == 0 {
        return None;
    }
    let (x, ex) = integrate(f, a, m, tolerance * 0.5, depth - 1, budget)?;
    let (y, ey) = integrate(f, m, b, tolerance * 0.5, depth - 1, budget)?;
    Some((
        std::array::from_fn(|i| x[i] + y[i]),
        std::array::from_fn(|i| ex[i] + ey[i]),
    ))
}

fn shared_use_agrees(sf: &NurbsSurface, pc: &NurbsCurve, curve: &NurbsCurve, bar: f64) -> bool {
    let (Ok([a, b]), Ok([x, y])) = (pc.domain(), curve.domain()) else {
        return false;
    };
    for t in super::super::edges::carrier_stations(pc, a, b) {
        let Ok(point) = image(sf, pc, t) else {
            return false;
        };
        if crate::project_point_to_curve(curve, point)
            .ok()
            .is_none_or(|p| p.distance > bar)
        {
            return false;
        }
    }
    for t in super::super::edges::carrier_stations(curve, x, y) {
        let Ok(point) = curve.evaluate(t) else {
            return false;
        };
        if image_distance(sf, pc, point).ok().is_none_or(|d| d > bar) {
            return false;
        }
    }
    true
}

pub(super) fn reconstruct(solid: &mut BrepSolid, bar: f64) -> (Vec<ReplacedTrim>, HashSet<usize>) {
    let tracing = super::super::clearance::motion_trace_enabled();
    if tracing {
        super::super::clearance::trace_motion(
            serde_json::json!({"kind":"paired-before","solid":solid}),
        );
    }
    let result = reconstruct_impl(solid, bar);
    if tracing {
        super::super::clearance::trace_motion(
            serde_json::json!({"kind":"paired-after","solid":solid,"changed_uses":result.0.len(),"certified_faces":result.1}),
        );
    }
    result
}

fn reconstruct_impl(solid: &mut BrepSolid, bar: f64) -> (Vec<ReplacedTrim>, HashSet<usize>) {
    let debug = std::env::var("BREP_DEBUG_JOINTS").is_ok();
    let faces: Vec<_> = solid.shells.iter().flat_map(|s| &s.faces).collect();
    let edge_indices: HashMap<_, _> = solid
        .edges
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect();
    let vertices: HashMap<_, _> = solid.vertices.iter().map(|v| (v.id, v.point)).collect();
    let mut uses: HashMap<u64, Vec<usize>> = HashMap::default();
    for (i, face) in faces.iter().enumerate() {
        for c in face.loops.iter().flat_map(|l| &l.coedges) {
            let item = uses.entry(c.edge_id).or_default();
            if !item.contains(&i) {
                item.push(i);
            }
        }
    }
    let mut accepted = Vec::new();
    for (fi, face) in faces.iter().enumerate() {
        if face.surface.degree_v != 1
            || face.surface.is_affine().unwrap_or(false)
            || face
                .surface
                .control_points
                .iter()
                .any(|r| r.len() != 2 || r[0].w <= 0.0 || r[0].w != r[1].w)
        {
            continue;
        }
        let needed = face.loops.iter().any(|l| {
            l.coedges.iter().enumerate().any(|(ci, c)| {
                let Some(edge) = edge_indices.get(&c.edge_id).map(|i| &solid.edges[*i]) else {
                    return false;
                };
                let (Some(&start), Some(&end)) = (
                    vertices.get(&edge.start_vertex_id),
                    vertices.get(&edge.end_vertex_id),
                ) else {
                    return false;
                };
                let (start, end) = if c.forward {
                    (start, end)
                } else {
                    (end, start)
                };
                let Ok([a, b]) = c.pcurve.domain() else {
                    return false;
                };
                let (Ok(p), Ok(q)) = (
                    image(&face.surface, &c.pcurve, a),
                    image(&face.surface, &c.pcurve, b),
                ) else {
                    return false;
                };
                if p.sub(start).length() > bar || q.sub(end).length() > bar {
                    return true;
                }
                let next = &l.coedges[(ci + 1) % l.coedges.len()].pcurve;
                next.domain()
                    .ok()
                    .and_then(|d| image(&face.surface, next, d[0]).ok())
                    .is_some_and(|r| q.sub(r).length() > bar)
            })
        });
        if !needed {
            continue;
        }
        let mut valid = true;
        let mut rebuilt = (*face).clone();
        let mut companions = Vec::<(usize, usize, usize, NurbsCurve)>::new();
        let mut new_edges = HashMap::<u64, (NurbsCurve, f64, f64)>::default();
        for (li, loop_record) in face.loops.iter().enumerate() {
            for (ci, c) in loop_record.coedges.iter().enumerate() {
                let Some(edge) = edge_indices.get(&c.edge_id).map(|i| &solid.edges[*i]) else {
                    valid = false;
                    break;
                };
                if edge.degenerate {
                    valid = false;
                    break;
                }
                let (Some(&start), Some(&end)) = (
                    vertices.get(&edge.start_vertex_id),
                    vertices.get(&edge.end_vertex_id),
                ) else {
                    valid = false;
                    break;
                };
                let (start, end) = if c.forward {
                    (start, end)
                } else {
                    (end, start)
                };
                let Ok([a, b]) = c.pcurve.domain() else {
                    valid = false;
                    break;
                };
                let (Ok(old_start), Ok(old_end)) = (
                    image(&face.surface, &c.pcurve, a),
                    image(&face.surface, &c.pcurve, b),
                ) else {
                    valid = false;
                    break;
                };
                let endpoint_miss = old_start.sub(start).length().max(old_end.sub(end).length());
                if uses.get(&edge.id).is_none_or(|v| v.len() != 2) {
                    valid = false;
                    break;
                }
                let other_index = *uses
                    .get(&edge.id)
                    .and_then(|v| v.iter().find(|&&f| f != fi))
                    .expect("two distinct uses");
                let other = faces[other_index];
                let locations: Vec<_> = other
                    .loops
                    .iter()
                    .enumerate()
                    .flat_map(|(l, r)| {
                        r.coedges
                            .iter()
                            .enumerate()
                            .filter(move |(_, c)| c.edge_id == edge.id)
                            .map(move |(c, _)| (l, c))
                    })
                    .collect();
                if locations.len() != 1 {
                    valid = false;
                    break;
                }
                let (oli, oci) = locations[0];
                let oc = &other.loops[oli].coedges[oci];
                let other_old = if oc.forward == c.forward {
                    Some(oc.pcurve.clone())
                } else {
                    oc.pcurve.reversed().ok()
                };
                let Some(other_old) = other_old else {
                    valid = false;
                    break;
                };
                let mut inherited_miss = endpoint_miss;
                for (sf, old, target) in [
                    (&face.surface, &c.pcurve, &other.surface),
                    (&other.surface, &other_old, &face.surface),
                ] {
                    let Ok([lo, hi]) = old.domain() else {
                        valid = false;
                        break;
                    };
                    for t in super::super::edges::carrier_stations(old, lo, hi) {
                        let Some(point) = image(sf, old, t).ok() else {
                            valid = false;
                            break;
                        };
                        let Some(proj) = crate::project_point_to_surface(target, point).ok() else {
                            valid = false;
                            break;
                        };
                        inherited_miss = inherited_miss.max(proj.distance);
                    }
                }
                let allowed = (8.0 * inherited_miss.max(bar)).min(4e-3);
                let Some(Boundary {
                    own: pc,
                    curve,
                    partner,
                }) = boundary(
                    &face.surface,
                    &other.surface,
                    &c.pcurve,
                    &other_old,
                    start,
                    end,
                    bar,
                    allowed,
                )
                else {
                    if debug {
                        eprintln!(
                            "paired: face {fi} edge {} unsupported coupled boundary",
                            edge.id
                        );
                    }
                    valid = false;
                    break;
                };
                let partner = if oc.forward == c.forward {
                    Some(partner)
                } else {
                    partner.reversed().ok()
                };
                let Some(partner) = partner else {
                    valid = false;
                    break;
                };
                if !super::super::clearance::trim_motion_clear(
                    &other.surface,
                    &oc.pcurve,
                    &partner,
                    edge,
                    &faces,
                    &solid.edges,
                ) {
                    if debug {
                        eprintln!(
                            "paired: face {fi} edge {} companion motion declined",
                            edge.id
                        );
                    }
                    valid = false;
                    break;
                }
                companions.push((other_index, oli, oci, partner));
                let Ok([p0, p1]) = pc.domain() else {
                    valid = false;
                    break;
                };
                let ends = [
                    image(&face.surface, &pc, p0).ok(),
                    image(&face.surface, &pc, p1).ok(),
                ];
                if ends[0].is_none_or(|p| p.sub(start).length() > bar * 0.5)
                    || ends[1].is_none_or(|p| p.sub(end).length() > bar * 0.5)
                {
                    if debug {
                        eprintln!(
                            "paired: face {fi} edge {} cannot reach settled vertices",
                            edge.id
                        );
                    }
                    valid = false;
                    break;
                }
                let Some(previous_curve) = super::super::edges::represented_curve(edge) else {
                    valid = false;
                    break;
                };
                let mut displacement = 0.0_f64;
                let mut edge_miss = 0.0_f64;
                let Ok([x, y]) = curve.domain() else {
                    valid = false;
                    break;
                };
                for t in super::super::edges::carrier_stations(&curve, x, y) {
                    let Some(point) = curve.evaluate(t).ok() else {
                        valid = false;
                        break;
                    };
                    let (Ok(d), Ok(proj)) = (
                        image_distance(&face.surface, &c.pcurve, point),
                        crate::project_point_to_curve(&previous_curve, point),
                    ) else {
                        valid = false;
                        break;
                    };
                    let Ok(partner_move) = image_distance(&other.surface, &other_old, point) else {
                        valid = false;
                        break;
                    };
                    displacement = displacement.max(d).max(partner_move);
                    edge_miss = edge_miss.max(proj.distance);
                }
                let Ok([lo, hi]) = previous_curve.domain() else {
                    valid = false;
                    break;
                };
                for t in super::super::edges::carrier_stations(&previous_curve, lo, hi) {
                    let Ok(point) = previous_curve.evaluate(t) else {
                        valid = false;
                        break;
                    };
                    let Ok(projection) = crate::project_point_to_curve(&curve, point) else {
                        valid = false;
                        break;
                    };
                    edge_miss = edge_miss.max(projection.distance);
                }
                let allowed = (8.0 * inherited_miss.max(bar)).min(4e-3);
                let edge_curve = if c.forward {
                    curve.clone()
                } else {
                    let Ok(reversed) = curve.reversed() else {
                        valid = false;
                        break;
                    };
                    reversed
                };
                let edge_motion_ok = if edge_miss <= bar {
                    true
                } else {
                    if edge_miss > allowed {
                        false
                    } else {
                        super::super::clearance::motion_clear(
                            edge,
                            &edge_curve,
                            &faces,
                            &solid.edges,
                        )
                    }
                };
                if !valid
                    || displacement > allowed
                    || !edge_motion_ok
                    || !super::super::clearance::trim_motion_clear(
                        &face.surface,
                        &c.pcurve,
                        &pc,
                        edge,
                        &faces,
                        &solid.edges,
                    )
                {
                    if debug {
                        eprintln!("paired: face {fi} edge {} declined motion {displacement:.3e}/{allowed:.3e}, edge miss {edge_miss:.3e}",edge.id);
                    }
                    valid = false;
                    break;
                }
                if edge_miss > bar {
                    let Ok([ea, eb]) = edge_curve.domain() else {
                        valid = false;
                        break;
                    };
                    let mut before = 0.0_f64;
                    let mut after = 0.0_f64;
                    for (source, lo, hi, value) in [
                        (&edge.curve, edge.t0, edge.t1, &mut before),
                        (&edge_curve, ea, eb, &mut after),
                    ] {
                        for t in super::super::edges::carrier_stations(source, lo, hi) {
                            let Some(point) = source.evaluate(t).ok() else {
                                valid = false;
                                break;
                            };
                            for surface in [&face.surface, &other.surface] {
                                let Some(projection) =
                                    crate::project_point_to_surface(surface, point).ok()
                                else {
                                    valid = false;
                                    break;
                                };
                                *value = value.max(projection.distance);
                            }
                        }
                    }
                    if !valid || after > bar {
                        valid = false;
                        break;
                    }
                    new_edges.insert(edge.id, (edge_curve, before, after));
                }
                rebuilt.loops[li].coedges[ci].pcurve = pc;
            }
            if !valid {
                break;
            }
        }
        if !needed || !valid {
            continue;
        }
        if !super::face_joints_not_worse(face, &rebuilt, bar) {
            if debug {
                eprintln!("paired: face {fi} declined worse joints");
            }
            continue;
        }
        let Some((area, volume, area_error, volume_error)) = material_reference(&rebuilt) else {
            if debug {
                eprintln!("paired: face {fi} unreadable material reference");
            }
            continue;
        };
        let (Ok(old_area), Ok(new_area), Ok(old_volume), Ok(new_volume)) = (
            crate::face_area(face),
            crate::face_area(&rebuilt),
            crate::face_volume_contribution(face),
            crate::face_volume_contribution(&rebuilt),
        ) else {
            continue;
        };
        let area_allowance = 1e-11 * area.abs().max(new_area.abs()) + bar * bar + area_error;
        let volume_allowance =
            1e-11 * volume.abs().max(new_volume.abs()) + bar * bar + volume_error;
        if !(new_area.is_finite() && new_volume.is_finite())
            || (new_area - area).abs() > area_allowance
            || (new_volume - volume).abs() > volume_allowance
            || (new_area - area).abs() > (old_area - area).abs() + area_allowance
            || (new_volume - volume).abs() > (old_volume - volume).abs() + volume_allowance
        {
            if debug {
                eprintln!("paired: face {fi} material declined area {old_area:.12}->{new_area:.12} reference {area:.12}; volume {old_volume:.12}->{new_volume:.12} reference {volume:.12}");
            }
            let Some(refined) = refined_material_candidate(face, &rebuilt, bar) else {
                continue;
            };
            rebuilt = refined;
            if debug {
                eprintln!(
                    "paired: face {fi} exact knot partition area {:.12}, volume {:.12}",
                    crate::face_area(&rebuilt).unwrap_or(f64::NAN),
                    crate::face_volume_contribution(&rebuilt).unwrap_or(f64::NAN)
                );
            }
        }
        if debug {
            eprintln!("paired: face {fi} exact loop area {old_area:.12}->{new_area:.12}, volume {old_volume:.12}->{new_volume:.12}");
        }
        accepted.push((fi, rebuilt, new_edges, companions));
    }
    let primary: HashSet<_> = accepted.iter().map(|(fi, _, _, _)| *fi).collect();
    let mut final_faces = HashMap::<usize, FaceRecord>::default();
    let mut proposals = HashMap::<u64, (NurbsCurve, f64, f64)>::default();
    for (fi, rebuilt, changes, _) in &accepted {
        final_faces.insert(*fi, rebuilt.clone());
        for (&id, change) in changes {
            if let Some((previous, _, _)) = proposals.get(&id) {
                if !super::super::image::difference_bound(previous, &change.0)
                    .is_some_and(|d| d <= bar)
                {
                    if debug {
                        eprintln!("paired: incompatible 3D proposals edge {id}");
                    }
                    return (Vec::new(), HashSet::default());
                }
            } else {
                proposals.insert(id, change.clone());
            }
        }
    }
    for (_, _, _, companions) in &accepted {
        for (fi, li, ci, pc) in companions {
            let f = final_faces.entry(*fi).or_insert_with(|| faces[*fi].clone());
            if primary.contains(fi) {
                // A whole-face proposal already owns this use. Check its
                // geometry below against the final shared curve.
                continue;
            }
            f.loops[*li].coedges[*ci].pcurve = pc.clone();
        }
    }
    // An inherited planar loop can be internally closed at the wrong image
    // of its settled vertices. Replacing one side would open its neighbours.
    // Reconstruct complete affected affine loops from the final represented
    // edges, and propagate to an affine neighbour only when its retained use
    // disagrees. Curved neighbours remain subject to their whole-loop proposal.
    let mut pending: Vec<_> = final_faces
        .keys()
        .copied()
        .filter(|fi| !primary.contains(fi) && faces[*fi].surface.is_affine().unwrap_or(false))
        .collect();
    let mut visited = HashSet::default();
    while let Some(fi) = pending.pop() {
        if !visited.insert(fi) {
            continue;
        }
        let mut rebuilt = final_faces
            .get(&fi)
            .cloned()
            .unwrap_or_else(|| faces[fi].clone());
        for (li, l) in rebuilt.loops.iter_mut().enumerate() {
            for (ci, c) in l.coedges.iter_mut().enumerate() {
                let edge = &solid.edges[edge_indices[&c.edge_id]];
                let Ok([pc_start, pc_end]) = c.pcurve.domain() else {
                    return (Vec::new(), HashSet::default());
                };
                let start_vertex = if c.forward {
                    edge.start_vertex_id
                } else {
                    edge.end_vertex_id
                };
                let end_vertex = if c.forward {
                    edge.end_vertex_id
                } else {
                    edge.start_vertex_id
                };
                let endpoints_settled = [(pc_start, start_vertex), (pc_end, end_vertex)]
                    .into_iter()
                    .all(|(t, id)| {
                        image(&rebuilt.surface, &c.pcurve, t)
                            .ok()
                            .is_some_and(|p| p.sub(vertices[&id]).length() <= bar)
                    });
                if endpoints_settled && !proposals.contains_key(&c.edge_id) {
                    continue;
                }
                // A curved neighbour may already own the exact boundary
                // isocurve while the inherited 3D edge/trim range is stale.
                // Construct that represented section and stage both uses;
                // never project inherited interior stations onto the carrier.
                if !proposals.contains_key(&c.edge_id) {
                    for &other in &uses[&c.edge_id] {
                        if other == fi
                            || primary.contains(&other)
                            || faces[other].surface.is_affine().unwrap_or(false)
                        {
                            continue;
                        }
                        let neighbour = final_faces
                            .get(&other)
                            .cloned()
                            .unwrap_or_else(|| faces[other].clone());
                        let matches: Vec<_> = neighbour
                            .loops
                            .iter()
                            .enumerate()
                            .flat_map(|(li, l)| {
                                l.coedges
                                    .iter()
                                    .enumerate()
                                    .filter(move |(_, n)| n.edge_id == edge.id)
                                    .map(move |(ci, n)| (li, ci, n.clone()))
                            })
                            .collect();
                        if matches.len() != 1 {
                            return (Vec::new(), HashSet::default());
                        }
                        let (nl, nc, n) = &matches[0];
                        let (start, end) = if n.forward {
                            (
                                vertices[&edge.start_vertex_id],
                                vertices[&edge.end_vertex_id],
                            )
                        } else {
                            (
                                vertices[&edge.end_vertex_id],
                                vertices[&edge.start_vertex_id],
                            )
                        };
                        let Some(pair) = boundary(
                            &neighbour.surface,
                            &rebuilt.surface,
                            &n.pcurve,
                            &c.pcurve,
                            start,
                            end,
                            bar,
                            4e-3,
                        ) else {
                            if debug {
                                eprintln!("paired: affine face {fi} edge {} curved boundary {other} unsupported",edge.id);
                            }
                            return (Vec::new(), HashSet::default());
                        };
                        let canonical = if n.forward {
                            pair.curve
                        } else {
                            let Ok(r) = pair.curve.reversed() else {
                                return (Vec::new(), HashSet::default());
                            };
                            r
                        };
                        let Some(previous) = super::super::edges::represented_curve(edge) else {
                            return (Vec::new(), HashSet::default());
                        };
                        if super::super::clearance::covering_motion_bound(&previous, &canonical)
                            .is_none()
                        {
                            if debug {
                                eprintln!("paired: affine face {fi} edge {} curved boundary movement declined",edge.id);
                            }
                            return (Vec::new(), HashSet::default());
                        }
                        let Ok([a, b]) = canonical.domain() else {
                            return (Vec::new(), HashSet::default());
                        };
                        proposals.insert(edge.id, (canonical, a, b));
                        let neighbour = final_faces
                            .entry(other)
                            .or_insert_with(|| faces[other].clone());
                        neighbour.loops[*nl].coedges[*nc].pcurve = pair.own;
                    }
                }
                let retained;
                let curve = if let Some((curve, _, _)) = proposals.get(&c.edge_id) {
                    curve
                } else {
                    let Some(r) = super::super::edges::represented_curve(edge) else {
                        return (Vec::new(), HashSet::default());
                    };
                    retained = r;
                    &retained
                };
                let oriented = if c.forward {
                    curve.clone()
                } else {
                    let Ok(r) = curve.reversed() else {
                        return (Vec::new(), HashSet::default());
                    };
                    r
                };
                let Some(pc) =
                    super::super::image::affine_inverse(&rebuilt.surface, &oriented, bar * 0.1)
                else {
                    if debug {
                        eprintln!(
                            "paired: affine face {fi} edge {} exact inverse declined",
                            c.edge_id
                        );
                    }
                    return (Vec::new(), HashSet::default());
                };
                let Ok([a, b]) = pc.domain() else {
                    return (Vec::new(), HashSet::default());
                };
                let start = if c.forward {
                    edge.start_vertex_id
                } else {
                    edge.end_vertex_id
                };
                let end = if c.forward {
                    edge.end_vertex_id
                } else {
                    edge.start_vertex_id
                };
                for (t, vertex) in [(a, start), (b, end)] {
                    if pc
                        .evaluate(t)
                        .and_then(|uv| rebuilt.surface.evaluate_extended(uv.x, uv.y))
                        .ok()
                        .is_none_or(|p| p.sub(vertices[&vertex]).length() > bar)
                    {
                        if debug {
                            eprintln!(
                                "paired: affine face {fi} edge {} settled endpoint declined",
                                c.edge_id
                            );
                        }
                        return (Vec::new(), HashSet::default());
                    }
                }
                // Whole positive images bound this additional local repair;
                // clearance below checks the entire swept correspondence.
                let old = &faces[fi].loops[li].coedges[ci].pcurve;
                let Some(old_image) = super::super::image::bilinear_image(&rebuilt.surface, old)
                else {
                    return (Vec::new(), HashSet::default());
                };
                let Some(new_image) = super::super::image::bilinear_image(&rebuilt.surface, &pc)
                else {
                    return (Vec::new(), HashSet::default());
                };
                // Different parameterizations require the same covering map
                // used by clearance, rather than an equal-parameter distance.
                let Some(movement) =
                    super::super::clearance::covering_motion_bound(&old_image, &new_image)
                else {
                    if debug {
                        eprintln!(
                            "paired: affine face {fi} edge {} whole movement declined",
                            c.edge_id
                        );
                    }
                    return (Vec::new(), HashSet::default());
                };
                if movement > 4e-3 {
                    return (Vec::new(), HashSet::default());
                }
                c.pcurve = pc;
                for &other in &uses[&c.edge_id] {
                    if other == fi || primary.contains(&other) || visited.contains(&other) {
                        continue;
                    }
                    let neighbour = final_faces.get(&other).unwrap_or(faces[other]);
                    let agrees = neighbour
                        .loops
                        .iter()
                        .flat_map(|l| &l.coedges)
                        .filter(|c| c.edge_id == edge.id)
                        .all(|c| shared_use_agrees(&neighbour.surface, &c.pcurve, curve, bar));
                    if !agrees {
                        if !neighbour.surface.is_affine().unwrap_or(false) {
                            if debug {
                                eprintln!("paired: affine closure edge {} requires unsupported curved neighbour {other}",edge.id);
                            }
                            return (Vec::new(), HashSet::default());
                        }
                        pending.push(other);
                    }
                }
            }
        }
        final_faces.insert(fi, rebuilt);
    }
    // The containment polygon samples according to represented knot count.
    // A newly exact curved boundary can be much less subdivided than its
    // inherited polyline neighbour, so its chords can cross a disjoint trim.
    // Refine only reported new pairs, preserving the complete rational image.
    // Genuine crossings survive refinement and the outer guard still rejects.
    let old_crossings = crate::loop_self_crossings(solid);
    for round in 0..=3 {
        let mut preview = solid.clone();
        for (&fi, face) in &final_faces {
            *face_at_mut(&mut preview.shells, fi) = face.clone();
        }
        let report = crate::loop_self_crossings(&preview);
        if !report.unreadable.is_empty() {
            return (Vec::new(), HashSet::default());
        }
        let introduced: Vec<_> = report
            .crossings
            .iter()
            .filter(|c| {
                !old_crossings.crossings.iter().any(|o| {
                    (o.face, o.loop_id, o.coedge_a, o.coedge_b)
                        == (c.face, c.loop_id, c.coedge_a, c.coedge_b)
                })
            })
            .collect();
        if introduced.is_empty() {
            break;
        }
        if round == 3 {
            return (Vec::new(), HashSet::default());
        }
        let mut refined = false;
        for crossing in introduced {
            let Some(fi) = faces.iter().position(|f| f.id == crossing.face) else {
                return (Vec::new(), HashSet::default());
            };
            let f = final_faces.entry(fi).or_insert_with(|| faces[fi].clone());
            for c in f
                .loops
                .iter_mut()
                .filter(|l| l.id == crossing.loop_id)
                .flat_map(|l| &mut l.coedges)
                .filter(|c| c.id == crossing.coedge_a || c.id == crossing.coedge_b)
            {
                if c.pcurve.degree < 2 {
                    continue;
                }
                let Some(pc) = exact_midpoint_partition(&c.pcurve, bar * 0.01) else {
                    return (Vec::new(), HashSet::default());
                };
                c.pcurve = pc;
                refined = true;
            }
        }
        if !refined {
            return (Vec::new(), HashSet::default());
        }
    }
    let final_targets: Vec<_> = faces
        .iter()
        .enumerate()
        .map(|(fi, f)| final_faces.get(&fi).unwrap_or(f))
        .collect();
    for (&id, (curve, _, _)) in &proposals {
        if !super::super::clearance::motion_clear(
            &solid.edges[edge_indices[&id]],
            curve,
            &final_targets,
            &solid.edges,
        ) {
            if debug {
                eprintln!("paired: edge {id} final material motion declined");
            }
            return (Vec::new(), HashSet::default());
        }
    }
    for (&fi, f) in &final_faces {
        if !super::face_joints_not_worse(faces[fi], f, bar)
            || !material_not_worse(faces[fi], f, bar)
        {
            if debug {
                eprintln!("paired: coupled face {fi} joints/material declined");
            }
            return (Vec::new(), HashSet::default());
        }
        for (li, l) in f.loops.iter().enumerate() {
            for (ci, c) in l.coedges.iter().enumerate() {
                let old_pc = &faces[fi].loops[li].coedges[ci].pcurve;
                if !proposals.contains_key(&c.edge_id)
                    && c.pcurve.degree == old_pc.degree
                    && c.pcurve.knots == old_pc.knots
                    && c.pcurve.control_points.len() == old_pc.control_points.len()
                    && c.pcurve
                        .control_points
                        .iter()
                        .zip(&old_pc.control_points)
                        .all(|(a, b)| a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w)
                {
                    continue;
                }

                if !super::super::clearance::trim_motion_clear(
                    &f.surface,
                    old_pc,
                    &c.pcurve,
                    &solid.edges[edge_indices[&c.edge_id]],
                    &final_targets,
                    &solid.edges,
                ) {
                    if debug {
                        eprintln!(
                            "paired: face {fi} edge {} final material motion declined",
                            c.edge_id
                        );
                    }
                    return (Vec::new(), HashSet::default());
                }
                let previous = &solid.edges[edge_indices[&c.edge_id]];
                let retained;
                let curve = if let Some((curve, _, _)) = proposals.get(&c.edge_id) {
                    curve
                } else {
                    let Some(r) = super::super::edges::represented_curve(previous) else {
                        return (Vec::new(), HashSet::default());
                    };
                    retained = r;
                    &retained
                };
                if !shared_use_agrees(&f.surface, &c.pcurve, curve, bar) {
                    if debug {
                        eprintln!(
                            "paired: coupled face {fi} final shared-use mismatch edge {}",
                            c.edge_id
                        );
                    }
                    return (Vec::new(), HashSet::default());
                }
            }
        }
    }
    // Every changed edge must have both final uses in this transaction.
    if proposals
        .keys()
        .any(|id| uses[id].iter().any(|fi| !final_faces.contains_key(fi)))
    {
        return (Vec::new(), HashSet::default());
    }
    drop(faces);
    let mut replaced = Vec::new();
    let mut updated = HashSet::default();
    for (fi, face) in final_faces {
        for (li, l) in face.loops.into_iter().enumerate() {
            for (ci, c) in l.coedges.into_iter().enumerate() {
                let previous = std::mem::replace(
                    &mut face_at_mut(&mut solid.shells, fi).loops[li].coedges[ci].pcurve,
                    c.pcurve,
                );
                let previous_edge = if let Some((curve, before, after)) = proposals
                    .get(&c.edge_id)
                    .filter(|_| updated.insert(c.edge_id))
                {
                    let edge = &mut solid.edges[edge_indices[&c.edge_id]];
                    let [a, b] = curve.domain().expect("constructed curve domain");
                    let previous = std::mem::replace(&mut edge.curve, curve.clone());
                    let (t0, t1) = (edge.t0, edge.t1);
                    edge.t0 = a;
                    edge.t1 = b;
                    Some(super::super::edges::ReplacedEdge {
                        edge_id: edge.id,
                        previous,
                        t0,
                        t1,
                        residual_before: *before,
                        residual_after: *after,
                    })
                } else {
                    None
                };
                replaced.push(ReplacedTrim {
                    face: fi,
                    loop_index: li,
                    coedge: ci,
                    previous,
                    previous_edge,
                });
            }
        }
    }
    (replaced, primary)
}

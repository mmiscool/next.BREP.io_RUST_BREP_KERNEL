//! Bounded native z-axis revolutions with horizontal planar sections.

use super::*;
use crate::{make_revolution, AnalyticSurface};

pub(super) fn enabled() -> bool {
    let requested = match std::env::var("BREP_SOUNDNESS_NURBS_CERTIFICATE") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    };
    super::trim_samples_enabled() && super::node_membership_enabled() && requested
}

fn floats_equal(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a, b)| a.to_bits() == b.to_bits())
}
fn curve_equal(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && floats_equal(&a.knots, &b.knots)
        && a.control_points.len() == b.control_points.len()
        && a.control_points
            .iter()
            .zip(&b.control_points)
            .all(|(a, b)| floats_equal(&[a.x, a.y, a.z, a.w], &[b.x, b.y, b.z, b.w]))
}
fn frame_equal(a: &crate::RevolutionFrame, b: &crate::RevolutionFrame) -> bool {
    [a.origin, a.axis, a.x_axis, a.y_axis]
        .iter()
        .zip([b.origin, b.axis, b.x_axis, b.y_axis])
        .all(|(a, b)| floats_equal(&[a.x, a.y, a.z], &[b.x, b.y, b.z]))
}
// Cached recognition can select a different evaluator in the full scanner.
// Require the complete fresh analytic description, never only its label.
fn cache_equal(a: &NurbsSurface, b: &NurbsSurface) -> bool {
    match (a.analytic(), b.analytic()) {
        (None, None) => true,
        (
            Some(AnalyticSurface::Plane {
                origin: a,
                u_dir: au,
                v_dir: av,
                u_domain: ad,
                v_domain: ae,
            }),
            Some(AnalyticSurface::Plane {
                origin: b,
                u_dir: bu,
                v_dir: bv,
                u_domain: bd,
                v_domain: be,
            }),
        ) => {
            [a, au, av]
                .iter()
                .zip([b, bu, bv])
                .all(|(a, b)| floats_equal(&[a.x, a.y, a.z], &[b.x, b.y, b.z]))
                && floats_equal(ad, bd)
                && floats_equal(ae, be)
        }
        (
            Some(AnalyticSurface::RuledRevolution {
                frame: a,
                rho0: a0,
                rho1: a1,
                height: ah,
            }),
            Some(AnalyticSurface::RuledRevolution {
                frame: b,
                rho0: b0,
                rho1: b1,
                height: bh,
            }),
        ) => frame_equal(a, b) && floats_equal(&[*a0, *a1, *ah], &[*b0, *b1, *bh]),
        (
            Some(AnalyticSurface::Revolution {
                frame: a,
                spans: an,
                sweep: as_,
                generatrix: ag,
            }),
            Some(AnalyticSurface::Revolution {
                frame: b,
                spans: bn,
                sweep: bs,
                generatrix: bg,
            }),
        ) => frame_equal(a, b) && an == bn && as_.to_bits() == bs.to_bits() && curve_equal(ag, bg),
        _ => false,
    }
}
#[derive(Clone, Copy)]
struct Support {
    z0: f64,
    z1: f64,
    rmin: f64,
    rmax: f64,
    general: bool,
}

/// Rebuild every bit of the canonical full z-axis revolution from its first
/// meridian. A linear height with positive radius is globally injective away
/// from the identified azimuth seam. Radius may be a curved quadratic.
fn native(s: &NurbsSurface, reserve: f64) -> Option<Support> {
    if s.degree_u != 2 || s.control_points.len() != 9 || !(1..=2).contains(&s.degree_v) {
        return None;
    }
    let p = &s.control_points[0];
    let n = s.degree_v + 1;
    if p.len() != n
        || s.control_points.iter().any(|r| r.len() != n)
        || s.knots_v != [vec![0.; n], vec![1.; n]].concat()
        || p.iter()
            .any(|h| h.w != 1. || ![h.x, h.y, h.z].iter().all(|v| v.is_finite()))
    {
        return None;
    }
    let profile = NurbsCurve::new(s.degree_v, s.knots_v.clone(), p.clone()).ok()?;
    let rebuilt = [0., p[0].z].into_iter().find_map(|z| {
        let b = make_revolution(
            Vec3::new(0., 0., z),
            Vec3::new(0., 0., 1.),
            &profile,
            std::f64::consts::TAU,
        )
        .ok()?;
        let same = s.degree_u == b.degree_u
            && floats_equal(&s.knots_u, &b.knots_u)
            && s.control_points
                .iter()
                .zip(&b.control_points)
                .all(|(a, b)| {
                    a.iter()
                        .zip(b)
                        .all(|(a, b)| floats_equal(&[a.x, a.y, a.z, a.w], &[b.x, b.y, b.z, b.w]))
                });
        same.then_some(b)
    })?;
    if !cache_equal(s, &rebuilt) {
        return None;
    }
    let radial = p.iter().find(|h| h.x.hypot(h.y) > reserve)?;
    let r = radial.x.hypot(radial.y);
    let dir = [radial.x / r, radial.y / r];
    let mut rmin = f64::INFINITY;
    let mut rmax = 0.0f64;
    for (i, h) in p.iter().enumerate() {
        let radius = h.x * dir[0] + h.y * dir[1];
        if radius < -reserve || (h.x * dir[1] - h.y * dir[0]).abs() > reserve {
            return None;
        }
        let z = p[0].z + (p[n - 1].z - p[0].z) * i as f64 / (n - 1) as f64;
        if (h.z - z).abs() > reserve {
            return None;
        }
        rmin = rmin.min(radius);
        rmax = rmax.max(radius);
    }
    if p[n - 1].z == p[0].z {
        // Native polar planes have a monotone radial parameter; their only
        // collapsed fibre is the legitimate boundary pole.
        if n != 2 || (p[0].x.hypot(p[0].y) - p[1].x.hypot(p[1].y)).abs() <= reserve {
            return None;
        }
    } else if p[n - 1].z <= p[0].z
        || rmin <= reserve
        || p.windows(2).any(|p| p[1].z - p[0].z <= reserve)
    {
        return None;
    }
    Some(Support {
        z0: p[0].z,
        z1: p[n - 1].z,
        rmin,
        rmax,
        general: s.degree_v == 2,
    })
}

fn bounded_wire(face: &FaceRecord) -> bool {
    face.loops.len() == 1
        && face.loops[0].coedges.iter().all(|c| {
            let p = &c.pcurve;
            p.degree > 0
                && p.degree <= 5
                && p.control_points.iter().all(|h| {
                    (0.1..=10.).contains(&h.w) && h.z == 0. && h.x.is_finite() && h.y.is_finite()
                })
                && NurbsCurve::new(p.degree, p.knots.clone(), p.control_points.clone()).is_ok()
        })
}

/// All positively weighted cap trim controls stay in the finite carrier.
/// The native polar caps and Cartesian cap qualify; an extended trim keeps
/// the full scanner rather than relying on extrapolated cached geometry.
fn cap_domain_wire(face: &FaceRecord) -> bool {
    if !bounded_wire(face) {
        return false;
    }
    let (Ok(u), Ok(v)) = (face.surface.domain_u(), face.surface.domain_v()) else {
        return false;
    };
    let pad = super::TRIM_SAMPLE_TOLERANCE;
    face.loops[0].coedges.iter().all(|c| {
        c.pcurve.control_points.iter().all(|h| {
            let Ok(p) = h.point() else {
                return false;
            };
            p.x >= u[0] - pad && p.x <= u[1] + pad && p.y >= v[0] - pad && p.y <= v[1] + pad
        })
    })
}

/// Positive B-spline basis bounds every trimmed point by its controls. Restrict
/// the UV wire to the four sides of the common height strip, including split
/// horizontal edges; interior material cannot leave that rectangle.
fn rectangle(face: &FaceRecord, lo: f64, hi: f64, band: f64) -> bool {
    if !bounded_wire(face) || hi <= lo {
        return false;
    }
    let mut covered = [false; 4];
    for c in &face.loops[0].coedges {
        let controls: Vec<_> = c
            .pcurve
            .control_points
            .iter()
            .map(|h| h.point().unwrap())
            .collect();
        if controls
            .iter()
            .any(|h| h.x < -band || h.x > 1. + band || h.y < lo - band || h.y > hi + band)
        {
            return false;
        }
        let candidates = [
            controls.iter().all(|h| h.x.abs() <= band),
            controls.iter().all(|h| (h.x - 1.).abs() <= band),
            controls.iter().all(|h| (h.y - lo).abs() <= band),
            controls.iter().all(|h| (h.y - hi).abs() <= band),
        ];
        let Some(side) = candidates.iter().position(|v| *v) else {
            return false;
        };
        covered[side] = true;
    }
    covered.into_iter().all(|v| v)
}

fn edge_curve(edge: &crate::topology::EdgeRecord) -> Option<NurbsCurve> {
    let mut curve = NurbsCurve::new(
        edge.curve.degree,
        edge.curve.knots.clone(),
        edge.curve.control_points.clone(),
    )
    .ok()?;
    let [a, b] = curve.domain().ok()?;
    if edge.t0 < a || edge.t1 > b || edge.t1 <= edge.t0 {
        return None;
    }
    // No near-end skip: if an exact trim cannot be extracted, decline.
    if edge.t0 > a {
        curve = curve.split(edge.t0).ok()?.1;
    }
    if edge.t1 < b {
        curve = curve.split(edge.t1).ok()?.0;
    }
    Some(curve)
}

/// Bound physical shared boundaries too: the watertight scanner consumes
/// those points rather than the UV wire alone. Local positive-weight control
/// hulls give a radial lower bound without sampling a high-degree fit.
fn physical(
    face: &FaceRecord,
    solid: &BrepSolid,
    zlo: f64,
    zhi: f64,
    radial: Option<(f64, f64)>,
    band: f64,
) -> bool {
    for c in face.loops.iter().flat_map(|l| &l.coedges) {
        let Some(e) = solid.edges.iter().find(|e| e.id == c.edge_id) else {
            return false;
        };
        let Some(curve) = edge_curve(e) else {
            return false;
        };
        let mut points = Vec::new();
        for h in &curve.control_points {
            if !h.w.is_finite() || !(0.1..=10.).contains(&h.w) {
                return false;
            }
            let Ok(p) = h.point() else { return false };
            if ![p.x, p.y, p.z].iter().all(|v| v.is_finite())
                || p.z < zlo - band
                || p.z > zhi + band
            {
                return false;
            }
            points.push(p);
        }
        if let Some((rlo, rhi)) = radial {
            if points.iter().any(|p| p.x.hypot(p.y) > rhi - band) {
                return false;
            }
            for i in curve.degree..points.len() {
                if curve.knots[i + 1] <= curve.knots[i] {
                    continue;
                }
                let hull = &points[i - curve.degree..=i];
                let x = hull.iter().map(|p| p.x).sum::<f64>();
                let y = hull.iter().map(|p| p.y).sum::<f64>();
                let r = x.hypot(y);
                if r <= band {
                    return false;
                }
                if hull.iter().any(|p| (p.x * x + p.y * y) / r < rlo + band) {
                    return false;
                }
            }
        }
    }
    true
}

/// Scope: exactly two horizontal caps, one native quadratic side and optionally
/// one coaxial cylindrical bore, one validating shell, standard scanner bands.
/// Every corner the scanner owns is in the proved height strip (UV material
/// and physical edges), including its trim-membership tolerance. Thus a side/cap pair cannot straddle a cap plane; caps cannot cross each
/// other; the quadratic side is injective by strict affine height and positive
/// radius. With a bore, every outer corner is strictly outside its cylinder,
/// so that pair cannot straddle either. No fold or closure reading is inferred
/// from recognition alone. Other supports/trims/options retain the full scan.
/// Structure every certified body has, read before the whole-body
/// `validate()` (most of a decline's cost): one shell of three or four faces,
/// one of them on a 9-row degree-2x2 net, the only kind `native` reports as a
/// general side. Necessary, never sufficient.
fn may_certify(solid: &BrepSolid) -> bool {
    solid.shells.len() == 1
        && (3..=4).contains(&solid.shells[0].faces.len())
        && solid.shells[0].faces.iter().any(|f| {
            f.surface.degree_u == 2 && f.surface.degree_v == 2 && f.surface.control_points.len() == 9
        })
}

pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    let proof = || -> Option<()> {
        if !may_certify(solid) || !solid.validate().is_empty() {
            return None;
        }
        let expected = SelfIntersectionOptions::for_solid(solid);
        if !floats_equal(
            &[
                options.chord_tolerance,
                options.straddle_band,
                options.min_crossing_length,
                options.weld,
            ],
            &[
                expected.chord_tolerance,
                expected.straddle_band,
                expected.min_crossing_length,
                expected.weld,
            ],
        ) || options.max_triangles != expected.max_triangles
        {
            return None;
        }
        let band = options.straddle_band / 128.;
        if !band.is_finite() || band <= 0. {
            return None;
        }
        let faces = &solid.shells[0].faces;
        let magnitude = faces
            .iter()
            .flat_map(|f| f.surface.control_points.iter().flatten())
            .try_fold(1.0f64, |m, h| {
                h.point()
                    .ok()
                    .map(|p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()))
            })?;
        if band <= 65536. * f64::EPSILON * magnitude {
            return None;
        }
        let mut caps = Vec::new();
        let mut sides = Vec::new();
        for f in faces {
            if !bounded_wire(f) {
                return None;
            }
            let s = &f.surface;
            let fresh = NurbsSurface::new(
                s.degree_u,
                s.degree_v,
                s.knots_u.clone(),
                s.knots_v.clone(),
                s.control_points.clone(),
            )
            .ok()?;
            if !cache_equal(s, &fresh) {
                return None;
            }
            let native = native(s, band / 16.);
            let h = s.control_points.first()?.first()?.point().ok()?.z;
            let planar = s.control_points.iter().flatten().all(|p| {
                p.w > 0. && p.w.is_finite() && p.point().is_ok_and(|p| (p.z - h).abs() < band / 16.)
            });
            if planar {
                let affine = s.is_affine().ok()?;
                if !cap_domain_wire(f) || (!affine && !native.is_some_and(|n| n.z0 == n.z1)) {
                    return None;
                }
                // Native polar caps charge constructor roundoff above. A raw
                // affine cap must be exactly horizontal in its stored net.
                if affine && s.control_points.iter().flatten().any(|p| p.z / p.w != h) {
                    return None;
                }
                if !physical(f, solid, h, h, None, band) {
                    return None;
                }
                caps.push(h);
            } else {
                sides.push((f, native?));
            }
        }
        if caps.len() != 2
            || sides.len() + 2 != faces.len()
            || sides.iter().filter(|(_, n)| n.general).count() != 1
        {
            return None;
        }
        caps.sort_by(f64::total_cmp);
        let (lo, hi) = (caps[0], caps[1]);
        if hi - lo <= band * 256. {
            return None;
        }
        for (f, n) in &sides {
            let dv = n.z1 - n.z0;
            // The scanner treats a parameter foot within this tolerance of
            // the wire as material. Charge that axial extension too.
            if dv * super::TRIM_SAMPLE_TOLERANCE + band * 2. > options.straddle_band / 8. {
                return None;
            }
            let (v0, v1) = ((lo - n.z0) / dv, (hi - n.z0) / dv);
            if v0 < 0. || v1 > 1. || !rectangle(f, v0, v1, band / (dv + n.rmax + 1.) / 16.) {
                return None;
            }
            if !physical(
                f,
                solid,
                lo,
                hi,
                Some((n.rmin / 2., n.rmax * 2. + band * 2.)),
                band,
            ) {
                return None;
            }
        }
        if sides.len() == 2 {
            let (_, outer) = sides.iter().find(|(_, n)| n.general)?;
            let (_, inner) = sides.iter().find(|(_, n)| !n.general)?;
            if inner.rmax - inner.rmin > band / 16.
                || outer.rmin / 2. <= inner.rmax * 2. + options.straddle_band * 16.
            {
                return None;
            }
        }
        Some(())
    };
    proof().is_some()
}


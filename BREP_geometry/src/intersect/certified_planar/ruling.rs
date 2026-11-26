//! Crossing evidence for exact polynomial extrusions parallel to Z.
//! Tolerant extrusion recognition and rounded projections are not used here.
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

#[derive(Clone, Debug)]
pub struct CertifiedRuling {
    profile_boxes: [[f64; 2]; 2],
    crossing_sign: i8,
    normal_factors: [i8; 2],
    profile_is_u: [bool; 2],
    sweep_sign: [i8; 2],
    ruling_xy: [f64; 2],
    source_profiles: [NurbsCurve; 2],
    curve: NurbsCurve,
}

struct ExactZ {
    profile: NurbsCurve,
    profile_is_u: bool,
    sweep_sign: i8,
    z: [f64; 2],
}

fn exact_z(surface: &NurbsSurface, profile_is_u: bool) -> Option<ExactZ> {
    surface.domain_u().ok()?;
    surface.domain_v().ok()?;
    let (degree, knots, sweep_degree, sweep_knots) = if profile_is_u {
        (
            surface.degree_u,
            &surface.knots_u,
            surface.degree_v,
            &surface.knots_v,
        )
    } else {
        (
            surface.degree_v,
            &surface.knots_v,
            surface.degree_u,
            &surface.knots_u,
        )
    };
    if sweep_degree != 1
        || sweep_knots.len() != 4
        || sweep_knots[0] != sweep_knots[1]
        || sweep_knots[2] != sweep_knots[3]
        || sweep_knots[0] >= sweep_knots[3]
    {
        return None;
    }
    let rows = &surface.control_points;
    let columns = rows.first()?.len();
    if columns == 0 || rows.iter().any(|row| row.len() != columns) {
        return None;
    }
    let (profiles, sweeps) = if profile_is_u {
        (rows.len(), columns)
    } else {
        (columns, rows.len())
    };
    if sweeps != 2 {
        return None;
    }
    let at = |i: usize, j: usize| if profile_is_u { rows[i][j] } else { rows[j][i] };
    let z = [at(0, 0).z, at(0, 1).z];
    if !z.iter().all(|x| x.is_finite()) || z[0] == z[1] {
        return None;
    }
    let mut controls = Vec::with_capacity(profiles);
    for i in 0..profiles {
        let (a, b) = (at(i, 0), at(i, 1));
        if a.w != 1.
            || b.w != 1.
            || a.x != b.x
            || a.y != b.y
            || a.z != z[0]
            || b.z != z[1]
            || ![a.x, a.y].iter().all(|x| x.is_finite())
        {
            return None;
        }
        // Selecting XY and replacing Z by exact zero introduces no rounding.
        controls.push(Vec4::from_point(Vec3::new(a.x, a.y, 0.), 1.));
    }
    Some(ExactZ {
        profile: NurbsCurve::new(degree, knots.clone(), controls).ok()?,
        profile_is_u,
        sweep_sign: if z[1] > z[0] { 1 } else { -1 },
        z,
    })
}

fn read(surface: &NurbsSurface) -> Option<ExactZ> {
    exact_z(surface, true).or_else(|| exact_z(surface, false))
}

/// Exact geometric identity, ignoring non-geometric caches and import flags.
pub fn same_curve(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && a.knots == b.knots
        && a.control_points.len() == b.control_points.len()
        && a.control_points
            .iter()
            .zip(&b.control_points)
            .all(|(a, b)| a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w)
}

impl CertifiedRuling {
    pub fn profile_boxes(&self) -> [[f64; 2]; 2] {
        self.profile_boxes
    }
    pub fn crossing_sign(&self) -> i8 {
        self.crossing_sign
    }
    pub fn normal_factors(&self) -> [i8; 2] {
        self.normal_factors
    }
    pub fn profile_is_u(&self) -> [bool; 2] {
        self.profile_is_u
    }
    pub fn sweep_sign(&self) -> [i8; 2] {
        self.sweep_sign
    }
    pub fn ruling_xy(&self) -> [f64; 2] {
        self.ruling_xy
    }
    pub fn source_profiles(&self) -> &[NurbsCurve; 2] {
        &self.source_profiles
    }
    pub fn curve(&self) -> &NurbsCurve {
        &self.curve
    }

    /// The supporting profiles and chart orientations must still be those
    /// certified. Tolerant matching or a nearest-foot reassociation is unsafe.
    pub fn matches_surfaces(&self, first: &NurbsSurface, second: &NurbsSurface) -> bool {
        let (Some(a), Some(b)) = (read(first), read(second)) else {
            return false;
        };
        let factors = [
            if a.profile_is_u {
                a.sweep_sign
            } else {
                -a.sweep_sign
            },
            if b.profile_is_u {
                b.sweep_sign
            } else {
                -b.sweep_sign
            },
        ];
        matches!(self.crossing_sign, -1 | 1)
            && self.normal_factors == factors
            && self.profile_is_u == [a.profile_is_u, b.profile_is_u]
            && self.sweep_sign == [a.sweep_sign, b.sweep_sign]
            && same_curve(&self.source_profiles[0], &a.profile)
            && same_curve(&self.source_profiles[1], &b.profile)
            && self.curve.control_points.iter().all(|p| {
                p.w == 1.
                    && p.x == self.ruling_xy[0]
                    && p.y == self.ruling_xy[1]
                    && p.z >= a.z[0].min(a.z[1])
                    && p.z <= a.z[0].max(a.z[1])
                    && p.z >= b.z[0].min(b.z[1])
                    && p.z <= b.z[0].max(b.z[1])
            })
    }

    /// Keep the original ruling instead of moving its controls to nearby
    /// canonical vertices. Only the curve/vertex incidence is tolerant; the
    /// curve carrying the crossing proof remains geometrically unchanged.
    pub fn retains_endpoints(
        &self,
        curve: &NurbsCurve,
        vertices: [Vec3; 2],
        tolerance: f64,
    ) -> bool {
        if !tolerance.is_finite() || tolerance <= 0. || !self.accepts_subcurve(curve) {
            return false;
        }
        let Ok([lo, hi]) = curve.domain() else {
            return false;
        };
        if curve.knots.as_slice() != [lo, lo, hi, hi] {
            return false;
        }
        curve.control_points.iter().zip(vertices).all(|(p, v)| {
            let coordinates = [(p.x, v.x), (p.y, v.y), (p.z, v.z)];
            if coordinates
                .iter()
                .any(|(a, b)| !a.is_finite() || !b.is_finite())
            {
                return false;
            }
            let mut squared = super::I::point(0.);
            for (a, b) in coordinates {
                // Normalize first: a tiny identity tolerance must not make
                // both squared distances underflow to zero and admit a gap.
                let Some(d) =
                    (super::I::point(a) - super::I::point(b)).divide(super::I::point(tolerance))
                else {
                    return false;
                };
                if !d.finite() {
                    return false;
                }
                squared = squared + d * d;
            }
            squared.finite() && squared.hi <= 1.
        })
    }

    /// A minted subpiece must retain this ruling exactly and stay inside
    /// its certified sweep overlap. Refits off the ruling lose the evidence.
    pub fn accepts_subcurve(&self, curve: &NurbsCurve) -> bool {
        if curve.degree != 1 || curve.control_points.len() != 2 {
            return false;
        }
        let (Some(a), Some(b)) = (
            self.curve.control_points.first(),
            self.curve.control_points.last(),
        ) else {
            return false;
        };
        curve.control_points.iter().all(|p| {
            p.w == 1.
                && p.x == self.ruling_xy[0]
                && p.y == self.ruling_xy[1]
                && p.z.is_finite()
                && p.z >= a.z.min(b.z)
                && p.z <= a.z.max(b.z)
        }) && curve.control_points[0].z != curve.control_points[1].z
    }
}

/// All complete-cover rulings with certified tangent signs, or None. This
/// narrow entry point changes no existing intersection route by itself.
pub fn sections(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<CertifiedRuling>> {
    let (a, b) = (read(first)?, read(second)?);
    let low = a.z[0].min(a.z[1]).max(b.z[0].min(b.z[1]));
    let high = a.z[0].max(a.z[1]).min(b.z[0].max(b.z[1]));
    if !tolerance.is_finite() || tolerance <= 0. || high - low <= tolerance {
        return None;
    }
    let roots = super::intersections_with_certificates(&a.profile, &b.profile, tolerance)?;
    if roots.is_empty() {
        return None;
    }
    let mut result = Vec::new();
    for root in roots {
        let sign = super::certified_crossing_sign(
            &a.profile,
            root.parameters[0],
            &b.profile,
            root.parameters[1],
            [0, 1],
        )?;
        let p = root.intersection.point;
        let curve = crate::make_line(Vec3::new(p.x, p.y, low), Vec3::new(p.x, p.y, high)).ok()?;
        result.push(CertifiedRuling {
            profile_boxes: root.parameters,
            crossing_sign: sign,
            normal_factors: [
                if a.profile_is_u {
                    a.sweep_sign
                } else {
                    -a.sweep_sign
                },
                if b.profile_is_u {
                    b.sweep_sign
                } else {
                    -b.sweep_sign
                },
            ],
            profile_is_u: [a.profile_is_u, b.profile_is_u],
            sweep_sign: [a.sweep_sign, b.sweep_sign],
            ruling_xy: [p.x, p.y],
            source_profiles: [a.profile.clone(), b.profile.clone()],
            curve,
        });
    }
    Some(result)
}


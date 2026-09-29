//! Coons fill with prescribed tangent ribbons.
//!
//! [`coons_with_ribbons`] is deliberately a SEPARATE constructor from
//! [`super::coons_bilinear`]: boundary interpolation and tangent-ribbon
//! interpolation are different promises with different preconditions, and
//! mixing them would let a caller believe a patch carries a cross-boundary
//! constraint it was never given.
//!
//! # The ribbon convention
//!
//! Boundary roles follow the caller's cyclic order and never move: boundary 0
//! is `v = 0`, boundary 1 is `u = 1`, boundary 2 is `v = 1`, boundary 3 is
//! `u = 0`.  Ribbon `k` is a vector-valued curve — a [`NurbsCurve`] whose
//! control points ARE the vectors — giving the patch's derivative with respect
//! to the parameter that crosses that boundary, in its INCREASING direction:
//!
//! * ribbons 0 and 2 are `dS/dv` along `v = 0` and `v = 1`;
//! * ribbons 3 and 1 are `dS/du` along `u = 0` and `u = 1`.
//!
//! Both parameters run `0 -> 1` across the patch, so two patches that share a
//! boundary share ONE ribbon field there, with no sign to get wrong.  Each
//! ribbon is parameterised over its own boundary's parameter as supplied; if
//! the constructor reverses that boundary it reverses the ribbon with it.
//!
//! # What is exact
//!
//! * With ribbons on ONE direction only (the other blended linearly), the
//!   prescribed cross-derivative field is interpolated EXACTLY — the correction
//!   term cancels the linear direction's own contribution identically, and no
//!   corner twist enters.  [`super::RibbonReport::cross_derivative_residual`]
//!   measures it on the built patch regardless.
//! * The boundaries stay exact exactly as in the bilinear constructor.
//!
//! # What is approximate, and why nothing here is called G1 unprompted
//!
//! With ribbons on BOTH directions each corner is handed two twist vectors —
//! `d(dS/dv)/du` from the v-ribbon and `d(dS/du)/dv` from the u-ribbon — and a
//! bicubically blended Coons patch has room for one.  This constructor carries
//! their mean and REPORTS the disagreement
//! ([`super::RibbonReport::twist_discrepancy`]); the cross-derivative residual
//! it then measures is the honest statement of what the patch achieved.  Read
//! that number before calling the join G1.
//!
//! # What refuses
//!
//! Rational input (a prescribed 3D cross-derivative does not determine a
//! homogeneous ribbon, so this is refused rather than approximated), half a
//! ribbon pair, and a ribbon whose end vector contradicts the tangent of the
//! boundary it meets there — without that agreement no patch can carry both.

use super::compat;
use super::{
    corner_band, report_skeleton, CoonsRefusal, CoonsReport, CompatWork, RibbonReport,
    BOUNDARY_SAMPLES,
};
use crate::{model_scale, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// Largest `|w - 1|` a curve may carry and still count as polynomial.
const POLYNOMIAL_WEIGHT_BAND: f64 = 1e-12;

/// The Coons patch through four boundaries that also interpolates the
/// prescribed cross-boundary derivative fields.
///
/// See the module documentation for the ribbon convention, the exactness
/// contract and the refusals.  With every ribbon `None` this is exactly
/// [`super::coons_bilinear_reported`].
pub fn coons_with_ribbons(
    boundaries: [NurbsCurve; 4],
    ribbons: [Option<NurbsCurve>; 4],
) -> Result<(NurbsSurface, CoonsReport), CoonsRefusal> {
    let v_prescribed = usize::from(ribbons[0].is_some()) + usize::from(ribbons[2].is_some());
    let u_prescribed = usize::from(ribbons[1].is_some()) + usize::from(ribbons[3].is_some());
    if v_prescribed == 1 {
        return Err(CoonsRefusal::RibbonPairing {
            direction: 'v',
            prescribed: v_prescribed,
        });
    }
    if u_prescribed == 1 {
        return Err(CoonsRefusal::RibbonPairing {
            direction: 'u',
            prescribed: u_prescribed,
        });
    }
    if v_prescribed == 0 && u_prescribed == 0 {
        return super::coons_bilinear_reported(boundaries);
    }
    build(boundaries, ribbons, v_prescribed == 2, u_prescribed == 2)
}

/// Euclidean point of a homogeneous control point.
fn point_of(control: Vec4) -> Vec3 {
    Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w)
}

/// Clamped Bezier knot vector of the given degree.
fn bezier_knots(degree: usize) -> Vec<f64> {
    let mut knots = vec![0.0; degree + 1];
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

/// Express a single Bezier segment's control points in the target degree and
/// knot vector, by elevation and knot insertion only.
///
/// This is how the cubic Hermite (and linear) blend functions enter the
/// patch's own tensor space without a fit: a polynomial of degree `n` lives in
/// every spline space of degree >= `n` over any knot vector, and elevation
/// plus insertion is the exact change of basis.
fn express(
    points: &[Vec3],
    target_degree: usize,
    target_knots: &[f64],
) -> Result<Vec<Vec3>, String> {
    let native_degree = points.len() - 1;
    let curve = NurbsCurve::new(
        native_degree,
        bezier_knots(native_degree),
        points
            .iter()
            .map(|point| Vec4::from_point(*point, 1.0))
            .collect(),
    )?;
    let elevated = compat::elevate_degree(&curve, target_degree)?;
    let carrier = NurbsCurve::new(
        target_degree,
        target_knots.to_vec(),
        vec![
            Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            };
            target_knots.len() - target_degree - 1
        ],
    )?;
    let (_, refined, _, _) = compat::merge_knots(&carrier, &elevated)?;
    Ok(refined.control_points.iter().copied().map(point_of).collect())
}

/// The cubic Hermite Bezier control points of a span with prescribed end
/// points and end derivatives (with respect to the span's own `[0, 1]`).
fn hermite(start: Vec3, start_derivative: Vec3, end: Vec3, end_derivative: Vec3) -> [Vec3; 4] {
    [
        start,
        start.add(start_derivative.scale(1.0 / 3.0)),
        end.sub(end_derivative.scale(1.0 / 3.0)),
        end,
    ]
}

/// Worst `|w - 1|` over a curve's control points.
fn weight_spread(curve: &NurbsCurve) -> f64 {
    curve
        .control_points
        .iter()
        .map(|control| (control.w - 1.0).abs())
        .fold(0.0f64, f64::max)
}

/// First derivative of a curve with respect to its own normalised parameter.
fn derivative_at(curve: &NurbsCurve, fraction: f64) -> Result<Vec3, String> {
    let [start, end] = curve.domain()?;
    let derivatives = curve.derivatives(start + (end - start) * fraction, 1)?;
    Ok(derivatives[1].scale(end - start))
}

fn build(
    boundaries: [NurbsCurve; 4],
    ribbons: [Option<NurbsCurve>; 4],
    v_hermite: bool,
    u_hermite: bool,
) -> Result<(NurbsSurface, CoonsReport), CoonsRefusal> {
    // Polynomial only: a prescribed 3D cross-derivative does not determine the
    // homogeneous ribbon a rational patch would need.
    for (index, curve) in boundaries.iter().enumerate() {
        let spread = weight_spread(curve);
        if spread > POLYNOMIAL_WEIGHT_BAND {
            return Err(CoonsRefusal::RationalRibbon {
                boundary: index,
                weight_spread: spread,
            });
        }
    }
    for (index, ribbon) in ribbons.iter().enumerate() {
        if let Some(curve) = ribbon {
            let spread = weight_spread(curve);
            if spread > POLYNOMIAL_WEIGHT_BAND {
                return Err(CoonsRefusal::RationalRibbon {
                    boundary: index,
                    weight_spread: spread,
                });
            }
        }
    }

    let oriented = super::orient_and_reconcile(&boundaries)?;
    let flip = |curve: &NurbsCurve, reversed: bool| -> Result<NurbsCurve, CoonsRefusal> {
        if reversed {
            curve.reversed().map_err(CoonsRefusal::Geometry)
        } else {
            Ok(curve.clone())
        }
    };
    // Ribbons follow their boundary's orientation; the transverse direction
    // they describe is untouched by that flip, so there is no sign change.
    let mut ribbon_curves: [Option<NurbsCurve>; 4] = [None, None, None, None];
    for index in 0..4 {
        if let Some(ribbon) = &ribbons[index] {
            ribbon_curves[index] = Some(flip(ribbon, oriented.reversed[index])?);
        }
    }
    let c0 = oriented.curves[0].clone();
    let d1 = oriented.curves[1].clone();
    let c1 = oriented.curves[2].reversed().map_err(CoonsRefusal::Geometry)?;
    let d0 = oriented.curves[3].reversed().map_err(CoonsRefusal::Geometry)?;
    let reverse_ribbon = |ribbon: &Option<NurbsCurve>| -> Result<Option<NurbsCurve>, CoonsRefusal> {
        match ribbon {
            Some(curve) => Ok(Some(curve.reversed().map_err(CoonsRefusal::Geometry)?)),
            None => Ok(None),
        }
    };
    // Along +u at v = 0 and v = 1; along +v at u = 0 and u = 1.
    let tv0 = ribbon_curves[0].clone();
    let tv1 = reverse_ribbon(&ribbon_curves[2])?;
    let tu0 = reverse_ribbon(&ribbon_curves[3])?;
    let tu1 = ribbon_curves[1].clone();

    // One compatible family per direction: the two boundaries plus, where the
    // direction is blended cubically, the two ribbons that travel with them.
    let mut u_family: Vec<NurbsCurve> = vec![c0, c1];
    if v_hermite {
        u_family.push(tv0.clone().expect("paired"));
        u_family.push(tv1.clone().expect("paired"));
    }
    let mut v_family: Vec<NurbsCurve> = vec![d0, d1];
    if u_hermite {
        v_family.push(tu0.clone().expect("paired"));
        v_family.push(tu1.clone().expect("paired"));
    }
    let u_native = if u_hermite { 3 } else { 1 };
    let v_native = if v_hermite { 3 } else { 1 };
    let (u_family, u_work) =
        compat::make_family_compatible(&u_family, u_native).map_err(|detail| {
            CoonsRefusal::Incompatible {
                direction: 'u',
                detail,
            }
        })?;
    let (v_family, v_work) =
        compat::make_family_compatible(&v_family, v_native).map_err(|detail| {
            CoonsRefusal::Incompatible {
                direction: 'v',
                detail,
            }
        })?;
    let mut work = [CompatWork::default(); 4];
    work[0] = u_work[0];
    work[2] = u_work[1];
    work[3] = v_work[0];
    work[1] = v_work[1];

    let degree_u = u_family[0].degree;
    let degree_v = v_family[0].degree;
    let knots_u = u_family[0].knots.clone();
    let knots_v = v_family[0].knots.clone();
    let rows = u_family[0].control_points.len();
    let columns = v_family[0].control_points.len();
    let net_of = |curve: &NurbsCurve| -> Vec<Vec3> {
        curve.control_points.iter().copied().map(point_of).collect()
    };
    let c0_net = net_of(&u_family[0]);
    let c1_net = net_of(&u_family[1]);
    let d0_net = net_of(&v_family[0]);
    let d1_net = net_of(&v_family[1]);
    let tv0_net = v_hermite.then(|| net_of(&u_family[2]));
    let tv1_net = v_hermite.then(|| net_of(&u_family[3]));
    let tu0_net = u_hermite.then(|| net_of(&v_family[2]));
    let tu1_net = u_hermite.then(|| net_of(&v_family[3]));

    // Corner points and the boundary tangents there.  These — not the ribbons
    // — are what the correction term must carry, so that it cancels the ruled
    // halves' own contribution identically along every boundary.
    let corner = |net: &[Vec3], last: bool| net[if last { net.len() - 1 } else { 0 }];
    let corners = [
        [corner(&c0_net, false), corner(&c1_net, false)],
        [corner(&c0_net, true), corner(&c1_net, true)],
    ];
    let tangent_u = [
        [
            derivative_at(&u_family[0], 0.0).map_err(CoonsRefusal::Geometry)?,
            derivative_at(&u_family[1], 0.0).map_err(CoonsRefusal::Geometry)?,
        ],
        [
            derivative_at(&u_family[0], 1.0).map_err(CoonsRefusal::Geometry)?,
            derivative_at(&u_family[1], 1.0).map_err(CoonsRefusal::Geometry)?,
        ],
    ];
    let tangent_v = [
        [
            derivative_at(&v_family[0], 0.0).map_err(CoonsRefusal::Geometry)?,
            derivative_at(&v_family[0], 1.0).map_err(CoonsRefusal::Geometry)?,
        ],
        [
            derivative_at(&v_family[1], 0.0).map_err(CoonsRefusal::Geometry)?,
            derivative_at(&v_family[1], 1.0).map_err(CoonsRefusal::Geometry)?,
        ],
    ];

    // A ribbon that contradicts the boundary tangent it meets cannot be
    // carried; measure the disagreement and refuse above the same vertex
    // identity band the corners were judged by.
    let mut points: Vec<Vec3> = Vec::new();
    for boundary in &boundaries {
        let [start, end] = boundary.domain().map_err(CoonsRefusal::Geometry)?;
        for index in 0..=BOUNDARY_SAMPLES {
            points.push(
                boundary
                    .evaluate(start + (end - start) * index as f64 / BOUNDARY_SAMPLES as f64)
                    .map_err(CoonsRefusal::Geometry)?,
            );
        }
    }
    let band = corner_band(model_scale(points.iter().copied()));
    let mut corner_tangent_gap = 0.0f64;
    if v_hermite {
        for (v_end, ribbon) in [(0usize, &u_family[2]), (1usize, &u_family[3])] {
            for (u_side, fraction) in [(0usize, 0.0f64), (1usize, 1.0f64)] {
                let [start, end] = ribbon.domain().map_err(CoonsRefusal::Geometry)?;
                let value = ribbon
                    .evaluate(start + (end - start) * fraction)
                    .map_err(CoonsRefusal::Geometry)?;
                let expected = tangent_v[u_side][v_end];
                let gap = value.sub(expected).length();
                corner_tangent_gap = corner_tangent_gap.max(gap);
                if gap > band {
                    return Err(CoonsRefusal::RibbonCornerTangent {
                        corner: u_side * 2 + v_end,
                        gap,
                        band,
                    });
                }
            }
        }
    }
    if u_hermite {
        for (u_side, ribbon) in [(0usize, &v_family[2]), (1usize, &v_family[3])] {
            for (v_end, fraction) in [(0usize, 0.0f64), (1usize, 1.0f64)] {
                let [start, end] = ribbon.domain().map_err(CoonsRefusal::Geometry)?;
                let value = ribbon
                    .evaluate(start + (end - start) * fraction)
                    .map_err(CoonsRefusal::Geometry)?;
                let expected = tangent_u[u_side][v_end];
                let gap = value.sub(expected).length();
                corner_tangent_gap = corner_tangent_gap.max(gap);
                if gap > band {
                    return Err(CoonsRefusal::RibbonCornerTangent {
                        corner: u_side * 2 + v_end,
                        gap,
                        band,
                    });
                }
            }
        }
    }

    // Corner twists.  Prescribed twice when both directions are cubic; the
    // patch carries the mean and the disagreement is reported, never hidden.
    let mut twist = [[Vec3::default(); 2]; 2];
    let mut twist_discrepancy = 0.0f64;
    if v_hermite && u_hermite {
        for side in 0..2 {
            for corner_index in 0..2 {
                // d(dS/dv)/du from the v-ribbon along v = corner_index.
                let from_v = derivative_at(
                    &u_family[2 + corner_index],
                    if side == 0 { 0.0 } else { 1.0 },
                )
                .map_err(CoonsRefusal::Geometry)?;
                // d(dS/du)/dv from the u-ribbon along u = side.
                let from_u = derivative_at(
                    &v_family[2 + side],
                    if corner_index == 0 { 0.0 } else { 1.0 },
                )
                .map_err(CoonsRefusal::Geometry)?;
                twist_discrepancy = twist_discrepancy.max(from_v.sub(from_u).length());
                twist[side][corner_index] = from_v.add(from_u).scale(0.5);
            }
        }
    }

    // ---- S1: blend the u-direction boundaries across v. ----
    let mut first: Vec<Vec<Vec3>> = Vec::with_capacity(rows);
    for row in 0..rows {
        let data: Vec<Vec3> = if v_hermite {
            hermite(
                c0_net[row],
                tv0_net.as_ref().expect("paired")[row],
                c1_net[row],
                tv1_net.as_ref().expect("paired")[row],
            )
            .to_vec()
        } else {
            vec![c0_net[row], c1_net[row]]
        };
        first.push(express(&data, degree_v, &knots_v).map_err(CoonsRefusal::Geometry)?);
    }
    // ---- S2: blend the v-direction boundaries across u. ----
    let mut second: Vec<Vec<Vec3>> = vec![vec![Vec3::default(); columns]; rows];
    for column in 0..columns {
        let data: Vec<Vec3> = if u_hermite {
            hermite(
                d0_net[column],
                tu0_net.as_ref().expect("paired")[column],
                d1_net[column],
                tu1_net.as_ref().expect("paired")[column],
            )
            .to_vec()
        } else {
            vec![d0_net[column], d1_net[column]]
        };
        let expressed = express(&data, degree_u, &knots_u).map_err(CoonsRefusal::Geometry)?;
        for (row, value) in expressed.into_iter().enumerate() {
            second[row][column] = value;
        }
    }
    // ---- S3: the corner correction, in its own native degrees first. ----
    let native_u = if u_hermite { 4 } else { 2 };
    let native_v = if v_hermite { 4 } else { 2 };
    let mut correction: Vec<Vec<Vec3>> = vec![vec![Vec3::default(); native_v]; native_u];
    for side in 0..2 {
        for corner_index in 0..2 {
            let row = if u_hermite { side * 3 } else { side };
            let column = if v_hermite { corner_index * 3 } else { corner_index };
            correction[row][column] = corners[side][corner_index];
        }
    }
    if u_hermite {
        for corner_index in 0..2 {
            let column = if v_hermite { corner_index * 3 } else { corner_index };
            correction[1][column] = corners[0][corner_index]
                .add(tangent_u[0][corner_index].scale(1.0 / 3.0));
            correction[2][column] = corners[1][corner_index]
                .sub(tangent_u[1][corner_index].scale(1.0 / 3.0));
        }
    }
    if v_hermite {
        for side in 0..2 {
            let row = if u_hermite { side * 3 } else { side };
            correction[row][1] =
                corners[side][0].add(tangent_v[side][0].scale(1.0 / 3.0));
            correction[row][2] =
                corners[side][1].sub(tangent_v[side][1].scale(1.0 / 3.0));
        }
    }
    if u_hermite && v_hermite {
        // The four interior cells carry both corner tangents and the twist.
        for side in 0..2 {
            for corner_index in 0..2 {
                let row = if side == 0 { 1 } else { 2 };
                let column = if corner_index == 0 { 1 } else { 2 };
                let u_sign = if side == 0 { 1.0 } else { -1.0 };
                let v_sign = if corner_index == 0 { 1.0 } else { -1.0 };
                correction[row][column] = corners[side][corner_index]
                    .add(tangent_u[side][corner_index].scale(u_sign / 3.0))
                    .add(tangent_v[side][corner_index].scale(v_sign / 3.0))
                    .add(twist[side][corner_index].scale(u_sign * v_sign / 9.0));
            }
        }
    }
    // Express the correction in the patch's tensor space: v first, then u.
    let mut correction_v: Vec<Vec<Vec3>> = Vec::with_capacity(native_u);
    for row in correction.iter() {
        correction_v.push(express(row, degree_v, &knots_v).map_err(CoonsRefusal::Geometry)?);
    }
    let mut third: Vec<Vec<Vec3>> = vec![vec![Vec3::default(); columns]; rows];
    for column in 0..columns {
        let data: Vec<Vec3> = correction_v.iter().map(|row| row[column]).collect();
        let expressed = express(&data, degree_u, &knots_u).map_err(CoonsRefusal::Geometry)?;
        for (row, value) in expressed.into_iter().enumerate() {
            third[row][column] = value;
        }
    }

    // ---- Assemble.  Boundary rows are assigned, never summed. ----
    let mut net: Vec<Vec<Vec4>> = vec![
        vec![
            Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            };
            columns
        ];
        rows
    ];
    for row in 0..rows {
        for column in 0..columns {
            let blended = first[row][column]
                .add(second[row][column])
                .sub(third[row][column]);
            net[row][column] = Vec4::from_point(blended, 1.0);
        }
    }
    for (row, control) in u_family[0].control_points.iter().enumerate() {
        net[row][0] = Vec4::from_point(point_of(*control), 1.0);
    }
    for (row, control) in u_family[1].control_points.iter().enumerate() {
        net[row][columns - 1] = Vec4::from_point(point_of(*control), 1.0);
    }
    for (column, control) in v_family[0].control_points.iter().enumerate() {
        net[0][column] = Vec4::from_point(point_of(*control), 1.0);
    }
    for (column, control) in v_family[1].control_points.iter().enumerate() {
        net[rows - 1][column] = Vec4::from_point(point_of(*control), 1.0);
    }
    net[0][0] = Vec4::from_point(corners[0][0], 1.0);
    net[rows - 1][0] = Vec4::from_point(corners[1][0], 1.0);
    net[0][columns - 1] = Vec4::from_point(corners[0][1], 1.0);
    net[rows - 1][columns - 1] = Vec4::from_point(corners[1][1], 1.0);

    let surface = NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, net)
        .map_err(CoonsRefusal::Geometry)?;

    // ---- Measure what the patch actually achieved. ----
    let mut residual = 0.0f64;
    let mut ribbon_scale = 0.0f64;
    let mut prescribed = [false; 4];
    let mut measure = |ribbon: &NurbsCurve,
                       along_u: bool,
                       at: f64,
                       surface: &NurbsSurface|
     -> Result<(), CoonsRefusal> {
        let [start, end] = ribbon.domain().map_err(CoonsRefusal::Geometry)?;
        for index in 0..=BOUNDARY_SAMPLES {
            let fraction = index as f64 / BOUNDARY_SAMPLES as f64;
            let expected = ribbon
                .evaluate(start + (end - start) * fraction)
                .map_err(CoonsRefusal::Geometry)?;
            let (u, v) = if along_u { (fraction, at) } else { (at, fraction) };
            let derivatives = surface.derivatives(u, v, 1).map_err(CoonsRefusal::Geometry)?;
            let achieved = if along_u {
                derivatives[0][1]
            } else {
                derivatives[1][0]
            };
            residual = residual.max(achieved.sub(expected).length());
            ribbon_scale = ribbon_scale.max(expected.length());
        }
        Ok(())
    };
    if v_hermite {
        prescribed[0] = true;
        prescribed[2] = true;
        measure(&u_family[2], true, 0.0, &surface)?;
        measure(&u_family[3], true, 1.0, &surface)?;
    }
    if u_hermite {
        prescribed[1] = true;
        prescribed[3] = true;
        measure(&v_family[2], false, 0.0, &surface)?;
        measure(&v_family[3], false, 1.0, &surface)?;
    }

    let mut report = report_skeleton(&oriented, work);
    report.degree_u = degree_u;
    report.degree_v = degree_v;
    report.control_rows = rows;
    report.control_columns = columns;
    report.min_weight = 1.0;
    report.boundary_residual = boundary_residual(&surface, &u_family, &v_family);
    report.ribbons = Some(RibbonReport {
        prescribed,
        corner_tangent_gap,
        twist_discrepancy,
        cross_derivative_residual: residual,
        cross_derivative_scale: ribbon_scale,
    });
    Ok((surface, report))
}

/// Worst distance between an assembled boundary row and the compatible
/// boundary curve it was built from.
fn boundary_residual(
    surface: &NurbsSurface,
    u_family: &[NurbsCurve],
    v_family: &[NurbsCurve],
) -> f64 {
    let net = &surface.control_points;
    let rows = net.len();
    let columns = net[0].len();
    let mut worst = 0.0f64;
    for row in 0..rows {
        worst = worst.max(
            point_of(net[row][0])
                .sub(point_of(u_family[0].control_points[row]))
                .length(),
        );
        worst = worst.max(
            point_of(net[row][columns - 1])
                .sub(point_of(u_family[1].control_points[row]))
                .length(),
        );
    }
    for column in 0..columns {
        worst = worst.max(
            point_of(net[0][column])
                .sub(point_of(v_family[0].control_points[column]))
                .length(),
        );
        worst = worst.max(
            point_of(net[rows - 1][column])
                .sub(point_of(v_family[1].control_points[column]))
                .length(),
        );
    }
    worst
}

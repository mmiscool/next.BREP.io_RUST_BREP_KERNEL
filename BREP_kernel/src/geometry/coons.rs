//! Four-boundary Coons surface fill.
//!
//! [`coons_bilinear`] takes four boundary curves in cyclic order and returns
//! the bilinearly blended Coons patch through them as ONE NURBS surface.
//! [`coons_with_ribbons`] is the separate constructor that additionally
//! interpolates prescribed cross-boundary derivative fields.  Geometry only:
//! no face, no naming, no feature, no schema.
//!
//! # What is exact
//!
//! * **The boundaries.**  The patch's four boundary iso-curves are the input
//!   curves themselves — same degree, same knots, same control points — after
//!   orientation normalisation and representation compatibility, which are
//!   exact algebra: reversal and affine knot re-mapping are bit-exact, degree
//!   elevation and knot insertion are exact to round-off (a curve that needed
//!   neither comes back with every float unchanged).  Nothing is sampled and
//!   refitted.  The one exception is
//!   the four corner control points, which are shared by two boundaries each
//!   and are set to the midpoint of the two incident endpoints; when the
//!   corners meet exactly (the ordinary case) that midpoint IS both endpoints
//!   and every boundary comes back bit for bit.  [`CoonsReport::boundary_residual`]
//!   is the measured worst disagreement, and it is `0.0` in that case.
//! * **Four straight lines** give exactly the bilinear patch through the four
//!   corners: degree (1, 1), a 2x2 control net.
//! * **The blend.**  The ruled halves and the corner bilinear are expressed in
//!   the patch's own knot vectors through the Greville abscissae — the closed
//!   form of "degree-elevate a linear function and insert knots" — so the
//!   interior is the Coons formula evaluated in NURBS arithmetic, not an
//!   approximation of it.
//!
//! # What is approximate
//!
//! * **The interior, against any surface you had in mind.**  A Coons patch is
//!   defined by its boundaries; it reproduces a sphere, a torus or anything
//!   else only to the extent that bilinear blending happens to agree with it.
//!   The tests record the deviation for a spherical quadrilateral rather than
//!   claiming a bound.
//! * **Rational inputs.**  With rational boundaries the blend is carried out
//!   in homogeneous space, so the interior is the projective Coons patch, not
//!   the Euclidean one.  The boundaries stay exact either way.  With all
//!   weights equal the two coincide.
//!
//! # What refuses
//!
//! Every refusal carries the number that was measured — see [`CoonsRefusal`].
//! A boundary that is a point, corners that do not meet inside the kernel's
//! vertex identity band, rational boundaries whose corner weights cannot be
//! reconciled, a blend that would put a control point at infinity, and — for
//! the ribbon constructor — rational input or a ribbon that contradicts the
//! boundary tangent it starts on.

use crate::{
    model_scale, KernelTolerances, NurbsCurve, NurbsSurface, Vec3, Vec4, VERTEX_MATCH_FLOOR,
};

#[path = "coons/compat.rs"]
pub(crate) mod compat;
#[path = "coons/ribbons.rs"]
mod ribbons;

pub use compat::CompatWork;
pub use ribbons::coons_with_ribbons;



/// Dimensionless part-per-diagonal factor of the corner-meeting band.  This is
/// the factor `BrepSolid::validate` uses for edge-endpoint-vs-vertex identity,
/// quoted here for the same question one stage earlier: two boundary ends that
/// meet inside this band ARE one corner, and a patch built on them can be sewn
/// without the validator then rejecting the very corner it was built from.
const CORNER_IDENTITY_K: f64 = 2e-5;

/// Relative band on the rational corner-weight closure (see
/// [`CoonsRefusal::CornerWeightClosure`]).  Dimensionless: it compares a ratio
/// against 1.
pub(crate) const WEIGHT_CLOSURE_BAND: f64 = 1e-9;

/// Uniform samples per boundary used to size the patch (`model_scale`) and to
/// measure achieved continuity.
const BOUNDARY_SAMPLES: usize = 32;

/// A Coons constructor's refusal, with the measurement that caused it.
///
/// Following the kernel's fail-safe contract: where the construction cannot be
/// certified it refuses, and the refusal carries the number a caller (or a
/// test, or a user) needs in order to act — never a bare "failed".
#[derive(Clone, Debug, PartialEq)]
pub enum CoonsRefusal {
    /// Boundary `boundary` has no extent: it is a point, not a curve.
    DegenerateBoundary {
        boundary: usize,
        extent: f64,
        band: f64,
    },
    /// Boundary `corner`'s end does not meet boundary `(corner + 1) % 4`'s
    /// start.  `gap` is the measured distance between them.
    CornerGap {
        corner: usize,
        gap: f64,
        band: f64,
    },
    /// The four boundaries are rational and their corner weights cannot all be
    /// matched by re-scaling: going once round the loop multiplies the weights
    /// by `ratio` instead of 1.  A Moebius re-parameterisation of one boundary
    /// would fix it, at the cost of that boundary's parameterisation — which
    /// this constructor will not change behind the caller's back.
    CornerWeightClosure { ratio: f64, band: f64 },
    /// The blend put a control point at or beyond infinity: the rational net
    /// weight at `(row, column)` is not positive.  Two opposite boundaries
    /// whose rational shoulder weights are small enough (arcs approaching a
    /// half turn) cancel the corner bilinear's weight entirely.
    NonPositiveWeight {
        row: usize,
        column: usize,
        weight: f64,
    },
    /// Degree/knot compatibility could not be reached exactly in the named
    /// direction.
    Incompatible { direction: char, detail: String },
    /// The ribbon constructor was handed rational geometry.  A prescribed 3D
    /// cross-derivative does not determine a homogeneous ribbon, so this is
    /// refused rather than approximated.  `weight_spread` is the measured
    /// `max |w - 1|` over the offending curve's control points.
    RationalRibbon {
        boundary: usize,
        weight_spread: f64,
    },
    /// A ribbon disagrees with the boundary tangent it starts or ends on, so
    /// no patch can carry both.  `gap` is the measured vector difference.
    RibbonCornerTangent {
        corner: usize,
        gap: f64,
        band: f64,
    },
    /// Ribbons were prescribed on one boundary of a direction but not its
    /// opposite.  A direction is blended cubically or linearly as a whole, so
    /// half a pair has no meaning; give both or neither.
    RibbonPairing { direction: char, prescribed: usize },
    /// An underlying kernel geometry error, verbatim.
    Geometry(String),
}

impl std::fmt::Display for CoonsRefusal {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DegenerateBoundary {
                boundary,
                extent,
                band,
            } => write!(
                formatter,
                "coons: boundary {boundary} is degenerate — extent {extent:.3e} is inside the \
                 identity band {band:.3e}"
            ),
            Self::CornerGap { corner, gap, band } => write!(
                formatter,
                "coons: boundary {corner} does not meet boundary {} — gap {gap:.3e} exceeds the \
                 vertex identity band {band:.3e}",
                (corner + 1) % 4
            ),
            Self::CornerWeightClosure { ratio, band } => write!(
                formatter,
                "coons: rational corner weights do not close — the loop multiplies weights by \
                 {ratio:.12} (band {band:.3e} about 1)"
            ),
            Self::NonPositiveWeight {
                row,
                column,
                weight,
            } => write!(
                formatter,
                "coons: blended control point ({row}, {column}) has non-positive weight \
                 {weight:.3e} — the patch would pass through infinity"
            ),
            Self::Incompatible { direction, detail } => {
                write!(formatter, "coons: {direction} direction incompatible — {detail}")
            }
            Self::RationalRibbon {
                boundary,
                weight_spread,
            } => write!(
                formatter,
                "coons: ribbon constructor is polynomial-only — boundary {boundary} is rational \
                 (max |w - 1| = {weight_spread:.3e})"
            ),
            Self::RibbonCornerTangent { corner, gap, band } => write!(
                formatter,
                "coons: the ribbon at corner {corner} contradicts the boundary tangent there — \
                 gap {gap:.3e} exceeds {band:.3e}"
            ),
            Self::RibbonPairing {
                direction,
                prescribed,
            } => write!(
                formatter,
                "coons: the {direction} direction has {prescribed} of its 2 ribbons — give both \
                 boundaries of a direction a ribbon, or neither"
            ),
            Self::Geometry(message) => formatter.write_str(message),
        }
    }
}

/// The lossy exit for stringly callers, exactly like [`crate::KernelRefusal`]'s.
impl From<CoonsRefusal> for String {
    fn from(refusal: CoonsRefusal) -> Self {
        refusal.to_string()
    }
}

/// What the constructor had to do, and what it measured doing it.
///
/// Nothing here is inferred: every field is either a decision the constructor
/// took (and can be replayed) or a number it measured.
#[derive(Clone, Debug)]
pub struct CoonsReport {
    /// Which boundaries were reversed to make the cyclic order run
    /// consistently round the patch.  Index is the caller's boundary index.
    pub reversed: [bool; 4],
    /// Distance from boundary `k`'s end to boundary `(k + 1) % 4`'s start,
    /// after orientation normalisation.
    pub corner_gaps: [f64; 4],
    /// The band those gaps were judged against.
    pub corner_band: f64,
    /// Factor each boundary's homogeneous control points were scaled by to
    /// make the corner weights agree.  All `1.0` for polynomial input.
    pub weight_scales: [f64; 4],
    /// Product of end weights over product of start weights: `1.0` when the
    /// rational corner weights close.
    pub weight_closure: f64,
    /// Degree elevation / knot insertion / domain re-mapping per boundary.
    pub work: [CompatWork; 4],
    pub degree_u: usize,
    pub degree_v: usize,
    pub control_rows: usize,
    pub control_columns: usize,
    /// Smallest rational weight in the blended control net.
    pub min_weight: f64,
    /// Worst distance between a boundary iso-curve's control point and the
    /// compatible input curve's — `0.0` when the boundaries came back bit for
    /// bit.  Homogeneous for [`coons_bilinear`]; Euclidean for
    /// [`coons_with_ribbons`], whose input is polynomial by contract, so the
    /// two coincide there.
    pub boundary_residual: f64,
    /// Ribbon measurements, present only for [`coons_with_ribbons`].
    pub ribbons: Option<RibbonReport>,
}

/// The measured continuity of a ribbon patch.  Read [`Self::cross_derivative_residual`]
/// before calling anything G1: it is the number that says whether the
/// prescribed cross-derivative field was actually met.
#[derive(Clone, Debug)]
pub struct RibbonReport {
    /// Which boundaries carried a prescribed ribbon.
    pub prescribed: [bool; 4],
    /// Worst disagreement between a ribbon's end vector and the tangent of the
    /// boundary it meets there.
    pub corner_tangent_gap: f64,
    /// Worst disagreement between the two corner twists a pair of ribbons
    /// prescribes at the same corner.  The patch carries their mean, so this
    /// is the quantity that limits exact ribbon interpolation.
    pub twist_discrepancy: f64,
    /// Worst `|dS/dn - T|` measured on the built patch along every boundary
    /// that carried a ribbon.
    pub cross_derivative_residual: f64,
    /// Worst `|T|` over the same samples — the scale
    /// [`Self::cross_derivative_residual`] should be read against.
    pub cross_derivative_scale: f64,
}

/// The bilinearly blended Coons patch through four boundary curves given in
/// cyclic order.
///
/// See the module documentation for the contract.  Use
/// [`coons_bilinear_reported`] when the measurements matter — which reversals
/// were applied, what the corners actually measured, whether the boundaries
/// came back bit for bit.
pub fn coons_bilinear(boundaries: [NurbsCurve; 4]) -> Result<NurbsSurface, CoonsRefusal> {
    coons_bilinear_reported(boundaries).map(|(surface, _)| surface)
}

/// [`coons_bilinear`] plus the [`CoonsReport`] of what it did and measured.
pub fn coons_bilinear_reported(
    boundaries: [NurbsCurve; 4],
) -> Result<(NurbsSurface, CoonsReport), CoonsRefusal> {
    let prepared = prepare(&boundaries)?;
    build_bilinear(prepared)
}

/// Homogeneous distance between two control points.
fn homogeneous_distance(first: Vec4, second: Vec4) -> f64 {
    ((first.x - second.x).powi(2)
        + (first.y - second.y).powi(2)
        + (first.z - second.z).powi(2)
        + (first.w - second.w).powi(2))
    .sqrt()
}

/// Midpoint of two homogeneous control points.  Exactly either one when they
/// are equal: `(a + a) * 0.5 == a` in IEEE arithmetic.
fn homogeneous_midpoint(first: Vec4, second: Vec4) -> Vec4 {
    first.add(second).scale(0.5)
}

/// Uniform 3D samples of a curve over its own domain.
fn samples(curve: &NurbsCurve, count: usize) -> Result<Vec<Vec3>, String> {
    let [start, end] = curve.domain()?;
    (0..=count)
        .map(|index| curve.evaluate(start + (end - start) * index as f64 / count as f64))
        .collect()
}

/// The corner-meeting band: the kernel's vertex identity band at this patch's
/// size.  Same expression `BrepSolid::validate` judges an edge end against its
/// vertex with — `heal_band(diagonal, 2e-5)` floored at [`VERTEX_MATCH_FLOOR`]
/// — so a patch whose corners this accepts is a patch the validator will
/// accept once it is sewn.
pub(crate) fn corner_band(scale: f64) -> f64 {
    KernelTolerances::default()
        .heal_band(scale, CORNER_IDENTITY_K)
        .max(VERTEX_MATCH_FLOOR)
}

/// The four boundaries, oriented into one cyclic traversal, weight-reconciled
/// and made representation-compatible in each direction.
pub(crate) struct Prepared {
    /// v = 0, running with +u.
    c0: NurbsCurve,
    /// v = 1, running with +u.
    c1: NurbsCurve,
    /// u = 0, running with +v.
    d0: NurbsCurve,
    /// u = 1, running with +v.
    d1: NurbsCurve,
    report: CoonsReport,
}

/// The cyclic traversal both constructors start from: four boundaries turned
/// the same way round the loop, their rational corner weights reconciled, and
/// the measurements that licensed it.
pub(crate) struct Oriented {
    /// `[0]` runs P00 -> P10, `[1]` P10 -> P11, `[2]` P11 -> P01, `[3]` P01 -> P00.
    pub(crate) curves: Vec<NurbsCurve>,
    pub(crate) reversed: [bool; 4],
    pub(crate) corner_gaps: [f64; 4],
    pub(crate) band: f64,
    pub(crate) weight_scales: [f64; 4],
    pub(crate) weight_closure: f64,
}

/// Orient, check and weight-reconcile — everything both constructors do before
/// they choose their blend.
pub(crate) fn orient_and_reconcile(
    boundaries: &[NurbsCurve; 4],
) -> Result<Oriented, CoonsRefusal> {
    let mut points: Vec<Vec3> = Vec::with_capacity(4 * (BOUNDARY_SAMPLES + 1));
    for boundary in boundaries {
        points.extend(samples(boundary, BOUNDARY_SAMPLES).map_err(CoonsRefusal::Geometry)?);
    }
    let scale = model_scale(points.iter().copied());
    let band = corner_band(scale);

    // A boundary with no extent cannot carry a patch side.
    for (index, boundary) in boundaries.iter().enumerate() {
        let boundary_points =
            samples(boundary, BOUNDARY_SAMPLES).map_err(CoonsRefusal::Geometry)?;
        let extent = model_scale(boundary_points.iter().copied());
        let spread = boundary_points
            .iter()
            .map(|point| point.sub(boundary_points[0]).length())
            .fold(0.0f64, f64::max);
        let extent = extent.min(spread);
        if extent <= band {
            return Err(CoonsRefusal::DegenerateBoundary {
                boundary: index,
                extent,
                band,
            });
        }
    }

    // Orientation: chain end-to-start round the loop.  Boundary 0's own
    // direction is chosen first, by which of its ends is nearer to boundary 1.
    let end_point = |curve: &NurbsCurve| -> Result<Vec3, CoonsRefusal> {
        let [_, end] = curve.domain().map_err(CoonsRefusal::Geometry)?;
        curve.evaluate(end).map_err(CoonsRefusal::Geometry)
    };
    let start_point = |curve: &NurbsCurve| -> Result<Vec3, CoonsRefusal> {
        let [start, _] = curve.domain().map_err(CoonsRefusal::Geometry)?;
        curve.evaluate(start).map_err(CoonsRefusal::Geometry)
    };
    let first_forward = end_point(&boundaries[0])?;
    let first_backward = start_point(&boundaries[0])?;
    let next_start = start_point(&boundaries[1])?;
    let next_end = end_point(&boundaries[1])?;
    let forward_gap = first_forward
        .sub(next_start)
        .length()
        .min(first_forward.sub(next_end).length());
    let backward_gap = first_backward
        .sub(next_start)
        .length()
        .min(first_backward.sub(next_end).length());
    let mut reversed = [false; 4];
    reversed[0] = backward_gap < forward_gap;

    let mut oriented: Vec<NurbsCurve> = Vec::with_capacity(4);
    oriented.push(if reversed[0] {
        boundaries[0].reversed().map_err(CoonsRefusal::Geometry)?
    } else {
        boundaries[0].clone()
    });
    let mut corner_gaps = [0.0f64; 4];
    for index in 1..4 {
        let previous_end = end_point(&oriented[index - 1])?;
        let candidate_start = start_point(&boundaries[index])?;
        let candidate_end = end_point(&boundaries[index])?;
        let forward = previous_end.sub(candidate_start).length();
        let backward = previous_end.sub(candidate_end).length();
        reversed[index] = backward < forward;
        corner_gaps[index - 1] = forward.min(backward);
        oriented.push(if reversed[index] {
            boundaries[index]
                .reversed()
                .map_err(CoonsRefusal::Geometry)?
        } else {
            boundaries[index].clone()
        });
    }
    corner_gaps[3] = end_point(&oriented[3])?
        .sub(start_point(&oriented[0])?)
        .length();
    for (corner, &gap) in corner_gaps.iter().enumerate() {
        if gap > band {
            return Err(CoonsRefusal::CornerGap { corner, gap, band });
        }
    }

    // Rational corner weights.  Two curves meeting at a corner only share ONE
    // homogeneous point when their weights there agree, and a curve may be
    // re-scaled freely (it is the same curve).  Propagating the scale round
    // the loop closes iff the end weights multiply to the start weights.
    let start_weight = |curve: &NurbsCurve| curve.control_points[0].w;
    let end_weight = |curve: &NurbsCurve| curve.control_points[curve.control_points.len() - 1].w;
    let mut weight_scales = [1.0f64; 4];
    for index in 1..4 {
        weight_scales[index] =
            weight_scales[index - 1] * end_weight(&oriented[index - 1]) / start_weight(&oriented[index]);
    }
    let weight_closure =
        weight_scales[3] * end_weight(&oriented[3]) / start_weight(&oriented[0]);
    if (weight_closure - 1.0).abs() > WEIGHT_CLOSURE_BAND {
        return Err(CoonsRefusal::CornerWeightClosure {
            ratio: weight_closure,
            band: WEIGHT_CLOSURE_BAND,
        });
    }
    for (index, curve) in oriented.iter_mut().enumerate() {
        if weight_scales[index] != 1.0 {
            let scaled = curve
                .control_points
                .iter()
                .map(|point| point.scale(weight_scales[index]))
                .collect();
            *curve = NurbsCurve::new(curve.degree, curve.knots.clone(), scaled)
                .map_err(CoonsRefusal::Geometry)?;
        }
    }

    Ok(Oriented {
        curves: oriented,
        reversed,
        corner_gaps,
        band,
        weight_scales,
        weight_closure,
    })
}

/// The report skeleton every constructor fills: what orientation measured,
/// with the blend-specific fields still to be written.
pub(crate) fn report_skeleton(oriented: &Oriented, work: [CompatWork; 4]) -> CoonsReport {
    CoonsReport {
        reversed: oriented.reversed,
        corner_gaps: oriented.corner_gaps,
        corner_band: oriented.band,
        weight_scales: oriented.weight_scales,
        weight_closure: oriented.weight_closure,
        work,
        degree_u: 0,
        degree_v: 0,
        control_rows: 0,
        control_columns: 0,
        min_weight: f64::INFINITY,
        boundary_residual: 0.0,
        ribbons: None,
    }
}

/// Orientation plus the two-curve compatibility the bilinear blend needs.
fn prepare(boundaries: &[NurbsCurve; 4]) -> Result<Prepared, CoonsRefusal> {
    let oriented = orient_and_reconcile(boundaries)?;
    // Chain corners: curves[0] runs P00 -> P10, [1] P10 -> P11,
    // [2] P11 -> P01, [3] P01 -> P00.
    let c0 = oriented.curves[0].clone();
    let d1 = oriented.curves[1].clone();
    let c1 = oriented.curves[2].reversed().map_err(CoonsRefusal::Geometry)?;
    let d0 = oriented.curves[3].reversed().map_err(CoonsRefusal::Geometry)?;

    let (u_family, u_work) = compat::make_family_compatible(&[c0, c1], 1).map_err(|detail| {
        CoonsRefusal::Incompatible {
            direction: 'u',
            detail,
        }
    })?;
    let (v_family, v_work) = compat::make_family_compatible(&[d0, d1], 1).map_err(|detail| {
        CoonsRefusal::Incompatible {
            direction: 'v',
            detail,
        }
    })?;

    // Work records go back into the CALLER's boundary order.
    let mut work = [CompatWork::default(); 4];
    work[0] = u_work[0];
    work[2] = u_work[1];
    work[3] = v_work[0];
    work[1] = v_work[1];

    let mut report = report_skeleton(&oriented, work);
    report.degree_u = u_family[0].degree;
    report.degree_v = v_family[0].degree;
    report.control_rows = u_family[0].control_points.len();
    report.control_columns = v_family[0].control_points.len();
    Ok(Prepared {
        c0: u_family[0].clone(),
        c1: u_family[1].clone(),
        d0: v_family[0].clone(),
        d1: v_family[1].clone(),
        report,
    })
}

/// Assemble the bilinearly blended patch from the prepared boundaries.
fn build_bilinear(prepared: Prepared) -> Result<(NurbsSurface, CoonsReport), CoonsRefusal> {
    let Prepared {
        c0,
        c1,
        d0,
        d1,
        mut report,
    } = prepared;
    let rows = c0.control_points.len();
    let columns = d0.control_points.len();
    let eta = compat::greville(&c0.knots, c0.degree);
    let xi = compat::greville(&d0.knots, d0.degree);

    // The four corners: one homogeneous point each, shared by two boundaries.
    let corner_00 = homogeneous_midpoint(c0.control_points[0], d0.control_points[0]);
    let corner_10 = homogeneous_midpoint(c0.control_points[rows - 1], d1.control_points[0]);
    let corner_01 = homogeneous_midpoint(c1.control_points[0], d0.control_points[columns - 1]);
    let corner_11 =
        homogeneous_midpoint(c1.control_points[rows - 1], d1.control_points[columns - 1]);

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
    for (row, net_row) in net.iter_mut().enumerate() {
        for (column, cell) in net_row.iter_mut().enumerate() {
            // S = ruled_v + ruled_u - bilinear, in homogeneous coordinates.
            let ruled_v = c0.control_points[row]
                .scale(1.0 - xi[column])
                .add(c1.control_points[row].scale(xi[column]));
            let ruled_u = d0.control_points[column]
                .scale(1.0 - eta[row])
                .add(d1.control_points[column].scale(eta[row]));
            let bilinear = corner_00
                .scale((1.0 - eta[row]) * (1.0 - xi[column]))
                .add(corner_10.scale(eta[row] * (1.0 - xi[column])))
                .add(corner_01.scale((1.0 - eta[row]) * xi[column]))
                .add(corner_11.scale(eta[row] * xi[column]));
            *cell = ruled_v.add(ruled_u).add(bilinear.scale(-1.0));
        }
    }
    // The boundary rows are ASSIGNED, not summed: `a + (b - a)` is not `b` in
    // floating point, and the boundaries are the one thing this constructor
    // promises exactly.
    for row in 0..rows {
        net[row][0] = c0.control_points[row];
        net[row][columns - 1] = c1.control_points[row];
    }
    for column in 0..columns {
        net[0][column] = d0.control_points[column];
        net[rows - 1][column] = d1.control_points[column];
    }
    net[0][0] = corner_00;
    net[rows - 1][0] = corner_10;
    net[0][columns - 1] = corner_01;
    net[rows - 1][columns - 1] = corner_11;

    let mut min_weight = f64::INFINITY;
    let mut worst = (0usize, 0usize);
    for (row, net_row) in net.iter().enumerate() {
        for (column, cell) in net_row.iter().enumerate() {
            if cell.w < min_weight {
                min_weight = cell.w;
                worst = (row, column);
            }
        }
    }
    if !(min_weight > 0.0) || !min_weight.is_finite() {
        return Err(CoonsRefusal::NonPositiveWeight {
            row: worst.0,
            column: worst.1,
            weight: min_weight,
        });
    }
    report.min_weight = min_weight;
    report.boundary_residual = boundary_residual(&net, &c0, &c1, &d0, &d1);

    let surface = NurbsSurface::new(
        c0.degree,
        d0.degree,
        c0.knots.clone(),
        d0.knots.clone(),
        net,
    )
    .map_err(CoonsRefusal::Geometry)?;
    Ok((surface, report))
}

/// Worst homogeneous distance between the assembled boundary rows and the
/// compatible boundary curves they were built from.  Non-zero only where two
/// boundaries disagreed about a corner.
fn boundary_residual(
    net: &[Vec<Vec4>],
    c0: &NurbsCurve,
    c1: &NurbsCurve,
    d0: &NurbsCurve,
    d1: &NurbsCurve,
) -> f64 {
    let rows = net.len();
    let columns = net[0].len();
    let mut worst = 0.0f64;
    for row in 0..rows {
        worst = worst.max(homogeneous_distance(net[row][0], c0.control_points[row]));
        worst = worst.max(homogeneous_distance(
            net[row][columns - 1],
            c1.control_points[row],
        ));
    }
    for column in 0..columns {
        worst = worst.max(homogeneous_distance(net[0][column], d0.control_points[column]));
        worst = worst.max(homogeneous_distance(
            net[rows - 1][column],
            d1.control_points[column],
        ));
    }
    worst
}

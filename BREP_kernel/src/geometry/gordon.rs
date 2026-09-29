//! Gordon surface through two crossing curve families.
//!
//! [`gordon_surface`] takes a family of u-curves (each runs in u at a
//! constant v) and a family of v-curves (each runs in v at a constant u) that
//! cross each other once per pair, and returns ONE NURBS surface that
//! interpolates every one of them.  Geometry only: no face, no naming, no
//! feature, no schema.
//!
//! # The construction
//!
//! The Boolean sum `S = L_v + L_u - T`, carried out in homogeneous space:
//!
//! * `L_v` skins the u-curves across v: `sum_a F_a(u) beta_a(v)`,
//! * `L_u` skins the v-curves across u: `sum_b G_b(v) alpha_b(u)`,
//! * `T` is the tensor interpolant of the crossing grid:
//!   `sum_a sum_b P_ab alpha_b(u) beta_a(v)`,
//!
//! where `alpha_b` and `beta_a` are the CARDINAL B-spline functions of the
//! crossing parameters — `alpha_b(u_c) = delta_bc` — of degree
//! `min(3, count - 1)` on the knot-averaging vector of those parameters.  All
//! three terms use the same cardinal functions, which is what makes the sum
//! interpolate the curves between the crossings and not only the crossings:
//! along `v = v_a` the `L_u` and `T` terms differ by
//! `sum_b alpha_b(u) (G_b(v_a) - P_ab)`, which vanishes when the curves meet.
//!
//! Two curves per family is degree-1 cardinal blending: the bilinear Coons
//! patch of [`crate::coons_bilinear`], reproduced to round-off.
//!
//! # Orientation
//!
//! The caller's u-curve 0 names +u and orders the v-curves; the caller's
//! v-curve 0 names +v and orders the u-curves.  Neither is ever reversed, so
//! the surface's handedness — and its normal `S_u x S_v` — follows those two
//! curves.  Every other curve is reversed and re-ordered to match, and
//! [`GordonReport`] says which.
//!
//! # What is exact
//!
//! * **Every input curve, to round-off.**  The cardinal functions are brought
//!   into the curves' own compatible knot structure by the Coons compatibility
//!   algebra — degree elevation and knot insertion, never sampling — so each
//!   iso-curve of the result at a network parameter is its input curve up to
//!   the arithmetic of one collocation solve.
//!   [`GordonReport::interpolation_bound`] is that residual, measured on the
//!   control nets; [`GordonReport::sampled_interpolation_residual`] measures
//!   it again with the kernel's surface and curve evaluators.
//! * **The four boundary curves, bit for bit** after orientation, weight
//!   reconciliation and compatibility, exactly as in the Coons fill: their
//!   rows are ASSIGNED into the net, and the four corners take the midpoint of
//!   the two incident boundary ends.
//! * **Any surface either skinning reproduces.**  `S` reproduces a surface
//!   when EITHER family's cardinal space contains it in the skinning
//!   direction: the iso-curves of a single-span bicubic reproduce it with four
//!   curves in one family and any number in the other, and the iso-curves of a
//!   biquadratic rational quadric patch reproduce it with three.
//!
//! # What is approximate
//!
//! * **The interior, against any surface you had in mind**, wherever neither
//!   cardinal space contains it.  A Gordon surface is defined by its network.
//! * **Rational networks** are blended in homogeneous space, so the interior
//!   is the projective Gordon surface.  Each curve's homogeneous scale is
//!   reconciled at the crossings first ([`GordonReport::u_weight_scales`]).
//! * **A network whose curves meet only within the band.**  An interior
//!   crossing's grid point is the midpoint of the two curves there, so both
//!   are off by half the gap.  A crossing ON a boundary curve is not split:
//!   the boundary is kept, and the interior curve absorbs the whole gap at its
//!   end.  A corner takes the midpoint of its two boundaries, as in the Coons
//!   fill, and — as there — a corner gap moves the blend near it.
//!   [`GordonReport::interpolation_bound`] and
//!   [`GordonReport::sampled_interpolation_residual`] report the result.
//!
//! # What refuses
//!
//! Everything is checked BEFORE the solve, and every refusal carries the
//! number it measured — see [`GordonRefusal`].  Out of scope, by name:
//! closed (periodic) curves, a pair that crosses more than once, a family
//! that does not cross the other in one consistent order, curves that run
//! past the network boundary, and crossings that do not share one parameter
//! per curve across a family (that needs a re-parameterisation, which this
//! constructor will not do behind the caller's back).  Interior regularity is
//! not checked: a network that folds builds a surface that folds.

use crate::coons::compat::{self, CompatWork};
use crate::coons::{corner_band, WEIGHT_CLOSURE_BAND};
use crate::{
    intersect_curves, model_scale, solve_dense, KnotVector, NurbsCurve, NurbsSurface, Vec3, Vec4,
    KNOT_IDENTITY_TOL,
};


/// Largest degree of a cardinal interpolation across a family.  Cubic is the
/// usual skinning degree; fewer curves lower it to `count - 1`, so two curves
/// blend linearly (Coons) and three quadratically.
const MAX_SKIN_DEGREE: usize = 3;

/// Uniform samples per curve used to size the network (`model_scale`), to
/// judge extents, and to measure the achieved interpolation.
const CURVE_SAMPLES: usize = 32;

/// Samples per curve for the closest-approach search that measures the gap of
/// a pair that does not meet.
const APPROACH_SAMPLES: usize = 96;

/// Newton iterations for polishing a crossing.  A transversal crossing
/// converges quadratically in a handful; the cap only bounds a pathology.
const CROSSING_ITERATIONS: usize = 60;

/// A Gordon constructor's refusal, with the measurement that caused it.
///
/// `family` is `'u'` for the `u_curves` argument and `'v'` for `v_curves`;
/// curve indices are the CALLER's indices into those slices.
#[derive(Clone, Debug, PartialEq)]
pub enum GordonRefusal {
    /// A family needs at least two curves: its two boundaries.
    TooFewCurves { family: char, count: usize },
    /// A caller-stated crossing band that is not a positive finite distance.
    InvalidBand { band: f64 },
    /// The curve has no extent: it is a point, not a curve.
    DegenerateCurve {
        family: char,
        curve: usize,
        extent: f64,
        band: f64,
    },
    /// The curve's start and end meet: a closed curve.  Periodic networks are
    /// out of scope — which of the coincident ends a crossing belongs to is
    /// not decidable from the curves.
    ClosedCurve {
        family: char,
        curve: usize,
        gap: f64,
        band: f64,
    },
    /// The two curves do not meet: `gap` is their measured closest approach.
    CurvesDoNotMeet {
        u_curve: usize,
        v_curve: usize,
        gap: f64,
        band: f64,
    },
    /// The two curves meet more than once inside the band.  `crossings` holds
    /// each `(u-curve parameter, v-curve parameter)` found.
    MultipleCrossings {
        u_curve: usize,
        v_curve: usize,
        crossings: Vec<(f64, f64)>,
        band: f64,
    },
    /// Along this curve the other family's crossings do not run in one order
    /// — neither increasing nor decreasing in the order the family's first
    /// curve sees them — so the two families do not form a grid.
    /// `parameters` are this curve's crossing parameters in that order.
    CrossingOrder {
        family: char,
        curve: usize,
        parameters: Vec<f64>,
    },
    /// Two curves of `family` cross the other family at the same parameter
    /// (`separation` apart): the skinning across them has no solution.
    CoincidentCrossings {
        family: char,
        first: usize,
        second: usize,
        separation: f64,
    },
    /// The curve runs past the network boundary: its crossing with the
    /// outermost curve of the other family at its `end` (0 = start, 1 = end,
    /// after orientation) sits at `parameter` instead of the end, and the
    /// overhang measures `gap` along the curve.
    Overhang {
        family: char,
        curve: usize,
        end: usize,
        parameter: f64,
        gap: f64,
        band: f64,
    },
    /// The two curves meet, but not at their families' shared crossing
    /// parameters: at those parameters they are `gap` apart.  A Gordon surface
    /// needs one parameter per crossing curve across the whole family;
    /// `u_parameter_offset` and `v_parameter_offset` say how far this pair's
    /// own crossing sits from the shared ones.
    InconsistentCrossing {
        u_curve: usize,
        v_curve: usize,
        gap: f64,
        band: f64,
        u_parameter_offset: f64,
        v_parameter_offset: f64,
    },
    /// The network is rational and no per-curve re-scaling makes the two
    /// curves' homogeneous weights agree at every crossing: at this pair the
    /// reconciled weights differ by `ratio`.
    WeightClosure {
        u_curve: usize,
        v_curve: usize,
        ratio: f64,
        band: f64,
    },
    /// The blend put a control point at or beyond infinity: the rational net
    /// weight at `(row, column)` is not positive.
    NonPositiveWeight {
        row: usize,
        column: usize,
        weight: f64,
    },
    /// Degree/knot compatibility, or the cardinal collocation, could not be
    /// reached in `family`.
    Incompatible { family: char, detail: String },
    /// An underlying kernel geometry error, verbatim.
    Geometry(String),
}

impl std::fmt::Display for GordonRefusal {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooFewCurves { family, count } => write!(
                formatter,
                "gordon: the {family} family has {count} curve(s) — a network needs at least two \
                 per family"
            ),
            Self::InvalidBand { band } => write!(
                formatter,
                "gordon: crossing band {band:.3e} is not a positive finite distance"
            ),
            Self::DegenerateCurve {
                family,
                curve,
                extent,
                band,
            } => write!(
                formatter,
                "gordon: {family}-curve {curve} is degenerate — extent {extent:.3e} is inside the \
                 band {band:.3e}"
            ),
            Self::ClosedCurve {
                family,
                curve,
                gap,
                band,
            } => write!(
                formatter,
                "gordon: {family}-curve {curve} is closed — its ends are {gap:.3e} apart, inside \
                 the band {band:.3e}; periodic networks are not supported"
            ),
            Self::CurvesDoNotMeet {
                u_curve,
                v_curve,
                gap,
                band,
            } => write!(
                formatter,
                "gordon: u-curve {u_curve} and v-curve {v_curve} do not meet — closest approach \
                 {gap:.3e} exceeds the crossing band {band:.3e}"
            ),
            Self::MultipleCrossings {
                u_curve,
                v_curve,
                crossings,
                band,
            } => write!(
                formatter,
                "gordon: u-curve {u_curve} and v-curve {v_curve} meet {} times within the band \
                 {band:.3e} (at {crossings:?}) — a network pair must cross once",
                crossings.len()
            ),
            Self::CrossingOrder {
                family,
                curve,
                parameters,
            } => write!(
                formatter,
                "gordon: along {family}-curve {curve} the crossings run out of order \
                 ({parameters:?}) — the families do not form a grid"
            ),
            Self::CoincidentCrossings {
                family,
                first,
                second,
                separation,
            } => write!(
                formatter,
                "gordon: {family}-curves {first} and {second} cross the other family at the same \
                 parameter ({separation:.3e} apart)"
            ),
            Self::Overhang {
                family,
                curve,
                end,
                parameter,
                gap,
                band,
            } => write!(
                formatter,
                "gordon: {family}-curve {curve} runs past the network boundary at its {} — the \
                 boundary crossing sits at parameter {parameter:.6}, {gap:.3e} along the curve \
                 (band {band:.3e})",
                if *end == 0 { "start" } else { "end" }
            ),
            Self::InconsistentCrossing {
                u_curve,
                v_curve,
                gap,
                band,
                u_parameter_offset,
                v_parameter_offset,
            } => write!(
                formatter,
                "gordon: u-curve {u_curve} and v-curve {v_curve} are {gap:.3e} apart at the \
                 families' shared crossing parameters (band {band:.3e}); their own crossing is \
                 offset by {u_parameter_offset:.3e} in u and {v_parameter_offset:.3e} in v"
            ),
            Self::WeightClosure {
                u_curve,
                v_curve,
                ratio,
                band,
            } => write!(
                formatter,
                "gordon: rational weights do not reconcile at u-curve {u_curve} x v-curve \
                 {v_curve} — ratio {ratio:.12} (band {band:.3e} about 1)"
            ),
            Self::NonPositiveWeight {
                row,
                column,
                weight,
            } => write!(
                formatter,
                "gordon: blended control point ({row}, {column}) has non-positive weight \
                 {weight:.3e} — the surface would pass through infinity"
            ),
            Self::Incompatible { family, detail } => {
                write!(formatter, "gordon: {family} family incompatible — {detail}")
            }
            Self::Geometry(message) => formatter.write_str(message),
        }
    }
}

/// The lossy exit for stringly callers, exactly like [`crate::CoonsRefusal`]'s.
impl From<GordonRefusal> for String {
    fn from(refusal: GordonRefusal) -> Self {
        refusal.to_string()
    }
}

/// What the constructor had to do, and what it measured doing it.
///
/// Per-curve vectors indexed by caller index say so; everything else about a
/// curve's POSITION is in network order — v increasing for u-curves, u
/// increasing for v-curves.
#[derive(Clone, Debug)]
pub struct GordonReport {
    /// Caller index of the u-curve at each network position (v increasing).
    pub u_order: Vec<usize>,
    /// Caller index of the v-curve at each network position (u increasing).
    pub v_order: Vec<usize>,
    /// Which u-curves were reversed to run with +u.  Caller order.
    pub u_reversed: Vec<bool>,
    /// Which v-curves were reversed to run with +v.  Caller order.
    pub v_reversed: Vec<bool>,
    /// The u at which each network v-curve sits: `0.0`, the shared interior
    /// crossing parameters, `1.0`.
    pub u_parameters: Vec<f64>,
    /// The v at which each network u-curve sits.
    pub v_parameters: Vec<f64>,
    /// The band every crossing was judged against.
    pub crossing_band: f64,
    /// Worst closest-approach distance of a pair at its own crossing.
    pub closest_approach: f64,
    /// The ADMITTED CROSSING MISS: the worst distance between a u-curve and a
    /// v-curve at the families' shared crossing parameters.  No surface passes
    /// through two curves that disagree there, so this is the input's own
    /// disagreement, and it is what separates "interpolated to round-off"
    /// (this reads round-off too) from "within the bound because the input
    /// missed by this much".  Read it beside [`Self::interpolation_bound`].
    pub crossing_gap: f64,
    /// Worst offset of a pair's own crossing parameter from the shared one.
    pub parameter_spread: f64,
    /// Factor each u-curve's homogeneous control points were scaled by to
    /// reconcile the weights at the crossings.  All `1.0` for polynomial
    /// input.  Caller order.
    pub u_weight_scales: Vec<f64>,
    /// As [`Self::u_weight_scales`], for the v-curves.
    pub v_weight_scales: Vec<f64>,
    /// The reconciled weight ratio farthest from 1 over every crossing.
    pub weight_closure: f64,
    /// Degree elevation / knot insertion / domain re-mapping per u-curve.
    pub u_work: Vec<CompatWork>,
    /// As [`Self::u_work`], for the v-curves.
    pub v_work: Vec<CompatWork>,
    /// Degree of the cardinal functions that skin the v-curves across u.
    pub skin_degree_u: usize,
    /// Degree of the cardinal functions that skin the u-curves across v.
    pub skin_degree_v: usize,
    pub degree_u: usize,
    pub degree_v: usize,
    pub control_rows: usize,
    pub control_columns: usize,
    /// Smallest rational weight in the blended control net.
    pub min_weight: f64,
    /// Worst homogeneous distance between the surface's iso-curve at a
    /// network parameter and its compatible (weight-reconciled) input curve,
    /// control point against control point.  The basis is a non-negative
    /// partition of unity, so this bounds the homogeneous curve deviation
    /// everywhere along the curve, not just at samples — in exact arithmetic.
    /// It is not a bound on [`Self::sampled_interpolation_residual`]: that
    /// reading is Euclidean and carries two evaluators' round-off, so at
    /// round-off scale it can read above this, and for rational input the
    /// Euclidean deviation can exceed the homogeneous one by up to
    /// `(1 + |point|) / weight`.  Round-off when
    /// [`Self::crossing_gap`] is; otherwise it carries that miss — half of it
    /// through an interior crossing, all of it at an interior curve's end on
    /// a boundary.
    pub interpolation_bound: f64,
    /// Worst Euclidean distance between the surface and an input curve,
    /// evaluated by the kernel's surface and curve evaluators at uniform
    /// samples of every curve — an independent reading of the same promise.
    pub sampled_interpolation_residual: f64,
}

/// The Gordon surface through two crossing curve families, judged against the
/// kernel's vertex identity band.
///
/// See the module documentation for the contract.  Use
/// [`gordon_surface_reported`] to state the crossing band and read what was
/// measured.
pub fn gordon_surface(
    u_curves: &[NurbsCurve],
    v_curves: &[NurbsCurve],
) -> Result<NurbsSurface, GordonRefusal> {
    gordon_surface_reported(u_curves, v_curves, None).map(|(surface, _)| surface)
}

/// [`gordon_surface`] with a caller-stated crossing band, plus the
/// [`GordonReport`] of what it did and measured.
///
/// `crossing_band` is the distance within which two curves meet.  `None`
/// takes the Coons corner band — the kernel's vertex identity band at the
/// network's size, `heal_band(diagonal, 2e-5)` floored at
/// [`crate::VERTEX_MATCH_FLOOR`] — so a network this accepts meets where the
/// validator would call two edge ends one vertex.
pub fn gordon_surface_reported(
    u_curves: &[NurbsCurve],
    v_curves: &[NurbsCurve],
    crossing_band: Option<f64>,
) -> Result<(NurbsSurface, GordonReport), GordonRefusal> {
    let network = prepare(u_curves, v_curves, crossing_band)?;
    build(network)
}

/// One family's curves in network order, oriented and weight-reconciled.
struct Family {
    /// Caller index at each network position.
    order: Vec<usize>,
    /// Oriented, domain-normalised curves in network order (unscaled).
    curves: Vec<NurbsCurve>,
    /// Caller order.
    reversed: Vec<bool>,
    /// Caller order.
    weight_scales: Vec<f64>,
    /// Caller order.
    work: Vec<CompatWork>,
    /// The parameter at which each curve of the OTHER family crosses this
    /// family, in the other family's network order.
    parameters: Vec<f64>,
}

/// Everything the solve needs, every check already passed.
struct Network {
    u: Family,
    v: Family,
    /// `crossings[a][b]`: the homogeneous grid point of network u-curve `a`
    /// and network v-curve `b` (see `prepare` for which curve's point).
    crossings: Vec<Vec<Vec4>>,
    band: f64,
    closest_approach: f64,
    crossing_gap: f64,
    parameter_spread: f64,
    weight_closure: f64,
}

/// Uniform 3D samples of a curve over its own domain.
fn samples(curve: &NurbsCurve, count: usize) -> Result<Vec<Vec3>, String> {
    let [start, end] = curve.domain()?;
    (0..=count)
        .map(|index| curve.evaluate(start + (end - start) * index as f64 / count as f64))
        .collect()
}

fn geometry(message: String) -> GordonRefusal {
    GordonRefusal::Geometry(message)
}

/// Homogeneous distance between two control points.
fn homogeneous_distance(first: Vec4, second: Vec4) -> f64 {
    ((first.x - second.x).powi(2)
        + (first.y - second.y).powi(2)
        + (first.z - second.z).powi(2)
        + (first.w - second.w).powi(2))
    .sqrt()
}

/// Midpoint of two homogeneous points.  Exactly either one when they are
/// equal: `(a + a) * 0.5 == a` in IEEE arithmetic.
fn homogeneous_midpoint(first: Vec4, second: Vec4) -> Vec4 {
    first.add(second).scale(0.5)
}

/// Polish a crossing of `first` and `second` from a seed by Newton on the
/// squared distance, run to full precision and clamped to both domains.
///
/// `intersect_curves` finds and counts crossings but stops at an absolute
/// 1e-12 residual; a network reproduced "to round-off" needs its crossing
/// parameters to the last ulp, so each accepted crossing is polished here.
/// A parameter pinned at a domain end drops to a one-variable Newton in the
/// other.  Every step is kept only if it does not increase the distance.
fn polish_crossing(
    first: &NurbsCurve,
    second: &NurbsCurve,
    seed_s: f64,
    seed_t: f64,
) -> Result<(f64, f64, f64), String> {
    let mut s = seed_s.clamp(0.0, 1.0);
    let mut t = seed_t.clamp(0.0, 1.0);
    let mut gap = first.evaluate(s)?.sub(second.evaluate(t)?).length();
    for _ in 0..CROSSING_ITERATIONS {
        let a = first.derivatives(s, 2)?;
        let b = second.derivatives(t, 2)?;
        let residual = a[0].sub(b[0]);
        let gradient_s = a[1].dot(residual);
        let gradient_t = -b[1].dot(residual);
        let h_ss = a[1].dot(a[1]) + a[2].dot(residual);
        let h_st = -a[1].dot(b[1]);
        let h_tt = b[1].dot(b[1]) - b[2].dot(residual);
        let determinant = h_ss * h_tt - h_st * h_st;
        let (mut step_s, mut step_t) = if determinant.abs() > f64::MIN_POSITIVE {
            (
                (-gradient_s * h_tt + gradient_t * h_st) / determinant,
                (-gradient_t * h_ss + gradient_s * h_st) / determinant,
            )
        } else {
            (0.0, 0.0)
        };
        let pinned_s = (s <= 0.0 && step_s < 0.0) || (s >= 1.0 && step_s > 0.0);
        let pinned_t = (t <= 0.0 && step_t < 0.0) || (t >= 1.0 && step_t > 0.0);
        if pinned_s && !pinned_t {
            step_s = 0.0;
            step_t = if h_tt.abs() > f64::MIN_POSITIVE { -gradient_t / h_tt } else { 0.0 };
        } else if pinned_t && !pinned_s {
            step_t = 0.0;
            step_s = if h_ss.abs() > f64::MIN_POSITIVE { -gradient_s / h_ss } else { 0.0 };
        } else if pinned_s && pinned_t {
            break;
        }
        let next_s = (s + step_s).clamp(0.0, 1.0);
        let next_t = (t + step_t).clamp(0.0, 1.0);
        if next_s == s && next_t == t {
            break;
        }
        let next_gap = first.evaluate(next_s)?.sub(second.evaluate(next_t)?).length();
        if next_gap > gap {
            break;
        }
        s = next_s;
        t = next_t;
        gap = next_gap;
    }
    Ok((s, t, gap))
}

/// The closest approach of two curves that `intersect_curves` found no
/// crossing for: the best pair of a dense sample grid, polished.
fn closest_approach(first: &NurbsCurve, second: &NurbsCurve) -> Result<(f64, f64, f64), String> {
    let first_points = samples(first, APPROACH_SAMPLES)?;
    let second_points = samples(second, APPROACH_SAMPLES)?;
    let mut best = (0usize, 0usize, f64::INFINITY);
    for (i, a) in first_points.iter().enumerate() {
        for (j, b) in second_points.iter().enumerate() {
            let distance = a.sub(*b).length();
            if distance < best.2 {
                best = (i, j, distance);
            }
        }
    }
    polish_crossing(
        first,
        second,
        best.0 as f64 / APPROACH_SAMPLES as f64,
        best.1 as f64 / APPROACH_SAMPLES as f64,
    )
}

/// Median of a non-empty slice.  For an exact network every entry is the same
/// float, and the median IS that float — a mean would re-round it.
fn median(values: &[f64]) -> f64 {
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let middle = sorted.len() / 2;
    if sorted.len() % 2 == 1 {
        sorted[middle]
    } else if sorted[middle - 1] == sorted[middle] {
        sorted[middle]
    } else {
        0.5 * (sorted[middle - 1] + sorted[middle])
    }
}

/// Decide the order of the crossing family along a curve, and whether the
/// curve runs against it.  `sequence` is the curve's crossing parameters in
/// the crossing family's network order.  `Ok(false)` keeps it, `Ok(true)`
/// reverses it.
fn orientation(
    family: char,
    curve: usize,
    sequence: &[f64],
    crossing_family: char,
    crossing_order: &[usize],
) -> Result<bool, GordonRefusal> {
    for (index, pair) in sequence.windows(2).enumerate() {
        let separation = (pair[1] - pair[0]).abs();
        if separation <= KNOT_IDENTITY_TOL {
            return Err(GordonRefusal::CoincidentCrossings {
                family: crossing_family,
                first: crossing_order[index],
                second: crossing_order[index + 1],
                separation,
            });
        }
    }
    if sequence.windows(2).all(|pair| pair[1] > pair[0]) {
        Ok(false)
    } else if sequence.windows(2).all(|pair| pair[1] < pair[0]) {
        Ok(true)
    } else {
        Err(GordonRefusal::CrossingOrder {
            family,
            curve,
            parameters: sequence.to_vec(),
        })
    }
}

/// Orient, order, check and reconcile the network.  Nothing is solved until
/// every refusal here has been passed.
fn prepare(
    u_input: &[NurbsCurve],
    v_input: &[NurbsCurve],
    crossing_band: Option<f64>,
) -> Result<Network, GordonRefusal> {
    for (family, curves) in [('u', u_input), ('v', v_input)] {
        if curves.len() < 2 {
            return Err(GordonRefusal::TooFewCurves {
                family,
                count: curves.len(),
            });
        }
    }
    if let Some(band) = crossing_band {
        if !(band > 0.0 && band.is_finite()) {
            return Err(GordonRefusal::InvalidBand { band });
        }
    }

    // Domains onto [0, 1] — exact, and the frame every parameter below is in.
    let (u_curves, u_work) = normalise_domains(u_input)?;
    let (v_curves, v_work) = normalise_domains(v_input)?;
    let mut points: Vec<Vec3> = Vec::new();
    for curve in u_curves.iter().chain(&v_curves) {
        points.extend(samples(curve, CURVE_SAMPLES).map_err(geometry)?);
    }
    let band = crossing_band.unwrap_or_else(|| corner_band(model_scale(points.iter().copied())));
    judge_curves('u', &u_curves, band)?;
    judge_curves('v', &v_curves, band)?;

    let crossings = find_crossings(&u_curves, &v_curves, band)?;
    let order = order_network(&crossings)?;

    // Network order, oriented, crossings re-polished on the curves actually
    // used: a reversed curve's knots are `1 - k`, re-rounded.
    let f = oriented(&u_curves, &order.u_order, &order.u_reversed)?;
    let g = oriented(&v_curves, &order.v_order, &order.v_reversed)?;
    let (rows, columns) = (f.len(), g.len());
    // `own_s[a][b]` / `own_t[a][b]`: network pair (a, b)'s own crossing.
    let mut own_s = vec![vec![0.0f64; columns]; rows];
    let mut own_t = vec![vec![0.0f64; columns]; rows];
    for a in 0..rows {
        for b in 0..columns {
            let (j, i) = (order.u_order[a], order.v_order[b]);
            let (s, t) = (crossings.s[j][i], crossings.t[j][i]);
            let seed_s = if order.u_reversed[j] { 1.0 - s } else { s };
            let seed_t = if order.v_reversed[i] { 1.0 - t } else { t };
            let (polished_s, polished_t, _) =
                polish_crossing(&f[a], &g[b], seed_s, seed_t).map_err(geometry)?;
            own_s[a][b] = polished_s;
            own_t[a][b] = polished_t;
        }
    }

    // Shared parameters: the network's ends by contract, the median of the
    // family's own crossings in between.
    let u_parameters = shared_parameters(columns, |b| (0..rows).map(|a| own_s[a][b]).collect());
    let v_parameters = shared_parameters(rows, |a| (0..columns).map(|b| own_t[a][b]).collect());
    for (family, parameters, network_order) in [
        ('v', &u_parameters, &order.v_order),
        ('u', &v_parameters, &order.u_order),
    ] {
        for (index, pair) in parameters.windows(2).enumerate() {
            let separation = pair[1] - pair[0];
            if separation <= KNOT_IDENTITY_TOL {
                return Err(GordonRefusal::CoincidentCrossings {
                    family,
                    first: network_order[index],
                    second: network_order[index + 1],
                    separation,
                });
            }
        }
    }
    let shared = SharedCrossings {
        f: &f,
        g: &g,
        u_parameters: &u_parameters,
        v_parameters: &v_parameters,
        own_s: &own_s,
        own_t: &own_t,
        u_order: &order.u_order,
        v_order: &order.v_order,
    };
    let (crossing_gap, parameter_spread) = judge_shared_crossings(&shared, band)?;
    let (f_scales, g_scales, weight_closure) = reconcile_weights(&shared)?;
    let f_scaled = scaled(&f, &f_scales)?;
    let g_scaled = scaled(&g, &g_scales)?;
    let grid = crossing_grid(&f_scaled, &g_scaled, &u_parameters, &v_parameters)?;

    // Report vectors go back to caller order.
    let mut u_weight_scales = vec![1.0f64; rows];
    for (a, &j) in order.u_order.iter().enumerate() {
        u_weight_scales[j] = f_scales[a];
    }
    let mut v_weight_scales = vec![1.0f64; columns];
    for (b, &i) in order.v_order.iter().enumerate() {
        v_weight_scales[i] = g_scales[b];
    }
    // `u_work` / `v_work` carry the domain records; `build` adds the
    // elevation and insertion counts.
    Ok(Network {
        u: Family {
            order: order.u_order,
            curves: f_scaled,
            reversed: order.u_reversed,
            weight_scales: u_weight_scales,
            work: u_work,
            parameters: u_parameters,
        },
        v: Family {
            order: order.v_order,
            curves: g_scaled,
            reversed: order.v_reversed,
            weight_scales: v_weight_scales,
            work: v_work,
            parameters: v_parameters,
        },
        crossings: grid,
        band,
        closest_approach: crossings.closest,
        crossing_gap,
        parameter_spread,
        weight_closure,
    })
}

/// Re-map every curve's knot domain onto `[0, 1]`, recording which moved.
fn normalise_domains(
    curves: &[NurbsCurve],
) -> Result<(Vec<NurbsCurve>, Vec<CompatWork>), GordonRefusal> {
    let mut normalised = Vec::with_capacity(curves.len());
    let mut work = vec![CompatWork::default(); curves.len()];
    for (index, curve) in curves.iter().enumerate() {
        let (moved, remapped) = compat::normalise_domain(curve).map_err(geometry)?;
        work[index].domain_remapped = remapped;
        normalised.push(moved);
    }
    Ok((normalised, work))
}

/// Refuse a point-like or closed curve.
fn judge_curves(family: char, curves: &[NurbsCurve], band: f64) -> Result<(), GordonRefusal> {
    for (index, curve) in curves.iter().enumerate() {
        let points = samples(curve, CURVE_SAMPLES).map_err(geometry)?;
        let spread = points
            .iter()
            .map(|point| point.sub(points[0]).length())
            .fold(0.0f64, f64::max);
        let extent = model_scale(points.iter().copied()).min(spread);
        if extent <= band {
            return Err(GordonRefusal::DegenerateCurve {
                family,
                curve: index,
                extent,
                band,
            });
        }
        let gap = points[0].sub(points[points.len() - 1]).length();
        if gap <= band {
            return Err(GordonRefusal::ClosedCurve {
                family,
                curve: index,
                gap,
                band,
            });
        }
    }
    Ok(())
}

/// Every caller pair's single crossing.
struct Crossings {
    /// `s[j][i]`: the parameter on caller u-curve `j` where it meets caller
    /// v-curve `i`.
    s: Vec<Vec<f64>>,
    /// `t[j][i]`: the parameter on caller v-curve `i`.
    t: Vec<Vec<f64>>,
    /// Worst closest-approach distance over every pair.
    closest: f64,
}

/// Find the one crossing of every u-curve with every v-curve, or refuse the
/// pair that has none or several.
fn find_crossings(
    u_curves: &[NurbsCurve],
    v_curves: &[NurbsCurve],
    band: f64,
) -> Result<Crossings, GordonRefusal> {
    let mut crossings = Crossings {
        s: vec![vec![0.0f64; v_curves.len()]; u_curves.len()],
        t: vec![vec![0.0f64; v_curves.len()]; u_curves.len()],
        closest: 0.0,
    };
    for (j, u_curve) in u_curves.iter().enumerate() {
        for (i, v_curve) in v_curves.iter().enumerate() {
            let hits = intersect_curves(u_curve, v_curve, band).map_err(geometry)?;
            let (s, t, gap) = match hits.len() {
                1 => polish_crossing(u_curve, v_curve, hits[0].s, hits[0].t).map_err(geometry)?,
                0 => {
                    let approach = closest_approach(u_curve, v_curve).map_err(geometry)?;
                    if approach.2 > band {
                        return Err(GordonRefusal::CurvesDoNotMeet {
                            u_curve: j,
                            v_curve: i,
                            gap: approach.2,
                            band,
                        });
                    }
                    approach
                }
                _ => {
                    return Err(GordonRefusal::MultipleCrossings {
                        u_curve: j,
                        v_curve: i,
                        crossings: hits.iter().map(|hit| (hit.s, hit.t)).collect(),
                        band,
                    })
                }
            };
            crossings.s[j][i] = s;
            crossings.t[j][i] = t;
            crossings.closest = crossings.closest.max(gap);
        }
    }
    Ok(crossings)
}

/// The network order and each curve's orientation.
struct NetworkOrder {
    /// Caller index of the u-curve at each network position.
    u_order: Vec<usize>,
    v_order: Vec<usize>,
    /// Caller order.
    u_reversed: Vec<bool>,
    v_reversed: Vec<bool>,
}

/// Caller u-curve 0 names +u and orders the v-curves; caller v-curve 0 names
/// +v and orders the u-curves.  Every other curve must see the other family in
/// that order or its reverse.
fn order_network(crossings: &Crossings) -> Result<NetworkOrder, GordonRefusal> {
    let (s, t) = (&crossings.s, &crossings.t);
    let (rows, columns) = (s.len(), s[0].len());
    let mut v_order: Vec<usize> = (0..columns).collect();
    v_order.sort_by(|&a, &b| s[0][a].total_cmp(&s[0][b]));
    let mut u_order: Vec<usize> = (0..rows).collect();
    u_order.sort_by(|&a, &b| t[a][0].total_cmp(&t[b][0]));
    let mut u_reversed = vec![false; rows];
    for (j, reversed) in u_reversed.iter_mut().enumerate() {
        let sequence: Vec<f64> = v_order.iter().map(|&i| s[j][i]).collect();
        *reversed = orientation('u', j, &sequence, 'v', &v_order)?;
    }
    let mut v_reversed = vec![false; columns];
    for (i, reversed) in v_reversed.iter_mut().enumerate() {
        let sequence: Vec<f64> = u_order.iter().map(|&j| t[j][i]).collect();
        *reversed = orientation('v', i, &sequence, 'u', &u_order)?;
    }
    Ok(NetworkOrder {
        u_order,
        v_order,
        u_reversed,
        v_reversed,
    })
}

/// The caller's curves in network order, reversed where the order says.
fn oriented(
    curves: &[NurbsCurve],
    order: &[usize],
    reversed: &[bool],
) -> Result<Vec<NurbsCurve>, GordonRefusal> {
    order
        .iter()
        .map(|&index| {
            if reversed[index] {
                curves[index].reversed().map_err(geometry)
            } else {
                Ok(curves[index].clone())
            }
        })
        .collect()
}

/// `0.0`, then the median of each interior position's own crossings, then
/// `1.0`.
fn shared_parameters(count: usize, own: impl Fn(usize) -> Vec<f64>) -> Vec<f64> {
    (0..count)
        .map(|index| {
            if index == 0 {
                0.0
            } else if index == count - 1 {
                1.0
            } else {
                median(&own(index))
            }
        })
        .collect()
}

/// The oriented network at its shared crossing parameters.
struct SharedCrossings<'a> {
    f: &'a [NurbsCurve],
    g: &'a [NurbsCurve],
    u_parameters: &'a [f64],
    v_parameters: &'a [f64],
    own_s: &'a [Vec<f64>],
    own_t: &'a [Vec<f64>],
    u_order: &'a [usize],
    v_order: &'a [usize],
}

/// Judge every pair at the shared parameters and return the worst gap and
/// parameter offset.  Overhangs first — a boundary crossing that sits inside a
/// curve is a curve running past the network — then the WORST inconsistent
/// pair, so the refusal names the culprit rather than the first pair it moved.
fn judge_shared_crossings(
    shared: &SharedCrossings<'_>,
    band: f64,
) -> Result<(f64, f64), GordonRefusal> {
    let (rows, columns) = (shared.f.len(), shared.g.len());
    let mut crossing_gap = 0.0f64;
    let mut spread = 0.0f64;
    let mut worst: Option<GordonRefusal> = None;
    for a in 0..rows {
        for b in 0..columns {
            let own_s = shared.own_s[a][b];
            let own_t = shared.own_t[a][b];
            let on_u = shared.f[a].evaluate(shared.u_parameters[b]).map_err(geometry)?;
            let on_v = shared.g[b].evaluate(shared.v_parameters[a]).map_err(geometry)?;
            let gap = on_u.sub(on_v).length();
            let u_offset = (own_s - shared.u_parameters[b]).abs();
            let v_offset = (own_t - shared.v_parameters[a]).abs();
            if b == 0 || b == columns - 1 {
                let along = on_u.sub(shared.f[a].evaluate(own_s).map_err(geometry)?).length();
                if along > band {
                    return Err(GordonRefusal::Overhang {
                        family: 'u',
                        curve: shared.u_order[a],
                        end: usize::from(b == columns - 1),
                        parameter: own_s,
                        gap: along,
                        band,
                    });
                }
            }
            if a == 0 || a == rows - 1 {
                let along = on_v.sub(shared.g[b].evaluate(own_t).map_err(geometry)?).length();
                if along > band {
                    return Err(GordonRefusal::Overhang {
                        family: 'v',
                        curve: shared.v_order[b],
                        end: usize::from(a == rows - 1),
                        parameter: own_t,
                        gap: along,
                        band,
                    });
                }
            }
            if gap > band && gap > crossing_gap {
                worst = Some(GordonRefusal::InconsistentCrossing {
                    u_curve: shared.u_order[a],
                    v_curve: shared.v_order[b],
                    gap,
                    band,
                    u_parameter_offset: u_offset,
                    v_parameter_offset: v_offset,
                });
            }
            crossing_gap = crossing_gap.max(gap);
            spread = spread.max(u_offset).max(v_offset);
        }
    }
    match worst {
        Some(refusal) => Err(refusal),
        None => Ok((crossing_gap, spread)),
    }
}

/// Per-curve homogeneous scales that make the two curves' weights agree at
/// every crossing, and the reconciled ratio farthest from 1.
///
/// Two curves share ONE homogeneous crossing point only when their weights
/// there agree, and a curve may be re-scaled freely.  Network u-curve 0 keeps
/// scale 1, every v-curve is scaled against it, every other u-curve against
/// v-curve 0, and then every remaining crossing must close.  Polynomial input
/// is left untouched, bit for bit.
fn reconcile_weights(
    shared: &SharedCrossings<'_>,
) -> Result<(Vec<f64>, Vec<f64>, f64), GordonRefusal> {
    let (rows, columns) = (shared.f.len(), shared.g.len());
    let mut f_scales = vec![1.0f64; rows];
    let mut g_scales = vec![1.0f64; columns];
    let polynomial = shared
        .f
        .iter()
        .chain(shared.g)
        .all(|curve| curve.control_points.iter().all(|point| point.w == 1.0));
    if polynomial {
        return Ok((f_scales, g_scales, 1.0));
    }
    let mut f_weight = vec![vec![1.0f64; columns]; rows];
    let mut g_weight = vec![vec![1.0f64; columns]; rows];
    for a in 0..rows {
        for b in 0..columns {
            f_weight[a][b] = shared.f[a]
                .evaluate_homogeneous(shared.u_parameters[b])
                .map_err(geometry)?
                .w;
            g_weight[a][b] = shared.g[b]
                .evaluate_homogeneous(shared.v_parameters[a])
                .map_err(geometry)?
                .w;
        }
    }
    for b in 0..columns {
        g_scales[b] = f_weight[0][b] / g_weight[0][b];
    }
    for a in 1..rows {
        f_scales[a] = g_scales[0] * g_weight[a][0] / f_weight[a][0];
    }
    let mut weight_closure = 1.0f64;
    for a in 0..rows {
        for b in 0..columns {
            let ratio = f_scales[a] * f_weight[a][b] / (g_scales[b] * g_weight[a][b]);
            if (ratio - 1.0).abs() > (weight_closure - 1.0).abs() {
                weight_closure = ratio;
            }
            if (ratio - 1.0).abs() > WEIGHT_CLOSURE_BAND {
                return Err(GordonRefusal::WeightClosure {
                    u_curve: shared.u_order[a],
                    v_curve: shared.v_order[b],
                    ratio,
                    band: WEIGHT_CLOSURE_BAND,
                });
            }
        }
    }
    Ok((f_scales, g_scales, weight_closure))
}

/// Each curve's homogeneous control points times its scale; a scale of
/// exactly 1 returns the curve unchanged.
fn scaled(curves: &[NurbsCurve], scales: &[f64]) -> Result<Vec<NurbsCurve>, GordonRefusal> {
    curves
        .iter()
        .zip(scales)
        .map(|(curve, &factor)| {
            if factor == 1.0 {
                return Ok(curve.clone());
            }
            NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                curve.control_points.iter().map(|point| point.scale(factor)).collect(),
            )
            .map_err(geometry)
        })
        .collect()
}

/// The homogeneous crossing grid `P_ab` the tensor term interpolates.
///
/// Along `v = v_a` the surface is `F_a(u)` plus
/// `sum_b alpha_b(u) (G_b(v_a) - P_ab)`, so a curve is interpolated exactly
/// when the grid takes the OTHER family's values across it.  A corner and an
/// interior crossing take the midpoint and split any gap.  A crossing of a
/// boundary curve with an interior curve takes the INTERIOR curve's point:
/// that is what makes the boundary row the boundary curve before it is
/// assigned, so the assignment moves nothing but round-off, and the interior
/// curve absorbs the gap at its end.
fn crossing_grid(
    f: &[NurbsCurve],
    g: &[NurbsCurve],
    u_parameters: &[f64],
    v_parameters: &[f64],
) -> Result<Vec<Vec<Vec4>>, GordonRefusal> {
    let (last_row, last_column) = (f.len() - 1, g.len() - 1);
    let mut grid = Vec::with_capacity(f.len());
    for (a, u_curve) in f.iter().enumerate() {
        let mut row = Vec::with_capacity(g.len());
        for (b, v_curve) in g.iter().enumerate() {
            let on_u = u_curve.evaluate_homogeneous(u_parameters[b]).map_err(geometry)?;
            let on_v = v_curve.evaluate_homogeneous(v_parameters[a]).map_err(geometry)?;
            let boundary_u = a == 0 || a == last_row;
            let boundary_v = b == 0 || b == last_column;
            row.push(match (boundary_u, boundary_v) {
                (true, false) => on_v,
                (false, true) => on_u,
                _ => homogeneous_midpoint(on_u, on_v),
            });
        }
        grid.push(row);
    }
    Ok(grid)
}

/// The cardinal B-spline functions of a strictly increasing parameter list
/// running from exactly `0.0` to exactly `1.0`: degree `min(3, count - 1)` on
/// the knot-averaging vector (Piegl & Tiller 9.8, which satisfies
/// Schoenberg-Whitney), with coefficients `A^-1 e_j` of the collocation matrix
/// `A[r][c] = N_c(parameters[r])`.
///
/// Each function comes back as a SCALAR curve — the value in `x`, weight 1 —
/// so the Coons compatibility algebra can elevate and refine it alongside the
/// curves it will multiply.  The first and last coefficients are the values at
/// the clamped ends and are set exactly: `delta_j0` and `delta_jn`.
fn cardinal_functions(parameters: &[f64]) -> Result<(usize, Vec<NurbsCurve>), String> {
    let count = parameters.len();
    let degree = MAX_SKIN_DEGREE.min(count - 1);
    let mut knots = vec![0.0f64; count + degree + 1];
    for knot in &mut knots[count..] {
        *knot = 1.0;
    }
    for j in 1..count - degree {
        knots[j + degree] = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
    }
    let knot_vector = KnotVector::new(knots.clone(), degree)?;
    let mut matrix = vec![vec![0.0f64; count]; count];
    for (row, &parameter) in parameters.iter().enumerate() {
        let span = knot_vector.find_span(parameter);
        for (offset, value) in knot_vector
            .basis_functions(span, parameter)
            .into_iter()
            .enumerate()
        {
            matrix[row][span - degree + offset] = value;
        }
    }
    let mut functions = Vec::with_capacity(count);
    for j in 0..count {
        let mut rhs = vec![0.0f64; count];
        rhs[j] = 1.0;
        let mut coefficients = solve_dense(matrix.clone(), rhs)
            .map_err(|error| format!("cardinal collocation at {parameters:?}: {error}"))?;
        coefficients[0] = if j == 0 { 1.0 } else { 0.0 };
        coefficients[count - 1] = if j == count - 1 { 1.0 } else { 0.0 };
        functions.push(NurbsCurve::new(
            degree,
            knots.clone(),
            coefficients
                .into_iter()
                .map(|value| Vec4 { x: value, y: 0.0, z: 0.0, w: 1.0 })
                .collect(),
        )?);
    }
    Ok((degree, functions))
}

/// Make one family and the OTHER family's cardinal functions compatible in
/// this family's running direction.  Returns the compatible curves, their
/// cardinal coefficient rows, and the per-curve work.
fn compatible_direction(
    family: char,
    curves: &[NurbsCurve],
    cardinals: &[NurbsCurve],
) -> Result<(Vec<NurbsCurve>, Vec<Vec<f64>>, Vec<CompatWork>), GordonRefusal> {
    let combined: Vec<NurbsCurve> = curves.iter().chain(cardinals).cloned().collect();
    let (compatible, work) = compat::make_family_compatible(&combined, 1)
        .map_err(|detail| GordonRefusal::Incompatible { family, detail })?;
    let coefficients = compatible[curves.len()..]
        .iter()
        .map(|function| function.control_points.iter().map(|point| point.x).collect())
        .collect();
    Ok((
        compatible[..curves.len()].to_vec(),
        coefficients,
        work[..curves.len()].to_vec(),
    ))
}

/// Solve the Boolean sum on the prepared network and measure the result.
fn build(network: Network) -> Result<(NurbsSurface, GordonReport), GordonRefusal> {
    let Network {
        mut u,
        mut v,
        crossings,
        band,
        closest_approach,
        crossing_gap,
        parameter_spread,
        weight_closure,
    } = network;
    let rows_in_network = u.curves.len();
    let columns_in_network = v.curves.len();

    // alpha_b(u) skins the v-curves across u; beta_a(v) skins the u-curves
    // across v.
    let (skin_degree_u, alphas) = cardinal_functions(&u.parameters).map_err(|detail| {
        GordonRefusal::Incompatible {
            family: 'v',
            detail,
        }
    })?;
    let (skin_degree_v, betas) = cardinal_functions(&v.parameters).map_err(|detail| {
        GordonRefusal::Incompatible {
            family: 'u',
            detail,
        }
    })?;
    let (f, alpha, f_work) = compatible_direction('u', &u.curves, &alphas)?;
    let (g, beta, g_work) = compatible_direction('v', &v.curves, &betas)?;
    for (a, &j) in u.order.iter().enumerate() {
        u.work[j].elevated_by = f_work[a].elevated_by;
        u.work[j].knots_inserted = f_work[a].knots_inserted;
    }
    for (b, &i) in v.order.iter().enumerate() {
        v.work[i].elevated_by = g_work[b].elevated_by;
        v.work[i].knots_inserted = g_work[b].knots_inserted;
    }

    let rows = f[0].control_points.len();
    let columns = g[0].control_points.len();
    let zero = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
    let mut net = vec![vec![zero; columns]; rows];
    // The tensor term's u-side, once per (row, u-curve): sum_b P_ab alpha_b[k].
    let crossing_rows: Vec<Vec<Vec4>> = (0..rows)
        .map(|k| {
            (0..rows_in_network)
                .map(|a| {
                    (0..columns_in_network).fold(zero, |sum, b| {
                        sum.add(crossings[a][b].scale(alpha[b][k]))
                    })
                })
                .collect()
        })
        .collect();
    for (k, net_row) in net.iter_mut().enumerate() {
        for (l, cell) in net_row.iter_mut().enumerate() {
            let mut sum = zero;
            for a in 0..rows_in_network {
                // L_v - T along u-curve a: (F_a[k] - sum_b P_ab alpha_b[k]) beta_a[l].
                sum = sum.add(
                    f[a].control_points[k]
                        .add(crossing_rows[k][a].scale(-1.0))
                        .scale(beta[a][l]),
                );
            }
            for b in 0..columns_in_network {
                sum = sum.add(g[b].control_points[l].scale(alpha[b][k]));
            }
            *cell = sum;
        }
    }
    // The boundary rows are ASSIGNED, not summed, as in the Coons fill: the
    // four boundary curves come back bit for bit, the corners take the
    // midpoint of their two boundary ends.
    let (last_row, last_column) = (rows - 1, columns - 1);
    let (f_first, f_last) = (&f[0], &f[rows_in_network - 1]);
    let (g_first, g_last) = (&g[0], &g[columns_in_network - 1]);
    for (k, net_row) in net.iter_mut().enumerate() {
        net_row[0] = f_first.control_points[k];
        net_row[last_column] = f_last.control_points[k];
    }
    for l in 0..columns {
        net[0][l] = g_first.control_points[l];
        net[last_row][l] = g_last.control_points[l];
    }
    net[0][0] = homogeneous_midpoint(f_first.control_points[0], g_first.control_points[0]);
    net[last_row][0] =
        homogeneous_midpoint(f_first.control_points[last_row], g_last.control_points[0]);
    net[0][last_column] =
        homogeneous_midpoint(f_last.control_points[0], g_first.control_points[last_column]);
    net[last_row][last_column] = homogeneous_midpoint(
        f_last.control_points[last_row],
        g_last.control_points[last_column],
    );

    let mut min_weight = f64::INFINITY;
    let mut worst = (0usize, 0usize);
    for (row, net_row) in net.iter().enumerate() {
        for (column, cell) in net_row.iter().enumerate() {
            if !(cell.w >= min_weight) {
                min_weight = cell.w;
                worst = (row, column);
            }
        }
    }
    if !(min_weight > 0.0) || !min_weight.is_finite() {
        return Err(GordonRefusal::NonPositiveWeight {
            row: worst.0,
            column: worst.1,
            weight: min_weight,
        });
    }

    let surface = NurbsSurface::new(
        f[0].degree,
        g[0].degree,
        f[0].knots.clone(),
        g[0].knots.clone(),
        net,
    )
    .map_err(geometry)?;

    // Measured, never claimed: each input curve against the surface's
    // iso-curve at its network parameter — control nets first, then the
    // kernel's evaluators at samples.
    let mut interpolation_bound = 0.0f64;
    let mut sampled = 0.0f64;
    for (a, curve) in f.iter().enumerate() {
        let iso = surface.iso_curve_v(v.parameters[a]).map_err(geometry)?;
        for (on_surface, on_curve) in iso.control_points.iter().zip(&curve.control_points) {
            interpolation_bound =
                interpolation_bound.max(homogeneous_distance(*on_surface, *on_curve));
        }
        for index in 0..=CURVE_SAMPLES {
            let parameter = index as f64 / CURVE_SAMPLES as f64;
            let on_surface = surface.evaluate(parameter, v.parameters[a]).map_err(geometry)?;
            let on_curve = u.curves[a].evaluate(parameter).map_err(geometry)?;
            sampled = sampled.max(on_surface.sub(on_curve).length());
        }
    }
    for (b, curve) in g.iter().enumerate() {
        let iso = surface.iso_curve_u(u.parameters[b]).map_err(geometry)?;
        for (on_surface, on_curve) in iso.control_points.iter().zip(&curve.control_points) {
            interpolation_bound =
                interpolation_bound.max(homogeneous_distance(*on_surface, *on_curve));
        }
        for index in 0..=CURVE_SAMPLES {
            let parameter = index as f64 / CURVE_SAMPLES as f64;
            let on_surface = surface.evaluate(u.parameters[b], parameter).map_err(geometry)?;
            let on_curve = v.curves[b].evaluate(parameter).map_err(geometry)?;
            sampled = sampled.max(on_surface.sub(on_curve).length());
        }
    }

    let report = GordonReport {
        u_order: u.order,
        v_order: v.order,
        u_reversed: u.reversed,
        v_reversed: v.reversed,
        u_parameters: u.parameters,
        v_parameters: v.parameters,
        crossing_band: band,
        closest_approach,
        crossing_gap,
        parameter_spread,
        u_weight_scales: u.weight_scales,
        v_weight_scales: v.weight_scales,
        weight_closure,
        u_work: u.work,
        v_work: v.work,
        skin_degree_u,
        skin_degree_v,
        degree_u: surface.degree_u,
        degree_v: surface.degree_v,
        control_rows: rows,
        control_columns: columns,
        min_weight,
        interpolation_bound,
        sampled_interpolation_residual: sampled,
    };
    Ok((surface, report))
}

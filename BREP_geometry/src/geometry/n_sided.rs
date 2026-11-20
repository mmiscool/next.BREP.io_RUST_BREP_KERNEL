//! N-sided filling (Golovanov §3.5): a smooth sheet of `N` four-sided patches
//! spanning a closed loop of 3 to 6 boundary curves.
//!
//! # Construction
//!
//! Each side is cut at its parameter midpoint `M_i`. A centre point `Q` is
//! chosen inside the loop, and a cubic *spoke* runs from every midpoint to the
//! centre. Patch `k` is the Coons patch ([`crate::coons_with_ribbons`]) of
//! the four curves round corner `P_k`: the second half of side `k − 1`, the
//! first half of side `k`, and the two spokes between them. Its parameters run
//! `u` along side `k` and `v` from side `k − 1`'s half towards the centre:
//! `(0, 0) = P_k`, `(1, 0) = M_k`, `(1, 1) = Q`, `(0, 1) = M_{k−1}`.
//!
//! Every patch is given all four cross-boundary derivative fields (ribbons):
//!
//! - **Across a side** the ribbon is the derivative into the fill. Without a
//!   prescribed tangent it is the linear blend of the derivatives the patch's
//!   corners already have there. With one it stays in the plane of the side's
//!   tangent and the prescribed direction, which is what makes the fill meet
//!   the neighbouring face tangentially.
//! - **Across a spoke** the two patches it divides share one tangent plane:
//!   the ribbon of the second is `λ(v) r(v) + μ(v) S'(v)`, a combination of the
//!   first's ribbon `r` and the spoke's own tangent `S'`. The spokes arrive at
//!   the centre in one plane, so the patches meet there in it as well.
//!
//! The Coons patch interpolates the ribbons up to the corner twists, which in
//! general disagree and of which it carries the mean. With prescribed tangent
//! planes the disagreement across the planes at a corner P_k is geometry (the
//! two planes turn unlike along the two sides: a vertex blend of fillets),
//! and the mean would leave each side's derivative out of its plane along the
//! whole half side; Gregory's direction-dependent corner twist is added
//! instead, as a spline graded toward the corner (exact along both sides but
//! in a span of 2⁻¹⁰ at the corner; zero with zero slope at the spokes). The
//! joins are still not claimed to be G1. They are measured: [`NSidedReport::spoke_normal_angle`] is
//! the worst angle between the normals of two patches across a spoke, and
//! [`NSidedReport::tangent_normal_angle`] the worst angle between a patch's
//! normal and the prescribed tangent plane along the sides (both as
//! `atan2(|n × m|, n · m)`).
//!
//! # Re-entrant corners
//!
//! A corner where the loop turns away from its interior (an L shape's inner
//! corner) cannot be a patch corner: a Coons patch whose corner angle passes
//! 180° folds there, wherever the centre is. Such a corner becomes a spoke
//! point instead: its two sides are taken as one side cut at the corner, the
//! spoke leaves the corner along the bisector of its angle (or towards the
//! centre, when that stays clear of both sides) and the patches either side
//! of it have convex corners there. The centre is aimed at a point of the
//! loop's kernel, the points from which all of the loop is in sight (Newell's
//! normal fixes the plane; chords of 32 points to a side bound it): the one
//! that leaves every patch's outline most convex, at the midpoints' mean's
//! height (refused when none leaves them all convex). Refused by name: two re-entrant corners
//! side by side ([`NSidedRefusal::AdjacentReentrant`]), a loop with no kernel
//! (not star-shaped, [`NSidedRefusal::Centre`]), and a tangent fill across a
//! re-entrant corner ([`NSidedRefusal::ReentrantTangent`]). Up to 12 sides are
//! taken when the loop has 3 to 6 once each re-entrant corner's two sides
//! count as one.
//!
//! # Rational sides
//!
//! The ribbon constructor is polynomial only, so a rational side (a circular
//! arc) is replaced by the cubic interpolant of evenly spaced samples along
//! it, as many as hold it within 1e-8 model units ([`STAND_IN_DEVIATION`]); the patch
//! edges are then that interpolant, not the arc, and
//! [`NSidedReport::boundary_deviation`] is the largest distance between the two
//! measured at the quarter points between the samples; a side whose stand-in
//! cannot be held within that bound is refused. A prescribed tangent field is
//! sampled in the same way (only its direction is used).
//!
//! # Refusals
//!
//! Fewer than 3 or more than 6 sides (counting a re-entrant corner's two
//! sides as one); two adjacent re-entrant corners, a loop that is not
//! star-shaped, or a tangent fill across a re-entrant corner; a corner whose two sides miss each other
//! by more than [`CORNER_FRACTION`] of the loop's size; a side with no length; a tangent field
//! over a domain not its side's, or one along its side where it should cross
//! it; and any refusal of a patch's Coons construction, verbatim.

use crate::coons::compat::normalise_domain;
use crate::coons::{coons_with_ribbons, CoonsRefusal};
use crate::fit::interpolate_curve;
use crate::{model_scale, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// Samples of a rational side or a field for its cubic stand-in, to start
/// with; a side's sampling doubles until its stand-in is within
/// [`STAND_IN_DEVIATION`] of it, up to [`FIT_SAMPLES_MOST`].
const FIT_SAMPLES: usize = 257;
const FIT_SAMPLES_MOST: usize = 16385;
/// The absolute distance (model units) a rational side's polynomial stand-in
/// is held within, at any size, so a pcurve of the true side on the patch
/// stays far inside the kernel's 1e-7 pcurve floor.
pub const STAND_IN_DEVIATION: f64 = 1e-8;
/// Two consecutive sides meet within this fraction of the loop's size: a
/// band relative to the loop, so it is as tight for a small corner as for a
/// large one (the feature joins picks within the same fraction).
pub const CORNER_FRACTION: f64 = 1e-5;
/// A side with a knot of its own this close to its parameter midpoint (in
/// its [0, 1]) is cut at that knot: a cut beside a knot leaves a half with a
/// sliver span at its end (5.7e-9 on a fitted fillet rib), and a Coons patch
/// reads its edge derivatives across that sliver, amplifying its net's
/// rounding (a ribbon missed by 12.8 against a scale of 2.6). The knot is
/// carried onto the midpoint by an exact reparameterisation of the side and
/// its field together ([`knot_onto_midpoint`]).
pub const MIDPOINT_KNOT_BAND: f64 = 1e-7;
/// The least angle (radians, 15 degrees) the retry construction lets a spoke
/// leave a side with a prescribed tangent at.
const SPOKE_LEAST_LEAVE: f64 = 0.2617993877991494;
/// The share of the field's own across component a shortened bump keeps.
const BUMP_ACROSS_KEPT: f64 = 0.25;
/// The reaches tried: 0.5 down by BUMP_REACH_STEP, BUMP_REACH_STEPS of them,
/// BUMP_REACH_LEAST when none keeps the across component.
const BUMP_REACH_STEP: f64 = 0.05;
const BUMP_REACH_STEPS: usize = 10;
const BUMP_REACH_LEAST: f64 = 0.05;
/// Samples over a reach where the across component is read.
const BUMP_REACH_SAMPLES: usize = 64;
/// Samples along each join where its normals are compared.
const JOIN_SAMPLES: usize = 33;
/// `|w − 1|` below which a curve counts as polynomial.
const POLYNOMIAL_BAND: f64 = 1e-12;

/// Why an N-sided fill was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum NSidedRefusal {
    /// The loop has `sides` sides; 3 to 6 are filled.
    SideCount { sides: usize },
    /// The tangent fields are not one per side.
    TangentCount { sides: usize, tangents: usize },
    /// Side `side` has no length.
    DegenerateSide { side: usize, extent: f64 },
    /// The end of side `side` misses the next side by `gap` (band `band`).
    CornerGap { side: usize, gap: f64, band: f64 },
    /// Side `side`'s tangent field is over another domain than the side.
    TangentDomain { side: usize },
    /// Side `side`'s tangent field runs along the side at its midpoint (the
    /// sine of its angle to the side is `sine`), so it gives no direction
    /// across it.
    TangentAlongSide { side: usize, sine: f64 },
    /// The centre the spokes meet at is not a point the patches can share.
    Centre { detail: String },
    /// Corners `corner` and the next are both re-entrant: the side between
    /// them would be a patch with no convex corner.
    AdjacentReentrant { corner: usize },
    /// A tangent fill across a loop with a re-entrant corner (`corner`).
    ReentrantTangent { corner: usize },
    /// Patch `patch`'s Coons construction refused.
    Patch { patch: usize, refusal: CoonsRefusal },
    /// Side `side` is rational and no cubic stand-in within
    /// [`STAND_IN_DEVIATION`] was found (the best was `deviation` off).
    StandIn { side: usize, deviation: f64 },
    /// A curve operation failed (verbatim).
    Geometry { detail: String },
}

impl std::fmt::Display for NSidedRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SideCount { sides } => write!(f, "N-sided fill: the loop has {sides} sides; 3 to 6 are filled"),
            Self::TangentCount { sides, tangents } => {
                write!(f, "N-sided fill: {tangents} tangent fields for {sides} sides; give one per side")
            }
            Self::DegenerateSide { side, extent } => {
                write!(f, "N-sided fill: side {side} has no length (extent {extent:.3e})")
            }
            Self::CornerGap { side, gap, band } => write!(
                f,
                "N-sided fill: the end of side {side} misses the next side by {gap:.3e} (corner band {band:.3e})"
            ),
            Self::TangentDomain { side } => {
                write!(f, "N-sided fill: the tangent field of side {side} is not over the side's own domain")
            }
            Self::TangentAlongSide { side, sine } => write!(
                f,
                "N-sided fill: the tangent field of side {side} runs along the side at its midpoint (sine {sine:.3e}), so it gives no direction across it"
            ),
            Self::Centre { detail } => write!(f, "N-sided fill: no centre point: {detail}"),
            Self::AdjacentReentrant { corner } => write!(
                f,
                "N-sided fill: corners {corner} and {} are both re-entrant; the side between them has no convex corner to fill from",
                corner + 1
            ),
            Self::ReentrantTangent { corner } => write!(
                f,
                "N-sided fill: corner {corner} is re-entrant; a tangent fill across a re-entrant corner is not built (fill in position)"
            ),
            Self::Patch { patch, refusal } => write!(f, "N-sided fill: patch {patch}: {refusal}"),
            Self::StandIn { side, deviation } => write!(
                f,
                "N-sided fill: side {side} is rational and its best cubic stand-in is {deviation:.3e} off it (bound {STAND_IN_DEVIATION:.0e})"
            ),
            Self::Geometry { detail } => write!(f, "N-sided fill: {detail}"),
        }
    }
}

impl std::error::Error for NSidedRefusal {}

fn geometry(detail: impl std::fmt::Display) -> NSidedRefusal {
    NSidedRefusal::Geometry { detail: detail.to_string() }
}

/// What an N-sided fill built and measured.
#[derive(Debug, Clone)]
pub struct NSidedReport {
    pub sides: usize,
    /// Which sides were reversed to run head to tail from side 0.
    pub reversed: Vec<bool>,
    /// The worst corner gap between consecutive sides as given.
    pub corner_gap: f64,
    /// The corner band the gaps were admitted within.
    pub corner_band: f64,
    /// The centre point all patches share.
    pub centre: Vec3,
    /// The re-entrant corners (corner `k` is where side `k` starts), each
    /// taken as a spoke point with its two sides whole patch edges.
    pub reentrant_corners: Vec<usize>,
    /// Largest distance between a rational side and its polynomial stand-in
    /// (0 when every side is polynomial).
    pub boundary_deviation: f64,
    /// Worst normal angle (radians) between the two patches across a spoke.
    pub spoke_normal_angle: f64,
    /// Worst normal angle (radians) between a patch and the prescribed tangent
    /// plane along the sides; `None` without tangent fields.
    pub tangent_normal_angle: Option<f64>,
    /// Worst corner-twist disagreement any patch carried the mean of.
    pub twist_discrepancy: f64,
    /// Worst cross-derivative residual of any patch against its ribbons.
    pub cross_derivative_residual: f64,
    /// Where each side (as given, in its own parameter) was cut at its
    /// midpoint: its parameter midpoint, or a knot of its own within
    /// [`MIDPOINT_KNOT_BAND`] of it; `None` for a side taken whole (one of a
    /// re-entrant corner's two sides). A caller splitting the side's edge
    /// for the patches cuts it here.
    pub split_parameters: Vec<Option<f64>>,
}

/// Fill the closed loop `sides` (3 to 6 curves, in order round the loop,
/// either way along each) with `N` four-sided patches.
///
/// `tangents`, when given, holds one vector-valued curve per side, over that
/// side's own parameter domain: at each parameter, a direction ACROSS the side
/// in the tangent plane the fill is to meet (the neighbouring face's plane);
/// only its direction matters, and its sign does not (the fill lies on the
/// side of the loop its interior is on). The patches come in corner order,
/// patch `k` at the start of side `k` (after orientation).
pub fn n_sided_fill(
    sides: &[NurbsCurve],
    tangents: Option<&[NurbsCurve]>,
) -> Result<(Vec<NurbsSurface>, NSidedReport), NSidedRefusal> {
    fill(sides, tangents, false, false)
}

/// [`n_sided_fill`] with each corner-twist bump along a side with a
/// prescribed tangent reaching only as far as keeps the field's own across
/// component (see `fill`): the retry for a fill whose patch folds beside a
/// side where the two twists at its corner disagree by far more than that
/// component.
pub fn n_sided_fill_shortening_reach(
    sides: &[NurbsCurve],
    tangents: &[NurbsCurve],
) -> Result<(Vec<NurbsSurface>, NSidedReport), NSidedRefusal> {
    fill(sides, Some(tangents), true, false)
}

/// [`n_sided_fill_shortening_reach`] with each spoke also leaving a side with
/// a prescribed tangent at least 15 degrees off it: the second retry, for a
/// fill whose patches part beside a spoke that leaves nearly along its side.
pub fn n_sided_fill_shortening_reach_spokes_off_sides(
    sides: &[NurbsCurve],
    tangents: &[NurbsCurve],
) -> Result<(Vec<NurbsSurface>, NSidedReport), NSidedRefusal> {
    fill(sides, Some(tangents), true, true)
}

fn fill(
    sides: &[NurbsCurve],
    tangents: Option<&[NurbsCurve]>,
    shorten_reach: bool,
    spokes_off_sides: bool,
) -> Result<(Vec<NurbsSurface>, NSidedReport), NSidedRefusal> {
    let count = sides.len();
    if !(3..=12).contains(&count) {
        return Err(NSidedRefusal::SideCount { sides: count });
    }
    if let Some(fields) = tangents {
        if fields.len() != count {
            return Err(NSidedRefusal::TangentCount { sides: count, tangents: fields.len() });
        }
    }
    let ends = |curve: &NurbsCurve| -> Result<(Vec3, Vec3), NSidedRefusal> {
        let [t0, t1] = curve.domain().map_err(geometry)?;
        Ok((curve.evaluate(t0).map_err(geometry)?, curve.evaluate(t1).map_err(geometry)?))
    };
    let mut points = Vec::new();
    for curve in sides {
        let (a, b) = ends(curve)?;
        points.push(a);
        points.push(b);
    }
    let scale = model_scale(points.iter().copied());
    let band = CORNER_FRACTION * scale;

    // Orient head to tail from side 0 as given; domains onto [0, 1].
    let mut oriented: Vec<NurbsCurve> = Vec::with_capacity(count);
    let mut fields: Vec<Option<NurbsCurve>> = Vec::with_capacity(count);
    let mut reversed = Vec::with_capacity(count);
    let mut corner_gap = 0.0_f64;
    for (index, side) in sides.iter().enumerate() {
        let (start, end) = ends(side)?;
        if start.sub(end).length() <= band && polyline_extent(side)? <= band {
            return Err(NSidedRefusal::DegenerateSide { side: index, extent: polyline_extent(side)? });
        }
        let flip = if index == 0 {
            // side 0 sets the sense; it must still meet side 1 at its end
            let (next_start, next_end) = ends(&sides[1])?;
            let forward = end.sub(next_start).length().min(end.sub(next_end).length());
            let backward = start.sub(next_start).length().min(start.sub(next_end).length());
            backward < forward
        } else {
            let previous_end = ends(&oriented[index - 1])?.1;
            start.sub(previous_end).length() > end.sub(previous_end).length()
        };
        let mut field = tangents.map(|all| all[index].clone());
        if let Some(curve) = &field {
            let [a0, a1] = side.domain().map_err(geometry)?;
            let [b0, b1] = curve.domain().map_err(geometry)?;
            let span = (a1 - a0).abs().max(1.0);
            if (a0 - b0).abs() > 1e-9 * span || (a1 - b1).abs() > 1e-9 * span {
                return Err(NSidedRefusal::TangentDomain { side: index });
            }
        }
        let mut curve = side.clone();
        if flip {
            curve = curve.reversed().map_err(geometry)?;
            if let Some(vector) = &field {
                field = Some(vector.reversed().map_err(geometry)?);
            }
        }
        let curve = normalise_domain(&curve).map_err(geometry)?.0;
        let field = match field {
            Some(vector) => Some(normalise_domain(&vector).map_err(geometry)?.0),
            None => None,
        };
        if index > 0 {
            let gap = ends(&oriented[index - 1])?.1.sub(ends(&curve)?.0).length();
            corner_gap = corner_gap.max(gap);
            if gap > band {
                return Err(NSidedRefusal::CornerGap { side: index - 1, gap, band });
            }
        }
        oriented.push(curve);
        fields.push(field);
        reversed.push(flip);
    }
    let closing = ends(&oriented[count - 1])?.1.sub(ends(&oriented[0])?.0).length();
    corner_gap = corner_gap.max(closing);
    if closing > band {
        return Err(NSidedRefusal::CornerGap { side: count - 1, gap: closing, band });
    }

    // Polynomial stand-ins for rational sides and fields.
    let mut boundary_deviation = 0.0_f64;
    for (index, curve) in oriented.iter_mut().enumerate() {
        if is_rational(curve) {
            let (fitted, deviation) = polynomial_stand_in(curve, true)?;
            if deviation > STAND_IN_DEVIATION {
                return Err(NSidedRefusal::StandIn { side: index, deviation });
            }
            boundary_deviation = boundary_deviation.max(deviation);
            *curve = fitted;
        }
    }
    for field in fields.iter_mut().flatten() {
        if is_rational(field) {
            *field = polynomial_stand_in(field, false)?.0;
        }
    }

    // Halves: A_i = side i over [0, 1/2], B_i over [1/2, 1], each on [0, 1]
    // (both scaled alike, so A_i'(1) = B_i'(0)).
    //
    // A re-entrant corner (the loop turns away from its interior there) is
    // no patch corner: a Coons patch with a corner angle past 180° folds at
    // it, wherever the centre is. It becomes a spoke point instead: its two
    // sides are taken as one side whose halves are the two sides whole, so
    // the spoke leaves the corner into the interior and splits its angle
    // between two convex patch corners. The centre is then chosen in the
    // loop's kernel (the points that see all of it), which a loop with a
    // re-entrant corner needs and the midpoints' mean need not lie in.
    let (reentrant, loop_normal) = reentrant_corners_oriented(&oriented)?;
    let mut groups: Vec<(usize, Option<usize>)> = (0..count).map(|index| (index, None)).collect();
    let mut kernel = None;
    if let Some(corner) = reentrant.iter().position(|is| *is) {
        if tangents.is_some() {
            return Err(NSidedRefusal::ReentrantTangent { corner });
        }
        if let Some(corner) = (0..count).find(|&k| reentrant[k] && reentrant[(k + 1) % count]) {
            return Err(NSidedRefusal::AdjacentReentrant { corner });
        }
        let start = (0..count).find(|&k| !reentrant[k]).expect("no two re-entrant corners are adjacent");
        groups.clear();
        let mut step = 0;
        while step < count {
            let index = (start + step) % count;
            if reentrant[(index + 1) % count] {
                groups.push((index, Some((index + 1) % count)));
                step += 2;
            } else {
                groups.push((index, None));
                step += 1;
            }
        }
        kernel = Some(star_kernel(&oriented, loop_normal, scale)?);
    }
    let count = groups.len();
    if !(3..=6).contains(&count) {
        return Err(NSidedRefusal::SideCount { sides: count });
    }
    let kinked: Vec<bool> = groups.iter().map(|(_, joined)| joined.is_some()).collect();
    let mut first = Vec::with_capacity(count);
    let mut second = Vec::with_capacity(count);
    let mut split_parameters: Vec<Option<f64>> = vec![None; sides.len()];
    // per side (in loop order) its second half's speed at the cut over its
    // first half's: 1 at a cut at the midpoint, (1 − cut) / cut at a knot
    let mut speed_ratio: Vec<f64> = Vec::with_capacity(groups.len());
    for &(index, joined) in &groups {
        match joined {
            None => {
                // A knot of the side's own within the band of its midpoint is
                // carried onto the midpoint: raised to full multiplicity
                // (exact) and each half's parameter mapped affinely, the
                // side and its field alike, so the curve and the plane it
                // reads at every point are unchanged and every half reads
                // the side at 0.5 as before; the cut is then that knot.
                // `BREP_N_SIDED_SLIVER_KNOTS`: a second knot beside the one
                // nearest the midpoint (a fitted rib carries one within 6e-9
                // of it) would leave a sliver span beside the cut whatever
                // the cut; it is removed where the curve is unchanged without
                // it (within SLIVER_KNOT_TOLERANCE of its size), side and
                // field alike (a field with a zero pole keeps its knots).
                if sliver_knots_on() {
                    let size = polyline_extent(&oriented[index])?.max(f64::MIN_POSITIVE);
                    oriented[index] = without_sliver_knots(&oriented[index], SLIVER_KNOT_TOLERANCE * size);
                    if let Some(field) = &fields[index] {
                        // relative to the field's smallest pole: the field's
                        // size is arbitrary, its directions are what counts
                        let least = field
                            .control_points
                            .iter()
                            .map(|pole| (pole.x * pole.x + pole.y * pole.y + pole.z * pole.z).sqrt())
                            .fold(f64::INFINITY, f64::min);
                        if least.is_finite() && least > 0.0 {
                            fields[index] = Some(without_sliver_knots(field, SLIVER_KNOT_TOLERANCE * least));
                        }
                    }
                }
                let near = oriented[index]
                    .knots
                    .iter()
                    .copied()
                    .filter(|knot| *knot > 0.0 && *knot < 1.0 && (knot - 0.5).abs() <= MIDPOINT_KNOT_BAND)
                    .min_by(|a, b| (a - 0.5).abs().total_cmp(&(b - 0.5).abs()));
                let cut = match near {
                    Some(knot) if knot != 0.5 => {
                        oriented[index] = knot_onto_midpoint(&oriented[index], knot)?;
                        if let Some(field) = &fields[index] {
                            fields[index] = Some(knot_onto_midpoint(field, knot)?);
                        }
                        knot
                    }
                    _ => 0.5,
                };
                speed_ratio.push((1.0 - cut) / cut);
                let [t0, t1] = sides[index].domain().map_err(geometry)?;
                split_parameters[index] = Some(if reversed[index] { t1 - cut * (t1 - t0) } else { t0 + cut * (t1 - t0) });
                let (a, b) = oriented[index].split(0.5).map_err(geometry)?;
                first.push(normalise_domain(&a).map_err(geometry)?.0);
                second.push(normalise_domain(&b).map_err(geometry)?.0);
            }
            Some(next) => {
                speed_ratio.push(1.0);
                first.push(oriented[index].clone());
                second.push(oriented[next].clone());
            }
        }
    }
    // A joined side is its first curve here (its start and, with no tangent
    // fields, nothing else is read from it); its end is `second`'s.
    let oriented: Vec<NurbsCurve> = groups.iter().map(|(index, _)| oriented[*index].clone()).collect();
    let fields: Vec<Option<NurbsCurve>> = groups.iter().map(|(index, _)| fields[*index].clone()).collect();
    let at = |curve: &NurbsCurve, t: f64| curve.evaluate(t).map_err(geometry);
    let slope = |curve: &NurbsCurve, t: f64| curve.deriv1(t).map(|(_, d)| d).map_err(geometry);
    let field_half = |field: &NurbsCurve, half: usize, s: f64| field.evaluate(0.5 * (s + half as f64)).map_err(geometry);
    let end_slope = |index: usize| -> Result<Vec3, NSidedRefusal> {
        if kinked[index] { slope(&second[index], 1.0) } else { slope(&oriented[index], 1.0) }
    };

    // Midpoints, side tangents there and the directions into the fill: in
    // the tangent plane at the midpoint (the prescribed one, or the plane of
    // the side and the bisector of its neighbours' directions), towards the
    // midpoints' mean. On a regular loop that is square to the side; on a
    // skewed one it is the way the surface's own cross curves run (a saddle's
    // rulings, which leave a side at a slant).
    let middle: Vec<Vec3> = first.iter().map(|half| at(half, 1.0)).collect::<Result<_, _>>()?;
    let mean = middle.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, point| sum.add(*point)).scale(1.0 / count as f64);
    // With a re-entrant corner the centre is aimed at the point of the
    // loop's kernel that leaves every patch's corners most convex, at the
    // midpoints' mean's height above the loop's plane.
    let mean = match &kernel {
        Some(kernel) => {
            let corners: Vec<Vec3> = first.iter().map(|half| at(half, 0.0)).collect::<Result<_, _>>()?;
            kernel.best_centre(&corners, &middle, mean)?
        }
        None => mean,
    };
    let mut across = Vec::with_capacity(count);
    for index in 0..count {
        let along = slope(&first[index], 1.0)?;
        let unit = along.normalized().map_err(|_| NSidedRefusal::DegenerateSide { side: index, extent: 0.0 })?;
        let raw = match &fields[index] {
            Some(field) => {
                let g = field.evaluate(0.5).map_err(geometry)?;
                let off = g.sub(unit.scale(g.dot(unit)));
                let sine = off.length() / g.length().max(f64::MIN_POSITIVE);
                if !(sine > 1e-6) {
                    return Err(NSidedRefusal::TangentAlongSide { side: index, sine });
                }
                // the sense into the loop: towards the other sides
                let previous = end_slope((index + count - 1) % count)?.scale(-1.0);
                let next = slope(&oriented[(index + 1) % count], 0.0)?;
                let inward = previous.normalized().map_err(geometry)?.add(next.normalized().map_err(geometry)?);
                if off.dot(inward) < 0.0 { off.scale(-1.0) } else { off }
            }
            None if kinked[index] => {
                // into the interior at a re-entrant corner: the bisector of
                // its angle past 180°
                let out = slope(&second[index], 0.0)?.normalized().map_err(geometry)?;
                unit.sub(out)
            }
            None => {
                let previous = end_slope((index + count - 1) % count)?.scale(-1.0);
                let next = slope(&oriented[(index + 1) % count], 0.0)?;
                let sum = previous.normalized().map_err(geometry)?.add(next.normalized().map_err(geometry)?);
                sum.sub(unit.scale(sum.dot(unit)))
            }
        };
        let square = raw.normalized().map_err(|_| NSidedRefusal::Centre {
            detail: format!("side {index} gives no direction into the loop at its midpoint"),
        })?;
        let plane_normal = unit.cross(square);
        let toward = mean.sub(middle[index]);
        let in_plane_toward = toward.sub(plane_normal.scale(toward.dot(plane_normal)));
        // At a re-entrant corner the spoke must leave into the interior
        // past both sides (a convex corner each side of it).
        let inside_both = |direction: Vec3| -> Result<bool, NSidedRefusal> {
            if !kinked[index] {
                return Ok(true);
            }
            let out = slope(&second[index], 0.0)?.normalized().map_err(geometry)?;
            Ok(unit.cross(direction).dot(loop_normal) > 1e-3 && out.cross(direction).dot(loop_normal) > 1e-3)
        };
        let direction = match in_plane_toward.normalized() {
            Ok(direction) if direction.dot(square) > 1e-3 && inside_both(direction)? => direction,
            _ => square,
        };
        // `spokes_off_sides`: a spoke leaving a side with a prescribed
        // tangent at least SPOKE_LEAST_LEAVE off the side. A spoke leaving
        // nearly along its side (7 degrees, at a star whose rounds meet at
        // radii 0.9 against 3) splits the plane at the midpoint between
        // directions that hardly differ, and the two patches beside it part
        // there by 1e-4 rad.
        let direction = if spokes_off_sides && fields[index].is_some() && !kinked[index] {
            let along = direction.dot(unit);
            if along.abs() > SPOKE_LEAST_LEAVE.cos() {
                unit.scale(along.signum() * SPOKE_LEAST_LEAVE.cos()).add(square.scale(SPOKE_LEAST_LEAVE.sin()))
            } else {
                direction
            }
        } else {
            direction
        };
        across.push(direction);
    }
    let reach: Vec<f64> = middle.iter().map(|point| mean.sub(*point).length()).collect();
    if reach.iter().any(|length| !(*length > band)) {
        return Err(NSidedRefusal::Centre { detail: "a side's midpoint is at the loop's centre".into() });
    }
    // Two estimates of the centre that err on opposite sides of a curved
    // fill by the same second-order amount: the midpoints' mean (on their
    // chords, inside a dome) and each midpoint shot along its tangent `across`
    // until it has come as far towards that mean as the mean is from it (on
    // the tangents, outside). For a paraboloid of revolution their mean is
    // exactly its vertex; on a plane both are the same point.
    let mut shot = Vec3::new(0.0, 0.0, 0.0);
    for index in 0..count {
        let toward = mean.sub(middle[index]);
        let unit = toward.scale(1.0 / reach[index]);
        let pace = across[index].dot(unit);
        if !(pace > 1e-3) {
            return Err(NSidedRefusal::Centre {
                detail: format!("side {index} turns away from the loop's centre at its midpoint"),
            });
        }
        shot = shot.add(middle[index].add(across[index].scale(reach[index] / pace)));
    }
    let shot = shot.scale(1.0 / count as f64);
    let centre = mean.add(shot).scale(0.5);
    let normal = (0..count)
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, index| {
            let along = slope(&first[index], 1.0).unwrap_or(Vec3::new(0.0, 0.0, 0.0));
            sum.add(along.cross(across[index]).normalized().unwrap_or(Vec3::new(0.0, 0.0, 0.0)))
        })
        .normalized()
        .map_err(|_| NSidedRefusal::Centre { detail: "the sides' normals cancel".into() })?;

    // Spokes: cubic from M_i (leaving along `across`) to Q (arriving in the
    // centre's tangent plane).
    let mut spokes = Vec::with_capacity(count);
    let mut spoke_bends = Vec::with_capacity(count);
    for index in 0..count {
        let toward = centre.sub(middle[index]);
        let arrive = toward.sub(normal.scale(toward.dot(normal))).normalized().map_err(|_| NSidedRefusal::Centre {
            detail: format!("the spoke from side {index} arrives along the centre's normal"),
        })?;
        let length = toward.length();
        let poles = [
            middle[index],
            middle[index].add(across[index].scale(length / 3.0)),
            centre.sub(arrive.scale(length / 3.0)),
            centre,
        ];
        spoke_bends.push(poles[1].sub(poles[2].scale(2.0)).add(poles[3]).scale(6.0));
        spokes.push(bezier(&poles)?);
    }

    // The field of derivatives into the fill along each whole side, `F_i(t)`
    // in the halves' parameter scale: at the side's start the derivative
    // along the side before it (reversed), at its midpoint the spoke's start,
    // at its end the derivative along the side after it. Without a prescribed
    // tangent it is the quadratic through those three; with one it is
    // `a(t) g(t) + b(t) T(t)` in the prescribed plane, `(a, b)` quadratic
    // through the three values' fits, plus each corner's misfit faded out by
    // the midpoint. One smooth field per side gives the two patches beside a
    // midpoint the same corner twist there.
    let raw_value = |index: usize, t: f64| -> Result<Vec3, NSidedRefusal> {
        let previous = (index + count - 1) % count;
        let next = (index + 1) % count;
        let values = [
            slope(&second[previous], 1.0)?.scale(-1.0),
            slope(&spokes[index], 0.0)?,
            slope(&first[next], 0.0)?,
        ];
        let lagrange = [2.0 * (t - 0.5) * (t - 1.0), -4.0 * t * (t - 1.0), 2.0 * t * (t - 0.5)];
        match &fields[index] {
            None => Ok(values
                .iter()
                .zip(lagrange)
                .fold(Vec3::new(0.0, 0.0, 0.0), |sum, (value, weight)| sum.add(value.scale(weight)))),
            Some(field) => {
                let along = |t: f64| slope(&oriented[index], t);
                let mut fit = [(0.0, 0.0); 3];
                let mut misfit = [Vec3::new(0.0, 0.0, 0.0); 3];
                for (k, at_t) in [0.0, 0.5, 1.0].into_iter().enumerate() {
                    let (g, d) = (field.evaluate(at_t).map_err(geometry)?, along(at_t)?);
                    fit[k] = in_plane(values[k], g, d)?;
                    misfit[k] = values[k].sub(g.scale(fit[k].0).add(d.scale(fit[k].1)));
                }
                let a = (0..3).map(|k| fit[k].0 * lagrange[k]).sum::<f64>();
                let b = (0..3).map(|k| fit[k].1 * lagrange[k]).sum::<f64>();
                let fade = |x: f64| if x > 0.0 { x * x } else { 0.0 };
                Ok(field
                    .evaluate(t)
                    .map_err(geometry)?
                    .scale(a)
                    .add(along(t)?.scale(b))
                    .add(misfit[0].scale(fade(1.0 - 2.0 * t)))
                    .add(misfit[2].scale(fade(2.0 * t - 1.0)))
                    .add(misfit[1].scale(1.0 - fade(1.0 - 2.0 * t) - fade(2.0 * t - 1.0))))
            }
        }
    };
    // A field's derivative in t, by second-order differences (central inside,
    // one-sided at the ends).
    const STEP: f64 = 1e-4;
    let raw_slope = |index: usize, t: f64| -> Result<Vec3, NSidedRefusal> {
        let f = |x: f64| raw_value(index, x);
        if t <= 0.0 {
            Ok(f(0.0)?.scale(-3.0).add(f(STEP)?.scale(4.0)).sub(f(2.0 * STEP)?).scale(0.5 / STEP))
        } else if t >= 1.0 {
            Ok(f(1.0)?.scale(3.0).sub(f(1.0 - STEP)?.scale(4.0)).add(f(1.0 - 2.0 * STEP)?).scale(0.5 / STEP))
        } else {
            Ok(f(t + STEP)?.sub(f(t - STEP)?).scale(0.5 / STEP))
        }
    };
    // The corner twists. At P_k the patch has one twist from side k's field
    // (`F_k'(0)`) and one from side k − 1's (`−F_{k−1}'(1)`); both are moved to
    // their mean by a bump that is zero, with zero slope, at the midpoint, so
    // the midpoint twists stay. With a prescribed tangent the bump is the
    // move's part in the prescribed plane, carried along in the plane (`x g(t)
    // + y T(t)`), so the field never leaves it; what remains of the twists'
    // disagreement is their part across the plane.
    let mut fixes: Vec<[Vec3; 2]> = vec![[Vec3::new(0.0, 0.0, 0.0); 2]; count];
    for k in 0..count {
        let previous = (k + count - 1) % count;
        let (start, end) = (raw_slope(k, 0.0)?, raw_slope(previous, 1.0)?);
        let mean = start.sub(end).scale(0.5);
        fixes[k][0] = mean.sub(start);
        fixes[previous][1] = mean.scale(-1.0).sub(end);
    }
    let mut plane_fixes: Vec<[(f64, f64); 2]> = vec![[(0.0, 0.0); 2]; count];
    for index in 0..count {
        if let Some(field) = &fields[index] {
            for (end, t) in [(0usize, 0.0), (1usize, 1.0)] {
                let (g, d) = (field.evaluate(t).map_err(geometry)?, slope(&oriented[index], t)?);
                plane_fixes[index][end] = in_plane(fixes[index][end], g, d)?;
            }
        }
    }
    // The bump's reach from each end of a side with a prescribed tangent: half
    // the side, or (`shorten_reach`) the longest of 0.5, 0.45, …,
    // 0.05 over which the in-plane move leaves at least BUMP_ACROSS_KEPT of
    // the field's own across component, in its own sense. Where the two
    // twists at a corner disagree by far more than the field's across size
    // (a long rib against a short one: a box corner of radii r, r and R for
    // about 0.293 R < r < 0.35 R), the move over half the side cancels the
    // across component and the patch folds beside the side; a shorter reach
    // carries the same twist to the corner, keeps the plane, and stays clear
    // of the fold. A reach that has to shorten on one side alone leaves every
    // other side as it was.
    let mut reaches: Vec<[f64; 2]> = vec![[0.5, 0.5]; count];
    if shorten_reach {
        for index in 0..count {
            let Some(field) = &fields[index] else { continue };
            for end in 0..2 {
                let move_across = plane_fixes[index][end].0;
                let sense = if end == 0 { 1.0 } else { -1.0 };
                let mut reach = BUMP_REACH_LEAST;
                for step in 0..BUMP_REACH_STEPS {
                    let candidate = 0.5 - BUMP_REACH_STEP * step as f64;
                    let mut kept = true;
                    for j in 1..BUMP_REACH_SAMPLES {
                        let r = candidate * j as f64 / BUMP_REACH_SAMPLES as f64;
                        let t = if end == 0 { r } else { 1.0 - r };
                        let (g, d) = (field.evaluate(t).map_err(geometry)?, slope(&oriented[index], t)?);
                        let own = in_plane(raw_value(index, t)?, g, d)?.0;
                        let shape = r * (1.0 - r / candidate) * (1.0 - r / candidate);
                        let total = own + move_across * sense * shape;
                        if !(total * own > 0.0 && total.abs() >= BUMP_ACROSS_KEPT * own.abs()) {
                            kept = false;
                            break;
                        }
                    }
                    if kept {
                        reach = candidate;
                        break;
                    }
                }
                reaches[index][end] = reach;
            }
        }
    }
    let field_value = |index: usize, t: f64| -> Result<Vec3, NSidedRefusal> {
        // φ(0) = 0, φ'(0) = 1, φ(w) = φ'(w) = 0 for the reach w (½ unless
        // shortened above); ψ the same at t = 1
        let [w0, w1] = reaches[index];
        let phi = if w0 == 0.5 {
            if t < 0.5 { t * (1.0 - 2.0 * t) * (1.0 - 2.0 * t) } else { 0.0 }
        } else if t < w0 {
            t * (1.0 - t / w0) * (1.0 - t / w0)
        } else {
            0.0
        };
        let psi = if w1 == 0.5 {
            if t > 0.5 { -(1.0 - t) * (2.0 * t - 1.0) * (2.0 * t - 1.0) } else { 0.0 }
        } else if t > 1.0 - w1 {
            -(1.0 - t) * (1.0 - (1.0 - t) / w1) * (1.0 - (1.0 - t) / w1)
        } else {
            0.0
        };
        let value = raw_value(index, t)?;
        Ok(match &fields[index] {
            None => value.add(fixes[index][0].scale(phi)).add(fixes[index][1].scale(psi)),
            Some(field) => {
                let (g, d) = (field.evaluate(t).map_err(geometry)?, slope(&oriented[index], t)?);
                let [(x0, y0), (x1, y1)] = plane_fixes[index];
                value.add(g.scale(x0 * phi + x1 * psi)).add(d.scale(y0 * phi + y1 * psi))
            }
        })
    };
    let hermite = |p0: Vec3, d0: Vec3, p1: Vec3, d1: Vec3, s: f64| {
        let (s2, s3) = (s * s, s * s * s);
        p0.scale(2.0 * s3 - 3.0 * s2 + 1.0)
            .add(d0.scale(s3 - 2.0 * s2 + s))
            .add(p1.scale(-2.0 * s3 + 3.0 * s2))
            .add(d1.scale(s3 - s2))
    };
    let scalar_hermite = |p0: f64, d0: f64, p1: f64, d1: f64, s: f64| {
        let (s2, s3) = (s * s, s * s * s);
        p0 * (2.0 * s3 - 3.0 * s2 + 1.0) + d0 * (s3 - 2.0 * s2 + s) + p1 * (-2.0 * s3 + 3.0 * s2) + d1 * (s3 - s2)
    };

    // The centre twists. Patch k's ribbon across spoke k − 1 is
    // `R_k = λ_k r_{k−1} + μ_k S'_{k−1}` (see below); at Q its twist from that
    // side is `R_k'(1) = λ_k'(1) S'_{k−2}(1) + λ_k(1) w_{k−1} + μ_k'(1)
    // S'_{k−1}(1) + μ_k(1) S''_{k−1}(1)` and from its own spoke ribbon `w_k =
    // r_k'(1)`. The `w`, `λ'(1)` and `μ'(1)` that make the two agree round all
    // N patches are one small linear system, solved by least squares near the
    // plain choices (the ribbons' chords), which agree only by chance.
    let arrive_at = |k: usize| slope(&spokes[k], 1.0);
    let mut coefficients = Vec::with_capacity(count);
    for k in 0..count {
        let (previous, before) = ((k + count - 1) % count, (k + count - 2) % count);
        coefficients.push(in_plane(arrive_at(k)?, arrive_at(before)?, arrive_at(previous)?)?);
    }
    let unknowns = 5 * count;
    let mut rows: Vec<Vec<f64>> = Vec::with_capacity(3 * count);
    let mut rhs: Vec<f64> = Vec::with_capacity(3 * count);
    for k in 0..count {
        let (previous, before) = ((k + count - 1) % count, (k + count - 2) % count);
        let (lambda, mu) = coefficients[k];
        let (sb, sp) = (arrive_at(before)?, arrive_at(previous)?);
        let bend = spoke_bends[previous];
        for axis in 0..3 {
            let pick = |v: Vec3| [v.x, v.y, v.z][axis];
            let mut row = vec![0.0; unknowns];
            row[3 * k + axis] = 1.0;
            row[3 * previous + axis] -= lambda;
            row[3 * count + k] = -pick(sb);
            row[4 * count + k] = -pick(sp);
            rows.push(row);
            rhs.push(mu * pick(bend));
        }
    }
    let mut plain = vec![0.0; unknowns];
    for k in 0..count {
        let previous = (k + count - 1) % count;
        let chord = arrive_at(previous)?.sub(slope(&first[k], 1.0)?);
        plain[3 * k] = chord.x;
        plain[3 * k + 1] = chord.y;
        plain[3 * k + 2] = chord.z;
        plain[3 * count + k] = coefficients[k].0 + 1.0;
        plain[4 * count + k] = coefficients[k].1;
    }
    let mut normal_matrix = vec![vec![0.0; unknowns]; unknowns];
    let mut normal_rhs = vec![0.0; unknowns];
    for (row, value) in rows.iter().zip(&rhs) {
        for i in 0..unknowns {
            if row[i] == 0.0 {
                continue;
            }
            normal_rhs[i] += row[i] * value;
            for j in 0..unknowns {
                normal_matrix[i][j] += row[i] * row[j];
            }
        }
    }
    let size = normal_matrix.iter().enumerate().map(|(i, row)| row[i]).fold(0.0_f64, f64::max).max(1.0);
    let regular = 1e-10 * size;
    for i in 0..unknowns {
        normal_matrix[i][i] += regular;
        normal_rhs[i] += regular * plain[i];
    }
    let solved = crate::solve_dense(normal_matrix, normal_rhs).map_err(geometry)?;
    let centre_twist = |k: usize| Vec3::new(solved[3 * k], solved[3 * k + 1], solved[3 * k + 2]);
    let lambda_slope = |k: usize| solved[3 * count + k];
    let mu_slope = |k: usize| solved[4 * count + k];

    // The spoke ribbon of each patch (dS/du on u = 1, v from M_k to Q): from
    // the side's midpoint derivative to the previous spoke's arrival, leaving
    // M_k with the side field's own twist and reaching Q with the solved one.
    let mut spoke_ribbons = Vec::with_capacity(count);
    for k in 0..count {
        let previous = (k + count - 1) % count;
        let start = slope(&first[k], 1.0)?;
        let end = slope(&spokes[previous], 1.0)?;
        let twist = raw_slope(k, 0.5)?.scale(0.5);
        let reach_twist = centre_twist(k);
        // on the knots of the side half the spoke's patch has across from
        // it (second[k − 1], run from M_{k−1} to P_k: mirrored in v)
        spoke_ribbons.push(sampled_on(|v| Ok(hermite(start, twist, end, reach_twist, v)), &interior_knots(&second[previous], true))?);
    }
    let mut patches = Vec::with_capacity(count);
    let mut twist_discrepancy = 0.0_f64;
    let mut cross_derivative_residual = 0.0_f64;
    for k in 0..count {
        let previous = (k + count - 1) % count;
        let before = (k + count - 2) % count;
        let a = &first[k];
        let b = &second[previous];
        let spoke = &spokes[k];
        let back = &spokes[previous];
        // r0 across side k's first half (dS/dv at v = 0), s along u; r3
        // across side k − 1's second half (dS/du at u = 0), s from M_{k−1} to
        // P_k: the two halves of the side fields.
        let r0 = sampled_on(|s| field_value(k, 0.5 * s), &interior_knots(a, false))?;
        let r3 = sampled_on(|s| field_value(previous, 0.5 * (s + 1.0)), &interior_knots(b, false))?;
        let r1 = spoke_ribbons[k].clone();
        // r2 across spoke k − 1 (dS/dv at v = 1), over the reversed spoke, s = 0
        // at Q. In the spoke's own v = 1 − s it is λ(v) r1^{k−1}(v) +
        // μ(v) S'_{k−1}(v): in the plane of the other patch's ribbon and the
        // spoke, so the two patches share their tangent plane along it; −r1 at
        // M_{k−1} with no twist of its own there, and S'_k(1) at Q.
        let other = &spoke_ribbons[previous];
        let target = slope(spoke, 1.0)?;
        let _ = (target, before);
        let (lambda, mu) = coefficients[k];
        let (lambda_end, mu_end) = (lambda_slope(k), mu_slope(k));
        let kink = if kinked[previous] {
            Some((slope(&first[previous], 1.0)?, slope(&second[previous], 0.0)?))
        } else {
            None
        };
        // over the reversed spoke: the patch runs u = 1 − s along it
        let r2 = sampled_on(|s| {
            let v = 1.0 - s;
            // at M_{k−1} the ribbon is the boundary's own derivative, the
            // second half's: the first half's times their speed ratio
            let l = scalar_hermite(-speed_ratio[previous], 0.0, lambda, lambda_end, v);
            let m = scalar_hermite(0.0, 0.0, mu, mu_end, v);
            let ribbon = other.evaluate(v).map_err(geometry)?.scale(l).add(slope(back, v)?.scale(m));
            Ok(match kink {
                // at a re-entrant corner the side leaves along `out`, not on
                // along `into`: the ribbon meets it there and fades back to
                // the shared-plane form with zero slope
                Some((into, out)) => ribbon.add(into.sub(out).scale(scalar_hermite(1.0, 0.0, 0.0, 0.0, v))),
                None => ribbon,
            })
        }, &interior_knots(a, true))?;
        let top = back.reversed().map_err(geometry)?;
        let top = normalise_domain(&top).map_err(geometry)?.0;
        let (surface, report) = coons_with_ribbons(
            [a.clone(), spoke.clone(), top, b.clone()],
            [Some(r0), Some(r1), Some(r2), Some(r3)],
        )
        .map_err(|refusal| NSidedRefusal::Patch { patch: k, refusal })?;
        if let Some(ribbons) = &report.ribbons {
            twist_discrepancy = twist_discrepancy.max(ribbons.twist_discrepancy);
            cross_derivative_residual = cross_derivative_residual.max(ribbons.cross_derivative_residual);
        }
        patches.push(surface);
    }

    // Corner twists across the prescribed planes. Along a side with a
    // prescribed tangent plane the patch's derivative into the fill must stay
    // in that plane, and how fast the plane turns along each side fixes the
    // twist's component across it: at a corner the two sides ask for two
    // different twists wherever the two planes turn unlike (the vertex blend
    // of three fillets: one turns about the corner, its neighbour hardly at
    // all), and a patch smooth at the corner has one. The Coons patch carries
    // their mean, which leaves each side's derivative out of its plane by a
    // cubic bump `H1(u) = u (1 − u)²` times the difference, largest a third of
    // the way along the half side. Gregory's patch meets both with the
    // direction-dependent twist `(u t_v + v t_u) / (u + v)`; its term is added
    // here as a spline on knots graded toward the corner, exact along both
    // sides but in the corner's first span (2⁻¹⁰ of the patch), zero with zero
    // slope along the spokes, so the joins between patches are untouched.
    if fields.iter().any(Option::is_some) {
        for k in 0..count {
            let previous = (k + count - 1) % count;
            let along_v0 = fields[k].as_ref().map(|field| (&oriented[k], field));
            let along_u0 = fields[previous].as_ref().map(|field| (&oriented[previous], field));
            patches[k] = corner_twist_corrected(&patches[k], along_v0, along_u0)?;
        }
    }

    // Measure the joins.
    let mut spoke_normal_angle = 0.0_f64;
    for k in 0..count {
        let next = (k + 1) % count;
        for index in 0..JOIN_SAMPLES {
            let v = index as f64 / (JOIN_SAMPLES - 1) as f64;
            // patch k at (1, v), patch k + 1 at (v, 1): the same point of spoke k
            let one = unit_normal(&patches[k], 1.0, v)?;
            let two = unit_normal(&patches[next], v, 1.0)?;
            if let (Some(one), Some(two)) = (one, two) {
                spoke_normal_angle = spoke_normal_angle.max(angle(one, two));
            }
        }
    }
    let tangent_normal_angle = if fields.iter().any(Option::is_some) {
        let mut worst = 0.0_f64;
        for k in 0..count {
            let previous = (k + count - 1) % count;
            for index in 0..JOIN_SAMPLES {
                let s = index as f64 / (JOIN_SAMPLES - 1) as f64;
                if let Some(field) = &fields[k] {
                    let plane = slope(&first[k], s)?.cross(field_half(field, 0, s)?);
                    if let (Some(n), Ok(m)) = (unit_normal(&patches[k], s, 0.0)?, plane.normalized()) {
                        worst = worst.max(line_angle(n, m));
                    }
                }
                if let Some(field) = &fields[previous] {
                    let plane = slope(&second[previous], s)?.cross(field_half(field, 1, s)?);
                    if let (Some(n), Ok(m)) = (unit_normal(&patches[k], 0.0, 1.0 - s)?, plane.normalized()) {
                        worst = worst.max(line_angle(n, m));
                    }
                }
            }
        }
        Some(worst)
    } else {
        None
    };

    Ok((
        patches,
        NSidedReport {
            sides: count,
            reversed,
            corner_gap,
            corner_band: band,
            centre,
            reentrant_corners: reentrant.iter().enumerate().filter(|(_, is)| **is).map(|(k, _)| k).collect(),
            boundary_deviation,
            spoke_normal_angle,
            tangent_normal_angle,
            twist_discrepancy,
            cross_derivative_residual,
            split_parameters,
        },
    ))
}

/// Halvings toward the corner of the knots that carry the corner-twist term
/// (after them, spans of 1/8). Ten levels, not more: every knot of the term
/// is a knot of the patch it is added to, and finer spans (2⁻²⁰, spans of
/// 1/32) made a tangent fill of arc sides take 44 s to build into a sheet
/// for no reading the 1e-4 rad bar can see.
const GRADED_LEVELS: i32 = 10;

/// The clamped cubic knots of the corner-twist term on [0, 1]: halving toward
/// the corner (0) down to 2⁻¹⁰, spans of 1/8 after. The term vanishes with
/// its slope at the far end (the midpoint), so it needs no grading there:
/// the seven knots it had toward 1 went into every patch for nothing the
/// 1e-4 rad bars read.
fn twist_knots() -> Vec<f64> {
    let mut interior: Vec<f64> = (4..=GRADED_LEVELS).rev().map(|level| 0.5_f64.powi(level)).collect();
    interior.extend((1..8).map(|k| k as f64 / 8.0));
    let mut knots = vec![0.0; 4];
    knots.extend(interior);
    knots.extend([1.0; 4]);
    knots
}

/// The cubic B-spline basis functions on `knots` at `x` (all of them).
fn cubic_basis(knots: &[f64], x: f64) -> Vec<f64> {
    let n = knots.len() - 4;
    let x = x.clamp(knots[3], knots[n]);
    let span = (3..n).rev().find(|&i| knots[i] <= x).unwrap_or(3).min(n - 1);
    let mut values = vec![0.0; knots.len() - 1];
    values[span] = 1.0;
    for degree in 1..=3 {
        let mut next = vec![0.0; knots.len() - 1 - degree];
        for i in 0..next.len() {
            let mut value = 0.0;
            if knots[i + degree] > knots[i] {
                value += (x - knots[i]) / (knots[i + degree] - knots[i]) * values[i];
            }
            if knots[i + degree + 1] > knots[i + 1] {
                value += (knots[i + degree + 1] - x) / (knots[i + degree + 1] - knots[i + 1]) * values[i + 1];
            }
            next[i] = value;
        }
        values = next;
    }
    values.truncate(n);
    values
}

/// `patch` with Gregory's corner-twist term added at its corner (0, 0) (see
/// the corner-twist notes in [`n_sided_fill`]). `along_v0` is the side and
/// field whose first half is the patch's `v = 0` edge (side parameter
/// `t = u / 2`), `along_u0` those whose second half is its `u = 0` edge
/// (`t = 1 − v / 2`). Along each the patch's derivative into it misses the
/// prescribed plane by `E(x) = (D · n) n`; that miss, fitted as a spline
/// on [`twist_knots`] vanishing at both ends (and in slope at the far one),
/// is what the term takes away. A rational patch is returned as it is.
fn corner_twist_corrected(
    patch: &NurbsSurface,
    along_v0: Option<(&NurbsCurve, &NurbsCurve)>,
    along_u0: Option<(&NurbsCurve, &NurbsCurve)>,
) -> Result<NurbsSurface, NSidedRefusal> {
    if patch.control_points.iter().flatten().any(|pole| (pole.w - 1.0).abs() > POLYNOMIAL_BAND) {
        return Ok(patch.clone());
    }
    let [u0, u1] = patch.domain_u().map_err(geometry)?;
    let [v0, v1] = patch.domain_v().map_err(geometry)?;
    let knots = twist_knots();
    let n = knots.len() - 4;
    let plane_normal = |side: &NurbsCurve, field: &NurbsCurve, t: f64| -> Result<Option<Vec3>, NSidedRefusal> {
        let tangent = side.deriv1(t).map_err(geometry)?.1;
        Ok(tangent.cross(field.evaluate(t).map_err(geometry)?).normalized().ok())
    };
    // the miss across the plane along an edge, as spline coefficients
    // e_1 .. e_{n−3} (e_0 = e_{n−2} = e_{n−1} = 0), least squares over three
    // points in every knot span
    let miss = |edge: Option<(&NurbsCurve, &NurbsCurve)>, on_v0: bool| -> Result<Vec<Vec3>, NSidedRefusal> {
        let zero = Vec3::new(0.0, 0.0, 0.0);
        let Some((side, field)) = edge else { return Ok(vec![zero; n]) };
        let free = n - 3;
        let mut matrix = vec![vec![0.0; free]; free];
        let mut rhs = [vec![0.0; free], vec![0.0; free], vec![0.0; free]];
        for span in 3..n {
            let (a, b) = (knots[span], knots[span + 1]);
            if !(b > a) {
                continue;
            }
            for fraction in [0.2, 0.5, 0.8] {
                let x = a + (b - a) * fraction;
                let (derivative, t) = if on_v0 {
                    let (_, _, sv) = patch.deriv1(u0 + (u1 - u0) * x, v0).map_err(geometry)?;
                    (sv.scale(v1 - v0), 0.5 * x)
                } else {
                    let (_, su, _) = patch.deriv1(u0, v0 + (v1 - v0) * x).map_err(geometry)?;
                    (su.scale(u1 - u0), 1.0 - 0.5 * x)
                };
                let Some(normal) = plane_normal(side, field, t)? else { continue };
                let value = normal.scale(derivative.dot(normal));
                // weight a span by its length, so the graded spans do not
                // swamp the fit
                let weight = b - a;
                let basis = cubic_basis(&knots, x);
                for i in 0..free {
                    let bi = basis[i + 1];
                    if bi == 0.0 {
                        continue;
                    }
                    rhs[0][i] += weight * bi * value.x;
                    rhs[1][i] += weight * bi * value.y;
                    rhs[2][i] += weight * bi * value.z;
                    for j in 0..free {
                        matrix[i][j] += weight * bi * basis[j + 1];
                    }
                }
            }
        }
        let size = (0..free).map(|i| matrix[i][i]).fold(0.0_f64, f64::max);
        for (i, row) in matrix.iter_mut().enumerate() {
            row[i] += 1e-14 * size;
        }
        let mut solved = Vec::with_capacity(3);
        for axis in &rhs {
            solved.push(crate::solve_dense(matrix.clone(), axis.clone()).map_err(geometry)?);
        }
        let mut out = vec![zero; n];
        for i in 0..free {
            out[i + 1] = Vec3::new(solved[0][i], solved[1][i], solved[2][i]);
        }
        Ok(out)
    };
    let (a, b) = (miss(along_v0, true)?, miss(along_u0, false)?);
    if a.iter().chain(&b).all(|e| e.length() == 0.0) {
        return Ok(patch.clone());
    }
    let term = gregory_twist_term(&knots, &a, &b, [u0, u1], [v0, v1])?;
    add_polynomial_surfaces(patch, &term)
}

/// A bicubic spline `G` on `domain_u × domain_v` (normalised `x, y ∈ [0, 1]`,
/// knots `knots` each way) that is zero along all four edges, zero in slope
/// across the far two, and has `∂G/∂y (x, 0) = −E(x)` and
/// `∂G/∂x (0, y) = −F(y)` exactly but in the corner's first knot span, for
/// the splines `E`, `F` with coefficients `a`, `b`: Gregory's corner term
/// `−(E(x) W(y) x + F(y) W(x) y) / (x + y)`, `W(t) = t (1 − t)²`, its two
/// edge rows set exactly and its inside sampled at the Greville points.
fn gregory_twist_term(
    knots: &[f64],
    a: &[Vec3],
    b: &[Vec3],
    domain_u: [f64; 2],
    domain_v: [f64; 2],
) -> Result<NurbsSurface, NSidedRefusal> {
    let n = knots.len() - 4;
    // ∂/∂y at y = 0 is κ (c_1 − c_0) on a clamped cubic
    let kappa = 3.0 / knots[4];
    let greville = |i: usize| (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0;
    let spline = |coefficients: &[Vec3], x: f64| {
        cubic_basis(knots, x).iter().zip(coefficients).fold(Vec3::new(0.0, 0.0, 0.0), |sum, (w, c)| sum.add(c.scale(*w)))
    };
    let w = |t: f64| t * (1.0 - t) * (1.0 - t);
    let zero = Vec3::new(0.0, 0.0, 0.0);
    let mut rows = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(n);
        for j in 0..n {
            let value = if i == 0 || j == 0 || i + 2 >= n || j + 2 >= n {
                zero
            } else if i == 1 && j == 1 {
                a[1].add(b[1]).scale(-0.5 / kappa)
            } else if j == 1 {
                a[i].scale(-1.0 / kappa)
            } else if i == 1 {
                b[j].scale(-1.0 / kappa)
            } else {
                let (x, y) = (greville(i), greville(j));
                spline(a, x).scale(w(y) * x).add(spline(b, y).scale(w(x) * y)).scale(-1.0 / (x + y))
            };
            row.push(Vec4::from_point(value, 1.0));
        }
        rows.push(row);
    }
    let map = |domain: [f64; 2]| knots.iter().map(|k| domain[0] + (domain[1] - domain[0]) * k).collect::<Vec<f64>>();
    NurbsSurface::new(3, 3, map(domain_u), map(domain_v), rows).map_err(geometry)
}

/// The sum of two polynomial surfaces over one domain, on a common degree
/// and knot vector (both refined exactly).
fn add_polynomial_surfaces(first: &NurbsSurface, second: &NurbsSurface) -> Result<NurbsSurface, NSidedRefusal> {
    use crate::KnotDirection;
    let (pu, pv) = (first.degree_u.max(second.degree_u), first.degree_v.max(second.degree_v));
    let lift = |surface: &NurbsSurface| -> Result<NurbsSurface, NSidedRefusal> {
        let mut out = surface.clone();
        if out.degree_u < pu {
            out = out.elevate_degree(KnotDirection::U, pu).map_err(geometry)?;
        }
        if out.degree_v < pv {
            out = out.elevate_degree(KnotDirection::V, pv).map_err(geometry)?;
        }
        Ok(out)
    };
    let (mut a, mut b) = (lift(first)?, lift(second)?);
    // the knots one vector lacks against the other, with multiplicity; two
    // knots within the identity tolerance are one knot, as `refine_knots`
    // snaps them
    let same_knot = |a: f64, b: f64| (a - b).abs() <= crate::curve::KNOT_IDENTITY_TOL;
    let missing = |have: &[f64], want: &[f64]| -> Vec<f64> {
        let mut out = Vec::new();
        let mut index = 0;
        while index < want.len() {
            let value = want[index];
            let wanted = want.iter().filter(|k| same_knot(**k, value)).count();
            let held = have.iter().filter(|k| same_knot(**k, value)).count();
            out.extend(std::iter::repeat_n(value, wanted.saturating_sub(held)));
            index += wanted.max(1);
        }
        out
    };
    for direction in [KnotDirection::U, KnotDirection::V] {
        let (ka, kb) = match direction {
            KnotDirection::U => (a.knots_u.clone(), b.knots_u.clone()),
            KnotDirection::V => (a.knots_v.clone(), b.knots_v.clone()),
        };
        a = a.refine_knots(direction, &missing(&ka, &kb)).map_err(geometry)?;
        b = b.refine_knots(direction, &missing(&kb, &ka)).map_err(geometry)?;
    }
    let same = |x: &[f64], y: &[f64]| x.len() == y.len() && x.iter().zip(y).all(|(p, q)| (p - q).abs() <= 1e-9);
    if !same(&a.knots_u, &b.knots_u) || !same(&a.knots_v, &b.knots_v) {
        return Err(geometry("the corner-twist term does not share the patch's knots"));
    }
    let poles = a
        .control_points
        .iter()
        .zip(&b.control_points)
        .map(|(row_a, row_b)| {
            row_a.iter().zip(row_b).map(|(p, q)| Vec4 { x: p.x + q.x, y: p.y + q.y, z: p.z + q.z, w: 1.0 }).collect()
        })
        .collect();
    NurbsSurface::new(pu, pv, a.knots_u.clone(), a.knots_v.clone(), poles).map_err(geometry)
}

/// The re-entrant corners of the loop `sides` (each over [0, 1], head to
/// tail; corner `k` is where side `k` starts) and the loop's normal (Newell's,
/// of points along every side; zero for a loop with no area, which then has
/// no re-entrant corner). A corner is re-entrant where the loop turns away
/// from its interior, `(T_in × T_out) · n < −1e-6` of the unit tangents.
fn reentrant_corners_oriented(sides: &[NurbsCurve]) -> Result<(Vec<bool>, Vec3), NSidedRefusal> {
    const PER_SIDE: usize = 16;
    let mut points = Vec::with_capacity(sides.len() * PER_SIDE);
    for side in sides {
        for k in 0..PER_SIDE {
            points.push(side.evaluate(k as f64 / PER_SIDE as f64).map_err(geometry)?);
        }
    }
    // about the first point (so a loop far from the origin sums no large
    // cancelling terms), and judged against the loop's own size (so a small
    // loop is not taken for one with no area)
    let origin = points[0];
    let local: Vec<Vec3> = points.iter().map(|point| point.sub(origin)).collect();
    let newell = (0..local.len()).fold(Vec3::new(0.0, 0.0, 0.0), |sum, k| sum.add(local[k].cross(local[(k + 1) % local.len()])));
    let extent = local.iter().map(|point| point.length()).fold(0.0_f64, f64::max);
    let length = newell.length();
    if !(length > 1e-12 * extent * extent) || !length.is_finite() {
        return Ok((vec![false; sides.len()], Vec3::new(0.0, 0.0, 0.0)));
    }
    let normal = newell.scale(1.0 / length);
    let count = sides.len();
    let mut reentrant = Vec::with_capacity(count);
    for k in 0..count {
        let into = sides[(k + count - 1) % count].deriv1(1.0).map_err(geometry)?.1.normalized();
        let out = sides[k].deriv1(0.0).map_err(geometry)?.1.normalized();
        reentrant.push(match (into, out) {
            (Ok(into), Ok(out)) => into.cross(out).dot(normal) < -1e-6,
            _ => false,
        });
    }
    Ok((reentrant, normal))
}

/// The re-entrant corners of a loop of 3 to 12 curves given in order round
/// it, either way along each (corner `k` is where side `k` starts, after
/// orienting head to tail from side 0). A boundary fill uses it to send a
/// four-sided loop with a re-entrant corner to [`n_sided_fill`].
pub fn reentrant_corners(sides: &[NurbsCurve]) -> Result<Vec<usize>, NSidedRefusal> {
    if sides.len() < 3 {
        return Ok(Vec::new());
    }
    let ends = |curve: &NurbsCurve| -> Result<(Vec3, Vec3), NSidedRefusal> {
        let [t0, t1] = curve.domain().map_err(geometry)?;
        Ok((curve.evaluate(t0).map_err(geometry)?, curve.evaluate(t1).map_err(geometry)?))
    };
    let mut oriented: Vec<NurbsCurve> = Vec::with_capacity(sides.len());
    for (index, side) in sides.iter().enumerate() {
        let (start, end) = ends(side)?;
        let flip = if index == 0 {
            let (next_start, next_end) = ends(&sides[1])?;
            let forward = end.sub(next_start).length().min(end.sub(next_end).length());
            let backward = start.sub(next_start).length().min(start.sub(next_end).length());
            backward < forward
        } else {
            let previous_end = ends(&oriented[index - 1])?.1;
            start.sub(previous_end).length() > end.sub(previous_end).length()
        };
        let curve = if flip { side.reversed().map_err(geometry)? } else { side.clone() };
        oriented.push(normalise_domain(&curve).map_err(geometry)?.0);
    }
    let (reentrant, _) = reentrant_corners_oriented(&oriented)?;
    Ok(reentrant.iter().enumerate().filter(|(_, is)| **is).map(|(k, _)| k).collect())
}

/// The kernel of a loop in its plane: the convex polygon of points that see
/// every point of the loop (the intersection of the half planes left of each
/// chord of 32 points to a side), in the frame `(e1, e2)` of the plane square
/// to `normal`.
struct Kernel {
    origin: Vec3,
    e1: Vec3,
    e2: Vec3,
    polygon: Vec<[f64; 2]>,
}

impl Kernel {
    /// The point of the kernel, placed in the plane through `through` (so a
    /// point above the loop's plane keeps its height), that makes the patch
    /// outlines `corners[k], middle[k], Q, middle[k − 1]` most convex: the
    /// largest least sine of their turns, over the kernel's centroid and a
    /// 24 x 24 grid of its points. Refused when no point leaves every outline
    /// convex (the patches would fold).
    fn best_centre(&self, corners: &[Vec3], middle: &[Vec3], through: Vec3) -> Result<Vec3, NSidedRefusal> {
        let flat = |p: Vec3| [p.sub(self.origin).dot(self.e1), p.sub(self.origin).dot(self.e2)];
        let corners: Vec<[f64; 2]> = corners.iter().map(|p| flat(*p)).collect();
        let middle: Vec<[f64; 2]> = middle.iter().map(|p| flat(*p)).collect();
        let count = corners.len();
        let score = |q: [f64; 2]| {
            let mut least = f64::INFINITY;
            for k in 0..count {
                let outline = [corners[k], middle[k], q, middle[(k + count - 1) % count]];
                for index in 0..4 {
                    let (a, b, c) = (outline[index], outline[(index + 1) % 4], outline[(index + 2) % 4]);
                    let (d1, d2) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
                    let size = (d1[0].hypot(d1[1]) * d2[0].hypot(d2[1])).max(f64::MIN_POSITIVE);
                    least = least.min((d1[0] * d2[1] - d1[1] * d2[0]) / size);
                }
            }
            least
        };
        let inside = |q: [f64; 2]| {
            (0..self.polygon.len()).all(|k| {
                let (a, b) = (self.polygon[k], self.polygon[(k + 1) % self.polygon.len()]);
                (b[0] - a[0]) * (q[1] - a[1]) - (b[1] - a[1]) * (q[0] - a[0]) > 0.0
            })
        };
        let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for point in &self.polygon {
            for axis in 0..2 {
                low[axis] = low[axis].min(point[axis]);
                high[axis] = high[axis].max(point[axis]);
            }
        }
        let mut best = (self.centroid(), score(self.centroid()));
        const GRID: usize = 24;
        for i in 1..GRID {
            for j in 1..GRID {
                let q = [
                    low[0] + (high[0] - low[0]) * i as f64 / GRID as f64,
                    low[1] + (high[1] - low[1]) * j as f64 / GRID as f64,
                ];
                if inside(q) {
                    let value = score(q);
                    if value > best.1 {
                        best = (q, value);
                    }
                }
            }
        }
        if !(best.1 > 1e-3) {
            return Err(NSidedRefusal::Centre {
                detail: "no point inside the loop leaves every patch convex round it".into(),
            });
        }
        let ([x, y], [x0, y0]) = (best.0, flat(through));
        Ok(through.add(self.e1.scale(x - x0)).add(self.e2.scale(y - y0)))
    }

    /// The kernel polygon's centroid.
    fn centroid(&self) -> [f64; 2] {
        let (mut area, mut cx, mut cy) = (0.0, 0.0, 0.0);
        for k in 0..self.polygon.len() {
            let (a, b) = (self.polygon[k], self.polygon[(k + 1) % self.polygon.len()]);
            let cross = a[0] * b[1] - b[0] * a[1];
            area += cross;
            cx += (a[0] + b[0]) * cross;
            cy += (a[1] + b[1]) * cross;
        }
        [cx / (3.0 * area), cy / (3.0 * area)]
    }
}

fn star_kernel(sides: &[NurbsCurve], normal: Vec3, scale: f64) -> Result<Kernel, NSidedRefusal> {
    const PER_SIDE: usize = 32;
    let not_star = || NSidedRefusal::Centre {
        detail: "the loop is not star-shaped: no point inside it sees all of its boundary".into(),
    };
    let helper = if normal.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let e1 = helper.sub(normal.scale(helper.dot(normal))).normalized().map_err(|_| not_star())?;
    let e2 = normal.cross(e1);
    // in the plane's frame about the loop's first point, so a loop far from
    // the origin clips and measures without cancellation
    let origin = sides[0].evaluate(0.0).map_err(geometry)?;
    let mut points: Vec<[f64; 2]> = Vec::with_capacity(sides.len() * PER_SIDE);
    for side in sides {
        for k in 0..PER_SIDE {
            let point = side.evaluate(k as f64 / PER_SIDE as f64).map_err(geometry)?.sub(origin);
            points.push([point.dot(e1), point.dot(e2)]);
        }
    }
    let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for point in &points {
        for axis in 0..2 {
            low[axis] = low[axis].min(point[axis]);
            high[axis] = high[axis].max(point[axis]);
        }
    }
    let pad = (high[0] - low[0]).max(high[1] - low[1]);
    let mut polygon =
        vec![[low[0] - pad, low[1] - pad], [high[0] + pad, low[1] - pad], [high[0] + pad, high[1] + pad], [low[0] - pad, high[1] + pad]];
    // keep the side of each chord the interior is on (left, the loop running
    // counter-clockwise about its Newell normal)
    for k in 0..points.len() {
        let (a, b) = (points[k], points[(k + 1) % points.len()]);
        let side = |p: [f64; 2]| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
        let mut clipped = Vec::with_capacity(polygon.len() + 1);
        for index in 0..polygon.len() {
            let (p, q) = (polygon[index], polygon[(index + 1) % polygon.len()]);
            let (sp, sq) = (side(p), side(q));
            if sp >= 0.0 {
                clipped.push(p);
            }
            if (sp >= 0.0) != (sq >= 0.0) {
                let t = sp / (sp - sq);
                clipped.push([p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t]);
            }
        }
        polygon = clipped;
        if polygon.len() < 3 {
            return Err(not_star());
        }
    }
    let area = 0.5
        * (0..polygon.len())
            .map(|k| {
                let (a, b) = (polygon[k], polygon[(k + 1) % polygon.len()]);
                a[0] * b[1] - b[0] * a[1]
            })
            .sum::<f64>();
    if !(area > 1e-6 * scale * scale) {
        return Err(not_star());
    }
    Ok(Kernel { origin, e1, e2, polygon })
}

/// The length of a curve's control polygon (zero only for a point).
fn polyline_extent(curve: &NurbsCurve) -> Result<f64, NSidedRefusal> {
    let points: Vec<Vec3> = curve
        .control_points
        .iter()
        .map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w))
        .collect();
    Ok(points.windows(2).map(|pair| pair[1].sub(pair[0]).length()).sum())
}

fn is_rational(curve: &NurbsCurve) -> bool {
    curve.control_points.iter().any(|pole| (pole.w - 1.0).abs() > POLYNOMIAL_BAND)
}

/// The cubic interpolant of points of `curve` (domain [0, 1]) at their own
/// parameters, and its largest distance from the curve at the three quarter
/// points between them. The points are spread over every knot span of the
/// curve alike (`per` to a span, at least [`FIT_SAMPLES`] in all), so a
/// feature the curve has in a short span is sampled as finely as a long
/// span; with `adaptive` the count doubles until the distance is within
/// [`STAND_IN_DEVIATION`] or [`FIT_SAMPLES_MOST`] points are reached (the
/// distance is reported either way). A direction field (`adaptive` off)
/// takes the first sampling.
fn polynomial_stand_in(curve: &NurbsCurve, adaptive: bool) -> Result<(NurbsCurve, f64), NSidedRefusal> {
    let mut breaks: Vec<f64> = curve.knots.iter().copied().filter(|knot| (0.0..=1.0).contains(knot)).collect();
    breaks.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    if breaks.first().is_none_or(|first| *first > 0.0) {
        breaks.insert(0, 0.0);
    }
    if breaks.last().is_none_or(|last| *last < 1.0) {
        breaks.push(1.0);
    }
    let spans = breaks.len() - 1;
    let mut per = FIT_SAMPLES.div_ceil(spans).max(4);
    loop {
        // one interpolant per knot span (an interpolant across spans of very
        // different lengths rings), joined end to end with a triple knot
        let mut parameters = Vec::with_capacity(spans * per + 1);
        let mut knots: Vec<f64> = Vec::new();
        let mut poles: Vec<Vec4> = Vec::new();
        for (index, pair) in breaks.windows(2).enumerate() {
            let local: Vec<f64> = (0..=per).map(|k| pair[0] + (pair[1] - pair[0]) * k as f64 / per as f64).collect();
            let points: Vec<Vec3> = local.iter().map(|t| curve.evaluate(*t)).collect::<Result<_, _>>().map_err(geometry)?;
            let piece = interpolate_curve(&points, 3, &local).map_err(geometry)?;
            let interior = &piece.knots[4..piece.knots.len() - 4];
            if index == 0 {
                knots.extend([pair[0]; 4]);
                poles.extend(piece.control_points.iter().copied());
            } else {
                knots.extend([pair[0]; 3]);
                poles.extend(piece.control_points.iter().skip(1).copied());
            }
            knots.extend(interior.iter().copied());
            parameters.extend(local[..per].iter().copied());
        }
        knots.extend([1.0; 4]);
        parameters.push(1.0);
        let fitted = NurbsCurve::new(3, knots, poles).map_err(geometry)?;
        let mut deviation = 0.0_f64;
        for pair in parameters.windows(2) {
            for fraction in [0.25, 0.5, 0.75] {
                let t = pair[0] + (pair[1] - pair[0]) * fraction;
                let gap = fitted.evaluate(t).map_err(geometry)?.sub(curve.evaluate(t).map_err(geometry)?).length();
                deviation = deviation.max(gap);
            }
        }
        if !adaptive || deviation <= STAND_IN_DEVIATION || parameters.len() >= FIT_SAMPLES_MOST {
            return Ok((fitted, deviation));
        }
        per *= 2;
    }
}

/// A single Bézier curve on [0, 1] through `poles`.
fn bezier(poles: &[Vec3]) -> Result<NurbsCurve, NSidedRefusal> {
    let degree = poles.len() - 1;
    let mut knots = vec![0.0; degree + 1];
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    NurbsCurve::new(degree, knots, poles.iter().map(|pole| Vec4::from_point(*pole, 1.0)).collect()).map_err(geometry)
}

/// The cubic interpolant over [0, 1] of the vector field `f` on the knots
/// of the boundary it travels with (`hint`, interior knots in the ribbon's
/// own parameter; the eighths when there are fewer than seven of them),
/// interpolating at the Greville points, with its end derivatives made `f`'s
/// own (rows of the interpolation in place of the second and second-to-last
/// Greville points). A Coons patch carries the union of its boundaries' and
/// ribbons' knots, so a ribbon on its boundary's knots adds none; a 33-point
/// ribbon added up to 31 to every patch (60 knots a direction on a fillet
/// star, most of a box's build in tessellating them). Read against `f` at
/// 513 even points and its spans' quarter points; when it misses by more
/// than 1e-9 of `f`'s size and by more than half again what the 33-point
/// ribbon ([`sampled`]) misses there, it gives way to that ribbon, as it does
/// for a boundary with more than 64 interior knots (its solve is dense).
fn sampled_on(f: impl Fn(f64) -> Result<Vec3, NSidedRefusal>, hint: &[f64]) -> Result<NurbsCurve, NSidedRefusal> {
    let mut interior: Vec<f64> = hint.iter().copied().filter(|knot| *knot > 1e-9 && *knot < 1.0 - 1e-9).collect();
    interior.sort_by(f64::total_cmp);
    interior.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
    if interior.len() < 7 {
        interior = (1..8).map(|k| k as f64 / 8.0).collect();
    }
    // a boundary refined past the 33-point ribbon's own size gains nothing
    // from its knots and would make the dense solve below its cost
    if interior.len() > 64 {
        return sampled(f);
    }
    let mut knots = vec![0.0; 4];
    knots.extend(interior);
    knots.extend([1.0; 4]);
    let n = knots.len() - 4;
    let greville: Vec<f64> = (0..n).map(|i| (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0).collect();
    // values at the Greville points but the second and the second-to-last,
    // whose rows carry the end derivatives instead
    let (start, end) = end_derivatives(&f)?;
    let mut matrix: Vec<Vec<f64>> = Vec::with_capacity(n);
    let mut rows: Vec<Vec3> = Vec::with_capacity(n);
    for (i, t) in greville.iter().enumerate() {
        if i == 1 {
            let c = 3.0 / (knots[4] - knots[1]);
            let mut row = vec![0.0; n];
            row[0] = -c;
            row[1] = c;
            matrix.push(row);
            rows.push(start);
        } else if i == n - 2 {
            let c = 3.0 / (knots[n + 2] - knots[n - 1]);
            let mut row = vec![0.0; n];
            row[n - 2] = -c;
            row[n - 1] = c;
            matrix.push(row);
            rows.push(end);
        } else {
            matrix.push(cubic_basis(&knots, *t));
            rows.push(f(*t)?);
        }
    }
    let mut axes = Vec::with_capacity(3);
    for axis in 0..3 {
        let rhs: Vec<f64> = rows.iter().map(|v| [v.x, v.y, v.z][axis]).collect();
        axes.push(crate::solve_dense(matrix.clone(), rhs).map_err(geometry)?);
    }
    let poles: Vec<Vec4> = (0..n).map(|i| Vec4::from_point(Vec3::new(axes[0][i], axes[1][i], axes[2][i]), 1.0)).collect();
    let curve = NurbsCurve::new(3, knots.clone(), poles).map_err(geometry)?;
    let values: Vec<Vec3> = greville.iter().map(|t| f(*t)).collect::<Result<_, _>>()?;
    // read against `f` at 513 even points and the quarter points of its
    // spans, and against what the 33-point ribbon misses there
    let size = values.iter().map(|v| v.length()).fold(0.0_f64, f64::max).max(f64::MIN_POSITIVE);
    let mut points: Vec<f64> = (0..=512).map(|i| i as f64 / 512.0).collect();
    for span in 3..n {
        let (a, b) = (knots[span], knots[span + 1]);
        if b > a {
            points.extend([0.25, 0.5, 0.75].map(|fraction| a + (b - a) * fraction));
        }
    }
    let worst = |candidate: &NurbsCurve| -> Result<f64, NSidedRefusal> {
        let mut worst = 0.0_f64;
        for t in &points {
            worst = worst.max(candidate.evaluate(*t).map_err(geometry)?.sub(f(*t)?).length());
        }
        Ok(worst)
    };
    let thinned = worst(&curve)?;
    if thinned <= 1e-9 * size {
        return Ok(curve);
    }
    let dense = sampled(&f)?;
    if thinned <= 1.5 * worst(&dense)? {
        Ok(curve)
    } else {
        Ok(dense)
    }
}

/// The interior knots of `curve` (over [0, 1]), mirrored (`1 − t`) when the
/// ribbon runs the other way along it.
fn interior_knots(curve: &NurbsCurve, mirrored: bool) -> Vec<f64> {
    let p = curve.degree;
    let knots = &curve.knots[p + 1..curve.knots.len() - p - 1];
    knots.iter().map(|knot| if mirrored { 1.0 - knot } else { *knot }).collect()
}

/// `BREP_N_SIDED_SLIVER_KNOTS` (on by default; `=0` keeps every knot): see
/// `without_sliver_knots`.
pub fn sliver_knots_on() -> bool {
    crate::switch!(live "BREP_N_SIDED_SLIVER_KNOTS", true)
}

/// How near (in a side's [0, 1]) two knots by its midpoint leave a sliver
/// span between them, and how far (relative to a side's size, or to a
/// field's smallest pole) the curve may move when one of them is removed.
const SLIVER_KNOT_WINDOW: f64 = 1e-6;
const SLIVER_KNOT_TOLERANCE: f64 = 1e-12;

/// `curve` (on [0, 1]) with as few simple interior knots within
/// SLIVER_KNOT_WINDOW of 0.5 as it can keep unchanged: each such knot is
/// removed where that moves the curve by at most `tolerance` (Piegl and
/// Tiller's single knot removal, A5.8, on the homogeneous poles), while at
/// least one stays; a knot that would move it more stays.
fn without_sliver_knots(curve: &NurbsCurve, tolerance: f64) -> NurbsCurve {
    let mut current = curve.clone();
    let mut kept: Vec<f64> = Vec::new();
    loop {
        let knots = &current.knots;
        // farthest from the midpoint first: the knot a fitted side carries
        // by its midpoint stays where another can go
        let mut band: Vec<usize> = (current.degree + 1..knots.len() - current.degree - 1)
            .filter(|&r| (knots[r] - 0.5).abs() <= SLIVER_KNOT_WINDOW && knots.iter().filter(|other| **other == knots[r]).count() == 1)
            .collect();
        band.sort_by(|&a, &b| (knots[b] - 0.5).abs().total_cmp(&(knots[a] - 0.5).abs()));
        if band.len() < 2 {
            return current;
        }
        let Some(&r) = band.iter().find(|&&r| !kept.contains(&knots[r])) else { return current };
        match remove_simple_knot(&current, r, tolerance) {
            Some(removed) => current = removed,
            None => kept.push(current.knots[r]),
        }
    }
}

/// `curve` without its simple knot `knots[r]`, if that moves it by at most
/// `tolerance` (A5.8 with one removal; the check is on the homogeneous poles).
fn remove_simple_knot(curve: &NurbsCurve, r: usize, tolerance: f64) -> Option<NurbsCurve> {
    let p = curve.degree;
    let knots = &curve.knots;
    let poles = &curve.control_points;
    let u = knots[r];
    let (first, last) = (r - p, r - 1);
    let off = first - 1;
    let lerp = |a: Vec4, b: Vec4, ta: f64, tb: f64| Vec4 { x: a.x * ta + b.x * tb, y: a.y * ta + b.y * tb, z: a.z * ta + b.z * tb, w: a.w * ta + b.w * tb };
    let mut temp = vec![Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }; 2 * p + 1];
    temp[0] = poles[off];
    temp[last + 1 - off] = poles[last + 1];
    let (mut i, mut j) = (first, last);
    let (mut ii, mut jj) = (1usize, last - off);
    while j > i {
        let alfi = (u - knots[i]) / (knots[i + p + 1] - knots[i]);
        let alfj = (u - knots[j]) / (knots[j + p + 1] - knots[j]);
        if !(alfi.abs() > 0.0) || !((1.0 - alfj).abs() > 0.0) {
            return None;
        }
        temp[ii] = lerp(poles[i], temp[ii - 1], 1.0 / alfi, -(1.0 - alfi) / alfi);
        temp[jj] = lerp(poles[j], temp[jj + 1], 1.0 / (1.0 - alfj), -alfj / (1.0 - alfj));
        i += 1;
        ii += 1;
        j -= 1;
        jj -= 1;
    }
    let distance = |a: Vec4, b: Vec4| ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2) + (a.w - b.w).powi(2)).sqrt();
    let off_by = if j < i {
        distance(temp[ii - 1], temp[jj + 1])
    } else {
        let alfi = (u - knots[i]) / (knots[i + p + 1] - knots[i]);
        distance(poles[i], lerp(temp[ii + 1], temp[ii - 1], alfi, 1.0 - alfi))
    };
    if !(off_by <= tolerance) {
        return None;
    }
    let mut new_poles = poles.clone();
    let (mut i, mut j) = (first, last);
    while j > i {
        new_poles[i] = temp[i - off];
        new_poles[j] = temp[j - off];
        i += 1;
        j -= 1;
    }
    let fout = (2 * r - 1 - p) / 2;
    new_poles.remove(fout);
    let mut new_knots = knots.clone();
    new_knots.remove(r);
    NurbsCurve::new(p, new_knots, new_poles).ok()
}

/// `curve` over [0, 1] with its knot `knot` carried onto 0.5: the knot
/// raised to full multiplicity (exact), then [0, knot] mapped onto [0, ½]
/// and [knot, 1] onto [½, 1]. The same points in the same order; only the
/// speeds of the two halves change, by `knot / ½` and `(1 − knot) / ½`.
fn knot_onto_midpoint(curve: &NurbsCurve, knot: f64) -> Result<NurbsCurve, NSidedRefusal> {
    let present = curve.knots.iter().filter(|value| **value == knot).count();
    let full = if present < curve.degree { curve.insert_knot(knot, curve.degree - present).map_err(geometry)? } else { curve.clone() };
    let knots: Vec<f64> = full
        .knots
        .iter()
        .map(|value| if *value <= knot { value * 0.5 / knot } else { 0.5 + (value - knot) * 0.5 / (1.0 - knot) })
        .collect();
    NurbsCurve::new(full.degree, knots, full.control_points.clone()).map_err(geometry)
}

/// The cubic interpolant over [0, 1] of the vector field `f`, sampled at 33
/// even parameters, with its end derivatives made `f`'s own (read by
/// third-order one-sided differences): exact for a field of degree three or
/// less. A ribbon's end derivative is a corner twist of the patch it is
/// handed to; the interpolant's own missed it by its interpolation error,
/// which on a fast-turning field (a fillet star whose rounds meet at radii
/// 0.9 against 3) left the two patches beside a spoke's midpoint end with
/// twists 7e-4 apart and their normals 3.2e-4 rad apart across the spoke.
fn sampled(f: impl Fn(f64) -> Result<Vec3, NSidedRefusal>) -> Result<NurbsCurve, NSidedRefusal> {
    const SAMPLES: usize = 33;
    let parameters: Vec<f64> = (0..SAMPLES).map(|index| index as f64 / (SAMPLES - 1) as f64).collect();
    let values: Vec<Vec3> = parameters.iter().map(|t| f(*t)).collect::<Result<_, _>>()?;
    let curve = interpolate_curve(&values, 3, &parameters).map_err(geometry)?;
    with_end_derivatives(curve, &f)
}

/// `f`'s derivatives at 0 and 1 by third-order one-sided differences, exact
/// for a cubic field.
fn end_derivatives(f: &impl Fn(f64) -> Result<Vec3, NSidedRefusal>) -> Result<(Vec3, Vec3), NSidedRefusal> {
    const STEP: f64 = 1e-4;
    let (first_value, last_value) = (f(0.0)?, f(1.0)?);
    let start = f(STEP)?
        .scale(18.0)
        .sub(f(2.0 * STEP)?.scale(9.0))
        .add(f(3.0 * STEP)?.scale(2.0))
        .sub(first_value.scale(11.0))
        .scale(1.0 / (6.0 * STEP));
    let end = last_value
        .scale(11.0)
        .sub(f(1.0 - STEP)?.scale(18.0))
        .add(f(1.0 - 2.0 * STEP)?.scale(9.0))
        .sub(f(1.0 - 3.0 * STEP)?.scale(2.0))
        .scale(1.0 / (6.0 * STEP));
    Ok((start, end))
}

/// `curve` (a clamped cubic over [0, 1] interpolating `f`) with its end
/// derivatives made `f`'s own, read by third-order one-sided differences
/// (exact for a cubic field, whose interpolant they then leave as it is):
/// the second and second-to-last poles moved along them.
fn with_end_derivatives(curve: NurbsCurve, f: &impl Fn(f64) -> Result<Vec3, NSidedRefusal>) -> Result<NurbsCurve, NSidedRefusal> {
    let (start, end) = end_derivatives(f)?;
    // a clamped cubic leaves P0 along 3 (P1 - P0) / (u4 - u1) and reaches
    // P(n-1) along 3 (P(n-1) - P(n-2)) / (u(n+2) - u(n-1))
    let (degree, knots) = (curve.degree, curve.knots.clone());
    let mut poles = curve.control_points.clone();
    let n = poles.len();
    if degree != 3 || n < 4 || poles.iter().any(|pole| (pole.w - 1.0).abs() > POLYNOMIAL_BAND) {
        return Ok(curve);
    }
    let point = |pole: &Vec4| Vec3::new(pole.x, pole.y, pole.z);
    let first = point(&poles[0]).add(start.scale((knots[4] - knots[1]) / 3.0));
    let last = point(&poles[n - 1]).sub(end.scale((knots[n + 2] - knots[n - 1]) / 3.0));
    poles[1] = Vec4::from_point(first, 1.0);
    poles[n - 2] = Vec4::from_point(last, 1.0);
    NurbsCurve::new(degree, knots, poles).map_err(geometry)
}

/// The least-squares `(λ, μ)` with `λ p + μ q` nearest `target`.
fn in_plane(target: Vec3, p: Vec3, q: Vec3) -> Result<(f64, f64), NSidedRefusal> {
    let (pp, pq, qq) = (p.dot(p), p.dot(q), q.dot(q));
    let (tp, tq) = (target.dot(p), target.dot(q));
    let determinant = pp * qq - pq * pq;
    if !(determinant > 1e-24 * pp * qq) || !(determinant.is_finite()) {
        return Err(NSidedRefusal::Centre { detail: "two directions that should span a plane are parallel".into() });
    }
    Ok(((tp * qq - tq * pq) / determinant, (tq * pp - tp * pq) / determinant))
}

/// The unit normal `∂S/∂u × ∂S/∂v`, `None` where it vanishes.
fn unit_normal(surface: &NurbsSurface, u: f64, v: f64) -> Result<Option<Vec3>, NSidedRefusal> {
    let (_, su, sv) = surface.deriv1(u, v).map_err(geometry)?;
    let normal = su.cross(sv);
    if !(normal.length() > 1e-12 * su.length() * sv.length()) {
        return Ok(None);
    }
    Ok(normal.normalized().ok())
}

/// The angle between two unit vectors, `atan2(|a × b|, a · b)`.
fn angle(a: Vec3, b: Vec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b))
}

/// The angle between two lines (unit vectors, sign ignored).
fn line_angle(a: Vec3, b: Vec3) -> f64 {
    a.cross(b).length().atan2(a.dot(b).abs())
}


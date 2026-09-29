//! Natural extension of a NURBS surface: the terminal Bézier span of one
//! parameter direction, extrapolated by a bounded parameter increment and
//! appended as a fresh span.
//!
//! This is the real answer to the question `evaluate_extended` only pretends
//! to answer. `evaluate_extended` continues an open direction along the
//! boundary TANGENT PLANE, without bound and without a surface: a solver that
//! has no solution slides onto that fictitious plane and converges there, far
//! outside the patch, with nothing downstream able to tell. An extension built
//! here is a `NurbsSurface` — the analytic continuation of the terminal
//! polynomial, with a domain that ends somewhere, so a solve that leaves the
//! extension leaves the surface and is refused rather than answered.
//!
//! The construction (Piegl & Tiller §5.3 knot insertion, plus the blossom form
//! of de Casteljau) is:
//!
//! 1. Refine a scratch copy so the last interior knot reaches multiplicity
//!    `degree`; the terminal `degree + 1` control points of each control line
//!    are then the Bézier control points of the terminal span `[a, b]`.
//! 2. Run one de Casteljau triangle in HOMOGENEOUS coordinates at the
//!    off-interval parameter, and read the new span's control points off an
//!    anti-diagonal of it. For the high side the continuation over `[b, b + δ]`
//!    is `cᵢ = f(b^{p-i}, x^i)` with `x = b + δ`; for the low side the
//!    continuation over `[a - δ, a]` is `eᵢ = f(x^{p-i}, a^i)` with
//!    `x = a - δ`. Both are one triangle with a single ratio.
//! 3. Append (or prepend) those `degree` control lines and the matching knots.
//!
//! The emitted surface keeps the input's knots and control net VERBATIM over
//! the original domain — nothing is refined in the output — so the original
//! parameterization survives exactly. Two things follow: evaluation inside the
//! old domain is bit-identical to the input's, and every pcurve already drawn
//! on the face stays valid with no remap (unlike the analytic
//! `extend_ruled_carrier`, which rescales v and has to walk the coedges).
//!
//! Every refusal is typed and carries the quantity that was measured.

use super::NurbsSurface;
use crate::curve::{knot_domain, KNOT_IDENTITY_TOL};
use crate::{NurbsCurve, Vec3, Vec4};

/// Smallest weight an extrapolated control point may carry. Matches
/// `NurbsSurface::ensure_valid`'s own `EPS` gate, so a strip this accepts is a
/// strip the constructor accepts.
const WEIGHT_EPS: f64 = 1e-12;

/// The largest angle the extension's cross-boundary tangent may turn through
/// before the extension is called a FOLD. π/2 is the sign change: past it the
/// extension has stopped going outwards and is doubling back over the patch it
/// came from, which is never a usable carrier.
pub const MAXIMUM_FOLD_ANGLE: f64 = std::f64::consts::FRAC_PI_2;

/// The largest the appended strip may be, as a multiple of the surface's own
/// 3D extent in the direction being extended: the extension may not be longer
/// than the surface it continues. A polynomial extrapolated by a fraction `r`
/// of its own span diverges like `r^degree`, so this is a backstop and not a
/// quality bar — callers that need a trustworthy extension ask for a small
/// increment and check the result, they do not lean on this limit.
pub const MAXIMUM_GROWTH_RATIO: f64 = 1.0;

/// How many stations across the far parameter direction the fold and growth
/// measures sample.
const CROSS_SAMPLES: usize = 9;
/// How many stations ALONG the extension the fold measure samples.
const FOLD_STATIONS: [f64; 4] = [0.25, 0.5, 0.75, 1.0];
/// How many chords the growth measure uses for the surface's own extent.
const EXTENT_SAMPLES: usize = 17;

/// Where a non-positive weight was found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WeightSite {
    /// An extrapolated CONTROL point of the appended strip, by its position in
    /// the extended net.
    Control { row: usize, column: usize },
    /// A PARAMETER inside the appended span. Every control weight is positive
    /// and the weight polynomial still dips through zero between them, which no
    /// check on the control net alone can see.
    Parameter { u: f64, v: f64 },
}

impl WeightSite {
    fn describe(&self) -> String {
        match self {
            Self::Control { row, column } => {
                format!("the extrapolated control point at row {row}, column {column}")
            }
            Self::Parameter { u, v } => format!("({u:.6}, {v:.6}) inside the appended span"),
        }
    }
}

/// Which boundary of the parameter square an extension grows from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceSide {
    /// Below the start of the u domain.
    UMin,
    /// Beyond the end of the u domain.
    UMax,
    /// Below the start of the v domain.
    VMin,
    /// Beyond the end of the v domain.
    VMax,
}

impl SurfaceSide {
    /// Whether this side extends the u direction (rather than v).
    pub fn is_u(self) -> bool {
        matches!(self, Self::UMin | Self::UMax)
    }

    /// Whether this side grows BELOW the domain start (rather than beyond its
    /// end).
    pub fn is_low(self) -> bool {
        matches!(self, Self::UMin | Self::VMin)
    }
}

/// Why a natural extension produced nothing. Each variant carries what was
/// measured, so a caller can report the number rather than the verdict.
#[derive(Clone, Debug)]
pub enum ExtendRefusal {
    /// The requested direction joins itself. A closed direction has no outside
    /// to extend INTO — it wraps — and appending a span there would overlay the
    /// surface on itself. `direction_u` says which direction was asked for.
    ClosedDirection { direction_u: bool },
    /// The parameter increment is not one: not positive, not finite, or not
    /// large enough to be a knot of its own. `KNOT_IDENTITY_TOL` is the band
    /// within which this kernel treats two knots as THE SAME knot, so an
    /// increment inside it appends a zero-width span whose basis functions are
    /// all zero — an extension that evaluates to a zero-weight point everywhere
    /// it was supposed to be new. Measured 2026-09-12: a caller asking for a
    /// 3e-18 extension off a noise-level overrun estimate produced exactly that.
    DegenerateIncrement { delta: f64 },
    /// The knot vector's clamped end is not at multiplicity `degree + 1`, so
    /// the terminal `degree + 1` control points are not the terminal Bézier
    /// span's and the construction does not apply. Vendor exporters do emit
    /// over-clamped ends (see `knot_find_span`); extending one needs the extra
    /// knots stripped first, which this does not do.
    UnsupportedKnots { multiplicity: usize, expected: usize },
    /// The rational extension's denominator has stopped being positive — at an
    /// extrapolated control point, or at a parameter between them. A rational
    /// patch extrapolated far enough reaches this whenever its weight
    /// polynomial has a real root outside the interval it was fitted on, and a
    /// patch whose control weights are all still positive can already have
    /// dipped through zero between them (measured on imported B-spline faces:
    /// `abc_00000013.step` does both).
    InvalidWeight { weight: f64, site: WeightSite },
    /// The extension turns back on itself: the cross-boundary tangent at some
    /// station of the extension makes `angle` with the same tangent at the
    /// boundary, exceeding `limit`.
    Fold { angle: f64, limit: f64 },
    /// The appended strip reaches `ratio` times the surface's own 3D extent in
    /// the extended direction, exceeding `limit`.
    ExcessiveGrowth { ratio: f64, limit: f64 },
    /// A lower-level failure (knot insertion, a surface constructor, an
    /// evaluation), carried verbatim so the call site can prefix it with its
    /// own operation name.
    Failed(String),
}

impl ExtendRefusal {
    /// The failure text, for a site with no more specific wording.
    pub fn describe(&self) -> String {
        match self {
            Self::ClosedDirection { direction_u } => format!(
                "the {} direction is closed — a closed direction wraps rather than extending",
                if *direction_u { "u" } else { "v" }
            ),
            Self::DegenerateIncrement { delta } => format!(
                "the parameter increment {delta:e} is not a usable one — it must be positive, \
                 finite, and larger than the {KNOT_IDENTITY_TOL:e} band within which two knots \
                 are the same knot"
            ),
            Self::UnsupportedKnots {
                multiplicity,
                expected,
            } => format!(
                "the extended end's knot multiplicity is {multiplicity}, not the clamped \
                 {expected} this construction needs"
            ),
            Self::InvalidWeight { weight, site } => format!(
                "the rational extension's weight is {weight:.3e} at {} — it has passed a root \
                 of its own weight polynomial",
                site.describe()
            ),
            Self::Fold { angle, limit } => format!(
                "the extension folds back: the cross-boundary tangent turns {:.2}° \
                 (limit {:.2}°)",
                angle.to_degrees(),
                limit.to_degrees()
            ),
            Self::ExcessiveGrowth { ratio, limit } => format!(
                "the extension reaches {ratio:.3}× the surface's own extent (limit {limit:.3})"
            ),
            Self::Failed(message) => message.clone(),
        }
    }
}

impl std::fmt::Display for ExtendRefusal {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.describe())
    }
}

impl NurbsCurve {
    /// Continue this curve past its low (`low`) or high end by `delta_param`,
    /// as the analytic continuation of its own terminal Bézier span: the curve
    /// analogue of [`NurbsSurface::extend_natural`], through the same knot
    /// and control-point construction, so a surface's isoline and the
    /// continued isoline agree. Everything over the original domain survives.
    /// A rational continuation whose appended weights are not positive is
    /// refused; the caller samples the span where it needs more than that.
    pub fn extend_natural(&self, low: bool, delta_param: f64) -> Result<NurbsCurve, ExtendRefusal> {
        if !delta_param.is_finite() || delta_param <= KNOT_IDENTITY_TOL {
            return Err(ExtendRefusal::DegenerateIncrement { delta: delta_param });
        }
        let knots = extend_knots(&self.knots, self.degree, low, delta_param)?;
        let strip = extrapolate_line(&self.control_points, &self.knots, self.degree, low, delta_param)?;
        if let Some(bad) = strip.iter().find(|point| !(point.w > 0.0)) {
            return Err(ExtendRefusal::Failed(format!(
                "NurbsCurve::extend_natural: an appended weight is {:.3e}",
                bad.w
            )));
        }
        let mut points = Vec::with_capacity(self.control_points.len() + self.degree);
        if low {
            points.extend_from_slice(&strip);
            points.extend_from_slice(&self.control_points);
        } else {
            points.extend_from_slice(&self.control_points);
            points.extend_from_slice(&strip);
        }
        NurbsCurve::new(self.degree, knots, points).map_err(ExtendRefusal::Failed)
    }
}

impl NurbsSurface {
    /// Continue this surface past one boundary of its parameter square by
    /// `delta_param`, as the analytic continuation of its own terminal Bézier
    /// span. See the module comment for the construction and for what survives
    /// of the input (everything, over the original domain).
    pub fn extend_natural(
        &self,
        side: SurfaceSide,
        delta_param: f64,
    ) -> Result<NurbsSurface, ExtendRefusal> {
        if !delta_param.is_finite() || delta_param <= KNOT_IDENTITY_TOL {
            return Err(ExtendRefusal::DegenerateIncrement {
                delta: delta_param,
            });
        }
        let along_u = side.is_u();
        let (closed_u, closed_v) = self.closed_directions().map_err(ExtendRefusal::Failed)?;
        if (along_u && closed_u) || (!along_u && closed_v) {
            return Err(ExtendRefusal::ClosedDirection {
                direction_u: along_u,
            });
        }

        let (degree, knots) = if along_u {
            (self.degree_u, &self.knots_u)
        } else {
            (self.degree_v, &self.knots_v)
        };
        let extended_knots = extend_knots(knots, degree, side.is_low(), delta_param)?;

        // Control LINES along the extended direction: the columns of the net
        // when extending u, its rows when extending v.
        let rows = self.control_points.len();
        let columns = self.control_points[0].len();
        let lines: Vec<Vec<Vec4>> = if along_u {
            (0..columns)
                .map(|column| (0..rows).map(|row| self.control_points[row][column]).collect())
                .collect()
        } else {
            self.control_points.clone()
        };
        let mut strips: Vec<Vec<Vec4>> = Vec::with_capacity(lines.len());
        for line in &lines {
            strips.push(extrapolate_line(
                line,
                knots,
                degree,
                side.is_low(),
                delta_param,
            )?);
        }

        // The strip's control lines, indexed the way the net is: outermost
        // last on the high side, outermost FIRST on the low side (control
        // points run in increasing parameter either way).
        let mut extended: Vec<Vec<Vec4>> = if along_u {
            let mut net: Vec<Vec<Vec4>> = Vec::with_capacity(rows + degree);
            if side.is_low() {
                for step in 0..degree {
                    net.push(strips.iter().map(|strip| strip[step]).collect());
                }
            }
            net.extend(self.control_points.iter().cloned());
            if !side.is_low() {
                for step in 0..degree {
                    net.push(strips.iter().map(|strip| strip[step]).collect());
                }
            }
            net
        } else {
            self.control_points
                .iter()
                .zip(&strips)
                .map(|(row, strip)| {
                    let mut line: Vec<Vec4> = Vec::with_capacity(columns + degree);
                    if side.is_low() {
                        line.extend_from_slice(strip);
                        line.extend_from_slice(row);
                    } else {
                        line.extend_from_slice(row);
                        line.extend_from_slice(strip);
                    }
                    line
                })
                .collect()
        };

        // Every APPENDED control point must be a usable homogeneous point.
        // The originals are the input's and were validated when it was built.
        let appended_rows: Vec<usize> = if along_u {
            if side.is_low() {
                (0..degree).collect()
            } else {
                (extended.len() - degree..extended.len()).collect()
            }
        } else {
            (0..extended.len()).collect()
        };
        let appended_columns: Vec<usize> = if along_u {
            (0..columns).collect()
        } else if side.is_low() {
            (0..degree).collect()
        } else {
            (columns..columns + degree).collect()
        };
        for &row in &appended_rows {
            for &column in &appended_columns {
                let point = extended[row][column];
                if !point.w.is_finite() || point.w <= WEIGHT_EPS {
                    return Err(ExtendRefusal::InvalidWeight {
                        weight: point.w,
                        site: WeightSite::Control { row, column },
                    });
                }
                if ![point.x, point.y, point.z].iter().all(|v| v.is_finite()) {
                    return Err(ExtendRefusal::InvalidWeight {
                        weight: f64::NAN,
                        site: WeightSite::Control { row, column },
                    });
                }
            }
        }

        let (degree_u, degree_v) = (self.degree_u, self.degree_v);
        let (knots_u, knots_v) = if along_u {
            (extended_knots, self.knots_v.clone())
        } else {
            (self.knots_u.clone(), extended_knots)
        };
        let net = std::mem::take(&mut extended);
        let candidate = NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, net)
            .map_err(ExtendRefusal::Failed)?;
        // Before anything EVALUATES the extension: a positive control net does
        // not make a positive denominator.
        check_weight_field(&candidate, side, delta_param)?;
        check_fold(self, &candidate, side, delta_param)?;
        check_growth(self, &candidate, side, delta_param)?;
        Ok(candidate)
    }
}

/// The extended knot vector. The original knots survive at their original
/// VALUES — one copy of the clamped end is re-used as the new interior knot of
/// multiplicity `degree`, which is what lets the appended span carry the same
/// polynomial without refining anything that was already there.
fn extend_knots(
    knots: &[f64],
    degree: usize,
    low: bool,
    delta: f64,
) -> Result<Vec<f64>, ExtendRefusal> {
    let [start, end] = knot_domain(knots, degree);
    let boundary = if low { start } else { end };
    let multiplicity = knots
        .iter()
        .filter(|knot| (**knot - boundary).abs() <= KNOT_IDENTITY_TOL)
        .count();
    if multiplicity != degree + 1 {
        return Err(ExtendRefusal::UnsupportedKnots {
            multiplicity,
            expected: degree + 1,
        });
    }
    let mut extended = Vec::with_capacity(knots.len() + degree);
    if low {
        extended.extend(std::iter::repeat_n(start - delta, degree + 1));
        extended.extend_from_slice(&knots[1..]);
    } else {
        extended.extend_from_slice(&knots[..knots.len() - 1]);
        extended.extend(std::iter::repeat_n(end + delta, degree + 1));
    }
    Ok(extended)
}

/// One control line's `degree` new control points, in increasing parameter
/// order. See the module comment for the two anti-diagonals.
fn extrapolate_line(
    line: &[Vec4],
    knots: &[f64],
    degree: usize,
    low: bool,
    delta: f64,
) -> Result<Vec<Vec4>, ExtendRefusal> {
    let curve = NurbsCurve::new(degree, knots.to_vec(), line.to_vec())
        .map_err(ExtendRefusal::Failed)?;
    let [start, end] = knot_domain(knots, degree);
    // The terminal Bézier span's far knot: the last knot before the end on the
    // high side, the first knot after the start on the low side. With no
    // interior knot at all the surface is already a single Bézier span and the
    // far knot is the opposite boundary.
    let (interval_start, interval_end) = if low {
        let next = knots
            .iter()
            .copied()
            .find(|knot| *knot > start + KNOT_IDENTITY_TOL)
            .unwrap_or(end);
        (start, next)
    } else {
        let previous = knots
            .iter()
            .rev()
            .copied()
            .find(|knot| *knot < end - KNOT_IDENTITY_TOL)
            .unwrap_or(start);
        (previous, end)
    };
    // Raising the far knot to multiplicity `degree` isolates the terminal span
    // as a Bézier segment. `insert_knot` clamps the request to what is missing,
    // so a span that is already Bézier is returned untouched.
    let split_at = if low { interval_end } else { interval_start };
    let refined = curve
        .insert_knot(split_at, degree)
        .map_err(ExtendRefusal::Failed)?;
    let bezier: Vec<Vec4> = if low {
        refined.control_points[..=degree].to_vec()
    } else {
        refined.control_points[refined.control_points.len() - degree - 1..].to_vec()
    };
    let span = interval_end - interval_start;
    if !(span > 0.0) {
        return Err(ExtendRefusal::Failed(format!(
            "NurbsSurface::extend_natural: terminal span [{interval_start}, {interval_end}] \
             has no width"
        )));
    }
    // x = (1 - ratio)·a + ratio·b, for x one increment outside the span.
    let ratio = if low {
        -delta / span
    } else {
        1.0 + delta / span
    };
    let mut triangle: Vec<Vec<Vec4>> = Vec::with_capacity(degree + 1);
    triangle.push(bezier);
    for level in 1..=degree {
        let previous = &triangle[level - 1];
        let next: Vec<Vec4> = (0..previous.len() - 1)
            .map(|index| {
                previous[index]
                    .scale(1.0 - ratio)
                    .add(previous[index + 1].scale(ratio))
            })
            .collect();
        triangle.push(next);
    }
    Ok(if low {
        // eᵢ = f(x^{p-i}, a^i) = d₀^{(p-i)}; the new points are e₀ … e_{p-1}
        // (e_p is the line's existing first control point).
        (0..degree).map(|i| triangle[degree - i][0]).collect()
    } else {
        // cᵢ = f(b^{p-i}, x^i) = d_{p-i}^{(i)}; the new points are c₁ … c_p
        // (c₀ is the line's existing last control point).
        (1..=degree).map(|i| triangle[i][degree - i]).collect()
    })
}

/// The rational denominator must stay positive across the whole appended span,
/// not merely at its control points. A weight polynomial interpolates its
/// control weights and can dip below them, so this samples the span itself —
/// the only check that catches an extension whose surface is singular INSIDE.
fn check_weight_field(
    extended: &NurbsSurface,
    side: SurfaceSide,
    delta: f64,
) -> Result<(), ExtendRefusal> {
    let along_u = side.is_u();
    let [u0, u1] = extended.domain_u().map_err(ExtendRefusal::Failed)?;
    let [v0, v1] = extended.domain_v().map_err(ExtendRefusal::Failed)?;
    // The appended span, in the extended surface's own domain.
    let (from, to) = if side.is_low() {
        let start = if along_u { u0 } else { v0 };
        (start, start + delta)
    } else {
        let end = if along_u { u1 } else { v1 };
        (end - delta, end)
    };
    for cross in cross_stations(extended, along_u)? {
        for index in 0..=CROSS_SAMPLES {
            let at = from + (to - from) * index as f64 / CROSS_SAMPLES as f64;
            let (u, v) = if along_u { (at, cross) } else { (cross, at) };
            let weight = extended
                .evaluate_homogeneous(u, v)
                .map_err(ExtendRefusal::Failed)?
                .w;
            if !weight.is_finite() || weight <= WEIGHT_EPS {
                return Err(ExtendRefusal::InvalidWeight {
                    weight,
                    site: WeightSite::Parameter { u, v },
                });
            }
        }
    }
    Ok(())
}

/// Parameters at which the fold and growth measures cross the OTHER direction.
fn cross_stations(surface: &NurbsSurface, along_u: bool) -> Result<Vec<f64>, ExtendRefusal> {
    let [low, high] = if along_u {
        surface.domain_v().map_err(ExtendRefusal::Failed)?
    } else {
        surface.domain_u().map_err(ExtendRefusal::Failed)?
    };
    Ok((0..CROSS_SAMPLES)
        .map(|index| low + (high - low) * index as f64 / (CROSS_SAMPLES - 1) as f64)
        .collect())
}

/// The cross-boundary tangent must not turn past `MAXIMUM_FOLD_ANGLE` anywhere
/// along the extension. Measured against the ORIGINAL surface's own boundary
/// tangent, so a fold is reported relative to the thing being continued.
fn check_fold(
    original: &NurbsSurface,
    extended: &NurbsSurface,
    side: SurfaceSide,
    delta: f64,
) -> Result<(), ExtendRefusal> {
    let along_u = side.is_u();
    let [start, end] = if along_u {
        original.domain_u().map_err(ExtendRefusal::Failed)?
    } else {
        original.domain_v().map_err(ExtendRefusal::Failed)?
    };
    let boundary = if side.is_low() { start } else { end };
    let direction = if side.is_low() { -1.0 } else { 1.0 };
    let mut worst = 0.0f64;
    for cross in cross_stations(original, along_u)? {
        let reference = cross_tangent(original, along_u, boundary, cross)?;
        let Ok(reference) = reference.normalized() else {
            continue;
        };
        for fraction in FOLD_STATIONS {
            let at = boundary + direction * delta * fraction;
            let tangent = cross_tangent(extended, along_u, at, cross)?;
            let Ok(tangent) = tangent.normalized() else {
                return Err(ExtendRefusal::Fold {
                    angle: std::f64::consts::PI,
                    limit: MAXIMUM_FOLD_ANGLE,
                });
            };
            let angle = reference.dot(tangent).clamp(-1.0, 1.0).acos();
            worst = worst.max(angle);
        }
    }
    if worst > MAXIMUM_FOLD_ANGLE {
        return Err(ExtendRefusal::Fold {
            angle: worst,
            limit: MAXIMUM_FOLD_ANGLE,
        });
    }
    Ok(())
}

fn cross_tangent(
    surface: &NurbsSurface,
    along_u: bool,
    at: f64,
    cross: f64,
) -> Result<Vec3, ExtendRefusal> {
    let (u, v) = if along_u { (at, cross) } else { (cross, at) };
    let derivatives = surface
        .derivatives(u, v, 1)
        .map_err(ExtendRefusal::Failed)?;
    Ok(if along_u {
        derivatives[1][0]
    } else {
        derivatives[0][1]
    })
}

/// The appended strip may not be longer than the surface it continues. The
/// growth ratio is the strip's largest 3D reach over the surface's largest 3D
/// extent in the same direction — a degenerate cross station (a pole, where the
/// extent collapses) therefore cannot inflate the ratio.
fn check_growth(
    original: &NurbsSurface,
    extended: &NurbsSurface,
    side: SurfaceSide,
    delta: f64,
) -> Result<(), ExtendRefusal> {
    let along_u = side.is_u();
    let [start, end] = if along_u {
        original.domain_u().map_err(ExtendRefusal::Failed)?
    } else {
        original.domain_v().map_err(ExtendRefusal::Failed)?
    };
    let boundary = if side.is_low() { start } else { end };
    let outside = if side.is_low() {
        start - delta
    } else {
        end + delta
    };
    let evaluate = |surface: &NurbsSurface, at: f64, cross: f64| -> Result<Vec3, ExtendRefusal> {
        let (u, v) = if along_u { (at, cross) } else { (cross, at) };
        surface.evaluate(u, v).map_err(ExtendRefusal::Failed)
    };
    let mut reach = 0.0f64;
    let mut extent = 0.0f64;
    for cross in cross_stations(original, along_u)? {
        let anchor = evaluate(original, boundary, cross)?;
        reach = reach.max(evaluate(extended, outside, cross)?.sub(anchor).length());
        let mut length = 0.0;
        let mut previous = evaluate(original, start, cross)?;
        for index in 1..EXTENT_SAMPLES {
            let at = start + (end - start) * index as f64 / (EXTENT_SAMPLES - 1) as f64;
            let point = evaluate(original, at, cross)?;
            length += point.sub(previous).length();
            previous = point;
        }
        extent = extent.max(length);
    }
    if extent <= 0.0 {
        return Err(ExtendRefusal::Failed(
            "NurbsSurface::extend_natural: the surface has no extent in the extended direction"
                .into(),
        ));
    }
    let ratio = reach / extent;
    if ratio > MAXIMUM_GROWTH_RATIO {
        return Err(ExtendRefusal::ExcessiveGrowth {
            ratio,
            limit: MAXIMUM_GROWTH_RATIO,
        });
    }
    Ok(())
}


//! Representation compatibility for the Coons constructors.
//!
//! Two curves can only become opposite boundaries of ONE tensor-product patch
//! when they share a degree, a knot vector and a parameter domain.  Everything
//! here reaches that state by EXACT algebra — affine knot re-mapping, degree
//! elevation and knot insertion — never by sampling and refitting, so the
//! curve that comes out is the curve that went in, bit for bit where no
//! operation was needed at all.
//!
//! The three primitives:
//!
//! * [`normalise_domain`] re-maps the knot domain onto `[0, 1]`.  Control
//!   points are untouched; a curve already on `[0, 1]` comes back with its
//!   knot floats unchanged (`(k - 0.0) / 1.0 == k`).
//! * [`elevate_degree`] raises the degree by `t`.  The curve is Bezier-
//!   decomposed with the shared [`NurbsCurve::insert_knot`], each segment is
//!   elevated by the closed binomial formula, the segments are rejoined, and
//!   the surplus junction knots are then REMOVED so each interior knot lands
//!   back at its original multiplicity `m` plus `t` — the minimal knot vector
//!   for the elevated curve.  Removal is cleanup only: it is applied one knot
//!   at a time and each removal is verified by re-inserting the knot and
//!   comparing control points, so a removal that would change the geometry is
//!   simply not taken (the knot stays, the curve is still exact).
//! * [`merge_knots`] refines two equal-degree curves on `[0, 1]` onto the
//!   union of their knots, then stamps ONE canonical knot array onto both so
//!   the tensor structure is exact rather than merely equal within a band.
//!
//! Knot values are compared with the kernel's [`crate::KNOT_IDENTITY_TOL`]:
//! two knots inside that band ARE the same knot, which is the same rule
//! `insert_knot` and `split` snap by.

use crate::{NurbsCurve, Vec4, KNOT_IDENTITY_TOL};

/// What a compatibility pass had to do to one curve.  Reported, never
/// inferred: a caller that promises "bit for bit" needs to see a zero here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CompatWork {
    /// Degrees added by [`elevate_degree`].
    pub elevated_by: usize,
    /// Knots added by [`merge_knots`].
    pub knots_inserted: usize,
    /// Whether [`normalise_domain`] moved the knot domain.
    pub domain_remapped: bool,
}

impl CompatWork {
    /// No elevation, no insertion, no re-mapping: the curve is unchanged.
    pub fn is_identity(&self) -> bool {
        self.elevated_by == 0 && self.knots_inserted == 0 && !self.domain_remapped
    }
}

/// Affinely re-map `curve`'s knot domain onto `[0, 1]`.
///
/// Exact in both directions: the control points never move, the clamped ends
/// land on exactly `0.0` and `1.0` (`(start - start) / span == 0.0`,
/// `(end - start) / span == 1.0`), and a curve already on `[0, 1]` keeps every
/// knot float it had.
pub(crate) fn normalise_domain(curve: &NurbsCurve) -> Result<(NurbsCurve, bool), String> {
    let [start, end] = curve.domain()?;
    let span = end - start;
    if span <= 0.0 || !span.is_finite() {
        return Err(format!(
            "coons: boundary knot domain [{start}, {end}] is not a positive span"
        ));
    }
    let moved = start != 0.0 || end != 1.0;
    if !moved {
        return Ok((curve.clone(), false));
    }
    let knots = curve
        .knots
        .iter()
        .map(|knot| ((knot - start) / span).clamp(0.0, 1.0))
        .collect();
    Ok((
        NurbsCurve::new(curve.degree, knots, curve.control_points.clone())?,
        true,
    ))
}

/// Binomial coefficient as an exact f64 for the small degrees a patch uses.
fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    let mut value = 1.0f64;
    for step in 0..k {
        value = value * (n - step) as f64 / (step + 1) as f64;
    }
    value
}

/// Degree-elevate ONE Bezier segment from degree `p` to `p + t`.
///
/// `Q_i = sum_j C(p,j) C(t,i-j) / C(p+t,i) * P_j` — the closed form, applied
/// to the homogeneous control points, so a rational segment elevates exactly
/// like a polynomial one.
fn elevate_bezier(points: &[Vec4], t: usize) -> Vec<Vec4> {
    let p = points.len() - 1;
    let elevated = p + t;
    (0..=elevated)
        .map(|index| {
            let mut accumulated = Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            };
            for source in index.saturating_sub(t)..=index.min(p) {
                let coefficient =
                    binomial(p, source) * binomial(t, index - source) / binomial(elevated, index);
                accumulated = accumulated.add(points[source].scale(coefficient));
            }
            accumulated
        })
        .collect()
}

/// Distinct interior knot values of `curve` with their multiplicities, in
/// increasing order.  Values inside [`KNOT_IDENTITY_TOL`] of each other are
/// ONE knot (the first occurrence names it); the clamped ends are excluded.
pub(super) fn interior_multiplicities(curve: &NurbsCurve) -> Vec<(f64, usize)> {
    let degree = curve.degree;
    let start = curve.knots[degree];
    let end = curve.knots[curve.knots.len() - 1 - degree];
    let mut groups: Vec<(f64, usize)> = Vec::new();
    for &knot in &curve.knots {
        if knot <= start + KNOT_IDENTITY_TOL || knot >= end - KNOT_IDENTITY_TOL {
            continue;
        }
        match groups.last_mut() {
            Some((value, count)) if (*value - knot).abs() <= KNOT_IDENTITY_TOL => *count += 1,
            _ => groups.push((knot, 1)),
        }
    }
    groups
}

/// Try to remove ONE instance of the knot `parameter` from `curve`.
///
/// Removal is the inverse of insertion, so it is solved as such: the reduced
/// control points are recovered by forward substitution through Boehm's
/// insertion recurrence, and the candidate is then VERIFIED by re-inserting
/// the knot and comparing every control point against the original.  The
/// returned residual is that round-trip distance — the only evidence that the
/// removal changed nothing.  `None` means the knot is not there to remove.
fn try_remove_knot(curve: &NurbsCurve, parameter: f64) -> Option<(NurbsCurve, f64)> {
    let degree = curve.degree;
    let occurrences: Vec<usize> = curve
        .knots
        .iter()
        .enumerate()
        .filter(|(_, knot)| (**knot - parameter).abs() <= KNOT_IDENTITY_TOL)
        .map(|(index, _)| index)
        .collect();
    let multiplicity = occurrences.len();
    let last = *occurrences.last()?;
    if multiplicity == 0 || last < degree + 1 || last + degree >= curve.knots.len() {
        return None;
    }
    let mut reduced_knots = curve.knots.clone();
    reduced_knots.remove(last);
    // In the reduced vector the knot's last occurrence sits at `last - 1`, so
    // that index IS the span index of `parameter`.  Boehm's insertion writes
    // control points `last - degree ..= last - multiplicity`; everything below
    // is copied straight through and everything above shifts by one.
    let first_affected = last - degree;
    let last_affected = last - multiplicity;
    if first_affected == 0 || last_affected + 1 >= curve.control_points.len() {
        return None;
    }
    let mut reduced_points: Vec<Vec4> = curve.control_points[..first_affected].to_vec();
    let mut previous = curve.control_points[first_affected - 1];
    for index in first_affected..=last_affected {
        let denominator = reduced_knots[index + degree] - reduced_knots[index];
        if denominator.abs() <= f64::MIN_POSITIVE {
            return None;
        }
        let alpha = (parameter - reduced_knots[index]) / denominator;
        if alpha.abs() <= 1e-12 {
            return None;
        }
        let point = curve.control_points[index]
            .add(previous.scale(alpha - 1.0))
            .scale(1.0 / alpha);
        reduced_points.push(point);
        previous = point;
    }
    reduced_points.extend_from_slice(&curve.control_points[last_affected + 2..]);
    let reduced = NurbsCurve::new(degree, reduced_knots, reduced_points).ok()?;
    // The verification: re-inserting the knot must rebuild the original net.
    let rebuilt = reduced.insert_knot(parameter, 1).ok()?;
    if rebuilt.control_points.len() != curve.control_points.len() {
        return None;
    }
    let residual = rebuilt
        .control_points
        .iter()
        .zip(&curve.control_points)
        .map(|(a, b)| {
            ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2) + (a.w - b.w).powi(2))
                .sqrt()
        })
        .fold(0.0f64, f64::max);
    Some((reduced, residual))
}

/// Relative round-off band a knot removal's re-insertion round trip must come
/// back inside.  Sized for accumulated f64 error over a Boehm forward
/// substitution and its inverse on a handful of control points — a removable
/// knot round-trips at ~1e-16 relative, a non-removable one at 1e-1.  There is
/// no tolerance to tune here: the gap between the two cases is fifteen orders
/// of magnitude.
const REMOVAL_ROUND_OFF: f64 = 1e-12;

/// Largest control-point coordinate magnitude — the scale the round-trip
/// residual of a knot removal is judged against.
fn net_extent(curve: &NurbsCurve) -> f64 {
    curve
        .control_points
        .iter()
        .flat_map(|point| [point.x, point.y, point.z, point.w])
        .fold(0.0f64, |worst, value| worst.max(value.abs()))
}

/// Raise `curve` to `target` degree exactly.
///
/// Bezier decomposition (knot insertion) -> per-segment binomial elevation ->
/// rejoin -> removal of the surplus junction knots.  The result has each
/// interior knot at its original multiplicity plus `target - degree`, which is
/// the minimal knot vector of the elevated curve; a removal that does not
/// verify is skipped, leaving a redundant knot rather than a wrong curve.
pub(super) fn elevate_degree(curve: &NurbsCurve, target: usize) -> Result<NurbsCurve, String> {
    let degree = curve.degree;
    if target < degree {
        return Err(format!(
            "coons: cannot lower degree {degree} to {target} — elevation only"
        ));
    }
    if target == degree {
        return Ok(curve.clone());
    }
    let raise = target - degree;
    let interior = interior_multiplicities(curve);
    // 1. Bezier decomposition: every interior knot to multiplicity `degree`.
    let mut decomposed = curve.clone();
    for &(knot, multiplicity) in &interior {
        if multiplicity > degree {
            return Err(format!(
                "coons: interior knot {knot} has multiplicity {multiplicity} above degree {degree}"
            ));
        }
        decomposed = decomposed.insert_knot(knot, degree - multiplicity)?;
    }
    let segments = interior.len() + 1;
    if decomposed.control_points.len() != segments * degree + 1 {
        return Err(format!(
            "coons: Bezier decomposition produced {} control points, expected {}",
            decomposed.control_points.len(),
            segments * degree + 1
        ));
    }
    // 2. Elevate each segment; the shared junction point is contributed once.
    let mut points: Vec<Vec4> = Vec::with_capacity(segments * target + 1);
    for segment in 0..segments {
        let start = segment * degree;
        let elevated = elevate_bezier(&decomposed.control_points[start..=start + degree], raise);
        if segment == 0 {
            points.extend_from_slice(&elevated);
        } else {
            points.extend_from_slice(&elevated[1..]);
        }
    }
    // 3. Rejoin at full junction multiplicity.
    let [domain_start, domain_end] = decomposed.domain()?;
    let mut knots: Vec<f64> = vec![domain_start; target + 1];
    for &(knot, _) in &interior {
        knots.extend(std::iter::repeat_n(knot, target));
    }
    knots.extend(std::iter::repeat_n(domain_end, target + 1));
    let mut elevated = NurbsCurve::new(target, knots, points)?;
    // 4. Drop the surplus: an interior knot of original multiplicity `m` needs
    //    multiplicity `m + raise`, so `degree - m` removals.
    //
    //    The acceptance band is ROUND-OFF, not a tolerance: a knot that is
    //    genuinely removable round-trips to a few ulps of the control net, and
    //    anything above that is a knot the curve does not actually carry —
    //    taking it would be a refit, which this module does not do.  A skipped
    //    removal costs one redundant knot and changes no geometry.
    let band = REMOVAL_ROUND_OFF * net_extent(&elevated).max(1.0);
    for &(knot, multiplicity) in &interior {
        for _ in 0..(degree - multiplicity) {
            match try_remove_knot(&elevated, knot) {
                Some((reduced, residual)) if residual <= band => elevated = reduced,
                _ => break,
            }
        }
    }
    Ok(elevated)
}

/// Refine two equal-degree curves on `[0, 1]` onto the union of their knots.
///
/// Returns the two refined curves and the number of knots each gained.  Both
/// come back carrying ONE canonical knot array, so the surface built from them
/// has an exactly shared tensor structure rather than two vectors that merely
/// agree within a band.
pub(super) fn merge_knots(
    first: &NurbsCurve,
    second: &NurbsCurve,
) -> Result<(NurbsCurve, NurbsCurve, usize, usize), String> {
    if first.degree != second.degree {
        return Err(format!(
            "coons: knot merge needs one degree, got {} and {}",
            first.degree, second.degree
        ));
    }
    let degree = first.degree;
    let mut targets: Vec<(f64, usize)> = interior_multiplicities(first);
    for (knot, multiplicity) in interior_multiplicities(second) {
        match targets
            .iter_mut()
            .find(|(value, _)| (*value - knot).abs() <= KNOT_IDENTITY_TOL)
        {
            Some((_, existing)) => *existing = (*existing).max(multiplicity),
            None => targets.push((knot, multiplicity)),
        }
    }
    targets.sort_by(|a, b| a.0.total_cmp(&b.0));
    for &(knot, multiplicity) in &targets {
        if multiplicity > degree {
            return Err(format!(
                "coons: merged interior knot {knot} would reach multiplicity {multiplicity}, \
                 above degree {degree}"
            ));
        }
    }
    let refine = |curve: &NurbsCurve| -> Result<(NurbsCurve, usize), String> {
        let mut refined = curve.clone();
        let before = refined.knots.len();
        for &(knot, multiplicity) in &targets {
            let present = refined
                .knots
                .iter()
                .filter(|value| (**value - knot).abs() <= KNOT_IDENTITY_TOL)
                .count();
            if multiplicity > present {
                refined = refined.insert_knot(knot, multiplicity - present)?;
            }
        }
        let inserted = refined.knots.len() - before;
        Ok((refined, inserted))
    };
    let (first_refined, first_inserted) = refine(first)?;
    let (second_refined, second_inserted) = refine(second)?;
    let mut canonical: Vec<f64> = vec![0.0; degree + 1];
    for &(knot, multiplicity) in &targets {
        canonical.extend(std::iter::repeat_n(knot, multiplicity));
    }
    canonical.extend(std::iter::repeat_n(1.0, degree + 1));
    if canonical.len() != first_refined.knots.len() || canonical.len() != second_refined.knots.len()
    {
        return Err(format!(
            "coons: knot merge did not converge — canonical {} vs {} and {}",
            canonical.len(),
            first_refined.knots.len(),
            second_refined.knots.len()
        ));
    }
    Ok((
        NurbsCurve::new(degree, canonical.clone(), first_refined.control_points)?,
        NurbsCurve::new(degree, canonical, second_refined.control_points)?,
        first_inserted,
        second_inserted,
    ))
}

/// Bring a whole family of curves into one degree and one knot vector.
///
/// The family is made compatible pairwise against a running representative:
/// every curve is elevated to the family's highest degree, then each is
/// refined onto the accumulated knot union.  Returns the compatible curves and
/// the per-curve work record.
pub(crate) fn make_family_compatible(
    curves: &[NurbsCurve],
    minimum_degree: usize,
) -> Result<(Vec<NurbsCurve>, Vec<CompatWork>), String> {
    if curves.is_empty() {
        return Err("coons: cannot make an empty curve family compatible".into());
    }
    let mut work = vec![CompatWork::default(); curves.len()];
    let mut normalised = Vec::with_capacity(curves.len());
    for (index, curve) in curves.iter().enumerate() {
        let (moved, remapped) = normalise_domain(curve)?;
        work[index].domain_remapped = remapped;
        normalised.push(moved);
    }
    let degree = normalised
        .iter()
        .map(|curve| curve.degree)
        .max()
        .unwrap_or(minimum_degree)
        .max(minimum_degree);
    let mut elevated = Vec::with_capacity(normalised.len());
    for (index, curve) in normalised.iter().enumerate() {
        work[index].elevated_by = degree - curve.degree;
        elevated.push(elevate_degree(curve, degree)?);
    }
    // One pass accumulates the union into curve 0, a second pushes the union
    // back out to every other curve.
    let mut representative = elevated[0].clone();
    for curve in elevated.iter().skip(1) {
        let (merged, _, _, _) = merge_knots(&representative, curve)?;
        representative = merged;
    }
    let mut compatible = Vec::with_capacity(elevated.len());
    for (index, curve) in elevated.iter().enumerate() {
        let (_, refined, _, inserted) = merge_knots(&representative, curve)?;
        work[index].knots_inserted = inserted;
        compatible.push(refined);
    }
    Ok((compatible, work))
}

/// Greville abscissae of a knot vector: `xi_i = (u_{i+1} + ... + u_{i+p}) / p`.
///
/// A degree-`p` B-spline basis reproduces any LINEAR function exactly when its
/// coefficients are that function sampled at these parameters, which is how
/// the ruled and bilinear halves of a Coons patch are expressed in the
/// patch's own knot vector without a single fit.
pub(super) fn greville(knots: &[f64], degree: usize) -> Vec<f64> {
    let count = knots.len() - degree - 1;
    (0..count)
        .map(|index| {
            knots[index + 1..=index + degree].iter().sum::<f64>() / degree as f64
        })
        .collect()
}


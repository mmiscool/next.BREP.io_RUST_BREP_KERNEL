//! `BREP_OFFSET_REVOLVED_CURVED_EROSION` (default on since the tier P
//! promotion of 2026-11-21; `=0` restores the refusal): the SHRUNK rounded
//! parallel body of a solid of revolution whose half-section has curved sides,
//! the half-section's erosion by d revolved a full turn, where the pipeline
//! refused and the polygon eroder ([`super::revolved_erosion`]) declined.
//!
//! The body's distance to a point is read in the point's meridian plane (the
//! mirror half of the section is never nearer), so its erosion is the
//! revolution of its half-section's erosion, the run along the axis no source
//! of distance. The half-section is read face by face: a plane square to the
//! axis, a cylinder or cone about it (segments), or a face whose net is exactly
//! `make_revolution` of its own generatrix (that curve, mapped into (ρ, z)).
//! Every rim must be a circle about the axis at the ends of its faces' pieces.
//!
//! The erosion's boundary lies on the pieces moved inward by d and on arcs of
//! radius d about the reflex corners. A curved piece's inward offset is
//! regular exactly where its curvature on the material's convex side stays
//! under 1/d; the roots of κ·d = 1 are solved on every knot span and the
//! sub-branches beyond them dropped (each such point has a nearer point on its
//! own curve). The regular branches are cut wherever two cross (found on
//! polylines, polished by Newton to one shared point) and a cut run is kept
//! when its middle is d from the boundary (no nearer source point). The kept
//! runs must close into one loop (or one run between two axis ends). Segments
//! and arcs stay exact; a curved offset is fitted by cubic interpolation per
//! source knot span within the offset pipeline's fit tolerance
//! (`intersection_fit`), and the fitted profile is checked densely to lie d
//! from the boundary within it before it is revolved and validated.
//!
//! Cause: a tube whose spline side bumps out inside one knot span, shrunk by
//! 0.1 to 0.5: the pipeline's fold carving refuses it, and the polygon eroder
//! (since the straight-meridian reading became exact) declines it.
use super::*;
use crate::{interpolate_curve, make_arc, make_line, revolve_profile_brep, AnalyticSurface, NurbsCurve};

/// The switch, default on (`=0` restores the refusal).
fn curved_erosion_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_CURVED_EROSION", true)
}


/// The shrunk rounded parallel body of `source` by `distance` (< 0), or `None`
/// when the lane is off or declines.
pub(super) fn revolved_curved_erosion(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !curved_erosion_on() || !(distance < 0.0) || !distance.is_finite() {
        return None;
    }
    match curved_erosion_impl(source, -distance) {
        Ok(result) => Some(result),
        Err(reason) => {
            os_debug!("revolved curved erosion declined: {reason}");
            None
        }
    }
}

/// A point of the meridian half-plane: x = ρ, y = height along the axis.
#[derive(Clone, Copy, Debug, PartialEq)]
struct V2 {
    x: f64,
    y: f64,
}

impl V2 {
    fn add(self, o: V2) -> V2 {
        V2 { x: self.x + o.x, y: self.y + o.y }
    }
    fn sub(self, o: V2) -> V2 {
        V2 { x: self.x - o.x, y: self.y - o.y }
    }
    fn scale(self, k: f64) -> V2 {
        V2 { x: self.x * k, y: self.y * k }
    }
    fn dot(self, o: V2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    fn cross(self, o: V2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
    /// The left normal (a quarter turn anticlockwise).
    fn left(self) -> V2 {
        V2 { x: -self.y, y: self.x }
    }
}

/// One piece of the half-section's boundary, run in the section's anticlockwise
/// sense (material on the left).
#[derive(Clone, Debug)]
enum Source {
    Line { a: V2, b: V2 },
    /// A planar curve with x = ρ, y = height (z = 0), over its whole domain,
    /// with its Bézier spans (homogeneous (X, Y, W)) and their parameter ranges.
    Curve { curve: NurbsCurve, t0: f64, t1: f64, spans: Vec<(f64, f64, Vec<[f64; 3]>)> },
}

impl Source {
    fn start(&self) -> V2 {
        match self {
            Source::Line { a, .. } => *a,
            Source::Curve { curve, t0, .. } => eval(curve, *t0).0,
        }
    }
    fn end(&self) -> V2 {
        match self {
            Source::Line { b, .. } => *b,
            Source::Curve { curve, t1, .. } => eval(curve, *t1).0,
        }
    }
    fn reversed(&self) -> Option<Source> {
        Some(match self {
            Source::Line { a, b } => Source::Line { a: *b, b: *a },
            Source::Curve { curve, .. } => curve_source(curve.reversed().ok()?).ok()?,
        })
    }
    /// Unit tangent at the start or the end.
    fn tangent(&self, at_end: bool) -> Option<V2> {
        let t = match self {
            Source::Line { a, b } => b.sub(*a),
            Source::Curve { curve, t0, t1, .. } => eval(curve, if at_end { *t1 } else { *t0 }).1,
        };
        let length = t.len();
        (length > 0.0).then(|| t.scale(1.0 / length))
    }
    /// The nearest distance from `p` to this piece.
    fn distance(&self, p: V2) -> f64 {
        match self {
            Source::Line { a, b } => segment_distance(p, *a, *b),
            Source::Curve { curve, spans, .. } => curve_distance(curve, spans, p),
        }
    }
}

/// Point, first and second derivative of a planar curve at `t`.
fn eval(curve: &NurbsCurve, t: f64) -> (V2, V2, V2) {
    match curve.derivatives(t, 2) {
        Ok(d) if d.len() >= 3 => (V2 { x: d[0].x, y: d[0].y }, V2 { x: d[1].x, y: d[1].y }, V2 { x: d[2].x, y: d[2].y }),
        _ => (V2 { x: f64::NAN, y: f64::NAN }, V2 { x: f64::NAN, y: f64::NAN }, V2 { x: f64::NAN, y: f64::NAN }),
    }
}

fn segment_distance(p: V2, a: V2, b: V2) -> f64 {
    let along = b.sub(a);
    let length2 = along.dot(along);
    if !(length2 > 0.0) {
        return p.sub(a).len();
    }
    let t = (p.sub(a).dot(along) / length2).clamp(0.0, 1.0);
    p.sub(a.add(along.scale(t))).len()
}

/// The distinct knots of `curve` inside [t0, t1], with t0 and t1.
fn span_breaks(curve: &NurbsCurve, t0: f64, t1: f64) -> Vec<f64> {
    let mut breaks = vec![t0];
    for &k in &curve.knots {
        if k > t0 + 1e-14 && k < t1 - 1e-14 && breaks.last().is_some_and(|last| k > *last + 1e-14) {
            breaks.push(k);
        }
    }
    breaks.push(t1);
    breaks
}

/// A curved source with its Bézier spans.
fn curve_source(curve: NurbsCurve) -> Result<Source, String> {
    let [t0, t1] = curve.domain()?;
    let spans = crate::soundness::analytic_body::bezier_spans(&curve).ok_or("a curved piece without Bézier spans")?;
    let breaks = span_breaks(&curve, t0, t1);
    if spans.len() + 1 != breaks.len() {
        return Err("a curved piece's spans do not match its knots".into());
    }
    let spans: Vec<(f64, f64, Vec<[f64; 3]>)> = spans.into_iter().zip(breaks.windows(2)).map(|(span, range)| (range[0], range[1], span)).collect();
    // A span must run at a fair pace: its speed's exact upper bound (the
    // hodograph's coefficients over the least weight squared) at most 64 times
    // its mean speed. Weights that crowd a feature into a sliver of the
    // parameter would hide it between any parameter stations, the fit's
    // included; the mean is read from 256 chords, which can only understate
    // it (and so decline more).
    for (a, b, span) in &spans {
        let coordinate = |i: usize| Bern(span.iter().map(|c| c[i]).collect());
        let (x, y, w) = (coordinate(0), coordinate(1), coordinate(2));
        let v = w.mul(&x.derivative()).combine(&w.derivative().mul(&x), -1.0);
        let u = w.mul(&y.derivative()).combine(&w.derivative().mul(&y), -1.0);
        let least_weight = w.0.iter().fold(f64::INFINITY, |m, c| m.min(*c));
        let fastest = v.0.iter().zip(&u.0).map(|(p, q)| p.hypot(*q)).fold(0.0, f64::max) / (least_weight * least_weight);
        let points: Vec<V2> = (0..=256).map(|k| eval(&curve, a + (b - a) * k as f64 / 256.0).0).collect();
        let mean = points.windows(2).map(|pair| pair[1].sub(pair[0]).len()).sum::<f64>();
        if !(least_weight > 0.0) || !(fastest <= 64.0 * mean) {
            return Err(format!("a span runs unevenly (speed up to {fastest:.3e} against a mean of {mean:.3e})"));
        }
    }
    Ok(Source::Curve { curve, t0, t1, spans })
}

/// The real roots in [0, 1] of a Bernstein polynomial, isolated by
/// subdivision until a piece's coefficients share a strict sign (none there)
/// or it is 1e-14 wide; `None` where they do not isolate.
fn bernstein_roots(f: Bern) -> Option<Vec<f64>> {
    let mut roots = Vec::new();
    let mut leaves = 0usize;
    let mut stack = vec![(f, 0.0f64, 1.0f64)];
    while let Some((piece, a, b)) = stack.pop() {
        if piece.0.iter().all(|c| *c > 0.0) || piece.0.iter().all(|c| *c < 0.0) {
            continue;
        }
        if b - a <= 1e-14 {
            leaves += 1;
            if leaves > 4096 {
                return None;
            }
            roots.push(0.5 * (a + b));
            continue;
        }
        let (left, right) = piece.halves();
        let mid = 0.5 * (a + b);
        stack.push((left, a, mid));
        stack.push((right, mid, b));
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup_by(|p, q| (*p - *q).abs() <= 1e-12);
    Some(roots)
}

/// The nearest distance from `p` to a curved source: on each Bézier span the
/// distance is stationary where G = (X − pₓW)(X′W − XW′) + (Y − p_yW)(Y′W − YW′)
/// vanishes (a polynomial), so the least of the span ends and G's roots is the
/// span's least distance, exactly (no seeds to miss a sharp tip).
fn curve_distance(curve: &NurbsCurve, spans: &[(f64, f64, Vec<[f64; 3]>)], p: V2) -> f64 {
    let mut best = f64::INFINITY;
    for (a, b, span) in spans {
        let coordinate = |i: usize| Bern(span.iter().map(|c| c[i]).collect());
        let (x, y, w) = (coordinate(0), coordinate(1), coordinate(2));
        let (x1, y1, w1) = (x.derivative(), y.derivative(), w.derivative());
        let gx = x.combine(&w, -p.x).mul(&x1.mul(&w).combine(&x.mul(&w1), -1.0));
        let gy = y.combine(&w, -p.y).mul(&y1.mul(&w).combine(&y.mul(&w1), -1.0));
        let mut at = vec![0.0, 1.0];
        match bernstein_roots(gx.combine(&gy, 1.0)) {
            Some(roots) => at.extend(roots),
            // G vanishing along the span (p the centre of a circular span):
            // every point is as near; any one reads it.
            None => at.push(0.5),
        }
        for u in at {
            best = best.min(eval(curve, a + (b - a) * u).0.sub(p).len());
        }
    }
    best
}

/// A branch of the eroded boundary's candidates, parameterised over [s0, s1].
#[derive(Clone, Debug)]
enum Branch {
    Line { a: V2, b: V2 },
    /// About `center`, radius d, from angle `a0` by `sweep` (clockwise when negative).
    Arc { center: V2, a0: f64, sweep: f64 },
    /// The inward offset of source piece `source` (a curve) over [t0, t1];
    /// an end is a fold where the offset beyond it was dropped (κ·d ≥ 1).
    Offset { source: usize, t0: f64, t1: f64, fold_start: bool, fold_end: bool },
}

struct Erosion<'a> {
    sources: &'a [Source],
    depth: f64,
}

impl Erosion<'_> {
    fn range(&self, branch: &Branch) -> (f64, f64) {
        match branch {
            Branch::Line { .. } | Branch::Arc { .. } => (0.0, 1.0),
            Branch::Offset { t0, t1, .. } => (*t0, *t1),
        }
    }
    /// Point and derivative of `branch` at `s`.
    fn at(&self, branch: &Branch, s: f64) -> (V2, V2) {
        match branch {
            Branch::Line { a, b } => (a.add(b.sub(*a).scale(s)), b.sub(*a)),
            Branch::Arc { center, a0, sweep } => {
                let angle = a0 + sweep * s;
                let radial = V2 { x: angle.cos(), y: angle.sin() };
                (center.add(radial.scale(self.depth)), radial.left().scale(self.depth * sweep))
            }
            Branch::Offset { source, .. } => {
                let Source::Curve { curve, .. } = &self.sources[*source] else { unreachable!("an offset branch of a curve") };
                let (c, d1, d2) = eval(curve, s);
                let speed = d1.len();
                let normal = d1.left().scale(1.0 / speed);
                // d/dt of the unit left normal: J(C″/|C′| − C′(C′·C″)/|C′|³).
                let turn = d2.scale(1.0 / speed).sub(d1.scale(d1.dot(d2) / speed.powi(3))).left();
                (c.add(normal.scale(self.depth)), d1.add(turn.scale(self.depth)))
            }
        }
    }
    /// Stations along `branch` for finding crossings: 2 on a line, 64 on an
    /// arc, 96 a knot span on an offset.
    fn stations(&self, branch: &Branch) -> Vec<f64> {
        match branch {
            Branch::Line { .. } => vec![0.0, 1.0],
            Branch::Arc { .. } => (0..=64).map(|k| k as f64 / 64.0).collect(),
            Branch::Offset { source, t0, t1, fold_start, fold_end } => {
                let Source::Curve { curve, .. } = &self.sources[*source] else { unreachable!() };
                let mut at = Vec::new();
                for span in span_breaks(curve, *t0, *t1).windows(2) {
                    for k in 0..96 {
                        at.push(span[0] + (span[1] - span[0]) * k as f64 / 96.0);
                    }
                }
                at.push(*t1);
                // Toward a fold the offset turns sharply: stations crowd
                // geometrically to it, so a crossing next to it is seen.
                let length = t1 - t0;
                if *fold_start {
                    at.extend((1..=48).map(|k| t0 + length * 0.5f64.powi(k + 6)));
                }
                if *fold_end {
                    at.extend((1..=48).map(|k| t1 - length * 0.5f64.powi(k + 6)));
                }
                at.sort_by(f64::total_cmp);
                at.dedup();
                at
            }
        }
    }
    /// The nearest distance from `p` to the section's boundary.
    fn reach(&self, p: V2) -> f64 {
        self.sources.iter().map(|source| source.distance(p)).fold(f64::INFINITY, f64::min)
    }
}

/// Where branches `i` (at `si`) and `j` (at `sj`) cross, polished by Newton
/// from a crossing of their polylines.
fn polish(erosion: &Erosion, bi: &Branch, bj: &Branch, mut si: f64, mut sj: f64) -> Option<(f64, f64, V2)> {
    let (ri, rj) = (erosion.range(bi), erosion.range(bj));
    for _ in 0..40 {
        let (pi, di) = erosion.at(bi, si);
        let (pj, dj) = erosion.at(bj, sj);
        let gap = pi.sub(pj);
        // Solve di·Δsi − dj·Δsj = −gap.
        let det = -di.cross(dj);
        if !(det.abs() > 0.0) {
            return None;
        }
        let dsi = (-gap).cross(dj.scale(-1.0)) / det;
        let dsj = di.cross(-gap) / det;
        si = (si + dsi).clamp(ri.0, ri.1);
        sj = (sj + dsj).clamp(rj.0, rj.1);
        if gap.len() <= 1e-15 * (1.0 + pi.len()) && dsi.abs() + dsj.abs() <= 1e-15 {
            break;
        }
    }
    let (pi, _) = erosion.at(bi, si);
    let (pj, _) = erosion.at(bj, sj);
    (pi.sub(pj).len() <= 1e-10 * (1.0 + pi.len())).then(|| (si, sj, pi.add(pj).scale(0.5)))
}

impl std::ops::Neg for V2 {
    type Output = V2;
    fn neg(self) -> V2 {
        V2 { x: -self.x, y: -self.y }
    }
}

/// Segment crossing of p0→p1 and q0→q1 as fractions along each.
fn segments_cross(p0: V2, p1: V2, q0: V2, q1: V2) -> Option<(f64, f64)> {
    let (u, w) = (p1.sub(p0), q1.sub(q0));
    let den = u.cross(w);
    if !(den.abs() > 0.0) {
        return None;
    }
    let a = q0.sub(p0).cross(w) / den;
    let b = q0.sub(p0).cross(u) / den;
    ((-1e-9..=1.0 + 1e-9).contains(&a) && (-1e-9..=1.0 + 1e-9).contains(&b)).then_some((a, b))
}

/// A polynomial on [0, 1] in Bernstein form.
#[derive(Clone, Debug)]
struct Bern(Vec<f64>);

fn choose(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64)
}

impl Bern {
    fn degree(&self) -> usize {
        self.0.len() - 1
    }
    fn derivative(&self) -> Bern {
        let n = self.degree();
        if n == 0 {
            return Bern(vec![0.0]);
        }
        Bern((0..n).map(|i| n as f64 * (self.0[i + 1] - self.0[i])).collect())
    }
    fn elevated(&self, to: usize) -> Bern {
        let mut b = self.clone();
        while b.degree() < to {
            let n = b.degree() + 1;
            let mut next = vec![0.0; n + 1];
            next[0] = b.0[0];
            next[n] = b.0[n - 1];
            for i in 1..n {
                let a = i as f64 / n as f64;
                next[i] = a * b.0[i - 1] + (1.0 - a) * b.0[i];
            }
            b = Bern(next);
        }
        b
    }
    fn mul(&self, other: &Bern) -> Bern {
        let (m, n) = (self.degree(), other.degree());
        let mut out = vec![0.0; m + n + 1];
        for i in 0..=m {
            for j in 0..=n {
                out[i + j] += choose(m, i) * choose(n, j) / choose(m + n, i + j) * self.0[i] * other.0[j];
            }
        }
        Bern(out)
    }
    fn combine(&self, other: &Bern, k: f64) -> Bern {
        let degree = self.degree().max(other.degree());
        let (a, b) = (self.elevated(degree), other.elevated(degree));
        Bern(a.0.iter().zip(&b.0).map(|(x, y)| x + k * y).collect())
    }
    /// The halves on [0, ½] and [½, 1] (de Casteljau).
    fn halves(&self) -> (Bern, Bern) {
        let mut row = self.0.clone();
        let n = row.len();
        let (mut left, mut right) = (vec![0.0; n], vec![0.0; n]);
        for level in 0..n {
            left[level] = row[0];
            right[n - 1 - level] = row[n - 1 - level];
            for i in 0..n - 1 - level {
                row[i] = 0.5 * (row[i] + row[i + 1]);
            }
        }
        (Bern(left), Bern(right))
    }
}

/// Where κ·d = ±1 on one Bézier span (homogeneous controls `(X, Y, W)`), as
/// fractions of the span. With p = (X/W, Y/W), κ = W³·D / (V² + U²)^{3/2}, D the
/// determinant of (A, A′, A″), V = W·X′ − W′·X and U = W·Y′ − W′·Y, so κ·d = ±1
/// exactly where F = d²·W⁶·D² − (V² + U²)³ vanishes, a polynomial; its roots are
/// isolated by subdividing its Bernstein form until a piece's coefficients all
/// share a sign (none there) or it is 1e-14 wide.
fn fold_roots(span: &[[f64; 3]], depth: f64) -> Result<Vec<f64>, String> {
    let coordinate = |i: usize| Bern(span.iter().map(|c| c[i]).collect());
    let (x, y, w) = (coordinate(0), coordinate(1), coordinate(2));
    let (x1, y1, w1) = (x.derivative(), y.derivative(), w.derivative());
    let (x2, y2, w2) = (x1.derivative(), y1.derivative(), w1.derivative());
    let minor = |a: &Bern, b: &Bern, c: &Bern, e: &Bern| a.mul(b).combine(&c.mul(e), -1.0);
    let det = x
        .mul(&minor(&y1, &w2, &w1, &y2))
        .combine(&y.mul(&minor(&x1, &w2, &w1, &x2)), -1.0)
        .combine(&w.mul(&minor(&x1, &y2, &y1, &x2)), 1.0);
    let v = minor(&w, &x1, &w1, &x);
    let u = minor(&w, &y1, &w1, &y);
    let speed2 = v.mul(&v).combine(&u.mul(&u), 1.0);
    let w3 = w.mul(&w).mul(&w);
    let lhs = w3.mul(&det);
    let f = lhs.mul(&lhs).combine(&speed2.mul(&speed2).mul(&speed2), -1.0 / (depth * depth));
    let size = f.0.iter().fold(0.0f64, |m, c| m.max(c.abs()));
    if !(size > 0.0) || !size.is_finite() {
        return Err("the fold polynomial vanishes (a curvature of exactly 1/d)".into());
    }
    // A piece whose coefficients share a strict sign holds no root (its
    // coefficients' rounding scales with its own terms, not the span's
    // largest, which may be twenty orders larger where the curve runs fast).
    bernstein_roots(f).ok_or_else(|| "the fold polynomial does not isolate its roots (κ·d stays near 1)".to_string())
}

/// A frame for the meridian half-plane.
struct Frame {
    origin: Vec3,
    axis: Vec3,
    x: Vec3,
}

impl Frame {
    fn to_2d(&self, p: Vec3) -> V2 {
        let w = p.sub(self.origin);
        let z = w.dot(self.axis);
        V2 { x: w.sub(self.axis.scale(z)).length(), y: z }
    }
    fn place(&self, p: V2) -> Vec3 {
        self.origin.add(self.x.scale(p.x)).add(self.axis.scale(p.y))
    }
    fn on_axis(&self, p: Vec3, tolerance: f64) -> bool {
        let w = p.sub(self.origin);
        w.sub(self.axis.scale(w.dot(self.axis))).length() <= tolerance
    }
    fn parallel(&self, direction: Vec3) -> bool {
        direction.normalized().is_ok_and(|d| d.cross(self.axis).length() <= 1e-9)
    }
}

/// The half-section of `source`: its pieces, chained and run anticlockwise,
/// and whether the chain runs from the axis back to it.
fn read_section(source: &BrepSolid, tolerance: f64) -> Result<(Frame, Vec<Source>, bool), String> {
    let [shell] = source.shells.as_slice() else { return Err("not one shell".into()) };
    // The axis: the first cylinder, cone or exact revolution.
    let mut frame: Option<Frame> = None;
    for face in &shell.faces {
        let found = match face.surface.analytic() {
            Some(AnalyticSurface::RuledRevolution { frame, .. }) => Some((frame.origin, frame.axis)),
            Some(AnalyticSurface::Plane { .. }) => None,
            _ => crate::revolution_structure(&face.surface).map(|s| (s.frame.origin, s.frame.axis)),
        };
        if let Some((origin, axis)) = found {
            let axis = axis.normalized().map_err(|e| e.to_string())?;
            let x = axis.perpendicular().map_err(|e| e.to_string())?;
            frame = Some(Frame { origin, axis, x });
            break;
        }
    }
    let frame = frame.ok_or("no face turns about an axis")?;

    // Each face's rims: the distinct (ρ, z) of its circular edges.
    let mut uses: std::collections::HashMap<u64, Vec<usize>> = std::collections::HashMap::new();
    for (index, face) in shell.faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            uses.entry(coedge.edge_id).or_default().push(index);
        }
    }
    let mut rims: Vec<Vec<V2>> = vec![Vec::new(); shell.faces.len()];
    for edge in &source.edges {
        let Some(faces) = uses.get(&edge.id) else { continue };
        if edge.degenerate || (faces.len() == 2 && faces[0] == faces[1]) {
            continue; // a pole or a seam
        }
        let [a, b] = faces.as_slice() else { return Err("an edge not between two faces".into()) };
        let Ok(Some(circle)) = super::rim_welds::fit_rim_circle(edge, tolerance) else {
            return Err("a rim that is not a circle".into());
        };
        if !frame.parallel(circle.axis) || !frame.on_axis(circle.center, tolerance) {
            return Err("a rim about another axis".into());
        }
        let at = frame.to_2d(edge.curve.evaluate(edge.t0).map_err(|e| e.to_string())?);
        for face in [*a, *b] {
            if !rims[face].iter().any(|p| p.sub(at).len() <= 1e-7 * (1.0 + at.len())) {
                rims[face].push(at);
            }
        }
    }

    let close = |p: V2, q: V2| p.sub(q).len() <= 1e-7 * (1.0 + p.len().max(q.len()));
    let mut pieces: Vec<Source> = Vec::new();
    for (index, face) in shell.faces.iter().enumerate() {
        let ends = &rims[index];
        let piece = match face.surface.analytic() {
            Some(AnalyticSurface::Plane { u_dir, v_dir, origin, .. }) => {
                if !frame.parallel(u_dir.cross(*v_dir)) {
                    return Err("a plane not square to the axis".into());
                }
                match ends.as_slice() {
                    [a, b] => Source::Line { a: *a, b: *b },
                    [a] => Source::Line { a: *a, b: V2 { x: 0.0, y: frame.to_2d(*origin).y } },
                    _ => return Err("a plane without one or two rims".into()),
                }
            }
            Some(AnalyticSurface::RuledRevolution { frame: own, rho0, rho1, height }) => {
                if !frame.parallel(own.axis) || !frame.on_axis(own.origin, tolerance) {
                    return Err("a cylinder or cone about another axis".into());
                }
                let along = |h: f64| frame.to_2d(own.origin.add(own.axis.scale(h))).y;
                Source::Line { a: V2 { x: *rho0, y: along(0.0) }, b: V2 { x: *rho1, y: along(*height) } }
            }
            _ => {
                let generatrix = super::revolved_parallel::exact_generatrix(face, frame.origin, frame.axis, tolerance)
                    .ok_or("a face that is no exact revolution about the axis")?;
                let mut controls = Vec::with_capacity(generatrix.control_points.len());
                for control in &generatrix.control_points {
                    let p = frame.to_2d(control.point()?);
                    controls.push(crate::Vec4::from_point(Vec3::new(p.x, p.y, 0.0), control.w));
                }
                curve_source(NurbsCurve::new(generatrix.degree, generatrix.knots.clone(), controls)?)?
            }
        };
        // The piece runs between its face's rims (or a rim and the axis).
        let (a, b) = (piece.start(), piece.end());
        for end in [a, b] {
            if end.x.abs() > 1e-7 * (1.0 + end.len()) && !ends.iter().any(|rim| close(*rim, end)) {
                return Err("a piece's end is no rim of its face".into());
            }
        }
        if ends.iter().any(|rim| !close(*rim, a) && !close(*rim, b)) {
            return Err("a rim of a face away from its piece's ends".into());
        }
        if a.sub(b).len() <= 1e-7 * (1.0 + a.len()) {
            return Err("a piece of no length".into());
        }
        pieces.push(piece);
    }

    // Chain the pieces end to end.
    let on_axis = |p: V2| p.x.abs() <= 1e-7 * (1.0 + p.len());
    let first = pieces.iter().position(|p| on_axis(p.start()) || on_axis(p.end()));
    let mut used = vec![false; pieces.len()];
    let start = first.unwrap_or(0);
    used[start] = true;
    let mut chain = vec![if on_axis(pieces[start].end()) && !on_axis(pieces[start].start()) {
        pieces[start].reversed().ok_or("reverse")?
    } else {
        pieces[start].clone()
    }];
    loop {
        let tail = chain[chain.len() - 1].end();
        if first.is_some() && on_axis(tail) {
            break;
        }
        let next: Vec<(usize, bool)> = (0..pieces.len())
            .filter(|&k| !used[k])
            .filter_map(|k| {
                if close(pieces[k].start(), tail) {
                    Some((k, false))
                } else if close(pieces[k].end(), tail) {
                    Some((k, true))
                } else {
                    None
                }
            })
            .collect();
        match next.as_slice() {
            [] => break,
            [(k, flip)] => {
                used[*k] = true;
                chain.push(if *flip { pieces[*k].reversed().ok_or("reverse")? } else { pieces[*k].clone() });
            }
            _ => return Err("an ambiguous join in the section".into()),
        }
    }
    if used.iter().any(|u| !u) {
        return Err("the section's pieces make more than one run".into());
    }
    let touches_axis = first.is_some();
    if touches_axis {
        if !on_axis(chain[0].start()) || !on_axis(chain[chain.len() - 1].end()) {
            return Err("the run does not end on the axis".into());
        }
    } else if !close(chain[chain.len() - 1].end(), chain[0].start()) {
        return Err("the section does not close".into());
    }
    // Orientation from a fine polygon (the axis run sweeps no area about ρ = 0).
    let mut area2 = 0.0;
    for piece in &chain {
        let points: Vec<V2> = match piece {
            Source::Line { a, b } => vec![*a, *b],
            Source::Curve { curve, t0, t1, .. } => (0..=256).map(|k| eval(curve, t0 + (t1 - t0) * k as f64 / 256.0).0).collect(),
        };
        for pair in points.windows(2) {
            area2 += pair[0].cross(pair[1]);
        }
    }
    if area2 < 0.0 {
        chain = chain.iter().rev().map(|p| p.reversed().ok_or("reverse")).collect::<Result<_, _>>()?;
    }
    Ok((frame, chain, touches_axis))
}

fn curved_erosion_impl(source: &BrepSolid, depth: f64) -> Result<OffsetShellResultRecord, String> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    let fit = crate::KernelTolerances::for_solid(source, 1e-7).intersection_fit;
    let (frame, sources, touches_axis) = read_section(source, 1e-7 * scale)?;
    if !sources.iter().any(|s| matches!(s, Source::Curve { .. })) {
        return Err("no curved piece (the polygon eroder's)".into());
    }
    let n = sources.len();
    let erosion = Erosion { sources: &sources, depth };

    // Candidates: each piece moved inward (a curve only where regular), and an
    // arc about each reflex corner.
    let mut branches: Vec<Branch> = Vec::new();
    for (k, piece) in sources.iter().enumerate() {
        match piece {
            Source::Line { a, b } => {
                let n_in = b.sub(*a).left().scale(1.0 / b.sub(*a).len());
                branches.push(Branch::Line { a: a.add(n_in.scale(depth)), b: b.add(n_in.scale(depth)) });
            }
            Source::Curve { curve, t0, t1, spans } => {
                // g = κ·d − 1, κ signed (anticlockwise turns positive, convex
                // for the material on the left), its roots solved exactly on
                // every Bézier span (see `fold_roots`), never sampled.
                let g = |t: f64| {
                    let (_, d1, d2) = eval(curve, t);
                    d1.cross(d2) / d1.len().powi(3) * depth - 1.0
                };
                let mut cuts = vec![*t0];
                for (a, b, span) in spans {
                    let (a, b) = (*a, *b);
                    for root in fold_roots(span, depth)? {
                        cuts.push(a + (b - a) * root);
                    }
                    if b < *t1 {
                        cuts.push(b); // a knot, where κ may jump across 1/d
                    }
                }
                cuts.push(*t1);
                cuts.sort_by(f64::total_cmp);
                cuts.dedup_by(|x, y| (*x - *y).abs() <= 1e-14 * (1.0 + x.abs()));
                let regular: Vec<bool> = cuts.windows(2).map(|pair| g(0.5 * (pair[0] + pair[1])) < 0.0).collect();
                if cuts.windows(2).any(|pair| !g(0.5 * (pair[0] + pair[1])).is_finite()) {
                    return Err("a curved piece stops (no tangent)".into());
                }
                for (i, pair) in cuts.windows(2).enumerate() {
                    if regular[i] {
                        // An end next to a dropped interval is a fold: a kept run may never end there.
                        let fold_start = i > 0 && !regular[i - 1];
                        let fold_end = i + 1 < regular.len() && !regular[i + 1];
                        branches.push(Branch::Offset { source: k, t0: pair[0], t1: pair[1], fold_start, fold_end });
                    }
                }
            }
        }
        // The corner after this piece.
        if k + 1 < n || !touches_axis {
            let next = &sources[(k + 1) % n];
            let (t1, t2) = (piece.tangent(true).ok_or("a piece without a tangent")?, next.tangent(false).ok_or("a piece without a tangent")?);
            let turn = t1.cross(t2);
            if t1.dot(t2) < 0.0 && turn.abs() <= 1e-9 {
                return Err("a cusp in the section".into());
            }
            if turn < -1e-12 {
                // Reflex for the material: the arc about the corner, clockwise
                // from one inward normal to the next.
                let (n1, n2) = (t1.left(), t2.left());
                let a0 = n1.y.atan2(n1.x);
                let mut sweep = n2.y.atan2(n2.x) - a0;
                while sweep >= 0.0 {
                    sweep -= std::f64::consts::TAU;
                }
                branches.push(Branch::Arc { center: piece.end(), a0, sweep });
            }
        }
    }
    if touches_axis {
        for (piece, at_end) in [(&sources[0], false), (&sources[n - 1], true)] {
            let t = piece.tangent(at_end).ok_or("a piece without a tangent")?;
            if t.x.abs() < 1.0 - 1e-9 {
                return Err("the section meets the axis at a slant".into());
            }
        }
    }

    // Crossings, on polylines, polished.
    let polylines: Vec<Vec<(f64, V2)>> = branches.iter().map(|b| erosion.stations(b).into_iter().map(|s| (s, erosion.at(b, s).0)).collect()).collect();
    let mut cuts: Vec<Vec<(f64, V2)>> = vec![Vec::new(); branches.len()];
    for i in 0..branches.len() {
        for j in i..branches.len() {
            let (pi, pj) = (&polylines[i], &polylines[j]);
            for (a, sa) in pi.windows(2).enumerate() {
                for (b, sb) in pj.windows(2).enumerate() {
                    if i == j && b <= a + 1 {
                        continue;
                    }
                    let Some((fa, fb)) = segments_cross(sa[0].1, sa[1].1, sb[0].1, sb[1].1) else { continue };
                    let si = sa[0].0 + (sa[1].0 - sa[0].0) * fa;
                    let sj = sb[0].0 + (sb[1].0 - sb[0].0) * fb;
                    let Some((si, sj, point)) = polish(&erosion, &branches[i], &branches[j], si, sj) else { continue };
                    // Joins end to end (a branch's end on the next's start) are no cut.
                    let (ri, rj) = (erosion.range(&branches[i]), erosion.range(&branches[j]));
                    let interior_i = si > ri.0 + 1e-12 * (ri.1 - ri.0) && si < ri.1 - 1e-12 * (ri.1 - ri.0);
                    let interior_j = sj > rj.0 + 1e-12 * (rj.1 - rj.0) && sj < rj.1 - 1e-12 * (rj.1 - rj.0);
                    if !interior_i && !interior_j {
                        continue;
                    }
                    if i == j && (si - sj).abs() <= 1e-12 * (ri.1 - ri.0) {
                        continue;
                    }
                    cuts[i].push((si, point));
                    cuts[j].push((sj, point));
                }
            }
        }
    }

    // The two branches flanking a dropped fold cross across it (the
    // swallowtail's tails), often closer to the fold than any polyline sees:
    // their crossing is sought from a grid of geometric steps off the fold's
    // two ends, then polished.
    for i in 0..branches.len() {
        let Branch::Offset { source, t1: fold_a, fold_end: true, .. } = branches[i] else { continue };
        let Some(j) = branches.iter().position(|b| matches!(b, Branch::Offset { source: s2, t0, fold_start: true, .. } if *s2 == source && *t0 > fold_a)) else { continue };
        let (ri, rj) = (erosion.range(&branches[i]), erosion.range(&branches[j]));
        let fold_b = rj.0;
        let steps = |length: f64| -> Vec<f64> { (0..48).map(|k| length * 0.5f64.powi(k)).collect() };
        let mut best: Option<(f64, f64, f64)> = None;
        for &di in &steps(ri.1 - ri.0) {
            for &dj in &steps(rj.1 - rj.0) {
                let (si, sj) = (fold_a - di, fold_b + dj);
                let gap = erosion.at(&branches[i], si).0.sub(erosion.at(&branches[j], sj).0).len();
                if best.is_none_or(|b| gap < b.2) {
                    best = Some((si, sj, gap));
                }
            }
        }
        if let Some((si, sj, _)) = best {
            if let Some((si, sj, point)) = polish(&erosion, &branches[i], &branches[j], si, sj) {
                let interior = si > ri.0 && si < ri.1 && sj > rj.0 && sj < rj.1;
                let known = cuts[i].iter().any(|c| (c.0 - si).abs() <= 1e-12 * (ri.1 - ri.0));
                if interior && !known {
                    cuts[i].push((si, point));
                    cuts[j].push((sj, point));
                }
            }
        }
    }
    // Runs between cuts, kept where their middle is d from the boundary.
    struct Run {
        branch: usize,
        s0: f64,
        s1: f64,
        start: V2,
        end: V2,
    }
    let near = 1e-7 * scale;
    let mut runs: Vec<Run> = Vec::new();
    for (index, branch) in branches.iter().enumerate() {
        let (r0, r1) = erosion.range(branch);
        let mut marks: Vec<(f64, V2)> = vec![(r0, erosion.at(branch, r0).0)];
        let mut own = cuts[index].clone();
        own.sort_by(|a, b| a.0.total_cmp(&b.0));
        // One crossing found twice (on the polylines and off a fold) cuts once.
        let end_point = erosion.at(branch, r1).0;
        for cut in own {
            let last = marks[marks.len() - 1];
            if cut.0 - last.0 > 1e-12 * (r1 - r0) && r1 - cut.0 > 1e-12 * (r1 - r0) && cut.1.sub(last.1).len() > near && cut.1.sub(end_point).len() > near {
                marks.push(cut);
            }
        }
        marks.push((r1, erosion.at(branch, r1).0));
        for pair in marks.windows(2) {
            let middle = erosion.at(branch, 0.5 * (pair[0].0 + pair[1].0)).0;
            let short = depth - erosion.reach(middle);
            if short > 1e-10 * scale && short < near {
                return Err(format!("a run ties at ({:.6}, {:.6}), {short:.3e} under the distance", middle.x, middle.y));
            }
            if short <= 1e-10 * scale {
                if middle.x < -tolerance {
                    return Err("the erosion crosses the axis".into());
                }
                // The boundary never ends at a fold (the offset's cusp lies
                // inside its swallowtail): a kept run that does missed the
                // crossing that trims it.
                if let Branch::Offset { fold_start, fold_end, .. } = branch {
                    if (*fold_start && pair[0].0 == r0) || (*fold_end && pair[1].0 == r1) {
                        return Err(format!("a kept run ends at a fold near ({:.6}, {:.6}): a crossing was missed", middle.x, middle.y));
                    }
                }
                runs.push(Run { branch: index, s0: pair[0].0, s1: pair[1].0, start: pair[0].1, end: pair[1].1 });
            }
        }
    }
    if runs.is_empty() {
        return Err("nothing is left".into());
    }

    // One loop, or one run between two axis ends.
    let meet = |p: V2, q: V2| p.sub(q).len() <= near;
    let on_axis = |p: V2| p.x.abs() <= near;
    let heads: Vec<usize> = (0..runs.len()).filter(|&k| !runs.iter().any(|other| meet(other.end, runs[k].start))).collect();
    let first = match heads.as_slice() {
        [] => 0,
        [one] if on_axis(runs[*one].start) => *one,
        _ => return Err(format!("{} runs start where none ends", heads.len())),
    };
    let mut used = vec![false; runs.len()];
    used[first] = true;
    let mut order = vec![first];
    loop {
        let tail = runs[order[order.len() - 1]].end;
        let next: Vec<usize> = (0..runs.len()).filter(|&k| !used[k] && meet(runs[k].start, tail)).collect();
        match next.as_slice() {
            [] => break,
            [one] => {
                used[*one] = true;
                order.push(*one);
            }
            _ => return Err("an ambiguous join in the erosion".into()),
        }
    }
    if used.iter().any(|u| !u) {
        return Err("the erosion is more than one loop".into());
    }
    let open = !heads.is_empty();
    let (head, tail) = (runs[order[0]].start, runs[order[order.len() - 1]].end);
    if open && !on_axis(tail) {
        return Err("the eroded run does not end on the axis".into());
    }
    if !open && !meet(head, tail) {
        return Err("the eroded loop does not close".into());
    }

    // The profile: exact segments and arcs, fitted offsets.
    let mut joints: Vec<V2> = order.iter().map(|&k| runs[k].start).collect();
    if open {
        joints.push(V2 { x: 0.0, y: tail.y });
        joints[0].x = 0.0;
    } else {
        joints.push(joints[0]);
    }
    let place = |p: V2| frame.place(p);
    let mut curves: Vec<NurbsCurve> = Vec::new();
    for (slot, &k) in order.iter().enumerate() {
        let run = &runs[k];
        let (from, to) = (joints[slot], if slot + 1 < order.len() || !open { joints[slot + 1] } else { V2 { x: 0.0, y: run.end.y } });
        match &branches[run.branch] {
            Branch::Line { .. } => {
                if from.sub(to).len() > tolerance {
                    curves.push(make_line(place(from), place(to))?);
                }
            }
            Branch::Arc { center, a0, sweep } => {
                let (start, stop) = (a0 + sweep * run.s0, a0 + sweep * run.s1);
                let arc = if stop > start {
                    make_arc(place(*center), frame.x, frame.axis, depth, start, stop)?
                } else {
                    make_arc(place(*center), frame.x, frame.axis, depth, stop, start)?.reversed()?
                };
                curves.push(arc);
            }
            Branch::Offset { source, .. } => {
                let Source::Curve { curve, .. } = &sources[*source] else { unreachable!() };
                let breaks = span_breaks(curve, run.s0, run.s1);
                let pieces = breaks.len() - 1;
                for (index, span) in breaks.windows(2).enumerate() {
                    let (a, b) = (span[0], span[1]);
                    let first_point = if index == 0 { from } else { erosion.at(&branches[run.branch], a).0 };
                    let last_point = if index + 1 == pieces { to } else { erosion.at(&branches[run.branch], b).0 };
                    let mut count = 8;
                    let fitted = loop {
                        let params: Vec<f64> = (0..=count).map(|i| a + (b - a) * i as f64 / count as f64).collect();
                        let points: Vec<Vec3> = params
                            .iter()
                            .enumerate()
                            .map(|(i, &t)| {
                                let p = if i == 0 { first_point } else if i == count { last_point } else { erosion.at(&branches[run.branch], t).0 };
                                place(p)
                            })
                            .collect();
                        let candidate = interpolate_curve(&points, 3, &params)?;
                        let mut worst: f64 = 0.0;
                        for i in 0..count {
                            for q in [0.25, 0.5, 0.75] {
                                let t = params[i] + (params[i + 1] - params[i]) * q;
                                let got = frame.to_2d(candidate.evaluate(t)?);
                                worst = worst.max(got.sub(erosion.at(&branches[run.branch], t).0).len());
                            }
                        }
                        if worst <= 0.25 * fit {
                            break candidate;
                        }
                        if count >= 4096 {
                            return Err(format!("an offset span does not fit within {fit:.1e} (worst {worst:.3e})"));
                        }
                        count *= 2;
                    };
                    curves.push(fitted);
                }
            }
        }
    }
    if open {
        let (bottom, top) = (joints[joints.len() - 1], joints[0]);
        if bottom.sub(top).len() > tolerance {
            curves.push(make_line(place(bottom), place(top))?);
        }
    }
    // Every fitted point lies d from the boundary within the fit tolerance.
    for curve in &curves {
        let [t0, t1] = curve.domain()?;
        for k in 0..=64 {
            let p = frame.to_2d(curve.evaluate(t0 + (t1 - t0) * k as f64 / 64.0)?);
            if p.x <= near {
                continue; // the closing run along the axis
            }
            let reach = erosion.reach(p);
            if (reach - depth).abs() > 2.0 * fit {
                return Err(format!("a profile point lies {reach:.9} from the boundary, not {depth:.9}"));
            }
        }
    }
    let solid = revolve_profile_brep(&curves, frame.origin, frame.axis, std::f64::consts::TAU)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the eroded revolution does not validate: {:?}", issues.first()));
    }
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
        .collect();
    Ok(OffsetShellResultRecord { solid, face_images })
}


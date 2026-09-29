//! Tangency at a point INTERIOR TO A SPAN: the foot parameter, eliminated by
//! projection (`∿`, [`SPLINE_FOOT_TANGENT_TYPE`]).
//!
//! `⌒` states tangency where a spline's tangent has a NAME — at an anchor,
//! where it is the anchor→handle direction. Tangency to a line or a circle
//! somewhere along a span has no such name: it needs the touch parameter `t`.
//!
//! The parameter is NOT a solver unknown. The layout the whole solver is built
//! on is two columns per movable point (`build_free_layout`, per-point
//! mobility, `POLISH_MAX_FREE_COORDS`, the point-signature convergence test,
//! the JSON round trip); a scalar unknown would touch every one of them. It is
//! ELIMINATED instead: `t*` is defined implicitly by the stationarity condition
//! — the curve's distance to the target is stationary at the touch point — and
//! the residual is evaluated there. By the implicit function theorem `t*` is a
//! C¹ function of the coordinates wherever that root is SIMPLE, so the existing
//! central-difference Jacobian differentiates through it unchanged, and the
//! constraint is one row like every other tangency.
//!
//! | lane | stationarity `g(t) = 0` | residual (length¹) |
//! |---|---|---|
//! | line (unit direction `d̂`, point `p`) | `cross(d̂, B′(t))` — QUADRATIC, closed-form roots | `cross(d̂, B(t*) − p)` |
//! | circle (centre `c`, radius `ρ`) | `dot(B(t) − c, B′(t))` — degree 5, Newton from the seed | `|B(t*) − c| − ρ` |
//!
//! A root of `g` says the touch point is where the distance STOPS changing; the
//! residual says that stationary distance is the one tangency asks for. Together
//! they are tangency, and separately neither is: that is why a configuration
//! with no root in `[0, 1]` is REFUSED by name rather than answered at a clamped
//! span end, which would drive the span's endpoint onto the line and report a
//! tangency that is not there.

use super::*;

/// One cubic Bézier span's control polygon, in chain order.
pub(super) type SpanControls = [[f64; 2]; 4];

/// A root of the stationarity function whose derivative is within this (times
/// the magnitude `g′` is naturally measured against) is a DOUBLE root: the two
/// stationary points have merged. For the line lane that is exactly the foot
/// sitting at an INFLECTION of the span — `g′ = cross(d̂, B″)` vanishes when `B″`
/// turns parallel to the tangent — and it is where the implicit function theorem
/// gives out: `dt*/dcoords` is unbounded, so a finite difference through it is a
/// junk Jacobian column. Refused by name instead.
const FOOT_SIMPLE_ROOT_EPS: f64 = 1e-6;

/// Below this the span has no second derivative to measure a root against — a
/// straight span, whose stationarity function is a constant. Relative to the
/// span's own polygon extent, so it is a shape test and not a size test.
const FOOT_STRAIGHT_EPS: f64 = 1e-9;

/// Coarse samples per span for the circle lane's scan — the plan's 16. Only the
/// no-seed and Newton-escaped paths pay for it; a seeded Newton is ~4 steps.
const FOOT_SCAN_SAMPLES: usize = 16;

/// Newton iteration cap for the circle lane's stationarity root. Reached only on
/// a configuration where the quintic is nearly flat, which the simple-root test
/// then refuses anyway.
const FOOT_NEWTON_MAX_ITER: usize = 40;

/// How far OUTSIDE `[0, 1]` a residual evaluation will follow the foot, in the
/// span's own parameter.
///
/// The span index is frozen per residual-model build, so a foot that solves to
/// within a hair of a span END would otherwise fall off its span the moment a
/// finite difference nudges a coordinate — and the row would jump from "the
/// residual at the foot" to the fallback, which is a discontinuity exactly where
/// the Jacobian is being measured. A cubic's polynomial continuation past its
/// span is smooth (the same four control points, the same Bernstein form), so
/// following the root a bounded distance into it keeps the row C¹ across the
/// seam. BOUNDED is the point: 1/64 of a span, never an unbounded extrapolation,
/// and the BUILD still accepts only `[0, 1]` so the span the row is frozen to is
/// never ambiguous.
pub(super) const FOOT_EVAL_MARGIN: f64 = 1.0 / 64.0;

/// Relaxation damping for the foot nudge — the same 0.5 the curvature nudge uses.
const FOOT_DAMP: f64 = 0.5;

/// A length at or below which a direction cannot be read off two points.
const FOOT_DEGENERATE_EPS: f64 = 1e-12;

/// What a spline span is being made tangent to.
#[derive(Clone, Copy, Debug)]
pub(super) enum FootTarget {
    /// A line through `origin` with UNIT direction `dir`.
    Line { origin: [f64; 2], dir: [f64; 2] },
    /// A circle (or arc) of centre `center` and radius `rho`.
    Circle { center: [f64; 2], rho: f64 },
}

/// The foot the solve settled on: which span, where in it, and whether the root
/// is simple (the only kind the residual model may differentiate through).
#[derive(Clone, Copy, Debug)]
pub(super) struct FootHit {
    pub span: usize,
    pub t: f64,
    pub simple: bool,
}

impl FootHit {
    /// The persisted form: `span + t`, one float in the spline's own chain
    /// parameter.
    pub(super) fn global(&self) -> f64 {
        self.span as f64 + self.t
    }
}

// ---------------------------------------------------------------------------
// Cubic Bézier evaluation (Bernstein form, the app's own spline convention)
// ---------------------------------------------------------------------------

/// The cubic Bernstein weights at `t` — also the sensitivity of `B(t)` to each
/// control point, which is what the relaxation nudge distributes along.
pub(super) fn bernstein3(t: f64) -> [f64; 4] {
    let u = 1.0 - t;
    [u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t]
}

/// `B(t)` for one span.
pub(super) fn eval_cubic(c: &SpanControls, t: f64) -> [f64; 2] {
    let w = bernstein3(t);
    let mut out = [0.0; 2];
    for i in 0..4 {
        out[0] += c[i][0] * w[i];
        out[1] += c[i][1] * w[i];
    }
    out
}

/// `B′(t) = 3 Σ (Cᵢ₊₁ − Cᵢ)·Bᵢ²(t)` — the hodograph, a QUADRATIC in `t`, which
/// is why the line lane's foot equation has closed-form roots.
pub(super) fn eval_cubic_d1(c: &SpanControls, t: f64) -> [f64; 2] {
    let u = 1.0 - t;
    let w = [3.0 * u * u, 6.0 * u * t, 3.0 * t * t];
    let mut out = [0.0; 2];
    for i in 0..3 {
        out[0] += (c[i + 1][0] - c[i][0]) * w[i];
        out[1] += (c[i + 1][1] - c[i][1]) * w[i];
    }
    out
}

/// `B″(t) = 6 Σ (Cᵢ₊₂ − 2Cᵢ₊₁ + Cᵢ)·Bᵢ¹(t)`.
pub(super) fn eval_cubic_d2(c: &SpanControls, t: f64) -> [f64; 2] {
    let w = [6.0 * (1.0 - t), 6.0 * t];
    let mut out = [0.0; 2];
    for i in 0..2 {
        for axis in 0..2 {
            out[axis] += (c[i + 2][axis] - 2.0 * c[i + 1][axis] + c[i][axis]) * w[i];
        }
    }
    out
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn norm(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}

/// The largest distance from a span's first control to any of the others — the
/// local length its length¹ residual is read against, and the scale the
/// straight-span test is relative to.
pub(super) fn span_extent(c: &SpanControls) -> f64 {
    (1..4).fold(0.0f64, |acc, i| acc.max(norm(sub(c[i], c[0]))))
}

impl FootTarget {
    /// The stationarity function whose roots are the candidate feet. Zero
    /// exactly where the distance from the target to the curve is stationary.
    fn g(&self, c: &SpanControls, t: f64) -> f64 {
        let d1 = eval_cubic_d1(c, t);
        match *self {
            FootTarget::Line { dir, .. } => cross(dir, d1),
            FootTarget::Circle { center, .. } => dot(sub(eval_cubic(c, t), center), d1),
        }
    }

    /// `(g′(t), the magnitude it is measured against)`. The ratio is what
    /// [`FOOT_SIMPLE_ROOT_EPS`] tests, so the double-root test is a SHAPE test:
    /// for the line lane it reads `sin(angle between d̂ and B″)`, i.e. "is the
    /// foot at an inflection", in no units at all.
    fn g_prime(&self, c: &SpanControls, t: f64) -> (f64, f64) {
        let d1 = eval_cubic_d1(c, t);
        let d2 = eval_cubic_d2(c, t);
        match *self {
            FootTarget::Line { dir, .. } => (cross(dir, d2), norm(d2)),
            FootTarget::Circle { center, .. } => {
                let radial = sub(eval_cubic(c, t), center);
                (
                    dot(d1, d1) + dot(radial, d2),
                    dot(d1, d1) + norm(radial) * norm(d2),
                )
            }
        }
    }

    /// The magnitude the stationarity function itself is naturally measured
    /// against — length¹ for the line lane (`cross(d̂, B′)`), length² for the
    /// circle lane (`dot(B − c, B′)`). What makes the Newton convergence test a
    /// relative one in both lanes rather than one lane's absolute bar applied to
    /// the other.
    fn g_scale(&self, c: &SpanControls, t: f64) -> f64 {
        let speed = norm(eval_cubic_d1(c, t));
        match *self {
            FootTarget::Line { .. } => speed,
            FootTarget::Circle { center, .. } => speed * norm(sub(eval_cubic(c, t), center)),
        }
    }

    /// Whether a root at `t` is SIMPLE — the only kind the residual model may
    /// differentiate through.
    fn root_is_simple(&self, c: &SpanControls, t: f64) -> bool {
        let (derivative, scale) = self.g_prime(c, t);
        scale > 0.0 && derivative.abs() > FOOT_SIMPLE_ROOT_EPS * scale
    }

    /// The constraint's residual at `t`, in LENGTH¹ units: the signed distance
    /// of the touch point from the line, or how far its distance to the centre
    /// misses the radius.
    ///
    /// Length¹ deliberately, for the reason every other atom here is: the LM
    /// polish minimizes the RAW residuals through `(JᵀJ + λI)Δ = −Jᵀr`, so an
    /// atom in area or angle units is weighted against the length atoms by an
    /// accident of scale.
    pub(super) fn residual(&self, c: &SpanControls, t: f64) -> f64 {
        let p = eval_cubic(c, t);
        match *self {
            FootTarget::Line { origin, dir } => cross(dir, sub(p, origin)),
            FootTarget::Circle { center, rho } => norm(sub(p, center)) - rho,
        }
    }

    /// The local length the length¹ residual is read against, so the SATISFIED
    /// bar is dimensionless — the same reading the `ϰ` lanes take. An absolute
    /// bar on a length residual would tighten as the model grows.
    pub(super) fn length_scale(&self, c: &SpanControls) -> f64 {
        match *self {
            FootTarget::Line { .. } => span_extent(c),
            FootTarget::Circle { rho, .. } => span_extent(c).max(rho),
        }
    }
}

// ---------------------------------------------------------------------------
// The foot solve
// ---------------------------------------------------------------------------

/// Every root of the line lane's stationarity function in `[lo, hi]`.
///
/// `g(t) = cross(d̂, B′(t))` is quadratic: with `cᵢ = cross(d̂, Cᵢ₊₁ − Cᵢ)` its
/// Bernstein coefficients, `g = c₀(1−t)² + 2c₁t(1−t) + c₂t²`. Closed form, so no
/// scan and no iteration — and the degenerate branches are explicit rather than
/// discovered: `A ≈ 0` is a linear `g` (one root), and all three coefficients
/// vanishing is a span whose tangent never leaves the line's direction, which is
/// not a touch point but a coincidence.
fn line_hodograph_roots(c: &SpanControls, dir: [f64; 2], lo: f64, hi: f64) -> Vec<f64> {
    let coefficients = [
        cross(dir, sub(c[1], c[0])),
        cross(dir, sub(c[2], c[1])),
        cross(dir, sub(c[3], c[2])),
    ];
    let scale = coefficients.iter().fold(0.0f64, |acc, &v| acc.max(v.abs()));
    let extent = span_extent(c);
    if !(scale > FOOT_STRAIGHT_EPS * extent.max(f64::MIN_POSITIVE)) {
        return Vec::new(); // straight (or line-parallel) span: no isolated foot.
    }
    let a = coefficients[0] - 2.0 * coefficients[1] + coefficients[2];
    let b = 2.0 * (coefficients[1] - coefficients[0]);
    let cc = coefficients[0];
    let mut roots: Vec<f64> = Vec::new();
    if a.abs() <= FOOT_STRAIGHT_EPS * scale {
        if b.abs() > FOOT_STRAIGHT_EPS * scale {
            roots.push(-cc / b);
        }
    } else {
        let disc = b * b - 4.0 * a * cc;
        if disc >= 0.0 {
            let root = disc.sqrt();
            roots.push((-b - root) / (2.0 * a));
            roots.push((-b + root) / (2.0 * a));
        }
    }
    roots.retain(|t| t.is_finite() && *t >= lo && *t <= hi);
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    roots.dedup_by(|a, b| (*a - *b).abs() <= 1e-15);
    roots
}

/// One Newton refinement of a stationarity root, clamped to `[lo, hi]`.
///
/// Driven to machine precision, NOT to a loose bar: the central-difference
/// Jacobian divides this residual by `2e-6·scale`, so a root left 1e-9 out in
/// `t` would put a 1e-3-relative error into the column. `None` when the step
/// leaves the interval or the derivative dies — the caller falls back to the
/// coarse scan, which is the plan's contract.
fn newton_root(target: &FootTarget, c: &SpanControls, seed: f64, lo: f64, hi: f64) -> Option<f64> {
    let mut t = seed.clamp(lo, hi);
    for _ in 0..FOOT_NEWTON_MAX_ITER {
        let value = target.g(c, t);
        let (derivative, scale) = target.g_prime(c, t);
        if !(derivative.abs() > FOOT_SIMPLE_ROOT_EPS * scale.max(f64::MIN_POSITIVE)) {
            return None; // flat here: a scan decides, not a divide by nothing.
        }
        let step = value / derivative;
        let next = t - step;
        if !next.is_finite() || next < lo || next > hi {
            return None;
        }
        let moved = (next - t).abs();
        t = next;
        if moved <= f64::EPSILON * (1.0 + t.abs()) {
            break;
        }
    }
    (target.g(c, t).abs() <= 1e-12 * target.g_scale(c, t).max(f64::MIN_POSITIVE)).then_some(t)
}

/// Every root of the stationarity function in `[lo, hi]` by coarse scan: sign
/// changes over [`FOOT_SCAN_SAMPLES`] intervals, each bracketed root bisected
/// and then Newton-polished. The circle lane's `g` is degree 5, so up to five
/// roots per span — the scan finds each one it brackets rather than assuming
/// there is one.
fn scan_roots(target: &FootTarget, c: &SpanControls, lo: f64, hi: f64) -> Vec<f64> {
    let mut roots: Vec<f64> = Vec::new();
    let step = (hi - lo) / FOOT_SCAN_SAMPLES as f64;
    let mut previous = (lo, target.g(c, lo));
    if previous.1 == 0.0 {
        roots.push(lo);
    }
    for i in 1..=FOOT_SCAN_SAMPLES {
        let t = if i == FOOT_SCAN_SAMPLES { hi } else { lo + step * i as f64 };
        let value = target.g(c, t);
        if value == 0.0 {
            roots.push(t);
        } else if previous.1 != 0.0 && (value > 0.0) != (previous.1 > 0.0) {
            let mut a = previous;
            let mut b = (t, value);
            // Bisection first: it cannot leave the bracket, so the Newton polish
            // below always starts from a root it is going to converge to.
            for _ in 0..60 {
                let mid = 0.5 * (a.0 + b.0);
                if mid <= a.0 || mid >= b.0 {
                    break;
                }
                let value_mid = target.g(c, mid);
                if (value_mid > 0.0) == (a.1 > 0.0) {
                    a = (mid, value_mid);
                } else {
                    b = (mid, value_mid);
                }
            }
            let bracketed = 0.5 * (a.0 + b.0);
            roots.push(newton_root(target, c, bracketed, lo, hi).unwrap_or(bracketed));
        }
        previous = (t, value);
    }
    roots.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    roots.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);
    roots
}

/// Every candidate foot in ONE span: closed form for the line lane, the coarse
/// scan for the circle lane.
fn foot_roots_in_span(target: &FootTarget, c: &SpanControls, lo: f64, hi: f64) -> Vec<f64> {
    match target {
        FootTarget::Line { dir, .. } => line_hodograph_roots(c, *dir, lo, hi),
        FootTarget::Circle { .. } => scan_roots(target, c, lo, hi),
    }
}

/// The foot in ONE span, seeded — what the residual model evaluates at, with the
/// span already frozen.
///
/// Newton from the seed is the whole of the circle lane's fast path (the plan's
/// contract: a scan only when the seed is missing or Newton leaves the span);
/// the line lane's roots are closed form, so the seed only PICKS between them.
/// Whichever route, the root nearest the seed wins, which is what keeps the
/// touch point on its own branch across a solve and across a drag.
pub(super) fn foot_in_span(
    target: &FootTarget,
    c: &SpanControls,
    seed: f64,
    lo: f64,
    hi: f64,
) -> Option<f64> {
    if let FootTarget::Circle { .. } = target {
        if let Some(t) = newton_root(target, c, seed, lo, hi) {
            return Some(t);
        }
    }
    let roots = foot_roots_in_span(target, c, lo, hi);
    roots
        .into_iter()
        .min_by(|a, b| {
            (a - seed)
                .abs()
                .partial_cmp(&(b - seed).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
}

/// The foot across a WHOLE spline: which span it sits in and where.
///
/// With a `seed` (the persisted `span + t`) the candidate nearest it wins — in
/// the seeded span or in a neighbouring one, which is how a dragged spline's
/// touch point migrates across a span boundary without the constraint jumping to
/// an unrelated part of the curve. Without a seed — the first solve after the
/// constraint is applied — the candidate with the SMALLEST residual wins: the
/// tangency this sketch is closest to already having, which is the one the user
/// picked the curve for. Ties go to the earlier parameter, so the answer is
/// deterministic.
pub(super) fn solve_spline_foot(
    target: &FootTarget,
    spans: &[SpanControls],
    seed: Option<f64>,
) -> Option<FootHit> {
    let mut best: Option<(f64, FootHit)> = None;
    for (span, controls) in spans.iter().enumerate() {
        let roots = match seed {
            // The seeded span keeps the Newton path; every other span is scanned,
            // because a seed says nothing about where a root in a DIFFERENT span
            // would be.
            Some(value) if (value.floor() as isize) == span as isize => {
                match foot_in_span(target, controls, value - span as f64, 0.0, 1.0) {
                    Some(t) => vec![t],
                    None => foot_roots_in_span(target, controls, 0.0, 1.0),
                }
            }
            _ => foot_roots_in_span(target, controls, 0.0, 1.0),
        };
        for t in roots {
            let hit = FootHit {
                span,
                t,
                simple: target.root_is_simple(controls, t),
            };
            let rank = match seed {
                Some(value) => (hit.global() - value).abs(),
                None => target.residual(controls, t).abs(),
            };
            if best.as_ref().map_or(true, |(current, _)| rank < *current) {
                best = Some((rank, hit));
            }
        }
    }
    best.map(|(_, hit)| hit)
}

// ---------------------------------------------------------------------------
// The constraint
// ---------------------------------------------------------------------------

/// A spline BODY, named by its first two control ids acting as a geometry
/// handle. Built by [`Engine::spline_body_info`].
#[derive(Clone, Debug)]
pub(super) struct SplineBody {
    /// Span count `n` (`3n + 1` control points).
    pub seg_count: usize,
    /// Engine point indices of the WHOLE control polygon, in chain order.
    pub controls: Vec<usize>,
}

impl Engine {
    /// The spline whose first two control points are `(a, b)`, in either slot
    /// order — the `(s0, s1)` geometry handle a `∿` constraint's point list
    /// carries.
    ///
    /// The pair is a HANDLE, not a tangent side: `∿` constrains the curve
    /// somewhere along a span, so the constraint has no anchor to name and needs
    /// the spline itself. Two ids are enough to identify one (a document cannot
    /// have two splines starting from the same two points without them being the
    /// same curve), and using the first two rather than a new document field is
    /// what keeps the record the same `{type, points}` shape every other
    /// constraint has.
    ///
    /// `None` for anything that is not a spline's leading pair — including a
    /// spline whose polygon names a point that does not exist, which is a corrupt
    /// document and not a case this constraint can answer on.
    pub(super) fn spline_body_info(&self, a: usize, b: usize) -> Option<SplineBody> {
        let key_a = point_key(&self.points[a].id);
        let key_b = point_key(&self.points[b].id);
        for geometry in &self.geometries {
            let Some(obj) = geometry.as_object() else {
                continue;
            };
            let geometry_type = obj.get("type").and_then(Value::as_str).unwrap_or("");
            if !is_spline_geometry_type(geometry_type) {
                continue;
            }
            let Some(points) = obj.get("points").and_then(Value::as_array) else {
                continue;
            };
            if points.len() < 4 {
                continue;
            }
            let first = point_key(&points[0]);
            let second = point_key(&points[1]);
            if !((first == key_a && second == key_b) || (first == key_b && second == key_a)) {
                continue;
            }
            let mut controls = Vec::with_capacity(points.len());
            for id in points {
                controls.push(self.point_index_of(id)?);
            }
            return Some(SplineBody {
                seg_count: (points.len() - 1) / 3,
                controls,
            });
        }
        None
    }

    /// One span's four control POINT INDICES.
    pub(super) fn span_point_indices(body: &SplineBody, span: usize) -> [usize; 4] {
        let i0 = 3 * span;
        [
            body.controls[i0],
            body.controls[i0 + 1],
            body.controls[i0 + 2],
            body.controls[i0 + 3],
        ]
    }

    /// Every span of a spline as a control polygon, read from the live points.
    fn span_controls_live(&self, body: &SplineBody) -> Vec<SpanControls> {
        (0..body.seg_count)
            .map(|span| {
                Self::span_point_indices(body, span).map(|index| self.point_xy(index))
            })
            .collect()
    }

    /// The `∿` point list plus every control point of the spline it names.
    ///
    /// The move signature has to cover every point the constraint MOVES, and
    /// this one moves span controls its own `points` never mention — the
    /// `(s0, s1)` pair is a geometry handle, not the geometry. Leave them out and
    /// a pass that nudged only a span's handles reads as a pass that changed
    /// nothing, which sets `status_solved`, arms the shortcut in
    /// [`Engine::process_single_constraint`] and freezes the constraint one nudge
    /// into its convergence — the same trap `curvature_signature_indices` exists
    /// for.
    ///
    /// The WHOLE polygon, not just the frozen span: which span the foot sits in
    /// is an OUTPUT of the solve, so a signature pinned to one span would arm the
    /// shortcut exactly when the foot is migrating.
    pub(super) fn foot_signature_indices(&self, indices: &[Option<usize>]) -> Vec<Option<usize>> {
        let mut out = indices.to_vec();
        let pair = |first: usize, second: usize| {
            match (
                indices.get(first).copied().flatten(),
                indices.get(second).copied().flatten(),
            ) {
                (Some(a), Some(b)) => self.spline_body_info(a, b),
                _ => None,
            }
        };
        if let Some(body) = pair(0, 1).or_else(|| pair(2, 3)) {
            out.extend(body.controls.iter().map(|index| Some(*index)));
        }
        out
    }

    /// Resolve a `∿` constraint's point list into "the spline body" and "the
    /// other pair". Shared by the relaxation and the residual-model build so both
    /// read one encoding.
    ///
    /// The spline body is recognized FIRST on both pairs, which is what
    /// disambiguates the sketcher's handle guide: `(s0, s1)` is simultaneously a
    /// spline's leading pair and a `line` geometry, and read as the line it would
    /// be a tangency of the curve to its own guide.
    pub(super) fn foot_roles(
        &self,
        indices: &[Option<usize>],
    ) -> Result<(SplineBody, usize, usize), String> {
        let p0 = self.req(indices, 0)?;
        let p1 = self.req(indices, 1)?;
        let p2 = self.req(indices, 2)?;
        let p3 = self.req(indices, 3)?;
        match (self.spline_body_info(p0, p1), self.spline_body_info(p2, p3)) {
            (Some(_), Some(_)) => Err(
                "Span tangent: both pairs name a spline body; one of them must be the line or circle"
                    .to_string(),
            ),
            (Some(body), None) => Ok((body, p2, p3)),
            (None, Some(body)) => Ok((body, p0, p1)),
            (None, None) => Err(
                "Span tangent: no spline body in the constraint (its point list needs the spline's first two control points)"
                    .to_string(),
            ),
        }
    }

    /// The target the span is made tangent to, read from the live points. A
    /// LINE when the other pair is a line geometry — the same test `⌒` makes —
    /// and a circle/arc `(centre, rim)` otherwise.
    fn foot_target_live(&self, q0: usize, q1: usize) -> Result<FootTarget, String> {
        let tolerance = self.tolerance();
        let length = self.distance(q0, q1);
        if self.is_line_pair(q0, q1) {
            if length < tolerance {
                return Err("Span tangent: the line is degenerate".to_string());
            }
            let origin = self.point_xy(q0);
            let far = self.point_xy(q1);
            Ok(FootTarget::Line {
                origin,
                dir: [(far[0] - origin[0]) / length, (far[1] - origin[1]) / length],
            })
        } else {
            if length < tolerance {
                return Err("Span tangent: the circle is degenerate".to_string());
            }
            Ok(FootTarget::Circle {
                center: self.point_xy(q0),
                rho: length,
            })
        }
    }

    /// Tangency at a point interior to a span (`∿`).
    ///
    /// One pass: resolve the roles, solve the foot across the whole spline,
    /// PERSIST it (so the next pass, the next solve and the next drag frame all
    /// start from this branch), then either report satisfied or nudge.
    ///
    /// The named refusals are the contract, not decoration. A configuration with
    /// no stationary point, and a foot that has landed on an inflection, are both
    /// states where this construction has no answer — the first because there is
    /// no touch point to hold, the second because `dt*/dcoords` is unbounded
    /// there and the Jacobian column through it would be noise. Saying so is the
    /// solver-level form of the kernel's refuse-don't-lie contract; the
    /// alternative is a junk step that moves the sketch somewhere nobody asked
    /// for.
    pub(super) fn c_spline_foot_tangent(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
    ) -> CResult {
        let (body, q0, q1) = match self.foot_roles(indices) {
            Ok(roles) => roles,
            Err(message) => {
                constraint.set_error(Value::String(message));
                return Ok(Value::Null);
            }
        };
        let target = match self.foot_target_live(q0, q1) {
            Ok(target) => target,
            Err(message) => {
                constraint.set_error(Value::String(message));
                return Ok(Value::Null);
            }
        };
        let spans = self.span_controls_live(&body);
        let Some(hit) = solve_spline_foot(&target, &spans, constraint.foot_t) else {
            constraint.set_error(Value::String(match target {
                FootTarget::Line { .. } => "Span tangent: the spline has no point where its tangent is parallel to the line".to_string(),
                FootTarget::Circle { .. } => "Span tangent: the spline has no point where it faces the circle's centre squarely".to_string(),
            }));
            return Ok(Value::Null);
        };
        // Persist before any refusal: the foot the solve FOUND is what the next
        // build should freeze its span from, whether or not this pass could use
        // it. (A graze the user then drags out of resumes from where it was.)
        constraint.set_foot_t(hit.global());
        if !hit.simple {
            constraint.set_error(Value::String(format!(
                "Span tangent: the touch point is an inflection of the spline (span {}, t {}) — the foot is a double root there and the constraint has no stable step",
                hit.span,
                fmt_number(hit.t)
            )));
            return Ok(Value::Null);
        }

        let controls = &spans[hit.span];
        let residual = target.residual(controls, hit.t);
        let scale = target.length_scale(controls);
        if residual.abs() <= self.tolerance() * scale {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        let span = Self::span_point_indices(&body, hit.span);
        let weights = self.free_span_weight(&span, hit.t);
        let span_movable = weights > 0.0;
        let target_movable = match target {
            // A line moves RIGIDLY or not at all: translating both ends keeps the
            // direction the foot was solved for, where moving one end rotates the
            // line about the other and can push the offset at the foot the wrong
            // way. Same policy `rotate_spline_and_circle` applies to a circle.
            FootTarget::Line { .. } => !self.points[q0].fixed && !self.points[q1].fixed,
            FootTarget::Circle { .. } => !self.points[q0].fixed || !self.points[q1].fixed,
        };
        if !span_movable && !target_movable {
            constraint.set_error(Value::String(
                "Span tangent: the spline span and the line or circle are both fully fixed".into(),
            ));
            return Ok(Value::Null);
        }
        let share = if span_movable && target_movable { 0.5 } else { 1.0 };
        let touch = eval_cubic(controls, hit.t);
        match target {
            FootTarget::Line { dir, .. } => {
                constraint.set_error(Value::String(format!(
                    "Span tangent constraint not satisfied\n        spline touch point {} from the line",
                    fmt_number(residual)
                )));
                let normal = [-dir[1], dir[0]];
                // `r = (B(t*) − p)·n̂`: the curve closes it by moving −r along n̂,
                // the line by +r.
                if span_movable {
                    self.nudge_span_along(&span, hit.t, normal, -residual * FOOT_DAMP * share);
                }
                if target_movable {
                    let step = residual * FOOT_DAMP * share;
                    for index in [q0, q1] {
                        self.points[index].x += normal[0] * step;
                        self.points[index].y += normal[1] * step;
                    }
                }
            }
            FootTarget::Circle { center, rho } => {
                constraint.set_error(Value::String(format!(
                    "Span tangent constraint not satisfied\n        spline touch point at radius {} against {}",
                    fmt_number(rho + residual),
                    fmt_number(rho)
                )));
                let radial = sub(touch, center);
                let distance = norm(radial);
                if distance <= FOOT_DEGENERATE_EPS {
                    constraint.set_error(Value::String(
                        "Span tangent: the touch point is the circle's centre".into(),
                    ));
                    return Ok(Value::Null);
                }
                let unit = [radial[0] / distance, radial[1] / distance];
                if span_movable {
                    self.nudge_span_along(&span, hit.t, unit, -residual * FOOT_DAMP * share);
                }
                if target_movable {
                    // Drive the radius to the distance the curve actually is —
                    // the existing helper already splits that over whichever of
                    // (centre, rim) is free.
                    self.adjust_circle_radius(q0, q1, distance, FOOT_DAMP * share);
                }
            }
        }
        Ok(Value::Null)
    }

    /// `Σ wᵢ²` over the span's FREE controls at `t` — zero exactly when the span
    /// cannot move its curve at the foot at all (every control fixed, or the only
    /// free ones carrying no Bernstein weight there).
    fn free_span_weight(&self, span: &[usize; 4], t: f64) -> f64 {
        self.span_nudge_weights(span, t)
            .iter()
            .map(|(_, weight)| weight * weight)
            .sum()
    }

    /// The free controls of one span and their Bernstein weights at `t`. A
    /// control that appears TWICE in the polygon (a cusp, built by repeating an
    /// id) gets its weights added: it is one point and moves once.
    fn span_nudge_weights(&self, span: &[usize; 4], t: f64) -> Vec<(usize, f64)> {
        let weights = bernstein3(t);
        let mut out: Vec<(usize, f64)> = Vec::with_capacity(4);
        for slot in 0..4 {
            let index = span[slot];
            if self.points[index].fixed {
                continue;
            }
            match out.iter_mut().find(|(existing, _)| *existing == index) {
                Some((_, weight)) => *weight += weights[slot],
                None => out.push((index, weights[slot])),
            }
        }
        out
    }

    /// Move a span's FREE controls along `dir` so that `B(t)` travels `delta`
    /// along it.
    ///
    /// Bernstein weights, minimum norm: `B(t) = Σ wᵢCᵢ`, so `δᵢ = delta·wᵢ/Σw²`
    /// is the smallest polygon change that produces the move. That distributes
    /// the correction the way the curve actually depends on its polygon —
    /// a control the foot barely feels is barely moved — and it leaves a fixed
    /// control alone, so a span with grounded anchors still closes the gap with
    /// its handles.
    fn nudge_span_along(&mut self, span: &[usize; 4], t: f64, dir: [f64; 2], delta: f64) -> bool {
        let weights = self.span_nudge_weights(span, t);
        let square: f64 = weights.iter().map(|(_, weight)| weight * weight).sum();
        if !(square > 0.0) || !delta.is_finite() {
            return false;
        }
        for (index, weight) in weights {
            let step = delta * weight / square;
            self.points[index].x += dir[0] * step;
            self.points[index].y += dir[1] * step;
        }
        true
    }

    /// Re-solve every `∿` constraint's foot on the configuration the solve ENDS
    /// on, and persist it.
    ///
    /// Relaxation's last word is the foot of the configuration relaxation
    /// stopped at; the LM polish then moves the points. Without this the emitted
    /// `_splineFootT` describes a configuration that no longer exists, and the
    /// next solve (or the next drag frame) would seed from it — so the stored
    /// touch point would lag one solve behind the geometry it belongs to.
    pub(super) fn refresh_spline_foot_seeds(&mut self) {
        for index in 0..self.constraints.len() {
            if self.constraints[index].ctype != CType::SplineFootTangent {
                continue;
            }
            let seed = self.constraints[index].foot_t;
            let point_idx = self.constraints[index].point_idx.clone();
            let resolved = self
                .foot_roles(&point_idx)
                .ok()
                .and_then(|(body, q0, q1)| {
                    let target = self.foot_target_live(q0, q1).ok()?;
                    let spans = self.span_controls_live(&body);
                    solve_spline_foot(&target, &spans, seed)
                });
            if let Some(hit) = resolved {
                self.constraints[index].set_foot_t(hit.global());
            }
        }
    }
}

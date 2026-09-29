//! Composable radius laws over a chain's cumulative arc-length abscissa for
//! our variable-radius fillet lane — the OCCT `Law_Composite` /
//! `Law_Constant` / `Law_S` / `Law_Interpol` model, in which the marcher
//! evaluates `ray = Law(t)` at each station so that ONE chain carries
//! per-edge/per-vertex radii and mixed constant/variable segments joined by
//! smooth transitions.
//!
//! A [`RadiusLaw`] is built from user segments (constant, linear, interpolated
//! point sets) laid end to end on the abscissa.  Wherever two adjacent
//! segments meet with a value or slope mismatch, the builder inserts a smooth
//! S-transition: a quintic Hermite bridge that matches the neighbouring
//! segments' value and first derivative at the window boundaries and carries
//! ZERO second derivative there.  Constant and linear segments also have zero
//! second derivative, so the composite is C2 at every constructed joint (the
//! C1 contract with headroom); interpolated segments are C1 monotone
//! (Fritsch–Carlson PCHIP), so joints against them are C1.
//!
//! Transition-window sizing is derived from the adjoining segment GEOMETRY,
//! not a tolerance: each junction claims exactly HALF of each adjoining
//! segment.  Half is the largest window that can never collide with the
//! neighbouring junction's window — two windows meeting inside one segment
//! meet exactly at its midpoint, where both evaluate the underlying segment
//! itself, so coverage stays contiguous and C1 by construction.
//!
//! Endpoint radii are met EXACTLY: no window ever reaches past a segment's
//! midpoint, so the law's first and last values are the untouched user values.
//! Evaluation is a binary search plus one Horner evaluation of a degree ≤ 5
//! polynomial — cheap enough to call per march station.

use crate::{KernelRefusal, KernelStage};

/// One user segment of a composite radius law, spanning `length` of abscissa.
#[derive(Clone, Debug)]
pub enum LawSegment {
    /// Constant radius over `length` (OCCT `Law_Constant`).
    Constant { length: f64, radius: f64 },
    /// Linear ramp from `start_radius` to `end_radius` over `length`
    /// (OCCT `Law_Linear`).
    Linear {
        length: f64,
        start_radius: f64,
        end_radius: f64,
    },
    /// Monotone C1 interpolation through `(abscissa_offset, radius)` points
    /// (OCCT `Law_Interpol`).  Offsets are measured from the segment start;
    /// the first must be exactly `0.0`, offsets strictly increase, and the
    /// last offset is the segment length.  Interpolation is shape-preserving
    /// (Fritsch–Carlson PCHIP): the law never overshoots the point radii, so
    /// positive inputs stay positive.
    Interpolated { points: Vec<(f64, f64)> },
}

/// One polynomial piece of the composite: `radius(s)` for `s ∈ [s0, s1]`,
/// evaluated as a degree ≤ 5 polynomial in the normalized `u = (s−s0)/(s1−s0)`.
#[derive(Clone, Debug)]
struct Piece {
    s0: f64,
    s1: f64,
    /// Power-basis coefficients `c0 + c1·u + … + c5·u⁵`.
    coeffs: [f64; 6],
}

impl Piece {
    fn width(&self) -> f64 {
        self.s1 - self.s0
    }

    fn evaluate(&self, s: f64) -> f64 {
        let u = ((s - self.s0) / self.width()).clamp(0.0, 1.0);
        poly_eval(&self.coeffs, u)
    }
}

/// A composable radius law over cumulative abscissa `[0, total_length]`.
///
/// Construction validates every input (refuse-or-exact: non-positive radii,
/// non-positive lengths, and non-monotone interpolation abscissas are named
/// errors); a constructed law is deterministic and cheap to evaluate.
#[derive(Clone, Debug)]
pub struct RadiusLaw {
    pieces: Vec<Piece>,
    total: f64,
}

impl RadiusLaw {
    /// A constant law: `radius` everywhere on `[0, length]`.
    pub fn constant(length: f64, radius: f64) -> Result<Self, KernelRefusal> {
        Self::from_segments(&[LawSegment::Constant { length, radius }])
    }

    /// The per-vertex chain model: radius `vertex_radii[i]` at chain vertex
    /// `i`, smoothly interpolated along the chain.  `edge_lengths[k]` is the
    /// arc length of chain edge `k`, so the abscissa breakpoints sit at the
    /// chain's edge junctions; `vertex_radii` needs exactly one entry per
    /// vertex (`edge_lengths.len() + 1`).  Interpolation is monotone C1
    /// (PCHIP): every vertex radius is met exactly and the law never
    /// overshoots the given radii.
    pub fn from_vertex_radii(edge_lengths: &[f64], vertex_radii: &[f64]) -> Result<Self, KernelRefusal> {
        if edge_lengths.is_empty() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "chain_edges",
                "radius_law: at least one chain edge length is required",
            ));
        }
        if vertex_radii.len() != edge_lengths.len() + 1 {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "vertex_radii_count",
                format!(
                    "radius_law: need exactly one radius per chain vertex (edges + 1): \
                     {} edges need {} radii, got {}",
                    edge_lengths.len(),
                    edge_lengths.len() + 1,
                    vertex_radii.len()
                ),
            ));
        }
        for length in edge_lengths {
            if !(length.is_finite() && *length > 0.0) {
                return Err(segment_length_refusal());
            }
        }
        let mut points = Vec::with_capacity(vertex_radii.len());
        let mut abscissa = 0.0;
        points.push((0.0, vertex_radii[0]));
        for (length, radius) in edge_lengths.iter().zip(vertex_radii[1..].iter()) {
            abscissa += length;
            points.push((abscissa, *radius));
        }
        Self::from_segments(&[LawSegment::Interpolated { points }])
    }

    /// Compose a law from consecutive segments with smooth junction
    /// transitions (see the module docs for the transition model).
    pub fn from_segments(segments: &[LawSegment]) -> Result<Self, KernelRefusal> {
        if segments.is_empty() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "segments",
                "radius_law: at least one segment is required",
            ));
        }
        // 1. Validate and normalize each segment into an evaluator with an
        //    absolute abscissa span.
        let mut evals: Vec<SegmentEval> = Vec::with_capacity(segments.len());
        let mut cursor = 0.0_f64;
        for segment in segments {
            let eval = SegmentEval::build(segment, cursor)?;
            cursor = eval.s_end();
            evals.push(eval);
        }
        let total = cursor;

        // 2. Decide which junctions need a smooth transition: exact value AND
        //    slope agreement passes through untouched (already C1); any
        //    mismatch gets the quintic bridge.
        let mut needs_transition = vec![false; evals.len().saturating_sub(1)];
        for i in 0..needs_transition.len() {
            let s_j = evals[i].s_end();
            let (vl, dl) = evals[i].value_slope(s_j);
            let (vr, dr) = evals[i + 1].value_slope(s_j);
            needs_transition[i] = vl != vr || dl != dr;
        }

        // 3. Emit pieces: each segment keeps its span minus the half-segment
        //    windows claimed by transitions at its junctions; each transition
        //    spans from the left segment's midpoint boundary to the right
        //    segment's midpoint boundary (window per side = half the adjoining
        //    segment — the maximal size that can never overlap the next
        //    junction's window).
        let mut pieces: Vec<Piece> = Vec::new();
        for i in 0..evals.len() {
            let left_trim = i > 0 && needs_transition[i - 1];
            let right_trim = i < needs_transition.len() && needs_transition[i];
            let mid = evals[i].s_start() + evals[i].length() * 0.5;
            let keep_from = if left_trim { mid } else { evals[i].s_start() };
            let keep_to = if right_trim { mid } else { evals[i].s_end() };
            if keep_to > keep_from {
                evals[i].emit_pieces(keep_from, keep_to, &mut pieces);
            }
            if right_trim {
                let a_s = evals[i].s_start() + evals[i].length() * 0.5;
                let b_s = evals[i + 1].s_start() + evals[i + 1].length() * 0.5;
                let (a, da) = evals[i].value_slope(a_s);
                let (b, db) = evals[i + 1].value_slope(b_s);
                pieces.push(quintic_bridge(a_s, b_s, a, da, b, db));
            }
        }
        debug_assert!(pieces
            .windows(2)
            .all(|pair| (pair[0].s1 - pair[1].s0).abs() == 0.0));
        Ok(Self { pieces, total })
    }

    /// Total abscissa length of the law's domain.
    pub fn total_length(&self) -> f64 {
        self.total
    }

    /// Evaluate the radius at abscissa `s` (clamped into `[0, total_length]`).
    pub fn radius_at(&self, s: f64) -> f64 {
        debug_assert!(s.is_finite(), "radius_law: abscissa must be finite");
        let s = s.clamp(0.0, self.total);
        // First piece whose end reaches s.
        let index = self
            .pieces
            .partition_point(|piece| piece.s1 < s)
            .min(self.pieces.len() - 1);
        self.pieces[index].evaluate(s)
    }

    /// Evaluate at normalized abscissa `fraction ∈ [0, 1]` of the domain.
    pub fn radius_at_fraction(&self, fraction: f64) -> f64 {
        self.radius_at(fraction * self.total)
    }

    /// Upper bound of `|d²radius/ds²|` over `[s_a, s_b]`, used by callers to
    /// derive a sampling density from the law's curvature (piecewise-linear
    /// interpolation error of a C1, piecewise-C2 function over step `h` is at
    /// most `h²·max|r''|/8`).  Per piece the bound is exact-family: the second
    /// derivative of a degree ≤ 5 piece is a cubic, and the maximum absolute
    /// value of a polynomial is bounded by the maximum absolute Bernstein
    /// coefficient (convex-hull property).  Pieces partially overlapping the
    /// query span use their whole-piece bound (conservative).
    pub fn max_second_derivative(&self, s_a: f64, s_b: f64) -> f64 {
        let lo = s_a.min(s_b).clamp(0.0, self.total);
        let hi = s_a.max(s_b).clamp(0.0, self.total);
        let mut bound = 0.0_f64;
        for piece in &self.pieces {
            if piece.s1 < lo || piece.s0 > hi {
                continue;
            }
            // d²/du²: degree ≤ 3 in u.
            let c = &piece.coeffs;
            let dd = [2.0 * c[2], 6.0 * c[3], 12.0 * c[4], 20.0 * c[5]];
            // Power → Bernstein (degree 3): b_j = Σ_{k≤j} a_k·C(j,k)/C(3,k).
            let b0 = dd[0];
            let b1 = dd[0] + dd[1] / 3.0;
            let b2 = dd[0] + 2.0 * dd[1] / 3.0 + dd[2] / 3.0;
            let b3 = dd[0] + dd[1] + dd[2] + dd[3];
            let max_u = b0.abs().max(b1.abs()).max(b2.abs()).max(b3.abs());
            let width = piece.width();
            if width > 0.0 {
                bound = bound.max(max_u / (width * width));
            }
        }
        bound
    }
}

/// Validated per-segment evaluator on an absolute abscissa span.
enum SegmentEval {
    Constant {
        s0: f64,
        length: f64,
        radius: f64,
    },
    Linear {
        s0: f64,
        length: f64,
        r0: f64,
        r1: f64,
    },
    Interpolated {
        s0: f64,
        /// Absolute abscissas of the interpolation points.
        xs: Vec<f64>,
        ys: Vec<f64>,
        /// PCHIP slopes at the points (radius per abscissa).
        ds: Vec<f64>,
    },
}

fn check_radius(radius: f64) -> Result<(), KernelRefusal> {
    if !(radius.is_finite() && radius > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "radius",
            "radius_law: every radius must be positive and finite",
        ));
    }
    Ok(())
}

/// The one refusal every segment kind shares: a span that is not a positive
/// finite length.  Minted once so the three sites carry one class and slug.
fn segment_length_refusal() -> KernelRefusal {
    KernelRefusal::input(
        KernelStage::Collect,
        "segment_length",
        "radius_law: segment length must be positive and finite",
    )
}

impl SegmentEval {
    fn build(segment: &LawSegment, s0: f64) -> Result<Self, KernelRefusal> {
        match segment {
            LawSegment::Constant { length, radius } => {
                if !(length.is_finite() && *length > 0.0) {
                    return Err(segment_length_refusal());
                }
                check_radius(*radius)?;
                Ok(Self::Constant {
                    s0,
                    length: *length,
                    radius: *radius,
                })
            }
            LawSegment::Linear {
                length,
                start_radius,
                end_radius,
            } => {
                if !(length.is_finite() && *length > 0.0) {
                    return Err(segment_length_refusal());
                }
                check_radius(*start_radius)?;
                check_radius(*end_radius)?;
                Ok(Self::Linear {
                    s0,
                    length: *length,
                    r0: *start_radius,
                    r1: *end_radius,
                })
            }
            LawSegment::Interpolated { points } => {
                if points.len() < 2 {
                    return Err(KernelRefusal::input(
                        KernelStage::Collect,
                        "interpolation_points",
                        "radius_law: an interpolated segment needs at least two points",
                    ));
                }
                if points[0].0 != 0.0 {
                    return Err(KernelRefusal::input(
                        KernelStage::Collect,
                        "interpolation_start",
                        "radius_law: interpolation abscissas must start at exactly 0",
                    ));
                }
                for pair in points.windows(2) {
                    if !(pair[1].0.is_finite() && pair[1].0 > pair[0].0) {
                        return Err(KernelRefusal::input(
                            KernelStage::Collect,
                            "interpolation_monotone",
                            "radius_law: interpolation abscissas must be strictly increasing",
                        ));
                    }
                }
                for (_, radius) in points {
                    check_radius(*radius)?;
                }
                let xs: Vec<f64> = points.iter().map(|(x, _)| s0 + x).collect();
                let ys: Vec<f64> = points.iter().map(|(_, y)| *y).collect();
                let ds = pchip_slopes(&xs, &ys);
                Ok(Self::Interpolated { s0, xs, ys, ds })
            }
        }
    }

    fn s_start(&self) -> f64 {
        match self {
            Self::Constant { s0, .. } | Self::Linear { s0, .. } | Self::Interpolated { s0, .. } => {
                *s0
            }
        }
    }

    fn length(&self) -> f64 {
        match self {
            Self::Constant { length, .. } | Self::Linear { length, .. } => *length,
            Self::Interpolated { s0, xs, .. } => xs[xs.len() - 1] - s0,
        }
    }

    fn s_end(&self) -> f64 {
        match self {
            Self::Constant { s0, length, .. } | Self::Linear { s0, length, .. } => s0 + length,
            Self::Interpolated { xs, .. } => xs[xs.len() - 1],
        }
    }

    /// Value and first derivative (radius per abscissa) of the ORIGINAL
    /// segment at `s` — junction windows take their boundary conditions from
    /// here, so consecutive windows meeting at a segment midpoint agree
    /// exactly.
    fn value_slope(&self, s: f64) -> (f64, f64) {
        match self {
            Self::Constant { radius, .. } => (*radius, 0.0),
            Self::Linear {
                s0,
                length,
                r0,
                r1,
            } => {
                let slope = (r1 - r0) / length;
                (r0 + slope * (s - s0), slope)
            }
            Self::Interpolated { xs, ys, ds, .. } => {
                let i = interval_index(xs, s);
                let h = xs[i + 1] - xs[i];
                let coeffs = hermite_cubic(ys[i], ds[i] * h, ys[i + 1], ds[i + 1] * h);
                let u = ((s - xs[i]) / h).clamp(0.0, 1.0);
                let deriv = poly_eval(&poly_derivative(&coeffs), u) / h;
                (poly_eval(&coeffs, u), deriv)
            }
        }
    }

    /// Append this segment's polynomial pieces restricted to `[from, to]`.
    fn emit_pieces(&self, from: f64, to: f64, out: &mut Vec<Piece>) {
        match self {
            Self::Constant { radius, .. } => out.push(Piece {
                s0: from,
                s1: to,
                coeffs: [*radius, 0.0, 0.0, 0.0, 0.0, 0.0],
            }),
            Self::Linear { .. } => {
                let (va, slope) = self.value_slope(from);
                out.push(Piece {
                    s0: from,
                    s1: to,
                    coeffs: [va, slope * (to - from), 0.0, 0.0, 0.0, 0.0],
                });
            }
            Self::Interpolated { xs, ys, ds, .. } => {
                for i in 0..xs.len() - 1 {
                    let (x0, x1) = (xs[i], xs[i + 1]);
                    let (a, b) = (x0.max(from), x1.min(to));
                    if b <= a {
                        continue;
                    }
                    let h = x1 - x0;
                    let cubic = hermite_cubic(ys[i], ds[i] * h, ys[i + 1], ds[i + 1] * h);
                    let coeffs = poly_restrict(&cubic, (a - x0) / h, (b - x0) / h);
                    out.push(Piece {
                        s0: a,
                        s1: b,
                        coeffs,
                    });
                }
            }
        }
    }
}

/// The quintic Hermite S-bridge over `[a_s, b_s]`: matches value and first
/// derivative of the neighbouring segments at the window boundaries and
/// carries zero second derivative there (so joints against constant/linear
/// segments are C2).  With flat sides (`da = db = 0`) it degenerates to the
/// classic monotone smoothstep `10u³ − 15u⁴ + 6u⁵` scaled between the two
/// values — the OCCT `Law_S` shape with no overshoot.
fn quintic_bridge(a_s: f64, b_s: f64, a: f64, da: f64, b: f64, db: f64) -> Piece {
    let w = b_s - a_s;
    // End conditions in normalized u: q(0)=a, q'(0)=A1, q''(0)=0,
    // q(1)=b, q'(1)=B1, q''(1)=0, with slopes scaled by the window width.
    let a1 = da * w;
    let b1 = db * w;
    let d = b - a - a1;
    let e = b1 - a1;
    // Solving the three remaining equations for c3..c5 gives:
    //   c3 = 10D − 4E,  c4 = 7E − 15D,  c5 = 6D − 3E.
    Piece {
        s0: a_s,
        s1: b_s,
        coeffs: [
            a,
            a1,
            0.0,
            10.0 * d - 4.0 * e,
            7.0 * e - 15.0 * d,
            6.0 * d - 3.0 * e,
        ],
    }
}

/// Horner evaluation of a degree ≤ 5 power-basis polynomial.
fn poly_eval(coeffs: &[f64; 6], u: f64) -> f64 {
    let mut value = coeffs[5];
    for k in (0..5).rev() {
        value = value * u + coeffs[k];
    }
    value
}

/// Coefficients of the derivative (in `u`) of a degree ≤ 5 polynomial.
fn poly_derivative(coeffs: &[f64; 6]) -> [f64; 6] {
    let mut out = [0.0; 6];
    for k in 1..6 {
        out[k - 1] = coeffs[k] * k as f64;
    }
    out
}

/// Coefficients of `p(x0 + (x1 − x0)·t)` for `t ∈ [0, 1]` — restricting a
/// normalized polynomial piece to a sub-interval of its own domain.
fn poly_restrict(coeffs: &[f64; 6], x0: f64, x1: f64) -> [f64; 6] {
    let h = x1 - x0;
    let mut result = [0.0; 6];
    // power = (x0 + h·t)^k, maintained iteratively.
    let mut power = [0.0; 6];
    power[0] = 1.0;
    for k in 0..6 {
        for j in 0..6 {
            result[j] += coeffs[k] * power[j];
        }
        if k < 5 {
            // power ← power · (x0 + h·t)
            let mut next = [0.0; 6];
            for j in 0..6 {
                next[j] += power[j] * x0;
                if j + 1 < 6 {
                    next[j + 1] += power[j] * h;
                }
            }
            power = next;
        }
    }
    result
}

/// Cubic Hermite power coefficients on `u ∈ [0, 1]` from end values and end
/// derivatives already scaled by the interval width.
fn hermite_cubic(y0: f64, d0: f64, y1: f64, d1: f64) -> [f64; 6] {
    let delta = y1 - y0;
    [
        y0,
        d0,
        3.0 * delta - 2.0 * d0 - d1,
        -2.0 * delta + d0 + d1,
        0.0,
        0.0,
    ]
}

/// Index of the interpolation interval containing `s` (clamped to the ends).
fn interval_index(xs: &[f64], s: f64) -> usize {
    let mut i = xs.partition_point(|x| *x <= s);
    i = i.clamp(1, xs.len() - 1);
    i - 1
}

/// Shape-preserving (Fritsch–Carlson PCHIP) slopes: C1, monotone on monotone
/// data, never overshooting the input values.  Deterministic.
fn pchip_slopes(xs: &[f64], ys: &[f64]) -> Vec<f64> {
    let n = xs.len();
    debug_assert!(n >= 2);
    let m = n - 1;
    let h: Vec<f64> = (0..m).map(|i| xs[i + 1] - xs[i]).collect();
    let secant: Vec<f64> = (0..m).map(|i| (ys[i + 1] - ys[i]) / h[i]).collect();
    let mut d = vec![0.0_f64; n];
    if n == 2 {
        d[0] = secant[0];
        d[1] = secant[0];
        return d;
    }
    // Interior: weighted harmonic mean where the secants agree in sign
    // (Fritsch–Carlson), zero at local extrema — this is what prevents
    // overshoot.
    for i in 1..m {
        let (s0, s1) = (secant[i - 1], secant[i]);
        if s0 * s1 > 0.0 {
            let w1 = 2.0 * h[i] + h[i - 1];
            let w2 = h[i] + 2.0 * h[i - 1];
            d[i] = (w1 + w2) / (w1 / s0 + w2 / s1);
        }
    }
    // Ends: the standard shape-preserving three-point estimate, clamped so
    // the end interval stays monotone.
    d[0] = end_slope(h[0], h[1], secant[0], secant[1]);
    d[n - 1] = end_slope(h[m - 1], h[m - 2], secant[m - 1], secant[m - 2]);
    d
}

/// One-sided three-point end-slope estimate with the Fritsch–Carlson
/// monotonicity clamps.
fn end_slope(h0: f64, h1: f64, s0: f64, s1: f64) -> f64 {
    let mut d = ((2.0 * h0 + h1) * s0 - h0 * s1) / (h0 + h1);
    if d * s0 <= 0.0 {
        d = 0.0;
    } else if s0 * s1 < 0.0 && d.abs() > 3.0 * s0.abs() {
        d = 3.0 * s0;
    }
    d
}

